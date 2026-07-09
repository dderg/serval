//! Drives the real pipeline stages — the same `FitStage`/`Planner`/
//! `run_lowerer`/`Shaper` types `setup_stages` wires into OS threads for a
//! live print — synchronously over unbounded channels on the calling thread.
//! No stage is reimplemented: this is the production pipeline observed with
//! its intermediate fitted-stage output (pre-axis-split spatial geometry)
//! tapped alongside the final shaped output.

use crossbeam_channel::unbounded;
use geometry::path::CurvatureProfile;
use geometry::path::lowering::PositionProfile;
use nurbs::bezier::extract_bezier_pieces;
use serde::Serialize;
use trajectory::{AxisChainSet, ShapedSegment};

pub mod audit;
pub mod waypoints;

use motion_pipeline::fit_stage::FitStage;
use motion_pipeline::planner::Planner;
use motion_pipeline::{StreamConfig, run_lowerer};

pub use planner_config::{AxisDecl, PostProcessorDecl};

/// The E lane rides as axis 3, past the three spatial axes — the same index the
/// production bridge and the seam harness assign the extruder.
pub const EXTRUDER_AXIS: usize = 3;
// Position tolerance for the cubic lowering — the same order the streamer ships.
pub const TRAJECTORY_FIT_TOL_MM: f64 = 0.005;
pub const TRAJECTORY_FIT_TOL_ACCEL_MM_S2: f64 = 50.0;
pub const VELOCITY_INTEGRATION_TOL: f64 = 1e-7;
/// Comfortably above any snapshot case's move count, so a case is always
/// planned as a single window instead of being split by the streaming
/// planner's look-ahead backstop.
pub const SNAPSHOT_MAX_BUFFER_MOVES: usize = 65_536;

#[derive(Debug, thiserror::Error)]
pub enum SnapshotError {
    #[error("max_path_deviation must be finite and positive, got {0}")]
    InvalidMaxPathDeviation(f64),
    #[error("max_accel_deviation must be positive, got {0}")]
    InvalidMaxAccelDeviation(f64),
    #[error("need at least 2 waypoints")]
    TooFewWaypoints,
    #[error("{0}")]
    InvalidLimits(String),
    #[error("move {index}: {detail}")]
    InvalidMove { index: usize, detail: String },
    #[error("no spatial moves after filtering zero-displacement pairs")]
    NoSpatialMoves,
    #[error("post-processor chain: {0}")]
    InvalidChain(String),
}

#[derive(Debug, Clone)]
pub struct SnapshotParams {
    pub max_velocity: f64,
    pub max_accel: f64,
    pub square_corner_velocity: f64,
    pub max_jerk: f64,
    pub max_extrude_only_velocity: Option<f64>,
    pub max_extrude_only_accel: Option<f64>,
    pub max_path_deviation: Option<f64>,
    pub max_accel_deviation: Option<f64>,
    pub axis_decls: Vec<AxisDecl>,
    pub post_processor_decls: Vec<PostProcessorDecl>,
}

#[derive(Debug, Serialize)]
pub struct SeamMetric {
    pub t: f64,
    pub axis: usize,
    pub dp: f64,
    pub dv: f64,
    pub da: f64,
}

/// The full snapshot dict: raw input path, the lowered per-axis polynomial
/// trajectory the firmware actually executes (each piece `[t0, t1, c0, c1,
/// ..., cn]` — monomial coefficients in local time `tau = t - t0`, trailing
/// near-zero coefficients trimmed), and the seam continuity metrics computed
/// from those pieces. Serializes to the exact JSON schema the snapshot
/// baselines and the web viewers consume.
#[derive(Debug, Serialize)]
pub struct Snapshot {
    pub raw_x: Vec<f64>,
    pub raw_y: Vec<f64>,
    pub traj_x_pieces: Vec<Vec<f64>>,
    pub traj_y_pieces: Vec<Vec<f64>>,
    pub traj_z_pieces: Vec<Vec<f64>>,
    pub traj_e_pieces: Vec<Vec<f64>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub toolhead_x_pieces: Option<Vec<Vec<f64>>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub toolhead_y_pieces: Option<Vec<Vec<f64>>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub toolhead_z_pieces: Option<Vec<Vec<f64>>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub toolhead_e_pieces: Option<Vec<Vec<f64>>>,
    pub traj_t_end: f64,
    pub traversal_time_s: f64,
    pub seam_max_dp: [f64; 4],
    pub seam_max_dv: [f64; 4],
    pub seam_max_da: [f64; 4],
    pub worst_seams: Vec<SeamMetric>,
}

pub fn pipeline_snapshot(
    waypoints: &[(f64, f64, f64, f64, f64)],
    params: SnapshotParams,
) -> Result<Snapshot, SnapshotError> {
    if let Some(v) = params.max_path_deviation {
        if !(v.is_finite() && v > 0.0) {
            return Err(SnapshotError::InvalidMaxPathDeviation(v));
        }
    }
    if let Some(v) = params.max_accel_deviation {
        if !(v > 0.0) {
            return Err(SnapshotError::InvalidMaxAccelDeviation(v));
        }
    }
    if waypoints.len() < 2 {
        return Err(SnapshotError::TooFewWaypoints);
    }

    let limits = geometry::VelocityLimits::try_new(
        params.max_velocity,
        params.max_accel,
        geometry::corner_deviation_from_scv(params.square_corner_velocity, params.max_accel),
        params.max_jerk,
    )
    .map_err(|e| SnapshotError::InvalidLimits(e.to_string()))?;

    let moves = build_moves(waypoints, limits)?;
    let raw_points = extract_raw_path(&moves);

    let axis_chains = build_axis_chains(&params).map_err(SnapshotError::InvalidChain)?;
    let corner = geometry::CornerFitConfig {
        kernel_variance_s2: axis_chains.max_spatial_kernel_variance_s2(),
        ..geometry::CornerFitConfig::default()
    };
    let config = StreamConfig {
        corner,
        integration_tol: VELOCITY_INTEGRATION_TOL,
        max_extrude_only_velocity_mm_s: params.max_extrude_only_velocity.unwrap_or(f64::INFINITY),
        max_extrude_only_accel_mm_s2: params.max_extrude_only_accel.unwrap_or(f64::INFINITY),
        fit_tol_mm: params.max_path_deviation.unwrap_or(TRAJECTORY_FIT_TOL_MM),
        fit_tol_accel_mm_s2: params
            .max_accel_deviation
            .unwrap_or(TRAJECTORY_FIT_TOL_ACCEL_MM_S2),
        max_buffer_moves: SNAPSHOT_MAX_BUFFER_MOVES,
        limits,
    };
    let (_fitted, shaped, toolhead) = run_pipeline(&moves, config, axis_chains);

    let traj = collect_trajectory_pieces(&shaped);
    let seams = seam_metrics(&traj);
    let toolhead_traj = toolhead.map(|segs| collect_trajectory_pieces(&segs));

    Ok(Snapshot {
        raw_x: raw_points.iter().map(|p| p.0).collect(),
        raw_y: raw_points.iter().map(|p| p.1).collect(),
        traj_x_pieces: traj.x,
        traj_y_pieces: traj.y,
        traj_z_pieces: traj.z,
        traj_e_pieces: traj.e,
        toolhead_x_pieces: toolhead_traj.as_ref().map(|t| t.x.clone()),
        toolhead_y_pieces: toolhead_traj.as_ref().map(|t| t.y.clone()),
        toolhead_z_pieces: toolhead_traj.as_ref().map(|t| t.z.clone()),
        toolhead_e_pieces: toolhead_traj.map(|t| t.e),
        traj_t_end: traj.t_end,
        traversal_time_s: traj.t_end,
        seam_max_dp: seams.max_dp,
        seam_max_dv: seams.max_dv,
        seam_max_da: seams.max_da,
        worst_seams: seams.worst,
    })
}

pub fn build_moves(
    waypoints: &[(f64, f64, f64, f64, f64)],
    limits: geometry::VelocityLimits,
) -> Result<Vec<geometry::Move>, SnapshotError> {
    let mut moves = Vec::with_capacity(waypoints.len() - 1);
    for (i, pair) in waypoints.windows(2).enumerate() {
        let (x0, y0, z0, e0, _) = pair[0];
        let (x1, y1, z1, e1, feedrate) = pair[1];
        let start = [x0, y0, z0];
        let end = [x1, y1, z1];
        let e_delta = e1 - e0;
        let ctx = geometry::MoveContext {
            extruder_axis: EXTRUDER_AXIS,
            feedrate_mm_s: feedrate,
            limits,
            source: geometry::SourceRange {
                start_line: i as u32,
                end_line: i as u32,
            },
        };
        match geometry::line_move(start, end, e_delta, ctx) {
            Ok(m) => moves.push(m),
            Err(geometry::FrontendError::ZeroMotion { .. }) => {}
            Err(e) => {
                return Err(SnapshotError::InvalidMove {
                    index: i,
                    detail: format!("{e:?}"),
                });
            }
        }
    }
    if moves.is_empty() {
        return Err(SnapshotError::NoSpatialMoves);
    }
    Ok(moves)
}

pub fn extract_raw_path(moves: &[geometry::Move]) -> Vec<(f64, f64)> {
    let mut points = Vec::with_capacity(moves.len() + 1);
    for (i, m) in moves.iter().enumerate() {
        if let Some(spatial) = &m.segment.spatial {
            let start = spatial.point_at(0.0);
            if i == 0 {
                points.push((start[0], start[1]));
            }
            let end = spatial.point_at(spatial.s_len());
            points.push((end[0], end[1]));
        }
    }
    points
}

fn default_axis_decls() -> Vec<AxisDecl> {
    vec![
        AxisDecl {
            name: "x".into(),
            follows: Vec::new(),
            motors: Vec::new(),
            post_processors: Vec::new(),
        },
        AxisDecl {
            name: "y".into(),
            follows: Vec::new(),
            motors: Vec::new(),
            post_processors: Vec::new(),
        },
        AxisDecl {
            name: "z".into(),
            follows: Vec::new(),
            motors: Vec::new(),
            post_processors: Vec::new(),
        },
        AxisDecl {
            name: "e".into(),
            follows: vec!["x".into(), "y".into(), "z".into()],
            motors: Vec::new(),
            post_processors: Vec::new(),
        },
    ]
}

const KNOWN_AXES: [&str; 4] = ["x", "y", "z", "e"];

/// Layers any explicitly-declared axes over the default x/y/z/e-follows-xyz
/// topology, by name — `AxisRegistry::try_new` requires all of x/y/z to be
/// present, so without this a caller declaring e.g. only `[axis e]` (to
/// attach pressure advance) would hit `MissingSpatialAxis` for x. A caller
/// only needs to declare the axis it's actually customizing; the rest fall
/// back to their default (no post-processors). Any name outside x/y/z/e is
/// rejected — this pipeline's topology is fixed, there's no data lane for a
/// 5th axis.
fn merge_axis_decls(explicit: &[AxisDecl]) -> Result<Vec<AxisDecl>, String> {
    for d in explicit {
        if !KNOWN_AXES.contains(&d.name.as_str()) {
            return Err(format!(
                "axis '{}': only x, y, z, e are supported here (this pipeline's topology is fixed)",
                d.name
            ));
        }
    }
    Ok(default_axis_decls()
        .into_iter()
        .map(|default| {
            explicit
                .iter()
                .find(|d| d.name == default.name)
                .cloned()
                .unwrap_or(default)
        })
        .collect())
}

/// The snapshot's post-processor chains, compiled through the exact same
/// path the live engine uses (`planner_config::AxisRegistry` +
/// `PostProcessorSet`) so a case's `[axis]`/`[post_processor]` sections
/// behave identically here as on a real printer. No axes declared at all
/// falls back to the default x/y/z/e-follows-xyz topology with no
/// post-processors — the historical no-shaping-no-PA behavior every
/// existing baseline was recorded with.
fn build_axis_chains(params: &SnapshotParams) -> Result<AxisChainSet, String> {
    let axis_decls = merge_axis_decls(&params.axis_decls)?;
    let registry = planner_config::AxisRegistry::try_new(axis_decls).map_err(|e| e.to_string())?;
    planner_config::PostProcessorSet::try_new(&registry, &params.post_processor_decls)
        .map_err(|e| e.to_string())?
        .compile(&registry)
        .map_err(|e| e.to_string())
}

/// The third element is the toolhead signal — the shaped segments before the
/// motor-side derivative-gain stages — present exactly when some chain makes
/// the motor command depart from it.
pub fn run_pipeline(
    moves: &[geometry::Move],
    config: StreamConfig,
    axis_chains: AxisChainSet,
) -> (
    Vec<geometry::Move>,
    Vec<ShapedSegment>,
    Option<Vec<ShapedSegment>>,
) {
    let spatial_home = moves
        .iter()
        .find_map(|m| m.segment.spatial.as_ref())
        .map_or([0.0, 0.0, 0.0], |seg| seg.point_at(0.0));
    let home_pos = vec![spatial_home[0], spatial_home[1], spatial_home[2], 0.0];

    let (raw_tx, raw_rx) = unbounded();
    for m in moves.iter().cloned() {
        raw_tx
            .send(m.into())
            .expect("unbounded channel never blocks");
    }
    drop(raw_tx);

    let (fitted_tx, fitted_rx) = unbounded();
    FitStage::new(config.corner).run(raw_rx, fitted_tx);
    let fitted: Vec<geometry::Move> = fitted_rx
        .into_iter()
        .filter_map(|item| match item {
            motion_pipeline::StreamInput::Move(m) => Some(m),
            motion_pipeline::StreamInput::Drain | motion_pipeline::StreamInput::Control(_) => None,
        })
        .collect();

    let (planner_tx, planner_rx) = unbounded();
    for fm in fitted.iter().cloned() {
        planner_tx
            .send(fm.into())
            .expect("unbounded channel never blocks");
    }
    drop(planner_tx);

    let (planned_tx, planned_rx) = unbounded();
    Planner::new(config).run(planner_rx, planned_tx);

    let (lowered_tx, lowered_rx) = unbounded();
    run_lowerer(
        planned_rx,
        lowered_tx,
        motion_pipeline::FitTol {
            pos_mm: config.fit_tol_mm,
            accel_mm_s2: config.fit_tol_accel_mm_s2,
        },
        axis_chains.clone(),
        home_pos,
        0.0,
    );

    let (shaped_tx, shaped_rx) = unbounded();
    let capture_toolhead = axis_chains.has_motor_side_stages();
    let mut shaper = motion_pipeline::Shaper::new(axis_chains);
    let toolhead_rx = if capture_toolhead {
        let (toolhead_tx, toolhead_rx) = unbounded();
        shaper = shaper.with_toolhead_tap(toolhead_tx);
        Some(toolhead_rx)
    } else {
        None
    };
    shaper.run(lowered_rx, shaped_tx);
    let shaped: Vec<ShapedSegment> = shaped_rx
        .into_iter()
        .filter_map(|item| match item {
            motion_pipeline::ShapedItem::Seg(seg) => Some(seg),
            motion_pipeline::ShapedItem::Control(_) => None,
        })
        .collect();
    let toolhead = toolhead_rx.map(|rx| {
        let segs: Vec<ShapedSegment> = rx.into_iter().collect();
        assert_eq!(
            segs.len(),
            shaped.len(),
            "toolhead tap must mirror every emitted segment"
        );
        segs
    });

    (fitted, shaped, toolhead)
}

/// The trajectory the firmware actually executes: the host lowering's own
/// per-axis polynomial pieces of position-versus-time. Each row is
/// `[t0, t1, c0, c1, ..., cn]` — monomial coefficients in local time
/// `tau = t - t0`, trailing near-zero coefficients trimmed so each piece's
/// true degree is visible in the snapshot. The visualizer differentiates
/// these analytically, so every derivative is exact and continuous — no
/// position sampling, and nothing copied from the planner's own acceleration,
/// so it stays an independent check.
#[derive(Debug)]
pub struct TrajectoryPieces {
    pub x: Vec<Vec<f64>>,
    pub y: Vec<Vec<f64>>,
    pub z: Vec<Vec<f64>>,
    pub e: Vec<Vec<f64>>,
    pub t_end: f64,
}

pub fn collect_trajectory_pieces(shaped: &[ShapedSegment]) -> TrajectoryPieces {
    fn collect(dst: &mut Vec<Vec<f64>>, axis: Option<&nurbs::ScalarNurbs>) {
        let Some(axis) = axis else { return };
        for p in extract_bezier_pieces(axis) {
            let scale = p.coeffs.iter().fold(0.0_f64, |m, c| m.max(c.abs()));
            let mut coeffs = p.coeffs.clone();
            while coeffs.len() > 1
                && coeffs
                    .last()
                    .is_some_and(|c| c.abs() <= 1e-12 * (scale + 1.0))
            {
                coeffs.pop();
            }
            let mut row = vec![p.u_start, p.u_end];
            row.extend_from_slice(&coeffs);
            dst.push(row);
        }
    }

    let mut out = TrajectoryPieces {
        x: Vec::new(),
        y: Vec::new(),
        z: Vec::new(),
        e: Vec::new(),
        t_end: 0.0,
    };
    for seg in shaped {
        collect(&mut out.x, seg.axes.first());
        collect(&mut out.y, seg.axes.get(1));
        collect(&mut out.z, seg.axes.get(2));
        collect(&mut out.e, seg.axes.get(3));
        out.t_end = seg.t_end;
    }
    out
}

pub struct SeamMetrics {
    pub max_dp: [f64; 4],
    pub max_dv: [f64; 4],
    pub max_da: [f64; 4],
    pub worst: Vec<SeamMetric>,
}

const WORST_SEAM_COUNT: usize = 10;

fn piece_state_at(p: &[f64], tau: f64) -> (f64, f64, f64) {
    let c = &p[2..];
    let mut pos = 0.0;
    let mut vel = 0.0;
    let mut acc = 0.0;
    for (k, &ck) in c.iter().enumerate().rev() {
        pos = pos * tau + ck;
        if k >= 1 {
            vel = vel * tau + k as f64 * ck;
        }
        if k >= 2 {
            acc = acc * tau + (k * (k - 1)) as f64 * ck;
        }
    }
    (pos, vel, acc)
}

fn piece_end_state(p: &[f64]) -> (f64, f64, f64) {
    piece_state_at(p, p[1] - p[0])
}

fn piece_start_state(p: &[f64]) -> (f64, f64, f64) {
    piece_state_at(p, 0.0)
}

pub fn seam_metrics(traj: &TrajectoryPieces) -> SeamMetrics {
    let axes = [&traj.x, &traj.y, &traj.z, &traj.e];
    let mut out = SeamMetrics {
        max_dp: [0.0; 4],
        max_dv: [0.0; 4],
        max_da: [0.0; 4],
        worst: Vec::new(),
    };
    for (axis, pieces) in axes.iter().enumerate() {
        for w in pieces.windows(2) {
            let (lp, lv, la) = piece_end_state(&w[0]);
            let (rp, rv, ra) = piece_start_state(&w[1]);
            let seam = SeamMetric {
                t: w[1][0],
                axis,
                dp: (rp - lp).abs(),
                dv: (rv - lv).abs(),
                da: (ra - la).abs(),
            };
            out.max_dp[axis] = out.max_dp[axis].max(seam.dp);
            out.max_dv[axis] = out.max_dv[axis].max(seam.dv);
            out.max_da[axis] = out.max_da[axis].max(seam.da);
            out.worst.push(seam);
        }
    }
    out.worst.sort_by(|a, b| {
        b.da.total_cmp(&a.da)
            .then(b.dv.total_cmp(&a.dv))
            .then(a.axis.cmp(&b.axis))
            .then(a.t.total_cmp(&b.t))
    });
    out.worst.truncate(WORST_SEAM_COUNT);
    out
}

#[cfg(test)]
mod tests;
