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

pub mod waypoints;

use motion_pipeline::fit_stage::FitStage;
use motion_pipeline::planner::Planner;
use motion_pipeline::{StreamConfig, run_lowerer};

pub const SAMPLES_PER_MM: f64 = 2.0;
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
    #[error("[arc_fit] min_run_facets must be at least 3")]
    ArcFitTooFewFacets,
    #[error("move {index}: {detail}")]
    InvalidMove { index: usize, detail: String },
    #[error("no spatial moves after filtering zero-displacement pairs")]
    NoSpatialMoves,
    #[error("post-processor chain: {0}")]
    InvalidChain(String),
}

#[derive(Debug, Clone, Copy)]
pub struct SnapshotParams {
    pub max_velocity: f64,
    pub max_accel: f64,
    pub square_corner_velocity: f64,
    pub max_jerk: f64,
    pub arc_fit: Option<u32>,
    pub max_extrude_only_velocity: Option<f64>,
    pub max_extrude_only_accel: Option<f64>,
    pub max_path_deviation: Option<f64>,
    pub max_accel_deviation: Option<f64>,
    pub pressure_advance: Option<f64>,
    pub smooth_zv_hz: Option<f64>,
    pub e_smooth_zv_hz: Option<f64>,
}

#[derive(Debug, Serialize)]
#[serde(tag = "type", rename_all = "lowercase")]
pub enum FittedSegment {
    Line { x0: f64, y0: f64, x1: f64, y1: f64 },
    Arc { x: Vec<f64>, y: Vec<f64> },
    Clothoid { x: Vec<f64>, y: Vec<f64> },
}

#[derive(Debug, Serialize)]
pub struct SeamMetric {
    pub t: f64,
    pub axis: usize,
    pub dp: f64,
    pub dv: f64,
    pub da: f64,
}

/// The full snapshot dict: raw input path, fitted spatial geometry, the
/// lowered per-axis polynomial trajectory the firmware actually executes
/// (each piece `[t0, t1, c0, c1, ..., cn]` — monomial coefficients in local
/// time `tau = t - t0`, trailing near-zero coefficients trimmed), and the
/// seam continuity metrics computed from those pieces. Serializes to the
/// exact JSON schema the snapshot baselines and the web viewers consume.
#[derive(Debug, Serialize)]
pub struct Snapshot {
    pub raw_x: Vec<f64>,
    pub raw_y: Vec<f64>,
    pub fitted_segments: Vec<FittedSegment>,
    pub traj_x_pieces: Vec<Vec<f64>>,
    pub traj_y_pieces: Vec<Vec<f64>>,
    pub traj_z_pieces: Vec<Vec<f64>>,
    pub traj_e_pieces: Vec<Vec<f64>>,
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
        params.square_corner_velocity,
        params.max_jerk,
    )
    .map_err(|e| SnapshotError::InvalidLimits(e.to_string()))?;
    let chain_cfg = arc_fit_config(params.arc_fit)?;

    let moves = build_moves(waypoints, limits)?;
    let raw_points = extract_raw_path(&moves);

    let config = StreamConfig {
        chain: chain_cfg,
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
    let axis_chains =
        build_axis_chains(&params).map_err(|e| SnapshotError::InvalidChain(e.to_string()))?;
    let (fitted, shaped) = run_pipeline(&moves, config, axis_chains);

    let fitted_segments = fitted
        .iter()
        .filter_map(|fm| fm.segment.spatial.as_ref().map(sample_segment))
        .collect();

    let traj = collect_trajectory_pieces(&shaped);
    let seams = seam_metrics(&traj);

    Ok(Snapshot {
        raw_x: raw_points.iter().map(|p| p.0).collect(),
        raw_y: raw_points.iter().map(|p| p.1).collect(),
        fitted_segments,
        traj_x_pieces: traj.x,
        traj_y_pieces: traj.y,
        traj_z_pieces: traj.z,
        traj_e_pieces: traj.e,
        traj_t_end: traj.t_end,
        traversal_time_s: traj.t_end,
        seam_max_dp: seams.max_dp,
        seam_max_dv: seams.max_dv,
        seam_max_da: seams.max_da,
        worst_seams: seams.worst,
    })
}

fn sample_segment(spatial: &geometry::path::Segment) -> FittedSegment {
    match spatial {
        geometry::path::Segment::Line(line) => FittedSegment::Line {
            x0: line.start[0],
            y0: line.start[1],
            x1: line.end[0],
            y1: line.end[1],
        },
        geometry::path::Segment::Arc(_) | geometry::path::Segment::Clothoid(_) => {
            let len = spatial.s_len();
            let n = ((len * SAMPLES_PER_MM).ceil() as usize).max(20);
            let mut xs = Vec::with_capacity(n);
            let mut ys = Vec::with_capacity(n);
            for k in 0..n {
                let s = len * (k as f64) / ((n - 1) as f64);
                let pt = spatial.point_at(s);
                xs.push(pt[0]);
                ys.push(pt[1]);
            }
            match spatial {
                geometry::path::Segment::Arc(_) => FittedSegment::Arc { x: xs, y: ys },
                _ => FittedSegment::Clothoid { x: xs, y: ys },
            }
        }
    }
}

fn arc_fit_config(arc_fit: Option<u32>) -> Result<geometry::ChainFitConfig, SnapshotError> {
    let Some(min_run_facets) = arc_fit else {
        return Ok(geometry::ChainFitConfig::default());
    };
    if min_run_facets < 3 {
        return Err(SnapshotError::ArcFitTooFewFacets);
    }
    Ok(geometry::ChainFitConfig::with_arc_fit(min_run_facets))
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

/// The snapshot's optional post-processor chains: smoothing on x/y, and on
/// the extruder lane an optional smoother ahead of pressure advance — the
/// same compiled stages the live config builds, so snapshot piece counts and
/// seams reflect the real wire.
fn build_axis_chains(
    params: &SnapshotParams,
) -> Result<AxisChainSet, trajectory::PostProcessorError> {
    use trajectory::chain::PostProcessorInstance;
    use trajectory::algos::{LinearPressureAdvance, SmoothZv};
    use trajectory::{CompiledChain, PostProcessorError};

    if params.pressure_advance.is_none()
        && params.smooth_zv_hz.is_none()
        && params.e_smooth_zv_hz.is_none()
    {
        return Ok(AxisChainSet::default());
    }
    let spatial_chain = |hz: Option<f64>| -> Result<CompiledChain, PostProcessorError> {
        match hz {
            Some(hz) => CompiledChain::compile(&[PostProcessorInstance::new(
                "smooth_zv",
                &SmoothZv,
                vec![hz],
            )]),
            None => Ok(CompiledChain::default()),
        }
    };
    let mut e_stages = Vec::new();
    if let Some(hz) = params.e_smooth_zv_hz {
        e_stages.push(PostProcessorInstance::new("smooth_zv", &SmoothZv, vec![hz]));
    }
    if let Some(k) = params.pressure_advance {
        e_stages.push(PostProcessorInstance::new(
            "pressure_advance",
            &LinearPressureAdvance,
            vec![k],
        ));
    }
    let e_chain = CompiledChain::compile(&e_stages)?;
    Ok(AxisChainSet {
        chains: vec![
            spatial_chain(params.smooth_zv_hz)?,
            spatial_chain(params.smooth_zv_hz)?,
            CompiledChain::default(),
            e_chain,
        ],
        followers: Vec::new(),
    })
}

pub fn run_pipeline(
    moves: &[geometry::Move],
    config: StreamConfig,
    axis_chains: AxisChainSet,
) -> (Vec<geometry::Move>, Vec<ShapedSegment>) {
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
    FitStage::new(config.chain).run(raw_rx, fitted_tx);
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
    motion_pipeline::Shaper::new(axis_chains).run(lowered_rx, shaped_tx);
    let shaped: Vec<ShapedSegment> = shaped_rx
        .into_iter()
        .filter_map(|item| match item {
            motion_pipeline::ShapedItem::Seg(seg) => Some(seg),
            motion_pipeline::ShapedItem::Control(_) => None,
        })
        .collect();

    (fitted, shaped)
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
