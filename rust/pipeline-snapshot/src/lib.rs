//! Drives the real pipeline stages — the same `FitStage`/`Planner`/
//! `run_lowerer`/`Shaper` types `setup_stages` wires into OS threads for a
//! live print — synchronously over unbounded channels on the calling thread.
//! No stage is reimplemented: this is the production pipeline observed with
//! its intermediate fitted-stage output (pre-axis-split spatial geometry)
//! tapped alongside the final shaped output.

use crossbeam_channel::{Receiver, Sender, unbounded};
use geometry::path::CurvatureProfile;
use geometry::path::lowering::PositionProfile;
use nurbs::bezier::extract_bezier_pieces;
use serde::Serialize;
use trajectory::{AxisChainSet, ContinuousAxis, ContinuousSegment};

pub mod audit;
pub mod waypoints;

use motion_pipeline::fit_stage::{FitDriver, FitStage};
use motion_pipeline::planner::Planner;
use motion_pipeline::{
    BaseItem, FitTol, Lowerer, PlannedItem, Shaper, StreamConfig, StreamInput, TrajectoryItem,
};

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
    #[error("corner_deviation must be finite and non-negative, got {0}")]
    InvalidCornerDeviation(f64),
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
    /// Direct corner budget in mm — the canonical form; when set,
    /// `square_corner_velocity` is ignored (it is the legacy alias).
    pub corner_deviation: Option<f64>,
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
    waypoints: &[waypoints::Waypoint],
    params: SnapshotParams,
) -> Result<Snapshot, SnapshotError> {
    pipeline_snapshot_streaming(waypoints, params, usize::MAX, |_| {})
}

/// Same computation as [`pipeline_snapshot`], but additionally invokes
/// `on_partial` with schema-complete prefix snapshots *while the pipeline
/// runs* — the stages are driven cooperatively move by move, so each partial
/// covers the shaped segments emitted so far (the raw input path is always
/// complete). A partial fires once at least `partial_batch_segments` new
/// segments accumulated since the last one, backing off proportionally to the
/// prefix length so rebuilding ever-longer prefixes stays a bounded fraction
/// of the total work. The returned final snapshot is identical to what
/// [`pipeline_snapshot`] produces for the same inputs.
pub fn pipeline_snapshot_streaming(
    waypoints: &[waypoints::Waypoint],
    params: SnapshotParams,
    partial_batch_segments: usize,
    mut on_partial: impl FnMut(&Snapshot),
) -> Result<Snapshot, SnapshotError> {
    assert!(
        partial_batch_segments > 0,
        "partial_batch_segments must be positive"
    );
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

    if let Some(v) = params.corner_deviation {
        if !(v.is_finite() && v >= 0.0) {
            return Err(SnapshotError::InvalidCornerDeviation(v));
        }
    }
    let corner_deviation_mm = params.corner_deviation.unwrap_or_else(|| {
        geometry::corner_deviation_from_scv(params.square_corner_velocity, params.max_accel)
    });
    let limits = geometry::VelocityLimits::try_new(
        params.max_velocity,
        params.max_accel,
        corner_deviation_mm,
        params.max_jerk,
    )
    .map_err(|e| SnapshotError::InvalidLimits(e.to_string()))?;

    let moves = build_moves(waypoints, limits)?;
    let raw_points = extract_raw_path(&moves);

    let axis_chains = build_axis_chains(&params).map_err(SnapshotError::InvalidChain)?;
    let corner = geometry::CornerFitConfig::default();
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
    let mut next_partial_at = partial_batch_segments;
    let (_fitted, shaped, toolhead) =
        run_pipeline_streaming(&moves, config, axis_chains, |shaped, toolhead| {
            if shaped.len() < next_partial_at {
                return;
            }
            on_partial(&snapshot_from_segments(&raw_points, shaped, toolhead));
            next_partial_at = shaped
                .len()
                .saturating_add(partial_batch_segments.max(shaped.len() / 8));
        });
    Ok(snapshot_from_segments(
        &raw_points,
        &shaped,
        toolhead.as_deref(),
    ))
}

fn snapshot_from_segments(
    raw_points: &[(f64, f64)],
    shaped: &[ContinuousSegment],
    toolhead: Option<&[ContinuousSegment]>,
) -> Snapshot {
    let traj = collect_trajectory_pieces(shaped);
    let seams = seam_metrics(&traj);
    let toolhead_traj = toolhead.map(collect_trajectory_pieces);

    Snapshot {
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
    }
}

pub fn build_moves(
    waypoints: &[waypoints::Waypoint],
    limits: geometry::VelocityLimits,
) -> Result<Vec<geometry::Move>, SnapshotError> {
    let mut moves = Vec::with_capacity(waypoints.len() - 1);
    for (i, pair) in waypoints.windows(2).enumerate() {
        let (x0, y0, z0, e0, _, _) = pair[0];
        let (x1, y1, z1, e1, feedrate, accel) = pair[1];
        let start = [x0, y0, z0];
        let end = [x1, y1, z1];
        let e_delta = e1 - e0;
        let move_limits = geometry::VelocityLimits::try_new(
            limits.max_velocity_mm_s,
            accel,
            limits.corner_deviation_mm,
            limits.max_jerk_mm_s3,
        )
        .map_err(|e| SnapshotError::InvalidMove {
            index: i,
            detail: e.to_string(),
        })?;
        let ctx = geometry::MoveContext {
            extruder_axis: EXTRUDER_AXIS,
            feedrate_mm_s: feedrate,
            limits: move_limits,
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
    Vec<ContinuousSegment>,
    Option<Vec<ContinuousSegment>>,
) {
    run_pipeline_streaming(moves, config, axis_chains, |_, _| {})
}

/// Same computation as [`run_pipeline`] — the stages see the identical item
/// sequence, so the output is bit-identical — but the four stages are driven
/// cooperatively on the calling thread, one input move at a time, and
/// `on_progress(shaped_so_far, toolhead_so_far)` runs after each move's
/// effects have propagated all the way through the shaper. That is what lets
/// a single-threaded host (wasm) observe the trajectory growing while
/// planning is still under way.
pub fn run_pipeline_streaming(
    moves: &[geometry::Move],
    config: StreamConfig,
    axis_chains: AxisChainSet,
    mut on_progress: impl FnMut(&[ContinuousSegment], Option<&[ContinuousSegment]>),
) -> (
    Vec<geometry::Move>,
    Vec<ContinuousSegment>,
    Option<Vec<ContinuousSegment>>,
) {
    let spatial_home = moves
        .iter()
        .find_map(|m| m.segment.spatial.as_ref())
        .map_or([0.0, 0.0, 0.0], |seg| seg.point_at(0.0));
    let home_pos = vec![spatial_home[0], spatial_home[1], spatial_home[2], 0.0];

    let (fitted_tx, fitted_rx) = unbounded();
    let (planned_tx, planned_rx) = unbounded();
    let (lowered_tx, lowered_rx) = unbounded();
    let (shaped_tx, shaped_rx) = unbounded();
    let capture_toolhead = axis_chains.has_motor_side_stages();
    let mut shaper = Shaper::new(
        axis_chains.clone(),
        FitTol {
            pos_mm: config.fit_tol_mm,
            accel_mm_s2: config.fit_tol_accel_mm_s2,
        },
    );
    let toolhead_rx = if capture_toolhead {
        let (toolhead_tx, toolhead_rx) = unbounded();
        shaper = shaper.with_toolhead_tap(toolhead_tx);
        Some(toolhead_rx)
    } else {
        None
    };

    let mut drive = PipelineDrive {
        fit: FitStage::new(config.corner).into_driver(fitted_tx),
        fitted_rx,
        planner: Planner::new(config),
        planned_tx,
        planned_rx,
        lowerer: Lowerer::new(axis_chains, home_pos, 0.0),
        lowered_tx,
        lowered_rx,
        shaper,
        shaped_tx,
        shaped_rx,
        toolhead_rx,
        fitted: Vec::new(),
        shaped: Vec::new(),
        toolhead: Vec::new(),
    };

    for m in moves.iter().cloned() {
        assert!(drive.fit.feed(m.into()), "fit stage output channel closed");
        drive.pump();
        on_progress(
            &drive.shaped,
            capture_toolhead.then_some(drive.toolhead.as_slice()),
        );
    }
    assert!(drive.fit.finish(), "fit stage output channel closed");
    drive.pump();
    assert!(
        drive.planner.finish(&drive.planned_tx),
        "planner output channel closed"
    );
    drive.pump();
    assert!(
        drive.shaper.finish(&drive.shaped_tx),
        "shaper output channel closed"
    );
    drive.pump();
    on_progress(
        &drive.shaped,
        capture_toolhead.then_some(drive.toolhead.as_slice()),
    );

    let toolhead = capture_toolhead.then(|| {
        assert_eq!(
            drive.toolhead.len(),
            drive.shaped.len(),
            "toolhead tap must mirror every emitted segment"
        );
        drive.toolhead
    });
    (drive.fitted, drive.shaped, toolhead)
}

/// The cooperative single-thread wiring of the four stages: unbounded
/// channels carry each stage's output, and `pump` walks them in pipeline
/// order, so one pass moves everything a fed move produced all the way to the
/// shaped output.
struct PipelineDrive {
    fit: FitDriver,
    fitted_rx: Receiver<StreamInput>,
    planner: Planner,
    planned_tx: Sender<PlannedItem>,
    planned_rx: Receiver<PlannedItem>,
    lowerer: Lowerer,
    lowered_tx: Sender<BaseItem>,
    lowered_rx: Receiver<BaseItem>,
    shaper: Shaper,
    shaped_tx: Sender<TrajectoryItem>,
    shaped_rx: Receiver<TrajectoryItem>,
    toolhead_rx: Option<Receiver<ContinuousSegment>>,
    fitted: Vec<geometry::Move>,
    shaped: Vec<ContinuousSegment>,
    toolhead: Vec<ContinuousSegment>,
}

impl PipelineDrive {
    fn pump(&mut self) {
        while let Ok(item) = self.fitted_rx.try_recv() {
            if let StreamInput::Move(m) = &item {
                self.fitted.push(m.clone());
            }
            assert!(
                self.planner.feed(item, &self.planned_tx),
                "planner output channel closed"
            );
        }
        while let Ok(item) = self.planned_rx.try_recv() {
            assert!(
                self.lowerer.feed(item, &self.lowered_tx),
                "lowerer output channel closed"
            );
        }
        while let Ok(item) = self.lowered_rx.try_recv() {
            assert!(
                self.shaper.feed(item, &self.shaped_tx),
                "shaper output channel closed"
            );
        }
        while let Ok(item) = self.shaped_rx.try_recv() {
            if let TrajectoryItem::Seg(seg) = item {
                self.shaped.push(seg);
            }
        }
        if let Some(rx) = &self.toolhead_rx {
            while let Ok(seg) = rx.try_recv() {
                self.toolhead.push(seg);
            }
        }
    }
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

pub fn collect_trajectory_pieces(shaped: &[ContinuousSegment]) -> TrajectoryPieces {
    let mut out = TrajectoryPieces {
        x: Vec::new(),
        y: Vec::new(),
        z: Vec::new(),
        e: Vec::new(),
        t_end: 0.0,
    };
    for seg in shaped {
        for (axis, dst) in [&mut out.x, &mut out.y, &mut out.z, &mut out.e]
            .into_iter()
            .enumerate()
        {
            if let Some(source) = seg.axes.get(axis) {
                collect_axis_pieces(dst, seg, axis, source);
            }
        }
        out.t_end = seg.t_end;
    }
    for dst in [&mut out.x, &mut out.y, &mut out.z, &mut out.e] {
        absorb_trailing_sliver(dst);
    }
    out
}

/// A spline-carried axis is already piecewise-polynomial in time, so its
/// Bezier pieces are the trajectory verbatim. Every other carrier — the
/// analytic move span, a hold, a nudge or buzz profile — is not a polynomial
/// of `t` in general (an arc's per-axis position never is), so it is
/// converted by matching the carrier's exact position/velocity/acceleration
/// at each piece boundary, refining until the interpolant tracks the carrier
/// to `SAMPLED_PIECE_TOL_MM`. On a straight move that reproduces the
/// jerk-limited phase polynomials exactly (they are cubics, the quintic
/// interpolant's error is zero), so no refinement happens and the emitted
/// piece is the phase itself.
fn collect_axis_pieces(
    dst: &mut Vec<Vec<f64>>,
    seg: &ContinuousSegment,
    axis: usize,
    source: &ContinuousAxis,
) {
    match source {
        ContinuousAxis::Spline(curve) => push_spline_pieces(dst, curve, 0.0, None),
        ContinuousAxis::RelativeSpline {
            base_position,
            curve,
        } => push_spline_pieces(dst, curve, *base_position, None),
        ContinuousAxis::PiecewiseRelativeSpline(pieces) => {
            let mut owned_from = f64::NEG_INFINITY;
            for piece in pieces.iter() {
                let window_start = piece.t_start.max(owned_from);
                if piece.t_end > window_start {
                    push_spline_pieces(
                        dst,
                        &piece.curve,
                        piece.base_position,
                        Some((window_start, piece.t_end)),
                    );
                }
                owned_from = piece.t_end.max(owned_from);
            }
        }
        _ => push_sampled_pieces(dst, seg, axis, source),
    }
}

/// The shortest window a snapshot row may carry: the device's step-time
/// resolution. Anything narrower is a numerical sliver of a clipped fit
/// window, and differentiating it manufactures derivative magnitudes the
/// firmware never executes, so it is coalesced into the row that owns the
/// instant on its right — the same ownership `owning_piece` applies at a
/// piecewise seam.
const MIN_PIECE_DURATION_S: f64 = 2e-9;

/// `owned` clips a windowed curve to the time range the piece owns: the fit
/// windows the shaper carries extend past their own span, and emitting those
/// tails would overlap the neighbouring piece's rows. The clip is a Taylor
/// re-expansion about the retained window's start, so the polynomial — and
/// with it every position, velocity and acceleration the row reports — is the
/// carrier's own, never a resampling of it.
fn push_spline_pieces(
    dst: &mut Vec<Vec<f64>>,
    curve: &nurbs::ScalarNurbs,
    base_position: f64,
    owned: Option<(f64, f64)>,
) {
    for p in extract_bezier_pieces(curve) {
        let (mut t0, mut t1) = (p.u_start, p.u_end);
        if let Some((window_start, window_end)) = owned {
            t0 = t0.max(window_start);
            t1 = t1.min(window_end);
        }
        if t1 <= t0 {
            continue;
        }
        let mut coeffs = shift_monomial(&p.coeffs, t0 - p.u_start);
        if let Some(c0) = coeffs.first_mut() {
            *c0 += base_position;
        }
        push_piece(dst, t0, t1, coeffs);
    }
}

/// Re-expands `coeffs` (ascending monomials in `tau`) about `tau = delta` by
/// iterated synthetic division, exactly reproducing the same polynomial in
/// the shifted local time.
fn shift_monomial(coeffs: &[f64], delta: f64) -> Vec<f64> {
    if delta == 0.0 || coeffs.len() < 2 {
        return coeffs.to_vec();
    }
    let mut descending: Vec<f64> = coeffs.iter().rev().copied().collect();
    let mut shifted = Vec::with_capacity(coeffs.len());
    while descending.len() > 1 {
        let mut quotient = Vec::with_capacity(descending.len() - 1);
        let mut acc = descending[0];
        for &next in &descending[1..] {
            quotient.push(acc);
            acc = next + acc * delta;
        }
        shifted.push(acc);
        descending = quotient;
    }
    shifted.push(descending[0]);
    shifted
}

const SAMPLED_PIECE_TOL_MM: f64 = 1e-6;
const SAMPLED_PIECE_MAX_DEPTH: u32 = 10;

fn push_sampled_pieces(
    dst: &mut Vec<Vec<f64>>,
    seg: &ContinuousSegment,
    axis: usize,
    source: &ContinuousAxis,
) {
    let mut breaks = vec![seg.t_start, seg.t_end];
    match source {
        ContinuousAxis::Analytic { span, .. } => breaks.extend(
            span.phases
                .iter()
                .flat_map(|phase| [span.t_start + phase.t0, span.t_start + phase.end_time()]),
        ),
        ContinuousAxis::Nudge(profile) => breaks.extend_from_slice(profile.breakpoints()),
        ContinuousAxis::Buzz { profile, .. } => breaks.extend_from_slice(profile.breakpoints()),
        _ => {}
    }
    breaks.retain(|t| *t > seg.t_start && *t < seg.t_end);
    breaks.push(seg.t_start);
    breaks.push(seg.t_end);
    breaks.sort_by(f64::total_cmp);
    breaks.dedup();
    for w in breaks.windows(2) {
        if w[1] > w[0] {
            push_interpolated_piece(dst, seg, axis, w[0], w[1], 0);
        }
    }
}

/// Interior stations, as fractions of the piece, the interpolant is checked
/// against the carrier at. A lone midpoint check is blind to a carrier that
/// is point-symmetric about that midpoint: the quintic inherits the symmetry
/// and meets the carrier exactly there while departing from it across the
/// rest of the piece, so a curved span would pass validation unrefined.
const SAMPLED_PIECE_PROBES: [f64; 5] = [1.0 / 6.0, 1.0 / 3.0, 0.5, 2.0 / 3.0, 5.0 / 6.0];

fn interior_deviation(coeffs: &[f64], h: f64, carrier: impl Fn(f64) -> f64) -> f64 {
    SAMPLED_PIECE_PROBES.iter().fold(0.0_f64, |worst, &frac| {
        let tau = frac * h;
        worst.max((eval_monomial(coeffs, tau) - carrier(tau)).abs())
    })
}

fn push_interpolated_piece(
    dst: &mut Vec<Vec<f64>>,
    seg: &ContinuousSegment,
    axis: usize,
    t0: f64,
    t1: f64,
    depth: u32,
) {
    let h = t1 - t0;
    let coeffs = quintic_hermite(h, sample_axis(seg, axis, t0), sample_axis(seg, axis, t1));
    let deviation = interior_deviation(&coeffs, h, |tau| sample_axis(seg, axis, t0 + tau).position);
    let mid = 0.5 * (t0 + t1);
    if depth < SAMPLED_PIECE_MAX_DEPTH && deviation > SAMPLED_PIECE_TOL_MM && mid > t0 && mid < t1 {
        push_interpolated_piece(dst, seg, axis, t0, mid, depth + 1);
        push_interpolated_piece(dst, seg, axis, mid, t1, depth + 1);
        return;
    }
    push_piece(dst, t0, t1, coeffs);
}

fn sample_axis(seg: &ContinuousSegment, axis: usize, t: f64) -> trajectory::Pva {
    seg.eval_axis(axis, t)
        .unwrap_or_else(|e| panic!("snapshot: axis {axis} at t={t} is not evaluable: {e}"))
}

/// The unique degree-5 polynomial in `tau = t - t0` matching position,
/// velocity and acceleration at both ends of a span of width `h`.
fn quintic_hermite(h: f64, start: trajectory::Pva, end: trajectory::Pva) -> Vec<f64> {
    let d0 =
        end.position - (start.position + start.velocity * h + 0.5 * start.acceleration * h * h);
    let d1 = end.velocity - (start.velocity + start.acceleration * h);
    let d2 = end.acceleration - start.acceleration;
    let h2 = h * h;
    let h3 = h2 * h;
    vec![
        start.position,
        start.velocity,
        0.5 * start.acceleration,
        (20.0 * d0 - 8.0 * d1 * h + d2 * h2) / (2.0 * h3),
        (-30.0 * d0 + 14.0 * d1 * h - 2.0 * d2 * h2) / (2.0 * h3 * h),
        (12.0 * d0 - 6.0 * d1 * h + d2 * h2) / (2.0 * h3 * h2),
    ]
}

fn eval_monomial(coeffs: &[f64], tau: f64) -> f64 {
    coeffs.iter().rev().fold(0.0, |acc, &c| acc * tau + c)
}

/// A row narrower than the device resolution is dropped and its window handed
/// to the row on its right, which is re-expanded about the earlier start so
/// its position, velocity and acceleration stay exactly the carrier's.
fn push_piece(dst: &mut Vec<Vec<f64>>, t0: f64, t1: f64, coeffs: Vec<f64>) {
    let mut t0 = t0;
    let mut coeffs = coeffs;
    while dst
        .last()
        .is_some_and(|row| row[1] - row[0] < MIN_PIECE_DURATION_S && row[0] <= t0)
    {
        let absorbed_start = dst.pop().expect("last row present")[0];
        coeffs = shift_monomial(&coeffs, absorbed_start - t0);
        t0 = absorbed_start;
    }
    let mut row = vec![t0, t1];
    row.extend_from_slice(&trim_trailing_zeros(coeffs, t1 - t0));
    dst.push(row);
}

/// The trailing row has no right neighbour to hand a sliver window to, so a
/// sliver there extends the row on its left instead.
fn absorb_trailing_sliver(dst: &mut Vec<Vec<f64>>) {
    while dst.len() > 1
        && dst
            .last()
            .is_some_and(|row| row[1] - row[0] < MIN_PIECE_DURATION_S)
    {
        let t_end = dst.pop().expect("last row present")[1];
        dst.last_mut().expect("previous row present")[1] = t_end;
    }
}

/// Trailing coefficients are trimmed by what they actually displace over the
/// piece's own duration — `|c_k| * h^k` millimetres — not by raw magnitude:
/// on a short row a numerically large high-order coefficient moves the
/// toolhead by nothing, while on a long row a tiny one still carries the
/// endpoint. The base position is excluded from the scale so a large offset
/// cannot swallow a real high-order term.
fn trim_trailing_zeros(mut coeffs: Vec<f64>, duration_s: f64) -> Vec<f64> {
    let contribution = |power: usize, c: f64| c.abs() * duration_s.powi(power as i32);
    let scale = coeffs
        .iter()
        .enumerate()
        .skip(1)
        .fold(0.0_f64, |m, (power, &c)| m.max(contribution(power, c)));
    let negligible = 1e-12 * (scale + 1.0);
    while coeffs.len() > 1 && contribution(coeffs.len() - 1, coeffs[coeffs.len() - 1]) <= negligible
    {
        coeffs.pop();
    }
    coeffs
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
