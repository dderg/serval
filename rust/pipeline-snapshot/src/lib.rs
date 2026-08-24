//! Drives the real pipeline stages — the same `FitStage`/`Planner`/
//! `run_lowerer`/`Shaper` types `setup_stages` wires into OS threads for a
//! live print — synchronously over unbounded channels on the calling thread.
//! No stage is reimplemented: this is the production pipeline observed with
//! its intermediate fitted-stage output (pre-axis-split spatial geometry)
//! tapped alongside the final shaped output.

use std::collections::HashMap;
use std::sync::{Arc, OnceLock};

use crossbeam_channel::{Receiver, Sender, unbounded};
use geometry::path::lowering::PositionProfile;
use geometry::path::{Arc as ArcSegment, Clothoid, CurvatureProfile, Line, PathSegment, Segment};
use geometry::{FollowerDemand, Move, SourceRange, StraightPhase, VelocityLimits};
use nurbs::ScalarNurbs;
use serde::{Deserialize, Serialize};
/// The exact state every consumer reads a carrier through. Owned by
/// `trajectory`; re-exported so a snapshot consumer needs this crate only.
pub use trajectory::continuous::Pvaj;
use trajectory::continuous::{
    AnalyticMoveSpan, BuzzProfile, NudgeProfile, SurfaceMode, interior_time_above,
    interior_time_below,
};
use trajectory::{AxisChainSet, ContinuousAxis, ContinuousSegment};

pub mod audit;
pub mod waypoints;

use motion_pipeline::fit_stage::{FitDriver, FitStage};
use motion_pipeline::planner::Planner;
use motion_pipeline::{
    BaseItem, FitTol, Lowerer, PlannedItem, Shaper, StreamConfig, StreamInput, TrajectoryItem,
};

pub use planner_config::{AxisDecl, PostProcessorDecl};

pub const SNAPSHOT_SCHEMA_VERSION: u32 = 2;
/// The E lane rides as axis 3, past the three spatial axes — the same index the
/// production bridge and the seam harness assign the extruder.
pub const EXTRUDER_AXIS: usize = 3;
// Position tolerance used when a post-processor must fit a non-polynomial signal.
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

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SeamMetric {
    pub t: f64,
    pub axis: usize,
    pub dp: f64,
    pub dv: f64,
    pub da: f64,
}

/// The full snapshot dict: the raw input path plus the exact trajectory the
/// firmware executes — the shaped carriers themselves, serialized verbatim,
/// never a polynomial fit of them — and the seam continuity metrics measured
/// from one-sided carrier evaluation.
#[derive(Debug, Serialize, Deserialize)]
pub struct Snapshot {
    pub schema_version: u32,
    pub raw_x: Vec<f64>,
    pub raw_y: Vec<f64>,
    pub trajectory: ExactTrajectory,
    /// The shaped signal before the motor-side derivative-gain stages,
    /// present exactly when some chain makes the motor command depart from it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub toolhead: Option<ExactTrajectory>,
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
    let trajectory = exact_trajectory(shaped);
    let seams = seam_metrics(&trajectory);

    Snapshot {
        schema_version: SNAPSHOT_SCHEMA_VERSION,
        raw_x: raw_points.iter().map(|p| p.0).collect(),
        raw_y: raw_points.iter().map(|p| p.1).collect(),
        traversal_time_s: trajectory.t_end(),
        trajectory,
        toolhead: toolhead.map(exact_trajectory),
        seam_max_dp: seams.max_dp,
        seam_max_dv: seams.max_dv,
        seam_max_da: seams.max_da,
        worst_seams: seams.worst,
    }
}

/// The shaped output is the pipeline's own, so a carrier it cannot represent
/// exactly is a pipeline bug, not a caller error: the snapshot refuses to
/// approximate it.
fn exact_trajectory(shaped: &[ContinuousSegment]) -> ExactTrajectory {
    ExactTrajectory::from_segments(shaped)
        .unwrap_or_else(|error| panic!("pipeline-snapshot: {error}"))
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

/// The exact trajectory the firmware executes: the shaped carriers
/// themselves. One analytic move span drives every axis of its move and one
/// shaper fit window is shared by consecutive segments, so spans and spline
/// curves are interned into tables the per-axis rows index. Tables and lanes
/// grow in emission order only, so a streaming prefix snapshot is a literal
/// prefix of the final one.
#[derive(Debug, Serialize, Deserialize)]
pub struct ExactTrajectory {
    pub spans: Vec<AnalyticSpan>,
    pub curves: Vec<SplineCurve>,
    pub axes: [Vec<CarrierRow>; 4],
    t_start: f64,
    t_end: f64,
    #[serde(skip)]
    runtime: OnceLock<Result<Runtime, ExactEvalError>>,
}

/// The window of the trajectory a carrier owns. A shaper fit window reaches
/// past the segment that carries it; the row is the part this lane answers
/// for, and both sides of every seam are read from the row that owns the
/// instant.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CarrierRow {
    pub t0: f64,
    pub t1: f64,
    pub carrier: Carrier,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Carrier {
    Analytic {
        span: usize,
        axis: usize,
    },
    Spline {
        curve: usize,
    },
    RelativeSpline {
        base_position: f64,
        curve: usize,
    },
    PiecewiseRelativeSpline {
        pieces: Vec<RelativeSplinePiece>,
    },
    Hold {
        position: f64,
    },
    Nudge {
        delta_mm: f64,
        speed_mm_s: f64,
        accel_mm_s2: f64,
        t_start: f64,
    },
    Buzz {
        base_position: f64,
        sign: f64,
        amplitude_mm: f64,
        freq_start_hz: f64,
        freq_end_hz: f64,
        duration: f64,
        ramp: f64,
        t_start: f64,
    },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RelativeSplinePiece {
    pub base_position: f64,
    pub curve: usize,
    pub t_start: f64,
    pub t_end: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SplineCurve {
    pub degree: u8,
    pub knots: Vec<f64>,
    pub control_points: Vec<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Phase {
    pub t0: f64,
    pub dt: f64,
    pub s0: f64,
    pub v0: f64,
    pub a0: f64,
    pub j: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Follower {
    pub axis_index: usize,
    pub ratio: f64,
    pub ratio_end: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Limits {
    pub max_velocity_mm_s: f64,
    pub accel_mm_s2: f64,
    pub corner_deviation_mm: f64,
    /// `None` is jerk limiting off: an infinite bound is JSON `null` on the
    /// way out and unrecoverable on the way back.
    pub max_jerk_mm_s3: Option<f64>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Spatial {
    Line {
        start: [f64; 3],
        end: [f64; 3],
    },
    Arc {
        origin: [f64; 3],
        u: [f64; 3],
        v: [f64; 3],
        radius: f64,
        start_angle: f64,
        sweep: f64,
    },
    Clothoid {
        start_pose: [f64; 3],
        u: [f64; 3],
        v: [f64; 3],
        kappa_0: f64,
        sigma: f64,
        length: f64,
    },
}

/// A jerk-limited phase law over one move's spatial geometry: the executable
/// carrier, as its constructor parameters. Rebuilt through
/// `AnalyticMoveSpan::try_new`, so a consumer evaluates the very curve the
/// planner emitted — an arc's per-axis position is never a polynomial of
/// time, and this schema never pretends otherwise.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AnalyticSpan {
    pub t_start: f64,
    pub t_end: f64,
    pub source_distance_origin: f64,
    pub phases: Vec<Phase>,
    pub axis_start_positions: Vec<f64>,
    pub surface_z_offset: Option<f64>,
    pub spatial: Option<Spatial>,
    pub virtual_path_mm: Option<f64>,
    pub followers: Vec<Follower>,
    pub feedrate_mm_s: f64,
    pub limits: Limits,
    pub source_start_line: u32,
    pub source_end_line: u32,
}

/// Which carrier owns an instant that two of them meet at. `Left` is the
/// limit from below, `Right` the limit from above, both evaluated
/// infinitesimally inside the owning row so a phase joint, a knot or a seam
/// reports the jump it really executes instead of one side of it twice.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SampleSide {
    Left,
    Right,
}

#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum ExactEvalError {
    #[error("axis {axis} is outside the snapshot's four lanes")]
    AxisOutOfRange { axis: usize },
    #[error("axis {axis} has no carrier covering t={t}")]
    OutsideDomain { axis: usize, t: f64 },
    #[error("axis {axis} carrier at t={t}: {source}")]
    Carrier {
        axis: usize,
        t: f64,
        source: trajectory::ContinuousError,
    },
    #[error("{location} cannot be represented exactly: {reason}")]
    Unrepresentable { location: String, reason: String },
}

/// The carriers the rows index. In-process this is the live carrier set the
/// shaper emitted; after a round trip through the schema it is rebuilt
/// through the runtime constructors, so evaluation runs the same code either
/// way.
#[derive(Debug)]
struct Runtime {
    axes: [Vec<ContinuousAxis>; 4],
}

/// How far outside a row's window a sample may land before it is a domain
/// error rather than the rounding of a display grid: the device's step-time
/// resolution.
const ROW_DOMAIN_SLACK_S: f64 = 1e-9;

impl ExactTrajectory {
    pub fn from_segments(shaped: &[ContinuousSegment]) -> Result<Self, ExactEvalError> {
        let mut spans = Vec::new();
        let mut curves = Vec::new();
        let mut span_slots = HashMap::new();
        let mut curve_slots = HashMap::new();
        let mut axes: [Vec<CarrierRow>; 4] = std::array::from_fn(|_| Vec::new());
        let mut carriers: [Vec<ContinuousAxis>; 4] = std::array::from_fn(|_| Vec::new());
        for seg in shaped {
            for axis in 0..4 {
                let Some(source) = seg.axes.get(axis) else {
                    continue;
                };
                let carrier = capture_carrier(
                    source,
                    &mut spans,
                    &mut span_slots,
                    &mut curves,
                    &mut curve_slots,
                )
                .map_err(|reason| ExactEvalError::Unrepresentable {
                    location: format!("axis {axis} carrier at t={}", seg.t_start),
                    reason,
                })?;
                axes[axis].push(CarrierRow {
                    t0: seg.t_start,
                    t1: seg.t_end,
                    carrier,
                });
                carriers[axis].push(source.clone());
            }
        }
        let runtime = OnceLock::new();
        runtime
            .set(Ok(Runtime { axes: carriers }))
            .expect("freshly created cell is empty");
        Ok(Self {
            spans,
            curves,
            axes,
            t_start: shaped.first().map_or(0.0, |seg| seg.t_start),
            t_end: shaped.last().map_or(0.0, |seg| seg.t_end),
            runtime,
        })
    }

    /// Rows and tables as given, with the bounds the rows themselves span.
    /// Carriers are rebuilt from the tables on first evaluation, so this
    /// assembles trajectories the pipeline never emitted — including the
    /// malformed ones an oracle has to be tested against.
    pub fn from_parts(
        spans: Vec<AnalyticSpan>,
        curves: Vec<SplineCurve>,
        axes: [Vec<CarrierRow>; 4],
    ) -> Self {
        let t_start = axes
            .iter()
            .filter_map(|rows| rows.first())
            .map(|row| row.t0)
            .filter(|t| t.is_finite())
            .fold(f64::INFINITY, f64::min);
        let t_end = axes
            .iter()
            .filter_map(|rows| rows.last())
            .map(|row| row.t1)
            .filter(|t| t.is_finite())
            .fold(f64::NEG_INFINITY, f64::max);
        Self {
            spans,
            curves,
            axes,
            t_start: if t_start.is_finite() { t_start } else { 0.0 },
            t_end: if t_end.is_finite() { t_end } else { 0.0 },
            runtime: OnceLock::new(),
        }
    }

    pub fn t_start(&self) -> f64 {
        self.t_start
    }

    pub fn t_end(&self) -> f64 {
        self.t_end
    }

    pub fn is_empty(&self) -> bool {
        self.axes.iter().all(|rows| rows.is_empty())
    }

    pub fn rows(&self, axis: usize) -> &[CarrierRow] {
        self.axes.get(axis).map_or(&[], Vec::as_slice)
    }

    /// Every instant at which some axis changes carrier, phase, knot or
    /// profile interval — the grid a consumer must land on to see the
    /// trajectory's real corners.
    pub fn breakpoints(&self) -> Vec<f64> {
        let mut out = Vec::new();
        for axis in 0..4 {
            self.append_axis_breakpoints(axis, &mut out);
        }
        sorted_dedup(out)
    }

    pub fn axis_breakpoints(&self, axis: usize) -> Vec<f64> {
        let mut out = Vec::new();
        self.append_axis_breakpoints(axis, &mut out);
        sorted_dedup(out)
    }

    /// The row's own window plus the carrier's breakpoints strictly inside
    /// it: a fit window's knots beyond the window belong to whichever row
    /// owns them.
    pub fn row_breakpoints(&self, axis: usize, row: usize) -> Vec<f64> {
        let window = &self.rows(axis)[row];
        let mut out = vec![window.t0, window.t1];
        out.extend(
            self.carrier(axis, row)
                .breakpoints()
                .into_iter()
                .filter(|t| *t > window.t0 && *t < window.t1),
        );
        sorted_dedup(out)
    }

    pub fn eval_axis(&self, axis: usize, t: f64, side: SampleSide) -> Result<Pvaj, ExactEvalError> {
        if axis >= 4 {
            return Err(ExactEvalError::AxisOutOfRange { axis });
        }
        let rows = self.rows(axis);
        if rows.is_empty() {
            return Err(ExactEvalError::OutsideDomain { axis, t });
        }
        let row = match side {
            SampleSide::Left => rows.partition_point(|row| row.t0 < t).saturating_sub(1),
            SampleSide::Right => rows.partition_point(|row| row.t1 <= t).min(rows.len() - 1),
        };
        self.eval_row(axis, row, t, side)
    }

    /// The one-sided state of a named row, for a caller that already knows
    /// which side of a seam it stands on — across a hold gap the two sides
    /// live at different instants, so the row cannot be searched for.
    pub fn eval_row(
        &self,
        axis: usize,
        row: usize,
        t: f64,
        side: SampleSide,
    ) -> Result<Pvaj, ExactEvalError> {
        let window = self
            .axes
            .get(axis)
            .and_then(|rows| rows.get(row))
            .ok_or(ExactEvalError::AxisOutOfRange { axis })?;
        if t < window.t0 - ROW_DOMAIN_SLACK_S || t > window.t1 + ROW_DOMAIN_SLACK_S {
            return Err(ExactEvalError::OutsideDomain { axis, t });
        }
        let first_interior = interior_time_above(window.t0);
        let last_interior = interior_time_below(window.t1);
        let t_eval = match side {
            SampleSide::Left => interior_time_below(t),
            SampleSide::Right => interior_time_above(t),
        }
        .clamp(
            first_interior.min(last_interior),
            first_interior.max(last_interior),
        );
        self.runtime()?.axes[axis][row]
            .eval_pvaj(t_eval)
            .map_err(|source| ExactEvalError::Carrier {
                axis,
                t: t_eval,
                source,
            })
    }

    fn append_axis_breakpoints(&self, axis: usize, out: &mut Vec<f64>) {
        for row in 0..self.rows(axis).len() {
            out.extend(self.row_breakpoints(axis, row));
        }
    }

    fn carrier(&self, axis: usize, row: usize) -> &ContinuousAxis {
        &self
            .runtime()
            .unwrap_or_else(|error| {
                panic!("pipeline-snapshot: exact trajectory is not evaluable: {error}")
            })
            .axes[axis][row]
    }

    fn runtime(&self) -> Result<&Runtime, ExactEvalError> {
        self.runtime
            .get_or_init(|| self.rebuild())
            .as_ref()
            .map_err(Clone::clone)
    }

    fn rebuild(&self) -> Result<Runtime, ExactEvalError> {
        let unrepresentable = |location: String| {
            move |reason: String| ExactEvalError::Unrepresentable { location, reason }
        };
        let mut spans = Vec::with_capacity(self.spans.len());
        for (index, span) in self.spans.iter().enumerate() {
            spans.push(Arc::new(
                span.rebuild()
                    .map_err(unrepresentable(format!("analytic span {index}")))?,
            ));
        }
        let mut curves = Vec::with_capacity(self.curves.len());
        for (index, curve) in self.curves.iter().enumerate() {
            curves.push(Arc::new(
                curve
                    .rebuild()
                    .map_err(unrepresentable(format!("spline curve {index}")))?,
            ));
        }
        let mut axes: [Vec<ContinuousAxis>; 4] = std::array::from_fn(|_| Vec::new());
        for (axis, rows) in self.axes.iter().enumerate() {
            for (index, row) in rows.iter().enumerate() {
                axes[axis].push(
                    row.rebuild(&spans, &curves)
                        .map_err(unrepresentable(format!("axis {axis} row {index}")))?,
                );
            }
        }
        Ok(Runtime { axes })
    }
}

fn sorted_dedup(mut values: Vec<f64>) -> Vec<f64> {
    values.sort_by(f64::total_cmp);
    values.dedup();
    values
}

/// Interning is by carrier identity, not by value: the shaped segments own
/// every `Arc` for the length of the walk, so an address identifies one
/// carrier, and two numerically equal spans of different moves stay distinct
/// entries of the table.
fn capture_carrier(
    source: &ContinuousAxis,
    spans: &mut Vec<AnalyticSpan>,
    span_slots: &mut HashMap<usize, usize>,
    curves: &mut Vec<SplineCurve>,
    curve_slots: &mut HashMap<usize, usize>,
) -> Result<Carrier, String> {
    let mut intern_curve = |curve: &Arc<ScalarNurbs>| {
        intern(curve_slots, curves, Arc::as_ptr(curve) as usize, || {
            Ok(SplineCurve::capture(curve))
        })
        .expect("capturing a spline curve cannot fail")
    };
    Ok(match source {
        ContinuousAxis::Analytic { span, axis } => Carrier::Analytic {
            span: intern(span_slots, spans, Arc::as_ptr(span) as usize, || {
                AnalyticSpan::capture(span)
            })?,
            axis: *axis,
        },
        ContinuousAxis::Spline(curve) => Carrier::Spline {
            curve: intern_curve(curve),
        },
        ContinuousAxis::RelativeSpline {
            base_position,
            curve,
        } => Carrier::RelativeSpline {
            base_position: *base_position,
            curve: intern_curve(curve),
        },
        ContinuousAxis::PiecewiseRelativeSpline(pieces) => Carrier::PiecewiseRelativeSpline {
            pieces: pieces
                .iter()
                .map(|piece| RelativeSplinePiece {
                    base_position: piece.base_position,
                    curve: intern_curve(&piece.curve),
                    t_start: piece.t_start,
                    t_end: piece.t_end,
                })
                .collect(),
        },
        ContinuousAxis::Hold { position, .. } => Carrier::Hold {
            position: *position,
        },
        ContinuousAxis::Nudge(profile) => Carrier::Nudge {
            delta_mm: profile.delta_mm(),
            speed_mm_s: profile.speed_mm_s(),
            accel_mm_s2: profile.accel_mm_s2(),
            t_start: profile.t_start(),
        },
        ContinuousAxis::Buzz {
            base_position,
            sign,
            profile,
        } => Carrier::Buzz {
            base_position: *base_position,
            sign: *sign,
            amplitude_mm: profile.amplitude_mm(),
            freq_start_hz: profile.freq_start_hz(),
            freq_end_hz: profile.freq_end_hz(),
            duration: profile.duration(),
            ramp: profile.ramp(),
            t_start: profile.t_start(),
        },
    })
}

fn intern<T>(
    slots: &mut HashMap<usize, usize>,
    table: &mut Vec<T>,
    identity: usize,
    capture: impl FnOnce() -> Result<T, String>,
) -> Result<usize, String> {
    if let Some(slot) = slots.get(&identity) {
        return Ok(*slot);
    }
    let slot = table.len();
    table.push(capture()?);
    slots.insert(identity, slot);
    Ok(slot)
}

impl CarrierRow {
    fn rebuild(
        &self,
        spans: &[Arc<AnalyticMoveSpan>],
        curves: &[Arc<ScalarNurbs>],
    ) -> Result<ContinuousAxis, String> {
        let curve_at = |slot: usize| {
            curves
                .get(slot)
                .cloned()
                .ok_or_else(|| format!("spline curve {slot} is not in the curve table"))
        };
        match &self.carrier {
            Carrier::Analytic { span, axis } => Ok(ContinuousAxis::Analytic {
                span: spans
                    .get(*span)
                    .cloned()
                    .ok_or_else(|| format!("analytic span {span} is not in the span table"))?,
                axis: *axis,
            }),
            Carrier::Spline { curve } => Ok(ContinuousAxis::Spline(curve_at(*curve)?)),
            Carrier::RelativeSpline {
                base_position,
                curve,
            } => Ok(ContinuousAxis::RelativeSpline {
                base_position: *base_position,
                curve: curve_at(*curve)?,
            }),
            Carrier::PiecewiseRelativeSpline { pieces } => {
                let rebuilt = pieces
                    .iter()
                    .map(|piece| {
                        Ok(trajectory::RelativeSplinePiece {
                            base_position: piece.base_position,
                            curve: curve_at(piece.curve)?,
                            t_start: piece.t_start,
                            t_end: piece.t_end,
                        })
                    })
                    .collect::<Result<Vec<_>, String>>()?;
                ContinuousAxis::try_piecewise_relative_spline(rebuilt.into())
                    .map_err(|error| error.to_string())
            }
            Carrier::Hold { position } => Ok(ContinuousAxis::Hold {
                position: *position,
                t_start: self.t0,
                t_end: self.t1,
            }),
            Carrier::Nudge {
                delta_mm,
                speed_mm_s,
                accel_mm_s2,
                t_start,
            } => NudgeProfile::try_new(*delta_mm, *speed_mm_s, *accel_mm_s2, *t_start)
                .map(ContinuousAxis::Nudge)
                .map_err(|error| format!("{error:?}")),
            Carrier::Buzz {
                base_position,
                sign,
                amplitude_mm,
                freq_start_hz,
                freq_end_hz,
                duration,
                ramp,
                t_start,
            } => BuzzProfile::try_new(
                *amplitude_mm,
                *freq_start_hz,
                *freq_end_hz,
                *duration,
                *ramp,
                *t_start,
            )
            .map(|profile| ContinuousAxis::Buzz {
                base_position: *base_position,
                sign: *sign,
                profile: Arc::new(profile),
            })
            .map_err(|error| format!("{error:?}")),
        }
    }
}

impl SplineCurve {
    fn capture(curve: &ScalarNurbs) -> Self {
        Self {
            degree: curve.degree(),
            knots: curve.knots().to_vec(),
            control_points: curve.control_points().to_vec(),
        }
    }

    fn rebuild(&self) -> Result<ScalarNurbs, String> {
        ScalarNurbs::try_new(self.degree, self.knots.clone(), self.control_points.clone())
            .map_err(|error| format!("{error:?}"))
    }
}

impl AnalyticSpan {
    fn capture(span: &AnalyticMoveSpan) -> Result<Self, String> {
        let surface_z_offset = match &span.surface {
            SurfaceMode::None => None,
            SurfaceMode::Constant(offset) => Some(*offset),
            SurfaceMode::Variable(_) => {
                return Err(
                    "a variable surface transform has no exact jerk; the shaper must \
                            materialize it as a spline axis before the snapshot serializes it"
                        .to_string(),
                );
            }
        };
        Ok(Self {
            t_start: span.t_start,
            t_end: span.t_end,
            source_distance_origin: span.source_distance_origin,
            phases: span
                .phases
                .iter()
                .map(|phase| Phase {
                    t0: phase.t0,
                    dt: phase.dt,
                    s0: phase.s0,
                    v0: phase.v0,
                    a0: phase.a0,
                    j: phase.j,
                })
                .collect(),
            axis_start_positions: span.axis_start_positions.to_vec(),
            surface_z_offset,
            spatial: span.source.segment.spatial.as_ref().map(Spatial::capture),
            virtual_path_mm: span.source.segment.virtual_path_mm,
            followers: span
                .source
                .segment
                .followers
                .iter()
                .map(|follower| Follower {
                    axis_index: follower.axis_index,
                    ratio: follower.ratio,
                    ratio_end: follower.ratio_end,
                })
                .collect(),
            feedrate_mm_s: span.source.feedrate_mm_s,
            limits: Limits::capture(span.source.limits),
            source_start_line: span.source.source.start_line,
            source_end_line: span.source.source.end_line,
        })
    }

    fn rebuild(&self) -> Result<AnalyticMoveSpan, String> {
        let followers: Vec<FollowerDemand> = self
            .followers
            .iter()
            .map(|follower| FollowerDemand {
                axis_index: follower.axis_index,
                ratio: follower.ratio,
                ratio_end: follower.ratio_end,
            })
            .collect();
        let segment = match (&self.spatial, self.virtual_path_mm) {
            (Some(spatial), None) => PathSegment::try_new(spatial.rebuild()?, followers),
            (None, Some(virtual_path_mm)) => {
                PathSegment::try_new_virtual(followers, virtual_path_mm)
            }
            _ => {
                return Err(
                    "an analytic span carries exactly one of a spatial segment or a \
                            virtual path length"
                        .to_string(),
                );
            }
        }
        .map_err(|error| format!("{error:?}"))?;
        let source = Move {
            segment,
            feedrate_mm_s: self.feedrate_mm_s,
            limits: self.limits.rebuild().map_err(str::to_string)?,
            source: SourceRange {
                start_line: self.source_start_line,
                end_line: self.source_end_line,
            },
        };
        AnalyticMoveSpan::try_new(
            source,
            self.phases
                .iter()
                .map(|phase| StraightPhase {
                    t0: phase.t0,
                    dt: phase.dt,
                    s0: phase.s0,
                    v0: phase.v0,
                    a0: phase.a0,
                    j: phase.j,
                })
                .collect(),
            self.source_distance_origin,
            self.t_start,
            self.t_end,
            self.axis_start_positions.iter().copied().collect(),
            match self.surface_z_offset {
                Some(offset) => SurfaceMode::Constant(offset),
                None => SurfaceMode::None,
            },
        )
        .map_err(|error| error.to_string())
    }
}

impl Limits {
    fn capture(limits: VelocityLimits) -> Self {
        Self {
            max_velocity_mm_s: limits.max_velocity_mm_s,
            accel_mm_s2: limits.accel_mm_s2,
            corner_deviation_mm: limits.corner_deviation_mm,
            max_jerk_mm_s3: limits
                .max_jerk_mm_s3
                .is_finite()
                .then_some(limits.max_jerk_mm_s3),
        }
    }

    fn rebuild(&self) -> Result<VelocityLimits, &'static str> {
        VelocityLimits::try_new(
            self.max_velocity_mm_s,
            self.accel_mm_s2,
            self.corner_deviation_mm,
            self.max_jerk_mm_s3.unwrap_or(f64::INFINITY),
        )
    }
}

impl Spatial {
    fn capture(segment: &Segment) -> Self {
        match segment {
            Segment::Line(line) => Self::Line {
                start: line.start,
                end: line.end,
            },
            Segment::Arc(arc) => Self::Arc {
                origin: arc.origin,
                u: arc.u,
                v: arc.v,
                radius: arc.radius,
                start_angle: arc.start_angle,
                sweep: arc.sweep,
            },
            Segment::Clothoid(clothoid) => Self::Clothoid {
                start_pose: clothoid.start_pose,
                u: clothoid.u,
                v: clothoid.v,
                kappa_0: clothoid.kappa_0,
                sigma: clothoid.sigma,
                length: clothoid.length,
            },
        }
    }

    fn rebuild(&self) -> Result<Segment, String> {
        match self {
            Self::Line { start, end } => Line::try_new(*start, *end).map(Segment::Line),
            Self::Arc {
                origin,
                u,
                v,
                radius,
                start_angle,
                sweep,
            } => ArcSegment::try_new(*origin, *u, *v, *radius, *start_angle, *sweep)
                .map(Segment::Arc),
            Self::Clothoid {
                start_pose,
                u,
                v,
                kappa_0,
                sigma,
                length,
            } => Clothoid::try_new(*start_pose, *u, *v, *kappa_0, *sigma, *length)
                .map(Segment::Clothoid),
        }
        .map_err(|error| format!("{error:?}"))
    }
}

pub struct SeamMetrics {
    pub max_dp: [f64; 4],
    pub max_dv: [f64; 4],
    pub max_da: [f64; 4],
    pub worst: Vec<SeamMetric>,
}

const WORST_SEAM_COUNT: usize = 10;

/// Every instant two carriers — or two phases of one carrier — meet at,
/// measured as the jump between the exact one-sided states there. Nothing is
/// differentiated or sampled: both sides come from the carrier that owns
/// them.
pub fn seam_metrics(traj: &ExactTrajectory) -> SeamMetrics {
    let mut out = SeamMetrics {
        max_dp: [0.0; 4],
        max_dv: [0.0; 4],
        max_da: [0.0; 4],
        worst: Vec::new(),
    };
    for axis in 0..4 {
        let rows = traj.rows(axis);
        for (index, row) in rows.iter().enumerate() {
            for t in traj.row_breakpoints(axis, index) {
                if t > row.t0 && t < row.t1 {
                    out.record(seam_at(traj, axis, (index, t), (index, t)));
                }
            }
            if let Some(next) = rows.get(index + 1) {
                out.record(seam_at(traj, axis, (index, row.t1), (index + 1, next.t0)));
            }
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

impl SeamMetrics {
    fn record(&mut self, seam: SeamMetric) {
        self.max_dp[seam.axis] = self.max_dp[seam.axis].max(seam.dp);
        self.max_dv[seam.axis] = self.max_dv[seam.axis].max(seam.dv);
        self.max_da[seam.axis] = self.max_da[seam.axis].max(seam.da);
        self.worst.push(seam);
    }
}

fn seam_at(
    traj: &ExactTrajectory,
    axis: usize,
    left: (usize, f64),
    right: (usize, f64),
) -> SeamMetric {
    let before = one_sided(traj, axis, left.0, left.1, SampleSide::Left);
    let after = one_sided(traj, axis, right.0, right.1, SampleSide::Right);
    SeamMetric {
        t: right.1,
        axis,
        dp: (after.position - before.position).abs(),
        dv: (after.velocity - before.velocity).abs(),
        da: (after.acceleration - before.acceleration).abs(),
    }
}

fn one_sided(traj: &ExactTrajectory, axis: usize, row: usize, t: f64, side: SampleSide) -> Pvaj {
    traj.eval_row(axis, row, t, side).unwrap_or_else(|error| {
        panic!("pipeline-snapshot: seam probe on axis {axis} row {row} at t={t}: {error}")
    })
}

#[cfg(test)]
mod tests;
