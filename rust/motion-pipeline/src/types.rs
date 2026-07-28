use std::sync::Arc;
use std::time::Instant;

use crossbeam_channel::{Receiver, Sender};
use geometry::{CornerFitConfig, Move, MoveVelocity, SurfaceTransform, VelocityLimits};
use trajectory::{AxisChainSet, ShapedSegment};

pub const CONTIGUITY_EPS_MM: f64 = 1e-6;

#[derive(Debug, Clone, Copy)]
pub struct StreamConfig {
    pub corner: CornerFitConfig,
    pub integration_tol: f64,
    pub max_extrude_only_velocity_mm_s: f64,
    pub max_extrude_only_accel_mm_s2: f64,
    pub fit_tol_mm: f64,
    pub fit_tol_accel_mm_s2: f64,
    /// Backstop cap on the planner's look-ahead window. A normal continuous
    /// path always offers a clean seam, so the window stays small; this only
    /// fires for a pathological window with no clean seam within the finality
    /// barrier at all (e.g. a single move longer than the whole look-ahead) —
    /// without it such a window would grow unbounded. It is a safety net, not
    /// the steady-state path.
    pub max_buffer_moves: usize,
    /// Path limits for planner-internal moves (homing). Stream moves submitted
    /// through the bridge carry their own per-move limits; this is the fallback
    /// used when the engine constructs a move itself.
    pub limits: VelocityLimits,
}

#[derive(Debug)]
pub enum StreamError {
    /// A move entered the pipeline whose spatial start does not meet the
    /// toolhead where the previous move left it. Real slicer output is always
    /// position-contiguous; a gap means the move stream was stitched wrong
    /// upstream. Caught at ingress so the offending move is named there, not
    /// as a downstream `ZeroMotion` deep in the fit stage.
    Discontinuity {
        line_no: u32,
        expected: [f64; 3],
        got: [f64; 3],
        gap_mm: f64,
    },
}

impl std::fmt::Display for StreamError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Discontinuity {
                line_no,
                expected,
                got,
                gap_mm,
            } => write!(
                f,
                "discontinuous move at line {line_no}: starts at {got:?} but the \
                 toolhead is at {expected:?} ({gap_mm:.6}mm gap) — move stream is \
                 not position-contiguous"
            ),
        }
    }
}

impl std::error::Error for StreamError {}

#[derive(Debug, thiserror::Error)]
pub enum PostProcessError {
    #[error("segment axis count mismatch: expected {expected}, got {got}")]
    AxisCountMismatch { expected: usize, got: usize },
    #[error("axis {axis}: cannot fit shaped signal on an empty template track")]
    DegenerateAxisTrack { axis: usize },
    #[error("axis {axis}: shaping window needs unavailable history at t={t}")]
    MissingHistory { axis: usize, t: f64 },
    #[error("axis {axis}: shaping window needs unavailable lookahead at t={t}")]
    MissingLookahead { axis: usize, t: f64 },
    #[error("axis {axis}: shaped sample is non-finite at t={t}")]
    NonFiniteSample { axis: usize, t: f64 },
    #[error(
        "axis {axis}: nonlinear advance composition missed the fit budget at t={t} over a {span_s}s span"
    )]
    AdvanceFitUnresolved { axis: usize, t: f64, span_s: f64 },
}

pub struct PlannedMove {
    pub geometry: Move,
    pub velocity: MoveVelocity,
}

/// Lowerer output: a dispatchable segment plus whether the trajectory is at
/// rest at its end — the shaper may clamp its convolution window past a rest
/// point (the signal is constant there), never past a moving end.
pub struct LoweredSegment {
    pub seg: ShapedSegment,
    pub rest_at_end: bool,
}

pub struct PipelineHandle {
    pub input: Sender<StreamInput>,
    pub output: Receiver<ShapedItem>,
    pub threads: Vec<std::thread::JoinHandle<()>>,
}

/// What flows into the fit stage and planner: geometry, the command to stop
/// looking ahead, or an ordered control token. `Drain` makes each stage
/// resolve and emit everything it is holding — the fit stage finalizes runs
/// and blends, the planner materializes the brake-to-rest — exactly what a
/// closed input does, but without ending the stream. The stages themselves
/// never consult a clock or peek at channel occupancy; whoever owns the
/// notion of time decides when to send `Drain`.
#[derive(Debug)]
pub enum StreamInput {
    Move(Move),
    Drain,
    Control(Control),
}

impl From<Move> for StreamInput {
    fn from(m: Move) -> Self {
        Self::Move(m)
    }
}

/// Ordered control tokens that flow through every stage with the geometry.
/// The pipeline is set up once and lives forever; these replace the old
/// teardown-and-rebuild lifecycle. Tokens that require the trajectory to be
/// at rest (`Dwell`, `SetAxisChains`, `Nudge`, `Barrier` after a flush) must
/// be preceded by a `Drain`; the stages assert emptiness rather than draining
/// implicitly, so a violated protocol fails loudly instead of hiding a
/// velocity discontinuity.
#[derive(Debug)]
pub enum Control {
    /// Advance the trajectory clock without motion (lowerer applies it).
    Dwell { secs: f64 },
    /// Drop all buffered state and restart the timeline at rest at `pos`.
    /// The sender is responsible for gating the dispatcher (discard) so
    /// motion already lowered ahead of this token is dropped, not executed.
    Reset { pos: Vec<f64> },
    /// Swap the post-processing chains (lowerer and shaper apply it).
    SetAxisChains(AxisChainSet),
    /// Swap the bed surface transform future moves are lowered against
    /// (lowerer applies it); `None` clears it. Upstream of the lowerer is
    /// gcode space, downstream is machine space — the warp is part of
    /// lowering, so the planner never sees it. `gcode_z_rebase` is the gcode
    /// Z that maps to the unchanged machine Z through the *new* transform,
    /// computed by the sender: rebasing the odometers keeps the physical
    /// position invariant across the swap instead of stepping Z by the
    /// correction delta on the next move.
    SetMesh {
        mesh: Option<Arc<SurfaceTransform>>,
        gcode_z_rebase: f64,
    },
    /// A pre-lowered single-axis correction (endstop nudge) for the
    /// dispatcher: it never touches the planned trajectory, so the stages
    /// forward it untouched. The follow-up `Dwell` the sender emits advances
    /// the stream clock over the nudge's duration.
    Nudge {
        mcu_id: u32,
        pieces: Vec<NudgePiece>,
    },
    /// Acknowledged by the dispatcher once everything ahead of it has been
    /// dispatched (or discarded): the pipeline-wide "everything before this
    /// point is done" fence.
    Barrier(Sender<BarrierAck>),
}

/// One phase of a nudge profile: a polynomial piece for a single axis,
/// already in stream time.
#[derive(Debug, Clone)]
pub struct NudgePiece {
    pub axis: u8,
    pub motor_mask: u8,
    pub piece: nurbs::bezier::BezierPiece,
}

/// The dispatcher's answer to a `Barrier`.
#[derive(Debug)]
pub struct BarrierAck {
    /// Stream time the dispatched trajectory has reached; `None` when nothing
    /// has been dispatched since the last reset.
    pub dispatched_through: Option<f64>,
    /// Host instant of the first dispatch since the last reset, for
    /// projecting stream time onto the wall clock.
    pub sync_instant: Option<Instant>,
    /// Dispatch errors captured since the previous barrier (error capture is
    /// enabled by the homing paths; otherwise a dispatch error is fatal).
    pub result: Result<(), String>,
}

/// Planner → lowerer. `Drain` marks that the trajectory emitted so far ends
/// in a materialized brake-to-rest: the lowerer holds the timeline at that
/// rest for the chain set's forward support before the next move, and the
/// shaper flushes its buffered tail with the convolution window clamped —
/// which the hold makes exact rather than speculative.
pub enum PlannedItem {
    Move(PlannedMove),
    Drain,
    Control(Control),
}

/// Lowerer → shaper.
pub enum LoweredItem {
    Seg(LoweredSegment),
    Drain,
    Control(Control),
}

/// Shaper → dispatcher.
pub enum ShapedItem {
    Seg(ShapedSegment),
    Control(Control),
}

/// Jerk-limited time to decelerate from `v` to rest under accel limit `a` and
/// jerk limit `j`: `v/a + a/j` once the ramp reaches `a` (`v > a²/j`), else the
/// triangular `2·√(v/j)`. Curvature only slows a real stop, so this
/// straight-line time is a safe over-estimate.
#[must_use]
pub fn jerk_limited_brake_time(v: f64, a: f64, j: f64) -> f64 {
    if v <= 0.0 {
        return 0.0;
    }
    if a <= 0.0 || j <= 0.0 {
        return f64::INFINITY;
    }
    if v > a * a / j {
        v / a + a / j
    } else {
        2.0 * (v / j).sqrt()
    }
}
