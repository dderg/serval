use std::sync::Arc;

use crossbeam_channel::Sender;
use geometry::path::lowering::PositionProfile;
use geometry::{CornerFitConfig, Move, MoveVelocity, SurfaceTransform, VelocityLimits};
use trajectory::{AxisChainSet, ContinuousSegment, NudgeProfile};

pub const CONTIGUITY_EPS_MM: f64 = 1e-6;

pub fn advance_odometer(pos: &mut [f64], movement: &Move) {
    let length = movement.segment.s_len();
    if let Some(segment) = &movement.segment.spatial {
        let end = segment.point_at(length);
        for axis in 0..3.min(pos.len()) {
            pos[axis] = end[axis];
        }
    }
    for follower in &movement.segment.followers {
        if let Some(slot) = pos.get_mut(follower.axis_index) {
            *slot += follower.delta_over(length);
        }
    }
}

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

impl StreamConfig {
    pub(crate) fn fit_tol(&self) -> crate::lowering::FitTol {
        crate::lowering::FitTol {
            pos_mm: self.fit_tol_mm,
            accel_mm_s2: self.fit_tol_accel_mm_s2,
        }
    }
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
pub(crate) enum PostProcessError {
    #[error("segment axis count mismatch: expected {expected}, got {got}")]
    AxisCountMismatch { expected: usize, got: usize },
    #[error("axis {axis}: cannot fit shaped signal on an empty template track")]
    DegenerateAxisTrack { axis: usize },
    #[error("axis {axis}: shaping window needs unavailable history at t={t}")]
    MissingHistory { axis: usize, t: f64 },
    #[error("axis {axis}: shaping window needs unavailable lookahead at t={t}")]
    MissingLookahead { axis: usize, t: f64 },
    #[error(
        "axis {axis} ({fit_context}): fit tolerance failed after {refinement_splits} refinement \
         splits on [{t_start}, {t_end}], nearest seeds [{lower_seed} ({lower_seed_provenance}), \
         {upper_seed} ({upper_seed_provenance})], at u={probe_u}, t={probe_t}: \
         position error {position_error}/{position_budget}, velocity error \
         {velocity_error}/{velocity_budget}, acceleration error \
         {acceleration_error}/{acceleration_budget}, \
         inward-left=({left_position}, {left_velocity}, {left_acceleration}), \
         inward-right=({right_position}, {right_velocity}, {right_acceleration}), \
         source=({source_position}, {source_velocity}, {source_acceleration}), \
         candidate=({candidate_position}, {candidate_velocity}, {candidate_acceleration}), \
         signal_detail={signal_detail:?}"
    )]
    FitTolerance {
        axis: usize,
        fit_context: &'static str,
        signal_detail: Option<String>,
        t_start: f64,
        t_end: f64,
        probe_u: f64,
        lower_seed: f64,
        upper_seed: f64,
        lower_seed_provenance: &'static str,
        upper_seed_provenance: &'static str,
        probe_t: f64,
        refinement_splits: usize,
        position_error: f64,
        position_budget: f64,
        velocity_error: f64,
        velocity_budget: f64,
        acceleration_error: f64,
        acceleration_budget: f64,
        source_position: f64,
        source_velocity: f64,
        source_acceleration: f64,
        left_position: f64,
        left_velocity: f64,
        left_acceleration: f64,
        right_position: f64,
        right_velocity: f64,
        right_acceleration: f64,
        candidate_position: f64,
        candidate_velocity: f64,
        candidate_acceleration: f64,
    },
    #[error("axis {axis}: shaped sample is non-finite at t={t}")]
    NonFiniteSample { axis: usize, t: f64 },
}

pub struct PlannedMove {
    pub geometry: Move,
    pub velocity: MoveVelocity,
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
/// teardown-and-rebuild lifecycle. Every token except `Reset` requires the
/// trajectory to be at rest and must be preceded by a `Drain`; the stages
/// assert emptiness rather than draining implicitly, so a violated protocol
/// fails loudly instead of hiding a velocity discontinuity.
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
    /// Opaque to every planning stage: forwarded in order, never inspected.
    Dispatch(DispatchCommand),
}

/// Commands the dispatcher executes once everything ahead of them has been
/// dispatched. Adding one touches this enum and the dispatcher only.
#[derive(Debug)]
pub enum DispatchCommand {
    /// A pre-lowered single-axis correction (endstop nudge). The follow-up
    /// `Dwell` the sender emits advances the stream clock over its duration.
    Nudge {
        mcu_id: u32,
        axis: u8,
        motor_mask: u8,
        profile: NudgeProfile,
    },
    /// Acknowledged once everything ahead of it has been dispatched (or
    /// discarded): the pipeline-wide "everything before this point is done"
    /// fence.
    Barrier(Sender<BarrierAck>),
}

/// The dispatcher's answer to a `Barrier`.
#[derive(Debug)]
pub struct BarrierAck {
    /// Stream time the dispatched trajectory has reached; `None` when nothing
    /// has been dispatched since the last reset.
    pub dispatched_through: Option<f64>,
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
pub enum BaseItem {
    Seg(ContinuousSegment),
    Drain,
    Control(Control),
}

/// Shaper → dispatcher.
pub enum TrajectoryItem {
    Seg(ContinuousSegment),
    Parked,
    Control(Control),
}
