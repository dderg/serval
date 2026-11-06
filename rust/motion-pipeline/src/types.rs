use std::sync::Arc;

use crossbeam_channel::Sender;
use geometry::path::lowering::PositionProfile;
use geometry::{
    CornerFitConfig, Move, MoveVelocity, StepCapacity, SurfaceTransform, VelocityLimits,
};
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

#[derive(Debug, Clone)]
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
    /// The mcus' measured step-dispatch capacity every planned move is
    /// held within; absent where no mcu steps the motors (snapshots,
    /// replays, simulated mcus).
    pub step_capacity: Option<Arc<StepCapacity>>,
}

impl StreamConfig {
    pub(crate) fn fit_tol(&self) -> crate::lowering::FitTol {
        crate::lowering::FitTol::absolute(self.fit_tol_mm, self.fit_tol_accel_mm_s2)
    }

    /// The config with the corner budget held on the path `chains` shape:
    /// the X and Y kernels' rounding, see [`geometry::ShapedCorners`].
    pub(crate) fn with_shaped_corners(self, chains: &AxisChainSet) -> Self {
        let kernel = |axis: usize| chains.chains.get(axis).and_then(|chain| chain.kernel());
        let shaped_corners = geometry::ShapedCorners::interned([kernel(0), kernel(1)]);
        if let Some(corners) = shaped_corners {
            log_corner_floor(corners, self.limits);
        }
        Self {
            corner: geometry::CornerFitConfig {
                shaped_corners,
                ..self.corner
            },
            ..self
        }
    }
}

/// What the shaper alone rounds a stopped 90° and 180° corner by at the
/// config's acceleration, and for an explicit `corner_deviation` the
/// accelerations at which each fits it: the trade a budget under the floor
/// buys with slower corners.
fn log_corner_floor(corners: &geometry::ShapedCorners, limits: geometry::VelocityLimits) {
    let accel = limits.accel_mm_s2;
    let floor_90 = corners.floor_mm(std::f64::consts::FRAC_PI_2, accel);
    let floor_180 = corners.floor_mm(std::f64::consts::PI, accel);
    let (budget, fits_at_90, fits_at_180) = match limits.corner {
        geometry::CornerBudget::Shaped(mm) => (
            mm,
            accel * (mm / floor_90).min(1.0),
            accel * (mm / floor_180).min(1.0),
        ),
        geometry::CornerBudget::OverFloor(_) | geometry::CornerBudget::ScvEquivalent(_) => {
            (f64::NAN, accel, accel)
        }
    };
    tracing::info!(
        subsystem = "motion",
        event = "corner_floor",
        max_accel = accel,
        corner_budget = ?limits.corner,
        floor_90_mm = floor_90,
        floor_180_mm = floor_180,
        corner_deviation_mm = budget,
        accel_fitting_90 = fits_at_90,
        accel_fitting_180 = fits_at_180,
        "the shaper rounds a stopped corner by {floor_90:.4} mm at 90° and {floor_180:.4} mm at \
         180° at {accel} mm/s²; corners whose stop rounds past the budget slow to fit it \
         (90° at {fits_at_90:.0} mm/s², 180° at {fits_at_180:.0} mm/s²)"
    );
}

#[derive(Debug)]
pub enum StreamError {
    /// A move entered the pipeline whose spatial start does not meet the
    /// toolhead where the previous move left it. Real slicer output is always
    /// position-contiguous; a gap means the move stream was stitched wrong
    /// upstream. The live ingress names the offending move before it enters;
    /// the lowerer refuses it for every other driver (a pipeline built at a
    /// home the first move does not start from), before the shaper smooths
    /// the jump into an unfittable pulse.
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
        "axis {axis}: the advance on this ratio-ramped follower would step its position by \
         {step_mm} mm at t={t}, where the velocity it reads steps from {velocity_before} to \
         {velocity_after} mm/s. A ratio ramp leaves the projected extrusion velocity-continuous \
         and no kernel follows the advance, so the step would reach the motor as is"
    )]
    RatioRampedPositionStep {
        axis: usize,
        t: f64,
        step_mm: f64,
        velocity_before: f64,
        velocity_after: f64,
    },
    #[error(
        "axis {axis}: this ratio-ramped follower would step its velocity from \
         {velocity_before} to {velocity_after} mm/s at t={t}, beyond the {resolution} mm/s \
         allowance (its instantaneous_corner_velocity plus the fit resolution). No kernel \
         follows a ratio ramp, so the step would reach the motor as is. The advance turns \
         every acceleration step of the leaders it rides into one, and the planner holds \
         those of unshaped leaders within instantaneous_corner_velocity: a larger one is a \
         planner bug, or comes from a leader stepping where the planner cannot see it"
    )]
    RatioRampedVelocityStep {
        axis: usize,
        t: f64,
        velocity_before: f64,
        velocity_after: f64,
        resolution: f64,
    },
    #[error(
        "axis {axis} ({fit_context}): the fit over [{t_start}, {t_end}] cannot follow its \
         source at t={t}, where the advance asks the motor for {demand_mm_s2} mm/s^2, past its \
         {limit_mm_s2} mm/s^2 acceleration limit by more than the {budget_mm_s2} mm/s^2 fit \
         budget. No kernel follows the advance, so its acceleration reaches the motor as is. \
         The planner holds an advanced extruder's demand r·(a + k·j) within the limit: a \
         larger one is a planner bug, or a track it did not plan"
    )]
    AdvancedDemandOverLimit {
        axis: usize,
        fit_context: &'static str,
        t_start: f64,
        t_end: f64,
        t: f64,
        demand_mm_s2: f64,
        limit_mm_s2: f64,
        budget_mm_s2: f64,
    },
    #[error(
        "axis {axis}: an unshaped leader leaves its curvature cap as a square root of the \
         time on [{from}, {to}], where the advance on this follower changes its velocity by \
         {change} mm/s faster than any fit resolves — beyond the {allowance} mm/s allowance \
         (its instantaneous_corner_velocity plus the leaders' fit resolution) such a change \
         may take"
    )]
    UnshapedLeaderOnsetVelocityStep {
        axis: usize,
        from: f64,
        to: f64,
        change: f64,
        allowance: f64,
    },
    #[error(
        "axis {axis}: {fit_context} span [{t_start}, {t_end}] has neither exact polynomials nor \
         an enclosure there to prove a piece against: every fitted piece is proven, so the \
         signal's formula there lacks a model"
    )]
    UnprovableSpan {
        axis: usize,
        fit_context: &'static str,
        t_start: f64,
        t_end: f64,
    },
    #[error(
        "axis {axis}: follower seam weld over [{t_start}, {t_end}] would move the emitted track \
         by up to {displacement_mm} mm (budget {position_budget_mm}) and its acceleration by up \
         to {acceleration_mm_s2} mm/s^2 (budget {acceleration_budget_mm_s2}): the emitted state \
         (p, v) = ({emitted_position}, {emitted_velocity}) does not continue the fitted start \
         (p, v) = ({fitted_position}, {fitted_velocity})"
    )]
    SeamWeld {
        axis: usize,
        t_start: f64,
        t_end: f64,
        displacement_mm: f64,
        position_budget_mm: f64,
        acceleration_mm_s2: f64,
        acceleration_budget_mm_s2: f64,
        emitted_position: f64,
        emitted_velocity: f64,
        fitted_position: f64,
        fitted_velocity: f64,
    },
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
    #[error(
        "axis {axis} ({fit_context}): the signal's velocity changes by {velocity_change} across \
         [{t_start}, {t_end}], a span the fit cannot split further, while its acceleration \
         there carries [{carried_low}, {carried_high}] (slack {velocity_slack}): a velocity \
         step narrower than the resolution floor; position error \
         {position_error}/{position_budget}, velocity error {velocity_error}, acceleration \
         error {acceleration_error}/{acceleration_budget}"
    )]
    VelocityStepAtFitFloor {
        axis: usize,
        fit_context: &'static str,
        t_start: f64,
        t_end: f64,
        velocity_change: f64,
        carried_low: f64,
        carried_high: f64,
        velocity_slack: f64,
        position_error: f64,
        position_budget: f64,
        velocity_error: f64,
        acceleration_error: f64,
        acceleration_budget: f64,
    },
    #[error(
        "axis {axis} ({fit_context}): the velocity break at {break_at} lies within the {floor} s \
         fit resolution floor of {moved_onto}; moving its velocity step {velocity_step} there \
         shifts position by {position_shift}, beyond the {position_budget} tolerance"
    )]
    CrowdedVelocityBreaks {
        axis: usize,
        fit_context: &'static str,
        break_at: f64,
        moved_onto: f64,
        velocity_step: f64,
        position_shift: f64,
        position_budget: f64,
        floor: f64,
    },
    #[error(
        "axis {axis}: the acceleration break hidden between {seam} and {past_sliver} changes \
         velocity by {velocity_step} across the sliver; moving that step onto the seam shifts \
         position by {position_shift}, beyond the {position_budget} tolerance"
    )]
    SliverVelocityCarry {
        axis: usize,
        seam: f64,
        past_sliver: f64,
        velocity_step: f64,
        position_shift: f64,
        position_budget: f64,
    },
    #[error("axis {axis}: shaped sample is non-finite at t={t}")]
    NonFiniteSample { axis: usize, t: f64 },
}

/// A committed move: its geometry, its planned velocity, and the extrusion
/// budget its plan took from the moves around it, which a re-plan of the
/// move must take again.
pub struct PlannedMove {
    pub geometry: Move,
    pub velocity: MoveVelocity,
    pub budget: geometry::ExtrusionBudget,
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

/// What a pipeline was fed, as the parity gate compares it: every move in
/// full, the stops at rest, dwells and resets, and any other control token
/// by kind.
#[derive(Debug, Clone, PartialEq)]
pub enum InputTrace {
    Move(Move),
    Drain,
    Dwell { secs: f64 },
    Reset { pos: Vec<f64> },
    Control(&'static str),
}

impl InputTrace {
    #[must_use]
    pub fn of(input: &StreamInput) -> Self {
        match input {
            StreamInput::Move(m) => Self::Move(m.clone()),
            StreamInput::Drain => Self::Drain,
            StreamInput::Control(Control::Dwell { secs }) => Self::Dwell { secs: *secs },
            StreamInput::Control(Control::Reset { pos }) => Self::Reset { pos: pos.clone() },
            StreamInput::Control(Control::SetAxisChains(_)) => Self::Control("SetAxisChains"),
            StreamInput::Control(Control::SetMesh { .. }) => Self::Control("SetMesh"),
            StreamInput::Control(Control::Dispatch(DispatchCommand::Nudge { .. })) => {
                Self::Control("Nudge")
            }
            StreamInput::Control(Control::Dispatch(DispatchCommand::Barrier(_))) => {
                Self::Control("Barrier")
            }
        }
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
