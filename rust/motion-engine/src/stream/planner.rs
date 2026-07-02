use crossbeam_channel::{Receiver, Sender};
use geometry::path::lowering::PositionProfile;
use geometry::path::{CurvatureProfile, Segment};
use geometry::{BoundaryState, Move, VelocityProfile, plan_velocity_stops};

use super::{
    Control, PlannedItem, PlannedMove, StreamConfig, StreamInput, jerk_limited_brake_time,
};

/// Cost governor, not a lookahead bound: velocity planning is ~linear in the
/// window, so re-planning on every arriving move would be quadratic. The window
/// keeps absorbing input and a re-plan fires once this many moves have arrived
/// since the last one (an input-empty drain still plans immediately).
const REPLAN_BATCH_MOVES: usize = 64;

/// Second pipeline stage: plans jerk-limited S-curve velocity over the
/// incoming geometry and emits `PlannedMove`s whose velocity bodies are final
/// under any future append.
///
/// The planner knows nothing about how the moves were produced: full stops
/// are derived from the geometry itself — a seam where consecutive moves are
/// not tangent-continuous (or where a move has no spatial body) anchors the
/// velocity to rest, so a velocity discontinuity is impossible by
/// construction.
///
/// The window of moves is re-planned warm-started from the profile state
/// `(v, a)` already emitted at the last seam, so the next window continues
/// the same jerk-limited curve — no velocity or acceleration discontinuity
/// at the cut, and a seam crossed mid-brake stays re-plannable. A prefix is emitted up to the furthest-forward
/// clean (zero-curvature) seam that is inside the plan's finality barrier and
/// clear of the brake-to-rest setback — past that point a move's velocity
/// body is still shaped by the window's tentative terminal rest, so it must
/// wait. `Drain` (or the input closing) materializes the deferred
/// brake-to-rest: the whole window is planned to rest and emitted. The
/// planner itself never decides when to stop looking ahead — that call
/// belongs to whoever sends `Drain`.
pub(crate) struct Planner {
    moves: Vec<Move>,
    entry: BoundaryState,
    next_plan_len: usize,
    config: StreamConfig,
}

impl Planner {
    pub(crate) fn new(config: StreamConfig) -> Self {
        Self {
            moves: Vec::new(),
            entry: BoundaryState::REST,
            next_plan_len: REPLAN_BATCH_MOVES,
            config,
        }
    }

    pub(crate) fn run(mut self, input: Receiver<StreamInput>, output: Sender<PlannedItem>) {
        while let Ok(item) = input.recv() {
            let ok = match item {
                StreamInput::Move(m) => self.absorb(m, &output),
                StreamInput::Drain => self.drain_to_rest(&output),
                StreamInput::Control(ctrl) => self.forward_control(ctrl, &output),
            };
            if !ok {
                return;
            }
        }
        self.drain_to_rest(&output);
    }

    /// `Reset` discards the window (nothing ahead of it may dispatch — the
    /// sender gates the dispatcher); every other token requires the window to
    /// have been drained first, because it is meaningless (or hides a
    /// velocity discontinuity) while moves are still being looked ahead.
    fn forward_control(&mut self, ctrl: Control, output: &Sender<PlannedItem>) -> bool {
        match &ctrl {
            Control::Reset { .. } => {
                self.moves.clear();
                self.entry = BoundaryState::REST;
                self.next_plan_len = REPLAN_BATCH_MOVES;
            }
            Control::Dwell { .. } | Control::SetAxisChains(_) | Control::Barrier(_) => {
                assert!(
                    self.moves.is_empty(),
                    "planner: control token arrived with {} undrained moves — a Drain must \
                     precede it",
                    self.moves.len()
                );
            }
        }
        output.send(PlannedItem::Control(ctrl)).is_ok()
    }

    fn absorb(&mut self, m: Move, output: &Sender<PlannedItem>) -> bool {
        self.moves.push(m);
        if self.moves.len() < self.next_plan_len.min(self.config.max_buffer_moves) {
            return true;
        }
        if !self.emit_committable(output) {
            return false;
        }
        if self.moves.len() >= self.config.max_buffer_moves {
            // Backstop only: a full window with no clean seam within the
            // finality barrier (e.g. one move longer than the whole
            // look-ahead). Drain to rest so memory stays bounded.
            tracing::info!(
                subsystem = "motion",
                event = "buffer_cap_drain",
                buffered = self.moves.len(),
                "[buffer-cap-drain] no committable seam — draining to rest"
            );
            return self.drain_to_rest(output);
        }
        true
    }

    fn plan(&self) -> VelocityProfile {
        let stop_before: Vec<bool> = (0..self.moves.len())
            .map(|i| i > 0 && self.stop_at_seam(i))
            .collect();
        let clock = std::time::Instant::now();
        let profile = plan_velocity_stops(
            &self.moves,
            &stop_before,
            self.config.integration_tol,
            self.config.max_extrude_only_velocity_mm_s,
            self.config.max_extrude_only_accel_mm_s2,
            self.entry,
        )
        .unwrap_or_else(|e| panic!("planner: velocity plan failed: {e:?}"));
        tracing::info!(
            subsystem = "motion",
            event = "pipe_plan",
            line_lo = self.moves.first().map_or(0, |m| m.source.start_line),
            line_hi = self.moves.last().map_or(0, |m| m.source.start_line),
            n = self.moves.len(),
            barrier = profile.barrier,
            v_barrier = profile.v_barrier,
            entry_v = self.entry.v,
            entry_a = self.entry.a,
            plan_us = clock.elapsed().as_micros(),
            t_us = crate::timing::mono_us(),
            "[pipe] plan"
        );
        profile
    }

    /// Materialize the brake-to-rest: plan the whole window to terminal rest
    /// and emit everything.
    fn drain_to_rest(&mut self, output: &Sender<PlannedItem>) -> bool {
        if self.moves.is_empty() {
            return true;
        }
        let profile = self.plan();
        let n = self.moves.len();
        self.emit(n, &profile, output)
    }

    /// Emit the prefix up to the furthest-forward clean seam that is inside
    /// the finality barrier and clear of the brake-to-rest setback.
    fn emit_committable(&mut self, output: &Sender<PlannedItem>) -> bool {
        let profile = self.plan();
        let setback = brake_to_rest_setback(&self.moves);
        let total_arc: f64 = self.moves.iter().map(|m| m.segment.s_len()).sum();
        let mut arc_to_seam = 0.0_f64;
        let mut chosen = 0usize;
        for i in 1..=profile.barrier {
            arc_to_seam += self.moves[i - 1].segment.s_len();
            if total_arc - arc_to_seam < setback {
                break;
            }
            if self.is_clean_seam(i) {
                chosen = i;
            }
        }
        self.next_plan_len = self.moves.len() - chosen + REPLAN_BATCH_MOVES;
        if chosen == 0 {
            return true;
        }
        self.emit(chosen, &profile, output)
    }

    fn emit(
        &mut self,
        count: usize,
        profile: &VelocityProfile,
        output: &Sender<PlannedItem>,
    ) -> bool {
        debug_assert_eq!(profile.moves.len(), self.moves.len());
        self.entry = profile.boundaries[count];
        for (geometry, velocity) in self.moves.drain(..count).zip(profile.moves.iter().cloned()) {
            if output
                .send(PlannedItem::Move(PlannedMove { geometry, velocity }))
                .is_err()
            {
                return false;
            }
        }
        self.next_plan_len = self.moves.len() + REPLAN_BATCH_MOVES;
        true
    }

    /// A non-forced emission may cut wherever the path resumes a straight line
    /// body (zero curvature) or comes to a full stop — never inside a curved
    /// piece, where the warm-start's straight-line jerk anchor would misstate
    /// the vector jerk state the curvature adds.
    fn is_clean_seam(&self, i: usize) -> bool {
        matches!(self.moves[i].segment.spatial, Some(Segment::Line(_))) || self.stop_at_seam(i)
    }

    /// The toolhead must be at rest across the seam entering move `i` unless
    /// the path is tangent-continuous there: both sides have spatial bodies
    /// and the exit heading of one is the entry heading of the other (within
    /// the same collinearity tolerance the fitter blends corners against).
    /// Anchoring every non-tangent seam to rest makes a velocity
    /// discontinuity impossible regardless of what produced the moves.
    fn stop_at_seam(&self, i: usize) -> bool {
        let prev = &self.moves[i - 1];
        let next = &self.moves[i];
        let (Some(a), Some(b)) = (&prev.segment.spatial, &next.segment.spatial) else {
            return true;
        };
        let t_in = a.heading_at(a.s_len());
        let t_out = b.heading_at(0.0);
        let cos_theta = t_in[0] * t_out[0] + t_in[1] * t_out[1] + t_in[2] * t_out[2];
        cos_theta.clamp(-1.0, 1.0).acos() > self.config.chain.corner.theta_min_rad
    }
}

/// Arc length the emission boundary is held back from the window's tentative
/// terminal. The lowering reconstructs each move's velocity body against its
/// run terminal, so a move within one braking distance of the window's
/// fictional rest has its body shaped by that fiction — it is not yet
/// terminal-independent and an appended move would change it. Holding the
/// boundary this far back makes every emitted body a function of geometry
/// alone, so the emitted trajectory is final under append and
/// output-equivalent to a full re-plan — positions exactly, seam timing within
/// the iterative velocity stage's tolerance. A safe over-estimate of the
/// jerk-limited stopping distance from the window's peak feedrate
/// (`v · t_brake` over-bounds the true `∫v dt`), so the held-back open tail
/// stays bounded.
fn brake_to_rest_setback(moves: &[Move]) -> f64 {
    let v_peak = moves
        .iter()
        .map(|m| m.feedrate_mm_s.min(m.limits.max_velocity_mm_s))
        .fold(0.0_f64, f64::max);
    let a_min = moves
        .iter()
        .map(|m| m.limits.accel_mm_s2)
        .fold(f64::INFINITY, f64::min);
    let j_min = moves
        .iter()
        .map(|m| m.limits.max_jerk_mm_s3)
        .fold(f64::INFINITY, f64::min);
    v_peak * jerk_limited_brake_time(v_peak, a_min, j_min)
}
