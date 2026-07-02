use crossbeam_channel::{Receiver, Sender, TryRecvError};
use geometry::fitter::UnblendReason;
use geometry::path::Segment;
use geometry::{Move, VelocityProfile, plan_velocity_stops};

use super::{FittedMove, PlannedMove, StreamConfig, jerk_limited_brake_time};

/// Cost governor, not a lookahead bound: velocity planning is ~linear in the
/// window, so re-planning on every arriving move would be quadratic. The window
/// keeps absorbing input and a re-plan fires once this many moves have arrived
/// since the last one (an input-empty drain still plans immediately).
const REPLAN_BATCH_MOVES: usize = 64;

/// Second pipeline stage: plans jerk-limited S-curve velocity over the fitted
/// geometry and emits `PlannedMove`s whose velocity bodies are final under any
/// future append.
///
/// The window of fitted moves is re-planned warm-started from the velocity
/// already emitted at the last seam. A prefix is emitted up to the
/// furthest-forward clean (zero-curvature) seam that is inside the plan's
/// finality barrier and clear of the brake-to-rest setback — past that point a
/// move's velocity body is still shaped by the window's tentative terminal
/// rest, so it must wait. When the input runs empty the deferred brake-to-rest
/// is materialized: the whole window is planned to rest and emitted, so moves
/// are never held hoping for input that may not come.
pub(crate) struct Planner {
    moves: Vec<Move>,
    unblended_before: Vec<Option<UnblendReason>>,
    entry_v: f64,
    next_plan_len: usize,
    config: StreamConfig,
}

impl Planner {
    pub(crate) fn new(config: StreamConfig) -> Self {
        Self {
            moves: Vec::new(),
            unblended_before: Vec::new(),
            entry_v: 0.0,
            next_plan_len: REPLAN_BATCH_MOVES,
            config,
        }
    }

    pub(crate) fn run(mut self, input: Receiver<FittedMove>, output: Sender<PlannedMove>) {
        loop {
            let received = if self.moves.is_empty() {
                input.recv().ok()
            } else {
                match input.try_recv() {
                    Ok(m) => Some(m),
                    Err(TryRecvError::Empty) => {
                        if !self.drain_to_rest(&output) {
                            return;
                        }
                        continue;
                    }
                    Err(TryRecvError::Disconnected) => None,
                }
            };
            let Some(m) = received else {
                self.drain_to_rest(&output);
                return;
            };
            self.moves.push(m.piece);
            self.unblended_before.push(m.unblended_before);
            if self.moves.len() >= self.next_plan_len.min(self.config.max_buffer_moves) {
                if !self.emit_committable(&output) {
                    return;
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
                    if !self.drain_to_rest(&output) {
                        return;
                    }
                }
            }
        }
    }

    fn plan(&self) -> VelocityProfile {
        let stop_before: Vec<bool> = self
            .unblended_before
            .iter()
            .map(|u| u.is_some_and(|r| r != UnblendReason::Collinear))
            .collect();
        let clock = std::time::Instant::now();
        let profile = plan_velocity_stops(
            &self.moves,
            &stop_before,
            self.config.integration_tol,
            self.config.max_extrude_only_velocity_mm_s,
            self.config.max_extrude_only_accel_mm_s2,
            self.entry_v,
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
            entry_v = self.entry_v,
            plan_us = clock.elapsed().as_micros(),
            t_us = crate::timing::mono_us(),
            "[pipe] plan"
        );
        profile
    }

    /// Materialize the brake-to-rest: plan the whole window to terminal rest
    /// and emit everything.
    fn drain_to_rest(&mut self, output: &Sender<PlannedMove>) -> bool {
        if self.moves.is_empty() {
            return true;
        }
        let profile = self.plan();
        let n = self.moves.len();
        self.emit(n, &profile, output)
    }

    /// Emit the prefix up to the furthest-forward clean seam that is inside
    /// the finality barrier and clear of the brake-to-rest setback.
    fn emit_committable(&mut self, output: &Sender<PlannedMove>) -> bool {
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
        output: &Sender<PlannedMove>,
    ) -> bool {
        debug_assert_eq!(profile.moves.len(), self.moves.len());
        self.entry_v = if count == self.moves.len() {
            0.0
        } else {
            profile.moves[count - 1].exit_v
        };
        self.unblended_before.drain(..count);
        for (geometry, velocity) in self.moves.drain(..count).zip(profile.moves.iter().cloned()) {
            if output.send(PlannedMove { geometry, velocity }).is_err() {
                return false;
            }
        }
        self.next_plan_len = self.moves.len() + REPLAN_BATCH_MOVES;
        true
    }

    /// A non-forced emission may cut wherever the fit output resumes a straight
    /// line body (zero curvature: an unblended seam or the exit of a blend) —
    /// never inside a blend, where curvature is nonzero and the velocity
    /// warm-start, which carries only a scalar entry speed, would be invalid.
    fn is_clean_seam(&self, i: usize) -> bool {
        self.unblended_before[i].is_some()
            || matches!(self.moves[i].segment.spatial, Some(Segment::Line(_)))
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
