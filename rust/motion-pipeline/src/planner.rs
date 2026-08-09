use crossbeam_channel::{Receiver, Sender, TryRecvError};
use geometry::path::lowering::PositionProfile;
use geometry::path::{CurvatureProfile, Segment};
use geometry::{BoundaryState, Move, VelocityProfile, plan_velocity_stops};

use crate::types::{
    Control, PlannedItem, PlannedMove, StreamConfig, StreamInput, jerk_limited_brake_time,
};

/// Cost governor, not a lookahead bound: velocity planning is ~linear in the
/// window, so re-planning on every arriving move would be quadratic. The window
/// keeps absorbing input and a re-plan fires once this many moves have arrived
/// since the last one (an input-empty drain still plans immediately).
const REPLAN_BATCH_MOVES: usize = 64;

/// When the input runs momentarily dry the planner stops batching and commits
/// what it can, so a rate-matched feed keeps the committed runway topped up
/// instead of letting it sag for up to a whole batch. Requiring a batch of
/// arrivals since the last plan bounds the re-plan rate when the feed
/// trickles in one move per wakeup: each re-plan costs O(window), so a
/// trickle-fed planner spends `window / QUIET_PLAN_MIN_MOVES` plan passes per
/// move — at high move rates this quotient, not the per-plan cost, is what
/// decides whether planning keeps up with the print.
const QUIET_PLAN_MIN_MOVES: usize = 32;

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
pub struct Planner {
    moves: Vec<Move>,
    entry: BoundaryState,
    moves_since_plan: usize,
    config: StreamConfig,
}

impl Planner {
    pub fn new(config: StreamConfig) -> Self {
        Self {
            moves: Vec::new(),
            entry: BoundaryState::REST,
            moves_since_plan: 0,
            config,
        }
    }

    /// Reads only what planning needs: drains whatever the input already
    /// holds, and the moment it would block with unplanned arrivals in the
    /// window, plans and emits the committable prefix before waiting. Anything
    /// beyond the window's own lookahead needs stays queued upstream, so a
    /// full pipeline backpressures to the entry instead of pooling here.
    pub fn run(mut self, input: Receiver<StreamInput>, output: Sender<PlannedItem>) {
        loop {
            let item = match input.try_recv() {
                Ok(item) => item,
                Err(TryRecvError::Disconnected) => break,
                Err(TryRecvError::Empty) => {
                    if !self.plan_on_quiet_input(&output) {
                        return;
                    }
                    match input.recv() {
                        Ok(item) => item,
                        Err(_) => break,
                    }
                }
            };
            if !self.feed(item, &output) {
                return;
            }
        }
        self.finish(&output);
    }

    /// One iteration of [`Planner::run`]'s loop, for single-threaded hosts
    /// that drive the stage item by item.
    pub fn feed(&mut self, item: StreamInput, output: &Sender<PlannedItem>) -> bool {
        match item {
            StreamInput::Move(m) => self.absorb(m, output),
            StreamInput::Drain => {
                self.drain_to_rest(output) && output.send(PlannedItem::Drain).is_ok()
            }
            StreamInput::Control(ctrl) => self.forward_control(ctrl, output),
        }
    }

    /// The input-closed path: brake the window to rest, emit it, and forward
    /// the `Drain`.
    pub fn finish(&mut self, output: &Sender<PlannedItem>) -> bool {
        self.drain_to_rest(output) && output.send(PlannedItem::Drain).is_ok()
    }

    fn plan_on_quiet_input(&mut self, output: &Sender<PlannedItem>) -> bool {
        if self.moves_since_plan < QUIET_PLAN_MIN_MOVES {
            return true;
        }
        self.moves_since_plan = 0;
        self.emit_committable(output)
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
                self.moves_since_plan = 0;
            }
            Control::Dwell { .. }
            | Control::SetAxisChains(_)
            | Control::SetMesh { .. }
            | Control::Nudge { .. }
            | Control::Barrier(_) => {
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
        self.moves_since_plan += 1;
        if self.moves_since_plan < REPLAN_BATCH_MOVES
            && self.moves.len() < self.config.max_buffer_moves
        {
            return true;
        }
        self.moves_since_plan = 0;
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
            return self.drain_to_rest(output) && output.send(PlannedItem::Drain).is_ok();
        }
        true
    }

    fn plan(&self) -> VelocityProfile {
        let stop_before: Vec<bool> = (0..self.moves.len())
            .map(|i| i > 0 && self.stop_at_seam(i))
            .collect();
        let clock = crate::timing::stopwatch();
        let profile = plan_velocity_stops(
            &self.moves,
            &stop_before,
            self.config.integration_tol,
            self.config.max_extrude_only_velocity_mm_s,
            self.config.max_extrude_only_accel_mm_s2,
            self.entry,
        )
        .unwrap_or_else(|e| panic!("planner: velocity plan failed: {e:?}"));
        tracing::debug!(
            subsystem = "motion",
            event = "pipe_plan",
            line_lo = self.moves.first().map_or(0, |m| m.source.start_line),
            line_hi = self.moves.last().map_or(0, |m| m.source.start_line),
            n = self.moves.len(),
            barrier = profile.barrier,
            v_barrier = profile.v_barrier,
            entry_v = self.entry.v,
            entry_a = self.entry.a,
            plan_us = clock.elapsed_us(),
            t_us = crate::timing::mono_us(),
            "[pipe] plan"
        );
        profile
    }

    /// Materialize the brake-to-rest: plan the whole window to terminal rest
    /// and emit everything.
    fn drain_to_rest(&mut self, output: &Sender<PlannedItem>) -> bool {
        self.moves_since_plan = 0;
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
        let horizon = self.terminal_independent_seam();
        let mut chosen = 0usize;
        for i in 1..=profile.barrier.min(horizon) {
            if self.is_clean_seam(i) {
                chosen = i;
            }
        }
        if chosen == 0 {
            return true;
        }
        self.emit(chosen, &profile, output)
    }

    /// Furthest-forward seam whose emitted bodies are terminal-independent.
    ///
    /// The lowering reconstructs each move's velocity body against its run
    /// terminal, so a move within one braking distance of the window's
    /// fictional rest has its body shaped by that fiction and an appended
    /// move would change it. That braking rides the *open tail* — the moves
    /// beyond the seam — so the tail's own peak feedrate and tightest
    /// accel/jerk budget are what set its length. Reading them off the whole
    /// window instead makes one already-passed travel move dictate the
    /// setback for every seam behind it, which on a print that interleaves
    /// 600 mm/s travels with 300 mm/s extrusions inflates the held-back tail
    /// ~3x and, with it, the re-plan window every arriving move is charged
    /// for. `v · t_brake` still over-bounds the true `∫v dt`, so the held-back
    /// tail keeps its safety factor; it is now just measured where it applies.
    fn terminal_independent_seam(&self) -> usize {
        let mut arc = 0.0_f64;
        let mut v_peak = 0.0_f64;
        let mut accel = f64::INFINITY;
        let mut jerk = f64::INFINITY;
        for i in (1..self.moves.len()).rev() {
            let m = &self.moves[i];
            arc += m.segment.s_len();
            v_peak = v_peak.max(m.feedrate_mm_s.min(m.limits.max_velocity_mm_s));
            accel = accel.min(m.limits.accel_mm_s2);
            jerk = jerk.min(m.limits.max_jerk_mm_s3);
            if arc >= v_peak * jerk_limited_brake_time(v_peak, accel, jerk) {
                return i;
            }
        }
        0
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
    /// the same collinearity tolerance the fit stage blends corners against).
    /// Anchoring every non-tangent seam to rest makes a velocity
    /// discontinuity impossible regardless of what produced the moves. The
    /// same principle covers the followers: a seam where an extruding axis's
    /// `de/ds` would step by more than the fit stage's ramp tolerance anchors
    /// to rest too — the fit stage ramps modest steps through its blends (its
    /// ramps anchor at the neighbors' own ratios, so blended seams pass here by
    /// construction), but an abrupt flow change on a collinear continuation
    /// has no corner to ramp through and would otherwise step `ė = r·v` at
    /// full speed.
    fn stop_at_seam(&self, i: usize) -> bool {
        let prev = &self.moves[i - 1];
        let next = &self.moves[i];
        let (Some(a), Some(b)) = (&prev.segment.spatial, &next.segment.spatial) else {
            return true;
        };
        let t_in = a.heading_at(a.s_len());
        let t_out = b.heading_at(0.0);
        let cos_theta = t_in[0] * t_out[0] + t_in[1] * t_out[1] + t_in[2] * t_out[2];
        if libm::acos(cos_theta.clamp(-1.0, 1.0)) > self.config.corner.theta_min_rad {
            return true;
        }
        follower_rate_step(prev, next, self.config.corner.extrusion_ramp_rel_tol)
    }
}

/// Whether any axis extruding on both sides of the seam demands a `de/ds`
/// step beyond `rel_tol`. Mirrors the fit stage's `ExtrusionStep` gate; axes
/// absent from a side carry ratio zero and are exempt (ramping to or from
/// zero is always allowed).
fn follower_rate_step(prev: &Move, next: &Move, rel_tol: f64) -> bool {
    let len_in = prev.segment.s_len();
    prev.segment.followers.iter().any(|f| {
        let r_in = f.ratio_at(len_in, len_in);
        let r_out = next
            .segment
            .followers
            .iter()
            .find(|g| g.axis_index == f.axis_index)
            .map_or(0.0, |g| g.ratio_at(0.0, next.segment.s_len()));
        r_in != 0.0 && r_out != 0.0 && (r_out - r_in).abs() > rel_tol * r_in.abs().max(r_out.abs())
    })
}

#[cfg(test)]
#[path = "planner_tests.rs"]
mod planner_tests;
