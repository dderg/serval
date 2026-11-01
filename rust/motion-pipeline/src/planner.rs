use geometry::path::Segment;
use geometry::{
    Move, VelocityPlanParams, VelocityProfile, plan_velocity_stops_select_prefix,
    seam_requires_stop,
};

use crate::types::{Control, PlannedItem, PlannedMove, StreamConfig, StreamInput};

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

pub struct Planner {
    moves: Vec<Move>,
    entry_v: f64,
    moves_since_plan: usize,
    config: StreamConfig,
}

impl Planner {
    pub fn new(config: StreamConfig) -> Self {
        Self {
            moves: Vec::new(),
            entry_v: 0.0,
            moves_since_plan: 0,
            config,
        }
    }

    pub fn feed(
        &mut self,
        item: StreamInput,
        output: &mut impl FnMut(PlannedItem) -> bool,
    ) -> bool {
        match item {
            StreamInput::Move(m) => self.absorb(m, output),
            StreamInput::Drain => self.drain_to_rest(output) && output(PlannedItem::Drain),
            StreamInput::Control(ctrl) => self.forward_control(ctrl, output),
        }
    }

    /// The input-closed path: brake the window to rest, emit it, and forward
    /// the `Drain`.
    pub fn finish(&mut self, output: &mut impl FnMut(PlannedItem) -> bool) -> bool {
        self.drain_to_rest(output) && output(PlannedItem::Drain)
    }

    pub fn idle(&mut self, output: &mut impl FnMut(PlannedItem) -> bool) -> bool {
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
    fn forward_control(
        &mut self,
        ctrl: Control,
        output: &mut impl FnMut(PlannedItem) -> bool,
    ) -> bool {
        match &ctrl {
            Control::Reset { .. } => {
                self.moves.clear();
                self.entry_v = 0.0;
                self.moves_since_plan = 0;
            }
            Control::Dwell { .. }
            | Control::SetAxisChains(_)
            | Control::SetMesh { .. }
            | Control::Dispatch(_) => {
                assert!(
                    self.moves.is_empty(),
                    "planner: control token arrived with {} undrained moves — a Drain must \
                     precede it",
                    self.moves.len()
                );
            }
        }
        output(PlannedItem::Control(ctrl))
    }

    fn absorb(&mut self, m: Move, output: &mut impl FnMut(PlannedItem) -> bool) -> bool {
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
            return self.drain_to_rest(output) && output(PlannedItem::Drain);
        }
        true
    }

    fn plan(&self, reconstruct_count: usize) -> VelocityProfile {
        self.plan_selected(|_| reconstruct_count)
    }

    fn plan_selected<F>(&self, select_prefix: F) -> VelocityProfile
    where
        F: FnOnce(usize) -> usize,
    {
        let stop_before: Vec<bool> = (0..self.moves.len())
            .map(|i| i > 0 && self.stop_at_seam(i))
            .collect();
        let clock = crate::timing::stopwatch();
        let profile = plan_velocity_stops_select_prefix(
            &self.moves,
            &stop_before,
            VelocityPlanParams {
                integration_tol: self.config.integration_tol,
                max_extrude_only_velocity_mm_s: self.config.max_extrude_only_velocity_mm_s,
                max_extrude_only_accel_mm_s2: self.config.max_extrude_only_accel_mm_s2,
                entry_v: self.entry_v,
            },
            select_prefix,
        )
        .unwrap_or_else(|e| panic!("planner: velocity plan failed: {e:?}"));
        tracing::debug!(
            subsystem = "motion",
            event = "pipe_plan",
            line_lo = self.moves.first().map_or(0, |m| m.source.start_line),
            line_hi = self.moves.last().map_or(0, |m| m.source.start_line),
            n = self.moves.len(),
            reconstructed = profile.moves.len(),
            barrier = profile.barrier,
            v_barrier = profile.v_barrier,
            entry_v = self.entry_v,
            plan_us = clock.elapsed_us(),
            t_us = crate::timing::mono_us(),
            "[pipe] plan"
        );
        profile
    }

    /// Materialize the brake-to-rest: plan the whole window to terminal rest
    /// and emit everything.
    fn drain_to_rest(&mut self, output: &mut impl FnMut(PlannedItem) -> bool) -> bool {
        self.moves_since_plan = 0;
        if self.moves.is_empty() {
            return true;
        }
        let profile = self.plan(self.moves.len());
        let n = self.moves.len();
        self.emit(n, &profile, output)
    }

    /// Emit the prefix up to the furthest-forward clean seam inside the
    /// finality barrier. Seams at or before the barrier sit on their forward
    /// ceiling, which appended moves can only leave unchanged, and every
    /// committed body is reconstructed from its own two seam speeds, so the
    /// committed prefix is the one a single window would plan.
    fn emit_committable(&mut self, output: &mut impl FnMut(PlannedItem) -> bool) -> bool {
        let profile = self.plan_selected(|barrier| {
            (1..=barrier)
                .rev()
                .find(|&i| self.is_clean_seam(i))
                .unwrap_or(0)
        });
        let chosen = profile.moves.len();
        if chosen == 0 {
            return true;
        }
        self.emit(chosen, &profile, output)
    }

    fn emit(
        &mut self,
        count: usize,
        profile: &VelocityProfile,
        output: &mut impl FnMut(PlannedItem) -> bool,
    ) -> bool {
        debug_assert_eq!(profile.moves.len(), count);
        self.entry_v = profile.boundary_speeds[count];
        for (geometry, velocity) in self.moves.drain(..count).zip(profile.moves.iter().cloned()) {
            if !output(PlannedItem::Move(PlannedMove { geometry, velocity })) {
                return false;
            }
        }
        true
    }

    fn is_clean_seam(&self, i: usize) -> bool {
        matches!(self.moves[i].segment.spatial, Some(Segment::Line(_))) || self.stop_at_seam(i)
    }

    fn stop_at_seam(&self, i: usize) -> bool {
        seam_requires_stop(&self.moves[i - 1], &self.moves[i], self.config.corner)
    }
}

#[cfg(test)]
#[path = "planner_tests.rs"]
mod planner_tests;
