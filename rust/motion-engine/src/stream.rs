use std::collections::{HashSet, VecDeque};

use geometry::path::lowering::PositionProfile;
use geometry::{
    ChainFitConfig, FitError, Move, VelocityConfig, VelocityError, VelocityLimits, fit_chain,
    plan_velocity_warm_start,
};
use trajectory::ShapedSegment;

use crate::lowering::{LoweringError, lower_move};

#[derive(Debug, Clone, Copy)]
pub struct StreamConfig {
    pub chain: ChainFitConfig,
    pub velocity: VelocityConfig,
    pub fit_tol_mm: f64,
    /// Look-ahead margin held back from each non-forced commit. The trailing
    /// `keep_secs` of planned trajectory stays uncommitted so committed
    /// junction velocities were planned with enough downstream context and are
    /// not pulled down by the buffer's pessimistic terminal-`v=0`.
    pub keep_secs: f64,
    /// Path limits for planner-internal moves (homing). Stream moves submitted
    /// through the bridge carry their own per-move limits; this is the fallback
    /// the planner uses when it constructs a move itself.
    pub limits: VelocityLimits,
}

#[derive(Debug)]
pub enum StreamError {
    Fit(FitError),
    Velocity(VelocityError),
    Lowering(LoweringError),
}

impl std::fmt::Display for StreamError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Fit(e) => write!(f, "chain fit: {e:?}"),
            Self::Velocity(e) => write!(f, "velocity plan: {e:?}"),
            Self::Lowering(e) => write!(f, "lowering: {e}"),
        }
    }
}

impl std::error::Error for StreamError {}

impl From<FitError> for StreamError {
    fn from(e: FitError) -> Self {
        Self::Fit(e)
    }
}
impl From<VelocityError> for StreamError {
    fn from(e: VelocityError) -> Self {
        Self::Velocity(e)
    }
}
impl From<LoweringError> for StreamError {
    fn from(e: LoweringError) -> Self {
        Self::Lowering(e)
    }
}

/// Streaming look-ahead planner over the new geometry pipeline. Buffers
/// submitted moves, re-plans the uncommitted window warm-started from the
/// dispatched velocity, and commits prefixes at clean (non-blended) seams so
/// the trajectory stays velocity-continuous across consecutive moves without
/// re-emitting committed pieces. Commit timing (real-time cadence) is the
/// caller's; this type owns the geometry/velocity state.
pub struct StreamState {
    buffer: VecDeque<Move>,
    /// Absolute velocity at the committed seam (entry of the buffer's first move).
    entry_v: f64,
    /// Absolute registry position at the committed seam (index = registry axis).
    odometer: Vec<f64>,
    /// Absolute planner time of the committed seam.
    t_committed: f64,
    config: StreamConfig,
}

impl StreamState {
    #[must_use]
    pub fn new(config: StreamConfig, home_pos: &[f64], t_start: f64) -> Self {
        Self {
            buffer: VecDeque::new(),
            entry_v: 0.0,
            odometer: home_pos.to_vec(),
            t_committed: t_start,
            config,
        }
    }

    pub fn reset(&mut self, home_pos: &[f64], t_start: f64) {
        self.buffer.clear();
        self.entry_v = 0.0;
        self.odometer = home_pos.to_vec();
        self.t_committed = t_start;
    }

    pub fn push(&mut self, m: Move) {
        self.buffer.push_back(m);
    }

    /// Advance the committed time cursor by an idle gap (a dwell) without
    /// emitting motion. Only valid when fully committed (buffer empty); the
    /// seam velocity must already be at rest.
    pub fn advance_time(&mut self, dt: f64) {
        debug_assert!(
            self.buffer.is_empty(),
            "advance_time requires a drained buffer"
        );
        debug_assert_eq!(self.entry_v, 0.0, "advance_time requires rest at the seam");
        if dt > 0.0 {
            self.t_committed += dt;
        }
    }

    /// Re-anchor the committed time cursor to `target_t` (the live MCU playhead
    /// plus lead) after the stream has gone idle and the machine has caught up.
    /// Without this, a move submitted after an idle gap is planned at a stale
    /// planner time and lands in the MCU's past. Only valid at rest with a
    /// drained buffer; never moves the cursor backward.
    pub fn advance_idle(&mut self, target_t: f64) {
        debug_assert!(
            self.buffer.is_empty(),
            "advance_idle requires a drained buffer"
        );
        debug_assert_eq!(self.entry_v, 0.0, "advance_idle requires rest at the seam");
        if target_t > self.t_committed {
            self.t_committed = target_t;
        }
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.buffer.is_empty()
    }

    #[must_use]
    pub fn buffered(&self) -> usize {
        self.buffer.len()
    }

    #[must_use]
    pub fn t_committed(&self) -> f64 {
        self.t_committed
    }

    #[must_use]
    pub fn entry_velocity(&self) -> f64 {
        self.entry_v
    }

    #[must_use]
    pub fn limits(&self) -> VelocityLimits {
        self.config.limits
    }

    /// Plan the buffer and commit a prefix at the latest clean seam. `force`
    /// (flush / dwell / idle drain / shutdown) commits the entire buffer to
    /// rest. Returns the `ShapedSegment`s to dispatch, in order; empty when
    /// nothing is committable yet.
    pub fn commit(&mut self, force: bool) -> Result<Vec<ShapedSegment>, StreamError> {
        if self.buffer.is_empty() {
            return Ok(Vec::new());
        }

        let moves: Vec<Move> = self.buffer.iter().cloned().collect();
        let outcome = fit_chain(&moves, self.config.chain)?;
        let profile = plan_velocity_warm_start(&outcome, self.config.velocity, self.entry_v)?;

        let n = outcome.moves.len();
        let mut pos = self.odometer.clone();
        let mut t = self.t_committed;
        let mut segs: Vec<ShapedSegment> = Vec::with_capacity(n);
        let mut start_times: Vec<f64> = Vec::with_capacity(n);
        for (gm, vm) in outcome.moves.iter().zip(&profile.moves) {
            start_times.push(t);
            let seg = lower_move(gm, vm, t, &pos, self.config.fit_tol_mm)?;
            t = seg.t_end;
            advance_odometer(&mut pos, gm);
            segs.push(seg);
        }
        let total_t = t - self.t_committed;

        let commit_count = if force {
            n
        } else {
            let unblended: HashSet<u32> =
                outcome.report.unblended.iter().map(|u| u.line_no).collect();
            let limit_t = self.t_committed + (total_t - self.config.keep_secs);
            let mut chosen = 0usize;
            for i in 1..n {
                if unblended.contains(&outcome.moves[i].source.start_line) {
                    if start_times[i] <= limit_t {
                        chosen = i;
                    } else {
                        break;
                    }
                }
            }
            chosen
        };

        if commit_count == 0 {
            return Ok(Vec::new());
        }

        let committed: Vec<ShapedSegment> = segs.into_iter().take(commit_count).collect();

        let mut seam_pos = self.odometer.clone();
        for gm in outcome.moves.iter().take(commit_count) {
            advance_odometer(&mut seam_pos, gm);
        }
        self.odometer = seam_pos;
        self.t_committed = committed.last().expect("commit_count > 0").t_end;
        self.entry_v = if commit_count == n {
            0.0
        } else {
            profile.moves[commit_count - 1].exit_v
        };

        if commit_count == n {
            self.buffer.clear();
        } else {
            let keep_line = outcome.moves[commit_count].source.start_line;
            while self
                .buffer
                .front()
                .is_some_and(|m| m.source.start_line < keep_line)
            {
                self.buffer.pop_front();
            }
        }

        Ok(committed)
    }
}

fn advance_odometer(pos: &mut [f64], gm: &Move) {
    let s_len = gm.segment.s_len();
    if let Some(seg) = &gm.segment.spatial {
        let end = seg.point_at(s_len);
        for axis in 0..3.min(pos.len()) {
            pos[axis] = end[axis];
        }
    }
    for f in &gm.segment.followers {
        if let Some(slot) = pos.get_mut(f.axis_index) {
            *slot += f.ratio * s_len;
        }
    }
}

#[cfg(test)]
mod tests;
