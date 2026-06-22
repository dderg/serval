use std::collections::{HashSet, VecDeque};

use geometry::path::lowering::PositionProfile;
use geometry::path::{Line, Segment};
use geometry::{
    ChainFitConfig, FitError, GeometryError, Move, VelocityConfig, VelocityError, VelocityLimits,
    fit_chain, plan_velocity_warm_start,
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
    /// Backstop cap. The continuity commit drains at every blend (each
    /// biclothoid rejoins the outgoing line at zero curvature, a clean seam), so
    /// a normal continuous path commits without ever stopping. This force-drain
    /// to rest only fires for a pathological buffer that has no clean seam within
    /// reach at all (e.g. a single move longer than the whole look-ahead window)
    /// — without it such a buffer would grow unbounded while nothing reaches the
    /// MCU. It is a safety net, not the steady-state path.
    pub max_buffer_moves: usize,
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
    Geometry(GeometryError),
}

impl std::fmt::Display for StreamError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Fit(e) => write!(f, "chain fit: {e:?}"),
            Self::Velocity(e) => write!(f, "velocity plan: {e:?}"),
            Self::Lowering(e) => write!(f, "lowering: {e}"),
            Self::Geometry(e) => write!(f, "head-trim geometry: {e:?}"),
        }
    }
}

impl From<GeometryError> for StreamError {
    fn from(e: GeometryError) -> Self {
        Self::Geometry(e)
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
/// dispatched velocity, and commits prefixes at zero-curvature seams so the
/// trajectory stays velocity-continuous across consecutive moves without
/// re-emitting committed pieces. A seam is committable wherever the fit output
/// resumes a straight line body: that holds at unblended junctions and equally
/// at the exit of every blend, since a biclothoid rejoins the outgoing line at
/// zero curvature. Committing through a blend consumes the next move's head, so
/// that move is replaced in the buffer by its head-trimmed remainder before the
/// next re-fit. Commit timing (real-time cadence) is the caller's; this type
/// owns the geometry/velocity state.
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

    /// Restart the committed timeline at the origin after the machine has gone
    /// idle and the playhead has caught up. The next dispatched segment then
    /// re-anchors against the live playhead at dispatch time — like a freshly
    /// opened stream — instead of inheriting the prior run's stale anchor, which
    /// would land the resumed move in the MCU's past. Position is preserved;
    /// only the time cursor resets. The caller must also drop its wall-clock
    /// sync so the playhead and committed clocks realign at the origin. Valid
    /// only at rest with a drained buffer.
    pub fn restart_idle_timeline(&mut self) {
        debug_assert!(
            self.buffer.is_empty(),
            "restart_idle_timeline requires a drained buffer"
        );
        debug_assert_eq!(
            self.entry_v, 0.0,
            "restart_idle_timeline requires rest at the seam"
        );
        self.t_committed = 0.0;
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

    #[must_use]
    pub fn max_buffer_moves(&self) -> usize {
        self.config.max_buffer_moves
    }

    /// Plan the buffer and commit a prefix at the latest zero-curvature seam.
    /// `force` (flush / dwell / idle drain / shutdown) commits the entire buffer
    /// to rest. Returns the `ShapedSegment`s to dispatch, in order; empty when
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
        let mut seam_xyz: Vec<[f64; 3]> = Vec::with_capacity(n);
        for (gm, vm) in outcome.moves.iter().zip(&profile.moves) {
            start_times.push(t);
            seam_xyz.push([pos[0], pos[1], pos[2]]);
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
                if start_times[i] > limit_t {
                    break;
                }
                if is_clean_seam(&outcome.moves, i, &unblended)
                    && self.head_trim_feasible(&outcome.moves, i, seam_xyz[i])
                {
                    chosen = i;
                }
            }
            chosen
        };

        tracing::info!(
            subsystem = "motion",
            event = "commit_decision",
            force,
            n,
            unblended = outcome.report.unblended.len(),
            commit_count,
            total_t,
            keep_secs = self.config.keep_secs,
            entry_v = self.entry_v,
            t_committed = self.t_committed,
            "[commit-decision]"
        );

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
            let head_consumed = blend_consumed_head(&outcome.moves, commit_count);
            while self
                .buffer
                .front()
                .is_some_and(|m| m.source.start_line < keep_line)
            {
                self.buffer.pop_front();
            }
            if head_consumed {
                self.trim_front_to_seam()?;
            }
        }

        Ok(committed)
    }

    /// Whether committing through the blend at output index `i` would leave a
    /// head-trimmable remainder. Committing through a blend consumes the kept
    /// move's head, so that move must be a `Line` (line-line blend) with a
    /// non-degenerate portion left past the seam. A boundary that fails this is
    /// refused by selection so the planner never produces a zero-length or
    /// non-line trim (which would otherwise abort the stream). The warn fires
    /// only on the rare refusal and records the geometry that triggered it.
    fn head_trim_feasible(&self, moves: &[Move], i: usize, seam_xyz: [f64; 3]) -> bool {
        if !blend_consumed_head(moves, i) {
            return true;
        }
        let keep_line = moves[i].source.start_line;
        let Some(front) = self
            .buffer
            .iter()
            .find(|m| m.source.start_line == keep_line)
        else {
            tracing::warn!(
                subsystem = "motion",
                event = "head_trim_refused",
                keep_line,
                reason = "no buffer move for kept line",
                "[head-trim-refused]"
            );
            return false;
        };
        match &front.segment.spatial {
            Some(Segment::Line(line)) => {
                let remainder = dist3(seam_xyz, line.end);
                if remainder <= TRIM_EPS_MM {
                    tracing::warn!(
                        subsystem = "motion",
                        event = "head_trim_refused",
                        keep_line,
                        remainder,
                        reason = "degenerate remainder",
                        "[head-trim-refused]"
                    );
                    false
                } else {
                    true
                }
            }
            other => {
                tracing::warn!(
                    subsystem = "motion",
                    event = "head_trim_refused",
                    keep_line,
                    spatial = ?other.as_ref().map(std::mem::discriminant),
                    reason = "kept move is not a line",
                    "[head-trim-refused]"
                );
                false
            }
        }
    }

    /// Replace the front buffer move with the portion that survives the seam: a
    /// line from the committed position to the move's original end. The committed
    /// blend already paid out the head (spatial and, proportionally, followers),
    /// so the per-mm follower ratios carry over unchanged. Selection has already
    /// proven this is a `Line` with a non-degenerate remainder
    /// ([`Self::head_trim_feasible`]).
    fn trim_front_to_seam(&mut self) -> Result<(), StreamError> {
        let front = self
            .buffer
            .front()
            .expect("trim requires a kept front move");
        let Some(Segment::Line(line)) = &front.segment.spatial else {
            return Err(StreamError::Geometry(GeometryError::ZeroMotion));
        };
        let new_start = [self.odometer[0], self.odometer[1], self.odometer[2]];
        let trimmed = Line::try_new(new_start, line.end)?;
        let segment = geometry::path::PathSegment::try_new(
            Segment::Line(trimmed),
            front.segment.followers.clone(),
        )?;
        let replacement = Move {
            segment,
            feedrate_mm_s: front.feedrate_mm_s,
            limits: front.limits,
            source: front.source,
        };
        *self.buffer.front_mut().expect("front checked above") = replacement;
        Ok(())
    }
}

/// A non-forced commit may cut wherever the fit output resumes a straight line
/// body (zero curvature: an unblended seam or the exit of a blend) — never
/// inside a blend, where curvature is nonzero and the velocity warm-start, which
/// carries only a scalar entry speed, would be invalid.
fn is_clean_seam(moves: &[Move], i: usize, unblended: &HashSet<u32>) -> bool {
    unblended.contains(&moves[i].source.start_line)
        || matches!(moves[i].segment.spatial, Some(Segment::Line(_)))
}

/// Whether the seam before output index `i` is the exit of a blend, i.e. the
/// blend consumed the head of the move resuming at `i`. The preceding output is
/// then the blend's trailing clothoid half, which shares the resuming move's
/// source line; an unblended seam is preceded by a different move's body.
fn blend_consumed_head(moves: &[Move], i: usize) -> bool {
    i > 0
        && matches!(moves[i - 1].segment.spatial, Some(Segment::Clothoid(_)))
        && moves[i - 1].source.start_line == moves[i].source.start_line
}

/// Minimum surviving spatial length of a head-trimmed move. Below this the
/// remainder is degenerate and the commit boundary is refused.
const TRIM_EPS_MM: f64 = 1e-6;

fn dist3(a: [f64; 3], b: [f64; 3]) -> f64 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    let dz = a[2] - b[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
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
