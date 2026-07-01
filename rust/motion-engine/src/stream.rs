use std::collections::{HashSet, VecDeque};
use std::time::Instant;

use geometry::path::lowering::PositionProfile;
use geometry::path::{CurvatureProfile, Line, Segment};
use geometry::{
    ChainFitConfig, FitError, GeometryError, Move, ResumeState, VelocityConfig, VelocityError,
    VelocityLimits, fit_chain_with_resume, plan_velocity_warm_start,
};
use nurbs::bezier::{BezierPiece, bezier_pieces_to_nurbs, extract_bezier_pieces};
use trajectory::{AxisChainSet, ChainStage, CompiledChain, ShapedSegment, ShapedSignal};

use crate::lowering::{LoweringError, lower_move};

const SEGMENT_TIME_EPS_S: f64 = 1e-9;
const CONTIGUITY_EPS_MM: f64 = 1e-6;
/// Curvature below this (1/mm) resumes as a straight seam (no carried circle).
const RESUME_KAPPA_EPS: f64 = 1e-6;

#[derive(Debug, Clone, Copy)]
pub struct StreamConfig {
    pub chain: ChainFitConfig,
    pub velocity: VelocityConfig,
    pub fit_tol_mm: f64,
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
    /// A producer-stall watermark fired the brake-to-rest solve too late: the
    /// locked lead remaining ahead of the playhead is already below the
    /// solve-time budget, so the deceleration ramp cannot be planned and
    /// dispatched before its first piece must play. Self-identifying on purpose
    /// — a downstream late dispatch traced here means the fixed solve-time
    /// constant was sized too short, not a generic clock fault. Never padded over.
    BrakeToRestShortfall {
        lead_remaining: f64,
        solve_const: f64,
    },
    /// A move entered the buffer whose spatial start does not meet the toolhead
    /// where the previous move (or the committed odometer) left it. Real slicer
    /// output is always position-contiguous; a gap means the move stream was
    /// stitched wrong upstream. Caught here so the offending move is named at
    /// ingress, not as a downstream `ZeroMotion` deep in the fitter.
    Discontinuity {
        line_no: u32,
        expected: [f64; 3],
        got: [f64; 3],
        gap_mm: f64,
    },
    PostProcess(PostProcessError),
}

impl std::fmt::Display for StreamError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Fit(e) => write!(f, "chain fit: {e:?}"),
            Self::Velocity(e) => write!(f, "velocity plan: {e:?}"),
            Self::Lowering(e) => write!(f, "lowering: {e}"),
            Self::Geometry(e) => write!(f, "head-trim geometry: {e:?}"),
            Self::BrakeToRestShortfall {
                lead_remaining,
                solve_const,
            } => write!(
                f,
                "brake-to-rest shortfall: locked lead {lead_remaining:.6}s below \
                 solve-time budget {solve_const:.6}s — solve-time constant too short"
            ),
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
            Self::PostProcess(e) => write!(f, "post-processing: {e}"),
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
impl From<PostProcessError> for StreamError {
    fn from(e: PostProcessError) -> Self {
        Self::PostProcess(e)
    }
}

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
    entry_v: f64,
    odometer: Vec<f64>,
    t_committed: f64,
    /// Spatial length consumed from the current front move's head by the blend
    /// committed at the last seam. Fed back into the next fit so the leading
    /// corner re-fits to the curvature it had before the head was trimmed; 0.0
    /// when the front move is untrimmed (fresh, force-drained, or commit at a
    /// plain seam). See docs/rewrite/windowed-fit-ceiling-jitter.md.
    committed_head_len: f64,
    /// G2 endpoint the last commit ended at — the next fit resumes from it so
    /// curvature is continuous across the seam. `None`, or curvature ~0, means a
    /// straight (zero-curvature) resume, which is every commit at a clean seam.
    resume: Option<ResumeState>,
    /// Velocity at the most recent plan's finality barrier, carried so the
    /// streaming driver can size the producer-stall brake-to-rest watermark from
    /// `t_brake(v_barrier)` without re-planning — the speed the planner is riding,
    /// whether or not that plan advanced the commit. `0.0` before the first plan.
    last_v_barrier: f64,
    config: StreamConfig,
    axis_chains: AxisChainSet,
    post_history: VecDeque<ShapedSegment>,
}

impl StreamState {
    #[must_use]
    pub fn new(
        config: StreamConfig,
        axis_chains: AxisChainSet,
        home_pos: &[f64],
        t_start: f64,
    ) -> Self {
        Self {
            buffer: VecDeque::new(),
            entry_v: 0.0,
            odometer: home_pos.to_vec(),
            t_committed: t_start,
            committed_head_len: 0.0,
            resume: None,
            last_v_barrier: 0.0,
            config,
            axis_chains,
            post_history: VecDeque::new(),
        }
    }

    pub fn set_axis_chains(&mut self, axis_chains: AxisChainSet) {
        self.axis_chains = axis_chains;
    }

    pub fn reset(&mut self, home_pos: &[f64], t_start: f64) {
        self.buffer.clear();
        self.entry_v = 0.0;
        self.odometer = home_pos.to_vec();
        self.t_committed = t_start;
        self.committed_head_len = 0.0;
        self.resume = None;
        self.last_v_barrier = 0.0;
        self.post_history.clear();
    }

    pub fn push(&mut self, m: Move) -> Result<(), StreamError> {
        if let Some(seg) = &m.segment.spatial {
            let got = seg.point_at(0.0);
            let expected = self.expected_spatial_end();
            let gap_mm = dist3(expected, got);
            if gap_mm > CONTIGUITY_EPS_MM {
                return Err(StreamError::Discontinuity {
                    line_no: m.source.start_line,
                    expected,
                    got,
                    gap_mm,
                });
            }
        }
        self.buffer.push_back(m);
        Ok(())
    }

    fn expected_spatial_end(&self) -> [f64; 3] {
        for m in self.buffer.iter().rev() {
            if let Some(seg) = &m.segment.spatial {
                return seg.point_at(m.segment.s_len());
            }
        }
        [self.odometer[0], self.odometer[1], self.odometer[2]]
    }

    pub fn advance_time(&mut self, dt: f64) {
        debug_assert!(
            self.buffer.is_empty(),
            "advance_time requires a drained buffer"
        );
        debug_assert_eq!(self.entry_v, 0.0, "advance_time requires rest at the seam");
        if dt > 0.0 {
            self.t_committed += dt;
            self.post_history.clear();
        }
    }

    pub fn advance_idle(&mut self, target_t: f64) {
        debug_assert!(
            self.buffer.is_empty(),
            "advance_idle requires a drained buffer"
        );
        debug_assert_eq!(self.entry_v, 0.0, "advance_idle requires rest at the seam");
        if target_t > self.t_committed {
            self.t_committed = target_t;
            self.post_history.clear();
        }
    }

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
        self.post_history.clear();
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

    /// Velocity at the most recent plan's finality barrier (`0.0` before any plan).
    #[must_use]
    pub fn last_v_barrier(&self) -> f64 {
        self.last_v_barrier
    }

    /// Jerk-limited time to brake from the last barrier velocity to rest, sizing
    /// the producer-stall flush watermark. A safe over-estimate for curved
    /// geometry, so the watermark fires slightly early — never late.
    #[must_use]
    pub fn stall_brake_time(&self) -> f64 {
        jerk_limited_brake_time(
            self.last_v_barrier,
            self.config.limits.accel_mm_s2,
            self.config.velocity.max_jerk_mm_s3,
        )
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
        let batch = crate::timing::next_batch_seq();
        let line_lo = moves.first().map_or(0, |m| m.source.start_line);
        let line_hi = moves.last().map_or(0, |m| m.source.start_line);

        let fit_clock = Instant::now();
        let outcome = fit_chain_with_resume(
            &moves,
            self.config.chain,
            self.committed_head_len,
            self.resume,
        )?;
        let fit_us = fit_clock.elapsed().as_micros();
        tracing::info!(
            subsystem = "motion",
            event = "pipe_fit",
            batch,
            line_lo,
            line_hi,
            n_in = moves.len(),
            n_out = outcome.moves.len(),
            fit_us,
            t_us = crate::timing::mono_us(),
            "[pipe] fit"
        );

        let plan_clock = Instant::now();
        let profile = plan_velocity_warm_start(&outcome, self.config.velocity, self.entry_v)?;
        let plan_us = plan_clock.elapsed().as_micros();
        tracing::info!(
            subsystem = "motion",
            event = "pipe_plan",
            batch,
            line_lo,
            line_hi,
            plan_us,
            t_us = crate::timing::mono_us(),
            "[pipe] plan"
        );

        let n = outcome.moves.len();
        let mut pos = self.odometer.clone();
        let mut t = self.t_committed;
        let mut segs: Vec<ShapedSegment> = Vec::with_capacity(n);
        let mut seam_xyz: Vec<[f64; 3]> = Vec::with_capacity(n);
        let lower_clock = Instant::now();
        for (gm, vm) in outcome.moves.iter().zip(&profile.moves) {
            seam_xyz.push([pos[0], pos[1], pos[2]]);
            let mut seg = lower_move(
                gm,
                vm,
                t,
                &pos,
                self.config.fit_tol_mm,
                &self.axis_chains.chains,
            )?;
            seg.source_line = gm.source.start_line;
            t = seg.t_end;
            advance_odometer(&mut pos, gm);
            segs.push(seg);
        }
        let lower_us = lower_clock.elapsed().as_micros();
        tracing::info!(
            subsystem = "motion",
            event = "pipe_lower",
            batch,
            line_lo,
            line_hi,
            n,
            lower_us,
            t_us = crate::timing::mono_us(),
            "[pipe] lower"
        );
        let total_t = t - self.t_committed;

        let commit_count = if force {
            n
        } else {
            let unblended: HashSet<u32> =
                outcome.report.unblended.iter().map(|u| u.line_no).collect();
            let setback =
                brake_to_rest_setback(&outcome.moves, self.config.velocity.max_jerk_mm_s3);
            let total_arc: f64 = outcome.moves.iter().map(|m| m.segment.s_len()).sum();
            let mut arc_to_seam = 0.0_f64;
            let mut chosen = 0usize;
            for i in 1..=profile.barrier {
                arc_to_seam += outcome.moves[i - 1].segment.s_len();
                if total_arc - arc_to_seam < setback {
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
        debug_assert!(
            force || commit_count <= profile.barrier,
            "commit boundary {commit_count} past finality barrier {} — \
             a seam still open to a future append would be committed",
            profile.barrier
        );
        self.last_v_barrier = profile.v_barrier;

        tracing::info!(
            subsystem = "motion",
            event = "commit_decision",
            batch,
            force,
            n,
            unblended = outcome.report.unblended.len(),
            commit_count,
            total_t,
            barrier = profile.barrier,
            v_barrier = profile.v_barrier,
            entry_v = self.entry_v,
            t_committed = self.t_committed,
            "[commit-decision]"
        );

        if commit_count == 0 {
            return Ok(Vec::new());
        }

        let commit_count = self.post_commit_count(commit_count, force, &segs);

        if commit_count == 0 {
            return Ok(Vec::new());
        }

        let committed = apply_axis_chains(
            &self.post_history,
            &segs,
            commit_count,
            force,
            &self.axis_chains.chains,
        )?;

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

        self.resume = if commit_count == n {
            None
        } else {
            resume_endpoint(
                outcome.moves[commit_count].segment.spatial.as_ref(),
                &self.odometer,
            )
        };

        if commit_count == n {
            self.buffer.clear();
            self.committed_head_len = 0.0;
        } else {
            let keep_line = outcome.moves[commit_count].source.start_line;
            while self
                .buffer
                .front()
                .is_some_and(|m| m.source.start_line < keep_line)
            {
                self.buffer.pop_front();
            }
            // `keep_line` retires moves wholly before the seam, but a move can be
            // fully committed yet still carry the seam's source line (a collinear
            // split or coalesced run emits one move as several pieces). Drop any
            // front whose geometry has already reached the committed odometer so the
            // trim below never sees a degenerate (zero-length) remainder.
            while self.front_reached_odometer() {
                self.buffer.pop_front();
            }
            // The clean seam can fall at an internal sub-piece boundary of the kept
            // raw move — a blend exit, or a collinear split that emits one move as
            // several line pieces — leaving the raw move starting behind the
            // committed odometer. Trim it to start exactly at the seam so the next
            // re-fit's first piece is C0 with what was just emitted, and feed the
            // consumed head length back as the next fit's blend-budget restore so the
            // trailing corner re-fits to the curvature it had before the head was
            // committed (else a shorter front yields a sharper apex and a corner cap
            // below the already-committed entry velocity — an OverCommitted abort).
            self.committed_head_len = if self.front_starts_behind_odometer() {
                self.trim_front_to_seam()?
            } else {
                0.0
            };
        }

        self.post_history
            .extend(segs.into_iter().take(commit_count));
        self.trim_post_history();

        Ok(committed)
    }

    /// Materialize the deferred brake-to-rest on a producer-stall watermark.
    /// `lead_remaining` is the locked lead still ahead of the playhead at trigger
    /// time. If it is already below the fixed solve-time budget the ramp cannot
    /// be planned and dispatched before its first piece must play, so we fail
    /// loud with [`StreamError::BrakeToRestShortfall`] instead of sending a piece
    /// into the MCU's past. Otherwise it is the ordinary forced drain-to-rest —
    /// and if a move arrives after this returns, the next `commit` simply resumes
    /// locked commits from the new entry velocity.
    pub fn commit_stall_brake(
        &mut self,
        lead_remaining: f64,
        solve_const: f64,
    ) -> Result<Vec<ShapedSegment>, StreamError> {
        if self.buffer.is_empty() {
            return Ok(Vec::new());
        }
        if lead_remaining <= solve_const {
            return Err(StreamError::BrakeToRestShortfall {
                lead_remaining,
                solve_const,
            });
        }
        self.commit(true)
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

    /// Whether the kept front move's geometric start sits behind the committed
    /// odometer — i.e. the seam landed inside this raw move and part of it was
    /// already emitted. True at a blend exit and at a collinear split where one raw
    /// move emitted as several line pieces.
    fn front_starts_behind_odometer(&self) -> bool {
        self.buffer
            .front()
            .and_then(|m| m.segment.spatial.as_ref())
            .is_some_and(|seg| {
                dist3(
                    seg.point_at(0.0),
                    [self.odometer[0], self.odometer[1], self.odometer[2]],
                ) > TRIM_EPS_MM
            })
    }

    /// Whether the front move's geometric end has reached the committed odometer —
    /// the move is fully committed and must be retired rather than trimmed.
    fn front_reached_odometer(&self) -> bool {
        self.buffer.front().is_some_and(|m| {
            m.segment.spatial.as_ref().is_some_and(|seg| {
                dist3(
                    seg.point_at(m.segment.s_len()),
                    [self.odometer[0], self.odometer[1], self.odometer[2]],
                ) <= TRIM_EPS_MM
            })
        })
    }

    /// Replace the front buffer move with the portion that survives the seam: a
    /// line from the committed position to the move's original end. The committed
    /// blend already paid out the head (spatial and, proportionally, followers),
    /// so the per-mm follower ratios carry over unchanged. Selection has already
    /// proven this is a `Line` with a non-degenerate remainder
    /// ([`Self::head_trim_feasible`]).
    fn trim_front_to_seam(&mut self) -> Result<f64, StreamError> {
        let front = self
            .buffer
            .front()
            .expect("trim requires a kept front move");
        let Some(Segment::Line(line)) = &front.segment.spatial else {
            return Err(StreamError::Geometry(GeometryError::ZeroMotion));
        };
        let new_start = [self.odometer[0], self.odometer[1], self.odometer[2]];
        let head_consumed = dist3(line.start, new_start);
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
        Ok(head_consumed)
    }

    fn post_commit_count(&self, commit_count: usize, force: bool, segs: &[ShapedSegment]) -> usize {
        if force || commit_count == 0 {
            return commit_count;
        }
        let forward_support = self
            .axis_chains
            .chains
            .iter()
            .map(|chain| chain.max_half_support().1)
            .fold(0.0, f64::max);
        if forward_support <= 0.0 {
            return commit_count;
        }
        let latest_safe_t = segs.last().map_or(0.0, |seg| seg.t_end - forward_support);
        segs.iter()
            .take(commit_count)
            .take_while(|seg| seg.t_end <= latest_safe_t + 1e-12)
            .count()
    }

    fn trim_post_history(&mut self) {
        let back_support = self
            .axis_chains
            .chains
            .iter()
            .map(|chain| chain.max_half_support().0.abs())
            .fold(0.0, f64::max);
        let keep_after = self.t_committed - back_support;
        while self
            .post_history
            .front()
            .is_some_and(|seg| seg.t_end < keep_after)
        {
            self.post_history.pop_front();
        }
    }
}

/// A non-forced commit may cut wherever the fit output resumes a straight line
/// body (zero curvature: an unblended seam or the exit of a blend) — never
/// inside a blend, where curvature is nonzero and the velocity warm-start, which
/// carries only a scalar entry speed, would be invalid.
fn is_clean_seam(moves: &[Move], i: usize, unblended: &HashSet<u32>) -> bool {
    unblended.contains(&moves[i].source.start_line)
        || matches!(
            moves[i].segment.spatial,
            Some(Segment::Line(_)) | Some(Segment::Arc(_))
        )
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

/// Jerk-limited time to decelerate from `v` to rest under accel limit `a` and
/// jerk limit `j`: `v/a + a/j` once the ramp reaches `a` (`v > a²/j`), else the
/// triangular `2·√(v/j)`. Used only to size the producer-stall flush watermark,
/// never to locate the finality barrier — the backward velocity sweep does that
/// exactly. Curvature only slows a real stop, so this straight-line time is a
/// safe over-estimate and the watermark fires slightly early.
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

/// Arc length the commit boundary is held back from the buffer's tentative
/// terminal. The lowering reconstructs each move's velocity body against its run
/// terminal, so a move within one braking distance of the buffer's fictional
/// rest has its body shaped by that fiction — it is not yet terminal-independent
/// and an appended move would change it. Holding the boundary this far back makes
/// every committed body a function of geometry alone, so the committed trajectory
/// is final under append and output-equivalent to a full re-plan — positions
/// exactly, seam timing within the iterative velocity stage's tolerance. The
/// brake-to-rest tail this defers is the flush-only artifact. A safe over-estimate of the
/// jerk-limited stopping distance from the buffer's peak feedrate (`v · t_brake`
/// over-bounds the true `∫v dt`), so the held-back open tail stays bounded.
fn brake_to_rest_setback(moves: &[Move], max_jerk_mm_s3: f64) -> f64 {
    let v_peak = moves
        .iter()
        .map(|m| m.feedrate_mm_s.min(m.limits.max_velocity_mm_s))
        .fold(0.0_f64, f64::max);
    let a_min = moves
        .iter()
        .map(|m| m.limits.accel_mm_s2)
        .fold(f64::INFINITY, f64::min);
    v_peak * jerk_limited_brake_time(v_peak, a_min, max_jerk_mm_s3)
}

fn dist3(a: [f64; 3], b: [f64; 3]) -> f64 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    let dz = a[2] - b[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// The G2 state the resuming (first uncommitted) segment starts at: seam
/// position, start tangent, and curvature vector (`dT/ds`, toward the osculating
/// centre, magnitude `|kappa|`). The next window's fit must reproduce this so
/// curvature is continuous across the seam. A straight resume (start curvature
/// ~0, i.e. every commit at a clean line seam) returns `None` — a no-op.
fn resume_endpoint(spatial: Option<&Segment>, pos: &[f64]) -> Option<ResumeState> {
    let seg = spatial?;
    let kappa_start = seg.kappa_endpoints().0;
    if kappa_start.abs() < RESUME_KAPPA_EPS {
        return None;
    }
    let l = seg.s_len();
    let t0 = seg.heading_at(0.0);
    let ds = (l * 1e-4).clamp(1e-7, l);
    let t1 = seg.heading_at(ds);
    let dt = [t1[0] - t0[0], t1[1] - t0[1], t1[2] - t0[2]];
    let dn = (dt[0] * dt[0] + dt[1] * dt[1] + dt[2] * dt[2]).sqrt();
    if dn < 1e-12 {
        return None;
    }
    let k = kappa_start.abs() / dn;
    let mut p = [0.0; 3];
    for (slot, &v) in p.iter_mut().zip(pos.iter()) {
        *slot = v;
    }
    Some(ResumeState {
        pos: p,
        tangent: t0,
        kappa: [dt[0] * k, dt[1] * k, dt[2] * k],
    })
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

fn apply_axis_chains(
    history: &VecDeque<ShapedSegment>,
    base: &[ShapedSegment],
    commit_count: usize,
    force: bool,
    chains: &[CompiledChain],
) -> Result<Vec<ShapedSegment>, PostProcessError> {
    let mut out: Vec<ShapedSegment> = base.iter().take(commit_count).cloned().collect();
    if chains.iter().all(CompiledChain::is_empty) {
        return Ok(out);
    }
    let n_axes = out.iter().map(|seg| seg.axes.len()).max().unwrap_or(0);
    for seg in history.iter().chain(base.iter()) {
        if seg.axes.len() != n_axes {
            return Err(PostProcessError::AxisCountMismatch {
                expected: n_axes,
                got: seg.axes.len(),
            });
        }
    }
    let default_chain = CompiledChain::default();
    for axis in 0..n_axes {
        let chain = chains.get(axis).unwrap_or(&default_chain);
        apply_axis_chain(history, base, &mut out, axis, force, chain)?;
    }
    Ok(out)
}

fn apply_axis_chain(
    history: &VecDeque<ShapedSegment>,
    base: &[ShapedSegment],
    out: &mut [ShapedSegment],
    axis: usize,
    force: bool,
    chain: &CompiledChain,
) -> Result<(), PostProcessError> {
    let Some(kernel) = chain.stages.iter().find_map(|stage| match stage {
        ChainStage::SmoothKernel(kernel) => Some(kernel),
        ChainStage::LinearPressureAdvance { .. } => None,
    }) else {
        return Ok(());
    };
    let (k_lo, k_hi) = kernel.support();
    let first_t = history
        .front()
        .or_else(|| base.first())
        .map_or(0.0, |seg| seg.t_start);
    let last_t = base.last().map_or(first_t, |seg| seg.t_end);
    let at_stream_boundary = history.is_empty();
    let signal_segments: Vec<&ShapedSegment> = history.iter().chain(base.iter()).collect();
    for seg in out.iter_mut() {
        let need_lo = seg.t_start + k_lo;
        let need_hi = seg.t_end + k_hi;
        if need_lo < first_t && !at_stream_boundary {
            return Err(PostProcessError::MissingHistory { axis, t: need_lo });
        }
        if need_hi > last_t && !force {
            return Err(PostProcessError::MissingLookahead { axis, t: need_hi });
        }
        let sig = ShapedSignal::new_from_evaluator(kernel, seg.t_start, seg.t_end, |t| {
            eval_axis_with_edges(
                &signal_segments,
                axis,
                t,
                first_t,
                last_t,
                at_stream_boundary,
                force,
            )
        });
        let shaped = fit_axis_from_signal(axis, &seg.axes[axis], &sig)?;
        seg.axes[axis] = apply_trailing_zero_support(chain, shaped);
        if !seg.axes[axis]
            .control_points()
            .iter()
            .all(|v| v.is_finite())
        {
            return Err(PostProcessError::NonFiniteSample {
                axis,
                t: seg.t_start,
            });
        }
    }
    Ok(())
}

fn eval_axis_with_edges(
    segments: &[&ShapedSegment],
    axis: usize,
    t: f64,
    first_t: f64,
    last_t: f64,
    at_stream_boundary: bool,
    force: bool,
) -> f64 {
    if t < first_t {
        if !at_stream_boundary {
            return f64::NAN;
        }
        return eval_segment_axis(segments.first().expect("non-empty base"), axis, first_t);
    }
    if t > last_t {
        if !force {
            return f64::NAN;
        }
        return eval_segment_axis(segments.last().expect("non-empty base"), axis, last_t);
    }

    let mut idx = segments.partition_point(|seg| seg.t_end + SEGMENT_TIME_EPS_S < t);
    if idx >= segments.len() {
        idx = segments.len().saturating_sub(1);
    }
    let start = idx.saturating_sub(1);
    let end = (idx + 2).min(segments.len());
    for seg in &segments[start..end] {
        if t >= seg.t_start - SEGMENT_TIME_EPS_S && t <= seg.t_end + SEGMENT_TIME_EPS_S {
            return eval_segment_axis(seg, axis, t);
        }
    }
    if force && (t - last_t).abs() <= SEGMENT_TIME_EPS_S {
        return eval_segment_axis(segments.last().expect("non-empty base"), axis, last_t);
    }
    f64::NAN
}

fn eval_segment_axis(seg: &ShapedSegment, axis: usize, t: f64) -> f64 {
    nurbs::eval::eval(&seg.axes[axis], t.clamp(seg.t_start, seg.t_end))
}

fn fit_axis_from_signal(
    axis: usize,
    template: &nurbs::ScalarNurbs<f64>,
    sig: &ShapedSignal<'_>,
) -> Result<nurbs::ScalarNurbs<f64>, PostProcessError> {
    let template_pieces = extract_bezier_pieces(template);
    if template_pieces.is_empty() {
        return Err(PostProcessError::DegenerateAxisTrack { axis });
    }
    let domain_lo = template_pieces.first().expect("checked non-empty").u_start;
    let domain_hi = template_pieces.last().expect("checked non-empty").u_end;
    // The template's breakpoints seed the partition, but the convolved signal can
    // need finer pieces than the unshaped trajectory had — so refine each span to
    // the shaper's own tolerance rather than inheriting the template's resolution.
    let mut pieces = Vec::with_capacity(template_pieces.len());
    for piece in &template_pieces {
        refine_shaped_span(
            axis,
            sig,
            piece.u_start,
            piece.u_end,
            domain_lo,
            domain_hi,
            0,
            &mut pieces,
        )?;
    }
    Ok(bezier_pieces_to_nurbs(&pieces))
}

const SHAPED_FIT_TOL_MM: f64 = 1e-3;
const SHAPED_FIT_MAX_DEPTH: u32 = 16;
const SHAPED_FIT_MIN_SPAN_S: f64 = 5e-5;

fn shaped_hermite(
    axis: usize,
    sig: &ShapedSignal<'_>,
    t0: f64,
    t1: f64,
    domain_lo: f64,
    domain_hi: f64,
) -> Result<BezierPiece<f64>, PostProcessError> {
    let p0 = finite_sample(axis, sig, t0)?;
    let p1 = finite_sample(axis, sig, t1)?;
    let v0 = finite_derivative(axis, sig, t0, t1, domain_lo, domain_hi)?;
    let v1 = finite_derivative(axis, sig, t1, t0, domain_lo, domain_hi)?;
    Ok(super::lowering::hermite_cubic(t0, t1, p0, v0, p1, v1))
}

#[allow(clippy::too_many_arguments)]
fn refine_shaped_span(
    axis: usize,
    sig: &ShapedSignal<'_>,
    t0: f64,
    t1: f64,
    domain_lo: f64,
    domain_hi: f64,
    depth: u32,
    out: &mut Vec<BezierPiece<f64>>,
) -> Result<(), PostProcessError> {
    let piece = shaped_hermite(axis, sig, t0, t1, domain_lo, domain_hi)?;
    let mut worst = 0.0_f64;
    for frac in [0.25_f64, 0.5, 0.75] {
        let tm = frac.mul_add(t1 - t0, t0);
        worst = worst.max((piece.evaluate(tm) - finite_sample(axis, sig, tm)?).abs());
    }
    if depth >= SHAPED_FIT_MAX_DEPTH
        || (t1 - t0) <= 2.0 * SHAPED_FIT_MIN_SPAN_S
        || worst <= SHAPED_FIT_TOL_MM
    {
        out.push(piece);
        return Ok(());
    }
    let tm = 0.5 * (t0 + t1);
    refine_shaped_span(axis, sig, t0, tm, domain_lo, domain_hi, depth + 1, out)?;
    refine_shaped_span(axis, sig, tm, t1, domain_lo, domain_hi, depth + 1, out)
}

fn finite_sample(axis: usize, sig: &ShapedSignal<'_>, t: f64) -> Result<f64, PostProcessError> {
    let value = sig.eval(t);
    if value.is_finite() {
        Ok(value)
    } else {
        Err(PostProcessError::NonFiniteSample { axis, t })
    }
}

fn finite_derivative(
    axis: usize,
    sig: &ShapedSignal<'_>,
    t: f64,
    other: f64,
    domain_lo: f64,
    domain_hi: f64,
) -> Result<f64, PostProcessError> {
    let h = ((t - other).abs() * 1e-5).clamp(1e-7, 1e-4);
    let lo = (t - h).max(domain_lo);
    let hi = (t + h).min(domain_hi);
    if hi <= lo {
        return Err(PostProcessError::DegenerateAxisTrack { axis });
    }
    let dlo = finite_sample(axis, sig, lo)?;
    let dhi = finite_sample(axis, sig, hi)?;
    Ok((dhi - dlo) / (hi - lo))
}

fn apply_trailing_zero_support(
    chain: &CompiledChain,
    mut track: nurbs::ScalarNurbs<f64>,
) -> nurbs::ScalarNurbs<f64> {
    let mut seen_kernel = false;
    for stage in &chain.stages {
        match stage {
            ChainStage::SmoothKernel(_) => seen_kernel = true,
            ChainStage::LinearPressureAdvance { k } if seen_kernel => {
                track = apply_pressure_advance_to_track(&track, *k);
            }
            ChainStage::LinearPressureAdvance { .. } => {}
        }
    }
    track
}

fn apply_pressure_advance_to_track(
    track: &nurbs::ScalarNurbs<f64>,
    k: f64,
) -> nurbs::ScalarNurbs<f64> {
    let pieces = extract_bezier_pieces(track);
    let out_pieces: Vec<BezierPiece<f64>> = pieces
        .iter()
        .map(|piece| {
            let derivative = piece.differentiate();
            let coeffs: Vec<f64> = piece
                .coeffs
                .iter()
                .enumerate()
                .map(|(i, c)| c + k * derivative.coeffs.get(i).copied().unwrap_or(0.0))
                .collect();
            BezierPiece {
                u_start: piece.u_start,
                u_end: piece.u_end,
                coeffs,
            }
        })
        .collect();
    bezier_pieces_to_nurbs(&out_pieces)
}

#[cfg(test)]
mod tests;
