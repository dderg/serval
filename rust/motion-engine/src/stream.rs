use std::collections::{HashSet, VecDeque};
use std::time::Instant;

use geometry::path::lowering::PositionProfile;
use geometry::path::{Line, Segment};
use geometry::{
    ChainFitConfig, FitError, FitOutcome, GeometryError, Move, VelocityConfig, VelocityError,
    VelocityLimits, fit_chain_with_head_restore, plan_velocity_warm_start,
};
use nurbs::bezier::{BezierPiece, bezier_pieces_to_nurbs, extract_bezier_pieces};
use trajectory::{AxisChainSet, ChainStage, CompiledChain, ShapedSegment, ShapedSignal};

use crate::lowering::{LoweringError, lower_move};

const SEGMENT_TIME_EPS_S: f64 = 1e-9;

/// Tolerance for matching the stored frontier position to a re-fit sub-piece
/// boundary. The frontier's move is re-fit at full length and reproduces
/// deterministically, so the match is exact up to float noise.
const FRONTIER_POS_EPS_MM: f64 = 1e-6;

fn move_start_spatial(m: &Move) -> [f64; 3] {
    m.segment.spatial.as_ref().map_or([0.0; 3], |seg| {
        let p = seg.point_at(0.0);
        [p[0], p[1], p[2]]
    })
}

fn move_end_spatial(m: &Move) -> Option<[f64; 3]> {
    let s_len = m.segment.s_len();
    m.segment.spatial.as_ref().map(|seg| {
        let p = seg.point_at(s_len);
        [p[0], p[1], p[2]]
    })
}

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

/// Streaming look-ahead planner over the new geometry pipeline. Buffers submitted
/// moves, and on each commit re-fits the buffered moves at full length, plans the
/// velocity warm-started from the committed frontier, lowers only the sub-pieces
/// from the frontier forward, and emits up to the finality barrier. The frontier
/// advances *through* blend sub-pieces (mid-blend), so a continuous curve drains
/// just like a cornered perimeter and a blend longer than the MCU ring drains
/// incrementally. Re-fitting at full length keeps blends stable (a move's head and
/// tail blends never overlap, 0.5 budget cap); lowering from the exact carried
/// frontier keeps emitted clocks monotonic without re-deriving committed pieces.
/// Commit timing (real-time cadence) is the caller's; this type owns the
/// geometry/velocity state.
pub struct StreamState {
    buffer: VecDeque<Move>,
    entry_v: f64,
    /// Committed frontier position (all axes). The buffer holds full, untrimmed raw
    /// moves; each commit re-fits them (a pure function of the moves, so the
    /// committed-adjacent geometry reproduces bit-for-bit) and lowers only the
    /// sub-pieces from this frontier on — which may sit mid-blend. C0/C1 continuity
    /// across commits falls out of that determinism plus the exact carried frontier.
    odometer: Vec<f64>,
    /// Trajectory time at the frontier (`odometer`). The next commit lowers from
    /// this exact value, so the first emitted clock continues the last one without
    /// re-deriving — and thus never drifts backward past the dispatch's guard.
    t_committed: f64,
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
            last_v_barrier: 0.0,
            config,
            axis_chains,
            post_history: VecDeque::new(),
        }
    }

    pub fn set_axis_chains(&mut self, axis_chains: AxisChainSet) {
        self.axis_chains = axis_chains;
    }

    /// Analytical resume position carried to the next commit: the vertex the next
    /// fit window restarts lowering from. Read-only; used by the seam test harness
    /// to compare the resume vertex against the previous commit's emitted endpoint.
    #[must_use]
    pub fn odometer(&self) -> &[f64] {
        &self.odometer
    }

    pub fn reset(&mut self, home_pos: &[f64], t_start: f64) {
        self.buffer.clear();
        self.entry_v = 0.0;
        self.odometer = home_pos.to_vec();
        self.t_committed = t_start;
        self.last_v_barrier = 0.0;
        self.post_history.clear();
    }

    pub fn push(&mut self, m: Move) {
        self.buffer.push_back(m);
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
        let outcome = fit_chain_with_head_restore(&moves, self.config.chain, 0.0)?;
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

        // Locate the committed frontier among the re-fit's sub-pieces. The first
        // buffered move is re-fit at full length (i=0), so its sub-pieces reproduce
        // deterministically and the frontier lands exactly on a sub-piece boundary.
        // Sub-pieces before the frontier were already emitted and must NOT be
        // re-lowered: re-deriving a committed clock drifts it and trips the
        // dispatch's monotonic-clock guard. Plan and lower only from the frontier on.
        let frontier = [self.odometer[0], self.odometer[1], self.odometer[2]];
        let mut emit_lo = outcome.moves.len();
        {
            let mut acc = move_start_spatial(&outcome.moves[0]);
            for (i, gm) in outcome.moves.iter().enumerate() {
                if dist3(acc, frontier) <= FRONTIER_POS_EPS_MM {
                    emit_lo = i;
                    break;
                }
                if let Some(end) = move_end_spatial(gm) {
                    acc = end;
                }
            }
        }
        if emit_lo >= outcome.moves.len() {
            if force {
                self.buffer.clear();
                self.entry_v = 0.0;
            }
            return Ok(Vec::new());
        }

        let plan_outcome = FitOutcome {
            moves: outcome.moves[emit_lo..].to_vec(),
            report: outcome.report.clone(),
        };
        let n = plan_outcome.moves.len();

        let plan_clock = Instant::now();
        let profile = plan_velocity_warm_start(&plan_outcome, self.config.velocity, self.entry_v)?;
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

        let mut pos = self.odometer.clone();
        let mut t = self.t_committed;
        let mut segs: Vec<ShapedSegment> = Vec::with_capacity(n);
        let lower_clock = Instant::now();
        for (gm, vm) in plan_outcome.moves.iter().zip(&profile.moves) {
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
        self.last_v_barrier = profile.v_barrier;

        // Emission boundary: emit up to the finality barrier (velocity no future
        // append can change), so a commit can land mid-blend and drain a blend
        // longer than the MCU ring. The brake-to-rest setback holds back enough tail
        // to stop on a producer stall; `post_commit_count` holds back for axis-chain
        // forward support; the geometry-final cap never emits the last raw move,
        // whose trailing biclothoid is still open and would reshape on the next push.
        let emit_hi = if force {
            n
        } else {
            // Hold back the larger of the brake-to-rest setback and the last raw
            // move's arc length. Holding back a full last-move arc keeps its
            // sub-pieces unemitted until a successor arrives and fixes its trailing
            // blend — by arc, not source line, since an arc-fit move stream can
            // share one gcode line across many raw moves.
            let last_raw_arc = self.buffer.back().map_or(0.0, |m| m.segment.s_len());
            let setback =
                brake_to_rest_setback(&plan_outcome.moves, self.config.velocity.max_jerk_mm_s3);
            let hold_back = setback.max(last_raw_arc);
            let total_arc: f64 = plan_outcome.moves.iter().map(|m| m.segment.s_len()).sum();
            let mut arc_to_seam = 0.0_f64;
            let mut chosen = 0usize;
            for i in 1..=profile.barrier {
                arc_to_seam += plan_outcome.moves[i - 1].segment.s_len();
                if total_arc - arc_to_seam < hold_back {
                    break;
                }
                chosen = i;
            }
            chosen
        };
        let emit_hi = self.post_commit_count(emit_hi, force, &segs);

        tracing::info!(
            subsystem = "motion",
            event = "commit_decision",
            batch,
            force,
            n,
            unblended = outcome.report.unblended.len(),
            emit_lo,
            emit_hi,
            total_t,
            barrier = profile.barrier,
            v_barrier = profile.v_barrier,
            entry_v = self.entry_v,
            t_committed = self.t_committed,
            "[commit-decision]"
        );

        if emit_hi == 0 {
            return Ok(Vec::new());
        }

        let committed = apply_axis_chains(
            &self.post_history,
            &segs,
            emit_hi,
            force,
            &self.axis_chains.chains,
        )?;

        // Advance the frontier to the exact end of the last emitted sub-piece. The
        // geometric position (odometer) anchors the next re-fit's trim; the stored t
        // keeps the next commit's first clock continuous; the planned exit velocity
        // warm-starts the next plan. None of these is re-derived next round.
        let mut frontier_pos = self.odometer.clone();
        for gm in plan_outcome.moves.iter().take(emit_hi) {
            advance_odometer(&mut frontier_pos, gm);
        }
        self.odometer = frontier_pos;
        self.t_committed = segs[emit_hi - 1].t_end;
        self.entry_v = if force {
            0.0
        } else {
            profile.moves[emit_hi - 1].exit_v
        };

        // Retire raw moves the frontier has fully passed, but never one still needed
        // to rebuild an un-emitted blend. A blend straddles two raw moves, so keep a
        // move until its junction with the next is fully behind the frontier. The
        // first buffered move is done once the frontier is strictly past the second
        // move (`sf > second`), or sits on the second move's straight body — a Line
        // at that source, which only happens at an unblended seam (`sf == second`).
        // A blend entry is a Clothoid at that source, so the move is kept. The last
        // buffered move is always kept (its trailing junction is still open).
        if force {
            self.buffer.clear();
            self.entry_v = 0.0;
        } else if let Some(next) = plan_outcome.moves.get(emit_hi) {
            let sf = next.source.start_line;
            let next_is_line = matches!(next.segment.spatial, Some(Segment::Line(_)));
            while let Some(second) = self.buffer.get(1) {
                let second_src = second.source.start_line;
                if sf > second_src || (sf == second_src && next_is_line) {
                    self.buffer.pop_front();
                } else {
                    break;
                }
            }
        }

        self.post_history.extend(committed.iter().cloned());
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
    let pieces = template_pieces
        .iter()
        .map(|piece| {
            let t0 = piece.u_start;
            let t1 = piece.u_end;
            let p0 = finite_sample(axis, sig, t0)?;
            let p1 = finite_sample(axis, sig, t1)?;
            let v0 = finite_derivative(axis, sig, t0, t1, domain_lo, domain_hi)?;
            let v1 = finite_derivative(axis, sig, t1, t0, domain_lo, domain_hi)?;
            Ok(super::lowering::hermite_cubic(t0, t1, p0, v0, p1, v1))
        })
        .collect::<Result<Vec<_>, _>>()?;
    Ok(bezier_pieces_to_nurbs(&pieces))
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
