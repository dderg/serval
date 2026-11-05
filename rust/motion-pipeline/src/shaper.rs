use std::borrow::BorrowMut;
use std::collections::{HashMap, VecDeque};
use std::sync::Arc;

use crossbeam_channel::Sender;
use geometry::MIN_DEVICE_INTERVAL_S;
use nurbs::bezier::{BezierPiece, bezier_pieces_to_nurbs, extract_bezier_pieces};
use trajectory::{
    AnalyticMoveSpan, AxisChainSet, ChainStage, CompiledChain, ContinuousAxis, ContinuousSegment,
    RelativeSplinePiece, ShapedSignal, next_toward,
};

use crate::follower_projection::{FollowerState, project_followers};
use crate::lowering::{
    AccelerationExemption, AccelerationTrend, FitTol, HeldSample, LADDER_MAX_DEGREE, LADDER_PROBES_U, LadderBudget,
    LadderFailure, LadderMiss, LadderPolicy, LadderPva, LadderSpan, LadderTruth, SeamContinuity,
    VelocitySign, acceleration_bound, exact_piece, held_miss, ladder_accept, ladder_fit,
    quintic_in_u,
};
use crate::types::{BaseItem, Control, PostProcessError, TrajectoryItem};
use crate::worker_pool::WorkerPool;

#[path = "shaper_hidden_breaks.rs"]
mod hidden_breaks;

#[path = "shaper_exact.rs"]
pub(crate) mod exact;

#[path = "shaper_hull.rs"]
pub(crate) mod hull;

#[path = "shaper_taylor.rs"]
pub(crate) mod taylor;

#[path = "shaper_axis_enclosure.rs"]
pub(crate) mod axis_enclosure;

use exact::ExactSpan;
use hull::{FittedPiece, HullBudget};
use taylor::{Ball, TaylorModel};

pub(crate) use exact::{ExactConvolution, ExactKernel, ExactPiece};
pub(crate) use hidden_breaks::ShapedBreaks;

/// A signal's enclosure over a stretch: a slice of one the signal keeps,
/// borrowed for the asking, or one built for it.
pub(crate) enum Enclosure<'a> {
    Kept(&'a [ExactPiece]),
    Held(std::cell::Ref<'a, [ExactPiece]>),
    Built(Vec<ExactPiece>),
}

impl Enclosure<'_> {
    /// The enclosure with every piece's position moved by `base`.
    pub(crate) fn offset_by(self, base: f64) -> Enclosure<'static> {
        Enclosure::Built(self.iter().map(|piece| piece.offset_by(base)).collect())
    }

    pub(crate) fn into_owned(self) -> Vec<ExactPiece> {
        match self {
            Enclosure::Built(pieces) => pieces,
            kept => kept.to_vec(),
        }
    }
}

impl std::ops::Deref for Enclosure<'_> {
    type Target = [ExactPiece];

    fn deref(&self) -> &[ExactPiece] {
        match self {
            Enclosure::Kept(pieces) => pieces,
            Enclosure::Held(pieces) => pieces,
            Enclosure::Built(pieces) => pieces,
        }
    }
}

pub(crate) trait TrackSignal {
    fn eval_pva(&self, t: f64) -> (f64, f64, f64);
    fn eval(&self, t: f64) -> f64 {
        self.eval_pva(t).0
    }
    fn deriv(&self, t: f64) -> f64 {
        self.eval_pva(t).1
    }
    fn second_deriv(&self, t: f64) -> f64 {
        self.eval_pva(t).2
    }
    /// The travel between two points the caller has already sampled; a
    /// signal with a better-conditioned delta than the difference of its
    /// samples integrates it from `t0` to `t1` instead.
    fn position_delta(&self, (t0, p0): (f64, f64), (t1, p1): (f64, f64)) -> f64 {
        let _ = (t0, t1);
        p1 - p0
    }
    /// The magnitude a sample reading `position` was rounded at: the
    /// position itself, or for a view read through an offset the larger of
    /// it and the position the offset was taken from.
    fn position_rounding_scale(&self, position: f64) -> f64 {
        position.abs()
    }
    /// The acceleration's trend over `span` as a fit resolving only
    /// `resolved` sees it: none across a break anywhere in the span, but a
    /// smooth extremum in an edge sliver outside `resolved` is not one.
    fn acceleration_monotonicity(
        &self,
        _span: (f64, f64),
        _resolved: (f64, f64),
    ) -> Option<AccelerationTrend> {
        None
    }
    fn diagnostic(&self, _t: f64) -> Option<String> {
        None
    }
    /// Times at which the acceleration rises from a point as a square root,
    /// whose shoulder can bend between the ladder's probes.
    fn square_root_onsets(&self) -> &[f64] {
        &[]
    }
    /// The signal's exact polynomial on every sub-interval of `[lo, hi]`
    /// between its breaks, each with an a-priori bound on its rounding;
    /// `None` for a signal that is not exactly piecewise polynomial there.
    fn exact_polynomial_on(&self, _lo: f64, _hi: f64) -> Option<Vec<ExactPiece>> {
        None
    }
    /// Rigorous polynomial enclosures of the signal over `[lo, hi]`: on each
    /// sub-interval between its breaks every order lies within its
    /// polynomial's `rounding` of it, sub-intervals refined until each
    /// order's bound is within its `accuracy` where the signal resolves
    /// that. Exact polynomials are their own enclosure; `None` for a
    /// signal with neither.
    fn enclosure_on(&self, lo: f64, hi: f64, accuracy: [f64; 3]) -> Option<Enclosure<'_>> {
        let _ = accuracy;
        self.exact_polynomial_on(lo, hi).map(Enclosure::Built)
    }
}

impl TrackSignal for ContinuousAxis {
    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        ContinuousAxis::eval_pva(self, t).map_or((f64::NAN, f64::NAN, f64::NAN), |pva| {
            (pva.position, pva.velocity, pva.acceleration)
        })
    }

    fn enclosure_on(&self, lo: f64, hi: f64, accuracy: [f64; 3]) -> Option<Enclosure<'_>> {
        axis_enclosure::continuous_axis_enclosure(self, (lo, hi), accuracy).map(Enclosure::Built)
    }
}

impl<S: TrackSignal> TrackSignal for &S {
    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        S::eval_pva(self, t)
    }
    fn eval(&self, t: f64) -> f64 {
        S::eval(self, t)
    }
    fn deriv(&self, t: f64) -> f64 {
        S::deriv(self, t)
    }
    fn second_deriv(&self, t: f64) -> f64 {
        S::second_deriv(self, t)
    }
    fn position_delta(&self, from: (f64, f64), to: (f64, f64)) -> f64 {
        S::position_delta(self, from, to)
    }
    fn position_rounding_scale(&self, position: f64) -> f64 {
        S::position_rounding_scale(self, position)
    }
    fn acceleration_monotonicity(
        &self,
        span: (f64, f64),
        resolved: (f64, f64),
    ) -> Option<AccelerationTrend> {
        S::acceleration_monotonicity(self, span, resolved)
    }
    fn diagnostic(&self, t: f64) -> Option<String> {
        S::diagnostic(self, t)
    }
    fn square_root_onsets(&self) -> &[f64] {
        S::square_root_onsets(self)
    }
    fn exact_polynomial_on(&self, lo: f64, hi: f64) -> Option<Vec<ExactPiece>> {
        S::exact_polynomial_on(self, lo, hi)
    }
    fn enclosure_on(&self, lo: f64, hi: f64, accuracy: [f64; 3]) -> Option<Enclosure<'_>> {
        S::enclosure_on(self, lo, hi, accuracy)
    }
}

impl<F> TrackSignal for ShapedSignal<'_, F>
where
    F: Fn(f64) -> f64,
{
    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        ShapedSignal::eval_pva(self, t)
    }
}

/// One signal read through a constant position offset: derivatives and
/// travel deltas are the underlying signal's, positions are measured from
/// `base`. Fitting a shifted view keeps every fit source near zero without
/// changing what the convolution means, so the emitted track carries its own
/// relative base instead of an unbounded cumulative one.
pub(crate) struct ShiftedTrackSignal<S: TrackSignal> {
    inner: S,
    base: f64,
}

impl<S: TrackSignal> ShiftedTrackSignal<S> {
    pub(crate) fn new(inner: S, base: f64) -> Self {
        Self { inner, base }
    }
}

impl<S: TrackSignal> TrackSignal for ShiftedTrackSignal<S> {
    fn eval(&self, t: f64) -> f64 {
        self.inner.eval(t) - self.base
    }

    fn deriv(&self, t: f64) -> f64 {
        self.inner.deriv(t)
    }

    fn second_deriv(&self, t: f64) -> f64 {
        self.inner.second_deriv(t)
    }

    fn position_delta(&self, from: (f64, f64), to: (f64, f64)) -> f64 {
        self.inner.position_delta(from, to)
    }

    fn position_rounding_scale(&self, position: f64) -> f64 {
        position
            .abs()
            .max(self.inner.position_rounding_scale(position + self.base))
    }

    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        let (position, velocity, acceleration) = self.inner.eval_pva(t);
        (position - self.base, velocity, acceleration)
    }

    fn diagnostic(&self, t: f64) -> Option<String> {
        self.inner.diagnostic(t)
    }

    fn exact_polynomial_on(&self, lo: f64, hi: f64) -> Option<Vec<ExactPiece>> {
        let pieces = self.inner.exact_polynomial_on(lo, hi)?;
        Some(
            pieces
                .into_iter()
                .map(|piece| piece.offset_by(self.base))
                .collect(),
        )
    }

    fn enclosure_on(&self, lo: f64, hi: f64, accuracy: [f64; 3]) -> Option<Enclosure<'_>> {
        Some(
            self.inner
                .enclosure_on(lo, hi, accuracy)?
                .offset_by(self.base),
        )
    }
}

/// A convolution read through its evaluator, which also holds the exact
/// construction behind it.
pub(crate) struct ConvolutionSignal<'a, S: TrackSignal> {
    evaluated: S,
    exact: ExactConvolution<'a>,
    recent: exact::RecentChunks,
}

impl<'a, S: TrackSignal> ConvolutionSignal<'a, S> {
    pub(crate) fn new(evaluated: S, exact: ExactConvolution<'a>) -> Self {
        Self {
            evaluated,
            exact,
            recent: exact::RecentChunks::default(),
        }
    }
}

impl<S: TrackSignal> TrackSignal for ConvolutionSignal<'_, S> {
    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        self.evaluated.eval_pva(t)
    }

    fn position_rounding_scale(&self, position: f64) -> f64 {
        self.evaluated.position_rounding_scale(position)
    }

    fn diagnostic(&self, t: f64) -> Option<String> {
        self.evaluated.diagnostic(t)
    }

    fn exact_polynomial_on(&self, lo: f64, hi: f64) -> Option<Vec<ExactPiece>> {
        self.exact.pieces_over(lo, hi, &self.recent)
    }
}

/// A signal read from one side of its breaks: off a span's exact
/// polynomials, from the sub-interval facing `toward`, where it has them;
/// through its own evaluator where it does not.
struct FacingRead<'a, S> {
    sig: &'a S,
    exact: Option<&'a ExactSpan>,
    toward: f64,
}

impl<S: TrackSignal> TrackSignal for FacingRead<'_, S> {
    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        match self.exact {
            Some(exact) => exact.pva(t, self.toward),
            None => self.sig.eval_pva(t),
        }
    }

    fn eval(&self, t: f64) -> f64 {
        match self.exact {
            Some(exact) => exact.pva(t, self.toward).0,
            None => self.sig.eval(t),
        }
    }

    fn second_deriv(&self, t: f64) -> f64 {
        match self.exact {
            Some(exact) => exact.pva(t, self.toward).2,
            None => self.sig.second_deriv(t),
        }
    }
}

pub(crate) const SEGMENT_TIME_EPS_S: f64 = 1e-9;

const FIT_GROUP_SEGMENTS: usize = 8;
const FIT_GROUP_SPAN_S: f64 = 0.04;

/// The ends of the complete fit groups among `segments`, whose first opens
/// one. A group closes on its fourth segment, on the segment that carries
/// it `FIT_GROUP_SPAN_S` past its start, or on a segment ending at rest.
/// Every target in a group reads one signal table laid about the group's
/// start, so the shaper shapes and commits whole groups, and each group —
/// and with it every fit — depends on its own segments alone, never on how
/// the emits are cut.
pub(crate) fn fit_group_ends(segments: &[ContinuousSegment]) -> Vec<usize> {
    let mut ends = Vec::new();
    let mut start = 0;
    for (index, segment) in segments.iter().enumerate() {
        if index + 1 - start == FIT_GROUP_SEGMENTS
            || segment.t_end - segments[start].t_start >= FIT_GROUP_SPAN_S
            || segment.rest_at_end
        {
            ends.push(index + 1);
            start = index + 1;
        }
    }
    ends
}

/// The most of the first `count` segments that fill whole fit groups.
pub(crate) fn whole_groups(segments: &[ContinuousSegment], count: usize) -> usize {
    fit_group_ends(&segments[..count])
        .last()
        .copied()
        .unwrap_or(0)
}

/// The fit groups `segments` falls into, every one of them whole unless
/// `clamps_past_signal` lets the last close where the signal ends.
pub(crate) fn fit_groups(
    segments: &[ContinuousSegment],
    clamps_past_signal: bool,
) -> Vec<std::ops::Range<usize>> {
    let mut ends = fit_group_ends(segments);
    if ends.last().copied().unwrap_or(0) < segments.len() {
        assert!(
            clamps_past_signal,
            "shaper: {} segments past the last whole fit group are being shaped before \
             their group closed",
            segments.len() - ends.last().copied().unwrap_or(0)
        );
        ends.push(segments.len());
    }
    let mut start = 0;
    ends.into_iter()
        .map(|end| {
            let group = start..end;
            start = end;
            group
        })
        .collect()
}

/// Buffered lowered segments plus the shaped-leader cache: `shaped` holds
/// `segments[i]` with every kerneled non-follower axis already fitted. A
/// frontier segment's convolution window is fully covered when it is first
/// fitted, so the fit is final — later emits reuse it bit-identically
/// instead of re-fitting the whole frontier (the projection cache in
/// `follower_projection` relies on the same argument).
#[derive(Default)]
struct PendingSegments {
    segments: VecDeque<ContinuousSegment>,
    shaped: ShapedCache,
    materialized: usize,
}

/// The pending segments' fitted leaders, as the motors run them and as
/// the follower projection reads them (see [`ColumnTrack`]).
#[derive(Default)]
struct ShapedCache {
    stored: VecDeque<ContinuousSegment>,
    read: VecDeque<ContinuousSegment>,
}

impl ShapedCache {
    fn len(&self) -> usize {
        self.stored.len()
    }

    fn is_empty(&self) -> bool {
        self.stored.is_empty()
    }

    fn clear(&mut self) {
        self.stored.clear();
        self.read.clear();
    }

    fn pop_front(&mut self) {
        self.stored.pop_front();
        self.read.pop_front();
    }

    fn extend(&mut self, stored: Vec<ContinuousSegment>, read: Vec<ContinuousSegment>) {
        assert_eq!(
            stored.len(),
            read.len(),
            "every fitted leader segment has its read form"
        );
        self.stored.extend(stored);
        self.read.extend(read);
    }

    fn window(&mut self, window: usize) -> (&[ContinuousSegment], &[ContinuousSegment]) {
        (
            &self.stored.make_contiguous()[..window],
            &self.read.make_contiguous()[..window],
        )
    }
}

impl PendingSegments {
    fn push(&mut self, segment: ContinuousSegment) {
        self.segments.push_back(segment);
    }

    fn clear(&mut self) {
        self.segments.clear();
        self.shaped.clear();
        self.materialized = 0;
    }

    fn len(&self) -> usize {
        self.segments.len()
    }

    fn is_empty(&self) -> bool {
        self.segments.is_empty()
    }

    fn ends_at_rest(&self) -> bool {
        self.segments
            .back()
            .is_some_and(|segment| segment.rest_at_end)
    }

    fn back(&self) -> Option<&ContinuousSegment> {
        self.segments.back()
    }

    fn iter(&self) -> impl Iterator<Item = &ContinuousSegment> {
        self.segments.iter()
    }

    /// Fits the changed sources of the segments buffered since the last
    /// call; returns how many segments and axes that was.
    fn materialize(
        &mut self,
        chains: &AxisChainSet,
        fit_tol: FitTol,
        pool: &WorkerPool,
    ) -> Result<(usize, usize), PostProcessError> {
        let fresh = &mut self.segments.make_contiguous()[self.materialized..];
        let rebuilt = materialize_changed_sources(fresh, chains, fit_tol, pool)?;
        let segments = fresh.len();
        self.materialized = self.segments.len();
        Ok((segments, rebuilt))
    }

    fn pop_front(&mut self) -> Option<ContinuousSegment> {
        let segment = self.segments.pop_front();
        if segment.is_some() {
            self.materialized = self
                .materialized
                .checked_sub(1)
                .expect("shaper committed a segment whose sources were never materialized");
            if !self.shaped.is_empty() {
                self.shaped.pop_front();
            }
        }
        assert!(
            self.shaped.len() <= self.segments.len(),
            "shaped-leader cache ran ahead of the pending segments"
        );
        segment
    }
}

/// Final pipeline stage: streaming axis-chain post-processing. Buffers lowered
/// segments until each one's convolution window is covered by lookahead, then
/// emits the shaped segment; keeps just-emitted raw segments as the history the
/// next windows read. A `Drain` marker flushes the buffered tail with the
/// window clamped past the terminal rest — exact, not speculative, because the
/// lowerer holds the timeline at that rest for the chains' forward support
/// before any subsequent motion. Time gaps in the signal (dwells, drain holds)
/// evaluate as the position held at the preceding rest.
pub struct Shaper {
    chains: AxisChainSet,
    fit_tol: FitTol,
    history: VecDeque<ContinuousSegment>,
    pending: PendingSegments,
    forward_support: f64,
    back_support: f64,
    history_trimmed: bool,
    follower_states: Vec<FollowerState>,
    toolhead_tap: Option<Sender<ContinuousSegment>>,
    pool: Arc<WorkerPool>,
    emit_batch: usize,
    segments_since_emit: usize,
    /// The commit and frontier counts of the last emit, while it committed
    /// nothing: until the pending groups reach past either, an emit would
    /// read exactly what that one read and commit nothing again.
    stalled: Option<(usize, usize)>,
    /// Leader group fits the planning side made ahead of the shaper
    /// ([`crate::leader_prefit`]).
    leader_prefits: LeaderPrefits,
}

impl Shaper {
    pub fn new(chains: AxisChainSet, fit_tol: FitTol, pool: Arc<WorkerPool>) -> Self {
        Self {
            forward_support: chains.forward_support(),
            back_support: chains.back_support(),
            chains,
            fit_tol,
            history: VecDeque::new(),
            pending: PendingSegments::default(),
            history_trimmed: false,
            follower_states: Vec::new(),
            toolhead_tap: None,
            pool,
            emit_batch: 1,
            segments_since_emit: 0,
            stalled: None,
            leader_prefits: LeaderPrefits::default(),
        }
    }

    /// Leader group fits made ahead on the planning side
    /// ([`crate::leader_prefit`]); a shaper fits any group it finds no
    /// match for itself.
    pub(crate) fn accept_leader_prefits(&mut self, fits: Vec<(LeaderGroupKey, Vec<ColumnTrack>)>) {
        self.leader_prefits.insert(fits);
    }

    pub(crate) fn chains(&self) -> &AxisChainSet {
        &self.chains
    }

    pub(crate) fn fit_tol(&self) -> FitTol {
        self.fit_tol
    }

    pub(crate) fn pool(&self) -> &Arc<WorkerPool> {
        &self.pool
    }

    pub(crate) fn leader_prefit_era(&self) -> u64 {
        self.leader_prefits.era()
    }

    /// Mirrors every emitted segment as it stands before the motor-side
    /// derivative-gain stages — the toolhead signal, where the emitted
    /// segments are the motor command.
    #[must_use]
    pub fn with_toolhead_tap(mut self, tap: Sender<ContinuousSegment>) -> Self {
        self.toolhead_tap = Some(tap);
        self
    }

    /// Takes in `segments` lowered segments before each emit instead of
    /// one, so a test can hold the output to the same trajectory however
    /// the emits are cut.
    #[cfg(feature = "test-support")]
    #[must_use]
    pub fn with_emit_batch(mut self, segments: usize) -> Self {
        assert!(segments > 0, "an emit batch holds at least one segment");
        self.emit_batch = segments;
        self
    }

    /// Fits the changed sources of every segment buffered since the last
    /// emit, which the window about to be shaped reads.
    fn materialize_pending(&mut self) {
        let started = crate::timing::stopwatch();
        let (segments, rebuilt) = self
            .pending
            .materialize(&self.chains, self.fit_tol, &self.pool)
            .unwrap_or_else(|error| panic!("shaper: {error}"));
        let elapsed_us = started.elapsed_us();
        crate::timing::record_stage(crate::timing::Stage::MaterializeSource, elapsed_us);
        if crate::timing::is_slow_phase(elapsed_us) {
            crate::timing::log_slow_phase(
                "materialize_source",
                elapsed_us,
                crate::timing::PhaseWorkload {
                    segments,
                    axes: rebuilt,
                    ..Default::default()
                },
                "",
            );
        }
    }

    pub fn feed(
        &mut self,
        item: BaseItem,
        output: &mut impl FnMut(TrajectoryItem) -> bool,
    ) -> bool {
        match item {
            BaseItem::Seg(item) => {
                self.pending.push(item);
                self.segments_since_emit += 1;
                if self.segments_since_emit < self.emit_batch {
                    return true;
                }
                self.emit(self.supported_count(), false, output)
            }
            BaseItem::Drain => {
                assert!(
                    self.pending.is_empty() || self.pending.ends_at_rest(),
                    "shaper: drain marker arrived while the trajectory is not at rest"
                );
                self.emit(self.pending.len(), true, output) && output(TrajectoryItem::Parked)
            }
            BaseItem::Control(ctrl) => {
                match &ctrl {
                    Control::Reset { .. } => {
                        self.pending.clear();
                        self.segments_since_emit = 0;
                        self.stalled = None;
                        self.history.clear();
                        self.history_trimmed = false;
                        self.leader_prefits.start_era();
                        for state in &mut self.follower_states {
                            state.reset_timeline();
                        }
                    }
                    Control::Dwell { .. }
                    | Control::SetAxisChains(_)
                    | Control::SetMesh { .. }
                    | Control::Dispatch(_) => {
                        assert!(
                            self.pending.is_empty() || self.pending.ends_at_rest(),
                            "shaper: control token arrived while the trajectory is not at \
                             rest — a Drain must precede it"
                        );
                        if !self.emit(self.pending.len(), true, output) {
                            return false;
                        }
                        if let Control::SetAxisChains(chains) = &ctrl {
                            let (new_forward, new_back) =
                                (chains.forward_support(), chains.back_support());
                            // The signal eras agree at the rest point this swap
                            // happens at, so kept history makes the resumed
                            // track exactly continuous with what was committed
                            // (a k-only change keeps the kernel bit-identical).
                            // Only a grown back support invalidates retention —
                            // fall back to the stream-boundary clamp then.
                            if new_back > self.back_support {
                                self.history.clear();
                                self.history_trimmed = false;
                                for state in &mut self.follower_states {
                                    state.clear_projected_history();
                                }
                            }
                            (self.forward_support, self.back_support) = (new_forward, new_back);
                            self.chains = chains.clone();
                            self.leader_prefits.start_era();
                        }
                    }
                }
                output(TrajectoryItem::Control(ctrl))
            }
        }
    }

    /// The input-closed path: flush the buffered tail with the window clamped
    /// past the end of the signal.
    pub fn finish(&mut self, output: &mut impl FnMut(TrajectoryItem) -> bool) -> bool {
        self.emit(self.pending.len(), true, output)
    }

    /// How many front segments have their forward convolution window covered
    /// by the buffered lookahead. A follower with its own kernel cascades:
    /// its convolution reads the projection, which is only final through the
    /// shaping frontier — the last pending segment whose directly-convolved
    /// tracks have their own lookahead covered. The time-based bound already
    /// includes the cascaded width, but segment granularity can leave a long
    /// straddling segment unprojectable, so gate on the frontier explicitly.
    /// A ratio-ramped follower needs no gate here: how far its smoothing
    /// reads ahead depends on how the leaders come to rest, so the
    /// projection itself commits only what it holds final (see
    /// `follower_projection::project_followers`).
    ///
    /// Every gate requires the buffer to reach strictly past `t_end +
    /// support`, with no slack: a fit reads its signal from both sides of its
    /// window's edges, so the segment past the last time a target reads must
    /// already be held, and then the slice every fit reads is the same
    /// whenever the shaper gets to it. The fit-time `MissingLookahead` checks
    /// can never fire on a gated segment. The lowerer's rest-holds are sized
    /// exactly to a chain's support, so a slack here admits sums that land a
    /// few ulps past the frontier and panics the fit (issue #405).
    fn supported_count(&self) -> usize {
        let Some(last) = self.pending.back() else {
            return 0;
        };
        let last_t = last.t_end;
        let plain = self
            .pending
            .iter()
            .take_while(|seg| seg.t_end + self.forward_support < last_t)
            .count();
        let own_hi = self.chains.max_follower_own_forward_support();
        if own_hi <= 0.0 {
            return plain;
        }
        let Some(frontier_t) = self.shaping_frontier_t(last_t) else {
            return 0;
        };
        let gated = self
            .pending
            .iter()
            .take_while(|seg| seg.t_end + own_hi < frontier_t)
            .count();
        plain.min(gated)
    }

    /// End time of the last pending segment whose direct convolution window is
    /// covered by the buffered lookahead.
    fn shaping_frontier_t(&self, last_t: f64) -> Option<f64> {
        let direct_hi = self.chains.direct_forward_support();
        self.pending
            .iter()
            .take_while(|seg| seg.t_end + direct_hi < last_t)
            .last()
            .map(|seg| seg.t_end)
    }

    fn emit(
        &mut self,
        count: usize,
        force: bool,
        output: &mut impl FnMut(TrajectoryItem) -> bool,
    ) -> bool {
        self.segments_since_emit = 0;
        let count = if force {
            count
        } else {
            whole_groups(self.pending.segments.make_contiguous(), count)
        };
        if count == 0 {
            return true;
        }
        let beyond_commit = self.chains.max_follower_own_forward_support() > 0.0
            || self.chains.has_projected_follower();
        let frontier_count = if !beyond_commit {
            count
        } else if force {
            self.pending.len()
        } else {
            let last_t = self.pending.back().expect("count > 0").t_end;
            let direct_hi = self.chains.direct_forward_support();
            let reached = self
                .pending
                .iter()
                .take_while(|seg| seg.t_end + direct_hi < last_t)
                .count();
            whole_groups(self.pending.segments.make_contiguous(), reached).max(count)
        };
        if !force && self.stalled == Some((count, frontier_count)) {
            return true;
        }
        self.materialize_pending();
        let pending = &mut self.pending;
        let segments = pending.segments.make_contiguous();
        let shaped = apply_axis_chains(
            AxisFitRequest {
                history: &self.history,
                base: segments,
                chains: &self.chains,
                fit_tol: self.fit_tol,
                boundary: if self.history_trimmed {
                    FitBoundary::Interior
                } else {
                    FitBoundary::StreamBoundary
                },
                lookahead: if force {
                    Lookahead::Forced
                } else {
                    Lookahead::Required
                },
                pool: &self.pool,
                prefits: &self.leader_prefits,
            },
            count,
            frontier_count,
            ShaperOutputs {
                follower_states: &mut self.follower_states,
                shaped_cache: &mut pending.shaped,
                toolhead_tap: self.toolhead_tap.as_ref(),
            },
        )
        .unwrap_or_else(|e| panic!("shaper: {e}"));
        let committed = shaped.len();
        for seg in shaped {
            if !output(TrajectoryItem::Seg(seg)) {
                return false;
            }
        }
        self.stalled = (committed == 0).then_some((count, frontier_count));
        if committed == 0 {
            return true;
        }
        for _ in 0..committed {
            let raw = self.pending.pop_front().expect("committed <= pending");
            self.history.push_back(raw);
        }
        let emitted_through = self
            .history
            .back()
            .expect("just pushed emitted segments")
            .t_end;
        // Retention is by COVERAGE, not by a segment's own end time: a dwell
        // leaves a gap-hold between segments, and that hold is anchored by
        // the segment before the gap. Drop the front only while the next
        // segment (plus its gap-hold back to the front's end) still covers
        // the back-support window strictly, so a fit whose window starts on
        // a segment's start still holds the segment before it.
        let keep_after = emitted_through - self.back_support;
        while self.history.len() >= 2 && self.history[1].t_start < keep_after {
            self.history.pop_front();
            self.history_trimmed = true;
        }
        self.leader_prefits.forget_through(emitted_through);
        true
    }
}

/// The axes of `segment` whose source is fitted before shaping: every
/// non-follower axis a chain changes, and a z read through a variable bed
/// surface. A source already fitted is kept.
fn axes_to_materialize<'a>(
    segment: &'a ContinuousSegment,
    chains: &'a AxisChainSet,
) -> impl Iterator<Item = usize> + 'a {
    segment
        .axes
        .iter()
        .enumerate()
        .filter(|(axis, source)| {
            let variable_surface_z = *axis == 2
                && matches!(source, ContinuousAxis::Analytic { span, .. } if span.has_variable_surface());
            let changed = chains
                .chains
                .get(*axis)
                .is_some_and(|chain| !chain.is_empty())
                && !chains.is_projected_follower(*axis);
            (variable_surface_z || changed) && !matches!(source, ContinuousAxis::Spline(_))
        })
        .map(|(axis, _)| axis)
}

fn materialize_source_axis(
    segment: &ContinuousSegment,
    axis: usize,
    chains: &AxisChainSet,
    fit_tol: FitTol,
    shared: Option<&SpanEnclosures>,
) -> Result<ContinuousAxis, PostProcessError> {
    let source = &segment.axes[axis];
    let variable_surface_z = axis == 2
        && matches!(source, ContinuousAxis::Analytic { span, .. } if span.has_variable_surface());
    let (mut breakpoints, cap_contacts) = match source {
        ContinuousAxis::Analytic { span, .. } => cap_contact_seeds(span),
        _ => (vec![segment.t_start, segment.t_end], Vec::new()),
    };
    if let ContinuousAxis::Analytic { span, .. } = source {
        if variable_surface_z {
            breakpoints.extend(span.surface_seam_times().ok_or_else(|| {
                fit_tolerance_without_probe(axis, segment.t_start, segment.t_end)
            })?);
        }
    }
    let mut curve = fit_axis_from_signal(
        AxisFit {
            axis,
            t_start: segment.t_start,
            t_end: segment.t_end,
            seed_breakpoints: &breakpoints,
            fit_tol,
            fit_context: "materialize_source",
            seam_continuity: SeamContinuity::Position,
        },
        &CapContactSource {
            source,
            axis,
            cap_contacts: &cap_contacts,
            shared,
            enclosure: std::cell::OnceCell::new(),
            whole: (segment.t_start, segment.t_end),
            accuracy: HullBudget {
                tol: fit_tol,
                velocity: f64::INFINITY,
            }
            .enclosure_accuracy(),
        },
    )?;
    if !chains.is_projected_follower(axis) {
        if let Some(chain) = chains.chains.get(axis) {
            curve = apply_zero_support_transform(chain.leading_transform(), axis, curve, fit_tol)?;
        }
    }
    Ok(ContinuousAxis::Spline(Arc::new(curve)))
}

/// Fits the changed sources of every segment in `segments`, one pool job
/// per axis: each fit reads only its own segment's source, so the batch an
/// emit needs fans out whole. Returns how many axes it rebuilt.
fn materialize_changed_sources<S: BorrowMut<ContinuousSegment> + Sync>(
    segments: &mut [S],
    chains: &AxisChainSet,
    fit_tol: FitTol,
    pool: &WorkerPool,
) -> Result<usize, PostProcessError> {
    let jobs: Vec<(usize, usize)> = segments
        .iter()
        .enumerate()
        .flat_map(|(index, segment)| {
            axes_to_materialize(segment.borrow(), chains).map(move |axis| (index, axis))
        })
        .collect();
    let participants = if cfg!(target_arch = "wasm32") {
        1
    } else {
        std::thread::available_parallelism()
            .map_or(1, |cores| cores.get())
            .min(jobs.len())
    };
    let accuracy = HullBudget {
        tol: fit_tol,
        velocity: f64::INFINITY,
    }
    .enclosure_accuracy();
    let shared: Vec<Option<SpanEnclosures>> = segments
        .iter()
        .enumerate()
        .map(|(index, segment)| {
            let segment = segment.borrow();
            let analytic = |axis: usize| match &segment.axes[axis] {
                ContinuousAxis::Analytic { span, .. } if axis_enclosure::encloses(span, axis) => {
                    Some(span)
                }
                _ => None,
            };
            let span = jobs
                .iter()
                .filter(|&&(job_index, _)| job_index == index)
                .find_map(|&(_, axis)| analytic(axis))?;
            let axes = jobs
                .iter()
                .filter(|&&(job_index, axis)| {
                    job_index == index
                        && analytic(axis).is_some_and(|other| Arc::ptr_eq(other, span))
                })
                .map(|&(_, axis)| axis)
                .collect();
            Some(SpanEnclosures {
                span: Arc::clone(span),
                axes,
                whole: (segment.t_start, segment.t_end),
                accuracy,
                built: std::sync::OnceLock::new(),
            })
        })
        .collect();
    let fitted = pool
        .map_indexed(
            participants,
            jobs.len(),
            || (),
            |(), job| {
                let (index, axis) = jobs[job];
                materialize_source_axis(
                    segments[index].borrow(),
                    axis,
                    chains,
                    fit_tol,
                    shared[index].as_ref(),
                )
            },
        )
        .expect("source materialize thread panicked");
    for (&(index, axis), source) in jobs.iter().zip(fitted) {
        Arc::make_mut(&mut segments[index].borrow_mut().axes)[axis] = source?;
    }
    Ok(jobs.len())
}

/// The share of a piece's tolerance the audit lets an enclosure's bound
/// spend before refining it.
#[cfg(feature = "fit-audit")]
const AUDIT_ENCLOSURE_SHARE: f64 = 1.0 / 1024.0;

/// Proven upper bounds on a fitted piece's position, velocity and
/// acceleration error over `resolved`, each settled against its `budgets`:
/// against the exact polynomials the fit proved it by where it had them,
/// else against the signal's enclosure there.
#[cfg(feature = "fit-audit")]
pub(crate) fn proven_error_bounds<S: TrackSignal>(
    sig: &S,
    exact: Option<&[ExactPiece]>,
    mono_u: &[f64],
    (t0, t1): (f64, f64),
    resolved: (f64, f64),
    budgets: [f64; 3],
) -> Option<[f64; 3]> {
    let piece = FittedPiece { mono_u, t0, t1 };
    if let Some(exact) = exact {
        return Some(hull::error_bounds(exact, piece, resolved, budgets));
    }
    let accuracy =
        [budgets[0], f64::INFINITY, budgets[2]].map(|budget| budget * AUDIT_ENCLOSURE_SHARE);
    let enclosure = sig.enclosure_on(resolved.0, resolved.1, accuracy)?;
    Some(hull::error_bounds(&enclosure, piece, resolved, budgets))
}

/// Fits the shaper's changed sources ahead of it, on the thread that lowers
/// the segments: a source fit reads only its own segment, so fitting it
/// before the shaper sees it is the fit the shaper would make, and the
/// planning side that waits on the shaper does it in parallel instead.
/// Chain swaps travel in order with the segments, so every segment is
/// fitted with the chains the shaper shapes it with.
pub(crate) struct SourceMaterializer {
    chains: AxisChainSet,
    fit_tol: FitTol,
    pool: Arc<WorkerPool>,
}

impl SourceMaterializer {
    pub(crate) fn of(shaper: &Shaper) -> Self {
        Self {
            chains: shaper.chains.clone(),
            fit_tol: shaper.fit_tol,
            pool: Arc::clone(&shaper.pool),
        }
    }

    pub(crate) fn materialize(&mut self, items: &mut [BaseItem]) {
        let started = crate::timing::stopwatch();
        let mut rest = items;
        while !rest.is_empty() {
            let swap = rest
                .iter()
                .position(|item| matches!(item, BaseItem::Control(Control::SetAxisChains(_))));
            let (run, after) = rest.split_at_mut(swap.unwrap_or(rest.len()));
            let mut segments: Vec<&mut ContinuousSegment> = run
                .iter_mut()
                .filter_map(|item| match item {
                    BaseItem::Seg(segment) => Some(segment),
                    BaseItem::Drain | BaseItem::Control(_) => None,
                })
                .collect();
            materialize_changed_sources(&mut segments, &self.chains, self.fit_tol, &self.pool)
                .unwrap_or_else(|error| panic!("shaper: {error}"));
            let Some((swap, after)) = after.split_first_mut() else {
                break;
            };
            if let BaseItem::Control(Control::SetAxisChains(chains)) = swap {
                self.chains = chains.clone();
            }
            rest = after;
        }
        crate::timing::record_stage(
            crate::timing::Stage::MaterializeSource,
            started.elapsed_us(),
        );
    }
}

/// An analytic source's materialize seeds — its ends, phase seams and
/// curvature-cap contacts — and those contacts.
fn cap_contact_seeds(span: &AnalyticMoveSpan) -> (Vec<f64>, Vec<f64>) {
    let cap_contacts: Vec<f64> = span.cap_contact_times().collect();
    assert!(
        cap_contacts.is_sorted(),
        "cap contacts are out of time order: {cap_contacts:?}"
    );
    let mut seeds = vec![span.t_start, span.t_end];
    seeds.extend(span.phase_seam_times());
    seeds.extend_from_slice(&cap_contacts);
    (seeds, cap_contacts)
}

/// The enclosures of a segment's analytic axes that read one span, built
/// once for all of them by the first axis's fit that asks — the axes share
/// each window's state and path trigonometry.
struct SpanEnclosures {
    span: Arc<AnalyticMoveSpan>,
    axes: Vec<usize>,
    whole: (f64, f64),
    accuracy: [f64; 3],
    built: std::sync::OnceLock<Option<Vec<Vec<ExactPiece>>>>,
}

impl SpanEnclosures {
    fn of(&self, axis: usize) -> Option<&[ExactPiece]> {
        let slot = self.axes.iter().position(|&shared| shared == axis)?;
        let built = self.built.get_or_init(|| {
            axis_enclosure::analytic_axes_enclosure(
                &self.span,
                &self.axes,
                self.whole,
                self.accuracy,
            )
        });
        built.as_ref().map(|axes| axes[slot].as_slice())
    }
}

/// A materialized source with the curvature-cap contacts its acceleration
/// approaches as a square root, and its enclosure over the whole segment,
/// built once at the fit's own accuracy — shared with the segment's other
/// analytic axes where `shared` reads it: every rung of every span is
/// proven against its stretch of it.
struct CapContactSource<'a> {
    source: &'a ContinuousAxis,
    axis: usize,
    cap_contacts: &'a [f64],
    shared: Option<&'a SpanEnclosures>,
    enclosure: std::cell::OnceCell<Option<Vec<ExactPiece>>>,
    whole: (f64, f64),
    accuracy: [f64; 3],
}

impl TrackSignal for CapContactSource<'_> {
    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        TrackSignal::eval_pva(self.source, t)
    }

    fn square_root_onsets(&self) -> &[f64] {
        self.cap_contacts
    }

    fn enclosure_on(&self, lo: f64, hi: f64, _accuracy: [f64; 3]) -> Option<Enclosure<'_>> {
        let whole = match self.shared {
            Some(shared) => shared.of(self.axis)?,
            None => self
                .enclosure
                .get_or_init(|| {
                    self.source
                        .enclosure_on(self.whole.0, self.whole.1, self.accuracy)
                        .map(Enclosure::into_owned)
                })
                .as_deref()?,
        };
        let first = whole.partition_point(|piece| piece.hi < lo);
        let past = whole.partition_point(|piece| piece.lo <= hi);
        Some(Enclosure::Kept(&whole[first..past.max(first)]))
    }
}

/// Whether the fit window's left edge is the start of the signal — where the
/// convolution clamps at the first sample — or an interior cut whose missing
/// history is an error.
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum FitBoundary {
    Interior,
    StreamBoundary,
}

/// Whether the fit may read past the last buffered sample by clamping (a
/// drain flush holds the timeline at the terminal rest) or must have its
/// lookahead covered.
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum Lookahead {
    Required,
    Forced,
}

impl FitBoundary {
    fn clamps_before_signal(self) -> bool {
        self == Self::StreamBoundary
    }
}

impl Lookahead {
    pub(crate) fn clamps_past_signal(self) -> bool {
        self == Self::Forced
    }
}

/// Everything one emit window's leader fits read: the signal the convolution
/// integrates (history plus the buffered base), the chains and tolerance it
/// is fitted with, and how the window's two edges behave.
#[derive(Clone, Copy)]
struct AxisFitRequest<'a> {
    history: &'a VecDeque<ContinuousSegment>,
    base: &'a [ContinuousSegment],
    chains: &'a AxisChainSet,
    fit_tol: FitTol,
    boundary: FitBoundary,
    lookahead: Lookahead,
    pool: &'a WorkerPool,
    prefits: &'a LeaderPrefits,
}

/// The shaper state one emit writes through: the follower projection states,
/// the shaped-leader cache, and the toolhead mirror.
struct ShaperOutputs<'a> {
    follower_states: &'a mut Vec<FollowerState>,
    shaped_cache: &'a mut ShapedCache,
    toolhead_tap: Option<&'a Sender<ContinuousSegment>>,
}

impl ShaperOutputs<'_> {
    fn mirror_toolhead(&self, out: &[ContinuousSegment]) {
        let Some(tap) = self.toolhead_tap else { return };
        for seg in out {
            tap.send(seg.clone())
                .expect("shaper: toolhead tap receiver dropped");
        }
    }
}

fn apply_axis_chains(
    request: AxisFitRequest,
    commit_count: usize,
    frontier_count: usize,
    outputs: ShaperOutputs,
) -> Result<Vec<ContinuousSegment>, PostProcessError> {
    let AxisFitRequest {
        history,
        base,
        chains,
        fit_tol,
        lookahead,
        pool,
        ..
    } = request;
    if chains.chains.iter().all(CompiledChain::is_empty)
        && !chains.has_projected_follower()
        && outputs.follower_states.iter().all(|s| !s.is_active())
    {
        let out: Vec<ContinuousSegment> = base.iter().take(commit_count).cloned().collect();
        outputs.mirror_toolhead(&out);
        return Ok(out);
    }
    let window = frontier_count.max(commit_count);
    let n_axes = base
        .iter()
        .take(window)
        .map(|seg| seg.axes.len())
        .max()
        .unwrap_or(0);
    let mut prev: Option<&ContinuousSegment> = None;
    for seg in history.iter().chain(base.iter()) {
        if seg.axes.len() != n_axes {
            return Err(PostProcessError::AxisCountMismatch {
                expected: n_axes,
                got: seg.axes.len(),
            });
        }
        if let Some(p) = prev {
            assert_gap_is_a_hold(p, seg);
        }
        prev = Some(seg);
    }
    assert!(
        outputs.shaped_cache.len() <= window,
        "frontier retreated below the shaped-leader cache: {} cached, window {window}",
        outputs.shaped_cache.len()
    );
    let work = crate::timing::PhaseWorkload {
        window,
        commit: commit_count,
        frontier: frontier_count,
        force: lookahead.clamps_past_signal(),
        ..Default::default()
    };
    if window > outputs.shaped_cache.len() {
        let mut fresh: Vec<ContinuousSegment> = base[outputs.shaped_cache.len()..window].to_vec();
        let read = fit_leader_axes(request, &mut fresh, n_axes, work)?;
        outputs.shaped_cache.extend(fresh, read);
    }
    let (frontier, read_frontier) = outputs.shaped_cache.window(window);
    let mut out: Vec<ContinuousSegment> = frontier.iter().take(commit_count).cloned().collect();
    let projection_started = crate::timing::stopwatch();
    let mut projection_timing = crate::follower_projection::ProjectionTiming::default();
    project_followers(
        crate::follower_projection::ProjectionWindow {
            base,
            frontier: read_frontier,
            chains,
            fit_tol,
            lookahead,
            pool,
        },
        &mut out,
        outputs.follower_states,
        &mut projection_timing,
    )?;
    let projection_us = projection_started.elapsed_us();
    projection_timing.record_stages(projection_us);
    if crate::timing::is_slow_phase(projection_us) {
        crate::timing::log_slow_phase(
            "follower_projection",
            projection_us,
            crate::timing::PhaseWorkload {
                segments: out.len(),
                axes: outputs
                    .follower_states
                    .iter()
                    .filter(|s| s.is_active())
                    .count(),
                ..work
            },
            &projection_timing.detail(),
        );
    }
    outputs.mirror_toolhead(&out);
    let motor_started = crate::timing::stopwatch();
    let (motor_axes, motor_pieces) = apply_motor_side_stages(request, &mut out)?;
    let motor_us = motor_started.elapsed_us();
    crate::timing::record_stage(crate::timing::Stage::MotorSide, motor_us);
    if crate::timing::is_slow_phase(motor_us) {
        crate::timing::log_slow_phase(
            "motor_side",
            motor_us,
            crate::timing::PhaseWorkload {
                segments: out.len(),
                axes: motor_axes,
                pieces: motor_pieces,
                ..work
            },
            "",
        );
    }
    Ok(out)
}

fn apply_motor_side_stages(
    request: AxisFitRequest,
    out: &mut [ContinuousSegment],
) -> Result<(usize, usize), PostProcessError> {
    let AxisFitRequest {
        chains, fit_tol, ..
    } = request;
    let mut rebuilt_axes = fit_motor_commands(request, out)?;
    let mut rebuilt_pieces = 0usize;
    for seg in out.iter_mut() {
        for axis in 0..seg.axes.len() {
            let Some(chain) = chains.chains.get(axis) else {
                continue;
            };
            let follower_linear_chain =
                chains.is_projected_follower(axis) && chain.follower_linear_transform();
            if follower_linear_chain
                || motor_command_column(chains, axis).is_some()
                || !chain.has_motor_side_gains()
            {
                continue;
            }
            let replacement = match &seg.axes[axis] {
                ContinuousAxis::Spline(track) => {
                    ContinuousAxis::Spline(Arc::new(apply_zero_support_transform(
                        chain.trailing_transform(),
                        axis,
                        (**track).clone(),
                        fit_tol,
                    )?))
                }
                ContinuousAxis::RelativeSpline {
                    base_position,
                    curve,
                } => ContinuousAxis::RelativeSpline {
                    base_position: *base_position,
                    curve: Arc::new(apply_zero_support_transform(
                        chain.trailing_transform(),
                        axis,
                        (**curve).clone(),
                        fit_tol,
                    )?),
                },
                ContinuousAxis::PiecewiseRelativeSpline(pieces) => {
                    rebuilt_pieces += pieces.len();
                    ContinuousAxis::PiecewiseRelativeSpline(apply_trailing_stages_to_pieces(
                        chain, axis, pieces, fit_tol,
                    )?)
                }
                _ => return Err(PostProcessError::DegenerateAxisTrack { axis }),
            };
            Arc::make_mut(&mut seg.axes)[axis] = replacement;
            rebuilt_axes += 1;
        }
    }
    Ok((rebuilt_axes, rebuilt_pieces))
}

fn motor_command_column(chains: &AxisChainSet, axis: usize) -> Option<FitColumn<'_>> {
    if chains.is_projected_follower(axis) {
        return None;
    }
    let chain = chains.chains.get(axis)?;
    match chain.trailing_transform()? {
        ChainStage::DerivativeGains { k1, k2 } => Some(FitColumn {
            axis,
            chain,
            signal: ColumnSignal::MotorCommand { k1: *k1, k2: *k2 },
        }),
        ChainStage::NonlinearAdvance(_) | ChainStage::SmoothKernel(_) => None,
    }
}

fn fit_motor_commands(
    request: AxisFitRequest,
    out: &mut [ContinuousSegment],
) -> Result<usize, PostProcessError> {
    let n_axes = out.first().map_or(0, |seg| seg.axes.len());
    let columns: Vec<FitColumn> = (0..n_axes)
        .filter_map(|axis| motor_command_column(request.chains, axis))
        .collect();
    let mut rebuilt = 0;
    for (axis, column, _) in fit_columns(request, out, &columns) {
        let column = column?.expect("a motor-command chain convolves with its kernel");
        for (seg, track) in out.iter_mut().zip(column) {
            Arc::make_mut(&mut seg.axes)[axis] = ContinuousAxis::Spline(track.stored);
            rebuilt += 1;
        }
    }
    Ok(rebuilt)
}

/// The trailing stages are position-blind — they add velocity and
/// acceleration terms — so each piece keeps the origin it was emitted with,
/// except that the transform moves the endpoints: every downstream origin is
/// re-derived from the transformed seam so the piecewise track stays exactly
/// continuous.
fn apply_trailing_stages_to_pieces(
    chain: &CompiledChain,
    axis: usize,
    pieces: &[RelativeSplinePiece],
    fit_tol: FitTol,
) -> Result<Arc<[RelativeSplinePiece]>, PostProcessError> {
    let mut out: Vec<RelativeSplinePiece> = Vec::with_capacity(pieces.len());
    for piece in pieces {
        let curve = Arc::new(apply_zero_support_transform(
            chain.trailing_transform(),
            axis,
            (*piece.curve).clone(),
            fit_tol,
        )?);
        let base_position = out.last().map_or(piece.base_position, |previous| {
            previous.base_position + nurbs::eval::eval(&previous.curve, previous.t_end)
                - nurbs::eval::eval(&curve, piece.t_start)
        });
        out.push(RelativeSplinePiece {
            base_position,
            curve,
            t_start: piece.t_start,
            t_end: piece.t_end,
        });
    }
    Ok(Arc::from(out))
}

fn constant_axis_column(
    segments: &[&ContinuousSegment],
    targets: &[ContinuousSegment],
    axis: usize,
) -> Option<Vec<ColumnTrack>> {
    let splines = segments
        .iter()
        .map(|seg| match &seg.axes[axis] {
            ContinuousAxis::Spline(curve) => Some(curve.as_ref()),
            _ => None,
        })
        .collect::<Option<Vec<_>>>()?;
    let mut control_points = splines
        .iter()
        .flat_map(|curve| curve.control_points().iter());
    let constant = *control_points.next()?;
    if !constant.is_finite() || control_points.any(|&point| point != constant) {
        return None;
    }
    targets
        .iter()
        .map(|seg| match &seg.axes[axis] {
            ContinuousAxis::Spline(curve) => Some(ColumnTrack {
                stored: Arc::clone(curve),
                read: None,
            }),
            _ => None,
        })
        .collect()
}

#[derive(Clone, Copy)]
pub(crate) enum ColumnSignal {
    /// A leader's shaped signal, whose seams carry what the followers
    /// reading it need ([`crate::follower_projection::leader_seam_continuity`]).
    Shaped {
        seam_continuity: SeamContinuity,
    },
    MotorCommand {
        k1: f64,
        k2: f64,
    },
}

impl ColumnSignal {
    fn fit_context(self) -> &'static str {
        match self {
            Self::Shaped { .. } => "smooth_kernel_target",
            Self::MotorCommand { .. } => "motor_command",
        }
    }

    fn convolution_input(self, shaped_input: AxisSignalTable) -> AxisSignalTable {
        match self {
            Self::Shaped { .. } => shaped_input,
            Self::MotorCommand { k1, k2 } => shaped_input.with_derivative_gains(k1, k2),
        }
    }

    fn seam_continuity(self) -> SeamContinuity {
        match self {
            Self::Shaped { seam_continuity } => seam_continuity,
            Self::MotorCommand { .. } => SeamContinuity::Velocity,
        }
    }
}

#[derive(Clone, Copy)]
pub(crate) struct FitColumn<'a> {
    axis: usize,
    chain: &'a CompiledChain,
    signal: ColumnSignal,
}

fn fit_axis_column(
    request: AxisFitRequest,
    targets: &[ContinuousSegment],
    column: FitColumn,
    target_workers: usize,
) -> Result<Option<Vec<ColumnTrack>>, PostProcessError> {
    let AxisFitRequest {
        history,
        base,
        fit_tol,
        boundary,
        lookahead,
        pool,
        ..
    } = request;
    let FitColumn {
        axis,
        chain,
        signal,
    } = column;
    let Some(kernel) = chain.kernel() else {
        return Ok(None);
    };
    let (k_lo, k_hi) = kernel.support();
    let first_t = history
        .front()
        .or_else(|| base.first())
        .map_or(0.0, |seg| seg.t_start);
    let last_t = base.last().map_or(first_t, |seg| seg.t_end);
    let window_segments: Vec<&ContinuousSegment> = history.iter().chain(base.iter()).collect();
    for seg in targets {
        let need_lo = seg.t_start - k_hi;
        let need_hi = seg.t_end - k_lo;
        if need_lo < first_t && !boundary.clamps_before_signal() {
            return Err(PostProcessError::MissingHistory { axis, t: need_lo });
        }
        if need_hi > last_t && !lookahead.clamps_past_signal() {
            return Err(PostProcessError::MissingLookahead { axis, t: need_hi });
        }
    }
    let window = SignalBounds {
        first_t,
        last_t,
        boundary,
        lookahead,
    };
    let groups = fit_groups(targets, lookahead.clamps_past_signal());
    let reads: Vec<(std::ops::Range<usize>, SignalBounds)> = groups
        .iter()
        .map(|group| {
            let span = (targets[group.start].t_start, targets[group.end - 1].t_end);
            read_window(&window_segments, span, kernel, window)
        })
        .collect();
    let prefitted: Vec<Option<&Vec<ColumnTrack>>> = groups
        .iter()
        .zip(&reads)
        .map(|(group, (range, bounds))| match signal {
            ColumnSignal::Shaped { .. } => request.prefits.fits.get(&LeaderGroupKey::new(
                axis,
                request.prefits.era,
                &targets[group.clone()],
                &window_segments[range.clone()],
                *bounds,
            )),
            ColumnSignal::MotorCommand { .. } => None,
        })
        .collect();
    let unfitted = || {
        reads
            .iter()
            .zip(&prefitted)
            .filter(|(_, prefit)| prefit.is_none())
            .map(|((range, _), _)| range)
    };
    let read_from = unfitted().map(|range| range.start).min().unwrap_or(0);
    let read_to = unfitted().map(|range| range.end).max().unwrap_or(read_from);
    let pieces: Vec<Vec<BezierPiece>> = window_segments[read_from..read_to]
        .iter()
        .map(|seg| match &seg.axes[axis] {
            ContinuousAxis::Spline(track) => extract_bezier_pieces(track),
            _ => panic!("shaper: changed axis was not materialized"),
        })
        .collect();
    let workers = target_workers.max(1).min(groups.len());
    let fitted = pool
        .map_indexed(
            workers,
            groups.len(),
            || (),
            |(), index| {
                if let Some(tracks) = prefitted[index] {
                    return Ok(tracks.clone());
                }
                let (range, bounds) = reads[index].clone();
                fit_group_alone(
                    axis,
                    &targets[groups[index].clone()],
                    kernel,
                    GroupRead {
                        segments: &window_segments[range.clone()],
                        pieces: &pieces[range.start - read_from..range.end - read_from],
                        bounds,
                    },
                    fit_tol,
                    signal,
                    pool,
                )
            },
        )
        .expect("leader target fit thread panicked");
    let mut tracks = Vec::with_capacity(targets.len());
    for result in fitted {
        tracks.extend(result?);
    }
    Ok(Some(tracks))
}

/// The segments one fit group's convolution reads, their axis pieces and
/// the bounds of that slice.
struct GroupRead<'r, 'a> {
    segments: &'r [&'a ContinuousSegment],
    pieces: &'r [Vec<BezierPiece>],
    bounds: SignalBounds,
}

/// One fit group's targets from the signal the group reads alone, about
/// the group's start: what else shares its emit never reaches the fit, so
/// the leader comes out the same however the shaper batches its emits.
fn fit_group_alone(
    axis: usize,
    group: &[ContinuousSegment],
    kernel: &nurbs::algebra::PiecewisePolynomialKernel,
    read: GroupRead,
    fit_tol: FitTol,
    signal: ColumnSignal,
    pool: &WorkerPool,
) -> Result<Vec<ColumnTrack>, PostProcessError> {
    if let Some(column) = constant_axis_column(read.segments, group, axis) {
        return Ok(column);
    }
    fit_axis_targets(
        axis,
        group,
        ShapedSignalSource::new(
            kernel,
            signal.convolution_input(AxisSignalTable::from_pieces(
                read.pieces.iter().flatten(),
                read.bounds,
            )),
            signal_breakpoints(read.segments, axis),
            group[0].t_start,
        ),
        fit_tol,
        signal,
        1,
        pool,
    )
}

/// What one leader column's fit of one group reads: the group, the slice
/// of segments its convolution reads and how that slice's edges behave,
/// within one era of the shaper's chains. Two fits with the same key are
/// the same fit wherever they run ([`fit_group_alone`]).
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) struct LeaderGroupKey {
    axis: usize,
    era: u64,
    group: [u64; 3],
    slice: [u64; 3],
    clamps_before_signal: bool,
    clamps_past_signal: bool,
}

impl LeaderGroupKey {
    fn new(
        axis: usize,
        era: u64,
        group: &[ContinuousSegment],
        slice: &[&ContinuousSegment],
        bounds: SignalBounds,
    ) -> Self {
        let span =
            |first: f64, last: f64, len: usize| [first.to_bits(), last.to_bits(), len as u64];
        Self {
            axis,
            era,
            group: span(group[0].t_start, group[group.len() - 1].t_end, group.len()),
            slice: span(slice[0].t_start, slice[slice.len() - 1].t_end, slice.len()),
            clamps_before_signal: bounds.boundary.clamps_before_signal(),
            clamps_past_signal: bounds.lookahead.clamps_past_signal(),
        }
    }

    fn group_end(&self) -> f64 {
        f64::from_bits(self.group[1])
    }
}

/// Leader group fits made ahead of the shaper, keyed by what they read.
/// The era advances wherever the chains or the timeline change, so a fit
/// made under the old ones never matches.
#[derive(Default)]
pub(crate) struct LeaderPrefits {
    era: u64,
    fits: HashMap<LeaderGroupKey, Vec<ColumnTrack>>,
}

impl LeaderPrefits {
    pub(crate) fn start_era(&mut self) {
        self.era += 1;
        self.fits.clear();
    }

    pub(crate) fn era(&self) -> u64 {
        self.era
    }

    pub(crate) fn insert(&mut self, fits: Vec<(LeaderGroupKey, Vec<ColumnTrack>)>) {
        self.fits.extend(fits);
    }

    fn forget_through(&mut self, emitted_through: f64) {
        self.fits.retain(|key, _| key.group_end() > emitted_through);
    }
}

/// The kerneled columns the shaper fits as leaders under `chains`.
pub(crate) fn leader_columns(chains: &AxisChainSet, n_axes: usize) -> Vec<FitColumn<'_>> {
    (0..n_axes)
        .filter(|&axis| !chains.is_projected_follower(axis))
        .filter_map(|axis| {
            let chain = chains.chains.get(axis)?;
            (!chain.is_empty()).then(|| FitColumn {
                axis,
                chain,
                signal: ColumnSignal::Shaped {
                    seam_continuity: crate::follower_projection::leader_seam_continuity(
                        chains, axis,
                    ),
                },
            })
        })
        .collect()
}

/// Whether every read of a kerneled leader column over `group` lies within
/// `window`, so its fit needs no segment the window does not hold.
pub(crate) fn reads_within(
    columns: &[FitColumn],
    group: &[ContinuousSegment],
    window: SignalBounds,
) -> bool {
    let span = (group[0].t_start, group[group.len() - 1].t_end);
    columns
        .iter()
        .filter_map(|column| column.chain.kernel())
        .all(|kernel| {
            let (k_lo, k_hi) = kernel.support();
            span.1 - k_lo < window.last_t
                && (span.0 - k_hi >= window.first_t || window.boundary.clamps_before_signal())
        })
}

/// One leader column's fit of `group`, whose reads lie within `window`
/// ([`reads_within`]), keyed as the shaper keys it; `None` where the
/// column has no kernel.
pub(crate) fn fit_leader_group(
    column: FitColumn,
    group: &[ContinuousSegment],
    segments: &[&ContinuousSegment],
    window: SignalBounds,
    (era, fit_tol): (u64, FitTol),
    pool: &WorkerPool,
) -> Option<Result<(LeaderGroupKey, Vec<ColumnTrack>), PostProcessError>> {
    let kernel = column.chain.kernel()?;
    let span = (group[0].t_start, group[group.len() - 1].t_end);
    let (range, bounds) = read_window(segments, span, kernel, window);
    let slice = &segments[range];
    let pieces: Vec<Vec<BezierPiece>> = slice
        .iter()
        .map(|seg| match &seg.axes[column.axis] {
            ContinuousAxis::Spline(track) => extract_bezier_pieces(track),
            _ => panic!("leader prefit: changed axis was not materialized"),
        })
        .collect();
    let key = LeaderGroupKey::new(column.axis, era, group, slice, bounds);
    Some(
        fit_group_alone(
            column.axis,
            group,
            kernel,
            GroupRead {
                segments: slice,
                pieces: &pieces,
                bounds,
            },
            fit_tol,
            column.signal,
            pool,
        )
        .map(|tracks| (key, tracks)),
    )
}

/// The segments a convolution of `kernel` over a fit group spanning `span`
/// reads: from the one starting strictly before the earliest time read to
/// the first one reaching strictly past the latest, so a gap a read lands
/// in carries its hold and a one-sided read at either edge of the window
/// finds signal. The slice depends on the group and the segments alone,
/// never on how far the window around them reaches (the lookahead gates
/// and the history retention hold both neighbours); an edge clamps only
/// where the read passes it, as the window's own edge would.
fn read_window(
    segments: &[&ContinuousSegment],
    (span_start, span_end): (f64, f64),
    kernel: &nurbs::algebra::PiecewisePolynomialKernel,
    window: SignalBounds,
) -> (std::ops::Range<usize>, SignalBounds) {
    let (k_lo, k_hi) = kernel.support();
    let (read_lo, read_hi) = (span_start - k_hi, span_end - k_lo);
    let lo = segments
        .partition_point(|seg| seg.t_start < read_lo)
        .saturating_sub(1);
    let hi = (segments.partition_point(|seg| seg.t_end <= read_hi) + 1).min(segments.len());
    let (first_t, last_t) = (segments[lo].t_start, segments[hi - 1].t_end);
    (
        lo..hi,
        SignalBounds {
            first_t,
            last_t,
            boundary: if read_lo < first_t {
                window.boundary
            } else {
                FitBoundary::Interior
            },
            lookahead: if read_hi > last_t {
                window.lookahead
            } else {
                Lookahead::Required
            },
        },
    )
}

/// The convolution's input for one axis, packaged for the target fits: the
/// kernel, the flattened signal it integrates, and the breakpoint sets that
/// seed the input and shaped partitions.
pub(crate) struct ShapedSignalSource<'a> {
    kernel: &'a nurbs::algebra::PiecewisePolynomialKernel,
    table: Arc<AxisSignalTable>,
    input_breaks: Vec<f64>,
    shaped_breaks: ShapedBreaks,
    input_degree: usize,
    shaped_offset: f64,
    gain_impulse_response: Option<GainImpulseResponse>,
    exact_kernel: ExactKernel,
}

#[derive(Clone, Copy)]
pub(crate) struct GainImpulse {
    at: f64,
    delta: f64,
    doublet: f64,
}

struct GainImpulseResponse {
    impulses: Vec<GainImpulse>,
    kernel_derivatives: [Vec<BezierPiece>; 4],
    support: (f64, f64),
}

impl GainImpulseResponse {
    fn new(kernel: &nurbs::algebra::PiecewisePolynomialKernel, impulses: &[GainImpulse]) -> Self {
        assert!(
            impulses.is_sorted_by(|left, right| left.at <= right.at),
            "gain impulses are out of time order"
        );
        let mut order = kernel.pieces.clone();
        let kernel_derivatives = std::array::from_fn(|_| {
            let current = order.clone();
            order = order.iter().map(BezierPiece::differentiate).collect();
            current
        });
        Self {
            impulses: impulses.to_vec(),
            kernel_derivatives,
            support: kernel.support(),
        }
    }

    fn kernel_derivatives_at(&self, tau: f64) -> [f64; 4] {
        let pieces = &self.kernel_derivatives[0];
        let piece = pieces
            .partition_point(|piece| piece.u_end <= tau)
            .min(pieces.len() - 1);
        self.kernel_derivatives
            .each_ref()
            .map(|order| order[piece].evaluate(tau))
    }

    fn response(&self, t: f64) -> (f64, f64, f64) {
        let (k_lo, k_hi) = self.support;
        let first = self
            .impulses
            .partition_point(|impulse| impulse.at <= t - k_hi);
        let end = self
            .impulses
            .partition_point(|impulse| impulse.at < t - k_lo);
        self.impulses[first..end]
            .iter()
            .fold((0.0, 0.0, 0.0), |(p, v, a), impulse| {
                let [k, dk, ddk, dddk] = self.kernel_derivatives_at(t - impulse.at);
                (
                    p + impulse.delta * k + impulse.doublet * dk,
                    v + impulse.delta * dk + impulse.doublet * ddk,
                    a + impulse.delta * ddk + impulse.doublet * dddk,
                )
            })
    }
}

struct WithGainImpulses<'r, S: TrackSignal> {
    convolved: S,
    response: Option<&'r GainImpulseResponse>,
}

impl<S: TrackSignal> TrackSignal for WithGainImpulses<'_, S> {
    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        let (p, v, a) = self.convolved.eval_pva(t);
        let Some(response) = self.response else {
            return (p, v, a);
        };
        let (dp, dv, da) = response.response(t);
        (p + dp, v + dv, a + da)
    }
}

const KERNEL_MASS_TOLERANCE: f64 = 1e-9;

pub(crate) fn unit_kernel_mass(kernel: &nurbs::algebra::PiecewisePolynomialKernel) -> f64 {
    let mass = trajectory::kernel_mass(kernel);
    assert!(
        (mass - 1.0).abs() <= KERNEL_MASS_TOLERANCE,
        "shaper kernel integrates to {mass}, not 1: it would scale every position it shapes"
    );
    mass
}

/// A shaped signal carries its input's offset at full weight: the kernel
/// has unit mass up to the rounding of its own integral, and a constant
/// window shapes to its constant exactly whether it is convolved or kept.
fn assert_unit_mass(kernel: &nurbs::algebra::PiecewisePolynomialKernel) {
    unit_kernel_mass(kernel);
}

impl<'a> ShapedSignalSource<'a> {
    pub(crate) fn new(
        kernel: &'a nurbs::algebra::PiecewisePolynomialKernel,
        input: AxisSignalTable,
        input_breaks: Vec<f64>,
        offset_at: f64,
    ) -> Self {
        let kernel_degree = kernel
            .pieces
            .iter()
            .map(|piece| piece.degree())
            .max()
            .expect("shaper kernel has no pieces");
        assert_unit_mass(kernel);
        let input_offset = input.eval_hinted(offset_at, &std::cell::Cell::new(0));
        assert!(
            input_offset.is_finite(),
            "shaper: convolution input offset at t={offset_at} is not finite"
        );
        let table = Arc::new(
            input
                .offset_by(input_offset)
                .with_piece_moments(kernel_degree),
        );
        Self {
            kernel,
            shaped_breaks: ShapedBreaks::new(kernel, &input_breaks),
            input_degree: table.max_degree(),
            gain_impulse_response: (!table.gain_impulses().is_empty())
                .then(|| GainImpulseResponse::new(kernel, table.gain_impulses())),
            table,
            input_breaks,
            shaped_offset: input_offset,
            exact_kernel: ExactKernel::new(kernel),
        }
    }

    /// The convolution's input where it is the table alone, so its jumps
    /// are the table's: derivative gains add impulses the table does not
    /// hold.
    fn hidden_break_input(&self) -> Option<&AxisSignalTable> {
        self.gain_impulse_response
            .is_none()
            .then_some(self.table.as_ref())
    }

    pub(crate) fn signal(&self) -> impl TrackSignal + use<'_, 'a> {
        let eval_table = Arc::clone(&self.table);
        let moment_table = Arc::clone(&self.table);
        let piece_hint = std::cell::Cell::new(0);
        let relative = ShapedSignal::new_from_polynomial_evaluator(
            self.kernel,
            move |t| eval_table.eval_hinted(t, &piece_hint),
            self.input_breaks.clone(),
            self.input_degree,
            move |lo, hi, degree, origin, orders| {
                moment_table.integrate_moments(lo, hi, degree, origin, orders)
            },
            self.table.input_jumps().to_vec(),
        );
        ConvolutionSignal::new(
            ShiftedTrackSignal::new(
                WithGainImpulses {
                    convolved: relative,
                    response: self.gain_impulse_response.as_ref(),
                },
                -self.shaped_offset,
            ),
            ExactConvolution {
                table: &self.table,
                kernel: &self.exact_kernel,
                impulses: self.table.gain_impulses(),
                position_offset: self.shaped_offset,
            },
        )
    }
}

fn fit_axis_targets(
    axis: usize,
    targets: &[ContinuousSegment],
    source: ShapedSignalSource,
    fit_tol: FitTol,
    signal: ColumnSignal,
    target_workers: usize,
    pool: &WorkerPool,
) -> Result<Vec<ColumnTrack>, PostProcessError> {
    if targets.is_empty() {
        return Ok(Vec::new());
    }
    let fit_context = signal.fit_context();
    let seam_continuity = signal.seam_continuity();
    let shaped_breaks = &source.shaped_breaks;
    let hidden_break_input = source.hidden_break_input();
    let shaped_times = shaped_breaks.times();
    let make_sig = || source.signal();
    let max_seed_spans = targets
        .iter()
        .map(|target| {
            let first = shaped_times.partition_point(|time| *time <= target.t_start);
            let last = shaped_times.partition_point(|time| *time < target.t_end);
            last.saturating_sub(first) + 1
        })
        .max()
        .unwrap_or(0);
    if max_seed_spans < LEADER_SPAN_PARALLEL_THRESHOLD {
        let fit_target = |target: &ContinuousSegment, sig: &_| {
            let pieces = fit_leader_target(
                AxisFit {
                    axis,
                    t_start: target.t_start,
                    t_end: target.t_end,
                    seed_breakpoints: shaped_times,
                    fit_tol,
                    fit_context,
                    seam_continuity,
                },
                sig,
                shaped_breaks,
                hidden_break_input,
            )?;
            column_track(signal, axis, pieces)
        };
        let workers = target_workers.max(1).min(targets.len());
        return pool
            .map_indexed(workers, targets.len(), make_sig, |sig, index| {
                fit_target(&targets[index], sig)
            })
            .expect("leader target fit thread panicked")
            .into_iter()
            .collect();
    }
    let plan_started = crate::timing::stopwatch();
    let seed_sets = targets
        .iter()
        .map(|target| {
            fit_seed_times(axis, target.t_start, target.t_end, shaped_times)
                .map(|seeds| shaped_breaks.select_seeds(seeds, hidden_break_input, fit_tol))
        })
        .collect::<Result<Vec<_>, _>>()?;
    let mut scale_times = seed_sets.iter().flatten().copied().collect::<Vec<_>>();
    scale_times.sort_by(f64::total_cmp);
    scale_times.dedup();
    let plan_workers = target_workers.max(1).min(scale_times.len());
    let scale_cache = pool
        .map_indexed(plan_workers, scale_times.len(), make_sig, |sig, index| {
            let t = scale_times[index];
            (t.to_bits(), scale_probe(sig, t))
        })
        .expect("leader scale probe thread panicked")
        .into_iter()
        .collect::<HashMap<_, _>>();
    let mut plans = Vec::with_capacity(targets.len());
    for (target, seeds) in targets.iter().zip(seed_sets) {
        let (mut position_scale, mut velocity_scale) = (0.0_f64, 0.0_f64);
        for &t in &seeds {
            let probe = scale_cache
                .get(&t.to_bits())
                .expect("leader scale probe is missing");
            if probe.position_rounding_scale.is_finite() {
                position_scale = position_scale.max(probe.position_rounding_scale);
            }
            if probe.velocity.is_finite() {
                velocity_scale = velocity_scale.max(probe.velocity.abs());
            }
        }
        let time_scale = target.t_start.abs().max(target.t_end.abs());
        let floors = ladder_resolution_floor_from_scales(
            position_scale,
            velocity_scale,
            time_scale,
            fit_tol,
        );
        let resolution = velocity_resolution(fit_tol, floors);
        let velocity_breaks = seeds[1..seeds.len() - 1]
            .iter()
            .map(|&t| VelocityBreak {
                t,
                step: scale_cache[&t.to_bits()].velocity_step,
            })
            .filter(|velocity_break| velocity_break.step.abs() > resolution)
            .collect::<Vec<_>>();
        plans.push(
            fit_plan_from_seeds(seeds, &velocity_breaks, floors, fit_tol.pos_mm).map_err(
                |crowded| {
                    crowded_velocity_breaks_error(axis, fit_context, floors, fit_tol, crowded)
                },
            )?,
        );
    }
    let jobs = plans
        .iter()
        .enumerate()
        .flat_map(|(target_index, plan)| {
            (0..plan.seeds.len() - 1).map(move |span_index| (target_index, span_index))
        })
        .collect::<Vec<_>>();
    let plan_us = plan_started.elapsed_us();
    if crate::timing::is_slow_phase(plan_us) {
        crate::timing::log_slow_phase(
            "leader_plan",
            plan_us,
            crate::timing::PhaseWorkload {
                segments: targets.len(),
                axes: 1,
                pieces: jobs.len(),
                ..Default::default()
            },
            &format!("axis={axis}"),
        );
    }
    let fit_span = |target_index: usize, span_index: usize, sig: &_| {
        let plan = &plans[target_index];
        let t_start = plan.seeds[span_index];
        let t_end = plan.seeds[span_index + 1];
        let mut refinement_splits = 0;
        let mut pieces = Vec::new();
        refine_shaped_span(
            sig,
            RefineSpan {
                t0: t_start,
                t1: t_end,
                slivers: plan.slivers[span_index],
            },
            &SpanFitPolicy {
                axis,
                fit_tol,
                fit_context,
                velocity_budget: f64::INFINITY,
                floors: plan.floors,
                velocity_sign: VelocitySign::Unconstrained,
                seam_continuity,
                requires_hull: true,
            },
            SpanSeeds {
                lower: t_start,
                upper: t_end,
            },
            &mut refinement_splits,
            &mut pieces,
        )
        .map(|()| (pieces, refinement_splits))
    };
    type SpanFit = Result<(Vec<BezierPiece>, usize), PostProcessError>;
    let workers = target_workers.max(1).min(jobs.len());
    let spans_started = crate::timing::stopwatch();
    let fitted: Vec<(usize, usize, SpanFit)> = pool
        .map_indexed(workers, jobs.len(), make_sig, |sig, index| {
            let (target_index, span_index) = jobs[index];
            (
                target_index,
                span_index,
                fit_span(target_index, span_index, sig),
            )
        })
        .expect("leader span fit thread panicked");
    let spans_us = spans_started.elapsed_us();
    if crate::timing::is_slow_phase(spans_us) {
        crate::timing::log_slow_phase(
            "leader_spans",
            spans_us,
            crate::timing::PhaseWorkload {
                segments: targets.len(),
                axes: 1,
                pieces: jobs.len(),
                ..Default::default()
            },
            &format!("axis={axis} workers={workers}"),
        );
    }
    let mut grouped: Vec<Vec<SpanFit>> = (0..targets.len()).map(|_| Vec::new()).collect();
    for (target_index, _, result) in fitted {
        grouped[target_index].push(result);
    }
    let mut tracks = Vec::with_capacity(targets.len());
    for (target_index, results) in grouped.into_iter().enumerate() {
        let mut pieces = Vec::with_capacity(plans[target_index].seeds.len());
        let mut refinement_splits = 0;
        let mut needs_serial_fit = false;
        for result in results {
            match result {
                Ok((mut span_pieces, splits)) => {
                    refinement_splits += splits;
                    pieces.append(&mut span_pieces);
                }
                Err(_) => needs_serial_fit = true,
            }
        }
        let pieces = if needs_serial_fit || refinement_splits > MAX_FIT_REFINEMENT_SPLITS {
            let sig = make_sig();
            fit_leader_target(
                AxisFit {
                    axis,
                    t_start: targets[target_index].t_start,
                    t_end: targets[target_index].t_end,
                    seed_breakpoints: shaped_times,
                    fit_tol,
                    fit_context,
                    seam_continuity,
                },
                &sig,
                shaped_breaks,
                hidden_break_input,
            )?
        } else {
            pieces
        };
        tracks.push(column_track(signal, axis, pieces)?);
    }
    Ok(tracks)
}

/// A fitted column track as the motors run it and, for a leader, as the
/// follower projection reads it. `stored` raises every piece to the
/// track's highest degree on absolute control points, which rounds a
/// piece's jerk to about `n(n−1)(n−2)·ε·|p|/h³`; `read` keeps each piece
/// at its own degree on its own origin, so an advance with no kernel after
/// it, which reads the leaders' jerk, reads it at the piece's precision
/// rather than at the track's distance from the origin. `None` where the
/// stored track is read as it is: a constant, or a motor command no
/// projection reads.
#[derive(Clone)]
pub(crate) struct ColumnTrack {
    pub stored: Arc<nurbs::ScalarNurbs>,
    pub read: Option<ContinuousAxis>,
}

fn column_track(
    signal: ColumnSignal,
    axis: usize,
    pieces: Vec<BezierPiece>,
) -> Result<ColumnTrack, PostProcessError> {
    match signal {
        ColumnSignal::Shaped { .. } => leader_track(axis, pieces),
        ColumnSignal::MotorCommand { .. } => Ok(ColumnTrack {
            stored: Arc::new(finite_track(axis, pieces)?),
            read: None,
        }),
    }
}

pub(crate) fn leader_track(
    axis: usize,
    pieces: Vec<BezierPiece>,
) -> Result<ColumnTrack, PostProcessError> {
    let relative = crate::follower_projection::localize_pieces(0.0, &pieces);
    let stored = finite_track(axis, pieces)?;
    let read = ContinuousAxis::try_piecewise_relative_spline(relative).unwrap_or_else(|error| {
        panic!("shaper: axis {axis}: the leader's own pieces do not join: {error}")
    });
    Ok(ColumnTrack {
        stored: Arc::new(stored),
        read: Some(read),
    })
}

fn finite_track(
    axis: usize,
    pieces: Vec<BezierPiece>,
) -> Result<nurbs::ScalarNurbs, PostProcessError> {
    let t_start = pieces.first().map_or(f64::NAN, |piece| piece.u_start);
    let track = fit_pieces_to_nurbs(axis, pieces)?;
    if !track.control_points().iter().all(|value| value.is_finite()) {
        return Err(PostProcessError::NonFiniteSample { axis, t: t_start });
    }
    Ok(track)
}

type TimedColumn = (
    usize,
    Result<Option<Vec<ColumnTrack>>, PostProcessError>,
    u128,
);

fn fit_columns(
    request: AxisFitRequest,
    targets: &[ContinuousSegment],
    columns: &[FitColumn],
) -> Vec<TimedColumn> {
    if columns.is_empty() {
        return Vec::new();
    }
    let parallel = cfg!(not(target_arch = "wasm32")) && columns.len() > 1 && !targets.is_empty();
    let target_workers = if cfg!(target_arch = "wasm32") || targets.len() < 4 {
        1
    } else {
        std::thread::available_parallelism()
            .map_or(1, |cores| cores.get())
            .div_ceil(columns.len().max(1))
            .min(4)
            .min(targets.len())
    };
    let column_workers = if parallel { columns.len() } else { 1 };
    request
        .pool
        .map_indexed(
            column_workers,
            columns.len(),
            || (),
            |(), index| {
                let column = columns[index];
                let column_started = crate::timing::stopwatch();
                let fitted = fit_axis_column(request, targets, column, target_workers);
                (column.axis, fitted, column_started.elapsed_us())
            },
        )
        .expect("axis fit thread panicked")
}

/// Fits the kerneled leader axes of `fresh` in place, as the motors run
/// them, and returns the same segments with the leaders as the follower
/// projection reads them (see [`ColumnTrack`]).
fn fit_leader_axes(
    request: AxisFitRequest,
    fresh: &mut [ContinuousSegment],
    n_axes: usize,
    work: crate::timing::PhaseWorkload,
) -> Result<Vec<ContinuousSegment>, PostProcessError> {
    let leader_columns = leader_columns(request.chains, n_axes);
    let fit_started = crate::timing::stopwatch();
    let columns = fit_columns(request, fresh, &leader_columns);
    let fit_elapsed_us = fit_started.elapsed_us();
    crate::timing::record_stage(crate::timing::Stage::LeaderFit, fit_elapsed_us);
    if crate::timing::is_slow_phase(fit_elapsed_us) {
        let per_axis: Vec<String> = columns
            .iter()
            .map(|(axis, _, elapsed_us)| format!("axis{axis}={elapsed_us}us"))
            .collect();
        crate::timing::log_slow_phase(
            "leader_fit",
            fit_elapsed_us,
            crate::timing::PhaseWorkload {
                segments: fresh.len(),
                axes: leader_columns.len(),
                force: request.lookahead.clamps_past_signal(),
                ..work
            },
            &per_axis.join(" "),
        );
    }
    let mut read_segments = fresh.to_vec();
    for (axis, column, _) in columns {
        let Some(column) = column? else { continue };
        for ((seg, read_seg), track) in fresh.iter_mut().zip(&mut read_segments).zip(column) {
            let read = track
                .read
                .unwrap_or_else(|| ContinuousAxis::Spline(Arc::clone(&track.stored)));
            Arc::make_mut(&mut read_seg.axes)[axis] = read;
            Arc::make_mut(&mut seg.axes)[axis] = ContinuousAxis::Spline(track.stored);
        }
    }
    Ok(read_segments)
}

/// Every time the window's signal changes polynomial on this axis: each
/// segment's NURBS knots plus the segment edges (a gap between segments is a
/// constant hold, which the edges delimit). The exact convolution integrates
/// between these, so its Gauss rule never straddles a polynomial change.
fn signal_breakpoints(segments: &[&ContinuousSegment], axis: usize) -> Vec<f64> {
    let mut breaks = Vec::new();
    for seg in segments {
        breaks.push(seg.t_start);
        if let ContinuousAxis::Spline(curve) = &seg.axes[axis] {
            breaks.extend_from_slice(curve.knots());
        }
        breaks.push(seg.t_end);
    }
    breaks
}

fn ordered_f64_key(value: f64) -> u64 {
    let bits = value.to_bits();
    if bits >> 63 == 0 {
        bits | (1_u64 << 63)
    } else {
        !bits
    }
}

fn f64_from_ordered_key(key: u64) -> f64 {
    let bits = if key >> 63 == 0 {
        !key
    } else {
        key & !(1_u64 << 63)
    };
    f64::from_bits(bits)
}

/// Sized for the deepest fit-ladder rung (degree 13, 14 coefficients) times
/// the widest kernel moment window the shaper builds.
const MOMENT_POWER_CAPACITY: usize = 24;

/// The signal's `order`-th derivative on one piece, written into `out`;
/// returns its coefficient count, zero once the derivative vanishes.
fn differentiate_into(coefficients: &[f64], order: usize, out: &mut [f64]) -> usize {
    let mut length = coefficients.len();
    out[..length].copy_from_slice(coefficients);
    for _ in 0..order {
        length = differentiate_in_place(out, length);
    }
    length
}

/// One more derivative of the first `length` coefficients of `out`, in
/// place; returns its coefficient count, zero once it vanishes.
fn differentiate_in_place(out: &mut [f64], length: usize) -> usize {
    if length <= 1 {
        return 0;
    }
    for power in 1..length {
        out[power - 1] = out[power] * power as f64;
    }
    length - 1
}

/// `TRANSLATION_BINOMIALS[m][p]` is the weight of a source moment of power
/// `p` in a translated moment of power `m`, built by the same running
/// product a per-term update would compute, so a lookup reproduces it bit
/// for bit without a division in the translation's inner loop.
const TRANSLATION_BINOMIALS: [[f64; MOMENT_POWER_CAPACITY]; MOMENT_POWER_CAPACITY] =
    translation_binomials();

const fn translation_binomials() -> [[f64; MOMENT_POWER_CAPACITY]; MOMENT_POWER_CAPACITY] {
    let mut table = [[0.0; MOMENT_POWER_CAPACITY]; MOMENT_POWER_CAPACITY];
    let mut moment_power = 0;
    while moment_power < MOMENT_POWER_CAPACITY {
        let mut choose = 1.0;
        let mut source_power = 0;
        while source_power <= moment_power {
            table[moment_power][source_power] = choose;
            choose *= (moment_power - source_power) as f64 / (source_power + 1) as f64;
            source_power += 1;
        }
        moment_power += 1;
    }
    table
}

/// `FALLING_FACTORIALS[p][n] = p!/(p−n)!`, the factor the `n`-th derivative
/// of `xᵖ` carries, as the ascending product `(p−n+1)···p` the per-term
/// fold computed, so a lookup reproduces it bit for bit.
pub(crate) const FALLING_FACTORIALS: [[f64; MOMENT_POWER_CAPACITY]; MOMENT_POWER_CAPACITY] =
    falling_factorials();

const fn falling_factorials() -> [[f64; MOMENT_POWER_CAPACITY]; MOMENT_POWER_CAPACITY] {
    let mut table = [[0.0; MOMENT_POWER_CAPACITY]; MOMENT_POWER_CAPACITY];
    let mut power = 0;
    while power < MOMENT_POWER_CAPACITY {
        let mut order = 0;
        while order <= power {
            let mut product = 1.0;
            let mut factor = power - order + 1;
            while factor <= power {
                product *= factor as f64;
                factor += 1;
            }
            table[power][order] = product;
            order += 1;
        }
        power += 1;
    }
    table
}

/// The powers of one origin shift, built once and shared by every moment
/// order translated between the same two origins.
struct MomentTranslation {
    delta_powers: [f64; MOMENT_POWER_CAPACITY],
    len: usize,
}

impl MomentTranslation {
    fn new(source_origin: f64, target_origin: f64, len: usize) -> Self {
        assert!(len <= MOMENT_POWER_CAPACITY);
        let delta = source_origin - target_origin;
        let mut delta_powers = [1.0; MOMENT_POWER_CAPACITY];
        for power in 1..len {
            delta_powers[power] = delta_powers[power - 1] * delta;
        }
        Self { delta_powers, len }
    }

    fn accumulate(&self, source: &[f64], target: &mut [f64]) {
        assert_eq!(source.len(), target.len());
        assert!(target.len() <= self.len);
        match target.len() {
            1 => self.accumulate_fixed::<1>(source, target),
            2 => self.accumulate_fixed::<2>(source, target),
            3 => self.accumulate_fixed::<3>(source, target),
            4 => self.accumulate_fixed::<4>(source, target),
            5 => self.accumulate_fixed::<5>(source, target),
            6 => self.accumulate_fixed::<6>(source, target),
            7 => self.accumulate_fixed::<7>(source, target),
            8 => self.accumulate_fixed::<8>(source, target),
            _ => self.accumulate_terms(source, target),
        }
    }

    fn accumulate_fixed<const LEN: usize>(&self, source: &[f64], target: &mut [f64]) {
        let source: &[f64; LEN] = source.try_into().expect("source moment length");
        let target: &mut [f64; LEN] = target.try_into().expect("target moment length");
        self.accumulate_terms(source, target);
    }

    /// [`Self::accumulate`] for every derivative order at once, each
    /// translation weight `C(m, s)·δ^{m−s}` formed once for all of them:
    /// the same products in the same order as one order at a time.
    fn accumulate_orders(
        &self,
        sources: &[f64],
        stride: usize,
        targets: &mut [&mut [f64]; MOMENT_ORDERS],
    ) {
        let len = targets[0].len();
        assert!(len <= self.len && len <= stride);
        match len {
            1 => self.accumulate_orders_fixed::<1>(sources, stride, targets),
            2 => self.accumulate_orders_fixed::<2>(sources, stride, targets),
            3 => self.accumulate_orders_fixed::<3>(sources, stride, targets),
            4 => self.accumulate_orders_fixed::<4>(sources, stride, targets),
            5 => self.accumulate_orders_fixed::<5>(sources, stride, targets),
            6 => self.accumulate_orders_fixed::<6>(sources, stride, targets),
            7 => self.accumulate_orders_fixed::<7>(sources, stride, targets),
            8 => self.accumulate_orders_fixed::<8>(sources, stride, targets),
            _ => self.accumulate_orders_terms(sources, stride, len, targets),
        }
    }

    fn accumulate_orders_fixed<const LEN: usize>(
        &self,
        sources: &[f64],
        stride: usize,
        targets: &mut [&mut [f64]; MOMENT_ORDERS],
    ) {
        let source = |order: usize| -> &[f64; LEN] {
            sources[order * stride..order * stride + LEN]
                .try_into()
                .expect("source moment length")
        };
        let (sources_p, sources_v, sources_a) = (source(0), source(1), source(2));
        let [position, velocity, acceleration] = targets;
        let position: &mut [f64; LEN] = (&mut position[..LEN]).try_into().expect("target length");
        let velocity: &mut [f64; LEN] = (&mut velocity[..LEN]).try_into().expect("target length");
        let acceleration: &mut [f64; LEN] = (&mut acceleration[..LEN])
            .try_into()
            .expect("target length");
        for moment_power in 0..LEN {
            let binomials = &TRANSLATION_BINOMIALS[moment_power];
            for source_power in 0..=moment_power {
                let weight =
                    binomials[source_power] * self.delta_powers[moment_power - source_power];
                position[moment_power] += weight * sources_p[source_power];
                velocity[moment_power] += weight * sources_v[source_power];
                acceleration[moment_power] += weight * sources_a[source_power];
            }
        }
    }

    fn accumulate_orders_terms(
        &self,
        sources: &[f64],
        stride: usize,
        len: usize,
        targets: &mut [&mut [f64]; MOMENT_ORDERS],
    ) {
        for (order, target) in targets.iter_mut().enumerate() {
            self.accumulate_terms(
                &sources[order * stride..order * stride + len],
                &mut target[..len],
            );
        }
    }

    #[inline]
    fn accumulate_terms(&self, source: &[f64], target: &mut [f64]) {
        for (moment_power, moment) in target.iter_mut().enumerate() {
            let binomials = &TRANSLATION_BINOMIALS[moment_power];
            for source_power in 0..=moment_power {
                *moment += binomials[source_power]
                    * self.delta_powers[moment_power - source_power]
                    * source[source_power];
            }
        }
    }
}

fn taylor_shift(coefficients: &[f64], shift: f64, shifted: &mut [f64]) {
    assert_eq!(coefficients.len(), shifted.len());
    assert!(coefficients.len() <= MOMENT_POWER_CAPACITY);
    let mut work = [0.0; MOMENT_POWER_CAPACITY];
    work[..coefficients.len()].copy_from_slice(coefficients);
    for (lowest, target) in shifted.iter_mut().enumerate() {
        for power in (lowest + 1..coefficients.len()).rev() {
            work[power - 1] = nurbs::fmadd(shift, work[power], work[power - 1]);
        }
        *target = work[lowest];
    }
}

const POWER_DIVISORS: [f64; MOMENT_POWER_CAPACITY] = power_divisors();

const fn power_divisors() -> [f64; MOMENT_POWER_CAPACITY] {
    let mut divisors = [0.0; MOMENT_POWER_CAPACITY];
    let mut power = 0;
    while power < MOMENT_POWER_CAPACITY {
        divisors[power] = power as f64;
        power += 1;
    }
    divisors
}

/// Moments over `[lo, hi]` about `origin` of polynomials re-expanded about
/// the window's midpoint, with everything that depends only on the window —
/// the midpoint shift, the power differences of its edges, the translation
/// back to `origin` — built once for all of a piece's derivative orders.
struct MomentWindow {
    shift: f64,
    edge_power_spans: [f64; MOMENT_POWER_CAPACITY],
    max_power: usize,
    translation: MomentTranslation,
}

impl MomentWindow {
    fn new(
        expansion_point: f64,
        lo: f64,
        hi: f64,
        origin: f64,
        coefficient_len: usize,
        moment_len: usize,
    ) -> Option<Self> {
        if hi <= lo {
            return None;
        }
        let max_power = coefficient_len + moment_len - 1;
        assert!(
            max_power < MOMENT_POWER_CAPACITY,
            "moment product degree {} exceeds capacity {}",
            max_power - 1,
            MOMENT_POWER_CAPACITY - 2
        );
        let base = lo + 0.5 * (hi - lo);
        let x_lo = lo - base;
        let x_hi = hi - base;
        let (mut lo_power, mut hi_power) = (1.0_f64, 1.0_f64);
        let mut edge_power_spans = [0.0; MOMENT_POWER_CAPACITY];
        for span in &mut edge_power_spans[1..=max_power] {
            lo_power *= x_lo;
            hi_power *= x_hi;
            *span = hi_power - lo_power;
        }
        Some(Self {
            shift: base - expansion_point,
            edge_power_spans,
            max_power,
            translation: MomentTranslation::new(base, origin, moment_len),
        })
    }

    /// `coefficients`, given about `expansion_point`, re-expanded about the
    /// window's midpoint into `centered`.
    fn center(&self, coefficients: &[f64], centered: &mut [f64]) {
        taylor_shift(coefficients, self.shift, centered);
    }

    /// Adds the moments of a polynomial whose coefficients are already
    /// about the window's midpoint.
    fn accumulate_centered(&self, centered: &[f64], moments: &mut [f64]) {
        assert_eq!(moments.len(), self.translation.len);
        assert!(centered.len() + moments.len() - 1 <= self.max_power);
        match moments.len() {
            1 => self.accumulate_centered_fixed::<1>(centered, moments),
            2 => self.accumulate_centered_fixed::<2>(centered, moments),
            3 => self.accumulate_centered_fixed::<3>(centered, moments),
            4 => self.accumulate_centered_fixed::<4>(centered, moments),
            5 => self.accumulate_centered_fixed::<5>(centered, moments),
            6 => self.accumulate_centered_fixed::<6>(centered, moments),
            7 => self.accumulate_centered_fixed::<7>(centered, moments),
            8 => self.accumulate_centered_fixed::<8>(centered, moments),
            _ => {
                let mut local_moments = [0.0; MOMENT_POWER_CAPACITY];
                let local_moments = &mut local_moments[..moments.len()];
                self.integrate_centered(centered, local_moments);
                self.translation.accumulate_terms(local_moments, moments);
            }
        }
    }

    fn accumulate_centered_fixed<const LEN: usize>(&self, centered: &[f64], moments: &mut [f64]) {
        let mut local_moments = [0.0; LEN];
        self.integrate_centered(centered, &mut local_moments);
        self.translation
            .accumulate_fixed::<LEN>(&local_moments, moments);
    }

    #[inline]
    fn integrate_centered(&self, centered: &[f64], local_moments: &mut [f64]) {
        let moment_len = local_moments.len();
        for (polynomial_power, &coefficient) in centered.iter().enumerate() {
            let powers = polynomial_power + 1..polynomial_power + 1 + moment_len;
            for ((moment, &span), &divisor) in local_moments
                .iter_mut()
                .zip(&self.edge_power_spans[powers.clone()])
                .zip(&POWER_DIVISORS[powers])
            {
                *moment += coefficient * span / divisor;
            }
        }
    }
}

/// Segment-tree range sums of every piece's moments, for all
/// `MOMENT_ORDERS` derivative orders at once: each node stores its orders
/// contiguously, so one traversal translates every order with one set of
/// origin-shift powers. Beside them each node bounds the magnitude of the
/// terms its moments were summed from: `masses` bounds `∫|x⁽ⁿ⁾|` as it was
/// evaluated, and `reaches` how far, through every translation below the
/// node, a point of it lies from its origin, so the terms of its `q`-th
/// moment sum to at most `mass·reach^q`.
struct MomentTree {
    degree: usize,
    leaf_count: usize,
    origins: Vec<f64>,
    moments: Vec<f64>,
    reaches: Vec<f64>,
    masses: Vec<f64>,
}

impl MomentTree {
    fn build(table: &AxisSignalTable, degree: usize) -> Self {
        let stride = degree + 1;
        assert!(
            stride <= MOMENT_POWER_CAPACITY,
            "moment degree {degree} exceeds capacity {}",
            MOMENT_POWER_CAPACITY - 1
        );
        let node_stride = MOMENT_ORDERS * stride;
        let leaf_count = table.coeffs.len().next_power_of_two();
        let node_count = 2 * leaf_count;
        let mut origins = vec![f64::NAN; node_count];
        let mut moments = vec![0.0; node_count * node_stride];
        let mut reaches = vec![0.0; node_count];
        let mut masses = vec![0.0; node_count * MOMENT_ORDERS];
        for i in 0..table.coeffs.len() {
            let node = leaf_count + i;
            let origin = table.origins[i];
            origins[node] = origin;
            reaches[node] = table.half_width(i);
            for (order, mass) in masses[node * MOMENT_ORDERS..(node + 1) * MOMENT_ORDERS]
                .iter_mut()
                .enumerate()
            {
                *mass = (table.ends[i] - table.starts[i]) * table.piece_magnitude(i, order);
            }
            let node_moments = &mut moments[node * node_stride..(node + 1) * node_stride];
            table.accumulate_piece_moments(
                i,
                table.starts[i],
                table.ends[i],
                origin,
                stride,
                node_moments.chunks_exact_mut(stride),
            );
        }
        for node in (1..leaf_count).rev() {
            let left = 2 * node;
            let right = left + 1;
            let origin = match (origins[left].is_finite(), origins[right].is_finite()) {
                (true, true) => 0.5 * (origins[left] + origins[right]),
                (true, false) => origins[left],
                (false, true) => origins[right],
                (false, false) => continue,
            };
            origins[node] = origin;
            let mut combined = [0.0; MOMENT_ORDERS * MOMENT_POWER_CAPACITY];
            for child in [left, right] {
                if origins[child].is_finite() {
                    let translation = MomentTranslation::new(origins[child], origin, stride);
                    let child_moments = &moments[child * node_stride..(child + 1) * node_stride];
                    for (source, target) in child_moments
                        .chunks_exact(stride)
                        .zip(combined[..node_stride].chunks_exact_mut(stride))
                    {
                        translation.accumulate(source, target);
                    }
                    reaches[node] =
                        reaches[node].max((origins[child] - origin).abs() + reaches[child]);
                    for order in 0..MOMENT_ORDERS {
                        masses[node * MOMENT_ORDERS + order] +=
                            masses[child * MOMENT_ORDERS + order];
                    }
                }
            }
            moments[node * node_stride..(node + 1) * node_stride]
                .copy_from_slice(&combined[..node_stride]);
        }
        Self {
            degree,
            leaf_count,
            origins,
            moments,
            reaches,
            masses,
        }
    }

    /// Adds every node's moments over `[start, end)` about `origin`, and
    /// with `bounds` each node's bound on the terms of its moments.
    fn accumulate(
        &self,
        start: usize,
        end: usize,
        origin: f64,
        targets: &mut [&mut [f64]; MOMENT_ORDERS],
        mut bounds: Option<&mut MomentBounds>,
    ) {
        if start >= end {
            return;
        }
        let moment_len = targets[0].len();
        let mut visit = |node: usize, targets: &mut [&mut [f64]; MOMENT_ORDERS]| {
            self.accumulate_node(node, origin, targets);
            if let Some(bounds) = bounds.as_deref_mut() {
                self.accumulate_node_magnitudes(node, origin, bounds, moment_len);
            }
        };
        let mut left = start + self.leaf_count;
        let mut right = end + self.leaf_count;
        let mut right_nodes = [0usize; usize::BITS as usize];
        let mut right_count = 0;
        while left < right {
            if left & 1 != 0 {
                visit(left, targets);
                left += 1;
            }
            if right & 1 != 0 {
                right -= 1;
                right_nodes[right_count] = right;
                right_count += 1;
            }
            left /= 2;
            right /= 2;
        }
        while right_count > 0 {
            right_count -= 1;
            visit(right_nodes[right_count], targets);
        }
    }

    fn accumulate_node(&self, node: usize, origin: f64, targets: &mut [&mut [f64]; MOMENT_ORDERS]) {
        let stride = self.degree + 1;
        let node_moments =
            &self.moments[node * MOMENT_ORDERS * stride..(node + 1) * MOMENT_ORDERS * stride];
        let moment_len = targets[0].len();
        MomentTranslation::new(self.origins[node], origin, moment_len).accumulate_orders(
            node_moments,
            stride,
            targets,
        );
    }
}

fn piece_midpoint(start: f64, end: f64) -> f64 {
    start + 0.5 * (end - start)
}

/// One axis of the emit window, flattened to time-sorted monomial pieces so
/// the convolution's Gauss sampling evaluates by binary search plus Horner
/// instead of a de Boor pass per sample. Each piece is expanded about its
/// midpoint, where a high-degree piece's monomial terms stay at the scale of
/// its values: about an edge they swing by orders of magnitude more and
/// cancel, and every moment and exact polynomial summed from them carries
/// that swing in its rounding. Semantics match the NURBS window it
/// is built from: before `first_t` the signal clamps at a stream boundary
/// and is otherwise missing (NaN, surfaced as `MissingHistory`); past
/// `last_t` it clamps under a drain flush and is otherwise missing; a time
/// gap between segments holds the value the preceding piece ends at.
pub(crate) struct AxisSignalTable {
    starts: Vec<f64>,
    ends: Vec<f64>,
    origins: Vec<f64>,
    coeffs: Vec<Vec<f64>>,
    first_t: f64,
    last_t: f64,
    boundary: FitBoundary,
    lookahead: Lookahead,
    moment_tree: Option<MomentTree>,
    input_jumps: Vec<(f64, f64, f64)>,
    entering_step: f64,
    leaving_step: f64,
    gain_impulses: Vec<GainImpulse>,
    boundaries: Vec<f64>,
}

/// The time extent of a flattened signal window and how its two edges
/// behave when the convolution reads past them.
#[derive(Clone, Copy)]
pub(crate) struct SignalBounds {
    pub first_t: f64,
    pub last_t: f64,
    pub boundary: FitBoundary,
    pub lookahead: Lookahead,
}

const MOMENT_ORDERS: usize = 3;

/// For each derivative order and moment power, a bound on the magnitude of
/// the terms that moment is summed from.
type MomentBounds = [[f64; MOMENT_POWER_CAPACITY]; MOMENT_ORDERS];

impl AxisSignalTable {
    #[cfg(test)]
    pub(crate) fn from_tracks<'a>(
        tracks: impl IntoIterator<Item = &'a nurbs::ScalarNurbs>,
        bounds: SignalBounds,
    ) -> Self {
        let pieces: Vec<BezierPiece> = tracks.into_iter().flat_map(extract_bezier_pieces).collect();
        Self::from_pieces(&pieces, bounds)
    }

    /// The table over monomial pieces in time order, each in its own
    /// variable from its start.
    pub(crate) fn from_pieces<'p>(
        pieces: impl IntoIterator<Item = &'p BezierPiece>,
        bounds: SignalBounds,
    ) -> Self {
        let mut table = Self {
            starts: Vec::new(),
            ends: Vec::new(),
            origins: Vec::new(),
            coeffs: Vec::new(),
            first_t: bounds.first_t,
            last_t: bounds.last_t,
            boundary: bounds.boundary,
            lookahead: bounds.lookahead,
            moment_tree: None,
            input_jumps: Vec::new(),
            entering_step: 0.0,
            leaving_step: 0.0,
            gain_impulses: Vec::new(),
            boundaries: Vec::new(),
        };
        for piece in pieces {
            if let Some(previous_end) = table.ends.last().copied() {
                assert!(
                    piece.u_start >= previous_end,
                    "signal pieces overlap: next starts at {} before previous end {previous_end}",
                    piece.u_start
                );
                if piece.u_start > previous_end {
                    let previous = table.coeffs.len() - 1;
                    let held = table.piece_at(previous, previous_end);
                    table.push_piece(previous_end, piece.u_start, vec![held]);
                }
            }
            let origin = piece_midpoint(piece.u_start, piece.u_end);
            let mut centered = vec![0.0; piece.coeffs.len()];
            taylor_shift(&piece.coeffs, origin - piece.u_start, &mut centered);
            table.push_piece(piece.u_start, piece.u_end, centered);
        }
        assert!(!table.coeffs.is_empty(), "empty signal window");
        table
    }

    pub(crate) fn offset_by(mut self, offset: f64) -> Self {
        for coefficients in &mut self.coeffs {
            coefficients[0] -= offset;
        }
        self
    }

    pub(crate) fn with_derivative_gains(mut self, k1: f64, k2: f64) -> Self {
        assert!(
            self.moment_tree.is_none(),
            "derivative gains must reshape the signal before its moments are built"
        );
        let last = self.coeffs.len() - 1;
        let held_before = self.piece_at(0, self.first_t);
        let held_after = self.piece_at(last, self.last_t);
        self.gain_impulses = self.input_step_impulses(k1, k2);
        for coefficients in &mut self.coeffs {
            for power in 0..coefficients.len() {
                let slope = coefficients
                    .get(power + 1)
                    .map_or(0.0, |&c| c * (power + 1) as f64);
                let curvature = coefficients
                    .get(power + 2)
                    .map_or(0.0, |&c| c * ((power + 1) * (power + 2)) as f64);
                coefficients[power] += k1 * slope + k2 * curvature;
            }
        }
        self.entering_step = self.piece_at(0, self.first_t) - held_before;
        self.leaving_step = held_after - self.piece_at(last, self.last_t);
        self
    }

    fn input_step_impulses(&self, k1: f64, k2: f64) -> Vec<GainImpulse> {
        let last = self.coeffs.len() - 1;
        let seams = (1..=last).map(|piece| {
            let seam = self.starts[piece];
            (
                seam,
                self.piece_at(piece, seam) - self.piece_at(piece - 1, seam),
                self.slope_at(piece, seam) - self.slope_at(piece - 1, seam),
            )
        });
        std::iter::once((self.first_t, 0.0, self.slope_at(0, self.first_t)))
            .chain(seams)
            .chain(std::iter::once((
                self.last_t,
                0.0,
                -self.slope_at(last, self.last_t),
            )))
            .map(|(at, position_step, slope_step)| GainImpulse {
                at,
                delta: k1 * position_step + k2 * slope_step,
                doublet: k2 * position_step,
            })
            .filter(|impulse| impulse.delta != 0.0 || impulse.doublet != 0.0)
            .collect()
    }

    pub(crate) fn gain_impulses(&self) -> &[GainImpulse] {
        &self.gain_impulses
    }

    fn held_before(&self) -> f64 {
        self.piece_at(0, self.first_t) - self.entering_step
    }

    fn held_after(&self) -> f64 {
        self.piece_at(self.coeffs.len() - 1, self.last_t) + self.leaving_step
    }

    pub(crate) fn max_degree(&self) -> usize {
        self.coeffs
            .iter()
            .map(|coefficients| coefficients.len() - 1)
            .max()
            .expect("empty signal window")
    }
    pub(crate) fn with_piece_moments(mut self, degree: usize) -> Self {
        let tree = MomentTree::build(&self, degree);
        let input_jumps = self.collect_input_jumps();
        self.moment_tree = Some(tree);
        self.input_jumps = input_jumps;
        let mut boundaries = self.starts.clone();
        boundaries.extend([
            *self.ends.last().expect("empty signal window"),
            self.first_t,
            self.last_t,
        ]);
        boundaries.sort_by(f64::total_cmp);
        boundaries.dedup();
        self.boundaries = boundaries;
        self
    }

    pub(crate) fn input_jumps(&self) -> &[(f64, f64, f64)] {
        &self.input_jumps
    }

    fn push_piece(&mut self, start: f64, end: f64, centered: Vec<f64>) {
        self.starts.push(start);
        self.ends.push(end);
        self.origins.push(piece_midpoint(start, end));
        self.coeffs.push(centered);
    }

    fn half_width(&self, piece: usize) -> f64 {
        (self.origins[piece] - self.starts[piece]).max(self.ends[piece] - self.origins[piece])
    }

    /// Piece `piece`'s offset from its origin at `t`, clamped onto the piece.
    fn offset_in(&self, piece: usize, t: f64) -> f64 {
        t.clamp(self.starts[piece], self.ends[piece]) - self.origins[piece]
    }

    /// `Σₖ |cₖ| k!/(k−n)! (h/2)ᵏ⁻ⁿ` for piece `piece`'s `order`-th
    /// derivative: a bound, at any point of the piece, on the terms its
    /// re-expansion about any point of the piece sums.
    fn piece_magnitude(&self, piece: usize, order: usize) -> f64 {
        let half_width = self.half_width(piece);
        let mut magnitude = 0.0;
        let mut reach = 1.0;
        for power in order..self.coeffs[piece].len() {
            let falling = FALLING_FACTORIALS[power][order];
            magnitude += self.coeffs[piece][power].abs() * falling * reach;
            reach *= half_width;
        }
        magnitude
    }

    fn slope_at(&self, piece: usize, at: f64) -> f64 {
        let mut derivative = [0.0; MOMENT_POWER_CAPACITY];
        let length = differentiate_into(&self.coeffs[piece], 1, &mut derivative);
        if length == 0 {
            return 0.0;
        }
        let tau = self.offset_in(piece, at);
        derivative[..length]
            .iter()
            .rev()
            .fold(0.0_f64, |acc, &c| nurbs::fmadd(acc, tau, c))
    }

    fn collect_input_jumps(&self) -> Vec<(f64, f64, f64)> {
        let mut jumps = Vec::new();
        let entering = self.slope_at(0, self.first_t);
        if entering != 0.0 || self.entering_step != 0.0 {
            jumps.push((self.first_t, self.entering_step, entering));
        }
        for piece in 1..self.coeffs.len() {
            let seam = self.starts[piece];
            let position_jump = self.piece_at(piece, seam) - self.piece_at(piece - 1, seam);
            let slope_jump = self.slope_at(piece, seam) - self.slope_at(piece - 1, seam);
            if position_jump != 0.0 || slope_jump != 0.0 {
                jumps.push((seam, position_jump, slope_jump));
            }
        }
        let leaving = self.slope_at(self.coeffs.len() - 1, self.last_t);
        if leaving != 0.0 || self.leaving_step != 0.0 {
            jumps.push((self.last_t, self.leaving_step, -leaving));
        }
        jumps
    }
    pub(crate) fn integrate_moments(
        &self,
        lo: f64,
        hi: f64,
        degree: usize,
        origin: f64,
        orders: [&mut [f64]; MOMENT_ORDERS],
    ) -> bool {
        self.integrate_bounded_moments(lo, hi, degree, origin, orders, None)
    }

    /// `integrate_moments`, and with `bounds` the magnitude of the terms
    /// each moment sums, gathered in the same walk: every partial piece,
    /// hold and tree node adds its mass times `(|δ| + reach)^q`.
    fn integrate_bounded_moments(
        &self,
        lo: f64,
        hi: f64,
        degree: usize,
        origin: f64,
        mut orders: [&mut [f64]; MOMENT_ORDERS],
        mut bounds: Option<&mut MomentBounds>,
    ) -> bool {
        let tree = self
            .moment_tree
            .as_ref()
            .expect("moment integration requested before moments were built");
        assert!(
            degree <= tree.degree,
            "requested moment degree {degree} exceeds prepared degree {}",
            tree.degree
        );
        if !lo.is_finite() || !hi.is_finite() || !origin.is_finite() || lo > hi {
            return false;
        }
        if (lo < self.first_t && !self.boundary.clamps_before_signal())
            || (hi > self.last_t && !self.lookahead.clamps_past_signal())
        {
            return false;
        }
        let moment_len = degree + 1;
        for moments in &mut orders {
            assert_eq!(moments.len(), moment_len);
            moments.fill(0.0);
        }
        if lo < self.first_t {
            let to = hi.min(self.first_t);
            Self::accumulate_held(self.held_before(), lo, to, origin, orders[0]);
            if let Some(bounds) = bounds.as_deref_mut() {
                Self::bound_held(
                    self.held_before(),
                    (lo, to),
                    origin,
                    &mut bounds[0][..moment_len],
                );
            }
        }
        let interior_lo = lo.max(self.first_t);
        let interior_hi = hi.min(self.last_t);
        if interior_hi > interior_lo {
            let first_piece = self.ends.partition_point(|&end| end <= interior_lo);
            let end_piece = self.starts.partition_point(|&start| start < interior_hi);
            let mut full_start = first_piece;
            let mut full_end = end_piece;
            if self.starts[first_piece] < interior_lo {
                full_start += 1;
                let to = interior_hi.min(self.ends[first_piece]);
                self.accumulate_piece_moments(
                    first_piece,
                    interior_lo,
                    to,
                    origin,
                    moment_len,
                    orders.iter_mut().map(|moments| &mut **moments),
                );
                if let Some(bounds) = bounds.as_deref_mut() {
                    self.bound_partial_piece(
                        first_piece,
                        (interior_lo, to),
                        origin,
                        bounds,
                        moment_len,
                    );
                }
            }
            let right_partial = full_start < full_end && self.ends[full_end - 1] > interior_hi;
            if right_partial {
                full_end -= 1;
            }
            tree.accumulate(
                full_start,
                full_end,
                origin,
                &mut orders,
                bounds.as_deref_mut(),
            );
            if right_partial {
                let from = self.starts[full_end];
                self.accumulate_piece_moments(
                    full_end,
                    from,
                    interior_hi,
                    origin,
                    moment_len,
                    orders.iter_mut().map(|moments| &mut **moments),
                );
                if let Some(bounds) = bounds.as_deref_mut() {
                    self.bound_partial_piece(
                        full_end,
                        (from, interior_hi),
                        origin,
                        bounds,
                        moment_len,
                    );
                }
            }
        }
        if hi > self.last_t {
            let from = lo.max(self.last_t);
            Self::accumulate_held(self.held_after(), from, hi, origin, orders[0]);
            if let Some(bounds) = bounds {
                Self::bound_held(
                    self.held_after(),
                    (from, hi),
                    origin,
                    &mut bounds[0][..moment_len],
                );
            }
        }
        true
    }

    fn accumulate_held(value: f64, lo: f64, hi: f64, origin: f64, moments: &mut [f64]) {
        if let Some(window) = MomentWindow::new(lo, lo, hi, origin, 1, moments.len()) {
            window.accumulate_centered(&[value], moments);
        }
    }

    /// Adds piece `piece`'s moments over `[lo, hi]` about `origin` to each
    /// target in turn, the `order`-th target receiving the piece's
    /// `order`-th derivative. The piece is re-expanded about the window
    /// once; each derivative is taken of that expansion.
    fn accumulate_piece_moments<'m>(
        &self,
        piece: usize,
        lo: f64,
        hi: f64,
        origin: f64,
        moment_len: usize,
        orders: impl Iterator<Item = &'m mut [f64]>,
    ) {
        let coefficients = &self.coeffs[piece];
        let Some(window) = MomentWindow::new(
            self.origins[piece],
            lo,
            hi,
            origin,
            coefficients.len(),
            moment_len,
        ) else {
            return;
        };
        let mut centered = [0.0; MOMENT_POWER_CAPACITY];
        window.center(coefficients, &mut centered[..coefficients.len()]);
        let mut length = coefficients.len();
        for (order, moments) in orders.enumerate() {
            if order > 0 {
                length = differentiate_in_place(&mut centered, length);
            }
            if length > 0 {
                window.accumulate_centered(&centered[..length], moments);
            }
        }
    }

    fn piece_at(&self, i: usize, t: f64) -> f64 {
        let tau = self.offset_in(i, t);
        self.coeffs[i]
            .iter()
            .rev()
            .fold(0.0_f64, |acc, &c| nurbs::fmadd(acc, tau, c))
    }

    /// Every Gauss node of one convolution cut window lands in the same
    /// piece - `merge_cuts` puts a cut at every input break - so one sampler
    /// walking a window resolves the piece once and pays two comparisons for
    /// the rest. The hint belongs to the sampler, not the table: the table is
    /// shared by every worker and a hint inside it would trade the search for
    /// cache-line contention.
    pub(crate) fn eval_hinted(&self, t: f64, hint: &std::cell::Cell<usize>) -> f64 {
        if t < self.first_t {
            if !self.boundary.clamps_before_signal() {
                return f64::NAN;
            }
            return self.held_before();
        }
        if t > self.last_t {
            if !self.lookahead.clamps_past_signal() {
                return f64::NAN;
            }
            return self.held_after();
        }
        let i = self.piece_covering(t, hint);
        if t >= self.starts[i] - SEGMENT_TIME_EPS_S {
            return self.piece_at(i, t);
        }
        if i > 0 {
            return self.piece_at(i - 1, self.ends[i - 1]);
        }
        f64::NAN
    }

    fn piece_covering(&self, t: f64, hint: &std::cell::Cell<usize>) -> usize {
        let at = hint.get();
        if at < self.coeffs.len() && t >= self.starts[at] && t <= self.ends[at] {
            return at;
        }
        let found = self
            .ends
            .partition_point(|&end| end < t)
            .min(self.coeffs.len() - 1);
        hint.set(found);
        found
    }
}

/// A time gap in the signal is evaluated as the position held at the
/// preceding rest, which is only sound if both sides of the gap agree on that
/// position.
fn assert_gap_is_a_hold(prev: &ContinuousSegment, next: &ContinuousSegment) {
    const GAP_HOLD_EPS_MM: f64 = 1e-6;
    if next.t_start - prev.t_end <= SEGMENT_TIME_EPS_S {
        return;
    }
    for axis in 0..prev.axes.len() {
        let held = eval_segment_axis(prev, axis, prev.t_end);
        let resumed = eval_segment_axis(next, axis, next.t_start);
        assert!(
            (held - resumed).abs() <= GAP_HOLD_EPS_MM,
            "shaper: axis {axis} moved across a signal time gap \
             [{:.9}, {:.9}]: held {held} vs resumed {resumed}",
            prev.t_end,
            next.t_start,
        );
    }
}

fn eval_segment_axis(seg: &ContinuousSegment, axis: usize, t: f64) -> f64 {
    seg.eval_axis(axis, t)
        .unwrap_or_else(|error| panic!("shaper: {error}"))
        .position
}

const MAX_FIT_REFINEMENT_SPLITS: usize = 512;
const LEADER_SPAN_PARALLEL_THRESHOLD: usize = 512;

/// A span's acceleration is read off a quintic whose odd coefficients are
/// solved from the endpoint positions, so one ulp of sampling noise reaches
/// the fit as `LADDER_HIGH_DEGREE_ACCEL_NOISE_GAIN · ulp / h²` of
/// acceleration. Sampling noise is an ulp of the position itself plus what an
/// ulp of the sample time moves it at the local velocity. Below the span
/// where that amplification reaches the acceleration budget no high-degree
/// rung can pass and bisecting quadruples it, so the shortest span worth a
/// high-degree rung — and the shortest knot span `bezier_pieces_to_nurbs` may
/// be handed — is the one that still leaves the budget to the signal.
const LADDER_HIGH_DEGREE_ACCEL_NOISE_GAIN: f64 = 32.0;

fn stored_acceleration_noise(position_scale: f64, span_s: f64) -> f64 {
    let control_point_rounding = 0.5 * f64::EPSILON * position_scale;
    let second_difference_rounding = 4.0 * control_point_rounding;
    let degree = LADDER_MAX_DEGREE as f64;
    degree * (degree - 1.0) * second_difference_rounding / (span_s * span_s)
}

fn stored_acceleration_floor(position_scale: f64, fit_tol: FitTol) -> f64 {
    let storage_share_of_accel_budget = 0.5 * fit_tol.accel_mm_s2;
    (stored_acceleration_noise(position_scale, 1.0) / storage_share_of_accel_budget).sqrt()
}

#[derive(Debug, Clone, Copy)]
struct LadderFloors {
    high_degree: f64,
    /// A cubic never touches endpoint acceleration: sampling noise reaches
    /// its acceleration only through `c3 = ¼(h(v0+v1)/2 − Δp)`, and `Δp` is
    /// the signal's own endpoint delta rather than a difference of absolute
    /// samples, so its error tracks the span rather than the track's offset
    /// from the origin. There is still no cubic noise floor worth computing:
    /// cubic refinement stops where the span stops being representable, and
    /// the probe checks in `candidate_ok` are what reject a cubic that did
    /// amplify.
    cubic: f64,
}

fn ladder_resolution_floor<S: TrackSignal>(
    sig: &S,
    seeds: &[f64],
    time_scale: f64,
    fit_tol: FitTol,
) -> LadderFloors {
    let (mut position_scale, mut velocity_scale) = (0.0_f64, 0.0_f64);
    for &t in seeds {
        let (position, velocity, _) = sig.eval_pva(t);
        if position.is_finite() {
            position_scale = position_scale.max(sig.position_rounding_scale(position));
        }
        if velocity.is_finite() {
            velocity_scale = velocity_scale.max(velocity.abs());
        }
    }
    ladder_resolution_floor_from_scales(position_scale, velocity_scale, time_scale, fit_tol)
}

fn ladder_resolution_floor_from_scales(
    position_scale: f64,
    velocity_scale: f64,
    time_scale: f64,
    fit_tol: FitTol,
) -> LadderFloors {
    let sample_noise = f64::EPSILON * nurbs::fmadd(time_scale, velocity_scale, position_scale);
    let representable = MIN_DEVICE_INTERVAL_S
        .max(8.0 * f64::EPSILON * time_scale)
        .max(stored_acceleration_floor(position_scale, fit_tol));
    LadderFloors {
        high_degree: representable
            .max((LADDER_HIGH_DEGREE_ACCEL_NOISE_GAIN * sample_noise / fit_tol.accel_mm_s2).sqrt()),
        cubic: representable,
    }
}

/// Two seed times closer than the refit's `minimum_span` bound no span the
/// ladder can fit: `bezier_pieces_to_nurbs` would turn them into an ulp-wide
/// knot span, and de Boor divides by knot spacing, so the *neighbouring*
/// full-width span evaluates to garbage. Exact convolution cut transitions
/// land an ulp away from segment edges by construction, so the collision is
/// routine rather than exceptional. A run of seeds each closer than the floor
/// to the last kept one rolls forward onto its rightmost member, so what it
/// absorbs can be many floors wide. It may roll onto a velocity break but
/// never past one, and never past the pinned `t_start`: those anchor it.
///
/// A velocity break within the floor of an anchor is moved onto it, one knot
/// carrying both, and the span beyond reads its edge velocity past the moved
/// break. Moving a break by `d` shifts position by its velocity step times
/// `d`; within `position_tolerance` that is the knot the signal would have
/// had at representable resolution, beyond it the breaks are an error of
/// whatever laid them.
///
/// The absorbed seeds may still be acceleration breaks of the signal, so the
/// plan records, per kept span, how far in from each edge they reach: the
/// ladder must read that edge's state past them.
fn coalesce_degenerate_seeds(
    seeds: &mut Vec<f64>,
    velocity_breaks: &[VelocityBreak],
    minimum_span: f64,
    position_tolerance: f64,
) -> Result<Vec<EdgeSlivers>, CrowdedVelocityBreaks> {
    assert!(
        velocity_breaks.is_sorted_by(|left, right| left.t <= right.t),
        "velocity breaks are not sorted: {velocity_breaks:?}"
    );
    let step_at = |t: f64| {
        velocity_breaks
            .binary_search_by(|probe| probe.t.total_cmp(&t))
            .ok()
            .map(|index| velocity_breaks[index].step.abs())
    };
    let moved = |break_at: f64, moved_onto: f64, velocity_step: f64| {
        let position_shift = velocity_step * (break_at - moved_onto).abs();
        if position_shift > position_tolerance {
            Err(CrowdedVelocityBreaks {
                break_at,
                moved_onto,
                velocity_step,
                position_shift,
            })
        } else {
            Ok(())
        }
    };
    let t_end = *seeds.last().expect("fit seeds are empty");
    let mut kept = Vec::with_capacity(seeds.len());
    kept.push(KeptSeed::pinned(seeds[0]));
    for &seed in &seeds[1..seeds.len() - 1] {
        let step = step_at(seed);
        let last = kept.last_mut().expect("kept seeds are empty");
        let gap = seed - last.t;
        if gap >= minimum_span {
            kept.push(KeptSeed::interior(seed, step));
        } else if last.anchors() {
            if let Some(step) = step {
                moved(seed, last.t, step)?;
                last.break_after = Some(seed);
                last.carried_steps += step;
            }
            last.absorbed_after = last.absorbed_after.max(gap);
        } else {
            last.absorbed_before += gap;
            last.t = seed;
            last.velocity_break = step.is_some();
            last.carried_steps = step.unwrap_or(0.0);
        }
    }
    let mut absorbed_end = 0.0_f64;
    let mut end_break = None;
    while kept.len() > 1 {
        let last = kept.last().expect("kept seeds are empty");
        let gap = t_end - last.t;
        if gap >= minimum_span {
            break;
        }
        if last.carried_steps > 0.0 {
            moved(last.t, t_end, last.carried_steps)?;
            end_break = Some(last.t);
        }
        absorbed_end = absorbed_end.max(gap + last.absorbed_before);
        kept.pop();
    }
    let mut end = KeptSeed::pinned(t_end);
    end.absorbed_before = absorbed_end;
    end.break_before = end_break;
    kept.push(end);
    *seeds = kept.iter().map(|seed| seed.t).collect();
    Ok(kept
        .windows(2)
        .map(|span| EdgeSlivers {
            start: span[0].absorbed_after,
            end: span[1].absorbed_before,
            start_break: span[0].break_after,
            end_break: span[1].break_before,
        })
        .collect())
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct VelocityBreak {
    t: f64,
    step: f64,
}

struct KeptSeed {
    t: f64,
    pinned: bool,
    velocity_break: bool,
    carried_steps: f64,
    absorbed_before: f64,
    absorbed_after: f64,
    break_before: Option<f64>,
    break_after: Option<f64>,
}

impl KeptSeed {
    fn pinned(t: f64) -> Self {
        Self {
            pinned: true,
            ..Self::interior(t, None)
        }
    }

    fn interior(t: f64, step: Option<f64>) -> Self {
        Self {
            t,
            pinned: false,
            velocity_break: step.is_some(),
            carried_steps: step.unwrap_or(0.0),
            absorbed_before: 0.0,
            absorbed_after: 0.0,
            break_before: None,
            break_after: None,
        }
    }

    fn anchors(&self) -> bool {
        self.pinned || self.velocity_break
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct CrowdedVelocityBreaks {
    break_at: f64,
    moved_onto: f64,
    velocity_step: f64,
    position_shift: f64,
}

/// How far into a span, from each edge, seeds absorbed by coalescing reach,
/// and the innermost velocity break among them, past which the edge's
/// velocity is read.
#[derive(Clone, Copy, Default, Debug, PartialEq)]
struct EdgeSlivers {
    start: f64,
    end: f64,
    start_break: Option<f64>,
    end_break: Option<f64>,
}

struct FitPlan {
    seeds: Vec<f64>,
    slivers: Vec<EdgeSlivers>,
    floors: LadderFloors,
}

pub(crate) fn fit_seed_times(
    axis: usize,
    t_start: f64,
    t_end: f64,
    seed_breakpoints: &[f64],
) -> Result<Vec<f64>, PostProcessError> {
    if !t_start.is_finite() || !t_end.is_finite() || t_end <= t_start {
        return Err(PostProcessError::DegenerateAxisTrack { axis });
    }
    let mut seeds = Vec::with_capacity(seed_breakpoints.len() + 2);
    seeds.push(t_start);
    seeds.extend(
        seed_breakpoints
            .iter()
            .copied()
            .filter(|time| *time > t_start && *time < t_end && time.is_finite()),
    );
    seeds.push(t_end);
    seeds.sort_by(f64::total_cmp);
    seeds.dedup();
    *seeds.first_mut().expect("fit seeds are empty") = t_start;
    if seeds.len() == 1 {
        seeds.push(t_end);
    } else {
        *seeds.last_mut().expect("fit seeds are empty") = t_end;
    }
    Ok(seeds)
}

fn fit_plan_from_seeds(
    mut seeds: Vec<f64>,
    velocity_breaks: &[VelocityBreak],
    floors: LadderFloors,
    position_tolerance: f64,
) -> Result<FitPlan, CrowdedVelocityBreaks> {
    let slivers = coalesce_degenerate_seeds(
        &mut seeds,
        velocity_breaks,
        floors.cubic,
        position_tolerance,
    )?;
    Ok(FitPlan {
        seeds,
        slivers,
        floors,
    })
}

fn crowded_velocity_breaks_error(
    axis: usize,
    fit_context: &'static str,
    floors: LadderFloors,
    fit_tol: FitTol,
    crowded: CrowdedVelocityBreaks,
) -> PostProcessError {
    PostProcessError::CrowdedVelocityBreaks {
        axis,
        fit_context,
        break_at: crowded.break_at,
        moved_onto: crowded.moved_onto,
        velocity_step: crowded.velocity_step,
        position_shift: crowded.position_shift,
        position_budget: fit_tol.pos_mm,
        floor: floors.cubic,
    }
}

fn plan_seeds_and_floors<S: TrackSignal>(
    fit: &AxisFit,
    sig: &S,
) -> Result<(Vec<f64>, LadderFloors), PostProcessError> {
    let seeds = fit_seed_times(fit.axis, fit.t_start, fit.t_end, fit.seed_breakpoints)?;
    let floors = seed_floors(fit, sig, &seeds);
    Ok((seeds, floors))
}

fn seed_floors<S: TrackSignal>(fit: &AxisFit, sig: &S, seeds: &[f64]) -> LadderFloors {
    let time_scale = fit.t_start.abs().max(fit.t_end.abs());
    ladder_resolution_floor(sig, seeds, time_scale, fit.fit_tol)
}

fn prepare_fit_plan<S: TrackSignal>(fit: &AxisFit, sig: &S) -> Result<FitPlan, PostProcessError> {
    let seeds = fit_seed_times(fit.axis, fit.t_start, fit.t_end, fit.seed_breakpoints)?;
    plan_over_seeds(fit, sig, seeds)
}

fn plan_over_seeds<S: TrackSignal>(
    fit: &AxisFit,
    sig: &S,
    seeds: Vec<f64>,
) -> Result<FitPlan, PostProcessError> {
    let floors = seed_floors(fit, sig, &seeds);
    let velocity_breaks =
        velocity_breaks_among(sig, &seeds, velocity_resolution(fit.fit_tol, floors));
    fit_plan_from_seeds(seeds, &velocity_breaks, floors, fit.fit_tol.pos_mm).map_err(|crowded| {
        crowded_velocity_breaks_error(fit.axis, fit.fit_context, floors, fit.fit_tol, crowded)
    })
}

/// The widest velocity step a span at the resolution floor can carry and
/// still pass the floor's velocity gate: what an acceleration within
/// tolerance carries across one floor.
fn velocity_resolution(fit_tol: FitTol, floors: LadderFloors) -> f64 {
    fit_tol.accel_mm_s2 * floors.cubic
}

/// [`velocity_resolution`] of a fit to `fit_tol` of a signal reaching
/// positions to `position_scale`, speeds to `velocity_scale` and times to
/// `time_scale`: the speeds such a fit cannot resolve.
pub(crate) fn velocity_resolution_at_scales(
    position_scale: f64,
    velocity_scale: f64,
    time_scale: f64,
    fit_tol: FitTol,
) -> f64 {
    velocity_resolution(
        fit_tol,
        ladder_resolution_floor_from_scales(position_scale, velocity_scale, time_scale, fit_tol),
    )
}

const CUBIC_ANCHOR_OFFSET_CURVATURE: f64 = 6.0;

/// The widest position step a span at the resolution floor can absorb and
/// still hold the acceleration tolerance: a cubic anchored `δ` off its
/// signal at one end bends by `6δ/h²` there.
fn position_resolution(fit_tol: FitTol, floors: LadderFloors) -> f64 {
    fit_tol.accel_mm_s2 * floors.cubic * floors.cubic / CUBIC_ANCHOR_OFFSET_CURVATURE
}

fn velocity_breaks_among<S: TrackSignal>(
    sig: &S,
    seeds: &[f64],
    velocity_resolution: f64,
) -> Vec<VelocityBreak> {
    seeds[1..seeds.len() - 1]
        .iter()
        .map(|&t| VelocityBreak {
            t,
            step: velocity_step_across(sig, t),
        })
        .filter(|velocity_break| velocity_break.step.abs() > velocity_resolution)
        .collect()
}

fn velocity_step_across<S: TrackSignal>(sig: &S, t: f64) -> f64 {
    sig.eval_pva(next_toward(t, f64::INFINITY)).1
        - sig.eval_pva(next_toward(t, f64::NEG_INFINITY)).1
}

struct ScaleProbe {
    position_rounding_scale: f64,
    velocity: f64,
    velocity_step: f64,
}

fn scale_probe<S: TrackSignal>(sig: &S, t: f64) -> ScaleProbe {
    let (position, velocity, _) = sig.eval_pva(t);
    ScaleProbe {
        position_rounding_scale: sig.position_rounding_scale(position),
        velocity,
        velocity_step: velocity_step_across(sig, t),
    }
}

pub(crate) struct FitResolution {
    pub velocity: f64,
    pub position: f64,
}

pub(crate) fn fit_resolution<S: TrackSignal>(
    fit: &AxisFit,
    sig: &S,
) -> Result<FitResolution, PostProcessError> {
    let (_, floors) = plan_seeds_and_floors(fit, sig)?;
    Ok(FitResolution {
        velocity: velocity_resolution(fit.fit_tol, floors),
        position: position_resolution(fit.fit_tol, floors),
    })
}

fn fit_pieces_to_nurbs(
    axis: usize,
    mut pieces: Vec<nurbs::bezier::BezierPiece>,
) -> Result<nurbs::ScalarNurbs, PostProcessError> {
    if pieces.is_empty() {
        return Err(PostProcessError::DegenerateAxisTrack { axis });
    }
    let max_len = pieces
        .iter()
        .map(|piece| piece.coeffs.len())
        .max()
        .unwrap_or(1);
    for piece in &mut pieces {
        piece.coeffs.resize(max_len, 0.0);
    }
    Ok(bezier_pieces_to_nurbs(&pieces))
}

/// One axis fit: the span to cover, the partition seeds, and the budgets it
/// must meet.
#[derive(Clone, Copy)]
pub(crate) struct AxisFit<'a> {
    pub axis: usize,
    pub t_start: f64,
    pub t_end: f64,
    pub seed_breakpoints: &'a [f64],
    pub fit_tol: FitTol,
    pub fit_context: &'static str,
    pub seam_continuity: SeamContinuity,
}

pub(crate) fn fit_axis_from_signal<S: TrackSignal>(
    fit: AxisFit,
    sig: &S,
) -> Result<nurbs::ScalarNurbs, PostProcessError> {
    let plan = prepare_fit_plan(&fit, sig)?;
    fit_axis_over_plan(&fit, sig, plan, false)
}

/// One leader target's fit, seeded with the shaped breaks the hull cannot
/// stand in for, and every span proven by it.
fn fit_leader_target<S: TrackSignal>(
    fit: AxisFit,
    sig: &S,
    shaped_breaks: &ShapedBreaks,
    hidden_break_input: Option<&AxisSignalTable>,
) -> Result<Vec<BezierPiece>, PostProcessError> {
    let seeds = shaped_breaks.select_seeds(
        fit_seed_times(fit.axis, fit.t_start, fit.t_end, fit.seed_breakpoints)?,
        hidden_break_input,
        fit.fit_tol,
    );
    let plan = plan_over_seeds(&fit, sig, seeds)?;
    fit_pieces_over_plan(&fit, sig, plan, true)
}

fn fit_axis_over_plan<S: TrackSignal>(
    fit: &AxisFit,
    sig: &S,
    plan: FitPlan,
    requires_hull: bool,
) -> Result<nurbs::ScalarNurbs, PostProcessError> {
    fit_pieces_to_nurbs(
        fit.axis,
        fit_pieces_over_plan(fit, sig, plan, requires_hull)?,
    )
}

fn fit_pieces_over_plan<S: TrackSignal>(
    fit: &AxisFit,
    sig: &S,
    plan: FitPlan,
    requires_hull: bool,
) -> Result<Vec<BezierPiece>, PostProcessError> {
    let policy = SpanFitPolicy {
        axis: fit.axis,
        fit_tol: fit.fit_tol,
        fit_context: fit.fit_context,
        velocity_budget: f64::INFINITY,
        floors: plan.floors,
        velocity_sign: VelocitySign::Unconstrained,
        seam_continuity: fit.seam_continuity,
        requires_hull,
    };
    let mut refinement_splits = 0;
    let mut pieces = Vec::with_capacity(plan.seeds.len());
    for (span, &slivers) in plan.seeds.windows(2).zip(&plan.slivers) {
        refine_shaped_span(
            sig,
            RefineSpan {
                t0: span[0],
                t1: span[1],
                slivers,
            },
            &policy,
            SpanSeeds {
                lower: span[0],
                upper: span[1],
            },
            &mut refinement_splits,
            &mut pieces,
        )?;
    }
    Ok(pieces)
}

/// The convolution samples one span is fitted against, held by node rather
/// than searched for: the ladder reads every rung off these, so a lookup runs
/// once per rung per node and used to be a linear scan of three heap vectors
/// allocated for every span.
struct SpanTruth {
    pos: [f64; LADDER_PROBES_U.len()],
    vel: [f64; LADDER_PROBES_U.len()],
    acc: [f64; LADDER_PROBES_U.len()],
    fit_pos: [f64; LADDER_FIT_NODES_U.len()],
}

impl SpanTruth {
    fn probe_index(u: f64) -> Option<usize> {
        LADDER_PROBES_U.iter().position(|&node| node == u)
    }

    fn pos_at(&self, u: f64) -> f64 {
        if let Some(index) = Self::probe_index(u) {
            return self.pos[index];
        }
        let index = LADDER_FIT_NODES_U
            .iter()
            .position(|&node| node == u)
            .unwrap_or_else(|| panic!("ladder probed unsampled node u={u}"));
        self.fit_pos[index]
    }

    fn vel_at(&self, u: f64) -> f64 {
        let index = Self::probe_index(u)
            .unwrap_or_else(|| panic!("ladder probed unsampled velocity node u={u}"));
        self.vel[index]
    }

    fn acc_at(&self, u: f64) -> f64 {
        let index = Self::probe_index(u)
            .unwrap_or_else(|| panic!("ladder probed unsampled accel node u={u}"));
        self.acc[index]
    }
}

const GOLDEN_SECTION_STEPS: u32 = 16;

pub(crate) fn golden_section_peak<E>(
    mut value: impl FnMut(f64) -> Result<f64, E>,
    mut low: f64,
    mut high: f64,
) -> Result<(f64, f64), E> {
    let shrink = 0.5 * (5.0_f64.sqrt() - 1.0);
    let mut left = high - shrink * (high - low);
    let mut right = low + shrink * (high - low);
    let (mut left_value, mut right_value) = (value(left)?, value(right)?);
    for _ in 0..GOLDEN_SECTION_STEPS {
        if left_value >= right_value {
            high = right;
            (right, right_value) = (left, left_value);
            left = high - shrink * (high - low);
            left_value = value(left)?;
        } else {
            low = left;
            (left, left_value) = (right, right_value);
            right = low + shrink * (high - low);
            right_value = value(right)?;
        }
    }
    Ok(if left_value >= right_value {
        (left, left_value)
    } else {
        (right, right_value)
    })
}

const LADDER_FIT_NODES_U: [f64; 3] = [0.0, 0.5, -0.5];

/// Everything one fit's spans are refined against: the axis being fitted,
/// its budgets, the resolution floors of the signal, the rung rules, and
/// whether the plan left breaks inside its spans that only the hull over
/// the signal's exact polynomials can hold a piece across.
#[derive(Clone, Copy)]
struct SpanFitPolicy {
    axis: usize,
    fit_tol: FitTol,
    fit_context: &'static str,
    velocity_budget: f64,
    floors: LadderFloors,
    velocity_sign: VelocitySign,
    seam_continuity: SeamContinuity,
    requires_hull: bool,
}

/// One span of a refinement tree.
#[derive(Clone, Copy)]
struct RefineSpan {
    t0: f64,
    t1: f64,
    slivers: EdgeSlivers,
}

/// The partition seeds the refined span sits between, reported by a fit
/// failure so the provenance of an unfittable span is readable.
#[derive(Clone, Copy)]
struct SpanSeeds {
    lower: f64,
    upper: f64,
}

/// How far inside a span edge the state the ladder anchors to is read: past
/// whatever coalescing absorbed there, and at least a cubic floor, but no
/// more than a quarter of the span.
fn edge_inset(h: f64, cubic_floor: f64, absorbed: f64) -> f64 {
    cubic_floor.max(absorbed).min(0.25 * h)
}

/// The velocity and acceleration a span is anchored to at one edge. Both
/// are read at the seam itself, so neighbouring pieces meet with matching
/// state wherever the signal is smooth — unless a break hides in the edge's
/// sliver: then the piece anchors to the acceleration past the sliver,
/// extrapolated back to the seam, the break's far side as it would read
/// there. Where the sliver holds seeds coalescing absorbed, the piece also
/// anchors to the velocity past the sliver carried back to the seam by that
/// acceleration: the velocity the absorbed break changes across the sliver
/// moves onto the seam as a velocity break, as coalescing moves a sub-floor
/// velocity break onto its anchor. Refining toward the seam could otherwise
/// never absorb it, a piece of span `h` bending by `Δv/h` to reach it. The
/// move shifts position by at most the moved step times the sliver, which
/// must stay within the position tolerance. A sliver no seed was absorbed
/// into keeps its seam velocity, so a break nothing declared still fails the
/// fit. The extrapolation runs through a second read as far again past
/// the sliver, so a smooth signal's jerk across a wide sliver cancels; the
/// inset is at most a quarter span, so that read stays inside the span. Its
/// miss of the seam reading only flags a candidate break: a smooth edge
/// whose jerk changes within the sliver misses as well, so the candidate is
/// confirmed by [`sliver_hides_break`]. At a square-root onset the signal
/// declares on the edge the acceleration is steep past any extrapolation
/// but hides no break: the edge anchors to the seam reading, where the span
/// across the onset anchors too.
fn edge_velocity_and_acceleration<S: TrackSignal>(
    sig: &S,
    axis: usize,
    seam: f64,
    past_sliver: f64,
    absorbed_seeds: bool,
    onset_on_edge: bool,
    fit_tol: FitTol,
) -> Result<EdgeAnchor, PostProcessError> {
    let (_, velocity, seam_acceleration) = sig.eval_pva(seam);
    let velocity = exact_value(axis, velocity, seam)?;
    let seam_acceleration = exact_value(axis, seam_acceleration, seam)?;
    let at_seam = EdgeAnchor {
        velocity,
        acceleration: seam_acceleration,
    };
    if onset_on_edge {
        return Ok(at_seam);
    }
    let (_, past_velocity, past_acceleration) = sig.eval_pva(past_sliver);
    let past_velocity = exact_value(axis, past_velocity, past_sliver)?;
    let past_acceleration = exact_value(axis, past_acceleration, past_sliver)?;
    let beyond = nurbs::fmadd(2.0, past_sliver - seam, seam);
    let beyond_acceleration = exact_value(axis, sig.second_deriv(beyond), beyond)?;
    let extrapolated_to_seam = 2.0 * past_acceleration - beyond_acceleration;
    if (extrapolated_to_seam - seam_acceleration).abs() > fit_tol.accel_mm_s2
        && sliver_hides_break(
            sig,
            axis,
            (seam, seam_acceleration),
            (past_sliver, past_acceleration),
            (beyond, beyond_acceleration),
            fit_tol,
        )?
    {
        if !absorbed_seeds {
            return Ok(EdgeAnchor {
                acceleration: extrapolated_to_seam,
                ..at_seam
            });
        }
        let carried_velocity = nurbs::fmadd(
            0.5 * (past_acceleration + extrapolated_to_seam),
            seam - past_sliver,
            past_velocity,
        );
        let velocity_step = carried_velocity - velocity;
        let position_shift = (velocity_step * (seam - past_sliver)).abs();
        if position_shift > fit_tol.pos_mm {
            return Err(PostProcessError::SliverVelocityCarry {
                axis,
                seam,
                past_sliver,
                velocity_step,
                position_shift,
                position_budget: fit_tol.pos_mm,
            });
        }
        return Ok(EdgeAnchor {
            velocity: carried_velocity,
            acceleration: extrapolated_to_seam,
        });
    }
    Ok(at_seam)
}

/// Whether the acceleration between the `seam` and `past` reads jumps rather
/// than bends. Less the line through the `past` and `beyond` reads, the
/// signal misses at the seam by what the extrapolation missed and not at
/// all past the sliver; bisecting toward where it crosses half that miss
/// closes in on a break, whose step stays across the crossing however close
/// the reads, while a smooth signal's change between them vanishes.
fn sliver_hides_break<S: TrackSignal>(
    sig: &S,
    axis: usize,
    seam: (f64, f64),
    past: (f64, f64),
    beyond: (f64, f64),
    fit_tol: FitTol,
) -> Result<bool, PostProcessError> {
    let slope = (beyond.1 - past.1) / (beyond.0 - past.0);
    let residual =
        |(t, acceleration): (f64, f64)| acceleration - nurbs::fmadd(slope, t - past.0, past.1);
    let half_miss = 0.5 * residual(seam);
    let (mut near, mut far) = (seam, past);
    loop {
        let mid = 0.5 * near.0 + 0.5 * far.0;
        if mid == near.0 || mid == far.0 {
            return Ok((far.1 - near.1).abs() > fit_tol.accel_mm_s2);
        }
        let read = (mid, exact_value(axis, sig.second_deriv(mid), mid)?);
        if (residual(read) - half_miss) * half_miss > 0.0 {
            near = read;
        } else {
            far = read;
        }
    }
}

/// The state a span edge anchors its rungs to.
#[derive(Debug, Clone, Copy)]
struct EdgeAnchor {
    velocity: f64,
    acceleration: f64,
}

/// What a span edge's state is read from: the edge, the side its span lies
/// on, how far in past its sliver the state is read, whether the sliver
/// absorbed seeds, and the break coalescing moved onto it. A refinement's
/// child span shares one edge with its parent, and where the inset there is
/// the same the edge reads the same state.
#[derive(Debug, Clone, Copy, PartialEq)]
struct EdgeKey {
    seam: f64,
    into_later: bool,
    past_sliver: f64,
    absorbed: bool,
    moved_break: Option<f64>,
}

/// A span edge's position and the velocity and acceleration its rungs
/// anchor to, read once per [`EdgeKey`] along a refinement.
#[derive(Debug, Clone, Copy)]
struct EdgeState {
    key: EdgeKey,
    position: f64,
    velocity: f64,
    acceleration: f64,
}

impl EdgeState {
    /// The edge's state through `read`, which faces into the span toward
    /// `other_end`, or `known` itself where it was read at the same key.
    fn read<S: TrackSignal>(
        read: &FacingRead<S>,
        key: EdgeKey,
        other_end: f64,
        policy: &SpanFitPolicy,
        known: Option<Self>,
    ) -> Result<Self, PostProcessError> {
        if let Some(known) = known.filter(|known| known.key == key) {
            return Ok(known);
        }
        let (axis, seam) = (policy.axis, key.seam);
        let position = exact_value(axis, read.eval(seam), seam)?;
        let anchor = edge_velocity_and_acceleration(
            read,
            axis,
            next_toward(seam, other_end),
            key.past_sliver,
            key.absorbed,
            square_root_onset_on_edge(read.sig, seam),
            policy.fit_tol,
        )?;
        let velocity = match key.moved_break {
            Some(break_at) => velocity_past_moved_break(read, axis, break_at, other_end, seam)?,
            None => anchor.velocity,
        };
        Ok(Self {
            key,
            position,
            velocity,
            acceleration: anchor.acceleration,
        })
    }
}

/// The velocity a span anchors to at a seam where coalescing moved a velocity
/// break onto it: the far side of the break, carried back to the seam by its
/// acceleration across the sub-floor gap.
fn velocity_past_moved_break<S: TrackSignal>(
    sig: &S,
    axis: usize,
    break_at: f64,
    into_span: f64,
    seam: f64,
) -> Result<f64, PostProcessError> {
    let past = next_toward(break_at, into_span);
    let (_, velocity, acceleration) = sig.eval_pva(past);
    let velocity = exact_value(axis, velocity, past)?;
    let acceleration = exact_value(axis, acceleration, past)?;
    Ok(nurbs::fmadd(acceleration, seam - past, velocity))
}

fn span_probe_t(t0: f64, t1: f64, u: f64) -> f64 {
    match u {
        -1.0 => next_toward(t0, t1),
        1.0 => next_toward(t1, t0),
        _ => nurbs::fmadd(0.5 * (u + 1.0), t1 - t0, t0),
    }
}

/// Ladder fit of the shaped signal over one span: the endpoint-anchored
/// quadratic carrying the span's own travel, the cubic through both endpoint
/// velocities, then the quintic Hermite matching the convolution's exact
/// (p, v, a) at both ends and degrees 6/7 from interior residuals — the
/// lowering ladder against pre-sampled truth, with the shaper's own budgets.
/// Every accepted piece is endpoint-anchored, so the span seams stay C0 no
/// matter which rung wins. On a span reaching a square-root onset the
/// acceleration's shoulder bends between the probes, so the rungs are also
/// held to the tolerance across the resolved stretch by `OnsetHold`: a miss
/// there splits a splittable span — grading the partition toward the onset
/// only as deep as its fit needs — and at the floor, where the closest rung
/// is kept, it is the closest there too. The stretch is sampled only once a
/// rung passes the probes or a kept closest rung is ranked. Returns the
/// accepted monomial-in-u fit, or the closest rung with the ladder's miss.
///
/// A rung that passes the probes is then held between them: proven by the
/// Bernstein hull of its error against the signal's exact polynomials
/// where the signal has them, else against its Taylor-model enclosure; a
/// signal with neither fails loudly with
/// [`PostProcessError::UnprovableSpan`]. Where it has exact polynomials
/// every truth sample is read off them too, each edge's from the side
/// facing into the span.
///
/// A rung a looser-than-absolute budget accepts on a splittable span is
/// refit to the absolute budget where its acceleration may pass the
/// budget's ceiling ([`FitTol::accel_ceiling`]); at the floor no finer
/// partition could hold it, and the looser budget stands.
fn shaped_ladder<S: TrackSignal>(
    sig: &S,
    exact: Option<&ExactSpan>,
    span: RefineSpan,
    policy: &SpanFitPolicy,
    keeps_closest_rung: bool,
    known_edges: [Option<EdgeState>; 2],
) -> Result<(SpanVerdict, [EdgeState; 2]), PostProcessError> {
    let RefineSpan { t0, t1, slivers } = span;
    let axis = policy.axis;
    let fit_tol = policy.fit_tol;
    let h = t1 - t0;
    let piece_tol = fit_tol.for_piece(h);
    let t_of = |u: f64| nurbs::fmadd(0.5 * (u + 1.0), h, t0);
    let from_start = FacingRead {
        sig,
        exact,
        toward: t1,
    };
    let from_end = FacingRead {
        sig,
        exact,
        toward: t0,
    };
    let t0_inside = span_probe_t(t0, t1, -1.0);
    let start_past_sliver = t0 + edge_inset(h, policy.floors.cubic, slivers.start);
    let end_past_sliver = t1 - edge_inset(h, policy.floors.cubic, slivers.end);
    let start = EdgeState::read(
        &from_start,
        EdgeKey {
            seam: t0,
            into_later: t1 > t0,
            past_sliver: start_past_sliver,
            absorbed: slivers.start > 0.0,
            moved_break: slivers.start_break,
        },
        t1,
        policy,
        known_edges[0],
    )?;
    let (p0, v0, a0) = (start.position, start.velocity, start.acceleration);
    let t1_inside = span_probe_t(t0, t1, 1.0);
    let end = EdgeState::read(
        &from_end,
        EdgeKey {
            seam: t1,
            into_later: t0 > t1,
            past_sliver: end_past_sliver,
            absorbed: slivers.end > 0.0,
            moved_break: slivers.end_break,
        },
        t0,
        policy,
        known_edges[1],
    )?;
    let (p1, v1, a1) = (end.position, end.velocity, end.acceleration);
    let base = quintic_in_u((p0, v0, a0), (p1, v1, a1), h);
    let endpoint_delta = exact_value(axis, sig.position_delta((t0, p0), (t1, p1)), t1)?;

    let mut truth = SpanTruth {
        pos: [f64::NAN; LADDER_PROBES_U.len()],
        vel: [f64::NAN; LADDER_PROBES_U.len()],
        acc: [f64::NAN; LADDER_PROBES_U.len()],
        fit_pos: [f64::NAN; LADDER_FIT_NODES_U.len()],
    };
    for (index, &u) in LADDER_PROBES_U.iter().enumerate() {
        let (pos, vel, acc) = match u {
            -1.0 => (p0, v0, a0),
            1.0 => (p1, v1, a1),
            _ => {
                let t = t_of(u);
                let (pos, vel, acc) = from_start.eval_pva(t);
                (
                    exact_value(axis, pos, t)?,
                    exact_value(axis, vel, t)?,
                    exact_value(axis, acc, t)?,
                )
            }
        };
        truth.pos[index] = pos;
        truth.vel[index] = vel;
        truth.acc[index] = acc;
    }
    for (index, &u) in LADDER_FIT_NODES_U.iter().enumerate() {
        truth.fit_pos[index] = match SpanTruth::probe_index(u) {
            Some(probe) => truth.pos[probe],
            None => finite_sample(axis, &from_start, t_of(u))?,
        };
    }

    let onset_hold = square_root_onset_within(sig, t0, t1).then(|| OnsetHold {
        read: &from_start,
        span: (t0, t1),
        resolved: (start_past_sliver, end_past_sliver),
        tol: piece_tol,
        ranks_misses: keeps_closest_rung,
        samples: std::array::from_fn(|_| std::cell::OnceCell::new()),
        edge_scans: std::cell::OnceCell::new(),
    });
    let held_miss = |rung: &[f64]| {
        onset_hold
            .as_ref()
            .map_or(0.0, |hold| hold.worst_miss(rung))
    };
    let ladder_span = LadderSpan { h, endpoint_delta };
    let exempt_stretches = exempt_accelerations(sig, exact, (t0, t1), piece_tol, policy);
    let exempt_at = |u: f64| {
        let t = span_probe_t(t0, t1, u);
        exempt_stretches
            .as_ref()
            .is_some_and(|(stretches, _)| stretches.iter().any(|&(lo, hi)| lo <= t && t <= hi))
    };
    let exemption = exempt_stretches
        .as_ref()
        .filter(|_| LADDER_PROBES_U.iter().any(|&u| exempt_at(u)))
        .map(|&(_, velocity)| AccelerationExemption {
            exempt: &exempt_at,
            velocity,
            ceiling: fit_tol.accel_ceiling(),
        });
    let ladder_truth = LadderTruth {
        position: &|u| truth.pos_at(u),
        velocity: &|u| truth.vel_at(u),
        acceleration: &|u| truth.acc_at(u),
        held_miss: &held_miss,
        held_ranks_misses: keeps_closest_rung,
        exemption,
    };
    let ladder_budget = LadderBudget {
        tol: piece_tol,
        velocity: policy.velocity_budget,
    };
    let ladder_policy = LadderPolicy {
        velocity_sign: policy.velocity_sign,
        acceleration_monotonicity: sig.acceleration_monotonicity(
            (t0_inside, t1_inside),
            (start_past_sliver, end_past_sliver),
        ),
        high_degree_span_floor: policy.floors.high_degree,
        seam_continuity: policy.seam_continuity,
    };
    let fitted = if keeps_closest_rung {
        ladder_fit(
            &base,
            ladder_span,
            &ladder_truth,
            ladder_budget,
            ladder_policy,
        )
        .map_err(Some)
    } else {
        ladder_accept(
            &base,
            ladder_span,
            &ladder_truth,
            ladder_budget,
            ladder_policy,
        )
        .ok_or(None)
    };
    match fitted {
        Ok(coefficients) => {
            let splittable = !keeps_closest_rung;
            if splittable
                && piece_tol.accel_mm_s2 > fit_tol.accel_mm_s2
                && acceleration_bound(&coefficients, h) > fit_tol.accel_ceiling()
            {
                let absolute = SpanFitPolicy {
                    fit_tol: fit_tol.absolute_budget(),
                    ..*policy
                };
                return shaped_ladder(sig, exact, span, &absolute, keeps_closest_rung, known_edges);
            }
            let resolved = (start_past_sliver, end_past_sliver);
            let piece = FittedPiece {
                mono_u: &coefficients,
                t0,
                t1,
            };
            let budget = HullBudget {
                tol: piece_tol,
                velocity: policy.velocity_budget,
            };
            let unproven = match exact {
                Some(exact) => hull::first_unproven(
                    exact.over(resolved.0, resolved.1),
                    piece,
                    resolved,
                    budget,
                ),
                None => match sig.enclosure_on(resolved.0, resolved.1, budget.enclosure_accuracy())
                {
                    Some(enclosure) => hull::first_unproven(&enclosure, piece, resolved, budget),
                    None => {
                        return Err(PostProcessError::UnprovableSpan {
                            axis,
                            fit_context: policy.fit_context,
                            t_start: t0,
                            t_end: t1,
                        });
                    }
                },
            };
            let Some(u) = unproven else {
                return Ok((SpanVerdict::Proven(coefficients), [start, end]));
            };
            let t = t_of(u);
            let read = if t < t1 { &from_start } else { &from_end };
            let (source_position, source_velocity, source_acceleration) = read.eval_pva(t);
            let piece = exact_piece(&coefficients, t0, t1, h);
            let velocity = piece.differentiate();
            let acceleration = velocity.differentiate();
            let candidate_position = piece.evaluate(t);
            let candidate_velocity = velocity.evaluate(t);
            let candidate_acceleration = acceleration.evaluate(t);
            Ok((
                SpanVerdict::Missed(
                    coefficients,
                    LadderFailure {
                        u,
                        error: LadderPva {
                            position: (candidate_position - source_position).abs(),
                            velocity: (candidate_velocity - source_velocity).abs(),
                            acceleration: (candidate_acceleration - source_acceleration).abs(),
                        },
                        source: LadderPva {
                            position: source_position,
                            velocity: source_velocity,
                            acceleration: source_acceleration,
                        },
                        candidate: LadderPva {
                            position: candidate_position,
                            velocity: candidate_velocity,
                            acceleration: candidate_acceleration,
                        },
                        left: LadderPva {
                            position: p0,
                            velocity: v0,
                            acceleration: a0,
                        },
                        right: LadderPva {
                            position: p1,
                            velocity: v1,
                            acceleration: a1,
                        },
                    },
                ),
                [start, end],
            ))
        }
        Err(Some(LadderMiss {
            closest_rung,
            failure,
        })) => Ok((SpanVerdict::Missed(closest_rung, failure), [start, end])),
        Err(None) => Ok((SpanVerdict::Refused, [start, end])),
    }
}

/// The stretches of `span` whose acceleration the proof exempts — the
/// enclosure's pieces inside an unshaped leader's square-root onset sliver
/// ([`crate::follower_projection::UnshapedLeaderStep`]) — and the velocity
/// the proof holds a fit to there; `None` where none of the span is exempt.
/// Only a signal fitted under a ceiling on its own acceleration has them:
/// a rung there is held to that ceiling instead of the signal.
fn exempt_accelerations<S: TrackSignal>(
    sig: &S,
    exact: Option<&ExactSpan>,
    (t0, t1): (f64, f64),
    piece_tol: FitTol,
    policy: &SpanFitPolicy,
) -> Option<(Vec<(f64, f64)>, f64)> {
    if exact.is_some() || !policy.fit_tol.accel_ceiling().is_finite() {
        return None;
    }
    let budget = HullBudget {
        tol: piece_tol,
        velocity: policy.velocity_budget,
    };
    let enclosure = sig.enclosure_on(t0, t1, budget.enclosure_accuracy())?;
    let mut velocity = f64::INFINITY;
    let stretches: Vec<(f64, f64)> = enclosure
        .iter()
        .filter_map(|piece| {
            let step = piece.unshaped_step?;
            velocity = velocity.min(step.allowance());
            Some((piece.lo, piece.hi))
        })
        .collect();
    (!stretches.is_empty()).then_some((stretches, velocity.min(policy.velocity_budget)))
}

/// What a span's ladder and hull made of it.
enum SpanVerdict {
    /// A rung, proven within the budget.
    Proven(Vec<f64>),
    /// The closest rung of a ladder that refused them all, or a rung the
    /// hull did not prove, and where it misses.
    Missed(Vec<f64>, LadderFailure),
    /// Every rung refused on a splittable span, which is split without
    /// ranking them.
    Refused,
}

/// Intervals between the check points a span reaching a square-root onset
/// is held to the fit tolerance at between the ladder's probes.
const SQUARE_ROOT_ONSET_CHECK_INTERVALS: usize = 64;

/// Exact samples the stretch between a resolved edge and the check point
/// next to it is scanned at before its peak is searched.
const SQUARE_ROOT_ONSET_EDGE_SCAN_POINTS: u32 = 8;

/// Share of the tolerance above which a rung's miss peaking at a check
/// point is searched for exactly between that point's neighbours.
const SQUARE_ROOT_ONSET_PEAK_SEARCH_SHARE: f64 = 0.25;

/// Where a span reaching a square-root onset holds its rungs to the fit
/// tolerance between the ladder's probes: at check points spread evenly
/// across the resolved stretch, its edges included, then — for a rung they
/// all hold — at the exact peaks between them. Next to an edge a rung's
/// miss of an onset there rises as the root of the distance from it, a
/// shoulder narrower than any grid of check points, so the stretch from
/// each resolved edge to the check point next to it is searched in that
/// root, where the shoulder is smooth. Each check point is sampled once,
/// the first time a rung reads it, from the edges inward, where a rung
/// misses an onset first: a rung that misses one is refused before the
/// rest are read, unless its misses are ranked.
struct OnsetHold<'a, S> {
    read: &'a FacingRead<'a, S>,
    span: (f64, f64),
    resolved: (f64, f64),
    tol: FitTol,
    ranks_misses: bool,
    samples: [std::cell::OnceCell<HeldSample>; SQUARE_ROOT_ONSET_CHECK_INTERVALS + 1],
    edge_scans: std::cell::OnceCell<[EdgeScan; 2]>,
}

/// The stretch between a resolved edge and the check point next to it, in
/// the root of the distance from the span edge.
#[derive(Clone, Copy)]
struct EdgeRoots {
    span_edge: f64,
    inward: f64,
    from: f64,
    to: f64,
    step: f64,
}

impl EdgeRoots {
    fn t_of_root(&self, root: f64) -> f64 {
        nurbs::fmadd(self.inward * root, root, self.span_edge)
    }

    fn scan_root(&self, k: u32) -> f64 {
        nurbs::fmadd(f64::from(k + 1), self.step, self.from)
    }
}

/// The exact samples of an edge stretch, evenly spaced in its root: the
/// same for every rung of the span.
struct EdgeScan {
    roots: EdgeRoots,
    samples: [HeldSample; SQUARE_ROOT_ONSET_EDGE_SCAN_POINTS as usize],
}

impl<S: TrackSignal> OnsetHold<'_, S> {
    fn check_t(&self, k: usize) -> f64 {
        if k == SQUARE_ROOT_ONSET_CHECK_INTERVALS {
            return self.resolved.1;
        }
        nurbs::fmadd(
            k as f64 / SQUARE_ROOT_ONSET_CHECK_INTERVALS as f64,
            self.resolved.1 - self.resolved.0,
            self.resolved.0,
        )
    }

    fn h(&self) -> f64 {
        self.span.1 - self.span.0
    }

    fn sample_at(&self, t: f64) -> HeldSample {
        let (position, _, acceleration) = self.read.eval_pva(t);
        HeldSample {
            x: 2.0 * (t - self.span.0) / self.h() - 1.0,
            position,
            acceleration,
        }
    }

    fn miss_at(&self, rung: &[f64], t: f64) -> f64 {
        held_miss(rung, self.h(), &[self.sample_at(t)], self.tol)
    }

    fn check_sample(&self, k: usize) -> HeldSample {
        *self.samples[k].get_or_init(|| self.sample_at(self.check_t(k)))
    }

    fn check_miss(&self, rung: &[f64], k: usize) -> f64 {
        held_miss(rung, self.h(), &[self.check_sample(k)], self.tol)
    }

    /// The rung's worst miss, or — unless its misses are ranked — the
    /// first check point's it misses past the tolerance: a refused rung
    /// only needs to be refused.
    fn worst_miss(&self, rung: &[f64]) -> f64 {
        let mut misses = [0.0; SQUARE_ROOT_ONSET_CHECK_INTERVALS + 1];
        for k in (0..=SQUARE_ROOT_ONSET_CHECK_INTERVALS / 2)
            .flat_map(|inward| [inward, SQUARE_ROOT_ONSET_CHECK_INTERVALS - inward])
            .take(SQUARE_ROOT_ONSET_CHECK_INTERVALS + 1)
        {
            misses[k] = self.check_miss(rung, k);
            if !self.ranks_misses && misses[k] > 1.0 {
                return misses[k];
            }
        }
        let at_checks = misses.iter().copied().fold(0.0, f64::max);
        if at_checks > 1.0 {
            return at_checks;
        }
        let last = SQUARE_ROOT_ONSET_CHECK_INTERVALS;
        let edge_scans = self.edge_scans.get_or_init(|| {
            [
                self.edge_scan(self.span.0, self.check_t(0), self.check_t(1)),
                self.edge_scan(self.span.1, self.check_t(last), self.check_t(last - 1)),
            ]
        });
        let mut worst = edge_scans
            .iter()
            .map(|scan| self.edge_peak(rung, scan))
            .fold(at_checks, f64::max);
        for (before, around) in misses.windows(3).enumerate() {
            if around[1] > SQUARE_ROOT_ONSET_PEAK_SEARCH_SHARE
                && around[1] >= around[0]
                && around[1] >= around[2]
            {
                let Ok((_, peak)) = golden_section_peak(
                    |t| Ok::<_, std::convert::Infallible>(self.miss_at(rung, t)),
                    self.check_t(before),
                    self.check_t(before + 2),
                );
                worst = worst.max(peak);
            }
        }
        worst
    }

    fn edge_scan(&self, span_edge: f64, resolved_edge: f64, next_check: f64) -> EdgeScan {
        let (from, to) = (
            (resolved_edge - span_edge).abs().sqrt(),
            (next_check - span_edge).abs().sqrt(),
        );
        let roots = EdgeRoots {
            span_edge,
            inward: (next_check - span_edge).signum(),
            from,
            to,
            step: (to - from) / f64::from(SQUARE_ROOT_ONSET_EDGE_SCAN_POINTS + 1),
        };
        EdgeScan {
            roots,
            samples: std::array::from_fn(|k| {
                let k = u32::try_from(k).expect("the edge scan has a handful of points");
                self.sample_at(roots.t_of_root(roots.scan_root(k)))
            }),
        }
    }

    /// The rung's largest miss between a resolved edge and the check point
    /// next to it, read in the root of the distance from the span edge.
    fn edge_peak(&self, rung: &[f64], scan: &EdgeScan) -> f64 {
        let roots = scan.roots;
        let mut scan_peak = (roots.from, 0.0);
        for (k, sample) in (0..).zip(&scan.samples) {
            let miss = held_miss(rung, self.h(), &[*sample], self.tol);
            if miss >= scan_peak.1 {
                scan_peak = (roots.scan_root(k), miss);
            }
        }
        let Ok((_, peak)) = golden_section_peak(
            |root| Ok::<_, std::convert::Infallible>(self.miss_at(rung, roots.t_of_root(root))),
            (scan_peak.0 - roots.step).max(roots.from),
            (scan_peak.0 + roots.step).min(roots.to),
        );
        peak.max(scan_peak.1)
    }
}

/// How far from a span edge a declared square-root onset still lies on it:
/// the onsets and the seeds the edges come from are computed apart, ulps of
/// a print's time apart, and no fit resolves anything this short.
pub(crate) const SQUARE_ROOT_ONSET_EDGE_SLACK_S: f64 = 1e-9;

fn square_root_onset_within<S: TrackSignal>(sig: &S, from: f64, to: f64) -> bool {
    let onsets = sig.square_root_onsets();
    onsets
        .get(onsets.partition_point(|&onset| onset < from))
        .is_some_and(|&onset| onset <= to)
}

pub(crate) fn square_root_onset_on_edge<S: TrackSignal>(sig: &S, edge: f64) -> bool {
    square_root_onset_within(
        sig,
        edge - SQUARE_ROOT_ONSET_EDGE_SLACK_S,
        edge + SQUARE_ROOT_ONSET_EDGE_SLACK_S,
    )
}

/// Refines one partition span. Where the signal has exact polynomials over
/// it they are built once, up front, and every piece refined from the span
/// reads its truth off them and is proven against them. A plan that left
/// breaks for the hull alone to hold has no other proof: a span without
/// exact polynomials is an error there.
fn refine_shaped_span<S: TrackSignal>(
    sig: &S,
    span: RefineSpan,
    policy: &SpanFitPolicy,
    seeds: SpanSeeds,
    refinement_splits: &mut usize,
    out: &mut Vec<BezierPiece>,
) -> Result<(), PostProcessError> {
    let exact = sig
        .exact_polynomial_on(span.t0, span.t1)
        .map(ExactSpan::new);
    if policy.requires_hull && exact.is_none() {
        return Err(PostProcessError::UnprovableSpan {
            axis: policy.axis,
            fit_context: policy.fit_context,
            t_start: span.t0,
            t_end: span.t1,
        });
    }
    refine_span_against(
        sig,
        exact.as_ref(),
        span,
        policy,
        seeds,
        refinement_splits,
        out,
        [None, None],
    )
}

#[allow(clippy::too_many_arguments)]
fn refine_span_against<S: TrackSignal>(
    sig: &S,
    exact: Option<&ExactSpan>,
    span: RefineSpan,
    policy: &SpanFitPolicy,
    seeds: SpanSeeds,
    refinement_splits: &mut usize,
    out: &mut Vec<BezierPiece>,
    known_edges: [Option<EdgeState>; 2],
) -> Result<(), PostProcessError> {
    let RefineSpan { t0, t1, slivers } = span;
    let fit_tol = policy.fit_tol;
    let velocity_budget = policy.velocity_budget;
    let floors = policy.floors;
    let tm = 0.5 * t0 + 0.5 * t1;
    let at_resolution_floor = tm - t0 < floors.cubic || t1 - tm < floors.cubic;
    let splittable = !at_resolution_floor && *refinement_splits < MAX_FIT_REFINEMENT_SPLITS;
    let (verdict, [start_edge, end_edge]) =
        shaped_ladder(sig, exact, span, policy, !splittable, known_edges)?;
    #[cfg(feature = "fit-audit")]
    let edge_insets = (
        edge_inset(t1 - t0, floors.cubic, slivers.start),
        edge_inset(t1 - t0, floors.cubic, slivers.end),
    );
    let missed = match verdict {
        SpanVerdict::Proven(mono_u) => {
            #[cfg(feature = "fit-audit")]
            crate::fit_audit::check(
                sig,
                crate::fit_audit::AcceptedPiece {
                    fit_context: policy.fit_context,
                    axis: policy.axis,
                    fit_tol: policy.fit_tol,
                    t0,
                    t1,
                    mono_u: &mono_u,
                    accepted_at_floor: false,
                    cubic_floor: floors.cubic,
                    edge_insets,
                    exact,
                },
            );
            out.push(exact_piece(&mono_u, t0, t1, t1 - t0));
            return Ok(());
        }
        SpanVerdict::Missed(mono_u, failure) => Some((mono_u, failure)),
        SpanVerdict::Refused => None,
    };
    if !splittable {
        let (mono_u, failure) =
            missed.expect("a span at its floor ranks the rungs its ladder refuses");
        if failure.error.position <= fit_tol.pos_mm
            && failure.error.velocity <= velocity_budget
            && (at_resolution_floor || t1 - t0 <= floors.high_degree)
            && sig
                .acceleration_monotonicity(
                    (next_toward(t0, t1), next_toward(t1, t0)),
                    (next_toward(t0, t1), next_toward(t1, t0)),
                )
                .is_none()
        {
            let velocity_change = failure.right.velocity - failure.left.velocity;
            let gate = FloorVelocityGate::over(sig, t0, t1, fit_tol);
            if !gate.admits(velocity_change) {
                return Err(PostProcessError::VelocityStepAtFitFloor {
                    axis: policy.axis,
                    fit_context: policy.fit_context,
                    t_start: t0,
                    t_end: t1,
                    velocity_change,
                    carried_low: gate.carried_low,
                    carried_high: gate.carried_high,
                    velocity_slack: gate.slack,
                    position_error: failure.error.position,
                    position_budget: fit_tol.pos_mm,
                    velocity_error: failure.error.velocity,
                    acceleration_error: failure.error.acceleration,
                    acceleration_budget: fit_tol.for_piece(t1 - t0).accel_mm_s2,
                });
            }
            #[cfg(feature = "fit-audit")]
            crate::fit_audit::check(
                sig,
                crate::fit_audit::AcceptedPiece {
                    fit_context: policy.fit_context,
                    axis: policy.axis,
                    fit_tol: policy.fit_tol,
                    t0,
                    t1,
                    mono_u: &mono_u,
                    accepted_at_floor: true,
                    cubic_floor: floors.cubic,
                    edge_insets,
                    exact,
                },
            );
            out.push(exact_piece(&mono_u, t0, t1, t1 - t0));
            return Ok(());
        }
        let probe_t = nurbs::fmadd(0.5 * (failure.u + 1.0), t1 - t0, t0);
        return Err(PostProcessError::FitTolerance {
            axis: policy.axis,
            t_start: t0,
            t_end: t1,
            probe_u: failure.u,
            probe_t,
            lower_seed: seeds.lower,
            upper_seed: seeds.upper,
            lower_seed_provenance: "phase/spline/kernel-shift breakpoint",
            upper_seed_provenance: "phase/spline/kernel-shift breakpoint",
            left_position: failure.left.position,
            left_velocity: failure.left.velocity,
            left_acceleration: failure.left.acceleration,
            signal_detail: sig.diagnostic(probe_t),
            right_position: failure.right.position,
            right_velocity: failure.right.velocity,
            right_acceleration: failure.right.acceleration,
            position_error: failure.error.position,
            position_budget: fit_tol.pos_mm,
            fit_context: policy.fit_context,
            refinement_splits: *refinement_splits,
            velocity_error: failure.error.velocity,
            velocity_budget,
            acceleration_error: failure.error.acceleration,
            acceleration_budget: fit_tol.for_piece(t1 - t0).accel_mm_s2,
            source_position: failure.source.position,
            source_velocity: failure.source.velocity,
            source_acceleration: failure.source.acceleration,
            candidate_position: failure.candidate.position,
            candidate_velocity: failure.candidate.velocity,
            candidate_acceleration: failure.candidate.acceleration,
        });
    }
    *refinement_splits += 1;
    refine_span_against(
        sig,
        exact,
        RefineSpan {
            t0,
            t1: tm,
            slivers: EdgeSlivers {
                end: 0.0,
                end_break: None,
                ..slivers
            },
        },
        policy,
        seeds,
        refinement_splits,
        out,
        [Some(start_edge), None],
    )?;
    refine_span_against(
        sig,
        exact,
        RefineSpan {
            t0: tm,
            t1,
            slivers: EdgeSlivers {
                start: 0.0,
                start_break: None,
                ..slivers
            },
        },
        policy,
        seeds,
        refinement_splits,
        out,
        [None, Some(end_edge)],
    )
}

/// What a floor-width span may carry in velocity across it: the velocity
/// its own probed acceleration accounts for, widened by what an acceleration
/// within tolerance carries over the span.
struct FloorVelocityGate {
    carried_low: f64,
    carried_high: f64,
    slack: f64,
}

impl FloorVelocityGate {
    fn over<S: TrackSignal>(sig: &S, t0: f64, t1: f64, fit_tol: FitTol) -> Self {
        let h = t1 - t0;
        let (lowest, highest) = LADDER_PROBES_U
            .iter()
            .map(|&u| sig.second_deriv(span_probe_t(t0, t1, u)))
            .fold(
                (f64::INFINITY, f64::NEG_INFINITY),
                |(lowest, highest), acceleration| {
                    (lowest.min(acceleration), highest.max(acceleration))
                },
            );
        Self {
            carried_low: lowest * h,
            carried_high: highest * h,
            slack: fit_tol.accel_mm_s2 * h,
        }
    }

    fn admits(&self, velocity_change: f64) -> bool {
        velocity_change >= self.carried_low - self.slack
            && velocity_change <= self.carried_high + self.slack
    }
}

fn fit_tolerance_without_probe(axis: usize, t_start: f64, t_end: f64) -> PostProcessError {
    PostProcessError::FitTolerance {
        axis,
        fit_context: "non-fit error",
        t_start,
        signal_detail: None,
        t_end,
        probe_u: f64::NAN,
        probe_t: f64::NAN,
        lower_seed: f64::NAN,
        upper_seed: f64::NAN,
        lower_seed_provenance: "unavailable",
        upper_seed_provenance: "unavailable",
        left_position: f64::NAN,
        left_velocity: f64::NAN,
        left_acceleration: f64::NAN,
        right_position: f64::NAN,
        right_velocity: f64::NAN,
        right_acceleration: f64::NAN,
        position_error: f64::NAN,
        position_budget: f64::NAN,
        refinement_splits: 0,
        velocity_error: f64::NAN,
        velocity_budget: f64::NAN,
        acceleration_error: f64::NAN,
        acceleration_budget: f64::NAN,
        source_position: f64::NAN,
        source_velocity: f64::NAN,
        source_acceleration: f64::NAN,
        candidate_position: f64::NAN,

        candidate_velocity: f64::NAN,
        candidate_acceleration: f64::NAN,
    }
}

fn tanh_acceleration_extremum_velocity(advance: trajectory::NonlinearAdvance) -> f64 {
    advance.linearization_velocity * libm::atanh(1.0 / libm::sqrt(3.0))
}

fn nonlinear_transition_breakpoints(sig: &NonlinearAdvanceSignal) -> Vec<f64> {
    if sig.adv.model == trajectory::AdvanceModel::Tanh {
        let extremum = tanh_acceleration_extremum_velocity(sig.adv);
        let mut roots = Vec::new();
        for (piece, coefficients) in sig.coeffs.iter().enumerate() {
            if coefficients.len() < 3
                || coefficients[2] == 0.0
                || coefficients
                    .iter()
                    .skip(3)
                    .any(|&coefficient| coefficient != 0.0)
            {
                continue;
            }
            for velocity in [-extremum, extremum] {
                let time =
                    sig.starts[piece] + (velocity - coefficients[1]) / (2.0 * coefficients[2]);
                if sig.starts[piece] < time && time < sig.ends[piece] {
                    roots.push(time);
                }
            }
        }
        return roots;
    }
    let mut roots = Vec::new();
    for (piece, coeffs) in sig.coeffs.iter().enumerate() {
        let (u_start, u_end) = (sig.starts[piece], sig.ends[piece]);
        let velocity = |t: f64| {
            let tau = t - u_start;
            coeffs
                .iter()
                .enumerate()
                .skip(1)
                .rev()
                .fold(0.0, |value, (power, &coefficient)| {
                    nurbs::fmadd(value, tau, power as f64 * coefficient)
                })
        };
        let mut lower_t = u_start;
        let mut lower_v = velocity(lower_t);
        for step in 1..=64 {
            let upper_t = nurbs::fmadd(step as f64 / 64.0, u_end - u_start, u_start);
            let upper_v = velocity(upper_t);
            if lower_v == 0.0 {
                roots.push(lower_t);
            } else if upper_v == 0.0 {
                roots.push(upper_t);
            } else if lower_v.is_sign_positive() != upper_v.is_sign_positive() {
                let positive = lower_v.is_sign_positive();
                let mut lower = ordered_f64_key(lower_t);
                let mut upper = ordered_f64_key(upper_t);
                while upper - lower > 1 {
                    let middle = lower + (upper - lower) / 2;
                    let middle_t = f64_from_ordered_key(middle);
                    if velocity(middle_t).is_sign_positive() == positive {
                        lower = middle;
                    } else {
                        upper = middle;
                    }
                }
                roots.push(f64_from_ordered_key(upper));
            }
            lower_t = upper_t;
            lower_v = upper_v;
        }
    }
    roots.sort_by(f64::total_cmp);
    roots.dedup();
    roots
}

fn finite_sample<S: TrackSignal>(axis: usize, sig: &S, t: f64) -> Result<f64, PostProcessError> {
    exact_value(axis, sig.eval(t), t)
}

fn exact_value(axis: usize, value: f64, t: f64) -> Result<f64, PostProcessError> {
    if value.is_finite() {
        Ok(value)
    } else {
        Err(PostProcessError::NonFiniteSample { axis, t })
    }
}

pub(crate) fn apply_zero_support_transform(
    transform: Option<&ChainStage>,
    axis: usize,
    track: nurbs::ScalarNurbs,
    fit_tol: FitTol,
) -> Result<nurbs::ScalarNurbs, PostProcessError> {
    match transform {
        Some(ChainStage::DerivativeGains { k1, k2 }) => {
            Ok(apply_derivative_gains_to_track(&track, *k1, *k2))
        }
        Some(ChainStage::NonlinearAdvance(advance)) => {
            apply_nonlinear_advance_to_track(axis, &track, *advance, fit_tol)
        }
        Some(ChainStage::SmoothKernel(_)) => unreachable!("zero-support transform is a kernel"),
        None => Ok(track),
    }
}

/// `y = x + a(ẋ)` is not polynomial in `x`, so the transformed track is
/// re-fitted from sampled `(p, v, a)` with the same ladder the convolution
/// fits use — the input track's own breakpoints seed the partition and each
/// span refines until it meets the shaper's position/acceleration budgets.
pub(crate) fn apply_nonlinear_advance_to_track(
    axis: usize,
    track: &nurbs::ScalarNurbs,
    adv: trajectory::NonlinearAdvance,
    fit_tol: FitTol,
) -> Result<nurbs::ScalarNurbs, PostProcessError> {
    with_nonlinear_advance_fit(axis, track, adv, fit_tol, |fit, sig| {
        fit_axis_from_signal(fit, sig)
    })
}

/// The shortest span the nonlinear-advance refit of `track` resolves: seed
/// times closer than this to a kept seed are coalesced onto it.
#[cfg(test)]
pub(crate) fn nonlinear_advance_resolution_floor(
    axis: usize,
    track: &nurbs::ScalarNurbs,
    adv: trajectory::NonlinearAdvance,
    fit_tol: FitTol,
) -> Result<f64, PostProcessError> {
    with_nonlinear_advance_fit(axis, track, adv, fit_tol, |fit, sig| {
        Ok(plan_seeds_and_floors(&fit, sig)?.1.cubic)
    })
}

fn with_nonlinear_advance_fit<R>(
    axis: usize,
    track: &nurbs::ScalarNurbs,
    adv: trajectory::NonlinearAdvance,
    fit_tol: FitTol,
    fit: impl FnOnce(AxisFit, &NonlinearAdvanceSignal) -> R,
) -> R {
    let sig = NonlinearAdvanceSignal::new(track, adv);
    let pieces = extract_bezier_pieces(track);
    let t_start = pieces
        .first()
        .expect("nonlinear advance: empty track")
        .u_start;
    let t_end = pieces.last().expect("nonlinear advance: empty track").u_end;
    let mut breakpoints = track.knots().to_vec();
    breakpoints.extend(nonlinear_transition_breakpoints(&sig));
    fit(
        AxisFit {
            axis,
            t_start,
            t_end,
            seed_breakpoints: &breakpoints,
            fit_tol,
            fit_context: "nonlinear_advance",
            seam_continuity: SeamContinuity::Velocity,
        },
        &sig,
    )
}

/// The advance law applied to a polynomial track, evaluated as a signal:
/// `y = x + a(v)`, `ẏ = v + a'(v)·acc`, `ÿ = acc + a''(v)·acc² + a'(v)·jerk`.
struct NonlinearAdvanceSignal {
    starts: Vec<f64>,
    ends: Vec<f64>,
    coeffs: Vec<Vec<f64>>,
    adv: trajectory::NonlinearAdvance,
    cursor: std::cell::Cell<usize>,
}

impl NonlinearAdvanceSignal {
    fn new(track: &nurbs::ScalarNurbs, adv: trajectory::NonlinearAdvance) -> Self {
        let pieces = extract_bezier_pieces(track);
        assert!(!pieces.is_empty(), "nonlinear advance: empty track");
        let starts: Vec<f64> = pieces.iter().map(|piece| piece.u_start).collect();
        let ends: Vec<f64> = pieces.iter().map(|piece| piece.u_end).collect();
        let mut coeffs: Vec<Vec<f64>> = pieces.into_iter().map(|piece| piece.coeffs).collect();
        for index in 1..coeffs.len() {
            let duration = ends[index - 1] - starts[index - 1];
            let previous = &coeffs[index - 1];
            let shared_position = previous.iter().rev().fold(0.0, |value, &coefficient| {
                nurbs::fmadd(value, duration, coefficient)
            });
            let shared_velocity = previous.iter().enumerate().skip(1).rev().fold(
                0.0,
                |value, (power, &coefficient)| {
                    nurbs::fmadd(value, duration, power as f64 * coefficient)
                },
            );
            coeffs[index][0] = shared_position;
            if coeffs[index].len() == 1 {
                coeffs[index].push(shared_velocity);
            } else {
                coeffs[index][1] = shared_velocity;
            }
        }
        Self {
            starts,
            ends,
            coeffs,
            adv,
            cursor: std::cell::Cell::new(0),
        }
    }

    fn piece_at(&self, t: f64) -> usize {
        let mut i = self.cursor.get().min(self.coeffs.len() - 1);
        while i > 0 && self.starts[i] >= t {
            i -= 1;
        }
        while i + 1 < self.coeffs.len() && self.ends[i] < t {
            i += 1;
        }
        self.cursor.set(i);
        i
    }

    /// Position and the first three derivatives of the monomial piece
    /// covering `t`, by synthetic division: the fold yields the Taylor
    /// coefficients, which scale by `k!` into derivatives.
    fn input_state(&self, t: f64) -> (f64, f64, f64, f64) {
        let i = self.piece_at(t);
        let tau = (t - self.starts[i]).clamp(0.0, self.ends[i] - self.starts[i]);
        let (mut p, mut v, mut a, mut j) = (0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64);
        for &c in self.coeffs[i].iter().rev() {
            j = nurbs::fmadd(j, tau, a);
            a = nurbs::fmadd(a, tau, v);
            v = nurbs::fmadd(v, tau, p);
            p = nurbs::fmadd(p, tau, c);
        }
        (p, v, 2.0 * a, 6.0 * j)
    }

    fn resolution_owned_curvature_velocity(&self, t: f64, velocity: f64, acceleration: f64) -> f64 {
        if acceleration == 0.0 || (velocity / acceleration).abs() >= MIN_DEVICE_INTERVAL_S {
            return velocity;
        }
        let piece = self.piece_at(t);
        if t <= self.ends[piece] && self.ends[piece] - t < MIN_DEVICE_INTERVAL_S {
            return velocity.abs().copysign(-acceleration);
        }
        if t >= self.starts[piece] && t - self.starts[piece] < MIN_DEVICE_INTERVAL_S {
            return velocity.abs().copysign(acceleration);
        }
        velocity
    }

    /// Taylor models of the advanced track's position, velocity and
    /// acceleration over `window` inside input piece `piece`: the formulas
    /// `eval_pva` evaluates, enclosed term by term.
    fn models_on(&self, piece: usize, window: taylor::Window) -> [TaylorModel; 3] {
        let coefficients = &self.coeffs[piece];
        let tau = window.time_since(self.starts[piece]);
        let [position, velocity, acceleration, jerk] =
            [0, 1, 2, 3].map(|order| taylor::derivative_of(coefficients, order, &tau));
        let mut models = nonlinear_advance_models(
            &self.adv,
            &[position, velocity, acceleration],
            &[velocity, acceleration, jerk],
        );
        let (velocity, acceleration) = (&velocity, &acceleration);
        if models[2].rad().is_finite()
            && self.curvature_velocity_may_flip(piece, window, velocity, acceleration)
        {
            let vl = self.adv.linearization_velocity;
            let unit = velocity.scale(Ball::rounded(1.0 / vl, 1));
            let shape = AdvanceShape::of(self.adv.model, &unit)
                .expect("the advance's shape modelled its own velocity");
            let curvature = shape
                .curvature
                .scale(Ball::rounded(self.adv.nonlinear_offset / (vl * vl), 2));
            let curvature_miss = self.adv.nonlinear_offset.abs() / (vl * vl) * shape.branch_miss[2];
            let squared = acceleration.mul(acceleration);
            models[2].widen(2.0 * (curvature.bound() + curvature_miss) * squared.bound());
        }
        models
    }

    /// Whether `resolution_owned_curvature_velocity` may read the velocity
    /// with its other sign somewhere on the window: next to a piece edge,
    /// where the velocity is within a device interval's travel of zero.
    fn curvature_velocity_may_flip(
        &self,
        piece: usize,
        window: taylor::Window,
        velocity: &TaylorModel,
        acceleration: &TaylorModel,
    ) -> bool {
        let near_edge = window.reaches(self.starts[piece], MIN_DEVICE_INTERVAL_S)
            || window.reaches(self.ends[piece], MIN_DEVICE_INTERVAL_S);
        let (centre, spread) = velocity.spread();
        near_edge && centre.abs() - spread < MIN_DEVICE_INTERVAL_S * acceleration.bound()
    }
}

/// Taylor models of `[p, v, a]` advanced by `adv`: the track `[p, v, a]`
/// plus the advance of the motion it reads, `[v, a, j]` — `p + advance(v)`,
/// `v + slope(v)·a`, `a + slope(v)·j + curvature(v)·a²` — each widened by
/// how far the shape may miss a branch its velocity's range reaches.
pub(crate) fn nonlinear_advance_models(
    adv: &trajectory::NonlinearAdvance,
    [position, track_velocity, track_acceleration]: &[TaylorModel; 3],
    [velocity, acceleration, jerk]: &[TaylorModel; 3],
) -> [TaylorModel; 3] {
    let vl = adv.linearization_velocity;
    let unit = velocity.scale(Ball::rounded(1.0 / vl, 1));
    let Some(shape) = AdvanceShape::of(adv.model, &unit) else {
        return [TaylorModel::UNBOUNDED; 3];
    };
    let slope = shape
        .slope
        .scale(Ball::rounded(adv.nonlinear_offset / vl, 1))
        .add_ball(Ball::exact(adv.linear_advance));
    let curvature = shape
        .curvature
        .scale(Ball::rounded(adv.nonlinear_offset / (vl * vl), 2));
    let squared = acceleration.mul(acceleration);
    let mut models = [
        position
            .add(&velocity.scale(Ball::exact(adv.linear_advance)))
            .add(&shape.value.scale(Ball::exact(adv.nonlinear_offset))),
        track_velocity.add(&slope.mul(acceleration)),
        track_acceleration
            .add(&curvature.mul(&squared))
            .add(&slope.mul(jerk)),
    ];
    let offset = adv.nonlinear_offset.abs();
    let [shape_miss, slope_miss, curvature_miss] = shape.branch_miss;
    let slope_miss = offset / vl * slope_miss;
    let curvature_miss = offset / (vl * vl) * curvature_miss;
    models[0].widen(offset * shape_miss);
    models[1].widen(slope_miss * acceleration.bound());
    models[2].widen(curvature_miss * squared.bound() + slope_miss * jerk.bound());
    models
}

/// How far a reciprocal advance's unit velocity may reach past zero onto
/// the branch opposite the one its window is modelled on. Within it the
/// branches `u/(1 ± u)` differ from `u/(1 + |u|)` by at most `3S²`, their
/// slopes by `5S` and their curvatures by `4.1`.
const RECIPROCAL_BRANCH_SLACK: f64 = 1e-9;

/// The unit-scaled advance shape `s(u)`, `s'(u)`, `s''(u)` of a velocity's
/// model, and how far each may miss the shape where the model's range
/// reaches a branch it does not follow.
struct AdvanceShape {
    value: TaylorModel,
    slope: TaylorModel,
    curvature: TaylorModel,
    branch_miss: [f64; 3],
}

impl AdvanceShape {
    fn of(model: trajectory::AdvanceModel, unit: &TaylorModel) -> Option<Self> {
        let one = TaylorModel::constant(Ball::exact(1.0));
        match model {
            trajectory::AdvanceModel::Tanh => {
                let value = unit.tanh();
                let slope = one.sub(&value.mul(&value));
                Some(Self {
                    curvature: value.mul(&slope).scale(Ball::exact(-2.0)),
                    value,
                    slope,
                    branch_miss: [0.0; 3],
                })
            }
            trajectory::AdvanceModel::Reciprocal => {
                let (centre, spread) = unit.spread();
                let subtraction = f64::EPSILON * (centre.abs() + spread);
                let sign = if centre - spread >= -RECIPROCAL_BRANCH_SLACK {
                    1.0
                } else if centre + spread <= RECIPROCAL_BRANCH_SLACK {
                    -1.0
                } else {
                    return None;
                };
                let reaches_other_branch = sign * centre - spread - subtraction < 0.0;
                let reciprocal = unit
                    .scale(Ball::exact(sign))
                    .add_ball(Ball::exact(1.0))
                    .reciprocal()?;
                let slope = reciprocal.mul(&reciprocal);
                Some(Self {
                    value: if sign > 0.0 {
                        one.sub(&reciprocal)
                    } else {
                        reciprocal.sub(&one)
                    },
                    curvature: slope.mul(&reciprocal).scale(Ball::exact(-2.0 * sign)),
                    slope,
                    branch_miss: if reaches_other_branch {
                        [
                            3.0 * RECIPROCAL_BRANCH_SLACK * RECIPROCAL_BRANCH_SLACK,
                            5.0 * RECIPROCAL_BRANCH_SLACK,
                            4.1,
                        ]
                    } else {
                        [0.0; 3]
                    },
                })
            }
        }
    }
}

impl TrackSignal for NonlinearAdvanceSignal {
    fn eval(&self, t: f64) -> f64 {
        let (p, v, _, _) = self.input_state(t);
        p + self.adv.advance(v)
    }

    fn deriv(&self, t: f64) -> f64 {
        let (_, v, a, _) = self.input_state(t);
        self.adv.slope(v).mul_add(a, v)
    }

    fn second_deriv(&self, t: f64) -> f64 {
        let (_, v, a, j) = self.input_state(t);
        let curvature_velocity = self.resolution_owned_curvature_velocity(t, v, a);
        self.adv
            .curvature(curvature_velocity)
            .mul_add(a * a, self.adv.slope(v).mul_add(j, a))
    }

    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        let (p, v, a, j) = self.input_state(t);
        let slope = self.adv.slope(v);
        let curvature_velocity = self.resolution_owned_curvature_velocity(t, v, a);
        (
            p + self.adv.advance(v),
            slope.mul_add(a, v),
            self.adv
                .curvature(curvature_velocity)
                .mul_add(a * a, slope.mul_add(j, a)),
        )
    }

    fn acceleration_monotonicity(
        &self,
        (start, end): (f64, f64),
        (resolved_start, resolved_end): (f64, f64),
    ) -> Option<AccelerationTrend> {
        let piece = self.piece_at(start);
        if self.piece_at(end) != piece
            || self.coeffs[piece]
                .iter()
                .skip(3)
                .any(|&coefficient| coefficient != 0.0)
        {
            return None;
        }
        let (_, start_velocity, acceleration, _) = self.input_state(start);
        let (_, end_velocity, _, _) = self.input_state(end);
        if acceleration == 0.0 || self.adv.nonlinear_offset == 0.0 {
            return Some(AccelerationTrend::Increasing);
        }
        let direction = acceleration * self.adv.nonlinear_offset;
        match self.adv.model {
            trajectory::AdvanceModel::Reciprocal => (start_velocity * end_velocity > 0.0)
                .then(|| AccelerationTrend::of_direction(direction)),
            trajectory::AdvanceModel::Tanh => {
                let extremum = tanh_acceleration_extremum_velocity(self.adv);
                let initial_velocity = self.coeffs[piece][1];
                let first = self.starts[piece] + (-extremum - initial_velocity) / acceleration;
                let second = self.starts[piece] + (extremum - initial_velocity) / acceleration;
                let lower = first.min(second);
                let upper = first.max(second);
                if resolved_end <= lower || resolved_start >= upper {
                    Some(AccelerationTrend::of_direction(direction))
                } else if resolved_start >= lower && resolved_end <= upper {
                    Some(AccelerationTrend::of_direction(-direction))
                } else {
                    None
                }
            }
        }
    }
    fn diagnostic(&self, t: f64) -> Option<String> {
        let piece = self.piece_at(t);
        let (p, v, a, j) = self.input_state(t);
        let curvature_velocity = self.resolution_owned_curvature_velocity(t, v, a);
        Some(format!(
            "nonlinear advance piece {piece}/{} on [{}, {}], input (p, v, a, j)=({p}, {v}, {a}, {j}), \
             slope={}, curvature={}",
            self.coeffs.len(),
            self.starts[piece],
            self.ends[piece],
            self.adv.slope(v),
            self.adv.curvature(curvature_velocity),
        ))
    }

    fn enclosure_on(&self, lo: f64, hi: f64, accuracy: [f64; 3]) -> Option<Enclosure<'_>> {
        let mut pieces = Vec::new();
        let first = self.ends.partition_point(|&end| end <= lo);
        for piece in first..self.coeffs.len() {
            if self.starts[piece] >= hi {
                break;
            }
            let span = (lo.max(self.starts[piece]), hi.min(self.ends[piece]));
            taylor::enclose(
                span,
                accuracy,
                &|window| self.models_on(piece, window),
                &mut pieces,
            );
        }
        Some(Enclosure::Built(pieces))
    }
}

pub(crate) fn apply_derivative_gains_to_track(
    track: &nurbs::ScalarNurbs,
    k1: f64,
    k2: f64,
) -> nurbs::ScalarNurbs {
    let pieces = extract_bezier_pieces(track);
    let mut out_pieces = Vec::with_capacity(pieces.len());
    for piece in &pieces {
        let derivative = piece.differentiate();
        let second = (k2 != 0.0).then(|| derivative.differentiate());
        let coeffs: Vec<f64> = piece
            .coeffs
            .iter()
            .enumerate()
            .map(|(i, c)| {
                let with_k1 = c + k1 * derivative.coeffs.get(i).copied().unwrap_or(0.0);
                match &second {
                    Some(dd) => with_k1 + k2 * dd.coeffs.get(i).copied().unwrap_or(0.0),
                    None => with_k1,
                }
            })
            .collect();

        out_pieces.push(BezierPiece {
            u_start: piece.u_start,
            u_end: piece.u_end,
            coeffs,
        });
    }
    nurbs::bezier::bezier_pieces_to_discontinuous_nurbs(&out_pieces)
}

#[cfg(test)]
#[path = "shaper_test_signals.rs"]
pub(crate) mod test_signals;

#[cfg(test)]
#[path = "shaper_moment_tests.rs"]
mod shaper_moment_tests;

#[cfg(test)]
#[path = "shaper_refinement_tests.rs"]
mod shaper_refinement_tests;

#[cfg(test)]
#[path = "shaper_offset_tests.rs"]
mod shaper_offset_tests;

#[cfg(test)]
#[path = "shaper_storage_tests.rs"]
mod shaper_storage_tests;

#[cfg(test)]
#[path = "shaper_edge_tests.rs"]
mod shaper_edge_tests;

#[cfg(test)]
#[path = "shaper_motor_command_tests.rs"]
mod shaper_motor_command_tests;

#[cfg(test)]
#[path = "shaper_cap_contact_tests.rs"]
mod shaper_cap_contact_tests;

#[cfg(test)]
#[path = "shaper_shaped_wave_tests.rs"]
mod shaper_shaped_wave_tests;

#[cfg(test)]
#[path = "shaper_enclosure_tests.rs"]
mod shaper_enclosure_tests;

#[cfg(test)]
#[path = "shaper_emit_window_tests.rs"]
mod shaper_emit_window_tests;
