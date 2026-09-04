use std::collections::{HashMap, VecDeque};
use std::sync::Arc;

use crossbeam_channel::{Receiver, Sender};
use nurbs::bezier::{BezierPiece, bezier_pieces_to_nurbs, extract_bezier_pieces};
use trajectory::{
    AxisChainSet, ChainStage, CompiledChain, ContinuousAxis, ContinuousSegment,
    RelativeSplinePiece, ShapedSignal, SurfaceMode,
};

use crate::follower_projection::{FollowerState, project_followers};
use crate::lowering::{
    FitTol, LADDER_PROBES_U, LadderFailure, LadderPolicy, exact_piece, ladder_fit, quintic_in_u,
};
use crate::types::{BaseItem, BaseSegment, Control, PostProcessError, TrajectoryItem};

pub(crate) trait TrackSignal {
    fn eval(&self, t: f64) -> f64;
    fn deriv(&self, t: f64) -> f64;
    fn second_deriv(&self, t: f64) -> f64;
    /// The travel between two points the caller has already sampled; a
    /// signal with a better-conditioned delta than the difference of its
    /// samples integrates it from `t0` to `t1` instead.
    fn position_delta(&self, (t0, p0): (f64, f64), (t1, p1): (f64, f64)) -> f64 {
        let _ = (t0, t1);
        p1 - p0
    }
    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        (self.eval(t), self.deriv(t), self.second_deriv(t))
    }
    fn diagnostic(&self, _t: f64) -> Option<String> {
        None
    }
}

impl TrackSignal for ContinuousAxis {
    fn eval(&self, t: f64) -> f64 {
        ContinuousAxis::eval_pva(self, t).map_or(f64::NAN, |pva| pva.position)
    }

    fn deriv(&self, t: f64) -> f64 {
        ContinuousAxis::eval_pva(self, t).map_or(f64::NAN, |pva| pva.velocity)
    }

    fn second_deriv(&self, t: f64) -> f64 {
        ContinuousAxis::eval_pva(self, t).map_or(f64::NAN, |pva| pva.acceleration)
    }
}

impl<F> TrackSignal for ShapedSignal<'_, F>
where
    F: Fn(f64) -> f64,
{
    fn eval(&self, t: f64) -> f64 {
        ShapedSignal::eval_pva(self, t).0
    }

    fn deriv(&self, t: f64) -> f64 {
        ShapedSignal::eval_pva(self, t).1
    }

    fn second_deriv(&self, t: f64) -> f64 {
        ShapedSignal::eval_pva(self, t).2
    }

    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        ShapedSignal::eval_pva(self, t)
    }
}

/// One signal read through a constant position offset: derivatives and
/// travel deltas are the underlying signal's, positions are measured from
/// `base`. Fitting a shifted view keeps every fit source near zero without
/// changing what the convolution means, so the emitted track carries its own
/// relative base instead of an unbounded cumulative one.
pub(crate) struct ShiftedTrackSignal<'a, S: TrackSignal> {
    inner: &'a S,
    base: f64,
}

impl<'a, S: TrackSignal> ShiftedTrackSignal<'a, S> {
    pub(crate) fn new(inner: &'a S, base: f64) -> Self {
        Self { inner, base }
    }
}

impl<S: TrackSignal> TrackSignal for ShiftedTrackSignal<'_, S> {
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

    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        let (position, velocity, acceleration) = self.inner.eval_pva(t);
        (position - self.base, velocity, acceleration)
    }

    fn diagnostic(&self, t: f64) -> Option<String> {
        self.inner.diagnostic(t)
    }
}

pub(crate) const SEGMENT_TIME_EPS_S: f64 = 1e-9;

/// Buffered lowered segments plus the shaped-leader cache: `shaped[i]` is
/// `segments[i]` with every kerneled non-follower axis already fitted. A
/// frontier segment's convolution window is fully covered when it is first
/// fitted, so the fit is final — later emits reuse it bit-identically
/// instead of re-fitting the whole frontier (the projection cache in
/// `follower_projection` relies on the same argument).
#[derive(Default)]
struct PendingSegments {
    segments: VecDeque<ContinuousSegment>,
    rests: VecDeque<bool>,
    shaped: VecDeque<ContinuousSegment>,
}

impl PendingSegments {
    fn push(&mut self, segment: ContinuousSegment, rest_at_end: bool) {
        self.segments.push_back(segment);
        self.rests.push_back(rest_at_end);
    }

    fn clear(&mut self) {
        self.segments.clear();
        self.rests.clear();
        self.shaped.clear();
    }

    fn len(&self) -> usize {
        self.segments.len()
    }

    fn is_empty(&self) -> bool {
        self.segments.is_empty()
    }

    fn ends_at_rest(&self) -> bool {
        self.rests.back() == Some(&true)
    }

    fn back(&self) -> Option<&ContinuousSegment> {
        self.segments.back()
    }

    fn iter(&self) -> impl Iterator<Item = &ContinuousSegment> {
        self.segments.iter()
    }

    fn pop_front(&mut self) -> Option<ContinuousSegment> {
        let segment = self.segments.pop_front();
        let rest = self.rests.pop_front();
        assert_eq!(segment.is_some(), rest.is_some());
        if segment.is_some() && !self.shaped.is_empty() {
            self.shaped.pop_front();
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
}

impl Shaper {
    pub fn new(chains: AxisChainSet, fit_tol: FitTol) -> Self {
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
        }
    }

    /// Mirrors every emitted segment as it stands before the motor-side
    /// derivative-gain stages — the toolhead signal, where the emitted
    /// segments are the motor command.
    #[must_use]
    pub fn with_toolhead_tap(mut self, tap: Sender<ContinuousSegment>) -> Self {
        self.toolhead_tap = Some(tap);
        self
    }

    pub fn run(mut self, input: Receiver<BaseItem>, output: Sender<TrajectoryItem>) {
        let mut deferred = None;
        loop {
            let item = match deferred.take() {
                Some(item) => item,
                None => match input.recv() {
                    Ok(item) => item,
                    Err(_) => {
                        self.finish(&output);
                        return;
                    }
                },
            };
            let BaseItem::Seg(segment) = item else {
                if !self.feed(item, &output) {
                    return;
                }
                continue;
            };
            self.buffer_segment(segment);
            for _ in 1..crate::STAGE_CHANNEL_CAP {
                let Ok(item) = input.try_recv() else {
                    break;
                };
                match item {
                    BaseItem::Seg(segment) => self.buffer_segment(segment),
                    other => {
                        deferred = Some(other);
                        break;
                    }
                }
            }
            if !self.emit(self.supported_count(), false, &output) {
                return;
            }
        }
    }

    fn buffer_segment(&mut self, item: BaseSegment) {
        let mut segment = item.segment;
        let started = crate::timing::stopwatch();
        let rebuilt = materialize_changed_sources(&mut segment, &self.chains, self.fit_tol)
            .unwrap_or_else(|error| panic!("shaper: {error}"));
        let elapsed_us = started.elapsed_us();
        if crate::timing::is_slow_phase(elapsed_us) {
            crate::timing::log_slow_phase(
                "materialize_source",
                elapsed_us,
                crate::timing::PhaseWorkload {
                    segments: 1,
                    axes: rebuilt,
                    ..Default::default()
                },
                "",
            );
        }
        let rest_at_end = segment.rest_at_end;
        self.pending.push(segment, rest_at_end);
    }

    /// One iteration of [`Shaper::run`]'s loop, for single-threaded hosts
    /// that drive the stage item by item.
    pub fn feed(&mut self, item: BaseItem, output: &Sender<TrajectoryItem>) -> bool {
        match item {
            BaseItem::Seg(item) => {
                self.buffer_segment(item);
                self.emit(self.supported_count(), false, output)
            }
            BaseItem::Drain => {
                assert!(
                    self.pending.is_empty() || self.pending.ends_at_rest(),
                    "shaper: drain marker arrived while the trajectory is not at rest"
                );
                self.emit(self.pending.len(), true, output)
                    && output.send(TrajectoryItem::Parked).is_ok()
            }
            BaseItem::Control(ctrl) => {
                match &ctrl {
                    Control::Reset { .. } => {
                        self.pending.clear();
                        self.history.clear();
                        self.history_trimmed = false;
                        for state in &mut self.follower_states {
                            state.reset_timeline();
                        }
                    }
                    Control::Dwell { .. }
                    | Control::SetAxisChains(_)
                    | Control::SetMesh { .. }
                    | Control::Nudge { .. }
                    | Control::Barrier(_) => {
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
                        }
                    }
                }
                output.send(TrajectoryItem::Control(ctrl)).is_ok()
            }
        }
    }

    /// The input-closed path: flush the buffered tail with the window clamped
    /// past the end of the signal.
    pub fn finish(&mut self, output: &Sender<TrajectoryItem>) -> bool {
        self.emit(self.pending.len(), true, output)
    }

    /// How many front segments have their forward convolution window covered
    /// by the buffered lookahead. A follower with its own kernel cascades:
    /// its convolution reads the projection, which is only final through the
    /// shaping frontier — the last pending segment whose directly-convolved
    /// tracks have their own lookahead covered. The time-based bound already
    /// includes the cascaded width, but segment granularity can leave a long
    /// straddling segment unprojectable, so gate on the frontier explicitly.
    ///
    /// Every gate compares `t_end + support` against the covered end with no
    /// slack — the same association and strictness as the fit-time
    /// `MissingLookahead` checks, so a gated segment can never demand
    /// lookahead the buffer does not hold. The lowerer's rest-holds are sized
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
            .take_while(|seg| seg.t_end + self.forward_support <= last_t)
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
            .take_while(|seg| seg.t_end + own_hi <= frontier_t)
            .count();
        plain.min(gated)
    }

    /// End time of the last pending segment whose direct convolution window is
    /// covered by the buffered lookahead.
    fn shaping_frontier_t(&self, last_t: f64) -> Option<f64> {
        let direct_hi = self.chains.direct_forward_support();
        self.pending
            .iter()
            .take_while(|seg| seg.t_end + direct_hi <= last_t)
            .last()
            .map(|seg| seg.t_end)
    }

    fn emit(&mut self, count: usize, force: bool, output: &Sender<TrajectoryItem>) -> bool {
        if count == 0 {
            return true;
        }
        let frontier_count = if self.chains.max_follower_own_forward_support() <= 0.0 {
            count
        } else if force {
            self.pending.len()
        } else {
            let last_t = self.pending.back().expect("count > 0").t_end;
            let direct_hi = self.chains.direct_forward_support();
            self.pending
                .iter()
                .take_while(|seg| seg.t_end + direct_hi <= last_t)
                .count()
                .max(count)
        };
        let pending = &mut self.pending;
        let segments = pending.segments.make_contiguous();
        let shaped = apply_axis_chains(
            &self.history,
            segments,
            count,
            frontier_count,
            force,
            !self.history_trimmed,
            &self.chains,
            self.fit_tol,
            &mut self.follower_states,
            self.toolhead_tap.as_ref(),
            &mut pending.shaped,
        )
        .unwrap_or_else(|e| panic!("shaper: {e}"));
        for seg in shaped {
            if output.send(TrajectoryItem::Seg(seg)).is_err() {
                return false;
            }
        }
        for _ in 0..count {
            let raw = self.pending.pop_front().expect("count <= pending");
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
        // the back-support window, or the window's start falls into a gap
        // whose anchoring segment was just discarded.
        let keep_after = emitted_through - self.back_support;
        while self.history.len() >= 2 && self.history[1].t_start <= keep_after {
            self.history.pop_front();
            self.history_trimmed = true;
        }
        true
    }
}

pub(crate) fn analytic_phase_boundary(span_start: f64, local_boundary: f64) -> f64 {
    let mut boundary = span_start + local_boundary;
    loop {
        let previous = next_toward(boundary, f64::NEG_INFINITY);
        if previous - span_start > local_boundary {
            boundary = previous;
        } else {
            break;
        }
    }
    while boundary - span_start <= local_boundary {
        boundary = next_toward(boundary, f64::INFINITY);
    }
    boundary
}

fn materialize_changed_sources(
    segment: &mut ContinuousSegment,
    chains: &AxisChainSet,
    fit_tol: FitTol,
) -> Result<usize, PostProcessError> {
    let mut replacements = Vec::new();
    for (axis, source) in segment.axes.iter().enumerate() {
        let variable_surface_z = axis == 2
            && matches!(
                source,
                ContinuousAxis::Analytic { span, .. }
                    if matches!(&span.surface, SurfaceMode::Variable(_))
            );
        let changed = chains
            .chains
            .get(axis)
            .is_some_and(|chain| !chain.is_empty())
            && !chains.is_projected_follower(axis);
        if !(variable_surface_z || changed) || matches!(source, ContinuousAxis::Spline(_)) {
            continue;
        }
        let mut breakpoints = vec![segment.t_start, segment.t_end];
        if let ContinuousAxis::Analytic { span, .. } = source {
            breakpoints.extend(
                span.phases
                    .iter()
                    .take(span.phases.len().saturating_sub(1))
                    .map(|phase| analytic_phase_boundary(span.t_start, phase.end_time())),
            );
            if variable_surface_z {
                if let SurfaceMode::Variable(surface) = &span.surface {
                    if let Some(spatial) = span.source.segment.spatial.as_ref() {
                        let transitions =
                            surface.path_transition_distances(spatial).map_err(|_| {
                                fit_tolerance_without_probe(axis, segment.t_start, segment.t_end)
                            })?;
                        for transition in transitions {
                            if let Some(phase) = span.phases.iter().find(|phase| {
                                transition.s >= phase.s0 && transition.s <= phase.end_distance()
                            }) {
                                let local_t =
                                    phase.time_at_distance(transition.s).ok_or_else(|| {
                                        fit_tolerance_without_probe(
                                            axis,
                                            segment.t_start,
                                            segment.t_end,
                                        )
                                    })?;
                                breakpoints.push(span.t_start + local_t);
                            }
                        }
                    }
                }
            }
        }
        let mut curve = fit_axis_from_signal(
            axis,
            segment.t_start,
            segment.t_end,
            &breakpoints,
            source,
            fit_tol,
            "materialize_source",
        )?;
        if !chains.is_projected_follower(axis) {
            if let Some(chain) = chains.chains.get(axis) {
                for stage in &chain.stages {
                    match stage {
                        ChainStage::SmoothKernel(_) => break,
                        ChainStage::DerivativeGains { k1, k2 } => {
                            curve = apply_derivative_gains_to_track(&curve, *k1, *k2);
                        }
                        ChainStage::NonlinearAdvance(advance) => {
                            curve =
                                apply_nonlinear_advance_to_track(axis, &curve, *advance, fit_tol)?;
                        }
                    }
                }
            }
        }
        replacements.push((axis, ContinuousAxis::Spline(Arc::new(curve))));
    }
    let rebuilt = replacements.len();
    let axes = Arc::make_mut(&mut segment.axes);
    for (axis, replacement) in replacements {
        axes[axis] = replacement;
    }
    Ok(rebuilt)
}

#[allow(clippy::too_many_arguments)]
fn apply_axis_chains(
    history: &VecDeque<ContinuousSegment>,
    base: &[ContinuousSegment],
    commit_count: usize,
    frontier_count: usize,
    force: bool,
    at_stream_boundary: bool,
    chains: &AxisChainSet,
    fit_tol: FitTol,
    follower_states: &mut Vec<FollowerState>,
    toolhead_tap: Option<&Sender<ContinuousSegment>>,
    shaped_cache: &mut VecDeque<ContinuousSegment>,
) -> Result<Vec<ContinuousSegment>, PostProcessError> {
    if chains.chains.iter().all(CompiledChain::is_empty)
        && follower_states.iter().all(|s| !s.is_active())
    {
        let out: Vec<ContinuousSegment> = base.iter().take(commit_count).cloned().collect();
        send_toolhead(toolhead_tap, &out);
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
    debug_assert!(
        shaped_cache.len() <= window,
        "frontier retreated below the shaped-leader cache"
    );
    let work = crate::timing::PhaseWorkload {
        window,
        commit: commit_count,
        frontier: frontier_count,
        force,
        ..Default::default()
    };
    if window > shaped_cache.len() {
        let mut fresh: Vec<ContinuousSegment> = base[shaped_cache.len()..window].to_vec();
        fit_leader_axes(
            history,
            base,
            &mut fresh,
            n_axes,
            force,
            at_stream_boundary,
            chains,
            fit_tol,
            work,
        )?;
        shaped_cache.extend(fresh);
    }
    let frontier = &shaped_cache.make_contiguous()[..window];
    let mut out: Vec<ContinuousSegment> = frontier.iter().take(commit_count).cloned().collect();
    let projection_started = crate::timing::stopwatch();
    let mut projection_timing = crate::follower_projection::ProjectionTiming::default();
    project_followers(
        base,
        frontier,
        &mut out,
        commit_count,
        force,
        chains,
        fit_tol,
        follower_states,
        &mut projection_timing,
    )?;
    let projection_us = projection_started.elapsed_us();
    if crate::timing::is_slow_phase(projection_us) {
        crate::timing::log_slow_phase(
            "follower_projection",
            projection_us,
            crate::timing::PhaseWorkload {
                segments: out.len(),
                axes: follower_states.iter().filter(|s| s.is_active()).count(),
                ..work
            },
            &projection_timing.detail(),
        );
    }
    send_toolhead(toolhead_tap, &out);
    let motor_started = crate::timing::stopwatch();
    let (motor_axes, motor_pieces) = apply_motor_side_stages(&mut out, chains, fit_tol)?;
    let motor_us = motor_started.elapsed_us();
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

fn send_toolhead(tap: Option<&Sender<ContinuousSegment>>, out: &[ContinuousSegment]) {
    let Some(tap) = tap else { return };
    for seg in out {
        tap.send(seg.clone())
            .expect("shaper: toolhead tap receiver dropped");
    }
}

/// Trailing derivative-gain stages produce the motor command (e.g. a
/// mode-inverse counter-drive), which the physical toolhead does not follow —
/// that is their entire purpose. They are therefore applied only after the
/// followers have projected onto the toolhead signal.
fn apply_motor_side_stages(
    out: &mut [ContinuousSegment],
    chains: &AxisChainSet,
    fit_tol: FitTol,
) -> Result<(usize, usize), PostProcessError> {
    let mut rebuilt_axes = 0usize;
    let mut rebuilt_pieces = 0usize;
    for seg in out.iter_mut() {
        for axis in 0..seg.axes.len() {
            let Some(chain) = chains.chains.get(axis) else {
                continue;
            };
            let follower_linear_chain = chains.is_projected_follower(axis)
                && chain
                    .stages
                    .iter()
                    .any(|stage| matches!(stage, ChainStage::SmoothKernel(_)))
                && chain.stages.iter().all(|stage| {
                    matches!(
                        stage,
                        ChainStage::DerivativeGains { .. } | ChainStage::SmoothKernel(_)
                    )
                });
            if follower_linear_chain {
                continue;
            }
            if !chain.has_motor_side_gains() {
                continue;
            }
            let replacement = match &seg.axes[axis] {
                ContinuousAxis::Spline(track) => ContinuousAxis::Spline(Arc::new(
                    apply_trailing_zero_support(chain, axis, (**track).clone(), fit_tol)?,
                )),
                ContinuousAxis::RelativeSpline {
                    base_position,
                    curve,
                } => ContinuousAxis::RelativeSpline {
                    base_position: *base_position,
                    curve: Arc::new(apply_trailing_zero_support(
                        chain,
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
        let curve = Arc::new(apply_trailing_zero_support(
            chain,
            axis,
            (*piece.curve).clone(),
            fit_tol,
        )?);
        let base_position = out.last().map_or(piece.base_position, |previous| {
            previous.base_position + nurbs::eval::eval(&previous.curve.as_view(), previous.t_end)
                - nurbs::eval::eval(&curve.as_view(), piece.t_start)
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
) -> Option<Vec<nurbs::ScalarNurbs>> {
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
            ContinuousAxis::Spline(curve) => Some((**curve).clone()),
            _ => None,
        })
        .collect()
}

fn fit_axis_column(
    history: &VecDeque<ContinuousSegment>,
    base: &[ContinuousSegment],
    targets: &[ContinuousSegment],
    axis: usize,
    force: bool,
    at_stream_boundary: bool,
    target_workers: usize,
    chain: &CompiledChain,
    fit_tol: FitTol,
) -> Result<Option<Vec<nurbs::ScalarNurbs>>, PostProcessError> {
    let Some(kernel) = chain.stages.iter().find_map(|stage| match stage {
        ChainStage::SmoothKernel(kernel) => Some(kernel),
        ChainStage::DerivativeGains { .. } | ChainStage::NonlinearAdvance(_) => None,
    }) else {
        return Ok(None);
    };
    let (k_lo, k_hi) = kernel.support();
    let first_t = history
        .front()
        .or_else(|| base.first())
        .map_or(0.0, |seg| seg.t_start);
    let last_t = base.last().map_or(first_t, |seg| seg.t_end);
    let signal_segments: Vec<&ContinuousSegment> = history.iter().chain(base.iter()).collect();
    for seg in targets {
        let need_lo = seg.t_start - k_hi;
        let need_hi = seg.t_end - k_lo;
        if need_lo < first_t && !at_stream_boundary {
            return Err(PostProcessError::MissingHistory { axis, t: need_lo });
        }
        if need_hi > last_t && !force {
            return Err(PostProcessError::MissingLookahead { axis, t: need_hi });
        }
    }
    if let Some(column) = constant_axis_column(&signal_segments, targets, axis) {
        return Ok(Some(column));
    }
    let input_breaks = signal_breakpoints(&signal_segments, axis);
    let shaped_breaks = shaped_signal_breakpoints(kernel, &input_breaks);
    let kernel_degree = kernel
        .pieces
        .iter()
        .map(|piece| piece.degree())
        .max()
        .expect("shaper kernel has no pieces");
    let table = Arc::new(
        AxisSignalTable::build(
            &signal_segments,
            axis,
            first_t,
            last_t,
            at_stream_boundary,
            force,
        )
        .with_piece_moments(kernel_degree),
    );
    let input_degree = table.max_degree();
    fit_axis_targets(
        axis,
        targets,
        kernel,
        table,
        input_breaks,
        shaped_breaks,
        input_degree,
        fit_tol,
        target_workers,
    )
    .map(Some)
}

fn fit_axis_targets(
    axis: usize,
    targets: &[ContinuousSegment],
    kernel: &nurbs::algebra::PiecewisePolynomialKernel,
    table: Arc<AxisSignalTable>,
    input_breaks: Vec<f64>,
    shaped_breaks: Vec<f64>,
    input_degree: usize,
    fit_tol: FitTol,
    target_workers: usize,
) -> Result<Vec<nurbs::ScalarNurbs>, PostProcessError> {
    if targets.is_empty() {
        return Ok(Vec::new());
    }
    let make_sig = || {
        let eval_table = Arc::clone(&table);
        let moment_table = Arc::clone(&table);
        let piece_hint = std::cell::Cell::new(0);
        ShapedSignal::new_from_polynomial_evaluator(
            kernel,
            move |t| eval_table.eval_hinted(t, &piece_hint),
            input_breaks.clone(),
            input_degree,
            move |lo, hi, degree, origin, moments| {
                moment_table.integrate_moments(lo, hi, degree, origin, moments)
            },
        )
    };
    let max_seed_spans = targets
        .iter()
        .map(|target| {
            let first = shaped_breaks.partition_point(|time| *time <= target.t_start);
            let last = shaped_breaks.partition_point(|time| *time < target.t_end);
            last.saturating_sub(first) + 1
        })
        .max()
        .unwrap_or(0);
    if max_seed_spans < LEADER_SPAN_PARALLEL_THRESHOLD {
        let fit_target = |target: &ContinuousSegment, sig: &_| {
            let track = fit_axis_from_signal(
                axis,
                target.t_start,
                target.t_end,
                &shaped_breaks,
                sig,
                fit_tol,
                "smooth_kernel_target",
            )?;
            if !track.control_points().iter().all(|value| value.is_finite()) {
                return Err(PostProcessError::NonFiniteSample {
                    axis,
                    t: target.t_start,
                });
            }
            Ok(track)
        };
        let workers = target_workers.max(1).min(targets.len());
        let mut fitted = if workers > 1 {
            let next_target = std::sync::atomic::AtomicUsize::new(0);
            std::thread::scope(|scope| {
                let next_target = &next_target;
                let fit_target = &fit_target;
                let handles: Vec<_> = (0..workers)
                    .map(|_| {
                        scope.spawn(move || {
                            let sig = make_sig();
                            let mut done = Vec::new();
                            loop {
                                let index =
                                    next_target.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                                let Some(target) = targets.get(index) else {
                                    return done;
                                };
                                done.push((index, fit_target(target, &sig)));
                            }
                        })
                    })
                    .collect();
                handles
                    .into_iter()
                    .flat_map(|handle| handle.join().expect("leader target fit thread panicked"))
                    .collect::<Vec<_>>()
            })
        } else {
            let sig = make_sig();
            targets
                .iter()
                .enumerate()
                .map(|(index, target)| (index, fit_target(target, &sig)))
                .collect()
        };
        fitted.sort_by_key(|(index, _)| *index);
        return fitted.into_iter().map(|(_, result)| result).collect();
    }
    let plan_started = crate::timing::stopwatch();
    let seed_sets = targets
        .iter()
        .map(|target| fit_seed_times(axis, target.t_start, target.t_end, &shaped_breaks))
        .collect::<Result<Vec<_>, _>>()?;
    let mut scale_times = seed_sets
        .iter()
        .flat_map(|seeds| seeds.iter().copied())
        .collect::<Vec<_>>();
    scale_times.sort_by(f64::total_cmp);
    scale_times.dedup();
    let plan_workers = target_workers.max(1).min(scale_times.len());
    let scale_samples = if plan_workers > 1 {
        let next_time = std::sync::atomic::AtomicUsize::new(0);
        std::thread::scope(|scope| {
            let next_time = &next_time;
            let scale_times = &scale_times;
            let handles: Vec<_> = (0..plan_workers)
                .map(|_| {
                    scope.spawn(move || {
                        let sig = make_sig();
                        let mut done = Vec::new();
                        loop {
                            let index =
                                next_time.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                            let Some(&t) = scale_times.get(index) else {
                                return done;
                            };
                            let (position, velocity, _) = sig.eval_pva(t);
                            done.push((t.to_bits(), (position, velocity)));
                        }
                    })
                })
                .collect();
            handles
                .into_iter()
                .flat_map(|handle| handle.join().expect("leader scale probe thread panicked"))
                .collect::<Vec<_>>()
        })
    } else {
        let sig = make_sig();
        scale_times
            .iter()
            .map(|&t| {
                let (position, velocity, _) = sig.eval_pva(t);
                (t.to_bits(), (position, velocity))
            })
            .collect()
    };
    let scale_cache = scale_samples.into_iter().collect::<HashMap<_, _>>();
    let mut plans = Vec::with_capacity(targets.len());
    for (target, seeds) in targets.iter().zip(seed_sets) {
        let (mut position_scale, mut velocity_scale) = (0.0_f64, 0.0_f64);
        for &t in &seeds {
            let &(position, velocity) = scale_cache
                .get(&t.to_bits())
                .expect("leader scale probe is missing");
            if position.is_finite() {
                position_scale = position_scale.max(position.abs());
            }
            if velocity.is_finite() {
                velocity_scale = velocity_scale.max(velocity.abs());
            }
        }
        let time_scale = target.t_start.abs().max(target.t_end.abs());
        let floors = ladder_resolution_floor_from_scales(
            position_scale,
            velocity_scale,
            time_scale,
            fit_tol,
        );
        plans.push(fit_plan_from_seeds(
            seeds,
            floors,
            target.t_start,
            target.t_end,
        ));
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
            axis,
            sig,
            t_start,
            t_end,
            fit_tol,
            "smooth_kernel_target",
            f64::INFINITY,
            plan.floors,
            plan.max_depth,
            0,
            t_start,
            t_end,
            &mut refinement_splits,
            &mut pieces,
        )
        .map(|()| (pieces, refinement_splits))
    };
    type SpanFit = Result<(Vec<BezierPiece>, usize), PostProcessError>;
    let workers = target_workers.max(1).min(jobs.len());
    let spans_started = crate::timing::stopwatch();
    let mut fitted: Vec<(usize, usize, SpanFit)> = if workers > 1 {
        let next_job = std::sync::atomic::AtomicUsize::new(0);
        std::thread::scope(|scope| {
            let next_job = &next_job;
            let fit_span = &fit_span;
            let jobs = &jobs;
            let handles: Vec<_> = (0..workers)
                .map(|_| {
                    scope.spawn(move || {
                        let sig = make_sig();
                        let mut done = Vec::new();
                        loop {
                            let index = next_job.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                            let Some(&(target_index, span_index)) = jobs.get(index) else {
                                return done;
                            };
                            done.push((
                                target_index,
                                span_index,
                                fit_span(target_index, span_index, &sig),
                            ));
                        }
                    })
                })
                .collect();
            handles
                .into_iter()
                .flat_map(|handle| handle.join().expect("leader span fit thread panicked"))
                .collect()
        })
    } else {
        let sig = make_sig();
        jobs.iter()
            .map(|&(target_index, span_index)| {
                (
                    target_index,
                    span_index,
                    fit_span(target_index, span_index, &sig),
                )
            })
            .collect()
    };
    fitted.sort_by_key(|(target_index, span_index, _)| (*target_index, *span_index));
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
        let track = if needs_serial_fit || refinement_splits > MAX_FIT_REFINEMENT_SPLITS {
            let sig = make_sig();
            fit_axis_from_signal(
                axis,
                targets[target_index].t_start,
                targets[target_index].t_end,
                &shaped_breaks,
                &sig,
                fit_tol,
                "smooth_kernel_target",
            )?
        } else {
            fit_pieces_to_nurbs(axis, pieces)?
        };
        if !track.control_points().iter().all(|value| value.is_finite()) {
            return Err(PostProcessError::NonFiniteSample {
                axis,
                t: targets[target_index].t_start,
            });
        }
        tracks.push(track);
    }
    Ok(tracks)
}

fn fit_leader_axes(
    history: &VecDeque<ContinuousSegment>,
    base: &[ContinuousSegment],
    fresh: &mut [ContinuousSegment],
    n_axes: usize,
    force: bool,
    at_stream_boundary: bool,
    chains: &AxisChainSet,
    fit_tol: FitTol,
    work: crate::timing::PhaseWorkload,
) -> Result<(), PostProcessError> {
    let default_chain = CompiledChain::default();
    let axis_chains: Vec<(usize, &CompiledChain)> = (0..n_axes)
        .filter(|&axis| !chains.is_projected_follower(axis))
        .map(|axis| (axis, chains.chains.get(axis).unwrap_or(&default_chain)))
        .filter(|(_, chain)| !chain.is_empty())
        .collect();
    let fit_started = crate::timing::stopwatch();
    type TimedColumn = (
        usize,
        Result<Option<Vec<nurbs::ScalarNurbs>>, PostProcessError>,
        u128,
    );
    let parallel = cfg!(not(target_arch = "wasm32")) && axis_chains.len() > 1 && !fresh.is_empty();
    let target_workers = if cfg!(target_arch = "wasm32") || fresh.len() < 4 {
        1
    } else {
        std::thread::available_parallelism()
            .map_or(1, |cores| cores.get())
            .div_ceil(axis_chains.len().max(1))
            .min(4)
            .min(fresh.len())
    };
    let columns: Vec<TimedColumn> = if parallel {
        let fresh_ref: &[ContinuousSegment] = fresh;
        std::thread::scope(|scope| {
            let handles: Vec<_> = axis_chains
                .iter()
                .map(|&(axis, chain)| {
                    scope.spawn(move || {
                        let column_started = crate::timing::stopwatch();
                        let column = fit_axis_column(
                            history,
                            base,
                            fresh_ref,
                            axis,
                            force,
                            at_stream_boundary,
                            target_workers,
                            chain,
                            fit_tol,
                        );
                        (axis, column, column_started.elapsed_us())
                    })
                })
                .collect();
            handles
                .into_iter()
                .map(|h| h.join().expect("axis fit thread panicked"))
                .collect()
        })
    } else {
        axis_chains
            .iter()
            .map(|&(axis, chain)| {
                let column_started = crate::timing::stopwatch();
                let column = fit_axis_column(
                    history,
                    base,
                    fresh,
                    axis,
                    force,
                    at_stream_boundary,
                    target_workers,
                    chain,
                    fit_tol,
                );
                (axis, column, column_started.elapsed_us())
            })
            .collect()
    };
    let fit_elapsed_us = fit_started.elapsed_us();
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
                axes: axis_chains.len(),
                force,
                ..work
            },
            &per_axis.join(" "),
        );
    }
    for (axis, column, _) in columns {
        let Some(column) = column? else { continue };
        for (seg, track) in fresh.iter_mut().zip(column) {
            Arc::make_mut(&mut seg.axes)[axis] = ContinuousAxis::Spline(Arc::new(track));
        }
    }
    Ok(())
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

pub(crate) fn shaped_signal_breakpoints(
    kernel: &nurbs::algebra::PiecewisePolynomialKernel,
    input_breaks: &[f64],
) -> Vec<f64> {
    let kernel_breaks: Vec<f64> = ShapedSignal::kernel_cut_boundaries(kernel).collect();
    let mut breaks = Vec::with_capacity(input_breaks.len() * (kernel_breaks.len() + 2));
    for &input_break in input_breaks {
        for &kernel_break in &kernel_breaks {
            ShapedSignal::output_cut_transitions(kernel, input_break, kernel_break, &mut breaks);
        }
    }
    breaks.sort_by(f64::total_cmp);
    breaks.dedup();
    breaks
}

/// Sized for the deepest fit-ladder rung (degree 13, 14 coefficients) times
/// the widest kernel moment window the shaper builds.
const MOMENT_POWER_CAPACITY: usize = 24;

fn accumulate_translated_moments(
    source_origin: f64,
    source: &[f64],
    target_origin: f64,
    target: &mut [f64],
) {
    assert_eq!(source.len(), target.len());
    assert!(target.len() <= MOMENT_POWER_CAPACITY);
    let delta = source_origin - target_origin;
    let mut delta_powers = [1.0; MOMENT_POWER_CAPACITY];
    for power in 1..target.len() {
        delta_powers[power] = delta_powers[power - 1] * delta;
    }
    for (moment_power, moment) in target.iter_mut().enumerate() {
        let mut choose = 1.0;
        for source_power in 0..=moment_power {
            *moment += choose * delta_powers[moment_power - source_power] * source[source_power];
            if source_power < moment_power {
                choose *= (moment_power - source_power) as f64 / (source_power + 1) as f64;
            }
        }
    }
}

fn taylor_shift(coefficients: &[f64], shift: f64, shifted: &mut [f64]) {
    assert_eq!(coefficients.len(), shifted.len());
    assert!(coefficients.len() <= MOMENT_POWER_CAPACITY);
    let mut work = [0.0; MOMENT_POWER_CAPACITY];
    work[..coefficients.len()].copy_from_slice(coefficients);
    let mut remaining = coefficients.len();
    for target in shifted.iter_mut() {
        for power in (1..remaining).rev() {
            work[power - 1] = nurbs::fmadd(shift, work[power], work[power - 1]);
        }
        *target = work[0];
        work.copy_within(1..remaining, 0);
        remaining -= 1;
    }
}

struct MomentTree {
    degree: usize,
    leaf_count: usize,
    origins: Vec<f64>,
    moments: Vec<f64>,
}

impl MomentTree {
    fn build(table: &AxisSignalTable, degree: usize) -> Self {
        let stride = degree + 1;
        assert!(
            stride <= MOMENT_POWER_CAPACITY,
            "moment degree {degree} exceeds capacity {}",
            MOMENT_POWER_CAPACITY - 1
        );
        let leaf_count = table.coeffs.len().next_power_of_two();
        let node_count = 2 * leaf_count;
        let mut origins = vec![f64::NAN; node_count];
        let mut moments = vec![0.0; node_count * stride];
        for i in 0..table.coeffs.len() {
            let node = leaf_count + i;
            let origin = table.starts[i] + 0.5 * (table.ends[i] - table.starts[i]);
            origins[node] = origin;
            AxisSignalTable::accumulate_polynomial_moments(
                &table.coeffs[i],
                table.starts[i],
                table.starts[i],
                table.ends[i],
                origin,
                &mut moments[node * stride..(node + 1) * stride],
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
            let mut combined = [0.0; MOMENT_POWER_CAPACITY];
            for child in [left, right] {
                if origins[child].is_finite() {
                    accumulate_translated_moments(
                        origins[child],
                        &moments[child * stride..(child + 1) * stride],
                        origin,
                        &mut combined[..stride],
                    );
                }
            }
            moments[node * stride..(node + 1) * stride].copy_from_slice(&combined[..stride]);
        }
        Self {
            degree,
            leaf_count,
            origins,
            moments,
        }
    }

    fn accumulate(&self, start: usize, end: usize, degree: usize, origin: f64, target: &mut [f64]) {
        if start >= end {
            return;
        }
        let mut left = start + self.leaf_count;
        let mut right = end + self.leaf_count;
        let mut right_nodes = [0usize; usize::BITS as usize];
        let mut right_count = 0;
        while left < right {
            if left & 1 != 0 {
                self.accumulate_node(left, degree, origin, target);
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
            self.accumulate_node(right_nodes[right_count], degree, origin, target);
        }
    }

    fn accumulate_node(&self, node: usize, degree: usize, origin: f64, target: &mut [f64]) {
        let stride = self.degree + 1;
        accumulate_translated_moments(
            self.origins[node],
            &self.moments[node * stride..node * stride + degree + 1],
            origin,
            target,
        );
    }
}
/// One axis of the emit window, flattened to time-sorted monomial pieces so
/// the convolution's Gauss sampling evaluates by binary search plus Horner
/// instead of a de Boor pass per sample. Semantics match the NURBS window it
/// is built from: before `first_t` the signal clamps at a stream boundary
/// and is otherwise missing (NaN, surfaced as `MissingHistory`); past
/// `last_t` it clamps under a drain flush and is otherwise missing; a time
/// gap between segments holds the value the preceding piece ends at.
pub(crate) struct AxisSignalTable {
    starts: Vec<f64>,
    ends: Vec<f64>,
    coeffs: Vec<Vec<f64>>,
    first_t: f64,
    last_t: f64,
    at_stream_boundary: bool,
    force: bool,
    moment_tree: Option<MomentTree>,
}

impl AxisSignalTable {
    fn build(
        segments: &[&ContinuousSegment],
        axis: usize,
        first_t: f64,
        last_t: f64,
        at_stream_boundary: bool,
        force: bool,
    ) -> Self {
        Self::from_tracks(
            segments.iter().map(|seg| match &seg.axes[axis] {
                ContinuousAxis::Spline(track) => track.as_ref(),
                _ => panic!("shaper: changed axis was not materialized"),
            }),
            first_t,
            last_t,
            at_stream_boundary,
            force,
        )
    }

    pub(crate) fn from_tracks<'a>(
        tracks: impl IntoIterator<Item = &'a nurbs::ScalarNurbs>,
        first_t: f64,
        last_t: f64,
        at_stream_boundary: bool,
        force: bool,
    ) -> Self {
        let mut table = Self {
            starts: Vec::new(),
            ends: Vec::new(),
            coeffs: Vec::new(),
            first_t,
            last_t,
            at_stream_boundary,
            force,
            moment_tree: None,
        };
        for track in tracks {
            for piece in extract_bezier_pieces(track) {
                if let Some(previous_end) = table.ends.last().copied() {
                    assert!(
                        piece.u_start >= previous_end,
                        "signal pieces overlap: next starts at {} before previous end {previous_end}",
                        piece.u_start
                    );
                    if piece.u_start > previous_end {
                        let previous = table.coeffs.len() - 1;
                        let held = table.piece_at(previous, previous_end);
                        table.starts.push(previous_end);
                        table.ends.push(piece.u_start);
                        table.coeffs.push(vec![held]);
                    }
                }
                table.starts.push(piece.u_start);
                table.ends.push(piece.u_end);
                table.coeffs.push(piece.coeffs);
            }
        }
        assert!(!table.coeffs.is_empty(), "empty signal window");
        table
    }

    pub(crate) fn max_degree(&self) -> usize {
        self.coeffs
            .iter()
            .map(|coefficients| coefficients.len() - 1)
            .max()
            .expect("empty signal window")
    }
    pub(crate) fn with_piece_moments(mut self, degree: usize) -> Self {
        self.moment_tree = Some(MomentTree::build(&self, degree));
        self
    }
    pub(crate) fn integrate_moments(
        &self,
        lo: f64,
        hi: f64,
        degree: usize,
        origin: f64,
        moments: &mut [f64],
    ) -> bool {
        assert_eq!(moments.len(), degree + 1);
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
        if (lo < self.first_t && !self.at_stream_boundary) || (hi > self.last_t && !self.force) {
            return false;
        }
        moments.fill(0.0);
        if lo < self.first_t {
            let held = [self.piece_at(0, self.first_t)];
            Self::accumulate_polynomial_moments(
                &held,
                lo,
                lo,
                hi.min(self.first_t),
                origin,
                moments,
            );
        }
        let interior_lo = lo.max(self.first_t);
        let interior_hi = hi.min(self.last_t);
        if interior_hi > interior_lo {
            let first_piece = self.ends.partition_point(|&end| end <= interior_lo);
            let end_piece = self.starts.partition_point(|&start| start < interior_hi);
            let mut full_start = first_piece;
            let mut full_end = end_piece;
            if self.starts[first_piece] < interior_lo {
                Self::accumulate_polynomial_moments(
                    &self.coeffs[first_piece],
                    self.starts[first_piece],
                    interior_lo,
                    interior_hi.min(self.ends[first_piece]),
                    origin,
                    moments,
                );
                full_start += 1;
            }
            let right_partial = if full_start < full_end && self.ends[full_end - 1] > interior_hi {
                full_end -= 1;
                Some(full_end)
            } else {
                None
            };
            tree.accumulate(full_start, full_end, degree, origin, moments);
            if let Some(piece) = right_partial {
                Self::accumulate_polynomial_moments(
                    &self.coeffs[piece],
                    self.starts[piece],
                    self.starts[piece],
                    interior_hi,
                    origin,
                    moments,
                );
            }
        }
        if hi > self.last_t {
            let held = [self.piece_at(self.coeffs.len() - 1, self.last_t)];
            Self::accumulate_polynomial_moments(
                &held,
                self.last_t,
                lo.max(self.last_t),
                hi,
                origin,
                moments,
            );
        }
        true
    }

    fn accumulate_polynomial_moments(
        coefficients: &[f64],
        polynomial_start: f64,
        lo: f64,
        hi: f64,
        origin: f64,
        moments: &mut [f64],
    ) {
        if hi <= lo {
            return;
        }
        let max_power = coefficients.len() + moments.len() - 1;
        assert!(
            max_power < MOMENT_POWER_CAPACITY,
            "moment product degree {} exceeds capacity {}",
            max_power - 1,
            MOMENT_POWER_CAPACITY - 2
        );
        let base = lo + 0.5 * (hi - lo);
        let mut local_coefficients = [0.0; MOMENT_POWER_CAPACITY];
        taylor_shift(
            coefficients,
            base - polynomial_start,
            &mut local_coefficients[..coefficients.len()],
        );
        let mut lo_powers = [1.0; MOMENT_POWER_CAPACITY];
        let mut hi_powers = [1.0; MOMENT_POWER_CAPACITY];
        let x_lo = lo - base;
        let x_hi = hi - base;
        for power in 1..=max_power {
            lo_powers[power] = lo_powers[power - 1] * x_lo;
            hi_powers[power] = hi_powers[power - 1] * x_hi;
        }
        let mut local_moments = [0.0; MOMENT_POWER_CAPACITY];
        for (moment_power, moment) in local_moments[..moments.len()].iter_mut().enumerate() {
            for (polynomial_power, coefficient) in local_coefficients
                .iter()
                .copied()
                .take(coefficients.len())
                .enumerate()
            {
                let power = polynomial_power + moment_power + 1;
                *moment += coefficient * (hi_powers[power] - lo_powers[power]) / power as f64;
            }
        }
        accumulate_translated_moments(base, &local_moments[..moments.len()], origin, moments);
    }

    fn piece_at(&self, i: usize, t: f64) -> f64 {
        let tau = (t - self.starts[i]).clamp(0.0, self.ends[i] - self.starts[i]);
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
            if !self.at_stream_boundary {
                return f64::NAN;
            }
            return self.piece_at(0, self.first_t);
        }
        if t > self.last_t {
            if !self.force {
                return f64::NAN;
            }
            return self.piece_at(self.coeffs.len() - 1, self.last_t);
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
const MIN_DEVICE_INTERVAL_S: f64 = 2e-9;

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
            position_scale = position_scale.max(position.abs());
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
    let representable = MIN_DEVICE_INTERVAL_S.max(8.0 * f64::EPSILON * time_scale);
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
/// routine rather than exceptional. Each cluster collapses to its rightmost
/// member — the transitions are right-owned, and the sub-ulp interval that
/// gets absorbed into the span on the left carries no integral. `t_start` and
/// `t_end` are pinned, so a cluster touching the start keeps the start.
fn coalesce_degenerate_seeds(seeds: &mut Vec<f64>, minimum_span: f64) {
    let t_end = *seeds.last().expect("fit seeds are empty");
    let mut kept = Vec::with_capacity(seeds.len());
    kept.push(seeds[0]);
    for &seed in &seeds[1..seeds.len() - 1] {
        if seed - *kept.last().expect("kept seeds are empty") >= minimum_span {
            kept.push(seed);
        } else if kept.len() > 1 {
            kept.pop();
            kept.push(seed);
        }
    }
    while kept.len() > 1 && t_end - *kept.last().expect("kept seeds are empty") < minimum_span {
        kept.pop();
    }
    kept.push(t_end);
    *seeds = kept;
}

struct FitPlan {
    seeds: Vec<f64>,
    floors: LadderFloors,
    max_depth: u32,
}

fn fit_seed_times(
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
    floors: LadderFloors,
    t_start: f64,
    t_end: f64,
) -> FitPlan {
    coalesce_degenerate_seeds(&mut seeds, floors.cubic);
    let max_depth = libm::log2((t_end - t_start) / floors.cubic).ceil().max(0.0) as u32;
    FitPlan {
        seeds,
        floors,
        max_depth,
    }
}

fn prepare_fit_plan<S: TrackSignal>(
    axis: usize,
    t_start: f64,
    t_end: f64,
    seed_breakpoints: &[f64],
    sig: &S,
    fit_tol: FitTol,
) -> Result<FitPlan, PostProcessError> {
    let seeds = fit_seed_times(axis, t_start, t_end, seed_breakpoints)?;
    let time_scale = t_start.abs().max(t_end.abs());
    let floors = ladder_resolution_floor(sig, &seeds, time_scale, fit_tol);
    Ok(fit_plan_from_seeds(seeds, floors, t_start, t_end))
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

pub(crate) fn fit_axis_from_signal<S: TrackSignal>(
    axis: usize,
    t_start: f64,
    t_end: f64,
    seed_breakpoints: &[f64],
    sig: &S,
    fit_tol: FitTol,
    fit_context: &'static str,
) -> Result<nurbs::ScalarNurbs, PostProcessError> {
    fit_axis_from_signal_with_velocity_budget(
        axis,
        t_start,
        t_end,
        seed_breakpoints,
        sig,
        fit_tol,
        fit_context,
        f64::INFINITY,
    )
}

fn fit_axis_from_signal_with_velocity_budget<S: TrackSignal>(
    axis: usize,
    t_start: f64,
    t_end: f64,
    seed_breakpoints: &[f64],
    sig: &S,
    fit_tol: FitTol,
    fit_context: &'static str,
    velocity_budget: f64,
) -> Result<nurbs::ScalarNurbs, PostProcessError> {
    let plan = prepare_fit_plan(axis, t_start, t_end, seed_breakpoints, sig, fit_tol)?;
    let mut refinement_splits = 0;
    let mut pieces = Vec::with_capacity(plan.seeds.len());
    for span in plan.seeds.windows(2) {
        refine_shaped_span(
            axis,
            sig,
            span[0],
            span[1],
            fit_tol,
            fit_context,
            velocity_budget,
            plan.floors,
            plan.max_depth,
            0,
            span[0],
            span[1],
            &mut refinement_splits,
            &mut pieces,
        )?;
    }
    fit_pieces_to_nurbs(axis, pieces)
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

const LADDER_FIT_NODES_U: [f64; 3] = [0.0, 0.5, -0.5];

/// Ladder fit of the shaped signal over one span: the endpoint-anchored
/// quadratic carrying the span's own travel, the cubic through both endpoint
/// velocities, then the quintic Hermite matching the convolution's exact
/// (p, v, a) at both ends and degrees 6/7 from interior residuals — the
/// lowering ladder against pre-sampled truth, with the shaper's own budgets.
/// Every accepted piece is endpoint-anchored, so the span seams stay C0 no
/// matter which rung wins. Returns the accepted monomial-in-u fit, or the
/// quintic base with `fits = false` so the caller can bisect.
#[allow(clippy::type_complexity)]
fn shaped_ladder<S: TrackSignal>(
    axis: usize,
    sig: &S,
    t0: f64,
    t1: f64,
    fit_tol: FitTol,
    velocity_budget: f64,
    enforce_velocity_sign: bool,
    high_degree_span_floor: f64,
) -> Result<(Vec<f64>, Option<LadderFailure>), PostProcessError> {
    let h = t1 - t0;
    let t_of = |u: f64| nurbs::fmadd(0.5 * (u + 1.0), h, t0);
    let interior_t_of = |u: f64| {
        if u == -1.0 {
            next_toward(t0, t1)
        } else if u == 1.0 {
            next_toward(t1, t0)
        } else {
            t_of(u)
        }
    };
    let t0_inside = interior_t_of(-1.0);
    let p0 = exact_value(axis, sig.eval(t0), t0)?;
    let (_, v0, a0) = sig.eval_pva(t0_inside);
    let v0 = exact_value(axis, v0, t0_inside)?;
    let a0 = exact_value(axis, a0, t0_inside)?;
    let t1_inside = interior_t_of(1.0);
    let p1 = exact_value(axis, sig.eval(t1), t1)?;
    let (_, v1, a1) = sig.eval_pva(t1_inside);
    let v1 = exact_value(axis, v1, t1_inside)?;
    let a1 = exact_value(axis, a1, t1_inside)?;
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
                let (pos, vel, acc) = sig.eval_pva(t);
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
            None => finite_sample(axis, sig, t_of(u))?,
        };
    }

    match ladder_fit(
        &base,
        h,
        fit_tol,
        &|u| truth.pos_at(u),
        &|u| truth.acc_at(u),
        &|u| truth.vel_at(u),
        endpoint_delta,
        velocity_budget,
        LadderPolicy {
            endpoint_anchored: true,
            enforce_velocity_sign,
            high_degree_span_floor,
        },
    ) {
        Ok(coefficients) => Ok((coefficients, None)),
        Err(failure) => Ok((base, Some(failure))),
    }
}

fn next_toward(value: f64, toward: f64) -> f64 {
    if value == toward {
        return value;
    }
    if value == 0.0 {
        return if toward > 0.0 {
            f64::from_bits(1)
        } else {
            f64::from_bits((1_u64 << 63) | 1)
        };
    }
    let bits = value.to_bits();
    if (toward > value) == (value > 0.0) {
        f64::from_bits(bits + 1)
    } else {
        f64::from_bits(bits - 1)
    }
}

fn refine_shaped_span<S: TrackSignal>(
    axis: usize,
    sig: &S,
    t0: f64,
    t1: f64,
    fit_tol: FitTol,
    fit_context: &'static str,
    velocity_budget: f64,
    floors: LadderFloors,
    max_depth: u32,
    depth: u32,
    lower_seed: f64,
    upper_seed: f64,
    refinement_splits: &mut usize,
    out: &mut Vec<BezierPiece>,
) -> Result<(), PostProcessError> {
    let enforce_velocity_sign = false;
    let (mono_u, failure) = shaped_ladder(
        axis,
        sig,
        t0,
        t1,
        fit_tol,
        velocity_budget,
        enforce_velocity_sign,
        floors.high_degree,
    )?;
    if failure.is_none() {
        out.push(exact_piece(&mono_u, t0, t1, t1 - t0));
        return Ok(());
    }
    let tm = 0.5 * t0 + 0.5 * t1;
    if t1 - t0 <= floors.cubic
        || depth >= max_depth
        || *refinement_splits >= MAX_FIT_REFINEMENT_SPLITS
        || tm <= t0
        || tm >= t1
    {
        let failure = failure.expect("failed fit has no ladder diagnostic");
        if failure.position_error <= fit_tol.pos_mm
            && failure.velocity_error <= velocity_budget
            && t1 - t0 <= floors.high_degree
        {
            out.push(exact_piece(&mono_u, t0, t1, t1 - t0));
            return Ok(());
        }
        let probe_t = nurbs::fmadd(0.5 * (failure.u + 1.0), t1 - t0, t0);
        return Err(PostProcessError::FitTolerance {
            axis,
            t_start: t0,
            t_end: t1,
            probe_u: failure.u,
            probe_t,
            lower_seed,
            upper_seed,
            lower_seed_provenance: "phase/spline/kernel-shift breakpoint",
            upper_seed_provenance: "phase/spline/kernel-shift breakpoint",
            left_position: failure.left_position,
            left_velocity: failure.left_velocity,
            left_acceleration: failure.left_acceleration,
            signal_detail: sig.diagnostic(probe_t),
            right_position: failure.right_position,
            right_velocity: failure.right_velocity,
            right_acceleration: failure.right_acceleration,
            position_error: failure.position_error,
            position_budget: fit_tol.pos_mm,
            fit_context,
            refinement_splits: *refinement_splits,
            velocity_error: failure.velocity_error,
            velocity_budget,
            acceleration_error: failure.acceleration_error,
            acceleration_budget: fit_tol.accel_mm_s2,
            source_position: failure.source_position,
            source_velocity: failure.source_velocity,
            source_acceleration: failure.source_acceleration,
            candidate_position: failure.candidate_position,
            candidate_velocity: failure.candidate_velocity,
            candidate_acceleration: failure.candidate_acceleration,
        });
    }
    *refinement_splits += 1;
    refine_shaped_span(
        axis,
        sig,
        t0,
        tm,
        fit_tol,
        fit_context,
        velocity_budget,
        floors,
        max_depth,
        depth + 1,
        lower_seed,
        upper_seed,
        refinement_splits,
        out,
    )?;
    refine_shaped_span(
        axis,
        sig,
        tm,
        t1,
        fit_tol,
        fit_context,
        velocity_budget,
        floors,
        max_depth,
        depth + 1,
        lower_seed,
        upper_seed,
        refinement_splits,
        out,
    )
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

/// The reciprocal shape's curvature flips sign with the input velocity, so
/// every velocity zero of the *normalized* signal is a one-sided acceleration
/// seam. The roots must come from the same C0-patched coefficients the signal
/// evaluates, not from the raw track pieces.
fn nonlinear_transition_breakpoints(sig: &NonlinearAdvanceSignal) -> Vec<f64> {
    if sig.adv.model != trajectory::AdvanceModel::Reciprocal {
        return Vec::new();
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

pub(crate) fn apply_trailing_zero_support(
    chain: &CompiledChain,
    axis: usize,
    mut track: nurbs::ScalarNurbs,
    fit_tol: FitTol,
) -> Result<nurbs::ScalarNurbs, PostProcessError> {
    let mut seen_kernel = false;
    for stage in &chain.stages {
        match stage {
            ChainStage::SmoothKernel(_) => seen_kernel = true,
            ChainStage::DerivativeGains { k1, k2 } if seen_kernel => {
                track = apply_derivative_gains_to_track(&track, *k1, *k2);
            }
            ChainStage::NonlinearAdvance(adv) if seen_kernel => {
                track = apply_nonlinear_advance_to_track(axis, &track, *adv, fit_tol)?;
            }
            ChainStage::DerivativeGains { .. } | ChainStage::NonlinearAdvance(_) => {}
        }
    }
    Ok(track)
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
    let sig = NonlinearAdvanceSignal::new(track, adv);
    let pieces = extract_bezier_pieces(track);
    let t_start = pieces
        .first()
        .expect("nonlinear advance: empty track")
        .u_start;
    let t_end = pieces.last().expect("nonlinear advance: empty track").u_end;
    let mut breakpoints = track.knots().to_vec();
    breakpoints.extend(
        pieces[..pieces.len().saturating_sub(1)]
            .iter()
            .map(|piece| f64_from_ordered_key(ordered_f64_key(piece.u_end) + 1)),
    );
    breakpoints.extend(nonlinear_transition_breakpoints(&sig));
    fit_axis_from_signal(
        axis,
        t_start,
        t_end,
        &breakpoints,
        &sig,
        fit_tol,
        "nonlinear_advance",
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
        while i > 0 && self.starts[i] > t {
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
}

pub(crate) fn apply_derivative_gains_to_track(
    track: &nurbs::ScalarNurbs,
    k1: f64,
    k2: f64,
) -> nurbs::ScalarNurbs {
    let pieces = extract_bezier_pieces(track);
    let out_pieces: Vec<BezierPiece> = pieces
        .iter()
        .map(|piece| {
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
            BezierPiece {
                u_start: piece.u_start,
                u_end: piece.u_end,
                coeffs,
            }
        })
        .collect();
    bezier_pieces_to_nurbs(&out_pieces)
}
