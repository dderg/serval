use std::collections::VecDeque;

use crossbeam_channel::{Receiver, Sender};
use nurbs::bezier::{BezierPiece, bezier_pieces_to_nurbs, extract_bezier_pieces};
use trajectory::{AxisChainSet, ChainStage, CompiledChain, ShapedSegment, ShapedSignal};

use crate::follower_projection::{FollowerState, project_followers};
use crate::lowering::{
    FIT_TRUNC_POS_FACTOR, FitTol, LADDER_PROBES_U, ladder_fit, quintic_in_u, truncated_piece,
};
use crate::types::{Control, LoweredItem, PostProcessError, ShapedItem};

/// The evaluable (position, velocity, acceleration) signal the shaped-track
/// fitter consumes: the kernel convolution for kerneled axes, the follower
/// projection for follower axes.
pub(crate) trait TrackSignal {
    fn eval(&self, t: f64) -> f64;
    fn deriv(&self, t: f64) -> f64;
    fn second_deriv(&self, t: f64) -> f64;
    /// `(eval, deriv, second_deriv)` at one `t`; implementations that share
    /// work across the three (the kernel convolution) override this.
    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        (self.eval(t), self.deriv(t), self.second_deriv(t))
    }
}

impl TrackSignal for ShapedSignal<'_> {
    fn eval(&self, t: f64) -> f64 {
        ShapedSignal::eval(self, t)
    }

    fn deriv(&self, t: f64) -> f64 {
        ShapedSignal::deriv(self, t)
    }

    fn second_deriv(&self, t: f64) -> f64 {
        ShapedSignal::second_deriv(self, t)
    }

    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        ShapedSignal::eval_pva(self, t)
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
    segments: VecDeque<ShapedSegment>,
    rests: VecDeque<bool>,
    shaped: VecDeque<ShapedSegment>,
}

impl PendingSegments {
    fn push(&mut self, segment: ShapedSegment, rest_at_end: bool) {
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

    fn back(&self) -> Option<&ShapedSegment> {
        self.segments.back()
    }

    fn iter(&self) -> impl Iterator<Item = &ShapedSegment> {
        self.segments.iter()
    }

    fn pop_front(&mut self) -> Option<ShapedSegment> {
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
    history: VecDeque<ShapedSegment>,
    pending: PendingSegments,
    forward_support: f64,
    back_support: f64,
    history_trimmed: bool,
    follower_states: Vec<FollowerState>,
    toolhead_tap: Option<Sender<ShapedSegment>>,
}

impl Shaper {
    pub fn new(chains: AxisChainSet) -> Self {
        Self {
            forward_support: chains.forward_support(),
            back_support: chains.back_support(),
            chains,
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
    pub fn with_toolhead_tap(mut self, tap: Sender<ShapedSegment>) -> Self {
        self.toolhead_tap = Some(tap);
        self
    }

    pub fn run(mut self, input: Receiver<LoweredItem>, output: Sender<ShapedItem>) {
        loop {
            match input.recv() {
                Ok(item) => {
                    if !self.feed(item, &output) {
                        return;
                    }
                }
                Err(_) => {
                    self.finish(&output);
                    return;
                }
            }
        }
    }

    /// One iteration of [`Shaper::run`]'s loop, for single-threaded hosts
    /// that drive the stage item by item.
    pub fn feed(&mut self, item: LoweredItem, output: &Sender<ShapedItem>) -> bool {
        match item {
            LoweredItem::Seg(item) => {
                self.pending.push(item.seg, item.rest_at_end);
                self.emit(self.supported_count(), false, output)
            }
            LoweredItem::Drain => {
                assert!(
                    self.pending.is_empty() || self.pending.ends_at_rest(),
                    "shaper: drain marker arrived while the trajectory is not at rest"
                );
                self.emit(self.pending.len(), true, output)
            }
            LoweredItem::Control(ctrl) => {
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
                output.send(ShapedItem::Control(ctrl)).is_ok()
            }
        }
    }

    /// The input-closed path: flush the buffered tail with the window clamped
    /// past the end of the signal.
    pub fn finish(&mut self, output: &Sender<ShapedItem>) -> bool {
        self.emit(self.pending.len(), true, output)
    }

    /// How many front segments have their forward convolution window covered
    /// by the buffered lookahead. A follower with its own kernel cascades:
    /// its convolution reads the projection, which is only final through the
    /// shaping frontier — the last pending segment whose directly-convolved
    /// tracks have their own lookahead covered. The time-based bound already
    /// includes the cascaded width, but segment granularity can leave a long
    /// straddling segment unprojectable, so gate on the frontier explicitly.
    fn supported_count(&self) -> usize {
        let Some(last) = self.pending.back() else {
            return 0;
        };
        let latest_safe_t = last.t_end - self.forward_support;
        let plain = self
            .pending
            .iter()
            .take_while(|seg| seg.t_end <= latest_safe_t + 1e-12)
            .count();
        let own_hi = self.chains.max_follower_own_forward_support();
        if own_hi <= 0.0 {
            return plain;
        }
        let Some(frontier_t) = self.shaping_frontier_t(last.t_end) else {
            return 0;
        };
        let gated = self
            .pending
            .iter()
            .take_while(|seg| seg.t_end + own_hi <= frontier_t + 1e-12)
            .count();
        plain.min(gated)
    }

    /// End time of the last pending segment whose direct convolution window is
    /// covered by the buffered lookahead.
    fn shaping_frontier_t(&self, last_t: f64) -> Option<f64> {
        let direct_hi = self.chains.direct_forward_support();
        self.pending
            .iter()
            .take_while(|seg| seg.t_end + direct_hi <= last_t + 1e-12)
            .last()
            .map(|seg| seg.t_end)
    }

    fn emit(&mut self, count: usize, force: bool, output: &Sender<ShapedItem>) -> bool {
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
                .take_while(|seg| seg.t_end + direct_hi <= last_t + 1e-12)
                .count()
                .max(count)
        };
        let pending = &mut self.pending;
        let base: &[ShapedSegment] = pending.segments.make_contiguous();
        let shaped = apply_axis_chains(
            &self.history,
            base,
            count,
            frontier_count,
            force,
            !self.history_trimmed,
            &self.chains,
            &mut self.follower_states,
            self.toolhead_tap.as_ref(),
            &mut pending.shaped,
        )
        .unwrap_or_else(|e| panic!("shaper: {e}"));
        for seg in shaped {
            if output.send(ShapedItem::Seg(seg)).is_err() {
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

#[allow(clippy::too_many_arguments)]
fn apply_axis_chains(
    history: &VecDeque<ShapedSegment>,
    base: &[ShapedSegment],
    commit_count: usize,
    frontier_count: usize,
    force: bool,
    at_stream_boundary: bool,
    chains: &AxisChainSet,
    follower_states: &mut Vec<FollowerState>,
    toolhead_tap: Option<&Sender<ShapedSegment>>,
    shaped_cache: &mut VecDeque<ShapedSegment>,
) -> Result<Vec<ShapedSegment>, PostProcessError> {
    if chains.chains.iter().all(CompiledChain::is_empty)
        && follower_states.iter().all(|s| !s.is_active())
    {
        let out: Vec<ShapedSegment> = base.iter().take(commit_count).cloned().collect();
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
    let mut prev: Option<&ShapedSegment> = None;
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
    if window > shaped_cache.len() {
        let mut fresh: Vec<ShapedSegment> = base[shaped_cache.len()..window].to_vec();
        fit_leader_axes(
            history,
            base,
            &mut fresh,
            n_axes,
            force,
            at_stream_boundary,
            chains,
        )?;
        shaped_cache.extend(fresh);
    }
    let frontier = &shaped_cache.make_contiguous()[..window];
    let mut out: Vec<ShapedSegment> = frontier.iter().take(commit_count).cloned().collect();
    project_followers(
        base,
        frontier,
        &mut out,
        commit_count,
        force,
        chains,
        follower_states,
    )?;
    send_toolhead(toolhead_tap, &out);
    apply_motor_side_stages(&mut out, chains)?;
    for seg in &mut out {
        pad_segment_axes_to_uniform_degree(seg);
    }
    Ok(out)
}

fn send_toolhead(tap: Option<&Sender<ShapedSegment>>, out: &[ShapedSegment]) {
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
    out: &mut [ShapedSegment],
    chains: &AxisChainSet,
) -> Result<(), PostProcessError> {
    for seg in out.iter_mut() {
        for axis in 0..seg.axes.len() {
            let Some(chain) = chains.chains.get(axis) else {
                continue;
            };
            if !chain.has_motor_side_gains() {
                continue;
            }
            seg.axes[axis] = apply_trailing_zero_support(chain, axis, seg.axes[axis].clone())?;
        }
    }
    Ok(())
}

/// Refit tracks can come out at different degrees per axis; the kinematics
/// lane mixing in enqueue adds axis curves together, which needs one degree
/// across the segment. Zero-padding the monomial pieces is exact.
fn pad_segment_axes_to_uniform_degree(seg: &mut ShapedSegment) {
    let max_degree = seg
        .axes
        .iter()
        .map(|curve| curve.degree() as usize)
        .max()
        .unwrap_or(0);
    for curve in &mut seg.axes {
        if (curve.degree() as usize) == max_degree {
            continue;
        }
        let mut pieces = extract_bezier_pieces(curve);
        for piece in &mut pieces {
            piece.coeffs.resize(max_degree + 1, 0.0);
        }
        *curve = bezier_pieces_to_nurbs(&pieces);
    }
}

/// Fit one kerneled axis over `targets`, returning the shaped column —
/// `None` when the chain has no kernel. Pure in its inputs, so columns for
/// different axes run on scoped threads.
fn fit_axis_column(
    history: &VecDeque<ShapedSegment>,
    base: &[ShapedSegment],
    targets: &[ShapedSegment],
    axis: usize,
    force: bool,
    at_stream_boundary: bool,
    chain: &CompiledChain,
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
    let signal_segments: Vec<&ShapedSegment> = history.iter().chain(base.iter()).collect();
    let input_breaks = signal_breakpoints(&signal_segments, axis);
    let table = AxisSignalTable::build(
        &signal_segments,
        axis,
        first_t,
        last_t,
        at_stream_boundary,
        force,
    );
    let sig = ShapedSignal::new_from_evaluator(kernel, |t| table.eval(t), input_breaks);
    let mut column = Vec::with_capacity(targets.len());
    for seg in targets {
        let need_lo = seg.t_start - k_hi;
        let need_hi = seg.t_end - k_lo;
        if need_lo < first_t && !at_stream_boundary {
            return Err(PostProcessError::MissingHistory { axis, t: need_lo });
        }
        if need_hi > last_t && !force {
            return Err(PostProcessError::MissingLookahead { axis, t: need_hi });
        }
        let track = fit_axis_from_signal(axis, &seg.axes[axis], &sig, 1.0)?;
        if !track.control_points().iter().all(|v| v.is_finite()) {
            return Err(PostProcessError::NonFiniteSample {
                axis,
                t: seg.t_start,
            });
        }
        column.push(track);
    }
    Ok(Some(column))
}

/// Fit every kerneled non-follower axis over `fresh`, one scoped thread per
/// axis when more than one needs fitting — the columns are independent and
/// the merge order is fixed, so the result is bit-identical to the serial
/// pass.
fn fit_leader_axes(
    history: &VecDeque<ShapedSegment>,
    base: &[ShapedSegment],
    fresh: &mut [ShapedSegment],
    n_axes: usize,
    force: bool,
    at_stream_boundary: bool,
    chains: &AxisChainSet,
) -> Result<(), PostProcessError> {
    let default_chain = CompiledChain::default();
    let axis_chains: Vec<(usize, &CompiledChain)> = (0..n_axes)
        .filter(|&axis| !chains.is_projected_follower(axis))
        .map(|axis| (axis, chains.chains.get(axis).unwrap_or(&default_chain)))
        .filter(|(_, chain)| !chain.is_empty())
        .collect();
    // A column fit costs milliseconds (ladder fits over the convolution
    // window) against tens of microseconds per scoped spawn, so parallel
    // pays from the first fresh segment: with the old >=8-segment gate the
    // dense-region steady state (1-2 fresh segments per emit) fitted every
    // axis serially and pegged one core while the rest idled.
    let fit_started = crate::timing::stopwatch();
    type TimedColumn = (
        usize,
        Result<Option<Vec<nurbs::ScalarNurbs>>, PostProcessError>,
        u128,
    );
    let parallel = cfg!(not(target_arch = "wasm32")) && axis_chains.len() > 1 && !fresh.is_empty();
    let columns: Vec<TimedColumn> = if parallel {
        let fresh_ref: &[ShapedSegment] = fresh;
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
                            chain,
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
                let column =
                    fit_axis_column(history, base, fresh, axis, force, at_stream_boundary, chain);
                (axis, column, column_started.elapsed_us())
            })
            .collect()
    };
    let fit_elapsed_us = fit_started.elapsed_us();
    if fit_elapsed_us >= 20_000 {
        let per_axis: Vec<String> = columns
            .iter()
            .map(|(axis, _, elapsed_us)| format!("axis{axis}={elapsed_us}us"))
            .collect();
        tracing::warn!(
            subsystem = "motion",
            event = "shaper_fit_slow",
            total_us = fit_elapsed_us as u64,
            fresh_segments = fresh.len(),
            columns = %per_axis.join(" "),
            "shaper leader fit pass exceeded 20ms"
        );
    }
    for (axis, column, _) in columns {
        let Some(column) = column? else { continue };
        for (seg, track) in fresh.iter_mut().zip(column) {
            seg.axes[axis] = track;
        }
    }
    Ok(())
}

/// Every time the window's signal changes polynomial on this axis: each
/// segment's NURBS knots plus the segment edges (a gap between segments is a
/// constant hold, which the edges delimit). The exact convolution integrates
/// between these, so its Gauss rule never straddles a polynomial change.
fn signal_breakpoints(segments: &[&ShapedSegment], axis: usize) -> Vec<f64> {
    let mut breaks = Vec::new();
    for seg in segments {
        breaks.push(seg.t_start);
        breaks.extend_from_slice(seg.axes[axis].knots());
        breaks.push(seg.t_end);
    }
    breaks
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
    cursor: std::cell::Cell<usize>,
}

impl AxisSignalTable {
    fn build(
        segments: &[&ShapedSegment],
        axis: usize,
        first_t: f64,
        last_t: f64,
        at_stream_boundary: bool,
        force: bool,
    ) -> Self {
        Self::from_tracks(
            segments.iter().map(|seg| &seg.axes[axis]),
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
            cursor: std::cell::Cell::new(0),
        };
        for track in tracks {
            for p in extract_bezier_pieces(track) {
                table.starts.push(p.u_start);
                table.ends.push(p.u_end);
                table.coeffs.push(p.coeffs);
            }
        }
        assert!(!table.coeffs.is_empty(), "empty signal window");
        table
    }

    fn piece_at(&self, i: usize, t: f64) -> f64 {
        let tau = (t - self.starts[i]).clamp(0.0, self.ends[i] - self.starts[i]);
        self.coeffs[i]
            .iter()
            .rev()
            .fold(0.0_f64, |acc, &c| nurbs::fmadd(acc, tau, c))
    }

    pub(crate) fn eval(&self, t: f64) -> f64 {
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
        let mut i = self.cursor.get().min(self.coeffs.len() - 1);
        while i > 0 && self.ends[i - 1] + SEGMENT_TIME_EPS_S >= t {
            i -= 1;
        }
        while i + 1 < self.coeffs.len() && self.ends[i] + SEGMENT_TIME_EPS_S < t {
            i += 1;
        }
        self.cursor.set(i);
        if t >= self.starts[i] - SEGMENT_TIME_EPS_S {
            return self.piece_at(i, t);
        }
        if i > 0 {
            return self.piece_at(i - 1, self.ends[i - 1]);
        }
        f64::NAN
    }
}

/// A time gap in the signal is evaluated as the position held at the
/// preceding rest, which is only sound if both sides of the gap agree on that
/// position.
fn assert_gap_is_a_hold(prev: &ShapedSegment, next: &ShapedSegment) {
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

fn eval_segment_axis(seg: &ShapedSegment, axis: usize, t: f64) -> f64 {
    nurbs::eval::eval(&seg.axes[axis], t.clamp(seg.t_start, seg.t_end))
}

pub(crate) fn fit_axis_from_signal<S: TrackSignal>(
    axis: usize,
    template: &nurbs::ScalarNurbs,
    sig: &S,
    tol_scale: f64,
) -> Result<nurbs::ScalarNurbs, PostProcessError> {
    let template_pieces = extract_bezier_pieces(template);
    if template_pieces.is_empty() {
        return Err(PostProcessError::DegenerateAxisTrack { axis });
    }
    // The template's breakpoints seed the partition, but the convolved signal can
    // need finer pieces than the unshaped trajectory had — so refine each span to
    // the shaper's own tolerance rather than inheriting the template's resolution.
    // Seed spans from the template's boundaries, coalescing any closer than
    // the fit floor: a sliver template piece (repeated NURBS knot, sub-µs
    // remainder) fitted on its own divides by its near-zero duration and
    // mints garbage endpoint accelerations at the seam.
    let t_lo = template_pieces.first().expect("checked non-empty").u_start;
    let t_hi = template_pieces.last().expect("checked non-empty").u_end;
    if t_hi - t_lo < SHAPED_FIT_MIN_SPAN_S {
        // A sliver segment (float-noise trim stub upstream) is too short for
        // the Hermite fit's 1/h² conditioning; a linear piece through the
        // exact endpoint positions carries its (physically invisible)
        // nanoseconds without minting garbage endpoint accelerations.
        let p0 = finite_sample(axis, sig, t_lo)?;
        let p1 = finite_sample(axis, sig, t_hi)?;
        let piece = BezierPiece {
            u_start: t_lo,
            u_end: t_hi,
            coeffs: vec![p0, (p1 - p0) / (t_hi - t_lo)],
        };
        return Ok(bezier_pieces_to_nurbs(&[piece]));
    }
    let mut seeds = vec![t_lo];
    for piece in &template_pieces {
        if piece.u_end - seeds.last().expect("seeded") >= SHAPED_FIT_MIN_SPAN_S {
            seeds.push(piece.u_end);
        }
    }
    while seeds.len() > 1 && t_hi - *seeds.last().expect("seeded") < SHAPED_FIT_MIN_SPAN_S {
        seeds.pop();
    }
    seeds.push(t_hi);
    let mut pieces = Vec::with_capacity(template_pieces.len());
    for w in seeds.windows(2) {
        refine_shaped_span(axis, sig, w[0], w[1], 0, tol_scale, &mut pieces)?;
    }
    let max_len = pieces.iter().map(|p| p.coeffs.len()).max().unwrap_or(1);
    for piece in &mut pieces {
        piece.coeffs.resize(max_len, 0.0);
    }
    Ok(bezier_pieces_to_nurbs(&pieces))
}

const SHAPED_FIT_TOL_MM: f64 = 1e-3;
const SHAPED_FIT_MAX_DEPTH: u32 = 16;
const SHAPED_FIT_MIN_SPAN_S: f64 = 5e-5;
const SHAPED_FIT_TOL_ACCEL_MM_S2: f64 = 50.0;

/// Sampled truth for one span, taken up front so stencil errors surface as
/// `PostProcessError` instead of poisoning the ladder closures.
struct SpanTruth {
    pos: Vec<(f64, f64)>,
    acc: Vec<(f64, f64)>,
}

impl SpanTruth {
    fn pos_at(&self, u: f64) -> f64 {
        self.pos
            .iter()
            .find(|(uu, _)| *uu == u)
            .unwrap_or_else(|| panic!("ladder probed unsampled node u={u}"))
            .1
    }

    fn acc_at(&self, u: f64) -> f64 {
        self.acc
            .iter()
            .find(|(uu, _)| *uu == u)
            .unwrap_or_else(|| panic!("ladder probed unsampled accel node u={u}"))
            .1
    }
}

const LADDER_FIT_NODES_U: [f64; 3] = [0.0, 0.5, -0.5];

/// Ladder fit of the shaped signal over one span: quintic Hermite matching
/// the convolution's exact (p, v, a) at both ends, degrees 6/7 from interior
/// residuals — the lowering ladder against pre-sampled truth, with the
/// shaper's own budgets. Returns the accepted monomial-in-u fit, or the
/// quintic base with `fits = false` so the caller can bisect.
#[allow(clippy::type_complexity)]
fn shaped_ladder<S: TrackSignal>(
    axis: usize,
    sig: &S,
    t0: f64,
    t1: f64,
    tol_scale: f64,
) -> Result<(Vec<f64>, bool), PostProcessError> {
    let h = t1 - t0;
    let t_of = |u: f64| nurbs::fmadd(0.5 * (u + 1.0), h, t0);
    let (p0, v0, a0) = sig.eval_pva(t0);
    let p0 = exact_value(axis, p0, t0)?;
    let v0 = exact_value(axis, v0, t0)?;
    let a0 = exact_value(axis, a0, t0)?;
    let (p1, v1, a1) = sig.eval_pva(t1);
    let p1 = exact_value(axis, p1, t1)?;
    let v1 = exact_value(axis, v1, t1)?;
    let a1 = exact_value(axis, a1, t1)?;
    let base = quintic_in_u((p0, v0, a0), (p1, v1, a1), h);

    let mut truth = SpanTruth {
        pos: Vec::with_capacity(LADDER_FIT_NODES_U.len() + LADDER_PROBES_U.len()),
        acc: Vec::with_capacity(LADDER_PROBES_U.len()),
    };
    for &u in &LADDER_FIT_NODES_U {
        if u == 0.0 {
            continue;
        }
        truth.pos.push((u, finite_sample(axis, sig, t_of(u))?));
    }
    for &u in &LADDER_PROBES_U {
        let t = t_of(u);
        let (pos, _, acc) = sig.eval_pva(t);
        truth.pos.push((u, exact_value(axis, pos, t)?));
        truth.acc.push((u, exact_value(axis, acc, t)?));
    }

    let tol = FitTol {
        pos_mm: SHAPED_FIT_TOL_MM,
        accel_mm_s2: SHAPED_FIT_TOL_ACCEL_MM_S2,
    }
    .scaled(tol_scale);
    match ladder_fit(&base, h, tol, &|u| truth.pos_at(u), &|u| truth.acc_at(u)) {
        Some(c) => Ok((c, true)),
        None => Ok((base, false)),
    }
}

fn refine_shaped_span<S: TrackSignal>(
    axis: usize,
    sig: &S,
    t0: f64,
    t1: f64,
    depth: u32,
    tol_scale: f64,
    out: &mut Vec<BezierPiece>,
) -> Result<(), PostProcessError> {
    let (mono_u, fits) = shaped_ladder(axis, sig, t0, t1, tol_scale)?;
    if fits || depth >= SHAPED_FIT_MAX_DEPTH || (t1 - t0) <= 2.0 * SHAPED_FIT_MIN_SPAN_S {
        out.push(truncated_piece(
            &mono_u,
            t0,
            t1,
            t1 - t0,
            FIT_TRUNC_POS_FACTOR * SHAPED_FIT_TOL_MM * tol_scale,
            tol_scale,
        ));
        return Ok(());
    }
    let tm = 0.5 * (t0 + t1);
    refine_shaped_span(axis, sig, t0, tm, depth + 1, tol_scale, out)?;
    refine_shaped_span(axis, sig, tm, t1, depth + 1, tol_scale, out)
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
) -> Result<nurbs::ScalarNurbs, PostProcessError> {
    let mut seen_kernel = false;
    for stage in &chain.stages {
        match stage {
            ChainStage::SmoothKernel(_) => seen_kernel = true,
            ChainStage::DerivativeGains { k1, k2 } if seen_kernel => {
                track = apply_derivative_gains_to_track(&track, *k1, *k2);
            }
            ChainStage::NonlinearAdvance(adv) if seen_kernel => {
                track = apply_nonlinear_advance_to_track(axis, &track, *adv)?;
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
) -> Result<nurbs::ScalarNurbs, PostProcessError> {
    let sig = NonlinearAdvanceSignal::new(track, adv);
    fit_axis_from_signal(axis, track, &sig, 1.0)
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
        Self {
            starts: pieces.iter().map(|p| p.u_start).collect(),
            ends: pieces.iter().map(|p| p.u_end).collect(),
            coeffs: pieces.into_iter().map(|p| p.coeffs).collect(),
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
        self.adv
            .curvature(v)
            .mul_add(a * a, self.adv.slope(v).mul_add(j, a))
    }

    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        let (p, v, a, j) = self.input_state(t);
        let slope = self.adv.slope(v);
        (
            p + self.adv.advance(v),
            slope.mul_add(a, v),
            self.adv.curvature(v).mul_add(a * a, slope.mul_add(j, a)),
        )
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
