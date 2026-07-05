use std::collections::VecDeque;

use crossbeam_channel::{Receiver, Sender};
use nurbs::bezier::{BezierPiece, bezier_pieces_to_nurbs, extract_bezier_pieces};
use trajectory::{AxisChainSet, ChainStage, CompiledChain, ShapedSegment, ShapedSignal};

use crate::lowering::{
    FIT_TRUNC_ACC_MM_S2, FIT_TRUNC_VEL_MM_S, LADDER_PROBES_U, eval_mono, eval_mono_dd,
    ladder_candidate, quintic_in_u,
};
use crate::{Control, LoweredItem, PostProcessError, ShapedItem};
use nurbs::chebyshev::{
    chebyshev_to_monomial_tau, monomial_u_to_chebyshev, truncate_chebyshev_c2_anchored,
};

const SEGMENT_TIME_EPS_S: f64 = 1e-9;

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
    pending: VecDeque<ShapedSegment>,
    pending_rest: VecDeque<bool>,
    forward_support: f64,
    back_support: f64,
}

impl Shaper {
    pub fn new(chains: AxisChainSet) -> Self {
        let (forward_support, back_support) = supports_of(&chains);
        Self {
            chains,
            history: VecDeque::new(),
            pending: VecDeque::new(),
            pending_rest: VecDeque::new(),
            forward_support,
            back_support,
        }
    }

    pub fn run(mut self, input: Receiver<LoweredItem>, output: Sender<ShapedItem>) {
        loop {
            match input.recv() {
                Ok(LoweredItem::Seg(item)) => {
                    self.pending.push_back(item.seg);
                    self.pending_rest.push_back(item.rest_at_end);
                    if !self.emit(self.supported_count(), false, &output) {
                        return;
                    }
                }
                Ok(LoweredItem::Drain) => {
                    assert!(
                        self.pending.is_empty() || self.pending_rest.back() == Some(&true),
                        "shaper: drain marker arrived while the trajectory is not at rest"
                    );
                    if !self.emit(self.pending.len(), true, &output) {
                        return;
                    }
                }
                Ok(LoweredItem::Control(ctrl)) => {
                    match &ctrl {
                        Control::Reset { .. } => {
                            self.pending.clear();
                            self.pending_rest.clear();
                            self.history.clear();
                        }
                        Control::Dwell { .. } | Control::SetAxisChains(_) | Control::Barrier(_) => {
                            assert!(
                                self.pending.is_empty() || self.pending_rest.back() == Some(&true),
                                "shaper: control token arrived while the trajectory is not at \
                                 rest — a Drain must precede it"
                            );
                            if !self.emit(self.pending.len(), true, &output) {
                                return;
                            }
                            if let Control::SetAxisChains(chains) = &ctrl {
                                (self.forward_support, self.back_support) = supports_of(chains);
                                self.chains = chains.clone();
                                self.history.clear();
                            }
                        }
                    }
                    if output.send(ShapedItem::Control(ctrl)).is_err() {
                        return;
                    }
                }
                Err(_) => {
                    self.emit(self.pending.len(), true, &output);
                    return;
                }
            }
        }
    }

    /// How many front segments have their forward convolution window covered
    /// by the buffered lookahead.
    fn supported_count(&self) -> usize {
        let Some(last) = self.pending.back() else {
            return 0;
        };
        let latest_safe_t = last.t_end - self.forward_support;
        self.pending
            .iter()
            .take_while(|seg| seg.t_end <= latest_safe_t + 1e-12)
            .count()
    }

    fn emit(&mut self, count: usize, force: bool, output: &Sender<ShapedItem>) -> bool {
        if count == 0 {
            return true;
        }
        let base = self.pending.make_contiguous();
        let shaped = apply_axis_chains(&self.history, base, count, force, &self.chains.chains)
            .unwrap_or_else(|e| panic!("shaper: {e}"));
        for seg in shaped {
            if output.send(ShapedItem::Seg(seg)).is_err() {
                return false;
            }
        }
        for _ in 0..count {
            let raw = self.pending.pop_front().expect("count <= pending");
            self.pending_rest.pop_front();
            self.history.push_back(raw);
        }
        let emitted_through = self
            .history
            .back()
            .expect("just pushed emitted segments")
            .t_end;
        let keep_after = emitted_through - self.back_support;
        while self
            .history
            .front()
            .is_some_and(|seg| seg.t_end < keep_after)
        {
            self.history.pop_front();
        }
        true
    }
}

fn supports_of(chains: &AxisChainSet) -> (f64, f64) {
    let forward = chains
        .chains
        .iter()
        .map(|chain| chain.max_half_support().1)
        .fold(0.0, f64::max);
    let back = chains
        .chains
        .iter()
        .map(|chain| chain.max_half_support().0.abs())
        .fold(0.0, f64::max);
    (forward, back)
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
    let default_chain = CompiledChain::default();
    for axis in 0..n_axes {
        let chain = chains.get(axis).unwrap_or(&default_chain);
        apply_axis_chain(history, base, &mut out, axis, force, chain)?;
    }
    for seg in &mut out {
        pad_segment_axes_to_uniform_degree(seg);
    }
    Ok(out)
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
    if idx > 0 && segments[idx].t_start > t {
        return eval_segment_axis(segments[idx - 1], axis, segments[idx - 1].t_end);
    }
    if force && (t - last_t).abs() <= SEGMENT_TIME_EPS_S {
        return eval_segment_axis(segments.last().expect("non-empty base"), axis, last_t);
    }
    f64::NAN
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
    let max_len = pieces.iter().map(|p| p.coeffs.len()).max().unwrap_or(1);
    for piece in &mut pieces {
        piece.coeffs.resize(max_len, 0.0);
    }
    Ok(bezier_pieces_to_nurbs(&pieces))
}

const SHAPED_FIT_TOL_MM: f64 = 1e-3;
const SHAPED_FIT_MAX_DEPTH: u32 = 16;
const SHAPED_FIT_MIN_SPAN_S: f64 = 5e-5;
/// Looser than the lowering's 50 mm/s²: the shaped signal's acceleration truth
/// comes from a second-difference stencil, not an analytic profile.
const SHAPED_FIT_TOL_ACCEL_MM_S2: f64 = 200.0;

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
/// sampled (p, v, a) at both ends, degrees 6/7 from interior residuals.
/// Returns the accepted monomial-in-u fit, or the degree-7 candidate with
/// `fits = false` so the caller can bisect.
#[allow(clippy::type_complexity)]
fn shaped_ladder(
    axis: usize,
    sig: &ShapedSignal<'_>,
    t0: f64,
    t1: f64,
    domain_lo: f64,
    domain_hi: f64,
) -> Result<(Vec<f64>, bool), PostProcessError> {
    let h = t1 - t0;
    let t_of = |u: f64| (0.5 * (u + 1.0)).mul_add(h, t0);
    let p0 = finite_sample(axis, sig, t0)?;
    let p1 = finite_sample(axis, sig, t1)?;
    let v0 = finite_derivative(axis, sig, t0, t1, domain_lo, domain_hi)?;
    let v1 = finite_derivative(axis, sig, t1, t0, domain_lo, domain_hi)?;
    let a0 = finite_second_derivative(axis, sig, t0, h, domain_lo, domain_hi)?;
    let a1 = finite_second_derivative(axis, sig, t1, h, domain_lo, domain_hi)?;
    let base = quintic_in_u((p0, v0, a0), (p1, v1, a1), h);

    let mut truth = SpanTruth {
        pos: Vec::with_capacity(LADDER_FIT_NODES_U.len() + LADDER_PROBES_U.len()),
        acc: Vec::with_capacity(LADDER_PROBES_U.len()),
    };
    for &u in LADDER_FIT_NODES_U.iter().chain(LADDER_PROBES_U.iter()) {
        truth.pos.push((u, finite_sample(axis, sig, t_of(u))?));
    }
    for &u in &LADDER_PROBES_U {
        truth.acc.push((
            u,
            finite_second_derivative(axis, sig, t_of(u), h, domain_lo, domain_hi)?,
        ));
    }

    let truth_p = |u: f64| truth.pos_at(u);
    let dd_scale = (2.0 / h) * (2.0 / h);
    let ok = |c: &[f64]| {
        LADDER_PROBES_U.iter().all(|&u| {
            (eval_mono(c, u) - truth.pos_at(u)).abs() <= SHAPED_FIT_TOL_MM
                && (eval_mono_dd(c, u) * dd_scale - truth.acc_at(u)).abs()
                    <= SHAPED_FIT_TOL_ACCEL_MM_S2
        })
    };
    for &degree in crate::lowering::ladder_degrees(h) {
        let c = ladder_candidate(&base, degree, &truth_p);
        if ok(&c) {
            return Ok((c, true));
        }
    }
    Ok((base, false))
}

fn shaped_piece_from_mono_u(mono_u: &[f64], t0: f64, t1: f64) -> BezierPiece<f64> {
    let h = t1 - t0;
    let cheb = truncate_chebyshev_c2_anchored(
        &monomial_u_to_chebyshev(mono_u),
        h,
        0.1 * SHAPED_FIT_TOL_MM,
        FIT_TRUNC_VEL_MM_S,
        FIT_TRUNC_ACC_MM_S2,
    );
    BezierPiece {
        u_start: t0,
        u_end: t1,
        coeffs: chebyshev_to_monomial_tau(&cheb, h),
    }
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
    let (mono_u, fits) = shaped_ladder(axis, sig, t0, t1, domain_lo, domain_hi)?;
    if fits || depth >= SHAPED_FIT_MAX_DEPTH || (t1 - t0) <= 2.0 * SHAPED_FIT_MIN_SPAN_S {
        out.push(shaped_piece_from_mono_u(&mono_u, t0, t1));
        return Ok(());
    }
    let tm = 0.5 * (t0 + t1);
    refine_shaped_span(axis, sig, t0, tm, domain_lo, domain_hi, depth + 1, out)?;
    refine_shaped_span(axis, sig, tm, t1, domain_lo, domain_hi, depth + 1, out)
}

/// Second-difference acceleration estimate with a stencil ~10× the velocity
/// stencil (h² in the denominator amplifies sample noise quadratically).
/// The general non-uniform 3-point formula handles domain-edge clamping.
fn finite_second_derivative(
    axis: usize,
    sig: &ShapedSignal<'_>,
    t: f64,
    span: f64,
    domain_lo: f64,
    domain_hi: f64,
) -> Result<f64, PostProcessError> {
    let h = (span.abs() * 0.05).clamp(1e-5, 5e-4);
    let a = (t - h).max(domain_lo);
    let c = (t + h).min(domain_hi);
    if c - a <= f64::EPSILON {
        return Err(PostProcessError::DegenerateAxisTrack { axis });
    }
    let b = 0.5 * (a + c);
    let fa = finite_sample(axis, sig, a)?;
    let fb = finite_sample(axis, sig, b)?;
    let fc = finite_sample(axis, sig, c)?;
    let (ba, cb, ca) = (b - a, c - b, c - a);
    Ok(2.0 * (fa / (ba * ca) - fb / (ba * cb) + fc / (ca * cb)))
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
