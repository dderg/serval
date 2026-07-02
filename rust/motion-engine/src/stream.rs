use std::collections::VecDeque;
use std::thread;
use std::time::Instant;

use crossbeam_channel::{Receiver, Sender, bounded};
use geometry::path::lowering::PositionProfile;
use geometry::{ChainFitConfig, Move, MoveVelocity, VelocityLimits};
use nurbs::bezier::{BezierPiece, bezier_pieces_to_nurbs, extract_bezier_pieces};
use trajectory::{AxisChainSet, ChainStage, CompiledChain, ShapedSegment, ShapedSignal};

use crate::lowering::lower_move;

pub(crate) mod fitter;
#[cfg(test)]
mod fitter_tests;
pub(crate) mod planner;

const SEGMENT_TIME_EPS_S: f64 = 1e-9;
pub(crate) const CONTIGUITY_EPS_MM: f64 = 1e-6;
const REST_EPS_MM_S: f64 = 1e-9;

#[derive(Debug, Clone, Copy)]
pub struct StreamConfig {
    pub chain: ChainFitConfig,
    pub integration_tol: f64,
    pub max_extrude_only_velocity_mm_s: f64,
    pub max_extrude_only_accel_mm_s2: f64,
    pub fit_tol_mm: f64,
    /// Backstop cap on the planner's look-ahead window. A normal continuous
    /// path always offers a clean seam, so the window stays small; this only
    /// fires for a pathological window with no clean seam within the finality
    /// barrier at all (e.g. a single move longer than the whole look-ahead) —
    /// without it such a window would grow unbounded. It is a safety net, not
    /// the steady-state path.
    pub max_buffer_moves: usize,
    /// Path limits for planner-internal moves (homing). Stream moves submitted
    /// through the bridge carry their own per-move limits; this is the fallback
    /// used when the engine constructs a move itself.
    pub limits: VelocityLimits,
}

#[derive(Debug)]
pub enum StreamError {
    /// A move entered the pipeline whose spatial start does not meet the
    /// toolhead where the previous move left it. Real slicer output is always
    /// position-contiguous; a gap means the move stream was stitched wrong
    /// upstream. Caught at ingress so the offending move is named there, not
    /// as a downstream `ZeroMotion` deep in the fitter.
    Discontinuity {
        line_no: u32,
        expected: [f64; 3],
        got: [f64; 3],
        gap_mm: f64,
    },
}

impl std::fmt::Display for StreamError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
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
        }
    }
}

impl std::error::Error for StreamError {}

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

// ---------------------------------------------------------------------------
// Pipeline types — items that flow between stages
// ---------------------------------------------------------------------------

pub struct PlannedMove {
    pub geometry: Move,
    pub velocity: MoveVelocity,
}

/// Lowerer output: a dispatchable segment plus whether the trajectory is at
/// rest at its end — the shaper may clamp its convolution window past a rest
/// point (the signal is constant there), never past a moving end.
pub struct LoweredSegment {
    pub seg: ShapedSegment,
    pub rest_at_end: bool,
}

pub struct PipelineHandle {
    pub input: Sender<StreamInput>,
    pub output: Receiver<ShapedItem>,
}

/// What flows into the fitter and planner: geometry, the command to stop
/// looking ahead, or an ordered control token. `Drain` makes each stage
/// resolve and emit everything it is holding — the fitter finalizes runs and
/// blends, the planner materializes the brake-to-rest — exactly what a closed
/// input does, but without ending the stream. The stages themselves never
/// consult a clock or peek at channel occupancy; whoever owns the notion of
/// time decides when to send `Drain`.
#[derive(Debug)]
pub enum StreamInput {
    Move(Move),
    Drain,
    Control(Control),
}

impl From<Move> for StreamInput {
    fn from(m: Move) -> Self {
        Self::Move(m)
    }
}

/// Ordered control tokens that flow through every stage with the geometry.
/// The pipeline is set up once and lives forever; these replace the old
/// teardown-and-rebuild lifecycle. Tokens that require the trajectory to be
/// at rest (`Dwell`, `SetAxisChains`, `Barrier` after a flush) must be
/// preceded by a `Drain`; the stages assert emptiness rather than draining
/// implicitly, so a violated protocol fails loudly instead of hiding a
/// velocity discontinuity.
#[derive(Debug)]
pub enum Control {
    /// Advance the trajectory clock without motion (lowerer applies it).
    Dwell { secs: f64 },
    /// Drop all buffered state and restart the timeline at rest at `pos`.
    /// The sender is responsible for gating the dispatcher (discard) so
    /// motion already lowered ahead of this token is dropped, not executed.
    Reset { pos: Vec<f64> },
    /// Swap the post-processing chains (lowerer and shaper apply it).
    SetAxisChains(AxisChainSet),
    /// Acknowledged by the dispatcher once everything ahead of it has been
    /// dispatched (or discarded): the pipeline-wide "everything before this
    /// point is done" fence.
    Barrier(Sender<BarrierAck>),
}

/// The dispatcher's answer to a `Barrier`.
#[derive(Debug)]
pub struct BarrierAck {
    /// Stream time the dispatched trajectory has reached; `None` when nothing
    /// has been dispatched since the last reset.
    pub dispatched_through: Option<f64>,
    /// Host instant of the first dispatch since the last reset, for
    /// projecting stream time onto the wall clock.
    pub sync_instant: Option<Instant>,
    /// Dispatch errors captured since the previous barrier (error capture is
    /// enabled by the homing paths; otherwise a dispatch error is fatal).
    pub result: Result<(), String>,
}

/// Planner → lowerer.
pub enum PlannedItem {
    Move(PlannedMove),
    Control(Control),
}

/// Lowerer → shaper.
pub enum LoweredItem {
    Seg(LoweredSegment),
    Control(Control),
}

/// Shaper → dispatcher.
pub enum ShapedItem {
    Seg(ShapedSegment),
    Control(Control),
}

/// Wires the pure stream stages (fitter → planner → lowerer → shaper) into
/// OS threads. Production goes through `stream_worker::setup_pipeline`, which
/// wraps these stages with the dispatcher and pump; this stage-only wiring is
/// also used standalone by offline consumers (seam harness, trajectory dump)
/// that have no hardware behind them.
pub fn setup_stages(
    config: StreamConfig,
    axis_chains: AxisChainSet,
    home_pos: Vec<f64>,
    t_start: f64,
) -> PipelineHandle {
    let (raw_tx, raw_rx) = bounded::<StreamInput>(64);
    let (fitted_tx, fitted_rx) = bounded::<StreamInput>(64);
    let (planned_tx, planned_rx) = bounded::<PlannedItem>(64);
    let (lowered_tx, lowered_rx) = bounded::<LoweredItem>(64);
    let (shaped_tx, shaped_rx) = bounded::<ShapedItem>(64);

    let fitter = fitter::Fitter::new(config.chain);
    spawn_stage("kalico-fit", move || fitter.run(raw_rx, fitted_tx));

    let planner = planner::Planner::new(config);
    spawn_stage("kalico-plan", move || planner.run(fitted_rx, planned_tx));

    let fit_tol = config.fit_tol_mm;
    let lower_chains = axis_chains.clone();
    spawn_stage("kalico-lower", move || {
        run_lowerer(
            planned_rx,
            lowered_tx,
            fit_tol,
            lower_chains,
            home_pos,
            t_start,
        );
    });

    let shaper = Shaper::new(axis_chains);
    spawn_stage("kalico-shape", move || shaper.run(lowered_rx, shaped_tx));

    PipelineHandle {
        input: raw_tx,
        output: shaped_rx,
    }
}

fn spawn_stage(name: &str, f: impl FnOnce() + Send + 'static) {
    thread::Builder::new()
        .name(name.to_string())
        .spawn(f)
        .unwrap_or_else(|e| panic!("spawn {name}: {e}"));
}

/// Third pipeline stage: lowers each planned move into a dispatchable
/// `ShapedSegment`. It is the persistent owner of the trajectory clock and
/// odometer: `Dwell` advances the clock without motion, `Reset` restarts the
/// timeline at rest at the given position, `SetAxisChains` swaps the chain
/// set future moves are lowered against.
pub(crate) fn run_lowerer(
    input: Receiver<PlannedItem>,
    output: Sender<LoweredItem>,
    fit_tol_mm: f64,
    mut axis_chains: AxisChainSet,
    home_pos: Vec<f64>,
    t_start: f64,
) {
    let mut odometer = home_pos;
    let mut t = t_start;

    while let Ok(item) = input.recv() {
        let planned = match item {
            PlannedItem::Move(planned) => planned,
            PlannedItem::Control(ctrl) => {
                match &ctrl {
                    Control::Dwell { secs } => {
                        assert!(*secs >= 0.0, "lowerer: negative dwell {secs}");
                        t += secs;
                    }
                    Control::Reset { pos } => {
                        odometer.clone_from(pos);
                        t = 0.0;
                    }
                    Control::SetAxisChains(chains) => axis_chains = chains.clone(),
                    Control::Barrier(_) => {}
                }
                if output.send(LoweredItem::Control(ctrl)).is_err() {
                    return;
                }
                continue;
            }
        };
        let clock = Instant::now();
        let mut seg = lower_move(
            &planned.geometry,
            &planned.velocity,
            t,
            &odometer,
            fit_tol_mm,
            &axis_chains.chains,
        )
        .unwrap_or_else(|e| panic!("lowerer: line {}: {e}", planned.geometry.source.start_line));
        seg.source_line = planned.geometry.source.start_line;

        t = seg.t_end;
        advance_odometer(&mut odometer, &planned.geometry);
        tracing::debug!(
            subsystem = "motion",
            event = "pipe_lower",
            line = seg.source_line,
            lower_us = clock.elapsed().as_micros(),
            t_us = crate::timing::mono_us(),
            "[pipe] lower"
        );

        let rest_at_end = planned.velocity.exit_v <= REST_EPS_MM_S;
        if output
            .send(LoweredItem::Seg(LoweredSegment { seg, rest_at_end }))
            .is_err()
        {
            return;
        }
    }
}

/// Final pipeline stage: streaming axis-chain post-processing. Buffers lowered
/// segments until each one's convolution window is covered by lookahead, then
/// emits the shaped segment; keeps just-emitted raw segments as the history the
/// next windows read. A rest ending lets the buffer flush with the window
/// clamped (the signal is constant past a rest point), so the trajectory's
/// tail is never held hostage to input that may not come.
pub(crate) struct Shaper {
    chains: AxisChainSet,
    history: VecDeque<ShapedSegment>,
    pending: VecDeque<ShapedSegment>,
    pending_rest: VecDeque<bool>,
    forward_support: f64,
    back_support: f64,
}

impl Shaper {
    pub(crate) fn new(chains: AxisChainSet) -> Self {
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

    pub(crate) fn run(mut self, input: Receiver<LoweredItem>, output: Sender<ShapedItem>) {
        loop {
            match input.recv() {
                Ok(LoweredItem::Seg(item)) => {
                    self.pending.push_back(item.seg);
                    self.pending_rest.push_back(item.rest_at_end);
                    if !self.emit(self.supported_count(), false, &output) {
                        return;
                    }
                    let at_rest = self.pending_rest.back() == Some(&true);
                    if at_rest && input.is_empty() && !self.emit(self.pending.len(), true, &output)
                    {
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

/// Jerk-limited time to decelerate from `v` to rest under accel limit `a` and
/// jerk limit `j`: `v/a + a/j` once the ramp reaches `a` (`v > a²/j`), else the
/// triangular `2·√(v/j)`. Curvature only slows a real stop, so this
/// straight-line time is a safe over-estimate.
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

pub(crate) fn dist3(a: [f64; 3], b: [f64; 3]) -> f64 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    let dz = a[2] - b[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

pub(crate) fn advance_odometer(pos: &mut [f64], gm: &Move) {
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
