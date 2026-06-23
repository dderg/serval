use std::collections::{HashSet, VecDeque};

use geometry::path::lowering::PositionProfile;
use geometry::{
    ChainFitConfig, FitError, Move, VelocityConfig, VelocityError, VelocityLimits, fit_chain,
    plan_velocity_warm_start,
};
use nurbs::bezier::{BezierPiece, bezier_pieces_to_nurbs, extract_bezier_pieces};
use trajectory::{AxisChainSet, ChainStage, CompiledChain, ShapedSegment, ShapedSignal};

use crate::lowering::{LoweringError, lower_move};

const SEGMENT_TIME_EPS_S: f64 = 1e-9;

#[derive(Debug, Clone, Copy)]
pub struct StreamConfig {
    pub chain: ChainFitConfig,
    pub velocity: VelocityConfig,
    pub fit_tol_mm: f64,
    pub keep_secs: f64,
    pub limits: VelocityLimits,
}

#[derive(Debug)]
pub enum StreamError {
    Fit(FitError),
    Velocity(VelocityError),
    Lowering(LoweringError),
    PostProcess(PostProcessError),
}

impl std::fmt::Display for StreamError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Fit(e) => write!(f, "chain fit: {e:?}"),
            Self::Velocity(e) => write!(f, "velocity plan: {e:?}"),
            Self::Lowering(e) => write!(f, "lowering: {e}"),
            Self::PostProcess(e) => write!(f, "post-processing: {e}"),
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

pub struct StreamState {
    buffer: VecDeque<Move>,
    entry_v: f64,
    odometer: Vec<f64>,
    t_committed: f64,
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

    #[must_use]
    pub fn limits(&self) -> VelocityLimits {
        self.config.limits
    }

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
            let seg = lower_move(
                gm,
                vm,
                t,
                &pos,
                self.config.fit_tol_mm,
                &self.axis_chains.chains,
            )?;
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

        self.post_history
            .extend(segs.into_iter().take(commit_count));
        self.trim_post_history();

        Ok(committed)
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
                track = apply_derivative_gain_to_track(&track, *k);
            }
            ChainStage::LinearPressureAdvance { .. } => {}
        }
    }
    track
}

fn apply_derivative_gain_to_track(
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
