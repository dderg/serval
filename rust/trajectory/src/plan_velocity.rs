use crate::fit::FittedSegment;
use crate::post_processor::AxisChainSet;
use crate::{ShapeBatchInput, ShapeError, ShapeSegmentInput};

pub use crate::beta::{PlanOutput, PlanStats};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SafetyMode {
    TerminalKnown,
    WorstCaseFuture,
}

#[derive(Debug, Clone, Copy)]
pub struct PlanSegment<'a> {
    pub temporal: temporal::multi::SegmentInput<'a>,
    pub followers: &'a [geometry::segment::FollowerDemand],
    pub feedrate_mm_s: f64,
}

#[derive(Debug)]
pub struct PlanInput<'a> {
    pub segments: &'a [PlanSegment<'a>],
    pub grid_strategy: temporal::multi::GridStrategy,
    pub worker_threads: usize,
    pub chains: &'a AxisChainSet,
    pub fit_tolerance_mm: f64,
    pub beta_max_iters: u8,
    pub beta_convergence_ratio: f64,
    pub initial_v: f64,
    pub initial_a: f64,
    pub terminal_v: f64,
    pub safety_mode: SafetyMode,
    pub follower_history: Option<&'a temporal::FollowerHistory>,
    pub start_d2_override: Option<[f64; 3]>,
}

pub fn plan_velocity(input: &PlanInput<'_>) -> Result<PlanOutput, ShapeError> {
    if input.segments.is_empty() {
        return Err(ShapeError::EmptySegments);
    }

    if !input.initial_v.is_finite() || input.initial_v < 0.0 {
        return Err(ShapeError::UnsupportedBoundaryVelocity);
    }
    if !input.terminal_v.is_finite() || input.terminal_v < 0.0 {
        return Err(ShapeError::UnsupportedBoundaryVelocity);
    }
    if !input.initial_a.is_finite() || (input.initial_v == 0.0 && input.initial_a != 0.0) {
        return Err(ShapeError::UnsupportedBoundaryAccel);
    }

    let segments: Vec<ShapeSegmentInput<'_>> = input
        .segments
        .iter()
        .map(|s| ShapeSegmentInput {
            temporal: s.temporal,
            followers: s.followers,
            feedrate_mm_s: s.feedrate_mm_s,
        })
        .collect();

    let shape_input = ShapeBatchInput {
        segments: &segments,
        chains: input.chains,
        follower_start: &[],
        follower_history: input.follower_history,
        grid_strategy: input.grid_strategy,
        worker_threads: input.worker_threads,
        fit_tolerance_mm: input.fit_tolerance_mm,
        beta_max_iters: input.beta_max_iters,
        beta_convergence_ratio: input.beta_convergence_ratio,
        initial_v: input.initial_v,
        initial_a: input.initial_a,
        terminal_v: input.terminal_v,
        start_d2_override: input.start_d2_override,
    };

    crate::beta::plan_velocity_inner(&shape_input, input.safety_mode)
}

#[cfg(test)]
mod tests;
