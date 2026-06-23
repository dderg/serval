#![allow(deprecated)]

use std::collections::VecDeque;

use geometry::segment::CubicSegment;
use nurbs::algebra::PiecewisePolynomialKernel;
use nurbs::bezier::BezierPiece;

use crate::emit_shaped::EmitSegmentMeta;
use crate::fit::FittedSegment;
use crate::plan_velocity::{PlanStats, SafetyMode};
use crate::post_processor::AxisChainSet;

#[derive(Debug, Clone)]
pub struct ReplanReport {
    pub split_us: u64,
    pub solve_us: u64,
    pub rebuild_us: u64,
    pub window_segments: usize,
    pub plan: PlanStats,
    pub fallback_rung: u8,
    pub binding: crate::ReplanBindingSummary,
}

mod decel_finder;
mod emit;
pub(crate) mod state;

#[cfg(test)]
mod tests;

#[derive(Debug, Clone)]
pub struct AxisLane {
    pub pieces: VecDeque<BezierPiece<f64>>,
    pub kernel: Option<PiecewisePolynomialKernel<f64>>,
    pub h: f64,
}

#[derive(Debug, Clone)]
pub struct UncommittedMove {
    pub segment: CubicSegment,
    pub t_start: f64,
    pub t_end: f64,
}

#[derive(Debug, Clone)]
pub struct ReplanContext {
    pub limits: temporal::Limits,
    pub chains: AxisChainSet,
    pub fit_tolerance_mm: f64,
    pub beta_max_iters: u8,
    pub beta_convergence_ratio: f64,
    pub worker_threads: usize,
    pub grid_strategy: temporal::multi::GridStrategy,
    pub fallback_initial_v: f64,
    pub safety_mode: SafetyMode,
    pub force_full_resolve: bool,
}

#[derive(Debug, Clone, Copy)]
pub struct EmitContext<'a> {
    pub chains: &'a AxisChainSet,
}

#[derive(Debug)]
pub struct ShaperState {
    pub axes: Vec<AxisLane>,

    pub uncommitted_moves: VecDeque<UncommittedMove>,

    pub t_appended: f64,
    pub t_decel_start: f64,
    pub t_shaped: f64,
    pub t_dispatched: f64,

    pub(crate) planned_fitted: Vec<FittedSegment>,
    pub(crate) planned_meta: Vec<EmitSegmentMeta>,
    pub(crate) pending_freeze: Vec<crate::ShapedSegment>,
    pub(crate) follower_emit_start: Vec<f64>,
}
