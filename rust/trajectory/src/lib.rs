mod beta;
pub mod emit_shaped;
pub mod fit;
mod kernel;
pub mod odometer;
mod pad;
mod parallel;
pub mod peak;
pub mod plan_velocity;
pub mod post_processor;
mod reparam;
mod shaper;
mod smooth_fit;
pub mod streaming;
pub mod utilization;

pub use beta::{ReplanBindingSummary, ReplanWorstBinding};
pub use emit_shaped::{emit_shaped, EmitSegmentMeta, PerAxisHistory, ShapeEmission};
pub use plan_velocity::{plan_velocity, PlanInput, PlanOutput, PlanSegment, PlanStats, SafetyMode};
pub use post_processor::{
    AxisChainSet, CompiledChain, PostProcessorError, PostProcessorInstance, PostProcessorType,
};
pub use streaming::ReplanReport;

#[derive(Debug)]
pub struct ShapeBatchInput<'a> {
    pub segments: &'a [ShapeSegmentInput<'a>],
    pub chains: &'a AxisChainSet,
    pub follower_start: &'a [f64],
    pub follower_history: Option<&'a temporal::FollowerHistory>,
    pub grid_strategy: temporal::multi::GridStrategy,
    pub worker_threads: usize,
    pub fit_tolerance_mm: f64,
    pub beta_max_iters: u8,
    pub beta_convergence_ratio: f64,
    pub initial_v: f64,
    pub initial_a: f64,
    pub terminal_v: f64,
    pub start_d2_override: Option<[f64; 3]>,
}

#[derive(Debug, Clone, Copy)]
pub struct ShapeSegmentInput<'a> {
    pub temporal: temporal::multi::SegmentInput<'a>,
    pub followers: &'a [geometry::segment::FollowerDemand],
    pub feedrate_mm_s: f64,
}

#[derive(Debug)]
pub struct ShapeBatchOutput {
    pub segments: Vec<ShapedSegment>,
    pub beta_iters: u8,
    pub temporal_status: temporal::multi::JoiningStatus,
    pub beta_warning: Option<BetaWarning>,
}

#[derive(Debug, Clone)]
pub struct BetaWarning {
    pub worst_ratio: f64,
    pub segments_exceeding: Vec<usize>,
}

#[derive(Debug, Clone)]
pub struct ShapedSegment {
    pub axes: Vec<nurbs::ScalarNurbs<f64>>,
    pub followers: Vec<geometry::segment::FollowerDemand>,
    pub t_start: f64,
    pub t_end: f64,
    pub motor_mask: u8,
    /// Gcode line of the move that produced this segment. Carried through
    /// lowering so the pipeline-stage (`pipe_*`) logs can trace one gcode line
    /// from fit to the pump hand-off. Zero when the producing path has no source
    /// line (batch shaping / synthetic segments).
    pub source_line: u32,
}

#[derive(Debug, thiserror::Error)]
pub enum ShapeError {
    #[error("temporal batch error: {0}")]
    TemporalBatch(#[from] temporal::multi::BatchError),
    #[error("temporal joining: {0:?}{1}")]
    TemporalJoining(temporal::multi::JoiningStatus, String),
    #[error("segment {index} unsolvable: {status:?}")]
    SegmentUnsolvable {
        index: usize,
        status: temporal::SolveStatus,
    },
    #[error("fit failure on segment {index}: {detail:?}")]
    FitFailure {
        index: usize,
        detail: nurbs::algebra::FitError,
    },
    #[error("algebra error on segment {index}: {detail}")]
    Algebra {
        index: usize,
        detail: nurbs::AlgebraError,
    },
    #[error(
        "segment {index}: zero tangent (cusp) at u={u} — the curve has a stationary \
         point and is not plannable as a single smooth segment"
    )]
    ZeroTangent { index: usize, u: f64 },
    #[error("arc-length table error on segment {index}: {detail}")]
    ArcLength { index: usize, detail: String },
    #[error("empty segment buffer")]
    EmptySegments,
    #[error("unsupported boundary velocity: initial_v and terminal_v must be finite and ≥ 0.0")]
    UnsupportedBoundaryVelocity,
    #[error("unsupported boundary accel: initial_a must be finite, and 0.0 when initial_v is 0.0")]
    UnsupportedBoundaryAccel,
    #[error(
        "witness fallback (rung 3) failed — single-segment rest-to-rest plan unsolvable; \
         rung1: {rung1}; rung3: {rung3}"
    )]
    WitnessFallbackFailed {
        rung1: Box<ShapeError>,
        rung3: Box<ShapeError>,
    },
}

pub fn shape_batch(input: &ShapeBatchInput<'_>) -> Result<ShapeBatchOutput, ShapeError> {
    if input.segments.is_empty() {
        return Err(ShapeError::EmptySegments);
    }

    beta::beta_loop(input)
}

#[cfg(test)]
mod tests;
