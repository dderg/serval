pub mod deadline;

pub mod limits;
pub use limits::{
    AxisSet, LimitKind, LimitSet, Limits, LimitsError, MAX_AXES, MAX_LIMIT_SETS, N_SPATIAL,
    kappa_set, restricted_norm,
};

pub mod topp;
pub use topp::counters;
pub use topp::{
    ScheduleError, ToleranceMode, schedule_segment, schedule_segment_with_followers,
    schedule_segment_with_tolerance,
};

pub mod multi;
pub use multi::{
    BatchError, BatchInput, BatchOutput, GridStrategy, JoiningStatus, JunctionInfo, SegmentInput,
    plan_batch,
};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FollowerDemand {
    pub axis: usize,
    pub ratio: f64,
    pub pa_k: f64,
}

#[derive(Debug, Clone, Default)]
pub struct FollowerHistory {
    pub dt: f64,
    pub axis_velocity: [Vec<f64>; 3],
}

#[derive(Debug, Clone, Copy)]
pub struct GridConfig {
    pub scheme: GridScheme,
    pub n: usize,
}

#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GridScheme {
    UniformArclength,
}

#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum BindingConstraint {
    None,
    Velocity { set: usize },
    AccelNorm { set: usize },
    JerkNorm { set: usize },
    PaVelocity { set: usize },
    PaAccel { set: usize },
    PaJerk { set: usize },
    Boundary,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WorstBinding {
    pub constraint: BindingConstraint,
    pub ratio: f64,
    pub grid_index: usize,
    pub s: f64,
    pub kind: LimitKind,
}

#[derive(Debug, Clone, Default)]
pub struct BindingSummary {
    pub histogram: Vec<(BindingConstraint, u32)>,
    pub worst: Option<WorstBinding>,
}

#[derive(Debug, Clone, Copy)]
pub struct GridSample {
    pub s: f64,

    pub v: f64,

    pub a: f64,

    pub b: f64,
    pub binding: BindingConstraint,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoundarySide {
    Start,
    End,
}

#[non_exhaustive]
#[derive(Debug, Clone, Copy)]
pub enum InfeasibleReason {
    BoundaryAboveMVC { side: BoundarySide, mvc_b: f64 },

    BoundaryBelowMinReachable { side: BoundarySide, min_b: f64 },

    BoundaryAboveMaxReachable { side: BoundarySide, max_b: f64 },
    SolverInfeasible,
}

#[non_exhaustive]
#[derive(Debug, Clone, Copy)]
pub enum SolveStatus {
    Solved,
    SolvedInexact {
        residual: f64,
    },
    Infeasible {
        at_grid: usize,
        reason: InfeasibleReason,
    },
    MaxIter {
        last_residual: f64,
    },
    SolvedSlp {
        outer_iters: u32,
    },
    DivergedSlp {
        last_max_ratio: f64,
        outer_iters: u32,
    },
    MaxIterSlp {
        last_max_ratio: f64,
    },
}

#[derive(Debug, Clone)]
pub struct TopProfile {
    pub samples: Vec<GridSample>,
    pub status: SolveStatus,
    pub grid_scheme: GridScheme,
    pub total_time: f64,
    pub binding: BindingSummary,
    /// True when this profile's solve stopped refining because the real-time
    /// deadline expired, so it may sit further below the kinematic limit than
    /// a time-unbounded solve would. A slow-but-converged solve is `false`.
    pub deadline_truncated: bool,
}
