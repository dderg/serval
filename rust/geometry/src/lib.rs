#![cfg_attr(not(test), forbid(unsafe_code))]

pub(crate) const LENGTH_EPS_MM: f64 = 1e-9;

/// The device's step-time resolution: the narrowest interval a trajectory
/// piece can mean anything over.
pub const MIN_DEVICE_INTERVAL_S: f64 = 2e-9;

pub mod curve;
pub mod error;
pub mod fitter;
pub mod frontend;
pub mod path;
pub mod ratio_ramp;
pub mod segment;
pub mod shaped_corner;
pub mod space;
pub mod step_capacity;
pub mod surface;
pub mod vec3;
pub mod velocity;

pub use error::GeometryError;
pub use fitter::{
    CornerFitConfig, FitError, SEAM_VELOCITY_STEP_MM_S, UnblendReason, seam_requires_stop,
};
pub use frontend::{
    CORNER_DEVIATION_SCV_FACTOR, CornerBudget, DEFAULT_CORNER_ALLOWANCE_MM, ExtruderCaps,
    FrontendError, LeaderSpill, LeaderSpillId, Move, MoveContext, RestSettle, RestSettleId,
    SPATIAL_AXIS_SETS, SettleDemand, VelocityLimits, axis_set, corner_deviation_from_scv,
    line_move, scv_from_corner_deviation,
};
pub use ratio_ramp::{
    ADVANCE_VALLEY_SPEED_MM_S, AdvanceValley, CRAWL_SIDE_WIDEST_MM, CrawlSide,
    RATIO_RAMP_ENVELOPE_SLOPE, RATIO_RAMP_MAX_TIME_S, RATIO_RAMP_MIN_TIME_S, RatioRampRule,
    RatioStep, SideWeight, TURNAROUND_SPEED_TIME_S, VALLEY_LEAD_BUDGET_S, VALLEY_RAMP_TIME_S,
    ValleyKind, coupled_bend, crawl_floor, crawl_side_width, leader_reach, rest_room_reach_mm,
    turns_around, valley_reach, widest_crawl_reach, widest_crawl_side_within,
};
pub use segment::{CubicSegment, FollowerDemand, SourceRange};
pub use shaped_corner::ShapedCorners;
pub use step_capacity::{FollowerStepLoad, McuStepLoad, StepCapacity, StepCapacityOverload};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FollowerWord {
    pub letter: u8,
    pub axis_index: usize,
}
pub use space::{GcodePos, MachinePos};
pub use surface::{
    Fade, MeshGrid, MeshGridSpec, MeshPatch, PatchAxis, SurfaceBounds, SurfaceContinuity,
    SurfaceError, SurfaceSample, SurfaceTransform, SurfaceTransition, SurfaceTransitionError,
};
pub use velocity::{
    BrakeMove, ExtrusionBudget, ExtrusionReach, FanOut, LawSegment, MoveVelocity, RailPair,
    RailPairShape, ScalarLaw, Serial, VelSample, VelocityError, VelocityPlanParams,
    VelocityProfile, VelocityReport, brake_horizon, brake_move, extrusion_budgets,
    plan_velocity_stops_reconstruct_prefix, plan_velocity_stops_select_prefix, replan_span,
};
