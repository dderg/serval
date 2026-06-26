#![cfg_attr(not(test), forbid(unsafe_code))]

pub mod curve;
pub mod error;
pub mod execution;
pub mod fitter;
pub mod frontend;
pub mod params;
pub mod path;
pub mod pipeline;
pub(crate) mod reduce;
pub mod segment;
pub mod splitter;
pub mod telemetry;
pub mod velocity;

pub use error::{Fatal, GeometryError, InternalDetails, InternalKind, Recovery, SlotDegeneracy};
pub use execution::lower_profile;
pub use fitter::{
    ArcFitConfig, ChainFitConfig, CornerFitConfig, FitError, FitOutcome, FitReport, UnblendReason,
    UnblendedJunction, fit_chain, fit_chain_with_head_restore, fit_corners,
};
pub use frontend::{FrontendError, Move, MoveContext, VelocityLimits, arc_move, line_move};
pub use params::FitterParams;
pub use pipeline::{GeometryPipeline, Item, Segments, degree_elevate_2_to_3};
pub use segment::{
    BlendFamily, CornerBlendSlot, CubicSegment, FollowerDemand, JunctionDeviation, Segment,
    SourceRange, SplitInfo,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FollowerWord {
    pub letter: u8,
    pub axis_index: usize,
}
pub use splitter::{SplitError, split_segment_to_cap};

pub use telemetry::TelemetryEvent;
pub use velocity::{
    MoveVelocity, StraightPhase, VelSample, VelocityConfig, VelocityError, VelocityProfile,
    VelocityReport, plan_velocity, plan_velocity_warm_start,
};
