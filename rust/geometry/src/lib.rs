#![cfg_attr(not(test), forbid(unsafe_code))]

pub(crate) const LENGTH_EPS_MM: f64 = 1e-9;

pub mod curve;
pub mod error;
pub mod fitter;
pub mod frontend;
pub mod path;
pub mod segment;
pub mod space;
pub mod surface;
pub(crate) mod vec3;
pub mod velocity;

pub use error::{Fatal, GeometryError, InternalDetails, InternalKind, Recovery, SlotDegeneracy};
pub use fitter::{CornerFitConfig, FitError, UnblendReason};
pub use frontend::{
    CORNER_DEVIATION_SCV_FACTOR, FrontendError, Move, MoveContext, VelocityLimits,
    corner_deviation_from_scv, line_move, scv_from_corner_deviation,
};
pub use segment::{CubicSegment, FollowerDemand, SourceRange};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FollowerWord {
    pub letter: u8,
    pub axis_index: usize,
}
pub use space::{GcodePos, MachinePos};
pub use surface::{
    Fade, MeshGrid, SurfaceBounds, SurfaceContinuity, SurfaceError, SurfaceSample,
    SurfaceTransform, SurfaceTransition, SurfaceTransitionError,
};
pub use velocity::{
    BoundaryState, LawSegment, MoveVelocity, ScalarLaw, VelSample, VelocityError, VelocityProfile,
    VelocityReport, plan_velocity_stops,
};
