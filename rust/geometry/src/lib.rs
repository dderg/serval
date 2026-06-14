#![cfg_attr(not(test), forbid(unsafe_code))]

pub mod curve;
pub mod error;
pub mod params;
pub mod pipeline;
pub(crate) mod reduce;
pub mod segment;
pub mod splitter;
pub mod telemetry;

pub use error::{Fatal, GeometryError, InternalDetails, InternalKind, Recovery, SlotDegeneracy};
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
