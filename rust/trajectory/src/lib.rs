#[cfg(test)]
mod fit;
mod kernel;
#[cfg(test)]
mod pad;
pub mod post_processor;
pub mod post_processors;
mod shaper;

pub use post_processor::{
    AxisChainSet, ChainStage, CompiledChain, PostProcessorError, PostProcessorInstance,
};
pub use shaper::ShapedSignal;

#[derive(Debug, Clone)]
pub struct ShapedSegment {
    pub axes: Vec<nurbs::ScalarNurbs<f64>>,
    pub followers: Vec<geometry::segment::FollowerDemand>,
    pub t_start: f64,
    pub t_end: f64,
    pub motor_mask: u8,
    pub source_line: u32,
}
