mod kernel;
pub mod chain;
pub mod algos;
mod convolution;

pub use chain::{
    AxisChainSet, ChainStage, CompiledChain, PostProcessorError, PostProcessorInstance,
};
pub use convolution::ShapedSignal;

#[derive(Debug, Clone)]
pub struct ShapedSegment {
    pub axes: Vec<nurbs::ScalarNurbs>,
    pub followers: Vec<geometry::segment::FollowerDemand>,
    pub t_start: f64,
    pub t_end: f64,
    pub motor_mask: u8,
    pub source_line: u32,
}
