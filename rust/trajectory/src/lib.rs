pub mod algos;
pub mod chain;
mod convolution;
mod kernel;

pub use chain::{
    AxisChainSet, ChainStage, CompiledChain, NonlinearAdvance, PostProcessorError,
    PostProcessorInstance,
};
pub use convolution::ShapedSignal;
pub use kernel::{
    build_smooth_mzv_kernel, build_smooth_zv_kernel, SMOOTH_MZV_DURATION_PER_HZ,
    SMOOTH_ZV_DURATION_PER_HZ,
};

#[derive(Debug, Clone)]
pub struct ShapedSegment {
    pub axes: Vec<nurbs::ScalarNurbs>,
    pub followers: Vec<geometry::segment::FollowerDemand>,
    /// Whether the followers in this segment ride a spatial arc-length
    /// profile (projectable onto the leaders' shaped motion) as opposed to a
    /// virtual self-driven path (an extrude-only move) or a rest hold.
    pub spatial_path: bool,
    pub t_start: f64,
    pub t_end: f64,
    pub motor_mask: u8,
    pub source_line: u32,
}
