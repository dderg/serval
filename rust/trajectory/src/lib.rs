pub mod continuous;

pub mod algos;
pub mod chain;
mod convolution;
mod kernel;

pub use chain::{
    AdvanceModel, AxisChainSet, ChainStage, CompiledChain, NonlinearAdvance, PostProcessorError,
    PostProcessorInstance,
};
pub use continuous::{
    AnalyticMoveSpan, BuzzProfile, ClockedMotorSpan, ContinuousAxis, ContinuousError,
    ContinuousSegment, MotorGroup, MotorSpan, MotorTerm, NudgeProfile, Pva, PvaBounds,
    RelativeSplinePiece, SurfaceMode, MAX_SPAN_SECS,
};
pub use convolution::ShapedSignal;
pub use kernel::{
    build_smooth_mzv_kernel, build_smooth_zv_kernel, SMOOTH_MZV_DURATION_PER_HZ,
    SMOOTH_ZV_DURATION_PER_HZ,
};
