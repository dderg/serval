use super::{Bound, ParamSpec, PostProcessorAlgo};
use crate::kernel::build_smooth_mzv_kernel;
use crate::post_processor::ChainStage;

pub const SMOOTH_MZV_T_SM_PER_HZ: f64 = 0.95625;

#[derive(Debug)]
pub struct SmoothMzv;

impl PostProcessorAlgo for SmoothMzv {
    fn type_name(&self) -> &'static str {
        "smooth_mzv"
    }

    fn params(&self) -> &'static [ParamSpec] {
        &[ParamSpec {
            key: "frequency_hz",
            bound: Bound::Positive,
        }]
    }

    fn compile(&self, values: &[f64]) -> ChainStage {
        let [frequency_hz] = values else {
            panic!("smooth_mzv expects exactly one param value");
        };
        ChainStage::SmoothKernel(build_smooth_mzv_kernel(
            SMOOTH_MZV_T_SM_PER_HZ / frequency_hz,
        ))
    }
}
