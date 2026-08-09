use super::{Bound, ParamSpec, PostProcessorAlgo};
use crate::chain::ChainStage;
use crate::kernel::build_smooth_mzv_kernel;

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

    fn compile(&self, values: &[f64]) -> Option<ChainStage> {
        let [frequency_hz] = values else {
            panic!("smooth_mzv expects exactly one param value");
        };
        Some(ChainStage::SmoothKernel(build_smooth_mzv_kernel(
            *frequency_hz,
        )))
    }
}
