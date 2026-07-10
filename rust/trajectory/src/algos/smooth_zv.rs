use super::{Bound, ParamSpec, PostProcessorAlgo};
use crate::chain::ChainStage;
use crate::kernel::build_smooth_zv_kernel;

#[derive(Debug)]
pub struct SmoothZv;

impl PostProcessorAlgo for SmoothZv {
    fn type_name(&self) -> &'static str {
        "smooth_zv"
    }

    fn params(&self) -> &'static [ParamSpec] {
        &[ParamSpec {
            key: "frequency_hz",
            bound: Bound::Positive,
        }]
    }

    fn compile(&self, values: &[f64]) -> Option<ChainStage> {
        let [frequency_hz] = values else {
            panic!("smooth_zv expects exactly one param value");
        };
        Some(ChainStage::SmoothKernel(build_smooth_zv_kernel(
            *frequency_hz,
        )))
    }
}
