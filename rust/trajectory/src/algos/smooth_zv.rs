use super::{Bound, ParamSpec, PostProcessorAlgo};
use crate::kernel::build_smooth_zv_kernel;
use crate::chain::ChainStage;

pub const SMOOTH_ZV_T_SM_PER_HZ: f64 = 0.8025;

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
            SMOOTH_ZV_T_SM_PER_HZ / frequency_hz,
        )))
    }
}
