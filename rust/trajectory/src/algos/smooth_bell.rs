use super::{Bound, ParamSpec, PostProcessorAlgo};
use crate::chain::ChainStage;
use crate::kernel::build_smooth_bell_kernel;

#[derive(Debug)]
pub struct SmoothBell;

impl PostProcessorAlgo for SmoothBell {
    fn type_name(&self) -> &'static str {
        "smooth_bell"
    }

    fn params(&self) -> &'static [ParamSpec] {
        &[ParamSpec {
            key: "smooth_time",
            bound: Bound::NonNegative,
        }]
    }

    fn compile(&self, values: &[f64]) -> Option<ChainStage> {
        let [smooth_time] = values else {
            panic!("smooth_bell expects exactly one param value");
        };
        (*smooth_time > 0.0)
            .then(|| ChainStage::SmoothKernel(build_smooth_bell_kernel(*smooth_time)))
    }
}
