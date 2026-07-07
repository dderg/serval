use super::{Bound, ParamSpec, PostProcessorAlgo};
use crate::kernel::build_smooth_triangle_kernel;
use crate::chain::ChainStage;

#[derive(Debug)]
pub struct SmoothTriangle;

impl PostProcessorAlgo for SmoothTriangle {
    fn type_name(&self) -> &'static str {
        "smooth_triangle"
    }

    fn params(&self) -> &'static [ParamSpec] {
        &[ParamSpec {
            key: "smooth_time",
            bound: Bound::NonNegative,
        }]
    }

    fn compile(&self, values: &[f64]) -> Option<ChainStage> {
        let [smooth_time] = values else {
            panic!("smooth_triangle expects exactly one param value");
        };
        (*smooth_time > 0.0)
            .then(|| ChainStage::SmoothKernel(build_smooth_triangle_kernel(*smooth_time)))
    }
}
