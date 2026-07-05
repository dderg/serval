use super::{Bound, ParamSpec, PostProcessorAlgo};
use crate::post_processor::ChainStage;

#[derive(Debug)]
pub struct LinearPressureAdvance;

impl PostProcessorAlgo for LinearPressureAdvance {
    fn type_name(&self) -> &'static str {
        "linear_pressure_advance"
    }

    fn params(&self) -> &'static [ParamSpec] {
        &[ParamSpec {
            key: "k",
            bound: Bound::NonNegative,
        }]
    }

    fn compile(&self, values: &[f64]) -> ChainStage {
        let [k] = values else {
            panic!("linear_pressure_advance expects exactly one param value");
        };
        ChainStage::LinearPressureAdvance { k: *k }
    }
}
