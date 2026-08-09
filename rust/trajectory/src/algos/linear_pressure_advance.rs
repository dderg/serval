use super::{Bound, ParamSpec, PostProcessorAlgo};
use crate::chain::ChainStage;

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

    fn compile(&self, values: &[f64]) -> Option<ChainStage> {
        let [k] = values else {
            panic!("linear_pressure_advance expects exactly one param value");
        };
        Some(ChainStage::DerivativeGains { k1: *k, k2: 0.0 })
    }
}
