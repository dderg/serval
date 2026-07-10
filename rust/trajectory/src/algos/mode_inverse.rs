use super::{Bound, ParamSpec, PostProcessorAlgo};
use crate::chain::ChainStage;

#[derive(Debug)]
pub struct ModeInverse;

impl PostProcessorAlgo for ModeInverse {
    fn type_name(&self) -> &'static str {
        "mode_inverse"
    }

    fn params(&self) -> &'static [ParamSpec] {
        &[
            ParamSpec {
                key: "frequency_hz",
                bound: Bound::Positive,
            },
            ParamSpec {
                key: "damping_ratio",
                bound: Bound::UnitInterval,
            },
        ]
    }

    fn compile(&self, values: &[f64]) -> Option<ChainStage> {
        let [frequency_hz, damping_ratio] = values else {
            panic!("mode_inverse expects exactly two param values");
        };
        let omega = 2.0 * std::f64::consts::PI * frequency_hz;
        Some(ChainStage::DerivativeGains {
            k1: 2.0 * damping_ratio / omega,
            k2: 1.0 / (omega * omega),
        })
    }
}
