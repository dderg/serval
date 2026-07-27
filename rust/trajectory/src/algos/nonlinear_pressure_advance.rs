use super::{Bound, ParamSpec, PostProcessorAlgo};
use crate::chain::{ChainStage, NonlinearAdvance};

#[derive(Debug)]
pub struct NonlinearPressureAdvance;

impl PostProcessorAlgo for NonlinearPressureAdvance {
    fn type_name(&self) -> &'static str {
        "nonlinear_pressure_advance"
    }

    fn params(&self) -> &'static [ParamSpec] {
        &[
            ParamSpec {
                key: "linear_advance",
                bound: Bound::NonNegative,
            },
            ParamSpec {
                key: "nonlinear_offset",
                bound: Bound::NonNegative,
            },
            ParamSpec {
                key: "linearization_velocity",
                bound: Bound::Positive,
            },
        ]
    }

    fn compile(&self, values: &[f64]) -> Option<ChainStage> {
        let [linear_advance, nonlinear_offset, linearization_velocity] = values else {
            panic!("nonlinear_pressure_advance expects exactly three param values");
        };
        if *nonlinear_offset == 0.0 {
            if *linear_advance == 0.0 {
                return None;
            }
            return Some(ChainStage::DerivativeGains {
                k1: *linear_advance,
                k2: 0.0,
            });
        }
        Some(ChainStage::NonlinearAdvance(NonlinearAdvance {
            linear_advance: *linear_advance,
            nonlinear_offset: *nonlinear_offset,
            linearization_velocity: *linearization_velocity,
        }))
    }
}
