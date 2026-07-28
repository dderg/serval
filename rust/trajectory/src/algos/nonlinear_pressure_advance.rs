use super::{Bound, ParamSpec, PostProcessorAlgo};
use crate::chain::{AdvanceModel, ChainStage, NonlinearAdvance};

const PARAMS: &[ParamSpec] = &[
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
];

fn compile(model: AdvanceModel, values: &[f64]) -> Option<ChainStage> {
    let [linear_advance, nonlinear_offset, linearization_velocity] = values else {
        panic!("a nonlinear pressure advance expects exactly three param values");
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
        model,
        linear_advance: *linear_advance,
        nonlinear_offset: *nonlinear_offset,
        linearization_velocity: *linearization_velocity,
    }))
}

#[derive(Debug)]
pub struct TanhPressureAdvance;

impl PostProcessorAlgo for TanhPressureAdvance {
    fn type_name(&self) -> &'static str {
        "tanh_pressure_advance"
    }

    fn params(&self) -> &'static [ParamSpec] {
        PARAMS
    }

    fn compile(&self, values: &[f64]) -> Option<ChainStage> {
        compile(AdvanceModel::Tanh, values)
    }
}

#[derive(Debug)]
pub struct ReciprPressureAdvance;

impl PostProcessorAlgo for ReciprPressureAdvance {
    fn type_name(&self) -> &'static str {
        "recipr_pressure_advance"
    }

    fn params(&self) -> &'static [ParamSpec] {
        PARAMS
    }

    fn compile(&self, values: &[f64]) -> Option<ChainStage> {
        compile(AdvanceModel::Reciprocal, values)
    }
}
