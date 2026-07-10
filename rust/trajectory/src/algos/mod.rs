use crate::chain::{ChainStage, PostProcessorError};

mod linear_pressure_advance;
mod mode_inverse;
mod smooth_bell;
mod smooth_mzv;
mod smooth_triangle;
mod smooth_zv;

pub use linear_pressure_advance::LinearPressureAdvance;
pub use mode_inverse::ModeInverse;
pub use smooth_bell::SmoothBell;
pub use smooth_mzv::SmoothMzv;
pub use smooth_triangle::SmoothTriangle;
pub use smooth_zv::SmoothZv;

pub static REGISTRY: &[&dyn PostProcessorAlgo] = &[
    &SmoothBell,
    &SmoothTriangle,
    &SmoothZv,
    &SmoothMzv,
    &LinearPressureAdvance,
    &ModeInverse,
];

pub fn lookup(type_name: &str) -> Option<&'static dyn PostProcessorAlgo> {
    REGISTRY
        .iter()
        .copied()
        .find(|algo| algo.type_name() == type_name)
}

pub fn supported_type_names() -> Vec<&'static str> {
    REGISTRY.iter().map(|algo| algo.type_name()).collect()
}

pub trait PostProcessorAlgo: std::fmt::Debug + Send + Sync {
    fn type_name(&self) -> &'static str;
    fn params(&self) -> &'static [ParamSpec];
    /// `None` means the parameters compile to a no-op — the post-processor
    /// contributes no stage to the axis chain (e.g. `smooth_time = 0`).
    fn compile(&self, values: &[f64]) -> Option<ChainStage>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Bound {
    Positive,
    NonNegative,
    UnitInterval,
}

#[derive(Debug, Clone, Copy)]
pub struct ParamSpec {
    pub key: &'static str,
    pub bound: Bound,
}

impl ParamSpec {
    pub fn check(&self, owner_name: &str, value: f64) -> Result<(), PostProcessorError> {
        let ok = match self.bound {
            Bound::Positive => value.is_finite() && value > 0.0,
            Bound::NonNegative => value.is_finite() && value >= 0.0,
            Bound::UnitInterval => value.is_finite() && (0.0..1.0).contains(&value),
        };
        if ok {
            Ok(())
        } else {
            Err(PostProcessorError::BadParam {
                name: owner_name.to_string(),
                key: self.key.to_string(),
                value,
            })
        }
    }
}
