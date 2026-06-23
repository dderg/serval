use crate::kernel::{build_smooth_mzv_kernel, build_smooth_zv_kernel};
use nurbs::algebra::PiecewisePolynomialKernel;

pub const SMOOTH_ZV_T_SM_PER_HZ: f64 = 0.8025;
pub const SMOOTH_MZV_T_SM_PER_HZ: f64 = 0.95625;

#[derive(Debug, Clone, PartialEq)]
pub enum PostProcessorType {
    SmoothZv { frequency_hz: f64 },
    SmoothMzv { frequency_hz: f64 },
    LinearPressureAdvance { k: f64 },
}

impl PostProcessorType {
    #[must_use]
    pub fn into_chain(self) -> CompiledChain {
        CompiledChain::compile(&[PostProcessorInstance::new("inline", self)])
            .expect("a single post-processor always compiles")
    }
}

#[derive(Debug, Clone)]
pub struct PostProcessorInstance {
    name: String,
    ty: PostProcessorType,
}

#[derive(Debug, Clone)]
pub enum ChainStage {
    SmoothKernel(PiecewisePolynomialKernel<f64>),
    LinearPressureAdvance { k: f64 },
}

impl ChainStage {
    #[must_use]
    pub fn half_support(&self) -> (f64, f64) {
        match self {
            Self::SmoothKernel(kernel) => kernel.support(),
            Self::LinearPressureAdvance { .. } => (0.0, 0.0),
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct CompiledChain {
    pub stages: Vec<ChainStage>,
}

#[derive(Debug, thiserror::Error)]
pub enum PostProcessorError {
    #[error("post_processor '{name}': unknown parameter '{key}'")]
    UnknownParam { name: String, key: String },
    #[error("post_processor '{name}': '{key}' must be finite and >= 0, got {value}")]
    BadParam {
        name: String,
        key: String,
        value: f64,
    },
    #[error(
        "axis chain unsupported: {detail}. v1 allows at most one kernel and one \
         derivative-gain post-processor per axis"
    )]
    UnsupportedComposition { detail: String },
}

impl PostProcessorInstance {
    pub fn new(name: &str, ty: PostProcessorType) -> Self {
        Self {
            name: name.to_string(),
            ty,
        }
    }

    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    #[must_use]
    pub fn ty(&self) -> &PostProcessorType {
        &self.ty
    }

    pub fn validate(&self) -> Result<(), PostProcessorError> {
        match self.ty {
            PostProcessorType::SmoothZv { frequency_hz }
            | PostProcessorType::SmoothMzv { frequency_hz } => {
                if frequency_hz.is_finite() && frequency_hz > 0.0 {
                    Ok(())
                } else {
                    Err(PostProcessorError::BadParam {
                        name: self.name.clone(),
                        key: "frequency_hz".to_string(),
                        value: frequency_hz,
                    })
                }
            }
            PostProcessorType::LinearPressureAdvance { k } => {
                if k.is_finite() && k >= 0.0 {
                    Ok(())
                } else {
                    Err(PostProcessorError::BadParam {
                        name: self.name.clone(),
                        key: "k".to_string(),
                        value: k,
                    })
                }
            }
        }
    }

    pub fn set_param(&mut self, key: &str, value: f64) -> Result<(), PostProcessorError> {
        let unknown = || PostProcessorError::UnknownParam {
            name: self.name.clone(),
            key: key.to_string(),
        };
        match &mut self.ty {
            PostProcessorType::SmoothZv { frequency_hz }
            | PostProcessorType::SmoothMzv { frequency_hz } => {
                if key == "frequency_hz" {
                    if !(value.is_finite() && value > 0.0) {
                        return Err(PostProcessorError::BadParam {
                            name: self.name.clone(),
                            key: key.to_string(),
                            value,
                        });
                    }
                    *frequency_hz = value;
                    Ok(())
                } else {
                    Err(unknown())
                }
            }
            PostProcessorType::LinearPressureAdvance { k } => {
                if key == "k" {
                    if !(value.is_finite() && value >= 0.0) {
                        return Err(PostProcessorError::BadParam {
                            name: self.name.clone(),
                            key: key.to_string(),
                            value,
                        });
                    }
                    *k = value;
                    Ok(())
                } else {
                    Err(unknown())
                }
            }
        }
    }
}

impl CompiledChain {
    pub fn compile(chain: &[PostProcessorInstance]) -> Result<Self, PostProcessorError> {
        let mut compiled = Self::default();
        let mut kernel_source: Option<&str> = None;
        let mut gain_source: Option<&str> = None;
        for inst in chain {
            inst.validate()?;
            match inst.ty() {
                PostProcessorType::SmoothZv { frequency_hz } => {
                    if let Some(prev) = kernel_source {
                        return Err(PostProcessorError::UnsupportedComposition {
                            detail: format!(
                                "second kernel post-processor '{}' after '{prev}'",
                                inst.name()
                            ),
                        });
                    }
                    kernel_source = Some(inst.name());
                    compiled
                        .stages
                        .push(ChainStage::SmoothKernel(build_smooth_zv_kernel(
                            SMOOTH_ZV_T_SM_PER_HZ / *frequency_hz,
                        )));
                }
                PostProcessorType::SmoothMzv { frequency_hz } => {
                    if let Some(prev) = kernel_source {
                        return Err(PostProcessorError::UnsupportedComposition {
                            detail: format!(
                                "second kernel post-processor '{}' after '{prev}'",
                                inst.name()
                            ),
                        });
                    }
                    kernel_source = Some(inst.name());
                    compiled
                        .stages
                        .push(ChainStage::SmoothKernel(build_smooth_mzv_kernel(
                            SMOOTH_MZV_T_SM_PER_HZ / *frequency_hz,
                        )));
                }
                PostProcessorType::LinearPressureAdvance { k } => {
                    if let Some(prev) = gain_source {
                        return Err(PostProcessorError::UnsupportedComposition {
                            detail: format!(
                                "second derivative-gain post-processor '{}' after '{prev}'",
                                inst.name()
                            ),
                        });
                    }
                    gain_source = Some(inst.name());
                    compiled
                        .stages
                        .push(ChainStage::LinearPressureAdvance { k: *k });
                }
            }
        }
        Ok(compiled)
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.stages.is_empty()
    }

    #[must_use]
    pub fn max_half_support(&self) -> (f64, f64) {
        self.stages.iter().fold((0.0, 0.0), |(lo, hi), stage| {
            let (stage_lo, stage_hi) = stage.half_support();
            (lo.min(stage_lo), hi.max(stage_hi))
        })
    }
}

#[derive(Debug, Clone, Default)]
pub struct AxisChainSet {
    pub chains: Vec<CompiledChain>,
    pub followers: Vec<(usize, Vec<usize>)>,
}

impl AxisChainSet {
    #[must_use]
    pub fn spatial(x: CompiledChain, y: CompiledChain, z: CompiledChain) -> Self {
        Self {
            chains: vec![x, y, z],
            followers: Vec::new(),
        }
    }

    #[must_use]
    pub fn passthrough_spatial() -> Self {
        Self {
            chains: vec![CompiledChain::default(); 3],
            followers: Vec::new(),
        }
    }

    #[must_use]
    pub fn spatial_from_kernels(kernels: &[Option<PiecewisePolynomialKernel<f64>>; 4]) -> Self {
        assert!(
            kernels[3].is_none(),
            "spatial_from_kernels: E-slot kernel must be None; follower chains \
             are declared via AxisChainSet::followers"
        );
        Self {
            chains: kernels[..3]
                .iter()
                .map(|k| CompiledChain {
                    stages: k.iter().cloned().map(ChainStage::SmoothKernel).collect(),
                })
                .collect(),
            followers: Vec::new(),
        }
    }

    #[must_use]
    pub fn n_axes(&self) -> usize {
        self.chains.len()
    }

    #[must_use]
    pub fn is_follower_axis(&self, axis: usize) -> bool {
        self.followers.iter().any(|(f, _)| *f == axis)
    }
}

#[cfg(test)]
mod tests;
