use crate::post_processors::PostProcessorAlgo;
use nurbs::algebra::PiecewisePolynomialKernel;

#[derive(Debug, Clone)]
pub struct PostProcessorInstance {
    name: String,
    algo: &'static dyn PostProcessorAlgo,
    values: Vec<f64>,
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

    fn composition_slot(&self) -> (usize, &'static str) {
        match self {
            Self::SmoothKernel(_) => (0, "kernel"),
            Self::LinearPressureAdvance { .. } => (1, "derivative-gain"),
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
    #[error("post_processor '{name}': parameter '{key}' is out of range, got {value}")]
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
    pub fn new(name: &str, algo: &'static dyn PostProcessorAlgo, values: Vec<f64>) -> Self {
        assert_eq!(
            values.len(),
            algo.params().len(),
            "post_processor '{name}': {} expects {} param values, got {}",
            algo.type_name(),
            algo.params().len(),
            values.len()
        );
        Self {
            name: name.to_string(),
            algo,
            values,
        }
    }

    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    #[must_use]
    pub fn algo(&self) -> &'static dyn PostProcessorAlgo {
        self.algo
    }

    #[must_use]
    pub fn param(&self, key: &str) -> Option<f64> {
        self.algo
            .params()
            .iter()
            .position(|spec| spec.key == key)
            .map(|idx| self.values[idx])
    }

    pub fn validate(&self) -> Result<(), PostProcessorError> {
        self.algo
            .params()
            .iter()
            .zip(&self.values)
            .try_for_each(|(spec, value)| spec.check(&self.name, *value))
    }

    pub fn set_param(&mut self, key: &str, value: f64) -> Result<(), PostProcessorError> {
        let idx = self
            .algo
            .params()
            .iter()
            .position(|spec| spec.key == key)
            .ok_or_else(|| PostProcessorError::UnknownParam {
                name: self.name.clone(),
                key: key.to_string(),
            })?;
        self.algo.params()[idx].check(&self.name, value)?;
        self.values[idx] = value;
        Ok(())
    }

    #[must_use]
    pub fn into_chain(self) -> CompiledChain {
        CompiledChain::compile(&[self]).expect("a single post-processor always compiles")
    }
}

impl CompiledChain {
    pub fn compile(chain: &[PostProcessorInstance]) -> Result<Self, PostProcessorError> {
        let mut compiled = Self::default();
        let mut slot_sources: [Option<&str>; 2] = [None, None];
        for inst in chain {
            inst.validate()?;
            let stage = inst.algo.compile(&inst.values);
            let (slot, kind) = stage.composition_slot();
            if let Some(prev) = slot_sources[slot] {
                return Err(PostProcessorError::UnsupportedComposition {
                    detail: format!(
                        "second {kind} post-processor '{}' after '{prev}'",
                        inst.name()
                    ),
                });
            }
            slot_sources[slot] = Some(inst.name());
            compiled.stages.push(stage);
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
