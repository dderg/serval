use crate::algos::PostProcessorAlgo;
use nurbs::algebra::PiecewisePolynomialKernel;

#[derive(Debug, Clone)]
pub struct PostProcessorInstance {
    name: String,
    algo: &'static dyn PostProcessorAlgo,
    values: Vec<f64>,
}

/// `DerivativeGains` is the operator `y = x + k1·ẋ + k2·ẍ`.
#[derive(Debug, Clone)]
pub enum ChainStage {
    SmoothKernel(PiecewisePolynomialKernel),
    DerivativeGains { k1: f64, k2: f64 },
}

impl ChainStage {
    #[must_use]
    pub fn half_support(&self) -> (f64, f64) {
        match self {
            Self::SmoothKernel(kernel) => kernel.support(),
            Self::DerivativeGains { .. } => (0.0, 0.0),
        }
    }

    fn composition_slot(&self) -> (usize, &'static str) {
        match self {
            Self::SmoothKernel(_) => (0, "kernel"),
            Self::DerivativeGains { .. } => (1, "derivative-gain"),
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct CompiledChain {
    pub stages: Vec<ChainStage>,
}

#[derive(Debug, thiserror::Error)]
pub enum PostProcessorError {
    #[error("chain '{name}': unknown parameter '{key}'")]
    UnknownParam { name: String, key: String },
    #[error("chain '{name}': parameter '{key}' is out of range, got {value}")]
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
    #[error(
        "post-processor '{name}' carries an acceleration gain (k2 = {k2}) and \
         must come after a smoothing kernel in post_processors: applied before \
         the kernel it runs in the lowerer, whose curved-path sampler carries \
         no jerk and so cannot form the transformed velocity"
    )]
    AccelGainNeedsPrecedingKernel { name: String, k2: f64 },
}

impl PostProcessorInstance {
    pub fn new(name: &str, algo: &'static dyn PostProcessorAlgo, values: Vec<f64>) -> Self {
        assert_eq!(
            values.len(),
            algo.params().len(),
            "chain '{name}': {} expects {} param values, got {}",
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
}

impl CompiledChain {
    pub fn compile(chain: &[PostProcessorInstance]) -> Result<Self, PostProcessorError> {
        let mut compiled = Self::default();
        let mut slot_sources: [Option<&str>; 2] = [None, None];
        for inst in chain {
            inst.validate()?;
            let Some(stage) = inst.algo.compile(&inst.values) else {
                continue;
            };
            if let ChainStage::DerivativeGains { k2, .. } = stage {
                let after_kernel = compiled
                    .stages
                    .iter()
                    .any(|s| matches!(s, ChainStage::SmoothKernel(_)));
                if k2 != 0.0 && !after_kernel {
                    return Err(PostProcessorError::AccelGainNeedsPrecedingKernel {
                        name: inst.name().to_string(),
                        k2,
                    });
                }
            }
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
    pub fn spatial_from_kernels(kernels: &[Option<PiecewisePolynomialKernel>; 4]) -> Self {
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

    /// Follower axes that ride on at least one leader: their tracks are not
    /// convolved with their own kernel but re-projected onto the leaders'
    /// shaped motion by the shaper.
    pub fn projected_followers(&self) -> impl Iterator<Item = (usize, &[usize])> {
        self.followers
            .iter()
            .filter(|(_, leaders)| !leaders.is_empty())
            .map(|(axis, leaders)| (*axis, leaders.as_slice()))
    }

    #[must_use]
    pub fn is_projected_follower(&self, axis: usize) -> bool {
        self.projected_followers().any(|(a, _)| a == axis)
    }

    /// The half-support of everything this axis's emitted track depends on.
    /// A projected follower rides its leaders' shaped signal and then applies
    /// its own chain on top, so the supports cascade: they add.
    #[must_use]
    pub fn axis_support(&self, axis: usize) -> (f64, f64) {
        let (own_lo, own_hi) = self.chains[axis].max_half_support();
        let (lead_lo, lead_hi) = self.leaders_support(axis);
        (own_lo + lead_lo, own_hi + lead_hi)
    }

    /// The envelope of the leaders' kernel supports for a projected follower;
    /// `(0, 0)` for every other axis.
    #[must_use]
    pub fn leaders_support(&self, axis: usize) -> (f64, f64) {
        self.projected_followers()
            .find(|(a, _)| *a == axis)
            .map_or((0.0, 0.0), |(_, leaders)| {
                leaders.iter().fold((0.0, 0.0), |(lo, hi), &l| {
                    let (l_lo, l_hi) = self.chains[l].max_half_support();
                    (lo.min(l_lo), hi.max(l_hi))
                })
            })
    }

    #[must_use]
    pub fn has_own_kernel(&self, axis: usize) -> bool {
        self.chains[axis]
            .stages
            .iter()
            .any(|s| matches!(s, ChainStage::SmoothKernel(_)))
    }

    #[must_use]
    pub fn forward_support(&self) -> f64 {
        (0..self.n_axes())
            .map(|axis| self.axis_support(axis).1)
            .fold(0.0, f64::max)
    }

    #[must_use]
    pub fn back_support(&self) -> f64 {
        (0..self.n_axes())
            .map(|axis| self.axis_support(axis).0.abs())
            .fold(0.0, f64::max)
    }

    /// The widest forward support among directly-convolved (non-follower)
    /// axes: how far past a segment's end the raw stream must extend before
    /// every such axis's shaped track — and therefore a follower's projection
    /// onto them — is final over that segment.
    #[must_use]
    pub fn direct_forward_support(&self) -> f64 {
        (0..self.n_axes())
            .filter(|&axis| !self.is_projected_follower(axis))
            .map(|axis| self.chains[axis].max_half_support().1)
            .fold(0.0, f64::max)
    }

    /// The widest forward support any projected follower's own kernel needs
    /// on top of the projection frontier before its convolution over a
    /// segment is final.
    #[must_use]
    pub fn max_follower_own_forward_support(&self) -> f64 {
        self.projected_followers()
            .filter(|(axis, _)| self.has_own_kernel(*axis))
            .map(|(axis, _)| self.chains[axis].max_half_support().1)
            .fold(0.0, f64::max)
    }
}

#[cfg(test)]
mod tests;
