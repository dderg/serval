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
    NonlinearAdvance(NonlinearAdvance),
}

/// Which saturating shape the nonlinear pressure-advance term uses. Both are
/// odd, bounded by `nonlinear_offset`, and share the small-signal slope
/// `nonlinear_offset / linearization_velocity`; `Reciprocal` approaches its
/// bound far more slowly than `Tanh`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdvanceModel {
    Tanh,
    Reciprocal,
}

impl AdvanceModel {
    /// `s(u)`, `s'(u)`, `s''(u)` of the unit-scaled shape.
    fn shape(self, u: f64) -> (f64, f64, f64) {
        match self {
            Self::Tanh => {
                let t = libm::tanh(u);
                let sech2 = t.mul_add(-t, 1.0);
                (t, sech2, -2.0 * t * sech2)
            }
            // `u/(1 + |u|)` is the odd extension of bleeding-edge-v2's
            // `1 − 1/(1 + u)`: identical for the forward flow the model was
            // written for, but finite on retraction, where the original is
            // singular at `u = −1` and sign-flipped past it.
            Self::Reciprocal => {
                let d = 1.0 + u.abs();
                let d2 = d * d;
                (u / d, 1.0 / d2, -2.0 * u.signum() / (d2 * d))
            }
        }
    }
}

/// The operator `y = x + a(ẋ)` with the saturating advance law
/// `a(v) = linear_advance·v + nonlinear_offset·s(v / linearization_velocity)`.
///
/// Above `linearization_velocity` the extra term flattens out, so the
/// commanded advance stops growing with speed the way the purely linear
/// model does — the nonlinear pressure-advance model.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NonlinearAdvance {
    pub model: AdvanceModel,
    pub linear_advance: f64,
    pub nonlinear_offset: f64,
    pub linearization_velocity: f64,
}

impl NonlinearAdvance {
    #[must_use]
    pub fn advance(&self, v: f64) -> f64 {
        let (s, _, _) = self.model.shape(v / self.linearization_velocity);
        self.linear_advance.mul_add(v, self.nonlinear_offset * s)
    }

    /// `da/dv`
    #[must_use]
    pub fn slope(&self, v: f64) -> f64 {
        let (_, ds, _) = self.model.shape(v / self.linearization_velocity);
        (self.nonlinear_offset / self.linearization_velocity).mul_add(ds, self.linear_advance)
    }

    /// `d²a/dv²`
    #[must_use]
    pub fn curvature(&self, v: f64) -> f64 {
        let vl = self.linearization_velocity;
        let (_, _, dds) = self.model.shape(v / vl);
        self.nonlinear_offset / (vl * vl) * dds
    }
}

impl ChainStage {
    /// The input time window this stage's output at `t` depends on, relative
    /// to `t`. A convolution `(f ∗ k)(t)` reads `f` over `[t - k_hi, t - k_lo]`,
    /// so the kernel's support enters reflected; the two coincide only for
    /// symmetric kernels.
    #[must_use]
    pub fn input_window(&self) -> (f64, f64) {
        match self {
            Self::SmoothKernel(kernel) => {
                let (k_lo, k_hi) = kernel.support();
                (-k_hi, -k_lo)
            }
            Self::DerivativeGains { .. } | Self::NonlinearAdvance(_) => (0.0, 0.0),
        }
    }

    #[must_use]
    pub fn kernel_variance_s2(&self) -> f64 {
        match self {
            Self::SmoothKernel(kernel) => kernel.second_moment(),
            Self::DerivativeGains { .. } | Self::NonlinearAdvance(_) => 0.0,
        }
    }

    fn composition_slot(&self) -> (usize, &'static str) {
        match self {
            Self::SmoothKernel(_) => (0, "kernel"),
            Self::DerivativeGains { .. } | Self::NonlinearAdvance(_) => (1, "derivative-gain"),
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

    #[must_use]
    pub fn compile_stage(&self) -> Option<ChainStage> {
        self.algo.compile(&self.values)
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

    /// Whether this chain ends in derivative-gain stages after a smoothing
    /// kernel — the motor-side stages whose output (the motor command)
    /// intentionally departs from the toolhead signal.
    #[must_use]
    pub fn has_motor_side_gains(&self) -> bool {
        let mut seen_kernel = false;
        self.stages.iter().any(|stage| match stage {
            ChainStage::SmoothKernel(_) => {
                seen_kernel = true;
                false
            }
            ChainStage::DerivativeGains { .. } | ChainStage::NonlinearAdvance(_) => seen_kernel,
        })
    }

    /// Whether this chain transforms its axis with zero-support stages alone.
    /// Nothing widens the shaping window, so the shaper never refits the
    /// column: the transform is baked into the materialized source before the
    /// shaped frontier exists, and an axis riding this one as a leader cannot
    /// see it by comparing raw against shaped.
    #[must_use]
    pub fn is_zero_support_only(&self) -> bool {
        !self.stages.is_empty()
            && self
                .stages
                .iter()
                .all(|stage| !matches!(stage, ChainStage::SmoothKernel(_)))
    }

    #[must_use]
    pub fn kernel_variance_s2(&self) -> f64 {
        self.stages
            .iter()
            .map(ChainStage::kernel_variance_s2)
            .fold(0.0, f64::max)
    }

    #[must_use]
    pub fn max_input_window(&self) -> (f64, f64) {
        self.stages.iter().fold((0.0, 0.0), |(lo, hi), stage| {
            let (stage_lo, stage_hi) = stage.input_window();
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
        let (own_lo, own_hi) = self.chains[axis].max_input_window();
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
                    let (l_lo, l_hi) = self.chains[l].max_input_window();
                    (lo.min(l_lo), hi.max(l_hi))
                })
            })
    }

    #[must_use]
    pub fn has_motor_side_stages(&self) -> bool {
        self.chains.iter().any(CompiledChain::has_motor_side_gains)
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
            .map(|axis| self.chains[axis].max_input_window().1)
            .fold(0.0, f64::max)
    }

    /// The widest forward support any projected follower's own kernel needs
    /// on top of the projection frontier before its convolution over a
    /// segment is final.
    #[must_use]
    pub fn max_follower_own_forward_support(&self) -> f64 {
        self.projected_followers()
            .filter(|(axis, _)| self.has_own_kernel(*axis))
            .map(|(axis, _)| self.chains[axis].max_input_window().1)
            .fold(0.0, f64::max)
    }
}

#[cfg(test)]
mod tests;
