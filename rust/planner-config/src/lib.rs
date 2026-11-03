use thiserror::Error;
use trajectory::{AxisChainSet, CompiledChain, PostProcessorInstance, algos};

#[derive(Debug, Clone, PartialEq)]
pub struct PostProcessorDecl {
    pub name: String,
    pub ty: String,
    pub params: Vec<(String, f64)>,
}

#[derive(Debug, Error)]
pub enum PostProcessorConfigError {
    #[error("unsupported [post_processor {name}] type: '{kind}'. Supported types: {supported}")]
    UnsupportedKind {
        name: String,
        kind: String,
        supported: String,
    },
    #[error("duplicate [post_processor {name}]")]
    Duplicate { name: String },
    #[error("[post_processor {name}]: missing required parameter '{key}'")]
    MissingParam { name: String, key: String },
    #[error("{0}")]
    Param(#[from] trajectory::PostProcessorError),
    #[error("unknown post_processor '{name}'")]
    UnknownInstance { name: String },
    #[error("axis '{axis}': post_processors references undeclared '{name}'")]
    UnknownAxisReference { axis: String, name: String },
    #[error(
        "axis '{axis}' leads follower axes, and post_processor '{name}' is a \
         derivative-gain stage placed before the axis kernel. That is ambiguous \
         for followers: pre-kernel stages count as toolhead motion the followers \
         track, but derivative gains model the motor side, which followers must \
         ignore. Move '{name}' after the kernel in the axis post_processors list"
    )]
    LeaderGainBeforeKernel { axis: String, name: String },
    #[error(
        "axis '{axis}' follows other axes, and post_processor '{name}' advances it with no \
         smoothing kernel in the axis post_processors and no ratio_ramp_time: {}",
        trajectory::FOLLOWER_TRANSFORM_WITHOUT_KERNEL
    )]
    FollowerAdvanceWithoutKernel { axis: String, name: String },
    #[error(
        "axis '{axis}' follows other axes, and post_processor '{name}' advances it: the \
         planner holds the advance within the motor's limits, so set [axis {axis}] \
         max_velocity and max_accel (the [extruder] axis inherits them from \
         max_extrude_only_velocity and max_extrude_only_accel, which default from \
         nozzle_diameter and filament_diameter)"
    )]
    FollowerAdvanceWithoutLimits { axis: String, name: String },
}

#[derive(Debug, Clone)]
pub struct PostProcessorSet {
    instances: Vec<PostProcessorInstance>,
    per_axis: Vec<Vec<String>>,
}

impl PostProcessorSet {
    pub fn try_new(
        registry: &AxisRegistry,
        decls: &[PostProcessorDecl],
    ) -> Result<Self, PostProcessorConfigError> {
        let mut instances: Vec<PostProcessorInstance> = Vec::with_capacity(decls.len());
        for d in decls {
            if instances.iter().any(|i| i.name() == d.name) {
                return Err(PostProcessorConfigError::Duplicate {
                    name: d.name.clone(),
                });
            }
            instances.push(build_instance(d)?);
        }

        let per_axis: Vec<Vec<String>> = registry
            .decls()
            .iter()
            .map(|d| d.post_processors.clone())
            .collect();
        for (axis_decl, names) in registry.decls().iter().zip(&per_axis) {
            for name in names {
                if !instances.iter().any(|i| i.name() == *name) {
                    return Err(PostProcessorConfigError::UnknownAxisReference {
                        axis: axis_decl.name.clone(),
                        name: name.clone(),
                    });
                }
            }
        }

        let set = Self {
            instances,
            per_axis,
        };
        set.compile(registry)?;
        Ok(set)
    }

    pub fn compile(
        &self,
        registry: &AxisRegistry,
    ) -> Result<AxisChainSet, PostProcessorConfigError> {
        assert_eq!(
            self.per_axis.len(),
            registry.n_axes(),
            "post-processor set built against a different axis registry"
        );
        let chains: Vec<CompiledChain> = self
            .per_axis
            .iter()
            .map(|names| {
                let chain: Vec<PostProcessorInstance> = names
                    .iter()
                    .map(|n| {
                        self.instances
                            .iter()
                            .find(|i| i.name() == *n)
                            .expect("validated in try_new")
                            .clone()
                    })
                    .collect();
                CompiledChain::compile(&chain).map_err(PostProcessorConfigError::Param)
            })
            .collect::<Result<_, _>>()?;
        let chains = Self::with_follower_options(registry, chains);
        let followers = registry.follower_index_map();
        self.reject_pre_kernel_gains_on_leaders(registry, &followers)?;
        let chains = AxisChainSet { chains, followers };
        self.reject_follower_advance_without_kernel(registry, &chains)?;
        self.reject_follower_advance_without_limits(registry, &chains)?;
        Ok(chains)
    }

    fn reject_follower_advance_without_limits(
        &self,
        registry: &AxisRegistry,
        chains: &AxisChainSet,
    ) -> Result<(), PostProcessorConfigError> {
        for (axis, leaders) in &chains.followers {
            let decl = &registry.decls()[*axis];
            if leaders.is_empty()
                || chains.chains[*axis].transform().is_none()
                || (decl.max_velocity.is_some() && decl.max_accel.is_some())
            {
                continue;
            }
            return Err(PostProcessorConfigError::FollowerAdvanceWithoutLimits {
                axis: decl.name.clone(),
                name: self.transform_name(*axis).clone(),
            });
        }
        Ok(())
    }

    fn transform_name(&self, axis: usize) -> &String {
        self.per_axis[axis]
            .iter()
            .find(|name| {
                !matches!(
                    self.instances
                        .iter()
                        .find(|i| i.name() == name.as_str())
                        .expect("validated in try_new")
                        .compile_stage(),
                    None | Some(trajectory::ChainStage::SmoothKernel(_))
                )
            })
            .expect("a transformed chain names the stage that transforms it")
    }

    fn with_follower_options(
        registry: &AxisRegistry,
        chains: Vec<CompiledChain>,
    ) -> Vec<CompiledChain> {
        chains
            .into_iter()
            .zip(registry.decls())
            .map(|(chain, decl)| {
                let chain = match decl.ratio_ramp_time {
                    Some(ramp_time) => chain.with_ratio_ramp_time(ramp_time),
                    None => chain,
                };
                match decl.instantaneous_corner_velocity {
                    Some(mm_s) => chain.with_instantaneous_corner_velocity(mm_s),
                    None => chain,
                }
            })
            .collect()
    }

    fn reject_follower_advance_without_kernel(
        &self,
        registry: &AxisRegistry,
        chains: &AxisChainSet,
    ) -> Result<(), PostProcessorConfigError> {
        let Some(axis) = chains.kernel_free_follower_transform() else {
            return Ok(());
        };
        Err(PostProcessorConfigError::FollowerAdvanceWithoutKernel {
            axis: registry.axis_name(axis).to_string(),
            name: self.transform_name(axis).clone(),
        })
    }

    fn reject_pre_kernel_gains_on_leaders(
        &self,
        registry: &AxisRegistry,
        followers: &[(usize, Vec<usize>)],
    ) -> Result<(), PostProcessorConfigError> {
        let mut is_leader = vec![false; self.per_axis.len()];
        for (_, leaders) in followers {
            for &l in leaders {
                is_leader[l] = true;
            }
        }
        for (axis, names) in self.per_axis.iter().enumerate() {
            if !is_leader[axis] {
                continue;
            }
            let mut seen_kernel = false;
            for name in names {
                let inst = self
                    .instances
                    .iter()
                    .find(|i| i.name() == *name)
                    .expect("validated in try_new");
                match inst.compile_stage() {
                    Some(trajectory::ChainStage::SmoothKernel(_)) => seen_kernel = true,
                    Some(
                        trajectory::ChainStage::DerivativeGains { .. }
                        | trajectory::ChainStage::NonlinearAdvance(_),
                    ) if !seen_kernel => {
                        return Err(PostProcessorConfigError::LeaderGainBeforeKernel {
                            axis: registry.axis_name(axis).to_string(),
                            name: name.clone(),
                        });
                    }
                    _ => {}
                }
            }
        }
        Ok(())
    }

    #[must_use]
    pub fn param(&self, name: &str, key: &str) -> Option<f64> {
        self.instances
            .iter()
            .find(|i| i.name() == name)
            .and_then(|i| i.param(key))
    }

    pub fn set_param(
        &mut self,
        name: &str,
        key: &str,
        value: f64,
    ) -> Result<(), PostProcessorConfigError> {
        let inst = self
            .instances
            .iter_mut()
            .find(|i| i.name() == name)
            .ok_or_else(|| PostProcessorConfigError::UnknownInstance { name: name.into() })?;
        inst.set_param(key, value)?;
        Ok(())
    }

    pub fn compile_active_chains(
        &self,
        registry: &AxisRegistry,
        bypass: bool,
    ) -> Result<AxisChainSet, PostProcessorConfigError> {
        if bypass {
            let chains = (0..registry.n_axes())
                .map(|_| CompiledChain::compile(&[]).map_err(PostProcessorConfigError::Param))
                .collect::<Result<_, _>>()?;
            return Ok(AxisChainSet {
                chains,
                followers: registry.follower_index_map(),
            });
        }
        self.compile(registry)
    }
}

fn build_instance(
    d: &PostProcessorDecl,
) -> Result<PostProcessorInstance, PostProcessorConfigError> {
    let algo = algos::lookup(&d.ty).ok_or_else(|| PostProcessorConfigError::UnsupportedKind {
        name: d.name.clone(),
        kind: d.ty.clone(),
        supported: algos::supported_type_names().join(", "),
    })?;
    for (key, _) in &d.params {
        if !algo.params().iter().any(|spec| spec.key == key) {
            return Err(trajectory::PostProcessorError::UnknownParam {
                name: d.name.clone(),
                key: key.clone(),
            }
            .into());
        }
    }
    let values = algo
        .params()
        .iter()
        .map(|spec| {
            d.params
                .iter()
                .find(|(key, _)| key == spec.key)
                .map(|(_, value)| *value)
                .ok_or_else(|| PostProcessorConfigError::MissingParam {
                    name: d.name.clone(),
                    key: spec.key.to_string(),
                })
        })
        .collect::<Result<Vec<f64>, _>>()?;
    let inst = PostProcessorInstance::new(&d.name, algo, values);
    inst.validate()?;
    Ok(inst)
}

const SPATIAL: [&str; 3] = ["x", "y", "z"];
const RESERVED_LETTERS: [u8; 9] = [b'i', b'j', b'p', b'q', b'f', b'g', b'm', b'n', b't'];

#[derive(Debug, Clone, PartialEq)]
pub struct AxisDecl {
    pub name: String,
    pub follows: Vec<String>,
    pub motors: Vec<String>,
    pub post_processors: Vec<String>,
    /// `ratio_ramp_time`: on an axis that follows others, the time over
    /// which its extrusion-ratio steps are spread along the path, see
    /// [`trajectory::CompiledChain::ratio_ramp_time`].
    pub ratio_ramp_time: Option<f64>,
    /// `max_velocity`: what the motor of an axis that follows others can
    /// do for every move it makes, its advance included. For the
    /// `[extruder]` axis the reader inherits `max_extrude_only_velocity`
    /// when it is unset.
    pub max_velocity: Option<f64>,
    /// `max_accel`, inherited like [`AxisDecl::max_velocity`] from
    /// `max_extrude_only_accel`.
    pub max_accel: Option<f64>,
    /// `instantaneous_corner_velocity`: the velocity change the motor of an
    /// axis that follows others takes at once, see
    /// [`trajectory::CompiledChain::instantaneous_corner_velocity`].
    pub instantaneous_corner_velocity: Option<f64>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct AxisRegistry {
    ordered: Vec<AxisDecl>,
}

#[derive(Debug, Error)]
pub enum AxisConfigError {
    #[error("axis '{name}' must be a single ascii letter a-z")]
    BadName { name: String },
    #[error("axis '{name}': letter is reserved for G-code structure")]
    ReservedLetter { name: String },
    #[error("duplicate axis '{name}'")]
    Duplicate { name: String },
    #[error("required spatial axis '{name}' is not declared")]
    MissingSpatialAxis { name: String },
    #[error("axis '{axis}': follows references undeclared axis '{target}'")]
    UnknownFollowTarget { axis: String, target: String },
    #[error("spatial axis '{name}' cannot declare follows")]
    SpatialAxisCannotFollow { name: String },
    #[error("axis '{axis}' is motor-mapped twice: by [kinematics] and by [axis {axis}] motors:")]
    MotorMappingDuplicate { axis: String },
    #[error(
        "axis '{axis}' is not motor-mapped: claim it in a [kinematics] role or give [axis {axis}] a motors: key"
    )]
    MotorMappingMissing { axis: String },
    #[error("[kinematics] claims axis '{axis}' but no [axis {axis}] section is declared")]
    UnknownClaimedAxis { axis: String },
    #[error(
        "axis '{axis}': ratio_ramp_time spreads the extrusion-ratio steps of an axis that \
         follows others, and '{axis}' follows none"
    )]
    RatioRampWithoutLeaders { axis: String },
    #[error("axis '{axis}': ratio_ramp_time must be finite and above 0, got {value}")]
    BadRatioRampTime { axis: String, value: f64 },
    #[error(
        "axis '{axis}': {option} is the capability of an axis that follows others, and \
         '{axis}' follows none (spatial limits live in [printer])"
    )]
    LimitWithoutLeaders { axis: String, option: &'static str },
    #[error("axis '{axis}': {option} must be finite and above 0, got {value}")]
    BadLimit {
        axis: String,
        option: &'static str,
        value: f64,
    },
}

impl AxisRegistry {
    pub fn try_new(decls: Vec<AxisDecl>) -> Result<Self, AxisConfigError> {
        for d in &decls {
            let bytes = d.name.as_bytes();
            if bytes.len() != 1 || !bytes[0].is_ascii_lowercase() {
                return Err(AxisConfigError::BadName {
                    name: d.name.clone(),
                });
            }
            if RESERVED_LETTERS.contains(&bytes[0]) {
                return Err(AxisConfigError::ReservedLetter {
                    name: d.name.clone(),
                });
            }
            if decls.iter().filter(|o| o.name == d.name).count() > 1 {
                return Err(AxisConfigError::Duplicate {
                    name: d.name.clone(),
                });
            }
            if let Some(value) = d.ratio_ramp_time {
                if !(value.is_finite() && value > 0.0) {
                    return Err(AxisConfigError::BadRatioRampTime {
                        axis: d.name.clone(),
                        value,
                    });
                }
                if d.follows.is_empty() {
                    return Err(AxisConfigError::RatioRampWithoutLeaders {
                        axis: d.name.clone(),
                    });
                }
            }
            for (option, value) in [
                ("max_velocity", d.max_velocity),
                ("max_accel", d.max_accel),
                (
                    "instantaneous_corner_velocity",
                    d.instantaneous_corner_velocity,
                ),
            ] {
                let Some(value) = value else { continue };
                if !(value.is_finite() && value > 0.0) {
                    return Err(AxisConfigError::BadLimit {
                        axis: d.name.clone(),
                        option,
                        value,
                    });
                }
                if d.follows.is_empty() {
                    return Err(AxisConfigError::LimitWithoutLeaders {
                        axis: d.name.clone(),
                        option,
                    });
                }
            }
        }
        let mut ordered = Vec::with_capacity(decls.len());
        for name in SPATIAL {
            let d = decls
                .iter()
                .find(|d| d.name == name)
                .ok_or(AxisConfigError::MissingSpatialAxis { name: name.into() })?;
            if !d.follows.is_empty() {
                return Err(AxisConfigError::SpatialAxisCannotFollow { name: name.into() });
            }
            ordered.push(d.clone());
        }
        for d in &decls {
            if SPATIAL.contains(&d.name.as_str()) {
                continue;
            }
            for target in &d.follows {
                if !decls.iter().any(|o| &o.name == target) {
                    return Err(AxisConfigError::UnknownFollowTarget {
                        axis: d.name.clone(),
                        target: target.clone(),
                    });
                }
            }
            ordered.push(d.clone());
        }
        Ok(Self { ordered })
    }

    pub fn validate_motor_mapping(
        &self,
        kinematics_axes: &[String],
    ) -> Result<(), AxisConfigError> {
        for claimed in kinematics_axes {
            if !self.ordered.iter().any(|d| &d.name == claimed) {
                return Err(AxisConfigError::UnknownClaimedAxis {
                    axis: claimed.clone(),
                });
            }
        }
        for d in &self.ordered {
            let claimed = kinematics_axes.iter().any(|c| c == &d.name);
            let has_motors = !d.motors.is_empty();
            if claimed && has_motors {
                return Err(AxisConfigError::MotorMappingDuplicate {
                    axis: d.name.clone(),
                });
            }
            if !claimed && !has_motors {
                return Err(AxisConfigError::MotorMappingMissing {
                    axis: d.name.clone(),
                });
            }
        }
        Ok(())
    }

    pub fn axis_index(&self, name: &str) -> Result<usize, AxisConfigError> {
        self.ordered
            .iter()
            .position(|d| d.name == name)
            .ok_or(AxisConfigError::BadName { name: name.into() })
    }

    #[must_use]
    pub fn n_axes(&self) -> usize {
        self.ordered.len()
    }

    #[must_use]
    pub fn axis_name(&self, index: usize) -> &str {
        &self.ordered[index].name
    }

    #[must_use]
    pub fn decls(&self) -> &[AxisDecl] {
        &self.ordered
    }

    #[must_use]
    pub fn follower_index_map(&self) -> Vec<(usize, Vec<usize>)> {
        self.ordered
            .iter()
            .enumerate()
            .skip(SPATIAL.len())
            .map(|(idx, d)| {
                let followed = d
                    .follows
                    .iter()
                    .map(|t| self.axis_index(t).expect("follows validated in try_new"))
                    .collect();
                (idx, followed)
            })
            .collect()
    }
}

impl Default for AxisRegistry {
    fn default() -> Self {
        Self::try_new(
            SPATIAL
                .iter()
                .map(|name| AxisDecl {
                    name: (*name).to_string(),
                    follows: vec![],
                    motors: vec![],
                    post_processors: vec![],
                    ratio_ramp_time: None,
                    max_velocity: None,
                    max_accel: None,
                    instantaneous_corner_velocity: None,
                })
                .collect(),
        )
        .expect("spatial-only registry is always valid")
    }
}

#[derive(Debug, Clone)]
pub struct PlannerConfig {
    pub axis_registry: AxisRegistry,
    pub cartesian: CartesianLimits,
    pub runtime: RuntimeLimits,
    pub corner: geometry::CornerFitConfig,
    pub post_processors: PostProcessorSet,
    /// While set, `compile_active_chains` yields identity chains — the
    /// pipeline runs unshaped so a calibration transient measures the raw
    /// plant. The configured `post_processors` stay untouched for restore.
    pub post_processor_bypass: bool,
    pub max_extrude_only_velocity: Option<f64>,
    pub max_extrude_only_accel: Option<f64>,
    pub fit_tolerance_mm: f64,
    pub fit_tolerance_accel_mm_s2: f64,
}

/// The `SET_VELOCITY_LIMIT` / `M204` / `RESET_VELOCITY_LIMIT` overrides in
/// force. Velocity and accel caps clamp the config base but never raise it;
/// a runtime corner deviation replaces the base outright.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct RuntimeLimits {
    pub velocity: Option<f64>,
    pub accel: Option<f64>,
    pub corner_deviation: Option<f64>,
}

#[derive(Debug, Clone, PartialEq, Error)]
pub enum RuntimeLimitError {
    #[error("{name} must be finite and positive")]
    NotPositive { name: &'static str },
    #[error("corner_deviation must be finite and non-negative")]
    InvalidCornerDeviation,
}

impl RuntimeLimits {
    pub fn set_velocity_cap(&mut self, velocity: Option<f64>) -> Result<(), RuntimeLimitError> {
        self.velocity = require_positive(velocity, "velocity")?;
        Ok(())
    }

    pub fn set_accel_cap(&mut self, accel: Option<f64>) -> Result<(), RuntimeLimitError> {
        self.accel = require_positive(accel, "accel")?;
        Ok(())
    }

    pub fn set_corner_deviation(
        &mut self,
        corner_deviation: Option<f64>,
    ) -> Result<(), RuntimeLimitError> {
        if corner_deviation.is_some_and(|d| !(d.is_finite() && d >= 0.0)) {
            return Err(RuntimeLimitError::InvalidCornerDeviation);
        }
        self.corner_deviation = corner_deviation;
        Ok(())
    }

    /// `(velocity, accel, corner_deviation)` a move submitted now runs under.
    #[must_use]
    pub fn effective(&self, base: &CartesianLimits) -> (f64, f64, f64) {
        let clamp = |cap: Option<f64>, base: f64| cap.map_or(base, |c| c.min(base));
        (
            clamp(self.velocity, base.max_velocity),
            clamp(self.accel, base.max_accel),
            self.corner_deviation.unwrap_or(base.corner_deviation),
        )
    }
}

fn require_positive(
    value: Option<f64>,
    name: &'static str,
) -> Result<Option<f64>, RuntimeLimitError> {
    if value.is_some_and(|v| !(v.is_finite() && v > 0.0)) {
        return Err(RuntimeLimitError::NotPositive { name });
    }
    Ok(value)
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CartesianLimits {
    pub max_velocity: f64,
    pub max_accel: f64,
    pub max_jerk: f64,
    pub max_z_velocity: f64,
    pub max_z_accel: f64,
    pub corner_deviation: f64,
}

impl Default for CartesianLimits {
    fn default() -> Self {
        Self {
            max_velocity: 300.0,
            max_accel: 3000.0,
            max_jerk: 100_000.0,
            max_z_velocity: 15.0,
            max_z_accel: 100.0,
            corner_deviation: geometry::corner_deviation_from_scv(
                DEFAULT_SQUARE_CORNER_VELOCITY_MM_S,
                3000.0,
            ),
        }
    }
}

impl CartesianLimits {
    /// Limits for a printer without z caps of its own: the z caps sit at the
    /// path caps, where they can never bind (no move leans more than fully
    /// on z).
    #[must_use]
    pub fn without_z_caps(
        max_velocity: f64,
        max_accel: f64,
        max_jerk: f64,
        corner_deviation: f64,
    ) -> Self {
        Self {
            max_velocity,
            max_accel,
            max_jerk,
            max_z_velocity: max_velocity,
            max_z_accel: max_accel,
            corner_deviation,
        }
    }

    pub fn validate(&self) -> Result<(), &'static str> {
        let ok = |c: f64| c.is_finite() && c > 0.0;
        if !(ok(self.max_velocity)
            && ok(self.max_accel)
            && ok(self.max_z_velocity)
            && ok(self.max_z_accel))
        {
            return Err("[printer] motion limits must be finite and positive");
        }
        if !(self.max_jerk > 0.0) {
            return Err("[printer] max_jerk must be positive (infinity disables jerk limiting)");
        }
        if !(self.corner_deviation.is_finite() && self.corner_deviation >= 0.0) {
            return Err("[printer] corner_deviation must be finite and non-negative");
        }
        Ok(())
    }
}

pub const DEFAULT_SQUARE_CORNER_VELOCITY_MM_S: f64 = 5.0;

impl PlannerConfig {
    pub fn compile_active_chains(&self) -> Result<AxisChainSet, PostProcessorConfigError> {
        self.post_processors
            .compile_active_chains(&self.axis_registry, self.post_processor_bypass)
    }

    #[must_use]
    pub fn effective_limits(&self) -> (f64, f64, f64) {
        self.runtime.effective(&self.cartesian)
    }

    /// A ratio ramp takes the place of the follower's smoothing kernel, so
    /// the moves it makes on its own — retracts, primes, which its advance
    /// leaves untouched — are bounded by the extrude-only caps alone, and
    /// those must be set.
    pub fn check_ratio_ramp_extrude_only_caps(&self) -> Result<(), String> {
        let Some(ramped) = self
            .axis_registry
            .decls()
            .iter()
            .find(|decl| decl.ratio_ramp_time.is_some())
        else {
            return Ok(());
        };
        let finite = |cap: Option<f64>| cap.is_some_and(f64::is_finite);
        if finite(self.max_extrude_only_velocity) && finite(self.max_extrude_only_accel) {
            return Ok(());
        }
        Err(format!(
            "axis '{}' has a ratio_ramp_time, so no smoothing kernel bounds the moves it makes \
             on its own (retracts, primes): set finite [extruder] max_extrude_only_velocity \
             and max_extrude_only_accel",
            ramped.name
        ))
    }
}

impl Default for PlannerConfig {
    fn default() -> Self {
        let axis_registry = AxisRegistry::default();
        let post_processors = PostProcessorSet::try_new(&axis_registry, &[])
            .expect("empty post-processor set is always valid");
        Self {
            axis_registry,
            cartesian: CartesianLimits::default(),
            runtime: RuntimeLimits::default(),
            corner: geometry::CornerFitConfig::default(),
            post_processors,
            post_processor_bypass: false,
            max_extrude_only_velocity: None,
            max_extrude_only_accel: None,
            fit_tolerance_mm: 0.005,
            fit_tolerance_accel_mm_s2: 50.0,
        }
    }
}

mod extruder_caps;
pub use extruder_caps::extruder_caps;

#[cfg(feature = "doc")]
pub mod from_doc;

#[cfg(all(test, feature = "doc"))]
mod from_doc_tests;

#[cfg(test)]
mod tests;
