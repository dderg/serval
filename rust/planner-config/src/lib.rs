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
        let followers = registry.follower_index_map();
        self.reject_pre_kernel_gains_on_leaders(registry, &followers)?;
        Ok(AxisChainSet { chains, followers })
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
                    Some(trajectory::ChainStage::DerivativeGains { .. }) if !seen_kernel => {
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
    pub fn is_spatial(&self, index: usize) -> bool {
        index < SPATIAL.len()
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

    #[must_use]
    pub fn follower_words(&self) -> Vec<geometry::FollowerWord> {
        self.ordered
            .iter()
            .enumerate()
            .skip(SPATIAL.len())
            .map(|(axis_index, d)| geometry::FollowerWord {
                letter: d.name.as_bytes()[0].to_ascii_uppercase(),
                axis_index,
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
    pub runtime_caps: RuntimeCaps,
    pub runtime_corner_deviation: Option<f64>,
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

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct RuntimeCaps {
    pub velocity: Option<f64>,
    pub accel: Option<f64>,
    /// REPLACES the static `[printer] max_jerk` while set — unlike
    /// velocity/accel this is not a min-cap, because calibration
    /// (SERVO_MEASURE_RINGDOWN) needs to RAISE jerk (to infinity) so the
    /// stop transient excites the plant unsmoothed.
    pub jerk_override: Option<f64>,
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

    #[must_use]
    pub fn for_move(&self, dx: f64, dy: f64, dz: f64) -> (f64, f64) {
        let d = (dx * dx + dy * dy + dz * dz).sqrt();
        let mut v = self.max_velocity;
        let mut a = self.max_accel;
        if d > 1e-9 && dz.abs() > 1e-9 {
            let z_unit = dz.abs() / d;
            v = v.min(self.max_z_velocity / z_unit);
            a = a.min(self.max_z_accel / z_unit);
        }
        (v, a)
    }
}

pub const DEFAULT_SQUARE_CORNER_VELOCITY_MM_S: f64 = 5.0;

pub fn validate_corner_budget(
    corner_deviation_mm: f64,
    max_accel_mm_s2: f64,
    chains: &AxisChainSet,
) -> Result<(), String> {
    if !(corner_deviation_mm > 0.0) {
        return Ok(());
    }
    for (axis, chain) in SPATIAL.iter().zip(&chains.chains) {
        let kernel_deviation_mm =
            geometry::kernel_corner_deviation_mm(chain.kernel_variance_s2(), max_accel_mm_s2);
        if kernel_deviation_mm >= corner_deviation_mm {
            return Err(format!(
                "smoothing kernel on axis {axis} already deviates \
                 {kernel_deviation_mm:.4} mm at accel {max_accel_mm_s2} mm/s^2, \
                 which exhausts corner_deviation = {corner_deviation_mm:.4} mm \
                 — increase corner_deviation or shorten the kernel"
            ));
        }
    }
    Ok(())
}

impl PlannerConfig {
    #[must_use]
    pub fn corner_deviation(&self) -> f64 {
        self.runtime_corner_deviation
            .unwrap_or(self.cartesian.corner_deviation)
    }

    /// The chains the pipeline should run right now: the configured
    /// post-processors, or identity chains while `post_processor_bypass`
    /// is set. Every live chain push must come through here so a
    /// parameter update during a bypass window cannot silently re-arm
    /// shaping.
    pub fn compile_active_chains(&self) -> Result<AxisChainSet, PostProcessorConfigError> {
        if self.post_processor_bypass {
            let chains = (0..self.axis_registry.n_axes())
                .map(|_| CompiledChain::compile(&[]).map_err(PostProcessorConfigError::Param))
                .collect::<Result<_, _>>()?;
            return Ok(AxisChainSet {
                chains,
                followers: self.axis_registry.follower_index_map(),
            });
        }
        self.post_processors.compile(&self.axis_registry)
    }

    pub fn validate_corner_budget(&self, chains: &AxisChainSet) -> Result<(), String> {
        validate_corner_budget(self.corner_deviation(), self.cartesian.max_accel, chains)
    }

    #[must_use]
    pub fn effective_limits(&self) -> (f64, f64, f64) {
        let clamp = |cap: Option<f64>, base: f64| cap.map_or(base, |c| c.min(base));
        (
            clamp(self.runtime_caps.velocity, self.cartesian.max_velocity),
            clamp(self.runtime_caps.accel, self.cartesian.max_accel),
            self.corner_deviation(),
        )
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
            runtime_caps: RuntimeCaps::default(),
            runtime_corner_deviation: None,
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

#[cfg(feature = "doc")]
pub mod from_doc;

#[cfg(all(test, feature = "doc"))]
mod from_doc_tests;

#[cfg(test)]
mod tests;
