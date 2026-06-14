use thiserror::Error;
use trajectory::{AxisChainSet, CompiledChain, PostProcessorInstance, PostProcessorType};

#[derive(Debug, Clone, PartialEq)]
pub struct PostProcessorDecl {
    pub name: String,
    pub ty: String,
    pub params: Vec<(String, f64)>,
}

#[derive(Debug, Error)]
pub enum PostProcessorConfigError {
    #[error(
        "unsupported [post_processor {name}] type: '{kind}'. Use smooth_zv,          smooth_mzv or linear_pressure_advance"
    )]
    UnsupportedKind { name: String, kind: String },
    #[error("duplicate [post_processor {name}]")]
    Duplicate { name: String },
    #[error("[post_processor {name}]: missing required parameter '{key}'")]
    MissingParam { name: String, key: String },
    #[error("[post_processor {name}]: parameter '{key}' must be finite and > 0, got {value}")]
    BadParamValue {
        name: String,
        key: String,
        value: f64,
    },
    #[error("{0}")]
    Param(#[from] trajectory::PostProcessorError),
    #[error("unknown post_processor '{name}'")]
    UnknownInstance { name: String },
    #[error("axis '{axis}': post_processors references undeclared '{name}'")]
    UnknownAxisReference { axis: String, name: String },
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
        Ok(AxisChainSet { chains, followers })
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
    let required_param = match d.ty.as_str() {
        "smooth_zv" | "smooth_mzv" => "frequency_hz",
        "linear_pressure_advance" => "k",
        other => {
            return Err(PostProcessorConfigError::UnsupportedKind {
                name: d.name.clone(),
                kind: other.to_string(),
            });
        }
    };
    let required_value = d
        .params
        .iter()
        .find(|(k, _)| k == required_param)
        .map(|(_, v)| *v)
        .ok_or_else(|| PostProcessorConfigError::MissingParam {
            name: d.name.clone(),
            key: required_param.to_string(),
        })?;
    let ty = match d.ty.as_str() {
        "smooth_zv" => {
            require_positive(d, required_param, required_value)?;
            PostProcessorType::SmoothZv {
                frequency_hz: required_value,
            }
        }
        "smooth_mzv" => {
            require_positive(d, required_param, required_value)?;
            PostProcessorType::SmoothMzv {
                frequency_hz: required_value,
            }
        }
        "linear_pressure_advance" => PostProcessorType::LinearPressureAdvance { k: required_value },
        _ => unreachable!("ty validated above"),
    };
    let mut inst = PostProcessorInstance::new(&d.name, ty);
    for (key, value) in &d.params {
        if key == required_param {
            continue;
        }
        inst.set_param(key, *value)?;
    }
    Ok(inst)
}

fn require_positive(
    d: &PostProcessorDecl,
    key: &str,
    value: f64,
) -> Result<(), PostProcessorConfigError> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err(PostProcessorConfigError::BadParamValue {
            name: d.name.clone(),
            key: key.to_string(),
            value,
        })
    }
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

    /// `(follower_axis_index, followed_axis_indices)` per non-spatial axis.
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
    pub limit_sections: Vec<LimitSection>,
    pub runtime_caps: RuntimeCaps,
    pub post_processors: PostProcessorSet,
    pub window_capacity: usize,
    pub beta_max_iters: u8,
    pub beta_convergence_ratio: f64,
    pub fit_tolerance_mm: f64,
    pub worker_threads: usize,
}

#[derive(Debug, Clone, PartialEq)]
pub struct LimitSection {
    pub name: String,
    pub axes: Vec<usize>,
    pub max_velocity: Option<f64>,
    pub max_accel: Option<f64>,
    pub max_jerk: Option<f64>,
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct RuntimeCaps {
    pub velocity: Option<f64>,
    pub accel: Option<f64>,
}

#[derive(Debug, Error)]
pub enum LimitConfigError {
    #[error("[limit {section}]: declare at least one of max_velocity, max_accel, max_jerk")]
    EmptySection { section: String },
    #[error("[limit {section}]: mixing spatial and follower axes in one set is not yet supported")]
    MixedSpatialFollower { section: String },
    #[error(
        "follower axis '{axis}': no [limit] section declares max_velocity and max_accel covering it"
    )]
    NoFollowerCoverage { axis: String },
    #[error("invalid limit configuration: {0}")]
    Invalid(#[from] temporal::LimitsError),
}

pub const JERK_DEFAULT_ACCEL_MULTIPLE: f64 = 2.0;

impl LimitSection {
    fn to_set(&self) -> Result<temporal::LimitSet, LimitConfigError> {
        if self.max_velocity.is_none() && self.max_accel.is_none() && self.max_jerk.is_none() {
            return Err(LimitConfigError::EmptySection {
                section: self.name.clone(),
            });
        }
        let j_max = self
            .max_jerk
            .or(self.max_accel.map(|a| a * JERK_DEFAULT_ACCEL_MULTIPLE))
            .unwrap_or(f64::INFINITY);
        Ok(temporal::LimitSet {
            axes: temporal::AxisSet::from_indices(&self.axes),
            v_max: self.max_velocity.unwrap_or(f64::INFINITY),
            a_max: self.max_accel.unwrap_or(f64::INFINITY),
            j_max,
        })
    }
}

impl PlannerConfig {
    pub fn to_temporal_limits(&self) -> Result<temporal::Limits, LimitConfigError> {
        let mut sets = Vec::with_capacity(self.limit_sections.len() + 1);
        let n_axes = self.axis_registry.n_axes();
        let mut follower_velocity_covered = vec![false; n_axes];
        let mut follower_accel_covered = vec![false; n_axes];

        for section in &self.limit_sections {
            let all_spatial = section
                .axes
                .iter()
                .all(|&i| self.axis_registry.is_spatial(i));
            let all_follower = section
                .axes
                .iter()
                .all(|&i| !self.axis_registry.is_spatial(i));
            if all_spatial {
                sets.push(section.to_set()?);
            } else if all_follower {
                sets.push(section.to_set()?);
                for &i in &section.axes {
                    if section.max_velocity.is_some_and(f64::is_finite) {
                        follower_velocity_covered[i] = true;
                    }
                    if section.max_accel.is_some_and(f64::is_finite) {
                        follower_accel_covered[i] = true;
                    }
                }
            } else {
                return Err(LimitConfigError::MixedSpatialFollower {
                    section: section.name.clone(),
                });
            }
        }

        for i in 0..n_axes {
            if self.axis_registry.is_spatial(i) {
                continue;
            }
            if !follower_velocity_covered[i] || !follower_accel_covered[i] {
                return Err(LimitConfigError::NoFollowerCoverage {
                    axis: self.axis_registry.axis_name(i).to_string(),
                });
            }
        }

        if self.runtime_caps.velocity.is_some() || self.runtime_caps.accel.is_some() {
            let a = self.runtime_caps.accel.unwrap_or(f64::INFINITY);
            sets.push(temporal::LimitSet {
                axes: temporal::AxisSet::spatial(),
                v_max: self.runtime_caps.velocity.unwrap_or(f64::INFINITY),
                a_max: a,
                j_max: if a.is_finite() {
                    a * JERK_DEFAULT_ACCEL_MULTIPLE
                } else {
                    f64::INFINITY
                },
            });
        }
        Ok(temporal::Limits::try_new(
            &sets,
            n_axes.max(temporal::N_SPATIAL),
        )?)
    }
}

impl Default for PlannerConfig {
    fn default() -> Self {
        let axis_registry = AxisRegistry::default();
        let post_processors = PostProcessorSet::try_new(&axis_registry, &[])
            .expect("empty post-processor set is always valid");
        Self {
            axis_registry,
            limit_sections: vec![
                LimitSection {
                    name: "gantry".into(),
                    axes: vec![0, 1],
                    max_velocity: Some(300.0),
                    max_accel: Some(3000.0),
                    max_jerk: None,
                },
                LimitSection {
                    name: "z".into(),
                    axes: vec![2],
                    max_velocity: Some(15.0),
                    max_accel: Some(100.0),
                    max_jerk: None,
                },
            ],
            runtime_caps: RuntimeCaps::default(),
            post_processors,
            window_capacity: 32,
            beta_max_iters: 10,
            beta_convergence_ratio: 0.05,
            fit_tolerance_mm: 0.005,
            worker_threads: 3,
        }
    }
}

#[cfg(test)]
mod tests;
