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
    if d.ty == "linear_pressure_advance" {
        inst.set_param(required_param, required_value)?;
    }
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
    pub cartesian: CartesianLimits,
    pub runtime_caps: RuntimeCaps,
    pub runtime_square_corner_velocity: Option<f64>,
    pub chain: geometry::ChainFitConfig,
    pub post_processors: PostProcessorSet,
    pub max_extrude_only_velocity: Option<f64>,
    pub max_extrude_only_accel: Option<f64>,
    pub fit_tolerance_mm: f64,
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

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CartesianLimits {
    pub max_velocity: f64,
    pub max_accel: f64,
    pub max_jerk: f64,
    pub max_z_velocity: f64,
    pub max_z_accel: f64,
    pub square_corner_velocity: f64,
}

impl Default for CartesianLimits {
    fn default() -> Self {
        Self {
            max_velocity: 300.0,
            max_accel: 3000.0,
            max_jerk: 100_000.0,
            max_z_velocity: 15.0,
            max_z_accel: 100.0,
            square_corner_velocity: DEFAULT_SQUARE_CORNER_VELOCITY_MM_S,
        }
    }
}

impl CartesianLimits {
    pub fn validate(&self) -> Result<(), &'static str> {
        let ok = |c: f64| c.is_finite() && c > 0.0;
        if !(ok(self.max_velocity)
            && ok(self.max_accel)
            && ok(self.max_jerk)
            && ok(self.max_z_velocity)
            && ok(self.max_z_accel))
        {
            return Err("[printer] motion limits must be finite and positive");
        }
        if !(self.square_corner_velocity.is_finite() && self.square_corner_velocity >= 0.0) {
            return Err("[printer] square_corner_velocity must be finite and non-negative");
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

impl PlannerConfig {
    #[must_use]
    pub fn square_corner_velocity(&self) -> f64 {
        self.runtime_square_corner_velocity
            .unwrap_or(self.cartesian.square_corner_velocity)
    }

    #[must_use]
    pub fn effective_limits(&self) -> (f64, f64, f64) {
        let clamp = |cap: Option<f64>, base: f64| cap.map_or(base, |c| c.min(base));
        (
            clamp(self.runtime_caps.velocity, self.cartesian.max_velocity),
            clamp(self.runtime_caps.accel, self.cartesian.max_accel),
            self.square_corner_velocity(),
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
            cartesian: CartesianLimits::default(),
            runtime_caps: RuntimeCaps::default(),
            runtime_square_corner_velocity: None,
            chain: geometry::ChainFitConfig::default(),
            post_processors,
            max_extrude_only_velocity: None,
            max_extrude_only_accel: None,
            fit_tolerance_mm: 0.005,
        }
    }
}

#[cfg(test)]
mod tests;
