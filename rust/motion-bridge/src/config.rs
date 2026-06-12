use thiserror::Error;
use trajectory::{AxisShaper, ShaperConfig};

#[derive(Debug, Error)]
pub enum ShaperConfigError {
    #[error("unsupported shaper type: '{kind}'. Use smooth_zv or smooth_mzv")]
    UnsupportedKind { kind: String },
}

const SPATIAL: [&str; 3] = ["x", "y", "z"];
const RESERVED_LETTERS: [u8; 9] = [b'i', b'j', b'p', b'q', b'f', b'g', b'm', b'n', b't'];

#[derive(Debug, Clone, PartialEq)]
pub struct AxisDecl {
    pub name: String,
    pub follows: Vec<String>,
    pub motors: Vec<String>,
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
    pub shaper: ShaperConfig,
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
                if section.max_velocity.is_none()
                    && section.max_accel.is_none()
                    && section.max_jerk.is_none()
                {
                    return Err(LimitConfigError::EmptySection {
                        section: section.name.clone(),
                    });
                }
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
                axes: temporal::AxisSet::all(),
                v_max: self.runtime_caps.velocity.unwrap_or(f64::INFINITY),
                a_max: a,
                j_max: if a.is_finite() {
                    a * JERK_DEFAULT_ACCEL_MULTIPLE
                } else {
                    f64::INFINITY
                },
            });
        }
        Ok(temporal::Limits::try_new(&sets)?)
    }
}

impl Default for PlannerConfig {
    fn default() -> Self {
        Self {
            axis_registry: AxisRegistry::default(),
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
            shaper: ShaperConfig {
                x: AxisShaper::Passthrough,
                y: AxisShaper::Passthrough,
                z: AxisShaper::Passthrough,
            },
            window_capacity: 32,
            beta_max_iters: 10,
            beta_convergence_ratio: 0.05,
            fit_tolerance_mm: 0.005,
            worker_threads: 3,
        }
    }
}

pub fn parse_axis_shaper(name: &str, freq: f64) -> Result<AxisShaper, ShaperConfigError> {
    match name {
        "" | "none" | "passthrough" => return Ok(AxisShaper::Passthrough),
        _ => {}
    }

    if !freq.is_finite() || freq <= 0.0 {
        return Ok(AxisShaper::Passthrough);
    }

    match name {
        "smooth_zv" | "smooth-zv" => Ok(AxisShaper::SmoothZv { frequency_hz: freq }),
        "smooth_mzv" | "smooth-mzv" => Ok(AxisShaper::SmoothMzv { frequency_hz: freq }),
        other => Err(ShaperConfigError::UnsupportedKind {
            kind: other.to_string(),
        }),
    }
}

#[cfg(test)]
mod tests;
