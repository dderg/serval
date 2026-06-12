use thiserror::Error;
use trajectory::{AxisShaper, ELimits, ShaperConfig};

#[derive(Debug, Error)]
pub enum ShaperConfigError {
    #[error("unsupported shaper type: '{kind}'. Use smooth_zv or smooth_mzv")]
    UnsupportedKind { kind: String },
}

#[derive(Debug, Clone)]
pub struct PlannerConfig {
    pub limit_sections: Vec<LimitSection>,
    pub runtime_caps: RuntimeCaps,
    pub shaper: ShaperConfig,
    pub e_limits: ELimits,
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
    #[error("unknown axis '{name}' in [limit] section (supported: x, y, z)")]
    UnknownAxis { name: String },
    #[error("[limit {section}]: declare at least one of max_velocity, max_accel, max_jerk")]
    EmptySection { section: String },
    #[error("invalid limit configuration: {0}")]
    Invalid(#[from] temporal::LimitsError),
}

pub fn axis_index(name: &str) -> Result<usize, LimitConfigError> {
    match name {
        "x" => Ok(0),
        "y" => Ok(1),
        "z" => Ok(2),
        other => Err(LimitConfigError::UnknownAxis {
            name: other.to_string(),
        }),
    }
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
        for section in &self.limit_sections {
            sets.push(section.to_set()?);
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
            e_limits: ELimits {
                v_max: 50.0,
                a_max: 5000.0,
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
