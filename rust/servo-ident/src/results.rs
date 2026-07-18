//! Wire structs for `manifest.json` (read) and `results.json` /
//! `plot_series.json` (written), matching `docs/rewrite/servo-cal-contracts.md`.

use std::collections::BTreeMap;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use ts_rs::TS;

use crate::metrics::Metrics;
use crate::resonance::Resonance;

#[derive(Debug, Clone, Deserialize, Serialize, JsonSchema, TS)]
pub struct Applied {
    pub servo: String,
    pub addr: String,
    #[serde(rename = "type")]
    pub ty: String,
    pub value: Value,
}

#[derive(Debug, Deserialize)]
pub struct Step {
    pub name: String,
    #[serde(default)]
    pub swept: Value,
    #[serde(default)]
    pub applied: Vec<Applied>,
    pub capture: String,
    #[serde(default)]
    pub accel: Option<String>,
    #[serde(default)]
    pub stops: Option<Vec<f64>>,
}

impl Step {
    pub fn swept_value(&self, key: &str) -> Option<f64> {
        self.swept.get(key).and_then(Value::as_f64)
    }

    pub fn swept_max(&self) -> Option<f64> {
        match &self.swept {
            Value::Object(m) => m
                .values()
                .filter_map(Value::as_f64)
                .fold(None, |acc, v| Some(acc.map_or(v, |a: f64| a.max(v)))),
            _ => None,
        }
    }
}

#[derive(Debug, Deserialize)]
pub struct Manifest {
    #[serde(default)]
    pub version: i64,
    pub experiment: String,
    #[serde(default)]
    pub tag: String,
    #[serde(default)]
    pub axis: Option<String>,
    #[serde(default)]
    pub kinematics: Option<String>,
    #[serde(default)]
    pub belts: Option<String>,
    #[serde(default)]
    pub stroke_plan: Value,
    #[serde(default)]
    pub ff_lead_cycles: u64,
    pub steps: Vec<Step>,
}

#[derive(Debug, Serialize, Deserialize, JsonSchema, TS)]
pub struct DriveResult {
    pub metrics: Metrics,
    pub psd_peaks: Vec<(f64, f64)>,
    pub resonance: Resonance,
}

#[derive(Debug, Serialize, Deserialize, JsonSchema, TS)]
pub struct Combined {
    pub on_ferr_peak_mm: f64,
    pub on_ferr_rms_mm: f64,
    pub cross_ferr_peak_mm: f64,
}

#[derive(Debug, Serialize, Deserialize, JsonSchema, TS)]
pub struct AccelResult {
    pub present: bool,
    pub psd_peaks: Vec<(f64, f64)>,
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema, TS)]
pub struct DifferentialMode {
    pub freq_hz: f64,
    pub gain: f64,
    pub gain_db: f64,
    pub damping: Option<f64>,
    pub coherence: f64,
}

#[derive(Debug, Serialize, Deserialize, JsonSchema, TS)]
pub struct DifferentialResult {
    pub pair: Vec<String>,
    pub segments: usize,
    pub modes: Vec<DifferentialMode>,
}

#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema, TS)]
pub struct RingdownMode {
    pub freq_hz: f64,
    pub zeta: f64,
    pub zeta_lo: f64,
    pub zeta_hi: f64,
    pub amp: f64,
    pub disp_um: f64,
    pub tails: usize,
    pub cycles: f64,
    pub r2: f64,
    pub fit_start_ms: f64,
}

#[derive(Debug, Serialize, Deserialize, JsonSchema, TS)]
pub struct RingdownSource {
    pub source: String,
    pub unit: String,
    pub tails: usize,
    pub noise_floor: f64,
    pub modes: Vec<RingdownMode>,
}

#[derive(Debug, Serialize, Deserialize, JsonSchema, TS)]
pub struct RingdownResult {
    pub guard_ms: f64,
    pub window_ms: f64,
    pub sources: Vec<RingdownSource>,
}

#[derive(Debug, Serialize, Deserialize, JsonSchema, TS)]
pub struct StepResult {
    pub name: String,
    pub drives: BTreeMap<String, DriveResult>,
    pub combined: Option<Combined>,
    pub accel: Option<AccelResult>,
    pub differential: Option<DifferentialResult>,
    #[serde(default)]
    pub ringdown: Option<RingdownResult>,
    pub flags: Vec<String>,
}

#[derive(Debug, Serialize, Deserialize, JsonSchema, TS)]
pub struct Verdict {
    pub recommended_step: Option<String>,
    pub reason: String,
    pub flags: Vec<String>,
    pub apply: Option<Vec<Applied>>,
}

#[derive(Debug, Serialize, Deserialize, JsonSchema, TS)]
pub struct Results {
    pub version: i64,
    pub fs_hz: f64,
    pub settle_band_counts: i64,
    pub torque_limit_per_mille: i64,
    pub steps: Vec<StepResult>,
    pub verdict: Verdict,
}

#[derive(Debug, Serialize, Deserialize, JsonSchema, TS)]
pub struct PlotDrive {
    pub ferr_counts: Vec<f64>,
    pub torque_per_mille: Vec<f64>,
}

#[derive(Debug, Serialize, Deserialize, JsonSchema, TS)]
pub struct PlotCombined {
    pub on_ferr_mm: Vec<f64>,
    pub cross_ferr_mm: Vec<f64>,
}

#[derive(Debug, Serialize, Deserialize, JsonSchema, TS)]
pub struct PlotAccel {
    pub t_s: Vec<f64>,
    pub magnitude: Vec<f64>,
}

#[derive(Debug, Serialize, Deserialize, JsonSchema, TS)]
pub struct PlotPsdAccel {
    pub freq_hz: Vec<f64>,
    pub psd: Vec<f64>,
}

#[derive(Debug, Serialize, Deserialize, JsonSchema, TS)]
pub struct PlotPsd {
    pub freq_hz: Vec<f64>,
    pub per_drive: BTreeMap<String, Vec<f64>>,
    pub accel: Option<PlotPsdAccel>,
}

#[derive(Debug, Serialize, Deserialize, JsonSchema, TS)]
pub struct PlotDifferential {
    pub freq_hz: Vec<f64>,
    pub mag_db: Vec<f64>,
    pub phase_deg: Vec<f64>,
    pub coherence: Vec<f64>,
    pub torque_db: Vec<f64>,
    pub coherence_min: f64,
    pub band: (f64, f64),
    pub modes: Vec<DifferentialMode>,
}

#[derive(Debug, Serialize, Deserialize, JsonSchema, TS)]
pub struct PlotRingdownTail {
    pub start_s: f64,
    pub value: Vec<f64>,
}

#[derive(Debug, Serialize, Deserialize, JsonSchema, TS)]
pub struct PlotRingdownSource {
    pub source: String,
    pub unit: String,
    pub fs_hz: f64,
    pub modes: Vec<RingdownMode>,
    pub psd_freq_hz: Vec<f64>,
    pub psd: Vec<f64>,
    pub tails: Vec<PlotRingdownTail>,
}

#[derive(Debug, Serialize, Deserialize, JsonSchema, TS)]
pub struct PlotRingdown {
    pub sources: Vec<PlotRingdownSource>,
}

#[derive(Debug, Serialize, Deserialize, JsonSchema, TS)]
pub struct PlotStep {
    pub name: String,
    pub fs_hz: f64,
    pub stride: usize,
    pub t_s: Vec<f64>,
    pub moving: Vec<(f64, f64)>,
    pub drives: BTreeMap<String, PlotDrive>,
    pub combined: Option<PlotCombined>,
    pub accel: Option<PlotAccel>,
    pub differential: Option<PlotDifferential>,
    #[serde(default)]
    pub ringdown: Option<PlotRingdown>,
    pub psd: PlotPsd,
}

#[derive(Debug, Serialize, Deserialize, JsonSchema, TS)]
pub struct PlotSeries {
    pub version: i64,
    pub steps: Vec<PlotStep>,
}
