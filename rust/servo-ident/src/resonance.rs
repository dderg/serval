//! Resonance detection and the accel-sweep recommendation.
//!
//! Per the binding contract (`docs/rewrite/servo-cal-contracts.md`),
//! resonance is read off the moving-segment following-error PSD: the power of
//! the strongest peak in the 20-450 Hz band over the mean power of the 1-4 Hz
//! band, flagged at ratio >= 8.0. The band and ratio constants come from
//! `scripts/servo_gain_report.py`. The accel recommendation ports
//! `scripts/servo_accel_report.py::recommend`.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

pub const RESONANCE_BAND_HZ: (f64, f64) = (20.0, 450.0);
pub const LOW_BAND_HZ: (f64, f64) = (1.0, 4.0);
pub const RESONANCE_RATIO_LIMIT: f64 = 8.0;

#[derive(Debug, Serialize, Deserialize, JsonSchema)]
pub struct Resonance {
    pub detected: bool,
    pub ratio: f64,
    pub peak_hz: f64,
}

pub fn detect_resonance(freqs: &[f64], psd: &[f64]) -> Resonance {
    let (lo, hi) = RESONANCE_BAND_HZ;
    let mut peak_power = 0.0f64;
    let mut peak_hz = 0.0f64;
    for (i, &f) in freqs.iter().enumerate() {
        if f >= lo && f < hi && psd[i] > peak_power {
            peak_power = psd[i];
            peak_hz = f;
        }
    }
    let (llo, lhi) = LOW_BAND_HZ;
    let mut low_sum = 0.0f64;
    let mut low_n = 0usize;
    for (i, &f) in freqs.iter().enumerate() {
        if f >= llo && f < lhi {
            low_sum += psd[i];
            low_n += 1;
        }
    }
    let ratio = if low_n > 0 && low_sum > 0.0 {
        peak_power / (low_sum / low_n as f64)
    } else {
        0.0
    };
    Resonance {
        detected: ratio >= RESONANCE_RATIO_LIMIT,
        ratio,
        peak_hz,
    }
}

/// Port of `servo_accel_report.recommend`: highest accel with zero torque
/// rail on every motor. `steps` is `(accel, rail_detected)` in any order.
pub fn recommend_accel(steps: &[(f64, bool)]) -> (Option<f64>, String) {
    let clean: Vec<f64> = steps
        .iter()
        .filter(|(_, rail)| !rail)
        .map(|(a, _)| *a)
        .collect();
    if clean.is_empty() {
        return (
            None,
            "every accel step hit the torque rail — lower the accel".to_string(),
        );
    }
    let accel = clean.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let mut note = "highest accel with zero rail samples on every motor".to_string();
    let hit: Vec<f64> = steps
        .iter()
        .filter(|(a, rail)| *rail && *a > accel)
        .map(|(a, _)| *a)
        .collect();
    if let Some(&lowest) = hit
        .iter()
        .min_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal))
    {
        note += &format!("; {lowest:.0} mm/s^2 rejected (torque rail)");
    }
    (Some(accel), note)
}
