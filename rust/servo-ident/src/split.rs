use crate::capture::Capture;
use crate::model::{coulomb_sign, PairDiscoveryError, PhysicalParams, Structure};
use crate::prep::{filter_segments, median_dt, segments, sinc_kernel};
use crate::profile_out::PairSplit;

pub const DIRECTION_SPLIT_LIMIT: f64 = 0.5;

pub struct SplitCapture<'a> {
    pub cap: &'a Capture,
    pub residual_filt: &'a [Vec<f64>],
    pub keep: &'a [bool],
}

#[derive(Debug, PartialEq)]
pub enum SplitError {
    PairDiscovery(PairDiscoveryError),
    ShapeMismatch(&'static str),
    InsufficientExcitation { first: usize, second: usize },
}

#[derive(Debug)]
pub struct PairReport {
    pub split: PairSplit,
    pub lambda: f64,
    pub fitted_direction_split: f64,
    pub stderr: f64,
    pub t_value: f64,
    pub intercept: f64,
    pub rms_before: f64,
    pub rms_after: f64,
    pub samples: usize,
    pub rejected: bool,
}

fn rms(values: &[f64]) -> f64 {
    (values.iter().map(|value| value * value).sum::<f64>() / values.len() as f64).sqrt()
}

fn belt_force_magnitude(
    structure: &Structure,
    params: &PhysicalParams,
    cap: &Capture,
    first: usize,
    cutoff_hz: f64,
) -> Vec<f64> {
    let mut raw = vec![0.0; cap.t.len()];
    for (sample, magnitude) in raw.iter_mut().enumerate() {
        let mut base_first = 0.0;
        for mode in 0..structure.mode_count() {
            let mut acc_mode = 0.0;
            let mut vel_mode = 0.0;
            for slot in 0..structure.axis_count() {
                let frame = structure.frame[mode][slot];
                acc_mode += frame * cap.acc[slot][sample];
                vel_mode += frame * cap.vel[slot][sample];
            }
            let mode_force = params.mass[mode] * acc_mode
                + params.viscous[mode] * vel_mode
                + params.coulomb[mode] * coulomb_sign(vel_mode);
            base_first += structure.frame[mode][first] * mode_force;
        }
        *magnitude = 2.0 * base_first.abs();
    }
    let dt = median_dt(&cap.t);
    let capture_segments = segments(&cap.t, dt);
    let kernel = if cutoff_hz > 0.0 {
        Some(sinc_kernel(cutoff_hz, dt))
    } else {
        None
    };
    filter_segments(&raw, &capture_segments, kernel.as_deref())
}

pub fn fit_pair_splits(
    structure: &Structure,
    params: &PhysicalParams,
    cutoff_hz: f64,
    capture: &SplitCapture<'_>,
) -> Result<Vec<PairReport>, SplitError> {
    let pairs = structure.pairs().map_err(SplitError::PairDiscovery)?;
    if capture.keep.len() != capture.cap.t.len() {
        return Err(SplitError::ShapeMismatch("keep mask length"));
    }
    if capture.residual_filt.len() != structure.axis_count()
        || capture
            .residual_filt
            .iter()
            .any(|channel| channel.len() != capture.cap.t.len())
    {
        return Err(SplitError::ShapeMismatch("filtered residual shape"));
    }
    let mut reports = Vec::with_capacity(pairs.len());

    for pair in pairs {
        let predictor_all =
            belt_force_magnitude(structure, params, capture.cap, pair.first, cutoff_hz);
        let mut predictor = Vec::new();
        let mut differential = Vec::new();
        for sample in 0..capture.keep.len() {
            if capture.keep[sample] {
                predictor.push(predictor_all[sample]);
                differential.push(
                    capture.residual_filt[pair.first][sample]
                        - pair.lambda * capture.residual_filt[pair.second][sample],
                );
            }
        }
        let samples = predictor.len();
        if samples < 3 {
            return Err(SplitError::InsufficientExcitation {
                first: pair.first,
                second: pair.second,
            });
        }
        let predictor_mean = predictor.iter().sum::<f64>() / samples as f64;
        let differential_mean = differential.iter().sum::<f64>() / samples as f64;
        let predictor_centered_norm2: f64 = predictor
            .iter()
            .map(|value| {
                let centered = value - predictor_mean;
                centered * centered
            })
            .sum();
        if predictor_centered_norm2 == 0.0 {
            return Err(SplitError::InsufficientExcitation {
                first: pair.first,
                second: pair.second,
            });
        }
        let fitted_direction_split = predictor
            .iter()
            .zip(&differential)
            .map(|(x, y)| (x - predictor_mean) * (y - differential_mean))
            .sum::<f64>()
            / predictor_centered_norm2;
        let intercept = differential_mean - fitted_direction_split * predictor_mean;
        let after: Vec<f64> = predictor
            .iter()
            .zip(&differential)
            .map(|(x, y)| y - intercept - fitted_direction_split * x)
            .collect();
        let sigma2 = after.iter().map(|value| value * value).sum::<f64>() / (samples - 2) as f64;
        let stderr = (sigma2 / predictor_centered_norm2).sqrt();
        let t_value = if stderr == 0.0 {
            if fitted_direction_split == 0.0 {
                0.0
            } else {
                fitted_direction_split.signum() * f64::INFINITY
            }
        } else {
            fitted_direction_split / stderr
        };
        let rejected = !fitted_direction_split.is_finite()
            || fitted_direction_split.abs() >= DIRECTION_SPLIT_LIMIT;
        reports.push(PairReport {
            split: PairSplit {
                first: pair.first,
                second: pair.second,
                direction_split: if rejected {
                    0.0
                } else {
                    fitted_direction_split
                },
            },
            lambda: pair.lambda,
            fitted_direction_split,
            stderr,
            t_value,
            intercept,
            rms_before: rms(&differential),
            rms_after: rms(&after),
            samples,
            rejected,
        });
    }
    Ok(reports)
}

pub fn report_pair_splits(reports: &[PairReport], axes: &[&str]) -> Vec<PairSplit> {
    let mut accepted = Vec::with_capacity(reports.len());
    for report in reports {
        let first = axes[report.split.first];
        let second = axes[report.split.second];
        eprintln!(
            "pair {first}/{second} (lambda={:+.0}): direction_split={:+.6}, stderr={:.3e}, t={:+.2}, intercept={:+.3}; rms(D) {:.2} -> {:.2}, {} samples",
            report.lambda,
            report.fitted_direction_split,
            report.stderr,
            report.t_value,
            report.intercept,
            report.rms_before,
            report.rms_after,
            report.samples,
        );
        if report.rejected {
            eprintln!(
                "  WARNING pair {first}/{second}: abs(direction_split) {:.6} is at or above the {:.1} cap; zeroed and omitted from the profile",
                report.fitted_direction_split.abs(),
                DIRECTION_SPLIT_LIMIT,
            );
        } else {
            accepted.push(report.split.clone());
        }
    }
    accepted
}

#[cfg(test)]
#[path = "split/tests.rs"]
mod tests;
