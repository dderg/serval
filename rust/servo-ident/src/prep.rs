//! Delay-aligned, reversal-blanked preparation of a capture for fitting,
//! plus band-limited residual reporting.
//!
//! The raw regression `tau[k] = m·a[k] + b·v[k] + coulomb(sign v[k])` is
//! polluted by everything the model cannot express: pulley/mesh torque
//! ripple, the feedback loop's transient corrections after each jerk event
//! (correlated with the accel regressor), and the loop delay between
//! commanded accel and realized torque. The delay is measured by
//! cross-correlating first differences and removed by re-aligning the
//! torque channel; samples around velocity reversals are blanked because
//! breakaway stiction does not follow the static coulomb model; the loop
//! transients that survive the steady-accel mask are rejected by the fit's
//! Huber reweighting, which sees them as the outliers they are.
//!
//! Band-limiting the fit itself (filtering both sides of the equation) was
//! tried and measured WORSE on synthetic truth: smearing a transient into
//! the passband converts an outlier the robust loss rejects outright into a
//! small correlated bias nothing can reject. The low-pass machinery here is
//! therefore used for REPORTING only: the in-band residual states how well
//! the model explains the torque in the band where a feedforward model has
//! authority, without ripple and quantization noise drowning the number.

use crate::capture::Capture;
use crate::model::{coulomb_sign, Structure, COULOMB_DEADBAND_MM_S};

#[derive(Debug, Clone)]
pub struct PrepOptions {
    /// Zero-phase low-pass cutoff applied to both sides of the regression
    /// (Hz), and used for the in-band residual report. 0 disables filtering.
    pub cutoff_hz: f64,
    /// Samples within this distance of a velocity-deadband sample of an
    /// active mode are dropped: breakaway/landing stiction transients are
    /// unmodeled. A mode idle for a whole segment (an axis-aligned stroke
    /// leaves the other mode at exactly zero) blanks nothing — it is not
    /// reversing, and its coulomb column is zero there anyway.
    pub blank_reversal_s: f64,
    /// Search range for the accel→torque delay. 0 disables alignment.
    pub max_delay_s: f64,
    /// Pulley circumference (mm of belt per revolution). When set, a
    /// sin/cos regressor pair at the pulley angle is added per motor so the
    /// eccentricity ripple — in-band and stroke-locked, hence a mass-bias
    /// if ignored — is absorbed by nuisance coefficients instead.
    pub ripple_period_mm: Option<f64>,
}

impl Default for PrepOptions {
    fn default() -> Self {
        Self {
            cutoff_hz: 60.0,
            blank_reversal_s: 0.03,
            max_delay_s: 0.005,
            ripple_period_mm: None,
        }
    }
}

/// Delay-aligned channels plus a validity mask, all sample-aligned with the
/// input capture.
#[derive(Debug)]
pub struct Prepped {
    pub t: Vec<f64>,
    /// Mode-space channels: `[mode][sample]`, frame-projected then filtered.
    pub acc_mode: Vec<Vec<f64>>,
    pub vel_mode: Vec<Vec<f64>>,
    /// Per-mode coulomb sign column (sign of the RAW mode velocity, then
    /// filtered like every other channel).
    pub cs_mode: Vec<Vec<f64>>,
    /// Measured torque per motor/slot, delay-aligned and filtered.
    pub torque: Vec<Vec<f64>>,
    /// Pulley-angle sin/cos nuisance columns per motor (empty when
    /// `ripple_period_mm` is unset), filtered like every other channel.
    pub extra: Vec<Vec<Vec<f64>>>,
    /// False where the sample must not enter the fit (filter warmup at
    /// segment edges, reversal neighborhoods, delay-shift tails).
    pub valid: Vec<bool>,
    /// Estimated accel→torque delay actually removed (s).
    pub delay_s: f64,
    pub segments: usize,
}

pub fn median_dt(t: &[f64]) -> f64 {
    let mut dts: Vec<f64> = (1..t.len()).map(|k| t[k] - t[k - 1]).collect();
    assert!(!dts.is_empty(), "capture too short for a sample interval");
    dts.sort_by(|a, b| a.partial_cmp(b).expect("non-finite dt"));
    dts[dts.len() / 2]
}

/// Contiguous runs of samples: the ident CSV holds only motion-active
/// cycles, so time jumps between strokes split the capture into segments
/// that must be filtered independently.
pub fn segments(t: &[f64], dt: f64) -> Vec<std::ops::Range<usize>> {
    let mut out = Vec::new();
    let mut start = 0;
    for k in 1..t.len() {
        if t[k] - t[k - 1] > 1.5 * dt {
            out.push(start..k);
            start = k;
        }
    }
    out.push(start..t.len());
    out
}

/// Odd-length Hann-windowed-sinc low-pass with unit DC gain. Symmetric, so
/// same-mode convolution is zero-phase.
pub fn sinc_kernel(cutoff_hz: f64, dt: f64) -> Vec<f64> {
    assert!(cutoff_hz > 0.0 && dt > 0.0);
    let fc = cutoff_hz * dt;
    assert!(fc < 0.5, "cutoff above Nyquist");
    let half = ((1.55 / fc).ceil() as usize).max(2);
    let n = 2 * half + 1;
    let mut k: Vec<f64> = (0..n)
        .map(|i| {
            let x = i as f64 - half as f64;
            let sinc = if x == 0.0 {
                2.0 * fc
            } else {
                libm::sin(2.0 * std::f64::consts::PI * fc * x) / (std::f64::consts::PI * x)
            };
            let w = 0.5 + 0.5 * libm::cos(2.0 * std::f64::consts::PI * i as f64 / (n - 1) as f64);
            let w = 1.0 - w;
            sinc * w
        })
        .collect();
    let sum: f64 = k.iter().sum();
    for v in &mut k {
        *v /= sum;
    }
    k
}

/// Same-mode convolution with reflected edge padding: near segment edges the
/// missing neighbors are mirrored, which approximates the signal well for
/// the ramp-from-rest strokes the ident grid produces and keeps the filter
/// shift-invariant enough that only a quarter-kernel warmup needs blanking.
fn convolve_same(x: &[f64], kernel: &[f64]) -> Vec<f64> {
    let half = kernel.len() / 2;
    let n = x.len();
    let reflect = |idx: isize| -> f64 {
        let m = if idx < 0 {
            (-idx) as usize
        } else if idx as usize >= n {
            2 * (n - 1) - idx as usize
        } else {
            idx as usize
        };
        x[m.min(n - 1)]
    };
    let mut out = vec![0.0; n];
    for (i, o) in out.iter_mut().enumerate() {
        let mut acc = 0.0;
        for (j, &kv) in kernel.iter().enumerate() {
            acc += kv * reflect(i as isize + j as isize - half as isize);
        }
        *o = acc;
    }
    out
}

pub(crate) fn filter_segments(
    x: &[f64],
    segs: &[std::ops::Range<usize>],
    kernel: Option<&[f64]>,
) -> Vec<f64> {
    match kernel {
        None => x.to_vec(),
        Some(k) => {
            let mut out = vec![0.0; x.len()];
            for seg in segs {
                let filtered = convolve_same(&x[seg.clone()], k);
                out[seg.clone()].copy_from_slice(&filtered);
            }
            out
        }
    }
}

/// Correlates FIRST DIFFERENCES of accel and torque: the jerk impulses
/// against the torque steps they cause. Raw-level correlation is useless
/// here — the coulomb+viscous trend makes the score grow monotonically with
/// lag — while differencing leaves sharp impulses whose alignment peaks at
/// the true delay.
fn estimate_delay_samples(
    acc: &[Vec<f64>],
    torque: &[Vec<f64>],
    segs: &[std::ops::Range<usize>],
    max_lag: usize,
) -> usize {
    let mut best_lag = 0;
    let mut best_score = f64::NEG_INFINITY;
    for lag in 0..=max_lag {
        let mut score = 0.0;
        for seg in segs {
            if seg.len() <= lag + 1 {
                continue;
            }
            for m in 0..acc.len() {
                for k in seg.start + 1..seg.end - lag {
                    let da = acc[m][k] - acc[m][k - 1];
                    let dtq = torque[m][k + lag] - torque[m][k + lag - 1];
                    score += da * dtq;
                }
            }
        }
        if score > best_score {
            best_score = score;
            best_lag = lag;
        }
    }
    best_lag
}

pub fn prep(cap: &Capture, structure: &Structure, opts: &PrepOptions) -> Prepped {
    let n = cap.t.len();
    let n_motors = cap.acc.len();
    assert_eq!(
        structure.axis_count(),
        n_motors,
        "frame slot count vs capture motor count"
    );
    let n_modes = structure.mode_count();
    let dt = median_dt(&cap.t);
    let segs = segments(&cap.t, dt);

    let mut acc_mode_raw = vec![vec![0.0; n]; n_modes];
    let mut vel_mode_raw = vec![vec![0.0; n]; n_modes];
    let mut cs_raw = vec![vec![0.0; n]; n_modes];
    for md in 0..n_modes {
        for k in 0..n {
            let mut a = 0.0;
            let mut v = 0.0;
            for s in 0..n_motors {
                let f = structure.frame[md][s];
                a += f * cap.acc[s][k];
                v += f * cap.vel[s][k];
            }
            acc_mode_raw[md][k] = a;
            vel_mode_raw[md][k] = v;
            cs_raw[md][k] = coulomb_sign(v);
        }
    }

    let max_lag = (opts.max_delay_s / dt).round() as usize;
    let lag = if max_lag > 0 {
        estimate_delay_samples(&cap.acc, &cap.torque, &segs, max_lag)
    } else {
        0
    };
    let mut torque = cap.torque.clone();
    let mut valid = vec![true; n];
    if lag > 0 {
        for tq in &mut torque {
            for seg in &segs {
                for k in seg.start..seg.end - lag {
                    tq[k] = tq[k + lag];
                }
                for v in valid.iter_mut().take(seg.end).skip(seg.end - lag) {
                    *v = false;
                }
            }
        }
    }

    let kernel = if opts.cutoff_hz > 0.0 {
        Some(sinc_kernel(opts.cutoff_hz, dt))
    } else {
        None
    };
    let warmup = kernel.as_ref().map_or(0, |k| k.len() / 4);
    let kref = kernel.as_deref();
    let filt = |chans: &[Vec<f64>]| -> Vec<Vec<f64>> {
        chans
            .iter()
            .map(|c| filter_segments(c, &segs, kref))
            .collect()
    };
    let acc_mode = filt(&acc_mode_raw);
    let vel_mode = filt(&vel_mode_raw);
    let cs_mode = filt(&cs_raw);
    let torque = filt(&torque);
    let extra: Vec<Vec<Vec<f64>>> = match opts.ripple_period_mm {
        None => Vec::new(),
        Some(period) => {
            assert!(period > 0.0, "ripple period must be positive");
            (0..n_motors)
                .map(|m| {
                    let mut pos = 0.0;
                    let mut sin_col = Vec::with_capacity(n);
                    let mut cos_col = Vec::with_capacity(n);
                    for k in 0..n {
                        pos += cap.vel[m][k] * dt;
                        let phase = 2.0 * std::f64::consts::PI * pos / period;
                        sin_col.push(libm::sin(phase));
                        cos_col.push(libm::cos(phase));
                    }
                    vec![
                        filter_segments(&sin_col, &segs, kref),
                        filter_segments(&cos_col, &segs, kref),
                    ]
                })
                .collect()
        }
    };

    for seg in &segs {
        let w = warmup.min(seg.len());
        for v in valid.iter_mut().take(seg.start + w).skip(seg.start) {
            *v = false;
        }
        for v in valid.iter_mut().take(seg.end).skip(seg.end - w) {
            *v = false;
        }
    }

    let blank = (opts.blank_reversal_s / dt).round() as usize;
    for md in 0..n_modes {
        for seg in segs.iter() {
            let mode_moves = seg
                .clone()
                .any(|k| vel_mode_raw[md][k].abs() > COULOMB_DEADBAND_MM_S);
            if !mode_moves {
                continue;
            }
            for k in seg.clone() {
                if vel_mode_raw[md][k].abs() <= COULOMB_DEADBAND_MM_S {
                    let lo = k.saturating_sub(blank).max(seg.start);
                    let hi = (k + blank + 1).min(seg.end);
                    for v in valid.iter_mut().take(hi).skip(lo) {
                        *v = false;
                    }
                }
            }
        }
    }

    Prepped {
        t: cap.t.clone(),
        acc_mode,
        vel_mode,
        cs_mode,
        torque,
        extra,
        valid,
        delay_s: lag as f64 * dt,
        segments: segs.len(),
    }
}

/// RMS of the residual series after zero-phase low-passing at `cutoff_hz`,
/// taken over the `keep` rows only. This is the model error in the band
/// where a feedforward model has authority; out-of-band ripple, loop
/// transients and torque quantization are excluded from the number instead
/// of dominating it.
pub fn band_limited_rms(residual: &[Vec<f64>], t: &[f64], keep: &[bool], cutoff_hz: f64) -> f64 {
    assert!(cutoff_hz > 0.0, "band_limited_rms needs a positive cutoff");
    let dt = median_dt(t);
    let segs = segments(t, dt);
    let kernel = sinc_kernel(cutoff_hz, dt);
    let mut sq = 0.0;
    let mut count = 0usize;
    for res in residual {
        let filtered = filter_segments(res, &segs, Some(&kernel));
        for (k, &v) in filtered.iter().enumerate() {
            if keep[k] {
                sq += v * v;
                count += 1;
            }
        }
    }
    assert!(count > 0, "band_limited_rms: no kept samples");
    (sq / count as f64).sqrt()
}
