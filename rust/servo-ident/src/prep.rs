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
    /// Samples within this distance of a velocity reversal of an active
    /// mode are dropped: breakaway/landing stiction transients and the
    /// coulomb sign-flip loop transient are unmodeled. A reversal is a
    /// sample inside the coulomb deadband OR a sign flip between adjacent
    /// samples - at high accel the velocity steps over the deadband
    /// entirely between samples (12 mm/s per sample at 50 k mm/s2 and
    /// 4 kHz), which used to leave every zigzag reversal unblanked. A mode
    /// idle for a whole segment (an axis-aligned stroke leaves the other
    /// mode at exactly zero) blanks nothing — it is not reversing, and its
    /// coulomb column is zero there anyway.
    pub blank_reversal_s: f64,
    /// Search range for the accel→torque delay. 0 disables alignment.
    pub max_delay_s: f64,
    /// Pulley circumference (mm of belt per revolution). When set, a
    /// sin/cos regressor pair at the pulley angle is added per motor so the
    /// eccentricity ripple — in-band and stroke-locked, hence a mass-bias
    /// if ignored — is absorbed by nuisance coefficients instead.
    pub ripple_period_mm: Option<f64>,
    /// Ringdown-measured resonances shaping the snap column per mode: the
    /// snap regressor is passed through H_z(s) = wz^2/(s^2 + 2 zeta wz s +
    /// wz^2), keeping the exact snap asymptote below the mode, rolling off
    /// above it (where raw snap amplifies junk the plant never responds
    /// to), and carrying the mode's own post-transition ring into the
    /// steady-accel windows the fit keeps - raw snap is zero there while
    /// the physical belt force keeps ringing.
    pub modal: Vec<ModalMode>,
}

/// One ringdown-measured mode: `mode` indexes the frame's mode list.
#[derive(Debug, Clone, Copy)]
pub struct ModalMode {
    pub mode: usize,
    pub freq_hz: f64,
    pub zeta: f64,
}

impl Default for PrepOptions {
    fn default() -> Self {
        Self {
            cutoff_hz: 60.0,
            blank_reversal_s: 0.03,
            max_delay_s: 0.005,
            ripple_period_mm: None,
            modal: Vec::new(),
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
    /// Second time-derivative of the mode acceleration (snap), from a
    /// central second difference of the raw projected accel, then filtered
    /// like every other channel. Raw it is an impulse train at the jerk
    /// breakpoints; the shared zero-phase kernel turns both it and the
    /// torque it explains into the same band-limited pair, which is what
    /// makes the regression consistent.
    pub snap_mode: Vec<Vec<f64>>,
    /// Per-mode coulomb sign column (sign of the RAW mode velocity, then
    /// filtered like every other channel).
    pub cs_mode: Vec<Vec<f64>>,
    /// Measured torque per motor/slot, delay-aligned and filtered.
    pub torque: Vec<Vec<f64>>,
    /// Following error (mm), frame-projected into mode space exactly like
    /// `acc_mode`/`vel_mode` and filtered identically. Not delay-aligned —
    /// following error is a state of the closed loop, not a torque-report
    /// artifact, so there is no accel→torque lag to remove from it.
    pub ferr_mode: Vec<Vec<f64>>,
    /// Per-mode jerk (da/dt) regressor column, filtered identically to the
    /// other channels. Nuisance column for the ferr fit: any timing skew
    /// between the commanded kinematics and the telemetry ferr samples
    /// makes `a(t-δ) ≈ a(t) - δ·j(t)`, so without this column the skew
    /// lands in the mass coefficient as a constant bias and the tuning
    /// loop nulls the correlation at the wrong mass. Its coefficient is
    /// `-m·δ` — dividing by the fitted mass measures the skew itself.
    pub jerk_mode: Vec<Vec<f64>>,
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

/// Bilinear-transform coefficients `[b0, b1, b2, a1, a2]` (a0 normalized
/// out) for the unit-DC-gain resonance
/// H_z(s) = wz^2 / (s^2 + 2 zeta wz s + wz^2), the RBJ low-pass with
/// Q = 1/(2 zeta). Causal on purpose: the physical belt force is causally
/// related to commanded snap, and the zero-phase band filter applied
/// identically to every channel afterwards preserves that relation.
pub fn modal_biquad(freq_hz: f64, zeta: f64, dt: f64) -> [f64; 5] {
    assert!(
        freq_hz > 0.0 && dt > 0.0,
        "modal frequency must be positive"
    );
    assert!(
        freq_hz * dt < 0.45,
        "modal frequency {freq_hz} Hz too close to Nyquist at dt {dt}"
    );
    assert!(zeta > 0.0 && zeta < 1.0, "modal zeta {zeta} outside (0, 1)");
    let w0 = 2.0 * std::f64::consts::PI * freq_hz * dt;
    let (sin_w0, cos_w0) = (libm::sin(w0), libm::cos(w0));
    let alpha = sin_w0 * zeta;
    let a0 = 1.0 + alpha;
    [
        (1.0 - cos_w0) / 2.0 / a0,
        (1.0 - cos_w0) / a0,
        (1.0 - cos_w0) / 2.0 / a0,
        -2.0 * cos_w0 / a0,
        (1.0 - alpha) / a0,
    ]
}

/// Direct-form-II-transposed biquad over one segment, state zeroed at the
/// segment start (segments are separated by dwells far longer than the
/// mode's decay).
pub fn biquad_in_place(chan: &mut [f64], [b0, b1, b2, a1, a2]: [f64; 5]) {
    let (mut z1, mut z2) = (0.0, 0.0);
    for v in chan.iter_mut() {
        let x = *v;
        let y = b0 * x + z1;
        z1 = b1 * x + z2 - a1 * y;
        z2 = b2 * x - a2 * y;
        *v = y;
    }
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
    let mut ferr_mode_raw = vec![vec![0.0; n]; n_modes];
    for md in 0..n_modes {
        for k in 0..n {
            let mut a = 0.0;
            let mut v = 0.0;
            let mut e = 0.0;
            for s in 0..n_motors {
                let f = structure.frame[md][s];
                a += f * cap.acc[s][k];
                v += f * cap.vel[s][k];
                e += f * cap.ferr[s][k];
            }
            acc_mode_raw[md][k] = a;
            vel_mode_raw[md][k] = v;
            cs_raw[md][k] = coulomb_sign(v);
            ferr_mode_raw[md][k] = e;
        }
    }

    let mut snap_raw = vec![vec![0.0; n]; n_modes];
    for md in 0..n_modes {
        for seg in &segs {
            for k in seg.start + 1..seg.end.saturating_sub(1) {
                snap_raw[md][k] = (acc_mode_raw[md][k + 1] - 2.0 * acc_mode_raw[md][k]
                    + acc_mode_raw[md][k - 1])
                    / (dt * dt);
            }
        }
    }
    let mut jerk_raw = vec![vec![0.0; n]; n_modes];
    for md in 0..n_modes {
        for seg in &segs {
            for k in seg.start + 1..seg.end.saturating_sub(1) {
                jerk_raw[md][k] = (acc_mode_raw[md][k + 1] - acc_mode_raw[md][k - 1]) / (2.0 * dt);
            }
        }
    }
    for m in &opts.modal {
        assert!(
            m.mode < n_modes,
            "modal mode index {} out of range ({} modes)",
            m.mode,
            n_modes
        );
        let coef = modal_biquad(m.freq_hz, m.zeta, dt);
        for seg in &segs {
            biquad_in_place(&mut snap_raw[m.mode][seg.clone()], coef);
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
    let snap_mode = filt(&snap_raw);
    let torque = filt(&torque);
    let ferr_mode = filt(&ferr_mode_raw);
    let jerk_mode = filt(&jerk_raw);
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
                let in_deadband = vel_mode_raw[md][k].abs() <= COULOMB_DEADBAND_MM_S;
                let crosses =
                    k + 1 < seg.end && vel_mode_raw[md][k] * vel_mode_raw[md][k + 1] < 0.0;
                if in_deadband || crosses {
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
        snap_mode,
        cs_mode,
        torque,
        ferr_mode,
        jerk_mode,
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

/// The operator's manual tuning heuristic, made a number: the mean of
/// `sign(accel) * ferr` over a short window right after every commanded
/// accel transition, per mode, on RAW unfiltered channels. Only the first
/// excursion when torque is applied carries clean command-path sign
/// information - everything after it is the drive's own compensation
/// (disturbance observer, integrator) reacting, which is why whole-capture
/// ferr regressions can point the wrong way. Positive bias = the plant
/// initially lags the command when torque steps in (feedforward
/// under-feeds, mass too low); negative = it initially overshoots
/// (over-fed). Returns per-mode bias (mm) and the total onset windows
/// scored; zero windows yields a 0.0 bias for that mode.
pub fn onset_bias(
    acc_mode: &[Vec<f64>],
    ferr_mode: &[Vec<f64>],
    dt: f64,
    window_s: f64,
) -> (Vec<f64>, usize) {
    assert!(dt > 0.0, "onset_bias: dt must be positive");
    assert!(window_s > 0.0, "onset_bias: window must be positive");
    assert_eq!(acc_mode.len(), ferr_mode.len(), "mode channel count");
    let window = ((window_s / dt).round() as usize).max(1);
    let mut biases = Vec::with_capacity(acc_mode.len());
    let mut total_windows = 0usize;
    for (acc, ferr) in acc_mode.iter().zip(ferr_mode) {
        assert_eq!(acc.len(), ferr.len(), "mode sample count");
        let windows = accel_step_windows(acc, window);
        total_windows += windows.len();
        let mut sum = 0.0;
        let mut count = 0usize;
        for w in &windows {
            let sign = acc[w.start].signum();
            for &e in &ferr[w.clone()] {
                sum += sign * e;
                count += 1;
            }
        }
        biases.push(if count == 0 { 0.0 } else { sum / count as f64 });
    }
    (biases, total_windows)
}

/// Commanded-accel transition windows for one mode's RAW accel channel,
/// the exact detection `onset_bias` keys on: a window opens at every sample
/// `k` where accel steps by more than `0.25·amax` from its predecessor
/// while `|acc[k]| ≥ 0.1·amax`, and runs (bounded by `window` samples) while
/// accel stays within `0.25·amax` of its value at the trigger. Windows do
/// not overlap — the scan resumes past each window's end. Empty when the
/// channel carries no excitation.
fn accel_step_windows(acc: &[f64], window: usize) -> Vec<std::ops::Range<usize>> {
    let amax = acc.iter().fold(0.0_f64, |m, a| m.max(a.abs()));
    if amax == 0.0 {
        return Vec::new();
    }
    let step_thresh = 0.25 * amax;
    let active_thresh = 0.1 * amax;
    let mut windows = Vec::new();
    let mut k = 1;
    while k < acc.len() {
        let stepped = (acc[k] - acc[k - 1]).abs() > step_thresh;
        if !stepped || acc[k].abs() < active_thresh {
            k += 1;
            continue;
        }
        let mut j = k;
        while j < acc.len().min(k + window) && (acc[j] - acc[k]).abs() <= step_thresh {
            j += 1;
        }
        windows.push(k..j);
        k = j;
    }
    windows
}
/// Accel-activity onset windows for one mode's RAW accel channel: a window
/// opens where commanded `|acc|` rises through `0.1·amax` from below — the
/// start of every accel and decel phase. Unlike `accel_step_windows` (the
/// `onset_bias` detector), this fires on jerk-limited ramps too, which
/// never move by `0.25·amax` between consecutive samples; a mode whose
/// accel is shaped rather than stepped still gets scored. Windows do not
/// overlap — the scan resumes past each window's end. Empty when the
/// channel carries no excitation.
fn accel_onset_windows(acc: &[f64], window: usize) -> Vec<std::ops::Range<usize>> {
    let amax = acc.iter().fold(0.0_f64, |m, a| m.max(a.abs()));
    if amax == 0.0 {
        return Vec::new();
    }
    let active_thresh = 0.1 * amax;
    let mut windows = Vec::new();
    let mut k = 1;
    while k < acc.len() {
        if acc[k - 1].abs() < active_thresh && acc[k].abs() >= active_thresh {
            let end = (k + window).min(acc.len());
            windows.push(k..end);
            k = end;
        } else {
            k += 1;
        }
    }
    windows
}

/// Cruise-arrival windows for one mode: a window opens where commanded
/// `|acc|` falls from `≥ 0.1·amax` to `< 0.1·amax` while the mode is still
/// moving (`|vel| > 0.1·vmax`), i.e. the accel ramp has just handed off to
/// steady cruise. Each window spans `window` samples. Empty without accel
/// or velocity excitation.
fn cruise_arrival_windows(acc: &[f64], vel: &[f64], window: usize) -> Vec<std::ops::Range<usize>> {
    let amax = acc.iter().fold(0.0_f64, |m, a| m.max(a.abs()));
    let vmax = vel.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
    if amax == 0.0 || vmax == 0.0 {
        return Vec::new();
    }
    let active_a = 0.1 * amax;
    let cruise_v = 0.1 * vmax;
    let mut windows = Vec::new();
    for k in 1..acc.len() {
        if acc[k - 1].abs() >= active_a && acc[k].abs() < active_a && vel[k].abs() > cruise_v {
            windows.push(k..(k + window).min(acc.len()));
        }
    }
    windows
}

/// Velocity sign-reversal windows for one mode: a window opens at the first
/// sample of a new nonzero commanded-velocity sign that differs from the
/// previous nonzero sign — the reversal itself, whether it lands on a zero
/// sample or steps clean across zero. Each window spans `window` samples.
fn reversal_windows(vel: &[f64], window: usize) -> Vec<std::ops::Range<usize>> {
    let sgn = |v: f64| {
        if v > 0.0 {
            1i8
        } else if v < 0.0 {
            -1i8
        } else {
            0i8
        }
    };
    let mut windows = Vec::new();
    let mut last = 0i8;
    for k in 0..vel.len() {
        let s = sgn(vel[k]);
        if s == 0 {
            continue;
        }
        if last != 0 && s != last {
            windows.push(k..(k + window).min(vel.len()));
        }
        last = s;
    }
    windows
}

/// Which commanded transient the transient-rms objective scores over.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransientKind {
    /// Onsets of commanded-accel activity — every accel/decel phase start
    /// (feedforward mass authority).
    Mass,
    /// Cruise arrivals — accel ramp handing off to steady velocity
    /// (viscous authority).
    Viscous,
    /// Commanded-velocity sign reversals (coulomb authority).
    Coulomb,
}

/// Per-mode transient following-error statistics.
#[derive(Debug, Clone, PartialEq)]
pub struct TransientRms {
    /// Sample-weighted RMS of ferr over all of the mode's window samples
    /// (mm); `None` when no window fired.
    pub rms: Option<f64>,
    /// Standard error of the per-window RMS values,
    /// `std(per-window rms)/sqrt(windows)` (mm); `None` when fewer than two
    /// windows fired (the sample standard deviation is undefined).
    pub sigma: Option<f64>,
    /// Windows that contributed samples.
    pub windows: usize,
}

/// Transient-scoped RMS of the RAW following error, per mode. Whole-capture
/// ferr rms dilutes the accel transients feedforward actually controls ~10x
/// with the inner servo loop's post-transient corrections; this scores only
/// the short windows right after each commanded transition, where the
/// command path — not the disturbance observer or integrator — still owns
/// the error. `kind` selects the trigger; the windowing and aggregation are
/// shared. `acc_mode`, `vel_mode` and `ferr_mode` are RAW mode-space
/// channels; `window_s` is the post-trigger window (samples at `dt`).
pub fn transient_rms(
    kind: TransientKind,
    acc_mode: &[Vec<f64>],
    vel_mode: &[Vec<f64>],
    ferr_mode: &[Vec<f64>],
    dt: f64,
    window_s: f64,
) -> Vec<TransientRms> {
    assert!(dt > 0.0, "transient_rms: dt must be positive");
    assert!(window_s > 0.0, "transient_rms: window must be positive");
    assert_eq!(acc_mode.len(), ferr_mode.len(), "mode channel count");
    assert_eq!(vel_mode.len(), ferr_mode.len(), "mode channel count");
    let window = ((window_s / dt).round() as usize).max(1);
    let mut out = Vec::with_capacity(ferr_mode.len());
    for ((acc, vel), ferr) in acc_mode.iter().zip(vel_mode).zip(ferr_mode) {
        assert_eq!(acc.len(), ferr.len(), "mode sample count");
        assert_eq!(vel.len(), ferr.len(), "mode sample count");
        let windows = match kind {
            TransientKind::Mass => accel_onset_windows(acc, window),
            TransientKind::Viscous => cruise_arrival_windows(acc, vel, window),
            TransientKind::Coulomb => reversal_windows(vel, window),
        };
        out.push(aggregate_transient(ferr, &windows));
    }
    out
}

/// Sample-weighted RMS plus its standard error over a mode's transient
/// windows: per-window RMS `sqrt(mean ferr²)`, aggregate RMS
/// `sqrt(Σ ferr² / Σ samples)` across every window sample, and
/// `sigma = std(per-window rms)/sqrt(windows)` (sample std, so `None`
/// below two windows).
fn aggregate_transient(ferr: &[f64], windows: &[std::ops::Range<usize>]) -> TransientRms {
    let mut total_sq = 0.0;
    let mut total_n = 0usize;
    let mut per_window = Vec::with_capacity(windows.len());
    for w in windows {
        let slice = &ferr[w.clone()];
        if slice.is_empty() {
            continue;
        }
        let sq: f64 = slice.iter().map(|v| v * v).sum();
        total_sq += sq;
        total_n += slice.len();
        per_window.push((sq / slice.len() as f64).sqrt());
    }
    let rms = (total_n > 0).then(|| (total_sq / total_n as f64).sqrt());
    let sigma = if per_window.len() < 2 {
        None
    } else {
        let mean = per_window.iter().sum::<f64>() / per_window.len() as f64;
        let var = per_window.iter().map(|x| (x - mean).powi(2)).sum::<f64>()
            / (per_window.len() - 1) as f64;
        Some(var.sqrt() / (per_window.len() as f64).sqrt())
    };
    TransientRms {
        rms,
        sigma,
        windows: per_window.len(),
    }
}
