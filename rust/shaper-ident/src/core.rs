//! Numeric core of input-shaper resonance identification, ported from
//! klippy/extras/shaper_calibrate.py. No Python here — see lib.rs for the
//! PyO3 surface.

use std::f64::consts::PI;

use realfft::RealFftPlanner;

pub const MIN_FREQ: f64 = 5.0;
pub const MAX_FREQ: f64 = 1000.0;
pub const WINDOW_T_SEC: f64 = 0.5;
pub const MAX_SHAPER_FREQ: f64 = 1000.0;

pub const SHAPER_VIBRATION_REDUCTION: f64 = 20.0;
pub const DEFAULT_DAMPING_RATIO: f64 = 0.1;

pub const TEST_DAMPING_RATIOS: [f64; 3] = [0.075, 0.1, 0.15];

/// Impulse shaper: amplitudes `a` and their times `t` (seconds).
pub struct Shaper {
    pub a: Vec<f64>,
    pub t: Vec<f64>,
}

/// A candidate input shaper family: its name, the min test frequency, and the
/// generator producing the `(A, T)` impulse sequence at a frequency + damping.
pub struct ShaperCfg {
    pub name: &'static str,
    pub min_freq: f64,
    pub init: fn(f64, f64) -> Shaper,
}

pub const INPUT_SHAPERS: [ShaperCfg; 6] = [
    ShaperCfg {
        name: "zv",
        min_freq: 21.0,
        init: get_zv_shaper,
    },
    ShaperCfg {
        name: "mzv",
        min_freq: 23.0,
        init: get_mzv_shaper,
    },
    ShaperCfg {
        name: "zvd",
        min_freq: 29.0,
        init: get_zvd_shaper,
    },
    ShaperCfg {
        name: "ei",
        min_freq: 29.0,
        init: get_ei_shaper,
    },
    ShaperCfg {
        name: "2hump_ei",
        min_freq: 39.0,
        init: get_2hump_ei_shaper,
    },
    ShaperCfg {
        name: "3hump_ei",
        min_freq: 48.0,
        init: get_3hump_ei_shaper,
    },
];

pub fn find_shaper_cfg(name: &str) -> Option<&'static ShaperCfg> {
    INPUT_SHAPERS.iter().find(|c| c.name == name)
}

fn get_zv_shaper(shaper_freq: f64, damping_ratio: f64) -> Shaper {
    let df = (1.0 - damping_ratio * damping_ratio).sqrt();
    let k = libm::exp(-damping_ratio * PI / df);
    let t_d = 1.0 / (shaper_freq * df);
    Shaper {
        a: vec![1.0, k],
        t: vec![0.0, 0.5 * t_d],
    }
}

fn get_zvd_shaper(shaper_freq: f64, damping_ratio: f64) -> Shaper {
    let df = (1.0 - damping_ratio * damping_ratio).sqrt();
    let k = libm::exp(-damping_ratio * PI / df);
    let t_d = 1.0 / (shaper_freq * df);
    Shaper {
        a: vec![1.0, 2.0 * k, k * k],
        t: vec![0.0, 0.5 * t_d, t_d],
    }
}

fn get_mzv_shaper(shaper_freq: f64, damping_ratio: f64) -> Shaper {
    let df = (1.0 - damping_ratio * damping_ratio).sqrt();
    let k = libm::exp(-0.75 * damping_ratio * PI / df);
    let t_d = 1.0 / (shaper_freq * df);
    let a1 = 1.0 - 1.0 / 2.0_f64.sqrt();
    let a2 = (2.0_f64.sqrt() - 1.0) * k;
    let a3 = a1 * k * k;
    Shaper {
        a: vec![a1, a2, a3],
        t: vec![0.0, 0.375 * t_d, 0.75 * t_d],
    }
}

fn get_ei_shaper(shaper_freq: f64, damping_ratio: f64) -> Shaper {
    let v_tol = 1.0 / SHAPER_VIBRATION_REDUCTION;
    let df = (1.0 - damping_ratio * damping_ratio).sqrt();
    let k = libm::exp(-damping_ratio * PI / df);
    let t_d = 1.0 / (shaper_freq * df);
    let a1 = 0.25 * (1.0 + v_tol);
    let a2 = 0.5 * (1.0 - v_tol) * k;
    let a3 = a1 * k * k;
    Shaper {
        a: vec![a1, a2, a3],
        t: vec![0.0, 0.5 * t_d, t_d],
    }
}

fn get_2hump_ei_shaper(shaper_freq: f64, damping_ratio: f64) -> Shaper {
    let v_tol = 1.0 / SHAPER_VIBRATION_REDUCTION;
    let df = (1.0 - damping_ratio * damping_ratio).sqrt();
    let k = libm::exp(-damping_ratio * PI / df);
    let t_d = 1.0 / (shaper_freq * df);
    let v2 = v_tol * v_tol;
    let x = libm::pow(v2 * ((1.0 - v2).sqrt() + 1.0), 1.0 / 3.0);
    let a1 = (3.0 * x * x + 2.0 * x + 3.0 * v2) / (16.0 * x);
    let a2 = (0.5 - a1) * k;
    let a3 = a2 * k;
    let a4 = a1 * k * k * k;
    Shaper {
        a: vec![a1, a2, a3, a4],
        t: vec![0.0, 0.5 * t_d, t_d, 1.5 * t_d],
    }
}

fn get_3hump_ei_shaper(shaper_freq: f64, damping_ratio: f64) -> Shaper {
    let v_tol = 1.0 / SHAPER_VIBRATION_REDUCTION;
    let df = (1.0 - damping_ratio * damping_ratio).sqrt();
    let k = libm::exp(-damping_ratio * PI / df);
    let t_d = 1.0 / (shaper_freq * df);
    let k2 = k * k;
    let a1 = 0.0625 * (1.0 + 3.0 * v_tol + 2.0 * (2.0 * (v_tol + 1.0) * v_tol).sqrt());
    let a2 = 0.25 * (1.0 - v_tol) * k;
    let a3 = (0.5 * (1.0 + v_tol) - 2.0 * a1) * k2;
    let a4 = a2 * k2;
    let a5 = a1 * k2 * k2;
    Shaper {
        a: vec![a1, a2, a3, a4, a5],
        t: vec![0.0, 0.5 * t_d, t_d, 1.5 * t_d, 2.0 * t_d],
    }
}

/// Modified Bessel function of the first kind, order 0. Ascending series;
/// converges quickly for the Kaiser beta (6.0) we use.
fn bessel_i0(x: f64) -> f64 {
    let mut sum = 1.0;
    let mut term = 1.0;
    let half_sq = (x * 0.5) * (x * 0.5);
    let mut k = 1.0_f64;
    loop {
        term *= half_sq / (k * k);
        sum += term;
        if term <= sum * 1e-16 {
            break;
        }
        k += 1.0;
    }
    sum
}

/// numpy.kaiser(m, beta).
fn kaiser(m: usize, beta: f64) -> Vec<f64> {
    if m == 1 {
        return vec![1.0];
    }
    let alpha = (m - 1) as f64 / 2.0;
    let denom = bessel_i0(beta);
    (0..m)
        .map(|n| {
            let r = (n as f64 - alpha) / alpha;
            bessel_i0(beta * (1.0 - r * r).sqrt()) / denom
        })
        .collect()
}

/// Welch-method PSD of a single-axis signal, matching Python's `_psd`.
/// Returns `(freqs, psd)`.
pub fn psd(x: &[f64], fs: f64, nfft: usize) -> (Vec<f64>, Vec<f64>) {
    let window = kaiser(nfft, 6.0);
    let scale = 1.0 / window.iter().map(|w| w * w).sum::<f64>();

    let overlap = nfft / 2;
    let step = nfft - overlap;
    let n_windows = (x.len() - overlap) / step;

    let mut planner = RealFftPlanner::<f64>::new();
    let r2c = planner.plan_fft_forward(nfft);
    let n_bins = nfft / 2 + 1;
    let mut psd_acc = vec![0.0_f64; n_bins];
    let mut scratch = r2c.make_input_vec();
    let mut spectrum = r2c.make_output_vec();

    for w in 0..n_windows {
        let base = w * step;
        let seg = &x[base..base + nfft];
        let mean = seg.iter().sum::<f64>() / nfft as f64;
        for i in 0..nfft {
            scratch[i] = window[i] * (seg[i] - mean);
        }
        r2c.process(&mut scratch, &mut spectrum).unwrap();
        for (k, c) in spectrum.iter().enumerate() {
            let mut p = c.norm_sqr() * scale / fs;
            if k != 0 && k != n_bins - 1 {
                p *= 2.0;
            }
            psd_acc[k] += p;
        }
    }
    let inv = 1.0 / n_windows as f64;
    for v in &mut psd_acc {
        *v *= inv;
    }
    let freqs = (0..n_bins).map(|k| k as f64 * fs / nfft as f64).collect();
    (freqs, psd_acc)
}

pub struct FreqResponse {
    pub freq_bins: Vec<f64>,
    pub psd_sum: Vec<f64>,
    pub psd_x: Vec<f64>,
    pub psd_y: Vec<f64>,
    pub psd_z: Vec<f64>,
}

/// Port of `calc_freq_response`. `samples` rows are `[t, x, y, z]`.
pub fn calc_freq_response(samples: &[[f64; 4]]) -> Option<FreqResponse> {
    let n = samples.len();
    if n == 0 {
        return None;
    }
    let t_span = samples[n - 1][0] - samples[0][0];
    let sampling_freq = n as f64 / t_span;
    let bl = int_bit_length((sampling_freq * WINDOW_T_SEC - 1.0) as i64);
    let m = 1usize << bl;
    if n <= m {
        return None;
    }
    let col = |c: usize| samples.iter().map(|r| r[c]).collect::<Vec<f64>>();
    let (fx, px) = psd(&col(1), sampling_freq, m);
    let (_fy, py) = psd(&col(2), sampling_freq, m);
    let (_fz, pz) = psd(&col(3), sampling_freq, m);
    let psd_sum = px
        .iter()
        .zip(&py)
        .zip(&pz)
        .map(|((a, b), c)| a + b + c)
        .collect();
    Some(FreqResponse {
        freq_bins: fx,
        psd_sum,
        psd_x: px,
        psd_y: py,
        psd_z: pz,
    })
}

/// Python `int(v).bit_length()`: v truncated toward zero, then bit length of
/// its magnitude.
fn int_bit_length(v: i64) -> u32 {
    let m = v.unsigned_abs();
    if m == 0 { 0 } else { 64 - m.leading_zeros() }
}

/// Port of `_estimate_shaper`: shaper response magnitude at each frequency.
pub fn estimate_shaper(shaper: &Shaper, damping_ratio: f64, freqs: &[f64]) -> Vec<f64> {
    let inv_d = 1.0 / shaper.a.iter().sum::<f64>();
    let t_last = *shaper.t.last().unwrap();
    let sq = (1.0 - damping_ratio * damping_ratio).sqrt();
    freqs
        .iter()
        .map(|&f| {
            let omega = 2.0 * PI * f;
            let damping = damping_ratio * omega;
            let omega_d = omega * sq;
            let mut s = 0.0;
            let mut c = 0.0;
            for i in 0..shaper.a.len() {
                let w = shaper.a[i] * libm::exp(-damping * (t_last - shaper.t[i]));
                s += w * libm::sin(omega_d * shaper.t[i]);
                c += w * libm::cos(omega_d * shaper.t[i]);
            }
            (s * s + c * c).sqrt() * inv_d
        })
        .collect()
}

/// Port of `_estimate_remaining_vibrations`. Returns `(ratio, vals)`.
pub fn estimate_remaining_vibrations(
    shaper: &Shaper,
    damping_ratio: f64,
    freq_bins: &[f64],
    psd: &[f64],
) -> (f64, Vec<f64>) {
    let vals = estimate_shaper(shaper, damping_ratio, freq_bins);
    (remaining_vibrations(&vals, psd), vals)
}

/// The score core shared by shapers and smoothers: how much of the measured
/// vibration energy survives the response `vals`, above the tolerance floor.
pub fn remaining_vibrations(vals: &[f64], psd: &[f64]) -> f64 {
    let psd_max = psd.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let vibr_threshold = psd_max / SHAPER_VIBRATION_REDUCTION;
    let mut remaining = 0.0;
    let mut all = 0.0;
    for i in 0..psd.len() {
        remaining += (vals[i] * psd[i] - vibr_threshold).max(0.0);
        all += (psd[i] - vibr_threshold).max(0.0);
    }
    remaining / all
}

/// Port of `_get_shaper_smoothing`.
pub fn get_shaper_smoothing(shaper: &Shaper, accel: f64, scv: f64) -> f64 {
    let half_accel = accel * 0.5;
    let inv_d = 1.0 / shaper.a.iter().sum::<f64>();
    let n = shaper.t.len();
    let ts = (0..n).map(|i| shaper.a[i] * shaper.t[i]).sum::<f64>() * inv_d;
    let mut offset_90 = 0.0;
    let mut offset_180 = 0.0;
    for i in 0..n {
        let dt = shaper.t[i] - ts;
        if shaper.t[i] >= ts {
            offset_90 += shaper.a[i] * (scv + half_accel * dt) * dt;
        }
        offset_180 += shaper.a[i] * half_accel * dt * dt;
    }
    offset_90 *= inv_d * 2.0_f64.sqrt();
    offset_180 *= inv_d;
    offset_90.max(offset_180)
}

fn bisect(mut func: impl FnMut(f64) -> bool) -> f64 {
    let mut left = 1.0;
    let mut right = 1.0;
    if !func(1e-9) {
        return 0.0;
    }
    while !func(left) {
        right = left;
        left *= 0.5;
    }
    if right == left {
        while func(right) {
            right *= 2.0;
        }
    }
    while right - left > 1e-8 {
        let middle = (left + right) * 0.5;
        if func(middle) {
            left = middle;
        } else {
            right = middle;
        }
    }
    left
}

/// Port of `find_shaper_max_accel`.
pub fn find_shaper_max_accel(shaper: &Shaper, scv: f64) -> f64 {
    const TARGET_SMOOTHING: f64 = 0.12;
    bisect(|test_accel| get_shaper_smoothing(shaper, test_accel, scv) <= TARGET_SMOOTHING)
}

pub struct FitResult {
    pub name: String,
    pub freq: f64,
    pub vals: Vec<f64>,
    pub vibrs: f64,
    pub smoothing: f64,
    pub score: f64,
    pub max_accel: f64,
}

/// How the caller specified the frequencies to sweep.
pub enum ShaperFreqs {
    /// `(start, end, step)`, each optional (defaults resolved per Python).
    Range(Option<f64>, Option<f64>, Option<f64>),
    /// An explicit list of frequencies.
    List(Vec<f64>),
}

/// Port of `fit_shaper` for a single shaper family. Returns `None` only when
/// no frequency was tested.
#[allow(clippy::too_many_arguments)]
pub fn fit_shaper(
    cfg: &ShaperCfg,
    freq_bins_in: &[f64],
    psd_in: &[f64],
    shaper_freqs: &ShaperFreqs,
    damping_ratio: Option<f64>,
    scv: f64,
    max_smoothing: Option<f64>,
    test_damping_ratios: Option<Vec<f64>>,
    max_freq: Option<f64>,
) -> Option<FitResult> {
    let damping_ratio = damping_ratio.unwrap_or(DEFAULT_DAMPING_RATIO);
    let test_damping_ratios = test_damping_ratios.unwrap_or_else(|| TEST_DAMPING_RATIOS.to_vec());

    let test_freqs: Vec<f64> = match shaper_freqs {
        ShaperFreqs::List(v) => v.clone(),
        ShaperFreqs::Range(start, end, step) => {
            let freq_end = end.unwrap_or(MAX_SHAPER_FREQ);
            let freq_start = start.unwrap_or(cfg.min_freq).min(freq_end - 1e-7);
            let freq_step = step.unwrap_or(0.2);
            arange(freq_start, freq_end, freq_step)
        }
    };
    if test_freqs.is_empty() {
        return None;
    }
    let test_max = test_freqs.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let base = max_freq.filter(|&m| m != 0.0).unwrap_or(MAX_FREQ);
    let max_freq = base.max(test_max);

    let mut freq_bins = Vec::new();
    let mut psd = Vec::new();
    for i in 0..freq_bins_in.len() {
        if freq_bins_in[i] <= max_freq {
            freq_bins.push(freq_bins_in[i]);
            psd.push(psd_in[i]);
        }
    }

    let mut best: Option<FitResult> = None;
    let mut results: Vec<FitResult> = Vec::new();

    for &test_freq in test_freqs.iter().rev() {
        let shaper = (cfg.init)(test_freq, damping_ratio);
        let shaper_smoothing = get_shaper_smoothing(&shaper, 5000.0, scv);
        if let (Some(ms), Some(_)) = (max_smoothing, best.as_ref()) {
            if shaper_smoothing > ms {
                return best;
            }
        }
        let mut shaper_vibrations = 0.0;
        let mut shaper_vals = vec![0.0_f64; freq_bins.len()];
        for &dr in &test_damping_ratios {
            let (vibrations, vals) = estimate_remaining_vibrations(&shaper, dr, &freq_bins, &psd);
            for i in 0..shaper_vals.len() {
                shaper_vals[i] = shaper_vals[i].max(vals[i]);
            }
            if vibrations > shaper_vibrations {
                shaper_vibrations = vibrations;
            }
        }
        let max_accel = find_shaper_max_accel(&shaper, scv);
        let shaper_score =
            shaper_smoothing * (libm::pow(shaper_vibrations, 1.5) + shaper_vibrations * 0.2 + 0.01);
        results.push(FitResult {
            name: cfg.name.to_string(),
            freq: test_freq,
            vals: shaper_vals,
            vibrs: shaper_vibrations,
            smoothing: shaper_smoothing,
            score: shaper_score,
            max_accel,
        });
        let last = results.last().unwrap();
        if best.as_ref().map_or(true, |b| b.vibrs > last.vibrs) {
            best = Some(clone_result(last));
        }
    }

    let best = best?;
    let mut selected = clone_result(&best);
    for res in results.iter().rev() {
        if res.vibrs < best.vibrs * 1.1 && res.score < selected.score {
            selected = clone_result(res);
        }
    }
    Some(selected)
}

fn clone_result(r: &FitResult) -> FitResult {
    FitResult {
        name: r.name.clone(),
        freq: r.freq,
        vals: r.vals.clone(),
        vibrs: r.vibrs,
        smoothing: r.smoothing,
        score: r.score,
        max_accel: r.max_accel,
    }
}

/// A candidate input-smoother family: fitted against the exact convolution
/// kernel the runtime executes (`trajectory::build_smooth_*_kernel`), so the
/// recommended frequency always refers to a kernel the engine can run.
pub struct SmootherCfg {
    pub name: &'static str,
    pub min_freq: f64,
    pub build: fn(f64) -> nurbs::algebra::PiecewisePolynomialKernel,
}

pub const INPUT_SMOOTHERS: [SmootherCfg; 2] = [
    SmootherCfg {
        name: "smooth_zv",
        min_freq: 18.0,
        build: trajectory::build_smooth_zv_kernel,
    },
    SmootherCfg {
        name: "smooth_mzv",
        min_freq: 20.0,
        build: trajectory::build_smooth_mzv_kernel,
    },
];

pub fn find_smoother_cfg(name: &str) -> Option<&'static SmootherCfg> {
    INPUT_SMOOTHERS.iter().find(|c| c.name == name)
}

fn eval_kernel(kernel: &nurbs::algebra::PiecewisePolynomialKernel, t: f64) -> f64 {
    let (lo, hi) = kernel.support();
    if t < lo || t > hi {
        return 0.0;
    }
    for p in &kernel.pieces {
        if t >= p.u_start - 1e-15 && t <= p.u_end + 1e-15 {
            return p.evaluate(t);
        }
    }
    panic!("kernel pieces do not cover t={t} within support [{lo}, {hi}]");
}

/// Port of `estimate_smoother_old` from bleeding-edge-v2: residual vibration
/// magnitude of a damped oscillator under the smoothing kernel, per frequency.
/// The kernel is unit-norm, so the response at 0 Hz is 1.
pub fn estimate_smoother(
    kernel: &nurbs::algebra::PiecewisePolynomialKernel,
    damping_ratio: f64,
    freqs: &[f64],
) -> Vec<f64> {
    let (lo, hi) = kernel.support();
    let span = hi - lo;
    let f_max = freqs.iter().copied().fold(0.0, f64::max);
    let n_t = (100.0 * (span * f_max).round()).max(1000.0) as usize;
    let dt = span / n_t as f64;
    let w: Vec<f64> = (0..=n_t)
        .map(|i| eval_kernel(kernel, lo + i as f64 * dt))
        .collect();
    let sq = (1.0 - damping_ratio * damping_ratio).sqrt();
    freqs
        .iter()
        .map(|&f| {
            let omega = 2.0 * PI * f;
            let damping = damping_ratio * omega;
            let omega_d = omega * sq;
            let mut vc = 0.0;
            let mut vs = 0.0;
            for (i, &wi) in w.iter().enumerate() {
                let tau = lo + i as f64 * dt - hi;
                let e = wi * libm::exp(damping * tau);
                let trapz = if i == 0 || i == n_t { 0.5 } else { 1.0 };
                vc += trapz * e * libm::cos(omega_d * tau);
                vs += trapz * e * libm::sin(omega_d * tau);
            }
            (vc * vc + vs * vs).sqrt() * dt
        })
        .collect()
}

/// First and second moments of the mirrored kernel `w(-t)`, split at t = 0.
/// The mirror matches `_get_smoother_smoothing` in bleeding-edge-v2, which
/// evaluates the polynomial at `-t` (convolution orientation).
pub struct SmootherMoments {
    pub m1_pos: f64,
    pub m1_neg: f64,
    pub m2_pos: f64,
    pub m2_neg: f64,
}

pub fn smoother_moments(kernel: &nurbs::algebra::PiecewisePolynomialKernel) -> SmootherMoments {
    let (lo, hi) = kernel.support();
    let mut m = SmootherMoments {
        m1_pos: 0.0,
        m1_neg: 0.0,
        m2_pos: 0.0,
        m2_neg: 0.0,
    };
    let integrate = |a: f64, b: f64, m1: &mut f64, m2: &mut f64| {
        const N: usize = 4096;
        let dt = (b - a) / N as f64;
        for i in 0..=N {
            let t = a + i as f64 * dt;
            let w = eval_kernel(kernel, -t);
            let trapz = if i == 0 || i == N { 0.5 } else { 1.0 };
            *m1 += trapz * t * w * dt;
            *m2 += trapz * t * t * w * dt;
        }
    };
    integrate(-hi, 0.0, &mut m.m1_neg, &mut m.m2_neg);
    integrate(0.0, -lo, &mut m.m1_pos, &mut m.m2_pos);
    m
}

/// Port of `_get_smoother_smoothing`: toolhead offset on 90/180 degree turns.
/// `inv_freq` rescales the unit-frequency moments (`m1 ~ 1/f`, `m2 ~ 1/f^2`).
pub fn get_smoother_smoothing(
    moments: &SmootherMoments,
    inv_freq: f64,
    accel: f64,
    scv: f64,
) -> f64 {
    let half_accel = accel * 0.5;
    let inv_f2 = inv_freq * inv_freq;
    let offset_90_x = scv * moments.m1_pos * inv_freq + half_accel * moments.m2_pos * inv_f2;
    let offset_90_y = scv * moments.m1_neg * inv_freq - half_accel * moments.m2_neg * inv_f2;
    let offset_90 = (offset_90_x * offset_90_x + offset_90_y * offset_90_y).sqrt();
    let offset_180 = half_accel * (moments.m2_pos + moments.m2_neg) * inv_f2;
    offset_90.max(offset_180.abs())
}

pub fn find_smoother_max_accel(moments: &SmootherMoments, inv_freq: f64, scv: f64) -> f64 {
    const TARGET_SMOOTHING: f64 = 0.12;
    bisect(|test_accel| {
        get_smoother_smoothing(moments, inv_freq, test_accel, scv) <= TARGET_SMOOTHING
    })
}

const SMOOTHER_NORM_FREQ_MAX: f64 = 10.0;
const SMOOTHER_NORM_FREQ_STEP: f64 = 0.01;

/// `numpy.interp` with edge clamping on a uniform grid starting at 0.
fn interp_uniform(vals: &[f64], step: f64, x: f64) -> f64 {
    if x <= 0.0 {
        return vals[0];
    }
    let pos = x / step;
    let i = pos as usize;
    if i + 1 >= vals.len() {
        return *vals.last().unwrap();
    }
    let frac = pos - i as f64;
    vals[i] + (vals[i + 1] - vals[i]) * frac
}

/// Fit a smoother family against a PSD, mirroring `fit_shaper`'s sweep and
/// selection. The kernel scales as `1/freq`, so the vibration response is
/// computed once on a normalized frequency grid for the 1 Hz kernel and
/// resampled per test frequency.
pub fn fit_smoother(
    cfg: &SmootherCfg,
    freq_bins_in: &[f64],
    psd_in: &[f64],
    shaper_freqs: &ShaperFreqs,
    scv: f64,
    max_smoothing: Option<f64>,
    test_damping_ratios: Option<Vec<f64>>,
    max_freq: Option<f64>,
) -> Option<FitResult> {
    let test_damping_ratios = test_damping_ratios.unwrap_or_else(|| TEST_DAMPING_RATIOS.to_vec());

    let test_freqs: Vec<f64> = match shaper_freqs {
        ShaperFreqs::List(v) => v.clone(),
        ShaperFreqs::Range(start, end, step) => {
            let freq_end = end.unwrap_or(MAX_SHAPER_FREQ);
            let freq_start = start.unwrap_or(cfg.min_freq).min(freq_end - 1e-7);
            let freq_step = step.unwrap_or(0.2);
            arange(freq_start, freq_end, freq_step)
        }
    };
    if test_freqs.is_empty() {
        return None;
    }
    let test_max = test_freqs.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let base = max_freq.filter(|&m| m != 0.0).unwrap_or(MAX_FREQ);
    let max_freq = base.max(test_max);

    let mut freq_bins = Vec::new();
    let mut psd = Vec::new();
    for i in 0..freq_bins_in.len() {
        if freq_bins_in[i] <= max_freq {
            freq_bins.push(freq_bins_in[i]);
            psd.push(psd_in[i]);
        }
    }

    let unit_kernel = (cfg.build)(1.0);
    let norm_freqs = arange(0.0, SMOOTHER_NORM_FREQ_MAX, SMOOTHER_NORM_FREQ_STEP);
    let mut unit_vals = vec![0.0_f64; norm_freqs.len()];
    for &dr in &test_damping_ratios {
        let vals = estimate_smoother(&unit_kernel, dr, &norm_freqs);
        for i in 0..unit_vals.len() {
            unit_vals[i] = unit_vals[i].max(vals[i]);
        }
    }
    let moments = smoother_moments(&unit_kernel);

    let mut best: Option<FitResult> = None;
    let mut results: Vec<FitResult> = Vec::new();

    for &test_freq in test_freqs.iter().rev() {
        let inv_freq = 1.0 / test_freq;
        let smoothing = get_smoother_smoothing(&moments, inv_freq, 5000.0, scv);
        if let (Some(ms), Some(_)) = (max_smoothing, best.as_ref()) {
            if smoothing > ms {
                return best;
            }
        }
        let vals: Vec<f64> = freq_bins
            .iter()
            .map(|&fb| interp_uniform(&unit_vals, SMOOTHER_NORM_FREQ_STEP, fb * inv_freq))
            .collect();
        let vibrations = remaining_vibrations(&vals, &psd);
        let max_accel = find_smoother_max_accel(&moments, inv_freq, scv);
        let score = smoothing * (libm::pow(vibrations, 1.5) + vibrations * 0.2 + 0.01);
        results.push(FitResult {
            name: cfg.name.to_string(),
            freq: test_freq,
            vals,
            vibrs: vibrations,
            smoothing,
            score,
            max_accel,
        });
        let last = results.last().unwrap();
        if best.as_ref().map_or(true, |b| b.vibrs > last.vibrs) {
            best = Some(clone_result(last));
        }
    }

    let best = best?;
    let mut selected = clone_result(&best);
    for res in results.iter().rev() {
        if res.vibrs < best.vibrs * 1.1 && res.score < selected.score {
            selected = clone_result(res);
        }
    }
    Some(selected)
}

/// numpy.arange(start, stop, step): half-open, floating accumulation.
fn arange(start: f64, stop: f64, step: f64) -> Vec<f64> {
    let n = ((stop - start) / step).ceil();
    let n = if n.is_finite() && n > 0.0 {
        n as usize
    } else {
        0
    };
    (0..n).map(|i| start + i as f64 * step).collect()
}
