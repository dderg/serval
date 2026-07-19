//! Ring-down modal analysis for `SERVO_MEASURE_RINGDOWN` runs: after each
//! stroke stops, the residual vibration is a free decay of the closed-loop
//! plant (drive compensation included, exactly as a print corner sees it).
//! Each post-stop tail is band-isolated around its PSD peaks via an FFT
//! analytic signal; the log envelope and unwrapped phase then give per-mode
//! decay rate and damped frequency, hence natural frequency and damping
//! ratio — without the steady-state excitation a drive's adaptive filters
//! can fight.

use core::f64::consts::PI;
use core::ops::Range;

use crate::metrics::{target_motion_segments, DEFAULT_SETTLE_BAND_COUNTS};
use crate::psd::{fft_pow2, welch_psd};
use crate::results::{
    PlotRingdown, PlotRingdownSource, PlotRingdownTail, RingdownMode, RingdownResult,
    RingdownSource,
};
use crate::scap::Scap;

pub const DEFAULT_GUARD_MS: f64 = 10.0;
pub const RINGDOWN_WINDOW_MARGIN_MS: f64 = 50.0;
pub const MIN_TAIL_MS: f64 = 250.0;
pub const RINGDOWN_BAND_HZ: (f64, f64) = (10.0, 450.0);
pub const MAX_RINGDOWN_MODES: usize = 5;
pub const PEAK_OVER_MEDIAN: f64 = 6.0;
pub const MIN_FIT_CYCLES: f64 = 3.0;
pub const MIN_FIT_R2: f64 = 0.5;
const MIN_PHASE_R2: f64 = 0.9;
const MAX_PLOT_TAILS: usize = 8;

#[derive(Debug, Clone, Copy)]
pub struct RingdownOptions {
    pub guard_s: f64,
    pub window_s: f64,
    pub band_hz: (f64, f64),
}

#[derive(Debug, Clone, Copy)]
pub struct DecayFit {
    pub freq_hz: f64,
    pub zeta: f64,
    pub sigma: f64,
    pub amp: f64,
    pub cycles: f64,
    pub r2: f64,
    pub fit_start_s: f64,
}

pub fn detrend_linear(x: &mut [f64]) {
    let n = x.len();
    if n < 2 {
        return;
    }
    let nf = n as f64;
    let mean_t = (nf - 1.0) / 2.0;
    let mean_x = x.iter().sum::<f64>() / nf;
    let mut num = 0.0;
    let mut den = 0.0;
    for (k, &v) in x.iter().enumerate() {
        let dt = k as f64 - mean_t;
        num += dt * (v - mean_x);
        den += dt * dt;
    }
    let slope = if den > 0.0 { num / den } else { 0.0 };
    for (k, v) in x.iter_mut().enumerate() {
        *v -= mean_x + slope * (k as f64 - mean_t);
    }
}

/// Band-limited analytic signal via FFT: positive-frequency bins inside
/// `[f_lo, f_hi]` are kept (doubled, raised-cosine edges), everything else
/// zeroed, then inverse-transformed. The tail is even-reflected on both
/// sides first so the transform sees no value discontinuity at the window
/// edges — an abrupt edge would otherwise splatter energy into every band
/// and masquerade as a fast decay. The remaining derivative kinks at the
/// mirror joints are what the per-fit edge skip absorbs. Returns
/// `(envelope, phase)` per original sample.
pub fn analytic_band(x: &[f64], fs: f64, f_lo: f64, f_hi: f64) -> (Vec<f64>, Vec<f64>) {
    let n = x.len();
    let m = (3 * n).next_power_of_two();
    let mut re = vec![0.0f64; m];
    for k in 0..n {
        re[k] = x[n - 1 - k];
        re[n + k] = x[k];
        re[2 * n + k] = x[n - 1 - k];
    }
    let mut im = vec![0.0f64; m];
    fft_pow2(&mut re, &mut im);
    let bin_hz = fs / m as f64;
    let edge = (0.05 * (f_hi - f_lo)).max(2.0 * bin_hz);
    for k in 0..m {
        let weight = if k == 0 || k > m / 2 {
            0.0
        } else {
            2.0 * band_weight(k as f64 * bin_hz, f_lo, f_hi, edge)
        };
        re[k] *= weight;
        im[k] *= weight;
    }
    for v in im.iter_mut() {
        *v = -*v;
    }
    fft_pow2(&mut re, &mut im);
    let inv = 1.0 / m as f64;
    let env = (n..2 * n)
        .map(|k| libm::hypot(re[k] * inv, im[k] * inv))
        .collect();
    let phase = (n..2 * n).map(|k| libm::atan2(-im[k], re[k])).collect();
    (env, phase)
}

fn band_weight(f: f64, f_lo: f64, f_hi: f64, edge: f64) -> f64 {
    if f < f_lo || f > f_hi {
        return 0.0;
    }
    let ramp = |u: f64| {
        if u >= 1.0 {
            1.0
        } else {
            0.5 - 0.5 * libm::cos(PI * u)
        }
    };
    ramp((f - f_lo) / edge).min(ramp((f_hi - f) / edge))
}

fn linear_regression(t: &[f64], y: &[f64]) -> (f64, f64, f64) {
    let n = t.len() as f64;
    let mean_t = t.iter().sum::<f64>() / n;
    let mean_y = y.iter().sum::<f64>() / n;
    let mut num = 0.0;
    let mut den = 0.0;
    for (tk, yk) in t.iter().zip(y) {
        num += (tk - mean_t) * (yk - mean_y);
        den += (tk - mean_t) * (tk - mean_t);
    }
    let slope = if den > 0.0 { num / den } else { 0.0 };
    let intercept = mean_y - slope * mean_t;
    let mut ss_res = 0.0;
    let mut ss_tot = 0.0;
    for (tk, yk) in t.iter().zip(y) {
        let pred = intercept + slope * tk;
        ss_res += (yk - pred) * (yk - pred);
        ss_tot += (yk - mean_y) * (yk - mean_y);
    }
    let r2 = if ss_tot > 0.0 {
        1.0 - ss_res / ss_tot
    } else {
        0.0
    };
    (slope, intercept, r2)
}

fn unwrap_phase(phase: &[f64]) -> Vec<f64> {
    let mut out = Vec::with_capacity(phase.len());
    let mut offset = 0.0;
    for (k, &p) in phase.iter().enumerate() {
        if k > 0 {
            let d = p - phase[k - 1];
            if d > PI {
                offset -= 2.0 * PI;
            } else if d < -PI {
                offset += 2.0 * PI;
            }
        }
        out.push(p + offset);
    }
    out
}

/// Fit one decaying mode from a band-isolated tail. The envelope fit runs
/// from the post-edge envelope peak down to the noise floor; the phase slope
/// over the same span gives the damped frequency. Rejects fits spanning
/// fewer than `MIN_FIT_CYCLES` cycles, off-band frequencies, incoherent
/// phase, or log-envelope fits below `MIN_FIT_R2`.
pub fn fit_decay(env: &[f64], phase: &[f64], fs: f64, f0: f64) -> Option<DecayFit> {
    let n = env.len();
    if f0 <= 0.0 {
        return None;
    }
    let period = (fs / f0).ceil() as usize;
    let edge = period.max((0.01 * fs).ceil() as usize);
    if n <= 4 * edge {
        return None;
    }
    let usable_end = n - edge;
    let search_end = (edge + 3 * period).min(usable_end);
    let start = (edge..search_end)
        .max_by(|&a, &b| env[a].partial_cmp(&env[b]).expect("envelope holds a NaN"))?;
    let amp_start = env[start];
    if amp_start <= 0.0 {
        return None;
    }
    let mut floor_tail: Vec<f64> = env[usable_end - (n / 5).max(1)..usable_end].to_vec();
    floor_tail.sort_by(|a, b| a.partial_cmp(b).expect("envelope holds a NaN"));
    let noise_floor = floor_tail[floor_tail.len() / 2];
    let threshold = (2.5 * noise_floor).max(amp_start * 0.02);
    let mut end = usable_end;
    for k in start..usable_end {
        if env[k] < threshold {
            end = k;
            break;
        }
    }
    let min_span = ((MIN_FIT_CYCLES * fs / f0).ceil() as usize).max(24);
    if end - start < min_span {
        return None;
    }
    let t: Vec<f64> = (start..end).map(|k| k as f64 / fs).collect();
    let log_env: Vec<f64> = env[start..end]
        .iter()
        .map(|&v| libm::log(v.max(1e-300)))
        .collect();
    let (slope, intercept, r2) = linear_regression(&t, &log_env);
    if r2 < MIN_FIT_R2 {
        return None;
    }
    let sigma = -slope;
    let unwrapped = unwrap_phase(&phase[start..end]);
    let (omega_d, _, phase_r2) = linear_regression(&t, &unwrapped);
    if omega_d <= 0.0 || phase_r2 < MIN_PHASE_R2 {
        return None;
    }
    let f_d = omega_d / (2.0 * PI);
    if (f_d - f0).abs() > 0.35 * f0 {
        return None;
    }
    let omega_n = libm::hypot(sigma, omega_d);
    Some(DecayFit {
        freq_hz: omega_n / (2.0 * PI),
        zeta: sigma / omega_n,
        sigma,
        amp: libm::exp(intercept + slope * (start as f64 / fs)),
        cycles: (end - start) as f64 / fs * f_d,
        r2,
        fit_start_s: start as f64 / fs,
    })
}

fn band_median(psd: &[f64], idx: &[usize]) -> f64 {
    let mut vals: Vec<f64> = idx.iter().map(|&i| psd[i]).collect();
    vals.sort_by(|a, b| a.partial_cmp(b).expect("psd holds a NaN"));
    if vals.is_empty() {
        0.0
    } else {
        vals[vals.len() / 2]
    }
}

/// PSD peaks worth fitting: strict local maxima inside the band, at least
/// `PEAK_OVER_MEDIAN` over the band's median power, deduped within
/// max(3 Hz, 5%), strongest first, at most `MAX_RINGDOWN_MODES`.
pub fn pick_peaks(freqs: &[f64], psd: &[f64], f_lo: f64, f_hi: f64) -> Vec<f64> {
    let band: Vec<usize> = (0..freqs.len())
        .filter(|&i| freqs[i] >= f_lo && freqs[i] <= f_hi)
        .collect();
    if band.len() < 3 {
        return Vec::new();
    }
    let floor = band_median(psd, &band);
    let mut candidates: Vec<usize> = band[1..band.len() - 1]
        .iter()
        .copied()
        .filter(|&i| {
            psd[i] > psd[i - 1] && psd[i] >= psd[i + 1] && psd[i] > PEAK_OVER_MEDIAN * floor
        })
        .collect();
    candidates.sort_by(|&a, &b| psd[b].partial_cmp(&psd[a]).expect("psd holds a NaN"));
    let mut picked: Vec<f64> = Vec::new();
    for i in candidates {
        let f = freqs[i];
        if picked
            .iter()
            .any(|&p| (f - p).abs() < 3.0_f64.max(0.05 * p))
        {
            continue;
        }
        picked.push(f);
        if picked.len() >= MAX_RINGDOWN_MODES {
            break;
        }
    }
    picked
}

fn isolation_band(f0: f64, peaks: &[f64], band_hz: (f64, f64), fs: f64) -> (f64, f64) {
    let mut lo = f0 / 1.35;
    let mut hi = f0 * 1.35;
    for &p in peaks {
        if p < f0 {
            lo = lo.max((p + f0) / 2.0);
        } else if p > f0 {
            hi = hi.min((p + f0) / 2.0);
        }
    }
    let min_half = 4.0_f64;
    (
        lo.min(f0 - min_half).max(band_hz.0.min(f0 - min_half)),
        hi.max(f0 + min_half).min((0.49 * fs).max(f0 + min_half)),
    )
}

/// All decay fits for one tail: detrend, find PSD peaks, isolate and fit
/// each. A heavily damped mode decays faster than the isolation filter's
/// own response can follow, biasing ζ low — such fits are redone once with
/// the band widened to at least `8σ/π` Hz (still clamped away from
/// neighboring peaks). Also returns the tail's Welch PSD for
/// averaging/plotting.
pub fn analyze_tail(
    tail: &[f64],
    fs: f64,
    band_hz: (f64, f64),
) -> Result<(Vec<DecayFit>, Vec<f64>, Vec<f64>), String> {
    let mut x = tail.to_vec();
    detrend_linear(&mut x);
    let (freqs, psd) = welch_psd(&x, fs)?;
    let f_hi = band_hz.1.min(0.45 * fs);
    let peaks = pick_peaks(&freqs, &psd, band_hz.0, f_hi);
    let mut fits = Vec::new();
    for &f0 in &peaks {
        let (lo, hi) = isolation_band(f0, &peaks, (band_hz.0, f_hi), fs);
        let (env, phase) = analytic_band(&x, fs, lo, hi);
        let Some(first) = fit_decay(&env, &phase, fs, f0) else {
            continue;
        };
        let needed_hz = 8.0 * first.sigma / PI;
        if hi - lo >= needed_hz {
            fits.push(first);
            continue;
        }
        let mut lo2 = (f0 - needed_hz / 2.0).max(band_hz.0.min(f0 / 2.0));
        let mut hi2 = (f0 + needed_hz / 2.0).min(0.49 * fs);
        for &p in &peaks {
            if p < f0 {
                lo2 = lo2.max((p + f0) / 2.0);
            } else if p > f0 {
                hi2 = hi2.min((p + f0) / 2.0);
            }
        }
        let (env2, phase2) = analytic_band(&x, fs, lo2, hi2);
        fits.push(fit_decay(&env2, &phase2, fs, f0).unwrap_or(first));
    }
    Ok((fits, freqs, psd))
}

fn median_of(mut vals: Vec<f64>) -> f64 {
    vals.sort_by(|a, b| a.partial_cmp(b).expect("mode statistic holds a NaN"));
    let mid = vals.len() / 2;
    if vals.len() % 2 == 1 {
        vals[mid]
    } else {
        0.5 * (vals[mid - 1] + vals[mid])
    }
}

fn mode_from_cluster(cluster: &[DecayFit], unit_is_accel: bool) -> RingdownMode {
    let freq_hz = median_of(cluster.iter().map(|f| f.freq_hz).collect());
    let zeta = median_of(cluster.iter().map(|f| f.zeta).collect());
    let amp = median_of(cluster.iter().map(|f| f.amp).collect());
    let omega = 2.0 * PI * freq_hz;
    let disp_um = if unit_is_accel {
        amp / (omega * omega) * 1000.0
    } else {
        amp
    };
    RingdownMode {
        freq_hz,
        zeta,
        zeta_lo: cluster.iter().map(|f| f.zeta).fold(f64::INFINITY, f64::min),
        zeta_hi: cluster
            .iter()
            .map(|f| f.zeta)
            .fold(f64::NEG_INFINITY, f64::max),
        amp,
        disp_um,
        tails: cluster.len(),
        cycles: cluster.iter().map(|f| f.cycles).sum::<f64>() / cluster.len() as f64,
        r2: cluster.iter().map(|f| f.r2).sum::<f64>() / cluster.len() as f64,
        fit_start_ms: median_of(cluster.iter().map(|f| f.fit_start_s * 1000.0).collect()),
    }
}

/// Cluster per-tail fits by frequency (within max(3 Hz, 6%)) and summarize
/// each cluster. With several tails a mode needs at least two sightings to
/// beat noise; a single-tail source keeps single sightings. Sorted by
/// frequency.
pub fn aggregate_modes(per_tail: &[Vec<DecayFit>], unit_is_accel: bool) -> Vec<RingdownMode> {
    let mut all: Vec<DecayFit> = per_tail.iter().flatten().copied().collect();
    if all.is_empty() {
        return Vec::new();
    }
    all.sort_by(|a, b| {
        a.freq_hz
            .partial_cmp(&b.freq_hz)
            .expect("fit frequency holds a NaN")
    });
    let mut clusters: Vec<Vec<DecayFit>> = Vec::new();
    for fit in all {
        match clusters.last_mut() {
            Some(cluster) => {
                let center = median_of(cluster.iter().map(|f| f.freq_hz).collect());
                if (fit.freq_hz - center).abs() < 3.0_f64.max(0.06 * center) {
                    cluster.push(fit);
                } else {
                    clusters.push(vec![fit]);
                }
            }
            None => clusters.push(vec![fit]),
        }
    }
    let min_tails = if per_tail.len() > 1 { 2 } else { 1 };
    let mut modes: Vec<RingdownMode> = clusters
        .iter()
        .filter(|c| c.len() >= min_tails)
        .map(|c| mode_from_cluster(c, unit_is_accel))
        .collect();
    modes.sort_by(|a, b| {
        a.freq_hz
            .partial_cmp(&b.freq_hz)
            .expect("mode frequency holds a NaN")
    });
    modes.truncate(MAX_RINGDOWN_MODES);
    modes
}

/// Post-stop sample ranges: from each motion segment's end (plus guard) to
/// the next segment start, capped at `max_len`. All ranges are then cropped
/// to the shortest so every tail shares one length (uniform PSD grids).
pub fn tail_ranges(
    segs: &[(usize, usize)],
    n: usize,
    guard: usize,
    max_len: usize,
) -> Vec<Range<usize>> {
    let mut out = Vec::new();
    for (i, &(_, e)) in segs.iter().enumerate() {
        let start = e + guard;
        let cap = if i + 1 < segs.len() { segs[i + 1].0 } else { n };
        let end = (start + max_len).min(cap);
        if end > start {
            out.push(start..end);
        }
    }
    let min_len = out.iter().map(Range::len).min().unwrap_or(0);
    for r in &mut out {
        r.end = r.start + min_len;
    }
    out
}

const PLOT_NOISE_MULT: f64 = 2.0;
const PLOT_SPAN_PAD_FRACTION: f64 = 0.3;
const MIN_PLOT_SPAN_MS: f64 = 100.0;

/// Samples worth drawing: the ring is over once every ~5 ms block of every
/// tail sits at the noise floor, so the plotted span ends there (padded
/// 30%) instead of stretching the ring into the left edge of a chart that
/// is mostly quiet dwell. A ring that never settles keeps the full window.
pub fn informative_plot_len(tails: &[Vec<f64>], fs: f64, noise_floor: f64) -> usize {
    let n = tails.iter().map(Vec::len).min().unwrap_or(0);
    if n == 0 {
        return 0;
    }
    let block = ((0.005 * fs).ceil() as usize).max(1);
    let mut last_active = 0usize;
    for tail in tails {
        let mut start = 0usize;
        while start < tail.len() {
            let end = (start + block).min(tail.len());
            let seg = &tail[start..end];
            let mean = seg.iter().sum::<f64>() / seg.len() as f64;
            let rms = (seg.iter().map(|&v| (v - mean) * (v - mean)).sum::<f64>()
                / seg.len() as f64)
                .sqrt();
            if rms > PLOT_NOISE_MULT * noise_floor {
                last_active = last_active.max(end);
            }
            start = end;
        }
    }
    let min_span = ((MIN_PLOT_SPAN_MS / 1000.0 * fs).ceil() as usize).min(n);
    let padded = last_active + (last_active as f64 * PLOT_SPAN_PAD_FRACTION) as usize + block;
    padded.max(min_span).min(n)
}

fn late_window_rms(tails: &[Vec<f64>]) -> f64 {
    let mut per_tail: Vec<f64> = Vec::new();
    for t in tails {
        let start = t.len() - (t.len() / 6).max(1);
        let seg = &t[start..];
        let mean = seg.iter().sum::<f64>() / seg.len() as f64;
        per_tail.push(
            (seg.iter().map(|&v| (v - mean) * (v - mean)).sum::<f64>() / seg.len() as f64).sqrt(),
        );
    }
    if per_tail.is_empty() {
        0.0
    } else {
        median_of(per_tail)
    }
}

struct SourceInput {
    source: String,
    unit: String,
    fs: f64,
    tails: Vec<Vec<f64>>,
    starts_s: Vec<f64>,
    plot_tails: bool,
}

fn analyze_source(
    input: &SourceInput,
    band_hz: (f64, f64),
) -> Result<(RingdownSource, PlotRingdownSource), String> {
    let unit_is_accel = input.unit == "mm/s2";
    let mut per_tail_fits = Vec::new();
    let mut psd_freqs: Vec<f64> = Vec::new();
    let mut psd_acc: Vec<f64> = Vec::new();
    for tail in &input.tails {
        let (fits, freqs, psd) = analyze_tail(tail, input.fs, band_hz)
            .map_err(|e| format!("source {}: {e}", input.source))?;
        per_tail_fits.push(fits);
        if psd_acc.is_empty() {
            psd_freqs = freqs;
            psd_acc = psd;
        } else {
            assert_eq!(
                psd_freqs.len(),
                psd.len(),
                "uniform tail lengths must share one Welch grid"
            );
            for (a, p) in psd_acc.iter_mut().zip(&psd) {
                *a += p;
            }
        }
    }
    let n_tails = input.tails.len();
    for p in psd_acc.iter_mut() {
        *p /= n_tails as f64;
    }
    let modes = aggregate_modes(&per_tail_fits, unit_is_accel);
    let noise_floor = late_window_rms(&input.tails);

    let mut plot_tails = Vec::new();
    if input.plot_tails {
        let plot_len = informative_plot_len(&input.tails, input.fs, noise_floor);
        for (i, tail) in input.tails.iter().take(MAX_PLOT_TAILS).enumerate() {
            plot_tails.push(PlotRingdownTail {
                start_s: input.starts_s.get(i).copied().unwrap_or(0.0),
                value: tail[..plot_len.min(tail.len())].to_vec(),
            });
        }
    }
    Ok((
        RingdownSource {
            source: input.source.clone(),
            unit: input.unit.clone(),
            tails: n_tails,
            noise_floor,
            modes: modes.clone(),
        },
        PlotRingdownSource {
            source: input.source.clone(),
            unit: input.unit.clone(),
            fs_hz: input.fs,
            modes,
            psd_freq_hz: psd_freqs,
            psd: psd_acc,
            tails: plot_tails,
        },
    ))
}

fn read_accel_axes(path: &std::path::Path) -> Result<(Vec<f64>, [Vec<f64>; 3]), String> {
    let text =
        std::fs::read_to_string(path).map_err(|e| format!("read {}: {e}", path.display()))?;
    let mut t = Vec::new();
    let mut axes: [Vec<f64>; 3] = [Vec::new(), Vec::new(), Vec::new()];
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let f: Vec<f64> = line
            .split(',')
            .map(|s| s.trim().parse::<f64>())
            .collect::<Result<_, _>>()
            .map_err(|_| format!("{}: non-numeric accel row {line:?}", path.display()))?;
        if f.len() < 4 {
            return Err(format!(
                "{}: expected time,accel_x,accel_y,accel_z rows",
                path.display()
            ));
        }
        t.push(f[0]);
        for (axis, v) in axes.iter_mut().zip(&f[1..4]) {
            axis.push(*v);
        }
    }
    if t.len() < 2 {
        return Err(format!("{}: too few accel samples", path.display()));
    }
    Ok((t, axes))
}

fn accel_tails(
    t: &[f64],
    axis: &[f64],
    stops: &[f64],
    guard_s: f64,
    window_s: f64,
) -> Result<Vec<Vec<f64>>, String> {
    let mut tails = Vec::new();
    for &stop in stops {
        let lo = stop + guard_s;
        let hi = stop + window_s;
        let start = t.partition_point(|&x| x < lo);
        let end = t.partition_point(|&x| x < hi);
        if end <= start {
            return Err(format!(
                "accel capture holds no samples in the ring-down window \
                 {lo:.3}..{hi:.3}s after the stop at {stop:.3}s — accel data \
                 spans {:.3}..{:.3}s",
                t[0],
                t[t.len() - 1]
            ));
        }
        tails.push(axis[start..end].to_vec());
    }
    let min_len = tails.iter().map(Vec::len).min().unwrap_or(0);
    for tail in &mut tails {
        tail.truncate(min_len);
    }
    Ok(tails)
}

fn stroke_segments(cap: &Scap, drive_idx: usize) -> Result<Vec<(usize, usize)>, String> {
    let target = cap.read_i64(drive_idx, "target_counts")?;
    Ok(target_motion_segments(&target, cap.fs())
        .into_iter()
        .filter(|&(s, e)| {
            let window = &target[s - 1..e];
            let lo = window.iter().min().expect("segment is nonempty");
            let hi = window.iter().max().expect("segment is nonempty");
            hi - lo > DEFAULT_SETTLE_BAND_COUNTS
        })
        .collect())
}

fn require_min_tail(tails: &[Vec<f64>], fs: f64, what: &str) -> Result<(), String> {
    let min_len = tails.iter().map(Vec::len).min().unwrap_or(0);
    let min_needed = (MIN_TAIL_MS / 1000.0 * fs) as usize;
    if min_len < min_needed {
        return Err(format!(
            "{what}: shortest ring-down tail is {min_len} samples \
             ({:.0} ms at {fs:.0} Hz) but at least {MIN_TAIL_MS:.0} ms is \
             needed — raise DWELL_MS",
            min_len as f64 / fs * 1000.0
        ));
    }
    Ok(())
}

struct ModeCluster {
    freq_hz: f64,
    zeta: f64,
    disp_um: f64,
    tails: usize,
}

fn cluster_step_modes(modes: &[&RingdownMode]) -> Vec<ModeCluster> {
    let mut sorted: Vec<&RingdownMode> = modes.to_vec();
    sorted.sort_by(|a, b| {
        a.freq_hz
            .partial_cmp(&b.freq_hz)
            .expect("mode frequency holds a NaN")
    });
    let mut clusters: Vec<Vec<&RingdownMode>> = Vec::new();
    for m in sorted {
        match clusters.last_mut() {
            Some(cluster)
                if (m.freq_hz - cluster[0].freq_hz).abs()
                    < 3.0_f64.max(0.06 * cluster[0].freq_hz) =>
            {
                cluster.push(m);
            }
            _ => clusters.push(vec![m]),
        }
    }
    clusters
        .iter()
        .map(|c| ModeCluster {
            freq_hz: median_of(c.iter().map(|m| m.freq_hz).collect()),
            zeta: median_of(c.iter().map(|m| m.zeta).collect()),
            disp_um: median_of(c.iter().map(|m| m.disp_um).collect()),
            tails: c.iter().map(|m| m.tails).sum(),
        })
        .collect()
}

/// One-line verdict for a ringdown run: the dominant residual-vibration mode
/// of the most informative source (accelerometer first, belt-combined next,
/// then raw drives), aggregated across every step, plus the shaper
/// parameters it implies.
pub fn ringdown_verdict_reason(steps: &[&RingdownResult]) -> String {
    let mut order: Vec<String> = Vec::new();
    let mut by_source: std::collections::BTreeMap<String, Vec<&RingdownMode>> =
        std::collections::BTreeMap::new();
    for rr in steps {
        for src in &rr.sources {
            if !order.contains(&src.source) {
                order.push(src.source.clone());
            }
            by_source
                .entry(src.source.clone())
                .or_default()
                .extend(src.modes.iter());
        }
    }
    let score = |name: &str| {
        by_source
            .get(name)
            .map(|modes| modes.iter().map(|m| m.disp_um).fold(0.0_f64, f64::max))
            .unwrap_or(0.0)
    };
    let accel_pick = order
        .iter()
        .filter(|n| n.starts_with("accel_"))
        .max_by(|a, b| {
            score(a)
                .partial_cmp(&score(b))
                .expect("mode displacement holds a NaN")
        })
        .filter(|n| score(n) > 0.0);
    let headline = accel_pick
        .or_else(|| order.iter().find(|n| *n == "combined" && score(n) > 0.0))
        .or_else(|| order.iter().find(|n| score(n) > 0.0));
    let Some(headline) = headline else {
        return "no resonant ring above the noise floor after any stop".to_string();
    };
    let clusters = cluster_step_modes(&by_source[headline]);
    let dominant = clusters
        .iter()
        .max_by(|a, b| {
            a.disp_um
                .partial_cmp(&b.disp_um)
                .expect("mode displacement holds a NaN")
        })
        .expect("headline source has modes");
    let mut reason = format!(
        "ring after stop: {:.1} Hz ζ {:.3}, {:.2} µm ({}, {} tails); \
         shaper hint frequency_hz={:.1} damping_ratio={:.3}",
        dominant.freq_hz,
        dominant.zeta,
        dominant.disp_um,
        headline,
        dominant.tails,
        dominant.freq_hz,
        dominant.zeta.max(0.0)
    );
    let secondary = clusters
        .iter()
        .filter(|c| (c.freq_hz - dominant.freq_hz).abs() > 1e-9)
        .filter(|c| c.disp_um >= 0.3 * dominant.disp_um)
        .max_by(|a, b| {
            a.disp_um
                .partial_cmp(&b.disp_um)
                .expect("mode displacement holds a NaN")
        });
    if let Some(s) = secondary {
        reason.push_str(&format!(
            "; also {:.1} Hz ζ {:.3}, {:.2} µm",
            s.freq_hz, s.zeta, s.disp_um
        ));
    }
    reason
}

/// Analyze one ringdown step capture (plus its optional accelerometer CSV)
/// into per-source aggregated modes and plot payloads. `stops_pt` are the
/// per-stroke commanded-stop print-times klippy recorded; they window the
/// accelerometer tails, while servo tails come from the capture's own
/// target-motion segments.
pub fn compute_step_ringdown(
    cap: &Scap,
    name: &str,
    belts: Option<&str>,
    axis: Option<&str>,
    accel_path: Option<&std::path::Path>,
    stops_pt: Option<&[f64]>,
    expected_strokes: usize,
    opts: &RingdownOptions,
) -> Result<(RingdownResult, PlotRingdown), String> {
    let fs = cap.fs();
    let guard = (opts.guard_s * fs).round() as usize;
    let max_len = (opts.window_s * fs).round() as usize;
    let mut sources: Vec<SourceInput> = Vec::new();

    let ref_segments = stroke_segments(cap, 0)?;
    if ref_segments.is_empty() {
        return Err(format!("step {name:?}: capture holds no strokes"));
    }
    if ref_segments.len() != expected_strokes {
        return Err(format!(
            "step {name:?}: capture holds {} strokes but the stroke plan \
             commanded {expected_strokes}",
            ref_segments.len()
        ));
    }
    if let Some(stops) = stops_pt {
        if stops.len() != expected_strokes {
            return Err(format!(
                "step {name:?}: manifest records {} stops but the stroke \
                 plan commanded {expected_strokes}",
                stops.len()
            ));
        }
    }

    let combined = match belts {
        Some(spec) => Some(crate::combine::compute_corexy_combine(cap, spec, axis)?),
        None => None,
    };
    if let Some(c) = &combined {
        let ranges = tail_ranges(&ref_segments, c.on_ferr.len(), guard, max_len);
        let tails: Vec<Vec<f64>> = ranges
            .iter()
            .map(|r| c.on_ferr[r.clone()].iter().map(|&v| v * 1000.0).collect())
            .collect();
        sources.push(SourceInput {
            source: "combined".to_string(),
            unit: "um".to_string(),
            fs,
            starts_s: ranges.iter().map(|r| r.start as f64 / fs).collect(),
            tails,
            plot_tails: true,
        });
    }
    for (idx, drive) in cap.header.drives.iter().enumerate() {
        if drive.counts_per_mm <= 0.0 {
            return Err(format!(
                "drive {:?} has non-positive counts_per_mm {}",
                drive.name, drive.counts_per_mm
            ));
        }
        let segs = stroke_segments(cap, idx)?;
        if segs.is_empty() {
            continue;
        }
        let ferr = cap.read_i64(idx, "following_error")?;
        let um_per_count = 1000.0 / drive.counts_per_mm;
        let ranges = tail_ranges(&segs, ferr.len(), guard, max_len);
        let tails: Vec<Vec<f64>> = ranges
            .iter()
            .map(|r| {
                ferr[r.clone()]
                    .iter()
                    .map(|&v| v as f64 * um_per_count)
                    .collect()
            })
            .collect();
        sources.push(SourceInput {
            source: drive.name.clone(),
            unit: "um".to_string(),
            fs,
            starts_s: ranges.iter().map(|r| r.start as f64 / fs).collect(),
            tails,
            plot_tails: combined.is_none(),
        });
    }

    if let Some(path) = accel_path {
        let stops = stops_pt.ok_or_else(|| {
            format!(
                "step {name:?} has an accelerometer capture but the manifest \
                 records no stops — re-run with a SERVO_MEASURE_RINGDOWN that \
                 writes them"
            )
        })?;
        let (t, axes) = read_accel_axes(path)?;
        let accel_fs = (t.len() - 1) as f64 / (t[t.len() - 1] - t[0]);
        let mut per_axis_tails = Vec::new();
        for axis_data in &axes {
            let mut tails = accel_tails(&t, axis_data, stops, opts.guard_s, opts.window_s)
                .map_err(|e| format!("step {name:?}: {e}"))?;
            require_min_tail(&tails, accel_fs, "accelerometer")?;
            for tail in &mut tails {
                detrend_linear(tail);
            }
            per_axis_tails.push(tails);
        }
        for (i, tails) in per_axis_tails.into_iter().enumerate() {
            let starts_s: Vec<f64> = stops.iter().map(|&s| s + opts.guard_s - t[0]).collect();
            sources.push(SourceInput {
                source: format!("accel_{}", ["x", "y", "z"][i]),
                unit: "mm/s2".to_string(),
                fs: accel_fs,
                tails,
                starts_s,
                plot_tails: true,
            });
        }
    }

    for input in &sources {
        require_min_tail(&input.tails, input.fs, &input.source)?;
    }

    let mut result_sources = Vec::new();
    let mut plot_sources = Vec::new();
    for input in &sources {
        let (rs, ps) = analyze_source(input, opts.band_hz)?;
        result_sources.push(rs);
        plot_sources.push(ps);
    }
    Ok((
        RingdownResult {
            guard_ms: opts.guard_s * 1000.0,
            window_ms: opts.window_s * 1000.0,
            sources: result_sources,
        },
        PlotRingdown {
            sources: plot_sources,
        },
    ))
}
