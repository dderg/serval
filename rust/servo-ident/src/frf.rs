//! Differential belt-pair FRF, ported from `scripts/servo_diff_report.py`:
//! the H1 Welch estimate with coherence (Hann window, half overlap,
//! mean-removed segments), half-power damping, and coherent mode picking.
//! The estimator conventions must match the Python so mode selection is
//! comparable within fp tolerance.

use core::f64::consts::SQRT_2;
use core::ops::Range;

use crate::psd::{fft_pow2, hanning};
use crate::results::DifferentialMode;
use crate::scap::Scap;

pub const MIN_NPERSEG: usize = 256;
pub const MIN_SEGMENTS: usize = 4;
pub const COHERENCE_MIN: f64 = 0.5;
pub const MAX_MODES: usize = 5;
pub const DEFAULT_NPERSEG: usize = 4096;

#[derive(Debug)]
pub struct DifferentialSeries {
    pub pair: Vec<String>,
    pub cmd_mm: Vec<f64>,
    pub act_mm: Vec<f64>,
    pub torque: Vec<f64>,
}

pub fn differential_series(cap: &Scap) -> Result<DifferentialSeries, String> {
    let drives = &cap.header.drives;
    if drives.len() != 2 {
        return Err(format!(
            "differential capture must hold exactly the two pair drives, got {}: [{}]",
            drives.len(),
            cap.drive_names().join(", ")
        ));
    }
    let mut per_drive = Vec::with_capacity(2);
    for (idx, d) in drives.iter().enumerate() {
        if d.counts_per_mm <= 0.0 {
            return Err(format!(
                "drive {:?} has non-positive counts_per_mm {}",
                d.name, d.counts_per_mm
            ));
        }
        let sign = if d.invert { -1.0 } else { 1.0 };
        let scale = sign / d.counts_per_mm;
        let cmd_mm: Vec<f64> = cap
            .read_f64(idx, "target_counts")?
            .iter()
            .map(|v| v * scale)
            .collect();
        let act_mm: Vec<f64> = cap
            .read_f64(idx, "position_actual")?
            .iter()
            .map(|v| v * scale)
            .collect();
        let torque: Vec<f64> = cap
            .read_f64(idx, "torque_actual")?
            .iter()
            .map(|v| v * sign)
            .collect();
        per_drive.push((cmd_mm, act_mm, torque));
    }
    let (b_cmd, b_act, b_torque) = per_drive.pop().unwrap();
    let (a_cmd, a_act, a_torque) = per_drive.pop().unwrap();
    let sub = |a: &[f64], b: &[f64]| a.iter().zip(b).map(|(&x, &y)| x - y).collect();
    Ok(DifferentialSeries {
        pair: cap.drive_names(),
        cmd_mm: sub(&a_cmd, &b_cmd),
        act_mm: sub(&a_act, &b_act),
        torque: sub(&a_torque, &b_torque),
    })
}

fn median(values: &[f64]) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    let mut sorted = values.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).expect("differential command holds a NaN"));
    let mid = sorted.len() / 2;
    if sorted.len() % 2 == 1 {
        sorted[mid]
    } else {
        0.5 * (sorted[mid - 1] + sorted[mid])
    }
}

pub fn active_slice(cmd: &[f64]) -> Result<Range<usize>, String> {
    let center = median(cmd);
    let dev: Vec<f64> = cmd.iter().map(|&v| (v - center).abs()).collect();
    let peak = dev.iter().fold(0.0_f64, |m, &v| m.max(v));
    if peak <= 0.0 {
        return Err(
            "capture holds no differential excitation (differential command \
             is flat); was the anti-phase buzz armed on this pair?"
                .to_string(),
        );
    }
    let threshold = 0.05 * peak;
    let first = dev.iter().position(|&d| d > threshold).unwrap();
    let last = dev.iter().rposition(|&d| d > threshold).unwrap();
    Ok(first..last + 1)
}

fn welch_segment_length(n: usize, mut nperseg: usize) -> Result<usize, String> {
    if !nperseg.is_power_of_two() {
        return Err(format!("nperseg {nperseg} is not a power of two"));
    }
    while nperseg * (MIN_SEGMENTS + 1) / 2 > n && nperseg > MIN_NPERSEG {
        nperseg /= 2;
    }
    if nperseg < MIN_NPERSEG || n < nperseg * (MIN_SEGMENTS + 1) / 2 {
        return Err(format!(
            "capture too short for a Welch FRF: {n} active samples but \
             {MIN_SEGMENTS} segments of {MIN_NPERSEG} are needed; sweep longer or slower"
        ));
    }
    Ok(nperseg)
}

#[derive(Debug)]
pub struct Frf {
    pub freqs: Vec<f64>,
    pub re: Vec<f64>,
    pub im: Vec<f64>,
    pub coherence: Vec<f64>,
    pub segments: usize,
}

impl Frf {
    pub fn magnitude(&self) -> Vec<f64> {
        self.re
            .iter()
            .zip(&self.im)
            .map(|(&r, &i)| libm::hypot(r, i))
            .collect()
    }
}

fn windowed_rfft(seg: &[f64], win: &[f64], bins: usize) -> (Vec<f64>, Vec<f64>) {
    let mean = seg.iter().sum::<f64>() / seg.len() as f64;
    let mut re: Vec<f64> = seg.iter().zip(win).map(|(&v, &w)| (v - mean) * w).collect();
    let mut im = vec![0.0; seg.len()];
    fft_pow2(&mut re, &mut im);
    re.truncate(bins);
    im.truncate(bins);
    (re, im)
}

pub fn welch_frf(x: &[f64], y: &[f64], fs: f64, nperseg: usize) -> Result<Frf, String> {
    if x.len() != y.len() {
        return Err(format!(
            "excitation and response lengths differ ({} vs {})",
            x.len(),
            y.len()
        ));
    }
    let nperseg = welch_segment_length(x.len(), nperseg)?;
    let step = nperseg / 2;
    let win = hanning(nperseg);
    let bins = nperseg / 2 + 1;
    let mut pxx = vec![0.0; bins];
    let mut pyy = vec![0.0; bins];
    let mut pxy_re = vec![0.0; bins];
    let mut pxy_im = vec![0.0; bins];
    let mut segments = 0usize;
    let mut start = 0usize;
    while start + nperseg <= x.len() {
        let (fx_re, fx_im) = windowed_rfft(&x[start..start + nperseg], &win, bins);
        let (fy_re, fy_im) = windowed_rfft(&y[start..start + nperseg], &win, bins);
        for b in 0..bins {
            pxx[b] += fx_re[b] * fx_re[b] + fx_im[b] * fx_im[b];
            pyy[b] += fy_re[b] * fy_re[b] + fy_im[b] * fy_im[b];
            pxy_re[b] += fx_re[b] * fy_re[b] + fx_im[b] * fy_im[b];
            pxy_im[b] += fx_re[b] * fy_im[b] - fx_im[b] * fy_re[b];
        }
        segments += 1;
        start += step;
    }
    let freqs: Vec<f64> = (0..bins).map(|b| b as f64 * fs / nperseg as f64).collect();
    let mut re = vec![0.0; bins];
    let mut im = vec![0.0; bins];
    let mut coherence = vec![0.0; bins];
    for b in 0..bins {
        if pxx[b] > 0.0 {
            re[b] = pxy_re[b] / pxx[b];
            im[b] = pxy_im[b] / pxx[b];
        }
        let denom = pxx[b] * pyy[b];
        if denom > 0.0 {
            coherence[b] = (pxy_re[b] * pxy_re[b] + pxy_im[b] * pxy_im[b]) / denom;
        }
    }
    Ok(Frf {
        freqs,
        re,
        im,
        coherence,
        segments,
    })
}

fn half_power_crossing(
    freqs: &[f64],
    mag: &[f64],
    i: usize,
    target: f64,
    direction: isize,
) -> Option<f64> {
    let n = mag.len() as isize;
    let mut j = i as isize;
    while 0 < j && j < n - 1 && mag[j as usize] > target {
        j += direction;
    }
    if mag[j as usize] > target {
        return None;
    }
    let prev = (j - direction) as usize;
    let (f0, f1) = (freqs[prev], freqs[j as usize]);
    let (m0, m1) = (mag[prev], mag[j as usize]);
    if m0 == m1 {
        return Some(f1);
    }
    Some(f0 + (f1 - f0) * (m0 - target) / (m0 - m1))
}

pub fn half_power_damping(freqs: &[f64], mag: &[f64], i_peak: usize) -> Option<f64> {
    let target = mag[i_peak] / SQRT_2;
    let lo = half_power_crossing(freqs, mag, i_peak, target, -1)?;
    let hi = half_power_crossing(freqs, mag, i_peak, target, 1)?;
    if freqs[i_peak] <= 0.0 {
        return None;
    }
    Some((hi - lo) / (2.0 * freqs[i_peak]))
}

pub fn find_modes(frf: &Frf, lo: f64, hi: f64) -> Result<Vec<DifferentialMode>, String> {
    let band: Vec<usize> = (0..frf.freqs.len())
        .filter(|&i| frf.freqs[i] >= lo && frf.freqs[i] <= hi)
        .collect();
    if !band.iter().any(|&i| frf.coherence[i] >= COHERENCE_MIN) {
        let max_coh = band
            .iter()
            .map(|&i| frf.coherence[i])
            .fold(0.0_f64, f64::max);
        return Err(format!(
            "no coherent differential response in {lo:.0}..{hi:.0} Hz \
             (max coherence {max_coh:.2}); raise AMPLITUDE or check that the \
             buzz really ran anti-phase on this pair"
        ));
    }
    let mag = frf.magnitude();
    let mut candidates: Vec<usize> = Vec::new();
    if band.len() > 2 {
        for &i in &band[1..band.len() - 1] {
            if mag[i] > mag[i - 1] && mag[i] >= mag[i + 1] && frf.coherence[i] >= COHERENCE_MIN {
                candidates.push(i);
            }
        }
    }
    candidates.sort_by(|&a, &b| {
        mag[b]
            .partial_cmp(&mag[a])
            .unwrap_or(core::cmp::Ordering::Equal)
    });
    let mut modes: Vec<DifferentialMode> = Vec::new();
    for i in candidates {
        let near_existing = modes
            .iter()
            .any(|m| (frf.freqs[i] - m.freq_hz).abs() < 3.0_f64.max(0.05 * m.freq_hz));
        if near_existing {
            continue;
        }
        modes.push(DifferentialMode {
            freq_hz: frf.freqs[i],
            gain: mag[i],
            gain_db: 20.0 * libm::log10(mag[i]),
            damping: half_power_damping(&frf.freqs, &mag, i),
            coherence: frf.coherence[i],
        });
        if modes.len() >= MAX_MODES {
            break;
        }
    }
    modes.sort_by(|a, b| a.freq_hz.partial_cmp(&b.freq_hz).unwrap());
    Ok(modes)
}
