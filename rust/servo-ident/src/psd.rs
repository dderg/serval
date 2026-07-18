//! Welch PSD, moving-segment PSD and strict-local-maximum peak picking,
//! ported from `scripts/servo_capture.py` (`welch_psd`, `moving_psd`,
//! `top_peaks`). The windowing, detrend, scaling and bin-doubling must match
//! the Python so peak selection is bit-comparable within fp tolerance.

use core::f64::consts::PI;

use crate::metrics::DriveSeries;

fn largest_pow2_le(n: usize) -> usize {
    let mut p = 1;
    while p * 2 <= n {
        p *= 2;
    }
    p
}

pub(crate) fn hanning(n: usize) -> Vec<f64> {
    if n == 1 {
        return vec![1.0];
    }
    (0..n)
        .map(|i| 0.5 - 0.5 * libm::cos(2.0 * PI * i as f64 / (n - 1) as f64))
        .collect()
}

pub(crate) fn fft_pow2(re: &mut [f64], im: &mut [f64]) {
    let n = re.len();
    let mut j = 0usize;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j |= bit;
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
    }
    let mut len = 2usize;
    while len <= n {
        let half = len / 2;
        let mut i = 0usize;
        while i < n {
            for k in 0..half {
                let ang = -2.0 * PI * k as f64 / len as f64;
                let (wr, wi) = (libm::cos(ang), libm::sin(ang));
                let ur = re[i + k];
                let ui = im[i + k];
                let xr = re[i + k + half];
                let xi = im[i + k + half];
                let vr = xr * wr - xi * wi;
                let vi = xr * wi + xi * wr;
                re[i + k] = ur + vr;
                im[i + k] = ui + vi;
                re[i + k + half] = ur - vr;
                im[i + k + half] = ui - vi;
            }
            i += len;
        }
        len <<= 1;
    }
}

/// Averaged one-sided PSD over half-overlapping Hann segments. Returns
/// `(freqs, psd)` with `nperseg/2 + 1` bins. `nperseg` is the largest power
/// of two not exceeding `min(1024, x.len())`.
pub fn welch_psd(x: &[f64], fs: f64) -> Result<(Vec<f64>, Vec<f64>), String> {
    let nperseg = largest_pow2_le(x.len().min(1024));
    if nperseg < 64 {
        return Err(format!(
            "segment too short for PSD ({} samples; need >= 64)",
            x.len()
        ));
    }
    let step = nperseg / 2;
    let win = hanning(nperseg);
    let win_sq_sum: f64 = win.iter().map(|&w| w * w).sum();
    let scale = 1.0 / (fs * win_sq_sum);
    let bins = nperseg / 2 + 1;
    let mut acc = vec![0.0f64; bins];
    let mut count = 0usize;
    let mut start = 0usize;
    while start + nperseg <= x.len() {
        let seg = &x[start..start + nperseg];
        let mean = seg.iter().sum::<f64>() / nperseg as f64;
        let mut re: Vec<f64> = seg
            .iter()
            .zip(&win)
            .map(|(&v, &w)| (v - mean) * w)
            .collect();
        let mut im = vec![0.0f64; nperseg];
        fft_pow2(&mut re, &mut im);
        for b in 0..bins {
            acc[b] += (re[b] * re[b] + im[b] * im[b]) * scale;
        }
        count += 1;
        start += step;
    }
    let mut psd: Vec<f64> = acc.iter().map(|&a| a / count as f64).collect();
    for p in psd.iter_mut().take(bins - 1).skip(1) {
        *p *= 2.0;
    }
    let freqs: Vec<f64> = (0..bins).map(|i| i as f64 * fs / nperseg as f64).collect();
    Ok((freqs, psd))
}

pub fn segments_welch_psd(
    x: &[f64],
    segs: &[(usize, usize)],
    fs: f64,
) -> Result<(Vec<f64>, Vec<f64>), String> {
    if segs.is_empty() {
        return Err("no moving segments in capture — nothing to analyze".to_string());
    }
    let mut moving = Vec::new();
    for &(s, e) in segs {
        moving.extend_from_slice(&x[s..e]);
    }
    welch_psd(&moving, fs)
}

pub fn moving_psd(
    d: &DriveSeries,
    segs: &[(usize, usize)],
    fs: f64,
) -> Result<(Vec<f64>, Vec<f64>), String> {
    segments_welch_psd(&d.following_error, segs, fs)
}

pub fn top_peaks(freqs: &[f64], psd: &[f64], count: usize) -> Vec<(f64, f64)> {
    if psd.len() < 3 {
        return Vec::new();
    }
    let mut local: Vec<usize> = (1..psd.len() - 1)
        .filter(|&i| psd[i] > psd[i - 1] && psd[i] > psd[i + 1])
        .collect();
    local.sort_by(|&a, &b| {
        psd[b]
            .partial_cmp(&psd[a])
            .unwrap_or(core::cmp::Ordering::Equal)
    });
    local
        .into_iter()
        .take(count)
        .map(|i| (freqs[i], psd[i]))
        .collect()
}
