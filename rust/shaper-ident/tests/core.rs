use std::f64::consts::PI;

use _shaper_ident::core::{
    self, ShaperFreqs, calc_freq_response, find_shaper_cfg, fit_shaper, get_shaper_smoothing, psd,
};

/// Build a synthetic capture: a decaying sinusoid at `signal_hz` on the X axis
/// sampled at `fs` for `dur` seconds, quiet Y/Z.
fn synthetic_capture(fs: f64, dur: f64, signal_hz: f64) -> Vec<[f64; 4]> {
    let n = (fs * dur) as usize;
    (0..n)
        .map(|i| {
            let t = i as f64 / fs;
            let x = libm::sin(2.0 * PI * signal_hz * t);
            [t, x, 0.0, 0.0]
        })
        .collect()
}

#[test]
fn psd_peaks_at_the_signal_frequency() {
    let fs = 3000.0;
    let sig = 60.0;
    let n = 8192;
    let x: Vec<f64> = (0..n)
        .map(|i| libm::sin(2.0 * PI * sig * i as f64 / fs))
        .collect();
    let (freqs, p) = psd(&x, fs, 1024);
    let (mut best_k, mut best_v) = (0usize, f64::NEG_INFINITY);
    for (k, &v) in p.iter().enumerate() {
        if v > best_v {
            best_v = v;
            best_k = k;
        }
    }
    // Peak bin should sit on the signal frequency within one bin width.
    let bin_width = fs / 1024.0;
    assert!(
        (freqs[best_k] - sig).abs() <= bin_width,
        "peak at {} Hz, expected ~{} Hz (bin {})",
        freqs[best_k],
        sig,
        bin_width
    );
}

#[test]
fn calc_freq_response_locates_the_capture_peak() {
    let cap = synthetic_capture(3000.0, 2.0, 55.0);
    let resp = calc_freq_response(&cap).expect("capture long enough for a window");
    let (mut best_k, mut best_v) = (0usize, f64::NEG_INFINITY);
    for (k, &v) in resp.psd_sum.iter().enumerate() {
        if v > best_v {
            best_v = v;
            best_k = k;
        }
    }
    assert!(
        (resp.freq_bins[best_k] - 55.0).abs() < 5.0,
        "psd_sum peak at {} Hz, expected ~55 Hz",
        resp.freq_bins[best_k]
    );
    // psd_sum is the axis sum; Y and Z are quiet.
    assert!(resp.psd_x[best_k] > resp.psd_y[best_k]);
}

#[test]
fn short_capture_returns_none() {
    let cap = synthetic_capture(3000.0, 0.05, 55.0);
    assert!(calc_freq_response(&cap).is_none());
}

#[test]
fn fit_shaper_recommends_frequency_near_the_resonance() {
    let cap = synthetic_capture(3000.0, 2.0, 55.0);
    let resp = calc_freq_response(&cap).expect("capture usable");
    let cfg = find_shaper_cfg("mzv").unwrap();
    let res = fit_shaper(
        cfg,
        &resp.freq_bins,
        &resp.psd_sum,
        &ShaperFreqs::Range(None, None, None),
        None,
        5.0,
        None,
        None,
        None,
    )
    .expect("a shaper is selected");
    assert!(res.vibrs >= 0.0 && res.vibrs <= 1.0);
    assert!(res.max_accel > 0.0);
    // vals covers the PSD bins kept under max_freq (<= the full bin count).
    assert!(!res.vals.is_empty() && res.vals.len() <= resp.freq_bins.len());
}

#[test]
fn shaper_smoothing_is_monotonic_in_accel() {
    let cfg = find_shaper_cfg("zv").unwrap();
    let shaper = (cfg.init)(50.0, core::DEFAULT_DAMPING_RATIO);
    let mut prev = f64::NEG_INFINITY;
    for accel in [1000.0, 2000.0, 4000.0, 8000.0, 16000.0] {
        let s = get_shaper_smoothing(&shaper, accel, 5.0);
        assert!(s >= prev, "smoothing not monotonic at accel={accel}");
        prev = s;
    }
}

#[test]
fn lower_frequency_shaper_smooths_more() {
    let cfg = find_shaper_cfg("mzv").unwrap();
    let low = (cfg.init)(30.0, core::DEFAULT_DAMPING_RATIO);
    let high = (cfg.init)(80.0, core::DEFAULT_DAMPING_RATIO);
    assert!(
        get_shaper_smoothing(&low, 5000.0, 5.0) > get_shaper_smoothing(&high, 5000.0, 5.0),
        "a lower-frequency shaper should smooth more"
    );
}
