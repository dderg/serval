use std::f64::consts::PI;

use _shaper_ident::core::{
    self, ShaperFreqs, calc_freq_response, estimate_smoother, find_shaper_cfg, find_smoother_cfg,
    fit_shaper, fit_smoother, get_shaper_smoothing, get_smoother_smoothing, psd, smoother_moments,
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

#[test]
fn smoother_response_is_unity_at_dc_and_notched_at_its_frequency() {
    for cfg in [
        find_smoother_cfg("smooth_zv").unwrap(),
        find_smoother_cfg("smooth_mzv").unwrap(),
    ] {
        let kernel = (cfg.build)(1.0);
        let vals = estimate_smoother(&kernel, 0.1, &[0.0, 1.0]);
        assert!(
            (vals[0] - 1.0).abs() < 1e-3,
            "{}: DC response {} should be ~1",
            cfg.name,
            vals[0]
        );
        assert!(
            vals[1] < 0.06,
            "{}: residual {} at the target frequency should be deeply notched",
            cfg.name,
            vals[1]
        );
    }
}

#[test]
fn fit_smoother_recommends_frequency_near_the_resonance() {
    let cap = synthetic_capture(3000.0, 2.0, 55.0);
    let resp = calc_freq_response(&cap).expect("capture usable");
    let cfg = find_smoother_cfg("smooth_mzv").unwrap();
    let res = fit_smoother(
        cfg,
        &resp.freq_bins,
        &resp.psd_sum,
        &ShaperFreqs::Range(None, None, None),
        5.0,
        None,
        None,
        None,
    )
    .expect("a smoother is selected");
    assert_eq!(res.name, "smooth_mzv");
    assert!(res.vibrs >= 0.0 && res.vibrs <= 1.0);
    assert!(res.max_accel > 0.0);
    assert!(res.smoothing > 0.0);
    assert!(!res.vals.is_empty() && res.vals.len() <= resp.freq_bins.len());
    // ±20% around the excited 55 Hz resonance: catches a scaling bug in the
    // normalized-response lookup, which would land the notch far away.
    assert!(
        (44.0..66.0).contains(&res.freq),
        "selected frequency {} not near the 55 Hz resonance",
        res.freq
    );
}

#[test]
fn smoother_smoothing_is_monotonic_in_accel() {
    let cfg = find_smoother_cfg("smooth_zv").unwrap();
    let moments = smoother_moments(&(cfg.build)(1.0));
    let mut prev = f64::NEG_INFINITY;
    for accel in [1000.0, 2000.0, 4000.0, 8000.0, 16000.0] {
        let s = get_smoother_smoothing(&moments, 1.0 / 50.0, accel, 5.0);
        assert!(s >= prev, "smoothing not monotonic at accel={accel}");
        prev = s;
    }
}

#[test]
fn lower_frequency_smoother_smooths_more() {
    let cfg = find_smoother_cfg("smooth_mzv").unwrap();
    let moments = smoother_moments(&(cfg.build)(1.0));
    assert!(
        get_smoother_smoothing(&moments, 1.0 / 30.0, 5000.0, 5.0)
            > get_smoother_smoothing(&moments, 1.0 / 80.0, 5000.0, 5.0),
        "a lower-frequency smoother should smooth more"
    );
}
