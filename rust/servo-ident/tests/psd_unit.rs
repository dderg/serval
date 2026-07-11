use core::f64::consts::PI;

use servo_ident::psd::{top_peaks, welch_psd};
use servo_ident::resonance::{detect_resonance, recommend_accel};

#[test]
fn welch_peak_lands_on_the_tone() {
    let fs = 1000.0;
    let f0 = 50.0;
    let x: Vec<f64> = (0..4096)
        .map(|k| libm::sin(2.0 * PI * f0 * k as f64 / fs))
        .collect();
    let (freqs, psd) = welch_psd(&x, fs).unwrap();
    let peaks = top_peaks(&freqs, &psd, 5);
    assert!(
        (peaks[0].0 - f0).abs() < 2.0,
        "top peak at {} Hz",
        peaks[0].0
    );
    // powers are sorted descending
    for w in peaks.windows(2) {
        assert!(w[0].1 >= w[1].1);
    }
}

#[test]
fn welch_rejects_short_input() {
    assert!(welch_psd(&[0.0; 40], 1000.0).is_err());
}

#[test]
fn resonance_ratio_flags_a_spike() {
    let freqs: Vec<f64> = (0..500).map(|i| i as f64).collect();
    let mut psd = vec![0.0; 500];
    for p in psd.iter_mut().take(4).skip(1) {
        *p = 1.0; // 1-4 Hz low band mean = 1.0
    }
    psd[100] = 100.0; // spike in the 20-450 Hz band
    let r = detect_resonance(&freqs, &psd);
    assert!(r.detected);
    assert_eq!(r.peak_hz, 100.0);
    assert!((r.ratio - 100.0).abs() < 1e-9);
}

#[test]
fn resonance_clear_when_flat() {
    let freqs: Vec<f64> = (0..500).map(|i| i as f64).collect();
    let mut psd = vec![1.0; 500];
    psd[100] = 3.0;
    let r = detect_resonance(&freqs, &psd);
    assert!(!r.detected);
}

#[test]
fn recommend_accel_picks_highest_clean() {
    let (chosen, note) = recommend_accel(&[(10000.0, false), (20000.0, false), (30000.0, true)]);
    assert_eq!(chosen, Some(20000.0));
    assert!(note.contains("30000"), "{note}");
}

#[test]
fn recommend_accel_none_when_all_rail() {
    let (chosen, _) = recommend_accel(&[(10000.0, true), (20000.0, true)]);
    assert_eq!(chosen, None);
}
