use core::f64::consts::PI;

use servo_ident::analyze::{accel_axis_psds, AccelSamples};

const FS: f64 = 1000.0;
const N: usize = 4096;
const TONE_HZ: f64 = 50.0;

fn tone_samples() -> AccelSamples {
    let t: Vec<f64> = (0..N).map(|k| k as f64 / FS).collect();
    let x: Vec<f64> = t
        .iter()
        .map(|&tv| 3.0 * libm::sin(2.0 * PI * TONE_HZ * tv))
        .collect();
    let y = vec![0.0; N];
    let z: Vec<f64> = t
        .iter()
        .map(|&tv| 1.0 * libm::sin(2.0 * PI * 2.0 * TONE_HZ * tv))
        .collect();
    AccelSamples { t, axes: [x, y, z] }
}

#[test]
fn total_psd_is_the_sum_of_per_axis_psds() {
    let psds = accel_axis_psds(&tone_samples()).unwrap();
    for b in 0..psds.freq_hz.len() {
        let sum: f64 = psds.per_axis.iter().map(|p| p[b]).sum();
        assert!(
            (psds.total[b] - sum).abs() <= 1e-12 * sum.max(1.0),
            "bin {b}: total {} != sum {sum}",
            psds.total[b]
        );
    }
}

#[test]
fn per_axis_psds_peak_at_their_tones() {
    let psds = accel_axis_psds(&tone_samples()).unwrap();
    let peak_bin = |p: &[f64]| {
        (0..p.len())
            .max_by(|&a, &b| p[a].partial_cmp(&p[b]).unwrap())
            .unwrap()
    };
    let x_peak_hz = psds.freq_hz[peak_bin(&psds.per_axis[0])];
    let z_peak_hz = psds.freq_hz[peak_bin(&psds.per_axis[2])];
    assert!((x_peak_hz - TONE_HZ).abs() < 2.0, "x peak at {x_peak_hz} Hz");
    assert!(
        (z_peak_hz - 2.0 * TONE_HZ).abs() < 2.0,
        "z peak at {z_peak_hz} Hz"
    );
    let y_max = psds.per_axis[1].iter().cloned().fold(0.0f64, f64::max);
    let x_max = psds.per_axis[0].iter().cloned().fold(0.0f64, f64::max);
    assert!(y_max < 1e-9 * x_max, "silent axis carries power: {y_max}");
    let total_peak_hz = psds.freq_hz[peak_bin(&psds.total)];
    assert!(
        (total_peak_hz - TONE_HZ).abs() < 2.0,
        "total peak at {total_peak_hz} Hz"
    );
}
