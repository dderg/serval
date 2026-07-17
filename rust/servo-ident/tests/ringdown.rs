//! Ring-down analysis: synthetic free-decay accuracy, tail extraction,
//! cross-tail aggregation, and a full ringdown run-directory analyze with a
//! hand-built scap + accelerometer CSV.

use std::path::{Path, PathBuf};

use serde_json::json;

use servo_ident::analyze::{analyze_run, build_run, build_run_incremental};
use servo_ident::results::{RingdownMode, RingdownResult, RingdownSource};
use servo_ident::ringdown::{
    aggregate_modes, analyze_tail, informative_plot_len, ringdown_verdict_reason, tail_ranges,
};

const FS: f64 = 4000.0;

/// White noise via the splitmix64 finalizer — a multiplicative hash alone
/// leaves strong spectral lines that read as fake modes.
fn pseudo_noise(k: usize, scale: f64) -> f64 {
    let mut s = (k as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15) ^ 0xD1B5_4A32_D192_ED03;
    s ^= s >> 30;
    s = s.wrapping_mul(0xBF58_476D_1CE4_E5B9);
    s ^= s >> 27;
    s = s.wrapping_mul(0x94D0_49BB_1331_11EB);
    s ^= s >> 31;
    (f64::from((s >> 32) as u32) / f64::from(u32::MAX) * 2.0 - 1.0) * scale
}

fn decay_tail(n: usize, fs: f64, modes: &[(f64, f64, f64)], noise: f64, seed: usize) -> Vec<f64> {
    (0..n)
        .map(|k| {
            let t = k as f64 / fs;
            let mut v = pseudo_noise(k.wrapping_add(seed), noise);
            for &(freq, zeta, amp) in modes {
                let omega_n = 2.0 * core::f64::consts::PI * freq;
                let sigma = zeta * omega_n;
                let omega_d = omega_n * (1.0 - zeta * zeta).sqrt();
                v += amp * libm::exp(-sigma * t) * libm::cos(omega_d * t + 0.3);
            }
            v
        })
        .collect()
}

fn fits_for(modes: &[(f64, f64, f64)], noise: f64) -> Vec<servo_ident::ringdown::DecayFit> {
    let tail = decay_tail(3800, FS, modes, noise, 0);
    let (fits, _, _) = analyze_tail(&tail, FS, (10.0, 450.0)).unwrap();
    fits
}

fn assert_mode_near(
    fits: &[servo_ident::ringdown::DecayFit],
    freq: f64,
    zeta: f64,
) -> servo_ident::ringdown::DecayFit {
    let fit = fits
        .iter()
        .find(|f| (f.freq_hz - freq).abs() < 0.02 * freq)
        .unwrap_or_else(|| panic!("no fit near {freq} Hz in {fits:?}"));
    assert!(
        (fit.zeta - zeta).abs() < 0.2 * zeta + 0.002,
        "zeta {} vs expected {zeta} at {freq} Hz",
        fit.zeta
    );
    *fit
}

#[test]
fn recovers_a_single_lightly_damped_mode() {
    let fits = fits_for(&[(42.0, 0.03, 500.0)], 2.0);
    assert_mode_near(&fits, 42.0, 0.03);
}

#[test]
fn recovers_two_separated_modes() {
    let fits = fits_for(&[(40.0, 0.03, 500.0), (120.0, 0.06, 250.0)], 2.0);
    assert_mode_near(&fits, 40.0, 0.03);
    assert_mode_near(&fits, 120.0, 0.06);
}

#[test]
fn recovers_very_light_and_heavy_damping() {
    let light = fits_for(&[(60.0, 0.005, 400.0)], 2.0);
    assert_mode_near(&light, 60.0, 0.005);
    let heavy = fits_for(&[(40.0, 0.15, 800.0)], 1.0);
    assert_mode_near(&heavy, 40.0, 0.15);
}

#[test]
fn noise_only_tails_aggregate_to_no_modes() {
    let per_tail: Vec<_> = (0..4)
        .map(|seed| {
            let tail = decay_tail(3800, FS, &[], 5.0, seed * 7919);
            let (fits, _, _) = analyze_tail(&tail, FS, (10.0, 450.0)).unwrap();
            fits
        })
        .collect();
    assert!(
        aggregate_modes(&per_tail, false).is_empty(),
        "pure noise must not produce aggregated modes"
    );
}

#[test]
fn aggregation_requires_two_sightings_across_tails() {
    let ringing = decay_tail(3800, FS, &[(42.0, 0.03, 500.0)], 2.0, 0);
    let quiet = decay_tail(3800, FS, &[], 2.0, 1);
    let fits = |x: &[f64]| analyze_tail(x, FS, (10.0, 450.0)).unwrap().0;
    let one_of_three = vec![fits(&ringing), fits(&quiet), fits(&quiet)];
    assert!(
        aggregate_modes(&one_of_three, false).is_empty(),
        "a single sighting across several tails is noise"
    );
    let two_of_three = vec![fits(&ringing), fits(&ringing), fits(&quiet)];
    let modes = aggregate_modes(&two_of_three, false);
    assert_eq!(modes.len(), 1, "two sightings must survive: {modes:?}");
    assert!((modes[0].freq_hz - 42.0).abs() < 1.0);
    assert_eq!(modes[0].tails, 2);
}

#[test]
fn plot_span_trims_a_fast_ring_but_keeps_a_persistent_one() {
    // Fast decay (τ = 25 ms): quiet long before the 1 s window ends.
    let fast = vec![decay_tail(4000, FS, &[(40.0, 0.1, 500.0)], 1.0, 0)];
    let noise = 0.6;
    let trimmed = informative_plot_len(&fast, FS, noise);
    assert!(
        (400..1600).contains(&trimmed),
        "fast ring should trim to a few hundred samples, got {trimmed}"
    );
    // Barely damped: still ringing at the end — keep the full window.
    let persistent = vec![decay_tail(4000, FS, &[(40.0, 0.001, 500.0)], 1.0, 0)];
    assert_eq!(informative_plot_len(&persistent, FS, noise), 4000);
    // Nothing but noise: the minimum span, not zero.
    let quiet = vec![decay_tail(4000, FS, &[], 1.0, 0)];
    assert_eq!(informative_plot_len(&quiet, FS, noise), 400);
}

#[test]
fn tail_ranges_are_uniform_and_capped() {
    let segs = vec![(100, 200), (900, 1000)];
    let ranges = tail_ranges(&segs, 1500, 40, 600);
    assert_eq!(ranges.len(), 2);
    // First tail: 240..min(240+600, 900) = 240..840 (600 long). Second:
    // 1040..min(1040+600, 1500) = 1040..1500 (460 long) — both crop to 460.
    assert_eq!(ranges[0], 240..700);
    assert_eq!(ranges[1], 1040..1500);
}

fn mode(freq_hz: f64, zeta: f64, disp_um: f64, tails: usize) -> RingdownMode {
    RingdownMode {
        freq_hz,
        zeta,
        zeta_lo: zeta,
        zeta_hi: zeta,
        amp: disp_um,
        disp_um,
        tails,
        cycles: 10.0,
        r2: 0.95,
        fit_start_ms: 20.0,
    }
}

fn source(name: &str, unit: &str, modes: Vec<RingdownMode>) -> RingdownSource {
    RingdownSource {
        source: name.to_string(),
        unit: unit.to_string(),
        tails: 6,
        noise_floor: 1.0,
        modes,
    }
}

#[test]
fn verdict_prefers_the_accelerometer_and_hints_shaper_params() {
    let rr = RingdownResult {
        guard_ms: 10.0,
        window_ms: 950.0,
        sources: vec![
            source("motor_a", "um", vec![mode(41.8, 0.028, 2.0, 6)]),
            source("accel_x", "mm/s2", vec![mode(41.2, 0.031, 0.42, 6)]),
            source("accel_y", "mm/s2", vec![]),
        ],
    };
    let reason = ringdown_verdict_reason(&[&rr]);
    assert!(reason.contains("41.2 Hz"), "{reason}");
    assert!(reason.contains("accel_x"), "{reason}");
    assert!(
        reason.contains("frequency_hz=41.2") && reason.contains("damping_ratio=0.031"),
        "{reason}"
    );
}

#[test]
fn verdict_with_no_modes_says_so() {
    let rr = RingdownResult {
        guard_ms: 10.0,
        window_ms: 950.0,
        sources: vec![source("motor_a", "um", vec![])],
    };
    let reason = ringdown_verdict_reason(&[&rr]);
    assert!(reason.contains("no resonant ring"), "{reason}");
}

// --- end-to-end run directory ---------------------------------------------

const COUNTS_PER_MM: f64 = 3276.8;
const RECORD_SIZE: usize = 23;
const RING_FREQ: f64 = 40.0;
const RING_ZETA: f64 = 0.03;
const STROKE_SAMPLES: usize = 800;
const DWELL_SAMPLES: usize = 4000;
const LEAD_SAMPLES: usize = 2000;
const DISPLACEMENT: i64 = 100_000;

fn scap_header() -> String {
    json!({
        "version": 2,
        "cycle_ns": 250_000,
        "record_size": RECORD_SIZE,
        "drives": [{"name": "motor_a", "counts_per_mm": COUNTS_PER_MM,
                     "rotation_distance": 40.0, "invert": false}],
        "channels": [
            {"name": "cycle_index", "dtype": "u64", "offset": 0},
            {"name": "flags", "dtype": "u8", "offset": 8},
            {"name": "target_counts", "dtype": "i32", "offset": 9},
            {"name": "position_actual", "dtype": "i32", "offset": 13},
            {"name": "following_error", "dtype": "i32", "offset": 17},
            {"name": "torque_actual", "dtype": "i16", "offset": 21}
        ]
    })
    .to_string()
}

fn ring_counts(k_since_stop: usize) -> i64 {
    let t = k_since_stop as f64 / FS;
    let omega_n = 2.0 * core::f64::consts::PI * RING_FREQ;
    let sigma = RING_ZETA * omega_n;
    let omega_d = omega_n * (1.0 - RING_ZETA * RING_ZETA).sqrt();
    (500.0 * libm::exp(-sigma * t) * libm::cos(omega_d * t) + pseudo_noise(k_since_stop, 2.0))
        .round() as i64
}

fn synth_scap(path: &Path) {
    let mut target = Vec::new();
    let mut moving = Vec::new();
    let push_span = |t: &mut Vec<i64>, m: &mut Vec<bool>, n: usize, f: &dyn Fn(usize) -> i64| {
        for k in 0..n {
            t.push(f(k));
            m.push(false);
        }
    };
    push_span(&mut target, &mut moving, LEAD_SAMPLES, &|_| 0);
    for k in 0..STROKE_SAMPLES {
        target.push(DISPLACEMENT * k as i64 / STROKE_SAMPLES as i64);
        moving.push(true);
    }
    push_span(&mut target, &mut moving, DWELL_SAMPLES, &|_| DISPLACEMENT);
    for k in 0..STROKE_SAMPLES {
        target.push(DISPLACEMENT - DISPLACEMENT * k as i64 / STROKE_SAMPLES as i64);
        moving.push(true);
    }
    push_span(&mut target, &mut moving, DWELL_SAMPLES, &|_| 0);

    let n = target.len();
    let mut ferr = vec![0i64; n];
    for k in 0..n {
        if moving[k] {
            ferr[k] = 200 + pseudo_noise(k, 5.0) as i64;
        }
    }
    for stop in [
        LEAD_SAMPLES + STROKE_SAMPLES,
        LEAD_SAMPLES + 2 * STROKE_SAMPLES + DWELL_SAMPLES,
    ] {
        for k in stop..(stop + DWELL_SAMPLES).min(n) {
            ferr[k] = ring_counts(k - stop);
        }
    }

    let mut bytes = scap_header().into_bytes();
    bytes.push(b'\n');
    for k in 0..n {
        let mut rec = [0u8; RECORD_SIZE];
        rec[0..8].copy_from_slice(&(k as u64).to_le_bytes());
        rec[8] = 1 | if moving[k] { 2 } else { 0 };
        rec[9..13].copy_from_slice(&(target[k] as i32).to_le_bytes());
        rec[13..17].copy_from_slice(&((target[k] - ferr[k]) as i32).to_le_bytes());
        rec[17..21].copy_from_slice(&(ferr[k] as i32).to_le_bytes());
        rec[21..23].copy_from_slice(&(100i16).to_le_bytes());
        bytes.extend_from_slice(&rec);
    }
    std::fs::write(path, bytes).unwrap();
}

fn synth_accel_csv(path: &Path, t0: f64, stops: &[f64]) {
    const ACCEL_FS: f64 = 3200.0;
    let total_s = 2.9;
    let mut text = String::from("#time,accel_x,accel_y,accel_z\n");
    let n = (total_s * ACCEL_FS) as usize;
    for k in 0..n {
        let t = t0 + k as f64 / ACCEL_FS;
        let mut ax = pseudo_noise(k, 5.0);
        for &stop in stops {
            if t >= stop {
                let tau = t - stop;
                let omega_n = 2.0 * core::f64::consts::PI * RING_FREQ;
                let sigma = RING_ZETA * omega_n;
                let omega_d = omega_n * (1.0 - RING_ZETA * RING_ZETA).sqrt();
                ax += 3000.0 * libm::exp(-sigma * tau) * libm::cos(omega_d * tau);
            }
        }
        let ay = pseudo_noise(k.wrapping_add(31), 5.0);
        let az = 9810.0 + pseudo_noise(k.wrapping_add(67), 8.0);
        text.push_str(&format!("{t:.6},{ax:.6},{ay:.6},{az:.6}\n"));
    }
    std::fs::write(path, text).unwrap();
}

fn temp_run_dir() -> PathBuf {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let dir =
        std::env::temp_dir().join(format!("servo_cal_ringdown_{}_{nanos}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

fn build_ringdown_run(dir: &Path) {
    synth_scap(&dir.join("step_rd_v250.scap"));
    let t0 = 100.0;
    let stops = [
        t0 + (LEAD_SAMPLES + STROKE_SAMPLES) as f64 / FS,
        t0 + (LEAD_SAMPLES + 2 * STROKE_SAMPLES + DWELL_SAMPLES) as f64 / FS,
    ];
    synth_accel_csv(&dir.join("step_rd_v250_accel.csv"), t0, &stops);
    let manifest = json!({
        "version": 1,
        "experiment": "ringdown",
        "tag": "rd",
        "axis": "X",
        "kinematics": "cartesian",
        "belts": null,
        "stroke_plan": {"center": 110.0, "speed": null,
                         "accel": 30000.0, "iterations": 1, "dwell_ms": 1000,
                         "cruise_ms": 200, "speeds": [250.0]},
        "steps": [{
            "name": "rd_v250",
            "swept": {"speed": 250.0},
            "applied": [],
            "capture": "step_rd_v250.scap",
            "accel": "step_rd_v250_accel.csv",
            "stops": stops
        }]
    });
    std::fs::write(
        dir.join("manifest.json"),
        serde_json::to_string_pretty(&manifest).unwrap(),
    )
    .unwrap();
}

#[test]
fn ringdown_run_dir_recovers_frequency_and_damping() {
    let dir = temp_run_dir();
    build_ringdown_run(&dir);

    let (results, plot) = build_run(&dir).unwrap();
    assert_eq!(results.steps.len(), 1);
    let rd = results.steps[0]
        .ringdown
        .as_ref()
        .expect("ringdown step carries a ringdown result");
    assert_eq!(rd.window_ms, 950.0);

    let by_name = |name: &str| {
        rd.sources
            .iter()
            .find(|s| s.source == name)
            .unwrap_or_else(|| panic!("missing source {name}"))
    };
    for name in ["motor_a", "accel_x"] {
        let src = by_name(name);
        assert_eq!(src.tails, 2);
        let m = src
            .modes
            .iter()
            .max_by(|a, b| a.disp_um.partial_cmp(&b.disp_um).unwrap())
            .unwrap_or_else(|| panic!("{name} found no ring: {:?}", src.modes));
        assert!(
            (m.freq_hz - RING_FREQ).abs() < 0.02 * RING_FREQ,
            "{name} freq {}",
            m.freq_hz
        );
        assert!(
            (m.zeta - RING_ZETA).abs() < 0.2 * RING_ZETA,
            "{name} zeta {}",
            m.zeta
        );
        assert_eq!(m.tails, 2);
    }
    assert!(
        by_name("accel_y").modes.is_empty(),
        "quiet axis must be clean"
    );

    let reason = &results.verdict.reason;
    assert!(reason.contains("accel_x"), "{reason}");
    assert!(reason.contains("shaper hint"), "{reason}");

    let pr = plot.steps[0]
        .ringdown
        .as_ref()
        .expect("plot step carries ringdown series");
    let accel_plot = pr
        .sources
        .iter()
        .find(|s| s.source == "accel_x")
        .expect("accel_x plot source");
    assert_eq!(accel_plot.tails.len(), 2, "both tails plotted");
    assert!(
        !accel_plot.envelope.is_empty(),
        "dominant-mode envelope drawn"
    );
    assert!(!accel_plot.psd_freq_hz.is_empty());
    for s in &pr.sources {
        for tail in &s.tails {
            assert!(tail.t_ms.len() <= 800);
            assert_eq!(tail.t_ms.len(), tail.value.len());
        }
    }

    analyze_run(&dir, false).unwrap();
    let full = std::fs::read_to_string(dir.join("results.json")).unwrap();
    let (results2, _) = build_run_incremental(&dir).unwrap();
    assert_eq!(
        serde_json::to_string_pretty(&results2).unwrap(),
        full,
        "incremental re-analyze must reproduce the full analyze"
    );

    std::fs::remove_dir_all(&dir).ok();
}

#[test]
fn ringdown_stop_count_mismatch_fails_loud() {
    let dir = temp_run_dir();
    build_ringdown_run(&dir);
    let manifest_path = dir.join("manifest.json");
    let mut manifest: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(&manifest_path).unwrap()).unwrap();
    manifest["steps"][0]["stops"] = json!([100.7]);
    std::fs::write(
        &manifest_path,
        serde_json::to_string_pretty(&manifest).unwrap(),
    )
    .unwrap();
    let err = build_run(&dir).unwrap_err();
    assert!(err.contains("1 stops"), "{err}");
    std::fs::remove_dir_all(&dir).ok();
}
