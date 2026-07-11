//! Ports of `test/test_servo_diff_report.py`: the H1 Welch FRF estimator,
//! active-slice trimming, coherent-mode picking, loud failures on unusable
//! captures, and the end-to-end differential run through `build_run`.

use core::f64::consts::PI;
use std::path::{Path, PathBuf};

use serde_json::{json, Value};

use servo_ident::analyze::{analyze_differential_capture, build_run};
use servo_ident::frf::{active_slice, differential_series, find_modes, welch_frf, MIN_SEGMENTS};
use servo_ident::scap::Scap;

const FS: f64 = 4000.0;
const MODE_HZ: f64 = 92.0;
const MODE_ZETA: f64 = 0.05;
const COUNTS_PER_MM: f64 = 100000.0;

fn chirp(f0: f64, f1: f64, seconds: f64, fs: f64) -> Vec<f64> {
    let n = (seconds * fs) as usize;
    (0..n)
        .map(|k| {
            let t = k as f64 / fs;
            libm::sin(2.0 * PI * (f0 * t + (f1 - f0) / (2.0 * seconds) * t * t))
        })
        .collect()
}

fn resonant_response(x: &[f64], fs: f64, f0: f64, zeta: f64) -> Vec<f64> {
    let k = libm::tan(PI * f0 / fs);
    let a0 = 1.0 + 2.0 * zeta * k + k * k;
    let b = k * k / a0;
    let a1 = 2.0 * (k * k - 1.0) / a0;
    let a2 = (1.0 - 2.0 * zeta * k + k * k) / a0;
    let mut y = vec![0.0; x.len()];
    for n in 0..x.len() {
        let at = |v: &[f64], back: usize| if n >= back { v[n - back] } else { 0.0 };
        y[n] = b * (x[n] + 2.0 * at(x, 1) + at(x, 2)) - a1 * at(&y, 1) - a2 * at(&y, 2);
    }
    y
}

fn noise(n: usize, mut state: u64) -> Vec<f64> {
    (0..n)
        .map(|_| {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            (state >> 11) as f64 / (1u64 << 53) as f64 - 0.5
        })
        .collect()
}

#[test]
fn welch_frf_recovers_resonance_frequency_and_damping() {
    let x = chirp(30.0, 150.0, 24.0, FS);
    let y = resonant_response(&x, FS, MODE_HZ, MODE_ZETA);
    let frf = welch_frf(&x, &y, FS, 4096).unwrap();
    assert!(frf.segments >= MIN_SEGMENTS);
    let modes = find_modes(&frf, 30.0, 150.0).unwrap();
    assert_eq!(modes.len(), 1, "modes: {modes:?}");
    let mode = &modes[0];
    assert!(
        (mode.freq_hz - MODE_HZ).abs() < 2.0,
        "freq {}",
        mode.freq_hz
    );
    let expected_gain = 1.0 / (2.0 * MODE_ZETA);
    assert!(
        (mode.gain - expected_gain).abs() < 0.25 * expected_gain,
        "gain {}",
        mode.gain
    );
    let damping = mode.damping.expect("half-power damping");
    assert!(
        (damping - MODE_ZETA).abs() < 0.5 * MODE_ZETA,
        "damping {damping}"
    );
    assert!(mode.coherence > 0.9, "coherence {}", mode.coherence);
}

#[test]
fn find_modes_fails_loudly_without_coherent_response() {
    let x = chirp(30.0, 150.0, 12.0, FS);
    let y = noise(x.len(), 0x9e3779b97f4a7c15);
    let frf = welch_frf(&x, &y, FS, 4096).unwrap();
    let err = find_modes(&frf, 30.0, 150.0).unwrap_err();
    assert!(err.contains("no coherent"), "{err}");
}

#[test]
fn active_slice_trims_quiet_head_and_tail() {
    let mut cmd = vec![0.0; 10000];
    for (i, c) in cmd[4000..6000].iter_mut().enumerate() {
        *c = libm::sin(60.0 * i as f64 / 1999.0);
    }
    let span = active_slice(&cmd).unwrap();
    assert!((3900..=4100).contains(&span.start), "start {}", span.start);
    assert!((5900..=6100).contains(&span.end), "end {}", span.end);
}

#[test]
fn active_slice_fails_loudly_on_flat_command() {
    let err = active_slice(&vec![0.0; 1000]).unwrap_err();
    assert!(err.contains("no differential excitation"), "{err}");
}

#[test]
fn welch_rejects_captures_too_short_for_segments() {
    let err = welch_frf(&[1.0; 300], &[1.0; 300], FS, 4096).unwrap_err();
    assert!(err.contains("too short"), "{err}");
}

struct DriveFixture {
    name: &'static str,
    invert: bool,
    target: Vec<i64>,
    position: Vec<i64>,
}

fn scap_bytes(drives: &[DriveFixture]) -> Vec<u8> {
    let n = drives[0].target.len();
    let block = 14usize;
    let record_size = 9 + drives.len() * block;
    let header = json!({
        "version": 2,
        "cycle_ns": (1e9 / FS) as u64,
        "record_size": record_size,
        "drives": drives.iter().map(|d| json!({
            "name": d.name,
            "counts_per_mm": COUNTS_PER_MM,
            "rotation_distance": 40.0,
            "invert": d.invert,
        })).collect::<Vec<_>>(),
        "channels": [
            {"name": "cycle_index", "dtype": "u64", "offset": 0},
            {"name": "flags", "dtype": "u8", "offset": 8},
            {"name": "target_counts", "dtype": "i32", "offset": 9},
            {"name": "position_actual", "dtype": "i32", "offset": 13},
            {"name": "torque_actual", "dtype": "i16", "offset": 17},
            {"name": "following_error", "dtype": "i32", "offset": 19},
        ],
    });
    let mut bytes = serde_json::to_vec(&header).unwrap();
    bytes.push(b'\n');
    for k in 0..n {
        let mut rec = vec![0u8; record_size];
        rec[..8].copy_from_slice(&(k as u64).to_le_bytes());
        for (di, d) in drives.iter().enumerate() {
            let base = 9 + di * block;
            let t = d.target[k] as i32;
            let p = d.position[k] as i32;
            rec[base..base + 4].copy_from_slice(&t.to_le_bytes());
            rec[base + 4..base + 8].copy_from_slice(&p.to_le_bytes());
            rec[base + 10..base + 14].copy_from_slice(&(t - p).to_le_bytes());
        }
        bytes.extend_from_slice(&rec);
    }
    bytes
}

fn synth_pair_bytes(invert_second: bool) -> Vec<u8> {
    let cmd_mm: Vec<f64> = chirp(30.0, 150.0, 24.0, FS)
        .into_iter()
        .map(|v| 0.05 * v)
        .collect();
    let excitation: Vec<f64> = cmd_mm.iter().map(|&v| 2.0 * v).collect();
    let diff_act_mm = resonant_response(&excitation, FS, MODE_HZ, MODE_ZETA);
    let drive_sign = if invert_second { -1.0 } else { 1.0 };
    let counts = |v: f64| (v * COUNTS_PER_MM).round() as i64;
    let n = cmd_mm.len();
    scap_bytes(&[
        DriveFixture {
            name: "motor_a",
            invert: false,
            target: (0..n).map(|k| counts(cmd_mm[k])).collect(),
            position: (0..n).map(|k| counts(0.5 * diff_act_mm[k])).collect(),
        },
        DriveFixture {
            name: "motor_a1",
            invert: invert_second,
            target: (0..n).map(|k| counts(drive_sign * -cmd_mm[k])).collect(),
            position: (0..n)
                .map(|k| counts(drive_sign * -0.5 * diff_act_mm[k]))
                .collect(),
        },
    ])
}

#[test]
fn differential_series_requires_exactly_two_drives() {
    let one = scap_bytes(&[DriveFixture {
        name: "motor_a",
        invert: false,
        target: vec![0; 16],
        position: vec![0; 16],
    }]);
    let cap = Scap::from_bytes(&one).unwrap();
    let err = differential_series(&cap).unwrap_err();
    assert!(err.contains("exactly the two pair drives"), "{err}");
}

#[test]
fn analyze_finds_the_mode_in_a_synthetic_capture() {
    let cap = Scap::from_bytes(&synth_pair_bytes(false)).unwrap();
    let (sr, _) = analyze_differential_capture(&cap, "buzz", 30.0, 150.0).unwrap();
    let d = sr.differential.expect("differential result");
    assert_eq!(d.pair, vec!["motor_a", "motor_a1"]);
    assert_eq!(d.modes.len(), 1, "modes: {:?}", d.modes);
    assert!((d.modes[0].freq_hz - MODE_HZ).abs() < 2.0);
}

#[test]
fn analyze_applies_header_invert_sign() {
    let cap = Scap::from_bytes(&synth_pair_bytes(true)).unwrap();
    let (sr, _) = analyze_differential_capture(&cap, "buzz", 30.0, 150.0).unwrap();
    let d = sr.differential.expect("differential result");
    assert!((d.modes[0].freq_hz - MODE_HZ).abs() < 2.0);
}

fn temp_run_dir() -> PathBuf {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let dir = std::env::temp_dir().join(format!("servo_diff_run_{}_{nanos}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

fn write_manifest(dir: &Path, stroke_plan: Value) {
    std::fs::write(dir.join("buzz.scap"), synth_pair_bytes(false)).unwrap();
    let manifest = json!({
        "version": 1,
        "experiment": "differential",
        "tag": "diff",
        "stroke_plan": stroke_plan,
        "steps": [{"name": "buzz", "capture": "buzz.scap"}],
    });
    std::fs::write(
        dir.join("manifest.json"),
        serde_json::to_string_pretty(&manifest).unwrap(),
    )
    .unwrap();
}

#[test]
fn differential_run_dir_produces_results_and_plot() {
    let dir = temp_run_dir();
    write_manifest(&dir, json!({"freq_start": 30.0, "freq_end": 150.0}));

    let (results, plot) = build_run(&dir).unwrap();
    assert_eq!(results.steps.len(), 1);
    let sr = &results.steps[0];
    assert!(sr.drives.is_empty());
    assert!(sr.combined.is_none());
    assert!(sr.accel.is_none());
    assert!(sr.flags.is_empty());
    let d = sr.differential.as_ref().expect("differential result");
    assert!((d.modes[0].freq_hz - MODE_HZ).abs() < 2.0);
    assert!(d.segments >= MIN_SEGMENTS);
    assert!(results.verdict.recommended_step.is_none());
    assert!(results.verdict.apply.is_none());
    assert!(
        results.verdict.reason.starts_with("modes: "),
        "{}",
        results.verdict.reason
    );

    let ps = &plot.steps[0];
    assert!(ps.t_s.len() <= 2000);
    assert_eq!(ps.moving.len(), 1);
    assert_eq!(ps.drives.len(), 2);
    for pd in ps.drives.values() {
        assert_eq!(pd.ferr_counts.len(), ps.t_s.len());
        assert_eq!(pd.torque_per_mille.len(), ps.t_s.len());
    }
    assert_eq!(ps.psd.per_drive.len(), 2);
    for series in ps.psd.per_drive.values() {
        assert_eq!(series.len(), ps.psd.freq_hz.len());
    }
    let pdiff = ps.differential.as_ref().expect("plot differential");
    assert!(!pdiff.freq_hz.is_empty());
    assert!(pdiff.freq_hz.len() <= 2000);
    assert_eq!(pdiff.mag_db.len(), pdiff.freq_hz.len());
    assert_eq!(pdiff.phase_deg.len(), pdiff.freq_hz.len());
    assert_eq!(pdiff.coherence.len(), pdiff.freq_hz.len());
    assert_eq!(pdiff.torque_db.len(), pdiff.freq_hz.len());
    assert_eq!(pdiff.band, (30.0, 150.0));
    assert!(pdiff.coherence_min > 0.0);
    assert!(pdiff.freq_hz.first().unwrap() >= &15.0);
    assert!(pdiff.freq_hz.last().unwrap() <= &180.0);
    assert_eq!(pdiff.modes.len(), d.modes.len());

    std::fs::remove_dir_all(&dir).ok();
}

#[test]
fn differential_run_requires_stroke_plan_band() {
    let dir = temp_run_dir();
    write_manifest(&dir, json!({}));
    let err = build_run(&dir).unwrap_err();
    assert!(err.contains("stroke_plan.freq_start"), "{err}");
    std::fs::remove_dir_all(&dir).ok();
}
