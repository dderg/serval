//! Run-directory integration test: hand-build a minimal gain_sweep run dir
//! from the two fixtures, analyze it, and check the results/plot contracts.

use std::io::Read;
use std::path::{Path, PathBuf};

use flate2::read::GzDecoder;
use serde_json::{json, Value};

use servo_ident::analyze::{analyze_run, build_run};

fn fixture_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../test/fixtures/servo_captures")
}

fn gunzip_to(name: &str, dst: &Path) {
    let src = fixture_dir().join(format!("{name}.gz"));
    let file = std::fs::File::open(&src).unwrap();
    let mut d = GzDecoder::new(file);
    let mut out = Vec::new();
    d.read_to_end(&mut out).unwrap();
    std::fs::write(dst, out).unwrap();
}

fn temp_run_dir() -> PathBuf {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let dir = std::env::temp_dir().join(format!("servo_cal_run_{}_{nanos}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

fn build_manifest(dir: &Path) {
    gunzip_to(
        "cal_p880_s550_i2273_20260710_151516.scap",
        &dir.join("step_s550.scap"),
    );
    gunzip_to(
        "cal_p1120_s700_i1786_20260710_151521.scap",
        &dir.join("step_s700.scap"),
    );
    let manifest = json!({
        "version": 1,
        "experiment": "gain_sweep",
        "tag": "cal",
        "axis": "X",
        "kinematics": "corexy",
        "belts": "motor_a:1+motor_a1:-1,motor_b:-1+motor_b1:-1",
        "steps": [
            {
                "name": "s550",
                "swept": {"position": 880, "speed": 550, "integral": 2273},
                "applied": [{"servo": "motor_a", "addr": "0x2001.0x01", "type": "u16", "value": 880}],
                "capture": "step_s550.scap"
            },
            {
                "name": "s700",
                "swept": {"position": 1120, "speed": 700, "integral": 1786},
                "applied": [{"servo": "motor_a", "addr": "0x2001.0x01", "type": "u16", "value": 1120}],
                "capture": "step_s700.scap"
            }
        ]
    });
    std::fs::write(
        dir.join("manifest.json"),
        serde_json::to_string_pretty(&manifest).unwrap(),
    )
    .unwrap();
}

#[test]
fn gain_sweep_run_dir_analyzes_and_picks_a_step() {
    let dir = temp_run_dir();
    build_manifest(&dir);

    let (results, plot) = build_run(&dir).unwrap();
    assert_eq!(results.steps.len(), 2);
    assert_eq!(results.fs_hz, 4000.0);
    // Both fixtures are clean (no resonance, no rail), so the higher speed wins.
    match &results.verdict.recommended_step {
        Some(name) => {
            assert_eq!(name, "s700", "clean sweep should pick the highest speed");
            let apply = results.verdict.apply.as_ref().expect("apply payload");
            assert_eq!(apply[0].value, json!(1120));
        }
        None => assert!(
            !results.verdict.reason.is_empty(),
            "a null pick must carry a reason"
        ),
    }
    for step in &results.steps {
        assert!(step.combined.is_some(), "corexy belts must combine");
    }
    for ps in &plot.steps {
        assert!(ps.t_s.len() <= 2000, "t_s series must be <= 2000 points");
        for d in ps.drives.values() {
            assert!(d.ferr_counts.len() <= 2000);
            assert!(d.torque_per_mille.len() <= 2000);
            assert_eq!(d.ferr_counts.len(), ps.t_s.len());
        }
        if let Some(c) = &ps.combined {
            assert!(c.on_ferr_mm.len() <= 2000);
            assert!(c.cross_ferr_mm.len() <= 2000);
        }
        assert!(
            ps.psd.freq_hz.len() <= 2000,
            "psd freq_hz must be <= 2000 bins"
        );
        for (name, series) in &ps.psd.per_drive {
            assert_eq!(
                series.len(),
                ps.psd.freq_hz.len(),
                "drive {name} psd length must match freq_hz"
            );
        }
        assert!(
            !ps.psd.per_drive.is_empty(),
            "every step must carry at least one drive's psd"
        );
    }

    analyze_run(&dir, false).unwrap();
    let results_text = std::fs::read_to_string(dir.join("results.json")).unwrap();
    let parsed: Value = serde_json::from_str(&results_text).unwrap();
    assert_eq!(parsed["version"], json!(1));
    assert!(parsed["steps"].as_array().unwrap().len() == 2);
    let plot_text = std::fs::read_to_string(dir.join("plot_series.json")).unwrap();
    let plot_parsed: Value = serde_json::from_str(&plot_text).unwrap();
    assert_eq!(plot_parsed["version"], json!(1));

    analyze_run(&dir, true).unwrap();
    let incr_text = std::fs::read_to_string(dir.join("results.json")).unwrap();
    assert_eq!(
        serde_json::from_str::<Value>(&incr_text).unwrap(),
        parsed,
        "incremental re-analyze must reproduce the full analyze byte-for-byte"
    );

    std::fs::remove_dir_all(&dir).ok();
}
