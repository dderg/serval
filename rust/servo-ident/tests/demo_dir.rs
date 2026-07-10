//! `servo-cal demo` structure test: the generated run directories must
//! parse against the same `Manifest`/`Results` structs `serve` and
//! `analyze` use, and the three attempts must actually differ in the
//! ambient notch value they journal — the whole point of the demo.

use std::path::{Path, PathBuf};

use serde_json::Value;

use servo_ident::demo::build_demo;
use servo_ident::results::Manifest;

fn fixture_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../test/fixtures/servo_captures")
}

fn temp_dir(label: &str) -> PathBuf {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let dir = std::env::temp_dir().join(format!(
        "servo_cal_demo_{label}_{}_{nanos}",
        std::process::id()
    ));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

fn notch_value(run_dir: &Path) -> i64 {
    let text = std::fs::read_to_string(run_dir.join("manifest.json")).unwrap();
    let v: Value = serde_json::from_str(&text).unwrap();
    v["ambient"]["journal_params"]["motor_a"]["0x2001.0x31"]
        .as_i64()
        .expect("journal_params.motor_a.0x2001.0x31 must be present")
}

#[test]
fn demo_runs_parse_and_ambient_notch_differs() {
    let out_dir = temp_dir("out");
    let run_dirs = build_demo(&out_dir, &fixture_dir()).unwrap();
    assert_eq!(run_dirs.len(), 3);

    let mut notches = Vec::new();
    for run_dir in &run_dirs {
        assert!(run_dir.join("manifest.json").is_file());
        assert!(run_dir.join("step_s550.scap").is_file());
        assert!(run_dir.join("step_s700.scap").is_file());
        assert!(run_dir.join("step_s550_accel.csv").is_file());
        assert!(run_dir.join("step_s700_accel.csv").is_file());

        let manifest_text = std::fs::read_to_string(run_dir.join("manifest.json")).unwrap();
        let manifest: Manifest = serde_json::from_str(&manifest_text)
            .expect("demo manifest must parse against the results::Manifest reader");
        assert_eq!(manifest.experiment, "gain_sweep");
        assert_eq!(manifest.steps.len(), 2);

        let results_text = std::fs::read_to_string(run_dir.join("results.json")).unwrap();
        let results: Value = serde_json::from_str(&results_text).unwrap();
        assert_eq!(results["version"], Value::from(1));
        assert_eq!(results["steps"].as_array().unwrap().len(), 2);

        assert!(run_dir.join("plot_series.json").is_file());

        notches.push(notch_value(run_dir));
    }

    assert_eq!(
        notches,
        vec![3, 1, 0],
        "ambient notch must walk 3 -> 1 -> 0 across attempts"
    );

    std::fs::remove_dir_all(&out_dir).ok();
}

#[test]
fn demo_missing_fixtures_dir_fails_loud() {
    let out_dir = temp_dir("missing");
    let bogus_fixtures = out_dir.join("no-such-fixtures");
    let err = build_demo(&out_dir, &bogus_fixtures).unwrap_err();
    assert!(err.contains("no-such-fixtures") || err.contains("--fixtures"));
    std::fs::remove_dir_all(&out_dir).ok();
}
