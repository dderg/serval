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

fn step<'a>(results: &'a Value, name: &str) -> &'a Value {
    results["steps"]
        .as_array()
        .unwrap()
        .iter()
        .find(|s| s["name"].as_str() == Some(name))
        .unwrap_or_else(|| panic!("results.json has no step {name:?}"))
}

fn step_flags(results: &Value, name: &str) -> Vec<String> {
    step(results, name)["flags"]
        .as_array()
        .unwrap()
        .iter()
        .map(|f| f.as_str().unwrap().to_string())
        .collect()
}

fn assert_psd_shape(plot: &Value) {
    for step in plot["steps"].as_array().unwrap() {
        let psd = &step["psd"];
        let freq_hz = psd["freq_hz"].as_array().unwrap();
        assert!(
            freq_hz.len() <= 2000,
            "step {}: psd freq_hz must be <= 2000 bins",
            step["name"]
        );
        let per_drive = psd["per_drive"].as_object().unwrap();
        assert!(
            !per_drive.is_empty(),
            "step {}: psd.per_drive must not be empty",
            step["name"]
        );
        for (drive, series) in per_drive {
            assert_eq!(
                series.as_array().unwrap().len(),
                freq_hz.len(),
                "step {}: drive {drive} psd length must match freq_hz",
                step["name"]
            );
        }
        let cartesian = psd["cartesian"].as_object().unwrap_or_else(|| {
            panic!(
                "step {}: demo manifests carry a spatial frame, psd.cartesian must be present",
                step["name"]
            )
        });
        assert_eq!(
            cartesian.keys().collect::<Vec<_>>(),
            vec!["x", "y"],
            "step {}: cartesian modes",
            step["name"]
        );
        for (mode, series) in cartesian {
            assert_eq!(
                series.as_array().unwrap().len(),
                freq_hz.len(),
                "step {}: cartesian mode {mode} psd length must match freq_hz",
                step["name"]
            );
        }
        let accel = &psd["accel"];
        assert!(
            !accel.is_null(),
            "step {}: demo steps all carry an accel capture, psd.accel must not be null",
            step["name"]
        );
        let accel_freq = accel["freq_hz"].as_array().unwrap();
        for key in ["psd", "psd_x", "psd_y", "psd_z"] {
            assert_eq!(
                accel[key].as_array().unwrap().len(),
                accel_freq.len(),
                "step {}: accel {key} and freq_hz must have equal length",
                step["name"]
            );
        }
        assert!(accel_freq.len() <= 2000);
    }
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
        let plot_text = std::fs::read_to_string(run_dir.join("plot_series.json")).unwrap();
        let plot: Value = serde_json::from_str(&plot_text).unwrap();
        assert_psd_shape(&plot);

        notches.push(notch_value(run_dir));
    }

    assert_eq!(
        notches,
        vec![3, 1, 0],
        "ambient notch must walk 3 -> 1 -> 0 across attempts"
    );

    std::fs::remove_dir_all(&out_dir).ok();
}

/// The whole point of the injected fixture: `attempt1`'s `s700` capture gets
/// a synthetic resonance stamped onto it (see `servo_ident::demo`'s record
/// patcher), so its step must flag `resonance_detected` and the gain_sweep
/// verdict must fall back to the clean `s550` step. `attempt2`/`attempt3`
/// never touch the injected bytes and must stay on the untouched `s700` pick.
#[test]
fn attempt1_resonance_injection_flips_the_verdict_to_the_safe_step() {
    let out_dir = temp_dir("resonance");
    let run_dirs = build_demo(&out_dir, &fixture_dir()).unwrap();
    assert_eq!(
        run_dirs.len(),
        3,
        "oldest first: attempt1, attempt2, attempt3"
    );

    let results_of = |run_dir: &Path| -> Value {
        let text = std::fs::read_to_string(run_dir.join("results.json")).unwrap();
        serde_json::from_str(&text).unwrap()
    };

    let attempt1 = results_of(&run_dirs[0]);
    assert!(
        step_flags(&attempt1, "s700").contains(&"resonance_detected".to_string()),
        "attempt1's injected s700 must flag resonance_detected: {:?}",
        step_flags(&attempt1, "s700")
    );
    assert_eq!(
        attempt1["verdict"]["recommended_step"],
        Value::from("s550"),
        "attempt1's verdict must fall back to the clean s550 step"
    );

    for run_dir in &run_dirs[1..] {
        let results = results_of(run_dir);
        assert!(
            !step_flags(&results, "s700").contains(&"resonance_detected".to_string()),
            "{}: s700 must stay clean (only attempt1's copy is injected)",
            run_dir.display()
        );
        assert_eq!(
            results["verdict"]["recommended_step"],
            Value::from("s700"),
            "{}: verdict must still pick the highest clean speed",
            run_dir.display()
        );
    }

    std::fs::remove_dir_all(&out_dir).ok();
}

/// The tuning grid (`docs/rewrite/servo-tuning-profiles.md`'s
/// `SERVO_DUMP_TUNING` shape) has nothing to render without a bench unless
/// `servo-cal demo` also ships a plausible `<out_dir>/drive_state.json` —
/// this asserts the file mirrors the shipped `PANEL_PARAMS` map (27 params,
/// 4 AWD corexy motors) with the config-pin mechanism present but empty.
#[test]
fn demo_writes_a_drive_state_the_panel_can_render() {
    let out_dir = temp_dir("drive_state");
    build_demo(&out_dir, &fixture_dir()).unwrap();

    let path = out_dir.join("drive_state.json");
    assert!(path.is_file(), "servo-cal demo must write drive_state.json");
    let text = std::fs::read_to_string(&path).unwrap();
    let drive_state: Value = serde_json::from_str(&text).expect("drive_state.json must parse");

    assert_eq!(drive_state["version"], Value::from(1));
    assert!(drive_state["created_utc"].as_str().is_some());

    let params = drive_state["params"].as_array().unwrap();
    assert_eq!(params.len(), 27, "must mirror all 27 shipped PANEL_PARAMS");
    let names: Vec<&str> = params.iter().map(|p| p["name"].as_str().unwrap()).collect();
    for expected in [
        "position_gain",
        "speed_gain",
        "integral_time",
        "torque_filter_cutoff",
        "notch_1_freq",
        "notch_3_depth",
        "notch_5_width",
        "speed_feedback_filter",
        "speed_observer_gain",
        "speed_observer_inertia",
        "speed_observer_cutoff",
        "disturbance_gain",
        "disturbance_inertia",
        "disturbance_cutoff",
        "disturbance_comp_torque",
    ] {
        assert!(names.contains(&expected), "missing panel param {expected}");
    }
    let notch_5_depth = params
        .iter()
        .find(|p| p["name"] == "notch_5_depth")
        .unwrap();
    assert_eq!(
        notch_5_depth["addr"],
        Value::from("0x2001.0x4f"),
        "A6-EC manual 7.10: notch 5 depth is C01.4E, +1 for the SDO subindex"
    );
    let position_gain = params
        .iter()
        .find(|p| p["name"] == "position_gain")
        .unwrap();
    assert_eq!(position_gain["c_code"], Value::from("C01.00"));
    assert_eq!(position_gain["addr"], Value::from("0x2001.0x01"));
    assert_eq!(position_gain["group"], Value::from("gains"));

    let motors = drive_state["motors"].as_object().unwrap();
    assert_eq!(motors.len(), 4);
    for name in ["motor_a", "motor_a1", "motor_b", "motor_b1"] {
        let readings = motors
            .get(name)
            .unwrap_or_else(|| panic!("missing motor {name}"));
        assert_eq!(readings["C01.00"], Value::from(880));
        assert_eq!(readings["C01.01"], Value::from(550));
        assert_eq!(readings["C01.02"], Value::from(2273));
        assert_eq!(readings["C01.03"], Value::from(220));
        assert_eq!(readings["C02.60"], Value::from(2000));
        assert_eq!(readings["C02.62"], Value::from(30));
        assert_eq!(readings["C02.63"], Value::from(150));
        let notch_1_freq_expected = if name == "motor_b" { 400 } else { 345 };
        assert_eq!(
            readings["C01.40"],
            Value::from(notch_1_freq_expected),
            "notch 1 frequency carries the deliberate motor_b drift"
        );
    }

    let config_pins = drive_state["config_pins"].as_object().unwrap();
    assert_eq!(config_pins.len(), 4);
    for name in ["motor_a", "motor_a1", "motor_b", "motor_b1"] {
        let pins = config_pins.get(name).unwrap();
        assert_eq!(
            pins.as_object().unwrap().len(),
            0,
            "no demo params are pinned"
        );
    }

    let slots = drive_state["slots"].as_object().unwrap();
    assert_eq!(slots.len(), 4);
    for (slot, name) in ["motor_a", "motor_a1", "motor_b", "motor_b1"]
        .iter()
        .enumerate()
    {
        assert_eq!(
            slots[*name],
            Value::from(slot),
            "slots must number the motors 0..N in sorted order"
        );
    }

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
