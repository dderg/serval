use std::path::PathBuf;

use _motion_engine::binding_report::BindingAccumulator;
use temporal::BindingConstraint;
use trajectory::{ReplanBindingSummary, ReplanWorstBinding};

#[test]
fn binding_events_land_in_host_rust_jsonl_tagged_with_print_id() {
    let dir: PathBuf =
        std::env::temp_dir().join(format!("kalico-binding-e2e-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();

    _motion_engine::logging::init_logging(&dir).expect("init");
    _motion_engine::logging::set_context(
        "k-1748700131-4412".to_string(),
        "print-1748700500".to_string(),
    );

    let limit_names = vec!["gantry".to_string(), "extruder".to_string()];
    let summary = ReplanBindingSummary {
        histogram: vec![
            (BindingConstraint::Velocity { set: 0 }, 7),
            (BindingConstraint::PaAccel { set: 1 }, 3),
        ],
        worst: Some(ReplanWorstBinding {
            constraint: BindingConstraint::PaAccel { set: 1 },
            ratio: 0.98,
        }),
    };

    let start = std::time::Instant::now();
    let mut acc = BindingAccumulator::new(start);
    acc.record(&summary, 4.25);
    acc.flush(start + std::time::Duration::from_millis(10), &limit_names);

    std::thread::sleep(std::time::Duration::from_millis(250));

    let contents = std::fs::read_to_string(dir.join("host-rust.jsonl")).unwrap();
    let lines: Vec<serde_json::Value> = contents
        .lines()
        .filter(|l| !l.is_empty())
        .map(|l| serde_json::from_str(l).expect("valid JSON line"))
        .collect();

    let rollup = lines
        .iter()
        .find(|r| r["event"] == "binding_rollup")
        .expect("binding_rollup must be emitted");
    assert_eq!(rollup["source"], "host-rust");
    assert_eq!(rollup["print_id"], "print-1748700500");
    assert_eq!(rollup["subsystem"], "motion");
    assert_eq!(rollup["limit"], "extruder");
    assert_eq!(rollup["derivative"], "accel");
    assert_eq!(rollup["via_pa"], true);
    assert!((rollup["ratio"].as_f64().unwrap() - 0.98).abs() < 1e-9);
    assert!((rollup["t"].as_f64().unwrap() - 4.25).abs() < 1e-9);
    assert_eq!(rollup["window_samples"].as_u64().unwrap(), 10);

    let hist: Vec<&serde_json::Value> = lines
        .iter()
        .filter(|r| r["event"] == "binding_hist")
        .collect();
    assert!(
        hist.iter().any(|r| r["limit"] == "gantry"
            && r["derivative"] == "velocity"
            && r["via_pa"] == false
            && r["count"].as_u64() == Some(7)),
        "expected gantry/velocity count=7 in binding_hist, got {hist:?}"
    );
    assert!(
        hist.iter().any(|r| r["limit"] == "extruder"
            && r["derivative"] == "accel"
            && r["via_pa"] == true
            && r["count"].as_u64() == Some(3)),
        "expected extruder/accel count=3 in binding_hist, got {hist:?}"
    );
    for r in &hist {
        assert_eq!(r["print_id"], "print-1748700500");
        assert_eq!(r["subsystem"], "motion");
    }
}
