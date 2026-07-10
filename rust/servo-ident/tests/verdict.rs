use std::collections::BTreeMap;

use serde_json::json;

use servo_ident::analyze::compute_verdict;
use servo_ident::results::{Applied, Step, StepResult};

fn step_result(name: &str, flags: &[&str]) -> StepResult {
    StepResult {
        name: name.to_string(),
        drives: BTreeMap::new(),
        combined: None,
        accel: None,
        flags: flags.iter().map(|s| s.to_string()).collect(),
    }
}

fn manifest_step(name: &str, swept: serde_json::Value) -> Step {
    Step {
        name: name.to_string(),
        swept,
        applied: vec![Applied {
            servo: "motor_a".to_string(),
            addr: "0x2001.0x01".to_string(),
            ty: "u16".to_string(),
            value: json!(1),
        }],
        capture: format!("step_{name}.scap"),
        accel: None,
    }
}

#[test]
fn gain_sweep_picks_highest_clean_speed() {
    let steps = vec![step_result("a", &[]), step_result("b", &[])];
    let manifest = vec![
        manifest_step("a", json!({"speed": 550})),
        manifest_step("b", json!({"speed": 700})),
    ];
    let v = compute_verdict("gain_sweep", &steps, &manifest).unwrap();
    assert_eq!(v.recommended_step.as_deref(), Some("b"));
    assert!(v.apply.is_some());
}

#[test]
fn gain_sweep_skips_resonant_top_step() {
    let steps = vec![
        step_result("a", &[]),
        step_result("b", &["resonance_detected"]),
    ];
    let manifest = vec![
        manifest_step("a", json!({"speed": 550})),
        manifest_step("b", json!({"speed": 700})),
    ];
    let v = compute_verdict("gain_sweep", &steps, &manifest).unwrap();
    assert_eq!(v.recommended_step.as_deref(), Some("a"));
}

#[test]
fn gain_sweep_null_when_all_flagged() {
    let steps = vec![
        step_result("a", &["torque_saturated"]),
        step_result("b", &["resonance_detected"]),
    ];
    let manifest = vec![
        manifest_step("a", json!({"speed": 550})),
        manifest_step("b", json!({"speed": 700})),
    ];
    let v = compute_verdict("gain_sweep", &steps, &manifest).unwrap();
    assert_eq!(v.recommended_step, None);
    assert!(v.apply.is_none());
    assert!(!v.reason.is_empty());
}

#[test]
fn refine_sweep_uses_single_swept_value() {
    let steps = vec![step_result("lo", &[]), step_result("hi", &[])];
    let manifest = vec![
        manifest_step("lo", json!({"gain": 600})),
        manifest_step("hi", json!({"gain": 800})),
    ];
    let v = compute_verdict("refine_sweep", &steps, &manifest).unwrap();
    assert_eq!(v.recommended_step.as_deref(), Some("hi"));
}

#[test]
fn accel_sweep_ports_recommend() {
    let steps = vec![
        step_result("a1", &[]),
        step_result("a2", &[]),
        step_result("a3", &["torque_saturated"]),
    ];
    let manifest = vec![
        manifest_step("a1", json!({"accel": 10000})),
        manifest_step("a2", json!({"accel": 20000})),
        manifest_step("a3", json!({"accel": 30000})),
    ];
    let v = compute_verdict("accel_sweep", &steps, &manifest).unwrap();
    assert_eq!(v.recommended_step.as_deref(), Some("a2"));
}

#[test]
fn inertia_sweep_defers_to_human() {
    let v = compute_verdict(
        "inertia_sweep",
        &[step_result("a", &[])],
        &[manifest_step("a", json!({"ratio": 200}))],
    )
    .unwrap();
    assert_eq!(v.recommended_step, None);
    assert!(v.reason.contains("overshoot"));
}

#[test]
fn tracking_and_grid_are_not_sweeps() {
    for exp in ["tracking", "inertia_grid"] {
        let v = compute_verdict(
            exp,
            &[step_result("a", &[])],
            &[manifest_step("a", json!({}))],
        )
        .unwrap();
        assert_eq!(v.recommended_step, None);
        assert_eq!(v.reason, "not a sweep");
    }
}

#[test]
fn unknown_experiment_fails_loud() {
    assert!(compute_verdict("bogus", &[], &[]).is_err());
}
