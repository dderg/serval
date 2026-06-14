use super::*;
use temporal::BindingConstraint;
use trajectory::{ReplanBindingSummary, ReplanWorstBinding};

#[test]
fn all_labeled_variants_map_to_correct_derivative_and_pa_flag() {
    let names = vec!["gantry".to_string(), "extruder".to_string()];
    let cases: &[(BindingConstraint, &str, bool)] = &[
        (BindingConstraint::Velocity { set: 0 }, "velocity", false),
        (BindingConstraint::AccelNorm { set: 0 }, "accel", false),
        (BindingConstraint::JerkNorm { set: 0 }, "jerk", false),
        (BindingConstraint::PaVelocity { set: 0 }, "velocity", true),
        (BindingConstraint::PaAccel { set: 0 }, "accel", true),
        (BindingConstraint::PaJerk { set: 0 }, "jerk", true),
    ];
    for &(constraint, expected_derivative, expected_via_pa) in cases {
        let label = label_binding(constraint, &names).unwrap_or_else(|| {
            panic!("{constraint:?} must produce a label");
        });
        assert_eq!(
            label.derivative, expected_derivative,
            "{constraint:?}: wrong derivative"
        );
        assert_eq!(
            label.via_pa, expected_via_pa,
            "{constraint:?}: wrong via_pa"
        );
    }
}

#[test]
fn labeled_set_index_resolves_to_name_or_runtime_caps_fallback() {
    let names = vec!["gantry".to_string()];
    let resolved = label_binding(BindingConstraint::Velocity { set: 0 }, &names).unwrap();
    assert_eq!(resolved.limit, "gantry");

    let fallback = label_binding(BindingConstraint::AccelNorm { set: 1 }, &names).unwrap();
    assert_eq!(fallback.limit, "runtime_caps");
}

#[test]
fn none_and_boundary_produce_no_label() {
    let names = vec!["gantry".to_string()];
    assert!(label_binding(BindingConstraint::None, &names).is_none());
    assert!(label_binding(BindingConstraint::Boundary, &names).is_none());
}

fn summary(set: usize, count: u32, ratio: f64) -> ReplanBindingSummary {
    ReplanBindingSummary {
        histogram: vec![(BindingConstraint::Velocity { set }, count)],
        worst: Some(ReplanWorstBinding {
            constraint: BindingConstraint::Velocity { set },
            ratio,
        }),
    }
}

#[test]
fn record_tallies_window_and_keeps_max_ratio_worst() {
    let t0 = std::time::Instant::now();
    let mut acc = BindingAccumulator::new(t0);
    acc.record(&summary(0, 3, 0.8), 1.0);
    acc.record(&summary(0, 2, 0.95), 2.0);
    assert_eq!(acc.window_count(BindingConstraint::Velocity { set: 0 }), 5);
    let (constraint, ratio, t) = acc.worst().unwrap();
    assert_eq!(constraint, BindingConstraint::Velocity { set: 0 });
    assert!((ratio - 0.95).abs() < 1e-12);
    assert!((t - 2.0).abs() < 1e-12);
}

#[test]
fn maybe_rollup_resets_only_after_the_interval() {
    let t0 = std::time::Instant::now();
    let names = vec!["gantry".to_string()];
    let mut acc = BindingAccumulator::new(t0);
    acc.record(&summary(0, 1, 0.9), 1.0);

    acc.maybe_rollup(t0 + std::time::Duration::from_millis(500), &names);
    assert_eq!(acc.window_count(BindingConstraint::Velocity { set: 0 }), 1);

    acc.maybe_rollup(t0 + std::time::Duration::from_millis(1100), &names);
    assert_eq!(acc.window_count(BindingConstraint::Velocity { set: 0 }), 0);
    assert!(acc.worst().is_none());
}

#[test]
fn flush_emits_and_clears_a_partial_window() {
    let t0 = std::time::Instant::now();
    let names = vec!["gantry".to_string()];
    let mut acc = BindingAccumulator::new(t0);
    acc.record(&summary(0, 1, 0.9), 1.0);
    acc.flush(t0 + std::time::Duration::from_millis(100), &names);
    assert_eq!(acc.window_count(BindingConstraint::Velocity { set: 0 }), 0);
}

#[test]
fn maybe_rollup_on_empty_window_past_interval_is_a_noop() {
    let t0 = std::time::Instant::now();
    let names = vec!["gantry".to_string()];
    let mut acc = BindingAccumulator::new(t0);
    acc.maybe_rollup(t0 + std::time::Duration::from_millis(1100), &names);
    assert!(acc.worst().is_none());
    assert_eq!(acc.window_count(BindingConstraint::Velocity { set: 0 }), 0);
}
