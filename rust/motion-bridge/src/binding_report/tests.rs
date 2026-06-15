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

/// G5 — the gap + limiter the `replan_anytime` event carries must match the
/// actual binding constraint and its verified ratio. With a known XY-velocity
/// binding at ratio 0.94 (a conservative floor), the event reports the matching
/// limiter and `gap = 1 - 0.94 = 0.06`.
#[test]
fn g5_anytime_event_matches_binding_and_gap() {
    let names = vec!["gantry".to_string(), "extruder".to_string()];
    let binding = ReplanBindingSummary {
        histogram: vec![(BindingConstraint::Velocity { set: 0 }, 7)],
        worst: Some(ReplanWorstBinding {
            constraint: BindingConstraint::Velocity { set: 0 },
            ratio: 0.94,
        }),
    };

    let fields = anytime_event_fields(&binding, &names);

    assert_eq!(fields.limiter_limit, "gantry");
    assert_eq!(fields.limiter_derivative, "velocity");
    assert!(!fields.limiter_via_pa);
    assert!((fields.binding_ratio - 0.94).abs() < 1e-12);
    assert!(
        (fields.gap - 0.06).abs() < 1e-9,
        "gap must equal (1 - ratio).max(0); got {}",
        fields.gap,
    );
}

/// G5 — a PA-jerk binding on set 1 (no name → runtime_caps fallback) at ratio
/// 0.88 produces the matching limiter and gap 0.12. Confirms the event tracks
/// the actual worst family, not a fixed one.
#[test]
fn g5_anytime_event_tracks_pa_jerk_family() {
    let names = vec!["gantry".to_string()];
    let binding = ReplanBindingSummary {
        histogram: vec![(BindingConstraint::PaJerk { set: 1 }, 3)],
        worst: Some(ReplanWorstBinding {
            constraint: BindingConstraint::PaJerk { set: 1 },
            ratio: 0.88,
        }),
    };

    let fields = anytime_event_fields(&binding, &names);

    assert_eq!(fields.limiter_limit, "runtime_caps");
    assert_eq!(fields.limiter_derivative, "jerk");
    assert!(fields.limiter_via_pa);
    assert!((fields.gap - 0.12).abs() < 1e-9);
}

/// G5 — an on-the-limit binding (ratio = 1.0, the converged optimum) reports
/// gap 0; no binding at all reports the `none` limiter and gap 0.
#[test]
fn g5_anytime_event_on_limit_and_no_binding() {
    let names = vec!["gantry".to_string()];

    let on_limit = ReplanBindingSummary {
        histogram: vec![(BindingConstraint::AccelNorm { set: 0 }, 1)],
        worst: Some(ReplanWorstBinding {
            constraint: BindingConstraint::AccelNorm { set: 0 },
            ratio: 1.0,
        }),
    };
    let f = anytime_event_fields(&on_limit, &names);
    assert!(f.gap.abs() < 1e-12, "on-limit gap must be 0; got {}", f.gap);
    assert_eq!(f.limiter_derivative, "accel");

    let none = ReplanBindingSummary::default();
    let f = anytime_event_fields(&none, &names);
    assert_eq!(f.limiter_limit, "none");
    assert_eq!(f.limiter_derivative, "none");
    assert!(f.gap.abs() < 1e-12);
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
