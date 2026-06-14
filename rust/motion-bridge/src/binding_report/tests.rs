use super::*;
use temporal::BindingConstraint;

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
