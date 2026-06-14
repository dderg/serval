use super::*;
use temporal::BindingConstraint;

#[test]
fn labels_pa_accel_with_resolved_name() {
    let names = vec!["gantry".to_string(), "extruder".to_string()];
    let label = label_binding(BindingConstraint::PaAccel { set: 1 }, &names).unwrap();
    assert_eq!(label.limit, "extruder");
    assert_eq!(label.derivative, "accel");
    assert!(label.via_pa);
}

#[test]
fn labels_spatial_velocity_without_pa() {
    let names = vec!["gantry".to_string()];
    let label = label_binding(BindingConstraint::Velocity { set: 0 }, &names).unwrap();
    assert_eq!(label.limit, "gantry");
    assert_eq!(label.derivative, "velocity");
    assert!(!label.via_pa);
}

#[test]
fn trailing_set_index_resolves_to_runtime_caps() {
    let names = vec!["gantry".to_string()];
    let label = label_binding(BindingConstraint::AccelNorm { set: 1 }, &names).unwrap();
    assert_eq!(label.limit, "runtime_caps");
}

#[test]
fn none_and_boundary_have_no_label() {
    let names = vec!["gantry".to_string()];
    assert!(label_binding(BindingConstraint::None, &names).is_none());
    assert!(label_binding(BindingConstraint::Boundary, &names).is_none());
}
