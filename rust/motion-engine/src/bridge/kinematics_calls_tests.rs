use crate::dispatch::{AXIS_X, AXIS_Y, KINEMATICS_COREXY};
use crate::kinematics::KinematicsModule;

#[test]
fn corexy_inverse_maps_motor_to_axes() {
    let xyz = KinematicsModule::from_tag(KINEMATICS_COREXY)
        .unwrap()
        .inverse([4.0, 2.0, 0.0]);
    assert!(
        (xyz[0] - 3.0).abs() < 1e-9,
        "expected x=3.0, got {}",
        xyz[0]
    );
    assert!(
        (xyz[1] - 1.0).abs() < 1e-9,
        "expected y=1.0, got {}",
        xyz[1]
    );
}

#[test]
fn corexy_forward_delta_only_slot0_moves_when_dx_eq_dy() {
    let m = KinematicsModule::from_tag(KINEMATICS_COREXY)
        .unwrap()
        .forward([1.0, 1.0, 0.0]);
    let slots: Vec<(usize, f64)> = (0..3)
        .filter(|&s| m[s].abs() > 1e-9)
        .map(|s| (s, m[s]))
        .collect();
    assert_eq!(
        slots.len(),
        1,
        "expected only 1 moving slot, got: {slots:?}"
    );
    assert_eq!(slots[0].0, AXIS_X, "moving slot should be AXIS_X (A motor)");
    assert!(
        (slots[0].1 - 2.0).abs() < 1e-9,
        "A delta should be 2.0, got {}",
        slots[0].1
    );
}

#[test]
fn corexy_forward_motor_positions_xyz() {
    let m = KinematicsModule::from_tag(KINEMATICS_COREXY)
        .unwrap()
        .forward([3.0, 1.0, 5.0]);
    assert!(
        (m[AXIS_X] - 4.0).abs() < 1e-9,
        "slot AXIS_X (A) = 4, got {}",
        m[AXIS_X]
    );
    assert!(
        (m[AXIS_Y] - 2.0).abs() < 1e-9,
        "slot AXIS_Y (B) = 2, got {}",
        m[AXIS_Y]
    );
}
