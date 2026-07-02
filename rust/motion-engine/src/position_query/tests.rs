use super::*;
use crate::mcu_config::KINEMATICS_COREXY;
use runtime::segment::KinematicTag;
use runtime::stepping_state::MAX_AXES;

#[test]
fn cartesian_identity_passthrough() {
    let mut m = [None; MAX_AXES];
    let mut v = [None; MAX_AXES];
    m[0] = Some(10.0);
    m[1] = Some(20.0);
    m[2] = Some(5.0);
    m[3] = Some(2.0);
    v[0] = Some(1.0);
    v[1] = Some(-1.0);
    v[2] = Some(0.0);
    v[3] = Some(3.0);
    let out = assemble_cartesian(&m, &v, KinematicTag::Cartesian as u8).unwrap();
    assert_eq!(out["x"], (10.0, 1.0));
    assert_eq!(out["y"], (20.0, -1.0));
    assert_eq!(out["z"], (5.0, 0.0));
    assert_eq!(out["e"], (2.0, 3.0));
}

#[test]
fn corexy_inverse_mix() {
    let expected_x = 10.0;
    let expected_y = 4.0;
    let motor_a = expected_x + expected_y;
    let motor_b = expected_x - expected_y;
    let mut m = [None; MAX_AXES];
    let v = [None; MAX_AXES];
    m[0] = Some(motor_a);
    m[1] = Some(motor_b);
    m[2] = Some(0.0);
    let out = assemble_cartesian(&m, &v, KINEMATICS_COREXY).unwrap();
    assert!((out["x"].0 - expected_x).abs() < 1e-9);
    assert!((out["y"].0 - expected_y).abs() < 1e-9);
}
