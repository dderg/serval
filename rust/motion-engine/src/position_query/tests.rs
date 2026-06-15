use super::*;
use crate::dispatch::KINEMATICS_COREXY;
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
    // motor A = x + y, motor B = x - y. For x=10, y=4: A=14, B=6.
    let mut m = [None; MAX_AXES];
    let v = [None; MAX_AXES];
    m[0] = Some(14.0);
    m[1] = Some(6.0);
    m[2] = Some(0.0);
    let out = assemble_cartesian(&m, &v, KINEMATICS_COREXY).unwrap();
    assert!((out["x"].0 - 10.0).abs() < 1e-9);
    assert!((out["y"].0 - 4.0).abs() < 1e-9);
}
