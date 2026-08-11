use super::homing_api::required_motor_axes;
use crate::kinematics::KinematicsKind;

#[test]
fn unfiltered_query_requires_every_motor_axis() {
    assert_eq!(
        required_motor_axes(KinematicsKind::Cartesian, None),
        Ok([true; 4])
    );
}

#[test]
fn corexy_position_query_requires_both_coupled_motors() {
    assert_eq!(
        required_motor_axes(KinematicsKind::CoreXy, Some(0)),
        Ok([true, true, false, false])
    );
    assert_eq!(
        required_motor_axes(KinematicsKind::CoreXy, Some(1)),
        Ok([true, true, false, false])
    );
}

#[test]
fn extrusion_query_ignores_unrelated_spatial_motors() {
    assert_eq!(
        required_motor_axes(KinematicsKind::CoreXy, Some(3)),
        Ok([false, false, false, true])
    );
    assert_eq!(
        required_motor_axes(KinematicsKind::Cartesian, Some(3)),
        Ok([false, false, false, true])
    );
}

#[test]
fn unknown_axis_fails_loudly() {
    assert_eq!(
        required_motor_axes(KinematicsKind::Cartesian, Some(4)),
        Err(4)
    );
}
