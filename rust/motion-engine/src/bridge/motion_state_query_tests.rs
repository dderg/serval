use super::{homing_api::required_motor_axes, planner_api::motion_history_host_now};
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

#[test]
fn history_rebases_use_the_router_clock_domain() {
    let before = host_rt::clock::instant_to_f64(std::time::Instant::now());
    let history_now = motion_history_host_now();
    let after = host_rt::clock::instant_to_f64(std::time::Instant::now());

    assert!((before..=after).contains(&history_now));
}
