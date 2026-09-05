use super::{
    homing_api::{history_state_at_query, required_motor_axes},
    planner_api::motion_history_host_now,
};
use crate::kinematics::KinematicsKind;
use crate::{motion_history::HistoryStore, types::AxisKey};
use std::sync::Arc;
use trajectory::{
    ClockedMotorSpan, ContinuousAxis, MotorGroup, MotorSpan, MotorTerm, NudgeProfile,
};

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

#[test]
fn same_mcu_query_uses_wire_clock_instead_of_stale_host_projection() {
    let key = AxisKey { mcu_id: 7, axis: 3 };
    const FREQ_HZ: f64 = 1_000.0;
    const START_HOST: f64 = 0.04;
    let profile = NudgeProfile::try_new(10.0, 10.0, 0.0, 0.0).expect("cruise-only nudge profile");
    let t_end = profile.t_end();
    let signal = MotorSpan::try_new(
        Arc::from([MotorGroup::Independent(MotorTerm {
            source_axis: 0,
            axis: ContinuousAxis::Nudge(profile),
            scale: 1.0,
        })]),
        0.0,
        t_end,
        0,
        0,
        false,
    )
    .expect("a cruise ramp is dispatchable");
    let span = ClockedMotorSpan::try_new(
        Arc::new(signal),
        0.0,
        t_end,
        START_HOST,
        START_HOST + t_end,
        0.0,
        FREQ_HZ,
    )
    .expect("the projected view spans at least one clock");
    let mut store = HistoryStore::default();
    store.record(key, span).expect("history accepts the view");

    let exact = history_state_at_query(&store, key, 7, 500, 0.5, f64::INFINITY).unwrap();
    let projected = history_state_at_query(&store, key, 8, 500, 0.5, f64::INFINITY).unwrap();

    assert!((exact.position - 5.0).abs() < 1.0e-9);
    assert!((projected.position - 4.6).abs() < 1.0e-9);
}
