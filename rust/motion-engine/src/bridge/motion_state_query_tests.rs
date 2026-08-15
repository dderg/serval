use super::{
    homing_api::{history_state_at_query, required_motor_axes},
    planner_api::motion_history_host_now,
};
use crate::kinematics::KinematicsKind;
use crate::{motion_history::HistoryStore, types::AxisKey};
use runtime::piece_ring::{MAX_PIECE_COEFFS, PieceEntry};

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
    let mut coeffs = [0.0; MAX_PIECE_COEFFS];
    coeffs[0] = 5.0;
    coeffs[1] = 5.0;
    let piece = PieceEntry {
        start_time: 0,
        duration: 1.0,
        coeff_count: 2,
        coeffs,
        ..PieceEntry::zeroed()
    };
    let mut store = HistoryStore::default();
    store.record(key, &piece, 1_000.0, 0.04);

    let exact = history_state_at_query(&store, key, 7, 500, 0.5, f64::INFINITY).unwrap();
    let projected = history_state_at_query(&store, key, 8, 500, 0.5, f64::INFINITY).unwrap();

    assert!((exact.position - 5.0).abs() < 1.0e-9);
    assert!((projected.position - 4.6).abs() < 1.0e-9);
}
