use super::HomingState;
use crate::lock_ext::LockExt;

const MCU: u32 = 3;

#[test]
fn window_start_is_the_earliest_arm_among_the_run_endstops() {
    let state = HomingState::default();
    state.note_arm(MCU, 0, 10.0);
    state.note_arm(MCU, 5, 10.5);
    let start = state.take_arm_window_start(&[(MCU, 0), (MCU, 5)]);
    assert_eq!(start, Some(10.0));
}

#[test]
fn window_start_ignores_arms_outside_the_run_endstop_set() {
    let state = HomingState::default();
    state.note_arm(MCU, 7, 1.0);
    state.note_arm(MCU, 0, 9.0);
    assert_eq!(state.take_arm_window_start(&[(MCU, 0)]), Some(9.0));
}

#[test]
fn window_start_distinguishes_the_same_endstop_id_on_two_mcus() {
    let state = HomingState::default();
    state.note_arm(1, 0, 4.0);
    state.note_arm(2, 0, 8.0);
    assert_eq!(state.take_arm_window_start(&[(2, 0)]), Some(8.0));
}

#[test]
fn rearming_replaces_the_previous_arm_time() {
    let state = HomingState::default();
    state.note_arm(MCU, 0, 1.0);
    state.note_arm(MCU, 0, 20.0);
    assert_eq!(state.take_arm_window_start(&[(MCU, 0)]), Some(20.0));
}

#[test]
fn window_start_is_absent_when_no_run_endstop_was_armed() {
    let state = HomingState::default();
    state.note_arm(MCU, 9, 1.0);
    assert_eq!(state.take_arm_window_start(&[(MCU, 0)]), None);
}

#[test]
fn taking_the_window_drains_every_recorded_arm() {
    let state = HomingState::default();
    state.note_arm(MCU, 0, 1.0);
    state.note_arm(MCU, 1, 2.0);
    state.take_arm_window_start(&[(MCU, 0)]);
    assert!(state.recent_arms.lock_ok().is_empty());
    assert_eq!(state.take_arm_window_start(&[(MCU, 1)]), None);
}

#[test]
fn arming_drops_a_trip_buffered_before_the_arm() {
    let state = HomingState::default();
    state.pending_trips.lock_ok().push((MCU, 0, 1234));
    state.note_arm(MCU, 0, 5.0);
    assert!(state.pending_trips.lock_ok().is_empty());
}

#[test]
fn arming_keeps_a_trip_buffered_for_another_endstop() {
    let state = HomingState::default();
    state.pending_trips.lock_ok().push((MCU, 1, 1234));
    state.note_arm(MCU, 0, 5.0);
    assert_eq!(*state.pending_trips.lock_ok(), vec![(MCU, 1, 1234)]);
}

#[test]
fn a_trip_buffered_after_the_arm_survives_until_the_run_consumes_it() {
    let state = HomingState::default();
    state.note_arm(MCU, 0, 5.0);
    state.pending_trips.lock_ok().push((MCU, 0, 4321));
    assert_eq!(state.take_arm_window_start(&[(MCU, 0)]), Some(5.0));
    assert_eq!(*state.pending_trips.lock_ok(), vec![(MCU, 0, 4321)]);
}
