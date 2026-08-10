use super::endstop::{TripMatch, match_trip};
use super::{HomingRun, HomingState, RemoteFreeze, TripMember};
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

fn run_with(members: Vec<TripMember>) -> HomingRun {
    let (tx, _rx) = crossbeam_channel::bounded(1);
    HomingRun {
        cohort: 1,
        remaining_trips: members,
        axis_key: crate::types::AxisKey {
            mcu_id: MCU,
            axis: 0,
        },
        all_axis_keys: vec![crate::types::AxisKey {
            mcu_id: MCU,
            axis: 0,
        }],
        window_start_host: 0.0,
        start_pos: geometry::MachinePos([0.0, 0.0, 0.0]),
        notify: tx,
        pending_suppresses: std::sync::Arc::new((
            std::sync::Mutex::new(0),
            std::sync::Condvar::new(),
        )),
    }
}

fn member(mcu: u32, id: u8, freeze: Option<RemoteFreeze>) -> TripMember {
    TripMember {
        endstop_mcu: mcu,
        endstop_id: id,
        remote_freeze: freeze,
    }
}

#[test]
fn non_final_trip_yields_its_remote_freeze_target_and_leaves_the_rest() {
    let freeze = RemoteFreeze {
        motor_mcu: 7,
        motor_idx: 1,
        stepper_idx: 2,
    };
    let mut run = run_with(vec![member(MCU, 0, Some(freeze)), member(4, 3, None)]);
    assert_eq!(
        match_trip(&mut run, MCU, 0),
        TripMatch::Partial(Some(freeze))
    );
    assert_eq!(run.remaining_trips, vec![member(4, 3, None)]);
}

#[test]
fn non_final_trip_without_binding_carries_no_freeze_target() {
    let mut run = run_with(vec![member(MCU, 0, None), member(MCU, 1, None)]);
    assert_eq!(match_trip(&mut run, MCU, 1), TripMatch::Partial(None));
}

#[test]
fn last_remaining_trip_is_final_and_carries_its_freeze_target() {
    let freeze = RemoteFreeze {
        motor_mcu: 7,
        motor_idx: 0,
        stepper_idx: 0,
    };
    let mut run = run_with(vec![member(MCU, 0, Some(freeze))]);
    assert_eq!(match_trip(&mut run, MCU, 0), TripMatch::Final(Some(freeze)));
    assert_eq!(run.remaining_trips.len(), 1);
}

#[test]
fn trip_from_an_unknown_endstop_is_unmatched_and_removes_nothing() {
    let mut run = run_with(vec![member(MCU, 0, None), member(MCU, 1, None)]);
    assert_eq!(match_trip(&mut run, 9, 0), TripMatch::Unmatched);
    assert_eq!(run.remaining_trips.len(), 2);
}

#[test]
fn trip_identity_distinguishes_same_endstop_id_across_mcus() {
    let freeze = RemoteFreeze {
        motor_mcu: 2,
        motor_idx: 0,
        stepper_idx: 1,
    };
    let mut run = run_with(vec![member(1, 0, None), member(2, 0, Some(freeze))]);
    assert_eq!(match_trip(&mut run, 2, 0), TripMatch::Partial(Some(freeze)));
}

#[test]
fn terminal_trip_waits_for_partial_suppress_completion() {
    let pending = std::sync::Arc::new((std::sync::Mutex::new(1usize), std::sync::Condvar::new()));
    let worker_pending = std::sync::Arc::clone(&pending);
    let worker = std::thread::spawn(move || {
        std::thread::sleep(std::time::Duration::from_millis(20));
        let (count, ready) = &*worker_pending;
        *count.lock_ok() = 0;
        ready.notify_all();
    });
    let started = std::time::Instant::now();
    super::endstop::wait_for_pending_suppresses(&pending).unwrap();
    worker.join().unwrap();
    assert!(started.elapsed() >= std::time::Duration::from_millis(20));
}
