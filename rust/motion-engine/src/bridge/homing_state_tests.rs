use super::endstop::{TripMatch, match_trip};
use super::homing_api::validate_trip_members;
use super::{HomingRun, HomingState, RemoteFreeze, TripMember};
use motion_core::lock_ext::LockExt;

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
    assert_eq!(state.take_arm_window_start(&[(MCU, 1)]), None);
}

#[test]
fn arming_drops_a_trip_buffered_before_the_arm() {
    let state = HomingState::default();
    state.lifecycle.lock_ok().trip_run((MCU, 0, 1234));
    state.note_arm(MCU, 0, 5.0);
    assert!(state.lifecycle.lock_ok().pending_trips.is_empty());
}

#[test]
fn arming_keeps_a_trip_buffered_for_another_endstop() {
    let state = HomingState::default();
    state.lifecycle.lock_ok().trip_run((MCU, 1, 1234));
    state.note_arm(MCU, 0, 5.0);
    assert_eq!(
        state.lifecycle.lock_ok().pending_trips,
        vec![(MCU, 1, 1234)]
    );
}

#[test]
fn a_trip_buffered_after_the_arm_survives_until_the_run_consumes_it() {
    let state = HomingState::default();
    state.note_arm(MCU, 0, 5.0);
    state.lifecycle.lock_ok().trip_run((MCU, 0, 4321));
    assert_eq!(state.take_arm_window_start(&[(MCU, 0)]), Some(5.0));
    assert_eq!(
        state.lifecycle.lock_ok().pending_trips,
        vec![(MCU, 0, 4321)]
    );
}

fn run_with(members: Vec<TripMember>) -> HomingRun {
    HomingRun {
        cohort: 1,
        remaining_trips: members,
        axis_key: motion_core::types::AxisKey {
            mcu_id: MCU,
            axis: 0,
        },
        all_axis_keys: vec![motion_core::types::AxisKey {
            mcu_id: MCU,
            axis: 0,
        }],
        window_start_host: 0.0,
        start_pos: geometry::MachinePos([0.0, 0.0, 0.0]),
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
        stepper_oid: 7,
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
    let members = vec![member(MCU, 0, None), member(MCU, 1, None)];
    assert_eq!(validate_trip_members(&members), Ok(()));
    let mut run = run_with(members);
    assert_eq!(match_trip(&mut run, MCU, 1), TripMatch::Partial(None));
}

#[test]
fn last_remaining_trip_is_final_and_carries_its_freeze_target() {
    let freeze = RemoteFreeze {
        stepper_oid: 7,
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
        stepper_oid: 7,
        motor_mcu: 2,
        motor_idx: 0,
        stepper_idx: 1,
    };
    let mut run = run_with(vec![member(1, 0, None), member(2, 0, Some(freeze))]);
    assert_eq!(match_trip(&mut run, 2, 0), TripMatch::Partial(Some(freeze)));
}

#[test]
fn same_oid_on_different_motor_mcus_freezes_each_motor_once() {
    let first = RemoteFreeze {
        stepper_oid: 7,
        motor_mcu: 1,
        motor_idx: 0,
        stepper_idx: 0,
    };
    let second = RemoteFreeze {
        motor_mcu: 2,
        ..first
    };
    let last = RemoteFreeze {
        stepper_oid: 8,
        motor_idx: 1,
        stepper_idx: 1,
        ..first
    };
    let members = vec![
        member(MCU, 0, Some(first)),
        member(MCU, 1, Some(second)),
        member(MCU, 2, Some(last)),
    ];
    assert_eq!(validate_trip_members(&members), Ok(()));
    let mut run = run_with(members);
    assert_eq!(
        match_trip(&mut run, MCU, 0),
        TripMatch::Partial(Some(first))
    );
    assert_eq!(match_trip(&mut run, MCU, 0), TripMatch::Unmatched);
    assert_eq!(
        match_trip(&mut run, MCU, 1),
        TripMatch::Partial(Some(second))
    );
    assert_eq!(match_trip(&mut run, MCU, 2), TripMatch::Final(Some(last)));
}

#[test]
fn duplicate_motor_binding_is_rejected_before_any_trip() {
    let freeze = RemoteFreeze {
        stepper_oid: 7,
        motor_mcu: 1,
        motor_idx: 0,
        stepper_idx: 0,
    };
    let members = vec![member(MCU, 0, Some(freeze)), member(MCU, 1, Some(freeze))];
    assert!(validate_trip_members(&members).is_err());
}

#[test]
fn aliased_final_motor_binding_is_rejected_before_any_trip() {
    let freeze = RemoteFreeze {
        stepper_oid: 7,
        motor_mcu: 1,
        motor_idx: 0,
        stepper_idx: 0,
    };
    let alias = RemoteFreeze {
        motor_idx: 1,
        stepper_idx: 2,
        ..freeze
    };
    let members = vec![
        member(MCU, 0, Some(freeze)),
        member(MCU, 1, None),
        member(MCU, 2, Some(alias)),
    ];
    assert!(validate_trip_members(&members).is_err());
}

#[test]
fn drive_fault_retains_its_result_until_polled() {
    let state = HomingState::default();
    state.begin(1, MCU).unwrap();
    state.arm(1).unwrap();
    state
        .register(run_with(vec![member(MCU, 0, None)]))
        .unwrap();
    let run = state.interrupt(Some(MCU), "drive fault".into()).unwrap();
    assert!(
        state
            .interrupt(Some(MCU), "second drive fault".into())
            .is_none()
    );
    assert!(state.begin(2, MCU).is_err());
    state.complete(run.cohort, Err("drive fault".into()));
    assert_eq!(state.poll().unwrap(), Some(Err("drive fault".into())));
    assert!(state.poll().is_err());
    state.begin(2, MCU).unwrap();
}

#[test]
fn trips_during_terminal_work_are_not_replayed_into_the_next_run() {
    let state = HomingState::default();
    state.begin(1, MCU).unwrap();
    state.arm(1).unwrap();
    state
        .register(run_with(vec![member(MCU, 0, None)]))
        .unwrap();
    let run = state.interrupt(None, "aborted".into()).unwrap();
    assert!(state.lifecycle.lock_ok().trip_run((MCU, 0, 123)).is_none());
    state.complete(run.cohort, Err("aborted".into()));
    assert!(state.lifecycle.lock_ok().trip_run((MCU, 0, 456)).is_none());
    assert!(state.lifecycle.lock_ok().pending_trips.is_empty());
    state.poll().unwrap();
    state.begin(2, MCU).unwrap();
    assert!(state.lifecycle.lock_ok().pending_trips.is_empty());
}

#[test]
fn terminal_error_is_delivered_once_before_partial_work_retires() {
    let state = HomingState::default();
    let run = run_with(vec![member(MCU, 0, None)]);
    state.lifecycle.lock_ok().pending_suppresses = 2;
    state.begin(1, MCU).unwrap();
    state.arm(1).unwrap();
    state.register(run).unwrap();
    state.lifecycle.lock_ok().take_terminal(|_| true).unwrap();
    state.complete(1, Err("suppress timeout".into()));
    state.retire_partial(1, Some("first late failure".into()));
    assert_eq!(state.poll().unwrap(), Some(Err("suppress timeout".into())));
    assert_eq!(state.poll().unwrap(), None);
    assert!(state.begin(2, MCU).is_err());
    assert!(state.interrupt(None, "aborted".into()).is_none());
    assert!(state.begin(2, MCU).is_err());
    assert!(state.lifecycle.lock_ok().trip_run((MCU, 0, 456)).is_none());
    assert!(
        state
            .retire_partial(1, Some("second late failure".into()))
            .is_none()
    );
    assert!(state.poll().is_err());
    state.begin(2, MCU).unwrap();
    state.arm(2).unwrap();
    let mut next_run = run_with(vec![member(MCU, 0, None)]);
    next_run.cohort = 2;
    state.register(next_run).unwrap();
    assert_eq!(state.poll().unwrap(), None);
    assert!(state.lifecycle.lock_ok().pending_trips.is_empty());
}

#[test]
fn successful_result_waits_until_partial_work_retires() {
    let state = HomingState::default();
    let run = run_with(vec![member(MCU, 0, None)]);
    state.lifecycle.lock_ok().pending_suppresses = 1;
    state.begin(1, MCU).unwrap();
    state.arm(1).unwrap();
    state.register(run).unwrap();
    state.lifecycle.lock_ok().take_terminal(|_| true).unwrap();
    let position = geometry::MachinePos([1.0, 2.0, 3.0]);
    state.complete(1, Ok((position, position, 2)));
    assert_eq!(state.poll().unwrap(), None);
    assert!(state.begin(2, MCU).is_err());
    assert!(state.retire_partial(1, None).is_none());
    assert_eq!(state.poll().unwrap(), Some(Ok((position, position, 2))));
    assert!(state.poll().is_err());
    state.begin(2, MCU).unwrap();
}

#[test]
fn abort_during_registration_prevents_the_run_from_becoming_active() {
    let state = HomingState::default();
    state.begin(1, MCU).unwrap();
    state.arm(1).unwrap();
    assert!(state.abort().is_none());
    assert!(
        state
            .register(run_with(vec![member(MCU, 0, None)]))
            .is_err()
    );
    state.cancel_registration();
    state.begin(2, MCU).unwrap();
}

#[test]
fn unrelated_drive_fault_does_not_claim_the_homing_run() {
    let state = HomingState::default();
    state.begin(1, MCU).unwrap();
    state.arm(1).unwrap();
    state
        .register(run_with(vec![member(MCU, 0, None)]))
        .unwrap();
    assert!(
        state
            .interrupt(Some(MCU + 1), "unrelated fault".into())
            .is_none()
    );
    assert!(state.lifecycle.lock_ok().trip_run((MCU, 0, 123)).is_some());
}

#[test]
fn final_trip_claims_completion_without_unpublishing_a_partial_run() {
    let state = HomingState::default();
    state.begin(1, MCU).unwrap();
    state.arm(1).unwrap();
    state
        .register(run_with(vec![member(MCU, 0, None), member(MCU, 1, None)]))
        .unwrap();
    let run = {
        let mut lifecycle = state.lifecycle.lock_ok();
        let run = lifecycle.trip_run((MCU, 0, 1)).unwrap();
        assert_eq!(match_trip(run, MCU, 0), TripMatch::Partial(None));
        let run = lifecycle.trip_run((MCU, 1, 2)).unwrap();
        assert_eq!(match_trip(run, MCU, 1), TripMatch::Final(None));
        lifecycle.take_terminal(|_| true).unwrap()
    };
    let position = geometry::MachinePos([1.0, 2.0, 3.0]);
    state.complete(run.cohort, Ok((position, position, 2)));
    assert_eq!(state.poll().unwrap(), Some(Ok((position, position, 2))));
    assert!(state.poll().is_err());
}

#[test]
fn partial_failure_replaces_success_before_the_result_can_be_polled() {
    let state = HomingState::default();
    state.begin(1, MCU).unwrap();
    state.arm(1).unwrap();
    state
        .register(run_with(vec![member(MCU, 0, None)]))
        .unwrap();
    state.lifecycle.lock_ok().pending_suppresses = 1;
    state.lifecycle.lock_ok().take_terminal(|_| true).unwrap();
    let position = geometry::MachinePos([1.0, 2.0, 3.0]);
    state.complete(1, Ok((position, position, 2)));
    assert_eq!(state.poll().unwrap(), None);
    assert!(
        state
            .retire_partial(1, Some("suppression failed".into()))
            .is_none()
    );
    assert_eq!(
        state.poll().unwrap(),
        Some(Err("suppression failed".into()))
    );
    state.begin(2, MCU).unwrap();
}

#[test]
fn terminal_wait_releases_lifecycle_for_partial_callback() {
    let state = HomingState::default();
    state.begin(1, MCU).unwrap();
    state.arm(1).unwrap();
    state
        .register(run_with(vec![member(MCU, 0, None)]))
        .unwrap();
    state.lifecycle.lock_ok().pending_suppresses = 1;
    let run = state.interrupt(None, "aborted".into()).unwrap();
    std::thread::scope(|scope| {
        let waiter = scope.spawn(|| state.wait_for_pending_suppresses(run.cohort));
        assert!(state.retire_partial(run.cohort, None).is_none());
        waiter.join().unwrap().unwrap();
    });
    state.complete(run.cohort, Err("aborted".into()));
    assert_eq!(state.poll().unwrap(), Some(Err("aborted".into())));
    state.begin(2, MCU).unwrap();
}

#[test]
fn abort_of_active_run_keeps_recovery_owned_until_completion() {
    let state = HomingState::default();
    state.begin(1, MCU).unwrap();
    state.arm(1).unwrap();
    state
        .register(run_with(vec![member(MCU, 0, None)]))
        .unwrap();
    let run = state.abort().unwrap();
    assert!(state.abort().is_none());
    assert!(state.interrupt(Some(MCU), "drive fault".into()).is_none());
    assert!(state.begin(2, MCU).is_err());
    state.complete(run.cohort, Err("aborted".into()));
    state.begin(2, MCU).unwrap();
    assert_eq!(state.poll().unwrap(), None);
}

#[test]
fn abort_during_completion_releases_the_result_without_polling() {
    let state = HomingState::default();
    state.begin(1, MCU).unwrap();
    state.arm(1).unwrap();
    state
        .register(run_with(vec![member(MCU, 0, None)]))
        .unwrap();
    let run = state.lifecycle.lock_ok().take_terminal(|_| true).unwrap();
    assert!(state.abort().is_none());
    assert!(state.begin(2, MCU).is_err());
    let position = geometry::MachinePos([1.0, 2.0, 3.0]);
    state.complete(run.cohort, Ok((position, position, 2)));
    state.begin(2, MCU).unwrap();
    assert_eq!(state.poll().unwrap(), None);
}

#[test]
fn abort_after_completion_releases_the_result_without_polling() {
    let state = HomingState::default();
    state.begin(1, MCU).unwrap();
    state.arm(1).unwrap();
    state
        .register(run_with(vec![member(MCU, 0, None)]))
        .unwrap();
    let run = state.lifecycle.lock_ok().take_terminal(|_| true).unwrap();
    let position = geometry::MachinePos([1.0, 2.0, 3.0]);
    state.complete(run.cohort, Ok((position, position, 2)));
    assert!(state.begin(2, MCU).is_err());
    assert!(state.abort().is_none());
    state.begin(2, MCU).unwrap();
    assert_eq!(state.poll().unwrap(), None);
}

#[test]
fn abort_waits_for_every_partial_callback_after_terminal_completion() {
    for abort_before_completion in [true, false] {
        let state = HomingState::default();
        state.begin(1, MCU).unwrap();
        state.arm(1).unwrap();
        state
            .register(run_with(vec![member(MCU, 0, None)]))
            .unwrap();
        state.lifecycle.lock_ok().pending_suppresses = 2;
        let run = state.lifecycle.lock_ok().take_terminal(|_| true).unwrap();
        if abort_before_completion {
            assert!(state.abort().is_none());
        }
        let position = geometry::MachinePos([1.0, 2.0, 3.0]);
        state.complete(run.cohort, Ok((position, position, 2)));
        if !abort_before_completion {
            assert!(state.abort().is_none());
        }
        assert!(state.begin(2, MCU).is_err());
        assert!(state.retire_partial(run.cohort, None).is_none());
        assert!(state.begin(2, MCU).is_err());
        assert!(
            state
                .retire_partial(run.cohort, Some("late suppression failure".into()))
                .is_none()
        );
        state.begin(2, MCU).unwrap();
        state.arm(2).unwrap();
        let mut next_run = run_with(vec![member(MCU, 0, None)]);
        next_run.cohort = 2;
        state.register(next_run).unwrap();
        assert_eq!(state.poll().unwrap(), None);
    }
}

#[test]
fn abort_with_retired_partial_work_still_waits_for_terminal_completion() {
    let state = HomingState::default();
    state.begin(1, MCU).unwrap();
    state.arm(1).unwrap();
    state
        .register(run_with(vec![member(MCU, 0, None)]))
        .unwrap();
    state.lifecycle.lock_ok().pending_suppresses = 1;
    let run = state.abort().unwrap();
    assert!(
        state
            .retire_partial(run.cohort, Some("late suppression failure".into()))
            .is_none()
    );
    assert!(state.begin(2, MCU).is_err());
    state.complete(run.cohort, Err("aborted".into()));
    state.begin(2, MCU).unwrap();
    assert_eq!(state.poll().unwrap(), None);
}
