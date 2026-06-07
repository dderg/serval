use super::*;

const V_MIN: u32 = 10 << 16;

fn cfg(kind: SourceKind, policy: ArmPolicy, sample_n: u8, gpio: PinId) -> SourceConfig {
    SourceConfig {
        kind,
        gpio,
        active_high: true,
        policy,
        sample_n,
        velocity_axis: VelocityAxis::X,
        v_min_q16: V_MIN,
    }
}

fn msg(source: SourceConfig) -> ArmMsg {
    let mut sources = [SourceConfig::EMPTY; MAX_SOURCES];
    sources[0] = source;
    ArmMsg {
        arm_id: 42,
        arm_clock: 0,
        source_count: 1,
        sources,
        stepper_count: 2,
        stepper_oids: [0, 1, 0, 0, 0, 0, 0, 0],
        grant_ticks: 0,
    }
}

fn sw_msg(grant_ticks: u64) -> ArmMsg {
    let mut sources = [SourceConfig::EMPTY; MAX_SOURCES];
    sources[0] = SourceConfig {
        kind: SourceKind::Software,
        gpio: 0,
        active_high: true,
        policy: ArmPolicy::TripImmediately,
        sample_n: 1,
        velocity_axis: VelocityAxis::XYZ,
        v_min_q16: 0,
    };
    ArmMsg {
        arm_id: 42,
        arm_clock: 0,
        source_count: 1,
        sources,
        stepper_count: 2,
        stepper_oids: [0, 1, 0, 0, 0, 0, 0, 0],
        grant_ticks,
    }
}

fn reset() -> std::sync::MutexGuard<'static, ()> {
    test_guard()
}

fn drain_trip() -> TripEvent {
    poll_trip().expect("trip event")
}

#[test]
fn source_policy_sample_matrix() {
    for kind in [SourceKind::Physical, SourceKind::TmcDiag] {
        for policy in [
            ArmPolicy::TripImmediately,
            ArmPolicy::WaitForClear,
            ArmPolicy::IgnoreUntilMoving,
        ] {
            for sample_n in [1, 3] {
                let _guard = reset();
                let source = cfg(kind, policy, sample_n, 1);
                arm(msg(source)).expect("arm");
                set_pin_level(1, true);
                if policy == ArmPolicy::WaitForClear {
                    assert_eq!(tick(1, [V_MIN, 0, 0], &[10, 20]), TripAction::Continue);
                    set_pin_level(1, false);
                    assert_eq!(tick(2, [V_MIN, 0, 0], &[10, 20]), TripAction::Continue);
                    set_pin_level(1, true);
                } else if policy == ArmPolicy::IgnoreUntilMoving {
                    assert_eq!(tick(1, [V_MIN, 0, 0], &[10, 20]), TripAction::Continue);
                    set_pin_level(1, false);
                    assert_eq!(tick(2, [V_MIN, 0, 0], &[10, 20]), TripAction::Continue);
                    set_pin_level(1, true);
                }

                for i in 1..=sample_n {
                    let action = tick(10 + u64::from(i), [V_MIN, 0, 0], &[10, 20]);
                    // Siren disabled → all samples Continue, including the terminal one.
                    assert_eq!(action, TripAction::Continue);
                    if i == sample_n {
                        let evt = drain_trip();
                        assert_eq!(evt.trip_source_idx, 0);
                    }
                }
            }
        }
    }
}

#[test]
fn ignore_until_moving_latch_requires_velocity_then_clear_once() {
    let _guard = reset();
    arm(msg(cfg(
        SourceKind::TmcDiag,
        ArmPolicy::IgnoreUntilMoving,
        1,
        2,
    )))
    .expect("arm");

    set_pin_level(2, true);
    assert_eq!(tick(1, [V_MIN - 1, 0, 0], &[1]), TripAction::Continue);
    assert_eq!(tick(2, [V_MIN, 0, 0], &[1]), TripAction::Continue);
    set_pin_level(2, false);
    assert_eq!(tick(3, [V_MIN, 0, 0], &[1]), TripAction::Continue);
    set_pin_level(2, true);
    // Siren disabled: fresh GPIO detection returns Continue, trip still queued.
    assert_eq!(tick(4, [V_MIN, 0, 0], &[1]), TripAction::Continue);
    assert_eq!(drain_trip().trip_clock, 4);

    reset_for_test();
    arm(msg(cfg(
        SourceKind::TmcDiag,
        ArmPolicy::IgnoreUntilMoving,
        1,
        2,
    )))
    .expect("arm");
    set_pin_level(2, false);
    assert_eq!(tick(1, [V_MIN, 0, 0], &[1]), TripAction::Continue);
    set_pin_level(2, true);
    // Siren disabled: fresh GPIO detection returns Continue, trip still queued.
    assert_eq!(tick(2, [0, 0, 0], &[1]), TripAction::Continue);
    assert!(poll_trip().is_some());
}

#[test]
fn wait_for_clear_ignores_assertion_at_arm() {
    let _guard = reset();
    arm(msg(cfg(
        SourceKind::Physical,
        ArmPolicy::WaitForClear,
        1,
        3,
    )))
    .expect("arm");
    set_pin_level(3, true);
    assert_eq!(tick(1, [0, 0, 0], &[1]), TripAction::Continue);
    set_pin_level(3, false);
    assert_eq!(tick(2, [0, 0, 0], &[1]), TripAction::Continue);
    set_pin_level(3, true);
    // Siren disabled: fresh GPIO detection returns Continue, trip still queued.
    assert_eq!(tick(3, [0, 0, 0], &[1]), TripAction::Continue);
    assert!(poll_trip().is_some());
}

#[test]
fn trip_immediately_assertion_at_arm_trips_on_first_sample() {
    let _guard = reset();
    arm(msg(cfg(
        SourceKind::Physical,
        ArmPolicy::TripImmediately,
        1,
        4,
    )))
    .expect("arm");
    set_pin_level(4, true);
    // Siren disabled: fresh GPIO detection returns Continue, trip still queued.
    assert_eq!(tick(1, [0, 0, 0], &[1]), TripAction::Continue);
    assert!(poll_trip().is_some());
}

#[test]
fn arm_policy_try_from_decodes_known_variants_and_rejects_others() {
    assert_eq!(ArmPolicy::try_from(0).unwrap(), ArmPolicy::TripImmediately);
    assert_eq!(ArmPolicy::try_from(1).unwrap(), ArmPolicy::WaitForClear);
    assert_eq!(
        ArmPolicy::try_from(2).unwrap(),
        ArmPolicy::IgnoreUntilMoving
    );
    assert_eq!(ArmPolicy::try_from(3).unwrap_err(), 3);
    assert_eq!(ArmPolicy::try_from(255).unwrap_err(), 255);
}

#[test]
fn unknown_policy_byte_falls_back_to_trip_immediately_behavior() {
    let _guard = reset();
    arm(msg(cfg(
        SourceKind::Physical,
        ArmPolicy::TripImmediately,
        1,
        4,
    )))
    .expect("arm");
    ARM.sources[0].policy.store(99, Ordering::Release);
    set_pin_level(4, true);
    // Siren disabled: fresh GPIO detection returns Continue, trip still queued.
    assert_eq!(tick(1, [0, 0, 0], &[1]), TripAction::Continue);
    assert!(poll_trip().is_some());
}

#[test]
fn multi_source_or_reports_first_asserted_source_index() {
    let _guard = reset();
    let mut sources = [SourceConfig::EMPTY; MAX_SOURCES];
    sources[0] = cfg(SourceKind::Physical, ArmPolicy::TripImmediately, 1, 5);
    sources[1] = cfg(SourceKind::Physical, ArmPolicy::TripImmediately, 1, 6);
    arm(ArmMsg {
        arm_id: 77,
        arm_clock: 0,
        source_count: 2,
        sources,
        stepper_count: 2,
        stepper_oids: [0, 1, 0, 0, 0, 0, 0, 0],
        grant_ticks: 0,
    })
    .expect("arm");
    set_pin_level(6, true);
    // Siren disabled: fresh GPIO detection returns Continue, trip still queued.
    assert_eq!(tick(1, [0, 0, 0], &[100, -200]), TripAction::Continue);
    let evt = drain_trip();
    assert_eq!(evt.arm_id, 77);
    assert_eq!(evt.trip_source_idx, 1);
    assert_eq!(evt.stepper_count, 2);
    assert_eq!(evt.steppers[0].oid, 0);
    assert_eq!(evt.steppers[0].step_count, 100);
    assert_eq!(evt.steppers[1].oid, 1);
    assert_eq!(evt.steppers[1].step_count, -200);
}

#[test]
fn future_arm_clock_ignores_early_assertions() {
    let _guard = reset();
    let mut m = msg(cfg(SourceKind::Physical, ArmPolicy::TripImmediately, 1, 7));
    m.arm_clock = 50;
    arm(m).expect("arm");
    set_pin_level(7, true);
    assert_eq!(tick(49, [0, 0, 0], &[1]), TripAction::Continue);
    assert!(poll_trip().is_none());
    // Siren disabled: fresh GPIO detection returns Continue, trip still queued.
    assert_eq!(tick(50, [0, 0, 0], &[2]), TripAction::Continue);
    assert_eq!(drain_trip().trip_clock, 50);
}

#[test]
fn tick_returns_continue_for_non_armed_non_tripped_states() {
    let _guard = reset();
    set_pin_level(8, true);
    for state in [ArmState::Idle, ArmState::TrippedSent, ArmState::Disarmed] {
        ARM.state.store(state as u8, Ordering::Release);
        assert_eq!(tick(1, [0, 0, 0], &[1]), TripAction::Continue);
    }
}

/// Tripping/TrippedReady WITHOUT the freeze latch must return Continue.
/// The latch is the sole AbortNow trigger; GPIO-detected states alone never freeze.
#[test]
fn tick_returns_continue_for_tripped_states_without_latch() {
    let _guard = reset();
    for state in [ArmState::Tripping, ArmState::TrippedReady] {
        FREEZE_LATCH.store(false, Ordering::Release);
        ARM.state.store(state as u8, Ordering::Release);
        assert_eq!(
            tick(1, [0, 0, 0], &[1]),
            TripAction::Continue,
            "tick() must return Continue when state={state:?} and latch not set"
        );
    }
}

/// Freeze latch set → AbortNow regardless of ARM state.
#[test]
fn tick_returns_abort_when_freeze_latch_set() {
    let _guard = reset();
    FREEZE_LATCH.store(true, Ordering::Release);
    for state in [
        ArmState::Idle,
        ArmState::Armed,
        ArmState::Tripping,
        ArmState::TrippedReady,
        ArmState::TrippedSent,
        ArmState::Disarmed,
    ] {
        ARM.state.store(state as u8, Ordering::Release);
        assert_eq!(
            tick(1, [0, 0, 0], &[1]),
            TripAction::AbortNow,
            "tick() must return AbortNow when freeze latch set, state={state:?}"
        );
    }
}

#[test]
fn exactly_one_terminal_for_trip_vs_disarm_schedules() {
    let _guard = reset();
    arm(msg(cfg(
        SourceKind::Physical,
        ArmPolicy::TripImmediately,
        1,
        9,
    )))
    .expect("arm");
    set_pin_level(9, true);

    let disarm_first = disarm(42);
    assert_eq!(disarm_first, DisarmStatus::Disarmed);
    assert_eq!(tick(1, [0, 0, 0], &[1]), TripAction::Continue);
    assert!(poll_trip().is_none());

    reset_for_test();
    arm(msg(cfg(
        SourceKind::Physical,
        ArmPolicy::TripImmediately,
        1,
        9,
    )))
    .expect("arm");
    set_pin_level(9, true);
    // Siren disabled: fresh GPIO detection returns Continue, trip still queued.
    assert_eq!(tick(1, [0, 0, 0], &[1]), TripAction::Continue);
    assert_eq!(disarm(42), DisarmStatus::AlreadyTripped);
    assert!(poll_trip().is_some());
}

#[test]
fn snapshot_seqlock_reader_retries_odd_and_never_returns_torn_read() {
    let _guard = reset();
    arm(msg(cfg(
        SourceKind::Physical,
        ArmPolicy::TripImmediately,
        1,
        10,
    )))
    .expect("arm");
    set_pin_level(10, true);
    // Siren disabled: fresh GPIO detection returns Continue, trip still queued.
    assert_eq!(
        tick(0x1_0000_0002, [0, 0, 0], &[123, 456]),
        TripAction::Continue
    );
    let evt = drain_trip();
    assert_eq!(evt.trip_clock, 0x1_0000_0002);
    assert_eq!(evt.steppers[0].step_count, 123);
    assert_eq!(evt.steppers[1].step_count, 456);
}

#[test]
fn active_low_polarity_uses_explicit_branch_not_xor() {
    let _guard = reset();
    let mut source = cfg(SourceKind::Physical, ArmPolicy::TripImmediately, 1, 11);
    source.active_high = false;
    // For active-low, HIGH = not asserted. Set HIGH before arming so arm()
    // does not see an asserted pin and immediately return AlreadyTripped.
    set_pin_level(11, true);
    arm(msg(source)).expect("arm");
    assert_eq!(tick(1, [0, 0, 0], &[1]), TripAction::Continue);
    set_pin_level(11, false);
    // Siren disabled: fresh GPIO detection returns Continue, trip still queued.
    assert_eq!(tick(2, [0, 0, 0], &[1]), TripAction::Continue);
    assert!(poll_trip().is_some());
}

#[test]
fn already_tripped_at_arm_time_active_high() {
    let _guard = reset();
    set_pin_level(12, true);
    let result = arm(msg(cfg(
        SourceKind::Physical,
        ArmPolicy::TripImmediately,
        1,
        12,
    )));
    assert_eq!(result, Ok(ArmStatus::AlreadyTripped));
    let evt = poll_trip().expect("trip event after AlreadyTripped");
    assert_eq!(evt.arm_id, 42);
    assert_eq!(evt.trip_source_idx, 0);
    assert_eq!(tick(1, [0, 0, 0], &[1]), TripAction::Continue);
}

#[test]
fn already_tripped_requires_trip_immediately_policy() {
    let _guard = reset();
    set_pin_level(13, true);
    let result = arm(msg(cfg(
        SourceKind::Physical,
        ArmPolicy::WaitForClear,
        1,
        13,
    )));
    assert_eq!(result, Ok(ArmStatus::Armed));
}

#[test]
fn software_source_does_not_trip_on_gpio() {
    let _guard = reset();
    arm(sw_msg(1000)).expect("arm");
    for i in 0..20_u16 {
        set_pin_level(i, true);
    }
    assert_eq!(tick(1, [0, 0, 0], &[1, 2]), TripAction::Continue);
    assert!(ARM.deadline_active.load(Ordering::Acquire));
}

#[test]
fn software_source_deadline_expires_and_trips() {
    let _guard = reset();
    arm(sw_msg(100)).expect("arm");
    assert_eq!(tick(1, [0, 0, 0], &[10, 20]), TripAction::Continue);
    assert!(ARM.deadline_active.load(Ordering::Acquire));
    assert_eq!(tick(100, [0, 0, 0], &[10, 20]), TripAction::Continue);
    assert_eq!(tick(101, [0, 0, 0], &[10, 20]), TripAction::AbortNow);
    let evt = drain_trip();
    assert_eq!(evt.arm_id, 42);
    assert_eq!(evt.trip_source_idx, TRIP_SOURCE_DEADLINE_EXPIRED);
    assert_eq!(evt.trip_clock, 101);
}

#[test]
fn extend_deadline_pushes_window_forward() {
    let _guard = reset();
    arm(sw_msg(100)).expect("arm");
    assert_eq!(tick(1, [0, 0, 0], &[]), TripAction::Continue);
    extend_deadline(42, 50);
    assert_eq!(ARM.deadline_clock_unchecked(), 150);
    assert_eq!(tick(101, [0, 0, 0], &[]), TripAction::Continue);
    assert_eq!(tick(150, [0, 0, 0], &[]), TripAction::AbortNow);
    assert_eq!(drain_trip().trip_source_idx, TRIP_SOURCE_DEADLINE_EXPIRED);
}

#[test]
fn extend_deadline_ignored_for_wrong_arm_id() {
    let _guard = reset();
    arm(sw_msg(100)).expect("arm");
    assert_eq!(tick(1, [0, 0, 0], &[]), TripAction::Continue);
    let deadline_before = ARM.deadline_clock_unchecked();
    extend_deadline(99, 50);
    assert_eq!(ARM.deadline_clock_unchecked(), deadline_before);
}

#[test]
fn extend_deadline_ignored_before_first_tick() {
    let _guard = reset();
    arm(sw_msg(100)).expect("arm");
    assert!(!ARM.deadline_active.load(Ordering::Acquire));
    extend_deadline(42, 50);
    assert!(!ARM.deadline_active.load(Ordering::Acquire));
    assert_eq!(ARM.deadline_clock_unchecked(), 0);
}

#[test]
fn software_trip_transitions_armed_to_tripped_ready() {
    let _guard = reset();
    arm(sw_msg(10_000)).expect("arm");
    assert_eq!(software_trip(42, 500, &[10, 20]), TripResult::Tripped);
    let evt = drain_trip();
    assert_eq!(evt.arm_id, 42);
    assert_eq!(evt.trip_source_idx, TRIP_SOURCE_SOFTWARE);
    assert_eq!(evt.trip_clock, 500);
}

#[test]
fn software_trip_wrong_arm_id_is_no_op() {
    let _guard = reset();
    arm(sw_msg(10_000)).expect("arm");
    assert_eq!(software_trip(99, 500, &[10, 20]), TripResult::WrongArmId);
    assert!(matches_u8(
        ARM.state.load(Ordering::Acquire),
        ArmState::Armed
    ));
}

#[test]
fn software_trip_on_non_armed_state_is_not_armed() {
    let _guard = reset();
    // Use arm_id=0 to match reset state so the state check (NotArmed) fires
    // before the arm_id check (WrongArmId) — both values must agree.
    ARM.arm_id.store(0, Ordering::Release);
    ARM.state.store(ArmState::Disarmed as u8, Ordering::Release);
    assert_eq!(software_trip(0, 500, &[]), TripResult::NotArmed);
}

/// After a first software_trip (Armed → TrippedReady + latch), a second
/// software_trip with the same arm_id returns Tripped via the already-tripped
/// branch (latch-only, no second snapshot). This is the relay-arrives-twice case.
#[test]
fn software_trip_second_call_already_tripped_returns_tripped() {
    let _guard = reset();
    arm(sw_msg(10_000)).expect("arm");
    assert_eq!(software_trip(42, 1, &[]), TripResult::Tripped);
    assert!(FREEZE_LATCH.load(Ordering::Acquire), "latch set after first trip");
    // State is now TrippedReady with matching arm_id → already-tripped branch.
    assert_eq!(software_trip(42, 2, &[]), TripResult::Tripped);
    // No duplicate event queued; the latch remains set.
    assert!(FREEZE_LATCH.load(Ordering::Acquire));
}

#[test]
fn deadline_active_false_resets_across_arm_calls() {
    let _guard = reset();
    arm(sw_msg(100)).expect("arm");
    tick(1, [0, 0, 0], &[]);
    assert!(ARM.deadline_active.load(Ordering::Acquire));
    disarm(42);
    arm(sw_msg(100)).expect("arm");
    assert!(
        !ARM.deadline_active.load(Ordering::Acquire),
        "deadline_active must be cleared on re-arm"
    );
}

#[test]
fn software_source_deadline_uses_saturating_add() {
    let _guard = reset();
    arm(sw_msg(u64::MAX)).expect("arm");
    assert_eq!(tick(1, [0, 0, 0], &[]), TripAction::Continue);
    assert_eq!(ARM.deadline_clock_unchecked(), u64::MAX);
}

#[test]
fn software_source_skips_gpio_no_gpio_trip() {
    let _guard = reset();
    let mut sources = [SourceConfig::EMPTY; MAX_SOURCES];
    sources[0] = SourceConfig {
        kind: SourceKind::Software,
        gpio: 0,
        active_high: true,
        policy: ArmPolicy::TripImmediately,
        sample_n: 1,
        velocity_axis: VelocityAxis::XYZ,
        v_min_q16: 0,
    };
    sources[1] = cfg(SourceKind::Physical, ArmPolicy::TripImmediately, 1, 15);
    arm(ArmMsg {
        arm_id: 42,
        arm_clock: 0,
        source_count: 2,
        sources,
        stepper_count: 2,
        stepper_oids: [0, 1, 0, 0, 0, 0, 0, 0],
        grant_ticks: 10_000,
    })
    .expect("arm");
    assert_eq!(tick(1, [0, 0, 0], &[]), TripAction::Continue);
    set_pin_level(15, true);
    // Siren disabled: fresh GPIO detection returns Continue, trip still queued.
    assert_eq!(tick(2, [0, 0, 0], &[]), TripAction::Continue);
    let evt = drain_trip();
    assert_eq!(evt.trip_source_idx, 1);
}

#[test]
fn software_trip_causes_tick_to_abort() {
    let _guard = reset();
    arm(sw_msg(100_000)).expect("arm");

    assert_eq!(tick(1, [0, 0, 0], &[0, 0]), TripAction::Continue);

    assert_eq!(software_trip(42, 50, &[10, 20]), TripResult::Tripped);

    assert_eq!(
        tick(51, [0, 0, 0], &[10, 20]),
        TripAction::AbortNow,
        "tick() must return AbortNow after software_trip — \
         otherwise the MCU keeps moving and crashes into the bed"
    );
}

/// Fresh GPIO detection must return `Continue` (siren disabled) AND queue
/// the trip event so the relay can observe it.
///
/// When the local endstop siren is disabled, the detecting MCU does not
/// self-freeze — it only reports the trip. The cross-MCU relay (bridge
/// reactor TripDispatch) sends trsync_trigger, which freezes via the
/// top-of-tick AbortNow path (TrippedReady|Tripping → AbortNow). That
/// relay path is tested separately and is unaffected by this change.
///
/// See docs/superpowers/specs/2026-05-31-trsync-cross-mcu-homing-design.md
#[test]
fn fresh_gpio_trip_returns_continue_and_queues_event() {
    let _guard = reset();

    // Arm a single active-high GPIO source on pin 20.
    let mut sources = [SourceConfig::EMPTY; MAX_SOURCES];
    sources[0] = SourceConfig {
        kind: SourceKind::Physical,
        gpio: 20,
        active_high: true,
        policy: ArmPolicy::TripImmediately,
        sample_n: 1,
        velocity_axis: VelocityAxis::X,
        v_min_q16: 0,
    };
    arm(ArmMsg {
        arm_id: 1,
        arm_clock: 0,
        source_count: 1,
        sources,
        stepper_count: 1,
        stepper_oids: [7, 0, 0, 0, 0, 0, 0, 0],
        grant_ticks: 0,
    })
    .expect("arm should succeed");

    // Assert the pin — source is now asserted.
    set_pin_level(20, true);

    // Tick at arm_clock (clock=0): the source should detect the assertion.
    // Siren is disabled: tick() must return Continue, NOT AbortNow.
    let action = tick(0, [0, 0, 0], &[0]);
    assert_eq!(
        action,
        TripAction::Continue,
        "fresh GPIO detection must return Continue (siren disabled); \
         got {action:?} — the local AbortNow has not been suppressed yet"
    );

    // The trip must still be reported: poll_trip() must return Some with
    // the correct arm_id so the relay can observe and dispatch it.
    let event = poll_trip().expect(
        "poll_trip() must return Some after a fresh GPIO trip — \
         the report (publish_snapshot + TRIP_EVENT_QUEUED) must still happen",
    );
    assert_eq!(
        event.arm_id, 1,
        "trip event arm_id must match the armed arm_id"
    );
    assert_eq!(
        event.trip_source_idx, 0,
        "trip event source index must be 0 (first and only source)"
    );
}

/// Same as above but for the case where software_trip arrives BEFORE
/// the first tick past arm_clock (the deadline isn't active yet).
/// tick() must still return AbortNow.
#[test]
fn software_trip_before_arm_clock_causes_tick_to_abort() {
    let _guard = reset();
    let mut msg = sw_msg(100_000);
    msg.arm_clock = 1000;
    arm(msg).expect("arm");

    assert_eq!(software_trip(42, 500, &[10, 20]), TripResult::Tripped);

    assert_eq!(
        tick(1001, [0, 0, 0], &[10, 20]),
        TripAction::AbortNow,
        "tick() must abort after software_trip even if deadline wasn't active"
    );
}

/// GPIO detection on the detecting tick → Continue + event queued.
/// Subsequent ticks (before relay arrives) → Continue, no AbortNow, no duplicate event.
/// This tests that state Tripping/TrippedReady without the latch never freezes.
#[test]
fn gpio_detection_continue_across_multiple_ticks_until_relay() {
    let _guard = reset();
    let mut sources = [SourceConfig::EMPTY; MAX_SOURCES];
    sources[0] = SourceConfig {
        kind: SourceKind::Physical,
        gpio: 21,
        active_high: true,
        policy: ArmPolicy::TripImmediately,
        sample_n: 1,
        velocity_axis: VelocityAxis::X,
        v_min_q16: 0,
    };
    arm(ArmMsg {
        arm_id: 10,
        arm_clock: 0,
        source_count: 1,
        sources,
        stepper_count: 1,
        stepper_oids: [0, 0, 0, 0, 0, 0, 0, 0],
        grant_ticks: 0,
    })
    .expect("arm");

    set_pin_level(21, true);

    // Detection tick.
    assert_eq!(tick(1, [0, 0, 0], &[1]), TripAction::Continue);
    assert!(!FREEZE_LATCH.load(Ordering::Acquire), "latch must not be set by GPIO detection");
    let evt = poll_trip().expect("event queued on detection tick");
    assert_eq!(evt.arm_id, 10);

    // Several more ticks while relay is in flight — all Continue.
    for clk in 2..=10u64 {
        assert_eq!(
            tick(clk, [0, 0, 0], &[1]),
            TripAction::Continue,
            "tick {clk} must be Continue while relay has not arrived"
        );
        assert!(poll_trip().is_none(), "no duplicate event on tick {clk}");
    }
}

/// GPIO detects first → Continue; relay software_trip arrives → Tripped (latch
/// set, no second snapshot); next tick → AbortNow.
#[test]
fn gpio_detect_then_software_trip_freezes_no_duplicate_event() {
    let _guard = reset();
    let mut sources = [SourceConfig::EMPTY; MAX_SOURCES];
    sources[0] = SourceConfig {
        kind: SourceKind::Physical,
        gpio: 22,
        active_high: true,
        policy: ArmPolicy::TripImmediately,
        sample_n: 1,
        velocity_axis: VelocityAxis::X,
        v_min_q16: 0,
    };
    arm(ArmMsg {
        arm_id: 11,
        arm_clock: 0,
        source_count: 1,
        sources,
        stepper_count: 1,
        stepper_oids: [0, 0, 0, 0, 0, 0, 0, 0],
        grant_ticks: 0,
    })
    .expect("arm");

    set_pin_level(22, true);

    // GPIO detection tick.
    assert_eq!(tick(1, [0, 0, 0], &[1]), TripAction::Continue);
    let _evt = poll_trip().expect("first event queued");

    // Relay arrives: software_trip with already-tripped state.
    let result = software_trip(11, 2, &[1]);
    assert_eq!(result, TripResult::Tripped, "relay must return Tripped even if GPIO was first");
    assert!(FREEZE_LATCH.load(Ordering::Acquire), "latch set by relay");

    // No duplicate event was queued.
    assert!(poll_trip().is_none(), "no second event after relay software_trip");

    // Next tick freezes.
    assert_eq!(tick(3, [0, 0, 0], &[1]), TripAction::AbortNow);
}

/// Full sequence: arm → GPIO detect (motion continues N ticks) →
/// software_trip (relay) → frozen → recovery → motion resumes.
#[test]
fn full_sequence_arm_gpio_relay_freeze_recovery() {
    let _guard = reset();
    let gpio_pin: PinId = 23;
    let mut sources = [SourceConfig::EMPTY; MAX_SOURCES];
    sources[0] = SourceConfig {
        kind: SourceKind::Physical,
        gpio: gpio_pin,
        active_high: true,
        policy: ArmPolicy::TripImmediately,
        sample_n: 1,
        velocity_axis: VelocityAxis::X,
        v_min_q16: 0,
    };
    arm(ArmMsg {
        arm_id: 12,
        arm_clock: 0,
        source_count: 1,
        sources,
        stepper_count: 1,
        stepper_oids: [0, 0, 0, 0, 0, 0, 0, 0],
        grant_ticks: 0,
    })
    .expect("arm");

    // Phase 1: motion with pin not asserted.
    for clk in 0..3u64 {
        assert_eq!(tick(clk, [0, 0, 0], &[1]), TripAction::Continue);
    }

    // Phase 2: GPIO detects — motion continues across multiple ticks.
    set_pin_level(gpio_pin, true);
    assert_eq!(tick(3, [0, 0, 0], &[1]), TripAction::Continue);
    let _evt = poll_trip().expect("detection event queued");
    for clk in 4..=8u64 {
        assert_eq!(
            tick(clk, [0, 0, 0], &[1]),
            TripAction::Continue,
            "motion must continue at tick {clk} before relay"
        );
    }
    assert!(!FREEZE_LATCH.load(Ordering::Acquire));

    // Phase 3: relay arrives.
    assert_eq!(software_trip(12, 9, &[1]), TripResult::Tripped);
    assert!(FREEZE_LATCH.load(Ordering::Acquire));

    // Phase 4: frozen ticks.
    assert_eq!(tick(9, [0, 0, 0], &[1]), TripAction::AbortNow);
    assert_eq!(tick(10, [0, 0, 0], &[1]), TripAction::AbortNow);

    // Phase 5: recovery — arm() clears the latch.
    disarm(12);
    arm(ArmMsg {
        arm_id: 13,
        arm_clock: 1000,
        source_count: 1,
        sources,
        stepper_count: 1,
        stepper_oids: [0, 0, 0, 0, 0, 0, 0, 0],
        grant_ticks: 0,
    })
    .expect("re-arm");
    assert!(!FREEZE_LATCH.load(Ordering::Acquire), "latch cleared by arm()");

    // Phase 6: motion resumes (arm_clock in future → Continue).
    assert_eq!(tick(11, [0, 0, 0], &[1]), TripAction::Continue);
    assert_eq!(tick(12, [0, 0, 0], &[1]), TripAction::Continue);
}

/// software_trip on an Armed state (no GPIO detected yet) sets the latch
/// and next tick returns AbortNow.
#[test]
fn software_trip_on_armed_sets_latch_and_next_tick_aborts() {
    let _guard = reset();
    arm(sw_msg(100_000)).expect("arm");
    assert!(!FREEZE_LATCH.load(Ordering::Acquire));

    assert_eq!(software_trip(42, 50, &[10, 20]), TripResult::Tripped);
    assert!(FREEZE_LATCH.load(Ordering::Acquire));

    let evt = drain_trip();
    assert_eq!(evt.trip_source_idx, TRIP_SOURCE_SOFTWARE);

    assert_eq!(tick(51, [0, 0, 0], &[10, 20]), TripAction::AbortNow);
}

/// arm() clears the freeze latch as part of the recovery cycle.
#[test]
fn arm_clears_freeze_latch_on_recovery() {
    let _guard = reset();
    arm(sw_msg(10_000)).expect("arm");
    software_trip(42, 1, &[]);
    assert!(FREEZE_LATCH.load(Ordering::Acquire));

    disarm(42);
    arm(sw_msg(10_000)).expect("re-arm");
    assert!(!FREEZE_LATCH.load(Ordering::Acquire), "arm() must clear the freeze latch");
    assert_eq!(tick(1, [0, 0, 0], &[]), TripAction::Continue, "motion resumes after recovery");
}

#[test]
fn disarm_of_frozen_arm_clears_latch_and_requests_purge() {
    let _guard = reset();
    arm(sw_msg(10_000)).expect("arm");
    software_trip(42, 1, &[]);
    assert!(FREEZE_LATCH.load(Ordering::Acquire));

    assert_eq!(disarm(42), DisarmStatus::AlreadyTripped);
    assert!(
        !FREEZE_LATCH.load(Ordering::Acquire),
        "disarm() of a frozen arm must clear the freeze latch"
    );
    assert!(
        take_ring_purge_request(),
        "frozen disarm must request a ring purge"
    );
    assert!(
        !take_ring_purge_request(),
        "purge request is one-shot"
    );
    assert_eq!(
        tick(2, [0, 0, 0], &[]),
        TripAction::Continue,
        "no AbortNow after frozen disarm — abandoned pieces are purged, not replayed"
    );
}

#[test]
fn disarm_without_freeze_requests_no_purge() {
    let _guard = reset();
    arm(sw_msg(10_000)).expect("arm");
    assert_eq!(disarm(42), DisarmStatus::Disarmed);
    assert!(
        !take_ring_purge_request(),
        "clean no-trip disarm must not purge — the move drained naturally"
    );
}

#[test]
fn disarm_unknown_arm_id_leaves_freeze_latch_alone() {
    let _guard = reset();
    arm(sw_msg(10_000)).expect("arm");
    software_trip(42, 1, &[]);
    assert_eq!(disarm(999), DisarmStatus::Unknown);
    assert!(
        FREEZE_LATCH.load(Ordering::Acquire),
        "a mismatched disarm must not unfreeze someone else's arm"
    );
    assert!(!take_ring_purge_request());
}
