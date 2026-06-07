//! ISR-level endstop integration tests: trip detector call site + freeze consumer.
//!
//! Tests drive `isr_sample_tick` (the TIM5 ISR body) and verify:
//!
//! 1. GPIO-trip detection is called each tick; the trip is reported (snapshot
//!    queued) but `Continue` is returned (siren disabled) so steps continue.
//! 2. After a `software_trip` latches `ARM.state = TrippedReady`, the next ISR
//!    tick returns `AbortNow`, `engine.tick` is skipped (zero steps dispatched),
//!    the widened clock is still published, and `last_tick_now` is cleared so the
//!    gap guard does not fire on unfreeze.
//! 3. Recovery: disarm + engine.reset + re-arm → engine.tick resumes, abandoned
//!    pieces do not replay.
//! 4. The trip snapshot's stepper_counts match `shared.stepper_counts` at the
//!    GPIO-detection tick.
//! 5. After a freeze (last_tick_now cleared), a large raw_cyccnt jump on the
//!    first post-recovery tick does NOT raise TickIntervalExceeded.
//!
//! Test style follows `tests/tick_interval_guard.rs`.

use core::sync::atomic::Ordering;

use runtime::clock::WidenState;
use runtime::endstop::{
    ArmMsg, ArmPolicy, ArmStatus, SourceConfig, SourceKind, TripEvent, VelocityAxis,
    arm, disarm, poll_trip, set_pin_level, software_trip,
};
use runtime::engine::Engine;
use runtime::piece_ring::PieceEntry;
use runtime::state::{IsrState, SharedState, TOTAL_RING_PIECES};
use runtime::step_queue::StepQueue;
use runtime::stepping_state::{MAX_AXES, StepMode, StepperBindingRust, TMC_CS_OID_NONE};
use runtime::tick::isr_sample_tick;

// 520 MHz clock, 40 kHz ISR → 13_000 cycles per tick.
const CLOCK_FREQ: u32 = 520_000_000;
const SAMPLE_RATE: u32 = 40_000;
const TICK_CYCLES: u32 = CLOCK_FREQ / SAMPLE_RATE; // 13_000

const ARM_ID: u32 = 55;
const GPIO_PIN: u16 = 10;

// ─── helpers ─────────────────────────────────────────────────────────────────

fn make_isr() -> IsrState {
    IsrState {
        engine: Engine::new(CLOCK_FREQ, SAMPLE_RATE),
        widen_state: WidenState::default(),
        last_tick_now: None,
    }
}

fn make_storage() -> Vec<PieceEntry> {
    vec![
        PieceEntry {
            start_time: 0,
            coeffs: [0.0; 4],
            duration: 0.0,
            _reserved: 0,
        };
        TOTAL_RING_PIECES
    ]
}

fn pulse_binding(stepper_oid: u8) -> StepperBindingRust {
    StepperBindingRust {
        stepper_oid,
        tmc_cs_oid: TMC_CS_OID_NONE,
        _pad: [0; 2],
    }
}

fn configure_axis(isr: &mut IsrState, axis_idx: u8, stepper_oid: u8, storage: &mut [PieceEntry]) {
    assert_eq!(
        isr.engine.configure_axis(
            axis_idx,
            StepMode::Pulse,
            0.0125,
            64,
            &[pulse_binding(stepper_oid)],
            storage.len(),
        ),
        0,
        "configure_axis {axis_idx} failed"
    );
}

fn install_queue(isr: &mut IsrState) -> Box<[StepQueue; MAX_AXES]> {
    let mut qs = Box::new(core::array::from_fn(|_| StepQueue::new()));
    let ptrs: [*mut StepQueue; MAX_AXES] =
        core::array::from_fn(|i| &mut qs[i] as *mut StepQueue);
    isr.engine.test_install_step_queues(ptrs);
    qs
}

/// A constant zero-displacement piece (no steps, but engine.tick returns active).
fn const_piece(start_time: u64, dur_s: f32) -> PieceEntry {
    PieceEntry {
        start_time,
        coeffs: [0.0; 4],
        duration: dur_s,
        _reserved: 0,
    }
}

fn push_one_piece(isr: &mut IsrState, axis: u8, piece: PieceEntry, storage: &mut [PieceEntry]) {
    assert_eq!(
        isr.engine.push_pieces(axis, &[piece], storage),
        0,
        "push_pieces axis {axis} failed"
    );
}

/// Arm a single GPIO source (TripImmediately) for ARM_ID with steppers [0, 1].
fn arm_gpio(arm_clock: u64) -> ArmStatus {
    let mut sources = [SourceConfig::EMPTY; runtime::endstop::MAX_SOURCES];
    sources[0] = SourceConfig {
        kind: SourceKind::Physical,
        gpio: GPIO_PIN,
        active_high: true,
        policy: ArmPolicy::TripImmediately,
        sample_n: 1,
        velocity_axis: VelocityAxis::X,
        v_min_q16: 0,
    };
    arm(ArmMsg {
        arm_id: ARM_ID,
        arm_clock,
        source_count: 1,
        sources,
        stepper_count: 2,
        stepper_oids: [0, 1, 0, 0, 0, 0, 0, 0],
        grant_ticks: 0,
    })
    .expect("arm must succeed")
}

fn read_widened_now(shared: &SharedState) -> u64 {
    let lo = u64::from(shared.widened_now_lo.load(Ordering::Acquire));
    let hi = u64::from(shared.widened_now_hi.load(Ordering::Acquire));
    (hi << 32) | lo
}

// ─── test 1: GPIO detection reports without freezing ─────────────────────────

/// Armed GPIO endstop, pin asserted mid-"motion" → ISR tick calls endstop::tick,
/// queues the trip report, returns Continue (siren disabled) → engine.tick runs
/// across MULTIPLE subsequent ticks before the relay arrives. No freeze until
/// software_trip sets the latch.
#[test]
fn gpio_detection_reports_without_freezing_steps_continue() {
    let _guard = runtime::endstop::test_guard();

    let mut isr = make_isr();
    let shared = SharedState::new();
    let mut storage = make_storage();
    configure_axis(&mut isr, 0, 0, &mut storage);
    let _qs = install_queue(&mut isr);

    push_one_piece(&mut isr, 0, const_piece(0, 10.0), &mut storage);
    arm_gpio(0);

    // Tick 0: pin not asserted. Engine active (piece in window).
    isr_sample_tick(&mut isr, &shared, &mut storage, 0);
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
    assert!(poll_trip().is_none(), "no trip yet");

    // Assert the pin before tick 1.
    set_pin_level(GPIO_PIN, true);

    // Tick 1: endstop::tick detects the GPIO trip. Siren is disabled → Continue.
    // engine.tick still runs (last_tick_now becomes Some → active).
    isr_sample_tick(&mut isr, &shared, &mut storage, TICK_CYCLES);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "no fault: detection tick runs engine.tick normally"
    );

    // Trip event is queued: detection reported.
    let evt = poll_trip().expect("trip event must be queued");
    assert_eq!(evt.arm_id, ARM_ID);
    assert_eq!(evt.trip_clock, u64::from(TICK_CYCLES));

    // Engine was active this tick (last_tick_now = Some).
    assert!(
        isr.last_tick_now.is_some(),
        "engine.tick ran: last_tick_now must be Some after active tick"
    );

    // Multiple subsequent ticks while relay is in flight — engine must keep running.
    for n in 2u32..=5 {
        isr_sample_tick(&mut isr, &shared, &mut storage, TICK_CYCLES * n);
        assert_eq!(
            shared.last_error.load(Ordering::Acquire),
            0,
            "tick {n} must not fault while relay is in flight"
        );
        assert!(
            isr.last_tick_now.is_some(),
            "engine.tick must run at tick {n}: last_tick_now must be Some"
        );
        assert!(
            poll_trip().is_none(),
            "no duplicate trip event at tick {n}"
        );
    }
}

// ─── test 2: software_trip → AbortNow → zero steps, clock still published ────

/// software_trip sets ArmState::TrippedReady; next ISR tick:
///   - endstop::tick returns AbortNow
///   - engine.tick is skipped (no step dispatch)
///   - widened clock is published (foreground scheduler must not freeze)
///   - last_tick_now is None (gap guard cleared for unfreeze safety)
///   - freeze latches: subsequent tick also AbortNow
#[test]
fn software_trip_freezes_engine_skips_dispatch_clock_published() {
    let _guard = runtime::endstop::test_guard();

    let mut isr = make_isr();
    let shared = SharedState::new();
    let mut storage = make_storage();
    configure_axis(&mut isr, 0, 0, &mut storage);
    configure_axis(&mut isr, 1, 1, &mut storage);
    let _qs = install_queue(&mut isr);

    push_one_piece(&mut isr, 0, const_piece(0, 10.0), &mut storage);
    arm_gpio(0);

    // Tick 0: engine active, endstop armed, pin low → no trip.
    isr_sample_tick(&mut isr, &shared, &mut storage, 0);
    assert!(isr.last_tick_now.is_some(), "tick 0 must be active");
    let widened_after_tick0 = read_widened_now(&shared);

    // Simulate the relay's trsync_trigger → runtime_stop_on_trigger callback.
    let result = software_trip(ARM_ID, u64::from(TICK_CYCLES), &[]);
    assert_eq!(result, runtime::endstop::TripResult::Tripped);

    // Tick 1: AbortNow path.
    isr_sample_tick(&mut isr, &shared, &mut storage, TICK_CYCLES);

    // Widened clock must advance (publish happens before the freeze check).
    let widened_after_tick1 = read_widened_now(&shared);
    assert!(
        widened_after_tick1 > widened_after_tick0,
        "widened clock must advance even on a frozen tick"
    );

    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "freeze tick must not raise a fault"
    );

    // last_tick_now must be None: gap guard dormant until recovery.
    assert!(
        isr.last_tick_now.is_none(),
        "freeze tick must clear last_tick_now"
    );

    // Tick 2: latch persists — still AbortNow.
    isr_sample_tick(&mut isr, &shared, &mut storage, TICK_CYCLES * 2);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "second freeze tick must not fault"
    );
    assert!(
        isr.last_tick_now.is_none(),
        "second freeze tick keeps last_tick_now None"
    );
}

// ─── test 3: recovery after freeze — engine accepts new motion ────────────────

/// Freeze → disarm → engine.reset → re-arm → engine.tick resumes. Abandoned
/// pieces in the cleared ring do not replay.
#[test]
fn recovery_after_freeze_accepts_new_motion() {
    let _guard = runtime::endstop::test_guard();

    let mut isr = make_isr();
    let shared = SharedState::new();
    let mut storage = make_storage();
    configure_axis(&mut isr, 0, 0, &mut storage);
    let _qs = install_queue(&mut isr);

    // Long piece — will be abandoned on engine.reset.
    push_one_piece(&mut isr, 0, const_piece(0, 100.0), &mut storage);
    arm_gpio(0);

    // Tick 0: establish active baseline.
    isr_sample_tick(&mut isr, &shared, &mut storage, 0);
    assert!(isr.last_tick_now.is_some());

    // Trip.
    software_trip(ARM_ID, 0, &[]);

    // Tick 1: frozen.
    isr_sample_tick(&mut isr, &shared, &mut storage, TICK_CYCLES);
    assert!(isr.last_tick_now.is_none(), "must be frozen");

    // --- Recovery sequence matching the host flow ---
    // disarm returns AlreadyTripped (state stays TrippedReady; arm() clears it).
    let ds = disarm(ARM_ID);
    assert!(
        matches!(
            ds,
            runtime::endstop::DisarmStatus::AlreadyTripped
                | runtime::endstop::DisarmStatus::Disarmed
        ),
        "disarm after trip: {ds:?}"
    );

    // One tick between disarm and new motion, as on hardware (host command
    // round-trips span many tick periods): it consumes the frozen-disarm
    // ring purge so it cannot eat the next move's pieces.
    isr_sample_tick(&mut isr, &shared, &mut storage, 2 * TICK_CYCLES);

    // engine.reset: clears axes + ring (abandoned pieces gone).
    isr.engine.reset();
    assert_eq!(isr.engine.num_axes, 0, "engine.reset must clear axes");

    // Re-arm: arm() resets ARM.state → Armed, clears TrippedReady latch.
    // arm_clock far ahead so detection won't fire during the next tick.
    arm_gpio(u64::from(TICK_CYCLES) * 100);

    // Reconfigure and push a fresh piece.
    configure_axis(&mut isr, 0, 0, &mut storage);
    let _ = install_queue(&mut isr);
    push_one_piece(
        &mut isr,
        0,
        const_piece(u64::from(TICK_CYCLES) * 5, 100.0),
        &mut storage,
    );

    // Tick 2: engine should run again (arm_clock far ahead → tick returns
    // Continue; piece window covers this tick).
    isr_sample_tick(&mut isr, &shared, &mut storage, TICK_CYCLES * 5);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "recovery tick must not fault"
    );
    assert!(
        isr.last_tick_now.is_some(),
        "engine.tick must be active after recovery"
    );
}

// ─── test 4: snapshot stepper_counts match shared.stepper_counts ─────────────

/// The GPIO-detection path of endstop::tick calls collect_stepper_counts(shared)
/// so the trip snapshot reflects the live per-OID counts at that tick.
#[test]
fn snapshot_stepper_counts_match_shared_counts_at_detection_tick() {
    let _guard = runtime::endstop::test_guard();

    let mut isr = make_isr();
    let shared = SharedState::new();
    let mut storage = make_storage();
    configure_axis(&mut isr, 0, 0, &mut storage);
    let _qs = install_queue(&mut isr);

    // Seed known step counts for OIDs 0 and 1 (the arm's stepper_oids).
    let expected_count_0: i32 = 1234;
    let expected_count_1: i32 = -567;
    shared.stepper_counts[0].store(expected_count_0, Ordering::Release);
    shared.stepper_counts[1].store(expected_count_1, Ordering::Release);

    // Assert pin before arm so the arm-time check does not fire AlreadyTripped
    // (TripImmediately would trigger at arm() time). WaitForClear requires a
    // clear-then-assert, so use WaitForClear policy here to control timing.
    set_pin_level(GPIO_PIN, false);
    {
        let mut sources = [SourceConfig::EMPTY; runtime::endstop::MAX_SOURCES];
        sources[0] = SourceConfig {
            kind: SourceKind::Physical,
            gpio: GPIO_PIN,
            active_high: true,
            policy: ArmPolicy::WaitForClear, // requires a clear before trip
            sample_n: 1,
            velocity_axis: VelocityAxis::X,
            v_min_q16: 0,
        };
        arm(ArmMsg {
            arm_id: ARM_ID,
            arm_clock: 0,
            source_count: 1,
            sources,
            stepper_count: 2,
            stepper_oids: [0, 1, 0, 0, 0, 0, 0, 0],
            grant_ticks: 0,
        })
        .expect("arm");
    }

    push_one_piece(&mut isr, 0, const_piece(0, 10.0), &mut storage);

    // Tick 0: pin low → WaitForClear notes the clear. No trip.
    isr_sample_tick(&mut isr, &shared, &mut storage, 0);
    assert!(poll_trip().is_none());

    // Now assert the pin.
    set_pin_level(GPIO_PIN, true);

    // Tick 1: pin asserted and cleared flag set → trip detected by endstop::tick.
    // collect_stepper_counts(shared) passes the live OID-indexed counts.
    isr_sample_tick(&mut isr, &shared, &mut storage, TICK_CYCLES);

    let evt = poll_trip().expect("trip event must be queued");
    assert_eq!(evt.stepper_count, 2);

    // stepper_oids = [0, 1]; snapshot counts indexed by OID.
    let snap0 = evt
        .steppers
        .iter()
        .find(|s| s.oid == 0)
        .expect("stepper OID 0 in snapshot");
    let snap1 = evt
        .steppers
        .iter()
        .find(|s| s.oid == 1)
        .expect("stepper OID 1 in snapshot");
    assert_eq!(
        snap0.step_count, expected_count_0,
        "snapshot OID 0 must match shared.stepper_counts[0]"
    );
    assert_eq!(
        snap1.step_count, expected_count_1,
        "snapshot OID 1 must match shared.stepper_counts[1]"
    );
}

// ─── test 5: gap guard silent after unfreeze with large clock jump ────────────

/// After a freeze (last_tick_now cleared to None) and recovery, a large
/// raw_cyccnt jump on the first active tick does NOT raise TickIntervalExceeded.
#[test]
fn unfreeze_does_not_trigger_gap_fault_after_large_clock_jump() {
    let _guard = runtime::endstop::test_guard();

    let mut isr = make_isr();
    let shared = SharedState::new();
    let mut storage = make_storage();
    configure_axis(&mut isr, 0, 0, &mut storage);
    let _qs = install_queue(&mut isr);

    push_one_piece(&mut isr, 0, const_piece(0, 100.0), &mut storage);
    arm_gpio(0);

    // Tick 0: active.
    isr_sample_tick(&mut isr, &shared, &mut storage, 0);
    assert!(isr.last_tick_now.is_some());

    // Freeze.
    software_trip(ARM_ID, 0, &[]);

    // Tick 1: frozen → last_tick_now = None.
    isr_sample_tick(&mut isr, &shared, &mut storage, TICK_CYCLES);
    assert!(isr.last_tick_now.is_none());

    // Recovery.
    disarm(ARM_ID);
    isr.engine.reset();
    arm_gpio(u64::from(TICK_CYCLES) * 1000);
    configure_axis(&mut isr, 0, 0, &mut storage);
    let _ = install_queue(&mut isr);
    push_one_piece(
        &mut isr,
        0,
        const_piece(u64::from(TICK_CYCLES) * 1000, 100.0),
        &mut storage,
    );

    // First tick after recovery with a huge raw_cyccnt (gap >> 2×period if
    // last_tick_now were Some). Must not fault: last_tick_now is None.
    isr_sample_tick(&mut isr, &shared, &mut storage, TICK_CYCLES * 1000);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "first tick after unfreeze with large gap must not fault"
    );
}

// ─── test 6: full sequence arm → GPIO detect → relay → frozen → recovery ─────

/// Complete homing sequence through the ISR:
///   1. arm → engine active several ticks (pin low)
///   2. GPIO asserted → detection tick (Continue, event queued, engine runs)
///   3. N more ticks while relay round-trip is in flight (Continue, engine runs)
///   4. software_trip (relay arrives) → freeze latch set
///   5. Next ISR tick → AbortNow, engine skipped, last_tick_now cleared
///   6. recovery (disarm → engine.reset → re-arm) → engine resumes
#[test]
fn full_sequence_gpio_detect_relay_freeze_recovery() {
    let _guard = runtime::endstop::test_guard();

    let mut isr = make_isr();
    let shared = SharedState::new();
    let mut storage = make_storage();
    configure_axis(&mut isr, 0, 0, &mut storage);
    let _qs = install_queue(&mut isr);

    push_one_piece(&mut isr, 0, const_piece(0, 10.0), &mut storage);
    arm_gpio(0);

    // Phase 1: normal motion, pin low.
    for n in 0u32..3 {
        isr_sample_tick(&mut isr, &shared, &mut storage, TICK_CYCLES * n);
        assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
        assert!(poll_trip().is_none());
    }

    // Phase 2: GPIO asserted → detection tick.
    set_pin_level(GPIO_PIN, true);
    isr_sample_tick(&mut isr, &shared, &mut storage, TICK_CYCLES * 3);
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
    assert!(isr.last_tick_now.is_some(), "engine ran on detection tick");
    let evt = poll_trip().expect("trip event queued after detection");
    assert_eq!(evt.arm_id, ARM_ID);

    // Phase 3: relay in-flight — engine keeps running.
    for n in 4u32..=7 {
        isr_sample_tick(&mut isr, &shared, &mut storage, TICK_CYCLES * n);
        assert_eq!(shared.last_error.load(Ordering::Acquire), 0, "tick {n}");
        assert!(
            isr.last_tick_now.is_some(),
            "engine must run at tick {n} before relay arrives"
        );
        assert!(poll_trip().is_none(), "no duplicate event at tick {n}");
    }

    // Phase 4: relay arrives via software_trip.
    let trip_result = software_trip(ARM_ID, u64::from(TICK_CYCLES) * 8, &[]);
    assert_eq!(trip_result, runtime::endstop::TripResult::Tripped);

    // Phase 5: next ISR tick → frozen.
    isr_sample_tick(&mut isr, &shared, &mut storage, TICK_CYCLES * 8);
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
    assert!(
        isr.last_tick_now.is_none(),
        "engine must be frozen: last_tick_now = None"
    );

    // Another frozen tick.
    isr_sample_tick(&mut isr, &shared, &mut storage, TICK_CYCLES * 9);
    assert!(isr.last_tick_now.is_none(), "still frozen");

    // Phase 6: recovery.
    let ds = disarm(ARM_ID);
    assert!(matches!(
        ds,
        runtime::endstop::DisarmStatus::AlreadyTripped
            | runtime::endstop::DisarmStatus::Disarmed
    ));
    // One tick between disarm and new motion, as on hardware: consumes the
    // frozen-disarm ring purge so it cannot eat the next move's pieces.
    isr_sample_tick(&mut isr, &shared, &mut storage, TICK_CYCLES * 10);
    isr.engine.reset();
    arm_gpio(u64::from(TICK_CYCLES) * 1000);
    configure_axis(&mut isr, 0, 0, &mut storage);
    let _ = install_queue(&mut isr);
    push_one_piece(
        &mut isr,
        0,
        const_piece(u64::from(TICK_CYCLES) * 20, 100.0),
        &mut storage,
    );

    isr_sample_tick(&mut isr, &shared, &mut storage, TICK_CYCLES * 20);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "recovery tick must not fault"
    );
    assert!(
        isr.last_tick_now.is_some(),
        "engine resumes after full recovery"
    );
}

#[test]
fn disarm_after_freeze_purges_rings_via_isr_no_rearm_needed() {
    // Single-approach homing: after the trip there is NO retract / second
    // approach, so nothing ever re-arms. The disarm alone must unfreeze the
    // engine and retire the abandoned pieces (returning ring credits), or
    // the host's post-homing drain hangs forever.
    let _guard = runtime::endstop::test_guard();

    let mut isr = make_isr();
    let shared = SharedState::new();
    let mut storage = make_storage();
    configure_axis(&mut isr, 0, 0, &mut storage);
    let _qs = install_queue(&mut isr);

    push_one_piece(&mut isr, 0, const_piece(0, 100.0), &mut storage);
    arm_gpio(0);

    isr_sample_tick(&mut isr, &shared, &mut storage, 0);
    assert!(isr.last_tick_now.is_some());

    software_trip(ARM_ID, 0, &[]);
    isr_sample_tick(&mut isr, &shared, &mut storage, TICK_CYCLES);
    assert!(isr.last_tick_now.is_none(), "must be frozen");
    let queued: usize = isr
        .engine
        .stepping_axes
        .iter()
        .flatten()
        .map(|a| a.ring.len())
        .sum();
    assert!(queued > 0, "abandoned piece still queued while frozen");

    let ds = disarm(ARM_ID);
    assert!(matches!(
        ds,
        runtime::endstop::DisarmStatus::AlreadyTripped
            | runtime::endstop::DisarmStatus::Disarmed
    ));

    // Next ISR tick: purge runs before any piece evaluation; engine unfrozen.
    isr_sample_tick(&mut isr, &shared, &mut storage, 2 * TICK_CYCLES);
    let queued_after: usize = isr
        .engine
        .stepping_axes
        .iter()
        .flatten()
        .map(|a| a.ring.len())
        .sum();
    assert_eq!(queued_after, 0, "abandoned pieces must be retired by the purge");
    assert!(
        isr.engine
            .stepping_axes
            .iter()
            .flatten()
            .all(|a| a.armed.is_none()),
        "armed piece must be dropped by the purge"
    );
}
