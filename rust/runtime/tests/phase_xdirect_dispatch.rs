#![cfg(feature = "motion-module-stepper")]
#![allow(clippy::unwrap_used)]

use core::sync::atomic::{AtomicU8, Ordering};
use heapless::Vec;

use runtime::dispatch_stepper::write_phase_coils;
use runtime::phase_lut::PHASE_LUT;
use runtime::state::{MAX_STEPPER_OIDS, SharedState};
use runtime::stepping_state::{AxisState, MAX_STEPPERS_PER_AXIS, StepMode, StepperRef};
use runtime::test_xdirect_capture;

fn make_phase_stepper(stepper_oid: u8, tmc_cs_oid: u8) -> StepperRef {
    StepperRef::new(stepper_oid, Some(tmc_cs_oid))
}

fn make_phase_axis(microstep_distance: f32, stepper: StepperRef, position: i32) -> AxisState {
    let mut steppers: Vec<StepperRef, MAX_STEPPERS_PER_AXIS> = Vec::new();
    let _ = steppers.push(stepper);
    AxisState {
        mode: AtomicU8::new(StepMode::Phase as u8),
        steppers,
        microstep_distance,
        last_step_count: position,
        ..AxisState::new_unconfigured()
    }
}

fn configure_phase_slot(shared: &SharedState, motor_idx: usize, axis_idx: usize) {
    assert!(motor_idx < MAX_STEPPER_OIDS);
    #[allow(clippy::cast_possible_truncation)]
    shared.phase_slot_idx[motor_idx].store(axis_idx as u8, Ordering::Release);
    let prev_count = shared.phase_motor_count.load(Ordering::Acquire);
    #[allow(clippy::cast_possible_truncation)]
    if (motor_idx as u8) >= prev_count {
        shared
            .phase_motor_count
            .store(motor_idx as u8 + 1, Ordering::Release);
    }
}

#[test]
fn phase_dispatch_records_correct_coils_for_motor_0() {
    let _guard = test_xdirect_capture::lock_for_test();
    test_xdirect_capture::clear();

    let shared = SharedState::new();
    let axis_idx: usize = 0;
    let motor_idx: usize = 0;
    configure_phase_slot(&shared, motor_idx, axis_idx);

    let axis = make_phase_axis(0.0125, make_phase_stepper(0, 2), 256);
    write_phase_coils(axis_idx, &axis, &shared, 0);

    let records = test_xdirect_capture::drain();
    assert_eq!(records.len(), 1, "expected exactly one SPI capture");
    let rec = &records[0];
    assert_eq!(rec.motor_idx, 0, "motor_idx mismatch");

    let (expected_a, expected_b) = PHASE_LUT[256];
    assert_eq!(rec.coil_a, expected_a, "coil_a must match PHASE_LUT[256]");
    assert_eq!(rec.coil_b, expected_b, "coil_b must match PHASE_LUT[256]");
}

#[test]
fn phase_dispatch_resolves_motor_idx_from_slot_table() {
    let _guard = test_xdirect_capture::lock_for_test();
    test_xdirect_capture::clear();

    let shared = SharedState::new();
    let axis_idx: usize = 1;
    let motor_idx: usize = 2;
    shared.phase_slot_idx[0].store(0u8, Ordering::Release);
    shared.phase_slot_idx[1].store(0u8, Ordering::Release);
    configure_phase_slot(&shared, motor_idx, axis_idx);

    let axis = make_phase_axis(0.0125, make_phase_stepper(1, 5), 512);
    write_phase_coils(axis_idx, &axis, &shared, 0);

    let records = test_xdirect_capture::drain();
    assert_eq!(records.len(), 1, "expected exactly one SPI capture");
    assert_eq!(
        records[0].motor_idx, 2,
        "motor_idx must resolve to 2, not 0 or 1"
    );

    let (expected_a, expected_b) = PHASE_LUT[512];
    assert_eq!(records[0].coil_a, expected_a);
    assert_eq!(records[0].coil_b, expected_b);
}

#[test]
fn phase_dispatch_no_capture_for_pulse_only_stepper() {
    let _guard = test_xdirect_capture::lock_for_test();
    test_xdirect_capture::clear();

    let shared = SharedState::new();
    let stepper = StepperRef::new(0, None);

    configure_phase_slot(&shared, 0, 0);
    let axis = make_phase_axis(0.0125, stepper, 256);
    write_phase_coils(0, &axis, &shared, 0);

    let records = test_xdirect_capture::drain();
    assert!(
        records.is_empty(),
        "Pulse-only stepper must not produce a capture"
    );
    assert_eq!(
        axis.steppers[0].last_phase_target.load(Ordering::Acquire),
        256,
        "a Pulse-only stepper still tracks its phase target"
    );
    assert_eq!(axis.steppers[0].position_count.load(Ordering::Acquire), 256);
}

#[test]
fn phase_dispatch_two_steppers_two_captures() {
    let _guard = test_xdirect_capture::lock_for_test();
    test_xdirect_capture::clear();

    let shared = SharedState::new();
    configure_phase_slot(&shared, 0, 0);
    configure_phase_slot(&shared, 1, 0);

    let mut steppers: Vec<StepperRef, MAX_STEPPERS_PER_AXIS> = Vec::new();
    let _ = steppers.push(make_phase_stepper(0, 3));
    let _ = steppers.push(make_phase_stepper(1, 4));

    let axis = AxisState {
        mode: AtomicU8::new(StepMode::Phase as u8),
        steppers,
        microstep_distance: 0.0125,
        last_step_count: 256,
        ..AxisState::new_unconfigured()
    };

    write_phase_coils(0, &axis, &shared, 0);

    let records = test_xdirect_capture::drain();
    assert_eq!(
        records.len(),
        2,
        "expected two SPI captures for two steppers"
    );
    assert_eq!(records[0].motor_idx, 0, "first stepper → motor_idx 0");
    assert_eq!(records[1].motor_idx, 1, "second stepper → motor_idx 1");

    let (expected_a, expected_b) = PHASE_LUT[256];
    assert_eq!(records[0].coil_a, expected_a);
    assert_eq!(records[0].coil_b, expected_b);
    assert_eq!(records[1].coil_a, expected_a);
    assert_eq!(records[1].coil_b, expected_b);
}

#[test]
fn phase_dispatch_at_phase_zero() {
    let _guard = test_xdirect_capture::lock_for_test();
    test_xdirect_capture::clear();

    let shared = SharedState::new();
    configure_phase_slot(&shared, 0, 0);
    let axis = make_phase_axis(0.0125, make_phase_stepper(0, 1), 0);

    write_phase_coils(0, &axis, &shared, 0);

    let records = test_xdirect_capture::drain();
    assert_eq!(records.len(), 1);
    let (expected_a, expected_b) = PHASE_LUT[0];
    assert_eq!(records[0].coil_a, expected_a, "PHASE_LUT[0] coil_a");
    assert_eq!(records[0].coil_b, expected_b, "PHASE_LUT[0] coil_b");
}

#[test]
fn phase_dispatch_empty_slot_table_latches_phase_motor_unmapped() {
    let _guard = test_xdirect_capture::lock_for_test();
    test_xdirect_capture::clear();

    let shared = SharedState::new();
    assert_eq!(shared.phase_motor_count.load(Ordering::Acquire), 0);

    let axis = make_phase_axis(0.0125, make_phase_stepper(0, 7), 256);
    write_phase_coils(0, &axis, &shared, 0);

    let records = test_xdirect_capture::drain();
    assert!(
        records.is_empty(),
        "no SPI write may reach an unmapped motor"
    );
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        runtime::error::FaultCode::PhaseMotorUnmapped.as_i32(),
        "unmapped phase motor must latch PhaseMotorUnmapped"
    );
    let detail = shared.fault_detail.load(Ordering::Acquire);
    assert_eq!(detail >> 16, 0, "axis_idx in detail high bits");
    assert_eq!(detail & 0xFFFF, 0, "stepper_oid in detail low bits");
}
