#![allow(clippy::indexing_slicing)]

#[cfg(feature = "motion-module-stepper")]
use super::ramp_phase_offset;
use super::write_phase_coils;
use crate::state::SharedState;
use crate::stepping_state::{AxisState, StepMode, StepperRef};
use crate::test_xdirect_capture;
use core::sync::atomic::{AtomicU8, Ordering};
use heapless::Vec;

fn make_axis(mode: StepMode, microstep_distance: f32, tmc_cs_oid: Option<u8>) -> AxisState {
    let mut steppers: Vec<StepperRef, 4> = Vec::new();
    let _ = steppers.push(StepperRef::new(0, tmc_cs_oid));
    AxisState {
        mode: AtomicU8::new(mode as u8),
        steppers,
        microstep_distance,
        ..AxisState::new_unconfigured()
    }
}

fn map_one_phase_motor(shared: &SharedState) {
    shared.phase_motor_count.store(1, Ordering::Release);
    shared.phase_slot_idx[0].store(0, Ordering::Release);
}

#[test]
fn write_phase_coils_publishes_lut_pair_and_advances_position() {
    let _guard = test_xdirect_capture::lock_for_test();
    test_xdirect_capture::clear();

    let shared = SharedState::new();
    map_one_phase_motor(&shared);
    let mut axis = make_axis(StepMode::Phase, 0.0125, Some(3));
    axis.last_step_count = 256;

    write_phase_coils(0, &axis, &shared, 0);

    let records = test_xdirect_capture::drain();
    assert_eq!(records.len(), 1, "one TMC stepper → one XDIRECT write");
    assert_eq!(records[0].motor_idx, 0);
    assert_eq!(records[0].coil_a, 0);
    assert_eq!(records[0].coil_b, 248);
    assert_eq!(
        axis.steppers[0].last_phase_target.load(Ordering::Acquire),
        256
    );
    assert_eq!(axis.steppers[0].position_count.load(Ordering::Acquire), 256);
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
}

#[test]
fn write_phase_coils_honors_phase_offset() {
    let shared = SharedState::new();
    let mut axis = make_axis(StepMode::Phase, 0.0125, None);
    axis.last_step_count = 256;
    axis.steppers[0]
        .phase_offset_microsteps
        .store(7, Ordering::Release);

    write_phase_coils(0, &axis, &shared, 0);

    assert_eq!(
        axis.steppers[0].last_phase_target.load(Ordering::Acquire),
        263
    );
    assert_eq!(axis.steppers[0].position_count.load(Ordering::Acquire), 263);
}

#[test]
fn write_phase_coils_adds_the_buzz_offset() {
    let shared = SharedState::new();
    let mut axis = make_axis(StepMode::Phase, 0.0125, None);
    axis.last_step_count = 256;

    write_phase_coils(0, &axis, &shared, -6);

    assert_eq!(
        axis.steppers[0].last_phase_target.load(Ordering::Acquire),
        250
    );
    assert_eq!(axis.steppers[0].position_count.load(Ordering::Acquire), 250);
}

#[cfg(feature = "motion-module-stepper")]
#[test]
fn ramp_phase_offset_advances_at_most_max_per_sample() {
    let axis = make_axis(StepMode::Phase, 0.0125, None);
    axis.steppers[0]
        .phase_offset_target
        .store(10, Ordering::Release);

    for expected in [4_i32, 8, 10] {
        ramp_phase_offset(&axis.steppers[0], 4);
        assert_eq!(
            axis.steppers[0]
                .phase_offset_microsteps
                .load(Ordering::Acquire),
            expected,
        );
    }
}

#[cfg(feature = "motion-module-stepper")]
#[test]
fn ramp_phase_offset_ramps_down_toward_a_lower_target() {
    let axis = make_axis(StepMode::Phase, 0.0125, None);
    axis.steppers[0]
        .phase_offset_microsteps
        .store(10, Ordering::Release);
    axis.steppers[0]
        .phase_offset_target
        .store(1, Ordering::Release);

    for expected in [6_i32, 2, 1] {
        ramp_phase_offset(&axis.steppers[0], 4);
        assert_eq!(
            axis.steppers[0]
                .phase_offset_microsteps
                .load(Ordering::Acquire),
            expected,
        );
    }
}

#[cfg(feature = "motion-module-stepper")]
#[test]
fn ramp_phase_offset_is_a_noop_when_max_per_sample_is_zero() {
    let axis = make_axis(StepMode::Phase, 0.0125, None);
    axis.steppers[0]
        .phase_offset_microsteps
        .store(3, Ordering::Release);
    axis.steppers[0]
        .phase_offset_target
        .store(99, Ordering::Release);

    ramp_phase_offset(&axis.steppers[0], 0);

    assert_eq!(
        axis.steppers[0]
            .phase_offset_microsteps
            .load(Ordering::Acquire),
        3,
    );
}
