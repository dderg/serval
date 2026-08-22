#![allow(clippy::indexing_slicing)]

#[cfg(feature = "motion-module-stepper")]
use super::ramp_phase_offset;
use super::write_phase_coils;
use crate::state::SharedState;
use crate::stepping_state::{AxisConfig, StepMode, StepperRef};
use core::sync::atomic::{AtomicI16, AtomicI32, AtomicU8, Ordering};
use heapless::Vec;

fn make_stepper() -> StepperRef {
    StepperRef {
        stepper_oid: 0,
        position_count: AtomicI32::new(0),
        tmc_cs_oid: None,
        last_coil_A: AtomicI16::new(0),
        last_coil_B: AtomicI16::new(0),
        phase_offset_microsteps: AtomicI32::new(0),
        phase_offset_target: AtomicI32::new(0),
        last_phase_target: AtomicI32::new(0),
    }
}

fn make_axis(mode: StepMode, microstep_distance: f32) -> AxisConfig {
    let mut steppers: Vec<StepperRef, 4> = Vec::new();
    let _ = steppers.push(make_stepper());
    AxisConfig {
        mode: AtomicU8::new(mode as u8),
        steppers,
        microstep_distance,
        ..AxisConfig::new_unconfigured()
    }
}

#[test]
fn write_phase_coils_publishes_lut_pair_and_advances_position() {
    let shared = SharedState::new();
    let mut axis = make_axis(StepMode::Phase, 0.0125);
    axis.last_step_count = 256;

    write_phase_coils(0, &axis, &shared, 0);

    assert_eq!(axis.steppers[0].last_coil_A.load(Ordering::Acquire), 0);
    assert_eq!(axis.steppers[0].last_coil_B.load(Ordering::Acquire), 248);
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
    let mut axis = make_axis(StepMode::Phase, 0.0125);
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
    let mut axis = make_axis(StepMode::Phase, 0.0125);
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
    let axis = make_axis(StepMode::Phase, 0.0125);
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
    let axis = make_axis(StepMode::Phase, 0.0125);
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
    let axis = make_axis(StepMode::Phase, 0.0125);
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
