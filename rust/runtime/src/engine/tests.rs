#![allow(
    clippy::unwrap_used,
    clippy::indexing_slicing,
    clippy::integer_division
)]

use core::sync::atomic::Ordering;

use crate::engine::Engine;
use crate::error::RUNTIME_OK;
use crate::state::SharedState;
use crate::stepping_state::{StepMode, StepperBindingRust, TMC_CS_OID_NONE};

const TICK_CLOCK_FREQ: u32 = 520_000_000;
const TICK_SAMPLE_RATE: u32 = 40_000;

fn engine_with_z_axis(mode: StepMode) -> Engine {
    let mut engine = Engine::default();
    let bindings = [
        StepperBindingRust {
            stepper_oid: 10,
            tmc_cs_oid: TMC_CS_OID_NONE,
            _pad: [0; 2],
        },
        StepperBindingRust {
            stepper_oid: 11,
            tmc_cs_oid: TMC_CS_OID_NONE,
            _pad: [0; 2],
        },
        StepperBindingRust {
            stepper_oid: 12,
            tmc_cs_oid: TMC_CS_OID_NONE,
            _pad: [0; 2],
        },
    ];
    let rc = engine.configure_axis(2, mode, 0.00125, &bindings);
    assert_eq!(rc, RUNTIME_OK);
    engine
}

#[allow(clippy::cast_possible_truncation)]
fn configure_pulse_axis(engine: &mut Engine, axis: usize, mstep: f32) {
    let bindings = [StepperBindingRust {
        stepper_oid: 10,
        tmc_cs_oid: TMC_CS_OID_NONE,
        _pad: [0; 2],
    }];
    assert_eq!(
        engine.configure_axis(axis as u8, StepMode::Pulse, mstep, &bindings),
        RUNTIME_OK
    );
}

#[test]
fn motor_state_reads_seeded_position() {
    let mut engine = engine_with_z_axis(StepMode::Pulse);
    engine.seed_position([12.5, -3.0, 7.0]);
    assert_eq!(engine.motor_state(2), Some((7.0, 0.0)));
    assert!(engine.motor_state(0).is_none());
    assert!(engine.motor_state(7).is_none());
}

#[test]
fn seed_position_rounds_to_the_nearest_microstep_and_aligns_the_phase_target() {
    let mut engine = engine_with_z_axis(StepMode::Pulse);
    let microstep_distance = 0.000_690_468_75_f32;
    let position = 5_792.071_3_f32;
    engine.stepping_axes[2].as_mut().unwrap().microstep_distance = microstep_distance;

    engine.seed_position([0.0, 0.0, position]);

    let axis = engine.stepping_axes[2].as_ref().unwrap();
    let expected_count = 8_388_607;
    assert_eq!(axis.last_step_count, expected_count);
    assert_eq!(axis.p_prev, position);
    assert_eq!(axis.v_prev, 0.0);
    for stepper in &axis.steppers {
        assert_eq!(
            stepper.position_count.load(Ordering::Acquire),
            expected_count
        );
        assert_eq!(
            stepper.last_phase_target.load(Ordering::Acquire),
            expected_count,
            "seed must align the phase target with the step count"
        );
    }
}

#[test]
fn seed_position_zeroes_non_spatial_motors() {
    let mut engine = Engine::default();
    configure_pulse_axis(&mut engine, 3, 0.01);
    engine.stepping_axes[3].as_mut().unwrap().last_step_count = 4242;

    engine.seed_position([1.0, 2.0, 3.0]);

    let axis = engine.stepping_axes[3].as_ref().unwrap();
    assert_eq!(axis.last_step_count, 0);
    assert_eq!(axis.steppers[0].position_count.load(Ordering::Acquire), 0);
}

#[test]
fn resonance_buzz_arm_activates_per_axis_stream() {
    crate::buzz_stream::reset_for_test();
    let axis = 2usize;
    let mut engine = Engine::new(TICK_CLOCK_FREQ, TICK_SAMPLE_RATE);
    configure_pulse_axis(&mut engine, axis, 0.01);
    let shared = SharedState::new();

    assert_eq!(
        engine.resonance_buzz(&shared, 1u8 << axis, 0, 100_000, 100_000, 100_000, 20, 2, 0),
        0
    );
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
    assert!(crate::buzz_stream::axis_active(axis));
    assert!(!crate::buzz_stream::axis_active(0));
    crate::buzz_stream::reset_for_test();
}

#[test]
fn resonance_buzz_disarm_form_clears_streams() {
    crate::buzz_stream::reset_for_test();
    let axis = 2usize;
    let mut engine = Engine::new(TICK_CLOCK_FREQ, TICK_SAMPLE_RATE);
    configure_pulse_axis(&mut engine, axis, 0.01);
    let shared = SharedState::new();

    assert_eq!(
        engine.resonance_buzz(&shared, 1u8 << axis, 0, 100_000, 100_000, 100_000, 20, 2, 0),
        0
    );
    assert!(crate::buzz_stream::axis_active(axis));
    assert_eq!(
        engine.resonance_buzz(&shared, 1u8 << axis, 0, 100_000, 100_000, 0, 20, 2, 0),
        0
    );
    assert!(!crate::buzz_stream::axis_active(axis));
    crate::buzz_stream::reset_for_test();
}

#[cfg(feature = "sample-stepping")]
#[test]
fn resonance_buzz_conflicts_with_an_anchored_sample_lane_on_the_same_axis() {
    use crate::error::FaultCode;
    crate::buzz_stream::reset_for_test();
    let axis = 2usize;
    let mut engine = Engine::new(TICK_CLOCK_FREQ, TICK_SAMPLE_RATE);
    configure_pulse_axis(&mut engine, axis, 0.01);
    let shared = SharedState::new();

    engine.sample_anchor(&shared, 10, 0, 0);
    assert!(engine.sample_lane_anchored(axis));

    assert_eq!(
        engine.resonance_buzz(&shared, 1u8 << axis, 0, 100_000, 100_000, 100_000, 20, 2, 0),
        crate::buzz::BUZZ_REJECT_LANE_ANCHORED
    );
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::BuzzAxisConflict.as_i32()
    );
    assert!(!crate::buzz_stream::axis_active(axis));
    crate::buzz_stream::reset_for_test();
}

#[test]
fn resonance_buzz_rejects_axis_without_step_queue() {
    crate::buzz_stream::reset_for_test();
    let axis = crate::step_queue::N_AXIS_STEP_QUEUES;
    let mut engine = Engine::new(TICK_CLOCK_FREQ, TICK_SAMPLE_RATE);
    configure_pulse_axis(&mut engine, axis, 0.01);
    let shared = SharedState::new();

    assert_eq!(
        engine.resonance_buzz(&shared, 1u8 << axis, 0, 100_000, 100_000, 100_000, 20, 2, 0),
        crate::buzz::BUZZ_REJECT_NO_STEP_QUEUE
    );
    assert!(!crate::buzz_stream::axis_active(0));
    crate::buzz_stream::reset_for_test();
}

#[test]
fn resonance_buzz_skips_axis_unconfigured_on_this_mcu() {
    crate::buzz_stream::reset_for_test();
    let mut engine = Engine::new(TICK_CLOCK_FREQ, TICK_SAMPLE_RATE);
    configure_pulse_axis(&mut engine, 2, 0.01);
    let shared = SharedState::new();
    assert_eq!(
        engine.resonance_buzz(&shared, 0b001, 0, 100_000, 100_000, 100_000, 20, 2, 0),
        0,
        "an unconfigured-here axis must be ignored, not rejected"
    );
    assert!(!crate::buzz_stream::axis_active(0));
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "no fault latched"
    );
    crate::buzz_stream::reset_for_test();
}

#[test]
fn resonance_buzz_routes_phase_mode_axis_to_xdirect() {
    crate::buzz_stream::reset_for_test();
    let mut engine = Engine::new(TICK_CLOCK_FREQ, TICK_SAMPLE_RATE);
    configure_pulse_axis(&mut engine, 0, 0.01);
    engine.stepping_axes[0]
        .as_ref()
        .unwrap()
        .mode
        .store(StepMode::Phase as u8, Ordering::Release);
    let shared = SharedState::new();
    assert_eq!(
        engine.resonance_buzz(&shared, 0b001, 0, 100_000, 100_000, 100_000, 20, 2, 0),
        0,
        "buzz on a Phase-mode axis must arm via XDIRECT, not fault"
    );
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0, "no fault");
    assert!(crate::buzz_stream::axis_active(0));
    assert!(
        crate::buzz_stream::is_xdirect(0),
        "phase-mode axis must be marked an XDIRECT buzz stream"
    );
    crate::buzz_stream::reset_for_test();
}

#[test]
fn resonance_buzz_routes_swept_pulse_axis_to_staircase() {
    crate::buzz_stream::reset_for_test();
    let mut engine = Engine::new(TICK_CLOCK_FREQ, TICK_SAMPLE_RATE);
    configure_pulse_axis(&mut engine, 0, 0.01);
    let shared = SharedState::new();
    assert_eq!(
        engine.resonance_buzz(&shared, 0b001, 0, 5_000, 60_000, 300_000, 200, 20, 0),
        0,
        "swept buzz on a Pulse axis must arm, not fault"
    );
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0, "no fault");
    assert!(crate::buzz_stream::axis_active(0));
    assert!(
        !crate::buzz_stream::is_xdirect(0),
        "pulse axis is not XDIRECT"
    );
    assert!(
        crate::buzz_stream::is_sweep(0),
        "swept pulse axis must run the staircase generator"
    );
    crate::buzz_stream::reset_for_test();
}

#[test]
fn resonance_buzz_routes_fixed_tone_pulse_axis_to_plain_tone() {
    crate::buzz_stream::reset_for_test();
    let mut engine = Engine::new(TICK_CLOCK_FREQ, TICK_SAMPLE_RATE);
    configure_pulse_axis(&mut engine, 0, 0.01);
    let shared = SharedState::new();
    assert_eq!(
        engine.resonance_buzz(&shared, 0b001, 0, 50_000, 50_000, 100_000, 100, 10, 0),
        0
    );
    assert!(crate::buzz_stream::axis_active(0));
    assert!(
        !crate::buzz_stream::is_sweep(0),
        "fixed tone is not a sweep"
    );
    assert!(!crate::buzz_stream::is_xdirect(0));
    crate::buzz_stream::reset_for_test();
}
