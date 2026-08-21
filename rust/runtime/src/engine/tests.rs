#![allow(
    clippy::unwrap_used,
    clippy::indexing_slicing,
    clippy::integer_division
)]

use core::sync::atomic::Ordering;

use crate::engine::Engine;
use crate::error::RUNTIME_OK;
use crate::stepping_state::{StepMode, StepperBindingRust, TMC_CS_OID_NONE};

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
