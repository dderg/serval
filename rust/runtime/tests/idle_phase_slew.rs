#![cfg(feature = "motion-module-stepper")]

use core::sync::atomic::Ordering;

use runtime::engine::Engine;
use runtime::state::SharedState;
use runtime::stepping_state::{StepMode, StepperBindingRust};

const CLOCK_FREQ: u32 = 520_000_000;
const SAMPLE_RATE: u32 = 40_000;
const TICK_CYCLES: u64 = (CLOCK_FREQ / SAMPLE_RATE) as u64;

fn make_engine(mode: StepMode) -> Engine {
    let mut engine = Engine::new(CLOCK_FREQ, SAMPLE_RATE);
    let binding = StepperBindingRust {
        stepper_oid: 5,
        tmc_cs_oid: 7,
        _pad: [0; 2],
    };
    let rc = engine.configure_axis(0, mode, 0.000_625, &[binding]);
    assert_eq!(rc, 0, "configure_axis failed");
    engine
}

#[test]
fn jog_slews_to_target_while_no_lane_is_anchored() {
    let mut engine = make_engine(StepMode::Phase);
    let shared = SharedState::new();
    shared.phase_motor_count.store(1, Ordering::Release);
    shared.phase_slot_idx[0].store(0, Ordering::Release);

    assert_eq!(engine.phase_jog_to(&shared, 5, 20, 1), 0);
    let q = engine.phase_state(5).expect("stepper must be found");
    assert!(!q.settled, "jog must leave a pending slew");

    for n in 1..=64_u64 {
        assert!(
            engine.tick(n * TICK_CYCLES, &shared) || engine.phase_state(5).unwrap().settled,
            "an unsettled phase axis must claim the tick"
        );
    }

    let q = engine.phase_state(5).expect("stepper must be found");
    assert!(
        q.settled,
        "idle ticks must ramp the offset to the jog target (phase={})",
        q.phase
    );
    assert_eq!(q.phase, 20);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "no fault during idle slew"
    );
}

#[test]
fn a_settled_phase_axis_does_not_claim_the_tick() {
    let mut engine = make_engine(StepMode::Phase);
    let shared = SharedState::new();

    assert!(
        !engine.tick(TICK_CYCLES, &shared),
        "no pending slew means no work"
    );
}

#[test]
fn idle_pulse_axis_does_not_dispatch() {
    let mut engine = make_engine(StepMode::Pulse);
    let shared = SharedState::new();

    let active = engine.tick(TICK_CYCLES, &shared);
    assert!(!active, "idle pulse axis must not report active");
}
