#![cfg(feature = "motion-module-stepper")]

use core::sync::atomic::Ordering;

use runtime::clock::WidenState;
use runtime::engine::Engine;
use runtime::error::FaultCode;
use runtime::state::{IsrState, SharedState};
use runtime::stepping_state::{StepMode, StepperBindingRust, TMC_CS_OID_NONE};
use runtime::tick::isr_sample_tick;

const CLOCK_FREQ: u32 = 520_000_000;
const SAMPLE_RATE: u32 = 40_000;
const TICK_CYCLES: u32 = CLOCK_FREQ / SAMPLE_RATE;

fn make_isr(engine: Engine) -> IsrState {
    IsrState {
        engine,
        widen_state: WidenState::default(),
        last_tick_now: None,
    }
}

fn binding() -> StepperBindingRust {
    StepperBindingRust {
        stepper_oid: 0,
        tmc_cs_oid: TMC_CS_OID_NONE,
        _pad: [0; 2],
    }
}

fn engine_with_axis0(mode: StepMode) -> Engine {
    let mut engine = Engine::new(CLOCK_FREQ, SAMPLE_RATE);
    let rc = engine.configure_axis(0, mode, 0.0125, &[binding()]);
    assert_eq!(rc, 0, "configure_axis failed");
    engine
}

/// An idle runtime: a Pulse axis with nothing queued claims no tick.
fn idle_isr() -> IsrState {
    make_isr(engine_with_axis0(StepMode::Pulse))
}

/// An active runtime: a Phase axis slewing one microstep per sample toward a
/// target far enough away that every tick in these tests claims the axis.
fn active_isr(shared: &SharedState) -> IsrState {
    let engine = engine_with_axis0(StepMode::Phase);
    shared
        .max_phase_offset_ramp_per_sample
        .store(1, Ordering::Release);
    engine.stepping_axes[0].as_ref().expect("axis").steppers[0]
        .phase_offset_target
        .store(1_000_000, Ordering::Release);
    make_isr(engine)
}

#[test]
fn idle_ticks_never_fault_even_with_large_gap() {
    let shared = SharedState::new();
    let mut isr = idle_isr();

    isr_sample_tick(&mut isr, &shared, 0);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "idle tick 0 must not fault"
    );

    isr_sample_tick(&mut isr, &shared, 1_000_000_000_u32);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "idle tick with huge gap must not fault — guard must never fire during idle"
    );

    isr_sample_tick(&mut isr, &shared, 1_000_013_000_u32);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "second idle tick must not fault"
    );
    assert!(
        isr.last_tick_now.is_none(),
        "idle ticks must leave last_tick_now as None"
    );
}

#[test]
fn active_motion_gap_latches_tick_interval_exceeded() {
    let shared = SharedState::new();
    let mut isr = active_isr(&shared);

    isr_sample_tick(&mut isr, &shared, 0);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "first active tick must not fault"
    );
    assert!(
        isr.last_tick_now.is_some(),
        "an active tick must set the interval baseline"
    );

    isr_sample_tick(&mut isr, &shared, TICK_CYCLES * 5);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::TickIntervalExceeded.as_i32(),
        "5×period gap during active motion must latch TickIntervalExceeded"
    );
}

#[test]
fn steady_cadence_of_active_ticks_never_faults() {
    let shared = SharedState::new();
    let mut isr = active_isr(&shared);

    for i in 0u32..60 {
        isr_sample_tick(&mut isr, &shared, TICK_CYCLES * i);
        assert_eq!(
            shared.last_error.load(Ordering::Acquire),
            0,
            "no fault expected at active tick {i} on a steady cadence"
        );
    }
}

#[test]
fn an_active_tick_after_an_idle_one_rebaselines_instead_of_faulting() {
    let shared = SharedState::new();
    let mut isr = active_isr(&shared);

    isr_sample_tick(&mut isr, &shared, TICK_CYCLES);
    assert!(isr.last_tick_now.is_some());

    isr.last_tick_now = None;

    isr_sample_tick(&mut isr, &shared, TICK_CYCLES * 1000);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "active tick after idle (last_tick_now=None) must not fault, even with huge gap"
    );
    assert!(
        isr.last_tick_now.is_some(),
        "active tick after idle re-baseline must set last_tick_now to Some"
    );
}

#[test]
fn gap_exactly_2x_period_does_not_fault() {
    let shared = SharedState::new();
    let mut isr = active_isr(&shared);

    isr_sample_tick(&mut isr, &shared, 0);
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
    assert!(isr.last_tick_now.is_some());

    isr_sample_tick(&mut isr, &shared, TICK_CYCLES * 2);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "gap == 2×period must not fault (strictly-greater-than threshold)"
    );
}

#[test]
fn large_gap_saturates_fault_detail_to_0xffff() {
    let shared = SharedState::new();
    let mut isr = active_isr(&shared);

    isr_sample_tick(&mut isr, &shared, 0);
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
    assert!(isr.last_tick_now.is_some());

    let gap_ticks_target: u32 = 0x1_0000;
    isr_sample_tick(&mut isr, &shared, gap_ticks_target * TICK_CYCLES);

    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::TickIntervalExceeded.as_i32(),
        "65536-tick gap must latch TickIntervalExceeded"
    );
    let detail = shared.fault_detail.load(Ordering::Acquire);
    assert_eq!(
        detail, 0xFFFF,
        "fault_detail must saturate at 0xFFFF for a {gap_ticks_target}-tick gap, got {detail}"
    );
}
