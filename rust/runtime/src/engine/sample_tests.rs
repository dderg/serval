#![allow(
    clippy::panic,
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::integer_division
)]

use core::sync::atomic::Ordering;

use crate::engine::Engine;
use crate::error::{FaultCode, RUNTIME_OK};
use crate::sample_run::encode_deltas;
use crate::state::{NO_HALT_REQUEST, SharedState};
use crate::stepping_state::{StepMode, StepperBindingRust, TMC_CS_OID_NONE};

const OID: u8 = 7;
const LANE: usize = 0;
const INTERVAL: u32 = 13_000;
const ANCHOR: u64 = 1_000_000;

fn phase_engine() -> (Engine, SharedState) {
    let mut engine = Engine::new(520_000_000, 40_000);
    let shared = SharedState::new();
    let bindings = [StepperBindingRust {
        stepper_oid: OID,
        tmc_cs_oid: TMC_CS_OID_NONE,
        _pad: [0; 2],
    }];
    assert_eq!(
        engine.configure_axis(0, StepMode::Phase, 0.00078125, &bindings),
        RUNTIME_OK
    );
    (engine, shared)
}

fn wire(base: i32, samples: &[i32]) -> ([u8; 64], usize, u8) {
    let mut buf = [0u8; 64];
    let written = encode_deltas(base, samples, &mut buf).expect("encodes");
    #[allow(clippy::cast_possible_truncation)]
    let count = samples.len() as u8;
    (buf, written, count)
}

fn feed(engine: &mut Engine, shared: &SharedState, base: i32, samples: &[i32]) {
    let (buf, len, count) = wire(base, samples);
    engine.sample_push_run(shared, OID, INTERVAL, count, &buf[..len]);
}

fn stepper_position(engine: &Engine) -> i32 {
    engine.stepping_axes[LANE]
        .as_ref()
        .expect("lane configured")
        .steppers[0]
        .position_count
        .load(Ordering::Acquire)
}

#[test]
fn a_bound_stepper_oid_resolves_to_its_lane() {
    let (engine, _shared) = phase_engine();
    assert_eq!(engine.sample_lane_for_oid(OID), Some(LANE));
}

#[test]
fn an_unbound_oid_latches_a_distinct_fault() {
    let (mut engine, shared) = phase_engine();
    engine.sample_anchor(&shared, OID + 1, ANCHOR, 0);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::SampleLaneUnknown.as_i32()
    );
    assert_eq!(
        shared.fault_detail.load(Ordering::Acquire) & 0xFFFF,
        u32::from(OID + 1)
    );
}

#[test]
fn an_anchored_lane_takes_over_the_tick() {
    let (mut engine, shared) = phase_engine();
    assert!(!engine.sample_lane_anchored(LANE));
    engine.sample_anchor(&shared, OID, ANCHOR, 0);
    assert!(engine.sample_lane_anchored(LANE));
    feed(&mut engine, &shared, 0, &[0, 4, 8, 12]);

    assert!(engine.tick(ANCHOR + u64::from(INTERVAL), &shared));
    assert_eq!(
        engine.sample_executed(OID),
        Some((ANCHOR + u64::from(INTERVAL), 4))
    );
    assert_eq!(stepper_position(&engine), 4);
    assert!(engine.tick(ANCHOR + 2 * u64::from(INTERVAL), &shared));
    assert_eq!(stepper_position(&engine), 8);
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
}

#[test]
fn a_trip_halt_request_is_applied_by_the_next_tick() {
    let (mut engine, shared) = phase_engine();
    engine.sample_anchor(&shared, OID, ANCHOR, 0);
    feed(&mut engine, &shared, 0, &[0, 100, 200, 300]);

    let halt_clock = ANCHOR + u64::from(INTERVAL) / 2;
    Engine::sample_request_halt(&shared, halt_clock);
    assert_ne!(
        shared.sample_halt_clock.load(Ordering::Acquire),
        NO_HALT_REQUEST
    );

    engine.tick(ANCHOR + 3 * u64::from(INTERVAL), &shared);
    assert_eq!(
        shared.sample_halt_clock.load(Ordering::Acquire),
        NO_HALT_REQUEST
    );
    assert_eq!(engine.sample_executed(OID), Some((halt_clock, 50)));

    engine.tick(ANCHOR + 10 * u64::from(INTERVAL), &shared);
    assert_eq!(engine.sample_executed(OID), Some((halt_clock, 50)));
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
}

#[test]
fn a_lane_frozen_on_a_trip_hold_does_not_refuse_a_phase_align() {
    let (mut engine, shared) = phase_engine();
    engine.sample_anchor(&shared, OID, ANCHOR, 0);
    feed(&mut engine, &shared, 0, &[0, 100, 200, 300]);
    assert_eq!(
        engine.phase_align_to(OID, 100),
        -2,
        "a lane playing samples owns the coils"
    );

    Engine::sample_request_halt(&shared, ANCHOR + u64::from(INTERVAL) / 2);
    engine.tick(ANCHOR + 3 * u64::from(INTERVAL), &shared);

    assert_eq!(
        engine.phase_align_to(OID, 100),
        0,
        "the frozen hold a trip left plays no sample and drives no coil; refusing here shut the \
         mcu down on every sensorless re-entry"
    );
}

#[test]
fn an_idle_anchored_lane_holding_its_last_sample_accepts_a_phase_align() {
    let (mut engine, shared) = phase_engine();
    engine.sample_anchor(&shared, OID, ANCHOR, 0);
    feed(&mut engine, &shared, 0, &[0, 100, 200, 200]);
    engine.tick(ANCHOR + 10 * u64::from(INTERVAL), &shared);

    assert_eq!(
        engine.phase_align_to(OID, 100),
        0,
        "a drained lane plays a zero-order hold; refusing here shut the mcu down on every \
         phase-mode entry"
    );
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
}

#[test]
fn a_mode_switch_unanchors_the_lane_so_the_next_tick_cannot_fault() {
    let (mut engine, shared) = phase_engine();
    engine.sample_anchor(&shared, OID, ANCHOR, 0);
    feed(&mut engine, &shared, 0, &[0, 100, 200, 200]);
    engine.tick(ANCHOR + 10 * u64::from(INTERVAL), &shared);

    assert_eq!(engine.set_axis_mode(0, 0), 0, "idle hold must not block");
    engine.tick(ANCHOR + 20 * u64::from(INTERVAL), &shared);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "an anchored lane under a Pulse mode byte would raise \
         PhaseModeNotAvailable"
    );
}

#[test]
fn the_first_halt_requester_fixes_the_clock() {
    let (_engine, shared) = phase_engine();
    Engine::sample_request_halt(&shared, 4_000);
    Engine::sample_request_halt(&shared, 9_000);
    assert_eq!(shared.sample_halt_clock.load(Ordering::Acquire), 4_000);
}

#[test]
fn a_sample_lane_on_a_pulse_axis_is_a_loud_fault() {
    let mut engine = Engine::new(520_000_000, 40_000);
    let shared = SharedState::new();
    let bindings = [StepperBindingRust {
        stepper_oid: OID,
        tmc_cs_oid: TMC_CS_OID_NONE,
        _pad: [0; 2],
    }];
    assert_eq!(
        engine.configure_axis(0, StepMode::Pulse, 0.00078125, &bindings),
        RUNTIME_OK
    );
    engine.sample_anchor(&shared, OID, ANCHOR, 0);
    feed(&mut engine, &shared, 0, &[0, 4]);
    engine.tick(ANCHOR + u64::from(INTERVAL), &shared);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::PhaseModeNotAvailable.as_i32()
    );
}

#[test]
fn a_halted_lane_swallows_runs_that_raced_the_trip() {
    // The Trident full-G28 fault of 2026-08-20: the Z trip halted every
    // lane while the pacer's next X run was already on the wire; the run
    // landed on the unanchored cursor and latched SampleRunRejected(-321,
    // NotAnchored), shutting the mcu down mid-home. A halted lane swallows
    // in-flight runs; the host re-anchors before anything resumes.
    let (mut engine, shared) = phase_engine();
    engine.sample_anchor(&shared, OID, ANCHOR, 0);
    feed(&mut engine, &shared, 0, &[0, 100, 200, 300]);

    let halt_clock = ANCHOR + u64::from(INTERVAL) / 2;
    Engine::sample_request_halt(&shared, halt_clock);
    engine.tick(ANCHOR + 3 * u64::from(INTERVAL), &shared);
    assert_eq!(engine.sample_executed(OID), Some((halt_clock, 50)));

    feed(&mut engine, &shared, 300, &[400, 500]);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "a run racing the halt is a sanctioned discontinuity, not a fault"
    );

    engine.tick(ANCHOR + 12 * u64::from(INTERVAL), &shared);
    assert_eq!(
        engine.sample_executed(OID),
        Some((halt_clock, 50)),
        "the swallowed run must not move the halted lane"
    );

    engine.sample_anchor(&shared, OID, ANCHOR + 20 * u64::from(INTERVAL), 50);
    feed(&mut engine, &shared, 50, &[54, 58, 58]);
    engine.tick(ANCHOR + 23 * u64::from(INTERVAL), &shared);
    assert_eq!(stepper_position(&engine), 58, "re-anchor resumes playback");
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
}

#[test]
fn force_idle_freezes_every_sample_lane() {
    let (mut engine, shared) = phase_engine();
    engine.sample_anchor(&shared, OID, ANCHOR, 0);
    feed(&mut engine, &shared, 0, &[0, 100, 200, 300]);

    let flush_clock = ANCHOR + u64::from(INTERVAL);
    engine.tick(flush_clock, &shared);
    assert_eq!(stepper_position(&engine), 100);

    crate::clock::publish_widened_now(&shared, flush_clock);
    engine.runtime_force_idle(&shared);
    assert_eq!(engine.sample_executed(OID), Some((flush_clock, 100)));

    for step in 2..6 {
        engine.tick(ANCHOR + step * u64::from(INTERVAL), &shared);
        assert_eq!(
            stepper_position(&engine),
            100,
            "no queued sample may execute after force idle"
        );
    }
    assert_eq!(engine.sample_executed(OID), Some((flush_clock, 100)));
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
}

#[test]
fn force_idle_receipts_the_fences_it_discarded() {
    let (mut engine, shared) = phase_engine();
    engine.sample_anchor(&shared, OID, ANCHOR, 0);
    feed(&mut engine, &shared, 0, &[0, 100, 200, 300]);
    engine.sample_push_barrier(&shared, OID, 77);
    assert_eq!(engine.sample_take_barrier_ack(), None);

    crate::clock::publish_widened_now(&shared, ANCHOR);
    engine.runtime_force_idle(&shared);
    assert_eq!(engine.sample_take_barrier_ack(), Some((OID, 77)));
}

#[test]
fn a_barrier_ack_names_the_stepper_oid_that_submitted_it() {
    let mut engine = Engine::new(520_000_000, 40_000);
    let shared = SharedState::new();
    let bindings = [
        StepperBindingRust {
            stepper_oid: OID,
            tmc_cs_oid: TMC_CS_OID_NONE,
            _pad: [0; 2],
        },
        StepperBindingRust {
            stepper_oid: OID + 1,
            tmc_cs_oid: TMC_CS_OID_NONE,
            _pad: [0; 2],
        },
    ];
    assert_eq!(
        engine.configure_axis(0, StepMode::Phase, 0.00078125, &bindings),
        RUNTIME_OK
    );
    engine.sample_anchor(&shared, OID, ANCHOR, 0);

    engine.sample_push_barrier(&shared, OID + 1, 42);
    assert_eq!(engine.sample_take_barrier_ack(), Some((OID + 1, 42)));
    assert_eq!(engine.sample_take_barrier_ack(), None);
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
}
