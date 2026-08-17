#![allow(
    clippy::panic,
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::integer_division
)]

use core::sync::atomic::Ordering;

use crate::engine::Engine;
use crate::error::{FaultCode, RUNTIME_ERR_MOTION_IN_PROGRESS, RUNTIME_OK};
use crate::piece_ring::PieceEntry;
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
        engine.configure_axis(0, StepMode::Phase, 0.00078125, 4, &bindings, 64),
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
fn an_anchored_lane_takes_over_the_tick_from_the_piece_path() {
    let (mut engine, shared) = phase_engine();
    assert!(!engine.sample_lane_anchored(LANE));
    engine.sample_anchor(&shared, OID, ANCHOR, 0);
    assert!(engine.sample_lane_anchored(LANE));
    feed(&mut engine, &shared, 0, &[0, 4, 8, 12]);

    let mut storage = [PieceEntry::zeroed(); 64];
    assert!(engine.tick(ANCHOR + u64::from(INTERVAL), &shared, &mut storage));
    assert_eq!(
        engine.sample_executed(OID),
        Some((ANCHOR + u64::from(INTERVAL), 4))
    );
    assert_eq!(stepper_position(&engine), 4);
    assert!(engine.tick(ANCHOR + 2 * u64::from(INTERVAL), &shared, &mut storage));
    assert_eq!(stepper_position(&engine), 8);
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
}

#[test]
fn pieces_are_refused_on_a_lane_the_sample_transport_owns() {
    let (mut engine, shared) = phase_engine();
    let mut storage = [PieceEntry::zeroed(); 64];
    let mut piece = PieceEntry::zeroed();
    piece.duration = 0.001;
    piece.coeff_count = 2;
    assert_eq!(engine.push_pieces(0, &[piece], &mut storage), RUNTIME_OK);

    engine.sample_anchor(&shared, OID, ANCHOR, 0);
    assert_eq!(
        engine.push_pieces(0, &[piece], &mut storage),
        RUNTIME_ERR_MOTION_IN_PROGRESS
    );
}

#[test]
fn a_trip_halt_request_is_applied_by_the_next_tick() {
    let (mut engine, shared) = phase_engine();
    engine.sample_anchor(&shared, OID, ANCHOR, 0);
    feed(&mut engine, &shared, 0, &[0, 100, 200, 300]);

    let mut storage = [PieceEntry::zeroed(); 64];
    let halt_clock = ANCHOR + u64::from(INTERVAL) / 2;
    Engine::sample_request_halt(&shared, halt_clock);
    assert_ne!(
        shared.sample_halt_clock.load(Ordering::Acquire),
        NO_HALT_REQUEST
    );

    engine.tick(ANCHOR + 3 * u64::from(INTERVAL), &shared, &mut storage);
    assert_eq!(
        shared.sample_halt_clock.load(Ordering::Acquire),
        NO_HALT_REQUEST
    );
    assert_eq!(engine.sample_executed(OID), Some((halt_clock, 50)));

    engine.tick(ANCHOR + 10 * u64::from(INTERVAL), &shared, &mut storage);
    assert_eq!(engine.sample_executed(OID), Some((halt_clock, 50)));
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
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
        engine.configure_axis(0, StepMode::Pulse, 0.00078125, 4, &bindings, 64),
        RUNTIME_OK
    );
    engine.sample_anchor(&shared, OID, ANCHOR, 0);
    feed(&mut engine, &shared, 0, &[0, 4]);
    let mut storage = [PieceEntry::zeroed(); 64];
    engine.tick(ANCHOR + u64::from(INTERVAL), &shared, &mut storage);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::PhaseModeNotAvailable.as_i32()
    );
}
