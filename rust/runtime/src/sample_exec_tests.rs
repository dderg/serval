#![allow(
    clippy::panic,
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::integer_division
)]

use crate::error::FaultCode;
use crate::sample_exec::{LaneOutput, SampleLane, SampleLaneFault, widen_wire_clock};
use crate::sample_run::{SampleRunError, encode_deltas};
use crate::state::SharedState;

use core::sync::atomic::Ordering;

const INTERVAL: u32 = 100;

fn shared() -> SharedState {
    SharedState::new()
}

fn wire(base: i32, samples: &[i32]) -> ([u8; 256], usize, u8) {
    let mut buf = [0u8; 256];
    let written = encode_deltas(base, samples, &mut buf).expect("encodes");
    #[allow(clippy::cast_possible_truncation)]
    let count = samples.len() as u8;
    (buf, written, count)
}

fn push(lane: &mut SampleLane, now: u64, base: i32, samples: &[i32]) {
    let (buf, len, count) = wire(base, samples);
    lane.push_run(now, INTERVAL, count, &buf[..len])
        .expect("run accepted");
}

fn position(lane: &mut SampleLane, shared: &SharedState, now: u64) -> i32 {
    match lane.tick(now, shared, 0) {
        LaneOutput::Position(p) => p,
        LaneOutput::Unanchored => panic!("lane reported unanchored"),
    }
}

#[test]
fn unanchored_lane_leaves_the_axis_alone() {
    let mut lane = SampleLane::new();
    let shared = shared();
    assert_eq!(lane.tick(1_000, &shared, 0), LaneOutput::Unanchored);
    assert!(!lane.is_anchored());
}

#[test]
fn anchor_holds_its_position_until_the_first_run_starts() {
    let mut lane = SampleLane::new();
    let shared = shared();
    lane.anchor(0, 1_000, 77).expect("anchor accepted");
    assert!(lane.is_anchored());
    assert_eq!(position(&mut lane, &shared, 0), 77);
    assert_eq!(position(&mut lane, &shared, 999), 77);
}

#[test]
fn samples_replay_exactly_on_their_own_clocks() {
    let mut lane = SampleLane::new();
    let shared = shared();
    lane.anchor(0, 1_000, 0).expect("anchor accepted");
    let samples = [0, 3, 9, 18, 30];
    push(&mut lane, 0, 0, &samples);
    for (index, expected) in samples.iter().copied().enumerate() {
        let now = 1_000 + (index as u64) * u64::from(INTERVAL);
        assert_eq!(position(&mut lane, &shared, now), expected, "index {index}");
    }
}

#[test]
fn between_samples_the_lane_interpolates_and_rounds_to_nearest() {
    let mut lane = SampleLane::new();
    let shared = shared();
    lane.anchor(0, 1_000, 0).expect("anchor accepted");
    push(&mut lane, 0, 0, &[0, 100]);
    assert_eq!(position(&mut lane, &shared, 1_000), 0);
    assert_eq!(position(&mut lane, &shared, 1_025), 25);
    assert_eq!(position(&mut lane, &shared, 1_050), 50);
    assert_eq!(position(&mut lane, &shared, 1_099), 99);
}

#[test]
fn rounding_is_symmetric_about_zero() {
    let mut lane = SampleLane::new();
    let shared = shared();
    lane.anchor(0, 1_000, 0).expect("anchor accepted");
    push(&mut lane, 0, 0, &[0, -100]);
    assert_eq!(position(&mut lane, &shared, 1_025), -25);
    assert_eq!(position(&mut lane, &shared, 1_075), -75);
}

#[test]
fn the_last_sample_of_a_run_interpolates_into_the_next_run() {
    let mut lane = SampleLane::new();
    let shared = shared();
    lane.anchor(0, 1_000, 0).expect("anchor accepted");
    push(&mut lane, 0, 0, &[0, 100]);
    push(&mut lane, 0, 100, &[300]);
    // Inside the second sample of run one, the bracket is (100, 300).
    assert_eq!(position(&mut lane, &shared, 1_150), 200);
    assert_eq!(position(&mut lane, &shared, 1_200), 300);
}

#[test]
fn a_run_that_ends_the_stream_holds_its_final_sample() {
    let mut lane = SampleLane::new();
    let shared = shared();
    lane.anchor(0, 1_000, 0).expect("anchor accepted");
    push(&mut lane, 0, 0, &[0, 40, 40]);
    assert_eq!(position(&mut lane, &shared, 1_250), 40);
    assert_eq!(position(&mut lane, &shared, 1_299), 40);
    assert_eq!(position(&mut lane, &shared, 9_999), 40);
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
}

#[test]
fn a_ring_that_drains_mid_motion_is_a_loud_underrun() {
    let mut lane = SampleLane::new();
    let shared = shared();
    lane.anchor(0, 1_000, 0).expect("anchor accepted");
    push(&mut lane, 0, 0, &[0, 40, 80]);
    assert_eq!(position(&mut lane, &shared, 1_100), 40);
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
    // Past the run's end with nothing behind it and 40 quanta/interval of
    // velocity still on the clock.
    let _ = position(&mut lane, &shared, 1_300);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::SampleRingUnderrun.as_i32()
    );
    assert_eq!(shared.fault_detail.load(Ordering::Acquire) & 0xFFFF, 40);
}

#[test]
fn a_run_whose_start_clock_already_passed_is_rejected() {
    let mut lane = SampleLane::new();
    lane.anchor(0, 1_000, 0).expect("anchor accepted");
    let (buf, len, count) = wire(0, &[0, 10]);
    let err = lane
        .push_run(1_100, INTERVAL, count, &buf[..len])
        .expect_err("late run rejected");
    assert!(matches!(err, SampleLaneFault::Late { deficit_ticks } if deficit_ticks == 100));
}

#[test]
fn an_anchor_in_the_past_is_rejected() {
    let mut lane = SampleLane::new();
    let err = lane.anchor(5_000, 1_000, 0).expect_err("late anchor");
    assert!(matches!(err, SampleLaneFault::Late { .. }));
    assert!(!lane.is_anchored());
}

#[test]
fn the_two_tick_tolerance_admits_a_run_that_is_only_just_late() {
    let mut lane = SampleLane::new();
    lane.anchor(1_002, 1_000, 0).expect("anchor inside slack");
    let (buf, len, count) = wire(0, &[0, 10]);
    lane.push_run(1_002, INTERVAL, count, &buf[..len])
        .expect("run inside slack");
}

#[test]
fn a_run_before_any_anchor_is_rejected() {
    let mut lane = SampleLane::new();
    let (buf, len, count) = wire(0, &[0, 10]);
    let err = lane
        .push_run(0, INTERVAL, count, &buf[..len])
        .expect_err("unanchored run rejected");
    assert!(matches!(
        err,
        SampleLaneFault::Run(SampleRunError::NotAnchored { .. })
    ));
}

#[test]
fn a_zero_count_run_is_rejected() {
    let mut lane = SampleLane::new();
    lane.anchor(0, 1_000, 0).expect("anchor accepted");
    let err = lane
        .push_run(0, INTERVAL, 0, &[])
        .expect_err("zero-count run rejected");
    assert!(matches!(
        err,
        SampleLaneFault::Run(SampleRunError::ZeroCount { .. })
    ));
}

#[test]
fn a_zero_interval_run_is_rejected() {
    let mut lane = SampleLane::new();
    lane.anchor(0, 1_000, 0).expect("anchor accepted");
    let (buf, len, count) = wire(0, &[0, 10]);
    let err = lane
        .push_run(0, 0, count, &buf[..len])
        .expect_err("zero-interval run rejected");
    assert!(matches!(
        err,
        SampleLaneFault::Run(SampleRunError::ZeroInterval { .. })
    ));
}

#[test]
fn a_truncated_payload_is_rejected_rather_than_shortened() {
    let mut lane = SampleLane::new();
    lane.anchor(0, 1_000, 0).expect("anchor accepted");
    let (buf, len, count) = wire(0, &[0, 10, 20]);
    let err = lane
        .push_run(0, INTERVAL, count, &buf[..len - 1])
        .expect_err("truncated payload rejected");
    assert!(matches!(
        err,
        SampleLaneFault::Run(SampleRunError::Truncated { .. })
    ));
}

#[test]
fn a_full_ring_rejects_the_next_run() {
    let mut lane = SampleLane::new();
    lane.anchor(0, 1_000, 0).expect("anchor accepted");
    let mut base = 0;
    let mut accepted = 0usize;
    loop {
        let samples = [base + 1, base + 2];
        let (buf, len, count) = wire(base, &samples);
        match lane.push_run(0, INTERVAL, count, &buf[..len]) {
            Ok(()) => {
                base += 2;
                accepted += 1;
                assert!(accepted < 1_000, "ring never filled");
            }
            Err(fault) => {
                assert_eq!(fault, SampleLaneFault::RingFull);
                assert_eq!(accepted, crate::sizing::SAMPLE_RUNS_PER_LANE);
                return;
            }
        }
    }
}

#[test]
fn retiring_runs_frees_ring_slots() {
    let mut lane = SampleLane::new();
    let shared = shared();
    lane.anchor(0, 1_000, 0).expect("anchor accepted");
    for _ in 0..crate::sizing::SAMPLE_RUNS_PER_LANE {
        push(&mut lane, 0, 0, &[0, 0]);
    }
    assert_eq!(lane.depth(), crate::sizing::SAMPLE_RUNS_PER_LANE);
    let span = 2 * u64::from(INTERVAL) * (crate::sizing::SAMPLE_RUNS_PER_LANE as u64);
    let _ = position(&mut lane, &shared, 1_000 + span);
    assert_eq!(lane.depth(), 0);
    push(&mut lane, 1_000 + span, 0, &[0, 0]);
}

#[test]
fn an_overlay_run_adds_onto_the_base_lane() {
    let mut lane = SampleLane::new();
    let shared = shared();
    lane.anchor(0, 1_000, 0).expect("anchor accepted");
    push(&mut lane, 0, 0, &[0, 100, 200]);
    let (buf, len, count) = wire(0, &[0, 5, 10]);
    lane.push_overlay(0, 1_000, INTERVAL, count, &buf[..len])
        .expect("overlay accepted");
    assert_eq!(position(&mut lane, &shared, 1_000), 0);
    assert_eq!(position(&mut lane, &shared, 1_100), 105);
    assert_eq!(position(&mut lane, &shared, 1_200), 210);
}

#[test]
fn an_exhausted_overlay_holds_its_nudge() {
    let mut lane = SampleLane::new();
    let shared = shared();
    lane.anchor(0, 1_000, 0).expect("anchor accepted");
    push(&mut lane, 0, 0, &[0, 0, 0, 0]);
    let (buf, len, count) = wire(0, &[0, 7]);
    lane.push_overlay(0, 1_000, INTERVAL, count, &buf[..len])
        .expect("overlay accepted");
    assert_eq!(position(&mut lane, &shared, 1_100), 7);
    assert_eq!(position(&mut lane, &shared, 1_300), 7);
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
}

#[test]
fn a_non_abutting_overlay_restarts_the_overlay_lane_from_zero() {
    let mut lane = SampleLane::new();
    let shared = shared();
    lane.anchor(0, 1_000, 0).expect("anchor accepted");
    push(&mut lane, 0, 0, &[0, 0, 0, 0, 0, 0]);
    let (buf, len, count) = wire(0, &[9]);
    lane.push_overlay(0, 1_000, INTERVAL, count, &buf[..len])
        .expect("first overlay");
    assert_eq!(position(&mut lane, &shared, 1_000), 9);
    let (buf, len, count) = wire(0, &[4]);
    lane.push_overlay(0, 1_400, INTERVAL, count, &buf[..len])
        .expect("second overlay re-anchors");
    assert_eq!(position(&mut lane, &shared, 1_400), 4);
}

#[test]
fn halt_freezes_the_interpolated_position_and_drops_queued_runs() {
    let mut lane = SampleLane::new();
    let shared = shared();
    lane.anchor(0, 1_000, 0).expect("anchor accepted");
    push(&mut lane, 0, 0, &[0, 100, 200, 300]);
    lane.halt(1_050, &shared, 0);
    assert!(lane.is_halted());
    assert_eq!(lane.executed(), (1_050, 50));
    assert_eq!(position(&mut lane, &shared, 1_300), 50);
    assert_eq!(lane.depth(), 0);
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
}

#[test]
fn halt_is_idempotent() {
    let mut lane = SampleLane::new();
    let shared = shared();
    lane.anchor(0, 1_000, 0).expect("anchor accepted");
    push(&mut lane, 0, 0, &[0, 100]);
    lane.halt(1_050, &shared, 0);
    lane.halt(1_090, &shared, 0);
    assert_eq!(lane.executed(), (1_050, 50));
}

#[test]
fn a_halted_lane_rejects_runs_until_it_is_re_anchored() {
    let mut lane = SampleLane::new();
    let shared = shared();
    lane.anchor(0, 1_000, 0).expect("anchor accepted");
    push(&mut lane, 0, 0, &[0, 100]);
    lane.halt(1_050, &shared, 0);
    let (buf, len, count) = wire(0, &[0, 10]);
    let err = lane
        .push_run(1_050, INTERVAL, count, &buf[..len])
        .expect_err("halted lane is unanchored");
    assert!(matches!(
        err,
        SampleLaneFault::Run(SampleRunError::NotAnchored { .. })
    ));
    lane.anchor(1_050, 2_000, 50).expect("re-anchor accepted");
    assert!(!lane.is_halted());
    assert_eq!(position(&mut lane, &shared, 2_000), 50);
}

#[test]
fn executed_position_tracks_the_last_driven_tick() {
    let mut lane = SampleLane::new();
    let shared = shared();
    lane.anchor(0, 1_000, 0).expect("anchor accepted");
    push(&mut lane, 0, 0, &[0, 100]);
    let _ = position(&mut lane, &shared, 1_075);
    assert_eq!(lane.executed(), (1_075, 75));
}

#[test]
fn a_run_may_change_the_interval() {
    let mut lane = SampleLane::new();
    let shared = shared();
    lane.anchor(0, 1_000, 0).expect("anchor accepted");
    push(&mut lane, 0, 0, &[0, 100]);
    let (buf, len, count) = wire(100, &[110, 120]);
    lane.push_run(0, 25, count, &buf[..len])
        .expect("faster run accepted");
    assert_eq!(position(&mut lane, &shared, 1_200), 110);
    assert_eq!(position(&mut lane, &shared, 1_225), 120);
}

#[test]
fn a_single_sample_run_is_legal() {
    let mut lane = SampleLane::new();
    let shared = shared();
    lane.anchor(0, 1_000, 0).expect("anchor accepted");
    push(&mut lane, 0, 0, &[42]);
    assert_eq!(position(&mut lane, &shared, 1_000), 42);
    assert_eq!(position(&mut lane, &shared, 1_050), 42);
}

#[test]
fn a_wire_clock_widens_to_the_candidate_nearest_now() {
    assert_eq!(widen_wire_clock(0, 1_000), 1_000);
    assert_eq!(widen_wire_clock(5_000, 4_000), 4_000);
}

#[test]
fn a_wire_clock_just_past_a_wrap_widens_upward() {
    let now = (1u64 << 32) - 100;
    assert_eq!(widen_wire_clock(now, 50), (1u64 << 32) + 50);
}

#[test]
fn a_wire_clock_just_before_a_wrap_widens_downward() {
    let now = (1u64 << 32) + 100;
    assert_eq!(widen_wire_clock(now, u32::MAX - 50), (1u64 << 32) - 51);
}
