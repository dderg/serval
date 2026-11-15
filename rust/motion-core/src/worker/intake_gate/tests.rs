use std::time::Instant;

use geometry::segment::SourceRange;
use geometry::{MoveContext, VelocityLimits, line_move};

use super::*;

const RESERVE_S: f64 = 0.5;
const TOLERANCE_S: f64 = 0.05;

fn gate_with_runway(runway_s: f64) -> IntakeGate {
    let frontier: Arc<CommittedFrontier> = Arc::default();
    frontier.advance_to(Instant::now() + Duration::from_secs_f64(runway_s));
    IntakeGate::new(frontier, RESERVE_S)
}

#[test]
fn the_cap_is_the_reserve_plus_the_pumps_stall_headroom() {
    let gate = gate_with_runway(0.0);
    assert_eq!(gate.cap_secs(), RESERVE_S + crate::pump::MAX_LEAD_SECS);
    gate.set_reserve(1.5);
    assert_eq!(gate.cap_secs(), 1.5 + crate::pump::MAX_LEAD_SECS);
}

#[test]
fn a_runway_short_of_the_cap_admits() {
    let gate = gate_with_runway(RESERVE_S + STALL_HEADROOM_S - 0.5);
    assert!(gate.admits());
    assert_eq!(gate.reopens_in(), None);
}

#[test]
fn a_runway_past_the_cap_reopens_once_the_playhead_eats_the_excess() {
    let gate = gate_with_runway(RESERVE_S + STALL_HEADROOM_S + 1.0);
    assert!(!gate.admits());
    let reopens_s = gate.reopens_in().expect("closed").as_secs_f64();
    assert!(
        (reopens_s - 1.0).abs() < TOLERANCE_S,
        "reopens in {reopens_s}s, expected 1s"
    );
}

#[test]
fn moves_waiting_in_the_inbox_count_against_the_cap() {
    let gate = gate_with_runway(RESERVE_S + STALL_HEADROOM_S - 0.5);
    let second = Duration::from_secs(1).as_nanos() as u64;
    gate.enter(second);
    assert!(
        !gate.admits(),
        "1 s of moves in the inbox on a 0.5 s margin"
    );
    gate.leave(second);
    assert!(gate.admits());
}

#[test]
fn a_pipeline_no_mcu_executes_always_admits() {
    let gate = IntakeGate::new(Arc::new(CommittedFrontier::unexecuted()), RESERVE_S);
    assert!(gate.admits());
}

#[test]
#[should_panic(expected = "left an inbox that held only")]
fn leaving_more_than_entered_fails_loudly() {
    let gate = gate_with_runway(0.0);
    gate.enter(10);
    gate.leave(11);
}

#[test]
fn a_move_counts_its_path_at_its_feedrate() {
    let ctx = MoveContext {
        extruder_axis: 3,
        feedrate_mm_s: 50.0,
        limits: VelocityLimits::try_new(
            300.0,
            5000.0,
            geometry::CornerBudget::Shaped(5.0),
            f64::INFINITY,
        )
        .unwrap(),
        source: SourceRange {
            start_line: 1,
            end_line: 1,
        },
    };
    let m = line_move([0.0; 3], [100.0, 0.0, 0.0], 0.0, ctx).unwrap();
    assert_eq!(shortest_ns(&m), 2_000_000_000);
}
