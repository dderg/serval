use std::time::Instant;

use geometry::segment::SourceRange;
use geometry::{MoveContext, VelocityLimits, line_move};

use super::*;

const RESERVE_S: f64 = 0.5;
const TOLERANCE_S: f64 = 0.05;

fn gate_with_runway(runway_s: f64) -> IntakeGate {
    let frontier: Arc<CommittedFrontier> = Arc::default();
    frontier.advance_to(Instant::now() + Duration::from_secs_f64(runway_s));
    IntakeGate::new(frontier, Arc::default(), RESERVE_S)
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

fn gate_with_planned_ahead(runway_s: f64, planned_ahead_s: f64) -> IntakeGate {
    let frontier: Arc<CommittedFrontier> = Arc::default();
    frontier.advance_to(Instant::now() + Duration::from_secs_f64(runway_s));
    let links: Arc<WorkerLinks> = Arc::default();
    let dispatched_s = 10.0_f64;
    links
        .last_move_time_bits
        .store(dispatched_s.to_bits(), Ordering::Release);
    let gate = IntakeGate::new(frontier, links, RESERVE_S);
    gate.set_planned_through(dispatched_s + planned_ahead_s);
    gate
}

#[test]
fn motion_a_stage_holds_for_lookahead_never_shuts_the_intake() {
    const HELD_STROKE_S: f64 = 2.21;
    let gate = gate_with_planned_ahead(RESERVE_S, HELD_STROKE_S);
    assert!(
        gate.admits(),
        "the shaper holds a {HELD_STROKE_S}s stroke until the next move arrives; \
         refusing that move starves it with the runway at the reserve"
    );
}

#[test]
fn the_peak_ahead_counts_planned_motion_the_pump_has_not_taken() {
    let gate = gate_with_planned_ahead(1.0, 3.0);
    gate.note_ahead();
    let peak_s = gate.take_peak_ahead();
    assert!(
        (peak_s - 4.0).abs() < TOLERANCE_S,
        "peak {peak_s}s, expected 4s"
    );
}

#[test]
fn planned_motion_the_dispatcher_passed_counts_nothing() {
    let gate = gate_with_planned_ahead(0.0, -6.0);
    gate.note_ahead();
    assert!(
        gate.take_peak_ahead() < TOLERANCE_S,
        "a dwell put the dispatched end past the plan"
    );
}

#[test]
fn the_peak_ahead_is_the_most_noted_since_it_was_last_taken() {
    let gate = gate_with_runway(1.0);
    let second = Duration::from_secs(1).as_nanos() as u64;
    gate.note_ahead();
    gate.enter(second);
    gate.note_ahead();
    gate.leave(second);
    gate.note_ahead();

    let peak_s = gate.take_peak_ahead();
    assert!(
        (peak_s - 2.0).abs() < TOLERANCE_S,
        "peak {peak_s}s, expected 2s"
    );
    assert_eq!(gate.take_peak_ahead(), 0.0);
}

#[test]
fn a_reopening_less_than_a_nanosecond_away_has_happened() {
    const SUB_NANOSECOND_S: f64 = 1.0e-10;
    let links: Arc<WorkerLinks> = Arc::default();
    let gate = IntakeGate::new(Arc::default(), links, RESERVE_S);
    gate.set_planned_through(gate.cap_secs() + SUB_NANOSECOND_S);
    assert_eq!(gate.reopens_in(), None);
    assert!(gate.admits());
}

#[test]
fn a_pipeline_no_mcu_executes_always_admits() {
    let gate = IntakeGate::new(
        Arc::new(CommittedFrontier::unexecuted()),
        Arc::default(),
        RESERVE_S,
    );
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
