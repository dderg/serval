use runtime::error::{
    RUNTIME_ERR_INTERNAL_INVARIANT, RUNTIME_ERR_SAMPLE_RATE_MISCONFIGURED,
    RUNTIME_ERR_SAMPLE_RING_FULL, RUNTIME_ERR_SAMPLE_RING_UNDERRUN, RUNTIME_ERR_SAMPLE_RUN_LATE,
    RUNTIME_ERR_SAMPLE_RUN_REJECTED,
};
use runtime::sample_run::SampleRunError;

use super::*;

const INTERVAL: u32 = 250_000;

fn entries(count: usize, vel_ff: i32) -> Vec<SetpointEntry> {
    (0..count)
        .map(|i| SetpointEntry {
            pos_counts: i as i32 * 10,
            vel_ff,
            torque_ff: i as i16,
            acc_mm_s2: 0.0,
        })
        .collect()
}

fn ring() -> SetpointRing {
    SetpointRing::new(0, INTERVAL)
}

fn header(start_index: u64, anchor: bool) -> RunHeader {
    RunHeader {
        start_index,
        interval_ticks: INTERVAL,
        origin_mm: 1.0,
        anchor,
        final_run: false,
    }
}

fn anchored_at(index: u64, count: usize, vel_ff: i32) -> SetpointRing {
    let mut r = ring();
    r.fill(&header(index, true), &entries(count, vel_ff))
        .expect("anchor fill");
    r
}

fn final_run_at(index: u64, count: usize, vel_ff: i32) -> SetpointRing {
    let mut r = ring();
    r.fill(
        &RunHeader {
            final_run: true,
            ..header(index, true)
        },
        &entries(count, vel_ff),
    )
    .expect("anchor fill");
    r
}

#[test]
fn plays_entries_in_grid_order() {
    let mut r = final_run_at(100, 3, 0);
    assert_eq!(r.len(), 3);
    for (step, expected) in entries(3, 0).into_iter().enumerate() {
        assert_eq!(r.play(100 + step as u64), Played::Entry(expected));
    }
    assert_eq!(r.play(103), Played::Drained);
    assert_eq!(r.played_count(), 3);
    assert!(r.take_fault().is_none());
}

#[test]
fn abutting_run_extends_the_queue() {
    let mut r = anchored_at(10, 2, 0);
    r.fill(&header(12, false), &entries(2, 0)).expect("abut");
    assert_eq!(r.len(), 4);
    assert_eq!(r.next_index(), 14);
}

#[test]
fn hole_between_runs_is_a_fault() {
    let mut r = anchored_at(10, 2, 0);
    let err = r
        .fill(&header(13, false), &entries(1, 0))
        .expect_err("hole must fault");
    assert_eq!(
        err,
        RingFault::Rejected(SampleRunError::Discontinuity {
            expected_clock: 12,
            start_clock: 13
        })
    );
    assert_eq!(
        r.take_fault().map(|v| (v & 0xFFFF) as i16 as i32),
        Some(RUNTIME_ERR_SAMPLE_RUN_REJECTED)
    );
}

#[test]
fn run_for_already_played_cycles_is_late_with_its_deficit() {
    let mut r = anchored_at(10, 2, 0);
    r.play(10);
    r.play(11);
    let err = r
        .fill(&header(10, false), &entries(1, 0))
        .expect_err("late run must fault");
    assert_eq!(err, RingFault::RunLate { deficit_us: 500 });
    assert_eq!(err.code(), RUNTIME_ERR_SAMPLE_RUN_LATE);
    let reg = r.take_fault().expect("latched");
    assert_eq!(reg >> 16, 500);
    assert_eq!((reg & 0xFFFF) as i16 as i32, RUNTIME_ERR_SAMPLE_RUN_LATE);
}

#[test]
fn first_run_without_an_anchor_is_a_fault() {
    let mut r = ring();
    let err = r
        .fill(&header(5, false), &entries(1, 0))
        .expect_err("unanchored fill must fault");
    assert_eq!(
        err,
        RingFault::Rejected(SampleRunError::NotAnchored { start_clock: 5 })
    );
}

#[test]
fn draining_a_run_that_did_not_declare_its_end_is_an_underrun() {
    let mut r = anchored_at(0, 1, 4_000);
    assert!(matches!(r.play(0), Played::Entry(_)));
    assert_eq!(r.play(1), Played::Drained);
    let reg = r.take_fault().expect("underrun latched");
    assert_eq!(
        (reg & 0xFFFF) as i16 as i32,
        RUNTIME_ERR_SAMPLE_RING_UNDERRUN
    );
    assert_eq!(reg >> 16, 4_000);
}

#[test]
fn draining_after_a_final_run_is_the_expected_hold() {
    let mut r = final_run_at(0, 2, 4_000);
    assert!(matches!(r.play(0), Played::Entry(_)));
    assert!(matches!(r.play(1), Played::Entry(_)));
    assert_eq!(r.play(2), Played::Drained);
    assert_eq!(r.play(3), Played::Drained);
    assert!(r.take_fault().is_none());
}

#[test]
fn a_non_dc_interval_is_rejected_instead_of_resampled() {
    let mut r = ring();
    let err = r
        .fill(
            &RunHeader {
                interval_ticks: INTERVAL / 2,
                ..header(0, true)
            },
            &entries(1, 0),
        )
        .expect_err("foreign interval must fault");
    assert_eq!(
        err,
        RingFault::IntervalMismatch {
            expected_ticks: INTERVAL,
            got_ticks: INTERVAL / 2
        }
    );
    assert_eq!(err.code(), RUNTIME_ERR_SAMPLE_RATE_MISCONFIGURED);
}

#[test]
fn origin_may_not_shift_inside_an_epoch() {
    let mut r = anchored_at(0, 1, 0);
    let err = r
        .fill(
            &RunHeader {
                origin_mm: 2.0,
                ..header(1, false)
            },
            &entries(1, 0),
        )
        .expect_err("origin shift must fault");
    assert!(matches!(err, RingFault::OriginShift { .. }));
    assert_eq!(err.as_str(), "sample_origin_shift");
}

#[test]
fn a_re_anchor_drops_whatever_was_queued() {
    let mut r = anchored_at(10, 4, 0);
    r.fill(
        &RunHeader {
            origin_mm: 7.5,
            ..header(50, true)
        },
        &entries(2, 0),
    )
    .expect("re-anchor");
    assert_eq!(r.len(), 2);
    assert_eq!(r.next_index(), 52);
    assert_eq!(r.origin_mm(), Some(7.5));
    assert_eq!(r.play(50), Played::Entry(entries(2, 0)[0]));
}

#[test]
fn a_fill_past_the_free_depth_is_a_fault() {
    let mut r = ring();
    let mut index = 0;
    while r.free() > MAX_FILL_CYCLES {
        let batch = entries(MAX_FILL_CYCLES, 0);
        let fill = if index == 0 {
            r.fill(&header(index, true), &batch)
        } else {
            r.fill(&header(index, false), &batch)
        };
        fill.expect("fill within depth");
        index += MAX_FILL_CYCLES as u64;
    }
    let head_room = MAX_FILL_CYCLES / 2;
    r.fill(&header(index, false), &entries(head_room, 0))
        .expect("partial fill");
    index += head_room as u64;
    let free = r.free();
    assert!(free < MAX_FILL_CYCLES);
    let err = r
        .fill(&header(index, false), &entries(free + 1, 0))
        .expect_err("overfill must fault");
    assert_eq!(
        err,
        RingFault::RingFull {
            free_cycles: free as u32,
            asked: free as u32 + 1
        }
    );
    assert_eq!(err.code(), RUNTIME_ERR_SAMPLE_RING_FULL);
}

#[test]
fn skipped_cycles_are_discarded_not_replayed() {
    let mut r = anchored_at(0, 5, 0);
    assert!(matches!(r.play(0), Played::Entry(_)));
    let expected = entries(5, 0)[3];
    assert_eq!(r.play(3), Played::Entry(expected));
    assert_eq!(r.skipped_count(), 2);
    assert!(r.take_fault().is_none());
}

#[test]
fn a_backwards_grid_index_is_a_broken_invariant() {
    let mut r = anchored_at(10, 3, 0);
    r.play(10);
    assert_eq!(r.play(9), Played::Drained);
    assert_eq!(
        r.take_fault().map(|v| (v & 0xFFFF) as i16 as i32),
        Some(RUNTIME_ERR_INTERNAL_INVARIANT)
    );
}

#[test]
fn reset_requires_the_next_run_to_re_anchor() {
    let mut r = anchored_at(10, 3, 0);
    r.reset();
    assert!(r.is_empty());
    assert_eq!(r.origin_mm(), None);
    assert!(matches!(
        r.fill(&header(13, false), &entries(1, 0)),
        Err(RingFault::Rejected(SampleRunError::NotAnchored { .. }))
    ));
}

#[test]
fn grid_indices_come_off_the_dc_period_exactly() {
    let mut grid = SampleGrid::new(u64::from(INTERVAL));
    let base = 8_123_456_789;
    assert_eq!(grid.index_of(base), Ok(0));
    assert_eq!(grid.index_of(base + u64::from(INTERVAL) * 7), Ok(7));
    assert_eq!(
        grid.index_of(base + u64::from(INTERVAL) * 7 + 13),
        Err(GridPhaseError {
            mono_ns: base + u64::from(INTERVAL) * 7 + 13,
            base_mono_ns: base,
            residual_ns: 13,
        })
    );
}

#[test]
fn executor_parses_only_the_cli_spellings() {
    assert_eq!(Executor::parse("piece"), Some(Executor::Piece));
    assert_eq!(
        Executor::parse("setpoint-ring"),
        Some(Executor::SetpointRing)
    );
    assert_eq!(Executor::parse("setpoint_ring"), None);
    assert_eq!(Executor::SetpointRing.wire(), Executor::WIRE_SETPOINT_RING);
}
