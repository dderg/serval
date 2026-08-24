use std::sync::Arc;

use mcu_protocol::messages::LANE_RUN_FLAG_REANCHOR;
use trajectory::{ContinuousAxis, MotorGroup, MotorSpan, MotorTerm, NudgeProfile};

use super::*;

const INTERVAL: u64 = 250_000;
const CPM: f64 = 3_276.8;
const GRID_INDEX: u64 = 1_000;
const GRID_CLOCK: u64 = 4_000_000_000;

fn clocked(signal: Arc<MotorSpan>, start_ns: u64, freq_hz: f64) -> ClockedMotorSpan {
    #[allow(clippy::cast_precision_loss)]
    let start_clock_exact = start_ns as f64;
    let duration = signal.t_end - signal.t_start;
    let start_host = start_clock_exact / CLOCK_FREQ_HZ;
    ClockedMotorSpan::try_new(
        Arc::clone(&signal),
        signal.t_start,
        signal.t_end,
        start_host,
        start_host + duration,
        start_clock_exact,
        freq_hz,
    )
    .expect("a positive-duration view on a positive clock")
}

fn linear_signal(duration_s: f64, from_mm: f64, to_mm: f64) -> Arc<MotorSpan> {
    let delta = to_mm - from_mm;
    let profile =
        NudgeProfile::try_new(delta, delta.abs() / duration_s, 0.0, 0.0).expect("cruise profile");
    let duration = profile.duration();
    let groups: Arc<[MotorGroup]> = Arc::from([
        MotorGroup::Independent(MotorTerm {
            source_axis: 0,
            axis: ContinuousAxis::Hold {
                position: from_mm,
                t_start: 0.0,
                t_end: duration,
            },
            scale: 1.0,
        }),
        MotorGroup::Independent(MotorTerm {
            source_axis: 0,
            axis: ContinuousAxis::Nudge(profile),
            scale: 1.0,
        }),
    ]);
    Arc::new(MotorSpan::try_new(groups, 0.0, duration, 0, 7, false).expect("motor span"))
}

fn linear_span(start_ns: u64, duration_s: f64, from_mm: f64, to_mm: f64) -> ClockedMotorSpan {
    clocked(
        linear_signal(duration_s, from_mm, to_mm),
        start_ns,
        CLOCK_FREQ_HZ,
    )
}

fn lane_filler(lanes: usize, ff_lead_ns: u64) -> ChainFiller {
    let specs: Vec<LaneSpec> = (0..lanes)
        .map(|axis| LaneSpec {
            axis: axis as u8,
            cmd_counts_per_mm: CPM,
            ff_lead_ns,
        })
        .collect();
    let mut f = ChainFiller::new(&specs, None, INTERVAL, 400);
    f.observe_grid(GRID_INDEX, GRID_CLOCK)
        .expect("the grid only advances");
    f
}

fn filler(lanes: usize) -> ChainFiller {
    lane_filler(lanes, 0)
}

#[test]
fn a_lane_run_starts_anchored_on_the_grid_index_covering_the_span() {
    let mut f = filler(1);
    f.push_spans(
        0,
        &[linear_span(GRID_CLOCK + INTERVAL * 4, 0.001, 0.0, 1.0)],
    )
    .expect("a fresh lane takes the view");
    let runs = f.drain().expect("fill");
    assert_eq!(runs.len(), 1);
    let run = &runs[0];
    assert_eq!(run.start_index, GRID_INDEX + 4);
    assert_eq!(run.interval_ticks, INTERVAL as u32);
    assert_eq!(run.flags & LANE_RUN_FLAG_REANCHOR, LANE_RUN_FLAG_REANCHOR);
    assert_eq!(run.sample_count as usize, run.samples.len());
    assert_eq!(run.samples.len(), 4);
    assert_eq!(run.samples[0].pos_counts, 0);
}

#[test]
#[allow(clippy::cast_possible_truncation)]
fn positions_are_the_span_evaluated_on_the_dc_grid_in_anchored_counts() {
    let start = GRID_CLOCK + INTERVAL * 4;
    let span = linear_span(start, 0.001, 0.0, 1.0);
    let mut f = filler(1);
    f.push_spans(0, std::slice::from_ref(&span))
        .expect("a fresh lane takes the view");
    let runs = f.drain().expect("fill");
    let origin = span.eval_at_clock(start).expect("in span").position;
    for (step, sample) in runs[0].samples.iter().enumerate() {
        let clock = start + INTERVAL * step as u64;
        let pva = span.eval_at_clock(clock).expect("in span");
        assert_eq!(
            sample.pos_counts,
            crate::scale::mm_to_counts(pva.position - origin, CPM)
        );
        assert_eq!(sample.vel_ff, (pva.velocity * CPM).round() as i32);
    }
    assert_eq!(
        runs[0].origin_mm_q16,
        (origin * 65536.0).round() as i32,
        "the anchor origin rides the wire so the endpoint can rebuild host mm"
    );
}

#[test]
fn the_next_drain_abuts_without_re_anchoring() {
    let start = GRID_CLOCK + INTERVAL * 4;
    let mut f = filler(1);
    f.push_spans(0, &[linear_span(start, 0.001, 0.0, 1.0)])
        .expect("stage");
    let first = f.drain().expect("first fill");
    let next_index = first[0].start_index + u64::from(first[0].sample_count);
    f.push_spans(0, &[linear_span(start + 1_000_000, 0.001, 1.0, 2.0)])
        .expect("stage the successor once the first is consumed");
    let second = f.drain().expect("second fill");
    assert_eq!(second.len(), 1);
    assert_eq!(second[0].start_index, next_index);
    assert_eq!(second[0].flags & LANE_RUN_FLAG_REANCHOR, 0);
    assert_eq!(second[0].origin_mm_q16, first[0].origin_mm_q16);
}

#[test]
fn a_coverage_gap_closes_the_run_and_the_resume_re_anchors() {
    let start = GRID_CLOCK + INTERVAL * 4;
    let gap_start = start + 1_000_000 + INTERVAL * 8;
    let mut f = filler(1);
    f.push_spans(
        0,
        &[
            linear_span(start, 0.001, 0.0, 1.0),
            linear_span(gap_start, 0.001, 5.0, 6.0),
        ],
    )
    .expect("both views fit the lane's two slots");
    let first = f.drain().expect("first fill");
    assert_eq!(first.len(), 1);
    assert_eq!(first[0].samples.len(), 4, "run stops where coverage stops");
    let second = f.drain().expect("second fill");
    assert_eq!(second.len(), 1);
    assert_eq!(
        second[0].flags & LANE_RUN_FLAG_REANCHOR,
        LANE_RUN_FLAG_REANCHOR,
        "resuming across a gap is a new anchor epoch"
    );
    assert_eq!(second[0].samples[0].pos_counts, 0);
}

#[test]
fn one_drain_never_exceeds_the_per_frame_cap() {
    let start = GRID_CLOCK + INTERVAL;
    let mut f = filler(1);
    f.push_spans(0, &[linear_span(start, 0.1, 0.0, 100.0)])
        .expect("stage");
    let runs = f.drain().expect("fill");
    assert_eq!(runs[0].samples.len(), MAX_FILL_CYCLES);
    assert!(f.wants_drain());
}

#[test]
fn every_lane_of_the_chain_fills_from_one_drain() {
    let start = GRID_CLOCK + INTERVAL * 2;
    let mut f = filler(3);
    f.push_spans(0, &[linear_span(start, 0.001, 0.0, 1.0)])
        .expect("stage");
    f.push_spans(2, &[linear_span(start, 0.001, 0.0, -1.0)])
        .expect("stage");
    let runs = f.drain().expect("fill");
    let axes: Vec<u8> = runs.iter().map(|r| r.axis_idx).collect();
    assert_eq!(axes, vec![0, 2]);
    assert!(runs[1].samples[3].pos_counts < 0);
}

#[test]
fn a_buzz_streams_through_the_same_runs() {
    let mut f = filler(2);
    let start = GRID_CLOCK + INTERVAL * 400;
    assert_eq!(
        f.arm_buzz(0b01, 0, 40_000, 40_000, 20_000, 500, 5, start),
        0
    );
    assert!(f.buzz_active());
    let runs = f.drain().expect("buzz fill");
    assert_eq!(runs.len(), 1, "only the driven lane gets samples");
    assert_eq!(runs[0].axis_idx, 0);
    assert_eq!(runs[0].start_index, GRID_INDEX + 400);
    assert_eq!(
        runs[0].flags & LANE_RUN_FLAG_REANCHOR,
        LANE_RUN_FLAG_REANCHOR
    );
    assert_eq!(
        runs[0].samples.len(),
        MAX_FILL_CYCLES,
        "a 500 ms buzz outlives one frame and keeps the pump draining"
    );
    assert!(runs[0].samples.iter().any(|s| s.pos_counts != 0));
    assert!(f.buzz_active());
    assert!(f.wants_drain());
}

#[test]
fn a_buzz_opens_on_the_first_grid_cycle_at_or_after_the_pump_anchor() {
    let mut f = filler(1);
    let start = GRID_CLOCK + INTERVAL * 400 + 1;
    assert_eq!(
        f.arm_buzz(0b01, 0, 40_000, 40_000, 20_000, 500, 5, start),
        0
    );
    let runs = f.drain().expect("buzz fill");
    assert_eq!(
        runs[0].start_index,
        GRID_INDEX + 401,
        "an anchor between two cycles snaps up to the next one this node can play"
    );
}

#[test]
fn an_undriven_lane_cannot_pull_the_buzz_anchor_earlier() {
    let mut f = filler(2);
    f.push_spans(1, &[linear_span(GRID_CLOCK + INTERVAL, 0.001, 0.0, 1.0)])
        .expect("stage trajectory on the lane the sweep leaves alone");
    let start = GRID_CLOCK + INTERVAL * 400;
    assert_eq!(
        f.arm_buzz(0b01, 0, 40_000, 40_000, 20_000, 500, 5, start),
        0
    );
    let runs = f.drain().expect("buzz fill");
    assert_eq!(runs.len(), 1);
    assert_eq!(
        runs[0].start_index,
        GRID_INDEX + 400,
        "the sweep starts on the instant it was armed on, not on the idle lane's index"
    );
}

#[test]
fn one_anchor_starts_every_transport_on_the_same_instant() {
    let anchor = GRID_CLOCK + INTERVAL * 400;
    let mut early = filler(1);
    let mut skewed = ChainFiller::new(
        &[LaneSpec {
            axis: 0,
            cmd_counts_per_mm: CPM,
            ff_lead_ns: 0,
        }],
        None,
        INTERVAL,
        400,
    );
    let skewed_index = 7;
    let skewed_clock = GRID_CLOCK + 3;
    skewed
        .observe_grid(skewed_index, skewed_clock)
        .expect("the grid only advances");
    assert_eq!(
        early.arm_buzz(0b01, 0, 40_000, 40_000, 20_000, 500, 5, anchor),
        0
    );
    assert_eq!(
        skewed.arm_buzz(0b01, 0, 40_000, 40_000, 20_000, 500, 5, anchor),
        0
    );
    let early_start = early.drain().expect("buzz fill")[0].start_index;
    let skewed_start = skewed.drain().expect("buzz fill")[0].start_index;
    let early_clock = GRID_CLOCK + (early_start - GRID_INDEX) * INTERVAL;
    let skewed_start_clock = skewed_clock + (skewed_start - skewed_index) * INTERVAL;
    assert_eq!(
        early_clock, anchor,
        "a node whose grid covers the anchor opens exactly on it"
    );
    assert!(
        skewed_start_clock >= anchor && skewed_start_clock < anchor + INTERVAL,
        "a skewed node opens on its own first cycle at or after the shared anchor: \
         {skewed_start_clock} vs {anchor}"
    );
}

#[test]
fn a_buzz_is_refused_before_the_endpoint_grid_is_known() {
    let specs = [LaneSpec {
        axis: 0,
        cmd_counts_per_mm: CPM,
        ff_lead_ns: 0,
    }];
    let mut f = ChainFiller::new(&specs, None, INTERVAL, 400);
    assert_eq!(
        f.arm_buzz(0b01, 0, 40_000, 40_000, 20_000, 500, 5, GRID_CLOCK),
        ERR_BUZZ_UNGRIDDED_START
    );
    assert!(!f.buzz_active(), "a refused arming leaves nothing armed");
}

#[test]
fn a_buzz_anchored_before_the_observed_grid_is_refused() {
    let mut f = filler(1);
    assert_eq!(
        f.arm_buzz(0b01, 0, 40_000, 40_000, 20_000, 500, 5, GRID_CLOCK - 1),
        ERR_BUZZ_START_IN_PAST
    );
    assert!(!f.buzz_active());
    assert!(!f.wants_drain());
}

#[test]
fn a_reset_makes_the_next_run_re_anchor() {
    let start = GRID_CLOCK + INTERVAL * 4;
    let mut f = filler(1);
    f.push_spans(0, &[linear_span(start, 0.001, 0.0, 1.0)])
        .expect("stage");
    f.drain().expect("first fill");
    assert_eq!(f.reset(), 1, "the converted view was never proven played");
    assert!(!f.wants_drain());
    f.push_spans(0, &[linear_span(start + 1_000_000, 0.001, 9.0, 10.0)])
        .expect("stage");
    let runs = f.drain().expect("post-reset fill");
    assert_eq!(
        runs[0].flags & LANE_RUN_FLAG_REANCHOR,
        LANE_RUN_FLAG_REANCHOR
    );
}

#[test]
fn nothing_is_filled_before_the_endpoint_grid_is_known() {
    let specs = [LaneSpec {
        axis: 0,
        cmd_counts_per_mm: CPM,
        ff_lead_ns: 0,
    }];
    let mut f = ChainFiller::new(&specs, None, INTERVAL, 400);
    f.push_spans(0, &[linear_span(GRID_CLOCK, 0.001, 0.0, 1.0)])
        .expect("stage");
    assert!(f.drain().expect("no grid, no runs").is_empty());
}

#[test]
fn a_lane_holds_an_active_view_and_a_successor_and_no_more() {
    let start = GRID_CLOCK + INTERVAL * 4;
    let mut f = filler(1);
    assert_eq!(f.free_span_slots(0), LANE_SPAN_SLOTS);
    f.push_spans(0, &[linear_span(start, 0.001, 0.0, 1.0)])
        .expect("active");
    assert_eq!(f.free_span_slots(0), 1);
    f.push_spans(0, &[linear_span(start + 1_000_000, 0.001, 1.0, 2.0)])
        .expect("successor");
    assert_eq!(f.free_span_slots(0), 0);
    assert_eq!(
        f.push_spans(0, &[linear_span(start + 2_000_000, 0.001, 2.0, 3.0)]),
        Err(FillError::SpanSlotsFull { axis: 0 }),
        "a third view has nowhere to go — the scheduler must pace on the slots"
    );
}

#[test]
fn a_view_mapped_on_a_foreign_clock_is_rejected() {
    let mut f = filler(1);
    let foreign = clocked(linear_signal(0.001, 0.0, 1.0), GRID_CLOCK, 1_000_000.0);
    assert_eq!(
        f.push_spans(0, &[foreign]),
        Err(FillError::SpanClockMismatch {
            axis: 0,
            clock_freq_hz: 1_000_000.0
        })
    );
}

#[test]
fn a_view_behind_the_staged_tail_is_rejected() {
    let start = GRID_CLOCK + INTERVAL * 8;
    let mut f = filler(1);
    let ahead = linear_span(start, 0.001, 0.0, 1.0);
    let behind = linear_span(start - 500_000, 0.001, 5.0, 6.0);
    let previous_end = ahead.end_clock;
    let start_clock = behind.start_clock;
    f.push_spans(0, &[ahead]).expect("stage");
    assert_eq!(
        f.push_spans(0, &[behind]),
        Err(FillError::SpanOutOfOrder {
            axis: 0,
            start_clock,
            previous_end
        })
    );
}

#[test]
fn conversion_consumes_a_view_and_only_playback_proof_retires_it() {
    let start = GRID_CLOCK + INTERVAL * 4;
    let first = linear_span(start, 0.001, 0.0, 1.0);
    let second = linear_span(start + 1_000_000, 0.001, 1.0, 2.0);
    let (first_end, second_end) = (first.end_clock, second.end_clock);
    let mut f = filler(1);
    f.push_spans(0, &[first, second]).expect("stage both");
    assert_eq!(f.take_consumed(0), 0, "staging converts nothing");

    let runs = f.drain().expect("fill");
    assert_eq!(runs[0].samples.len(), 8);
    assert_eq!(f.take_consumed(0), 2, "both views were fully converted");
    assert_eq!(f.take_consumed(0), 0, "consumption is credited once");
    assert_eq!(
        f.free_span_slots(0),
        LANE_SPAN_SLOTS,
        "released views free their staging slots"
    );

    assert_eq!(
        f.retire_through(0, first_end - 1),
        0,
        "playback short of the end retires nothing"
    );
    assert_eq!(f.retire_through(0, first_end), 1);
    assert_eq!(f.retire_through(0, second_end), 1);
    assert_eq!(f.retire_through(0, second_end), 0);
}

#[test]
fn a_cut_abandons_unresolved_views_without_retiring_them() {
    let start = GRID_CLOCK + INTERVAL * 4;
    let mut f = filler(1);
    f.push_spans(
        0,
        &[
            linear_span(start, 0.001, 0.0, 1.0),
            linear_span(start + 1_000_000, 0.001, 1.0, 2.0),
        ],
    )
    .expect("stage both");
    f.drain().expect("fill");
    assert_eq!(f.cut_axis(0), 2, "two converted views were never proven");
    assert_eq!(
        f.retire_through(0, u64::MAX),
        0,
        "an abandoned view is never retired"
    );
    assert_eq!(f.free_span_slots(0), LANE_SPAN_SLOTS);
    assert!(!f.wants_drain());
}

#[test]
fn a_cut_abandons_a_staged_view_that_never_reached_the_ring() {
    let start = GRID_CLOCK + INTERVAL * 4;
    let mut f = filler(1);
    f.push_spans(0, &[linear_span(start, 0.001, 0.0, 1.0)])
        .expect("stage");
    assert_eq!(f.cut_axis(0), 1);
    assert!(!f.wants_drain());
}

#[test]
#[allow(clippy::cast_possible_truncation)]
fn the_feedforward_lead_reaches_across_into_the_successor() {
    let start = GRID_CLOCK + INTERVAL * 4;
    let lead = INTERVAL * 2;
    let mut f = lane_filler(1, lead);
    let successor = linear_span(start + 1_000_000, 0.001, 1.0, 3.0);
    let lead_pva = successor
        .eval_at_clock(start + INTERVAL * 3 + lead)
        .expect("the lead lands inside the successor");
    f.push_spans(0, &[linear_span(start, 0.001, 0.0, 1.0), successor.clone()])
        .expect("stage both");
    let runs = f.drain().expect("fill");
    assert_eq!(
        runs[0].samples[3].vel_ff,
        (lead_pva.velocity * CPM).round() as i32,
        "the last sample of the active view already carries the successor's velocity"
    );
    assert!(
        lead_pva.velocity > 1_500.0,
        "the successor must be moving faster than the active view for this to bite"
    );
}
