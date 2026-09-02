//! A lane that holds while the other axes print gets its hold views coalesced.
//! The merge builds a fresh `Hold` motor span over the combined clock interval,
//! so the only things it may absorb are abutting explicit holds whose evaluated
//! positions are bit-identical — and never past the dispatch span bound every
//! endpoint cursor relies on.

use super::pump_loop::Pump;
use super::sched::append_spans_merging_holds;
use super::stall::ConsumptionStallWatch;
use super::{AxisKey, AxisQueue, EnqueueMsg, PumpCallbacks, PumpMsg, SendError, SpanSink};
use crate::pump::MAX_LEAD_SECS;
use std::collections::{BTreeMap, VecDeque};
use std::sync::Arc;
use std::sync::atomic::AtomicU64;
use trajectory::{
    ClockedMotorSpan, ContinuousAxis, MAX_SPAN_SECS, MotorGroup, MotorSpan, MotorTerm,
};

/// The Voron 0 main mcu: an F103 at 72 MHz.
const EPOCH_FREQ: f64 = 72_000_000.0;
const ANCHOR: u64 = 869_400_000_000;
const HOLD_SECS: f64 = 0.005;
const HOLD_POSITION: f64 = 3.25;
const SOURCE_AXIS: usize = 2;
const SOURCE_LINE: u32 = 7;

fn view(t_start: f64, secs: f64, position: f64, explicit_hold: bool) -> ClockedMotorSpan {
    let t_end = t_start + secs;
    let groups: Arc<[MotorGroup]> = Arc::from(vec![MotorGroup::Independent(MotorTerm {
        source_axis: SOURCE_AXIS,
        axis: ContinuousAxis::Hold {
            position,
            t_start,
            t_end,
        },
        scale: 1.0,
    })]);
    let signal = MotorSpan::try_new(groups, t_start, t_end, 0, SOURCE_LINE, explicit_hold)
        .expect("a hold motor span is dispatchable");
    ClockedMotorSpan::try_new(
        Arc::new(signal),
        t_start,
        t_end,
        t_start,
        t_end,
        ANCHOR as f64 + t_start * EPOCH_FREQ,
        EPOCH_FREQ,
    )
    .expect("the projected view spans at least one clock")
}

fn hold(index: u64) -> ClockedMotorSpan {
    view(index as f64 * HOLD_SECS, HOLD_SECS, HOLD_POSITION, true)
}

fn merged_run(count: u64) -> VecDeque<ClockedMotorSpan> {
    let mut queue = VecDeque::new();
    for index in 0..count {
        append_spans_merging_holds(&mut queue, vec![hold(index)], true);
    }
    queue
}

fn assert_tiles(
    queue: &VecDeque<ClockedMotorSpan>,
    first: &ClockedMotorSpan,
    last: &ClockedMotorSpan,
) {
    let mut clock = first.start_clock;
    for staged in queue {
        assert_eq!(
            staged.start_clock, clock,
            "staged views must tile the run without gap or overlap"
        );
        clock = staged.end_clock;
    }
    assert_eq!(clock, last.end_clock);
}

#[test]
fn abutting_identical_holds_collapse_into_one_view() {
    let queue = merged_run(4);
    assert_eq!(queue.len(), 1, "20 ms of holds is one 20 ms view");
    let merged = &queue[0];
    assert!(merged.signal.is_explicit_hold);
    assert_eq!(merged.start_clock, hold(0).start_clock);
    assert_eq!(merged.end_clock, hold(3).end_clock);
    for t in [merged.stream_t_start, merged.stream_t_end] {
        assert_eq!(
            merged.signal.position(t).unwrap().to_bits(),
            HOLD_POSITION.to_bits(),
            "the merged hold reports the absorbed position exactly"
        );
    }
    assert_eq!(
        merged.eval_at_clock(merged.end_clock).unwrap().position,
        HOLD_POSITION
    );
}

#[test]
fn merging_never_exceeds_the_dispatch_span_bound() {
    let queue = merged_run(20);
    assert!(
        queue.len() > 1,
        "100 ms of holds cannot become one view under the 25 ms bound"
    );
    for staged in &queue {
        assert!(
            staged.stream_t_end - staged.stream_t_start <= MAX_SPAN_SECS,
            "merged view of {} s exceeds the dispatch bound",
            staged.stream_t_end - staged.stream_t_start
        );
    }
    assert_tiles(&queue, &hold(0), &hold(19));
}

#[test]
fn a_hold_one_ulp_away_stays_a_separate_view() {
    let mut queue = VecDeque::from([hold(0)]);
    let nudged = f64::from_bits(HOLD_POSITION.to_bits() + 1);
    append_spans_merging_holds(
        &mut queue,
        vec![view(HOLD_SECS, HOLD_SECS, nudged, true)],
        true,
    );
    assert_eq!(queue.len(), 2);
}

#[test]
fn a_hold_across_a_clock_gap_stays_a_separate_view() {
    let mut queue = VecDeque::from([hold(0)]);
    append_spans_merging_holds(&mut queue, vec![hold(2)], true);
    assert_eq!(queue.len(), 2);
    assert!(queue[0].end_clock < queue[1].start_clock);
}

#[test]
fn a_view_not_marked_an_explicit_hold_never_merges() {
    let mut absorbing = VecDeque::from([hold(0)]);
    append_spans_merging_holds(
        &mut absorbing,
        vec![view(HOLD_SECS, HOLD_SECS, HOLD_POSITION, false)],
        true,
    );
    assert_eq!(absorbing.len(), 2, "a non-hold successor is never absorbed");

    let mut absorbed = VecDeque::from([view(0.0, HOLD_SECS, HOLD_POSITION, false)]);
    append_spans_merging_holds(&mut absorbed, vec![hold(1)], true);
    assert_eq!(absorbed.len(), 2, "a non-hold tail never absorbs a hold");
}

#[test]
fn a_fresh_epoch_is_fenced_from_the_staged_tail() {
    let mut queue = VecDeque::from([hold(0)]);
    append_spans_merging_holds(&mut queue, vec![hold(1), hold(2)], false);
    assert_eq!(
        queue.len(),
        2,
        "the first incoming view is fenced from the tail; the rest still merge"
    );
    assert_eq!(queue[0].end_clock, hold(0).end_clock);
    assert_eq!(queue[1].start_clock, hold(1).start_clock);
    assert_eq!(queue[1].end_clock, hold(2).end_clock);
}

#[derive(Clone, Copy)]
struct NullSink;

impl SpanSink for NullSink {
    fn send_frame(
        &self,
        _key: AxisKey,
        _spans: &[ClockedMotorSpan],
        _new_head: u32,
        _room: u32,
    ) -> Result<i32, SendError> {
        Ok(mcu_protocol::result_codes::OK)
    }
}

fn pump_with(callbacks: PumpCallbacks) -> Pump<NullSink> {
    Pump {
        queues: BTreeMap::new(),
        junctions: super::JunctionTracker::default(),
        cohort: None,
        halted: BTreeMap::new(),
        sink: NullSink,
        callbacks,
        history: None,
        ledger: Arc::new(crate::drain::DrainLedger::new()),
        pending_barrier_acks: Vec::new(),
        backlog: Arc::new(AtomicU64::new(0)),
        release_plan: crate::pump::ReleasePlan::default(),
        data_open: true,
        intake_batch_open: false,
        consumption_stall: ConsumptionStallWatch::new(std::time::Duration::from_secs(60)),
        mem_probe: super::memstat::MemPressureProbe::new(),
    }
}

fn synced_pump() -> Pump<NullSink> {
    pump_with(PumpCallbacks {
        mcu_clock_of: Box::new(|_| Some((ANCHOR, EPOCH_FREQ))),
        ..PumpCallbacks::noop(super::stepcompress_sink::SHIM_RING_DEPTH)
    })
}

fn enqueue_run(pump: &mut Pump<NullSink>, key: AxisKey, count: u64) {
    for index in 0..count {
        pump.enqueue(EnqueueMsg {
            key,
            spans: vec![hold(index)],
            epoch: crate::anchor::StreamEpoch::Continuation,
            lead_secs: MAX_LEAD_SECS,
            source_line: SOURCE_LINE,
            epoch_freq: None,
            batch_end: true,
        });
    }
}

#[test]
fn the_pump_merges_staged_holds() {
    let key = AxisKey { mcu_id: 0, axis: 2 };
    let mut pump = synced_pump();
    enqueue_run(&mut pump, key, 4);

    let queue: &AxisQueue = pump.queues.get(&key).expect("the lane was staged");
    assert_eq!(queue.spans.len(), 1);
    assert_eq!(queue.staged_motion, 0, "a hold run carries no motion");
    assert_eq!(queue.seam_end_clock, Some(hold(3).end_clock));
    assert!(queue.seam_end_at_rest);
}

#[test]
fn a_drip_cohort_keeps_every_hold_view_separate() {
    let key = AxisKey { mcu_id: 0, axis: 2 };
    let mut pump = synced_pump();
    assert!(pump.handle_control_msg(PumpMsg::DripArm(super::DripArm {
        cohort: 1,
        participants: vec![key],
        timeout: std::time::Duration::from_secs(60),
    })));
    enqueue_run(&mut pump, key, 4);

    let queue: &AxisQueue = pump.queues.get(&key).expect("the lane was staged");
    assert_eq!(
        queue.spans.len(),
        4,
        "the cohort release floor counts views, so coalescing would starve it"
    );
    assert_tiles(&queue.spans, &hold(0), &hold(3));
}
