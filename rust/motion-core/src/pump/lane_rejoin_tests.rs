//! A lane that sits out single-lane traffic (motors_sync-style nudges) while
//! the stream stays globally continuous resumes with a forward hole in its
//! own timeline. The pump must sanction that gap at enqueue when the lane
//! parked at rest — and stay loud when it did not.

use super::*;
use crate::lock_ext::LockExt;
use crossbeam_channel::unbounded;
use std::sync::atomic::AtomicU64;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use trajectory::{
    ClockedMotorSpan, ContinuousAxis, MotorGroup, MotorSpan, MotorTerm, NudgeProfile,
};

const FREQ: f64 = 1_000_000.0;
const SOURCE_AXIS: usize = 1;
const SOURCE_LINE: u32 = 42;
const PARKED_POSITION: f64 = 1.0;

#[derive(Clone, Default)]
struct MarkRecordingSink {
    seam_gaps: Arc<Mutex<Vec<(AxisKey, u64)>>>,
    reanchors: Arc<Mutex<Vec<(AxisKey, u64)>>>,
}

impl SpanSink for MarkRecordingSink {
    fn send_frame(
        &self,
        _key: AxisKey,
        _spans: &[ClockedMotorSpan],
        _new_head: u32,
        _room: u32,
    ) -> Result<i32, SendError> {
        Ok(mcu_protocol::result_codes::OK)
    }

    fn mark_seam_gap(&self, key: AxisKey, at_start_clock: u64) {
        self.seam_gaps.lock_ok().push((key, at_start_clock));
    }

    fn mark_reanchor(&self, key: AxisKey, at_start_clock: u64, _epoch_freq: Option<f64>) {
        self.reanchors.lock_ok().push((key, at_start_clock));
    }
}

#[allow(clippy::cast_precision_loss)]
fn clocked(
    signal: Arc<MotorSpan>,
    stream_t_start: f64,
    stream_t_end: f64,
    start_clock: u64,
) -> ClockedMotorSpan {
    ClockedMotorSpan::try_new(
        signal,
        stream_t_start,
        stream_t_end,
        stream_t_start,
        stream_t_end,
        start_clock as f64,
        FREQ,
    )
    .expect("the projected view spans at least one clock")
}

fn hold_span(start_clock: u64, secs: f64) -> ClockedMotorSpan {
    hold_span_at(start_clock, secs, PARKED_POSITION)
}

#[allow(clippy::cast_precision_loss)]
fn hold_span_at(start_clock: u64, secs: f64, position: f64) -> ClockedMotorSpan {
    let t_start = start_clock as f64 / FREQ;
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
    let signal = MotorSpan::try_new(groups, t_start, t_end, 0, SOURCE_LINE, true)
        .expect("a hold motor span is dispatchable");
    clocked(Arc::new(signal), t_start, t_end, start_clock)
}

/// A view that ends mid-travel: the lane it belongs to did not park, so a
/// later hole in its timeline is missing trajectory rather than a dwell.
#[allow(clippy::cast_precision_loss)]
fn moving_span(start_clock: u64, secs: f64) -> ClockedMotorSpan {
    let t_start = start_clock as f64 / FREQ;
    let profile = NudgeProfile::try_new(1.0, 100.0, 0.0, t_start).expect("cruise profile");
    let t_end = t_start + profile.duration();
    assert!(
        secs < profile.duration(),
        "the view must end inside the travel, or the lane parks at rest"
    );
    let groups: Arc<[MotorGroup]> = Arc::from(vec![
        MotorGroup::Independent(MotorTerm {
            source_axis: SOURCE_AXIS,
            axis: ContinuousAxis::Nudge(profile),
            scale: 1.0,
        }),
        MotorGroup::Independent(MotorTerm {
            source_axis: SOURCE_AXIS,
            axis: ContinuousAxis::Hold {
                position: PARKED_POSITION,
                t_start,
                t_end,
            },
            scale: 1.0,
        }),
    ]);
    let signal = MotorSpan::try_new(groups, t_start, t_end, 0, SOURCE_LINE, false)
        .expect("a nudge motor span is dispatchable");
    clocked(Arc::new(signal), t_start, t_start + secs, start_clock)
}

fn with_pump(
    body: impl FnOnce(&crossbeam_channel::Sender<PumpMsg>, &crossbeam_channel::Sender<EnqueueMsg>),
) -> MarkRecordingSink {
    let sink = MarkRecordingSink::default();
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();
    let sink_clone = sink.clone();
    let handle = std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            sink_clone,
            PumpCallbacks {
                mcu_clock_of: Box::new(|_| Some((0, FREQ))),
                ..PumpCallbacks::noop(64)
            },
            None,
            Arc::new(crate::drain::DrainLedger::new()),
            Arc::new(AtomicU64::new(0)),
        );
    });
    body(&ctl, &data);
    std::thread::sleep(Duration::from_millis(150));
    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
    sink
}

fn enqueue(
    data: &crossbeam_channel::Sender<EnqueueMsg>,
    key: AxisKey,
    spans: Vec<ClockedMotorSpan>,
    epoch: crate::anchor::StreamEpoch,
) {
    data.send(EnqueueMsg {
        epoch_freq: None,
        key,
        spans,
        epoch,
        lead_secs: MAX_LEAD_SECS,
        source_line: SOURCE_LINE,
        batch_end: true,
    })
    .unwrap();
}

#[test]
fn a_lane_resuming_at_rest_across_a_hole_gets_a_sanctioned_seam_gap() {
    let key = AxisKey { mcu_id: 0, axis: 1 };
    let resume_start = 5_000_000_u64;
    let sink = with_pump(|_, data| {
        enqueue(
            data,
            key,
            vec![hold_span(1_000_000, 0.01)],
            crate::anchor::StreamEpoch::Continuation,
        );
        enqueue(
            data,
            key,
            vec![hold_span(resume_start, 0.01)],
            crate::anchor::StreamEpoch::Continuation,
        );
    });
    assert_eq!(
        sink.seam_gaps.lock_ok().as_slice(),
        &[(key, resume_start)],
        "the ~4s lane-local hole after an at-rest park must be sanctioned"
    );
}

#[test]
fn a_lane_hole_after_a_moving_park_stays_unmarked() {
    let key = AxisKey { mcu_id: 0, axis: 1 };
    let travelling = moving_span(1_000_000, 0.005);
    let resume_position = travelling
        .signal
        .position(travelling.stream_t_end)
        .expect("the view evaluates at its own end");
    let sink = with_pump(|_, data| {
        enqueue(
            data,
            key,
            vec![travelling],
            crate::anchor::StreamEpoch::Continuation,
        );
        enqueue(
            data,
            key,
            vec![hold_span_at(5_000_000, 0.01, resume_position)],
            crate::anchor::StreamEpoch::Continuation,
        );
    });
    assert!(
        sink.seam_gaps.lock_ok().is_empty(),
        "a hole after a view that ended in motion is missing trajectory — \
         it must stay loud downstream, not be sanctioned"
    );
}

#[test]
fn a_contiguous_continuation_is_never_marked() {
    let key = AxisKey { mcu_id: 0, axis: 1 };
    let first = hold_span(1_000_000, 0.01);
    let next_start = first.end_clock;
    let sink = with_pump(|_, data| {
        enqueue(
            data,
            key,
            vec![first],
            crate::anchor::StreamEpoch::Continuation,
        );
        enqueue(
            data,
            key,
            vec![hold_span(next_start, 0.01)],
            crate::anchor::StreamEpoch::Continuation,
        );
    });
    assert!(
        sink.seam_gaps.lock_ok().is_empty(),
        "abutting views must not spend the seam guard's strictness"
    );
}

#[test]
fn a_fresh_epoch_resume_is_marked_as_reanchor_not_gap() {
    let key = AxisKey { mcu_id: 0, axis: 1 };
    let sink = with_pump(|_, data| {
        enqueue(
            data,
            key,
            vec![hold_span(1_000_000, 0.01)],
            crate::anchor::StreamEpoch::Continuation,
        );
        enqueue(
            data,
            key,
            vec![hold_span(5_000_000, 0.01)],
            crate::anchor::StreamEpoch::Reanchor,
        );
    });
    assert!(
        sink.seam_gaps.lock_ok().is_empty(),
        "a retimed epoch already cuts the seam; gap marking must not double-fire"
    );
    assert_eq!(sink.reanchors.lock_ok().as_slice(), &[(key, 5_000_000)],);
}

/// The EtherCAT explicit-hold `Reposition` (`enqueue.rs`) carries no views at
/// all: its whole job is to retire the lane's junction across a redefined
/// position. The committed seam it leaves behind belongs to the timeline that
/// just retired, so the next continuation must not be measured against it —
/// doing so sanctions a hole that is not one (at rest) or reports missing
/// trajectory that is not missing (mid-motion).
#[test]
fn a_fresh_epoch_without_views_retires_the_committed_seam() {
    let key = AxisKey { mcu_id: 0, axis: 1 };
    let sink = with_pump(|_, data| {
        enqueue(
            data,
            key,
            vec![hold_span(1_000_000, 0.01)],
            crate::anchor::StreamEpoch::Continuation,
        );
        enqueue(
            data,
            key,
            Vec::new(),
            crate::anchor::StreamEpoch::Reposition,
        );
        enqueue(
            data,
            key,
            vec![hold_span(5_000_000, 0.01)],
            crate::anchor::StreamEpoch::Continuation,
        );
    });
    assert!(
        sink.seam_gaps.lock_ok().is_empty(),
        "the seam the reposition retired must not make the next continuation \
         look like a sat-out lane hole"
    );
    assert!(
        sink.reanchors.lock_ok().is_empty(),
        "a reposition with no views names no clock to cut the stream at"
    );
}
