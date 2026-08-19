//! A lane that sits out single-lane traffic (motors_sync-style nudges) while
//! the stream stays globally continuous resumes with a forward hole in its
//! own timeline. The pump must sanction that gap at enqueue when the lane
//! parked at rest — and stay loud when it did not.

use super::*;
use crossbeam_channel::unbounded;
use runtime::piece_ring::PieceEntry;
use std::sync::atomic::AtomicU64;
use std::sync::{Arc, Mutex};
use std::time::Duration;

const FREQ: f64 = 1_000_000.0;

#[derive(Clone, Default)]
struct MarkRecordingSink {
    seam_gaps: Arc<Mutex<Vec<(AxisKey, u64)>>>,
    reanchors: Arc<Mutex<Vec<(AxisKey, u64)>>>,
}

impl PieceSink for MarkRecordingSink {
    fn send_frame(
        &self,
        _key: AxisKey,
        _pieces: &[PieceEntry],
        _new_head: u32,
        _room: u32,
    ) -> Result<i32, SendError> {
        Ok(mcu_protocol::result_codes::OK)
    }

    fn mark_seam_gap(&self, key: AxisKey, at_start_clock: u64) {
        self.seam_gaps.lock().unwrap().push((key, at_start_clock));
    }

    fn mark_reanchor(&self, key: AxisKey, at_start_clock: u64, _epoch_freq: Option<f64>) {
        self.reanchors.lock().unwrap().push((key, at_start_clock));
    }
}

fn hold_piece(start_time: u64, duration: f32) -> (PieceEntry, f64) {
    hold_piece_at(start_time, duration, 1.0)
}

fn hold_piece_at(start_time: u64, duration: f32, pos: f32) -> (PieceEntry, f64) {
    let mut p = PieceEntry::zeroed();
    p.start_time = start_time;
    p.duration = duration;
    p.coeff_count = 1;
    p.coeffs[0] = pos;
    (p, start_time as f64 / FREQ)
}

fn moving_piece(start_time: u64, duration: f32) -> (PieceEntry, f64) {
    let mut p = PieceEntry::zeroed();
    p.start_time = start_time;
    p.duration = duration;
    p.coeff_count = 2;
    p.coeffs[0] = 1.0;
    p.coeffs[1] = 0.5;
    (p, start_time as f64 / FREQ)
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
    pieces: Vec<(PieceEntry, f64)>,
    epoch: crate::anchor::StreamEpoch,
) {
    data.send(EnqueueMsg {
        epoch_freq: None,
        key,
        pieces,
        epoch,
        lead_secs: MAX_LEAD_SECS,
        source_line: u32::MAX,
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
            vec![hold_piece(1_000_000, 0.01)],
            crate::anchor::StreamEpoch::Continuation,
        );
        enqueue(
            data,
            key,
            vec![hold_piece(resume_start, 0.01)],
            crate::anchor::StreamEpoch::Continuation,
        );
    });
    assert_eq!(
        sink.seam_gaps.lock().unwrap().as_slice(),
        &[(key, resume_start)],
        "the ~4s lane-local hole after an at-rest park must be sanctioned"
    );
}

#[test]
fn a_lane_hole_after_a_moving_park_stays_unmarked() {
    let key = AxisKey { mcu_id: 0, axis: 1 };
    let sink = with_pump(|_, data| {
        enqueue(
            data,
            key,
            vec![moving_piece(1_000_000, 0.01)],
            crate::anchor::StreamEpoch::Continuation,
        );
        enqueue(
            data,
            key,
            vec![hold_piece_at(5_000_000, 0.01, 1.5)],
            crate::anchor::StreamEpoch::Continuation,
        );
    });
    assert!(
        sink.seam_gaps.lock().unwrap().is_empty(),
        "a hole after a piece that ended in motion is missing trajectory — \
         it must stay loud downstream, not be sanctioned"
    );
}

#[test]
fn a_contiguous_continuation_is_never_marked() {
    let key = AxisKey { mcu_id: 0, axis: 1 };
    let first = hold_piece(1_000_000, 0.01);
    #[allow(clippy::cast_possible_truncation)]
    let next_start = first.0.end_time(FREQ as f32);
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
            vec![hold_piece(next_start, 0.01)],
            crate::anchor::StreamEpoch::Continuation,
        );
    });
    assert!(
        sink.seam_gaps.lock().unwrap().is_empty(),
        "abutting pieces must not spend the seam guard's strictness"
    );
}

#[test]
fn a_fresh_epoch_resume_is_marked_as_reanchor_not_gap() {
    let key = AxisKey { mcu_id: 0, axis: 1 };
    let sink = with_pump(|_, data| {
        enqueue(
            data,
            key,
            vec![hold_piece(1_000_000, 0.01)],
            crate::anchor::StreamEpoch::Continuation,
        );
        enqueue(
            data,
            key,
            vec![hold_piece(5_000_000, 0.01)],
            crate::anchor::StreamEpoch::Reanchor,
        );
    });
    assert!(
        sink.seam_gaps.lock().unwrap().is_empty(),
        "a retimed epoch already cuts the seam; gap marking must not double-fire"
    );
    assert_eq!(
        sink.reanchors.lock().unwrap().as_slice(),
        &[(key, 5_000_000)],
    );
}
