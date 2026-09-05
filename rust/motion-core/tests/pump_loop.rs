use std::sync::atomic::AtomicU64;
use std::sync::{Arc, Mutex};

use crossbeam_channel::{Receiver, TrySendError, unbounded};
use motion_core::pump::{
    AxisFrame, AxisKey, DripArm, EnqueueMsg, HeartbeatMsg, PumpCallbacks, PumpMsg, RetiredBy,
    SendError, SpanSink, run_pump,
};
use trajectory::{ClockedMotorSpan, ContinuousAxis, MotorGroup, MotorSpan, MotorTerm};

const FREQ: f64 = 1e6;
const SPAN_SECS: f64 = 0.001;
const SPAN_TICKS: u64 = 1000;
const SOURCE_LINE: u32 = u32::MAX;

struct RecordingSink(Arc<Mutex<Vec<(AxisKey, usize)>>>);
impl SpanSink for RecordingSink {
    fn send_frame(
        &self,
        key: AxisKey,
        spans: &[ClockedMotorSpan],
        _new_head: u32,
        _room: u32,
    ) -> Result<i32, SendError> {
        self.0.lock().unwrap().push((key, spans.len()));
        Ok(0)
    }
}

/// Records each bundled MCU transaction as `(mcu_id, axes-in-the-bundle)` so a
/// test can assert that same-MCU axes go out together rather than one
/// round-trip per axis.
struct BundleSink(Arc<Mutex<Vec<(u32, Vec<u8>)>>>);
impl SpanSink for BundleSink {
    fn send_frame(
        &self,
        _key: AxisKey,
        _spans: &[ClockedMotorSpan],
        _new_head: u32,
        _room: u32,
    ) -> Result<i32, SendError> {
        unreachable!("BundleSink delivers via send_mcu_frames, not per-axis send_frame")
    }

    fn send_mcu_frames(&self, mcu_id: u32, frames: &[AxisFrame]) -> Result<(), SendError> {
        let axes = frames.iter().map(|f| f.axis).collect();
        self.0.lock().unwrap().push((mcu_id, axes));
        Ok(())
    }
}

#[allow(clippy::cast_precision_loss)]
fn span_at(start_clock: u64, start_host: f64, from_mm: f64, to_mm: f64) -> ClockedMotorSpan {
    let t_start = start_clock as f64 / FREQ;
    let t_end = t_start + SPAN_SECS;
    let curve = nurbs::ScalarNurbs::try_new(
        1,
        vec![t_start, t_start, t_end, t_end],
        vec![from_mm, to_mm],
    )
    .expect("a linear lane curve is valid");
    let groups: Arc<[MotorGroup]> = Arc::from(vec![MotorGroup::Independent(MotorTerm {
        source_axis: 0,
        axis: ContinuousAxis::Spline(Arc::new(curve)),
        scale: 1.0,
    })]);
    let signal = MotorSpan::try_new(groups, t_start, t_end, 0, SOURCE_LINE, false)
        .expect("a spline motor span is dispatchable");
    ClockedMotorSpan::try_new(
        Arc::new(signal),
        t_start,
        t_end,
        start_host,
        start_host + SPAN_SECS,
        start_clock as f64,
        FREQ,
    )
    .expect("the projected view spans at least one clock")
}

#[allow(clippy::cast_precision_loss)]
fn span(start_clock: u64) -> ClockedMotorSpan {
    span_at(start_clock, start_clock as f64 / FREQ, 0.0, 0.0)
}

#[test]
fn pump_stalls_on_ring_full_resumes_on_heartbeat() {
    let rec = Arc::new(Mutex::new(Vec::new()));
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();
    let sink = RecordingSink(rec.clone());
    let handle = std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            sink,
            PumpCallbacks::noop(2),
            None,
            std::sync::Arc::new(motion_core::drain::DrainLedger::new()),
            Arc::new(AtomicU64::new(0)),
        )
    });

    let key = AxisKey { mcu_id: 1, axis: 0 };
    data.send(EnqueueMsg {
        epoch_freq: None,
        key,
        spans: vec![span(0), span(SPAN_TICKS)],
        epoch: motion_core::anchor::StreamEpoch::Reposition,
        lead_secs: motion_core::pump::MAX_LEAD_SECS,
        source_line: SOURCE_LINE,
        batch_end: true,
    })
    .unwrap();
    data.send(EnqueueMsg {
        epoch_freq: None,
        key,
        spans: vec![span(2 * SPAN_TICKS)],
        epoch: motion_core::anchor::StreamEpoch::Continuation,
        lead_secs: motion_core::pump::MAX_LEAD_SECS,
        source_line: SOURCE_LINE,
        batch_end: true,
    })
    .unwrap();
    std::thread::sleep(std::time::Duration::from_millis(50));
    assert_eq!(
        rec.lock().unwrap().as_slice(),
        [(key, 2)],
        "both depth-2 ring slots fill in one batch, and the third view stalls"
    );

    ctl.send(PumpMsg::Heartbeat(HeartbeatMsg {
        mcu_id: 1,
        axes: vec![0],
        consumed_counts: None,
        retired_counts: vec![2],
        retired_by: RetiredBy::Pulse,
    }))
    .unwrap();
    std::thread::sleep(std::time::Duration::from_millis(50));
    assert_eq!(
        rec.lock().unwrap().as_slice(),
        [(key, 2), (key, 1)],
        "retirement frees the ring and the stalled view ships"
    );

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}

fn run_pump_with_clock(
    control_rx: Receiver<PumpMsg>,
    data_rx: Receiver<EnqueueMsg>,
    rec: Arc<Mutex<Vec<(AxisKey, usize)>>>,
) -> std::thread::JoinHandle<()> {
    std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            RecordingSink(rec),
            PumpCallbacks {
                mcu_clock_of: Box::new(|_mcu| Some((0u64, FREQ))),
                ..PumpCallbacks::noop(64)
            },
            None,
            std::sync::Arc::new(motion_core::drain::DrainLedger::new()),
            Arc::new(AtomicU64::new(0)),
        )
    })
}

fn sent_spans(rec: &Arc<Mutex<Vec<(AxisKey, usize)>>>) -> usize {
    rec.lock().unwrap().iter().map(|(_, n)| n).sum()
}

#[test]
fn continuous_junction_position_passes() {
    let rec = Arc::new(Mutex::new(Vec::new()));
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();
    let handle = run_pump_with_clock(control_rx, data_rx, rec.clone());

    let key = AxisKey { mcu_id: 1, axis: 0 };
    data.send(EnqueueMsg {
        epoch_freq: None,
        key,
        spans: vec![span_at(0, 0.0, 10.0, 12.5)],
        epoch: motion_core::anchor::StreamEpoch::Reposition,
        lead_secs: motion_core::pump::MAX_LEAD_SECS,
        source_line: SOURCE_LINE,
        batch_end: true,
    })
    .unwrap();
    data.send(EnqueueMsg {
        epoch_freq: None,
        key,
        spans: vec![span_at(SPAN_TICKS, SPAN_SECS, 12.5, 15.0)],
        epoch: motion_core::anchor::StreamEpoch::Continuation,
        lead_secs: motion_core::pump::MAX_LEAD_SECS,
        source_line: SOURCE_LINE,
        batch_end: true,
    })
    .unwrap();
    std::thread::sleep(std::time::Duration::from_millis(50));
    assert_eq!(sent_spans(&rec), 2);

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}

#[test]
fn junction_position_discontinuity_is_fatal() {
    let rec = Arc::new(Mutex::new(Vec::new()));
    let (_ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();
    let handle = run_pump_with_clock(control_rx, data_rx, rec.clone());

    let key = AxisKey { mcu_id: 1, axis: 0 };
    data.send(EnqueueMsg {
        epoch_freq: None,
        key,
        spans: vec![span_at(0, 0.0, 10.0, 12.5)],
        epoch: motion_core::anchor::StreamEpoch::Reposition,
        lead_secs: motion_core::pump::MAX_LEAD_SECS,
        source_line: SOURCE_LINE,
        batch_end: true,
    })
    .unwrap();
    data.send(EnqueueMsg {
        epoch_freq: None,
        key,
        spans: vec![span_at(SPAN_TICKS, SPAN_SECS, 12.8, 15.0)],
        epoch: motion_core::anchor::StreamEpoch::Continuation,
        lead_secs: motion_core::pump::MAX_LEAD_SECS,
        source_line: SOURCE_LINE,
        batch_end: true,
    })
    .unwrap();

    assert!(
        handle.join().is_err(),
        "0.3mm junction position jump must panic the pump"
    );
}

#[test]
fn underrun_reanchor_keeps_junction_continuity_guard_armed() {
    let rec = Arc::new(Mutex::new(Vec::new()));
    let (_ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();
    let handle = run_pump_with_clock(control_rx, data_rx, rec.clone());

    let key = AxisKey { mcu_id: 1, axis: 0 };
    data.send(EnqueueMsg {
        epoch_freq: None,
        key,
        spans: vec![span_at(0, 0.0, 10.0, 12.5)],
        epoch: motion_core::anchor::StreamEpoch::Reposition,
        lead_secs: motion_core::pump::MAX_LEAD_SECS,
        source_line: SOURCE_LINE,
        batch_end: true,
    })
    .unwrap();
    data.send(EnqueueMsg {
        epoch_freq: None,
        key,
        spans: vec![span_at(SPAN_TICKS, SPAN_SECS, 12.8, 15.0)],
        epoch: motion_core::anchor::StreamEpoch::Reanchor,
        lead_secs: motion_core::pump::MAX_LEAD_SECS,
        source_line: SOURCE_LINE,
        batch_end: true,
    })
    .unwrap();

    assert!(
        handle.join().is_err(),
        "an underrun re-anchor replays the same continuous track, so a 0.3mm \
         position jump across it must panic the pump instead of reaching the \
         MCU as a -310 step burst"
    );
}

#[test]
fn underrun_reanchor_with_continuous_position_passes() {
    let rec = Arc::new(Mutex::new(Vec::new()));
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();
    let handle = run_pump_with_clock(control_rx, data_rx, rec.clone());

    let key = AxisKey { mcu_id: 1, axis: 0 };
    data.send(EnqueueMsg {
        epoch_freq: None,
        key,
        spans: vec![span_at(0, 0.0, 10.0, 12.5)],
        epoch: motion_core::anchor::StreamEpoch::Reposition,
        lead_secs: motion_core::pump::MAX_LEAD_SECS,
        source_line: SOURCE_LINE,
        batch_end: true,
    })
    .unwrap();
    data.send(EnqueueMsg {
        epoch_freq: None,
        key,
        spans: vec![span_at(SPAN_TICKS, SPAN_SECS, 12.5, 15.0)],
        epoch: motion_core::anchor::StreamEpoch::Reanchor,
        lead_secs: motion_core::pump::MAX_LEAD_SECS,
        source_line: SOURCE_LINE,
        batch_end: true,
    })
    .unwrap();
    std::thread::sleep(std::time::Duration::from_millis(50));
    assert_eq!(sent_spans(&rec), 2);

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}

#[test]
fn fresh_stream_resets_junction_position_baseline() {
    let rec = Arc::new(Mutex::new(Vec::new()));
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();
    let handle = run_pump_with_clock(control_rx, data_rx, rec.clone());

    let key = AxisKey { mcu_id: 1, axis: 0 };
    data.send(EnqueueMsg {
        epoch_freq: None,
        key,
        spans: vec![span_at(0, 0.0, 10.0, 12.5)],
        epoch: motion_core::anchor::StreamEpoch::Reposition,
        lead_secs: motion_core::pump::MAX_LEAD_SECS,
        source_line: SOURCE_LINE,
        batch_end: true,
    })
    .unwrap();
    data.send(EnqueueMsg {
        epoch_freq: None,
        key,
        spans: vec![span_at(SPAN_TICKS, SPAN_SECS, 50.0, 55.0)],
        epoch: motion_core::anchor::StreamEpoch::Reposition,
        lead_secs: motion_core::pump::MAX_LEAD_SECS,
        source_line: SOURCE_LINE,
        batch_end: true,
    })
    .unwrap();
    std::thread::sleep(std::time::Duration::from_millis(50));
    assert_eq!(sent_spans(&rec), 2);

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}

/// A stationary ethercat lane streams no views, so a position redefinition
/// (post-homing set_position adopting the measured servo position) reaches it
/// only as a span-free Reposition carrier — which must still forget the
/// junction baseline, or the first real move after it panics on the stale
/// pre-redefinition end position.
#[test]
fn empty_reposition_carrier_resets_junction_position_baseline() {
    let rec = Arc::new(Mutex::new(Vec::new()));
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();
    let handle = run_pump_with_clock(control_rx, data_rx, rec.clone());

    let key = AxisKey { mcu_id: 1, axis: 0 };
    data.send(EnqueueMsg {
        epoch_freq: None,
        key,
        spans: vec![span_at(0, 0.0, 10.0, 12.5)],
        epoch: motion_core::anchor::StreamEpoch::Reposition,
        lead_secs: motion_core::pump::MAX_LEAD_SECS,
        source_line: SOURCE_LINE,
        batch_end: true,
    })
    .unwrap();
    data.send(EnqueueMsg {
        epoch_freq: None,
        key,
        spans: Vec::new(),
        epoch: motion_core::anchor::StreamEpoch::Reposition,
        lead_secs: motion_core::pump::MAX_LEAD_SECS,
        source_line: SOURCE_LINE,
        batch_end: true,
    })
    .unwrap();
    data.send(EnqueueMsg {
        epoch_freq: None,
        key,
        spans: vec![span_at(SPAN_TICKS, SPAN_SECS, 50.0, 55.0)],
        epoch: motion_core::anchor::StreamEpoch::Continuation,
        lead_secs: motion_core::pump::MAX_LEAD_SECS,
        source_line: SOURCE_LINE,
        batch_end: true,
    })
    .unwrap();
    std::thread::sleep(std::time::Duration::from_millis(50));
    assert_eq!(sent_spans(&rec), 2);

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}

#[test]
fn bundles_same_mcu_axes_into_one_transaction() {
    let rec = Arc::new(Mutex::new(Vec::new()));
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();
    let sink = BundleSink(rec.clone());
    let handle = std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            sink,
            PumpCallbacks::noop(8),
            None,
            std::sync::Arc::new(motion_core::drain::DrainLedger::new()),
            Arc::new(AtomicU64::new(0)),
        )
    });

    for axis in 0..3u8 {
        data.send(EnqueueMsg {
            epoch_freq: None,
            key: AxisKey { mcu_id: 1, axis },
            spans: vec![span(0)],
            epoch: if axis == 0 {
                motion_core::anchor::StreamEpoch::Reposition
            } else {
                motion_core::anchor::StreamEpoch::Continuation
            },
            lead_secs: motion_core::pump::MAX_LEAD_SECS,
            source_line: SOURCE_LINE,
            batch_end: true,
        })
        .unwrap();
    }
    std::thread::sleep(std::time::Duration::from_millis(50));

    let calls = rec.lock().unwrap().clone();
    assert_eq!(
        calls.len(),
        1,
        "all three same-MCU axes must ship in one bundled transaction, not one per axis; got {calls:?}"
    );
    let (mcu, mut axes) = calls.into_iter().next().unwrap();
    axes.sort_unstable();
    assert_eq!(mcu, 1);
    assert_eq!(
        axes,
        vec![0, 1, 2],
        "the bundle must carry every axis of the MCU"
    );

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}

#[test]
fn intake_backpressures_at_backlog_cap_and_resumes_on_retirement() {
    // With the ring full and no retirement, the pump stops pulling once its
    // total host backlog reaches the cap, so a bounded data channel fills and
    // the producer's send is refused (backpressure). Retirement lets it push and
    // pull again, releasing the channel. The flood far exceeds the cap so the
    // refusal is the cap, not a transient.
    let rec = Arc::new(Mutex::new(Vec::new()));
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = crossbeam_channel::bounded::<EnqueueMsg>(8);
    let sink = RecordingSink(rec.clone());
    let handle = std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            sink,
            PumpCallbacks::noop(4),
            None,
            std::sync::Arc::new(motion_core::drain::DrainLedger::new()),
            Arc::new(AtomicU64::new(0)),
        )
    });

    let key = AxisKey { mcu_id: 1, axis: 0 };
    let mut accepted = 0u32;
    let mut hit_full = false;
    let flood = 24000u64; // comfortably above PUMP_INTAKE_BACKLOG_CAP
    for i in 0..flood {
        match data.try_send(EnqueueMsg {
            epoch_freq: None,
            key,
            spans: vec![span(i * SPAN_TICKS)],
            epoch: if i == 0 {
                motion_core::anchor::StreamEpoch::Reposition
            } else {
                motion_core::anchor::StreamEpoch::Continuation
            },
            lead_secs: motion_core::pump::MAX_LEAD_SECS,
            source_line: SOURCE_LINE,
            batch_end: true,
        }) {
            Ok(()) => accepted += 1,
            Err(TrySendError::Full(_)) => {
                hit_full = true;
                break;
            }
            Err(TrySendError::Disconnected(_)) => break,
        }
        if i % 64 == 0 {
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
    }
    assert!(
        hit_full,
        "pump must stop pulling at the backlog cap so the data channel backpressures; accepted={accepted}"
    );
    assert!(
        (accepted as u64) < flood,
        "intake must be bounded, not drain everything; accepted={accepted}"
    );

    ctl.send(PumpMsg::Heartbeat(HeartbeatMsg {
        mcu_id: 1,
        axes: vec![0],
        consumed_counts: None,
        retired_counts: vec![4],
        retired_by: RetiredBy::Pulse,
    }))
    .unwrap();
    std::thread::sleep(std::time::Duration::from_millis(30));
    assert!(
        data.try_send(EnqueueMsg {
            epoch_freq: None,
            key,
            spans: vec![span(flood * SPAN_TICKS)],
            epoch: motion_core::anchor::StreamEpoch::Continuation,
            lead_secs: motion_core::pump::MAX_LEAD_SECS,
            source_line: SOURCE_LINE,
            batch_end: true,
        })
        .is_ok(),
        "after retirement the pump resumes pulling and the channel drains"
    );

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}

#[test]
fn intake_feeds_a_second_axis_even_when_the_first_axis_ring_is_full() {
    // Regression: a per-axis ring-room intake gate stalls behind a full axis and
    // starves axes whose views arrive after it on the shared channel — this hung
    // the homing drip cohort (idle axes got zero views, floor pinned at 0).
    // Intake is bounded by TOTAL backlog, so a full axis A must not stop the pump
    // from feeding axis B.
    let rec = Arc::new(Mutex::new(Vec::new()));
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();
    let key_a = AxisKey { mcu_id: 1, axis: 0 };
    let key_b = AxisKey { mcu_id: 1, axis: 1 };
    let sink = RecordingSink(rec.clone());
    let handle = std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            sink,
            PumpCallbacks {
                ring_depth_of: Box::new(move |k| if k == key_a { 2 } else { 64 }),
                ..PumpCallbacks::noop(0)
            },
            None,
            std::sync::Arc::new(motion_core::drain::DrainLedger::new()),
            Arc::new(AtomicU64::new(0)),
        )
    });

    // Axis A overruns its depth-2 ring with no retirement (stays full), then
    // axis B's views arrive behind A's on the same channel.
    for i in 0..8u64 {
        data.send(EnqueueMsg {
            epoch_freq: None,
            key: key_a,
            spans: vec![span(i * SPAN_TICKS)],
            epoch: if i == 0 {
                motion_core::anchor::StreamEpoch::Reposition
            } else {
                motion_core::anchor::StreamEpoch::Continuation
            },
            lead_secs: motion_core::pump::MAX_LEAD_SECS,
            source_line: SOURCE_LINE,
            batch_end: true,
        })
        .unwrap();
    }
    for i in 0..4u64 {
        data.send(EnqueueMsg {
            epoch_freq: None,
            key: key_b,
            spans: vec![span((100 + i) * SPAN_TICKS)],
            epoch: if i == 0 {
                motion_core::anchor::StreamEpoch::Reposition
            } else {
                motion_core::anchor::StreamEpoch::Continuation
            },
            lead_secs: motion_core::pump::MAX_LEAD_SECS,
            source_line: SOURCE_LINE,
            batch_end: true,
        })
        .unwrap();
    }
    std::thread::sleep(std::time::Duration::from_millis(50));

    let b_sent: usize = rec
        .lock()
        .unwrap()
        .iter()
        .filter(|(k, _)| *k == key_b)
        .map(|(_, n)| n)
        .sum();
    assert!(
        b_sent > 0,
        "axis B must be fed even though axis A's ring is full (no starvation behind a full axis)"
    );

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}

#[test]
fn drip_cohort_finishes_over_cap_projection_batch_before_backpressuring() {
    let rec = Arc::new(Mutex::new(Vec::new()));
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();
    let key_a = AxisKey { mcu_id: 1, axis: 0 };
    let key_b = AxisKey { mcu_id: 1, axis: 1 };
    let sink = RecordingSink(rec.clone());
    let handle = std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            sink,
            PumpCallbacks {
                ring_depth_of: Box::new(move |k| if k == key_a { 4 } else { 64 }),
                mcu_clock_of: Box::new(|_mcu| Some((0u64, FREQ))),
                ..PumpCallbacks::noop(0)
            },
            None,
            std::sync::Arc::new(motion_core::drain::DrainLedger::new()),
            Arc::new(AtomicU64::new(0)),
        )
    });

    ctl.send(PumpMsg::DripArm(DripArm {
        cohort: 1,
        participants: vec![key_a, key_b],
        timeout: std::time::Duration::from_secs(5),
    }))
    .unwrap();

    data.send(EnqueueMsg {
        epoch_freq: None,
        key: key_a,
        spans: (0..9000u64).map(span).collect(),
        epoch: motion_core::anchor::StreamEpoch::Reposition,
        lead_secs: motion_core::pump::DRIP_WINDOW_SECS,
        source_line: SOURCE_LINE,
        batch_end: false,
    })
    .unwrap();
    data.send(EnqueueMsg {
        epoch_freq: None,
        key: key_b,
        spans: (0..4u64).map(span).collect(),
        epoch: motion_core::anchor::StreamEpoch::Reposition,
        lead_secs: motion_core::pump::DRIP_WINDOW_SECS,
        source_line: SOURCE_LINE,
        batch_end: true,
    })
    .unwrap();

    std::thread::sleep(std::time::Duration::from_millis(80));

    let b_sent: usize = rec
        .lock()
        .unwrap()
        .iter()
        .filter(|(k, _)| *k == key_b)
        .map(|(_, n)| n)
        .sum();
    assert_eq!(
        b_sent, 4,
        "the pump must finish the current projection batch after crossing the hard cap"
    );

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}
