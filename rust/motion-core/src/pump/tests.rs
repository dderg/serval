use super::pump_loop::{
    PUMP_INTAKE_BACKLOG_HARD_CAP, PUMP_INTAKE_BACKLOG_SOFT_CAP, PUMP_INTAKE_MIN_RUNWAY_SECS, Pump,
    wants_spans,
};
use super::*;
use crate::lock_ext::LockExt;
use crossbeam_channel::unbounded;
use std::collections::BTreeMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use trajectory::{ClockedMotorSpan, ContinuousAxis, MotorGroup, MotorSpan, MotorTerm};

/// The default fixture clock: one tick per microsecond, so a span's start
/// clock is its start time in microseconds and [`SPAN_SECS`] is [`SPAN_TICKS`].
const FREQ: f64 = 1_000_000.0;
const SPAN_SECS: f64 = 0.001;
const SPAN_TICKS: u64 = 1_000;

fn hold_span(
    start_clock: u64,
    secs: f64,
    freq: f64,
    position: f64,
    motor_mask: u8,
) -> ClockedMotorSpan {
    let t_start = start_clock as f64 / freq;
    let t_end = t_start + secs;
    let groups: Arc<[MotorGroup]> = Arc::from(vec![MotorGroup::Independent(MotorTerm {
        source_axis: 0,
        axis: ContinuousAxis::Hold {
            position,
            t_start,
            t_end,
        },
        scale: 1.0,
    })]);
    let signal = MotorSpan::try_new(groups, t_start, t_end, motor_mask, u32::MAX, true)
        .expect("a hold motor span is dispatchable");
    ClockedMotorSpan::try_new(
        Arc::new(signal),
        t_start,
        t_end,
        t_start,
        t_end,
        start_clock as f64,
        freq,
    )
    .expect("the projected view spans at least one clock")
}

fn moving_span(
    start_clock: u64,
    secs: f64,
    freq: f64,
    from: f64,
    to: f64,
    motor_mask: u8,
) -> ClockedMotorSpan {
    let t_start = start_clock as f64 / freq;
    let t_end = t_start + secs;
    let curve =
        nurbs::ScalarNurbs::try_new(1, vec![t_start, t_start, t_end, t_end], vec![from, to])
            .expect("a linear lane curve is valid");
    let groups: Arc<[MotorGroup]> = Arc::from(vec![MotorGroup::Independent(MotorTerm {
        source_axis: 0,
        axis: ContinuousAxis::Spline(Arc::new(curve)),
        scale: 1.0,
    })]);
    let signal = MotorSpan::try_new(groups, t_start, t_end, motor_mask, u32::MAX, false)
        .expect("a spline motor span is dispatchable");
    ClockedMotorSpan::try_new(
        Arc::new(signal),
        t_start,
        t_end,
        t_start,
        t_end,
        start_clock as f64,
        freq,
    )
    .expect("the projected view spans at least one clock")
}

/// Distinct hold positions per index, so the queue's hold coalescing cannot
/// merge the views a count-based assertion tracks one by one.
fn make_span(index: u64) -> ClockedMotorSpan {
    hold_span(index * SPAN_TICKS, SPAN_SECS, FREQ, index as f64, 0)
}

fn wait_until(mut cond: impl FnMut() -> bool, what: &str) {
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
    while !cond() {
        assert!(
            std::time::Instant::now() < deadline,
            "timed out waiting for: {what}"
        );
        std::thread::yield_now();
    }
}

fn make_enqueue(
    key: AxisKey,
    spans: Vec<ClockedMotorSpan>,
    epoch: crate::anchor::StreamEpoch,
) -> EnqueueMsg {
    EnqueueMsg {
        epoch_freq: None,
        key,
        spans,
        epoch,
        lead_secs: MAX_LEAD_SECS,
        source_line: u32::MAX,
        batch_end: true,
    }
}

#[test]
fn room_full_then_drains() {
    let mut q = AxisQueue::new(4);
    assert_eq!(q.room(), 4);
    q.pushed = 4;
    assert_eq!(q.room(), 0);
    q.consumed = 1;
    assert_eq!(q.room(), 1);
}

#[test]
fn consumed_spans_reopen_capacity_before_execution_retires_them() {
    let mut q = AxisQueue::new(64);
    q.pushed = 64;
    q.consumed = 64;
    q.retired = 0;

    assert_eq!(q.room(), 64);
    assert_ne!(q.pushed, q.retired);
}

#[test]
fn room_correct_across_u32_wrap() {
    let mut q = AxisQueue::new(8);
    q.pushed = 2;
    q.consumed = u32::MAX;
    assert_eq!(
        q.room(),
        5,
        "legitimate counter rollover: consumed is numerically larger than pushed only because \
         the u32 odometer wrapped, so 3 spans are genuinely awaiting consumption. wrapping_sub \
         recovers 3; saturating subtraction would wrongly report a drained ring."
    );
}

#[test]
fn room_recovers_when_consumed_overtakes_pushed() {
    let mut q = AxisQueue::new(4);
    q.pushed = 100;
    q.consumed = 101;
    assert_eq!(
        q.room(),
        4,
        "consumed overtook pushed by 1 after a lost response; the inversion must reconcile to a \
         drained ring instead of wedging with zero room"
    );
}

#[test]
fn schedule_resends_orphan_when_consumed_overtook_pushed() {
    let key = AxisKey { mcu_id: 1, axis: 0 };
    let mut q = AxisQueue::new(8);
    q.pushed = 100;
    q.consumed = 101;
    q.spans.push_back(make_span(101));
    let mut queues: BTreeMap<AxisKey, AxisQueue> = BTreeMap::new();
    queues.insert(key, q);

    const MAX_PER_FRAME: usize = 32;
    match schedule(
        &queues,
        |_| crate::pump::BundleLimits {
            spans_per_axis: MAX_PER_FRAME,
        },
        |_, _| None,
        |_| usize::MAX,
    ) {
        Schedule::Send(frames) => {
            assert_eq!(frames.len(), 1, "exactly the inverted axis is scheduled");
            assert_eq!(frames[0].key, key);
        }
        other => {
            panic!("consumed>pushed inversion must schedule a re-send, not wedge; got {other:?}")
        }
    }
}

#[test]
fn run_pump_delivers_span_despite_retired_over_pushed_inversion() {
    const RING_DEPTH: u32 = 8;
    let key = AxisKey { mcu_id: 1, axis: 0 };

    let sink = RecordingSink::new();
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();
    let sink_clone = sink.clone();
    let handle = std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            sink_clone,
            PumpCallbacks::noop(RING_DEPTH),
            None,
            std::sync::Arc::new(crate::drain::DrainLedger::new()),
            Arc::new(AtomicU64::new(0)),
        );
    });

    data.send(make_enqueue(
        key,
        vec![make_span(0)],
        crate::anchor::StreamEpoch::Continuation,
    ))
    .unwrap();
    wait_until(
        || sink.recorded().len() == 1,
        "first span delivered, creating the axis queue with pushed=1 (a heartbeat \
         for an axis with no queue yet is dropped)",
    );

    ctl.send(PumpMsg::Heartbeat(HeartbeatMsg {
        mcu_id: 1,
        axes: vec![0],
        consumed_counts: None,
        retired_counts: vec![2],
        retired_by: RetiredBy::Pulse,
    }))
    .unwrap();
    let (ack_tx, ack_rx) = mpsc::sync_channel::<()>(1);
    ctl.send(PumpMsg::Barrier(ack_tx)).unwrap();
    ack_rx
        .recv()
        .expect("barrier acks only after the retired=2 heartbeat ahead of it applies");

    data.send(make_enqueue(
        key,
        vec![moving_span(SPAN_TICKS, SPAN_SECS, FREQ, 0.0, 1.0, 0)],
        crate::anchor::StreamEpoch::Continuation,
    ))
    .unwrap();
    wait_until(
        || sink.recorded().len() == 2,
        "second span delivered despite retired(2) > pushed(1): buggy room() \
         underflows to 0 and wedges here; the fix reopens room",
    );

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}

#[test]
fn history_records_spans_at_send_time_not_enqueue_time() {
    const RING_DEPTH: u32 = 8;
    let key = AxisKey { mcu_id: 1, axis: 0 };

    let store = Arc::new(Mutex::new(crate::motion_history::HistoryStore::default()));
    let history = HistoryRecorder {
        store: Arc::clone(&store),
    };

    let sink = RecordingSink::new();
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();
    let sink_clone = sink.clone();
    let handle = std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            sink_clone,
            PumpCallbacks::noop(RING_DEPTH),
            Some(history),
            std::sync::Arc::new(crate::drain::DrainLedger::new()),
            Arc::new(AtomicU64::new(0)),
        );
    });

    let host_t = 2.5_f64;
    let span = hold_span((host_t * FREQ) as u64, SPAN_SECS, FREQ, 0.0, 0);
    assert!((span.start_host - host_t).abs() < 1e-12);
    data.send(make_enqueue(
        key,
        vec![span],
        crate::anchor::StreamEpoch::Continuation,
    ))
    .unwrap();
    wait_until(|| sink.recorded().len() == 1, "span sent to the MCU");
    wait_until(
        || store.lock_ok().state_at_host(key, host_t, None).is_ok(),
        "sent span recorded into motion history with its host key",
    );

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}

#[derive(Clone)]
struct RecordingSink {
    calls: Arc<Mutex<Vec<u32>>>,
}

impl RecordingSink {
    fn new() -> Self {
        Self {
            calls: Arc::new(Mutex::new(Vec::new())),
        }
    }
    fn recorded(&self) -> Vec<u32> {
        self.calls.lock_ok().clone()
    }
}

impl SpanSink for RecordingSink {
    fn send_frame(
        &self,
        _key: AxisKey,
        _spans: &[ClockedMotorSpan],
        new_head: u32,
        _room: u32,
    ) -> Result<i32, SendError> {
        self.calls.lock_ok().push(new_head);
        Ok(mcu_protocol::result_codes::OK)
    }
}

fn staged_axis(span_count: u64, runway_secs: f64) -> AxisQueue {
    let secs = runway_secs / span_count as f64;
    let ticks = (secs * FREQ) as u64;
    assert!(ticks > 0, "a staged view must span at least one clock");
    let mut queue = AxisQueue::new(u32::MAX);
    queue
        .spans
        .extend((0..span_count).map(|index| hold_span(index * ticks, secs, FREQ, index as f64, 0)));
    queue
}

#[test]
fn pump_intake_uses_runway_only_below_the_hard_cap() {
    let key = |axis| AxisKey { mcu_id: 1, axis };
    let below_soft_cap = BTreeMap::from([(
        key(0),
        staged_axis(
            PUMP_INTAKE_BACKLOG_SOFT_CAP - 1,
            PUMP_INTAKE_MIN_RUNWAY_SECS * 2.0,
        ),
    )]);
    assert!(wants_spans(&below_soft_cap));

    let one_axis_shallow = BTreeMap::from([
        (
            key(0),
            staged_axis(
                PUMP_INTAKE_BACKLOG_SOFT_CAP / 2,
                PUMP_INTAKE_MIN_RUNWAY_SECS * 2.0,
            ),
        ),
        (
            key(1),
            staged_axis(
                PUMP_INTAKE_BACKLOG_SOFT_CAP / 2,
                PUMP_INTAKE_MIN_RUNWAY_SECS * 0.5,
            ),
        ),
    ]);
    assert!(wants_spans(&one_axis_shallow));

    let all_axes_ready = BTreeMap::from([
        (
            key(0),
            staged_axis(
                PUMP_INTAKE_BACKLOG_SOFT_CAP / 2,
                PUMP_INTAKE_MIN_RUNWAY_SECS * 2.0,
            ),
        ),
        (
            key(1),
            staged_axis(
                PUMP_INTAKE_BACKLOG_SOFT_CAP / 2,
                PUMP_INTAKE_MIN_RUNWAY_SECS * 2.0,
            ),
        ),
    ]);
    assert!(!wants_spans(&all_axes_ready));

    let shallow_at_hard_cap = BTreeMap::from([(
        key(0),
        staged_axis(
            PUMP_INTAKE_BACKLOG_HARD_CAP,
            PUMP_INTAKE_MIN_RUNWAY_SECS * 0.5,
        ),
    )]);
    assert!(!wants_spans(&shallow_at_hard_cap));
}

#[test]
fn overlay_span_after_move_is_exempt_from_junction_continuity() {
    const JUNCTION_FREQ: f64 = 180_000_000.0;
    let sink = RecordingSink::new();
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();
    let sink_clone = sink.clone();
    let handle = std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            sink_clone,
            PumpCallbacks {
                mcu_clock_of: Box::new(|_mcu| Some((0u64, JUNCTION_FREQ))),
                ..PumpCallbacks::noop(8)
            },
            None,
            std::sync::Arc::new(crate::drain::DrainLedger::new()),
            Arc::new(AtomicU64::new(0)),
        );
    });
    let key = AxisKey { mcu_id: 1, axis: 2 };
    let move_ticks = (SPAN_SECS * JUNCTION_FREQ) as u64;

    data.send(make_enqueue(
        key,
        vec![moving_span(0, SPAN_SECS, JUNCTION_FREQ, 0.0, 11.0, 0)],
        crate::anchor::StreamEpoch::Continuation,
    ))
    .unwrap();
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
    while sink.recorded().is_empty() {
        assert!(
            std::time::Instant::now() < deadline,
            "first span never sent"
        );
        std::thread::yield_now();
    }

    // The overlay restarts from 0.0 while the move ended at 11.0: an 11 mm
    // position jump that would be fatal on a bare (mask 0) seam.
    data.send(make_enqueue(
        key,
        vec![moving_span(
            move_ticks,
            SPAN_SECS,
            JUNCTION_FREQ,
            0.0,
            0.5,
            0b10,
        )],
        crate::anchor::StreamEpoch::Continuation,
    ))
    .unwrap();
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
    while sink.recorded().len() < 2 {
        assert!(
            std::time::Instant::now() < deadline,
            "overlay span never sent — pump likely panicked on the junction check"
        );
        std::thread::yield_now();
    }

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
    assert_eq!(sink.recorded().len(), 2, "both spans dispatched, no panic");
}

#[test]
fn junction_jumps_math() {
    let (tick_us, host_us) = junction_jumps(2000, 2.0e-3, 1000, 1.0e-3, 1_000_000.0);
    assert!((tick_us - 1000.0).abs() < 1e-6, "tick_jump_us={tick_us}");
    assert!((host_us - 1000.0).abs() < 1e-6, "host_jump_us={host_us}");

    let (tick_us2, host_us2) = junction_jumps(900, 0.9e-3, 1000, 1.0e-3, 1_000_000.0);
    assert!(tick_us2 < 0.0, "overlap should be negative tick jump");
    assert!(host_us2 < 0.0, "overlap should be negative host jump");

    let freq = 520_000_000.0_f64;
    let prev_end_ticks: u64 = 10_000;
    let (tick_us3, host_us3) = junction_jumps(prev_end_ticks, 5.0e-4, prev_end_ticks, 0.0, freq);
    assert!(
        (tick_us3).abs() < 1e-6,
        "tick gap should be zero, got {tick_us3}"
    );
    assert!((host_us3 - 500.0).abs() < 1e-3, "host_jump_us={host_us3}");
}

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

struct HaltedSink;

impl SpanSink for HaltedSink {
    fn send_frame(
        &self,
        _key: AxisKey,
        _spans: &[ClockedMotorSpan],
        _new_head: u32,
        _room: u32,
    ) -> Result<i32, SendError> {
        Err(SendError::Halted("endpoint stream halted".into()))
    }
}

/// The four gated views a Flush must drop, on a 1 kHz fixture clock where one
/// view is exactly one tick. Distinct hold positions keep the queue's hold
/// coalescing from collapsing them into one.
fn gated_spans(gated_tick: u64, freq: f64) -> Vec<ClockedMotorSpan> {
    (0u64..4)
        .map(|i| hold_span(gated_tick + i, 1.0 / freq, freq, 1.0 + i as f64, 0))
        .collect()
}

#[test]
fn flush_clears_queued_spans_and_junctions() {
    let key = AxisKey { mcu_id: 1, axis: 0 };
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();

    let freq: f64 = 1_000.0;
    let lead_secs: f64 = 0.001;
    let gated_tick: u64 = 1_000;

    let clock: Arc<Mutex<Option<(u64, f64)>>> = Arc::new(Mutex::new(Some((0, freq))));
    let clock_pump = Arc::clone(&clock);
    let sink = RecordingSink::new();
    let sink_pump = sink.clone();

    let handle = std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            sink_pump,
            PumpCallbacks {
                mcu_clock_of: Box::new(move |_mcu| *clock_pump.lock_ok()),
                ..PumpCallbacks::noop(64)
            },
            None,
            std::sync::Arc::new(crate::drain::DrainLedger::new()),
            Arc::new(AtomicU64::new(0)),
        );
    });

    data.send(EnqueueMsg {
        epoch_freq: None,
        key,
        spans: gated_spans(gated_tick, freq),
        epoch: crate::anchor::StreamEpoch::Reposition,
        lead_secs,
        source_line: u32::MAX,
        batch_end: true,
    })
    .unwrap();

    std::thread::sleep(std::time::Duration::from_millis(30));

    ctl.send(PumpMsg::Flush(vec![key])).unwrap();
    std::thread::sleep(std::time::Duration::from_millis(20));

    *clock.lock_ok() = Some((gated_tick + 1_000, freq));

    data.send(EnqueueMsg {
        epoch_freq: None,
        key,
        // A deliverable "now" probe (== the advanced clock), not a stale past
        // view — the pump's in-past guard aborts on past start clocks.
        spans: vec![hold_span(gated_tick + 1_000, 1.0 / freq, freq, 0.0, 0)],
        epoch: crate::anchor::StreamEpoch::Continuation,
        lead_secs,
        source_line: u32::MAX,
        batch_end: true,
    })
    .unwrap();
    {
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
        while sink.recorded().is_empty() {
            assert!(
                std::time::Instant::now() < deadline,
                "pump never sent the post-flush probe span — deadlocked"
            );
            std::thread::yield_now();
        }
    }

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();

    let recorded = sink.recorded();
    assert_eq!(
        recorded.len(),
        1,
        "sink must see only the post-flush probe span; \
         {} sends means the {} gated spans survived Flush",
        recorded.len(),
        4
    );
}

#[test]
fn on_abandon_reports_flushed_not_pushed_spans() {
    let key = AxisKey { mcu_id: 1, axis: 0 };
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();

    let freq: f64 = 1_000.0;
    let lead_secs: f64 = 0.001;
    let gated_tick: u64 = 1_000;

    let clock: Arc<Mutex<Option<(u64, f64)>>> = Arc::new(Mutex::new(Some((0, freq))));
    let clock_pump = Arc::clone(&clock);
    let sink = RecordingSink::new();
    let sink_pump = sink.clone();
    let abandoned_total = Arc::new(Mutex::new(0u32));
    let abandoned_pump = Arc::clone(&abandoned_total);

    let handle = std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            sink_pump,
            PumpCallbacks {
                mcu_clock_of: Box::new(move |_mcu| *clock_pump.lock_ok()),
                on_abandon: Box::new(move |_k: AxisKey, n: u32| {
                    *abandoned_pump.lock_ok() += n;
                }),
                ..PumpCallbacks::noop(64)
            },
            None,
            std::sync::Arc::new(crate::drain::DrainLedger::new()),
            Arc::new(AtomicU64::new(0)),
        );
    });

    data.send(EnqueueMsg {
        epoch_freq: None,
        key,
        spans: gated_spans(gated_tick, freq),
        epoch: crate::anchor::StreamEpoch::Reposition,
        lead_secs,
        source_line: u32::MAX,
        batch_end: true,
    })
    .unwrap();

    std::thread::sleep(std::time::Duration::from_millis(30));
    ctl.send(PumpMsg::Flush(vec![key])).unwrap();
    std::thread::sleep(std::time::Duration::from_millis(20));
    *clock.lock_ok() = Some((gated_tick + 1_000, freq));

    data.send(EnqueueMsg {
        epoch_freq: None,
        key,
        spans: vec![hold_span(gated_tick + 1_000, 1.0 / freq, freq, 0.0, 0)],
        epoch: crate::anchor::StreamEpoch::Continuation,
        lead_secs,
        source_line: u32::MAX,
        batch_end: true,
    })
    .unwrap();
    {
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
        while sink.recorded().is_empty() {
            assert!(
                std::time::Instant::now() < deadline,
                "pump never sent the post-flush probe span — deadlocked"
            );
            std::thread::yield_now();
        }
    }

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();

    assert_eq!(
        *abandoned_total.lock_ok(),
        4,
        "on_abandon must report the 4 Flush-dropped spans and not the pushed probe"
    );
}

#[test]
fn flush_unknown_key_is_noop() {
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (_data, data_rx) = unbounded::<EnqueueMsg>();
    let handle = std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            NullSink,
            PumpCallbacks::noop(64),
            None,
            std::sync::Arc::new(crate::drain::DrainLedger::new()),
            Arc::new(AtomicU64::new(0)),
        );
    });

    let never_enqueued = AxisKey {
        mcu_id: 99,
        axis: 7,
    };
    ctl.send(PumpMsg::Flush(vec![never_enqueued])).unwrap();
    std::thread::sleep(std::time::Duration::from_millis(20));
    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}

#[test]
fn barrier_ack_means_flushed_axes_emit_nothing() {
    let key = AxisKey { mcu_id: 1, axis: 0 };
    let sink = RecordingSink::new();
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();
    let backlog = Arc::new(AtomicU64::new(0));
    let backlog_pump = Arc::clone(&backlog);

    let sink_clone = sink.clone();
    let handle = std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            sink_clone,
            PumpCallbacks::noop(0),
            None,
            std::sync::Arc::new(crate::drain::DrainLedger::new()),
            backlog_pump,
        );
    });

    data.send(make_enqueue(
        key,
        (0..3).map(make_span).collect(),
        crate::anchor::StreamEpoch::Continuation,
    ))
    .unwrap();
    poll_until(
        || backlog.load(Ordering::Acquire) == 3,
        "ring-full pump never staged the 3 un-pushed spans",
    );

    ctl.send(PumpMsg::Flush(vec![key])).unwrap();
    let (ack_tx, ack_rx) = mpsc::sync_channel(1);
    ctl.send(PumpMsg::Barrier(ack_tx)).unwrap();

    ack_rx
        .recv_timeout(std::time::Duration::from_secs(2))
        .expect("barrier must be acknowledged");

    poll_until(
        || backlog.load(Ordering::Acquire) == 0,
        "Flush must clear the staged backlog",
    );

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();

    assert!(
        sink.recorded().is_empty(),
        "spans flushed before the barrier must never reach the sink; got {:?}",
        sink.recorded()
    );
}

#[test]
fn barrier_acks_on_idle_pump() {
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (_data, data_rx) = unbounded::<EnqueueMsg>();
    let handle = std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            RecordingSink::new(),
            PumpCallbacks::noop(8),
            None,
            std::sync::Arc::new(crate::drain::DrainLedger::new()),
            Arc::new(AtomicU64::new(0)),
        );
    });
    let (ack_tx, ack_rx) = mpsc::sync_channel(1);
    ctl.send(PumpMsg::Barrier(ack_tx)).unwrap();
    ack_rx
        .recv_timeout(std::time::Duration::from_secs(2))
        .expect("barrier on an idle pump must ack promptly");
    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}

fn poll_until<F: Fn() -> bool>(pred: F, what: &str) {
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
    while !pred() {
        assert!(std::time::Instant::now() < deadline, "{what}");
        std::thread::yield_now();
    }
}

#[test]
fn pump_backlog_reflects_unpushed_spans() {
    let backlog = Arc::new(AtomicU64::new(0));
    let backlog_thread = Arc::clone(&backlog);
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();
    let handle = std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            RecordingSink::new(),
            PumpCallbacks::noop(0),
            None,
            std::sync::Arc::new(crate::drain::DrainLedger::new()),
            backlog_thread,
        );
    });

    data.send(make_enqueue(
        AxisKey { mcu_id: 1, axis: 0 },
        (0..3).map(make_span).collect(),
        crate::anchor::StreamEpoch::Continuation,
    ))
    .unwrap();

    poll_until(
        || backlog.load(Ordering::Acquire) == 3,
        "ring-full pump never reported the 3 unpushed spans",
    );

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}

#[test]
fn pump_backlog_drains_to_zero_when_pushed() {
    let backlog = Arc::new(AtomicU64::new(0));
    let backlog_thread = Arc::clone(&backlog);
    let sink = RecordingSink::new();
    let sink_clone = sink.clone();
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();
    let handle = std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            sink_clone,
            PumpCallbacks::noop(8),
            None,
            std::sync::Arc::new(crate::drain::DrainLedger::new()),
            backlog_thread,
        );
    });

    data.send(make_enqueue(
        AxisKey { mcu_id: 1, axis: 0 },
        (0..3).map(make_span).collect(),
        crate::anchor::StreamEpoch::Continuation,
    ))
    .unwrap();

    poll_until(|| !sink.recorded().is_empty(), "pump never pushed spans");
    poll_until(
        || backlog.load(Ordering::Acquire) == 0,
        "backlog never returned to zero after the ring consumed the spans",
    );

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}

fn queue_pump<S: SpanSink>(
    key: AxisKey,
    consumption_stall_fatal: Duration,
    on_drip_stall: impl Fn(String) + Send + 'static,
    sink: S,
) -> Pump<S> {
    let mut queues = BTreeMap::new();
    let mut q = AxisQueue::new(1);
    q.pushed = 1;
    q.retired = 0;
    q.spans.push_back(make_span(0));
    queues.insert(key, q);
    Pump {
        queues,
        junctions: JunctionTracker::default(),
        cohort: None,
        halted: BTreeMap::new(),
        sink,
        callbacks: PumpCallbacks {
            on_drip_stall: Box::new(on_drip_stall),
            ..PumpCallbacks::noop(1)
        },
        history: None,
        ledger: Arc::new(crate::drain::DrainLedger::new()),
        pending_barrier_acks: Vec::new(),
        backlog: Arc::new(AtomicU64::new(0)),
        holding_ahead: false,
        data_open: true,
        intake_batch_open: false,
        consumption_stall: super::stall::ConsumptionStallWatch::new(consumption_stall_fatal),
        mem_probe: super::memstat::MemPressureProbe::new(),
    }
}

fn stalled_queue_pump(
    key: AxisKey,
    consumption_stall_fatal: Duration,
    on_drip_stall: impl Fn(String) + Send + 'static,
) -> Pump<NullSink> {
    queue_pump(key, consumption_stall_fatal, on_drip_stall, NullSink)
}

#[test]
fn send_pass_deadline_yields_with_work_pending() {
    let key = AxisKey { mcu_id: 1, axis: 0 };
    let sink = RecordingSink::new();
    let mut pump = queue_pump(key, Duration::from_secs(1), |_| {}, sink.clone());
    let q = pump.queues.get_mut(&key).unwrap();
    q.pushed = 0;
    q.spans.clear();
    q.ring_depth = 4_000;
    let queued: u64 = 3_000;
    for i in 0..queued {
        q.spans.push_back(make_span(i));
    }

    assert_eq!(pump.send_ready_until(std::time::Instant::now()), Ok(true));

    assert_eq!(
        sink.recorded().len(),
        1,
        "an expired pass deadline still sends exactly one bundle"
    );
    let remaining = pump.queues[&key].spans.len() as u64;
    assert!(
        remaining > 0 && remaining < queued,
        "one bundle went out, the rest waits so intake can interleave (remaining={remaining})"
    );

    assert_eq!(
        pump.send_ready_until(std::time::Instant::now() + Duration::from_secs(60)),
        Ok(true)
    );
    assert!(
        pump.queues[&key].spans.is_empty(),
        "a roomy deadline drains the queue"
    );
}

#[test]
fn halt_drops_queued_and_new_spans_until_resume() {
    let key = AxisKey { mcu_id: 1, axis: 0 };
    let (escalated_tx, escalated_rx) = mpsc::channel();
    let mut pump = stalled_queue_pump(key, Duration::from_secs(1), move |message| {
        escalated_tx.send(message).unwrap()
    });
    let (ack_tx, _ack_rx) = mpsc::sync_channel(1);

    pump.handle_control_msg(PumpMsg::Halt {
        keys: vec![key],
        ack: ack_tx,
    });
    assert!(pump.halted.contains_key(&key));
    assert!(pump.queues[&key].spans.is_empty());
    pump.enqueue(make_enqueue(
        key,
        vec![make_span(10)],
        crate::anchor::StreamEpoch::Continuation,
    ));
    assert!(pump.queues[&key].spans.is_empty());
    assert!(matches!(
        escalated_rx.try_recv(),
        Err(mpsc::TryRecvError::Empty)
    ));

    pump.handle_control_msg(PumpMsg::Resume(vec![key]));
    pump.enqueue(make_enqueue(
        key,
        vec![make_span(20)],
        crate::anchor::StreamEpoch::Continuation,
    ));
    assert_eq!(pump.queues[&key].spans.len(), 1);
}

/// A halted axis' motion is discarded on the endpoint, so a transport that
/// keeps a host-side stage (the setpoint ring) must be told to drop it in the
/// same breath. The pump owns that hand-off, so it is asserted here rather
/// than at the sink.
#[derive(Clone)]
struct CutRecordingSink {
    cut: Arc<Mutex<Vec<AxisKey>>>,
}

impl SpanSink for CutRecordingSink {
    fn send_frame(
        &self,
        _key: AxisKey,
        _spans: &[ClockedMotorSpan],
        _new_head: u32,
        _room: u32,
    ) -> Result<i32, SendError> {
        Ok(mcu_protocol::result_codes::OK)
    }

    fn cut_staged(&self, keys: &[AxisKey]) -> Result<(), SendError> {
        self.cut.lock_ok().extend_from_slice(keys);
        Ok(())
    }
}

#[test]
fn halting_an_axis_cuts_the_transport_s_staged_motion() {
    let key = AxisKey { mcu_id: 1, axis: 0 };
    let cut = Arc::new(Mutex::new(Vec::new()));
    let sink = CutRecordingSink {
        cut: Arc::clone(&cut),
    };
    let mut pump = queue_pump(key, Duration::from_secs(1), |_| {}, sink);
    let (ack_tx, _ack_rx) = mpsc::sync_channel(1);

    pump.handle_control_msg(PumpMsg::Halt {
        keys: vec![key],
        ack: ack_tx,
    });

    assert_eq!(
        *cut.lock_ok(),
        vec![key],
        "the halted key must reach the sink's stage-cut hook"
    );
}

#[test]
fn halted_stream_rejection_is_not_retryable() {
    assert!(matches!(
        SendError::mcu_reject(2, mcu_protocol::result_codes::STREAM_HALTED),
        SendError::Halted(_)
    ));
    assert!(matches!(
        SendError::mcu_reject(2, mcu_protocol::result_codes::RING_FULL),
        SendError::Transient(_)
    ));
}

#[test]
fn send_rejected_while_halted_discards_bundle_and_infers_halt() {
    let key = AxisKey { mcu_id: 2, axis: 1 };
    let mut pump = queue_pump(key, Duration::from_secs(1), |_| {}, HaltedSink);
    let queue = pump.queues.get_mut(&key).unwrap();
    queue.ring_depth = 4;
    queue.pushed = 0;
    let (abandoned_tx, abandoned_rx) = mpsc::channel();
    pump.callbacks.on_abandon =
        Box::new(move |abandoned_key, count| abandoned_tx.send((abandoned_key, count)).unwrap());

    assert_eq!(pump.send_ready(), Ok(true));

    assert!(matches!(pump.halted.get(&key), Some(Some(_))));
    assert!(pump.queues[&key].spans.is_empty());
    assert_eq!(abandoned_rx.recv().unwrap(), (key, 1));
}

#[test]
fn inferred_halt_without_host_ack_escalates() {
    let key = AxisKey { mcu_id: 2, axis: 1 };
    let (escalated_tx, escalated_rx) = mpsc::channel();
    let mut pump = stalled_queue_pump(key, Duration::from_secs(1), move |message| {
        escalated_tx.send(message).unwrap()
    });
    pump.halted.insert(
        key,
        Some(std::time::Instant::now() - Duration::from_secs(2)),
    );

    pump.enqueue(make_enqueue(
        key,
        vec![make_span(10)],
        crate::anchor::StreamEpoch::Continuation,
    ));

    let message = escalated_rx.recv().unwrap();
    assert!(message.contains("endpoint halt was not acknowledged"));
}

#[test]
fn consumption_stall_past_threshold_with_frozen_counter_escalates() {
    let key = AxisKey { mcu_id: 1, axis: 0 };
    let threshold = Duration::from_millis(50);
    let escalated: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let escalated_cb = Arc::clone(&escalated);
    let mut pump = stalled_queue_pump(key, threshold, move |msg: String| {
        escalated_cb.lock_ok().push(msg)
    });

    assert_eq!(
        pump.send_ready()
            .expect("first stall observation is not fatal"),
        false
    );
    assert!(escalated.lock_ok().is_empty());

    std::thread::sleep(threshold * 2);

    let result = pump.send_ready();
    assert!(
        result.is_err(),
        "consumed count frozen past the threshold must escalate and stop the pump loop"
    );
    let msgs = escalated.lock_ok();
    assert_eq!(msgs.len(), 1, "on_drip_stall must fire exactly once");
    assert!(
        msgs[0].contains("consumption stall"),
        "escalation message should explain the consumption stall: {}",
        msgs[0]
    );
}

#[test]
fn consumption_stall_resets_when_heartbeat_advances_counter() {
    let key = AxisKey { mcu_id: 1, axis: 0 };
    let threshold = Duration::from_millis(50);
    let escalated: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let escalated_cb = Arc::clone(&escalated);
    let mut pump = stalled_queue_pump(key, threshold, move |msg: String| {
        escalated_cb.lock_ok().push(msg)
    });
    pump.queues.get_mut(&key).unwrap().ring_depth = 2;
    pump.queues.get_mut(&key).unwrap().pushed = 2;

    pump.send_ready().unwrap();
    let (_, consumed_at_onset, _) = pump
        .consumption_stall
        .started()
        .expect("first observation tracked");
    assert_eq!(consumed_at_onset, 0);

    std::thread::sleep(threshold / 2);
    pump.handle_control_msg(PumpMsg::Heartbeat(HeartbeatMsg {
        mcu_id: 1,
        axes: vec![0],
        consumed_counts: Some(vec![1]),
        retired_counts: vec![0],
        retired_by: RetiredBy::Pulse,
    }));
    pump.queues.get_mut(&key).unwrap().pushed = 3;

    let result = pump.send_ready();
    assert!(
        result.is_ok(),
        "consumption advancing before the threshold must not escalate"
    );
    assert!(
        escalated.lock_ok().is_empty(),
        "no escalation once consumption progressed"
    );
    let (tracked_key, tracked_consumed, _) = pump
        .consumption_stall
        .started()
        .expect("still stalled on a full ring, just with a fresh timer");
    assert_eq!(tracked_key, key);
    assert_eq!(
        tracked_consumed, 1,
        "stall tracking must reset to the newly observed consumed count"
    );

    std::thread::sleep(threshold / 2 + Duration::from_millis(5));
    let result = pump.send_ready();
    assert!(
        result.is_ok(),
        "elapsed time since the reset is still under the threshold"
    );
    assert!(escalated.lock_ok().is_empty());
}

#[test]
fn non_stallfull_outcome_between_stalls_resets_the_timer() {
    let key = AxisKey { mcu_id: 1, axis: 0 };
    let threshold = Duration::from_millis(50);
    let escalated: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let escalated_cb = Arc::clone(&escalated);
    let mut pump = stalled_queue_pump(key, threshold, move |msg: String| {
        escalated_cb.lock_ok().push(msg)
    });

    pump.send_ready().unwrap();
    assert!(pump.consumption_stall.started().is_some());

    std::thread::sleep(threshold * 2);

    pump.queues.get_mut(&key).unwrap().spans.clear();
    let result = pump.send_ready();
    assert!(result.is_ok());
    assert!(
        pump.consumption_stall.started().is_none(),
        "an Idle outcome must clear the stall tracking even though the old \
         stall was already past the threshold"
    );

    pump.queues
        .get_mut(&key)
        .unwrap()
        .spans
        .push_back(make_span(0));
    let result = pump.send_ready();
    assert!(
        result.is_ok(),
        "the timer restarts fresh after the Idle outcome, so this new \
         StallFull observation is not immediately fatal"
    );
    assert!(escalated.lock_ok().is_empty());
    assert!(pump.consumption_stall.started().is_some());
}

const BUZZ_MCU: u32 = 9;
const BUZZ_CYCLES_PER_SECOND: f64 = 1_000_000.0;

struct BuzzFixture {
    pump: Pump<NullSink>,
    control: crossbeam_channel::Sender<PumpMsg>,
    _control_rx: crossbeam_channel::Receiver<PumpMsg>,
    clock_queries: Arc<Mutex<Vec<u32>>>,
}

/// A pump whose mcu clock walks forward one second on every query, so two
/// routes that each resolved their own start would be armed a second apart.
fn buzz_fixture() -> BuzzFixture {
    let clock_queries: Arc<Mutex<Vec<u32>>> = Arc::new(Mutex::new(Vec::new()));
    let queries_for_clock = Arc::clone(&clock_queries);
    let pump = Pump {
        queues: BTreeMap::new(),
        junctions: JunctionTracker::default(),
        cohort: None,
        halted: BTreeMap::new(),
        sink: NullSink,
        callbacks: PumpCallbacks {
            mcu_clock_of: Box::new(move |mcu_id| {
                let mut queries = queries_for_clock.lock_ok();
                queries.push(mcu_id);
                #[allow(clippy::cast_possible_truncation)]
                let now = queries.len() as u64 * BUZZ_CYCLES_PER_SECOND as u64;
                Some((now, BUZZ_CYCLES_PER_SECOND))
            }),
            ..PumpCallbacks::noop(super::stepcompress_sink::SHIM_RING_DEPTH)
        },
        history: None,
        ledger: Arc::new(crate::drain::DrainLedger::new()),
        pending_barrier_acks: Vec::new(),
        backlog: Arc::new(AtomicU64::new(0)),
        holding_ahead: false,
        data_open: true,
        intake_batch_open: false,
        consumption_stall: super::stall::ConsumptionStallWatch::new(Duration::from_secs(60)),
        mem_probe: super::memstat::MemPressureProbe::new(),
    };
    let (control, control_rx) = crossbeam_channel::unbounded();
    BuzzFixture {
        pump,
        control,
        _control_rx: control_rx,
        clock_queries,
    }
}

fn buzz_wave() -> BuzzWave {
    BuzzWave {
        freq_start_millihz: 40_000,
        freq_end_millihz: 40_000,
        amplitude_nm: 20_000,
        duration_ms: 100,
        ramp_ms: 5,
    }
}

fn pulse_endpoint(
    control: &crossbeam_channel::Sender<PumpMsg>,
    axis: usize,
    oid: u32,
) -> Arc<Mutex<StepcompressEndpoint>> {
    let clock_of: ClockSource = Arc::new(|_| Some((0, BUZZ_CYCLES_PER_SECOND)));
    let egress: FrameEgress = Arc::new(|_| Ok(()));
    let motors = vec![step_shim::MotorConfig {
        oid,
        microstep_distance: 0.01,
        invert_dir: false,
        cycles_per_second: BUZZ_CYCLES_PER_SECOND,
        min_rearm_cycles: 0,
        encoder: step_shim::StepEncoder::Classic {
            max_error_ticks: step_shim::compress::DEFAULT_MAX_ERROR_TICKS,
        },
    }];
    Arc::new(Mutex::new(StepcompressEndpoint::new(
        BUZZ_MCU,
        step_shim::StepShim::new(motors, super::stepcompress_sink::SHIM_RING_DEPTH),
        vec![axis],
        vec![oid],
        egress,
        control.clone(),
        clock_of,
        4,
    )))
}

fn phase_endpoint(
    control: &crossbeam_channel::Sender<PumpMsg>,
    axis: u8,
    oid: u32,
) -> Arc<Mutex<SampleEndpoint>> {
    let clock_of: ClockSource = Arc::new(|_| Some((0, BUZZ_CYCLES_PER_SECOND)));
    let egress: FrameEgress = Arc::new(|_| Ok(()));
    let lanes = vec![SampleLaneConfig {
        axis,
        oid,
        cycles_per_second: BUZZ_CYCLES_PER_SECOND,
        sample_rate_hz: 2_000,
        position_quantum_mm: 0.01,
        max_units_per_sample: 4_096,
        ring_depth: 64,
    }];
    let endpoint = SampleEndpoint::new(BUZZ_MCU, &lanes, egress, clock_of, control.clone())
        .expect("the lane config is representable");
    Arc::new(Mutex::new(endpoint))
}

fn submit_buzz(pump: &mut Pump<NullSink>, routes: Vec<BuzzRoute>) -> Result<BuzzToken, String> {
    let (reply, answer) = mpsc::sync_channel(1);
    pump.handle_control_msg(PumpMsg::Buzz {
        params: BuzzParams {
            routes: routes.into(),
            wave: buzz_wave(),
        },
        reply,
    });
    answer.recv().expect("the pump answers every arming")
}

/// The same pulse endpoint named twice used to pass preflight — every route
/// looked idle — and then arm once before the duplicate found the endpoint
/// already sweeping, leaving one transport in motion off a refused request.
#[test]
fn a_transport_named_twice_is_refused_with_nothing_armed() {
    let mut fixture = buzz_fixture();
    let endpoint = pulse_endpoint(&fixture.control, 0, 7);
    let route = || BuzzRoute::Pulse {
        mcu_id: BUZZ_MCU,
        endpoint: Arc::clone(&endpoint),
        axis_mask: 0b001,
        sign_mask: 0,
    };
    let error = submit_buzz(&mut fixture.pump, vec![route(), route()])
        .expect_err("one arming may name a transport once");
    assert!(
        error.contains("named twice"),
        "the duplicate must be named as the reason: {error}"
    );
    assert!(
        endpoint.lock_ok().buzz_complete(),
        "a refused arming leaves the endpoint untouched"
    );
}

/// A phase lane driven with a sign of zero is refused by the endpoint at arm
/// time. Preflight has to catch it, or the routes ahead of it are already
/// sweeping when the refusal arrives.
#[test]
fn an_invalid_lane_sign_is_refused_before_any_route_is_armed() {
    let mut fixture = buzz_fixture();
    let pulse = pulse_endpoint(&fixture.control, 0, 7);
    let phase = phase_endpoint(&fixture.control, 1, 8);
    let error = submit_buzz(
        &mut fixture.pump,
        vec![
            BuzzRoute::Pulse {
                mcu_id: BUZZ_MCU,
                endpoint: Arc::clone(&pulse),
                axis_mask: 0b001,
                sign_mask: 0,
            },
            BuzzRoute::Phase {
                mcu_id: BUZZ_MCU,
                endpoint: Arc::clone(&phase),
                lanes: vec![BuzzLane { axis: 1, sign: 0.0 }],
            },
        ],
    )
    .expect_err("a lane driven with sign 0 drives nothing");
    assert!(
        error.contains("sign 0"),
        "the refusal must name the sign: {error}"
    );
    assert!(
        pulse.lock_ok().buzz_complete(),
        "the route ahead of the invalid one must not have been armed"
    );
}

/// Two transports of one mcu are one sweep: they must be anchored on the one
/// start the pump resolved for that mcu, not on whatever instant the pump
/// happened to reach each transport at.
#[test]
fn routes_on_one_mcu_are_armed_from_a_single_resolved_start() {
    let mut fixture = buzz_fixture();
    let pulse = pulse_endpoint(&fixture.control, 0, 7);
    let phase = phase_endpoint(&fixture.control, 1, 8);
    submit_buzz(
        &mut fixture.pump,
        vec![
            BuzzRoute::Pulse {
                mcu_id: BUZZ_MCU,
                endpoint: Arc::clone(&pulse),
                axis_mask: 0b001,
                sign_mask: 0,
            },
            BuzzRoute::Phase {
                mcu_id: BUZZ_MCU,
                endpoint: Arc::clone(&phase),
                lanes: vec![BuzzLane { axis: 1, sign: 1.0 }],
            },
        ],
    )
    .expect("both idle transports accept one sweep");
    assert_eq!(
        *fixture.clock_queries.lock_ok(),
        vec![BUZZ_MCU],
        "the start of one mcu is resolved once and shared by its routes"
    );
    assert!(
        !pulse.lock_ok().buzz_complete(),
        "the pulse lane is sweeping"
    );
    assert!(
        !phase.lock_ok().buzz_complete().expect("no latched fatal"),
        "the phase lane is sweeping"
    );
}
