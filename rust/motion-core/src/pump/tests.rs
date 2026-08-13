use super::pump_loop::{
    PUMP_INTAKE_BACKLOG_HARD_CAP, PUMP_INTAKE_BACKLOG_SOFT_CAP, PUMP_INTAKE_MIN_RUNWAY_SECS, Pump,
    wants_pieces,
};
use super::*;
use crossbeam_channel::unbounded;
use runtime::piece_ring::PieceEntry;
use std::collections::BTreeMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc;
use std::sync::{Arc, Mutex};
use std::time::Duration;

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
    pieces: Vec<(PieceEntry, f64)>,
    epoch: crate::anchor::StreamEpoch,
) -> EnqueueMsg {
    EnqueueMsg {
        epoch_freq: None,
        key,
        pieces,
        epoch,
        lead_secs: MAX_LEAD_SECS,
        source_line: u32::MAX,
    }
}

#[test]
fn room_full_then_drains() {
    let mut q = AxisQueue::new(4);
    assert_eq!(q.room(), 4);
    q.pushed = 4;
    assert_eq!(q.room(), 0);
    q.accepted = 1;
    assert_eq!(q.room(), 1);
}

#[test]
fn accepted_pieces_reopen_capacity_before_execution_retires_them() {
    let mut q = AxisQueue::new(64);
    q.pushed = 64;
    q.accepted = 64;
    q.retired = 0;

    assert_eq!(q.room(), 64);
    assert_ne!(q.pushed, q.retired);
}

#[test]
fn room_correct_across_u32_wrap() {
    let mut q = AxisQueue::new(8);
    q.pushed = 2;
    q.accepted = u32::MAX;
    assert_eq!(
        q.room(),
        5,
        "legitimate counter rollover: accepted is numerically larger than pushed only because \
         the u32 odometer wrapped, so 3 pieces are genuinely awaiting acceptance. wrapping_sub \
         recovers 3; saturating subtraction would wrongly report a drained ring."
    );
}

#[test]
fn room_recovers_when_accepted_overtakes_pushed() {
    let mut q = AxisQueue::new(4);
    q.pushed = 100;
    q.accepted = 101;
    assert_eq!(
        q.room(),
        4,
        "accepted overtook pushed by 1 after a lost response; the inversion must reconcile to a \
         drained ring instead of wedging with zero room"
    );
}

#[test]
fn schedule_resends_orphan_when_accepted_overtook_pushed() {
    let key = AxisKey { mcu_id: 1, axis: 0 };
    let mut q = AxisQueue::new(8);
    q.pushed = 100;
    q.accepted = 101;
    q.pieces.push_back(make_piece(101));
    let mut queues: BTreeMap<AxisKey, AxisQueue> = BTreeMap::new();
    queues.insert(key, q);

    const MAX_PER_FRAME: usize = 32;
    match schedule(
        &queues,
        |_| crate::pump::BundleLimits {
            wire_budget: usize::MAX,
            pieces_per_axis: MAX_PER_FRAME,
        },
        |_, _| None,
        |_| usize::MAX,
    ) {
        Schedule::Send(frames) => {
            assert_eq!(frames.len(), 1, "exactly the inverted axis is scheduled");
            assert_eq!(frames[0].key, key);
        }
        other => {
            panic!("accepted>pushed inversion must schedule a re-send, not wedge; got {other:?}")
        }
    }
}

#[test]
fn run_pump_delivers_piece_despite_retired_over_pushed_inversion() {
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
        vec![make_piece(0)],
        crate::anchor::StreamEpoch::Continuation,
    ))
    .unwrap();
    wait_until(
        || sink.recorded().len() == 1,
        "first piece delivered, creating the axis queue with pushed=1 (a heartbeat \
         for an axis with no queue yet is dropped)",
    );

    ctl.send(PumpMsg::Heartbeat(HeartbeatMsg {
        mcu_id: 1,
        accepted_counts: None,
        retired_counts: vec![2],
    }))
    .unwrap();
    let (ack_tx, ack_rx) = mpsc::sync_channel::<()>(1);
    ctl.send(PumpMsg::Barrier(ack_tx)).unwrap();
    ack_rx
        .recv()
        .expect("barrier acks only after the retired=2 heartbeat ahead of it applies");

    data.send(make_enqueue(
        key,
        vec![make_piece(1)],
        crate::anchor::StreamEpoch::Continuation,
    ))
    .unwrap();
    wait_until(
        || sink.recorded().len() == 2,
        "second piece delivered despite retired(2) > pushed(1): buggy room() \
         underflows to 0 and wedges here; the fix reopens room",
    );

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}

#[test]
fn history_records_pieces_at_send_time_not_enqueue_time() {
    const RING_DEPTH: u32 = 8;
    let key = AxisKey { mcu_id: 1, axis: 0 };

    let store = Arc::new(Mutex::new(crate::motion_history::HistoryStore::default()));
    let nominal_freqs = Arc::new(Mutex::new(std::collections::HashMap::from([(
        1u32,
        50_000_000u32,
    )])));
    let history = HistoryRecorder {
        store: Arc::clone(&store),
        nominal_freqs,
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
    let piece = (
        PieceEntry {
            start_time: 100,
            duration: 0.001,
            ..PieceEntry::zeroed()
        },
        host_t,
    );
    data.send(make_enqueue(
        key,
        vec![piece],
        crate::anchor::StreamEpoch::Continuation,
    ))
    .unwrap();
    wait_until(|| sink.recorded().len() == 1, "piece sent to the MCU");
    wait_until(
        || {
            store
                .lock()
                .unwrap()
                .state_at_host(key, host_t, None)
                .is_ok()
        },
        "sent piece recorded into motion history with its host key",
    );

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}

#[test]
fn physical_write_cursor_advances_and_wraps_at_n() {
    let mut q = AxisQueue::new(4);
    assert_eq!(q.physical_write_cursor, 0);
    q.advance_write_cursor(3);
    assert_eq!(q.physical_write_cursor, 3);
    q.advance_write_cursor(3);
    assert_eq!(q.physical_write_cursor, 2);
}

#[derive(Clone)]
struct RecordingSink {
    calls: Arc<Mutex<Vec<(u16, u32)>>>,
}

impl RecordingSink {
    fn new() -> Self {
        Self {
            calls: Arc::new(Mutex::new(Vec::new())),
        }
    }
    fn recorded(&self) -> Vec<(u16, u32)> {
        self.calls.lock().unwrap().clone()
    }
}

impl PieceSink for RecordingSink {
    fn send_frame(
        &self,
        _key: AxisKey,
        _pieces: &[PieceEntry],
        start_slot: u16,
        new_head: u32,
        _room: u32,
    ) -> Result<i32, SendError> {
        self.calls.lock().unwrap().push((start_slot, new_head));
        Ok(mcu_protocol::result_codes::OK)
    }
}

fn make_piece(t: u64) -> (PieceEntry, f64) {
    (
        PieceEntry {
            start_time: t,
            duration: 0.001,
            ..PieceEntry::zeroed()
        },
        t as f64,
    )
}

fn staged_runway(piece_count: u64, runway_secs: f64) -> BTreeMap<AxisKey, AxisQueue> {
    let duration = runway_secs / piece_count as f64;
    let mut queue = AxisQueue::new(u32::MAX);
    queue.pieces.extend((0..piece_count).map(|index| {
        (
            PieceEntry {
                start_time: index,
                duration: duration as f32,
                ..PieceEntry::zeroed()
            },
            index as f64 * duration,
        )
    }));
    BTreeMap::from([(AxisKey { mcu_id: 1, axis: 0 }, queue)])
}

#[test]
fn pump_intake_extends_only_dense_backlogs_to_the_hard_cap() {
    let sparse = staged_runway(
        PUMP_INTAKE_BACKLOG_SOFT_CAP,
        PUMP_INTAKE_MIN_RUNWAY_SECS * 2.0,
    );
    assert!(!wants_pieces(&sparse));

    let dense = staged_runway(
        PUMP_INTAKE_BACKLOG_SOFT_CAP,
        PUMP_INTAKE_MIN_RUNWAY_SECS * 0.5,
    );
    assert!(wants_pieces(&dense));

    let pathological = staged_runway(
        PUMP_INTAKE_BACKLOG_HARD_CAP,
        PUMP_INTAKE_MIN_RUNWAY_SECS * 0.5,
    );
    assert!(!wants_pieces(&pathological));
}

#[test]
fn run_pump_sets_start_slot_from_cursor_and_advances_it() {
    const RING_DEPTH: u32 = 8;
    const N: u32 = 3;

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
        AxisKey { mcu_id: 1, axis: 0 },
        (0..N).map(|i| make_piece(i as u64)).collect(),
        crate::anchor::StreamEpoch::Continuation,
    ))
    .unwrap();
    {
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
        while sink.recorded().is_empty() {
            assert!(
                std::time::Instant::now() < deadline,
                "pump did not drain first batch within deadline"
            );
            std::thread::yield_now();
        }
    }

    data.send(make_enqueue(
        AxisKey { mcu_id: 1, axis: 0 },
        (N..N * 2).map(|i| make_piece(i as u64)).collect(),
        crate::anchor::StreamEpoch::Continuation,
    ))
    .unwrap();
    {
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
        while sink.recorded().len() < 2 {
            assert!(
                std::time::Instant::now() < deadline,
                "pump did not drain second batch within deadline"
            );
            std::thread::yield_now();
        }
    }

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();

    let recorded = sink.recorded();
    assert_eq!(
        recorded.len(),
        2,
        "expected exactly 2 sends, got {}",
        recorded.len()
    );

    let (s0, h0) = recorded[0];
    let (s1, h1) = recorded[1];

    assert_eq!(s0, 0, "first start_slot should be 0");
    assert_eq!(h0, N, "first new_head should be N={N}");

    let expected_s1 = (N % RING_DEPTH) as u16;
    assert_eq!(s1, expected_s1, "second start_slot should be {expected_s1}");
    assert_eq!(h1, N * 2, "second new_head should be {}", N * 2);
}

fn make_piece_pos(t: u64, mask: u8, c0: f32, c3: f32) -> (PieceEntry, f64) {
    let d = 0.001_f64;
    let (b0, b1, b2, b3) = (c0 as f64, c0 as f64, c3 as f64, c3 as f64);
    let mono = [
        b0,
        3.0 * (b1 - b0) / d,
        3.0 * (b2 - 2.0 * b1 + b0) / (d * d),
        (b3 - 3.0 * b2 + 3.0 * b1 - b0) / (d * d * d),
    ];
    let cheb = nurbs::chebyshev::monomial_tau_to_chebyshev(&mono, d);
    let mut coeffs = [0.0_f32; runtime::piece_ring::MAX_PIECE_COEFFS];
    for (dst, src) in coeffs.iter_mut().zip(&cheb) {
        *dst = *src as f32;
    }
    (
        PieceEntry {
            start_time: t,
            duration: d as f32,
            motor_mask: mask,
            coeff_count: cheb.len() as u8,
            coeffs,
            ..PieceEntry::zeroed()
        },
        t as f64,
    )
}

#[test]
fn overlay_piece_after_move_is_exempt_from_junction_continuity() {
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
                mcu_clock_of: Box::new(|_mcu| Some((0u64, 180_000_000.0))),
                ..PumpCallbacks::noop(8)
            },
            None,
            std::sync::Arc::new(crate::drain::DrainLedger::new()),
            Arc::new(AtomicU64::new(0)),
        );
    });
    let key = AxisKey { mcu_id: 1, axis: 2 };

    data.send(make_enqueue(
        key,
        vec![make_piece_pos(0, 0, 0.0, 11.0)],
        crate::anchor::StreamEpoch::Continuation,
    ))
    .unwrap();
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
    while sink.recorded().is_empty() {
        assert!(
            std::time::Instant::now() < deadline,
            "first piece never sent"
        );
        std::thread::yield_now();
    }

    data.send(make_enqueue(
        key,
        vec![make_piece_pos(10_000, 0b10, 0.0, 0.5)],
        crate::anchor::StreamEpoch::Continuation,
    ))
    .unwrap();
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
    while sink.recorded().len() < 2 {
        assert!(
            std::time::Instant::now() < deadline,
            "overlay piece never sent — pump likely panicked on the junction check"
        );
        std::thread::yield_now();
    }

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
    assert_eq!(sink.recorded().len(), 2, "both pieces dispatched, no panic");
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

impl PieceSink for NullSink {
    fn send_frame(
        &self,
        _key: AxisKey,
        _pieces: &[PieceEntry],
        _start_slot: u16,
        _new_head: u32,
        _room: u32,
    ) -> Result<i32, SendError> {
        Ok(mcu_protocol::result_codes::OK)
    }
}

struct HaltedSink;

impl PieceSink for HaltedSink {
    fn send_frame(
        &self,
        _key: AxisKey,
        _pieces: &[PieceEntry],
        _start_slot: u16,
        _new_head: u32,
        _room: u32,
    ) -> Result<i32, SendError> {
        Err(SendError::Halted("endpoint stream halted".into()))
    }
}

#[test]
fn flush_clears_queued_pieces_and_junctions() {
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
                mcu_clock_of: Box::new(move |_mcu| *clock_pump.lock().unwrap()),
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
        pieces: (0u64..4)
            .map(|i| {
                let mut p = PieceEntry {
                    start_time: gated_tick + i,
                    duration: 0.001,
                    ..PieceEntry::zeroed()
                };
                // Distinct hold values so enqueue's hold merging cannot
                // coalesce the gated pieces this test counts one by one.
                p.coeffs[0] = 1.0 + i as f32;
                (p, (gated_tick + i) as f64)
            })
            .collect(),
        epoch: crate::anchor::StreamEpoch::Reposition,
        lead_secs,
        source_line: u32::MAX,
    })
    .unwrap();

    std::thread::sleep(std::time::Duration::from_millis(30));

    ctl.send(PumpMsg::Flush(vec![key])).unwrap();
    std::thread::sleep(std::time::Duration::from_millis(20));

    *clock.lock().unwrap() = Some((gated_tick + 1_000, freq));

    data.send(EnqueueMsg {
        epoch_freq: None,
        key,
        pieces: vec![(
            PieceEntry {
                // A deliverable "now" probe (== the advanced clock), not a stale
                // past piece — the pump's in-past guard aborts on past start_times.
                start_time: gated_tick + 1_000,
                duration: 0.001,
                ..PieceEntry::zeroed()
            },
            (gated_tick + 1_000) as f64,
        )],
        epoch: crate::anchor::StreamEpoch::Continuation,
        lead_secs,
        source_line: u32::MAX,
    })
    .unwrap();
    {
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
        while sink.recorded().is_empty() {
            assert!(
                std::time::Instant::now() < deadline,
                "pump never sent the post-flush probe piece — deadlocked"
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
        "sink must see only the post-flush probe piece; \
         {} sends means the {} gated pieces survived Flush",
        recorded.len(),
        4
    );
}

#[test]
fn on_abandon_reports_flushed_not_pushed_pieces() {
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
                mcu_clock_of: Box::new(move |_mcu| *clock_pump.lock().unwrap()),
                on_abandon: Box::new(move |_k: AxisKey, n: u32| {
                    *abandoned_pump.lock().unwrap() += n;
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
        pieces: (0u64..4)
            .map(|i| {
                let mut p = PieceEntry {
                    start_time: gated_tick + i,
                    duration: 0.001,
                    ..PieceEntry::zeroed()
                };
                // Distinct hold values so enqueue's hold merging cannot
                // coalesce the gated pieces this test counts one by one.
                p.coeffs[0] = 1.0 + i as f32;
                (p, (gated_tick + i) as f64)
            })
            .collect(),
        epoch: crate::anchor::StreamEpoch::Reposition,
        lead_secs,
        source_line: u32::MAX,
    })
    .unwrap();

    std::thread::sleep(std::time::Duration::from_millis(30));
    ctl.send(PumpMsg::Flush(vec![key])).unwrap();
    std::thread::sleep(std::time::Duration::from_millis(20));
    *clock.lock().unwrap() = Some((gated_tick + 1_000, freq));

    data.send(EnqueueMsg {
        epoch_freq: None,
        key,
        pieces: vec![(
            PieceEntry {
                // A deliverable "now" probe (== the advanced clock), not a stale
                // past piece — the pump's in-past guard aborts on past start_times.
                start_time: gated_tick + 1_000,
                duration: 0.001,
                ..PieceEntry::zeroed()
            },
            (gated_tick + 1_000) as f64,
        )],
        epoch: crate::anchor::StreamEpoch::Continuation,
        lead_secs,
        source_line: u32::MAX,
    })
    .unwrap();
    {
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
        while sink.recorded().is_empty() {
            assert!(
                std::time::Instant::now() < deadline,
                "pump never sent the post-flush probe piece — deadlocked"
            );
            std::thread::yield_now();
        }
    }

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();

    assert_eq!(
        *abandoned_total.lock().unwrap(),
        4,
        "on_abandon must report the 4 Flush-dropped pieces and not the pushed probe"
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
        (0..3).map(|i| make_piece(i as u64)).collect(),
        crate::anchor::StreamEpoch::Continuation,
    ))
    .unwrap();
    poll_until(
        || backlog.load(Ordering::Acquire) == 3,
        "ring-full pump never staged the 3 un-pushed pieces",
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
        "pieces flushed before the barrier must never reach the sink; got {:?}",
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
fn pump_backlog_reflects_unpushed_pieces() {
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
        (0..3).map(|i| make_piece(i as u64)).collect(),
        crate::anchor::StreamEpoch::Continuation,
    ))
    .unwrap();

    poll_until(
        || backlog.load(Ordering::Acquire) == 3,
        "ring-full pump never reported the 3 unpushed pieces",
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
        (0..3).map(|i| make_piece(i as u64)).collect(),
        crate::anchor::StreamEpoch::Continuation,
    ))
    .unwrap();

    poll_until(|| !sink.recorded().is_empty(), "pump never pushed pieces");
    poll_until(
        || backlog.load(Ordering::Acquire) == 0,
        "backlog never returned to zero after the ring accepted the pieces",
    );

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}

fn queue_pump<S: PieceSink>(
    key: AxisKey,
    acceptance_stall_fatal: Duration,
    on_drip_stall: impl Fn(String) + Send + 'static,
    sink: S,
) -> Pump<S> {
    let mut queues = BTreeMap::new();
    let mut q = AxisQueue::new(1);
    q.pushed = 1;
    q.retired = 0;
    q.pieces.push_back(make_piece(0));
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
        acceptance_stall: super::stall::AcceptanceStallWatch::new(acceptance_stall_fatal),
        mem_probe: super::memstat::MemPressureProbe::new(),
    }
}

fn stalled_queue_pump(
    key: AxisKey,
    acceptance_stall_fatal: Duration,
    on_drip_stall: impl Fn(String) + Send + 'static,
) -> Pump<NullSink> {
    queue_pump(key, acceptance_stall_fatal, on_drip_stall, NullSink)
}

#[test]
fn send_pass_deadline_yields_with_work_pending() {
    let key = AxisKey { mcu_id: 1, axis: 0 };
    let sink = RecordingSink::new();
    let mut pump = queue_pump(key, Duration::from_secs(1), |_| {}, sink.clone());
    let q = pump.queues.get_mut(&key).unwrap();
    q.pushed = 0;
    q.pieces.clear();
    q.ring_depth = 4_000;
    let queued: u64 = 3_000;
    for i in 0..queued {
        q.pieces.push_back(make_piece(i * 1_000));
    }

    assert_eq!(pump.send_ready_until(std::time::Instant::now()), Ok(true));

    assert_eq!(
        sink.recorded().len(),
        1,
        "an expired pass deadline still sends exactly one bundle"
    );
    let remaining = pump.queues[&key].pieces.len() as u64;
    assert!(
        remaining > 0 && remaining < queued,
        "one bundle went out, the rest waits so intake can interleave (remaining={remaining})"
    );

    assert_eq!(
        pump.send_ready_until(std::time::Instant::now() + Duration::from_secs(60)),
        Ok(true)
    );
    assert!(
        pump.queues[&key].pieces.is_empty(),
        "a roomy deadline drains the queue"
    );
}

#[test]
fn halt_drops_queued_and_new_pieces_until_resume() {
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
    assert!(pump.queues[&key].pieces.is_empty());
    pump.enqueue(make_enqueue(
        key,
        vec![make_piece(10)],
        crate::anchor::StreamEpoch::Continuation,
    ));
    assert!(pump.queues[&key].pieces.is_empty());
    assert!(matches!(
        escalated_rx.try_recv(),
        Err(mpsc::TryRecvError::Empty)
    ));

    pump.handle_control_msg(PumpMsg::Resume(vec![key]));
    pump.enqueue(make_enqueue(
        key,
        vec![make_piece(20)],
        crate::anchor::StreamEpoch::Continuation,
    ));
    assert_eq!(pump.queues[&key].pieces.len(), 1);
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
    assert!(pump.queues[&key].pieces.is_empty());
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
        vec![make_piece(10)],
        crate::anchor::StreamEpoch::Continuation,
    ));

    let message = escalated_rx.recv().unwrap();
    assert!(message.contains("endpoint halt was not acknowledged"));
}

#[test]
fn acceptance_stall_past_threshold_with_frozen_accepted_escalates() {
    let key = AxisKey { mcu_id: 1, axis: 0 };
    let threshold = Duration::from_millis(50);
    let escalated: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let escalated_cb = Arc::clone(&escalated);
    let mut pump = stalled_queue_pump(key, threshold, move |msg: String| {
        escalated_cb.lock().unwrap().push(msg)
    });

    assert_eq!(
        pump.send_ready()
            .expect("first stall observation is not fatal"),
        false
    );
    assert!(escalated.lock().unwrap().is_empty());

    std::thread::sleep(threshold * 2);

    let result = pump.send_ready();
    assert!(
        result.is_err(),
        "accepted count frozen past the threshold must escalate and stop the pump loop"
    );
    let msgs = escalated.lock().unwrap();
    assert_eq!(msgs.len(), 1, "on_drip_stall must fire exactly once");
    assert!(
        msgs[0].contains("acceptance stall"),
        "escalation message should explain the acceptance stall: {}",
        msgs[0]
    );
}

#[test]
fn acceptance_stall_resets_when_heartbeat_advances_accepted() {
    let key = AxisKey { mcu_id: 1, axis: 0 };
    let threshold = Duration::from_millis(50);
    let escalated: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let escalated_cb = Arc::clone(&escalated);
    let mut pump = stalled_queue_pump(key, threshold, move |msg: String| {
        escalated_cb.lock().unwrap().push(msg)
    });
    pump.queues.get_mut(&key).unwrap().ring_depth = 2;
    pump.queues.get_mut(&key).unwrap().pushed = 2;

    pump.send_ready().unwrap();
    let (_, accepted_at_onset, _) = pump
        .acceptance_stall
        .started()
        .expect("first observation tracked");
    assert_eq!(accepted_at_onset, 0);

    std::thread::sleep(threshold / 2);
    pump.handle_control_msg(PumpMsg::Heartbeat(HeartbeatMsg {
        mcu_id: 1,
        accepted_counts: Some(vec![1]),
        retired_counts: vec![0],
    }));
    pump.queues.get_mut(&key).unwrap().pushed = 3;

    let result = pump.send_ready();
    assert!(
        result.is_ok(),
        "acceptance advancing before the threshold must not escalate"
    );
    assert!(
        escalated.lock().unwrap().is_empty(),
        "no escalation once acceptance progressed"
    );
    let (tracked_key, tracked_accepted, _) = pump
        .acceptance_stall
        .started()
        .expect("still stalled on a full ring, just with a fresh timer");
    assert_eq!(tracked_key, key);
    assert_eq!(
        tracked_accepted, 1,
        "stall tracking must reset to the newly observed accepted count"
    );

    std::thread::sleep(threshold / 2 + Duration::from_millis(5));
    let result = pump.send_ready();
    assert!(
        result.is_ok(),
        "elapsed time since the reset is still under the threshold"
    );
    assert!(escalated.lock().unwrap().is_empty());
}

#[test]
fn non_stallfull_outcome_between_stalls_resets_the_timer() {
    let key = AxisKey { mcu_id: 1, axis: 0 };
    let threshold = Duration::from_millis(50);
    let escalated: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let escalated_cb = Arc::clone(&escalated);
    let mut pump = stalled_queue_pump(key, threshold, move |msg: String| {
        escalated_cb.lock().unwrap().push(msg)
    });

    pump.send_ready().unwrap();
    assert!(pump.acceptance_stall.started().is_some());

    std::thread::sleep(threshold * 2);

    pump.queues.get_mut(&key).unwrap().pieces.clear();
    let result = pump.send_ready();
    assert!(result.is_ok());
    assert!(
        pump.acceptance_stall.started().is_none(),
        "an Idle outcome must clear the stall tracking even though the old \
         stall was already past the threshold"
    );

    pump.queues
        .get_mut(&key)
        .unwrap()
        .pieces
        .push_back(make_piece(0));
    let result = pump.send_ready();
    assert!(
        result.is_ok(),
        "the timer restarts fresh after the Idle outcome, so this new \
         StallFull observation is not immediately fatal"
    );
    assert!(escalated.lock().unwrap().is_empty());
    assert!(pump.acceptance_stall.started().is_some());
}

mod pushpieces_retransmit_tests {
    use super::super::{SendError, pushpieces_retransmit_serial};
    use host_rt::transport::TransportError;

    #[test]
    fn recovers_after_transient_failures_within_budget() {
        let mut calls = 0u32;
        let res = pushpieces_retransmit_serial(0, 10, None, || {
            calls += 1;
            if calls < 3 {
                Err(TransportError::Timeout)
            } else {
                Ok(vec![0xAB, 0xCD])
            }
        });
        assert_eq!(res.expect("should recover"), vec![0xAB, 0xCD]);
        assert_eq!(
            calls, 3,
            "succeeds on the 3rd attempt after 2 transient losses"
        );
    }

    #[test]
    fn first_attempt_success_does_not_retry() {
        let mut calls = 0u32;
        let res = pushpieces_retransmit_serial(0, 10, None, || {
            calls += 1;
            Ok(vec![1, 2, 3])
        });
        assert_eq!(res.expect("ok"), vec![1, 2, 3]);
        assert_eq!(
            calls, 1,
            "healthy link: exactly one attempt, no extra latency"
        );
    }

    #[test]
    fn persistent_corruption_gives_up_as_transient_after_budget() {
        let mut calls = 0u32;
        let res = pushpieces_retransmit_serial(0, 4, None, || {
            calls += 1;
            Err(TransportError::Timeout)
        });
        assert!(
            matches!(res, Err(SendError::Transient(_))),
            "budget exhaustion returns Transient (backstop handles it), not Fatal"
        );
        assert_eq!(
            calls, 4,
            "exactly max_attempts attempts — no infinite retry"
        );
    }

    #[test]
    fn expired_front_lead_stops_retrying_immediately() {
        let past = std::time::Instant::now() - std::time::Duration::from_millis(1);
        let mut calls = 0u32;
        let res = pushpieces_retransmit_serial(0, 10, Some(past), || {
            calls += 1;
            Err(TransportError::Timeout)
        });
        match res {
            Err(SendError::Transient(msg)) => assert!(
                msg.contains("transport unresponsive"),
                "give-up must name the unresponsive transport: {msg}"
            ),
            other => panic!("expected Transient, got {other:?}"),
        }
        assert_eq!(
            calls, 1,
            "retrying past the front piece's lead cannot succeed — one attempt only"
        );
    }

    #[test]
    fn distant_deadline_leaves_the_attempt_budget_in_charge() {
        let far = std::time::Instant::now() + std::time::Duration::from_secs(60);
        let mut calls = 0u32;
        let res = pushpieces_retransmit_serial(0, 3, Some(far), || {
            calls += 1;
            Err(TransportError::Timeout)
        });
        assert!(matches!(res, Err(SendError::Transient(_))));
        assert_eq!(
            calls, 3,
            "deep lead: the attempt-count budget still caps retries"
        );
    }

    #[test]
    fn dead_transport_closed_fails_fast_no_retry() {
        let mut calls = 0u32;
        let res = pushpieces_retransmit_serial(0, 10, None, || {
            calls += 1;
            Err(TransportError::Closed)
        });
        assert!(
            matches!(res, Err(SendError::Fatal(_))),
            "Closed = dead transport → Fatal"
        );
        assert_eq!(calls, 1, "no retry on a dead transport");
    }

    #[test]
    fn dead_transport_io_fails_fast_no_retry() {
        let mut calls = 0u32;
        let res = pushpieces_retransmit_serial(0, 10, None, || {
            calls += 1;
            Err(TransportError::Io(std::io::Error::from(
                std::io::ErrorKind::BrokenPipe,
            )))
        });
        assert!(
            matches!(res, Err(SendError::Fatal(_))),
            "Io = dead transport → Fatal"
        );
        assert_eq!(calls, 1, "no retry on a dead transport");
    }

    #[test]
    fn mcu_shutdown_fails_fast_no_retry() {
        let mut calls = 0u32;
        let res = pushpieces_retransmit_serial(0, 10, None, || {
            calls += 1;
            Err(TransportError::McuShutdown("fault -112".into()))
        });
        assert!(
            matches!(res, Err(SendError::Fatal(_))),
            "McuShutdown is a genuine MCU failure → fail loud, not retry"
        );
        assert_eq!(calls, 1, "no retry once the MCU has shut down");
    }
}
