use super::pump_loop::Pump;
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

fn make_enqueue(key: AxisKey, pieces: Vec<(PieceEntry, f64)>, fresh_stream: bool) -> EnqueueMsg {
    EnqueueMsg {
        key,
        pieces,
        fresh_stream,
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
    q.retired = 1;
    assert_eq!(q.room(), 1);
}

#[test]
fn room_correct_across_u32_wrap() {
    let mut q = AxisQueue::new(8);
    q.pushed = 2;
    q.retired = u32::MAX;
    assert_eq!(
        q.room(),
        5,
        "legitimate counter rollover: retired is numerically larger than pushed \
         only because the u32 odometer wrapped, so 3 pieces are genuinely in \
         flight. wrapping_sub recovers 3; a saturating_sub / max(0, ..) would \
         collapse this to 0 in_flight and wrongly report a full ring."
    );
}

#[test]
fn room_recovers_when_retired_overtakes_pushed() {
    let mut q = AxisQueue::new(4);
    q.pushed = 100;
    q.retired = 101;
    assert_eq!(
        q.room(),
        4,
        "retired overtook pushed by 1 (a PushPieces the MCU applied but whose \
         response was lost): in_flight = pushed.wrapping_sub(retired) underflows \
         to u32::MAX and saturating_sub pins room at 0 forever — the mid-print \
         wedge. An inversion (in_flight > ring_depth) must reconcile to a \
         drained ring, not zero room."
    );
}

#[test]
fn schedule_resends_orphan_when_retired_overtook_pushed() {
    let key = AxisKey { mcu_id: 1, axis: 0 };
    let mut q = AxisQueue::new(8);
    q.pushed = 100;
    q.retired = 101;
    q.pieces.push_back(make_piece(101));
    let mut queues: BTreeMap<AxisKey, AxisQueue> = BTreeMap::new();
    queues.insert(key, q);

    const MAX_PER_FRAME: usize = 32;
    match schedule(
        &queues,
        MAX_PER_FRAME,
        usize::MAX,
        |_, _| None,
        |_| usize::MAX,
    ) {
        Schedule::Send(frames) => {
            assert_eq!(frames.len(), 1, "exactly the inverted axis is scheduled");
            assert_eq!(frames[0].key, key);
        }
        other => {
            panic!("retired>pushed inversion must schedule a re-send, not wedge; got {other:?}")
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

    data.send(make_enqueue(key, vec![make_piece(0)], false))
        .unwrap();
    wait_until(
        || sink.recorded().len() == 1,
        "first piece delivered, creating the axis queue with pushed=1 (a heartbeat \
         for an axis with no queue yet is dropped)",
    );

    ctl.send(PumpMsg::Heartbeat(HeartbeatMsg {
        mcu_id: 1,
        retired_counts: vec![2],
    }))
    .unwrap();
    let (ack_tx, ack_rx) = mpsc::sync_channel::<()>(1);
    ctl.send(PumpMsg::Barrier(ack_tx)).unwrap();
    ack_rx
        .recv()
        .expect("barrier acks only after the retired=2 heartbeat ahead of it applies");

    data.send(make_enqueue(key, vec![make_piece(1)], false))
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
    data.send(make_enqueue(key, vec![piece], false)).unwrap();
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
        false,
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
        false,
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
        false,
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
        false,
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
        fresh_stream: true,
        lead_secs,
        source_line: u32::MAX,
    })
    .unwrap();

    std::thread::sleep(std::time::Duration::from_millis(30));

    ctl.send(PumpMsg::Flush(vec![key])).unwrap();
    std::thread::sleep(std::time::Duration::from_millis(20));

    *clock.lock().unwrap() = Some((gated_tick + 1_000, freq));

    data.send(EnqueueMsg {
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
        fresh_stream: false,
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
        fresh_stream: true,
        lead_secs,
        source_line: u32::MAX,
    })
    .unwrap();

    std::thread::sleep(std::time::Duration::from_millis(30));
    ctl.send(PumpMsg::Flush(vec![key])).unwrap();
    std::thread::sleep(std::time::Duration::from_millis(20));
    *clock.lock().unwrap() = Some((gated_tick + 1_000, freq));

    data.send(EnqueueMsg {
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
        fresh_stream: false,
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
        false,
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
        false,
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
        false,
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

fn stalled_queue_pump(
    key: AxisKey,
    retirement_stall_fatal: Duration,
    on_drip_stall: impl Fn(String) + Send + 'static,
) -> Pump<NullSink> {
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
        sink: NullSink,
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
        last_stallfull_log: None,
        retirement_stall_fatal,
        stall_full_since: None,
    }
}

#[test]
fn retirement_stall_past_threshold_with_frozen_retired_escalates() {
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
        "retired frozen past the threshold must escalate and stop the pump loop"
    );
    let msgs = escalated.lock().unwrap();
    assert_eq!(msgs.len(), 1, "on_drip_stall must fire exactly once");
    assert!(
        msgs[0].contains("retirement stall"),
        "escalation message should explain the retirement stall: {}",
        msgs[0]
    );
}

#[test]
fn retirement_stall_resets_when_heartbeat_advances_retired() {
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
    let (_, retired_at_onset, _) = pump.stall_full_since.expect("first observation tracked");
    assert_eq!(retired_at_onset, 0);

    std::thread::sleep(threshold / 2);
    pump.handle_control_msg(PumpMsg::Heartbeat(HeartbeatMsg {
        mcu_id: 1,
        retired_counts: vec![1],
    }));
    pump.queues.get_mut(&key).unwrap().pushed = 3;

    let result = pump.send_ready();
    assert!(
        result.is_ok(),
        "retired advancing before the threshold must not escalate"
    );
    assert!(
        escalated.lock().unwrap().is_empty(),
        "no escalation once retired progressed"
    );
    let (tracked_key, tracked_retired, _) = pump
        .stall_full_since
        .expect("still stalled on a full ring, just with a fresh timer");
    assert_eq!(tracked_key, key);
    assert_eq!(
        tracked_retired, 1,
        "stall tracking must reset to the newly observed retired count"
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
    assert!(pump.stall_full_since.is_some());

    std::thread::sleep(threshold * 2);

    pump.queues.get_mut(&key).unwrap().pieces.clear();
    let result = pump.send_ready();
    assert!(result.is_ok());
    assert!(
        pump.stall_full_since.is_none(),
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
    assert!(pump.stall_full_since.is_some());
}

mod pushpieces_retransmit_tests {
    use super::super::{SendError, pushpieces_retransmit_serial};
    use host_rt::transport::TransportError;

    #[test]
    fn recovers_after_transient_failures_within_budget() {
        let mut calls = 0u32;
        let res = pushpieces_retransmit_serial(0, 10, || {
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
        let res = pushpieces_retransmit_serial(0, 10, || {
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
        let res = pushpieces_retransmit_serial(0, 4, || {
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
    fn dead_transport_closed_fails_fast_no_retry() {
        let mut calls = 0u32;
        let res = pushpieces_retransmit_serial(0, 10, || {
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
        let res = pushpieces_retransmit_serial(0, 10, || {
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
        let res = pushpieces_retransmit_serial(0, 10, || {
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
