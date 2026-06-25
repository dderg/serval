use super::*;
use crossbeam_channel::unbounded;
use std::sync::atomic::AtomicU64;
use std::sync::mpsc;
use std::sync::{Arc, Mutex};

fn make_enqueue(key: AxisKey, pieces: Vec<(PieceEntry, f64)>, fresh_stream: bool) -> EnqueueMsg {
    EnqueueMsg {
        key,
        pieces,
        fresh_stream,
        lead_secs: MAX_LEAD_SECS,
        source_line: u32::MAX,
        generation: 0,
        brake_tail: vec![],
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
    assert_eq!(q.room(), 5);
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
            coeffs: [0.0; 4],
            duration: 0.001,
            motor_mask: 0,
            _reserved: [0; 3],
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
            |_key| RING_DEPTH,
            |_mcu| None,
            |_| {},
            |_, _| {},
            |_| {},
            Arc::new(AtomicU64::new(0)),
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
    (
        PieceEntry {
            start_time: t,
            coeffs: [c0, c0, c3, c3],
            duration: 0.001,
            motor_mask: mask,
            _reserved: [0; 3],
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
            |_key| 8,
            |_mcu| Some((0u64, 180_000_000.0)),
            |_| {},
            |_, _| {},
            |_| {},
            Arc::new(AtomicU64::new(0)),
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
            |_key| 64,
            move |_mcu| *clock_pump.lock().unwrap(),
            |_| {},
            |_, _| {},
            |_| {},
            Arc::new(AtomicU64::new(0)),
            Arc::new(AtomicU64::new(0)),
        );
    });

    data.send(EnqueueMsg {
        key,
        pieces: (0u64..4)
            .map(|i| {
                (
                    PieceEntry {
                        start_time: gated_tick + i,
                        coeffs: [0.0; 4],
                        duration: 0.001,
                        motor_mask: 0,
                        _reserved: [0; 3],
                    },
                    (gated_tick + i) as f64,
                )
            })
            .collect(),
        fresh_stream: true,
        lead_secs,
        source_line: u32::MAX,
        generation: 0,
        brake_tail: vec![],
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
                start_time: 1,
                coeffs: [0.0; 4],
                duration: 0.001,
                motor_mask: 0,
                _reserved: [0; 3],
            },
            1.0,
        )],
        fresh_stream: false,
        lead_secs,
        source_line: u32::MAX,
        generation: 0,
        brake_tail: vec![],
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
            |_key| 64,
            move |_mcu| *clock_pump.lock().unwrap(),
            |_| {},
            move |_k: AxisKey, n: u32| {
                *abandoned_pump.lock().unwrap() += n;
            },
            |_| {},
            Arc::new(AtomicU64::new(0)),
            Arc::new(AtomicU64::new(0)),
        );
    });

    data.send(EnqueueMsg {
        key,
        pieces: (0u64..4)
            .map(|i| {
                (
                    PieceEntry {
                        start_time: gated_tick + i,
                        coeffs: [0.0; 4],
                        duration: 0.001,
                        motor_mask: 0,
                        _reserved: [0; 3],
                    },
                    (gated_tick + i) as f64,
                )
            })
            .collect(),
        fresh_stream: true,
        lead_secs,
        source_line: u32::MAX,
        generation: 0,
        brake_tail: vec![],
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
                start_time: 1,
                coeffs: [0.0; 4],
                duration: 0.001,
                motor_mask: 0,
                _reserved: [0; 3],
            },
            1.0,
        )],
        fresh_stream: false,
        lead_secs,
        source_line: u32::MAX,
        generation: 0,
        brake_tail: vec![],
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
            |_key| 64,
            |_mcu| None,
            |_| {},
            |_, _| {},
            |_| {},
            Arc::new(AtomicU64::new(0)),
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
            |_| 0,
            |_| None,
            |_| {},
            |_, _| {},
            |_| {},
            backlog_pump,
            Arc::new(AtomicU64::new(0)),
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
            |_| 8,
            |_| None,
            |_| {},
            |_, _| {},
            |_| {},
            Arc::new(AtomicU64::new(0)),
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
            |_key| 0,
            |_mcu| None,
            |_| {},
            |_, _| {},
            |_| {},
            backlog_thread,
            Arc::new(AtomicU64::new(0)),
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
            |_key| 8,
            |_mcu| None,
            |_| {},
            |_, _| {},
            |_| {},
            backlog_thread,
            Arc::new(AtomicU64::new(0)),
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
