use std::sync::mpsc;
use std::sync::{Arc, Mutex};

use _motion_engine::pump::{
    AxisKey, EnqueueMsg, HeartbeatMsg, PieceSink, PumpMsg, SendError, run_pump,
};
use runtime::piece_ring::PieceEntry;

struct RecordingSink(Arc<Mutex<Vec<(AxisKey, usize)>>>);
impl PieceSink for RecordingSink {
    fn send_frame(
        &self,
        key: AxisKey,
        pieces: &[PieceEntry],
        _start_slot: u16,
        _new_head: u32,
    ) -> Result<i32, SendError> {
        self.0.lock().unwrap().push((key, pieces.len()));
        Ok(0)
    }
}

fn p(start: u64) -> (PieceEntry, f64) {
    (
        PieceEntry {
            start_time: start,
            coeffs: [0.0; 4],
            duration: 0.001,
            motor_mask: 0,
            _reserved: [0; 3],
        },
        start as f64,
    )
}

fn timed_piece(host: f64, duration: f32) -> (PieceEntry, f64) {
    (
        PieceEntry {
            start_time: (host * 1_000_000.0) as u64,
            coeffs: [0.0; 4],
            duration,
            motor_mask: 0,
            _reserved: [0; 3],
        },
        host,
    )
}

#[test]
fn pump_stalls_on_ring_full_resumes_on_heartbeat() {
    let rec = Arc::new(Mutex::new(Vec::new()));
    let (tx, rx) = mpsc::channel();
    let depth = |_k: AxisKey| 2u32;
    let sink = RecordingSink(rec.clone());
    let handle = std::thread::spawn(move || {
        run_pump(
            rx,
            sink,
            depth,
            |_| None,
            |_| {},
            |_, _| {},
            |_| {},
            |_, _| {},
        )
    });

    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key: AxisKey { mcu_id: 1, axis: 0 },
        pieces: vec![p(0), p(1)],
        fresh_stream: true,
        lead_secs: _motion_engine::pump::MAX_LEAD_SECS,
    }))
    .unwrap();
    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key: AxisKey { mcu_id: 1, axis: 0 },
        pieces: vec![p(2)],
        fresh_stream: false,
        lead_secs: _motion_engine::pump::MAX_LEAD_SECS,
    }))
    .unwrap();
    std::thread::sleep(std::time::Duration::from_millis(50));
    assert_eq!(
        rec.lock().unwrap().len(),
        1,
        "first frame (2 pieces) sent, third stalled"
    );
    assert_eq!(rec.lock().unwrap()[0], (AxisKey { mcu_id: 1, axis: 0 }, 2));

    tx.send(PumpMsg::Heartbeat(HeartbeatMsg {
        mcu_id: 1,
        retired_counts: vec![2],
    }))
    .unwrap();
    std::thread::sleep(std::time::Duration::from_millis(50));
    assert_eq!(rec.lock().unwrap().len(), 2);
    assert_eq!(rec.lock().unwrap()[1], (AxisKey { mcu_id: 1, axis: 0 }, 1));

    tx.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}

#[test]
fn pump_publishes_dispatch_room_on_enqueue_and_retire() {
    let rec = Arc::new(Mutex::new(Vec::new()));
    let rooms = Arc::new(Mutex::new(Vec::new()));
    let rooms_for_pump = Arc::clone(&rooms);
    let (tx, rx) = mpsc::channel();
    let sink = RecordingSink(rec);
    let handle = std::thread::spawn(move || {
        run_pump(
            rx,
            sink,
            |_k| 8u32,
            |_| None,
            |_| {},
            |_, _| {},
            |_| {},
            move |key, room| {
                rooms_for_pump.lock().unwrap().push((key, room));
            },
        )
    });

    let x = AxisKey { mcu_id: 1, axis: 0 };
    let y = AxisKey { mcu_id: 1, axis: 1 };
    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key: x,
        pieces: vec![timed_piece(0.0, 0.25), timed_piece(0.25, 0.25)],
        fresh_stream: true,
        lead_secs: _motion_engine::pump::MAX_LEAD_SECS,
    }))
    .unwrap();
    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key: y,
        pieces: vec![timed_piece(0.0, 0.75)],
        fresh_stream: true,
        lead_secs: _motion_engine::pump::MAX_LEAD_SECS,
    }))
    .unwrap();

    std::thread::sleep(std::time::Duration::from_millis(50));
    tx.send(PumpMsg::Heartbeat(HeartbeatMsg {
        mcu_id: 1,
        retired_counts: vec![1, 1],
    }))
    .unwrap();
    tx.send(PumpMsg::Heartbeat(HeartbeatMsg {
        mcu_id: 1,
        retired_counts: vec![2, 1],
    }))
    .unwrap();

    // ring_depth = 8. Room = depth - (enqueued - retired).
    for _ in 0..20 {
        let snapshot = rooms.lock().unwrap().clone();
        if snapshot.len() >= 3 {
            // enqueue x (2 pieces): 8 - 2 = 6 room left.
            assert_eq!(snapshot[0], (x, 6.0));
            // enqueue y (1 piece): 8 - 1 = 7 room left.
            assert_eq!(snapshot[1], (y, 7.0));
            // x retires 1 of 2: outstanding 1 -> 8 - 1 = 7 room freed.
            assert_eq!(snapshot[2], (x, 7.0));
            tx.send(PumpMsg::Shutdown).unwrap();
            handle.join().unwrap();
            return;
        }
        std::thread::sleep(std::time::Duration::from_millis(10));
    }

    tx.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
    panic!("pump did not publish the expected dispatch-room credits");
}

#[test]
fn flush_resyncs_enqueued_so_dispatch_room_recovers_after_abort() {
    let rec = Arc::new(Mutex::new(Vec::new()));
    let rooms = Arc::new(Mutex::new(Vec::new()));
    let rooms_for_pump = Arc::clone(&rooms);
    let (tx, rx) = mpsc::channel();
    let sink = RecordingSink(rec);
    let handle = std::thread::spawn(move || {
        run_pump(
            rx,
            sink,
            |_k| 2u32,
            |_| None,
            |_| {},
            |_, _| {},
            |_| {},
            move |key, room| {
                rooms_for_pump.lock().unwrap().push((key, room));
            },
        )
    });

    // ring_depth = 2 but 5 pieces enqueued: 2 fit the ring, 3 stay backlogged.
    let x = AxisKey { mcu_id: 1, axis: 0 };
    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key: x,
        pieces: (0..5)
            .map(|i| timed_piece(f64::from(i) * 0.25, 0.25))
            .collect(),
        fresh_stream: true,
        lead_secs: _motion_engine::pump::MAX_LEAD_SECS,
    }))
    .unwrap();
    std::thread::sleep(std::time::Duration::from_millis(50));

    // A homing trip aborts: flush drops the backlog. The abandoned pieces must
    // stop counting as outstanding, otherwise dispatch_room stays pinned at 0.
    tx.send(PumpMsg::Flush(vec![x])).unwrap();
    // The MCU discards its ring: retired advances to what was pushed (2).
    tx.send(PumpMsg::Heartbeat(HeartbeatMsg {
        mcu_id: 1,
        retired_counts: vec![2],
    }))
    .unwrap();

    for _ in 0..30 {
        if rooms
            .lock()
            .unwrap()
            .last()
            .is_some_and(|&(_, room)| (room - 2.0).abs() < 1e-9)
        {
            tx.send(PumpMsg::Shutdown).unwrap();
            handle.join().unwrap();
            return;
        }
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
    tx.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
    panic!("dispatch room did not recover after the flush abandoned the backlog");
}

fn piece_at(start: u64, host: f64, start_pos: f32, end_pos: f32) -> (PieceEntry, f64) {
    (
        PieceEntry {
            start_time: start,
            coeffs: [start_pos, start_pos, end_pos, end_pos],
            duration: 0.001,
            motor_mask: 0,
            _reserved: [0; 3],
        },
        host,
    )
}

fn run_pump_with_clock(
    rx: mpsc::Receiver<PumpMsg>,
    rec: Arc<Mutex<Vec<(AxisKey, usize)>>>,
) -> std::thread::JoinHandle<()> {
    std::thread::spawn(move || {
        run_pump(
            rx,
            RecordingSink(rec),
            |_k| 64u32,
            |_mcu| Some((0u64, 1e6_f64)),
            |_| {},
            |_, _| {},
            |_| {},
            |_, _| {},
        )
    })
}

#[test]
fn continuous_junction_position_passes() {
    let rec = Arc::new(Mutex::new(Vec::new()));
    let (tx, rx) = mpsc::channel();
    let handle = run_pump_with_clock(rx, rec.clone());

    let key = AxisKey { mcu_id: 1, axis: 0 };
    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key,
        pieces: vec![piece_at(0, 0.0, 10.0, 12.5)],
        fresh_stream: true,
        lead_secs: _motion_engine::pump::MAX_LEAD_SECS,
    }))
    .unwrap();
    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key,
        pieces: vec![piece_at(2000, 0.002, 12.5, 15.0)],
        fresh_stream: false,
        lead_secs: _motion_engine::pump::MAX_LEAD_SECS,
    }))
    .unwrap();
    std::thread::sleep(std::time::Duration::from_millis(50));
    let sent_pieces: usize = rec.lock().unwrap().iter().map(|(_, n)| n).sum();
    assert_eq!(sent_pieces, 2);

    tx.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}

#[test]
fn junction_position_discontinuity_is_fatal() {
    let rec = Arc::new(Mutex::new(Vec::new()));
    let (tx, rx) = mpsc::channel();
    let handle = run_pump_with_clock(rx, rec.clone());

    let key = AxisKey { mcu_id: 1, axis: 0 };
    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key,
        pieces: vec![piece_at(0, 0.0, 10.0, 12.5)],
        fresh_stream: true,
        lead_secs: _motion_engine::pump::MAX_LEAD_SECS,
    }))
    .unwrap();
    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key,
        pieces: vec![piece_at(2000, 0.002, 12.8, 15.0)],
        fresh_stream: false,
        lead_secs: _motion_engine::pump::MAX_LEAD_SECS,
    }))
    .unwrap();

    assert!(
        handle.join().is_err(),
        "0.3mm junction position jump must panic the pump"
    );
}

#[test]
fn fresh_stream_resets_junction_position_baseline() {
    let rec = Arc::new(Mutex::new(Vec::new()));
    let (tx, rx) = mpsc::channel();
    let handle = run_pump_with_clock(rx, rec.clone());

    let key = AxisKey { mcu_id: 1, axis: 0 };
    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key,
        pieces: vec![piece_at(0, 0.0, 10.0, 12.5)],
        fresh_stream: true,
        lead_secs: _motion_engine::pump::MAX_LEAD_SECS,
    }))
    .unwrap();
    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key,
        pieces: vec![piece_at(2000, 0.002, 50.0, 55.0)],
        fresh_stream: true,
        lead_secs: _motion_engine::pump::MAX_LEAD_SECS,
    }))
    .unwrap();
    std::thread::sleep(std::time::Duration::from_millis(50));
    let sent_pieces: usize = rec.lock().unwrap().iter().map(|(_, n)| n).sum();
    assert_eq!(sent_pieces, 2);

    tx.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}
