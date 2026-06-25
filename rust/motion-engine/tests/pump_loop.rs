use std::sync::atomic::AtomicU64;
use std::sync::{Arc, Mutex};

use _motion_engine::pump::{
    AxisFrame, AxisKey, DripArm, EnqueueMsg, HeartbeatMsg, PieceSink, PumpMsg, SendError, run_pump,
};
use crossbeam_channel::{Receiver, TrySendError, unbounded};
use runtime::piece_ring::PieceEntry;

struct RecordingSink(Arc<Mutex<Vec<(AxisKey, usize)>>>);
impl PieceSink for RecordingSink {
    fn send_frame(
        &self,
        key: AxisKey,
        pieces: &[PieceEntry],
        _start_slot: u16,
        _new_head: u32,
        _room: u32,
    ) -> Result<i32, SendError> {
        self.0.lock().unwrap().push((key, pieces.len()));
        Ok(0)
    }
}

/// Records each bundled MCU transaction as `(mcu_id, axes-in-the-bundle)` so a
/// test can assert that same-MCU axes go out together rather than one
/// round-trip per axis.
struct BundleSink(Arc<Mutex<Vec<(u32, Vec<u8>)>>>);
impl PieceSink for BundleSink {
    fn send_frame(
        &self,
        _key: AxisKey,
        _pieces: &[PieceEntry],
        _start_slot: u16,
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

#[test]
fn pump_stalls_on_ring_full_resumes_on_heartbeat() {
    let rec = Arc::new(Mutex::new(Vec::new()));
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();
    let depth = |_k: AxisKey| 2u32;
    let sink = RecordingSink(rec.clone());
    let handle = std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            sink,
            depth,
            |_| None,
            |_| {},
            |_, _| {},
            |_| {},
            Arc::new(AtomicU64::new(0)),
            Arc::new(AtomicU64::new(0)),
        )
    });

    data.send(EnqueueMsg {
        key: AxisKey { mcu_id: 1, axis: 0 },
        pieces: vec![p(0), p(1)],
        fresh_stream: true,
        lead_secs: _motion_engine::pump::MAX_LEAD_SECS,
        source_line: u32::MAX,
        generation: 0,
        brake_tail: vec![],
    })
    .unwrap();
    data.send(EnqueueMsg {
        key: AxisKey { mcu_id: 1, axis: 0 },
        pieces: vec![p(2)],
        fresh_stream: false,
        lead_secs: _motion_engine::pump::MAX_LEAD_SECS,
        source_line: u32::MAX,
        generation: 0,
        brake_tail: vec![],
    })
    .unwrap();
    std::thread::sleep(std::time::Duration::from_millis(50));
    assert_eq!(
        rec.lock().unwrap().len(),
        1,
        "first frame (2 pieces) sent, third stalled"
    );
    assert_eq!(rec.lock().unwrap()[0], (AxisKey { mcu_id: 1, axis: 0 }, 2));

    ctl.send(PumpMsg::Heartbeat(HeartbeatMsg {
        mcu_id: 1,
        retired_counts: vec![2],
    }))
    .unwrap();
    std::thread::sleep(std::time::Duration::from_millis(50));
    assert_eq!(rec.lock().unwrap().len(), 2);
    assert_eq!(rec.lock().unwrap()[1], (AxisKey { mcu_id: 1, axis: 0 }, 1));

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
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
    control_rx: Receiver<PumpMsg>,
    data_rx: Receiver<EnqueueMsg>,
    rec: Arc<Mutex<Vec<(AxisKey, usize)>>>,
) -> std::thread::JoinHandle<()> {
    std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            RecordingSink(rec),
            |_k| 64u32,
            |_mcu| Some((0u64, 1e6_f64)),
            |_| {},
            |_, _| {},
            |_| {},
            Arc::new(AtomicU64::new(0)),
            Arc::new(AtomicU64::new(0)),
        )
    })
}

#[test]
fn continuous_junction_position_passes() {
    let rec = Arc::new(Mutex::new(Vec::new()));
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();
    let handle = run_pump_with_clock(control_rx, data_rx, rec.clone());

    let key = AxisKey { mcu_id: 1, axis: 0 };
    data.send(EnqueueMsg {
        key,
        pieces: vec![piece_at(0, 0.0, 10.0, 12.5)],
        fresh_stream: true,
        lead_secs: _motion_engine::pump::MAX_LEAD_SECS,
        source_line: u32::MAX,
        generation: 0,
        brake_tail: vec![],
    })
    .unwrap();
    data.send(EnqueueMsg {
        key,
        pieces: vec![piece_at(2000, 0.002, 12.5, 15.0)],
        fresh_stream: false,
        lead_secs: _motion_engine::pump::MAX_LEAD_SECS,
        source_line: u32::MAX,
        generation: 0,
        brake_tail: vec![],
    })
    .unwrap();
    std::thread::sleep(std::time::Duration::from_millis(50));
    let sent_pieces: usize = rec.lock().unwrap().iter().map(|(_, n)| n).sum();
    assert_eq!(sent_pieces, 2);

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
        key,
        pieces: vec![piece_at(0, 0.0, 10.0, 12.5)],
        fresh_stream: true,
        lead_secs: _motion_engine::pump::MAX_LEAD_SECS,
        source_line: u32::MAX,
        generation: 0,
        brake_tail: vec![],
    })
    .unwrap();
    data.send(EnqueueMsg {
        key,
        pieces: vec![piece_at(2000, 0.002, 12.8, 15.0)],
        fresh_stream: false,
        lead_secs: _motion_engine::pump::MAX_LEAD_SECS,
        source_line: u32::MAX,
        generation: 0,
        brake_tail: vec![],
    })
    .unwrap();

    assert!(
        handle.join().is_err(),
        "0.3mm junction position jump must panic the pump"
    );
}

#[test]
fn fresh_stream_resets_junction_position_baseline() {
    let rec = Arc::new(Mutex::new(Vec::new()));
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();
    let handle = run_pump_with_clock(control_rx, data_rx, rec.clone());

    let key = AxisKey { mcu_id: 1, axis: 0 };
    data.send(EnqueueMsg {
        key,
        pieces: vec![piece_at(0, 0.0, 10.0, 12.5)],
        fresh_stream: true,
        lead_secs: _motion_engine::pump::MAX_LEAD_SECS,
        source_line: u32::MAX,
        generation: 0,
        brake_tail: vec![],
    })
    .unwrap();
    data.send(EnqueueMsg {
        key,
        pieces: vec![piece_at(2000, 0.002, 50.0, 55.0)],
        fresh_stream: true,
        lead_secs: _motion_engine::pump::MAX_LEAD_SECS,
        source_line: u32::MAX,
        generation: 0,
        brake_tail: vec![],
    })
    .unwrap();
    std::thread::sleep(std::time::Duration::from_millis(50));
    let sent_pieces: usize = rec.lock().unwrap().iter().map(|(_, n)| n).sum();
    assert_eq!(sent_pieces, 2);

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}

#[test]
fn bundles_same_mcu_axes_into_one_transaction() {
    let rec = Arc::new(Mutex::new(Vec::new()));
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();
    let depth = |_k: AxisKey| 8u32;
    let sink = BundleSink(rec.clone());
    let handle = std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            sink,
            depth,
            |_| None,
            |_| {},
            |_, _| {},
            |_| {},
            Arc::new(AtomicU64::new(0)),
            Arc::new(AtomicU64::new(0)),
        )
    });

    // Three axes on the same MCU, each with a piece eligible to ship now
    // (mcu_clock_of returns None => no horizon gate).
    for axis in 0..3u8 {
        data.send(EnqueueMsg {
            key: AxisKey { mcu_id: 1, axis },
            pieces: vec![p(0)],
            fresh_stream: axis == 0,
            lead_secs: _motion_engine::pump::MAX_LEAD_SECS,
            source_line: u32::MAX,
            generation: 0,
            brake_tail: vec![],
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
    let depth = |_k: AxisKey| 4u32;
    let sink = RecordingSink(rec.clone());
    let handle = std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            sink,
            depth,
            |_| None,
            |_| {},
            |_, _| {},
            |_| {},
            Arc::new(AtomicU64::new(0)),
            Arc::new(AtomicU64::new(0)),
        )
    });

    let key = AxisKey { mcu_id: 1, axis: 0 };
    let mut accepted = 0u32;
    let mut hit_full = false;
    let flood = 24000u64; // comfortably above PUMP_INTAKE_BACKLOG_CAP
    for i in 0..flood {
        match data.try_send(EnqueueMsg {
            key,
            pieces: vec![p(i)],
            fresh_stream: i == 0,
            lead_secs: _motion_engine::pump::MAX_LEAD_SECS,
            source_line: u32::MAX,
            generation: 0,
            brake_tail: vec![],
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

    // Retirement (<= pushed; a larger value wraps room() to zero) lets the pump
    // push from its backlog and pull again, draining the channel.
    ctl.send(PumpMsg::Heartbeat(HeartbeatMsg {
        mcu_id: 1,
        retired_counts: vec![4],
    }))
    .unwrap();
    std::thread::sleep(std::time::Duration::from_millis(30));
    assert!(
        data.try_send(EnqueueMsg {
            key,
            pieces: vec![p(9999)],
            fresh_stream: false,
            lead_secs: _motion_engine::pump::MAX_LEAD_SECS,
            source_line: u32::MAX,
            generation: 0,
            brake_tail: vec![],
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
    // starves axes whose pieces arrive after it on the shared channel — this hung
    // the homing drip cohort (idle axes got zero pieces, floor pinned at 0).
    // Intake is bounded by TOTAL backlog, so a full axis A must not stop the pump
    // from feeding axis B.
    let rec = Arc::new(Mutex::new(Vec::new()));
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();
    let key_a = AxisKey { mcu_id: 1, axis: 0 };
    let key_b = AxisKey { mcu_id: 1, axis: 1 };
    let depth = move |k: AxisKey| if k == key_a { 2u32 } else { 64u32 };
    let sink = RecordingSink(rec.clone());
    let handle = std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            sink,
            depth,
            |_| None,
            |_| {},
            |_, _| {},
            |_| {},
            Arc::new(AtomicU64::new(0)),
            Arc::new(AtomicU64::new(0)),
        )
    });

    // Axis A overruns its depth-2 ring with no retirement (stays full), then
    // axis B's pieces arrive behind A's on the same channel.
    for i in 0..8u64 {
        data.send(EnqueueMsg {
            key: key_a,
            pieces: vec![p(i)],
            fresh_stream: i == 0,
            lead_secs: _motion_engine::pump::MAX_LEAD_SECS,
            source_line: u32::MAX,
            generation: 0,
            brake_tail: vec![],
        })
        .unwrap();
    }
    for i in 0..4u64 {
        data.send(EnqueueMsg {
            key: key_b,
            pieces: vec![p(100 + i)],
            fresh_stream: i == 0,
            lead_secs: _motion_engine::pump::MAX_LEAD_SECS,
            source_line: u32::MAX,
            generation: 0,
            brake_tail: vec![],
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
fn drip_cohort_intake_bypasses_cap_and_feeds_all_participants() {
    // Regression (homing Z stall): during a drip cohort a homing axis lowers a
    // burst larger than the intake cap. The pump must keep draining so the other
    // participants — queued behind it on the shared channel — still get fed;
    // otherwise the cohort floor (min executed) never leaves 0 and homing stalls.
    let rec = Arc::new(Mutex::new(Vec::new()));
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();
    let key_a = AxisKey { mcu_id: 1, axis: 0 };
    let key_b = AxisKey { mcu_id: 1, axis: 1 };
    let depth = move |k: AxisKey| if k == key_a { 4u32 } else { 64u32 };
    let sink = RecordingSink(rec.clone());
    let handle = std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            sink,
            depth,
            |_mcu| Some((0u64, 1e6_f64)),
            |_| {},
            |_, _| {},
            |_| {},
            Arc::new(AtomicU64::new(0)),
            Arc::new(AtomicU64::new(0)),
        )
    });

    ctl.send(PumpMsg::DripArm(DripArm {
        cohort: 1,
        participants: vec![key_a, key_b],
        timeout: std::time::Duration::from_secs(5),
    }))
    .unwrap();

    // Axis A lowers a burst far larger than PUMP_INTAKE_BACKLOG_CAP, with a
    // depth-4 ring and no retirement so its backlog piles up over the cap.
    for i in 0..2500u64 {
        data.send(EnqueueMsg {
            key: key_a,
            pieces: vec![piece_at(i, i as f64, 0.0, 0.0)],
            fresh_stream: i == 0,
            lead_secs: _motion_engine::pump::DRIP_WINDOW_SECS,
            source_line: u32::MAX,
            generation: 0,
            brake_tail: vec![],
        })
        .unwrap();
    }
    for i in 0..4u64 {
        data.send(EnqueueMsg {
            key: key_b,
            pieces: vec![piece_at(i, i as f64, 0.0, 0.0)],
            fresh_stream: i == 0,
            lead_secs: _motion_engine::pump::DRIP_WINDOW_SECS,
            source_line: u32::MAX,
            generation: 0,
            brake_tail: vec![],
        })
        .unwrap();
    }
    std::thread::sleep(std::time::Duration::from_millis(80));

    let b_sent: usize = rec
        .lock()
        .unwrap()
        .iter()
        .filter(|(k, _)| *k == key_b)
        .map(|(_, n)| n)
        .sum();
    assert!(
        b_sent > 0,
        "cohort participant B must be fed despite A's over-cap burst (drip bypasses the intake cap)"
    );

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}

/// End-to-end: a held brake tail is promoted and dispatched to the MCU when the
/// committed frontier drains toward the playhead, and the brake generation is
/// bumped so the planner can re-anchor. The realtime path, not the pure decision.
#[test]
fn pump_brakes_to_rest_when_finals_starve() {
    use std::sync::atomic::Ordering;

    struct StartSink(Arc<Mutex<Vec<u64>>>);
    impl PieceSink for StartSink {
        fn send_frame(
            &self,
            _k: AxisKey,
            pieces: &[PieceEntry],
            _s: u16,
            _n: u32,
            _r: u32,
        ) -> Result<i32, SendError> {
            let mut g = self.0.lock().unwrap();
            g.extend(pieces.iter().map(|p| p.start_time));
            Ok(0)
        }
    }

    let sent = Arc::new(Mutex::new(Vec::<u64>::new()));
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();
    let playhead = Arc::new(AtomicU64::new(0));
    let brake_gen = Arc::new(AtomicU64::new(0));
    let freq = 1_000_000.0_f64; // 1 MHz: ticks == microseconds; watermark = 50_000 ticks.

    let sink = StartSink(sent.clone());
    let playhead_clk = Arc::clone(&playhead);
    let brake_gen_pump = Arc::clone(&brake_gen);
    let handle = std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            sink,
            |_k| 256u32,
            move |_mcu| Some((playhead_clk.load(Ordering::Acquire), freq)),
            |_| {},
            |_, _| {},
            |_| {},
            Arc::new(AtomicU64::new(0)),
            brake_gen_pump,
        )
    });

    // Final at t=1.0s; provisional brake-to-rest at 1.05s and 1.10s (ticks @ 1 MHz).
    data.send(EnqueueMsg {
        key: AxisKey { mcu_id: 1, axis: 0 },
        pieces: vec![p(1_000_000)],
        fresh_stream: true,
        lead_secs: _motion_engine::pump::MAX_LEAD_SECS,
        source_line: u32::MAX,
        generation: 0,
        brake_tail: vec![p(1_050_000), p(1_100_000)],
    })
    .unwrap();

    // Frontier (1e6) is far ahead of the playhead (0): the final flushes, the
    // brake stays held in reserve, no brake event.
    std::thread::sleep(std::time::Duration::from_millis(60));
    assert_eq!(
        *sent.lock().unwrap(),
        vec![1_000_000],
        "only the final piece sent; brake held in reserve"
    );
    assert_eq!(
        brake_gen.load(Ordering::Acquire),
        0,
        "no brake while lead is healthy"
    );

    // Starve: advance the playhead to within the flush watermark (40k < 50k ticks)
    // of the frontier, then wake the pump with a heartbeat.
    playhead.store(960_000, Ordering::Release);
    ctl.send(PumpMsg::Heartbeat(HeartbeatMsg {
        mcu_id: 1,
        retired_counts: vec![1],
    }))
    .unwrap();
    std::thread::sleep(std::time::Duration::from_millis(60));

    assert_eq!(
        brake_gen.load(Ordering::Acquire),
        1,
        "toolhead starved -> brake generation bumped once"
    );
    let got = sent.lock().unwrap().clone();
    assert!(
        got.contains(&1_050_000) && got.contains(&1_100_000),
        "brake-to-rest pieces dispatched on starvation: {got:?}"
    );

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}

/// Edge D: once the pump has braked (generation 1), a forward dispatch tagged with
/// the pre-brake generation 0 is dropped — it continues past a frontier the
/// machine has left — while a dispatch at the current generation is accepted.
#[test]
fn pump_fences_stale_forward_dispatch_after_brake() {
    use std::sync::atomic::Ordering;

    struct RecSink(Arc<Mutex<Vec<u64>>>);
    impl PieceSink for RecSink {
        fn send_frame(
            &self,
            _k: AxisKey,
            pieces: &[PieceEntry],
            _s: u16,
            _n: u32,
            _r: u32,
        ) -> Result<i32, SendError> {
            self.0
                .lock()
                .unwrap()
                .extend(pieces.iter().map(|p| p.start_time));
            Ok(0)
        }
    }

    let sent = Arc::new(Mutex::new(Vec::<u64>::new()));
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();
    let brake_gen = Arc::new(AtomicU64::new(0));
    let brake_gen_pump = Arc::clone(&brake_gen);
    let sink = RecSink(sent.clone());
    let handle = std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            sink,
            |_k| 256u32,
            |_mcu| Some((0u64, 1_000_000.0_f64)),
            |_| {},
            |_, _| {},
            |_| {},
            Arc::new(AtomicU64::new(0)),
            brake_gen_pump,
        )
    });

    // The pump has braked: it is now on generation 1.
    brake_gen.store(1, Ordering::Release);

    // A stale forward dispatch from before the brake (generation 0) must be dropped.
    data.send(EnqueueMsg {
        key: AxisKey { mcu_id: 1, axis: 0 },
        pieces: vec![p(500_000)],
        fresh_stream: false,
        lead_secs: _motion_engine::pump::MAX_LEAD_SECS,
        source_line: u32::MAX,
        generation: 0,
        brake_tail: vec![],
    })
    .unwrap();
    std::thread::sleep(std::time::Duration::from_millis(60));
    assert!(
        sent.lock().unwrap().is_empty(),
        "stale generation-0 dispatch must be fenced, not sent: {:?}",
        sent.lock().unwrap()
    );

    // A current dispatch (generation 1) is accepted and reaches the MCU.
    data.send(EnqueueMsg {
        key: AxisKey { mcu_id: 1, axis: 0 },
        pieces: vec![p(600_000)],
        fresh_stream: true,
        lead_secs: _motion_engine::pump::MAX_LEAD_SECS,
        source_line: u32::MAX,
        generation: 1,
        brake_tail: vec![],
    })
    .unwrap();
    std::thread::sleep(std::time::Duration::from_millis(60));
    assert_eq!(
        *sent.lock().unwrap(),
        vec![600_000],
        "current-generation dispatch accepted after the fence"
    );

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}
