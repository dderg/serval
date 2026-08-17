//! Offline reproduction of the field -308 `PieceStartInPast` mechanism
//! (issues #405/#408, reproduced on the Trident bench 2026-08-16): at high
//! accel the planner emits ~100 µs pieces, so the MCU's per-axis piece ring
//! holds only `ring_depth x piece_duration` of real time. The pump can send
//! only into ring room, and room opens only via heartbeat retirement credit —
//! so the send margin is ring-bound, and a transport/heartbeat stall longer
//! than the ring's time depth puts the head piece into the MCU's past.

use std::collections::BTreeMap;
use std::sync::atomic::AtomicU64;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use runtime::piece_ring::PieceEntry;

use super::margin::front_margin_secs;
use super::pump_loop::Pump;
use super::*;

const FREQ: f64 = 1e6;
const PIECE_TICKS: u64 = 100;
const PIECE_SECS: f64 = PIECE_TICKS as f64 / FREQ;
const RING_DEPTH: u32 = 32;
const RING_TIME_DEPTH_SECS: f64 = RING_DEPTH as f64 * PIECE_SECS;
const BASE_TICKS: u64 = 10_000;
const HEARTBEAT_TICKS: u64 = 1_000;
const HARDWARE_PAST_GUARD_SECS: f64 = 500e-6;

#[derive(Clone)]
struct FrameRecordingSink {
    sends: Arc<Mutex<Vec<(u8, u64, u64)>>>,
}

impl FrameRecordingSink {
    fn new() -> Self {
        Self {
            sends: Arc::new(Mutex::new(Vec::new())),
        }
    }
    fn recorded(&self) -> Vec<(u8, u64, u64)> {
        self.sends.lock().unwrap().clone()
    }
}

impl PieceSink for FrameRecordingSink {
    fn send_frame(
        &self,
        _key: AxisKey,
        _pieces: &[PieceEntry],
        _start_slot: u16,
        _new_head: u32,
        _room: u32,
    ) -> Result<i32, SendError> {
        unreachable!("send_mcu_frames is overridden");
    }

    fn send_mcu_frames(&self, _mcu_id: u32, frames: &[AxisFrame]) -> Result<(), SendError> {
        let mut sends = self.sends.lock().unwrap();
        for f in frames {
            if let Some(front) = f.pieces.first() {
                sends.push((f.axis, front.start_time, f.guard_mcu_clock));
            }
        }
        Ok(())
    }
}

fn dense_piece(index: u64) -> (PieceEntry, f64) {
    let start = BASE_TICKS + index * PIECE_TICKS;
    (
        PieceEntry {
            start_time: start,
            #[allow(clippy::cast_possible_truncation)]
            duration: PIECE_SECS as f32,
            coeff_count: 2,
            ..PieceEntry::zeroed()
        },
        start as f64 / FREQ,
    )
}

fn dense_pump(
    key: AxisKey,
    sink: FrameRecordingSink,
    clock: Arc<Mutex<u64>>,
) -> Pump<FrameRecordingSink> {
    Pump {
        queues: BTreeMap::new(),
        junctions: JunctionTracker::default(),
        cohort: None,
        halted: BTreeMap::new(),
        sink,
        callbacks: PumpCallbacks {
            mcu_clock_of: Box::new(move |_| Some((*clock.lock().unwrap(), FREQ))),
            ..PumpCallbacks::noop(RING_DEPTH)
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
        margins: super::margin::SendMarginTracker::new(),
        windows: std::collections::HashMap::new(),
        resume_epochs: std::collections::HashMap::new(),
    }
    .tap_enqueue(key)
}

trait TapEnqueue {
    fn tap_enqueue(self, key: AxisKey) -> Self;
}

impl TapEnqueue for Pump<FrameRecordingSink> {
    fn tap_enqueue(mut self, key: AxisKey) -> Self {
        self.enqueue(EnqueueMsg {
            epoch_freq: None,
            key,
            pieces: (0..500).map(dense_piece).collect(),
            epoch: crate::anchor::StreamEpoch::Continuation,
            lead_secs: MAX_LEAD_SECS,
            source_line: u32::MAX,
            batch_end: true,
        });
        self
    }
}

fn retired_at(clock_ticks: u64) -> u32 {
    if clock_ticks <= BASE_TICKS {
        return 0;
    }
    #[allow(clippy::cast_possible_truncation)]
    let retired = ((clock_ticks - BASE_TICKS) / PIECE_TICKS) as u32;
    retired.min(500)
}

fn heartbeat_at(pump: &mut Pump<FrameRecordingSink>, key: AxisKey, clock_ticks: u64) {
    let retired = retired_at(clock_ticks);
    let mut counts = vec![0u32; usize::from(key.axis) + 1];
    counts[usize::from(key.axis)] = retired;
    assert!(pump.handle_control_msg(PumpMsg::Heartbeat(HeartbeatMsg {
        mcu_id: key.mcu_id,
        consumed_counts: None,
        retired_counts: counts,
    })));
}

fn send_all_ready(pump: &mut Pump<FrameRecordingSink>) {
    assert_eq!(
        pump.send_ready_until(Instant::now() + Duration::from_secs(60))
            .map(|_| ()),
        Ok(())
    );
}

#[test]
fn front_margin_secs_signs() {
    assert_eq!(front_margin_secs(1_500, 1_000, 1e6), 0.0005);
    assert_eq!(front_margin_secs(1_000, 1_500, 1e6), -0.0005);
}

#[test]
fn send_margin_is_ring_bound_not_lead_bound() {
    let key = AxisKey { mcu_id: 1, axis: 0 };
    let sink = FrameRecordingSink::new();
    let clock = Arc::new(Mutex::new(0u64));
    let mut pump = dense_pump(key, sink.clone(), clock.clone());

    let mut now = 0u64;
    while now < BASE_TICKS + 30_000 {
        send_all_ready(&mut pump);
        now += HEARTBEAT_TICKS;
        *clock.lock().unwrap() = now;
        heartbeat_at(&mut pump, key, now);
    }

    let steady: Vec<_> = sink
        .recorded()
        .into_iter()
        .filter(|&(_, _, guard_clock)| guard_clock > BASE_TICKS + 5_000)
        .collect();
    assert!(
        steady.len() > 20,
        "dense stream must keep sending in steady state, got {} sends",
        steady.len()
    );
    let heartbeat_slack = HEARTBEAT_TICKS as f64 / FREQ;
    for (axis, start, guard_clock) in steady {
        let margin = front_margin_secs(start, guard_clock, FREQ);
        assert!(
            margin <= RING_TIME_DEPTH_SECS + heartbeat_slack,
            "axis {axis}: send margin {margin}s exceeds the ring time depth \
             {RING_TIME_DEPTH_SECS}s + heartbeat slack — margin must be ring-bound, \
             not lead-bound (lead is {MAX_LEAD_SECS}s with 50 ms of pieces staged)"
        );
        assert!(
            margin >= 0.0,
            "axis {axis}: steady-state send margin {margin}s went negative with heartbeats \
             flowing every {heartbeat_slack}s"
        );
    }
    let staged_left = pump.queues[&key].pieces.len();
    assert!(
        staged_left > 100,
        "the host must still hold a deep staged queue while margins stay ring-bound, \
         got {staged_left}"
    );
}

#[test]
fn heartbeat_stall_longer_than_ring_depth_puts_head_piece_in_the_past() {
    let key = AxisKey { mcu_id: 1, axis: 0 };
    let sink = FrameRecordingSink::new();
    let clock = Arc::new(Mutex::new(0u64));
    let mut pump = dense_pump(key, sink.clone(), clock.clone());

    let mut now = 0u64;
    while now < BASE_TICKS + 10_000 {
        send_all_ready(&mut pump);
        now += HEARTBEAT_TICKS;
        *clock.lock().unwrap() = now;
        heartbeat_at(&mut pump, key, now);
    }
    send_all_ready(&mut pump);
    let sends_before_stall = sink.recorded().len();

    let stall_ticks = 10_000;
    now += stall_ticks;
    *clock.lock().unwrap() = now;
    send_all_ready(&mut pump);

    assert_eq!(
        sink.recorded().len(),
        sends_before_stall,
        "with no heartbeat credit the ring is full and the pump must not send"
    );

    let q = &pump.queues[&key];
    assert_eq!(
        q.room(),
        0,
        "in-flight pieces fill the ring during the stall"
    );
    let (head, _) = q.pieces.front().expect("staged pieces remain");
    let head_margin = front_margin_secs(head.start_time, now, FREQ);
    assert!(
        head_margin
            < -(stall_ticks as f64 / FREQ - RING_TIME_DEPTH_SECS - HEARTBEAT_TICKS as f64 / FREQ),
        "a credit stall of {}s against a {}s ring must leave the head piece deep in the \
         MCU's past, got margin {head_margin}s",
        stall_ticks as f64 / FREQ,
        RING_TIME_DEPTH_SECS,
    );
    assert!(
        head_margin < -HARDWARE_PAST_GUARD_SECS,
        "the head piece is beyond the hardware guard ({HARDWARE_PAST_GUARD_SECS}s): the next \
         send after the late credit arrives aborts with -308 PieceStartInPast"
    );
}
