pub use crate::types::AxisKey;
use crossbeam_channel::{Receiver, Select, TryRecvError};
use runtime::piece_ring::PieceEntry;
use std::collections::BTreeMap;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

mod drip;
mod junction;
mod sched;
mod wire_sink;

use drip::DripCohort;
use junction::check_junction_position_continuity;

pub use drip::{DRIP_WINDOW_SECS, DripArm};
pub use junction::{
    JUNCTION_POSITION_FATAL_MM, JUNCTION_POSITION_LOG_MM, JunctionSeam, JunctionTracker,
    junction_jumps,
};
pub use sched::{AxisFrame, AxisQueue, FramePlan, Schedule, append_pieces_merging_holds, schedule};
#[cfg(test)]
pub(crate) use wire_sink::pushpieces_retransmit_serial;
pub use wire_sink::{McuTransport, WireSink};

#[cfg(test)]
mod drip_tests;
#[cfg(test)]
mod sched_tests;
#[cfg(test)]
mod tests;

pub struct EnqueueMsg {
    pub key: AxisKey,
    pub pieces: Vec<(PieceEntry, f64)>,
    pub fresh_stream: bool,
    pub lead_secs: f64,
    pub source_line: u32,
}

/// Records each piece into the motion-history store at the moment it is
/// accepted by the MCU, so the store mirrors what the MCU can actually
/// execute. Recording at dispatch time instead would flood the ring with an
/// entire move up front — a long homing move evicts its own start before the
/// endstop trip is resolved against it.
pub struct HistoryRecorder {
    pub store: Arc<std::sync::Mutex<crate::motion_history::HistoryStore>>,
    pub nominal_freqs: Arc<std::sync::Mutex<std::collections::HashMap<u32, u32>>>,
}

impl HistoryRecorder {
    fn record(&self, key: AxisKey, piece: &PieceEntry, host_t: f64) {
        let nominal_freq = *self
            .nominal_freqs
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .get(&key.mcu_id)
            .unwrap_or_else(|| {
                panic!(
                    "no nominal clock frequency registered for mcu {} \
                     — set_nominal_clock_freq was not called before streaming",
                    key.mcu_id
                )
            });
        self.store.lock().unwrap_or_else(|p| p.into_inner()).record(
            key,
            piece,
            nominal_freq,
            host_t,
        );
    }
}

pub struct HeartbeatMsg {
    pub mcu_id: u32,
    pub retired_counts: Vec<u32>,
}

pub enum PumpMsg {
    Heartbeat(HeartbeatMsg),
    Flush(Vec<AxisKey>),
    DripArm(DripArm),
    DripDisarm(u64),
    Barrier(std::sync::mpsc::SyncSender<()>),
    Shutdown,
}

#[derive(Debug)]
pub enum SendError {
    Fatal(String),
    Transient(String),
}

impl SendError {
    fn retryable_mcu_reject(mcu_id: u32, result: i32) -> Self {
        Self::Transient(format!(
            "mcu {mcu_id} rejected PushPieces frame: result {result}"
        ))
    }
}

impl std::fmt::Display for SendError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Fatal(s) => write!(f, "fatal: {s}"),
            Self::Transient(s) => write!(f, "transient: {s}"),
        }
    }
}

pub trait PieceSink: Send {
    fn send_frame(
        &self,
        key: AxisKey,
        pieces: &[PieceEntry],
        start_slot: u16,
        new_head: u32,
        room: u32,
    ) -> Result<i32, SendError>;

    /// Deliver every axis frame destined for `mcu_id` as one bundled
    /// transaction. A whole bundle either lands or it doesn't — the caller
    /// commits the ring bookkeeping for all axes only on `Ok`, so a failed
    /// bundle re-sends byte-identical frames to the same ring slots.
    ///
    /// The default fans out to per-axis `send_frame`; a transport that can
    /// pack multiple axes into one round-trip overrides this to collapse the
    /// per-frame overhead that dominates dense-stream delivery.
    fn send_mcu_frames(&self, mcu_id: u32, frames: &[AxisFrame]) -> Result<(), SendError> {
        for f in frames {
            self.send_frame(
                AxisKey {
                    mcu_id,
                    axis: f.axis,
                },
                &f.pieces,
                f.start_slot,
                f.new_head,
                f.room,
            )?;
        }
        Ok(())
    }
}

// How far ahead of the MCU playhead the pump pushes pieces — the depth of the
// MCU-side buffer that absorbs host-side scheduling hiccups. A piece that lands
// past the playhead faults (PieceStartInPast → stream halt), so this must exceed
// the worst-case host stall between pump pushes. The per-axis ring (≈496 pieces)
// is the hard cap; for dense piece streams the pump stalls on a full ring well
// before this horizon, so raising it only deepens the buffer for sparse (long,
// slow) moves where stalls are otherwise most likely to slip a piece into the
// past.
pub const MAX_LEAD_SECS: f64 = 2.0;

// Bound on the planner→pump piece-data channel. When the pump stops pulling
// (ring full or at the lead horizon), the planner's dispatch send blocks once
// this many segment messages are queued — propagating backpressure to the input
// channel and the gcode reader. Host-side depth does not extend MCU lead (that
// is the ring's job), so this only decouples planner bursts from pump intake.
pub const PUMP_DATA_CHANNEL_CAP: usize = 1024;

// Bound on total host-side staged pieces across all axes. Intake stops here so
// the data channel backpressures the planner during streaming. It is a TOTAL,
// not per-axis (all axes interleave on one channel). A drip cohort BYPASSES it:
// a homing axis can lower into a burst larger than the cap, and stopping intake
// there would leave the other participants' messages unpulled behind it on the
// shared channel — starving the cohort floor and freezing the planner on the
// full channel. Drip is finite, so greedy draining is safe there.
//
// Sized ≈4× a typical MCU ring (an F407 ring holds ~1877 pieces), roughly
// 5–15 s of typical motion: the host staging buffer must be DEEPER than the
// MCU rings, or the pump throttles the planner before the frontier is deep
// enough to absorb host scheduling gaps — the playhead then overruns the
// committed end (anchor_underrun → drive fault).
const PUMP_INTAKE_BACKLOG_CAP: u64 = 8192;

const MAX_PER_FRAME: usize = 32;

// How long an axis ring may sit at room()==0 with `q.retired` frozen before the
// pump treats it as the MCU having stopped retiring pieces rather than a normal
// transient full-ring wait.
const RETIREMENT_STALL_FATAL: Duration = Duration::from_secs(10);

// A PushPieces bundle occupies the serial line for its whole wire length
// (~20 ms/KiB at 500 kbaud) while its front piece's arrival lead keeps
// draining, so the cap must be in bytes — variable-degree entries span
// 20..=48 B and a count cap is wrong at both ends. send_ready() loops until
// Idle, so this bounds per-transaction latency, not throughput.
const BUNDLE_WIRE_BYTE_BUDGET: usize = 1024;

fn wants_pieces(queues: &BTreeMap<AxisKey, AxisQueue>) -> bool {
    let staged: u64 = queues.values().map(|q| q.pieces.len() as u64).sum();
    staged < PUMP_INTAKE_BACKLOG_CAP
}

// Mirrors the MCU's MAX_START_IN_PAST_SECS: 500us over host-projection
// jitter on real hardware; in the simulator (MCU_SIM_SOCK_DIR set) the
// virtual clock races arbitrarily far ahead of the host projection, so
// the guard widens to the MCU's own mcu-sim grace instead of aborting on
// infrastructure jitter.
fn pump_past_guard_secs() -> f64 {
    static GUARD: std::sync::OnceLock<f64> = std::sync::OnceLock::new();
    *GUARD.get_or_init(|| {
        if std::env::var_os("MCU_SIM_SOCK_DIR").is_some() {
            10.0
        } else {
            500e-6
        }
    })
}

fn log_piece_submit(
    mcu_id: u32,
    axis: u8,
    freq: Option<f32>,
    piece: &PieceEntry,
    prev_end: Option<u64>,
) -> Option<u64> {
    let end_ticks: u64 = freq.map_or(0, |f| piece.end_time(f));
    let gap_ticks_in_frame: i64 = prev_end.map_or(0, |pe| piece.start_time as i64 - pe as i64);
    tracing::trace!(
        subsystem = "motion",
        event = "pump_piece_submit",
        mcu = mcu_id,
        axis = axis,
        start_time = piece.start_time,
        duration_s = piece.duration,
        end_ticks,
        gap_ticks_in_frame,
        motor_mask = piece.motor_mask,
        "[pump-submit] piece submitted to MCU \
         (gap_ticks_in_frame: 0=contiguous, <0=overlap, >0=gap)"
    );
    if freq.is_some() {
        Some(end_ticks)
    } else {
        prev_end
    }
}

struct Pump<S, F, C, A, O, D> {
    queues: BTreeMap<AxisKey, AxisQueue>,
    junctions: JunctionTracker,
    cohort: Option<DripCohort>,
    sink: S,
    ring_depth_of: F,
    mcu_clock_of: C,
    on_fatal_transport: A,
    on_abandon: O,
    on_drip_stall: D,
    history: Option<HistoryRecorder>,
    ledger: Arc<crate::drain::DrainLedger>,
    /// Barrier acks are held until the end of the loop iteration, after
    /// intake and the ledger publish — so a caller that receives the ack has
    /// a ledger covering everything it enqueued before sending the barrier.
    pending_barrier_acks: Vec<std::sync::mpsc::SyncSender<()>>,
    backlog: Arc<AtomicU64>,
    holding_ahead: bool,
    data_open: bool,
    last_stallfull_log: Option<Instant>,
    retirement_stall_fatal: Duration,
    stall_full_since: Option<(AxisKey, u32, Instant)>,
}

impl<S, F, C, A, O, D> Pump<S, F, C, A, O, D>
where
    S: PieceSink,
    F: Fn(AxisKey) -> u32,
    C: Fn(u32) -> Option<(u64, f64)>,
    A: Fn(AxisKey) + Send + 'static,
    O: Fn(AxisKey, u32),
    D: Fn(String) + Send,
{
    fn handle_control_msg(&mut self, msg: PumpMsg) -> bool {
        match msg {
            PumpMsg::Shutdown => return false,
            PumpMsg::Flush(keys) => {
                for key in keys {
                    if let Some(q) = self.queues.get_mut(&key) {
                        let dropped = q.pieces.len() as u32;
                        q.pieces.clear();
                        if dropped > 0 {
                            (self.on_abandon)(key, dropped);
                        }
                    }
                    self.junctions.forget(key);
                }
            }
            PumpMsg::Heartbeat(HeartbeatMsg {
                mcu_id,
                retired_counts,
            }) => {
                for (axis, &c) in retired_counts.iter().enumerate() {
                    let key = AxisKey {
                        mcu_id,
                        axis: axis as u8,
                    };
                    if let Some(q) = self.queues.get_mut(&key) {
                        q.retired = c;
                    }
                    if let Some(co) = &mut self.cohort {
                        if co.participants.contains(&key) {
                            let prev = co.last_retired.get(&key).copied().unwrap_or(0);
                            if c < prev {
                                (self.on_drip_stall)(format!(
                                    "drip cohort {}: retired regression on mcu{} axis{}: \
                                     was {prev} now {c} — MCU retired counter must not decrease",
                                    co.id, mcu_id, axis
                                ));
                                self.cohort = None;
                                break;
                            }
                            co.last_retired.insert(key, c);
                        }
                    }
                }
            }
            PumpMsg::DripArm(arm) => {
                let mut baseline = BTreeMap::new();
                let mut last_retired = BTreeMap::new();
                for &k in &arm.participants {
                    let retired = self.queues.get(&k).map_or(0, |q| q.retired);
                    baseline.insert(k, retired);
                    last_retired.insert(k, retired);
                }
                let step_deadline = Instant::now() + arm.timeout;
                self.cohort = Some(DripCohort {
                    id: arm.cohort,
                    participants: arm.participants.into_iter().collect(),
                    timeout: arm.timeout,
                    baseline,
                    last_retired,
                    step_deadline,
                    deadline_floor: 0,
                });
            }
            PumpMsg::DripDisarm(c) => {
                if self.cohort.as_ref().map_or(false, |co| co.id == c) {
                    self.cohort = None;
                }
            }
            PumpMsg::Barrier(ack) => {
                self.pending_barrier_acks.push(ack);
            }
        }
        true
    }

    fn enqueue(&mut self, msg: EnqueueMsg) {
        let EnqueueMsg {
            key,
            pieces,
            fresh_stream,
            lead_secs,
            source_line,
        } = msg;
        if let Some(co) = self.cohort.as_ref() {
            if !co.participants.contains(&key) {
                let id = co.id;
                (self.on_drip_stall)(format!(
                    "drip cohort {id}: enqueue for non-participant \
                     mcu{} axis{} during active cohort — homing must \
                     drip every axis",
                    key.mcu_id, key.axis
                ));
                self.cohort = None;
                return;
            }
        }
        if fresh_stream {
            self.junctions.forget(key);
        }
        if self.cohort.is_some() && !pieces.is_empty() {
            if let Some((ack_now, freq)) = (self.mcu_clock_of)(key.mcu_id) {
                let first_start = pieces[0].0.start_time;
                let produce_lead_us = (first_start as i64 - ack_now as i64) as f64 / freq * 1e6;
                let durs: Vec<f32> = pieces.iter().map(|p| p.0.duration).collect();
                let min_dur = durs.iter().copied().fold(f32::INFINITY, f32::min);
                let max_dur = durs.iter().copied().fold(0.0_f32, f32::max);
                let total: f32 = durs.iter().sum();
                tracing::warn!(
                    subsystem = "motion",
                    event = "drip_enqueue_lead",
                    mcu = key.mcu_id,
                    axis = key.axis,
                    n = pieces.len(),
                    produce_lead_us,
                    min_dur_us = min_dur * 1e6,
                    max_dur_us = max_dur * 1e6,
                    total_secs = total,
                    "[drip-diag] pieces reached pump with this much lead before their start"
                );
            }
        }
        if !pieces.is_empty() {
            if let Some((_ack_now, freq)) = (self.mcu_clock_of)(key.mcu_id) {
                if let Some(seam) = self.junctions.observe(key, &pieces, source_line, freq) {
                    check_junction_position_continuity(&seam);
                    let (tick_jump_us, host_jump_us) = junction_jumps(
                        seam.first_start_ticks,
                        seam.next_start_host,
                        seam.prev_end_ticks,
                        seam.prev_end_host,
                        freq,
                    );
                    let anomalous =
                        tick_jump_us < -50.0 || (tick_jump_us - host_jump_us).abs() > 50.0;
                    if fresh_stream || !anomalous {
                        tracing::debug!(
                            subsystem = "motion",
                            event = "junction_jump",
                            key = ?seam.key,
                            tick_jump_us,
                            host_jump_us,
                            fresh = fresh_stream,
                            "[junction] jump"
                        );
                    } else {
                        let reason = if tick_jump_us < -50.0 {
                            "overlap_risk"
                        } else {
                            "projection_divergence"
                        };
                        tracing::warn!(
                            subsystem = "motion",
                            event = "junction_jump_anomalous",
                            key = ?seam.key,
                            tick_jump_us,
                            host_jump_us,
                            fresh = fresh_stream,
                            reason,
                            prev_source_line = seam.prev_source_line,
                            next_source_line = source_line,
                            "[junction] anomalous jump"
                        );
                    }
                }
            }
        }
        let ring_depth = (self.ring_depth_of)(key);
        // Hold merging is off during drip cohorts: their release floor is
        // piece-count-based and coalescing would starve it. Without a synced
        // clock there is no freq to prove seam contiguity, so append as-is.
        let hold_merge_freq = if self.cohort.is_none() {
            (self.mcu_clock_of)(key.mcu_id).map(|(_, freq)| freq)
        } else {
            None
        };
        let q = self
            .queues
            .entry(key)
            .or_insert_with(|| AxisQueue::new(ring_depth));
        q.lead_secs = lead_secs;
        match hold_merge_freq {
            Some(freq) => append_pieces_merging_holds(&mut q.pieces, pieces, freq, !fresh_stream),
            None => q.pieces.extend(pieces),
        }
    }

    fn horizon_of(&self, k: &AxisKey, q: &AxisQueue) -> Option<u64> {
        match (self.mcu_clock_of)(k.mcu_id) {
            Some((ack_now, freq)) =>
            {
                #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
                Some(ack_now + (q.lead_secs * freq) as u64)
            }
            None if self
                .cohort
                .as_ref()
                .map_or(false, |co| co.participants.contains(k)) =>
            {
                Some(0)
            }
            None => None,
        }
    }

    fn poll_ms(&self) -> u64 {
        let cohort_active = self.cohort.is_some();
        let short_lead = (self.holding_ahead || cohort_active)
            && self
                .queues
                .values()
                .any(|q| q.lead_secs < 0.1 && !q.pieces.is_empty());
        if short_lead || cohort_active { 10 } else { 50 }
    }

    fn drain_control(&mut self, control_rx: &Receiver<PumpMsg>) -> Result<bool, ()> {
        let mut activity = false;
        loop {
            match control_rx.try_recv() {
                Ok(m) => {
                    activity = true;
                    if !self.handle_control_msg(m) {
                        return Err(());
                    }
                }
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => return Err(()),
            }
        }
        Ok(activity)
    }

    fn drain_data(&mut self, data_rx: &Receiver<EnqueueMsg>) -> bool {
        let mut activity = false;
        while self.data_open && (self.cohort.is_some() || wants_pieces(&self.queues)) {
            match data_rx.try_recv() {
                Ok(e) => {
                    activity = true;
                    self.enqueue(e);
                }
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => {
                    self.data_open = false;
                    break;
                }
            }
        }
        activity
    }

    fn check_cohort_deadline(&mut self) {
        if let Some(ref co) = self.cohort {
            let now = Instant::now();
            let floor = co.floor(&self.queues);
            if floor > co.deadline_floor {
                let co = self.cohort.as_mut().unwrap();
                co.step_deadline = now + co.timeout;
                co.deadline_floor = floor;
            } else if now >= co.step_deadline {
                let co = self.cohort.as_ref().unwrap();
                let lagging: Vec<String> = co
                    .participants
                    .iter()
                    .map(|k| {
                        format!(
                            "mcu{} axis{}: executed {} queued {}",
                            k.mcu_id,
                            k.axis,
                            co.executed(k, &self.queues),
                            self.queues.get(k).map_or(0, |q| q.pieces.len()),
                        )
                    })
                    .collect();
                let id = co.id;
                (self.on_drip_stall)(format!(
                    "drip cohort {id}: floor stalled at {floor} for {:?}; \
                     participants: [{}]",
                    co.timeout,
                    lagging.join(", ")
                ));
                self.cohort = None;
            }
        }
    }

    fn handle_stall_full(&mut self, stall_key: AxisKey) -> Result<(), ()> {
        let now = std::time::Instant::now();
        let due = self
            .last_stallfull_log
            .is_none_or(|t| now.duration_since(t) >= Duration::from_secs(1));
        let Some(q) = self.queues.get(&stall_key) else {
            return Ok(());
        };
        let current_retired = q.retired;
        if due {
            self.last_stallfull_log = Some(now);
            let in_flight = q.pushed.wrapping_sub(q.retired);
            tracing::debug!(
                subsystem = "motion",
                event = "pump_stall_full",
                mcu = stall_key.mcu_id,
                axis = stall_key.axis,
                pushed = q.pushed,
                retired = q.retired,
                in_flight,
                ring_depth = q.ring_depth,
                room = q.room(),
                pending = q.pieces.len(),
                "pump StallFull (room==0): ring full, awaiting MCU retirement"
            );
        }
        match self.stall_full_since {
            Some((k, r, t)) if k == stall_key && r == current_retired => {
                if now.duration_since(t) >= self.retirement_stall_fatal {
                    let stalled_secs = now.duration_since(t).as_secs_f64();
                    tracing::error!(
                        subsystem = "motion",
                        event = "pump_retirement_stall_fatal",
                        mcu = stall_key.mcu_id,
                        axis = stall_key.axis,
                        pushed = q.pushed,
                        retired = q.retired,
                        ring_depth = q.ring_depth,
                        pending = q.pieces.len(),
                        stalled_secs,
                        "MCU stopped retiring pieces on this axis: retired count \
                         has not advanced while heartbeats kept arriving — the \
                         ring is permanently full and the pump would spin forever"
                    );
                    (self.on_drip_stall)(format!(
                        "pump retirement stall: mcu{} axis{} retired stuck at {} \
                         for {stalled_secs:.1}s with pushed={} ring_depth={} \
                         pending={} — MCU stopped retiring pieces on this axis",
                        stall_key.mcu_id,
                        stall_key.axis,
                        current_retired,
                        q.pushed,
                        q.ring_depth,
                        q.pieces.len(),
                    ));
                    return Err(());
                }
            }
            _ => {
                self.stall_full_since = Some((stall_key, current_retired, now));
            }
        }
        Ok(())
    }

    fn build_bundle(&self, frames: Vec<FramePlan>) -> Vec<AxisFrame> {
        frames
            .into_iter()
            .map(|f| {
                let n = f.pieces.len() as u32;
                let q = self.queues.get(&f.key).expect("planned key exists");
                debug_assert!(
                    q.ring_depth <= u32::from(u16::MAX),
                    "ring_depth {} exceeds u16::MAX; start_slot cast is lossy",
                    q.ring_depth
                );
                AxisFrame {
                    axis: f.key.axis,
                    start_slot: q.physical_write_cursor as u16,
                    new_head: q.pushed.wrapping_add(n),
                    room: q.room(),
                    pieces: f.pieces,
                }
            })
            .collect()
    }

    // Host-side guard: refuse to submit a piece whose start_time is already in
    // the MCU's past. Catching it here fails loud on the host with the
    // offending mcu/axis/deficit instead of letting the MCU (or the EtherCAT
    // endpoint ring) trip a cryptic -308 PieceStartInPast after the fact.
    // Mirrors the MCU's MAX_START_IN_PAST_SECS=200us threshold with a margin
    // above host-projection jitter so a healthy print never false-aborts.
    fn guard_pieces_not_in_past(&self, mcu_id: u32, bundle: &[AxisFrame]) {
        if let Some((mcu_now, freq)) = (self.mcu_clock_of)(mcu_id) {
            if self.cohort.is_some() {
                if let Some(front) = bundle.first().and_then(|af| af.pieces.first()) {
                    tracing::warn!(
                        subsystem = "motion",
                        event = "pump_send_projection",
                        mcu = mcu_id,
                        projected_now = mcu_now,
                        front_start = front.start_time,
                        release_lead_us =
                            ((front.start_time as i64 - mcu_now as i64) as f64 / freq * 1e6),
                        "[drip-diag] projection at send"
                    );
                }
            }
            if freq > 0.0 {
                let guard_ticks = (pump_past_guard_secs() * freq) as u64;
                for af in bundle {
                    for piece in &af.pieces {
                        if piece.start_time + guard_ticks < mcu_now {
                            let deficit_us =
                                ((mcu_now - piece.start_time) as f64 / freq * 1e6) as u64;
                            tracing::error!(
                                subsystem = "motion",
                                event = "pump_piece_in_past",
                                mcu = mcu_id,
                                axis = af.axis,
                                start_time = piece.start_time,
                                mcu_now,
                                deficit_us,
                                "[pump-guard] piece already in the MCU's past at send time — failing loud on host before the MCU/endpoint trips -308"
                            );
                            eprintln!(
                                "pump: piece in past at send — mcu {mcu_id} axis {} start_time={} mcu_now={mcu_now} deficit_us={deficit_us} — aborting host before MCU -308",
                                af.axis, piece.start_time
                            );
                            let _ = std::io::Write::flush(&mut std::io::stderr());
                            std::process::abort();
                        }
                    }
                }
            }
        }
    }

    fn send_bundle_logged(&self, mcu_id: u32, bundle: &[AxisFrame]) -> Result<(), SendError> {
        let send_started = Instant::now();
        let send_result = self.sink.send_mcu_frames(mcu_id, bundle);
        let send_elapsed = send_started.elapsed();
        if send_elapsed >= Duration::from_millis(5) {
            tracing::warn!(
                subsystem = "motion",
                event = "pump_send_blocked",
                mcu = mcu_id,
                elapsed_ms = send_elapsed.as_millis() as u64,
                frames = bundle.len(),
                ok = send_result.is_ok(),
                "[pump-send] send_mcu_frames blocked {}ms on mcu {} ({} frames, ok={})",
                send_elapsed.as_millis() as u64,
                mcu_id,
                bundle.len(),
                send_result.is_ok()
            );
        }
        send_result
    }

    fn commit_sent_bundle(&mut self, mcu_id: u32, bundle: &[AxisFrame]) {
        for af in bundle {
            let key = AxisKey {
                mcu_id,
                axis: af.axis,
            };
            let freq = (self.mcu_clock_of)(mcu_id).map(|(_, f)| f as f32);
            let mut prev_end: Option<u64> = None;
            for piece in &af.pieces {
                prev_end = log_piece_submit(mcu_id, af.axis, freq, piece, prev_end);
            }
            let n = af.pieces.len() as u32;
            let q = self.queues.get_mut(&key).expect("planned key exists");
            for _ in 0..af.pieces.len() {
                let (piece, host_t) = q
                    .pieces
                    .pop_front()
                    .expect("sent frame outran its axis queue");
                if let Some(history) = &self.history {
                    history.record(key, &piece, host_t);
                }
            }
            q.pushed = q.pushed.wrapping_add(n);
            q.advance_write_cursor(n);
        }
    }

    fn send_ready(&mut self) -> Result<bool, ()> {
        let mut activity = false;
        loop {
            let sched = {
                let hz_of = |k: &AxisKey, q: &AxisQueue| self.horizon_of(k, q);
                schedule(
                    &self.queues,
                    MAX_PER_FRAME,
                    BUNDLE_WIRE_BYTE_BUDGET,
                    hz_of,
                    |_| usize::MAX,
                )
            };
            match sched {
                Schedule::Idle => {
                    self.stall_full_since = None;
                    break;
                }
                Schedule::StallFull(stall_key) => {
                    self.handle_stall_full(stall_key)?;
                    break;
                }
                Schedule::StallAhead(_stall_key) => {
                    self.stall_full_since = None;
                    self.holding_ahead = true;
                    break;
                }
                Schedule::Send(frames) => {
                    self.stall_full_since = None;
                    if frames.is_empty() {
                        break;
                    }
                    activity = true;
                    let mcu_id = frames[0].key.mcu_id;
                    let bundle = self.build_bundle(frames);
                    self.guard_pieces_not_in_past(mcu_id, &bundle);
                    let send_result = self.send_bundle_logged(mcu_id, &bundle);
                    match send_result {
                        Ok(()) => {
                            self.commit_sent_bundle(mcu_id, &bundle);
                        }
                        Err(SendError::Fatal(ref e)) => {
                            tracing::error!(
                                subsystem = "motion",
                                event = "send_frame_fatal",
                                mcu = mcu_id,
                                error = %e,
                                "pump send_mcu_frames FATAL transport error — invoking fatal-transport action"
                            );
                            (self.on_fatal_transport)(AxisKey {
                                mcu_id,
                                axis: bundle.first().map_or(0, |f| f.axis),
                            });
                            return Err(());
                        }
                        Err(SendError::Transient(ref e)) => {
                            tracing::error!(
                                subsystem = "motion",
                                event = "send_frame_transient",
                                mcu = mcu_id,
                                error = %e,
                                "pump send_mcu_frames failed"
                            );
                            break;
                        }
                    }
                }
            }
        }
        Ok(activity)
    }

    fn idle_wait(
        &mut self,
        control_rx: &Receiver<PumpMsg>,
        data_rx: &Receiver<EnqueueMsg>,
        poll_ms: u64,
    ) -> Result<(), ()> {
        let mut sel = Select::new();
        let ctrl_op = sel.recv(control_rx);
        let want_data = self.data_open && (self.cohort.is_some() || wants_pieces(&self.queues));
        let data_op = if want_data {
            sel.recv(data_rx)
        } else {
            usize::MAX
        };
        let selected = if self.holding_ahead || self.cohort.is_some() {
            sel.select_timeout(Duration::from_millis(poll_ms))
        } else {
            Ok(sel.select())
        };
        if let Ok(op) = selected {
            let idx = op.index();
            if idx == ctrl_op {
                match op.recv(control_rx) {
                    Ok(m) => {
                        if !self.handle_control_msg(m) {
                            return Err(());
                        }
                    }
                    Err(_) => return Err(()),
                }
            } else if idx == data_op {
                match op.recv(data_rx) {
                    Ok(e) => self.enqueue(e),
                    Err(_) => self.data_open = false,
                }
            }
        }
        Ok(())
    }

    fn publish_ledger(&self) {
        let snapshot = self
            .queues
            .iter()
            .map(|(k, q)| {
                (
                    (k.mcu_id, k.axis),
                    crate::drain::AxisDrainState {
                        pending: q.pieces.len() as u32,
                        pushed: q.pushed,
                        retired: q.retired,
                    },
                )
            })
            .collect();
        self.ledger.publish(snapshot);
    }

    fn run(&mut self, control_rx: Receiver<PumpMsg>, data_rx: Receiver<EnqueueMsg>) {
        self.run_loop(&control_rx, &data_rx);
        self.backlog.store(0, Ordering::Release);
    }

    fn run_loop(&mut self, control_rx: &Receiver<PumpMsg>, data_rx: &Receiver<EnqueueMsg>) {
        loop {
            let poll_ms = self.poll_ms();
            let mut activity = false;

            match self.drain_control(control_rx) {
                Ok(a) => activity |= a,
                Err(()) => return,
            }

            activity |= self.drain_data(data_rx);

            self.check_cohort_deadline();

            self.holding_ahead = false;
            match self.send_ready() {
                Ok(a) => activity |= a,
                Err(()) => return,
            }

            let unpushed: u64 = self.queues.values().map(|q| q.pieces.len() as u64).sum();
            self.backlog.store(unpushed, Ordering::Release);

            self.publish_ledger();
            for ack in self.pending_barrier_acks.drain(..) {
                let _ = ack.send(());
            }

            if activity {
                continue;
            }

            if self.idle_wait(control_rx, data_rx, poll_ms).is_err() {
                return;
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
pub fn run_pump<S, F, C, A, O, D>(
    control_rx: Receiver<PumpMsg>,
    data_rx: Receiver<EnqueueMsg>,
    sink: S,
    ring_depth_of: F,
    mcu_clock_of: C,
    on_fatal_transport: A,
    on_abandon: O,
    history: Option<HistoryRecorder>,
    ledger: Arc<crate::drain::DrainLedger>,
    on_drip_stall: D,
    backlog: Arc<AtomicU64>,
) where
    S: PieceSink,
    F: Fn(AxisKey) -> u32,
    C: Fn(u32) -> Option<(u64, f64)>,
    A: Fn(AxisKey) + Send + 'static,
    O: Fn(AxisKey, u32),
    D: Fn(String) + Send,
{
    let mut pump = Pump {
        queues: BTreeMap::new(),
        junctions: JunctionTracker::default(),
        cohort: None,
        sink,
        ring_depth_of,
        mcu_clock_of,
        on_fatal_transport,
        on_abandon,
        on_drip_stall,
        history,
        ledger,
        pending_barrier_acks: Vec::new(),
        backlog,
        holding_ahead: false,
        data_open: true,
        last_stallfull_log: None,
        retirement_stall_fatal: RETIREMENT_STALL_FATAL,
        stall_full_since: None,
    };
    pump.run(control_rx, data_rx);
}
