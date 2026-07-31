use std::collections::BTreeMap;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, Select, TryRecvError};

use super::diag;
use super::drip::DripCohort;
use super::junction::{JunctionTracker, check_junction_position_continuity};
use super::memstat::MemPressureProbe;
use super::messages::{
    EnqueueMsg, HeartbeatMsg, HistoryRecorder, PieceSink, PumpCallbacks, PumpMsg, SendError,
};
use super::sched::{
    AxisFrame, AxisQueue, FramePlan, Schedule, append_pieces_merging_holds, schedule,
};
use super::stall::RetirementStallWatch;
use crate::types::AxisKey;

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
// this many axis-lane messages are queued — propagating backpressure to the
// input channel and the gcode reader. It only needs to cover the pump's own
// stalls (one wire-send transaction, ~20 ms/KiB bundle) — the staging queues
// behind it hold the real depth — and every queued message is added latency
// for fences and the queued commands riding them.
pub const PUMP_DATA_CHANNEL_CAP: usize = 128;

// Bound on total host-side staged pieces across all axes. Intake stops here so
// the data channel backpressures the planner during streaming. It is a TOTAL,
// not per-axis (all axes interleave on one channel). A drip cohort BYPASSES it:
// a homing axis can lower into a burst larger than the cap, and stopping intake
// there would leave the other participants' messages unpulled behind it on the
// shared channel — starving the cohort floor and freezing the planner on the
// full channel. Drip is finite, so greedy draining is safe there.
//
// Sizing is a latency/stall-margin trade: staged pieces plus the MCU rings
// are the committed depth that keeps the playhead fed through upstream
// stalls, and the longest known stall is a full planner re-plan pass
// (~0.9 s measured on dense jerk-limited infill, planner_bench, M-series;
// expect 2–3 s on a loaded Pi). Staging must stay comfortably above the
// total ring cache (62 KB default = 1322 pieces per MCU, 2–3 MCUs typical)
// or the pump throttles the planner before the frontier can absorb such a
// gap — the playhead then overruns the committed end (anchor_underrun →
// drive fault). Every staged piece beyond that margin is queued-command
// latency: fan changes wait behind the whole backlog. 4096 ≈ 2–3 s of
// typical motion, ~1.5× the two-MCU ring cache.
const PUMP_INTAKE_BACKLOG_CAP: u64 = 4096;

// How long an axis ring may sit at room()==0 with `q.retired` frozen before the
// pump treats it as the MCU having stopped retiring pieces rather than a normal
// transient full-ring wait.
pub(super) const RETIREMENT_STALL_FATAL: Duration = Duration::from_secs(10);

const INFERRED_HALT_FATAL: Duration = Duration::from_secs(1);

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

pub(super) struct Pump<S> {
    pub(super) queues: BTreeMap<AxisKey, AxisQueue>,
    pub(super) junctions: JunctionTracker,
    pub(super) cohort: Option<DripCohort>,
    pub(super) halted: BTreeMap<AxisKey, Option<Instant>>,
    pub(super) sink: S,
    pub(super) callbacks: PumpCallbacks,
    pub(super) history: Option<HistoryRecorder>,
    pub(super) ledger: Arc<crate::drain::DrainLedger>,
    /// Barrier acks are held until the end of the loop iteration, after
    /// intake and the ledger publish — so a caller that receives the ack has
    /// a ledger covering everything it enqueued before sending the barrier.
    pub(super) pending_barrier_acks: Vec<std::sync::mpsc::SyncSender<()>>,
    pub(super) backlog: Arc<AtomicU64>,
    pub(super) holding_ahead: bool,
    pub(super) data_open: bool,
    pub(super) retirement_stall: RetirementStallWatch,
    pub(super) mem_probe: MemPressureProbe,
}

impl<S: PieceSink> Pump<S> {
    fn halt_keys(&mut self, keys: impl IntoIterator<Item = AxisKey>, inferred: bool) {
        let inferred_at = inferred.then(Instant::now);
        for key in keys {
            if inferred {
                self.halted.entry(key).or_insert(inferred_at);
            } else {
                self.halted.insert(key, None);
            }
            if let Some(q) = self.queues.get_mut(&key) {
                let dropped = q.pieces.len() as u32;
                q.pieces.clear();
                q.staged_motion = 0;
                if dropped > 0 {
                    (self.callbacks.on_abandon)(key, dropped);
                }
            }
            self.junctions.forget(key);
        }
    }

    pub(super) fn handle_control_msg(&mut self, msg: PumpMsg) -> bool {
        match msg {
            PumpMsg::Shutdown => return false,
            PumpMsg::Flush(keys) => {
                for key in keys {
                    if let Some(q) = self.queues.get_mut(&key) {
                        let dropped = q.pieces.len() as u32;
                        q.pieces.clear();
                        q.staged_motion = 0;
                        if dropped > 0 {
                            (self.callbacks.on_abandon)(key, dropped);
                        }
                    }
                    self.junctions.forget(key);
                }
            }
            PumpMsg::Halt { keys, ack } => {
                self.halt_keys(keys, false);
                self.pending_barrier_acks.push(ack);
            }
            PumpMsg::Resume(keys) => {
                for key in keys {
                    self.halted.remove(&key);
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
                                (self.callbacks.on_drip_stall)(format!(
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

    pub(super) fn enqueue(&mut self, msg: EnqueueMsg) {
        let EnqueueMsg {
            key,
            pieces,
            epoch,
            lead_secs,
            source_line,
            epoch_freq,
        } = msg;
        if let Some(inferred_at) = self.halted.get(&key).copied() {
            let dropped = pieces.len() as u32;
            if dropped > 0 {
                (self.callbacks.on_abandon)(key, dropped);
            }
            self.junctions.forget(key);
            if let Some(halted_at) = inferred_at {
                if halted_at.elapsed() >= INFERRED_HALT_FATAL {
                    (self.callbacks.on_drip_stall)(format!(
                        "mcu{} axis{} endpoint halt was not acknowledged by the host within {}ms",
                        key.mcu_id,
                        key.axis,
                        halted_at.elapsed().as_millis()
                    ));
                }
            }
            return;
        }
        if let Some(co) = self.cohort.as_ref() {
            if !co.participants.contains(&key) {
                let id = co.id;
                (self.callbacks.on_drip_stall)(format!(
                    "drip cohort {id}: enqueue for non-participant \
                     mcu{} axis{} during active cohort — homing must \
                     drip every axis",
                    key.mcu_id, key.axis
                ));
                self.cohort = None;
                return;
            }
        }
        if epoch.is_fresh() {
            if let Some((first, _)) = pieces.first() {
                tracing::info!(
                    subsystem = "motion",
                    event = "reanchor_mark",
                    mcu = key.mcu_id,
                    axis = key.axis,
                    at_start_clock = first.start_time,
                    "[reanchor] marking fresh-epoch cut"
                );
                self.sink.mark_reanchor(key, first.start_time, epoch_freq);
            }
        }
        if epoch.position_redefined() {
            self.junctions.forget(key);
        }
        if !pieces.is_empty() {
            if let Some((_ack_now, freq)) = (self.callbacks.mcu_clock_of)(key.mcu_id) {
                if let Some(seam) = self.junctions.observe(key, &pieces, source_line, freq) {
                    check_junction_position_continuity(&seam);
                    diag::log_junction_jump(&seam, source_line, epoch.is_fresh(), freq);
                }
            }
        }
        if !pieces.is_empty() {
            if let Some((ack_now, freq)) = (self.callbacks.mcu_clock_of)(key.mcu_id) {
                if freq > 0.0 {
                    let (first, _) = &pieces[0];
                    let margin_s = (first.start_time as i64 - ack_now as i64) as f64 / freq;
                    let warn_floor = crate::anchor::LOW_MARGIN_WARN_SECS - pump_past_guard_secs();
                    if margin_s < warn_floor {
                        tracing::warn!(
                            subsystem = "motion",
                            event = "pump_enqueue_low_lead",
                            mcu = key.mcu_id,
                            axis = key.axis,
                            margin_us = margin_s * 1e6,
                            start_time = first.start_time,
                            ack_now,
                            ?epoch,
                            lead_secs,
                            source_line,
                            n_pieces = pieces.len(),
                            first_is_hold = super::sched::is_hold_piece(first),
                            first_duration_s = f64::from(first.duration),
                            "[pump-enqueue] pieces arrived with less lead than \
                             the low-margin floor — -308 precursor, with \
                             provenance"
                        );
                    }
                }
            }
        }
        let ring_depth = (self.callbacks.ring_depth_of)(key);
        // Hold merging is off during drip cohorts: their release floor is
        // piece-count-based and coalescing would starve it. Without a synced
        // clock there is no freq to prove seam contiguity, so append as-is.
        let hold_merge_freq = if self.cohort.is_none() {
            (self.callbacks.mcu_clock_of)(key.mcu_id).map(|(_, freq)| freq)
        } else {
            None
        };
        let q = self
            .queues
            .entry(key)
            .or_insert_with(|| AxisQueue::new(ring_depth));
        q.lead_secs = lead_secs;
        q.staged_motion += pieces
            .iter()
            .filter(|(p, _)| !super::sched::is_hold_piece(p))
            .count() as u32;
        match hold_merge_freq {
            Some(freq) => {
                append_pieces_merging_holds(&mut q.pieces, pieces, freq, !epoch.is_fresh());
            }
            None => q.pieces.extend(pieces),
        }
    }

    fn horizon_of(&self, k: &AxisKey, q: &AxisQueue) -> Option<u64> {
        match (self.callbacks.mcu_clock_of)(k.mcu_id) {
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
        let Some(co) = &self.cohort else {
            return;
        };
        let now = Instant::now();
        let floor = co.floor(&self.queues);
        if floor > co.deadline_floor {
            let co = self.cohort.as_mut().unwrap();
            co.step_deadline = now + co.timeout;
            co.deadline_floor = floor;
            return;
        }
        if now < co.step_deadline {
            return;
        }
        let fully_executed = co.participants.iter().all(|k| {
            self.queues
                .get(k)
                .is_none_or(|q| q.pieces.is_empty() && q.pushed == q.retired)
        });
        if fully_executed {
            tracing::warn!(
                subsystem = "motion",
                event = "drip_cohort_executed_awaiting_trip",
                cohort = co.id,
                floor,
                "drip cohort fully executed with no trip; the host trip \
                 deadline adjudicates — not a stall"
            );
            let co = self.cohort.as_mut().unwrap();
            co.step_deadline = now + co.timeout;
            return;
        }
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
        (self.callbacks.on_drip_stall)(format!(
            "drip cohort {id}: floor stalled at {floor} for {:?}; \
             participants: [{}]",
            co.timeout,
            lagging.join(", ")
        ));
        self.cohort = None;
    }

    fn handle_stall_full(&mut self, stall_key: AxisKey) -> Result<(), ()> {
        let now = std::time::Instant::now();
        let Some(q) = self.queues.get(&stall_key) else {
            return Ok(());
        };
        let current_retired = q.retired;
        let observation = self
            .retirement_stall
            .observe(stall_key, current_retired, now);
        if observation.log_due {
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
        if let Some(stalled_secs) = observation.stalled_secs {
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
            (self.callbacks.on_drip_stall)(format!(
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
    fn guard_pieces_not_in_past(&self, mcu_id: u32, bundle: &[AxisFrame], context: &str) {
        if let Some((mcu_now, freq)) = (self.callbacks.mcu_clock_of)(mcu_id) {
            if freq > 0.0 {
                let guard_ticks = (pump_past_guard_secs() * freq) as u64;
                for af in bundle {
                    for (piece_idx, piece) in af.pieces.iter().enumerate() {
                        if piece.start_time + guard_ticks < mcu_now {
                            let deficit_us =
                                ((mcu_now - piece.start_time) as f64 / freq * 1e6) as u64;
                            let key = AxisKey {
                                mcu_id,
                                axis: af.axis,
                            };
                            let (queue_lead_secs, queue_pending, queue_staged_motion) =
                                self.queues.get(&key).map_or((f64::NAN, 0, 0), |q| {
                                    (q.lead_secs, q.pieces.len(), q.staged_motion)
                                });
                            tracing::error!(
                                subsystem = "motion",
                                event = "pump_piece_in_past",
                                mcu = mcu_id,
                                axis = af.axis,
                                start_time = piece.start_time,
                                mcu_now,
                                deficit_us,
                                context,
                                piece_idx,
                                is_hold = super::sched::is_hold_piece(piece),
                                duration_s = f64::from(piece.duration),
                                coeff_count = piece.coeff_count,
                                queue_lead_secs,
                                queue_pending,
                                queue_staged_motion,
                                cohort_active = self.cohort.is_some(),
                                "[pump-guard] piece already in the MCU's past {context} — failing loud on host before the MCU/endpoint trips -308"
                            );
                            eprintln!(
                                "pump: piece in past {context} — mcu {mcu_id} axis {} start_time={} mcu_now={mcu_now} deficit_us={deficit_us} piece_idx={piece_idx} is_hold={} duration_s={} coeff_count={} queue_lead_secs={queue_lead_secs} queue_pending={queue_pending} queue_staged_motion={queue_staged_motion} cohort_active={} — aborting host before MCU -308",
                                af.axis,
                                piece.start_time,
                                super::sched::is_hold_piece(piece),
                                f64::from(piece.duration),
                                piece.coeff_count,
                                self.cohort.is_some(),
                            );
                            for (queue_key, q) in &self.queues {
                                let head_start =
                                    q.pieces.front().map_or(0, |(piece, _)| piece.start_time);
                                eprintln!(
                                    "pump-queue: mcu{} axis{} pending={} staged_motion={} pushed={} retired={} ring_depth={} lead_secs={} head_start={head_start}",
                                    queue_key.mcu_id,
                                    queue_key.axis,
                                    q.pieces.len(),
                                    q.staged_motion,
                                    q.pushed,
                                    q.retired,
                                    q.ring_depth,
                                    q.lead_secs,
                                );
                            }
                            super::transit_trace::dump_last_to_stderr(64);
                            std::process::abort();
                        }
                    }
                }
            }
        }
    }

    fn send_bundle_logged(&mut self, mcu_id: u32, bundle: &[AxisFrame]) -> Result<(), SendError> {
        let mem_before = self.mem_probe.sample();
        let send_started = Instant::now();
        let send_result = self.sink.send_mcu_frames(mcu_id, bundle);
        let send_elapsed = send_started.elapsed();
        if send_elapsed >= Duration::from_millis(5) {
            let mem_after = self.mem_probe.sample();
            let (majflt_delta, vm_swap_before_kb, vm_swap_after_kb) = match (mem_before, mem_after)
            {
                (Some(before), Some(after)) => (
                    Some(after.majflt.saturating_sub(before.majflt)),
                    Some(before.vm_swap_kb),
                    Some(after.vm_swap_kb),
                ),
                _ => (None, None, None),
            };
            tracing::warn!(
                subsystem = "motion",
                event = "pump_send_blocked",
                mcu = mcu_id,
                elapsed_ms = send_elapsed.as_millis() as u64,
                frames = bundle.len(),
                ok = send_result.is_ok(),
                majflt_delta,
                vm_swap_before_kb,
                vm_swap_after_kb,
                "[pump-send] send_mcu_frames blocked {}ms on mcu {} ({} frames, ok={}, majflt_delta={:?}, vm_swap_kb={:?}->{:?})",
                send_elapsed.as_millis() as u64,
                mcu_id,
                bundle.len(),
                send_result.is_ok(),
                majflt_delta,
                vm_swap_before_kb,
                vm_swap_after_kb
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
            let freq = (self.callbacks.mcu_clock_of)(mcu_id).map(|(_, f)| f as f32);
            let mut prev_end: Option<u64> = None;
            for piece in &af.pieces {
                prev_end = diag::log_piece_submit(mcu_id, af.axis, freq, piece, prev_end);
            }
            let n = af.pieces.len() as u32;
            let q = self.queues.get_mut(&key).expect("planned key exists");
            for _ in 0..af.pieces.len() {
                let (piece, host_t) = q
                    .pieces
                    .pop_front()
                    .expect("sent frame outran its axis queue");
                if super::sched::is_hold_piece(&piece) {
                    q.wire_hold_tail += 1;
                } else {
                    q.wire_hold_tail = 0;
                    q.staged_motion = q.staged_motion.saturating_sub(1);
                }
                if let Some(history) = &self.history {
                    history.record(key, &piece, host_t);
                }
            }
            q.pushed = q.pushed.wrapping_add(n);
            q.advance_write_cursor(n);
        }
    }

    // A send pass monopolizes the loop while its synchronous wire round-trips
    // run (~2 ms per EtherCAT bundle, ~20 ms per 1 KiB serial bundle at
    // 500 kbaud), while newly produced earlier-deadline pieces for another
    // axis wait in the data channel (observed: a 130 ms pass aged a z-hop
    // burst 53 ms into the MCU past). A wall-clock deadline bounds intake and
    // control latency identically on every transport; the deadline is checked
    // after each bundle, so every pass sends at least one.
    const SEND_PASS_BUDGET: Duration = Duration::from_millis(10);

    pub(super) fn send_ready(&mut self) -> Result<bool, ()> {
        self.send_ready_until(Instant::now() + Self::SEND_PASS_BUDGET)
    }

    pub(super) fn send_ready_until(&mut self, pass_deadline: Instant) -> Result<bool, ()> {
        let mut activity = false;
        loop {
            let sched = {
                let hz_of = |k: &AxisKey, q: &AxisQueue| self.horizon_of(k, q);
                schedule(
                    &self.queues,
                    |mcu_id| self.sink.bundle_limits(mcu_id),
                    hz_of,
                    |_| usize::MAX,
                )
            };
            match sched {
                Schedule::Idle => {
                    self.retirement_stall.reset();
                    break;
                }
                Schedule::StallFull(stall_key) => {
                    self.handle_stall_full(stall_key)?;
                    break;
                }
                Schedule::StallAhead(_stall_key) => {
                    self.retirement_stall.reset();
                    self.holding_ahead = true;
                    break;
                }
                Schedule::Send(frames) => {
                    self.retirement_stall.reset();
                    if frames.is_empty() {
                        break;
                    }
                    activity = true;
                    let mcu_id = frames[0].key.mcu_id;
                    let bundle = self.build_bundle(frames);
                    self.guard_pieces_not_in_past(mcu_id, &bundle, "at send");
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
                            (self.callbacks.on_fatal_transport)(AxisKey {
                                mcu_id,
                                axis: bundle.first().map_or(0, |f| f.axis),
                            });
                            return Err(());
                        }
                        Err(SendError::Halted(ref e)) => {
                            tracing::debug!(
                                subsystem = "motion",
                                event = "send_frame_halted",
                                mcu = mcu_id,
                                error = %e,
                                "pump frame met an endpoint halt and was discarded"
                            );
                            self.halt_keys(
                                bundle.iter().map(|frame| AxisKey {
                                    mcu_id,
                                    axis: frame.axis,
                                }),
                                true,
                            );
                            break;
                        }
                        Err(SendError::Transient(ref e)) => {
                            tracing::error!(
                                subsystem = "motion",
                                event = "send_frame_transient",
                                mcu = mcu_id,
                                error = %e,
                                "pump send_mcu_frames failed"
                            );
                            self.guard_pieces_not_in_past(
                                mcu_id,
                                &bundle,
                                "after a failed send (transport gave no response \
                                 while the piece's scheduling lead ran out)",
                            );
                            break;
                        }
                    }
                }
            }
            if Instant::now() >= pass_deadline {
                break;
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
                        staged_motion: q.staged_motion,
                        hold_tail: q.wire_hold_tail,
                    },
                )
            })
            .collect();
        self.ledger.publish(snapshot);
    }

    pub(super) fn run(&mut self, control_rx: Receiver<PumpMsg>, data_rx: Receiver<EnqueueMsg>) {
        self.run_loop(&control_rx, &data_rx);
        self.backlog.store(0, Ordering::Release);
    }

    fn run_loop(&mut self, control_rx: &Receiver<PumpMsg>, data_rx: &Receiver<EnqueueMsg>) {
        loop {
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

            if self.idle_wait(control_rx, data_rx, self.poll_ms()).is_err() {
                return;
            }
        }
    }
}

pub fn run_pump<S: PieceSink>(
    control_rx: Receiver<PumpMsg>,
    data_rx: Receiver<EnqueueMsg>,
    sink: S,
    callbacks: PumpCallbacks,
    history: Option<HistoryRecorder>,
    ledger: Arc<crate::drain::DrainLedger>,
    backlog: Arc<AtomicU64>,
) {
    let mut pump = Pump {
        queues: BTreeMap::new(),
        junctions: JunctionTracker::default(),
        cohort: None,
        halted: BTreeMap::new(),
        sink,
        callbacks,
        history,
        ledger,
        pending_barrier_acks: Vec::new(),
        backlog,
        holding_ahead: false,
        data_open: true,
        retirement_stall: RetirementStallWatch::new(RETIREMENT_STALL_FATAL),
        mem_probe: MemPressureProbe::new(),
    };
    pump.run(control_rx, data_rx);
}
