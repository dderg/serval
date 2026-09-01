use std::collections::{BTreeMap, HashMap, HashSet};
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, Select, TryRecvError};

use super::diag;
use super::drip::DripCohort;
use super::junction::{JunctionTracker, check_junction_position_continuity};
use super::memstat::MemPressureProbe;
use super::messages::{
    BuzzParams, BuzzStart, BuzzToken, BuzzTransport, DrainTick, EnqueueMsg, HeartbeatMsg,
    HistoryRecorder, PumpCallbacks, PumpMsg, SendError, SpanSink,
};
use super::sched::{
    AxisFrame, AxisQueue, FramePlan, ReleaseHorizons, Schedule, append_spans_merging_holds,
    schedule,
};
use super::stall::ConsumptionStallWatch;
use crate::types::AxisKey;
use trajectory::ClockedMotorSpan;

// How far ahead of the MCU playhead the pump pushes views — the depth of the
// host-side buffer that absorbs scheduling hiccups. A view whose start_clock
// has already passed cannot be executed, so this must exceed the worst-case
// host stall between pump pushes. Each transport's own depth (`room()`) is the
// hard cap; for dense streams the pump stalls on a full endpoint well before
// this horizon, so raising it only deepens the buffer for sparse (long, slow)
// moves where stalls are otherwise most likely to slip a view into the past.
pub const MAX_LEAD_SECS: f64 = 2.0;

// Bound on the planner→pump span-data channel. When the pump stops pulling
// (ring full or at the lead horizon), the planner's dispatch send blocks once
// this many axis-lane messages are queued — propagating backpressure to the
// input channel and the gcode reader. It only needs to cover the pump's own
// stalls (one wire-send transaction, ~20 ms/KiB bundle) — the staging queues
// behind it hold the real depth — and every queued message is added latency
// for fences and the queued commands riding them.
pub const PUMP_DATA_CHANNEL_CAP: usize = 128;

pub(super) const PUMP_INTAKE_BACKLOG_SOFT_CAP: u64 = 4096;
pub(super) const PUMP_INTAKE_BACKLOG_HARD_CAP: u64 = 8192;
pub(super) const PUMP_INTAKE_MIN_RUNWAY_SECS: f64 = 5.0;

fn staged_axis_runway_secs(queue: &AxisQueue) -> f64 {
    let Some(first) = queue.spans.front() else {
        return 0.0;
    };
    let last = queue.spans.back().expect("nonempty queue has a tail");
    let runway = last.end_host - first.start_host;
    assert!(
        runway.is_finite() && runway >= 0.0,
        "pump staged axis runway must be finite and nonnegative, got {runway}"
    );
    runway
}

fn minimum_staged_runway_secs(queues: &BTreeMap<AxisKey, AxisQueue>) -> f64 {
    queues
        .values()
        .map(staged_axis_runway_secs)
        .reduce(f64::min)
        .unwrap_or(0.0)
}

pub(super) fn wants_spans(queues: &BTreeMap<AxisKey, AxisQueue>) -> bool {
    let staged: u64 = queues.values().map(|q| q.spans.len() as u64).sum();
    staged < PUMP_INTAKE_BACKLOG_SOFT_CAP
        || (staged < PUMP_INTAKE_BACKLOG_HARD_CAP
            && minimum_staged_runway_secs(queues) < PUMP_INTAKE_MIN_RUNWAY_SECS)
}

pub(super) const CONSUMPTION_STALL_FATAL: Duration = Duration::from_secs(10);
const INFERRED_HALT_FATAL: Duration = Duration::from_secs(1);

// Mirrors the MCU's MAX_START_IN_PAST_SECS: 500us over host-projection
// jitter on real hardware; in the simulator (MCU_SIM_SOCK_DIR set) the
// virtual clock races arbitrarily far ahead of the host projection, so
// the guard widens to the MCU's own mcu-sim grace instead of aborting on
// infrastructure jitter.
pub(crate) fn pump_past_guard_secs() -> f64 {
    static GUARD: std::sync::OnceLock<f64> = std::sync::OnceLock::new();
    *GUARD.get_or_init(|| {
        if std::env::var_os("MCU_SIM_SOCK_DIR").is_some() {
            10.0
        } else {
            500e-6
        }
    })
}

/// How a lane came to be halted. An acknowledged halt supersedes an inferred
/// one; an inferred one never overwrites an acknowledgement.
#[derive(Clone, Copy, Debug)]
pub(super) enum HaltKind {
    Inferred(Instant),
    Acknowledged,
}

/// Why a send pass stopped, and whether it shipped anything.
#[derive(Clone, Copy, Debug)]
pub(super) struct PassEnd {
    pub(super) sent: bool,
    pub(super) waiting_on_clock: bool,
}

/// What the lane's committed wire stream must do at the first incoming view.
#[derive(Clone, Copy, Debug)]
enum LaneCut {
    /// A retimed epoch: the incoming clock bears no relation to the timeline
    /// the transport still holds, so it cuts its stream at this view.
    Reanchor { at_start_clock: u64 },
    /// A `Rejoin` epoch: stream time itself jumped a drained-to-rest hole.
    RejoinGap { at_start_clock: u64 },
    /// A continuation resuming past a lane-local hole it sat out at rest.
    SatOutGap { seam_end: u64, at_start_clock: u64 },
    /// A continuation resuming past a hole its last view left mid-motion:
    /// trajectory content is missing, so the gap is never sanctioned.
    HoleMidMotion { seam_end: u64, at_start_clock: u64 },
    /// A retimed epoch carrying no view: the committed seam belongs to the
    /// timeline that just retired and can gate nothing.
    RetireSeam,
    /// The committed stream runs straight into this view.
    Continues,
}

pub(super) struct Pump<S> {
    pub(super) queues: BTreeMap<AxisKey, AxisQueue>,
    pub(super) junctions: JunctionTracker,
    pub(super) cohort: Option<DripCohort>,
    pub(super) halted: BTreeMap<AxisKey, HaltKind>,
    pub(super) sink: S,
    pub(super) callbacks: PumpCallbacks,
    pub(super) history: Option<HistoryRecorder>,
    pub(super) ledger: Arc<crate::drain::DrainLedger>,
    pub(super) pending_barrier_acks: Vec<std::sync::mpsc::SyncSender<()>>,
    pub(super) backlog: Arc<AtomicU64>,
    pub(super) horizons: ReleaseHorizons,
    pub(super) data_open: bool,
    pub(super) intake_batch_open: bool,
    pub(super) consumption_stall: ConsumptionStallWatch,
    pub(super) mem_probe: MemPressureProbe,
}

impl<S: SpanSink> Pump<S> {
    /// This key's staged work is void: drop it, tell the host what it lost,
    /// and forget the junction so the next view is not held contiguous with
    /// motion that never ran.
    fn abandon_staged(&mut self, key: AxisKey) {
        if let Some(q) = self.queues.get_mut(&key) {
            let dropped = q.spans.len() as u32;
            q.spans.clear();
            q.staged_motion = 0;
            q.seam_end_clock = None;
            if dropped > 0 {
                (self.callbacks.on_abandon)(key, dropped);
            }
        }
        self.junctions.forget(key);
    }

    fn halt_keys(
        &mut self,
        keys: impl IntoIterator<Item = AxisKey>,
        kind: HaltKind,
    ) -> Result<(), SendError> {
        let keys: Vec<AxisKey> = keys.into_iter().collect();
        self.sink.cut_staged(&keys)?;
        for key in keys {
            match kind {
                HaltKind::Inferred(_) => {
                    self.halted.entry(key).or_insert(kind);
                }
                HaltKind::Acknowledged => {
                    self.halted.insert(key, kind);
                }
            }
            if let Some(q) = self.queues.get_mut(&key) {
                q.wire_hold_tail = 0;
                q.wire_end_clock = None;
            }
            self.abandon_staged(key);
        }
        Ok(())
    }

    pub(super) fn handle_control_msg(&mut self, msg: PumpMsg) -> bool {
        match msg {
            PumpMsg::Shutdown => return false,
            PumpMsg::Flush(keys) => {
                if let Err(e) = self.sink.flush_keys(&keys) {
                    tracing::error!(
                        subsystem = "motion",
                        event = "stepcompress_flush_fatal",
                        error = ?e,
                        "stepcompress flush rejected — invoking fatal-transport action"
                    );
                    for key in keys {
                        (self.callbacks.on_fatal_transport)(key);
                    }
                    return false;
                }
                for key in keys {
                    self.abandon_staged(key);
                }
            }
            PumpMsg::Halt { keys, ack } => {
                if let Err(error) = self.halt_keys(keys.clone(), HaltKind::Acknowledged) {
                    tracing::error!(
                        subsystem = "motion",
                        event = "halt_cut_fatal",
                        error = ?error,
                        "endpoint rejected the halt cut"
                    );
                    for key in keys {
                        (self.callbacks.on_fatal_transport)(key);
                    }
                    return false;
                }
                self.pending_barrier_acks.push(ack);
            }
            PumpMsg::Resume(keys) => {
                for key in keys {
                    self.halted.remove(&key);
                }
            }
            PumpMsg::Heartbeat(HeartbeatMsg {
                mcu_id,
                axes,
                consumed_counts,
                retired_counts,
                retired_by,
            }) => {
                let consumed_counts = consumed_counts.as_ref().unwrap_or(&retired_counts);
                assert_eq!(
                    consumed_counts.len(),
                    retired_counts.len(),
                    "heartbeat consumed/retired axis count mismatch for mcu{mcu_id}"
                );
                assert_eq!(
                    axes.len(),
                    retired_counts.len(),
                    "heartbeat names {} axes for {} counts on mcu{mcu_id}",
                    axes.len(),
                    retired_counts.len()
                );
                for (slot, &axis) in axes.iter().enumerate() {
                    let key = AxisKey { mcu_id, axis };
                    let mut c = retired_counts[slot];
                    if let Some(q) = self.queues.get_mut(&key) {
                        q.credit(retired_by, consumed_counts[slot], c);
                        c = q.retired;
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
                    execution_floor: 0,
                });
            }
            PumpMsg::DripDisarm(c) => {
                if self.cohort.as_ref().map_or(false, |co| co.id == c) {
                    self.cohort = None;
                }
            }
            PumpMsg::StepcompressBarrierAck { mcu_id, oid, seq } => {
                if let Err(e) = self.sink.on_barrier_ack(mcu_id, oid, seq) {
                    tracing::error!(
                        subsystem = "motion",
                        event = "stepcompress_barrier_ack_fatal",
                        mcu = mcu_id,
                        oid,
                        seq,
                        error = ?e,
                        "stepcompress barrier ack rejected — invoking fatal-transport action"
                    );
                    (self.callbacks.on_fatal_transport)(AxisKey { mcu_id, axis: 0 });
                    return false;
                }
            }
            PumpMsg::StepcompressFatal { mcu_id, error } => {
                tracing::error!(
                    subsystem = "motion",
                    event = "stepcompress_endpoint_fatal",
                    mcu = mcu_id,
                    error = %error,
                    "stepcompress endpoint reported a fatal condition — invoking \
                     fatal-transport action"
                );
                (self.callbacks.on_fatal_transport)(AxisKey { mcu_id, axis: 0 });
                return false;
            }
            PumpMsg::MarkReanchor {
                key,
                at_start_clock,
                epoch_freq,
            } => {
                tracing::info!(
                    subsystem = "motion",
                    event = "sibling_lane_reanchor_mark",
                    mcu = key.mcu_id,
                    axis = key.axis,
                    at_start_clock,
                    "[reanchor] projection rebase cut a sibling lane without pieces"
                );
                self.cut_lane_at(key, at_start_clock, epoch_freq);
            }
            PumpMsg::Buzz { params, reply } => {
                let armed = self.arm_buzz(&params);
                if let Err(error) = &armed {
                    tracing::warn!(
                        subsystem = "motion",
                        event = "resonance_buzz_rejected",
                        error,
                        "[buzz] arming refused — no transport was touched"
                    );
                }
                let _ = reply.send(armed);
            }
            PumpMsg::Barrier(ack) => {
                self.pending_barrier_acks.push(ack);
            }
        }
        true
    }

    /// Arm one sweep across every transport it names. The whole point of
    /// routing it here is that the pump is the only thread allowed to touch
    /// a transport while it is streaming: it clears every route first, so a
    /// machine whose Y is a servo, Z a pulse lane and X a phase lane either
    /// starts all three off one profile or starts none of them. Every route
    /// of one mcu is anchored on the one start resolved for that mcu, so the
    /// axes of one sweep stay in phase across transports.
    fn arm_buzz(&mut self, params: &BuzzParams) -> Result<BuzzToken, String> {
        if params.routes.is_empty() {
            return Err("resonance buzz names no transport to drive".to_string());
        }
        let profile = Arc::new(
            params
                .wave
                .profile()
                .map_err(|error| format!("resonance buzz profile rejected: {error}"))?,
        );
        let clock_of = &*self.callbacks.mcu_clock_of;
        let mut starts: HashMap<u32, BuzzStart> = HashMap::new();
        let mut transports: HashSet<(u32, BuzzTransport)> = HashSet::new();
        let mut host_axes: HashMap<u32, u8> = HashMap::new();
        for route in params.routes.iter() {
            let mcu_id = route.mcu_id();
            let transport = route.transport();
            if !transports.insert((mcu_id, transport)) {
                return Err(format!(
                    "resonance buzz rejected: mcu {mcu_id} {transport:?} transport is named twice \
                     in one arming"
                ));
            }
            if transport != BuzzTransport::Ethercat {
                let driven = route.driven_mask();
                let claimed = host_axes.entry(mcu_id).or_default();
                let clash = *claimed & driven;
                if clash != 0 {
                    return Err(format!(
                        "resonance buzz rejected: mcu {mcu_id} axis mask 0x{clash:02x} is driven \
                         by more than one route of this arming"
                    ));
                }
                *claimed |= driven;
            }
            if self
                .queues
                .iter()
                .any(|(key, queue)| key.mcu_id == mcu_id && !queue.spans.is_empty())
            {
                return Err(format!(
                    "resonance buzz rejected: mcu {mcu_id} still has trajectory staged in the pump"
                ));
            }
            let start = match starts.get(&mcu_id) {
                Some(start) => *start,
                None => {
                    let start = super::messages::anchored_start(
                        mcu_id,
                        clock_of,
                        super::stepcompress_sink::SEND_LEAD_SECONDS,
                    )?;
                    starts.insert(mcu_id, start);
                    start
                }
            };
            route.ready(start)?;
        }
        for route in params.routes.iter() {
            let start = starts[&route.mcu_id()];
            route.arm(&profile, params.wave, start)?;
        }
        Ok(BuzzToken::new(Arc::clone(&params.routes)))
    }

    pub(super) fn enqueue(&mut self, msg: EnqueueMsg) {
        let EnqueueMsg {
            key,
            spans,
            epoch,
            lead_secs,
            source_line,
            epoch_freq,
            batch_end: _,
        } = msg;
        if let Some(kind) = self.halted.get(&key).copied() {
            let dropped = spans.len() as u32;
            if dropped > 0 {
                (self.callbacks.on_abandon)(key, dropped);
            }
            self.junctions.forget(key);
            if let HaltKind::Inferred(halted_at) = kind {
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
        let first = spans.first();
        match self.lane_cut_for(key, epoch, first) {
            LaneCut::Reanchor { at_start_clock } => {
                tracing::info!(
                    subsystem = "motion",
                    event = "reanchor_mark",
                    mcu = key.mcu_id,
                    axis = key.axis,
                    at_start_clock,
                    "[reanchor] marking fresh-epoch cut"
                );
                self.cut_lane_at(key, at_start_clock, epoch_freq);
            }
            LaneCut::RejoinGap { at_start_clock } => {
                tracing::info!(
                    subsystem = "motion",
                    event = "seam_gap_mark",
                    mcu = key.mcu_id,
                    axis = key.axis,
                    at_start_clock,
                    "[rejoin] marking a sanctioned forward seam gap"
                );
                self.sink.mark_seam_gap(key, at_start_clock);
            }
            LaneCut::SatOutGap {
                seam_end,
                at_start_clock,
            } => {
                tracing::info!(
                    subsystem = "motion",
                    event = "lane_rejoin_gap_mark",
                    mcu = key.mcu_id,
                    axis = key.axis,
                    seam_end,
                    at_start_clock,
                    "[rejoin] lane sat out single-lane traffic at rest — \
                     sanctioning its forward seam gap"
                );
                self.sink.mark_seam_gap(key, at_start_clock);
            }
            LaneCut::HoleMidMotion {
                seam_end,
                at_start_clock,
            } => {
                tracing::error!(
                    subsystem = "motion",
                    event = "lane_hole_mid_motion",
                    mcu = key.mcu_id,
                    axis = key.axis,
                    seam_end,
                    at_start_clock,
                    "[rejoin] forward lane hole while the lane's last span \
                     ended in motion — trajectory content is missing; the \
                     endpoint seam guard will fail loud"
                );
            }
            LaneCut::RetireSeam => self.clear_lane_seam(key),
            LaneCut::Continues => {}
        }
        if epoch.position_redefined() {
            self.junctions.forget(key);
        }
        if let Some(seam) = self.junctions.observe(key, &spans, source_line) {
            check_junction_position_continuity(&seam);
            if let Some((_ack_now, freq)) = (self.callbacks.mcu_clock_of)(key.mcu_id) {
                diag::log_junction_jump(&seam, source_line, epoch.is_fresh(), freq);
            }
        }
        if let Some(first) = first {
            if let Some((ack_now, freq)) = (self.callbacks.mcu_clock_of)(key.mcu_id) {
                if freq > 0.0 {
                    let margin_s = (first.start_clock as i64 - ack_now as i64) as f64 / freq;
                    let warn_floor = crate::anchor::LOW_MARGIN_WARN_SECS - pump_past_guard_secs();
                    if margin_s < warn_floor {
                        tracing::warn!(
                            subsystem = "motion",
                            event = "pump_enqueue_low_lead",
                            mcu = key.mcu_id,
                            axis = key.axis,
                            margin_us = margin_s * 1e6,
                            start_clock = first.start_clock,
                            ack_now,
                            ?epoch,
                            lead_secs,
                            source_line,
                            n_spans = spans.len(),
                            first_is_hold = super::sched::is_hold_span(first),
                            first_duration_s = first.stream_t_end - first.stream_t_start,
                            "[pump-enqueue] spans arrived with less lead than \
                             the low-margin floor — -308 precursor, with \
                             provenance"
                        );
                    }
                }
            }
        }
        let ring_depth = (self.callbacks.ring_depth_of)(key);
        let lane_seam_track = spans
            .last()
            .map(|last| (last.end_clock, super::sched::span_ends_at_rest(last)));
        // Hold merging is off during drip cohorts: their release floor is
        // view-count-based and coalescing would starve it.
        let merge_holds = self.cohort.is_none();
        let q = self
            .queues
            .entry(key)
            .or_insert_with(|| AxisQueue::new(ring_depth));
        q.lead_secs = lead_secs;
        if let Some((end, at_rest)) = lane_seam_track {
            q.seam_end_clock = Some(end);
            q.seam_end_at_rest = at_rest;
        }
        q.staged_motion += spans
            .iter()
            .filter(|span| !super::sched::is_hold_span(span))
            .count() as u32;
        if merge_holds {
            append_spans_merging_holds(&mut q.spans, spans, !epoch.is_fresh());
        } else {
            q.spans.extend(spans);
        }
    }

    fn lane_cut_for(
        &self,
        key: AxisKey,
        epoch: crate::anchor::StreamEpoch,
        first: Option<&ClockedMotorSpan>,
    ) -> LaneCut {
        let Some(first) = first else {
            return if epoch.is_fresh() {
                LaneCut::RetireSeam
            } else {
                LaneCut::Continues
            };
        };
        let at_start_clock = first.start_clock;
        if epoch.is_fresh() {
            return if epoch == crate::anchor::StreamEpoch::Rejoin {
                LaneCut::RejoinGap { at_start_clock }
            } else {
                LaneCut::Reanchor { at_start_clock }
            };
        }
        let Some((seam_end, at_rest)) = self
            .queues
            .get(&key)
            .and_then(|q| q.seam_end_clock.map(|end| (end, q.seam_end_at_rest)))
        else {
            return LaneCut::Continues;
        };
        #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
        let floor = (first.clock_freq_hz * super::sched::LANE_REJOIN_GAP_FLOOR_SECS) as u64;
        if at_start_clock <= seam_end.saturating_add(floor) {
            return LaneCut::Continues;
        }
        if at_rest {
            LaneCut::SatOutGap {
                seam_end,
                at_start_clock,
            }
        } else {
            LaneCut::HoleMidMotion {
                seam_end,
                at_start_clock,
            }
        }
    }

    fn cut_lane_at(&mut self, key: AxisKey, at_start_clock: u64, epoch_freq: Option<f64>) {
        self.sink.mark_reanchor(key, at_start_clock, epoch_freq);
        self.clear_lane_seam(key);
    }

    fn clear_lane_seam(&mut self, key: AxisKey) {
        if let Some(q) = self.queues.get_mut(&key) {
            q.seam_end_clock = None;
        }
    }

    /// Sample every mcu's clock once and derive each staged lane's release
    /// horizon from that one reading: the scheduling pass that follows judges
    /// one instant.
    fn resample_horizons(&mut self) {
        let Self {
            queues,
            horizons,
            callbacks,
            cohort,
            ..
        } = self;
        horizons.resample(
            queues,
            |mcu_id| (callbacks.mcu_clock_of)(mcu_id),
            |key, q, clock| match clock {
                Some((ack_now, freq)) =>
                {
                    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
                    Some(ack_now + (q.lead_secs * freq) as u64)
                }
                None if cohort
                    .as_ref()
                    .is_some_and(|co| co.participants.contains(key)) =>
                {
                    Some(0)
                }
                None => None,
            },
        );
    }

    /// `None` parks on the channels alone; `Some(d)` parks with a deadline
    /// because only elapsed time can unblock the loop.
    fn wake_after(&self, deferred_work: bool) -> Option<Duration> {
        let cohort_active = self.cohort.is_some();
        if !(deferred_work || cohort_active) {
            return None;
        }
        let short_lead = self
            .queues
            .values()
            .any(|q| q.lead_secs < 0.1 && !q.spans.is_empty());
        Some(Duration::from_millis(if cohort_active || short_lead {
            10
        } else {
            50
        }))
    }

    fn wants_more_data(&self) -> bool {
        self.data_open && (self.intake_batch_open || wants_spans(&self.queues))
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
        while self.wants_more_data() {
            match data_rx.try_recv() {
                Ok(e) => {
                    activity = true;
                    self.intake_batch_open = !e.batch_end;
                    self.enqueue(e);
                }
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => {
                    assert!(
                        !self.intake_batch_open,
                        "pump data channel disconnected before the projection batch ended"
                    );
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
        let execution_floor = co.active_execution_floor(&self.queues);
        if execution_floor != co.execution_floor {
            let co = self.cohort.as_mut().unwrap();
            co.step_deadline = now + co.timeout;
            co.execution_floor = execution_floor;
            return;
        }
        if now < co.step_deadline {
            return;
        }
        let fully_executed = co.participants.iter().all(|k| {
            self.queues
                .get(k)
                .is_none_or(|q| q.spans.is_empty() && q.pushed == q.retired)
        });
        if fully_executed {
            tracing::warn!(
                subsystem = "motion",
                event = "drip_cohort_executed_awaiting_trip",
                cohort = co.id,
                execution_floor,
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
                    "mcu{} axis{}: executed {} queued {} in_flight {}",
                    k.mcu_id,
                    k.axis,
                    co.executed(k, &self.queues),
                    self.queues.get(k).map_or(0, |q| q.spans.len()),
                    self.queues
                        .get(k)
                        .map_or(0, |q| q.pushed.wrapping_sub(q.retired)),
                )
            })
            .collect();
        let id = co.id;
        (self.callbacks.on_drip_stall)(format!(
            "drip cohort {id}: execution stalled at floor {execution_floor} for {:?}; \
             participants: [{}]",
            co.timeout,
            lagging.join(", ")
        ));
        self.cohort = None;
    }

    fn handle_stall_full(&mut self, stall_key: AxisKey) -> Result<(), ()> {
        let now = std::time::Instant::now();
        let q = self
            .queues
            .get(&stall_key)
            .expect("the scheduler stalls on a queue it found");
        let current_consumed = q.consumed;
        if let (Some((mcu_clock, _)), Some(wire_end_clock)) = (
            (self.callbacks.mcu_clock_of)(stall_key.mcu_id),
            q.wire_end_clock,
        ) {
            if mcu_clock <= wire_end_clock {
                self.consumption_stall.reset();
                return Ok(());
            }
        }
        let observation = self
            .consumption_stall
            .observe(stall_key, current_consumed, now);
        if observation.log_due {
            let awaiting_consumption = q.pushed.wrapping_sub(q.consumed);
            tracing::debug!(
                subsystem = "motion",
                event = "pump_stall_full",
                mcu = stall_key.mcu_id,
                axis = stall_key.axis,
                pushed = q.pushed,
                consumed = q.consumed,
                retired = q.retired,
                awaiting_consumption,
                ring_depth = q.ring_depth,
                room = q.room(),
                pending = q.spans.len(),
                "pump StallFull (room==0): endpoint has not consumed the next span"
            );
        }
        if let Some(stalled_secs) = observation.stalled_secs {
            tracing::error!(
                subsystem = "motion",
                event = "pump_consumption_stall_fatal",
                mcu = stall_key.mcu_id,
                axis = stall_key.axis,
                pushed = q.pushed,
                consumed = q.consumed,
                retired = q.retired,
                ring_depth = q.ring_depth,
                pending = q.spans.len(),
                stalled_secs,
                "endpoint stopped consuming spans on this axis while heartbeats continued"
            );
            (self.callbacks.on_drip_stall)(format!(
                "pump consumption stall: mcu{} axis{} consumed stuck at {} for \
                 {stalled_secs:.1}s with pushed={} retired={} ring_depth={} pending={}",
                stall_key.mcu_id,
                stall_key.axis,
                current_consumed,
                q.pushed,
                q.retired,
                q.ring_depth,
                q.spans.len(),
            ));
            return Err(());
        }
        Ok(())
    }

    fn build_bundle(&self, frames: Vec<FramePlan>) -> Vec<AxisFrame> {
        frames
            .into_iter()
            .map(|f| {
                let n = f.spans.len() as u32;
                let q = self.queues.get(&f.key).expect("planned key exists");
                AxisFrame {
                    axis: f.key.axis,
                    new_head: q.pushed.wrapping_add(n),
                    room: q.room(),
                    spans: f.spans,
                    guard_recorded_ns: 0,
                    guard_mcu_clock: 0,
                }
            })
            .collect()
    }

    // Host-side guard: refuse to submit a view whose start_clock is already in
    // the MCU's past. Catching it here fails loud on the host with the
    // offending mcu/axis/deficit instead of letting the MCU (or the EtherCAT
    // endpoint ring) trip a cryptic -308 start-in-past after the fact.
    // Mirrors the MCU's MAX_START_IN_PAST_SECS=200us threshold with a margin
    // above host-projection jitter so a healthy print never false-aborts.
    fn guard_spans_not_in_past(&self, mcu_id: u32, bundle: &mut [AxisFrame], context: &str) {
        let guard_recorded_ns = super::transit_trace::trace_now_ns();
        if let Some((mcu_now, freq)) = (self.callbacks.mcu_clock_of)(mcu_id) {
            for frame in &mut *bundle {
                frame.guard_recorded_ns = guard_recorded_ns;
                frame.guard_mcu_clock = mcu_now;
            }
            if freq > 0.0 {
                let guard_ticks = (pump_past_guard_secs() * freq) as u64;
                for af in bundle {
                    for (span_idx, span) in af.spans.iter().enumerate() {
                        if span.start_clock + guard_ticks < mcu_now {
                            let deficit_us =
                                ((mcu_now - span.start_clock) as f64 / freq * 1e6) as u64;
                            let key = AxisKey {
                                mcu_id,
                                axis: af.axis,
                            };
                            let (queue_lead_secs, queue_pending, queue_staged_motion) =
                                self.queues.get(&key).map_or((f64::NAN, 0, 0), |q| {
                                    (q.lead_secs, q.spans.len(), q.staged_motion)
                                });
                            tracing::error!(
                                subsystem = "motion",
                                event = "pump_span_in_past",
                                mcu = mcu_id,
                                axis = af.axis,
                                start_clock = span.start_clock,
                                mcu_now,
                                deficit_us,
                                context,
                                span_idx,
                                is_hold = super::sched::is_hold_span(span),
                                duration_s = span.stream_t_end - span.stream_t_start,
                                queue_lead_secs,
                                queue_pending,
                                queue_staged_motion,
                                cohort_active = self.cohort.is_some(),
                                "[pump-guard] span already in the MCU's past {context} — failing loud on host before the MCU/endpoint trips -308"
                            );
                            eprintln!(
                                "pump: span in past {context} — mcu {mcu_id} axis {} start_clock={} mcu_now={mcu_now} deficit_us={deficit_us} span_idx={span_idx} is_hold={} duration_s={} cohort_active={}",
                                af.axis,
                                span.start_clock,
                                super::sched::is_hold_span(span),
                                span.stream_t_end - span.stream_t_start,
                                self.cohort.is_some(),
                            );
                            for (queue_key, q) in &self.queues {
                                let head_start = q.spans.front().map_or(0, |span| span.start_clock);
                                eprintln!(
                                    "pump-queue: mcu{} axis{} pending={} staged_motion={} pushed={} retired={} ring_depth={} lead_secs={} head_start={head_start}",
                                    queue_key.mcu_id,
                                    queue_key.axis,
                                    q.spans.len(),
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
            let mut prev_end: Option<u64> = None;
            for span in &af.spans {
                prev_end = diag::log_span_submit(mcu_id, af.axis, span, prev_end);
            }
            let n = af.spans.len() as u32;
            let q = self.queues.get_mut(&key).expect("planned key exists");
            for _ in 0..af.spans.len() {
                let span = q
                    .spans
                    .pop_front()
                    .expect("sent frame outran its axis queue");
                q.wire_end_clock = Some(span.end_clock);
                if super::sched::is_hold_span(&span) {
                    q.wire_hold_tail += 1;
                } else {
                    q.wire_hold_tail = 0;
                    q.staged_motion = q.staged_motion.saturating_sub(1);
                }
                if let Some(history) = &self.history {
                    if let Err(error) = history.record(key, span) {
                        panic!(
                            "mcu{} axis{}: motion history rejected a dispatched span: {error}",
                            key.mcu_id, key.axis
                        );
                    }
                }
            }
            q.pushed = q.pushed.wrapping_add(n);
        }
    }

    // A send pass monopolizes the loop while its synchronous wire round-trips
    // run (~2 ms per EtherCAT bundle, ~20 ms per 1 KiB serial bundle at
    // 500 kbaud), while newly produced earlier-deadline views for another
    // axis wait in the data channel (observed: a 130 ms pass aged a z-hop
    // burst 53 ms into the MCU past). A wall-clock deadline bounds intake and
    // control latency identically on every transport; the deadline is checked
    // after each bundle, so every pass sends at least one.
    const SEND_PASS_BUDGET: Duration = Duration::from_millis(10);

    pub(super) fn send_ready(&mut self) -> Result<PassEnd, ()> {
        self.send_ready_until(Instant::now() + Self::SEND_PASS_BUDGET)
    }

    pub(super) fn send_ready_until(&mut self, pass_deadline: Instant) -> Result<PassEnd, ()> {
        let mut pass = PassEnd {
            sent: false,
            waiting_on_clock: false,
        };
        loop {
            self.resample_horizons();
            let sched = {
                let releasable_cap = |key: &AxisKey| {
                    if self
                        .cohort
                        .as_ref()
                        .is_some_and(|cohort| cohort.participants.contains(key))
                    {
                        1
                    } else {
                        usize::MAX
                    }
                };
                schedule(
                    &self.queues,
                    |mcu_id| self.sink.bundle_limits(mcu_id),
                    &self.horizons,
                    releasable_cap,
                )
            };
            if !matches!(sched, Schedule::StallFull(_)) {
                self.consumption_stall.reset();
            }
            match sched {
                Schedule::Idle => break,
                Schedule::StallFull(stall_key) => {
                    self.handle_stall_full(stall_key)?;
                    break;
                }
                Schedule::StallAhead(_stall_key) => {
                    pass.waiting_on_clock = true;
                    break;
                }
                Schedule::Send(frames) => {
                    pass.sent = true;
                    let mcu_id = frames[0].key.mcu_id;
                    let bundle = self.build_bundle(frames);
                    if !self.send_bundle_grouped(mcu_id, bundle)? {
                        break;
                    }
                }
            }
            if Instant::now() >= pass_deadline {
                break;
            }
        }
        Ok(pass)
    }

    /// `Ok(true)` while an endpoint still owes another window after this one;
    /// `Err(())` is fatal.
    fn drain_ticks(&mut self) -> Result<bool, ()> {
        match self.sink.drain_tick() {
            DrainTick::Quiet => Ok(false),
            DrainTick::Pending => Ok(true),
            DrainTick::Failed { mcu_id, error } => {
                tracing::error!(
                    subsystem = "motion",
                    event = "setpoint_drain_tick_failed",
                    mcu = mcu_id,
                    error = ?error,
                    "setpoint-ring drain tick failed — invoking fatal-transport action"
                );
                (self.callbacks.on_fatal_transport)(AxisKey { mcu_id, axis: 0 });
                Err(())
            }
        }
    }

    /// A bundle is atomic per endpoint, so a mixed-lane mcu (a pulse lane
    /// beside a phase lane) is shipped as one transaction per endpoint, each
    /// committed on its own answer. The uniform case — every mcu with a single
    /// endpoint — ships the bundle untouched.
    fn send_bundle_grouped(&mut self, mcu_id: u32, bundle: Vec<AxisFrame>) -> Result<bool, ()> {
        let group_of = |frame: &AxisFrame| {
            self.sink.lane_group(AxisKey {
                mcu_id,
                axis: frame.axis,
            })
        };
        let head = group_of(&bundle[0]);
        if bundle.iter().all(|frame| group_of(frame) == head) {
            return self.send_bundle(mcu_id, bundle);
        }
        let mut groups: BTreeMap<u8, Vec<AxisFrame>> = BTreeMap::new();
        for frame in bundle {
            groups.entry(group_of(&frame)).or_default().push(frame);
        }
        for (_, group) in groups {
            if !self.send_bundle(mcu_id, group)? {
                return Ok(false);
            }
        }
        Ok(true)
    }

    /// `Ok(true)` keeps the send pass going; `Ok(false)` ends it (the endpoint
    /// halted, or the transport gave no answer); `Err(())` is fatal.
    fn send_bundle(&mut self, mcu_id: u32, mut bundle: Vec<AxisFrame>) -> Result<bool, ()> {
        self.guard_spans_not_in_past(mcu_id, &mut bundle, "at send");
        let send_started_ns = super::transit_trace::trace_now_ns();
        let send_started_at = Instant::now();
        let outcome = self.send_bundle_logged(mcu_id, &bundle);
        let send_elapsed_ns = send_started_at.elapsed().as_nanos() as u64;
        let result = if outcome.is_ok() {
            mcu_protocol::result_codes::OK
        } else {
            super::transit_trace::transport_error_result()
        };
        for frame in &bundle {
            super::transit_trace::record(super::transit_trace::TransitTraceRecord {
                sequence: 0,
                mcu_id,
                axis: frame.axis,
                piece_count: frame.spans.len() as u32,
                room: frame.room,
                guard_recorded_ns: frame.guard_recorded_ns,
                guard_mcu_clock: frame.guard_mcu_clock,
                send_started_ns,
                send_elapsed_ns,
                host_front_start_time: frame.spans.first().map_or(0, |span| span.start_clock),
                result,
            });
        }
        match outcome {
            Ok(()) => {
                self.commit_sent_bundle(mcu_id, &bundle);
                Ok(true)
            }
            Err(SendError::Fatal(e)) => {
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
                Err(())
            }
            Err(SendError::Halted(e)) => {
                tracing::debug!(
                    subsystem = "motion",
                    event = "send_frame_halted",
                    mcu = mcu_id,
                    error = %e,
                    "pump frame met an endpoint halt and was discarded"
                );
                if let Err(error) = self.halt_keys(
                    bundle.iter().map(|frame| AxisKey {
                        mcu_id,
                        axis: frame.axis,
                    }),
                    HaltKind::Inferred(Instant::now()),
                ) {
                    tracing::error!(
                        subsystem = "motion",
                        event = "halt_cut_fatal",
                        mcu = mcu_id,
                        error = ?error,
                        "endpoint rejected the inferred halt cut"
                    );
                    (self.callbacks.on_fatal_transport)(AxisKey {
                        mcu_id,
                        axis: bundle.first().map_or(0, |frame| frame.axis),
                    });
                    return Err(());
                }
                Ok(false)
            }
            Err(SendError::Transient(e)) => {
                tracing::error!(
                    subsystem = "motion",
                    event = "send_frame_transient",
                    mcu = mcu_id,
                    error = %e,
                    "pump send_mcu_frames failed"
                );
                self.guard_spans_not_in_past(
                    mcu_id,
                    &mut bundle,
                    "after a failed send (transport gave no response \
                     while the view's scheduling lead ran out)",
                );
                Ok(false)
            }
        }
    }

    /// Block until a channel has something for the next intake pass, or until
    /// `timeout` elapses. Consumes nothing: `drain_control` and `drain_data`
    /// are the only readers, so the intake machine exists exactly once.
    fn park(
        &self,
        control_rx: &Receiver<PumpMsg>,
        data_rx: &Receiver<EnqueueMsg>,
        timeout: Option<Duration>,
    ) {
        let mut sel = Select::new();
        sel.recv(control_rx);
        if self.wants_more_data() {
            sel.recv(data_rx);
        }
        match timeout {
            Some(timeout) => {
                let _ = sel.ready_timeout(timeout);
            }
            None => {
                sel.ready();
            }
        }
    }

    pub(super) fn publish_ledger(&self) {
        let snapshot = self
            .queues
            .iter()
            .map(|(k, q)| {
                (
                    (k.mcu_id, k.axis),
                    crate::drain::AxisDrainState {
                        pending: q.spans.len() as u32,
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

            self.publish_ledger();
            for ack in self.pending_barrier_acks.drain(..) {
                let _ = ack.send(());
            }

            self.check_cohort_deadline();

            let pass = match self.send_ready() {
                Ok(pass) => pass,
                Err(()) => return,
            };
            activity |= pass.sent;

            let owes_window = match self.drain_ticks() {
                Ok(owes) => owes,
                Err(()) => return,
            };

            let unpushed: u64 = self.queues.values().map(|q| q.spans.len() as u64).sum();
            self.backlog.store(unpushed, Ordering::Release);

            if activity {
                continue;
            }

            self.park(
                control_rx,
                data_rx,
                self.wake_after(pass.waiting_on_clock || owes_window),
            );
        }
    }
}

pub fn run_pump<S: SpanSink>(
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
        horizons: ReleaseHorizons::default(),
        data_open: true,
        intake_batch_open: false,
        consumption_stall: ConsumptionStallWatch::new(CONSUMPTION_STALL_FATAL),
        mem_probe: MemPressureProbe::new(),
    };
    pump.run(control_rx, data_rx);
}
