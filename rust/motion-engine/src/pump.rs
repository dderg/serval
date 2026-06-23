use runtime::piece_ring::PieceEntry;
use std::collections::{BTreeMap, BTreeSet, HashMap, VecDeque};
use std::sync::Arc;
use std::sync::Weak;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{Receiver, RecvError, RecvTimeoutError};
use std::sync::{LazyLock, Mutex};
use std::time::{Duration, Instant};

/// Per-frame `[transit-diag]` healthy-path logging on the pump send thread
/// throttled delivery below real-time on dense streams (the structured write +
/// `SystemTime` + format ran synchronously between transport pushes). The alert
/// path still fires every frame; the healthy lead sample is emitted once per
/// this many frames to keep coarse delivery-lead telemetry at negligible cost.
const TRANSIT_DIAG_HEALTHY_SAMPLE_STRIDE: u64 = 64;
static TRANSIT_DIAG_FRAME_SEQ: AtomicU64 = AtomicU64::new(0);

/// Previous `PushPieces` frame on one axis, kept solely so the per-frame
/// transit diagnostic can decompose where dispatch lead is being spent:
/// `Δarrival_lead = schedule_advance − mcu_clock_advance`. Separating the
/// host's planned advance (`front_start_time` delta) from the MCU clock's
/// real-time advance (`arrival_clock` delta), and bracketing both with the
/// wall-clock send gap and the blocking-call duration, tells late-arrival
/// apart from host-pacing starvation, transport stall, and MCU clock burn.
struct PrevTransitFrame {
    send_instant: Instant,
    front_start_time: u64,
    arrival_clock: u64,
    arrival_lead_ticks: i64,
}

static TRANSIT_PREV_FRAME: LazyLock<Mutex<HashMap<AxisKey, PrevTransitFrame>>> =
    LazyLock::new(|| Mutex::new(HashMap::new()));

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug, PartialOrd, Ord)]
pub struct AxisKey {
    pub mcu_id: u32,
    pub axis: u8,
}

#[derive(Debug)]
pub struct AxisQueue {
    pub pieces: VecDeque<(PieceEntry, f64)>,
    pub pushed: u32,
    pub retired: u32,
    pub ring_depth: u32,
    pub physical_write_cursor: u32,
    pub lead_secs: f64,
}

impl AxisQueue {
    pub fn new(ring_depth: u32) -> Self {
        Self {
            pieces: VecDeque::new(),
            pushed: 0,
            retired: 0,
            ring_depth,
            physical_write_cursor: 0,
            lead_secs: MAX_LEAD_SECS,
        }
    }
    pub fn room(&self) -> u32 {
        let in_flight = self.pushed.wrapping_sub(self.retired);
        self.ring_depth.saturating_sub(in_flight)
    }
    pub fn advance_write_cursor(&mut self, n: u32) {
        if self.ring_depth == 0 {
            return;
        }
        self.physical_write_cursor = (self.physical_write_cursor + n) % self.ring_depth;
    }
}

#[derive(Debug)]
pub struct FramePlan {
    pub key: AxisKey,
    pub pieces: Vec<PieceEntry>,
    pub start_slot: u16,
}

/// One axis' pieces within a single-MCU bundle, carrying the ring bookkeeping
/// the transport needs. `schedule()` only ever groups axes of one MCU into a
/// `Send`, so a slice of these is exactly the work for one MCU transaction.
pub struct AxisFrame {
    pub axis: u8,
    pub pieces: Vec<PieceEntry>,
    pub start_slot: u16,
    pub new_head: u32,
    pub room: u32,
}

#[derive(Debug)]
pub enum Schedule {
    Send(Vec<FramePlan>),
    StallFull(AxisKey),
    StallAhead(AxisKey),
    Idle,
}

/// Select the globally earliest-host-time piece across all queues, then emit
/// the same-MCU prefix as one frame batch.
///
/// ## Invariants preserved
///
/// 1. **StallFull gates all MCUs.** When the globally-earliest piece belongs to
///    a ring-full queue, no other queue may advance — cross-MCU issue-side
///    coherence requires that a hardware-blocked MCU is never overtaken.
///
/// 2. **Cap-gated queues are skipped, not globally-gating.** When the
///    globally-earliest piece has `releasable_cap_of == 0`, that queue is
///    excluded from this round and the search continues among remaining
///    queues in host-time order. A horizon-blocked head, by contrast,
///    returns `StallAhead` immediately.
///
/// 3. **Ordering must use host time.** `start_time` values are raw MCU clock
///    ticks in per-MCU clock domains (H7: ~520 MHz / own epoch; F446: ~180 MHz /
///    own epoch). Comparing ticks across MCUs is meaningless; F446 values are
///    numerically smaller and would always win, starving the H7.  The `f64`
///    sidecar in each `(PieceEntry, f64)` pair carries the minting host time
///    (`t0 + u_start`, seconds) and is the only valid cross-queue ordering key.
#[must_use]
pub fn schedule(
    queues: &BTreeMap<AxisKey, AxisQueue>,
    max_per_frame: usize,
    horizon_of: impl Fn(&AxisKey, &AxisQueue) -> Option<u64>,
    releasable_cap_of: impl Fn(&AxisKey) -> usize,
) -> Schedule {
    let mut stall_ahead_candidate: Option<AxisKey> = None;
    let mut cap_skipped: BTreeSet<AxisKey> = BTreeSet::new();

    let head_key = loop {
        let candidate = queues
            .iter()
            .filter(|(k, q)| !q.pieces.is_empty() && !cap_skipped.contains(*k))
            .min_by(|(ka, qa), (kb, qb)| {
                let ha = qa.pieces.front().unwrap().1;
                let hb = qb.pieces.front().unwrap().1;
                ha.total_cmp(&hb).then(ka.cmp(kb))
            });
        let (&k, q) = match candidate {
            None => {
                if let Some(k) = stall_ahead_candidate {
                    return Schedule::StallAhead(k);
                }
                return Schedule::Idle;
            }
            Some(c) => c,
        };

        if q.room() == 0 {
            return Schedule::StallFull(k);
        }

        if releasable_cap_of(&k) == 0 {
            if stall_ahead_candidate.is_none() {
                stall_ahead_candidate = Some(k);
            }
            cap_skipped.insert(k);
            continue;
        }

        let head_start_ticks = q.pieces.front().unwrap().0.start_time;
        if let Some(horizon) = horizon_of(&k, q) {
            if head_start_ticks > horizon {
                return Schedule::StallAhead(k);
            }
        }

        break k;
    };

    let mut taken: BTreeMap<AxisKey, usize> = BTreeMap::new();
    let mut maxed: BTreeSet<AxisKey> = cap_skipped;
    loop {
        let next = queues
            .iter()
            .filter_map(|(k, q)| {
                if maxed.contains(k) {
                    return None;
                }
                let already = taken.get(k).copied().unwrap_or(0);
                q.pieces
                    .get(already)
                    .map(|&(ref p, host)| (*k, p.start_time, host))
            })
            .min_by(|(ka, _, ha), (kb, _, hb)| ha.total_cmp(hb).then(ka.cmp(kb)));
        let (k, start_ticks, _host) = match next {
            Some(n) => n,
            None => break,
        };
        if k.mcu_id != head_key.mcu_id {
            break;
        }
        let already = taken.get(&k).copied().unwrap_or(0);
        let q = &queues[&k];
        let room = q.room() as usize;
        let cap = releasable_cap_of(&k);
        if already >= room || already >= max_per_frame || already >= cap {
            maxed.insert(k);
            continue;
        }
        if let Some(horizon) = horizon_of(&k, q) {
            if start_ticks > horizon {
                if stall_ahead_candidate.is_none() {
                    stall_ahead_candidate = Some(k);
                }
                maxed.insert(k);
                continue;
            }
        }
        *taken.entry(k).or_insert(0) += 1;
    }

    if taken.is_empty() {
        if let Some(k) = stall_ahead_candidate {
            return Schedule::StallAhead(k);
        }
        return Schedule::StallFull(head_key);
    }

    let frames: Vec<FramePlan> = taken
        .into_iter()
        .filter(|(_, n)| *n > 0)
        .map(|(k, n)| FramePlan {
            key: k,
            pieces: queues[&k].pieces.iter().take(n).map(|(p, _)| *p).collect(),
            start_slot: 0,
        })
        .collect();
    debug_assert!(!frames.is_empty());
    Schedule::Send(frames)
}

#[cfg(test)]
mod sched_tests;

#[cfg(test)]
mod tests;

#[cfg(test)]
mod drip_tests;

pub const DRIP_WINDOW_SECS: f64 = 0.100;

pub struct DripArm {
    pub cohort: u64,
    pub participants: Vec<AxisKey>,
    pub timeout: Duration,
}

pub struct EnqueueMsg {
    pub key: AxisKey,
    /// Each entry pairs the `PieceEntry` with its minting host time (`t0 + u_start`, seconds).
    /// The host time is the ordering key used by `schedule()`; the raw `start_time` tick is
    /// MCU-clock-domain-specific and incomparable across MCUs.
    pub pieces: Vec<(PieceEntry, f64)>,
    pub fresh_stream: bool,
    pub lead_secs: f64,
}

pub struct HeartbeatMsg {
    pub mcu_id: u32,
    pub retired_counts: Vec<u32>,
}

pub enum PumpMsg {
    Enqueue(EnqueueMsg),
    Heartbeat(HeartbeatMsg),
    Flush(Vec<AxisKey>),
    DripArm(DripArm),
    DripDisarm(u64),
    Barrier(std::sync::mpsc::SyncSender<()>),
    Shutdown,
}

/// Error from [`PieceSink::send_frame`].
///
/// `Fatal` means the transport is permanently broken and the caller must not
/// retry — the process should abort or restart.  `Transient` means the frame
/// was not delivered but the transport may recover; the caller can back off and
/// retry.
#[derive(Debug)]
pub enum SendError {
    /// Unrecoverable transport failure (broken pipe, connection reset, peer
    /// closed).  Callers that receive this must invoke their fatal-fault action
    /// immediately; retrying will not help.
    Fatal(String),
    /// Recoverable or non-transport error (MCU rejected the frame, ring full,
    /// etc.).
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

/// Compute the tick-domain and host-domain gaps at a batch boundary.
///
/// Returns `(tick_jump_us, host_jump_us)` where negative values indicate overlap.
/// The difference between the two isolates clock-projection error from planner-intent gaps.
pub fn junction_jumps(
    first_start_ticks: u64,
    first_host: f64,
    prev_end_ticks: u64,
    prev_end_host: f64,
    approx_freq_hz: f64,
) -> (f64, f64) {
    let tick_jump_us =
        (first_start_ticks as i64 - prev_end_ticks as i64) as f64 / approx_freq_hz * 1e6;
    let host_jump_us = (first_host - prev_end_host) * 1e6;
    (tick_jump_us, host_jump_us)
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

#[derive(Clone, Copy)]
struct JunctionEnd {
    end_ticks: u64,
    end_host: f64,
    end_pos: f32,
}

pub const JUNCTION_POSITION_LOG_MM: f32 = 0.0125;
pub const JUNCTION_POSITION_FATAL_MM: f32 = 0.1;

/// A position jump between the last dispatched piece and the next one is the
/// host-side image of MCU fault -300/-310 (the step burst that killed the
/// Trident two-jog test, 2026-06-10): the MCU will emit |Δpos|·steps_per_mm
/// steps in one 25µs sample into a 32-deep queue. Dying here, before the push,
/// preserves both endpoint values; the MCU fault preserves neither.
fn check_junction_position_continuity(
    key: AxisKey,
    prev_end_pos: f32,
    next_start_pos: f32,
    prev_end_host: f64,
    next_start_host: f64,
) {
    let jump = (next_start_pos - prev_end_pos).abs();
    if jump >= JUNCTION_POSITION_FATAL_MM {
        panic!(
            "junction position discontinuity on mcu{} axis{}: prev piece ends at \
             {prev_end_pos} (host t={prev_end_host:.6}), next starts at \
             {next_start_pos} (host t={next_start_host:.6}), |Δ|={jump}mm — \
             this becomes a one-sample step burst on the MCU (fault -300/-310)",
            key.mcu_id, key.axis
        );
    }
    if jump >= JUNCTION_POSITION_LOG_MM {
        tracing::error!(
            subsystem = "motion",
            event = "junction_position_discontinuity",
            key = ?key,
            prev_end = prev_end_pos,
            next_start = next_start_pos,
            jump_mm = jump,
            prev_end_host = prev_end_host,
            next_start_host = next_start_host,
            "[junction-pos] position discontinuity"
        );
    }
}

struct DripCohort {
    id: u64,
    participants: BTreeSet<AxisKey>,
    timeout: Duration,
    baseline: BTreeMap<AxisKey, u32>,
    last_retired: BTreeMap<AxisKey, u32>,
    step_deadline: Instant,
    deadline_floor: u32,
}

impl DripCohort {
    fn executed(&self, k: &AxisKey, queues: &BTreeMap<AxisKey, AxisQueue>) -> u32 {
        let retired = queues.get(k).map_or(0, |q| q.retired);
        let baseline = self.baseline.get(k).copied().unwrap_or(0);
        retired.wrapping_sub(baseline)
    }

    fn floor(&self, queues: &BTreeMap<AxisKey, AxisQueue>) -> u32 {
        self.participants
            .iter()
            .map(|k| self.executed(k, queues))
            .min()
            .unwrap_or(0)
    }
}

/// Run the piece pump loop.
///
/// `on_fatal_transport` is called (at most once) when [`SendError::Fatal`] is
/// returned by the sink, indicating an unrecoverable transport failure.  The
/// production call site passes an `abort()`-based action; tests inject a
/// channel-send or a flag-set so they can assert detection without terminating
/// the process.  After the callback returns the pump loop exits.
#[allow(clippy::too_many_lines)]
pub fn run_pump<S, F, C, A, O, D>(
    rx: Receiver<PumpMsg>,
    sink: S,
    ring_depth_of: F,
    mcu_clock_of: C,
    on_fatal_transport: A,
    on_abandon: O,
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
    let mut queues: BTreeMap<AxisKey, AxisQueue> = BTreeMap::new();
    let mut junction_ends: BTreeMap<AxisKey, JunctionEnd> = BTreeMap::new();
    let mut cohort: Option<DripCohort> = None;
    const MAX_PER_FRAME: usize = 32;

    let apply = |msg: PumpMsg,
                 queues: &mut BTreeMap<AxisKey, AxisQueue>,
                 junction_ends: &mut BTreeMap<AxisKey, JunctionEnd>,
                 cohort: &mut Option<DripCohort>|
     -> bool {
        match msg {
            PumpMsg::Shutdown => return false,
            PumpMsg::Flush(keys) => {
                for key in keys {
                    if let Some(q) = queues.get_mut(&key) {
                        let dropped = q.pieces.len() as u32;
                        q.pieces.clear();
                        if dropped > 0 {
                            on_abandon(key, dropped);
                        }
                    }
                    junction_ends.remove(&key);
                }
            }
            PumpMsg::Enqueue(EnqueueMsg {
                key,
                pieces,
                fresh_stream,
                lead_secs,
            }) => {
                if let Some(co) = cohort.as_ref() {
                    if !co.participants.contains(&key) {
                        let id = co.id;
                        on_drip_stall(format!(
                            "drip cohort {id}: enqueue for non-participant \
                             mcu{} axis{} during active cohort — homing must \
                             drip every axis",
                            key.mcu_id, key.axis
                        ));
                        *cohort = None;
                        return true;
                    }
                }
                if fresh_stream {
                    junction_ends.remove(&key);
                }
                if !pieces.is_empty() {
                    // Clock not yet synced — skip junction bookkeeping; µs math is
                    // meaningless without a real frequency and end_time needs it too.
                    if let Some((_ack_now, freq)) = mcu_clock_of(key.mcu_id) {
                        let (first_entry, first_host) = &pieces[0];
                        if first_entry.motor_mask == 0 {
                            if let Some(&JunctionEnd {
                                end_ticks: prev_end_ticks,
                                end_host: prev_end_host,
                                end_pos: prev_end_pos,
                            }) = junction_ends.get(&key)
                            {
                                check_junction_position_continuity(
                                    key,
                                    prev_end_pos,
                                    first_entry.coeffs[0],
                                    prev_end_host,
                                    *first_host,
                                );
                                let (tick_jump_us, host_jump_us) = junction_jumps(
                                    first_entry.start_time,
                                    *first_host,
                                    prev_end_ticks,
                                    prev_end_host,
                                    freq,
                                );
                                let anomalous = tick_jump_us < -50.0
                                    || (tick_jump_us - host_jump_us).abs() > 50.0;
                                if fresh_stream || !anomalous {
                                    tracing::debug!(
                                        subsystem = "motion",
                                        event = "junction_jump",
                                        key = ?key,
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
                                        key = ?key,
                                        tick_jump_us,
                                        host_jump_us,
                                        fresh = fresh_stream,
                                        reason,
                                        "[junction] anomalous jump"
                                    );
                                }
                            }
                            let (last_entry, last_host) = pieces.last().unwrap();
                            #[allow(clippy::cast_possible_truncation)]
                            let last_end_ticks = last_entry.end_time(freq as f32);
                            let last_end_host = last_host + last_entry.duration as f64;
                            junction_ends.insert(
                                key,
                                JunctionEnd {
                                    end_ticks: last_end_ticks,
                                    end_host: last_end_host,
                                    end_pos: last_entry.coeffs[3],
                                },
                            );
                        }
                    }
                }
                let q = queues
                    .entry(key)
                    .or_insert_with(|| AxisQueue::new(ring_depth_of(key)));
                q.lead_secs = lead_secs;
                q.pieces.extend(pieces);
            }
            PumpMsg::Heartbeat(HeartbeatMsg {
                mcu_id,
                retired_counts,
            }) => {
                // retired_counts[i] is axis index i; same numbering as PushPieces.axis_idx in runtime_ffi.rs — do not reorder either side.
                for (axis, &c) in retired_counts.iter().enumerate() {
                    let key = AxisKey {
                        mcu_id,
                        axis: axis as u8,
                    };
                    if let Some(q) = queues.get_mut(&key) {
                        q.retired = c;
                    }
                    if let Some(co) = cohort {
                        if co.participants.contains(&key) {
                            let prev = co.last_retired.get(&key).copied().unwrap_or(0);
                            if c < prev {
                                on_drip_stall(format!(
                                    "drip cohort {}: retired regression on mcu{} axis{}: \
                                     was {prev} now {c} — MCU retired counter must not decrease",
                                    co.id, mcu_id, axis
                                ));
                                *cohort = None;
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
                    let retired = queues.get(&k).map_or(0, |q| q.retired);
                    baseline.insert(k, retired);
                    last_retired.insert(k, retired);
                }
                let step_deadline = Instant::now() + arm.timeout;
                *cohort = Some(DripCohort {
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
                if cohort.as_ref().map_or(false, |co| co.id == c) {
                    *cohort = None;
                }
            }
            PumpMsg::Barrier(ack) => {
                let _ = ack.send(());
            }
        }
        true
    };

    let horizon_of = |k: &AxisKey, q: &AxisQueue, cohort: &Option<DripCohort>| -> Option<u64> {
        match mcu_clock_of(k.mcu_id) {
            Some((ack_now, freq)) =>
            {
                #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
                Some(ack_now + (q.lead_secs * freq) as u64)
            }
            None if cohort
                .as_ref()
                .map_or(false, |co| co.participants.contains(k)) =>
            {
                Some(0)
            }
            None => None,
        }
    };

    let mut holding_ahead = false;

    loop {
        let cohort_active = cohort.is_some();
        let short_lead = (holding_ahead || cohort_active)
            && queues
                .values()
                .any(|q| q.lead_secs < 0.1 && !q.pieces.is_empty());
        let poll_ms: u64 = if short_lead || cohort_active { 10 } else { 50 };

        let first = if holding_ahead || cohort_active {
            match rx.recv_timeout(Duration::from_millis(poll_ms)) {
                Ok(m) => Some(m),
                Err(RecvTimeoutError::Timeout) => None,
                Err(RecvTimeoutError::Disconnected) => return,
            }
        } else {
            match rx.recv() {
                Ok(m) => Some(m),
                Err(RecvError) => return,
            }
        };

        if let Some(msg) = first {
            if !apply(msg, &mut queues, &mut junction_ends, &mut cohort) {
                return;
            }
            while let Ok(m) = rx.try_recv() {
                if !apply(m, &mut queues, &mut junction_ends, &mut cohort) {
                    return;
                }
            }
        }

        if let Some(ref co) = cohort {
            let now = Instant::now();
            let floor = co.floor(&queues);
            if floor > co.deadline_floor {
                let co = cohort.as_mut().unwrap();
                co.step_deadline = now + co.timeout;
                co.deadline_floor = floor;
            } else if now >= co.step_deadline {
                let co = cohort.as_ref().unwrap();
                let lagging: Vec<String> = co
                    .participants
                    .iter()
                    .map(|k| {
                        format!(
                            "mcu{} axis{}: executed {} queued {}",
                            k.mcu_id,
                            k.axis,
                            co.executed(k, &queues),
                            queues.get(k).map_or(0, |q| q.pieces.len()),
                        )
                    })
                    .collect();
                let id = co.id;
                on_drip_stall(format!(
                    "drip cohort {id}: floor stalled at {floor} for {:?}; \
                     participants: [{}]",
                    co.timeout,
                    lagging.join(", ")
                ));
                cohort = None;
            }
        }

        holding_ahead = false;
        'send: loop {
            let hz_of = |k: &AxisKey, q: &AxisQueue| horizon_of(k, q, &cohort);
            match schedule(&queues, MAX_PER_FRAME, hz_of, |_| usize::MAX) {
                Schedule::Idle => break 'send,
                Schedule::StallFull(_stall_key) => {
                    break 'send;
                }
                Schedule::StallAhead(_stall_key) => {
                    holding_ahead = true;
                    break 'send;
                }
                Schedule::Send(frames) => {
                    if frames.is_empty() {
                        break 'send;
                    }
                    let mcu_id = frames[0].key.mcu_id;
                    let bundle: Vec<AxisFrame> = frames
                        .into_iter()
                        .map(|f| {
                            let n = f.pieces.len() as u32;
                            let q = queues.get(&f.key).expect("planned key exists");
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
                        .collect();
                    match sink.send_mcu_frames(mcu_id, &bundle) {
                        Ok(()) => {
                            for af in &bundle {
                                let key = AxisKey {
                                    mcu_id,
                                    axis: af.axis,
                                };
                                let n = af.pieces.len() as u32;
                                let q = queues.get_mut(&key).expect("planned key exists");
                                for _ in 0..af.pieces.len() {
                                    q.pieces.pop_front();
                                }
                                q.pushed = q.pushed.wrapping_add(n);
                                q.advance_write_cursor(n);
                            }
                        }
                        Err(SendError::Fatal(ref e)) => {
                            tracing::error!(
                                subsystem = "motion",
                                event = "send_frame_fatal",
                                mcu = mcu_id,
                                error = %e,
                                "pump send_mcu_frames FATAL transport error — invoking fatal-transport action"
                            );
                            on_fatal_transport(AxisKey {
                                mcu_id,
                                axis: bundle.first().map_or(0, |f| f.axis),
                            });
                            return;
                        }
                        Err(SendError::Transient(ref e)) => {
                            tracing::error!(
                                subsystem = "motion",
                                event = "send_frame_transient",
                                mcu = mcu_id,
                                error = %e,
                                "pump send_mcu_frames failed"
                            );
                            break 'send;
                        }
                    }
                }
            }
        }

        let unpushed: u64 = queues.values().map(|q| q.pieces.len() as u64).sum();
        backlog.store(unpushed, Ordering::Release);
    }
}

pub enum McuTransport {
    Serial(Weak<host_rt::host_io::McuHostIo>),
    EtherCat(Weak<host_rt::mcu_serial_conn::McuSerialConn>),
}

impl std::fmt::Debug for McuTransport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Serial(_) => write!(f, "McuTransport::Serial"),
            Self::EtherCat(_) => write!(f, "McuTransport::EtherCat"),
        }
    }
}

pub struct WireSink {
    pub transports: HashMap<u32, McuTransport>,
    pub timeout: Duration,
    /// Live per-MCU clock frequency (Hz), queried from the router per frame.
    /// `None` while the MCU clock is not yet synced. Single source of truth for
    /// the `[transit-diag]` µs conversion — no second freq table.
    pub freq_of: Arc<dyn Fn(u32) -> Option<f64> + Send + Sync>,
}

impl WireSink {
    /// Call `PushPieces` on the transport for the given axis.
    ///
    /// Returns `Err(SendError::Fatal(...))` for EtherCAT transport errors that
    /// represent permanent connection loss (`Closed` or `Io`); all other
    /// failures map to `Err(SendError::Transient(...))`.
    ///
    /// The reader thread owns all socket reads; `WouldBlock`/`TimedOut` never
    /// surface here.  `TransportError::Timeout` (deadline exhausted on the
    /// caller's `recv_timeout`) is transient — the session may still be alive.
    fn call_push_pieces(
        &self,
        mcu_id: u32,
        frames: &[AxisFrame],
    ) -> Result<mcu_protocol::messages::PushPiecesResponse, SendError> {
        use host_rt::transport::TransportError;

        let axes: Vec<mcu_protocol::messages::AxisPieces> = frames
            .iter()
            .map(|f| {
                let mut pieces_bytes = Vec::with_capacity(f.pieces.len() * 32);
                for p in &f.pieces {
                    pieces_bytes.extend_from_slice(&p.to_le_bytes());
                }
                mcu_protocol::messages::AxisPieces {
                    axis_idx: f.axis,
                    piece_count: f.pieces.len() as u8,
                    start_slot: f.start_slot,
                    new_head: f.new_head,
                    pieces_bytes,
                }
            })
            .collect();
        let msg = mcu_protocol::messages::PushPieces { axes };
        let body = mcu_protocol::codec::Encode::encoded_to_vec(&msg);

        let transport = self.transports.get(&mcu_id).ok_or_else(|| {
            SendError::Transient(format!(
                "WireSink: no transport for mcu_id {mcu_id}; \
                     this is a logic bug in init_planner — the MCU was enqueued \
                     without registering its transport"
            ))
        })?;

        let resp_body = match transport {
            McuTransport::Serial(weak) => {
                let io = weak.upgrade().ok_or_else(|| {
                    SendError::Transient(format!("McuHostIo for mcu {mcu_id} detached"))
                })?;
                let (_kind, b) = io
                    .kalico_call_on_channel(
                        mcu_protocol::MCU_CHANNEL_PIECES,
                        mcu_protocol::MessageKind::PushPieces,
                        body,
                        self.timeout,
                    )
                    .map_err(|e| {
                        SendError::Transient(format!("serial PushPieces mcu {mcu_id}: {e:?}"))
                    })?;
                b
            }
            McuTransport::EtherCat(weak) => {
                let conn = weak.upgrade().ok_or_else(|| {
                    // The endpoint conn was released (last strong Arc dropped):
                    // session is gone, treat as fatal so the pump exits rather
                    // than spinning on a dead axis.
                    SendError::Fatal(format!(
                        "ethercat conn for mcu {mcu_id} detached (released)"
                    ))
                })?;
                let (_kind, b) = conn
                    .kalico_call_on_channel(
                        mcu_protocol::MCU_CHANNEL_PIECES,
                        mcu_protocol::MessageKind::PushPieces,
                        body,
                        self.timeout,
                    )
                    .map_err(|e| {
                        if matches!(&e, TransportError::Closed | TransportError::Io(_)) {
                            SendError::Fatal(format!("ethercat PushPieces mcu {mcu_id}: {e:?}"))
                        } else {
                            SendError::Transient(format!("ethercat PushPieces mcu {mcu_id}: {e:?}"))
                        }
                    })?;
                b
            }
        };

        use mcu_protocol::codec::Decode as _;
        mcu_protocol::messages::PushPiecesResponse::decode(&resp_body).map_err(|e| {
            SendError::Transient(format!("decode PushPiecesResponse mcu {mcu_id}: {e:?}"))
        })
    }

    /// Emit the per-axis transit diagnostic for one axis of a just-completed
    /// frame. `front_start_time` is this axis' echo from the response;
    /// `arrival_clock` is the frame-global MCU clock; `send_started_at` /
    /// `send_elapsed_us` belong to the whole MCU round-trip.
    #[allow(clippy::too_many_arguments)]
    fn emit_transit_diag(
        &self,
        key: AxisKey,
        host_front_start_time: u64,
        piece_count: usize,
        room: u32,
        send_started_at: Instant,
        send_elapsed_us: f64,
        front_start_time: u64,
        arrival_clock: u64,
    ) {
        let arrival_lead_ticks = front_start_time as i64 - arrival_clock as i64;
        let zero_st = host_front_start_time == 0;
        let past_arrival = arrival_lead_ticks < 0;
        let frame_seq = TRANSIT_DIAG_FRAME_SEQ.fetch_add(1, Ordering::Relaxed);
        let healthy_sample = frame_seq % TRANSIT_DIAG_HEALTHY_SAMPLE_STRIDE == 0;

        let prev = {
            let mut map = TRANSIT_PREV_FRAME.lock().expect("transit prev-frame map");
            map.insert(
                key,
                PrevTransitFrame {
                    send_instant: send_started_at,
                    front_start_time,
                    arrival_clock,
                    arrival_lead_ticks,
                },
            )
        };

        if !(zero_st || past_arrival || healthy_sample) {
            return;
        }
        let approx_freq_hz = (self.freq_of)(key.mcu_id);
        let host_send_secs = {
            use std::time::{SystemTime, UNIX_EPOCH};
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .map(|d| d.as_secs_f64())
                .unwrap_or(0.0)
        };
        // Clock not yet synced -> the µs conversion is meaningless; render N/A.
        // Alert gating uses arrival_lead_ticks (tick domain), so the ALERT still
        // fires without a frequency.
        let arrival_lead_us = approx_freq_hz
            .map(|f| format!("{:.1}", (arrival_lead_ticks as f64 / f) * 1e6))
            .unwrap_or_else(|| "N/A".to_owned());

        let ticks_to_us = |ticks: i64| {
            approx_freq_hz
                .map(|f| format!("{:.1}", (ticks as f64 / f) * 1e6))
                .unwrap_or_else(|| "N/A".to_owned())
        };
        // Where did the lead go since this axis' previous frame?
        //   Δarrival_lead = schedule_advance − mcu_clock_advance
        // schedule_advance: how much further ahead the host planned.
        // mcu_clock_advance: how much the MCU clock (real time) moved.
        // send_gap: wall time between our two sends to this axis.
        let (send_gap_us, schedule_advance_us, mcu_clock_advance_us, delta_arrival_lead_us) =
            match &prev {
                Some(p) => (
                    format!(
                        "{:.1}",
                        send_started_at.duration_since(p.send_instant).as_secs_f64() * 1e6
                    ),
                    ticks_to_us(front_start_time as i64 - p.front_start_time as i64),
                    ticks_to_us(arrival_clock as i64 - p.arrival_clock as i64),
                    ticks_to_us(arrival_lead_ticks - p.arrival_lead_ticks),
                ),
                None => (
                    "N/A".to_owned(),
                    "N/A".to_owned(),
                    "N/A".to_owned(),
                    "N/A".to_owned(),
                ),
            };
        if zero_st || past_arrival {
            let alert = if zero_st && past_arrival {
                "host_start_time=0 (clock-sync gap) AND piece in MCU past"
            } else if zero_st {
                "host_start_time=0 (router clock_freq=0 at dispatch — clock-sync gap)"
            } else {
                "piece arrived in MCU past (arrival_lead<0) — PieceStartInPast risk"
            };
            tracing::warn!(
                subsystem = "motion",
                event = "transit_diag_alert",
                mcu = key.mcu_id,
                axis = key.axis,
                host_front_start_time,
                mcu_front_start_time = front_start_time,
                arrival_clock,
                arrival_lead_ticks,
                arrival_lead_us = %arrival_lead_us,
                host_send_unix_secs = host_send_secs,
                send_elapsed_us,
                send_gap_us = %send_gap_us,
                schedule_advance_us = %schedule_advance_us,
                mcu_clock_advance_us = %mcu_clock_advance_us,
                delta_arrival_lead_us = %delta_arrival_lead_us,
                piece_count,
                room,
                alert,
                "[transit-diag] alert"
            );
        } else {
            tracing::info!(
                subsystem = "motion",
                event = "transit_diag",
                mcu = key.mcu_id,
                axis = key.axis,
                host_front_start_time,
                mcu_front_start_time = front_start_time,
                arrival_clock,
                arrival_lead_ticks,
                arrival_lead_us = %arrival_lead_us,
                host_send_unix_secs = host_send_secs,
                send_elapsed_us,
                send_gap_us = %send_gap_us,
                schedule_advance_us = %schedule_advance_us,
                mcu_clock_advance_us = %mcu_clock_advance_us,
                delta_arrival_lead_us = %delta_arrival_lead_us,
                piece_count,
                room,
                "[transit-diag]"
            );
        }
    }
}

impl PieceSink for WireSink {
    /// Single-axis convenience — the pump drives WireSink via `send_mcu_frames`;
    /// this exists only to satisfy the trait and routes through the same path.
    fn send_frame(
        &self,
        key: AxisKey,
        pieces: &[PieceEntry],
        start_slot: u16,
        new_head: u32,
        room: u32,
    ) -> Result<i32, SendError> {
        let frame = AxisFrame {
            axis: key.axis,
            pieces: pieces.to_vec(),
            start_slot,
            new_head,
            room,
        };
        self.send_mcu_frames(key.mcu_id, std::slice::from_ref(&frame))
            .map(|()| mcu_protocol::result_codes::OK)
    }

    fn send_mcu_frames(&self, mcu_id: u32, frames: &[AxisFrame]) -> Result<(), SendError> {
        debug_assert!(
            frames.iter().all(|f| f.pieces.len() <= 255),
            "PushPieces axis block exceeds u8 piece_count; schedule() must cap at MAX_PER_FRAME"
        );

        let send_started_at = Instant::now();
        let resp = self.call_push_pieces(mcu_id, frames)?;
        let send_elapsed_us = send_started_at.elapsed().as_secs_f64() * 1e6;

        // Per-axis transit-diag from the response's per-axis echo against the
        // frame-global arrival clock. Emitted even on a fatal frame — a negative
        // arrival_lead is exactly the PieceStartInPast signature we want logged.
        for f in frames {
            let Some(diag) = resp.axes.iter().find(|a| a.axis_idx == f.axis) else {
                continue;
            };
            let key = AxisKey {
                mcu_id,
                axis: f.axis,
            };
            let host_front_start_time = f.pieces.first().map(|p| p.start_time).unwrap_or(0);
            self.emit_transit_diag(
                key,
                host_front_start_time,
                f.pieces.len(),
                f.room,
                send_started_at,
                send_elapsed_us,
                diag.front_start_time,
                resp.arrival_clock,
            );
        }

        if resp.result != mcu_protocol::result_codes::OK {
            return Err(SendError::retryable_mcu_reject(mcu_id, resp.result));
        }
        Ok(())
    }
}

#[cfg(test)]
mod wire_sink_tests;
