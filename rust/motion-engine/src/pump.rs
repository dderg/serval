use crossbeam_channel::{Receiver, Select, TryRecvError};
use runtime::piece_ring::PieceEntry;
use std::collections::{BTreeMap, BTreeSet, HashMap, VecDeque};
use std::sync::Arc;
use std::sync::Weak;
use std::sync::atomic::{AtomicU64, Ordering};
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
        if in_flight > self.ring_depth {
            self.ring_depth
        } else {
            self.ring_depth - in_flight
        }
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
                let host_a = qa.pieces.front().unwrap().1;
                let host_b = qb.pieces.front().unwrap().1;
                host_a.total_cmp(&host_b).then(ka.cmp(kb))
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
    pub pieces: Vec<(PieceEntry, f64)>,
    pub fresh_stream: bool,
    pub lead_secs: f64,
    pub source_line: u32,
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
// Sized several times the total MCU ring cache (≈1024 pieces/MCU): the host
// staging buffer must be DEEPER than the MCU rings, or the pump throttles the
// planner before the frontier is deep enough to absorb host scheduling gaps —
// the playhead then overruns the committed end (anchor_underrun → drive fault).
const PUMP_INTAKE_BACKLOG_CAP: u64 = 16384;

#[derive(Clone, Copy)]
struct JunctionEnd {
    end_ticks: u64,
    end_host: f64,
    end_pos: f32,
    source_line: u32,
}

pub const JUNCTION_POSITION_LOG_MM: f32 = 0.0125;
pub const JUNCTION_POSITION_FATAL_MM: f32 = 0.1;

#[derive(Clone, Copy, Debug)]
pub struct JunctionSeam {
    pub key: AxisKey,
    pub prev_end_pos: f32,
    pub next_start_pos: f32,
    pub prev_end_host: f64,
    pub next_start_host: f64,
    pub prev_source_line: u32,
    pub next_source_line: u32,
    pub prev_end_ticks: u64,
    pub first_start_ticks: u64,
}

impl JunctionSeam {
    #[must_use]
    pub fn jump(&self) -> f32 {
        (self.next_start_pos - self.prev_end_pos).abs()
    }

    #[must_use]
    pub fn is_fatal(&self) -> bool {
        self.jump() >= JUNCTION_POSITION_FATAL_MM
    }
}

#[derive(Default)]
pub struct JunctionTracker {
    ends: BTreeMap<AxisKey, JunctionEnd>,
}

impl JunctionTracker {
    pub fn forget(&mut self, key: AxisKey) {
        self.ends.remove(&key);
    }

    pub fn observe(
        &mut self,
        key: AxisKey,
        pieces: &[(PieceEntry, f64)],
        source_line: u32,
        freq: f64,
    ) -> Option<JunctionSeam> {
        let (first_entry, first_host) = pieces.first()?;
        if first_entry.motor_mask != 0 {
            return None;
        }
        let seam = self.ends.get(&key).map(|prev| JunctionSeam {
            key,
            prev_end_pos: prev.end_pos,
            next_start_pos: first_entry.coeffs[0],
            prev_end_host: prev.end_host,
            next_start_host: *first_host,
            prev_source_line: prev.source_line,
            next_source_line: source_line,
            prev_end_ticks: prev.end_ticks,
            first_start_ticks: first_entry.start_time,
        });
        let (last_entry, last_host) = pieces.last().unwrap();
        #[allow(clippy::cast_possible_truncation)]
        let last_end_ticks = last_entry.end_time(freq as f32);
        let last_end_host = last_host + last_entry.duration as f64;
        self.ends.insert(
            key,
            JunctionEnd {
                end_ticks: last_end_ticks,
                end_host: last_end_host,
                end_pos: last_entry.coeffs[3],
                source_line,
            },
        );
        seam
    }

    pub fn observe_msg(
        &mut self,
        key: AxisKey,
        pieces: &[(PieceEntry, f64)],
        fresh_stream: bool,
        source_line: u32,
        freq: Option<f64>,
    ) -> Option<JunctionSeam> {
        if fresh_stream {
            self.forget(key);
        }
        self.observe(key, pieces, source_line, freq?)
    }
}

fn check_junction_position_continuity(seam: &JunctionSeam) {
    let jump = seam.jump();
    if jump >= JUNCTION_POSITION_LOG_MM {
        tracing::error!(
            subsystem = "motion",
            event = "junction_position_discontinuity",
            key = ?seam.key,
            fatal = jump >= JUNCTION_POSITION_FATAL_MM,
            prev_end = seam.prev_end_pos,
            next_start = seam.next_start_pos,
            jump_mm = jump,
            prev_end_host = seam.prev_end_host,
            next_start_host = seam.next_start_host,
            prev_source_line = seam.prev_source_line,
            next_source_line = seam.next_source_line,
            "[junction-pos] position discontinuity"
        );
    }
    if jump >= JUNCTION_POSITION_FATAL_MM {
        panic!(
            "junction position discontinuity on mcu{} axis{}: prev piece ends at \
             {} (host t={:.6}, line {}), next starts at {} (host t={:.6}, line \
             {}), |Δ|={jump}mm — this becomes a one-sample step burst on the MCU \
             (fault -300/-310)",
            seam.key.mcu_id,
            seam.key.axis,
            seam.prev_end_pos,
            seam.prev_end_host,
            seam.prev_source_line,
            seam.next_start_pos,
            seam.next_start_host,
            seam.next_source_line,
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

#[allow(clippy::too_many_lines)]
pub fn run_pump<S, F, C, A, O, D>(
    control_rx: Receiver<PumpMsg>,
    data_rx: Receiver<EnqueueMsg>,
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
    let mut junctions = JunctionTracker::default();
    let mut cohort: Option<DripCohort> = None;
    const MAX_PER_FRAME: usize = 32;

    let apply = |msg: PumpMsg,
                 queues: &mut BTreeMap<AxisKey, AxisQueue>,
                 junctions: &mut JunctionTracker,
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
                    junctions.forget(key);
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

    let apply_enqueue = |msg: EnqueueMsg,
                         queues: &mut BTreeMap<AxisKey, AxisQueue>,
                         junctions: &mut JunctionTracker,
                         cohort: &mut Option<DripCohort>| {
        let EnqueueMsg {
            key,
            pieces,
            fresh_stream,
            lead_secs,
            source_line,
        } = msg;
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
                return;
            }
        }
        if fresh_stream {
            junctions.forget(key);
        }
        if !pieces.is_empty() {
            if let Some((_ack_now, freq)) = mcu_clock_of(key.mcu_id) {
                if let Some(seam) = junctions.observe(key, &pieces, source_line, freq) {
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
        let q = queues
            .entry(key)
            .or_insert_with(|| AxisQueue::new(ring_depth_of(key)));
        q.lead_secs = lead_secs;
        q.pieces.extend(pieces);
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
    let mut data_open = true;
    let mut last_stallfull_log: Option<std::time::Instant> = None;

    loop {
        let cohort_active = cohort.is_some();
        let short_lead = (holding_ahead || cohort_active)
            && queues
                .values()
                .any(|q| q.lead_secs < 0.1 && !q.pieces.is_empty());
        let poll_ms: u64 = if short_lead || cohort_active { 10 } else { 50 };

        let mut activity = false;

        loop {
            match control_rx.try_recv() {
                Ok(m) => {
                    activity = true;
                    if !apply(m, &mut queues, &mut junctions, &mut cohort) {
                        return;
                    }
                }
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => return,
            }
        }

        while data_open && (cohort.is_some() || wants_pieces(&queues)) {
            match data_rx.try_recv() {
                Ok(e) => {
                    activity = true;
                    apply_enqueue(e, &mut queues, &mut junctions, &mut cohort);
                }
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => {
                    data_open = false;
                    break;
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
                Schedule::StallFull(stall_key) => {
                    let now = std::time::Instant::now();
                    let due = last_stallfull_log
                        .is_none_or(|t| now.duration_since(t) >= Duration::from_secs(1));
                    if due {
                        last_stallfull_log = Some(now);
                        if let Some(q) = queues.get(&stall_key) {
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
                    }
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
                    activity = true;
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

        if activity {
            continue;
        }

        let mut sel = Select::new();
        let ctrl_op = sel.recv(&control_rx);
        let want_data = data_open && (cohort.is_some() || wants_pieces(&queues));
        let data_op = if want_data {
            sel.recv(&data_rx)
        } else {
            usize::MAX
        };
        let selected = if holding_ahead || cohort.is_some() {
            sel.select_timeout(Duration::from_millis(poll_ms))
        } else {
            Ok(sel.select())
        };
        if let Ok(op) = selected {
            let idx = op.index();
            if idx == ctrl_op {
                match op.recv(&control_rx) {
                    Ok(m) => {
                        if !apply(m, &mut queues, &mut junctions, &mut cohort) {
                            return;
                        }
                    }
                    Err(_) => return,
                }
            } else if idx == data_op {
                match op.recv(&data_rx) {
                    Ok(e) => apply_enqueue(e, &mut queues, &mut junctions, &mut cohort),
                    Err(_) => data_open = false,
                }
            }
        }
    }
}

fn wants_pieces(queues: &BTreeMap<AxisKey, AxisQueue>) -> bool {
    let staged: u64 = queues.values().map(|q| q.pieces.len() as u64).sum();
    staged < PUMP_INTAKE_BACKLOG_CAP
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
    pub freq_of: Arc<dyn Fn(u32) -> Option<f64> + Send + Sync>,
}

impl WireSink {
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
