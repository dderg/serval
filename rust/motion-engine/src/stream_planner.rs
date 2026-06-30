use std::collections::VecDeque;
use std::sync::Arc;
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, RecvTimeoutError, Sender, TrySendError, bounded};
use trajectory::{AxisChainSet, ShapedSegment};

use crate::pump::AxisKey;
use crate::stream::{StreamConfig, StreamError, StreamState};

const LEAD: f64 = crate::anchor::DEFAULT_LEAD_SECS;

#[cfg(test)]
pub(crate) fn lead_secs() -> f64 {
    LEAD
}

pub struct HomeDripParams {
    pub home_pos: [f64; 4],
    pub start: [f64; 3],
    pub axis: u8,
    pub direction: f64,
    pub speed_mm_s: f64,
    pub max_travel_mm: f64,
    pub cohort: u64,
    pub participants: Vec<AxisKey>,
    pub notify: crossbeam_channel::Sender<Result<(), String>>,
}

pub struct NudgeParams {
    pub mcu_id: u32,
    pub axis: u8,
    pub motor_mask: u8,
    pub delta_mm: f64,
    pub speed: f64,
    pub accel: f64,
    pub notify: crossbeam_channel::Sender<Result<(), String>>,
}

impl std::fmt::Debug for NudgeParams {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("NudgeParams")
            .field("mcu_id", &self.mcu_id)
            .field("axis", &self.axis)
            .field("motor_mask", &self.motor_mask)
            .field("delta_mm", &self.delta_mm)
            .field("speed", &self.speed)
            .field("accel", &self.accel)
            .finish_non_exhaustive()
    }
}

impl std::fmt::Debug for HomeDripParams {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("HomeDripParams")
            .field("home_pos", &self.home_pos)
            .field("start", &self.start)
            .field("axis", &self.axis)
            .field("direction", &self.direction)
            .field("speed_mm_s", &self.speed_mm_s)
            .field("max_travel_mm", &self.max_travel_mm)
            .field("cohort", &self.cohort)
            .field("participants", &self.participants)
            .finish_non_exhaustive()
    }
}

#[derive(Debug, thiserror::Error)]
pub enum DispatchError {
    #[error(
        "motion-engine: curve for mcu {mcu_id} exceeds caps \
         (pieces {pieces} > {max_pieces}); \
         logical-move splitting not yet implemented (Task 13 follow-up)."
    )]
    CapsExceeded {
        mcu_id: u32,
        pieces: usize,
        max_pieces: usize,
    },
    #[error("compute_ack_clock: {0}")]
    ComputeAckClock(String),
    #[error(
        "compute_ack_clock returned 0 after 5s — \
         clock-sync didn't establish for mcu {mcu_id} (mcu_h={mcu_handle:?})"
    )]
    ClockSyncTimeout {
        mcu_id: u32,
        mcu_handle: host_rt::passthrough_queue::McuHandle,
    },
    #[error("MCU {0}: connection dropped during dispatch")]
    ConnectionDropped(u32),
    #[error("piece pump thread is gone; cannot dispatch")]
    PumpGone,
    #[error(
        "no nominal clock frequency registered for mcu {0} \
         — set_nominal_clock_freq was not called"
    )]
    MissingNominalFreq(u32),
    #[error("nudge target mcu_id={mcu_id} axis={axis} not present in mcu_configs")]
    NudgeTargetMissing { mcu_id: u32, axis: u8 },
    #[error("motion-engine: failed to freeze reference clock for mcu {mcu_id}: {reason}")]
    ReferenceCaptureFailed { mcu_id: u32, reason: String },
}

// Producer-stall watermark budget. When the input goes quiet, materialize the
// brake-to-rest tail this long before the committed frontier would run dry. The
// total watermark is sized per the last barrier velocity at runtime —
// `t_brake(v_barrier)` (the jerk-limited decel ramp) plus this fixed solve-time
// budget for the final fit+plan+lower over the open tail plus `STALL_MARGIN` —
// so the decel always finishes before dispatch (no PieceStartInPast). If the
// locked lead at trigger time is already below the solve budget, the commit
// fails loud (BrakeToRestShortfall): the constant was sized too short.
const STALL_SOLVE_CONST: f64 = 0.05;
const STALL_MARGIN: f64 = 0.1;
const T_IDLE: Duration = Duration::from_secs(3600);
// Cap the moves coalesced into one commit. fit+plan+lower cost is ~linear in
// segments, so a single commit's wall-clock latency scales with the batch size.
// Draining all the way to `max_buffer_moves` builds 500+-move batches whose plan
// alone runs multiple seconds — longer than the MCU's buffered lead — so the
// ring drains to a fault (PieceStartInPast) while the planner is blocked in that
// one commit. A bounded batch keeps each commit well under the lead, so dispatch
// stays frequent, the pump backlog reflects reality, and the host's backlog
// throttle engages instead of flooding the channel. `max_buffer_moves` remains
// the memory backstop for the rare no-clean-seam case.
const COALESCE_BATCH_MOVES: usize = 64;

pub const INPUT_CHANNEL_CAP: usize = 8192;

type DispatchFn = Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync>;
type NudgeDispatchFn =
    Arc<dyn Fn(u32, &crate::nudge::NudgePiece) -> Result<(), DispatchError> + Send + Sync>;

#[derive(Debug)]
pub enum StreamMsg {
    Move(geometry::Move),
    Flush { notify: Sender<Option<Instant>> },
    Dwell { duration_s: f64, notify: Sender<()> },
    StreamOpen { home_pos: Vec<f64> },
    Reset { recovered_pos: Vec<f64> },
    SetAxisChains(AxisChainSet),
    HomeDrip(HomeDripParams),
    Nudge(NudgeParams),
    Shutdown,
}

#[allow(missing_debug_implementations)]
pub struct StreamPlannerHandle {
    sender: Sender<StreamMsg>,
    join_handle: Option<JoinHandle<()>>,
    last_move_time_bits: Arc<AtomicU64>,
    commit_fire_count: Arc<AtomicU32>,
    uncommitted_intake_secs: Arc<AtomicU64>,
}

#[derive(Debug)]
pub enum StreamPlannerError {
    ChannelClosed,
    ChannelFull,
}

impl std::fmt::Display for StreamPlannerError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::ChannelClosed => write!(f, "stream planner channel closed"),
            Self::ChannelFull => write!(
                f,
                "stream planner input channel full ({INPUT_CHANNEL_CAP} \
                 moves) — host backpressure gate bypassed"
            ),
        }
    }
}

impl std::error::Error for StreamPlannerError {}

impl StreamPlannerHandle {
    pub fn spawn(
        config: StreamConfig,
        axis_chains: AxisChainSet,
        home_pos: Vec<f64>,
        dispatch: DispatchFn,
        nudge_dispatch: NudgeDispatchFn,
    ) -> Self {
        let (tx, rx) = bounded(INPUT_CHANNEL_CAP);
        let last_move_time_bits = Arc::new(AtomicU64::new(0));
        let commit_fire_count = Arc::new(AtomicU32::new(0));
        let uncommitted_intake_secs = Arc::new(AtomicU64::new(0));

        let last_thread = Arc::clone(&last_move_time_bits);
        let commit_thread = Arc::clone(&commit_fire_count);
        let intake_thread = Arc::clone(&uncommitted_intake_secs);
        let join = thread::Builder::new()
            .name("kalico-stream-planner".to_string())
            .spawn(move || {
                let state = StreamState::new(config, axis_chains, &home_pos, 0.0);
                run_loop(
                    rx,
                    dispatch,
                    nudge_dispatch,
                    state,
                    &last_thread,
                    &commit_thread,
                    &intake_thread,
                );
            })
            .expect("spawn stream planner thread");

        Self {
            sender: tx,
            join_handle: Some(join),
            last_move_time_bits,
            commit_fire_count,
            uncommitted_intake_secs,
        }
    }

    pub fn submit_move(&self, m: geometry::Move) -> Result<(), StreamPlannerError> {
        try_submit_move(&self.sender, m)
    }

    pub fn pending_channel_moves(&self) -> usize {
        self.sender.len()
    }

    pub fn flush(&self) -> Result<(), StreamPlannerError> {
        let (tx, rx) = crossbeam_channel::bounded(1);
        self.sender
            .send(StreamMsg::Flush { notify: tx })
            .map_err(|_| StreamPlannerError::ChannelClosed)?;
        rx.recv()
            .map(|_committed_through| ())
            .map_err(|_| StreamPlannerError::ChannelClosed)
    }

    pub fn flush_start(
        &self,
    ) -> Result<crossbeam_channel::Receiver<Option<Instant>>, StreamPlannerError> {
        let (tx, rx) = crossbeam_channel::bounded(1);
        self.sender
            .send(StreamMsg::Flush { notify: tx })
            .map_err(|_| StreamPlannerError::ChannelClosed)?;
        Ok(rx)
    }

    pub fn dwell(&self, duration_s: f64) -> Result<(), StreamPlannerError> {
        let (tx, rx) = crossbeam_channel::bounded(1);
        self.sender
            .send(StreamMsg::Dwell {
                duration_s,
                notify: tx,
            })
            .map_err(|_| StreamPlannerError::ChannelClosed)?;
        rx.recv().map_err(|_| StreamPlannerError::ChannelClosed)
    }

    pub fn stream_open(&self, home_pos: Vec<f64>) -> Result<(), StreamPlannerError> {
        self.sender
            .send(StreamMsg::StreamOpen { home_pos })
            .map_err(|_| StreamPlannerError::ChannelClosed)
    }

    pub fn reset(&self, recovered_pos: Vec<f64>) -> Result<(), StreamPlannerError> {
        self.sender
            .send(StreamMsg::Reset { recovered_pos })
            .map_err(|_| StreamPlannerError::ChannelClosed)
    }

    pub fn update_axis_chains(&self, chains: AxisChainSet) -> Result<(), StreamPlannerError> {
        self.sender
            .send(StreamMsg::SetAxisChains(chains))
            .map_err(|_| StreamPlannerError::ChannelClosed)
    }

    pub fn home_drip(&self, p: HomeDripParams) -> Result<(), StreamPlannerError> {
        self.sender
            .send(StreamMsg::HomeDrip(p))
            .map_err(|_| StreamPlannerError::ChannelClosed)
    }

    pub fn submit_nudge(&self, p: NudgeParams) -> Result<(), StreamPlannerError> {
        self.sender
            .send(StreamMsg::Nudge(p))
            .map_err(|_| StreamPlannerError::ChannelClosed)
    }

    #[must_use]
    pub fn last_move_time(&self) -> f64 {
        f64::from_bits(self.last_move_time_bits.load(Ordering::Acquire))
    }

    #[must_use]
    pub fn commit_fire_count(&self) -> u32 {
        self.commit_fire_count.load(Ordering::Acquire)
    }

    #[must_use]
    pub fn uncommitted_intake_secs(&self) -> f64 {
        f64::from_bits(self.uncommitted_intake_secs.load(Ordering::Acquire))
    }

    pub fn shutdown(&mut self) {
        let _ = self.sender.send(StreamMsg::Shutdown);
        if let Some(h) = self.join_handle.take() {
            let _ = h.join();
        }
    }
}

impl Drop for StreamPlannerHandle {
    fn drop(&mut self) {
        if self.join_handle.is_some() {
            self.shutdown();
        }
    }
}

fn try_submit_move(
    sender: &Sender<StreamMsg>,
    m: geometry::Move,
) -> Result<(), StreamPlannerError> {
    sender.try_send(StreamMsg::Move(m)).map_err(|e| match e {
        TrySendError::Full(_) => StreamPlannerError::ChannelFull,
        TrySendError::Disconnected(_) => StreamPlannerError::ChannelClosed,
    })
}

fn nominal_t(m: &geometry::Move) -> Option<f64> {
    if m.feedrate_mm_s > 0.0 {
        Some(m.segment.s_len() / m.feedrate_mm_s)
    } else {
        None
    }
}

struct IntakeTally<'a> {
    per_move: VecDeque<f64>,
    secs: f64,
    atomic: &'a AtomicU64,
}

impl<'a> IntakeTally<'a> {
    fn new(atomic: &'a AtomicU64) -> Self {
        Self {
            per_move: VecDeque::new(),
            secs: 0.0,
            atomic,
        }
    }

    fn publish(&self) {
        self.atomic.store(self.secs.to_bits(), Ordering::Release);
    }

    fn record_intake(&mut self, m: &geometry::Move) {
        let Some(dt) = nominal_t(m) else {
            fatal(&format!(
                "intake tally: non-positive feedrate {} on line {}",
                m.feedrate_mm_s, m.source.start_line
            ));
        };
        self.per_move.push_back(dt);
        self.secs += dt;
        self.publish();
    }

    fn subtract_committed(&mut self, popped: usize) {
        for _ in 0..popped {
            match self.per_move.pop_front() {
                Some(dt) => self.secs -= dt,
                None => fatal("intake tally underflow: committed more moves than recorded"),
            }
        }
        if self.per_move.is_empty() {
            self.secs = 0.0;
        }
        self.publish();
    }

    fn reset(&mut self) {
        self.per_move.clear();
        self.secs = 0.0;
        self.publish();
    }
}

fn fatal(msg: &str) -> ! {
    tracing::error!(
        subsystem = "motion",
        event = "stream_planner_fatal",
        error = msg,
        "stream planner encountered an unrecoverable error — aborting"
    );
    eprintln!("kalico stream planner fatal: {msg}");
    std::thread::sleep(Duration::from_millis(100));
    std::process::abort();
}

fn dispatch_committed(
    segs: &[ShapedSegment],
    dispatch: &DispatchFn,
    sync_instant: &mut Option<Instant>,
    last_move_time_bits: &AtomicU64,
    commit_fire_count: &AtomicU32,
) {
    if segs.is_empty() {
        return;
    }
    tracing::info!(
        subsystem = "motion",
        event = "dispatch_committed",
        n = segs.len(),
        t_start = segs[0].t_start,
        t_end = segs[segs.len() - 1].t_end,
        "[dispatch-committed]"
    );
    for s in segs {
        let n_ax = s.axes.len();
        tracing::info!(
            subsystem = "motion",
            event = "pipe_dispatch",
            line = s.source_line,
            t_us = crate::timing::mono_us(),
            seg_t_start = s.t_start,
            seg_t_end = s.t_end,
            x_end = if n_ax > 0 {
                nurbs::eval::eval(&s.axes[0], s.t_end)
            } else {
                0.0
            },
            y_end = if n_ax > 1 {
                nurbs::eval::eval(&s.axes[1], s.t_end)
            } else {
                0.0
            },
            z_start = if n_ax > 2 {
                nurbs::eval::eval(&s.axes[2], s.t_start)
            } else {
                0.0
            },
            z_end = if n_ax > 2 {
                nurbs::eval::eval(&s.axes[2], s.t_end)
            } else {
                0.0
            },
            "[pipe] dispatch"
        );
        if let Err(e) = dispatch(s) {
            fatal(&format!("dispatch failed: {e}"));
        }
    }
    if sync_instant.is_none() {
        *sync_instant = Some(Instant::now());
    }
    if let Some(last) = segs.last() {
        last_move_time_bits.store(last.t_end.to_bits(), Ordering::Release);
    }
    commit_fire_count.fetch_add(1, Ordering::AcqRel);
}

fn dispatch_or_err(
    segs: &[ShapedSegment],
    dispatch: &DispatchFn,
    sync_instant: &mut Option<Instant>,
    last_move_time_bits: &AtomicU64,
    commit_fire_count: &AtomicU32,
) -> Result<(), String> {
    if segs.is_empty() {
        return Ok(());
    }
    for s in segs {
        dispatch(s).map_err(|e| format!("dispatch failed: {e}"))?;
    }
    if sync_instant.is_none() {
        *sync_instant = Some(Instant::now());
    }
    if let Some(last) = segs.last() {
        last_move_time_bits.store(last.t_end.to_bits(), Ordering::Release);
    }
    commit_fire_count.fetch_add(1, Ordering::AcqRel);
    Ok(())
}

fn run_home_drip(
    state: &mut StreamState,
    p: &HomeDripParams,
    dispatch: &DispatchFn,
    sync_instant: &mut Option<Instant>,
    last_move_time_bits: &AtomicU64,
    commit_fire_count: &AtomicU32,
) -> Result<(), String> {
    let travel = p.direction * p.max_travel_mm;
    let (dx, dy, dz) = match p.axis {
        0 => (travel, 0.0, 0.0),
        1 => (0.0, travel, 0.0),
        2 => (0.0, 0.0, travel),
        other => return Err(format!("HomeDrip: unsupported axis {other} (only 0/1/2)")),
    };
    let m =
        crate::classify::build_move(p.start, dx, dy, dz, 0, 0.0, state.limits(), p.speed_mm_s, 0)
            .map_err(|e| format!("HomeDrip build_move: {e:?}"))?;
    state.push(m).map_err(|e| format!("HomeDrip push: {e}"))?;
    let segs = state
        .commit(true)
        .map_err(|e| format!("HomeDrip commit: {e}"))?;
    dispatch_or_err(
        &segs,
        dispatch,
        sync_instant,
        last_move_time_bits,
        commit_fire_count,
    )
    .map_err(|e| format!("HomeDrip {e}"))
}

fn run_nudge(
    state: &mut StreamState,
    p: &NudgeParams,
    dispatch: &DispatchFn,
    nudge_dispatch: &NudgeDispatchFn,
    sync_instant: &mut Option<Instant>,
    last_move_time_bits: &AtomicU64,
    commit_fire_count: &AtomicU32,
) -> Result<(), String> {
    let segs = state
        .commit(true)
        .map_err(|e| format!("nudge drain: {e}"))?;
    dispatch_or_err(
        &segs,
        dispatch,
        sync_instant,
        last_move_time_bits,
        commit_fire_count,
    )
    .map_err(|e| format!("nudge drain {e}"))?;
    let t_base = state.t_committed();
    let nudge_segs = crate::nudge::plan_nudge_profile(
        p.axis,
        p.delta_mm,
        p.speed,
        p.accel,
        p.motor_mask,
        t_base,
    )?;
    let total_dur: f64 = nudge_segs
        .iter()
        .map(|s| s.piece.u_end - s.piece.u_start)
        .sum();
    for s in &nudge_segs {
        nudge_dispatch(p.mcu_id, s).map_err(|e| format!("nudge dispatch: {e}"))?;
    }
    state.advance_time(total_dur);
    last_move_time_bits.store(state.t_committed().to_bits(), Ordering::Release);
    Ok(())
}

/// Re-anchor the stream timeline if the buffer drained and the machine caught
/// up (an idle gap), then buffer the move. Re-anchoring only fires on the first
/// move of a batch — once the buffer is non-empty, subsequent coalesced moves
/// just push.
fn handle_move_arrival(
    state: &mut StreamState,
    m: geometry::Move,
    sync_instant: &mut Option<Instant>,
    tally: &mut IntakeTally,
) -> Result<(), StreamError> {
    tracing::info!(
        subsystem = "motion",
        event = "pipe_ingress",
        line = m.source.start_line,
        t_us = crate::timing::mono_us(),
        buffered = state.buffered(),
        "[pipe] ingress"
    );
    let esc = sync_instant.map_or(0.0, |t| t.elapsed().as_secs_f64());
    let reanchor = state.is_empty() && esc > state.t_committed() + 1e-6;
    if reanchor {
        tracing::info!(
            subsystem = "motion",
            event = "reanchor_decision",
            esc,
            t_committed = state.t_committed(),
            "[reanchor-decision]"
        );
        *sync_instant = None;
        state.restart_idle_timeline();
    }
    tally.record_intake(&m);
    state.push(m)
}

/// Handle one non-move control message. Returns `true` when the loop should
/// exit (shutdown).
fn handle_control(
    msg: StreamMsg,
    state: &mut StreamState,
    dispatch: &DispatchFn,
    nudge_dispatch: &NudgeDispatchFn,
    sync_instant: &mut Option<Instant>,
    last_move_time_bits: &AtomicU64,
    commit_fire_count: &AtomicU32,
    tally: &mut IntakeTally,
) -> bool {
    match msg {
        StreamMsg::Move(_) => unreachable!("moves handled by the coalescing path"),
        StreamMsg::Flush { notify } => {
            let segs = state
                .commit(true)
                .unwrap_or_else(|e| fatal(&format!("flush: {e}")));
            dispatch_committed(
                &segs,
                dispatch,
                sync_instant,
                last_move_time_bits,
                commit_fire_count,
            );
            tally.reset();
            let finish = sync_instant.map(|t| {
                t + Duration::try_from_secs_f64((state.t_committed() + LEAD).max(0.0))
                    .unwrap_or(Duration::ZERO)
            });
            let _ = notify.send(finish);
        }
        StreamMsg::Dwell { duration_s, notify } => {
            let segs = state
                .commit(true)
                .unwrap_or_else(|e| fatal(&format!("dwell drain: {e}")));
            dispatch_committed(
                &segs,
                dispatch,
                sync_instant,
                last_move_time_bits,
                commit_fire_count,
            );
            tally.reset();
            state.advance_time(duration_s);
            let _ = notify.send(());
        }
        StreamMsg::StreamOpen { home_pos } => {
            *sync_instant = None;
            state.reset(&home_pos, 0.0);
            tally.reset();
        }
        StreamMsg::Reset { recovered_pos } => {
            *sync_instant = None;
            state.reset(&recovered_pos, 0.0);
            tally.reset();
        }
        StreamMsg::SetAxisChains(chains) => {
            state.set_axis_chains(chains);
        }
        StreamMsg::HomeDrip(p) => {
            *sync_instant = None;
            state.reset(&p.home_pos, 0.0);
            tally.reset();
            let result = run_home_drip(
                state,
                &p,
                dispatch,
                sync_instant,
                last_move_time_bits,
                commit_fire_count,
            );
            let _ = p.notify.send(result);
        }
        StreamMsg::Nudge(p) => {
            let result = run_nudge(
                state,
                &p,
                dispatch,
                nudge_dispatch,
                sync_instant,
                last_move_time_bits,
                commit_fire_count,
            );
            tally.reset();
            let _ = p.notify.send(result);
        }
        StreamMsg::Shutdown => {
            let segs = state
                .commit(true)
                .unwrap_or_else(|e| fatal(&format!("shutdown drain: {e}")));
            dispatch_committed(
                &segs,
                dispatch,
                sync_instant,
                last_move_time_bits,
                commit_fire_count,
            );
            tally.reset();
            return true;
        }
    }
    false
}

fn run_loop(
    rx: Receiver<StreamMsg>,
    dispatch: DispatchFn,
    nudge_dispatch: NudgeDispatchFn,
    mut state: StreamState,
    last_move_time_bits: &AtomicU64,
    commit_fire_count: &AtomicU32,
    uncommitted_intake_secs: &AtomicU64,
) {
    let mut sync_instant: Option<Instant> = None;
    let mut tally = IntakeTally::new(uncommitted_intake_secs);

    loop {
        let watermark = state.stall_brake_time() + STALL_SOLVE_CONST + STALL_MARGIN;
        let next_timeout = if state.is_empty() {
            T_IDLE
        } else {
            let esc = sync_instant.map_or(0.0, |t| t.elapsed().as_secs_f64());
            let remaining = (state.t_committed() + LEAD - watermark) - esc;
            Duration::try_from_secs_f64(remaining.max(0.0)).unwrap_or(Duration::ZERO)
        };

        let msg = match rx.recv_timeout(next_timeout) {
            Ok(m) => m,
            Err(RecvTimeoutError::Timeout) => {
                let esc = sync_instant.map_or(0.0, |t| t.elapsed().as_secs_f64());
                let lead_remaining = (state.t_committed() + LEAD) - esc;
                tracing::info!(
                    subsystem = "motion",
                    event = "idle_drain",
                    buffered = state.buffered(),
                    t_committed = state.t_committed(),
                    lead_remaining,
                    v_barrier = state.last_v_barrier(),
                    sync_set = sync_instant.is_some(),
                    "[idle-drain] producer-stall brake-to-rest"
                );
                let segs = state
                    .commit_stall_brake(lead_remaining, STALL_SOLVE_CONST)
                    .unwrap_or_else(|e| fatal(&format!("stall brake-to-rest: {e}")));
                dispatch_committed(
                    &segs,
                    &dispatch,
                    &mut sync_instant,
                    last_move_time_bits,
                    commit_fire_count,
                );
                tally.reset();
                continue;
            }
            Err(RecvTimeoutError::Disconnected) => return,
        };

        match msg {
            StreamMsg::Move(m) => {
                handle_move_arrival(&mut state, m, &mut sync_instant, &mut tally)
                    .unwrap_or_else(|e| fatal(&format!("ingress: {e}")));
                // Coalesce the burst up to COALESCE_BATCH_MOVES, then fit that
                // batch ONCE. Committing per move re-fits the growing buffer each
                // time (O(n²)); one fit per bounded batch is O(n) and keeps each
                // commit's latency under the MCU's buffered lead so dispatch stays
                // continuous instead of stalling in one multi-second mega-commit.
                let mut deferred: Option<StreamMsg> = None;
                let coalesce_cap = COALESCE_BATCH_MOVES.min(state.max_buffer_moves());
                while state.buffered() < coalesce_cap {
                    match rx.try_recv() {
                        Ok(StreamMsg::Move(m2)) => {
                            handle_move_arrival(&mut state, m2, &mut sync_instant, &mut tally)
                                .unwrap_or_else(|e| fatal(&format!("ingress: {e}")));
                        }
                        Ok(other) => {
                            deferred = Some(other);
                            break;
                        }
                        Err(_) => break,
                    }
                }
                let buffered_before = state.buffered();
                tracing::info!(
                    subsystem = "motion",
                    event = "coalesce_done",
                    channel_pending = rx.len(),
                    buffered = buffered_before,
                    uncommitted_secs = tally.secs,
                    coalesce_cap,
                    t_us = crate::timing::mono_us(),
                    "[intake] coalesced batch ready to commit; channel_pending = moves submitted but not yet pulled (invisible to backpressure)"
                );
                let segs = state
                    .commit(false)
                    .unwrap_or_else(|e| fatal(&format!("commit: {e}")));
                tally.subtract_committed(buffered_before - state.buffered());
                dispatch_committed(
                    &segs,
                    &dispatch,
                    &mut sync_instant,
                    last_move_time_bits,
                    commit_fire_count,
                );
                if segs.is_empty() && !state.is_empty() {
                    // No committable seam this batch (small buffer wholly within
                    // the brake setback). The idle-drain timeout never fires while
                    // moves keep arriving — each `recv` resets it — so a throttled
                    // trickle would freeze the frontier into MCU starvation. When
                    // the delivered lead has drained below the stall watermark,
                    // force the brake-to-rest now instead of waiting for silence.
                    let esc = sync_instant.map_or(0.0, |t| t.elapsed().as_secs_f64());
                    let lead_remaining = (state.t_committed() + LEAD) - esc;
                    if lead_remaining < watermark {
                        tracing::info!(
                            subsystem = "motion",
                            event = "thin_lead_drain",
                            buffered = state.buffered(),
                            t_committed = state.t_committed(),
                            lead_remaining,
                            watermark,
                            v_barrier = state.last_v_barrier(),
                            "[thin-lead-drain] uncommittable buffer, lead draining — brake-to-rest"
                        );
                        let segs = state
                            .commit_stall_brake(lead_remaining, STALL_SOLVE_CONST)
                            .unwrap_or_else(|e| fatal(&format!("thin-lead drain: {e}")));
                        dispatch_committed(
                            &segs,
                            &dispatch,
                            &mut sync_instant,
                            last_move_time_bits,
                            commit_fire_count,
                        );
                        tally.reset();
                    }
                }
                if state.buffered() >= state.max_buffer_moves() {
                    // Backstop only: no committable seam within reach (e.g. one
                    // move longer than the whole look-ahead window). Drain to
                    // rest so motion keeps flowing and memory stays bounded.
                    tracing::info!(
                        subsystem = "motion",
                        event = "buffer_cap_drain",
                        buffered = state.buffered(),
                        t_committed = state.t_committed(),
                        "[buffer-cap-drain] no committable seam — draining to rest"
                    );
                    let segs = state
                        .commit(true)
                        .unwrap_or_else(|e| fatal(&format!("buffer-cap drain: {e}")));
                    dispatch_committed(
                        &segs,
                        &dispatch,
                        &mut sync_instant,
                        last_move_time_bits,
                        commit_fire_count,
                    );
                    tally.reset();
                }
                if let Some(other) = deferred {
                    if handle_control(
                        other,
                        &mut state,
                        &dispatch,
                        &nudge_dispatch,
                        &mut sync_instant,
                        last_move_time_bits,
                        commit_fire_count,
                        &mut tally,
                    ) {
                        return;
                    }
                }
            }
            other => {
                if handle_control(
                    other,
                    &mut state,
                    &dispatch,
                    &nudge_dispatch,
                    &mut sync_instant,
                    last_move_time_bits,
                    commit_fire_count,
                    &mut tally,
                ) {
                    return;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests;
