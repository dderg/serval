use std::sync::Arc;
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, RecvTimeoutError, Sender, unbounded};
use trajectory::ShapedSegment;

use crate::planner::{DispatchError, HomeDripParams, NudgeParams};
use crate::stream::{StreamConfig, StreamState};

const LEAD: f64 = crate::anchor::DEFAULT_LEAD_SECS;
const SAFETY_MARGIN: f64 = 0.05;
const T_IDLE: Duration = Duration::from_secs(3600);

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
}

#[derive(Debug)]
pub enum StreamPlannerError {
    ChannelClosed,
}

impl std::fmt::Display for StreamPlannerError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::ChannelClosed => write!(f, "stream planner channel closed"),
        }
    }
}

impl std::error::Error for StreamPlannerError {}

impl StreamPlannerHandle {
    pub fn spawn(
        config: StreamConfig,
        home_pos: Vec<f64>,
        dispatch: DispatchFn,
        nudge_dispatch: NudgeDispatchFn,
    ) -> Self {
        let (tx, rx) = unbounded();
        let last_move_time_bits = Arc::new(AtomicU64::new(0));
        let commit_fire_count = Arc::new(AtomicU32::new(0));

        let last_thread = Arc::clone(&last_move_time_bits);
        let commit_thread = Arc::clone(&commit_fire_count);
        let join = thread::Builder::new()
            .name("kalico-stream-planner".to_string())
            .spawn(move || {
                let state = StreamState::new(config, &home_pos, 0.0);
                run_loop(
                    rx,
                    dispatch,
                    nudge_dispatch,
                    state,
                    &last_thread,
                    &commit_thread,
                );
            })
            .expect("spawn stream planner thread");

        Self {
            sender: tx,
            join_handle: Some(join),
            last_move_time_bits,
            commit_fire_count,
        }
    }

    pub fn submit_move(&self, m: geometry::Move) -> Result<(), StreamPlannerError> {
        self.sender
            .send(StreamMsg::Move(m))
            .map_err(|_| StreamPlannerError::ChannelClosed)
    }

    pub fn flush(&self) -> Result<(), StreamPlannerError> {
        let (tx, rx) = crossbeam_channel::bounded(1);
        self.sender
            .send(StreamMsg::Flush { notify: tx })
            .map_err(|_| StreamPlannerError::ChannelClosed)?;
        match rx.recv() {
            Ok(finish) => {
                if let Some(deadline) = finish {
                    let now = Instant::now();
                    if deadline > now {
                        std::thread::sleep(deadline - now);
                    }
                }
                Ok(())
            }
            Err(_) => Err(StreamPlannerError::ChannelClosed),
        }
    }

    /// Non-blocking flush: returns a receiver that yields the play-out
    /// completion deadline (`None` if the stream was idle). Mirrors the old
    /// planner's `flush_start`/`wait_moves_poll` protocol.
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
    for s in segs {
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
    state.push(m);
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

fn run_loop(
    rx: Receiver<StreamMsg>,
    dispatch: DispatchFn,
    nudge_dispatch: NudgeDispatchFn,
    mut state: StreamState,
    last_move_time_bits: &AtomicU64,
    commit_fire_count: &AtomicU32,
) {
    let mut sync_instant: Option<Instant> = None;

    loop {
        let next_timeout = if state.is_empty() {
            T_IDLE
        } else {
            let esc = sync_instant.map_or(0.0, |t| t.elapsed().as_secs_f64());
            let remaining = (state.t_committed() + LEAD - SAFETY_MARGIN) - esc;
            Duration::try_from_secs_f64(remaining.max(0.0)).unwrap_or(Duration::ZERO)
        };

        let msg = match rx.recv_timeout(next_timeout) {
            Ok(m) => m,
            Err(RecvTimeoutError::Timeout) => {
                tracing::info!(
                    subsystem = "motion",
                    event = "idle_drain",
                    buffered = state.buffered(),
                    t_committed = state.t_committed(),
                    sync_set = sync_instant.is_some(),
                    "[idle-drain]"
                );
                let segs = state
                    .commit(true)
                    .unwrap_or_else(|e| fatal(&format!("idle drain: {e}")));
                dispatch_committed(
                    &segs,
                    &dispatch,
                    &mut sync_instant,
                    last_move_time_bits,
                    commit_fire_count,
                );
                continue;
            }
            Err(RecvTimeoutError::Disconnected) => return,
        };

        match msg {
            StreamMsg::Move(m) => {
                let esc = sync_instant.map_or(0.0, |t| t.elapsed().as_secs_f64());
                let reanchor = state.is_empty() && esc > state.t_committed() + 1e-6;
                tracing::info!(
                    subsystem = "motion",
                    event = "reanchor_decision",
                    esc,
                    sync_set = sync_instant.is_some(),
                    is_empty = state.is_empty(),
                    buffered = state.buffered(),
                    t_committed = state.t_committed(),
                    reanchor,
                    "[reanchor-decision]"
                );
                if reanchor {
                    sync_instant = None;
                    state.restart_idle_timeline();
                }
                state.push(m);
                let segs = state
                    .commit(false)
                    .unwrap_or_else(|e| fatal(&format!("commit: {e}")));
                dispatch_committed(
                    &segs,
                    &dispatch,
                    &mut sync_instant,
                    last_move_time_bits,
                    commit_fire_count,
                );
                if state.buffered() >= state.max_buffer_moves() {
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
                }
            }
            StreamMsg::Flush { notify } => {
                let segs = state
                    .commit(true)
                    .unwrap_or_else(|e| fatal(&format!("flush: {e}")));
                dispatch_committed(
                    &segs,
                    &dispatch,
                    &mut sync_instant,
                    last_move_time_bits,
                    commit_fire_count,
                );
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
                    &dispatch,
                    &mut sync_instant,
                    last_move_time_bits,
                    commit_fire_count,
                );
                state.advance_time(duration_s);
                let _ = notify.send(());
            }
            StreamMsg::StreamOpen { home_pos } => {
                sync_instant = None;
                state.reset(&home_pos, 0.0);
            }
            StreamMsg::Reset { recovered_pos } => {
                sync_instant = None;
                state.reset(&recovered_pos, 0.0);
            }
            StreamMsg::HomeDrip(p) => {
                sync_instant = None;
                state.reset(&p.home_pos, 0.0);
                let result = run_home_drip(
                    &mut state,
                    &p,
                    &dispatch,
                    &mut sync_instant,
                    last_move_time_bits,
                    commit_fire_count,
                );
                let _ = p.notify.send(result);
            }
            StreamMsg::Nudge(p) => {
                let result = run_nudge(
                    &mut state,
                    &p,
                    &dispatch,
                    &nudge_dispatch,
                    &mut sync_instant,
                    last_move_time_bits,
                    commit_fire_count,
                );
                let _ = p.notify.send(result);
            }
            StreamMsg::Shutdown => {
                let segs = state
                    .commit(true)
                    .unwrap_or_else(|e| fatal(&format!("shutdown drain: {e}")));
                dispatch_committed(
                    &segs,
                    &dispatch,
                    &mut sync_instant,
                    last_move_time_bits,
                    commit_fire_count,
                );
                return;
            }
        }
    }
}

#[cfg(test)]
mod tests;
