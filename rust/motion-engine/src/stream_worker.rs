use std::collections::{HashMap, VecDeque};
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, Sender, TrySendError, bounded};
use trajectory::{AxisChainSet, ShapedSegment};

use crate::pump::AxisKey;
use crate::stream::{
    CONTIGUITY_EPS_MM, PipelineHandle, StreamConfig, advance_odometer, dist3, setup_pipeline,
};

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
}

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
pub struct StreamWorkerHandle {
    sender: Sender<StreamMsg>,
    join_handle: Option<JoinHandle<()>>,
    last_move_time_bits: Arc<AtomicU64>,
    commit_fire_count: Arc<AtomicU32>,
    uncommitted_intake_secs: Arc<AtomicU64>,
}

#[derive(Debug)]
pub enum StreamWorkerError {
    ChannelClosed,
    ChannelFull,
}

impl std::fmt::Display for StreamWorkerError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::ChannelClosed => write!(f, "stream worker channel closed"),
            Self::ChannelFull => write!(
                f,
                "stream worker input channel full ({INPUT_CHANNEL_CAP} \
                 moves) — host backpressure gate bypassed"
            ),
        }
    }
}

impl std::error::Error for StreamWorkerError {}

impl StreamWorkerHandle {
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

        let tally = Arc::new(Mutex::new(IntakeTally::new(Arc::clone(
            &uncommitted_intake_secs,
        ))));
        let shared = ConsumerShared {
            dispatch,
            discard: Arc::new(AtomicBool::new(false)),
            sync_instant: Arc::new(Mutex::new(None)),
            last_move_time_bits: Arc::clone(&last_move_time_bits),
            commit_fire_count: Arc::clone(&commit_fire_count),
            tally: Arc::clone(&tally),
        };
        let join = thread::Builder::new()
            .name("kalico-stream-worker".to_string())
            .spawn(move || {
                let worker = Worker {
                    config,
                    axis_chains,
                    nudge_dispatch,
                    odometer: home_pos,
                    t_next: 0.0,
                    active: None,
                    shared,
                    tally,
                };
                run_loop(rx, worker);
            })
            .expect("spawn stream worker thread");

        Self {
            sender: tx,
            join_handle: Some(join),
            last_move_time_bits,
            commit_fire_count,
            uncommitted_intake_secs,
        }
    }

    pub fn submit_move(&self, m: geometry::Move) -> Result<(), StreamWorkerError> {
        try_submit_move(&self.sender, m)
    }

    pub fn pending_channel_moves(&self) -> usize {
        self.sender.len()
    }

    pub fn flush(&self) -> Result<(), StreamWorkerError> {
        let (tx, rx) = crossbeam_channel::bounded(1);
        self.sender
            .send(StreamMsg::Flush { notify: tx })
            .map_err(|_| StreamWorkerError::ChannelClosed)?;
        rx.recv()
            .map(|_committed_through| ())
            .map_err(|_| StreamWorkerError::ChannelClosed)
    }

    pub fn flush_start(
        &self,
    ) -> Result<crossbeam_channel::Receiver<Option<Instant>>, StreamWorkerError> {
        let (tx, rx) = crossbeam_channel::bounded(1);
        self.sender
            .send(StreamMsg::Flush { notify: tx })
            .map_err(|_| StreamWorkerError::ChannelClosed)?;
        Ok(rx)
    }

    pub fn dwell(&self, duration_s: f64) -> Result<(), StreamWorkerError> {
        let (tx, rx) = crossbeam_channel::bounded(1);
        self.sender
            .send(StreamMsg::Dwell {
                duration_s,
                notify: tx,
            })
            .map_err(|_| StreamWorkerError::ChannelClosed)?;
        rx.recv().map_err(|_| StreamWorkerError::ChannelClosed)
    }

    pub fn stream_open(&self, home_pos: Vec<f64>) -> Result<(), StreamWorkerError> {
        self.sender
            .send(StreamMsg::StreamOpen { home_pos })
            .map_err(|_| StreamWorkerError::ChannelClosed)
    }

    pub fn reset(&self, recovered_pos: Vec<f64>) -> Result<(), StreamWorkerError> {
        self.sender
            .send(StreamMsg::Reset { recovered_pos })
            .map_err(|_| StreamWorkerError::ChannelClosed)
    }

    pub fn update_axis_chains(&self, chains: AxisChainSet) -> Result<(), StreamWorkerError> {
        self.sender
            .send(StreamMsg::SetAxisChains(chains))
            .map_err(|_| StreamWorkerError::ChannelClosed)
    }

    pub fn home_drip(&self, p: HomeDripParams) -> Result<(), StreamWorkerError> {
        self.sender
            .send(StreamMsg::HomeDrip(p))
            .map_err(|_| StreamWorkerError::ChannelClosed)
    }

    pub fn submit_nudge(&self, p: NudgeParams) -> Result<(), StreamWorkerError> {
        self.sender
            .send(StreamMsg::Nudge(p))
            .map_err(|_| StreamWorkerError::ChannelClosed)
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

impl Drop for StreamWorkerHandle {
    fn drop(&mut self) {
        if self.join_handle.is_some() {
            self.shutdown();
        }
    }
}

fn try_submit_move(sender: &Sender<StreamMsg>, m: geometry::Move) -> Result<(), StreamWorkerError> {
    sender.try_send(StreamMsg::Move(m)).map_err(|e| match e {
        TrySendError::Full(_) => StreamWorkerError::ChannelFull,
        TrySendError::Disconnected(_) => StreamWorkerError::ChannelClosed,
    })
}

fn nominal_t(m: &geometry::Move) -> Option<f64> {
    if m.feedrate_mm_s > 0.0 {
        Some(m.segment.s_len() / m.feedrate_mm_s)
    } else {
        None
    }
}

/// Nominal seconds of motion ingested into the pipeline but not yet dispatched
/// to the pump, published for the host's backpressure gate. Entries retire as
/// the consumer's dispatched-line frontier passes them.
struct IntakeTally {
    per_move: VecDeque<(u32, f64)>,
    secs: f64,
    atomic: Arc<AtomicU64>,
}

impl IntakeTally {
    fn new(atomic: Arc<AtomicU64>) -> Self {
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
        self.per_move.push_back((m.source.start_line, dt));
        self.secs += dt;
        self.publish();
    }

    fn retire_dispatched(&mut self, dispatched_line: u32) {
        while self
            .per_move
            .front()
            .is_some_and(|&(line, _)| line < dispatched_line)
        {
            let (_, dt) = self.per_move.pop_front().expect("front checked");
            self.secs -= dt;
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
        event = "stream_worker_fatal",
        error = msg,
        "stream worker encountered an unrecoverable error — aborting"
    );
    eprintln!("kalico stream worker fatal: {msg}");
    std::thread::sleep(Duration::from_millis(100));
    std::process::abort();
}

/// State a committed `ShapedSegment` needs to reach the pump: per-MCU clock
/// anchoring/projection, the axis-lane split, and the motion-history/drain
/// bookkeeping that split feeds.
pub(crate) struct SegmentDispatchCtx {
    pub(crate) router: Arc<Mutex<host_rt::passthrough_queue::PassthroughRouter>>,
    pub(crate) anchor: Arc<Mutex<crate::anchor::Anchor>>,
    pub(crate) mcu_configs: Vec<crate::dispatch::McuAxisConfig>,
    pub(crate) pump_tx: Sender<crate::pump::EnqueueMsg>,
    pub(crate) drain: Arc<crate::drain::DrainSync>,
    pub(crate) counter: Arc<AtomicU64>,
    pub(crate) active_drip_cohort: Arc<Mutex<Option<u64>>>,
    pub(crate) motion_history: Arc<Mutex<crate::motion_history::HistoryStore>>,
    pub(crate) nominal_freqs: Arc<Mutex<HashMap<u32, u32>>>,
}

/// Anchor a committed segment to the MCU clock, split it into per-axis
/// pieces, and hand each piece to the pump.
pub(crate) fn dispatch_segment(
    ctx: &SegmentDispatchCtx,
    seg: &ShapedSegment,
) -> Result<(), DispatchError> {
    tracing::debug!(
        subsystem = "engine",
        event = "dispatch_entered",
        seg_t_start = seg.t_start,
        seg_t_end = seg.t_end,
        "[engine-trace] dispatch entered"
    );

    let host_now = {
        let r = ctx.router.lock().unwrap_or_else(|p| p.into_inner());
        r.host_now_secs()
    };

    let (t0, fresh) = ctx
        .anchor
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .anchor_segment(seg.t_start, seg.t_end, host_now);

    if fresh {
        let r = ctx.router.lock().unwrap_or_else(|p| p.into_inner());
        for cfg in ctx.mcu_configs.iter() {
            let h = crate::types::mcu_handle_from_raw(cfg.mcu_id);
            r.log_seg0_lead(h, t0 + seg.t_start, t0);
        }
    }

    let project = |mcu_id: u32, host_secs: f64| -> u64 {
        let r = ctx.router.lock().unwrap_or_else(|p| p.into_inner());
        r.host_time_to_mcu_clock(crate::types::mcu_handle_from_raw(mcu_id), host_secs)
            .unwrap_or(0)
    };

    let active_cohort: Option<u64> = *ctx
        .active_drip_cohort
        .lock()
        .unwrap_or_else(|p| p.into_inner());

    let max_piece_secs = if active_cohort.is_some() {
        Some(0.025_f64)
    } else {
        None::<f64>
    };
    let lead_secs = if active_cohort.is_some() {
        crate::pump::DRIP_WINDOW_SECS
    } else {
        crate::pump::MAX_LEAD_SECS
    };

    let msgs = crate::enqueue::enqueue_segment(
        seg,
        &ctx.mcu_configs,
        t0,
        fresh,
        host_now,
        lead_secs,
        project,
        max_piece_secs,
    );

    let nominal_freqs = ctx
        .nominal_freqs
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .clone();
    if fresh {
        ctx.motion_history
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .drop_pieces_on_reanchor();
    }
    for m in msgs {
        let nominal_freq = *nominal_freqs
            .get(&m.key.mcu_id)
            .ok_or(DispatchError::MissingNominalFreq(m.key.mcu_id))?;
        {
            let mut store = ctx.motion_history.lock().unwrap_or_else(|p| p.into_inner());
            for (piece, host_t) in &m.pieces {
                store.record(m.key, piece, nominal_freq, *host_t);
            }
        }
        ctx.drain
            .add_sent(m.key.mcu_id, m.key.axis, m.pieces.len() as u32);
        ctx.pump_tx.send(m).map_err(|_| DispatchError::PumpGone)?;
    }

    tracing::info!(
        subsystem = "motion",
        event = "pipe_pump_in",
        line = seg.source_line,
        t_us = crate::timing::mono_us(),
        "[pipe] handed to pump"
    );

    ctx.counter.fetch_add(1, Ordering::Relaxed);
    Ok(())
}

/// State the pipeline-output consumer thread shares with the worker.
#[derive(Clone)]
struct ConsumerShared {
    dispatch: DispatchFn,
    /// Set by a teardown that must drop in-flight motion (reset, stream open,
    /// homing restart) instead of dispatching it.
    discard: Arc<AtomicBool>,
    sync_instant: Arc<Mutex<Option<Instant>>>,
    last_move_time_bits: Arc<AtomicU64>,
    commit_fire_count: Arc<AtomicU32>,
    /// Shared with the worker: entries retire as segments dispatch.
    tally: Arc<Mutex<IntakeTally>>,
}

impl ConsumerShared {
    fn dispatch_segment(&self, seg: &ShapedSegment) {
        let n_ax = seg.axes.len();
        tracing::info!(
            subsystem = "motion",
            event = "pipe_dispatch",
            line = seg.source_line,
            t_us = crate::timing::mono_us(),
            seg_t_start = seg.t_start,
            seg_t_end = seg.t_end,
            x_end = if n_ax > 0 {
                nurbs::eval::eval(&seg.axes[0], seg.t_end)
            } else {
                0.0
            },
            y_end = if n_ax > 1 {
                nurbs::eval::eval(&seg.axes[1], seg.t_end)
            } else {
                0.0
            },
            z_end = if n_ax > 2 {
                nurbs::eval::eval(&seg.axes[2], seg.t_end)
            } else {
                0.0
            },
            "[pipe] dispatch"
        );
        if let Err(e) = (self.dispatch)(seg) {
            fatal(&format!("dispatch failed: {e}"));
        }
        let mut sync = self.sync_instant.lock().unwrap_or_else(|p| p.into_inner());
        if sync.is_none() {
            *sync = Some(Instant::now());
        }
        drop(sync);
        self.last_move_time_bits
            .store(seg.t_end.to_bits(), Ordering::Release);
        self.tally
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .retire_dispatched(seg.source_line);
        self.commit_fire_count.fetch_add(1, Ordering::AcqRel);
    }
}

struct ActivePipeline {
    input: Sender<geometry::Move>,
    consumer: JoinHandle<Option<f64>>,
    discard: Arc<AtomicBool>,
}

struct Worker {
    config: StreamConfig,
    axis_chains: AxisChainSet,
    nudge_dispatch: NudgeDispatchFn,
    /// Expected toolhead position after every move ingested so far; the next
    /// pipeline (and the ingress contiguity check) starts here.
    odometer: Vec<f64>,
    /// Stream time where the next pipeline's timeline resumes.
    t_next: f64,
    active: Option<ActivePipeline>,
    shared: ConsumerShared,
    tally: Arc<Mutex<IntakeTally>>,
}

impl Worker {
    fn input(&mut self) -> &Sender<geometry::Move> {
        if self.active.is_none() {
            let handle = setup_pipeline(
                self.config,
                self.axis_chains.clone(),
                self.odometer.clone(),
                self.t_next,
            );
            let discard = Arc::new(AtomicBool::new(false));
            self.active = Some(ActivePipeline {
                input: handle.input,
                consumer: spawn_consumer(handle.output, self.shared.clone(), Arc::clone(&discard)),
                discard,
            });
        }
        &self.active.as_ref().expect("just ensured").input
    }

    /// Close the pipeline input and wait for every stage to drain through the
    /// consumer. With `discard` the in-flight motion is dropped instead of
    /// dispatched (reset paths); otherwise the timeline advances to the last
    /// dispatched segment's end.
    fn teardown(&mut self, discard: bool) {
        let Some(active) = self.active.take() else {
            return;
        };
        if discard {
            active.discard.store(true, Ordering::Release);
        }
        drop(active.input);
        let dispatched_through = active
            .consumer
            .join()
            .unwrap_or_else(|_| fatal("pipeline consumer panicked"));
        if let Some(t_end) = dispatched_through {
            self.t_next = t_end;
        }
        self.tally.lock().unwrap_or_else(|p| p.into_inner()).reset();
    }

    fn handle_move(&mut self, m: geometry::Move) {
        tracing::info!(
            subsystem = "motion",
            event = "pipe_ingress",
            line = m.source.start_line,
            t_us = crate::timing::mono_us(),
            "[pipe] ingress"
        );
        if let Some(seg) = &m.segment.spatial {
            use geometry::path::lowering::PositionProfile;
            let got = seg.point_at(0.0);
            let expected = [self.odometer[0], self.odometer[1], self.odometer[2]];
            let gap_mm = dist3(expected, got);
            if gap_mm > CONTIGUITY_EPS_MM {
                fatal(&format!(
                    "ingress: discontinuous move at line {}: starts at {got:?} but \
                     the toolhead is at {expected:?} ({gap_mm:.6}mm gap) — move \
                     stream is not position-contiguous",
                    m.source.start_line
                ));
            }
        }
        self.tally
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .record_intake(&m);
        advance_odometer(&mut self.odometer, &m);
        if self.input().send(m).is_err() {
            fatal("pipeline input closed — a stage died");
        }
    }

    /// Handle one non-move control message. Returns `true` when the loop
    /// should exit (shutdown).
    fn handle_control(&mut self, msg: StreamMsg) -> bool {
        match msg {
            StreamMsg::Move(_) => unreachable!("moves handled by the ingress path"),
            StreamMsg::Flush { notify } => {
                self.teardown(false);
                let finish = self
                    .shared
                    .sync_instant
                    .lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .map(|t| {
                        t + Duration::try_from_secs_f64((self.t_next + LEAD).max(0.0))
                            .unwrap_or(Duration::ZERO)
                    });
                let _ = notify.send(finish);
            }
            StreamMsg::Dwell { duration_s, notify } => {
                self.teardown(false);
                if duration_s > 0.0 {
                    self.t_next += duration_s;
                }
                let _ = notify.send(());
            }
            StreamMsg::StreamOpen { home_pos } => {
                self.reset_to(home_pos);
            }
            StreamMsg::Reset { recovered_pos } => {
                self.reset_to(recovered_pos);
            }
            StreamMsg::SetAxisChains(chains) => {
                self.teardown(false);
                self.axis_chains = chains;
            }
            StreamMsg::HomeDrip(p) => {
                let result = self.run_home_drip(&p);
                let _ = p.notify.send(result);
            }
            StreamMsg::Nudge(p) => {
                let result = self.run_nudge(&p);
                let _ = p.notify.send(result);
            }
            StreamMsg::Shutdown => {
                self.teardown(false);
                return true;
            }
        }
        false
    }

    fn reset_to(&mut self, pos: Vec<f64>) {
        self.teardown(true);
        self.odometer = pos;
        self.t_next = 0.0;
        *self
            .shared
            .sync_instant
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = None;
    }

    /// Run a homing drip as a one-shot pipeline dispatched synchronously, so a
    /// dispatch failure surfaces to the homing caller instead of aborting.
    fn run_home_drip(&mut self, p: &HomeDripParams) -> Result<(), String> {
        self.reset_to(p.home_pos.to_vec());
        let travel = p.direction * p.max_travel_mm;
        let (dx, dy, dz) = match p.axis {
            0 => (travel, 0.0, 0.0),
            1 => (0.0, travel, 0.0),
            2 => (0.0, 0.0, travel),
            other => return Err(format!("HomeDrip: unsupported axis {other} (only 0/1/2)")),
        };
        let m = crate::classify::build_move(
            p.start,
            dx,
            dy,
            dz,
            0,
            0.0,
            self.config.limits,
            p.speed_mm_s,
            0,
        )
        .map_err(|e| format!("HomeDrip build_move: {e:?}"))?;
        advance_odometer(&mut self.odometer, &m);

        let handle = setup_pipeline(
            self.config,
            self.axis_chains.clone(),
            p.home_pos.to_vec(),
            0.0,
        );
        handle
            .input
            .send(m)
            .map_err(|_| "HomeDrip: pipeline input closed".to_string())?;
        drop(handle.input);
        let mut result = Ok(());
        while let Ok(seg) = handle.output.recv() {
            if result.is_err() {
                continue;
            }
            if let Err(e) = (self.shared.dispatch)(&seg) {
                result = Err(format!("HomeDrip dispatch failed: {e}"));
                continue;
            }
            let mut sync = self
                .shared
                .sync_instant
                .lock()
                .unwrap_or_else(|p| p.into_inner());
            if sync.is_none() {
                *sync = Some(Instant::now());
            }
            drop(sync);
            self.shared
                .last_move_time_bits
                .store(seg.t_end.to_bits(), Ordering::Release);
            self.shared.commit_fire_count.fetch_add(1, Ordering::AcqRel);
            self.t_next = seg.t_end;
        }
        result
    }

    fn run_nudge(&mut self, p: &NudgeParams) -> Result<(), String> {
        self.teardown(false);
        let nudge_segs = crate::nudge::plan_nudge_profile(
            p.axis,
            p.delta_mm,
            p.speed,
            p.accel,
            p.motor_mask,
            self.t_next,
        )?;
        let total_dur: f64 = nudge_segs
            .iter()
            .map(|s| s.piece.u_end - s.piece.u_start)
            .sum();
        for s in &nudge_segs {
            (self.nudge_dispatch)(p.mcu_id, s).map_err(|e| format!("nudge dispatch: {e}"))?;
        }
        self.t_next += total_dur;
        self.shared
            .last_move_time_bits
            .store(self.t_next.to_bits(), Ordering::Release);
        Ok(())
    }
}

fn spawn_consumer(
    output: Receiver<ShapedSegment>,
    shared: ConsumerShared,
    discard: Arc<AtomicBool>,
) -> JoinHandle<Option<f64>> {
    thread::Builder::new()
        .name("kalico-dispatch".to_string())
        .spawn(move || {
            let mut dispatched_through = None;
            while let Ok(seg) = output.recv() {
                if discard.load(Ordering::Acquire) {
                    continue;
                }
                shared.dispatch_segment(&seg);
                dispatched_through = Some(seg.t_end);
            }
            dispatched_through
        })
        .expect("spawn pipeline consumer thread")
}

fn run_loop(rx: Receiver<StreamMsg>, mut worker: Worker) {
    loop {
        let Ok(msg) = rx.recv() else {
            worker.teardown(false);
            return;
        };
        match msg {
            StreamMsg::Move(m) => worker.handle_move(m),
            other => {
                if worker.handle_control(other) {
                    return;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests;
