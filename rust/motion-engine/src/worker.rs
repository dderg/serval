use std::collections::{HashMap, VecDeque};
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, Sender, TrySendError, bounded};
use trajectory::AxisChainSet;

use motion_pipeline::{
    BarrierAck, CONTIGUITY_EPS_MM, Control, StreamConfig, StreamInput, advance_odometer, dist3,
    setup_stages,
};

use crate::types::AxisKey;

mod dispatch;

pub use dispatch::DispatchError;
pub(crate) use dispatch::{
    Consumer, ConsumerShared, DispatchFn, NudgeDispatchFn, SegmentDispatchCtx, dispatch_nudge,
    dispatch_segment,
};

/// Host-monotonic end of the trajectory committed to the pump. The dispatcher
/// advances it as each segment is anchored; the ingress pacer reads the
/// remaining runway to decide whether a silent input warrants waiting for
/// more moves before sending `Drain`. Expressed as an `Instant` deadline so
/// readers never touch the MCU clock domain: the dispatcher, which holds `t0`
/// and the projected playhead anyway, does the one conversion at dispatch
/// time. Clearing is always safe — a falsely-zero runway only causes an
/// unnecessarily early drain — so abort paths may clear it out-of-band.
#[derive(Debug, Default)]
pub struct CommittedFrontier {
    deadline: Mutex<Option<Instant>>,
}

impl CommittedFrontier {
    pub fn advance_to(&self, deadline: Instant) {
        let mut guard = self.deadline.lock().unwrap_or_else(|p| p.into_inner());
        *guard = Some(guard.map_or(deadline, |d| d.max(deadline)));
    }

    pub fn clear(&self) {
        *self.deadline.lock().unwrap_or_else(|p| p.into_inner()) = None;
    }

    pub fn runway_secs(&self) -> f64 {
        self.deadline
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .map_or(0.0, |d| {
                d.saturating_duration_since(Instant::now()).as_secs_f64()
            })
    }
}

/// Runway kept in reserve when the pacer waits out a silent input instead of
/// sending `Drain`. A drain fired at the reserve must still travel
/// fit → plan → lower → shape → dispatch and reach the pump before the
/// playhead overruns the committed frontier; those stages take milliseconds,
/// so this covers them with a wide margin while staying under the anchor's
/// 250 ms initial lead. Overrunning anyway is not fatal — the anchor
/// re-anchors forward with a logged stutter.
const RUNWAY_RESERVE_S: f64 = 0.1;

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

pub const INPUT_CHANNEL_CAP: usize = 8192;

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

/// Connection-layer resources the pump needs: the wire sink over the live
/// transports and the callbacks that reach back into the connection
/// supervisor (ring depths, clock sync, endpoint-death and drip-stall
/// escalation). The bridge assembles these; the pipeline owns the pump built
/// from them.
pub(crate) struct PumpResources {
    pub(crate) sink: crate::pump::WireSink,
    pub(crate) ring_depth_of: Box<dyn Fn(AxisKey) -> u32 + Send>,
    pub(crate) mcu_clock_of: Box<dyn Fn(u32) -> Option<(u64, f64)> + Send>,
    pub(crate) on_fatal_transport: Box<dyn Fn(AxisKey) + Send + 'static>,
    pub(crate) on_abandon: Box<dyn Fn(AxisKey, u32) + Send>,
    pub(crate) on_drip_stall: Box<dyn Fn(String) + Send>,
    pub(crate) backlog: Arc<AtomicU64>,
}

/// Clock-domain and bookkeeping resources the dispatcher anchors segments
/// against. The pump enqueue side is not here — it is created inside
/// `setup_pipeline`, which owns both ends.
pub(crate) struct DispatchResources {
    pub(crate) router: Arc<Mutex<host_rt::passthrough_queue::PassthroughRouter>>,
    pub(crate) anchor: Arc<Mutex<crate::anchor::Anchor>>,
    pub(crate) mcu_configs: Vec<crate::mcu_config::McuAxisConfig>,
    pub(crate) drain: Arc<crate::drain::DrainSync>,
    pub(crate) counter: Arc<AtomicU64>,
    pub(crate) active_drip_cohort: Arc<Mutex<Option<u64>>>,
    pub(crate) motion_history: Arc<Mutex<crate::motion_history::HistoryStore>>,
    pub(crate) nominal_freqs: Arc<Mutex<HashMap<u32, u32>>>,
}

pub(crate) struct MotionPipeline {
    pub(crate) worker: StreamWorkerHandle,
    /// Out-of-band pump control (drip arm/disarm, flush, heartbeats): the
    /// paths that must act while the in-band stream is gated or stalled.
    pub(crate) pump_control: Sender<crate::pump::PumpMsg>,
    pub(crate) pump_thread: JoinHandle<()>,
}

/// Boot-time constructor of the entire motion pipeline:
/// fitter → planner → lowerer → shaper → dispatcher → pump, wired once and
/// never torn down. Everything downstream of the ingress — including the pump
/// thread and the enqueue channel between dispatcher and pump — is owned
/// here; the bridge only supplies the connection-layer resources.
pub(crate) fn setup_pipeline(
    config: StreamConfig,
    axis_chains: AxisChainSet,
    home_pos: Vec<f64>,
    dispatch: DispatchResources,
    pump: PumpResources,
) -> MotionPipeline {
    let (pump_control, control_rx) = crossbeam_channel::unbounded::<crate::pump::PumpMsg>();
    let (pump_data, data_rx) =
        bounded::<crate::pump::EnqueueMsg>(crate::pump::PUMP_DATA_CHANNEL_CAP);
    let pump_thread = thread::Builder::new()
        .name("push-pieces-pump".into())
        .spawn(move || {
            crate::pump::run_pump(
                control_rx,
                data_rx,
                pump.sink,
                pump.ring_depth_of,
                pump.mcu_clock_of,
                pump.on_fatal_transport,
                pump.on_abandon,
                pump.on_drip_stall,
                pump.backlog,
            );
        })
        .expect("spawn push-pieces-pump thread");
    let ctx = Arc::new(SegmentDispatchCtx {
        frontier: Arc::default(),
        router: dispatch.router,
        anchor: dispatch.anchor,
        mcu_configs: dispatch.mcu_configs,
        pump_tx: pump_data,
        drain: dispatch.drain,
        counter: dispatch.counter,
        active_drip_cohort: dispatch.active_drip_cohort,
        motion_history: dispatch.motion_history,
        nominal_freqs: dispatch.nominal_freqs,
    });
    let worker = StreamWorkerHandle::spawn_with_ctx(config, axis_chains, home_pos, ctx);
    MotionPipeline {
        worker,
        pump_control,
        pump_thread,
    }
}

impl StreamWorkerHandle {
    /// Builds the worker's dispatchers from the dispatch context. Use
    /// `setup_pipeline` from production — it owns the pump this
    /// context enqueues into.
    fn spawn_with_ctx(
        config: StreamConfig,
        axis_chains: AxisChainSet,
        home_pos: Vec<f64>,
        ctx: Arc<SegmentDispatchCtx>,
    ) -> Self {
        let frontier = Arc::clone(&ctx.frontier);
        let dispatch_ctx = Arc::clone(&ctx);
        let dispatch: DispatchFn = Arc::new(move |seg| dispatch_segment(&dispatch_ctx, seg));
        let nudge_dispatch: NudgeDispatchFn =
            Arc::new(move |mcu_id, np| dispatch_nudge(&ctx, mcu_id, np));
        Self::spawn(
            config,
            axis_chains,
            home_pos,
            dispatch,
            nudge_dispatch,
            frontier,
        )
    }

    /// Test seam: `spawn_with_ctx` is the production entry; this variant
    /// injects the dispatchers directly so tests can capture output without
    /// a router or pump.
    pub(crate) fn spawn(
        config: StreamConfig,
        axis_chains: AxisChainSet,
        home_pos: Vec<f64>,
        dispatch: DispatchFn,
        nudge_dispatch: NudgeDispatchFn,
        frontier: Arc<CommittedFrontier>,
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
            sync_instant: Arc::new(Mutex::new(None)),
            last_move_time_bits: Arc::clone(&last_move_time_bits),
            commit_fire_count: Arc::clone(&commit_fire_count),
            tally: Arc::clone(&tally),
        };
        let join = thread::Builder::new()
            .name("kalico-stream-worker".to_string())
            .spawn(move || {
                let pipeline = setup_stages(config, axis_chains, home_pos.clone(), 0.0);
                let discard = Arc::new(AtomicBool::new(false));
                let capture_errors = Arc::new(AtomicBool::new(false));
                let consumer = Consumer {
                    shared: shared.clone(),
                    discard: Arc::clone(&discard),
                    capture_errors: Arc::clone(&capture_errors),
                    frontier: Arc::clone(&frontier),
                    dispatched_through: None,
                    pending_error: None,
                };
                let output = pipeline.output;
                thread::Builder::new()
                    .name("kalico-dispatch".to_string())
                    .spawn(move || consumer.run(&output))
                    .expect("spawn pipeline consumer thread");
                let worker = Worker {
                    config,
                    nudge_dispatch,
                    odometer: home_pos,
                    t_next: 0.0,
                    input: pipeline.input,
                    discard,
                    capture_errors,
                    shared,
                    tally,
                    frontier,
                    undrained: false,
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
pub(crate) struct IntakeTally {
    per_move: VecDeque<(u32, f64)>,
    secs: f64,
    atomic: Arc<AtomicU64>,
}

impl IntakeTally {
    pub(crate) fn new(atomic: Arc<AtomicU64>) -> Self {
        Self {
            per_move: VecDeque::new(),
            secs: 0.0,
            atomic,
        }
    }

    fn publish(&self) {
        self.atomic.store(self.secs.to_bits(), Ordering::Release);
    }

    pub(crate) fn record_intake(&mut self, m: &geometry::Move) {
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

    pub(crate) fn retire_dispatched(&mut self, dispatched_line: u32) {
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

    pub(crate) fn reset(&mut self) {
        self.per_move.clear();
        self.secs = 0.0;
        self.publish();
    }
}

pub(crate) fn fatal(msg: &str) -> ! {
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

struct Worker {
    config: StreamConfig,
    nudge_dispatch: NudgeDispatchFn,
    /// Expected toolhead position after every move ingested so far; the
    /// ingress contiguity check anchors here.
    odometer: Vec<f64>,
    /// Stream time the dispatched timeline has reached, mirrored from barrier
    /// acks; nudges (which bypass the pipeline) plan from it.
    t_next: f64,
    input: Sender<StreamInput>,
    discard: Arc<AtomicBool>,
    capture_errors: Arc<AtomicBool>,
    shared: ConsumerShared,
    tally: Arc<Mutex<IntakeTally>>,
    frontier: Arc<CommittedFrontier>,
    /// The pipeline holds moves ingested since the last `Drain`; the pacer
    /// only schedules a drain while this is set.
    undrained: bool,
}

impl Worker {
    fn send(&mut self, item: StreamInput) {
        if self.input.send(item).is_err() {
            fatal("pipeline input closed — a stage died");
        }
    }

    /// Fence: everything sent before this has been dispatched (or discarded)
    /// once it returns. Advances the worker's timeline mirror.
    fn barrier(&mut self) -> BarrierAck {
        let (tx, rx) = crossbeam_channel::bounded(1);
        self.send(StreamInput::Control(Control::Barrier(tx)));
        let ack = rx
            .recv()
            .unwrap_or_else(|_| fatal("pipeline dropped a barrier — a stage died"));
        if let Some(t) = ack.dispatched_through {
            self.t_next = t;
        }
        ack
    }

    /// Drain the lookahead and fence: the pipeline is empty and the full
    /// braked-to-rest trajectory is dispatched when this returns, so no
    /// intake remains uncommitted.
    fn drain_and_fence(&mut self) -> BarrierAck {
        self.send(StreamInput::Drain);
        self.undrained = false;
        let ack = self.barrier();
        self.tally.lock().unwrap_or_else(|p| p.into_inner()).reset();
        ack
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
        self.send(m.into());
        self.undrained = true;
    }

    /// The pacer's one decision. Called when the inbox is silent while the
    /// pipeline holds undrained moves: with runway beyond the reserve there is
    /// provably time to wait for more input, so report how long; at the
    /// reserve, send `Drain` so the fitter and planner materialize the
    /// brake-to-rest and the drained trajectory beats the playhead to the
    /// pump.
    fn drain_or_runway(&mut self) -> Option<Duration> {
        let wait_s = self.frontier.runway_secs() - RUNWAY_RESERVE_S;
        if wait_s > 0.0 {
            return Some(Duration::from_secs_f64(wait_s));
        }
        tracing::info!(
            subsystem = "motion",
            event = "pipe_drain",
            t_us = crate::timing::mono_us(),
            "[pipe] runway exhausted — draining pipeline to rest"
        );
        self.send(StreamInput::Drain);
        self.undrained = false;
        None
    }

    /// Handle one non-move control message. Returns `true` when the loop
    /// should exit (shutdown).
    fn handle_control(&mut self, msg: StreamMsg) -> bool {
        match msg {
            StreamMsg::Move(_) => unreachable!("moves handled by the ingress path"),
            StreamMsg::Flush { notify } => {
                let ack = self.drain_and_fence();
                let finish = ack.sync_instant.map(|t| {
                    t + Duration::try_from_secs_f64((self.t_next + LEAD).max(0.0))
                        .unwrap_or(Duration::ZERO)
                });
                let _ = notify.send(finish);
            }
            StreamMsg::Dwell { duration_s, notify } => {
                self.drain_and_fence();
                if duration_s > 0.0 {
                    self.send(StreamInput::Control(Control::Dwell { secs: duration_s }));
                    let before = self.t_next;
                    if self.barrier().dispatched_through.is_none() {
                        self.t_next = before + duration_s;
                    }
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
                self.drain_and_fence();
                self.send(StreamInput::Control(Control::SetAxisChains(chains)));
                self.barrier();
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
                self.drain_and_fence();
                return true;
            }
        }
        false
    }

    /// Drop everything queued without dispatching it and restart the timeline
    /// at rest at `pos`. The discard gate goes up out-of-band (segments
    /// already past the shaper are dropped immediately) and the in-band
    /// `Reset` lifts it when it catches up, so nothing sent before this call
    /// reaches the pump and everything sent after does.
    fn reset_to(&mut self, pos: Vec<f64>) {
        self.discard.store(true, Ordering::Release);
        self.frontier.clear();
        self.send(StreamInput::Control(Control::Reset { pos: pos.clone() }));
        self.undrained = false;
        self.barrier();
        self.odometer = pos;
        self.t_next = 0.0;
        self.tally.lock().unwrap_or_else(|p| p.into_inner()).reset();
    }

    /// Run a homing drip through the pipeline with dispatch errors captured,
    /// so a failure surfaces to the homing caller instead of aborting.
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

        self.capture_errors.store(true, Ordering::Release);
        self.send(m.into());
        self.undrained = true;
        let ack = self.drain_and_fence();
        self.capture_errors.store(false, Ordering::Release);
        ack.result
    }

    fn run_nudge(&mut self, p: &NudgeParams) -> Result<(), String> {
        self.drain_and_fence();
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
        if total_dur > 0.0 {
            self.send(StreamInput::Control(Control::Dwell { secs: total_dur }));
        }
        self.shared
            .last_move_time_bits
            .store(self.t_next.to_bits(), Ordering::Release);
        Ok(())
    }
}

fn run_loop(rx: Receiver<StreamMsg>, mut worker: Worker) {
    loop {
        let received = if worker.undrained {
            match rx.try_recv() {
                Ok(msg) => Some(msg),
                Err(crossbeam_channel::TryRecvError::Empty) => match worker.drain_or_runway() {
                    None => continue,
                    Some(wait) => match rx.recv_timeout(wait) {
                        Ok(msg) => Some(msg),
                        Err(crossbeam_channel::RecvTimeoutError::Timeout) => continue,
                        Err(crossbeam_channel::RecvTimeoutError::Disconnected) => None,
                    },
                },
                Err(crossbeam_channel::TryRecvError::Disconnected) => None,
            }
        } else {
            rx.recv().ok()
        };
        let Some(msg) = received else {
            worker.drain_and_fence();
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
