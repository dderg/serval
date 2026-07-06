use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crossbeam_channel::{Sender, TrySendError, bounded};
use trajectory::AxisChainSet;

use motion_pipeline::{StreamConfig, setup_stages};

use crate::types::AxisKey;

mod dispatch;
mod ingress;

#[cfg(test)]
pub(crate) use ingress::lead_secs;

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

pub const INPUT_CHANNEL_CAP: usize = 64;

#[derive(Debug)]
pub enum StreamMsg {
    Move(geometry::Move),
    Flush {
        notify: Sender<Option<Instant>>,
    },
    /// Sequence point: resolves with the stream time at which everything
    /// submitted before it ends. `force` drains the pipeline (brake to rest)
    /// so the answer is immediate; otherwise it resolves as the stream
    /// naturally commits past it.
    Fence {
        id: u64,
        force: bool,
    },
    Dwell {
        duration_s: f64,
        notify: Sender<()>,
    },
    StreamOpen {
        home_pos: Vec<f64>,
    },
    Reset {
        recovered_pos: Vec<f64>,
    },
    SetAxisChains(AxisChainSet),
    SetMesh {
        mesh: Option<std::sync::Arc<geometry::SurfaceTransform>>,
        gcode_z_rebase: f64,
    },
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
    fences: Arc<crate::fence::FenceRegistry>,
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
                "stream worker input channel full ({INPUT_CHANNEL_CAP} moves)"
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
    pub(crate) drain: Arc<crate::drain::DrainLedger>,
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
                pump.drain,
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
        counter: dispatch.counter,
        active_drip_cohort: dispatch.active_drip_cohort,
        motion_history: dispatch.motion_history,
        nominal_freqs: dispatch.nominal_freqs,
    });
    let worker = StreamWorkerHandle::spawn_with_ctx(
        config,
        axis_chains,
        home_pos,
        ctx,
        Some(pump_control.clone()),
    );
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
        pump_control: Option<Sender<crate::pump::PumpMsg>>,
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
            pump_control,
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
        pump_control: Option<Sender<crate::pump::PumpMsg>>,
    ) -> Self {
        let (tx, rx) = bounded(INPUT_CHANNEL_CAP);
        let last_move_time_bits = Arc::new(AtomicU64::new(0));
        let commit_fire_count = Arc::new(AtomicU32::new(0));
        let fences = Arc::new(crate::fence::FenceRegistry::default());
        let shared = ConsumerShared {
            dispatch,
            sync_instant: Arc::new(Mutex::new(None)),
            last_move_time_bits: Arc::clone(&last_move_time_bits),
            commit_fire_count: Arc::clone(&commit_fire_count),
            fences: Arc::clone(&fences),
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
                ingress::Ingress {
                    config,
                    nudge_dispatch,
                    odometer: home_pos,
                    t_next: 0.0,
                    input: pipeline.input,
                    discard,
                    capture_errors,
                    shared,
                    frontier,
                    undrained: false,
                    last_line: 0,
                    pump_control,
                }
                .run(rx);
            })
            .expect("spawn stream worker thread");

        Self {
            sender: tx,
            join_handle: Some(join),
            last_move_time_bits,
            commit_fire_count,
            fences,
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

    /// Non-blocking: `ChannelFull` means the caller must retry after
    /// yielding, exactly like `fence_start` — a blocking send here wedges
    /// the klippy reactor for as long as the backpressured pipe takes to
    /// admit one message.
    pub fn flush_try_start(
        &self,
    ) -> Result<crossbeam_channel::Receiver<Option<Instant>>, StreamWorkerError> {
        let (tx, rx) = crossbeam_channel::bounded(1);
        self.sender
            .try_send(StreamMsg::Flush { notify: tx })
            .map_err(|e| match e {
                TrySendError::Full(_) => StreamWorkerError::ChannelFull,
                TrySendError::Disconnected(_) => StreamWorkerError::ChannelClosed,
            })?;
        Ok(rx)
    }

    /// Non-blocking: `ChannelFull` means the caller must retry after
    /// yielding. A blocking send here would freeze the klippy reactor thread
    /// (and with it the heater keepalives) for as long as the backpressured
    /// pipe takes to admit one message — seconds at full buffers.
    pub fn fence_start(&self, force: bool) -> Result<u64, StreamWorkerError> {
        let id = self.fences.alloc_id();
        self.sender
            .try_send(StreamMsg::Fence { id, force })
            .map_err(|e| match e {
                TrySendError::Full(_) => StreamWorkerError::ChannelFull,
                TrySendError::Disconnected(_) => StreamWorkerError::ChannelClosed,
            })?;
        Ok(id)
    }

    /// `None` while the fence is pending; `Some(t)` once resolved, where `t`
    /// is the stream time the fenced motion ends at (`None` inside when the
    /// stream was reset or nothing was ever dispatched). Consumes the result.
    pub fn fence_take(&self, id: u64) -> Option<Option<f64>> {
        self.fences.take(id)
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

    pub fn update_mesh(
        &self,
        mesh: Option<std::sync::Arc<geometry::SurfaceTransform>>,
        gcode_z_rebase: f64,
    ) -> Result<(), StreamWorkerError> {
        self.sender
            .send(StreamMsg::SetMesh {
                mesh,
                gcode_z_rebase,
            })
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

#[cfg(test)]
mod tests;
