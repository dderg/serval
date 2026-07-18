use crate::lock_ext::LockExt;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crossbeam_channel::{Sender, TrySendError, bounded};
use trajectory::AxisChainSet;

use motion_pipeline::{StreamConfig, setup_stages};

use crate::types::AxisKey;

mod dispatch;
mod ingress;
mod pump_sink;

#[cfg(test)]
pub(crate) use ingress::lead_secs;

pub use dispatch::{DispatchError, SegmentSink};
use dispatch::{Dispatcher, WorkerLinks};
use pump_sink::PumpSink;

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
        let mut guard = self.deadline.lock_ok();
        *guard = Some(guard.map_or(deadline, |d| d.max(deadline)));
    }

    pub fn clear(&self) {
        *self.deadline.lock_ok() = None;
    }

    pub fn runway_secs(&self) -> f64 {
        self.deadline.lock_ok().map_or(0.0, |d| {
            d.saturating_duration_since(Instant::now()).as_secs_f64()
        })
    }
}

#[derive(Debug)]
pub struct HomeDripParams {
    pub home_pos: [f64; 4],
    pub start: [f64; 3],
    pub axis: u8,
    pub direction: f64,
    pub speed_mm_s: f64,
    pub max_travel_mm: f64,
    pub cohort: u64,
    pub participants: Vec<AxisKey>,
}

#[derive(Debug)]
pub struct NudgeParams {
    pub mcu_id: u32,
    pub axis: u8,
    pub motor_mask: u8,
    pub delta_mm: f64,
    pub speed: f64,
    pub accel: f64,
}

/// Every slot in the pipe is queued-command latency (fan changes ride fences
/// behind the buffered moves), so this covers host reactor scheduling gaps
/// and nothing more; the submitter parks on the feed wakeup when it fills.
pub const INPUT_CHANNEL_CAP: usize = 16;
const SHUTDOWN_SEND_TIMEOUT: Duration = Duration::from_secs(1);
const SHUTDOWN_JOIN_TIMEOUT: Duration = Duration::from_secs(5);

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
        notify: crossbeam_channel::Sender<()>,
    },
    HomeDrip {
        params: HomeDripParams,
        notify: Sender<Result<(), String>>,
    },
    Nudge {
        params: NudgeParams,
        notify: Sender<Result<(), String>>,
    },
    Shutdown,
}

#[allow(missing_debug_implementations)]
pub struct StreamWorkerHandle {
    sender: Sender<StreamMsg>,
    join_handle: Option<JoinHandle<()>>,
    downstream_handles: Vec<JoinHandle<()>>,
    links: Arc<WorkerLinks>,
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
pub struct PumpResources {
    pub sink: crate::pump::WireSink,
    pub callbacks: crate::pump::PumpCallbacks,
    pub history: crate::pump::HistoryRecorder,
    pub drain: Arc<crate::drain::DrainLedger>,
    pub backlog: Arc<AtomicU64>,
}

/// Clock-domain and bookkeeping resources the dispatcher anchors segments
/// against. The pump enqueue side is not here — it is created inside
/// `setup_pipeline`, which owns both ends.
pub struct DispatchResources {
    pub router: Arc<Mutex<host_rt::passthrough_queue::PassthroughRouter>>,
    pub anchor: Arc<Mutex<crate::anchor::Anchor>>,
    pub mcu_configs: Vec<crate::mcu_config::McuAxisConfig>,
    pub counter: Arc<AtomicU64>,
    pub active_drip_cohort: Arc<Mutex<Option<u64>>>,
    pub motion_history: Arc<Mutex<crate::motion_history::HistoryStore>>,
}

pub struct MotionPipeline {
    pub worker: StreamWorkerHandle,
    /// Out-of-band pump control (drip arm/disarm, flush, heartbeats): the
    /// paths that must act while the in-band stream is gated or stalled.
    pub pump_control: Sender<crate::pump::PumpMsg>,
    pub pump_thread: JoinHandle<()>,
}

/// Boot-time constructor of the entire motion pipeline:
/// fit stage → planner → lowerer → shaper → dispatcher → pump, wired once and
/// never torn down. Everything downstream of the ingress — including the pump
/// thread and the enqueue channel between dispatcher and pump — is owned
/// here; the bridge only supplies the connection-layer resources.
pub fn setup_pipeline(
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
            host_rt::thread_prio::elevate_current_thread(
                host_rt::thread_prio::PUMP_RT_PRIORITY,
                "push-pieces-pump",
            );
            crate::pump::run_pump(
                control_rx,
                data_rx,
                pump.sink,
                pump.callbacks,
                Some(pump.history),
                pump.drain,
                pump.backlog,
            );
        })
        .expect("spawn push-pieces-pump thread");
    let frontier: Arc<CommittedFrontier> = Arc::default();
    let sink = PumpSink {
        router: dispatch.router,
        anchor: dispatch.anchor,
        mcu_configs: dispatch.mcu_configs,
        pump_tx: pump_data,
        counter: dispatch.counter,
        active_drip_cohort: dispatch.active_drip_cohort,
        motion_history: dispatch.motion_history,
        frontier: Arc::clone(&frontier),
    };
    let worker = StreamWorkerHandle::spawn(
        config,
        axis_chains,
        home_pos,
        sink,
        frontier,
        Some(pump_control.clone()),
    );
    MotionPipeline {
        worker,
        pump_control,
        pump_thread,
    }
}

impl StreamWorkerHandle {
    /// Wires the full stream worker: ingress → pure stages → dispatcher, all
    /// threads spawned here as siblings, mirroring `setup_stages`. Production
    /// (`setup_pipeline`) passes the pump-backed sink; tests pass a capture
    /// sink and no pump.
    pub fn spawn(
        config: StreamConfig,
        axis_chains: AxisChainSet,
        home_pos: Vec<f64>,
        sink: impl SegmentSink,
        frontier: Arc<CommittedFrontier>,
        pump_control: Option<Sender<crate::pump::PumpMsg>>,
    ) -> Self {
        let (tx, rx) = bounded(INPUT_CHANNEL_CAP);
        let links = Arc::new(WorkerLinks::default());

        let pipeline = setup_stages(config, axis_chains, home_pos.clone(), 0.0);
        let mut downstream_handles = pipeline.threads;

        let dispatcher = Dispatcher::new(sink, Arc::clone(&links), Arc::clone(&frontier));
        let output = pipeline.output;
        let dispatcher_handle = thread::Builder::new()
            .name("kalico-dispatch".to_string())
            .spawn(move || dispatcher.run(&output))
            .expect("spawn pipeline dispatcher thread");
        downstream_handles.push(dispatcher_handle);

        let ingress = ingress::Ingress {
            config,
            odometer: home_pos,
            t_next: 0.0,
            input: pipeline.input,
            links: Arc::clone(&links),
            frontier,
            intake: ingress::IntakeState::default(),
            last_line: 0,
            pump_control,
        };
        let join = thread::Builder::new()
            .name("kalico-stream-worker".to_string())
            .spawn(move || ingress.run(rx))
            .expect("spawn stream worker thread");

        Self {
            sender: tx,
            join_handle: Some(join),
            downstream_handles,
            links,
        }
    }

    pub fn submit_move(&self, m: geometry::Move) -> Result<(), StreamWorkerError> {
        self.try_send_arming(StreamMsg::Move(m))
    }

    /// Non-blocking send that arms the feed wakeup on a full channel: the
    /// caller that receives `ChannelFull` parks on the wakeup fd and the
    /// ingress pings it when it next frees a slot. The post-arm retry closes
    /// the race where the ingress freed a slot (and saw the flag unarmed)
    /// between the first attempt and the arm.
    fn try_send_arming(&self, msg: StreamMsg) -> Result<(), StreamWorkerError> {
        let msg = match self.sender.try_send(msg) {
            Ok(()) => return Ok(()),
            Err(TrySendError::Disconnected(_)) => return Err(StreamWorkerError::ChannelClosed),
            Err(TrySendError::Full(msg)) => msg,
        };
        self.links.wakeup.arm();
        try_send_msg(&self.sender, msg)
    }

    /// Fd the host reactor parks on for `ChannelFull` retries and fence
    /// resolution. Owned by the pipeline — callers must not close it.
    #[must_use]
    pub fn feed_wakeup_read_fd(&self) -> i32 {
        self.links.wakeup.read_fd()
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

    /// Non-blocking: `ChannelFull` means the caller must retry after
    /// yielding, exactly like `fence_start` — a blocking send here wedges
    /// the klippy reactor for as long as the backpressured pipe takes to
    /// admit one message.
    pub fn flush_try_start(
        &self,
    ) -> Result<crossbeam_channel::Receiver<Option<Instant>>, StreamWorkerError> {
        let (tx, rx) = crossbeam_channel::bounded(1);
        self.try_send_arming(StreamMsg::Flush { notify: tx })?;
        Ok(rx)
    }

    /// Non-blocking: `ChannelFull` means the caller must retry after
    /// yielding. A blocking send here would freeze the klippy reactor thread
    /// (and with it the heater keepalives) for as long as the backpressured
    /// pipe takes to admit one message — seconds at full buffers.
    pub fn fence_start(&self, force: bool) -> Result<u64, StreamWorkerError> {
        let id = self.links.fences.alloc_id();
        self.try_send_arming(StreamMsg::Fence { id, force })?;
        Ok(id)
    }

    /// `None` while the fence is pending; `Some(t)` once resolved, where `t`
    /// is the stream time the fenced motion ends at (`None` inside when the
    /// stream was reset or nothing was ever dispatched). Consumes the result.
    pub fn fence_take(&self, id: u64) -> Option<Option<f64>> {
        self.links.fences.take(id)
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

    /// Blocks until the pipeline has drained and adopted the new transform:
    /// the caller's own mesh copy (used for bridge-level space crossings)
    /// must never run ahead of the mesh the lowerer is actually warping with.
    pub fn update_mesh(
        &self,
        mesh: Option<std::sync::Arc<geometry::SurfaceTransform>>,
        gcode_z_rebase: f64,
    ) -> Result<(), StreamWorkerError> {
        let (tx, rx) = crossbeam_channel::bounded(1);
        self.sender
            .send(StreamMsg::SetMesh {
                mesh,
                gcode_z_rebase,
                notify: tx,
            })
            .map_err(|_| StreamWorkerError::ChannelClosed)?;
        rx.recv().map_err(|_| StreamWorkerError::ChannelClosed)
    }

    pub fn home_drip(
        &self,
        params: HomeDripParams,
    ) -> Result<crossbeam_channel::Receiver<Result<(), String>>, StreamWorkerError> {
        let (notify, result) = crossbeam_channel::bounded(1);
        self.sender
            .send(StreamMsg::HomeDrip { params, notify })
            .map_err(|_| StreamWorkerError::ChannelClosed)?;
        Ok(result)
    }

    pub fn submit_nudge(
        &self,
        params: NudgeParams,
    ) -> Result<crossbeam_channel::Receiver<Result<(), String>>, StreamWorkerError> {
        let (notify, result) = crossbeam_channel::bounded(1);
        self.sender
            .send(StreamMsg::Nudge { params, notify })
            .map_err(|_| StreamWorkerError::ChannelClosed)?;
        Ok(result)
    }

    #[must_use]
    pub fn last_move_time(&self) -> f64 {
        f64::from_bits(self.links.last_move_time_bits.load(Ordering::Acquire))
    }

    #[must_use]
    pub fn commit_fire_count(&self) -> u32 {
        self.links.commit_fire_count.load(Ordering::Acquire)
    }

    pub fn shutdown(&mut self) {
        self.prepare_shutdown();
        let _ = self
            .sender
            .send_timeout(StreamMsg::Shutdown, SHUTDOWN_SEND_TIMEOUT);
        let deadline = Instant::now() + SHUTDOWN_JOIN_TIMEOUT;
        if let Some(h) = self.join_handle.take() {
            join_worker_thread(h, deadline);
        }
        for handle in self.downstream_handles.drain(..) {
            join_worker_thread(handle, deadline);
        }
    }

    pub fn prepare_shutdown(&self) {
        self.links.shutting_down.store(true, Ordering::Release);
    }
}

impl Drop for StreamWorkerHandle {
    fn drop(&mut self) {
        if self.join_handle.is_some() {
            self.shutdown();
        }
    }
}

fn join_worker_thread(handle: JoinHandle<()>, deadline: Instant) {
    let name = handle.thread().name().unwrap_or("unnamed").to_owned();
    while !handle.is_finished() && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(10));
    }
    if !handle.is_finished() {
        tracing::error!(
            subsystem = "motion",
            event = "shutdown_motion_thread_join_timeout",
            thread = name,
            timeout_ms = SHUTDOWN_JOIN_TIMEOUT.as_millis() as u64,
            "motion thread did not exit before the shutdown deadline; detaching it"
        );
        return;
    }
    if let Err(error) = handle.join() {
        tracing::error!(
            subsystem = "motion",
            event = "shutdown_motion_thread_join_panicked",
            thread = name,
            error = ?error,
            "motion thread had already panicked during shutdown"
        );
    }
}

fn try_send_msg(sender: &Sender<StreamMsg>, msg: StreamMsg) -> Result<(), StreamWorkerError> {
    sender.try_send(msg).map_err(|e| match e {
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
