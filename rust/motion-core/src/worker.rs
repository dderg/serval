use crate::lock_ext::LockExt;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crossbeam_channel::{Sender, TrySendError, bounded};
use trajectory::AxisChainSet;

use motion_pipeline::{Pipeline, StreamConfig, TrajectoryItem};

mod dispatch;
mod ingress;
mod pump_sink;

pub use dispatch::DispatchError;
use dispatch::{Dispatcher, WorkerLinks};
use pump_sink::{Projection, PumpSink};

/// The execution owner's committed host-monotonic frontier, observed by the planning pacer.
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
        notify: Sender<Result<(), String>>,
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
    Reset {
        pos: Vec<f64>,
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
    /// Arm a resonance buzz across every configured route in one atomic
    /// request. The ingress drains the pipeline and fences the pump once, so
    /// every route starts from a quiescent lane; the pump validates all
    /// routes before it mutates any of them and returns a single token that
    /// covers the whole set.
    Buzz {
        params: crate::pump::BuzzParams,
        notify: Sender<Result<crate::pump::BuzzToken, String>>,
    },
    Shutdown,
}

#[allow(missing_debug_implementations)]
pub struct StreamWorkerHandle {
    sender: Sender<StreamMsg>,
    join_handle: Option<JoinHandle<()>>,
    links: Arc<WorkerLinks>,
    execution_control: Sender<crate::pump::PumpMsg>,
}

#[derive(Debug)]
pub enum StreamWorkerError {
    ChannelClosed,
    ChannelFull,
    ExecutionHalted(String),
}

impl std::fmt::Display for StreamWorkerError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::ChannelClosed => write!(f, "stream worker channel closed"),
            Self::ExecutionHalted(reason) => write!(f, "execution halted: {reason}"),
            Self::ChannelFull => write!(
                f,
                "stream worker input channel full ({INPUT_CHANNEL_CAP} moves)"
            ),
        }
    }
}

impl std::error::Error for StreamWorkerError {}

pub struct ExecutionResources {
    pub sink: crate::pump::WireSink,
    pub callbacks: crate::pump::PumpCallbacks,
    pub history: Arc<Mutex<crate::motion_history::HistoryStore>>,
    pub drain: Arc<crate::drain::DrainLedger>,
    pub router: Arc<Mutex<host_rt::passthrough_queue::PassthroughRouter>>,
    pub anchor: Arc<Mutex<crate::anchor::Anchor>>,
    pub mcu_configs: Vec<crate::mcu_config::McuAxisConfig>,
    pub counter: Arc<AtomicU64>,
    pub transports: Arc<crate::axis_transport::AxisTransports>,
}

pub struct MotionPipeline {
    pub worker: StreamWorkerHandle,
    /// Out-of-band pump control (drip arm/disarm, flush, heartbeats): the
    /// paths that must act while the in-band stream is gated or stalled.
    pub pump_control: Sender<crate::pump::PumpMsg>,
    pub pump_thread: JoinHandle<()>,
}

const TRAJECTORY_CHANNEL_CAP: usize = 16;

pub fn setup_pipeline(
    config: StreamConfig,
    axis_chains: AxisChainSet,
    home_pos: Vec<f64>,
    resources: ExecutionResources,
    pump_channel: (
        Sender<crate::pump::PumpMsg>,
        crossbeam_channel::Receiver<crate::pump::PumpMsg>,
    ),
) -> MotionPipeline {
    let (pump_control, control_rx) = pump_channel;
    let (output, trajectory_rx) = bounded::<TrajectoryItem>(TRAJECTORY_CHANNEL_CAP);
    let frontier: Arc<CommittedFrontier> = Arc::default();
    let links = Arc::new(WorkerLinks::default());
    let mut dispatcher = Dispatcher::new(Arc::clone(&links), Arc::clone(&frontier));
    let admission = Arc::clone(&links);
    let mut projection = Projection {
        transports: resources.transports,
        router: resources.router,
        anchor: resources.anchor,
        mcu_configs: resources.mcu_configs,
        counter: resources.counter,
        frontier: Arc::clone(&frontier),
        frozen_projection: std::collections::HashMap::new(),
    };
    let pump_thread = thread::Builder::new()
        .name("kalico-execution".into())
        .spawn(move || {
            host_rt::thread_prio::elevate_current_thread(
                host_rt::thread_prio::PUMP_RT_PRIORITY,
                "kalico-execution",
            );
            let mut pump = crate::pump::Pump::new(
                resources.sink,
                resources.callbacks,
                Some(resources.history),
                resources.drain,
            );
            pump.run(
                &control_rx,
                &trajectory_rx,
                move |item, pump| {
                    pump.publish_ledger();
                    if let Some(reason) = &pump.fatal_reason {
                        dispatcher.halt(reason);
                    }
                    dispatcher.feed(
                        item,
                        &mut PumpSink {
                            projection: &mut projection,
                            pump,
                        },
                    );
                },
                |item, cohort_active| admission.bypasses_capacity(item, cohort_active),
            );
        })
        .expect("spawn motion execution owner");
    let worker = StreamWorkerHandle::spawn(
        config,
        axis_chains,
        home_pos,
        output,
        links,
        frontier,
        pump_control.clone(),
    );
    MotionPipeline {
        worker,
        pump_control,
        pump_thread,
    }
}

impl StreamWorkerHandle {
    fn spawn(
        config: StreamConfig,
        axis_chains: AxisChainSet,
        home_pos: Vec<f64>,
        output: Sender<TrajectoryItem>,
        links: Arc<WorkerLinks>,
        frontier: Arc<CommittedFrontier>,
        pump: Sender<crate::pump::PumpMsg>,
    ) -> Self {
        let (tx, rx) = bounded(INPUT_CHANNEL_CAP);
        let pipeline = Pipeline::new(config, axis_chains, home_pos.clone(), 0.0);
        let ingress = ingress::Ingress {
            config,
            odometer: home_pos,
            t_next: 0.0,
            pipeline,
            output,
            links: Arc::clone(&links),
            frontier,
            undrained_since: None,
            worst_drain_s: 0.0,
            last_line: 0,
            pump: pump.clone(),
        };
        let join = thread::Builder::new()
            .name("kalico-planning".to_string())
            .spawn(move || ingress.run(rx))
            .expect("spawn motion planning coordinator");
        Self {
            sender: tx,
            join_handle: Some(join),
            links,
            execution_control: pump,
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
        let (notify, done) = bounded(1);
        self.sender
            .send(StreamMsg::Flush { notify })
            .map_err(|_| StreamWorkerError::ChannelClosed)?;
        done.recv()
            .map_err(|_| StreamWorkerError::ChannelClosed)?
            .map_err(StreamWorkerError::ExecutionHalted)
    }

    /// Non-blocking: `ChannelFull` means the caller must retry after
    /// yielding, exactly like `fence_start` — a blocking send here wedges
    /// the klippy reactor for as long as the backpressured pipe takes to
    /// admit one message.
    pub fn flush_try_start(
        &self,
    ) -> Result<crossbeam_channel::Receiver<Result<(), String>>, StreamWorkerError> {
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

    pub fn discard_pending(&self) {
        self.links.discard.store(true, Ordering::Release);
    }

    pub fn reset(&self, pos: Vec<f64>) -> Result<(), StreamWorkerError> {
        self.sender
            .send(StreamMsg::Reset { pos })
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

    /// Queue one buzz request behind everything already submitted. The
    /// receiver yields the pump's single verdict: a token covering every
    /// route, or the first route that failed validation with nothing armed.
    pub fn submit_buzz(
        &self,
        params: crate::pump::BuzzParams,
    ) -> Result<
        crossbeam_channel::Receiver<Result<crate::pump::BuzzToken, String>>,
        StreamWorkerError,
    > {
        let (notify, result) = crossbeam_channel::bounded(1);
        self.sender
            .send(StreamMsg::Buzz { params, notify })
            .map_err(|_| StreamWorkerError::ChannelClosed)?;
        Ok(result)
    }

    #[must_use]
    pub fn last_move_time(&self) -> f64 {
        f64::from_bits(self.links.last_move_time_bits.load(Ordering::Acquire))
    }

    pub fn shutdown(&mut self) {
        self.prepare_shutdown();
        let _ = self
            .execution_control
            .send_timeout(crate::pump::PumpMsg::Shutdown, SHUTDOWN_SEND_TIMEOUT);
        let _ = self
            .sender
            .send_timeout(StreamMsg::Shutdown, SHUTDOWN_SEND_TIMEOUT);
        let deadline = Instant::now() + SHUTDOWN_JOIN_TIMEOUT;
        if let Some(h) = self.join_handle.take() {
            join_worker_thread(h, deadline);
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
