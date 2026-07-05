use std::collections::{HashMap, HashSet};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::PyDict;

use host_rt::clock::RealClock;
use host_rt::host_io::parser::{DataDictionary, FieldValue, MsgProtoParser};
use host_rt::host_io::{McuHostIo, McuHostIoConfig};
use host_rt::mcu_serial_conn::McuSerialConn;
use host_rt::passthrough_queue::PassthroughRouter;

use crate::classify;
use crate::config::{self, PlannerConfig};
use crate::kinematics::SPATIAL_AXES;
use crate::mcu_config::{McuAxisConfig, McuCaps, build_mcu_configs};
use crate::types::mcu_handle_from_raw;
use crate::worker::{StreamWorkerError, StreamWorkerHandle};

mod ethercat_endpoint;
mod homing_api;
mod motion_caps;
mod passthrough;
mod planner_api;
mod runtime_caps;
mod servo;
mod state;
mod telemetry;

#[cfg(test)]
use ethercat_endpoint::{EndpointClaimError, endpoint_args};
use ethercat_endpoint::{
    arm_endpoint_death_watchdog, handshake_ethercat_endpoint, message_for_claim_error,
    poll_socket_ready, report_ethercat_endpoint_death, spawn_ethercat_endpoint,
};
use motion_caps::{
    axis_ring_depth, drip_cohort_participants, require_events_dir_for_mcu_transport,
    resolve_motion_caps, ring_depth_for_axis_inner,
};
#[cfg(test)]
use runtime_caps::place_motor_response;
use runtime_caps::{
    collect_motor_positions_inner, query_ethercat_runtime_caps, query_runtime_caps,
    require_positive, slot_for_axis,
};
use state::{EthercatDrive, FlushWait, HomingRun, McuConnection};

fn abort_after_tracing_appender_drains() {
    let _ = std::io::Write::flush(&mut std::io::stderr());
    std::thread::sleep(std::time::Duration::from_millis(100));
    if std::env::var_os("NO_EXIT_ON_FAULT").is_none() {
        std::process::abort();
    }
}

const DRAIN_TIMEOUT: Duration = Duration::from_secs(60);

const ETHERCAT_CLOCK_FREQ_HZ: u32 = 1_000_000_000;

fn router_err(e: host_rt::passthrough_queue::RouterError) -> PyErr {
    PyRuntimeError::new_err(e.to_string())
}

fn planner_err(e: StreamWorkerError) -> PyErr {
    PyRuntimeError::new_err(e.to_string())
}

fn open_serial_with_retry(
    serial_path: &str,
    effective_baud: u32,
    is_pipe: bool,
    config: &McuHostIoConfig,
    deadline: Instant,
    timeout_s: f64,
) -> PyResult<McuHostIo> {
    let host_io = loop {
        let result = if is_pipe {
            #[cfg(target_family = "unix")]
            {
                McuHostIo::open_pipe_with_config(serial_path, config.clone())
            }
            #[cfg(not(target_family = "unix"))]
            {
                McuHostIo::open_with_config(serial_path, effective_baud, config.clone())
            }
        } else {
            McuHostIo::open_with_config(serial_path, effective_baud, config.clone())
        };
        match result {
            Ok(io) => break io,
            Err(e) => {
                if Instant::now() >= deadline {
                    return Err(PyRuntimeError::new_err(format!(
                        "attach_serial: could not open {serial_path} within {timeout_s}s: {e}"
                    )));
                }
                tracing::warn!(
                    subsystem = "mcu-comms",
                    event = "attach_open_retry",
                    serial_path,
                    error = %e,
                    "attach_serial: retrying open"
                );
                std::thread::sleep(Duration::from_millis(500));
            }
        }
    };
    Ok(host_io)
}

/// Backstop only. The continuity commit drains at every blend, so the buffer
/// normally hovers near the open-tail length past the finality barrier; this
/// force-drain to rest fires solely when no clean seam exists within reach. Set
/// well above a realistic open tail so a dense (but normal) stream never trips it.
const STREAM_MAX_BUFFER_MOVES: usize = 128;
/// Velocity-profile ODE/sampling tolerance for the streaming planner, in v²
/// units. The offline default (1e-7) drives the adaptive RK4 to a precision far
/// below the physical noise floor — ~0.015 mm/s velocity error at this value on
/// a 300 mm/s move — at ~9× the host cost. Streaming runs on the Pi against a
/// real-time playhead, so it is tuned to the hardware budget: at 1e-4 a 40-move
/// burst plans in ~0.4 s instead of ~3.8 s, with trajectory time unchanged to
/// five significant figures (the residual is non-monotonic integration noise,
/// not a slower path).
const STREAM_INTEGRATION_TOL: f64 = 1e-4;

#[pyclass(name = "MotionEngine")]
#[allow(missing_debug_implementations)]
pub struct PyMotionEngine {
    router: Arc<Mutex<PassthroughRouter>>,
    parser: Arc<Mutex<Option<Arc<MsgProtoParser>>>>,
    mcus: Arc<Mutex<HashMap<u32, McuConnection>>>,
    #[allow(dead_code)]
    planner: Mutex<Option<StreamWorkerHandle>>,
    planner_config: Mutex<PlannerConfig>,
    commanded_pos: Mutex<[f64; 3]>,
    last_g5_pq: Mutex<Option<(f64, f64)>>,
    mcu_axis_configs: Arc<Mutex<Vec<McuAxisConfig>>>,
    dispatched_segments: Arc<AtomicU64>,
    pump_backlog: Arc<AtomicU64>,
    dispatch_anchor: Arc<Mutex<crate::anchor::Anchor>>,
    fallback_clock_conversions: Arc<AtomicU64>,
    clock_freqs: Arc<Mutex<HashMap<u32, f64>>>,
    nominal_clock_freqs: Arc<Mutex<HashMap<u32, u32>>>,
    events_dir: Mutex<Option<std::path::PathBuf>>,
    pump_tx: Arc<Mutex<Option<crossbeam_channel::Sender<crate::pump::PumpMsg>>>>,
    pump_thread: Mutex<Option<JoinHandle<()>>>,
    live_position_cache: Arc<
        Mutex<(
            std::collections::HashMap<String, (f64, f64)>,
            std::time::Instant,
        )>,
    >,
    position_poll_thread: Mutex<Option<std::thread::JoinHandle<()>>>,
    position_poll_stop: Arc<std::sync::atomic::AtomicBool>,
    drain: std::sync::Arc<crate::drain::DrainSync>,
    active_drip_cohort: Arc<Mutex<Option<u64>>>,
    motion_history: Arc<Mutex<crate::motion_history::HistoryStore>>,
    homing_run: Arc<Mutex<Option<HomingRun>>>,
    pending_trip: Arc<Mutex<Option<(u32, u8, u64)>>>,
    pending_flushes: Mutex<HashMap<u64, FlushWait>>,
    pending_drain_flush: Mutex<Option<crossbeam_channel::Receiver<Option<std::time::Instant>>>>,
    next_flush_id: std::sync::atomic::AtomicU64,
    // Monotonic id stamped on every streamed move as its `source.start_line`.
    // The continuity-commit drains the look-ahead buffer by line number
    // (`front.start_line < keep_line`) and detects consumed blend heads by line
    // equality, so each move MUST carry a distinct increasing id. Passing a
    // constant (0) makes the drain a no-op — the buffer never empties and every
    // commit re-dispatches the whole accumulated path from the start.
    move_seq: std::sync::atomic::AtomicU64,
    homing_result:
        Mutex<Option<crossbeam_channel::Receiver<Result<([f64; 3], [f64; 3], u64), String>>>>,
    latched_drive_fault: Arc<Mutex<HashMap<u32, u16>>>,
    latched_endpoint_death: Arc<Mutex<HashMap<u32, String>>>,
    remote_triggers: Mutex<HashMap<u8, (u32, host_rt::host_io::InterceptorId)>>,
    endpoint_calls: crate::bg_call::BgCalls,
    shut_down: AtomicBool,
}

#[pymethods]
impl PyMotionEngine {
    #[new]
    fn new() -> Self {
        let clock: Arc<dyn host_rt::clock::Clock + Send + Sync> = Arc::new(RealClock);
        Self {
            router: Arc::new(Mutex::new(PassthroughRouter::with_clock(clock))),
            parser: Arc::new(Mutex::new(None)),
            mcus: Arc::new(Mutex::new(HashMap::new())),
            planner: Mutex::new(None),
            planner_config: Mutex::new(PlannerConfig::default()),
            commanded_pos: Mutex::new([0.0; 3]),
            last_g5_pq: Mutex::new(None),
            mcu_axis_configs: Arc::new(Mutex::new(Vec::new())),
            dispatched_segments: Arc::new(AtomicU64::new(0)),
            pump_backlog: Arc::new(AtomicU64::new(0)),
            dispatch_anchor: Arc::new(Mutex::new(crate::anchor::Anchor::new())),
            fallback_clock_conversions: Arc::new(AtomicU64::new(0)),
            clock_freqs: Arc::new(Mutex::new(HashMap::new())),
            nominal_clock_freqs: Arc::new(Mutex::new(HashMap::new())),
            events_dir: Mutex::new(None),
            pump_tx: Arc::new(Mutex::new(None)),
            pump_thread: Mutex::new(None),
            live_position_cache: Arc::new(Mutex::new((
                std::collections::HashMap::new(),
                std::time::Instant::now(),
            ))),
            position_poll_thread: Mutex::new(None),
            position_poll_stop: Arc::new(std::sync::atomic::AtomicBool::new(false)),
            drain: std::sync::Arc::new(crate::drain::DrainSync::new()),
            active_drip_cohort: Arc::new(Mutex::new(None)),
            motion_history: Arc::new(Mutex::new(crate::motion_history::HistoryStore::default())),
            homing_run: Arc::new(Mutex::new(None)),
            pending_trip: Arc::new(Mutex::new(None)),
            pending_flushes: Mutex::new(HashMap::new()),
            pending_drain_flush: Mutex::new(None),
            next_flush_id: std::sync::atomic::AtomicU64::new(1),
            move_seq: std::sync::atomic::AtomicU64::new(0),
            homing_result: Mutex::new(None),
            latched_drive_fault: Arc::new(Mutex::new(HashMap::new())),
            latched_endpoint_death: Arc::new(Mutex::new(HashMap::new())),
            remote_triggers: Mutex::new(HashMap::new()),
            endpoint_calls: crate::bg_call::BgCalls::default(),
            shut_down: AtomicBool::new(false),
        }
    }

    fn version(&self) -> &'static str {
        env!("CARGO_PKG_VERSION")
    }

    fn init_logging(&self, events_dir: String) -> PyResult<()> {
        let path = std::path::Path::new(&events_dir);
        crate::logging::init_logging(path).map_err(|e| {
            pyo3::exceptions::PyRuntimeError::new_err(format!("init_logging failed: {e}"))
        })?;
        let mut guard = self.events_dir.lock().unwrap_or_else(|p| p.into_inner());
        *guard = Some(path.to_path_buf());
        Ok(())
    }

    #[pyo3(signature = (session_id, print_id=String::new()))]
    fn set_session_context(&self, session_id: String, print_id: String) {
        crate::logging::set_context(session_id, print_id);
    }

    #[pyo3(signature = (label, serial_path, baud))]
    fn claim_mcu(&self, label: &str, serial_path: &str, baud: u32) -> PyResult<u32> {
        let _ = (serial_path, baud);
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        let handle = router.claim_mcu(label);
        let raw = handle.raw();
        self.mcus.lock().unwrap_or_else(|p| p.into_inner()).insert(
            raw,
            McuConnection {
                label: label.to_owned(),
                host_io: None,
                runtime_rx_priority: None,
                runtime_rx_bulk: None,
                runtime_caps: None,
                identify_caps: 0,
                mcu_transport_supported: false,
                ethercat_socket: None,
                endpoint_process: None,
                endpoint_conn: None,
                ethercat_slot_axes: Vec::new(),
            },
        );
        Ok(raw)
    }

    #[pyo3(signature = (label, socket_path, interface, endpoint_binary, dynamics_profile, drives))]
    fn claim_ethercat_node(
        &self,
        label: &str,
        socket_path: &str,
        interface: &str,
        endpoint_binary: &str,
        dynamics_profile: Option<String>,
        drives: Vec<EthercatDrive>,
    ) -> PyResult<u32> {
        if drives.is_empty() {
            return Err(PyRuntimeError::new_err(format!(
                "ethercat {label}: claim received no drives"
            )));
        }
        let mut drives = drives;
        drives.sort_by_key(|d| d.1);
        let slot_axes: Vec<usize> = drives.iter().map(|d| d.1).collect();

        if let Err(e) = std::fs::remove_file(socket_path) {
            if e.kind() != std::io::ErrorKind::NotFound {
                return Err(PyRuntimeError::new_err(format!(
                    "ethercat {label}: failed to remove stale socket {socket_path}: {e}"
                )));
            }
        }

        let events_dir = self
            .events_dir
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone();
        let mut child = spawn_ethercat_endpoint(
            endpoint_binary,
            interface,
            socket_path,
            dynamics_profile.as_deref(),
            events_dir.as_deref(),
            &drives,
        )
        .map_err(|e| {
            PyRuntimeError::new_err(format!("ethercat {label}: endpoint failed to start — {e}"))
        })?;

        let socket_deadline = Instant::now() + Duration::from_secs(15);
        if let Err(detail) = poll_socket_ready(socket_path, socket_deadline, &mut child) {
            let _ = child.kill();
            let _ = child.wait();
            return Err(PyRuntimeError::new_err(format!(
                "ethercat {label}: {detail}"
            )));
        }

        let handshake_deadline = Instant::now() + Duration::from_secs(25);
        let conn = handshake_ethercat_endpoint(socket_path, handshake_deadline).map_err(|e| {
            let _ = child.kill();
            let _ = child.wait();
            PyRuntimeError::new_err(message_for_claim_error(label, interface, &e))
        })?;

        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        let handle = router.claim_mcu(label);
        let raw = handle.raw();
        drop(router);
        self.register_ethercat_mcu(raw, label, socket_path, child, conn, slot_axes);
        Ok(raw)
    }

    fn release_mcu(&self, handle: u32) -> PyResult<()> {
        let Some(mut conn) = ({
            let mut mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            mcus.remove(&handle)
        }) else {
            return Ok(());
        };

        let mut endpoint_process = conn.endpoint_process.take();
        let endpoint_conn = conn.endpoint_conn.take();

        drop(endpoint_conn);

        if let Some(ref mut child) = endpoint_process {
            let pid = libc::pid_t::try_from(child.id()).expect("child PID exceeds pid_t range");

            #[allow(unsafe_code)]
            let _ = unsafe { libc::kill(pid, libc::SIGTERM) };

            let reap_deadline = Instant::now() + Duration::from_secs(5);
            loop {
                match child.try_wait() {
                    Ok(Some(_)) => break,
                    Ok(None) => {}
                    Err(_) => break,
                }
                if Instant::now() >= reap_deadline {
                    let _ = child.kill();
                    let _ = child.wait();
                    tracing::warn!(
                        subsystem = "engine",
                        event = "release_mcu_endpoint_sigkill",
                        pid,
                        "release_mcu: ethercat endpoint did not exit within 5 s after SIGTERM — SIGKILL sent"
                    );
                    break;
                }
                std::thread::sleep(Duration::from_millis(50));
            }
        }

        drop(conn);

        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        router.release_mcu(mcu_handle_from_raw(handle));
        Ok(())
    }

    fn shutdown(&self) {
        if self.shut_down.swap(true, Ordering::SeqCst) {
            tracing::debug!(
                subsystem = "engine",
                event = "shutdown_called_twice",
                "engine.shutdown() called twice (idempotent no-op)"
            );
            return;
        }

        let planner = self
            .planner
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .take();
        if let Some(mut p) = planner {
            p.shutdown();
        }

        let pump_join = {
            let tx = self
                .pump_tx
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .take();
            if let Some(tx) = tx {
                let _ = tx.send(crate::pump::PumpMsg::Shutdown);
            }
            self.pump_thread
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .take()
        };
        if let Some(h) = pump_join {
            if let Err(e) = h.join() {
                tracing::error!(
                    subsystem = "engine",
                    event = "shutdown_pump_join_panicked",
                    error = ?e,
                    "engine.shutdown(): push-pieces-pump join panicked"
                );
            }
        }

        self.position_poll_stop
            .store(true, std::sync::atomic::Ordering::Relaxed);
        if let Some(h) = self
            .position_poll_thread
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .take()
        {
            if let Err(e) = h.join() {
                log::error!("engine.shutdown(): live-position-poll join panicked: {e:?}");
            }
        }

        let handles: Vec<u32> = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            mcus.keys().copied().collect()
        };
        for h in handles {
            if let Err(e) = self.release_mcu(h) {
                tracing::error!(
                    subsystem = "engine",
                    event = "shutdown_release_mcu_failed",
                    mcu_handle = h,
                    error = %e,
                    "engine.shutdown(): release_mcu failed"
                );
            }
        }
    }

    fn detach_serial(&self, mcu_handle: u32) -> PyResult<()> {
        let mut mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
        if let Some(conn) = mcus.get_mut(&mcu_handle) {
            conn.runtime_rx_priority = None;
            conn.runtime_rx_bulk = None;
            conn.host_io = None;
        }
        Ok(())
    }

    #[pyo3(signature = (mcu_handle, serial_path, baud, timeout_s = 30.0, klippy_non_critical = false, expect_native = true))]
    fn attach_serial(
        &self,
        mcu_handle: u32,
        serial_path: &str,
        baud: u32,
        timeout_s: f64,
        klippy_non_critical: bool,
        expect_native: bool,
    ) -> PyResult<()> {
        use std::time::{Duration, Instant};
        let deadline = Instant::now() + Duration::from_secs_f64(timeout_s);
        let effective_baud = if baud == 0 { 250_000 } else { baud };
        let config = McuHostIoConfig::default();

        if self.try_reuse_existing_connection(
            mcu_handle,
            serial_path,
            klippy_non_critical,
            expect_native,
        )? {
            return Ok(());
        }

        let mcu_label: String = self.with_mcu(
            mcu_handle,
            |h| format!("attach_serial: unknown mcu_handle {h} (claim_mcu not called)"),
            |conn| {
                conn.runtime_rx_priority = None;
                conn.runtime_rx_bulk = None;
                conn.host_io = None;
                Ok(conn.label.clone())
            },
        )?;

        let is_pipe = baud == 0
            || serial_path.starts_with("/tmp/")
            || serial_path.starts_with("/dev/pts/")
            || serial_path.contains("klipper_host")
            || serial_path.contains("klipper_sim");

        let host_io = open_serial_with_retry(
            serial_path,
            effective_baud,
            is_pipe,
            &config,
            deadline,
            timeout_s,
        )?;

        self.register_freshly_attached_mcu(
            mcu_handle,
            serial_path,
            &mcu_label,
            klippy_non_critical,
            expect_native,
            host_io,
        )
    }

    fn get_mcu_capabilities(&self, mcu_handle: u32) -> PyResult<u64> {
        self.with_mcu(
            mcu_handle,
            |h| format!("get_mcu_capabilities: unknown mcu_handle {h}"),
            |conn| Ok(conn.identify_caps),
        )
    }

    fn ring_depth_for_axis(&self, mcu_handle: u32, axis: u8) -> PyResult<u16> {
        let configs = self
            .mcu_axis_configs
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        ring_depth_for_axis_inner(&configs, mcu_handle, axis).map_err(PyRuntimeError::new_err)
    }
}

impl Drop for PyMotionEngine {
    fn drop(&mut self) {
        self.shutdown();
    }
}

#[derive(Clone)]
pub(crate) struct TripDeps {
    homing_run: Arc<Mutex<Option<HomingRun>>>,
    pending_trip: Arc<Mutex<Option<(u32, u8, u64)>>>,
    active_drip_cohort: Arc<Mutex<Option<u64>>>,
    pump_tx: Arc<Mutex<Option<crossbeam_channel::Sender<crate::pump::PumpMsg>>>>,
    mcus: Arc<Mutex<HashMap<u32, McuConnection>>>,
    router: Arc<Mutex<PassthroughRouter>>,
    motion_history: Arc<Mutex<crate::motion_history::HistoryStore>>,
    mcu_axis_configs: Arc<Mutex<Vec<McuAxisConfig>>>,
}

impl PyMotionEngine {
    fn with_mcu<R>(
        &self,
        handle: u32,
        unknown_mcu_err: impl FnOnce(u32) -> String,
        f: impl FnOnce(&mut McuConnection) -> PyResult<R>,
    ) -> PyResult<R> {
        let mut mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
        let mc = mcus
            .get_mut(&handle)
            .ok_or_else(|| PyRuntimeError::new_err(unknown_mcu_err(handle)))?;
        f(mc)
    }

    fn try_reuse_existing_connection(
        &self,
        mcu_handle: u32,
        serial_path: &str,
        klippy_non_critical: bool,
        expect_native: bool,
    ) -> PyResult<bool> {
        let existing_io: Option<Arc<McuHostIo>> = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            mcus.get(&mcu_handle)
                .and_then(|conn| conn.host_io.as_ref().map(Arc::clone))
        };
        if let Some(io) = existing_io {
            if io.is_alive() {
                tracing::info!(
                    subsystem = "mcu-comms",
                    event = "attach_reuse_connection",
                    serial_path,
                    "attach_serial: reusing existing connection (reactor alive, skipping close/reopen)"
                );

                let (rx_priority, rx_bulk) = io.take_runtime_event_subscription().map_err(|e| {
                    PyRuntimeError::new_err(format!(
                        "attach_serial: runtime_event re-subscribe: {e:?}"
                    ))
                })?;

                let (mcu_transport_supported, identify_caps) = if !expect_native {
                    tracing::info!(
                        subsystem = "mcu-comms",
                        event = "attach_identify_skipped_reuse",
                        serial_path,
                        "attach_serial: kalico identify skipped on reuse (plugin-attached peripheral, not declared via an [mcu] section)"
                    );
                    (false, 0u64)
                } else {
                    match io.kalico_identify(std::time::Duration::from_secs(5)) {
                        Ok(out) => {
                            tracing::info!(
                                subsystem = "mcu-comms",
                                event = "attach_reidentified",
                                serial_path,
                                reset_epoch = out.reset_epoch,
                                capabilities = out.capabilities,
                                "attach_serial: kalico re-identified (reset_epoch/caps as hex)"
                            );
                            (true, out.capabilities)
                        }
                        Err(e) => {
                            tracing::warn!(
                                subsystem = "mcu-comms",
                                event = "attach_identify_timeout_reuse",
                                serial_path,
                                error = %e,
                                "attach_serial: kalico_identify timed out on reuse; treating as Klipper-protocol-only"
                            );
                            (false, 0u64)
                        }
                    }
                };

                let runtime_caps = if mcu_transport_supported {
                    match query_runtime_caps(&io, std::time::Duration::from_secs(2)) {
                        Ok(caps) => {
                            tracing::debug!(
                                subsystem = "mcu-comms",
                                event = "attach_runtime_caps_reuse",
                                serial_path,
                                total_piece_memory = caps.total_piece_memory,
                                "[caps-trace] attach_serial reuse: runtime caps"
                            );
                            Some(caps)
                        }
                        Err(e) => {
                            return Err(PyRuntimeError::new_err(format!(
                                "attach_serial: QueryRuntimeCaps failed for {serial_path} \
                                 ({e}) — a kalico-native MCU must report runtime caps; \
                                 firmware is too old, mismatched, or not flashed. \
                                 Refusing to attach with guessed caps."
                            )));
                        }
                    }
                } else {
                    None
                };

                let critical = mcu_transport_supported && !klippy_non_critical;
                io.set_critical(critical);
                tracing::info!(
                    subsystem = "mcu-comms",
                    event = "attach_criticality_reuse",
                    serial_path,
                    critical,
                    mcu_transport = mcu_transport_supported,
                    klippy_non_critical,
                    "attach_serial: reuse — criticality set"
                );

                self.with_mcu(
                    mcu_handle,
                    |h| format!("attach_serial: unknown mcu_handle {h}"),
                    |conn| {
                        conn.runtime_rx_priority = Some(rx_priority);
                        conn.runtime_rx_bulk = Some(rx_bulk);
                        conn.runtime_caps = runtime_caps;
                        conn.identify_caps = identify_caps;
                        conn.mcu_transport_supported = mcu_transport_supported;
                        Ok(())
                    },
                )?;
                return Ok(true);
            }
        }
        Ok(false)
    }

    fn register_freshly_attached_mcu(
        &self,
        mcu_handle: u32,
        serial_path: &str,
        mcu_label: &str,
        klippy_non_critical: bool,
        expect_native: bool,
        host_io: McuHostIo,
    ) -> PyResult<()> {
        let (rx_priority, rx_bulk) = host_io.take_runtime_event_subscription().map_err(|e| {
            PyRuntimeError::new_err(format!("attach_serial: runtime_event subscribe: {e:?}"))
        })?;

        let (mcu_transport_supported, identify_caps) = if !expect_native {
            tracing::info!(
                subsystem = "mcu-comms",
                event = "attach_identify_skipped",
                serial_path,
                "attach_serial: kalico identify skipped (plugin-attached peripheral, not declared via an [mcu] section)"
            );
            (false, 0u64)
        } else {
            match host_io.kalico_identify(std::time::Duration::from_secs(5)) {
                Ok(out) => {
                    tracing::info!(
                        subsystem = "mcu-comms",
                        event = "attach_identified",
                        serial_path,
                        reset_epoch = out.reset_epoch,
                        capabilities = out.capabilities,
                        "attach_serial: kalico identified (reset_epoch/caps as hex)"
                    );
                    (true, out.capabilities)
                }
                Err(e) => {
                    tracing::warn!(
                        subsystem = "mcu-comms",
                        event = "attach_identify_timeout",
                        serial_path,
                        error = %e,
                        "attach_serial: kalico_identify timed out; continuing attach as a Klipper-protocol-only MCU"
                    );
                    (false, 0u64)
                }
            }
        };

        let runtime_caps = if mcu_transport_supported {
            match query_runtime_caps(&host_io, std::time::Duration::from_secs(2)) {
                Ok(caps) => {
                    tracing::debug!(
                        subsystem = "mcu-comms",
                        event = "attach_runtime_caps",
                        serial_path,
                        total_piece_memory = caps.total_piece_memory,
                        "[caps-trace] attach_serial: runtime caps"
                    );
                    Some(caps)
                }
                Err(e) => {
                    return Err(PyRuntimeError::new_err(format!(
                        "attach_serial: QueryRuntimeCaps failed for {serial_path} \
                         ({e}) — a kalico-native MCU must report runtime caps; \
                         firmware is too old, mismatched, or not flashed. \
                         Refusing to attach with guessed caps."
                    )));
                }
            }
        } else {
            None
        };

        let critical = mcu_transport_supported && !klippy_non_critical;
        host_io.set_critical(critical);
        tracing::info!(
            subsystem = "mcu-comms",
            event = "attach_criticality",
            serial_path,
            critical,
            mcu_transport = mcu_transport_supported,
            klippy_non_critical,
            "attach_serial: criticality set"
        );

        let host_io_arc = Arc::new(host_io);

        {
            let events_dir_guard = self
                .events_dir
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner);
            require_events_dir_for_mcu_transport(
                mcu_transport_supported,
                events_dir_guard.as_deref(),
                mcu_label,
            )
            .map_err(PyRuntimeError::new_err)?;
        }

        if mcu_transport_supported {
            let events_dir_guard = self.events_dir.lock().unwrap_or_else(|p| p.into_inner());
            if let Some(ref dir) = *events_dir_guard {
                use crate::logging::writer::{
                    DEFAULT_BACKUP_COUNT, DEFAULT_MAX_BYTES, FSYNC_INTERVAL, RotatingJsonlWriter,
                };
                let source = mcu_label.to_owned();
                let jsonl_path = dir.join(format!("{source}.jsonl"));
                match RotatingJsonlWriter::new(
                    &jsonl_path,
                    DEFAULT_MAX_BYTES,
                    DEFAULT_BACKUP_COUNT,
                    FSYNC_INTERVAL,
                ) {
                    Ok(writer) => {
                        let arc_writer = Arc::new(Mutex::new(writer));
                        let mcu_h = mcu_handle_from_raw(mcu_handle);
                        let hook = crate::mcu_log::build_mcu_log_hook(
                            Arc::clone(&self.router),
                            mcu_h,
                            arc_writer,
                            source,
                        );
                        host_io_arc.set_mcu_log_hook(Box::new(hook));
                    }
                    Err(e) => {
                        tracing::warn!(
                            subsystem = "mcu-comms",
                            event = "attach_mcu_log_open_failed",
                            jsonl_path = %jsonl_path.display(),
                            error = %e,
                            "attach_serial: mcu-log: failed to open jsonl writer"
                        );
                    }
                }
            } else {
                unreachable!(
                    "attach_serial: events_dir is None for a kalico-native MCU \
                     — require_events_dir_for_mcu_transport should have \
                     rejected this call before reaching hook wiring"
                );
            }
        }

        self.with_mcu(
            mcu_handle,
            |h| format!("attach_serial: unknown mcu_handle {h}"),
            |conn| {
                conn.host_io = Some(host_io_arc);
                conn.runtime_rx_priority = Some(rx_priority);
                conn.runtime_rx_bulk = Some(rx_bulk);
                conn.runtime_caps = runtime_caps;
                conn.identify_caps = identify_caps;
                conn.mcu_transport_supported = mcu_transport_supported;
                Ok(())
            },
        )
    }

    fn ethercat_conn(&self, mcu_handle: u32, what: &str) -> PyResult<Arc<McuSerialConn>> {
        let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
        let mc = mcus.get(&mcu_handle).ok_or_else(|| {
            PyRuntimeError::new_err(format!("{what}: unknown mcu_handle {mcu_handle}"))
        })?;
        mc.endpoint_conn.clone().ok_or_else(|| {
            PyRuntimeError::new_err(format!(
                "{what}: mcu {mcu_handle} ({}) is not an EtherCAT endpoint",
                mc.label
            ))
        })
    }

    fn host_io_for_mcu(&self, caller: &str, mcu: u32) -> PyResult<Arc<McuHostIo>> {
        let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
        let conn = mcus.get(&mcu).ok_or_else(|| {
            PyRuntimeError::new_err(format!("{caller}: unknown mcu_handle {mcu}"))
        })?;
        conn.host_io.as_ref().cloned().ok_or_else(|| {
            PyRuntimeError::new_err(format!(
                "{caller}: attach_serial has not been called for this MCU"
            ))
        })
    }

    fn handle_endstop_trip(&self, event_mcu: u32, endstop_id: u8, trip_clock: u64) {
        dispatch_endstop_trip(&self.trip_deps(), event_mcu, endstop_id, trip_clock);
    }
}

pub(crate) fn dispatch_endstop_trip(
    deps: &TripDeps,
    event_mcu: u32,
    endstop_id: u8,
    trip_clock: u64,
) {
    let run_opt: Option<HomingRun> = {
        let mut guard = deps.homing_run.lock().unwrap_or_else(|p| p.into_inner());
        guard.take()
    };
    let run = match run_opt {
        None => {
            tracing::warn!(
                subsystem = "trip-relay",
                event = "early_trip_buffered",
                mcu = event_mcu,
                endstop_id,
                trip_clock,
                "terminal report arrived before the homing run was registered — buffered"
            );
            *deps.pending_trip.lock().unwrap_or_else(|p| p.into_inner()) =
                Some((event_mcu, endstop_id, trip_clock));
            return;
        }
        Some(r) => r,
    };
    if run.endstop_id != endstop_id || run.endstop_mcu != event_mcu {
        let mut guard = deps.homing_run.lock().unwrap_or_else(|p| p.into_inner());
        *guard = Some(run);
        return;
    }

    {
        let mut cohort_guard = deps
            .active_drip_cohort
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        *cohort_guard = None;
    }

    let pump_tx_opt = deps
        .pump_tx
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .clone();

    let transports: HashMap<u32, Arc<dyn host_rt::mcu_call::McuCall>> = {
        let mcus = deps.mcus.lock().unwrap_or_else(|p| p.into_inner());
        mcus.iter()
            .filter_map(|(&id, conn)| {
                if let Some(io) = conn.host_io.as_ref() {
                    Some((id, Arc::clone(io) as Arc<dyn host_rt::mcu_call::McuCall>))
                } else {
                    conn.endpoint_conn
                        .as_ref()
                        .map(|ec| (id, Arc::clone(ec) as Arc<dyn host_rt::mcu_call::McuCall>))
                }
            })
            .collect()
    };

    let router_arc = Arc::clone(&deps.router);
    let history_arc = Arc::clone(&deps.motion_history);
    let configs: Vec<McuAxisConfig> = deps
        .mcu_axis_configs
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .clone();

    std::thread::Builder::new()
        .name("homing-trip-handler".into())
        .spawn(move || {
            let stop_timeout = Duration::from_secs(3);

            let stepper_mcu_ids: std::collections::HashSet<u32> =
                run.all_axis_keys.iter().map(|k| k.mcu_id).collect();

            if let Some(tx) = pump_tx_opt.as_ref() {
                let _ = tx.send(crate::pump::PumpMsg::Flush(run.all_axis_keys.clone()));
                let _ = tx.send(crate::pump::PumpMsg::DripDisarm(run.cohort));
            }

            use mcu_protocol::codec::Decode as _;
            let stop_call = |mcu_id: u32| -> Result<mcu_protocol::messages::StopResponse, String> {
                let transport = transports
                    .get(&mcu_id)
                    .ok_or_else(|| format!("Stop: no transport for mcu {mcu_id}"))?;
                let (_kind, body) = transport
                    .mcu_call(mcu_protocol::MessageKind::Stop, Vec::new(), stop_timeout)
                    .map_err(|e| format!("Stop call failed for mcu {mcu_id}: {e:?}"))?;
                mcu_protocol::messages::StopResponse::decode(&body)
                    .map_err(|e| format!("Stop decode failed for mcu {mcu_id}: {e:?}"))
            };

            let discard_clock = match crate::homing::broadcast_stop(
                &stepper_mcu_ids,
                run.axis_key.mcu_id,
                stop_call,
            ) {
                Ok(c) => c,
                Err(e) => {
                    let _ = run.notify.send(Err(e));
                    return;
                }
            };

            let axis_key = run.axis_key;
            let reconstruct_cartesian = |source_mcu: u32, clock: u64| -> Result<[f64; 3], String> {
                let cfg = configs
                    .iter()
                    .find(|c| c.mcu_id == axis_key.mcu_id)
                    .ok_or_else(|| {
                        format!("EndstopTrip: no axis config for mcu {}", axis_key.mcu_id)
                    })?;
                crate::homing::reconstruct_cartesian_position(
                    source_mcu,
                    clock,
                    cfg,
                    &router_arc,
                    &history_arc,
                    run.window_start_host,
                )
            };

            let outcome = reconstruct_cartesian(run.endstop_mcu, trip_clock).and_then(|trip| {
                reconstruct_cartesian(axis_key.mcu_id, discard_clock)
                    .map(|final_pos| (trip, final_pos, trip_clock))
            });

            let outcome = outcome.and_then(|positions| {
                if let Some(tx) = pump_tx_opt.as_ref() {
                    let (ack_tx, ack_rx) = std::sync::mpsc::sync_channel(1);
                    let _ = tx.send(crate::pump::PumpMsg::Barrier(ack_tx));
                    if ack_rx.recv_timeout(Duration::from_secs(1)).is_err() {
                        return Err("EndstopTrip: pump did not acknowledge the flush barrier \
                                 before stream resume"
                            .into());
                    }
                }
                for &mcu_id in &stepper_mcu_ids {
                    let transport = transports
                        .get(&mcu_id)
                        .ok_or_else(|| format!("ResumeStream: no transport for mcu {mcu_id}"))?;
                    let (_kind, body) = transport
                        .mcu_call(
                            mcu_protocol::MessageKind::ResumeStream,
                            Vec::new(),
                            stop_timeout,
                        )
                        .map_err(|e| format!("ResumeStream call failed for mcu {mcu_id}: {e:?}"))?;
                    let resp = mcu_protocol::messages::ResumeStreamResponse::decode(&body)
                        .map_err(|e| {
                            format!("ResumeStream decode failed for mcu {mcu_id}: {e:?}")
                        })?;
                    if resp.result != 0 {
                        return Err(format!(
                            "ResumeStream rejected by mcu {mcu_id}: result={}",
                            resp.result
                        ));
                    }
                }
                Ok(positions)
            });
            let _ = run.notify.send(outcome);
        })
        .expect("spawn homing-trip-handler");
}

impl PyMotionEngine {
    fn register_ethercat_mcu(
        &self,
        raw: u32,
        label: &str,
        socket_path: &str,
        child: std::process::Child,
        conn: McuSerialConn,
        slot_axes: Vec<usize>,
    ) {
        self.mcus.lock().unwrap_or_else(|p| p.into_inner()).insert(
            raw,
            McuConnection {
                label: label.to_owned(),
                host_io: None,
                runtime_rx_priority: None,
                runtime_rx_bulk: None,
                runtime_caps: None,
                identify_caps: 0,
                mcu_transport_supported: true,
                ethercat_socket: Some(socket_path.to_owned()),
                endpoint_process: Some(child),
                endpoint_conn: Some(Arc::new(conn)),
                ethercat_slot_axes: slot_axes,
            },
        );
        self.nominal_clock_freqs
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .insert(raw, ETHERCAT_CLOCK_FREQ_HZ);
    }
}

#[cfg(test)]
mod tests;

#[cfg(test)]
mod claim_error_message_tests;

#[cfg(test)]
mod drip_cohort_participants_tests;

#[cfg(test)]
mod axis_ring_depth_tests;

#[cfg(test)]
mod ring_depth_for_axis_tests;

#[cfg(test)]
mod place_motor_response_tests;

#[cfg(test)]
mod require_events_dir_tests;

#[cfg(test)]
mod resolve_motion_caps_tests;

#[cfg(test)]
mod ethercat_endpoint_tests;

#[cfg(test)]
mod kinematics_calls_tests;

#[cfg(test)]
mod submit_nudge_validation_tests;
