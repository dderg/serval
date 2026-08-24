use crate::lock_ext::LockExt;
use std::collections::{HashMap, HashSet};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
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
use crate::mcu_config::{McuAxisConfig, McuTopologyInput, build_mcu_configs};
use crate::types::mcu_handle_from_raw;
use crate::worker::{StreamWorkerError, StreamWorkerHandle};

mod attach;
mod axis_transport_api;
mod clock_regression;
pub use clock_regression::{PyClockSyncEstimator, PyDecayRegression};
mod drain_wait;
mod endstop;
mod ethercat_endpoint;
mod homing_api;
mod motion_caps;
mod passthrough;
mod pipeline_setup;
#[cfg(test)]
mod pipeline_setup_tests;
mod planner_api;
mod runtime_caps;
mod servo;
#[cfg(test)]
mod servo_tests;
mod state;
mod telemetry;

use endstop::{TripDeps, dispatch_endstop_trip};
#[cfg(test)]
use ethercat_endpoint::{EndpointClaimError, ReportedExecutor, endpoint_args};
use ethercat_endpoint::{
    SampleGrid, arm_endpoint_death_watchdog, build_ring_filler, handshake_ethercat_endpoint,
    message_for_claim_error, poll_socket_ready, report_ethercat_endpoint_death,
    spawn_ethercat_endpoint, verify_sample_grid,
};
use motion_caps::{drip_cohort_participants, require_events_dir_for_mcu_transport};
#[cfg(test)]
use runtime_caps::place_motor_response;
use runtime_caps::{
    collect_motor_positions_inner, query_ethercat_runtime_caps, query_runtime_caps,
    require_positive, slots_for_axis,
};
use state::{
    EthercatDrive, FlushState, FlushWait, HomingRun, HomingState, LatchedFaults, McuConnection,
    PositionPoll, PumpHandles, RemoteFreeze, TripMember,
};

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

fn open_link_with_retry(
    what: &str,
    link_desc: &str,
    deadline: Instant,
    timeout_s: f64,
    mut open: impl FnMut() -> Result<McuHostIo, host_rt::transport::TransportError>,
) -> PyResult<McuHostIo> {
    loop {
        match open() {
            Ok(io) => return Ok(io),
            Err(e) => {
                if Instant::now() >= deadline {
                    return Err(PyRuntimeError::new_err(format!(
                        "{what}: could not open {link_desc} within {timeout_s}s: {e}"
                    )));
                }
                tracing::warn!(
                    subsystem = "mcu-comms",
                    event = "attach_open_retry",
                    link = link_desc,
                    error = %e,
                    "{what}: retrying open"
                );
                std::thread::sleep(Duration::from_millis(500));
            }
        }
    }
}

fn open_serial_with_retry(
    serial_path: &str,
    effective_baud: u32,
    is_pipe: bool,
    config: &McuHostIoConfig,
    deadline: Instant,
    timeout_s: f64,
) -> PyResult<McuHostIo> {
    open_link_with_retry("attach_serial", serial_path, deadline, timeout_s, || {
        if is_pipe {
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
        }
    })
}

fn open_canbus_with_retry(
    interface: &str,
    uuid: u64,
    config: &McuHostIoConfig,
    deadline: Instant,
    timeout_s: f64,
) -> PyResult<McuHostIo> {
    let link_desc = format!("{interface} uuid={uuid:012x}");
    open_link_with_retry("attach_canbus", &link_desc, deadline, timeout_s, || {
        McuHostIo::open_canbus_with_config(interface, uuid, config.clone())
    })
}

/// Backstop only. The continuity commit drains at every blend, so the buffer
/// normally hovers near the open-tail length past the finality barrier; this
/// force-drain to rest fires solely when no clean seam exists within reach.
///
/// It must sit well above the open tail a *legitimate* dense stream carries,
/// or the backstop stops being a backstop: the open tail is the re-plan batch
/// plus the brake-to-rest setback measured in moves, and at 600 mm/s with a
/// `max_jerk` of 2·max_accel that setback is ~150 mm — about 300 moves
/// of half-millimetre slicer output. At 128 the Voron 0 motion repro tripped
/// it 41 times in 4673 moves, i.e. once per window: every look-ahead ended in
/// a full stop. A `Move` is 192 bytes, so this ceiling costs 200 kB of window.
const STREAM_MAX_BUFFER_MOVES: usize = 1024;
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
    planner: Mutex<Option<StreamWorkerHandle>>,
    planner_config: Mutex<PlannerConfig>,
    commanded_pos: Mutex<geometry::GcodePos>,
    bed_mesh: Mutex<Option<Arc<geometry::SurfaceTransform>>>,
    last_g5_pq: Mutex<Option<(f64, f64)>>,
    mcu_axis_configs: Arc<Mutex<Vec<McuAxisConfig>>>,
    axis_transports: Mutex<Arc<crate::axis_transport::AxisTransports>>,
    stepcompress_endpoints: Arc<Mutex<HashMap<u32, Arc<Mutex<crate::pump::StepcompressEndpoint>>>>>,
    sample_endpoints: Arc<Mutex<HashMap<u32, Arc<Mutex<crate::pump::SampleEndpoint>>>>>,
    /// The sweep the last `resonance_buzz` armed, kept so completion is asked
    /// of the routes it actually drove.
    pub(crate) buzz_token: Mutex<Option<crate::pump::BuzzToken>>,
    dispatched_segments: Arc<AtomicU64>,
    dispatch_anchor: Arc<Mutex<crate::anchor::Anchor>>,
    fallback_clock_conversions: Arc<AtomicU64>,
    clock_freqs: Arc<Mutex<HashMap<u32, f64>>>,
    nominal_clock_freqs: Arc<Mutex<HashMap<u32, u32>>>,
    events_dir: Mutex<Option<std::path::PathBuf>>,
    pump: PumpHandles,
    position_poll: PositionPoll,
    drain: std::sync::Arc<crate::drain::DrainLedger>,
    motion_history: Arc<Mutex<crate::motion_history::HistoryStore>>,
    homing: Arc<HomingState>,
    flush: FlushState,
    // Monotonic id stamped on every streamed move as its `source.start_line`.
    // The continuity-commit drains the look-ahead buffer by line number
    // (`front.start_line < keep_line`) and detects consumed blend heads by line
    // equality, so each move MUST carry a distinct increasing id. Passing a
    // constant (0) makes the drain a no-op — the buffer never empties and every
    // commit re-dispatches the whole accumulated path from the start.
    move_seq: std::sync::atomic::AtomicU64,
    latched: LatchedFaults,
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
            commanded_pos: Mutex::new(geometry::GcodePos([0.0; 3])),
            bed_mesh: Mutex::new(None),
            last_g5_pq: Mutex::new(None),
            mcu_axis_configs: Arc::new(Mutex::new(Vec::new())),
            axis_transports: Mutex::new(Arc::new(crate::axis_transport::AxisTransports::default())),
            stepcompress_endpoints: Arc::new(Mutex::new(HashMap::new())),
            sample_endpoints: Arc::new(Mutex::new(HashMap::new())),
            buzz_token: Mutex::new(None),
            dispatched_segments: Arc::new(AtomicU64::new(0)),
            dispatch_anchor: Arc::new(Mutex::new(crate::anchor::Anchor::new())),
            fallback_clock_conversions: Arc::new(AtomicU64::new(0)),
            clock_freqs: Arc::new(Mutex::new(HashMap::new())),
            nominal_clock_freqs: Arc::new(Mutex::new(HashMap::new())),
            events_dir: Mutex::new(None),
            pump: PumpHandles::default(),
            position_poll: PositionPoll::default(),
            drain: std::sync::Arc::new(crate::drain::DrainLedger::new()),
            motion_history: Arc::new(Mutex::new(crate::motion_history::HistoryStore::default())),
            homing: Arc::new(HomingState::default()),
            flush: FlushState::default(),
            move_seq: std::sync::atomic::AtomicU64::new(0),
            latched: LatchedFaults::default(),
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
        let mut guard = self.events_dir.lock_ok();
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
        let mut router = self.router.lock_ok();
        let handle = router.claim_mcu(label);
        let raw = handle.raw();
        self.mcus.lock_ok().insert(
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
                sample_grid: None,
                ring_filler: None,
            },
        );
        Ok(raw)
    }

    #[pyo3(signature = (label, socket_path, interface, endpoint_binary, cycle_us, dynamics_profile, drives, late_tolerance_us=None, group_delay_us=None))]
    #[allow(clippy::too_many_arguments)]
    fn claim_ethercat_node(
        &self,
        label: &str,
        socket_path: &str,
        interface: &str,
        endpoint_binary: &str,
        cycle_us: u32,
        dynamics_profile: Option<String>,
        drives: Vec<EthercatDrive>,
        late_tolerance_us: Option<f64>,
        group_delay_us: Option<f64>,
    ) -> PyResult<u32> {
        if drives.is_empty() {
            return Err(PyRuntimeError::new_err(format!(
                "ethercat {label}: claim received no drives"
            )));
        }
        let mut drives = drives;
        drives.sort_by_key(|d| d.axis);
        let slot_axes: Vec<usize> = drives.iter().map(|d| d.axis).collect();

        if let Err(e) = std::fs::remove_file(socket_path) {
            if e.kind() != std::io::ErrorKind::NotFound {
                return Err(PyRuntimeError::new_err(format!(
                    "ethercat {label}: failed to remove stale socket {socket_path}: {e}"
                )));
            }
        }

        let events_dir = self.events_dir.lock_ok().clone();
        let mut child = spawn_ethercat_endpoint(
            endpoint_binary,
            interface,
            socket_path,
            cycle_us,
            dynamics_profile.as_deref(),
            late_tolerance_us,
            group_delay_us.unwrap_or(f64::from(cycle_us)),
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

        let sample_grid = verify_sample_grid(&conn, handshake_deadline).map_err(|e| {
            let _ = child.kill();
            let _ = child.wait();
            PyRuntimeError::new_err(message_for_claim_error(label, interface, &e))
        })?;

        let ring_filler = build_ring_filler(sample_grid, dynamics_profile.as_deref(), &drives)
            .map_err(|e| {
                let _ = child.kill();
                let _ = child.wait();
                PyRuntimeError::new_err(format!(
                    "ethercat {label}: the host cannot build the endpoint's setpoint filler — {e}"
                ))
            })?;

        let mut router = self.router.lock_ok();
        let handle = router.claim_mcu(label);
        let raw = handle.raw();
        drop(router);
        self.register_ethercat_mcu(
            raw,
            label,
            socket_path,
            child,
            conn,
            slot_axes,
            sample_grid,
            ring_filler,
        );
        Ok(raw)
    }

    fn release_mcu(&self, handle: u32) -> PyResult<()> {
        let Some(mut conn) = ({
            let mut mcus = self.mcus.lock_ok();
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

        let mut router = self.router.lock_ok();
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

        let mut planner = self.planner.lock_ok().take();
        if let Some(p) = planner.as_ref() {
            p.prepare_shutdown();
        }

        let pump_join = {
            let tx = self.pump.tx.lock_ok().take();
            if let Some(tx) = tx {
                let _ = tx.send(crate::pump::PumpMsg::Shutdown);
            }
            self.pump.thread.lock_ok().take()
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

        if let Some(mut p) = planner.take() {
            p.shutdown();
        }

        self.position_poll
            .stop
            .store(true, std::sync::atomic::Ordering::Relaxed);
        if let Some(h) = self.position_poll.thread.lock_ok().take() {
            if let Err(e) = h.join() {
                tracing::error!(
                    subsystem = "engine",
                    event = "shutdown_position_poll_join_panicked",
                    error = ?e,
                    "engine.shutdown(): live-position-poll join panicked"
                );
            }
        }

        let handles: Vec<u32> = {
            let mcus = self.mcus.lock_ok();
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
        let mut mcus = self.mcus.lock_ok();
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

        let config = McuHostIoConfig {
            mcu_label: Some(mcu_label.clone()),
            ..McuHostIoConfig::default()
        };

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

    #[pyo3(signature = (mcu_handle, interface, uuid, timeout_s = 30.0, klippy_non_critical = false, expect_native = true))]
    fn attach_canbus(
        &self,
        mcu_handle: u32,
        interface: &str,
        uuid: &str,
        timeout_s: f64,
        klippy_non_critical: bool,
        expect_native: bool,
    ) -> PyResult<()> {
        use std::time::{Duration, Instant};
        let deadline = Instant::now() + Duration::from_secs_f64(timeout_s);
        let uuid_value = u64::from_str_radix(uuid, 16).map_err(|e| {
            PyRuntimeError::new_err(format!("attach_canbus: invalid canbus_uuid {uuid:?}: {e}"))
        })?;
        if uuid_value > 0xffff_ffff_ffff {
            return Err(PyRuntimeError::new_err(format!(
                "attach_canbus: canbus_uuid {uuid:?} exceeds 6 bytes"
            )));
        }
        let link_desc = format!("{interface}:{uuid_value:012x}");

        if self.try_reuse_existing_connection(
            mcu_handle,
            &link_desc,
            klippy_non_critical,
            expect_native,
        )? {
            return Ok(());
        }

        let mcu_label: String = self.with_mcu(
            mcu_handle,
            |h| format!("attach_canbus: unknown mcu_handle {h} (claim_mcu not called)"),
            |conn| {
                conn.runtime_rx_priority = None;
                conn.runtime_rx_bulk = None;
                conn.host_io = None;
                Ok(conn.label.clone())
            },
        )?;

        let config = McuHostIoConfig {
            mcu_label: Some(mcu_label.clone()),
            ..McuHostIoConfig::default()
        };

        let host_io = open_canbus_with_retry(interface, uuid_value, &config, deadline, timeout_s)?;

        self.register_freshly_attached_mcu(
            mcu_handle,
            &link_desc,
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
}

impl Drop for PyMotionEngine {
    fn drop(&mut self) {
        self.shutdown();
    }
}

impl PyMotionEngine {
    fn with_mcu<R>(
        &self,
        handle: u32,
        unknown_mcu_err: impl FnOnce(u32) -> String,
        f: impl FnOnce(&mut McuConnection) -> PyResult<R>,
    ) -> PyResult<R> {
        let mut mcus = self.mcus.lock_ok();
        let mc = mcus
            .get_mut(&handle)
            .ok_or_else(|| PyRuntimeError::new_err(unknown_mcu_err(handle)))?;
        f(mc)
    }

    fn ethercat_conn(&self, mcu_handle: u32, what: &str) -> PyResult<Arc<McuSerialConn>> {
        let mcus = self.mcus.lock_ok();
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
        let mcus = self.mcus.lock_ok();
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

impl PyMotionEngine {
    fn register_ethercat_mcu(
        &self,
        raw: u32,
        label: &str,
        socket_path: &str,
        child: std::process::Child,
        conn: McuSerialConn,
        slot_axes: Vec<usize>,
        sample_grid: SampleGrid,
        ring_filler: crate::pump::RingFiller,
    ) {
        let ethercat = McuConnection {
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
            sample_grid: Some(sample_grid),
            ring_filler: Some(ring_filler),
        };
        tracing::info!(
            subsystem = "engine",
            event = "ethercat_sample_grid",
            label,
            sample_grid = ?ethercat.sample_grid,
            "ethercat endpoint sample grid accepted at claim"
        );
        self.mcus.lock_ok().insert(raw, ethercat);
        self.nominal_clock_freqs
            .lock_ok()
            .insert(raw, ETHERCAT_CLOCK_FREQ_HZ);
        self.router
            .lock_ok()
            .set_nominal_freq(
                crate::types::mcu_handle_from_raw(raw),
                f64::from(ETHERCAT_CLOCK_FREQ_HZ),
            )
            .expect("ethercat mcu handle was claimed on this router");
    }
}

#[cfg(test)]
mod tests;

#[cfg(test)]
mod claim_error_message_tests;

#[cfg(test)]
mod drip_cohort_participants_tests;

#[cfg(test)]
mod place_motor_response_tests;

#[cfg(test)]
mod require_events_dir_tests;

#[cfg(test)]
mod ethercat_endpoint_tests;

#[cfg(test)]
mod kinematics_calls_tests;

#[cfg(test)]
mod submit_nudge_validation_tests;

#[cfg(test)]
mod homing_state_tests;

#[cfg(test)]
mod motion_state_query_tests;
