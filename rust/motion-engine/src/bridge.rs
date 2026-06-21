use std::collections::{HashMap, HashSet, VecDeque};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::Receiver;
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
use host_rt::passthrough_queue::{NotifyId, PassthroughEntry, PassthroughRouter};

use crate::classify;
use crate::config::{self, PlannerConfig};
use crate::dispatch::{McuAxisConfig, McuCaps, build_mcu_configs};
use crate::kinematics::{KinematicsModule, SPATIAL_AXES};
use crate::planner::{DispatchError, HomeDripParams, NudgeParams};
use crate::stream_planner::{StreamPlannerError, StreamPlannerHandle};
use crate::types::{cq_id_from_raw, mcu_handle_from_raw, stats_to_pydict};

struct HomingRun {
    cohort: u64,
    endstop_id: u8,
    endstop_mcu: u32,
    axis: u8,
    axis_key: crate::pump::AxisKey,
    all_axis_keys: Vec<crate::pump::AxisKey>,
    window_start_clock: u64,
    notify: crossbeam_channel::Sender<Result<([f64; 3], [f64; 3], u64), String>>,
}

fn abort_after_tracing_appender_drains() {
    let _ = std::io::Write::flush(&mut std::io::stderr());
    std::thread::sleep(std::time::Duration::from_millis(100));
    if std::env::var_os("NO_EXIT_ON_FAULT").is_none() {
        std::process::abort();
    }
}

fn trip_position_to_motor_frame(
    axis: u8,
    motor_pos: f64,
    _configs: &[crate::dispatch::McuAxisConfig],
    _axis_mcu: u32,
) -> [f64; SPATIAL_AXES] {
    assert!(
        (axis as usize) < SPATIAL_AXES,
        "follower axis {axis} in homing trip is a bug — a follower axis must never reach homing recovery"
    );
    let mut frame = [0.0f64; SPATIAL_AXES];
    frame[axis as usize] = motor_pos;
    frame
}

struct McuConnection {
    label: String,
    serial_path: String,
    baud: u32,
    host_io: Option<Arc<McuHostIo>>,
    runtime_rx_priority: Option<Receiver<host_rt::host_io::runtime_events::RuntimeEvent>>,
    runtime_rx_bulk: Option<Receiver<host_rt::host_io::runtime_events::RuntimeEvent>>,
    runtime_caps: Option<mcu_protocol::messages::RuntimeCapsResponse>,
    identify_caps: u64,
    mcu_transport_supported: bool,
    ethercat_socket: Option<String>,
    endpoint_process: Option<std::process::Child>,
    endpoint_conn: Option<Arc<McuSerialConn>>,
}

const DRAIN_TIMEOUT: Duration = Duration::from_secs(60);

const ETHERCAT_CLOCK_FREQ_HZ: u32 = 1_000_000_000;

#[derive(Debug, thiserror::Error)]
enum RuntimeCapsError {
    #[error("mcu_call QueryRuntimeCaps: {0}")]
    Call(String),
    #[error("QueryRuntimeCaps: unexpected response kind {got:?}")]
    UnexpectedKind { got: mcu_protocol::MessageKind },
    #[error("decode RuntimeCapsResponse: {0}")]
    Decode(String),
}

fn require_positive(value: Option<f64>, name: &str) -> PyResult<()> {
    if let Some(v) = value {
        if !(v.is_finite() && v > 0.0) {
            return Err(PyValueError::new_err(format!(
                "{name} must be finite and positive"
            )));
        }
    }
    Ok(())
}

fn decode_runtime_caps_body(
    body: &[u8],
) -> Result<mcu_protocol::messages::RuntimeCapsResponse, RuntimeCapsError> {
    use mcu_protocol::codec::{Cursor, Decode};
    use mcu_protocol::messages::RuntimeCapsResponse;
    let mut c = Cursor::new(body);
    RuntimeCapsResponse::decode_from(&mut c).map_err(|e| RuntimeCapsError::Decode(format!("{e:?}")))
}

fn query_runtime_caps(
    io: &McuHostIo,
    timeout: std::time::Duration,
) -> Result<mcu_protocol::messages::RuntimeCapsResponse, RuntimeCapsError> {
    use mcu_protocol::MessageKind;
    let (kind, body) = io
        .mcu_call(MessageKind::QueryRuntimeCaps, Vec::new(), timeout)
        .map_err(|e| RuntimeCapsError::Call(format!("{e:?}")))?;
    if kind != MessageKind::RuntimeCapsResponse {
        return Err(RuntimeCapsError::UnexpectedKind { got: kind });
    }
    decode_runtime_caps_body(&body)
}

enum MotorQuery {
    Serial(Arc<McuHostIo>),
    EtherCat(Arc<McuSerialConn>),
}

impl MotorQuery {
    fn is_ethercat(&self) -> bool {
        matches!(self, MotorQuery::EtherCat(_))
    }
}

fn place_motor_response(
    resp: &mcu_protocol::messages::MotorStateResponse,
    cfg_axes: &[usize],
    is_ethercat: bool,
    motors: &mut [Option<f64>],
    vmotors: &mut [Option<f64>],
) {
    let mut put = |slot: usize, m: &mcu_protocol::messages::MotorSample| {
        if slot < motors.len() {
            motors[slot] = Some(f64::from(m.pos_q16) / 65536.0);
            vmotors[slot] = Some(f64::from(m.vel_q16) / 65536.0);
        }
    };
    if is_ethercat {
        for (m, &slot) in resp.motors.iter().zip(cfg_axes.iter()) {
            put(slot, m);
        }
    } else {
        for m in &resp.motors {
            put(m.slot as usize, m);
        }
    }
}

fn collect_motor_positions_inner(
    mcu_axis_configs: &Mutex<Vec<crate::dispatch::McuAxisConfig>>,
    mcus: &Mutex<HashMap<u32, McuConnection>>,
    timeout: std::time::Duration,
) -> Result<HashMap<String, (f64, f64)>, String> {
    use host_rt::mcu_call::McuCall;
    use mcu_protocol::MessageKind;
    use mcu_protocol::codec::{Cursor, Decode};
    use mcu_protocol::messages::MotorStateResponse;
    use runtime::stepping_state::MAX_AXES;

    let configs = mcu_axis_configs
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .clone();
    if configs.is_empty() {
        return Err("query_motor_positions: no axes configured".into());
    }
    let kin_tag = configs
        .iter()
        .find(|c| c.axes.contains(&0usize))
        .map(|c| c.kinematics)
        .unwrap_or(runtime::segment::KinematicTag::Cartesian as u8);

    let mut motors: [Option<f64>; MAX_AXES] = [None; MAX_AXES];
    let mut vmotors: [Option<f64>; MAX_AXES] = [None; MAX_AXES];

    for cfg in &configs {
        let q = {
            let map = mcus.lock().unwrap_or_else(|p| p.into_inner());
            let Some(conn) = map.get(&cfg.mcu_id) else {
                continue;
            };
            if conn.ethercat_socket.is_some() {
                match conn.endpoint_conn.as_ref() {
                    Some(ep) => MotorQuery::EtherCat(Arc::clone(ep)),
                    None => continue,
                }
            } else {
                match conn.host_io.as_ref() {
                    Some(io) => MotorQuery::Serial(Arc::clone(io)),
                    None => continue,
                }
            }
        };
        let (kind, body) = match &q {
            MotorQuery::Serial(io) => {
                io.mcu_call(MessageKind::QueryMotorState, Vec::new(), timeout)
            }
            MotorQuery::EtherCat(ep) => {
                ep.mcu_call(MessageKind::QueryMotorState, Vec::new(), timeout)
            }
        }
        .map_err(|e| format!("query mcu {}: {e:?}", cfg.mcu_id))?;
        if kind != MessageKind::MotorStateResponse {
            return Err(format!(
                "query mcu {}: unexpected kind {kind:?}",
                cfg.mcu_id
            ));
        }
        let mut c = Cursor::new(&body);
        let resp = MotorStateResponse::decode_from(&mut c)
            .map_err(|e| format!("query mcu {}: decode {e:?}", cfg.mcu_id))?;
        place_motor_response(&resp, &cfg.axes, q.is_ethercat(), &mut motors, &mut vmotors);
    }
    crate::position_query::assemble_cartesian(&motors, &vmotors, kin_tag)
}

fn query_ethercat_runtime_caps(
    conn: &McuSerialConn,
    timeout: std::time::Duration,
) -> Result<mcu_protocol::messages::RuntimeCapsResponse, RuntimeCapsError> {
    use host_rt::mcu_call::McuCall;
    use mcu_protocol::MessageKind;
    let (kind, body) = conn
        .mcu_call(MessageKind::QueryRuntimeCaps, Vec::new(), timeout)
        .map_err(|e| RuntimeCapsError::Call(format!("{e:?}")))?;
    if kind != MessageKind::RuntimeCapsResponse {
        return Err(RuntimeCapsError::UnexpectedKind { got: kind });
    }
    decode_runtime_caps_body(&body)
}

#[derive(Debug)]
enum EndpointClaimError {
    DriveOffline { slave_idx: u8, fault_code: u16 },
    DriveFault { slave_idx: u8, fault_code: u16 },
    Protocol(String),
}

fn message_for_claim_error(label: &str, interface: &str, e: &EndpointClaimError) -> String {
    match e {
        EndpointClaimError::DriveOffline {
            slave_idx,
            fault_code,
        } => match fault_code {
            1 | 2 => format!(
                "ethercat {label}: EtherCAT bus on {interface}: no slaves responding \
                 (bringup rc=-{fault_code}) — check cable and drive power, then FIRMWARE_RESTART"
            ),
            10..=12 => format!(
                "ethercat {label}: realtime endpoint could not acquire RT scheduling \
                 (bringup rc=-{fault_code}) — grant CAP_SYS_NICE + CAP_IPC_LOCK to \
                 klipper.service and isolate a CPU core, then FIRMWARE_RESTART"
            ),
            // 0 = no bringup rc (e.g. the stub's simulated failure) — omit the suffix.
            0 => format!(
                "ethercat {label}: drive (slave {slave_idx}) offline \
                 — check drive power, then FIRMWARE_RESTART"
            ),
            _ => format!(
                "ethercat {label}: drive (slave {slave_idx}) offline \
                 (bringup rc=-{fault_code}) — check drive power, then FIRMWARE_RESTART"
            ),
        },
        EndpointClaimError::DriveFault {
            slave_idx,
            fault_code,
        } => format!(
            "ethercat {label}: drive (slave {slave_idx}) \
             fault 0x{fault_code:04x} — check drive, then FIRMWARE_RESTART"
        ),
        EndpointClaimError::Protocol(s) => {
            format!("ethercat {label}: endpoint protocol error — {s}")
        }
    }
}

#[cfg(test)]
mod claim_error_message_tests {
    use super::{EndpointClaimError, message_for_claim_error};

    #[test]
    fn bus_dead_ec_init_failure() {
        let msg = message_for_claim_error(
            "node_x",
            "eth0",
            &EndpointClaimError::DriveOffline {
                slave_idx: 0,
                fault_code: 1,
            },
        );
        assert_eq!(
            msg,
            "ethercat node_x: EtherCAT bus on eth0: no slaves responding \
             (bringup rc=-1) — check cable and drive power, then FIRMWARE_RESTART"
        );
    }

    #[test]
    fn bus_dead_no_slaves() {
        let msg = message_for_claim_error(
            "node_x",
            "eth0",
            &EndpointClaimError::DriveOffline {
                slave_idx: 0,
                fault_code: 2,
            },
        );
        assert_eq!(
            msg,
            "ethercat node_x: EtherCAT bus on eth0: no slaves responding \
             (bringup rc=-2) — check cable and drive power, then FIRMWARE_RESTART"
        );
    }

    #[test]
    fn drive_offline_with_rc() {
        let msg = message_for_claim_error(
            "node_x",
            "eth0",
            &EndpointClaimError::DriveOffline {
                slave_idx: 1,
                fault_code: 5,
            },
        );
        assert_eq!(
            msg,
            "ethercat node_x: drive (slave 1) offline \
             (bringup rc=-5) — check drive power, then FIRMWARE_RESTART"
        );
    }

    #[test]
    fn rt_acquisition_failure_names_the_capability() {
        let msg = message_for_claim_error(
            "node_x",
            "eth0",
            &EndpointClaimError::DriveOffline {
                slave_idx: 1,
                fault_code: 12,
            },
        );
        assert_eq!(
            msg,
            "ethercat node_x: realtime endpoint could not acquire RT scheduling \
             (bringup rc=-12) — grant CAP_SYS_NICE + CAP_IPC_LOCK to \
             klipper.service and isolate a CPU core, then FIRMWARE_RESTART"
        );
    }

    #[test]
    fn drive_offline_stub_fault_code_zero_takes_drive_branch() {
        let msg = message_for_claim_error(
            "node_x",
            "eth0",
            &EndpointClaimError::DriveOffline {
                slave_idx: 1,
                fault_code: 0,
            },
        );
        assert_eq!(
            msg,
            "ethercat node_x: drive (slave 1) offline \
             — check drive power, then FIRMWARE_RESTART"
        );
    }

    #[test]
    fn drive_fault_unchanged() {
        let msg = message_for_claim_error(
            "node_x",
            "eth0",
            &EndpointClaimError::DriveFault {
                slave_idx: 1,
                fault_code: 0x0021,
            },
        );
        assert_eq!(
            msg,
            "ethercat node_x: drive (slave 1) fault 0x0021 — check drive, then FIRMWARE_RESTART"
        );
    }
}

/// The caller (`claim_ethercat_node`) removes any stale socket file at
/// `socket_path` before calling this function. That pre-spawn removal is
/// necessary: `FrameServer::bind` unlinks-and-rebinds on the path, but that
/// happens *after* the process starts — between spawn and bind, a pre-existing
/// file would let `poll_socket_ready` return immediately on existence, racing
/// `handshake_ethercat_endpoint`'s connect ahead of the actual listener.
fn spawn_ethercat_endpoint(
    binary: &str,
    interface: &str,
    socket_path: &str,
    counts_per_mm: f64,
    rotation_distance: f64,
    velocity_ff: bool,
    dynamics_profile: Option<&str>,
    torque_clamp_pct: f64,
    following_error_counts: Option<u32>,
    max_torque_tenth_pct: Option<u16>,
) -> Result<std::process::Child, String> {
    let mut cmd = std::process::Command::new(binary);
    cmd.arg(interface)
        .arg("--socket")
        .arg(socket_path)
        .arg("--counts-per-mm")
        .arg(counts_per_mm.to_string())
        .arg("--rotation-distance")
        .arg(rotation_distance.to_string())
        .arg("--torque-clamp-pct")
        .arg(torque_clamp_pct.to_string());
    if velocity_ff {
        cmd.arg("--velocity-ff");
    }
    if let Some(p) = dynamics_profile {
        cmd.arg("--dynamics-profile").arg(p);
    }
    if let Some(ferr) = following_error_counts {
        cmd.arg("--following-error-counts").arg(ferr.to_string());
    }
    if let Some(tq) = max_torque_tenth_pct {
        cmd.arg("--max-torque-tenth-pct").arg(tq.to_string());
    }
    cmd.spawn().map_err(|e| format!("spawn {binary}: {e}"))
}

fn poll_socket_ready(
    path: &str,
    deadline: Instant,
    child: &mut std::process::Child,
) -> Result<(), String> {
    loop {
        if std::path::Path::new(path).exists() {
            return Ok(());
        }
        if Instant::now() >= deadline {
            return Err(format!("endpoint socket {path} did not appear within 15 s"));
        }
        match child.try_wait() {
            Ok(Some(status)) => {
                return Err(format!(
                    "endpoint process exited before socket appeared \
                     (exit status: {status})"
                ));
            }
            Ok(None) => {}
            Err(e) => {
                return Err(format!("try_wait on endpoint process failed: {e}"));
            }
        }
        std::thread::sleep(Duration::from_millis(20));
    }
}

fn handshake_ethercat_endpoint(
    socket_path: &str,
    deadline: Instant,
) -> Result<McuSerialConn, EndpointClaimError> {
    use host_rt::mcu_call::McuCall;
    use mcu_protocol::MessageKind;
    use mcu_protocol::codec::{Cursor, Decode};
    use mcu_protocol::messages::ClaimHandshakeReply;

    // Retry connect until the endpoint's listener is up. ConnectionRefused and
    // NotFound both mean the endpoint hasn't bound yet (bind latency, or the
    // endpoint is mid-unlink-and-rebind of a stale path). Every other error is
    // immediately fatal as a Protocol error — we don't mask real failures.
    let conn = loop {
        match McuSerialConn::connect(socket_path) {
            Ok(c) => break c,
            Err(e)
                if e.kind() == std::io::ErrorKind::ConnectionRefused
                    || e.kind() == std::io::ErrorKind::NotFound =>
            {
                if Instant::now() >= deadline {
                    return Err(EndpointClaimError::Protocol(format!(
                        "connect to {socket_path}: timed out waiting for listener ({e})"
                    )));
                }
                std::thread::sleep(Duration::from_millis(50));
            }
            Err(e) => {
                return Err(EndpointClaimError::Protocol(format!(
                    "connect to {socket_path}: {e}"
                )));
            }
        }
    };

    let remaining = deadline.saturating_duration_since(Instant::now());
    let (kind, body) = conn
        .mcu_call(MessageKind::ClaimHandshake, Vec::new(), remaining)
        .map_err(|e| EndpointClaimError::Protocol(format!("ClaimHandshake call: {e:?}")))?;

    if kind != MessageKind::ClaimHandshakeReply {
        return Err(EndpointClaimError::Protocol(format!(
            "expected ClaimHandshakeReply (0x{:04x}), got 0x{:04x}",
            MessageKind::ClaimHandshakeReply.as_u16(),
            kind.as_u16(),
        )));
    }

    let reply = ClaimHandshakeReply::decode_from(&mut Cursor::new(&body))
        .map_err(|e| EndpointClaimError::Protocol(format!("decode ClaimHandshakeReply: {e:?}")))?;

    for s in &reply.slave_statuses {
        match s.state {
            mcu_protocol::messages::SlaveState::Offline => {
                return Err(EndpointClaimError::DriveOffline {
                    slave_idx: s.slave_idx,
                    fault_code: s.fault_code,
                });
            }
            mcu_protocol::messages::SlaveState::Fault => {
                return Err(EndpointClaimError::DriveFault {
                    slave_idx: s.slave_idx,
                    fault_code: s.fault_code,
                });
            }
            mcu_protocol::messages::SlaveState::Ok => {}
        }
    }

    Ok(conn)
}

#[derive(Debug, Clone)]
struct EngineEvent {
    kind: String,
    mcu: u32,
    notify_id: u64,
    response_bytes: Vec<u8>,
    sent_time: f64,
    receive_time: f64,
}

impl EngineEvent {
    fn to_pydict(&self, py: Python<'_>) -> PyResult<Py<PyDict>> {
        let d = PyDict::new(py);
        d.set_item("type", &self.kind)?;
        d.set_item("mcu", self.mcu)?;
        d.set_item("notify_id", self.notify_id)?;
        d.set_item("data", pyo3::types::PyBytes::new(py, &self.response_bytes))?;
        d.set_item("sent_time", self.sent_time)?;
        d.set_item("receive_time", self.receive_time)?;
        Ok(d.unbind())
    }
}

fn router_err(e: host_rt::passthrough_queue::RouterError) -> PyErr {
    PyRuntimeError::new_err(e.to_string())
}

fn planner_err(e: StreamPlannerError) -> PyErr {
    PyRuntimeError::new_err(e.to_string())
}

/// Look-ahead horizon the streaming planner holds back from each non-forced
/// commit, so newly-arriving moves are planned with their predecessors still
/// buffered. Drained to rest on flush/dwell/idle.
const STREAM_KEEP_SECS: f64 = 0.5;

fn resolve_motion_caps(
    caps: Option<mcu_protocol::messages::RuntimeCapsResponse>,
    label: &str,
    handle: u32,
) -> Result<McuCaps, String> {
    caps.map(McuCaps::from).ok_or_else(|| {
        format!(
            "no runtime caps for {label} MCU (handle={handle}) — cannot size piece rings; \
             firmware not flashed or QueryRuntimeCaps failed at attach"
        )
    })
}

fn require_events_dir_for_mcu_transport(
    mcu_transport: bool,
    events_dir: Option<&std::path::Path>,
    mcu_label: &str,
) -> Result<(), String> {
    if mcu_transport && events_dir.is_none() {
        return Err(format!(
            "attach_serial({mcu_label}): init_logging must be called before \
             attach_serial for a kalico-native MCU — the dedicated \
             mcu-*.jsonl writer cannot be installed without an events_dir. \
             All McuLog events would be silently discarded to the general \
             runtime_rx channel with no NDJSON output, which violates the \
             observability spec (§4, Decision C). Call init_logging first."
        ));
    }
    Ok(())
}

#[pyclass(name = "MotionEngine")]
#[allow(missing_debug_implementations)]
pub struct PyMotionEngine {
    router: Arc<Mutex<PassthroughRouter>>,
    parser: Arc<Mutex<Option<Arc<MsgProtoParser>>>>,
    mcus: Arc<Mutex<HashMap<u32, McuConnection>>>,
    events: Arc<Mutex<VecDeque<EngineEvent>>>,
    #[allow(dead_code)]
    handlers: Mutex<HashMap<(u32, String, u32), Py<PyAny>>>,
    // `Mutex<Option<..>>` (not `OnceLock`) so `shutdown()` can *take* the handle
    // and join the `kalico-planner` thread. A `OnceLock` cannot be drained, so
    // the planner thread would only be joined when the whole engine dropped —
    // which never happens on klippy's in-process FIRMWARE_RESTART loop.
    planner: Mutex<Option<StreamPlannerHandle>>,
    planner_config: Mutex<PlannerConfig>,
    commanded_pos: Mutex<[f64; 3]>,
    last_g5_pq: Mutex<Option<(f64, f64)>>,
    mcu_axis_configs: Arc<Mutex<Vec<McuAxisConfig>>>,
    dispatched_segments: Arc<AtomicU64>,
    fallback_clock_conversions: Arc<AtomicU64>,
    clock_freqs: Arc<Mutex<HashMap<u32, f64>>>,
    nominal_clock_freqs: Arc<Mutex<HashMap<u32, u32>>>,
    events_dir: Mutex<Option<std::path::PathBuf>>,
    pump_tx: Arc<Mutex<Option<std::sync::mpsc::Sender<crate::pump::PumpMsg>>>>,
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
    next_flush_id: std::sync::atomic::AtomicU64,
    homing_result:
        Mutex<Option<crossbeam_channel::Receiver<Result<([f64; 3], [f64; 3], u64), String>>>>,
    latched_drive_fault: Arc<Mutex<HashMap<u32, u16>>>,
    remote_triggers: Mutex<HashMap<u8, (u32, host_rt::host_io::InterceptorId)>>,
    // Latched once `shutdown()` has run a full teardown. Subsequent calls (the
    // Drop backstop, a second `klippy:disconnect`, the failed-connect path) see
    // this and no-op, so double-teardown is provably safe and observable.
    shut_down: AtomicBool,
}

pub(crate) fn axis_ring_depth(total_pieces: u32, num_axes: u32) -> u32 {
    (total_pieces / num_axes.max(1)).max(1)
}

pub(crate) fn drip_cohort_participants(configs: &[McuAxisConfig]) -> Vec<crate::pump::AxisKey> {
    configs
        .iter()
        .flat_map(|cfg| {
            cfg.axes.iter().map(move |&a| crate::pump::AxisKey {
                mcu_id: cfg.mcu_id,
                axis: a as u8,
            })
        })
        .collect()
}

#[cfg(test)]
mod drip_cohort_participants_tests {
    use super::drip_cohort_participants;
    use crate::dispatch::{AXIS_X, AXIS_Y, AXIS_Z, McuAxisConfig, McuCaps};
    use crate::pump::AxisKey;

    const FOLLOWER_E: usize = 3;

    fn cfg(mcu_id: u32, axes: Vec<usize>) -> McuAxisConfig {
        McuAxisConfig {
            mcu_id,
            axes,
            caps: McuCaps {
                total_piece_memory: 0,
            },
            kinematics: 1,
        }
    }

    #[test]
    fn includes_every_configured_axis_so_lane_3_enqueues_stay_in_cohort() {
        let configs = vec![
            cfg(0, vec![AXIS_Y, AXIS_Z, FOLLOWER_E]),
            cfg(1, vec![AXIS_X]),
        ];
        let participants = drip_cohort_participants(&configs);
        assert_eq!(
            participants,
            vec![
                AxisKey {
                    mcu_id: 0,
                    axis: AXIS_Y as u8
                },
                AxisKey {
                    mcu_id: 0,
                    axis: AXIS_Z as u8
                },
                AxisKey {
                    mcu_id: 0,
                    axis: FOLLOWER_E as u8
                },
                AxisKey {
                    mcu_id: 1,
                    axis: AXIS_X as u8
                },
            ]
        );
    }
}

#[cfg(test)]
mod axis_ring_depth_tests {
    use super::axis_ring_depth;

    #[test]
    fn typical_two_axis_mcu_splits_evenly() {
        assert_eq!(axis_ring_depth(1984, 2), 1984 / 2);
    }

    #[test]
    fn single_axis_mcu_gets_full_depth() {
        assert_eq!(axis_ring_depth(1984, 1), 1984);
    }

    #[test]
    fn lower_clamp_keeps_at_least_one() {
        assert_eq!(axis_ring_depth(0, 2), 1);
    }

    #[test]
    fn zero_num_axes_treated_as_one() {
        assert_eq!(axis_ring_depth(1000, 0), 1000);
    }
}

pub(crate) fn ring_depth_for_axis_inner(
    configs: &[crate::dispatch::McuAxisConfig],
    mcu_handle: u32,
    axis: u8,
) -> Result<u16, String> {
    let cfg = configs
        .iter()
        .find(|c| c.mcu_id == mcu_handle)
        .ok_or_else(|| {
            format!(
                "ring_depth_for_axis: unknown mcu_handle {mcu_handle} \
                 (init_planner not yet called?)"
            )
        })?;
    let axis_usize = usize::from(axis);
    if !cfg.axes.contains(&axis_usize) {
        return Err(format!(
            "ring_depth_for_axis: axis {axis} is not configured on mcu_handle \
             {mcu_handle} (configured axes: {:?})",
            cfg.axes
        ));
    }
    let depth = axis_ring_depth(cfg.caps.total_pieces() as u32, cfg.axes.len() as u32);
    if depth > u32::from(u16::MAX) {
        return Err(format!(
            "ring depth {depth} exceeds u16::MAX (65535) for mcu {mcu_handle} axis {axis}; \
             a >65535-piece ring would need >2 MB of SRAM and is impossible here — \
             check total_piece_memory configuration"
        ));
    }
    #[allow(clippy::cast_possible_truncation)]
    Ok(depth as u16)
}

#[cfg(test)]
mod ring_depth_for_axis_tests {
    use super::ring_depth_for_axis_inner;
    use crate::dispatch::{AXIS_X, AXIS_Y, AXIS_Z, McuAxisConfig, McuCaps};

    fn configs() -> Vec<McuAxisConfig> {
        vec![
            McuAxisConfig {
                mcu_id: 1,
                axes: vec![AXIS_X, AXIS_Y],
                kinematics: 0,
                caps: McuCaps {
                    total_piece_memory: 62 * 1024,
                },
            },
            McuAxisConfig {
                mcu_id: 2,
                axes: vec![AXIS_Z],
                kinematics: 1,
                caps: McuCaps {
                    total_piece_memory: 62 * 1024,
                },
            },
        ]
    }

    #[test]
    fn success_two_axis_mcu() {
        let expected = (1984 / 2) as u16;
        assert_eq!(
            ring_depth_for_axis_inner(&configs(), 1, AXIS_X as u8).unwrap(),
            expected
        );
        assert_eq!(
            ring_depth_for_axis_inner(&configs(), 1, AXIS_Y as u8).unwrap(),
            expected
        );
    }

    #[test]
    fn success_single_axis_mcu() {
        let expected = 1984u16;
        assert_eq!(
            ring_depth_for_axis_inner(&configs(), 2, AXIS_Z as u8).unwrap(),
            expected
        );
    }

    #[test]
    fn unknown_mcu_handle_errors() {
        let e = ring_depth_for_axis_inner(&configs(), 99, AXIS_X as u8).unwrap_err();
        assert!(e.contains("unknown mcu_handle 99"), "got: {e}");
    }

    #[test]
    fn axis_not_on_mcu_errors() {
        let e = ring_depth_for_axis_inner(&configs(), 1, AXIS_Z as u8).unwrap_err();
        assert!(e.contains("not configured"), "got: {e}");
    }

    #[test]
    fn ring_depth_over_u16_is_hard_error_not_clamp() {
        let configs = vec![McuAxisConfig {
            mcu_id: 0,
            axes: vec![AXIS_X],
            kinematics: 0,
            caps: McuCaps {
                total_piece_memory: 70_000 * 32,
            },
        }];
        let res = ring_depth_for_axis_inner(&configs, 0, AXIS_X as u8);
        assert!(
            res.is_err(),
            "depth > u16::MAX must be a hard error, not a clamp"
        );
        let e = res.unwrap_err();
        assert!(
            e.contains("exceeds u16::MAX"),
            "error message should mention u16::MAX, got: {e}"
        );
    }
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
            events: Arc::new(Mutex::new(VecDeque::new())),
            handlers: Mutex::new(HashMap::new()),
            planner: Mutex::new(None),
            planner_config: Mutex::new(PlannerConfig::default()),
            commanded_pos: Mutex::new([0.0; 3]),
            last_g5_pq: Mutex::new(None),
            mcu_axis_configs: Arc::new(Mutex::new(Vec::new())),
            dispatched_segments: Arc::new(AtomicU64::new(0)),
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
            next_flush_id: std::sync::atomic::AtomicU64::new(1),
            homing_result: Mutex::new(None),
            latched_drive_fault: Arc::new(Mutex::new(HashMap::new())),
            remote_triggers: Mutex::new(HashMap::new()),
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
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        let handle = router.claim_mcu(label);
        let raw = handle.raw();
        self.mcus.lock().unwrap_or_else(|p| p.into_inner()).insert(
            raw,
            McuConnection {
                label: label.to_owned(),
                serial_path: serial_path.to_owned(),
                baud,
                host_io: None,
                runtime_rx_priority: None,
                runtime_rx_bulk: None,
                runtime_caps: None,
                identify_caps: 0,
                mcu_transport_supported: false,
                ethercat_socket: None,
                endpoint_process: None,
                endpoint_conn: None,
            },
        );
        Ok(raw)
    }

    #[pyo3(signature = (label, socket_path, interface, endpoint_binary, counts_per_mm, rotation_distance, velocity_ff, dynamics_profile, torque_clamp_pct, following_error_counts=None, max_torque_tenth_pct=None))]
    fn claim_ethercat_node(
        &self,
        label: &str,
        socket_path: &str,
        interface: &str,
        endpoint_binary: &str,
        counts_per_mm: f64,
        rotation_distance: f64,
        velocity_ff: bool,
        dynamics_profile: Option<String>,
        torque_clamp_pct: f64,
        following_error_counts: Option<u32>,
        max_torque_tenth_pct: Option<u16>,
    ) -> PyResult<u32> {
        if let Err(e) = std::fs::remove_file(socket_path) {
            if e.kind() != std::io::ErrorKind::NotFound {
                return Err(PyRuntimeError::new_err(format!(
                    "ethercat {label}: failed to remove stale socket {socket_path}: {e}"
                )));
            }
        }

        let mut child = spawn_ethercat_endpoint(
            endpoint_binary,
            interface,
            socket_path,
            counts_per_mm,
            rotation_distance,
            velocity_ff,
            dynamics_profile.as_deref(),
            torque_clamp_pct,
            following_error_counts,
            max_torque_tenth_pct,
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
        self.register_ethercat_mcu(raw, label, socket_path, child, conn);
        Ok(raw)
    }

    fn set_torque(&self, mcu_handle: u32, value: bool, print_time: f64) -> PyResult<()> {
        let reference_mcu = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            *mcus
                .iter()
                .find(|(_, mc)| mc.label == "mcu")
                .map(|(raw, _)| raw)
                .ok_or_else(|| {
                    PyRuntimeError::new_err(
                        "set_torque: no MCU labeled 'mcu' claimed — \
                         cannot resolve the print_time reference clock",
                    )
                })?
        };
        let execute_at_ns = {
            let router = self.router.lock().unwrap_or_else(|p| p.into_inner());
            let host_secs = router
                .print_time_to_host_secs(mcu_handle_from_raw(reference_mcu), print_time)
                .ok_or_else(|| {
                    PyRuntimeError::new_err(format!(
                        "set_torque: reference mcu {reference_mcu} clock not synced — \
                         cannot convert print_time {print_time}"
                    ))
                })?;
            router
                .host_time_to_mcu_clock(mcu_handle_from_raw(mcu_handle), host_secs)
                .map_err(|e| {
                    PyRuntimeError::new_err(format!(
                        "set_torque: no clock mapping for mcu {mcu_handle}: {e:?}"
                    ))
                })?
        };
        if execute_at_ns == 0 {
            return Err(PyRuntimeError::new_err(format!(
                "set_torque: EtherCAT clock for mcu {mcu_handle} not seeded \
                 (init_planner not run?)"
            )));
        }
        let conn = self.ethercat_conn(mcu_handle, "set_torque")?;
        tracing::info!(
            subsystem = "engine",
            event = "servo_torque_command",
            mcu_handle,
            value,
            print_time,
            execute_at_ns,
            "servo torque command"
        );
        let result = crate::servo_torque::send_set_torque(&conn, value, execute_at_ns)
            .map_err(PyRuntimeError::new_err)?;
        if result != 0 {
            tracing::error!(
                subsystem = "engine",
                event = "servo_torque_rejected",
                mcu_handle,
                value,
                result,
                "servo torque command rejected"
            );
            return Err(PyRuntimeError::new_err(format!(
                "servo torque {} failed: endpoint result {result}",
                if value { "enable" } else { "disable" }
            )));
        }
        Ok(())
    }

    fn start_servo_capture(
        &self,
        mcu_handle: u32,
        path: String,
        started_utc: String,
        drive_name: String,
    ) -> PyResult<()> {
        let conn = self.ethercat_conn(mcu_handle, "start_servo_capture")?;
        tracing::info!(
            subsystem = "engine",
            event = "servo_capture_start",
            mcu_handle,
            path,
            "servo capture start"
        );
        let result =
            crate::servo_capture::send_start_capture(&conn, &path, &started_utc, &drive_name)
                .map_err(PyRuntimeError::new_err)?;
        if result != 0 {
            return Err(PyRuntimeError::new_err(format!(
                "servo capture start failed: endpoint result {result}"
            )));
        }
        Ok(())
    }

    fn stop_servo_capture(&self, mcu_handle: u32) -> PyResult<(i32, u64, Option<u64>)> {
        let conn = self.ethercat_conn(mcu_handle, "stop_servo_capture")?;
        tracing::info!(
            subsystem = "engine",
            event = "servo_capture_stop",
            mcu_handle,
            "servo capture stop"
        );
        let resp =
            crate::servo_capture::send_stop_capture(&conn).map_err(PyRuntimeError::new_err)?;
        tracing::info!(
            subsystem = "engine",
            event = "servo_capture_stopped",
            mcu_handle,
            result = resp.result,
            samples = resp.samples,
            "servo capture stopped"
        );
        let overflow = (resp.overflow_cycle
            != mcu_protocol::messages::StopCaptureResponse::NO_OVERFLOW)
            .then_some(resp.overflow_cycle);
        Ok((resp.result, resp.samples, overflow))
    }

    fn set_drive_limits(
        &self,
        mcu_handle: u32,
        following_error_counts: u32,
        max_torque_tenth_pct: u16,
    ) -> PyResult<()> {
        let conn = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            let mc = mcus.get(&mcu_handle).ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "set_drive_limits: unknown mcu_handle {mcu_handle}"
                ))
            })?;
            mc.endpoint_conn.clone().ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "set_drive_limits: mcu {mcu_handle} ({}) is not an EtherCAT endpoint",
                    mc.label
                ))
            })?
        };
        tracing::info!(
            subsystem = "engine",
            event = "servo_drive_limits",
            mcu_handle,
            following_error_counts,
            max_torque_tenth_pct,
            "servo drive limits set"
        );
        let result = crate::servo_torque::send_drive_limits(
            &conn,
            following_error_counts,
            max_torque_tenth_pct,
        )
        .map_err(PyRuntimeError::new_err)?;
        if result != 0 {
            return Err(PyRuntimeError::new_err(format!(
                "set_drive_limits: SDO write failed: endpoint result {result}"
            )));
        }
        Ok(())
    }

    fn restore_drive_limits(&self, mcu_handle: u32) -> PyResult<()> {
        let conn = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            let mc = mcus.get(&mcu_handle).ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "restore_drive_limits: unknown mcu_handle {mcu_handle}"
                ))
            })?;
            mc.endpoint_conn.clone().ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "restore_drive_limits: mcu {mcu_handle} ({}) is not an EtherCAT endpoint",
                    mc.label
                ))
            })?
        };
        tracing::info!(
            subsystem = "engine",
            event = "servo_drive_limits",
            mcu_handle,
            "servo drive limits restored"
        );
        let result = crate::servo_torque::send_restore_drive_limits(&conn)
            .map_err(PyRuntimeError::new_err)?;
        if result != 0 {
            return Err(PyRuntimeError::new_err(format!(
                "restore_drive_limits: SDO write failed: endpoint result {result}"
            )));
        }
        Ok(())
    }

    #[pyo3(signature = (mcu_handle, axis, pos_mm, timeout_s = 2.0))]
    fn finalize_homed_axis(
        &self,
        py: Python<'_>,
        mcu_handle: u32,
        axis: usize,
        pos_mm: f64,
        timeout_s: f64,
    ) -> PyResult<()> {
        let _ = axis;
        let conn = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            let mc = mcus.get(&mcu_handle).ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "finalize_homed_axis: unknown mcu_handle {mcu_handle}"
                ))
            })?;
            match mc.endpoint_conn.clone() {
                Some(conn) => conn,
                None => return Ok(()),
            }
        };
        let home_q16 = crate::dispatch::encode_q16(pos_mm);
        tracing::info!(
            subsystem = "engine",
            event = "servo_finalize_home",
            mcu_handle,
            pos_mm,
            home_q16,
            "servo home finalize"
        );
        let timeout = std::time::Duration::from_secs_f64(timeout_s);
        let result = py
            .detach(|| crate::servo_torque::send_seed_servo_home(&conn, home_q16, timeout))
            .map_err(PyRuntimeError::new_err)?;
        if result != 0 {
            return Err(PyRuntimeError::new_err(format!(
                "finalize_homed_axis: method-35 home-set failed: endpoint result {result}"
            )));
        }
        Ok(())
    }

    fn take_drive_fault(&self, mcu_handle: u32) -> PyResult<Option<u16>> {
        Ok(self
            .latched_drive_fault
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .remove(&mcu_handle))
    }

    fn sdo_read(&self, mcu_handle: u32, index: u16, subindex: u8) -> PyResult<(u8, u32)> {
        let conn = self.ethercat_conn(mcu_handle, "sdo_read")?;
        tracing::info!(
            subsystem = "engine",
            event = "servo_sdo_read",
            mcu_handle,
            index,
            subindex,
            "servo SDO read"
        );
        let r = crate::servo_sdo::send_sdo_read(&conn, index, subindex)
            .map_err(PyRuntimeError::new_err)?;
        if r.result != 0 {
            tracing::error!(
                subsystem = "engine",
                event = "servo_sdo_read_failed",
                mcu_handle,
                index,
                subindex,
                result = r.result,
                "servo SDO read failed"
            );
            return Err(PyRuntimeError::new_err(format!(
                "SDO read 0x{index:04x}.{subindex}: {}",
                crate::servo_sdo::failure_text(r.result)
            )));
        }
        Ok((r.size, u32::from_le_bytes(r.data)))
    }

    fn sdo_write(
        &self,
        mcu_handle: u32,
        index: u16,
        subindex: u8,
        size: u8,
        value: i64,
    ) -> PyResult<(u8, u32)> {
        let conn = self.ethercat_conn(mcu_handle, "sdo_write")?;
        tracing::info!(
            subsystem = "engine",
            event = "servo_sdo_write",
            mcu_handle,
            index,
            subindex,
            size,
            value,
            "servo SDO write"
        );
        let r = crate::servo_sdo::send_sdo_write(&conn, index, subindex, size, value)
            .map_err(PyRuntimeError::new_err)?;
        if r.result != 0 {
            tracing::error!(
                subsystem = "engine",
                event = "servo_sdo_write_failed",
                mcu_handle,
                index,
                subindex,
                size,
                value,
                result = r.result,
                "servo SDO write failed"
            );
            let readback = u32::from_le_bytes(r.readback_data);
            return Err(PyRuntimeError::new_err(format!(
                "SDO write 0x{index:04x}.{subindex} = {value} (size {size}): {} \
                 (drive reports raw 0x{readback:x})",
                crate::servo_sdo::failure_text(r.result)
            )));
        }
        Ok((r.readback_size, u32::from_le_bytes(r.readback_data)))
    }

    fn release_mcu(&self, handle: u32) -> PyResult<()> {
        // Pull the whole McuConnection out of the map but keep it alive (it owns
        // `host_io`) until *after* the endpoint child is reaped. Teardown order
        // matters: the endpoint must see session-end (socket close + SIGTERM)
        // before we close the host_io pts fd, which is the EBUSY-relevant step.
        //
        // Removing from the map BEFORE closing the endpoint socket (below) is
        // also the ec-heartbeat-poll race guard: the supervision thread confirms
        // every EOF/child-exit fault against `mcus.get(&mcu_id)` under the lock,
        // so by the time the socket close it observes as peer_closed() has
        // happened, the entry is already gone and the fault is read as a clean
        // release rather than fired into std::process::abort().
        let Some(mut conn) = ({
            let mut mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            mcus.remove(&handle)
        }) else {
            // Already released — idempotent no-op (shutdown may call twice, the
            // failed-connect path may call before any attach).
            return Ok(());
        };

        let mut endpoint_process = conn.endpoint_process.take();
        let endpoint_conn = conn.endpoint_conn.take();

        // Drop our Arc on the endpoint connection so the socket closes (signals
        // session end to the endpoint). Router/pump Arcs may still be live;
        // SIGTERM is the authoritative termination signal below.
        drop(endpoint_conn);

        if let Some(ref mut child) = endpoint_process {
            // Capture PID before any wait so it is valid in diagnostic messages
            // (after wait() the OS may reuse the pid_t value).
            let pid = libc::pid_t::try_from(child.id()).expect("child PID exceeds pid_t range");

            // SIGTERM: ask the endpoint to exit gracefully.
            // `libc::kill` is the only stable way to send a specific signal to
            // a child process on Unix; there is no safe std API for this.
            // ESRCH (no such process) = already exited = fine; discard the return value.
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

        // Endpoint is dead; now close the host_io. Dropping the McuConnection
        // drops its `Arc<McuHostIo>` — the last strong ref (pump/heartbeat
        // hold `Weak` only), so `McuHostIo::Drop` runs here: it sends the
        // reactor Shutdown and joins the reactor thread, which closes the pts
        // fd and releases TIOCEXCL — clearing the EBUSY for the next process.
        drop(conn);

        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        router.release_mcu(mcu_handle_from_raw(handle));
        self.handlers
            .lock()
            .unwrap()
            .retain(|&(mcu, _, _), _| mcu != handle);
        Ok(())
    }

    /// The single, complete, ordered, idempotent teardown primitive.
    ///
    /// It is the authoritative release path on every klippy exit that can leave
    /// state behind (`klippy:disconnect`, the failed-connect arms, and the Drop
    /// backstop). Calling it more than once is a clean no-op — the second call
    /// finds empty maps / `None` handles and the latched `shut_down` flag.
    ///
    /// Ordering — two hazards drive the order, one in each direction:
    ///
    ///   Hazard A (planner → pump): while the planner holds an uncommitted decel
    ///   tail (`t_dispatched < t_appended`, true after essentially any motion),
    ///   its `recv_timeout` fires `run_commit_and_dispatch`, whose dispatch closure
    ///   does `pump_tx.send(..)`. If the pump's `Receiver` were already gone that
    ///   send yields `DispatchError::PumpGone` → the planner calls `fatal()` →
    ///   `std::process::abort()`, which skips every `Drop` — leaking the pts fd.
    ///   Fix: join the planner BEFORE sending `PumpMsg::Shutdown`; once the
    ///   planner thread is joined no further dispatch can fire.
    ///
    ///   Hazard B (pump → EtherCAT conn): the pump may still be draining
    ///   already-queued pieces for an EtherCAT MCU after `release_mcu` drops the
    ///   last strong `Arc<McuSerialConn>`. In `call_push_pieces` the
    ///   `Weak::upgrade()` then returns `None` → `SendError::Fatal` →
    ///   `on_fatal_transport` → `std::process::abort()` — the same pts-fd leak.
    ///   Fix: join the pump BEFORE calling `release_mcu`; once the pump thread is
    ///   joined no send can be in flight.
    ///
    ///   Together: planner join → pump Shutdown + join → per-MCU release_mcu.
    ///
    ///   Post-join heartbeat sends: the ec-heartbeat-poll thread holds a clone of
    ///   `pump_tx`. After the pump's `Receiver` is dropped (pump joined), those
    ///   sends silently return `Err` and are discarded by the callback — harmless.
    fn shutdown(&self) {
        if self.shut_down.swap(true, Ordering::SeqCst) {
            tracing::debug!(
                subsystem = "engine",
                event = "shutdown_called_twice",
                "engine.shutdown() called twice (idempotent no-op)"
            );
            return;
        }

        // Step 1 — planner: join before the pump receives Shutdown so the planner
        // can never dispatch into a dead pump Receiver (Hazard A).
        let planner = self
            .planner
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .take();
        if let Some(mut p) = planner {
            p.shutdown();
        }

        // Step 2 — pump: join before releasing MCU transports so no queued piece
        // can hit a dead EtherCAT Weak after release_mcu drops the strong Arc
        // (Hazard B). run_pump exits immediately on Shutdown, abandoning queued
        // pieces — safe because the planner is already joined and no new pieces
        // will arrive.
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

        // Step 3 — per-MCU release_mcu: endpoint socket/child first, then
        // host_io fd (the EBUSY-relevant close), then router prune. The pump is
        // already joined so no send is in flight when the strong Arc drops.
        let handles: Vec<u32> = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            mcus.keys().copied().collect()
        };
        for h in handles {
            if let Err(e) = self.release_mcu(h) {
                // Fail loud: a release error means an fd / child may be leaked.
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

    fn alloc_command_queue(&self, handle: u32) -> PyResult<u32> {
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        let qid = router
            .alloc_command_queue(mcu_handle_from_raw(handle))
            .map_err(router_err)?;
        Ok(qid.raw())
    }

    #[pyo3(signature = (mcu, queue, data, min_clock=0, req_clock=0))]
    fn passthrough_send(
        &self,
        mcu: u32,
        queue: u32,
        data: &[u8],
        min_clock: u64,
        req_clock: u64,
    ) -> PyResult<()> {
        let entry = PassthroughEntry::new(data.to_vec(), min_clock, req_clock, NotifyId::none());
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        router
            .push(mcu_handle_from_raw(mcu), cq_id_from_raw(queue), entry)
            .map_err(router_err)?;
        Ok(())
    }

    #[pyo3(signature = (mcu, queue, data, min_clock=0, req_clock=0))]
    fn passthrough_query(
        &self,
        mcu: u32,
        queue: u32,
        data: &[u8],
        min_clock: u64,
        req_clock: u64,
    ) -> PyResult<u64> {
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        let mcu_h = mcu_handle_from_raw(mcu);

        let events_ref = Arc::clone(&self.events);
        let mcu_raw = mcu;

        let nid = router
            .register_notify(
                mcu_h,
                Box::new(move |resp| {
                    let ev = EngineEvent {
                        kind: "query_response".to_owned(),
                        mcu: mcu_raw,
                        notify_id: 0,
                        response_bytes: resp.bytes,
                        sent_time: resp.sent_time,
                        receive_time: resp.receive_time,
                    };
                    events_ref
                        .lock()
                        .unwrap_or_else(|p| p.into_inner())
                        .push_back(ev);
                }),
            )
            .map_err(router_err)?;

        let entry = PassthroughEntry::new(data.to_vec(), min_clock, req_clock, nid);
        router
            .push(mcu_h, cq_id_from_raw(queue), entry)
            .map_err(router_err)?;

        Ok(nid.raw())
    }

    #[pyo3(signature = (_mcu, _queue, _data, _timeout))]
    fn passthrough_send_wait_ack(
        &self,
        _mcu: u32,
        _queue: u32,
        _data: &[u8],
        _timeout: f64,
    ) -> PyResult<Vec<u8>> {
        Err(pyo3::exceptions::PyNotImplementedError::new_err(
            "send_wait_ack requires reactor thread — deferred to Phase 2",
        ))
    }

    #[pyo3(signature = (mcu, name, oid, callback))]
    fn passthrough_register_handler(
        &self,
        mcu: u32,
        name: &str,
        oid: u32,
        callback: Py<PyAny>,
    ) -> PyResult<()> {
        self.handlers
            .lock()
            .unwrap()
            .insert((mcu, name.to_owned(), oid), callback);
        Ok(())
    }

    fn passthrough_register_flush_callback(&self, mcu: u32, callback: Py<PyAny>) -> PyResult<()> {
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        let mcu_h = mcu_handle_from_raw(mcu);

        let cb: Box<dyn Fn() + Send> = Box::new(move || {
            Python::attach(|py| {
                if let Err(e) = callback.call0(py) {
                    e.print(py);
                }
            });
        });

        router
            .register_flush_callback(mcu_h, cb)
            .map_err(router_err)?;
        Ok(())
    }

    fn poll_event(&self, py: Python<'_>) -> PyResult<Option<Py<PyDict>>> {
        let mut events = self.events.lock().unwrap_or_else(|p| p.into_inner());
        match events.pop_front() {
            Some(ev) => Ok(Some(ev.to_pydict(py)?)),
            None => Ok(None),
        }
    }

    fn add_config_cmd(&self, mcu: u32, data: &[u8]) -> PyResult<bool> {
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        router
            .add_config_cmd(mcu_handle_from_raw(mcu), data.to_vec())
            .map_err(router_err)
    }

    fn add_init_cmd(&self, mcu: u32, data: &[u8]) -> PyResult<bool> {
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        router
            .add_init_cmd(mcu_handle_from_raw(mcu), data.to_vec())
            .map_err(router_err)
    }

    fn add_restart_cmd(&self, mcu: u32, data: &[u8]) -> PyResult<bool> {
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        router
            .add_restart_cmd(mcu_handle_from_raw(mcu), data.to_vec())
            .map_err(router_err)
    }

    fn begin_config_phase(&self, mcu: u32) -> PyResult<()> {
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        router
            .begin_config_phase(mcu_handle_from_raw(mcu))
            .map_err(router_err)
    }

    fn next_config_entry(&self, mcu: u32) -> PyResult<Option<Vec<u8>>> {
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        router
            .next_config_entry(mcu_handle_from_raw(mcu))
            .map_err(router_err)
    }

    fn get_stats(&self, py: Python<'_>, mcu: u32) -> PyResult<Py<PyDict>> {
        let router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        let stats = router
            .get_stats(mcu_handle_from_raw(mcu))
            .map_err(router_err)?;
        stats_to_pydict(py, &stats)
    }

    fn set_msgproto_dict(&self, dict_json: &[u8]) -> PyResult<()> {
        let json_str = std::str::from_utf8(dict_json)
            .map_err(|e| PyRuntimeError::new_err(format!("dict_json utf8: {e}")))?;
        let dict: DataDictionary = serde_json::from_str(json_str)
            .map_err(|e| PyRuntimeError::new_err(format!("dict json parse: {e}")))?;
        let parser = MsgProtoParser::from_dictionary(dict)
            .map_err(|e| PyRuntimeError::new_err(format!("parser build: {e:?}")))?;
        *self.parser.lock().unwrap_or_else(|p| p.into_inner()) = Some(Arc::new(parser));
        Ok(())
    }

    // Narrow fd-release hook for the serial arduino-reset path (MCU._disconnect
    // → serial.disconnect()). It only nils host_io/runtime_rx for one MCU; it
    // does NOT touch endpoint_conn/endpoint_process, so it cannot tear an
    // EtherCAT MCU down on its own. The authoritative full teardown is
    // `shutdown()`; detach_serial is harmless before it (shutdown is idempotent).
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

        {
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

                    let (rx_priority, rx_bulk) =
                        io.take_runtime_event_subscription().map_err(|e| {
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

                    let mut mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
                    let conn = mcus.get_mut(&mcu_handle).ok_or_else(|| {
                        PyRuntimeError::new_err(format!(
                            "attach_serial: unknown mcu_handle {mcu_handle}"
                        ))
                    })?;
                    conn.runtime_rx_priority = Some(rx_priority);
                    conn.runtime_rx_bulk = Some(rx_bulk);
                    conn.runtime_caps = runtime_caps;
                    conn.identify_caps = identify_caps;
                    conn.mcu_transport_supported = mcu_transport_supported;
                    return Ok(());
                }
            }
        }

        let mcu_label: String = {
            let mut mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            let conn = mcus.get_mut(&mcu_handle).ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "attach_serial: unknown mcu_handle {mcu_handle} (claim_mcu not called)"
                ))
            })?;
            conn.runtime_rx_priority = None;
            conn.runtime_rx_bulk = None;
            conn.host_io = None;
            conn.label.clone()
        };

        let is_pipe = baud == 0
            || serial_path.starts_with("/tmp/")
            || serial_path.starts_with("/dev/pts/")
            || serial_path.contains("klipper_host")
            || serial_path.contains("klipper_sim");

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
                &mcu_label,
            )
            .map_err(PyRuntimeError::new_err)?;
        }

        if mcu_transport_supported {
            let events_dir_guard = self.events_dir.lock().unwrap_or_else(|p| p.into_inner());
            if let Some(ref dir) = *events_dir_guard {
                use crate::logging::writer::{
                    DEFAULT_BACKUP_COUNT, DEFAULT_MAX_BYTES, FSYNC_INTERVAL, RotatingJsonlWriter,
                };
                let source = mcu_label.clone();
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

        let mut mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
        let conn = mcus.get_mut(&mcu_handle).ok_or_else(|| {
            PyRuntimeError::new_err(format!("attach_serial: unknown mcu_handle {mcu_handle}"))
        })?;
        conn.host_io = Some(host_io_arc);
        conn.runtime_rx_priority = Some(rx_priority);
        conn.runtime_rx_bulk = Some(rx_bulk);
        conn.runtime_caps = runtime_caps;
        conn.identify_caps = identify_caps;
        conn.mcu_transport_supported = mcu_transport_supported;
        Ok(())
    }

    fn get_mcu_capabilities(&self, mcu_handle: u32) -> PyResult<u64> {
        let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
        let conn = mcus.get(&mcu_handle).ok_or_else(|| {
            PyRuntimeError::new_err(format!(
                "get_mcu_capabilities: unknown mcu_handle {mcu_handle}"
            ))
        })?;
        Ok(conn.identify_caps)
    }

    fn ring_depth_for_axis(&self, mcu_handle: u32, axis: u8) -> PyResult<u16> {
        let configs = self
            .mcu_axis_configs
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        ring_depth_for_axis_inner(&configs, mcu_handle, axis).map_err(PyRuntimeError::new_err)
    }

    #[pyo3(signature = (mcu_handle, bus_id, rate, timeout_s = 5.0))]
    fn register_phase_bus(
        &self,
        py: Python<'_>,
        mcu_handle: u32,
        bus_id: u8,
        rate: u32,
        timeout_s: f64,
    ) -> PyResult<()> {
        let io = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            let conn = mcus.get(&mcu_handle).ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "register_phase_bus: unknown mcu_handle {mcu_handle}"
                ))
            })?;
            if !conn.mcu_transport_supported {
                return Ok(());
            }
            conn.host_io
                .as_ref()
                .ok_or_else(|| {
                    PyRuntimeError::new_err(
                        "register_phase_bus: attach_serial has not been called for this MCU",
                    )
                })?
                .clone()
        };
        let timeout = std::time::Duration::from_secs_f64(timeout_s);
        let msg = format!("runtime_register_phase_bus bus_id={bus_id} rate={rate}");
        let params = py.detach(|| -> PyResult<_> {
            use host_rt::transport::Transport;
            io.call(&msg, "kalico_register_phase_bus_response", timeout)
                .map_err(|e| {
                    PyRuntimeError::new_err(format!("register_phase_bus: transport error: {e:?}"))
                })
        })?;
        let result = params.try_get_i32("result").ok_or_else(|| {
            PyRuntimeError::new_err(
                "register_phase_bus: response missing or non-integer result field",
            )
        })?;
        if result != 0 {
            return Err(PyRuntimeError::new_err(format!(
                "register_phase_bus: MCU returned error {result} (bus_id={bus_id})"
            )));
        }
        Ok(())
    }

    #[pyo3(signature = (mcu_handle, motor_idx, bus_id, cs_pin_id, slot_idx, timeout_s = 5.0))]
    fn register_phase_motor(
        &self,
        py: Python<'_>,
        mcu_handle: u32,
        motor_idx: u8,
        bus_id: u8,
        cs_pin_id: u8,
        slot_idx: u8,
        timeout_s: f64,
    ) -> PyResult<()> {
        let io = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            let conn = mcus.get(&mcu_handle).ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "register_phase_motor: unknown mcu_handle {mcu_handle}"
                ))
            })?;
            if !conn.mcu_transport_supported {
                return Ok(());
            }
            conn.host_io
                .as_ref()
                .ok_or_else(|| {
                    PyRuntimeError::new_err(
                        "register_phase_motor: attach_serial has not been called for this MCU",
                    )
                })?
                .clone()
        };
        let timeout = std::time::Duration::from_secs_f64(timeout_s);
        let msg = format!(
            "runtime_register_phase_motor motor_idx={motor_idx} \
             bus_id={bus_id} cs_pin_id={cs_pin_id} slot_idx={slot_idx}"
        );
        let params = py.detach(|| -> PyResult<_> {
            use host_rt::transport::Transport;
            io.call(&msg, "kalico_register_phase_motor_response", timeout)
                .map_err(|e| {
                    PyRuntimeError::new_err(format!("register_phase_motor: transport error: {e:?}"))
                })
        })?;
        let result = params.try_get_i32("result").ok_or_else(|| {
            PyRuntimeError::new_err(
                "register_phase_motor: response missing or non-integer result field",
            )
        })?;
        if result != 0 {
            return Err(PyRuntimeError::new_err(format!(
                "register_phase_motor: MCU returned error {result} \
                 (motor_idx={motor_idx} bus_id={bus_id} cs_pin_id={cs_pin_id})"
            )));
        }
        Ok(())
    }

    #[pyo3(signature = (mcu_id, axis_idx, motor_mask, delta_mm, speed, accel))]
    fn submit_nudge(
        &self,
        _py: Python<'_>,
        mcu_id: u32,
        axis_idx: u8,
        motor_mask: u8,
        delta_mm: f64,
        speed: f64,
        accel: f64,
    ) -> PyResult<f64> {
        if runtime::piece_ring::stepper_sel_from_mask(motor_mask).is_err() {
            return Err(PyRuntimeError::new_err(format!(
                "submit_nudge: multi-bit motor_mask {motor_mask:#010b} not supported"
            )));
        }
        let (tx, rx) = crossbeam_channel::bounded(1);
        {
            let guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
            let planner = guard.as_ref().ok_or_else(|| {
                PyRuntimeError::new_err("planner not initialized — call init_planner first")
            })?;
            planner
                .submit_nudge(crate::planner::NudgeParams {
                    mcu_id,
                    axis: axis_idx,
                    motor_mask,
                    delta_mm,
                    speed,
                    accel,
                    notify: tx,
                })
                .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;
        }
        rx.recv()
            .map_err(|_| PyRuntimeError::new_err("nudge notify dropped"))?
            .map_err(PyRuntimeError::new_err)?;
        let (accel_t, cruise_t, _v) = crate::nudge::calc_move_time(delta_mm, speed, accel);
        Ok(accel_t + cruise_t + accel_t)
    }

    fn get_identify_data(&self, mcu_handle: u32) -> PyResult<Vec<u8>> {
        let io = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            let conn = mcus.get(&mcu_handle).ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "get_identify_data: unknown mcu_handle {mcu_handle}"
                ))
            })?;
            conn.host_io
                .as_ref()
                .ok_or_else(|| {
                    PyRuntimeError::new_err(
                        "get_identify_data: attach_serial has not been called for this MCU",
                    )
                })?
                .clone()
        };
        Ok(io.raw_identify_bytes().to_vec())
    }

    #[pyo3(signature = (mcu_handle, msg, response, timeout_s = 5.0))]
    fn engine_call(
        &self,
        py: Python<'_>,
        mcu_handle: u32,
        msg: &str,
        response: &str,
        timeout_s: f64,
    ) -> PyResult<Py<PyDict>> {
        use std::time::Duration;

        let io = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            let conn = mcus.get(&mcu_handle).ok_or_else(|| {
                PyRuntimeError::new_err(format!("engine_call: unknown mcu_handle {mcu_handle}"))
            })?;
            conn.host_io
                .as_ref()
                .ok_or_else(|| {
                    PyRuntimeError::new_err(
                        "engine_call: attach_serial has not been called for this MCU",
                    )
                })?
                .clone()
        };

        let msg_owned = msg.to_owned();
        let response_owned = response.to_owned();
        let params = py.detach(|| -> PyResult<_> {
            use host_rt::transport::Transport;
            io.call(
                &msg_owned,
                &response_owned,
                Duration::from_secs_f64(timeout_s),
            )
            .map_err(|e| PyRuntimeError::new_err(format!("engine_call: {e}")))
        })?;

        let d = PyDict::new(py);
        for (k, v) in &params.fields {
            use host_rt::transport::MessageValue;
            match v {
                MessageValue::U32(n) => d.set_item(k, n)?,
                MessageValue::I32(n) => d.set_item(k, n)?,
                MessageValue::U64(n) => d.set_item(k, n)?,
                MessageValue::Bytes(b) => {
                    d.set_item(k, pyo3::types::PyBytes::new(py, b.as_slice()))?
                }
                MessageValue::String(s) => d.set_item(k, s)?,
            }
        }
        if params.sent_time_raw != 0.0 {
            d.set_item("#sent_time_raw", params.sent_time_raw)?;
            d.set_item("#receive_time_raw", params.recv_time_raw)?;
        }
        Ok(d.unbind())
    }

    fn take_runtime_event(&self, py: Python<'_>, mcu_handle: u32) -> PyResult<Option<Py<PyDict>>> {
        use host_rt::host_io::runtime_events::RuntimeEvent;
        use std::sync::mpsc::TryRecvError;

        let event = {
            let mut mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            let conn = mcus.get_mut(&mcu_handle).ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "take_runtime_event: unknown mcu_handle {mcu_handle}"
                ))
            })?;
            let mut taken = None;
            for lane in [&mut conn.runtime_rx_priority, &mut conn.runtime_rx_bulk] {
                if let Some(rx) = lane.as_mut() {
                    if let Ok(ev) = rx.try_recv() {
                        taken = Some(ev);
                        break;
                    }
                }
            }
            match taken {
                Some(ev) => ev,
                None => return Ok(None),
            }
        };

        let d = PyDict::new(py);
        match event {
            RuntimeEvent::Status(s) => {
                d.set_item("type", "status")?;
                d.set_item("engine_status", s.engine_status)?;
                d.set_item("queue_depth", s.queue_depth)?;
                d.set_item("current_segment_id", s.current_segment_id)?;
                d.set_item("last_fault", s.last_fault)?;
                d.set_item("fault_detail", s.fault_detail)?;
                d.set_item("retired_through_segment_id", s.retired_through_segment_id)?;
            }
            RuntimeEvent::CreditFreed(c) => {
                d.set_item("type", "credit_freed")?;
                d.set_item("retired_through_segment_id", c.retired_through_segment_id)?;
                d.set_item("free_slots", c.free_slots)?;
            }
            RuntimeEvent::Fault(f) => {
                d.set_item("type", "fault")?;
                d.set_item("fault_code", f.fault_code)?;
                d.set_item("fault_detail", f.fault_detail)?;
                d.set_item("segment_id", f.segment_id)?;
                d.set_item("synthesized", f.synthesized)?;
            }
            RuntimeEvent::Trace(_) => {
                return Ok(None);
            }
            RuntimeEvent::Heartbeat { .. } => {
                return Ok(None);
            }
            RuntimeEvent::EndstopTrip(t) => {
                d.set_item("type", "endstop_trip")?;
                d.set_item("endstop_id", t.endstop_id)?;
                d.set_item("trip_clock", t.trip_clock)?;
                self.handle_endstop_trip(mcu_handle, t.endstop_id, t.trip_clock);
            }
            RuntimeEvent::UnknownOutput { format, msg } => {
                d.set_item("type", "output")?;
                d.set_item("format", format)?;
                d.set_item("msg", msg)?;
            }
            RuntimeEvent::PassthroughResponse { name, params } => {
                d.set_item("type", "response")?;
                d.set_item("name", name)?;
                for (k, v) in &params.fields {
                    use host_rt::transport::MessageValue;
                    match v {
                        MessageValue::U32(n) => d.set_item(k, *n)?,
                        MessageValue::I32(n) => d.set_item(k, *n)?,
                        MessageValue::U64(n) => d.set_item(k, *n)?,
                        MessageValue::Bytes(b) => {
                            d.set_item(k, pyo3::types::PyBytes::new(py, b.as_slice()))?
                        }
                        MessageValue::String(s) => d.set_item(k, s)?,
                    }
                }
                if params.sent_time_raw != 0.0 {
                    d.set_item("#sent_time_raw", params.sent_time_raw)?;
                    d.set_item("#receive_time_raw", params.recv_time_raw)?;
                }
            }
            RuntimeEvent::McuLog(_) => {
                return Ok(None);
            }
        }
        Ok(Some(d.unbind()))
    }

    fn engine_get_clock_async(&self, mcu_handle: u32) -> PyResult<()> {
        let io =
            {
                let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
                let conn = mcus.get(&mcu_handle).ok_or_else(|| {
                    pyo3::exceptions::PyRuntimeError::new_err(format!(
                        "engine_get_clock_async: unknown mcu_handle {mcu_handle}"
                    ))
                })?;
                conn.host_io.as_ref().ok_or_else(|| {
                pyo3::exceptions::PyRuntimeError::new_err(
                    "engine_get_clock_async: attach_serial has not been called for this MCU",
                )
            })?.clone()
            };

        io.get_clock_async().map_err(|e| {
            pyo3::exceptions::PyRuntimeError::new_err(format!("engine_get_clock_async: {e}"))
        })
    }

    #[pyo3(signature = (mcu_handle, msg))]
    fn engine_send(&self, mcu_handle: u32, msg: &str) -> PyResult<()> {
        let io = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            let conn = mcus.get(&mcu_handle).ok_or_else(|| {
                PyRuntimeError::new_err(format!("engine_send: unknown mcu_handle {mcu_handle}"))
            })?;
            conn.host_io
                .as_ref()
                .ok_or_else(|| {
                    PyRuntimeError::new_err(
                        "engine_send: attach_serial has not been called for this MCU",
                    )
                })?
                .clone()
        };
        io.send_fire_and_forget(msg)
            .map_err(|e| PyRuntimeError::new_err(format!("engine_send: {e}")))
    }

    fn engine_mark_expected_disconnect(&self, mcu_handle: u32) -> PyResult<()> {
        let io = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            let conn = mcus.get(&mcu_handle).ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "engine_mark_expected_disconnect: unknown mcu_handle {mcu_handle}"
                ))
            })?;
            conn.host_io.as_ref().ok_or_else(|| {
                PyRuntimeError::new_err(
                    "engine_mark_expected_disconnect: attach_serial has not been called for this MCU",
                )
            })?.clone()
        };
        io.mark_expected_disconnect()
            .map_err(|e| PyRuntimeError::new_err(format!("engine_mark_expected_disconnect: {e}")))
    }

    #[pyo3(signature = (mcu, freq, offset, last_clock, host_now_raw))]
    fn set_clock_est(
        &self,
        _py: Python<'_>,
        mcu: u32,
        freq: f64,
        offset: f64,
        last_clock: u64,
        host_now_raw: f64,
    ) -> PyResult<()> {
        self.clock_freqs
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .insert(mcu, freq);

        use std::sync::atomic::{AtomicUsize, Ordering as AOrd};
        static SET_CLOCK_EST_CALLS: AtomicUsize = AtomicUsize::new(0);
        let call_n = SET_CLOCK_EST_CALLS.fetch_add(1, AOrd::Relaxed);
        if call_n < 5 || call_n % 100 == 0 {
            tracing::debug!(
                subsystem = "engine",
                event = "set_clock_est",
                call_n,
                mcu,
                freq = freq as u64,
                offset,
                last_clock,
                "[engine-trace] set_clock_est"
            );
        }
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        router
            .set_clock_est_rebased(
                mcu_handle_from_raw(mcu),
                freq,
                offset,
                last_clock,
                host_now_raw,
            )
            .map_err(router_err)?;
        Ok(())
    }

    #[pyo3(signature = (mcu, freq_hz))]
    fn set_nominal_clock_freq(&self, mcu: u32, freq_hz: u32) -> PyResult<()> {
        if freq_hz == 0 {
            return Err(PyRuntimeError::new_err(
                "set_nominal_clock_freq: freq_hz must be nonzero",
            ));
        }
        self.nominal_clock_freqs
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .insert(mcu, freq_hz);
        Ok(())
    }

    fn extract_old(&self, py: Python<'_>, mcu: u32) -> PyResult<Py<PyDict>> {
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        let (sent, received) = router
            .extract_old(mcu_handle_from_raw(mcu))
            .map_err(router_err)?;

        let d = PyDict::new(py);

        let sent_list: Vec<Py<PyDict>> = sent
            .iter()
            .map(|e| {
                let ed = PyDict::new(py);
                ed.set_item("seq", e.seq).unwrap();
                ed.set_item("data", pyo3::types::PyBytes::new(py, &e.bytes))
                    .unwrap();
                ed.set_item("timestamp", e.timestamp).unwrap();
                ed.unbind()
            })
            .collect();

        let received_list: Vec<Py<PyDict>> = received
            .iter()
            .map(|e| {
                let ed = PyDict::new(py);
                ed.set_item("seq", e.seq).unwrap();
                ed.set_item("data", pyo3::types::PyBytes::new(py, &e.bytes))
                    .unwrap();
                ed.set_item("timestamp", e.timestamp).unwrap();
                ed.unbind()
            })
            .collect();

        d.set_item("sent", sent_list)?;
        d.set_item("received", received_list)?;
        Ok(d.unbind())
    }

    #[pyo3(signature = (
        axes,
        limits,
        post_processors,
        mcus,
        kinematics_axes,
        cartesian_limits,
        window_capacity = 32,
        beta_max_iters = 10,
        arc_fit = None,
    ))]
    #[allow(clippy::too_many_arguments, clippy::type_complexity)]
    fn init_planner(
        &self,
        axes: Vec<(String, Vec<String>, Vec<String>, Vec<String>)>,
        limits: Vec<(String, Vec<String>, Option<f64>, Option<f64>, Option<f64>)>,
        post_processors: Vec<(String, String, Vec<(String, f64)>)>,
        mcus: Vec<(u32, Vec<u8>, u8)>,
        kinematics_axes: Vec<String>,
        cartesian_limits: (f64, f64, f64, f64, f64, f64),
        window_capacity: usize,
        beta_max_iters: u8,
        arc_fit: Option<(f64, f64)>,
    ) -> PyResult<()> {
        if self
            .planner
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .is_some()
        {
            return Err(PyRuntimeError::new_err("planner already initialized"));
        }

        let axis_registry = config::AxisRegistry::try_new(
            axes.into_iter()
                .map(
                    |(name, follows, motors, post_processors)| config::AxisDecl {
                        name,
                        follows,
                        motors,
                        post_processors,
                    },
                )
                .collect(),
        )
        .map_err(|e| PyValueError::new_err(e.to_string()))?;

        axis_registry
            .validate_motor_mapping(&kinematics_axes)
            .map_err(|e| PyValueError::new_err(e.to_string()))?;

        let pp_decls: Vec<config::PostProcessorDecl> = post_processors
            .into_iter()
            .map(|(name, ty, params)| config::PostProcessorDecl { name, ty, params })
            .collect();
        let post_processor_set = config::PostProcessorSet::try_new(&axis_registry, &pp_decls)
            .map_err(|e| PyValueError::new_err(e.to_string()))?;

        let limit_sections: Vec<config::LimitSection> = limits
            .into_iter()
            .map(|(name, axes, max_velocity, max_accel, max_jerk)| {
                let axes = axes
                    .iter()
                    .map(|a| axis_registry.axis_index(a))
                    .collect::<Result<Vec<usize>, _>>()
                    .map_err(|e| PyValueError::new_err(e.to_string()))?;
                Ok(config::LimitSection {
                    name,
                    axes,
                    max_velocity,
                    max_accel,
                    max_jerk,
                })
            })
            .collect::<PyResult<_>>()?;

        let (
            max_velocity,
            max_accel,
            max_jerk,
            max_z_velocity,
            max_z_accel,
            square_corner_velocity,
        ) = cartesian_limits;
        let cartesian = config::CartesianLimits {
            max_velocity,
            max_accel,
            max_jerk,
            max_z_velocity,
            max_z_accel,
            square_corner_velocity,
        };
        cartesian.validate().map_err(PyValueError::new_err)?;

        let mut cfg = config::PlannerConfig::default();
        cfg.axis_registry = axis_registry;
        cfg.limit_sections = limit_sections;
        cfg.cartesian = cartesian;
        // [limit <name>] sections are parsed so existing configs load, but they
        // are no longer the motion-limit source ([printer] is) and the live
        // stream pipeline never consults them — so they are left unvalidated.
        cfg.post_processors = post_processor_set;
        cfg.window_capacity = window_capacity;
        cfg.beta_max_iters = beta_max_iters;
        cfg.chain = match arc_fit {
            Some((facet_length_mm, max_angle_deg)) => {
                if !(facet_length_mm.is_finite() && facet_length_mm > 0.0) {
                    return Err(PyValueError::new_err(
                        "[arc_fit] facet_length_mm must be finite and positive",
                    ));
                }
                if !(max_angle_deg.is_finite() && max_angle_deg > 0.0 && max_angle_deg < 180.0) {
                    return Err(PyValueError::new_err(
                        "[arc_fit] max_angle_deg must be finite and in (0, 180)",
                    ));
                }
                geometry::ChainFitConfig::with_arc_fit(facet_length_mm, max_angle_deg.to_radians())
            }
            None => geometry::ChainFitConfig::default(),
        };

        *self
            .planner_config
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = cfg.clone();

        let ec_conns: HashMap<u32, Arc<McuSerialConn>> = {
            // Collect (handle, conn, socket_path) in one lock acquisition to
            // close the release_mcu race window between separate lookups.
            let ethercat_handles: Vec<(u32, Arc<McuSerialConn>, String)> = {
                let mcus_lock = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
                mcus.iter()
                    .filter_map(|(handle, _, _)| {
                        let c = mcus_lock.get(handle)?;
                        let socket = c.ethercat_socket.as_ref()?;
                        let conn = c.endpoint_conn.as_ref()?.clone();
                        Some((*handle, conn, socket.clone()))
                    })
                    .collect()
            };

            let mut out = HashMap::new();
            for (mcu_id, conn, socket) in ethercat_handles {
                let caps = query_ethercat_runtime_caps(&conn, std::time::Duration::from_secs(5))
                    .map_err(|e| {
                        PyRuntimeError::new_err(format!(
                            "init_planner: QueryRuntimeCaps failed for ethercat mcu \
                                 {mcu_id} ({socket}): {e} — endpoint must respond with \
                                 RuntimeCapsResponse; is ethercat-rt running?"
                        ))
                    })?;
                tracing::debug!(
                    subsystem = "engine",
                    event = "init_planner_ethercat_caps",
                    mcu_id,
                    total_piece_memory = caps.total_piece_memory,
                    "[caps-trace] init_planner: ethercat mcu caps"
                );
                {
                    let mut mcus_lock = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
                    if let Some(c) = mcus_lock.get_mut(&mcu_id) {
                        c.runtime_caps = Some(caps);
                    }
                }
                out.insert(mcu_id, conn);
            }
            out
        };

        let caps_by_handle: std::collections::HashMap<u32, McuCaps> = {
            let mcus_lock = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            mcus.iter()
                .map(|(handle, _, _)| {
                    let conn = mcus_lock.get(handle).ok_or_else(|| {
                        PyRuntimeError::new_err(format!(
                            "init_planner: unknown mcu_handle {handle}"
                        ))
                    })?;
                    let caps = resolve_motion_caps(conn.runtime_caps, &conn.label, *handle)
                        .map_err(PyRuntimeError::new_err)?;
                    Ok((*handle, caps))
                })
                .collect::<PyResult<_>>()?
        };
        let mcu_configs = build_mcu_configs(&mcus, &caps_by_handle)
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        *self
            .mcu_axis_configs
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = mcu_configs.clone();

        let counter = Arc::clone(&self.dispatched_segments);
        let router_arc = Arc::clone(&self.router);

        let ethercat_mcu_ids: HashSet<u32> = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            mcu_configs
                .iter()
                .filter(|c| {
                    mcus.get(&c.mcu_id)
                        .map_or(false, |conn| conn.ethercat_socket.is_some())
                })
                .map(|c| c.mcu_id)
                .collect()
        };

        let host_ios: HashMap<u32, Arc<McuHostIo>> = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            let mut out = HashMap::new();
            for cfg_mcu in &mcu_configs {
                if ethercat_mcu_ids.contains(&cfg_mcu.mcu_id) {
                    continue;
                }
                let conn = mcus.get(&cfg_mcu.mcu_id).ok_or_else(|| {
                    PyRuntimeError::new_err(format!(
                        "init_planner: unknown mcu_handle {}",
                        cfg_mcu.mcu_id
                    ))
                })?;
                let io = conn.host_io.as_ref().ok_or_else(|| {
                    PyRuntimeError::new_err(format!(
                        "init_planner: attach_serial has not been called for MCU {}",
                        cfg_mcu.mcu_id
                    ))
                })?;
                out.insert(cfg_mcu.mcu_id, Arc::clone(io));
            }
            out
        };

        let ring_depth_table: HashMap<crate::pump::AxisKey, u32> = {
            let mut t = HashMap::new();
            for cfg_mcu in &mcu_configs {
                let total = cfg_mcu.caps.total_pieces() as u32;
                let n = cfg_mcu.axes.len() as u32;
                let depth = axis_ring_depth(total, n);
                for &axis in &cfg_mcu.axes {
                    t.insert(
                        crate::pump::AxisKey {
                            mcu_id: cfg_mcu.mcu_id,
                            axis: axis as u8,
                        },
                        depth,
                    );
                }
            }
            t
        };

        {
            let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
            let now_ns = crate::motion_node::monotonic_ns();
            for &mcu_id in &ethercat_mcu_ids {
                let mcu_h = mcu_handle_from_raw(mcu_id);
                let _ = router.set_clock_est_from_sample(
                    mcu_h,
                    f64::from(ETHERCAT_CLOCK_FREQ_HZ),
                    Instant::now(),
                    now_ns,
                );
            }
        }

        let (pump_tx_init, pump_rx) = std::sync::mpsc::channel::<crate::pump::PumpMsg>();
        let (credit_tx, credit_rx) = crossbeam_channel::unbounded::<crate::pump::FrontierMsg>();
        let initial_frontier = {
            let r = self.router.lock().unwrap_or_else(|p| p.into_inner());
            r.host_now_secs()
        };
        let frontier_bits = Arc::new(AtomicU64::new(initial_frontier.to_bits()));
        let gate_bypass = Arc::new(AtomicBool::new(false));

        let wire_transports: HashMap<u32, crate::pump::McuTransport> = {
            let mut t = HashMap::new();
            for (&id, io) in &host_ios {
                t.insert(id, crate::pump::McuTransport::Serial(Arc::downgrade(io)));
            }
            for (&id, conn) in &ec_conns {
                t.insert(
                    id,
                    crate::pump::McuTransport::EtherCat(Arc::downgrade(conn)),
                );
            }
            t
        };

        let pump_timeout = Duration::from_secs(5);
        let ring_depth_table_for_pump = ring_depth_table.clone();
        let router_for_pump = Arc::clone(&self.router);
        let drain_for_pump = self.drain.clone();
        let router_for_freq = Arc::clone(&self.router);
        let pump_thread_handle = std::thread::Builder::new()
            .name("push-pieces-pump".into())
            .spawn(move || {
                let sink = crate::pump::WireSink {
                    transports: wire_transports,
                    timeout: pump_timeout,
                    freq_of: Arc::new(move |mcu_id: u32| {
                        let r = router_for_freq.lock().unwrap_or_else(|p| p.into_inner());
                        r.ack_clock_and_freq(mcu_handle_from_raw(mcu_id))
                            .map(|(_, f)| f)
                    }),
                };
                crate::pump::run_pump(
                    pump_rx,
                    sink,
                    move |k| {
                        ring_depth_table_for_pump
                            .get(&k)
                            .copied()
                            .unwrap_or_else(|| {
                                tracing::error!(
                                    subsystem = "engine",
                                    event = "pump_missing_ring_depth",
                                    axis_key = ?k,
                                    "pump: no ring_depth for axis — absent from init_planner config; using sentinel depth 1 (expect PieceStartInPast fault)"
                                );
                                1
                            })
                    },
                    move |mcu_id: u32| {
                        let r = router_for_pump.lock().unwrap_or_else(|p| p.into_inner());
                        r.ack_clock_and_freq(mcu_handle_from_raw(mcu_id))
                    },
                    |key| {
                        tracing::error!(
                            mcu_id = key.mcu_id,
                            axis = key.axis,
                            "EXIT_ON_FAULT — EtherCAT transport broken-pipe in pump; \
                             aborting klippy so systemd restarts it"
                        );
                        abort_after_tracing_appender_drains();
                    },
                    move |key: crate::pump::AxisKey, n: u32| {
                        drain_for_pump.unsend(key.mcu_id, key.axis, n);
                    },
                    |msg: String| {
                        tracing::error!(
                            msg,
                            "EXIT_ON_FAULT — drip cohort stalled; \
                             aborting klippy so systemd restarts it"
                        );
                        abort_after_tracing_appender_drains();
                    },
                    move |key: crate::pump::AxisKey, freed_time: f64| {
                        if credit_tx
                            .send(crate::pump::FrontierMsg {
                                key,
                                freed_time,
                            })
                            .is_err()
                        {
                            tracing::error!(
                                key = ?key,
                                freed_time,
                                "EXIT_ON_FAULT — stream planner frontier receiver closed; \
                                 aborting klippy so systemd restarts it"
                            );
                            abort_after_tracing_appender_drains();
                        }
                    },
                );
            })
            .expect("spawn push-pieces-pump thread");

        *self.pump_tx.lock().unwrap_or_else(|p| p.into_inner()) = Some(pump_tx_init.clone());
        *self.pump_thread.lock().unwrap_or_else(|p| p.into_inner()) = Some(pump_thread_handle);

        {
            let configs = Arc::clone(&self.mcu_axis_configs);
            let mcus = Arc::clone(&self.mcus);
            let cache = Arc::clone(&self.live_position_cache);
            let stop = Arc::clone(&self.position_poll_stop);
            let handle = std::thread::Builder::new()
                .name("live-position-poll".into())
                .spawn(move || {
                    use std::sync::atomic::Ordering;
                    let period = std::time::Duration::from_millis(200);
                    let timeout = std::time::Duration::from_millis(250);
                    while !stop.load(Ordering::Relaxed) {
                        std::thread::sleep(period);
                        if stop.load(Ordering::Relaxed) {
                            break;
                        }
                        match collect_motor_positions_inner(&configs, &mcus, timeout) {
                            Ok(map) => {
                                let mut c = cache.lock().unwrap_or_else(|p| p.into_inner());
                                *c = (map, std::time::Instant::now());
                            }
                            Err(e) => {
                                if !e.contains("no axes configured") {
                                    tracing::warn!(
                                        error = %e,
                                        "live-position poll failed; serving stale cache"
                                    );
                                }
                            }
                        }
                    }
                })
                .expect("spawn live-position-poll thread");
            *self
                .position_poll_thread
                .lock()
                .unwrap_or_else(|p| p.into_inner()) = Some(handle);
        }

        for cfg_mcu in &mcu_configs {
            let mcu_id = cfg_mcu.mcu_id;
            let pump_tx_hb = pump_tx_init.clone();
            let drain_hb = self.drain.clone();

            if ethercat_mcu_ids.contains(&mcu_id) {
                let conn = ec_conns
                    .get(&mcu_id)
                    .expect("ec_conns built from ethercat_mcu_ids")
                    .clone();

                let mcu_label = {
                    let mcus_lock = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
                    mcus_lock
                        .get(&mcu_id)
                        .map(|c| c.label.clone())
                        .unwrap_or_else(|| format!("mcu-{mcu_id}"))
                };

                let homing_run_hb = Arc::clone(&self.homing_run);
                let active_cohort_hb = Arc::clone(&self.active_drip_cohort);
                let pump_tx_fault = pump_tx_init.clone();
                let latched_fault_hb = Arc::clone(&self.latched_drive_fault);
                let mcu_label_hb = mcu_label.clone();
                conn.attach_heartbeat_callback(Arc::new(
                    move |hb: &mcu_protocol::messages::StatusHeartbeat| {
                        if hb.fault_code != 0 {
                            let run_opt = {
                                let mut guard =
                                    homing_run_hb.lock().unwrap_or_else(|p| p.into_inner());
                                match guard.as_ref().map(|r| r.axis_key.mcu_id) {
                                    Some(axis_mcu)
                                        if crate::homing::route_drive_fault(
                                            mcu_id,
                                            Some(axis_mcu),
                                        ) == crate::homing::DriveFaultRoute::HomingError =>
                                    {
                                        guard.take()
                                    }
                                    _ => None,
                                }
                            };
                            match run_opt {
                                Some(run) => {
                                    latched_fault_hb
                                        .lock()
                                        .unwrap_or_else(|p| p.into_inner())
                                        .insert(mcu_id, hb.fault_code);
                                    *active_cohort_hb.lock().unwrap_or_else(|p| p.into_inner()) =
                                        None;
                                    let _ = pump_tx_fault.send(crate::pump::PumpMsg::Flush(
                                        run.all_axis_keys.clone(),
                                    ));
                                    let _ = pump_tx_fault
                                        .send(crate::pump::PumpMsg::DripDisarm(run.cohort));
                                    let _ = run.notify.send(Err(format!(
                                        "drive fault 0x{:04x} during homing — \
                                     following-error/torque limit exceeded (endstop failure?)",
                                        hb.fault_code
                                    )));
                                }
                                None => {
                                    let prev = latched_fault_hb
                                        .lock()
                                        .unwrap_or_else(|p| p.into_inner())
                                        .insert(mcu_id, hb.fault_code);
                                    if prev != Some(hb.fault_code) {
                                        tracing::error!(
                                            mcu_id,
                                            mcu_label = %mcu_label_hb,
                                            fault_code = hb.fault_code,
                                            "ethercat drive fault — latched for klippy to report"
                                        );
                                    }
                                }
                            }
                            return;
                        }
                        let _ = pump_tx_hb.send(crate::pump::PumpMsg::Heartbeat(
                            crate::pump::HeartbeatMsg {
                                mcu_id,
                                retired_counts: hb.retired_counts.clone(),
                            },
                        ));
                        for (axis, &r) in hb.retired_counts.iter().enumerate() {
                            drain_hb.set_retired(mcu_id, axis as u8, r);
                        }
                    },
                ));

                // Weak so the supervision thread never keeps the conn (and its
                // reader thread / socket) alive past release_mcu: when the last
                // strong Arc drops, upgrade() fails and the thread exits quietly,
                // letting Drop run shutdown(Both)+join. A strong Arc here would
                // pin the reader thread until this loop happened to notice the
                // release, leaking finished-but-unjoined readers across repeated
                // standalone claim/release.
                let conn_for_poll = Arc::downgrade(&conn);
                let mcus_for_supervision = Arc::clone(&self.mcus);
                let label_for_supervision = mcu_label.clone();
                let on_endpoint_death: Box<dyn Fn(&str) + Send + 'static> =
                    Box::new(move |reason: &str| {
                        tracing::error!(
                            mcu_label = label_for_supervision,
                            mcu_id,
                            reason,
                            "EXIT_ON_FAULT — ethercat endpoint died mid-session; \
                             aborting klippy so systemd restarts it"
                        );
                        abort_after_tracing_appender_drains();
                    });

                let _ = std::thread::Builder::new()
                    .name(format!("ec-heartbeat-poll-{mcu_id}"))
                    .spawn(move || {
                        loop {
                            // Released conn -> exit quietly. This is the common
                            // case: release_mcu drops the last strong Arc, the
                            // upgrade fails, and the thread exits before probing.
                            // The residual race — upgrading the Weak while the conn
                            // is still strong but the MCU was already removed from
                            // the map — is closed by the mcus-map re-check below,
                            // which confirms every fault under the lock.
                            let Some(conn) = conn_for_poll.upgrade() else {
                                return;
                            };

                            // The reader thread sets peer_closed on EOF/IO; no poll here.
                            let peer_eof = conn.peer_closed();
                            drop(conn);

                            // Both fault probes (EOF and child-exit) are confirmed
                            // against the mcus map under one lock acquisition, so a
                            // deliberate release can never be misread as a fault.
                            // release_mcu removes the McuConnection from the map
                            // BEFORE it closes the endpoint socket; that socket
                            // close is exactly what sets peer_closed(). So if we
                            // upgraded the Weak in the race window where the conn
                            // was still strong but the MCU was already removed,
                            // `mcus.get(&mcu_id)` is None here and we exit quietly
                            // instead of firing EXIT_ON_FAULT.
                            let fault_reason = {
                                let mut mcus = mcus_for_supervision
                                    .lock()
                                    .unwrap_or_else(|p| p.into_inner());
                                let Some(c) = mcus.get_mut(&mcu_id) else {
                                    // MCU was released — normal shutdown, exit quietly.
                                    return;
                                };
                                if peer_eof {
                                    Some("conn EOF".to_string())
                                } else if let Some(ref mut child) = c.endpoint_process {
                                    match child.try_wait() {
                                        Ok(Some(status)) => Some(format!("child exited: {status}")),
                                        Ok(None) => None,
                                        Err(e) => Some(format!("try_wait error: {e}")),
                                    }
                                } else {
                                    None
                                }
                            };

                            if let Some(reason) = fault_reason {
                                on_endpoint_death(&reason);
                                return;
                            }

                            std::thread::sleep(Duration::from_millis(1));
                        }
                    })
                    .expect("spawn ec-heartbeat-poll thread");
            } else {
                let io = host_ios
                    .get(&mcu_id)
                    .expect("host_io map built from mcu_configs")
                    .clone();
                io.attach_heartbeat_callback(Arc::new(move |retired: &[u32]| {
                    let _ = pump_tx_hb.send(crate::pump::PumpMsg::Heartbeat(
                        crate::pump::HeartbeatMsg {
                            mcu_id,
                            retired_counts: retired.to_vec(),
                        },
                    ));
                    for (axis, &r) in retired.iter().enumerate() {
                        drain_hb.set_retired(mcu_id, axis as u8, r);
                    }
                }));
            }
        }

        let mcu_configs_for_cb = mcu_configs;
        let router_for_cb = Arc::clone(&router_arc);

        let anchor_mutex = Arc::new(std::sync::Mutex::new(crate::anchor::Anchor::new()));
        let pump_tx_for_cb = pump_tx_init.clone();
        let drain_disp = self.drain.clone();
        let counter_for_cb = Arc::clone(&counter);
        let active_drip_cohort_for_cb = Arc::clone(&self.active_drip_cohort);
        let motion_history_for_cb = Arc::clone(&self.motion_history);
        let nominal_freqs_for_cb = Arc::clone(&self.nominal_clock_freqs);
        let frontier_for_cb = Arc::clone(&frontier_bits);
        let gate_bypass_for_cb = Arc::clone(&gate_bypass);

        let nudge_mcu_configs = mcu_configs_for_cb.clone();
        let nudge_router = Arc::clone(&router_for_cb);
        let nudge_pump_tx = pump_tx_for_cb.clone();
        let nudge_drain = drain_disp.clone();
        let nudge_counter = Arc::clone(&counter_for_cb);
        let nudge_active_drip_cohort = Arc::clone(&active_drip_cohort_for_cb);
        let nudge_motion_history = Arc::clone(&motion_history_for_cb);
        let nudge_nominal_freqs = Arc::clone(&nominal_freqs_for_cb);
        let nudge_anchor_arc = Arc::clone(&anchor_mutex);

        let dispatch: Arc<
            dyn Fn(&trajectory::ShapedSegment) -> Result<(), DispatchError> + Send + Sync,
        > = Arc::new(
            move |seg: &trajectory::ShapedSegment| -> Result<(), DispatchError> {
                tracing::debug!(
                    subsystem = "engine",
                    event = "dispatch_entered",
                    seg_t_start = seg.t_start,
                    seg_t_end = seg.t_end,
                    "[engine-trace] dispatch entered"
                );

                let host_now = {
                    let r = router_for_cb.lock().unwrap_or_else(|p| p.into_inner());
                    r.host_now_secs()
                };

                let frontier = f64::from_bits(frontier_for_cb.load(Ordering::Acquire));
                let mut anchor = anchor_mutex.lock().unwrap_or_else(|p| p.into_inner());
                let seg_host_start = anchor.projected_host_start(seg.t_start, host_now);
                let gate_limit = frontier + crate::pump::LOOKAHEAD_SECS;
                if !gate_bypass_for_cb.load(Ordering::Acquire) && seg_host_start > gate_limit {
                    return Err(DispatchError::Gated);
                }
                let (t0, fresh) = anchor
                    .anchor_segment(seg.t_start, seg.t_end, host_now)
                    .map_err(|late| DispatchError::SegmentLate {
                        gap_s: late.gap_s,
                        seg_t_start: late.seg_t_start,
                    })?;
                drop(anchor);

                if fresh {
                    let r = router_for_cb.lock().unwrap_or_else(|p| p.into_inner());
                    for cfg in mcu_configs_for_cb.iter() {
                        let h = crate::types::mcu_handle_from_raw(cfg.mcu_id);
                        r.log_seg0_deficit(h, t0 + seg.t_start, t0);
                    }
                }

                let project = |mcu_id: u32, host_secs: f64| -> u64 {
                    let r = router_for_cb.lock().unwrap_or_else(|p| p.into_inner());
                    r.host_time_to_mcu_clock(crate::types::mcu_handle_from_raw(mcu_id), host_secs)
                        .unwrap_or(0)
                };

                let active_cohort: Option<u64> = *active_drip_cohort_for_cb
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
                    &mcu_configs_for_cb,
                    t0,
                    fresh,
                    host_now,
                    lead_secs,
                    project,
                    max_piece_secs,
                );

                let nominal_freqs = nominal_freqs_for_cb
                    .lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .clone();
                for m in msgs {
                    let nominal_freq = *nominal_freqs
                        .get(&m.key.mcu_id)
                        .ok_or(DispatchError::MissingNominalFreq(m.key.mcu_id))?;
                    {
                        let mut store = motion_history_for_cb
                            .lock()
                            .unwrap_or_else(|p| p.into_inner());
                        for (piece, _host_t) in &m.pieces {
                            store.record(m.key, piece, nominal_freq);
                        }
                    }
                    drain_disp.add_sent(m.key.mcu_id, m.key.axis, m.pieces.len() as u32);
                    pump_tx_for_cb
                        .send(crate::pump::PumpMsg::Enqueue(m))
                        .map_err(|_| DispatchError::PumpGone)?;
                }

                counter_for_cb.fetch_add(1, Ordering::Relaxed);
                Ok(())
            },
        );

        let nudge_dispatch: Arc<
            dyn Fn(u32, &crate::nudge::NudgePiece) -> Result<(), DispatchError> + Send + Sync,
        > = Arc::new(
            move |mcu_id: u32, np: &crate::nudge::NudgePiece| -> Result<(), DispatchError> {
                let axis = np.axis;
                if !nudge_mcu_configs.iter().any(|c| c.mcu_id == mcu_id) {
                    return Err(DispatchError::NudgeTargetMissing { mcu_id, axis });
                }

                let host_now = {
                    let r = nudge_router.lock().unwrap_or_else(|p| p.into_inner());
                    r.host_now_secs()
                };

                let active_cohort: Option<u64> = *nudge_active_drip_cohort
                    .lock()
                    .unwrap_or_else(|p| p.into_inner());

                let lead_secs = if active_cohort.is_some() {
                    crate::pump::DRIP_WINDOW_SECS
                } else {
                    crate::pump::MAX_LEAD_SECS
                };

                let (t0, fresh) = nudge_anchor_arc
                    .lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .anchor_segment(np.piece.u_start, np.piece.u_end, host_now)
                    .map_err(|late| DispatchError::SegmentLate {
                        gap_s: late.gap_s,
                        seg_t_start: late.seg_t_start,
                    })?;

                if fresh {
                    let r = nudge_router.lock().unwrap_or_else(|p| p.into_inner());
                    let h = crate::types::mcu_handle_from_raw(mcu_id);
                    r.log_seg0_deficit(h, t0 + np.piece.u_start, t0);
                }

                let project = |proj_mcu_id: u32, host_secs: f64| -> u64 {
                    let r = nudge_router.lock().unwrap_or_else(|p| p.into_inner());
                    r.host_time_to_mcu_clock(
                        crate::types::mcu_handle_from_raw(proj_mcu_id),
                        host_secs,
                    )
                    .unwrap_or(0)
                };

                let max_piece_secs = if active_cohort.is_some() {
                    Some(0.025_f64)
                } else {
                    None::<f64>
                };

                let pieces = crate::enqueue::flatten_bezier_pieces(
                    std::slice::from_ref(&np.piece),
                    t0,
                    mcu_id,
                    axis as usize,
                    host_now,
                    &project,
                    max_piece_secs,
                    np.motor_mask,
                );

                if !pieces.is_empty() {
                    let key = crate::pump::AxisKey { mcu_id, axis };
                    let nominal_freq = {
                        let freqs = nudge_nominal_freqs
                            .lock()
                            .unwrap_or_else(|p| p.into_inner());
                        *freqs
                            .get(&mcu_id)
                            .ok_or(DispatchError::MissingNominalFreq(mcu_id))?
                    };
                    {
                        let mut store = nudge_motion_history
                            .lock()
                            .unwrap_or_else(|p| p.into_inner());
                        for (piece, _host_t) in &pieces {
                            store.record(key, piece, nominal_freq);
                        }
                    }
                    nudge_drain.add_sent(mcu_id, axis, pieces.len() as u32);
                    nudge_pump_tx
                        .send(crate::pump::PumpMsg::Enqueue(crate::pump::EnqueueMsg {
                            key,
                            pieces,
                            fresh_stream: fresh,
                            lead_secs,
                        }))
                        .map_err(|_| DispatchError::PumpGone)?;
                }

                nudge_counter.fetch_add(1, Ordering::Relaxed);
                Ok(())
            },
        );

        {
            let mut guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
            if guard.is_some() {
                return Err(PyRuntimeError::new_err(
                    "planner already initialized (raced)",
                ));
            }
            let cart = cfg.cartesian;
            let stream_cfg = crate::stream::StreamConfig {
                chain: cfg.chain,
                velocity: geometry::VelocityConfig {
                    max_jerk_mm_s3: cart.max_jerk,
                    ..geometry::VelocityConfig::default()
                },
                fit_tol_mm: cfg.fit_tolerance_mm,
                keep_secs: STREAM_KEEP_SECS,
                limits: geometry::VelocityLimits::try_new(
                    cart.max_velocity,
                    cart.max_accel,
                    cart.square_corner_velocity,
                )
                .map_err(PyRuntimeError::new_err)?,
            };
            let home = vec![0.0; cfg.axis_registry.n_axes()];
            let frontier_keys = ring_depth_table.keys().copied().collect();
            *guard = Some(StreamPlannerHandle::spawn(
                stream_cfg,
                home,
                dispatch,
                nudge_dispatch,
                credit_rx,
                frontier_bits,
                gate_bypass,
                frontier_keys,
            ));
        }
        Ok(())
    }

    #[pyo3(signature = (dx, dy, dz, de, feedrate))]
    fn submit_move(
        &self,
        py: Python<'_>,
        dx: f64,
        dy: f64,
        dz: f64,
        de: f64,
        feedrate: f64,
    ) -> PyResult<()> {
        tracing::debug!(
            subsystem = "motion",
            event = "submit_move_enter",
            dx,
            dy,
            dz,
            de,
            feedrate,
            "engine.submit_move enter"
        );
        py.detach(|| -> PyResult<()> {
            let followers = self.e_followers(de)?;
            let (extruder_axis, e_delta) = match followers.as_slice() {
                [] => (0usize, 0.0),
                [(axis, delta)] => (*axis, *delta),
                _ => {
                    return Err(PyRuntimeError::new_err(
                        "submit_move: multiple follower axes not yet supported by the new pipeline",
                    ));
                }
            };
            let pos = *self.commanded_pos.lock().unwrap_or_else(|p| p.into_inner());
            let (max_v, max_a, scv) = {
                let cfg = self
                    .planner_config
                    .lock()
                    .unwrap_or_else(|p| p.into_inner());
                let (mut v, mut a) = cfg.cartesian.for_move(dx, dy, dz);
                if let Some(rv) = cfg.runtime_caps.velocity {
                    v = v.min(rv);
                }
                if let Some(ra) = cfg.runtime_caps.accel {
                    a = a.min(ra);
                }
                (v, a, cfg.square_corner_velocity())
            };
            let limits = geometry::VelocityLimits::try_new(max_v, max_a, scv)
                .map_err(PyRuntimeError::new_err)?;
            let m =
                classify::build_move(pos, dx, dy, dz, extruder_axis, e_delta, limits, feedrate, 0)
                    .map_err(|e| PyRuntimeError::new_err(format!("{e:?}")))?;

            {
                let guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
                let planner = guard.as_ref().ok_or_else(|| {
                    PyRuntimeError::new_err("planner not initialized — call init_planner first")
                })?;
                planner.submit_move(m).map_err(planner_err)?;
            }

            let mut pos = self.commanded_pos.lock().unwrap_or_else(|p| p.into_inner());
            pos[0] += dx;
            pos[1] += dy;
            pos[2] += dz;
            *self.last_g5_pq.lock().unwrap_or_else(|p| p.into_inner()) = None;
            Ok(())
        })
    }

    #[pyo3(signature = (i, j, p, q, dx, dy, dz, de, feedrate))]
    fn submit_bezier(
        &self,
        py: Python<'_>,
        i: Option<f64>,
        j: Option<f64>,
        p: f64,
        q: f64,
        dx: f64,
        dy: f64,
        dz: f64,
        de: f64,
        feedrate: f64,
    ) -> PyResult<()> {
        tracing::debug!(
            subsystem = "motion",
            event = "submit_bezier_enter",
            i = ?i,
            j = ?j,
            p,
            q,
            dx,
            dy,
            dz,
            de,
            feedrate,
            "engine.submit_bezier enter"
        );
        py.detach(|| -> PyResult<()> {
            Err(PyRuntimeError::new_err(
                "submit_bezier (G5 cubic) is not yet supported by the new geometry pipeline \
                 — V1 streams G0/G1 line moves (and reconstructs arcs from facets); curve \
                 faceting is a follow-up. Slice without G5.",
            ))
        })
    }

    #[pyo3(signature = (i, j, dx, dy, dz, de, feedrate))]
    fn submit_quadratic(
        &self,
        py: Python<'_>,
        i: f64,
        j: f64,
        dx: f64,
        dy: f64,
        dz: f64,
        de: f64,
        feedrate: f64,
    ) -> PyResult<()> {
        tracing::debug!(
            subsystem = "motion",
            event = "submit_quadratic_enter",
            i,
            j,
            dx,
            dy,
            dz,
            de,
            feedrate,
            "engine.submit_quadratic enter"
        );
        py.detach(|| -> PyResult<()> {
            Err(PyRuntimeError::new_err(
                "submit_quadratic (G2/G3 arc as quadratic) is not yet supported by the new \
                 geometry pipeline — V1 streams G0/G1 line moves; curve faceting is a \
                 follow-up. Decompose arcs into line segments upstream.",
            ))
        })
    }

    fn wait_moves(&self, py: Python<'_>) -> PyResult<()> {
        let guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
        let planner = guard.as_ref().ok_or_else(|| {
            PyRuntimeError::new_err("planner not initialized — call init_planner first")
        })?;
        py.detach(|| planner.flush()).map_err(planner_err)
    }

    fn drain_motion(&self, py: Python<'_>) -> PyResult<()> {
        let guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
        let planner = guard.as_ref().ok_or_else(|| {
            PyRuntimeError::new_err("planner not initialized — call init_planner first")
        })?;
        py.detach(|| planner.flush()).map_err(planner_err)?;
        let drain = self.drain.clone();
        py.detach(|| drain.wait_drained(DRAIN_TIMEOUT))
            .map_err(PyRuntimeError::new_err)
    }

    fn wait_moves_start(&self) -> PyResult<u64> {
        let rx = {
            let guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
            let planner = guard.as_ref().ok_or_else(|| {
                PyRuntimeError::new_err("planner not initialized — call init_planner first")
            })?;
            planner.flush_start().map_err(planner_err)?
        };
        let mut pending = self
            .pending_flushes
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        let id = self.next_flush_id.fetch_add(1, Ordering::Relaxed);
        pending.insert(id, FlushWait { rx, deadline: None });
        Ok(id)
    }

    fn wait_moves_poll(&self, flush_id: u64) -> PyResult<bool> {
        let mut pending = self
            .pending_flushes
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        let Some(wait) = pending.get_mut(&flush_id) else {
            return Err(PyRuntimeError::new_err(format!(
                "wait_moves_poll: unknown flush id {flush_id}"
            )));
        };
        if wait.deadline.is_none() {
            match wait.rx.try_recv() {
                Ok(finish) => {
                    wait.deadline = Some(finish.unwrap_or_else(std::time::Instant::now));
                }
                Err(crossbeam_channel::TryRecvError::Empty) => return Ok(false),
                Err(crossbeam_channel::TryRecvError::Disconnected) => {
                    pending.remove(&flush_id);
                    return Err(PyRuntimeError::new_err(
                        "wait_moves_poll: planner channel closed",
                    ));
                }
            }
        }
        let done = wait
            .deadline
            .map(|d| std::time::Instant::now() >= d)
            .unwrap_or(false);
        if done {
            pending.remove(&flush_id);
        }
        Ok(done)
    }

    fn motion_drain_poll(&self, py: Python<'_>) -> PyResult<bool> {
        let guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
        let planner = guard.as_ref().ok_or_else(|| {
            PyRuntimeError::new_err("planner not initialized — call init_planner first")
        })?;
        py.detach(|| planner.flush()).map_err(planner_err)?;
        Ok(self.drain.is_drained_now())
    }

    fn motion_drain_finalize(&self) {}

    fn submit_dwell(&self, duration_s: f64) -> PyResult<()> {
        let guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
        let planner = guard.as_ref().ok_or_else(|| {
            PyRuntimeError::new_err("planner not initialized — call init_planner first")
        })?;
        planner.dwell(duration_s).map_err(planner_err)?;
        *self.last_g5_pq.lock().unwrap_or_else(|p| p.into_inner()) = None;
        Ok(())
    }

    #[pyo3(signature = (x, y, z, host_now))]
    fn set_position(&self, py: Python<'_>, x: f64, y: f64, z: f64, host_now: f64) -> PyResult<()> {
        {
            let mut pos = self.commanded_pos.lock().unwrap_or_else(|p| p.into_inner());
            *pos = [x, y, z];
        }
        *self.last_g5_pq.lock().unwrap_or_else(|p| p.into_inner()) = None;
        let planner_guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
        if let Some(planner) = planner_guard.as_ref() {
            py.detach(|| planner.flush()).map_err(planner_err)?;
            {
                let drain = self.drain.clone();
                py.detach(|| drain.wait_drained(DRAIN_TIMEOUT))
                    .map_err(PyRuntimeError::new_err)?;
            }

            planner
                .stream_open(vec![x, y, z, 0.0])
                .map_err(planner_err)?;

            self.drain.reset();

            let sends = {
                let configs = self
                    .mcu_axis_configs
                    .lock()
                    .unwrap_or_else(|p| p.into_inner());
                let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
                let ethercat_mcu_ids: HashSet<u32> = configs
                    .iter()
                    .filter(|c| {
                        mcus.get(&c.mcu_id)
                            .map_or(false, |conn| conn.ethercat_socket.is_some())
                    })
                    .map(|c| c.mcu_id)
                    .collect();
                crate::dispatch::build_serial_seed_sends(&configs, &ethercat_mcu_ids, x, y, z)
            };
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            for s in sends {
                let conn = mcus.get(&s.mcu_id).unwrap_or_else(|| {
                    panic!(
                        "set_position seed: planner up but mcu_id {} absent \
                         (broken invariant)",
                        s.mcu_id
                    )
                });
                let io = conn.host_io.as_ref().unwrap_or_else(|| {
                    panic!(
                        "set_position seed: serial mcu_id {} has no host_io \
                         (broken invariant — attach_serial not called?)",
                        s.mcu_id
                    )
                });
                io.send_typed(
                    "runtime_seed_position",
                    &[
                        ("x_q16", FieldValue::I32(s.x_q16)),
                        ("y_q16", FieldValue::I32(s.y_q16)),
                        ("z_q16", FieldValue::I32(s.z_q16)),
                    ],
                )
                .map_err(|e| {
                    PyRuntimeError::new_err(format!(
                        "set_position seed send to mcu_id {} failed: {e:?}",
                        s.mcu_id
                    ))
                })?;
            }
        }

        {
            let configs: Vec<crate::dispatch::McuAxisConfig> = self
                .mcu_axis_configs
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .clone();
            let positions = [x, y, z];
            let rebases: Vec<(crate::pump::AxisKey, u64, f64)> = {
                let router = self.router.lock().unwrap_or_else(|p| p.into_inner());
                configs
                    .iter()
                    .flat_map(|cfg| {
                        let handle = crate::types::mcu_handle_from_raw(cfg.mcu_id);
                        let now_clock =
                            router.host_time_to_mcu_clock(handle, host_now).unwrap_or(0);
                        cfg.axes
                            .iter()
                            .filter(|&&a| a < SPATIAL_AXES)
                            .map(move |&axis| {
                                let key = crate::pump::AxisKey {
                                    mcu_id: cfg.mcu_id,
                                    axis: axis as u8,
                                };
                                (key, now_clock, positions[axis])
                            })
                            .collect::<Vec<_>>()
                    })
                    .collect()
            };
            let follower_rebases: Vec<(crate::pump::AxisKey, u64)> = {
                let router = self.router.lock().unwrap_or_else(|p| p.into_inner());
                configs
                    .iter()
                    .flat_map(|cfg| {
                        let handle = crate::types::mcu_handle_from_raw(cfg.mcu_id);
                        let now_clock =
                            router.host_time_to_mcu_clock(handle, host_now).unwrap_or(0);
                        cfg.axes
                            .iter()
                            .filter(|&&a| a >= 3)
                            .map(move |&axis| {
                                (
                                    crate::pump::AxisKey {
                                        mcu_id: cfg.mcu_id,
                                        axis: axis as u8,
                                    },
                                    now_clock,
                                )
                            })
                            .collect::<Vec<_>>()
                    })
                    .collect()
            };
            {
                let mut store = self
                    .motion_history
                    .lock()
                    .unwrap_or_else(|p| p.into_inner());
                for (key, now_clock, pos) in rebases {
                    store.rebase_axis(key, now_clock, pos);
                }
                for (key, now_clock) in follower_rebases {
                    let held_position = store.final_position(key).unwrap_or(0.0);
                    store.rebase_axis(key, now_clock, held_position);
                }
            }
        }

        Ok(())
    }

    fn effective_limits(&self) -> (f64, f64, f64) {
        self.planner_config
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .effective_limits()
    }

    #[pyo3(signature = (velocity))]
    fn set_velocity_cap(&self, velocity: Option<f64>) -> PyResult<()> {
        require_positive(velocity, "velocity")?;
        self.planner_config
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .runtime_caps
            .velocity = velocity;
        Ok(())
    }

    #[pyo3(signature = (accel))]
    fn set_accel_cap(&self, accel: Option<f64>) -> PyResult<()> {
        require_positive(accel, "accel")?;
        self.planner_config
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .runtime_caps
            .accel = accel;
        Ok(())
    }

    #[pyo3(signature = (square_corner_velocity))]
    fn set_square_corner_velocity(&self, square_corner_velocity: Option<f64>) -> PyResult<()> {
        if let Some(scv) = square_corner_velocity {
            if !(scv.is_finite() && scv >= 0.0) {
                return Err(PyValueError::new_err(
                    "square_corner_velocity must be finite and non-negative",
                ));
            }
        }
        self.planner_config
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .runtime_square_corner_velocity = square_corner_velocity;
        Ok(())
    }

    fn update_post_processor(&self, name: &str, key: &str, value: f64) -> PyResult<()> {
        // Input shaping is not applied by the V1 streaming pipeline; record the
        // parameter on the config so it round-trips, but there is no planner
        // kernel to update yet.
        self.planner_config
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .post_processors
            .set_param(name, key, value)
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        Ok(())
    }

    fn get_last_move_time(&self) -> f64 {
        match self
            .planner
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .as_ref()
        {
            Some(p) => p.last_move_time(),
            None => 0.0,
        }
    }

    fn motion_lead_secs(&self) -> f64 {
        crate::anchor::DEFAULT_LEAD_SECS
    }

    fn dispatched_segment_count(&self) -> u64 {
        self.dispatched_segments.load(Ordering::Relaxed)
    }

    fn fallback_clock_conversions(&self) -> u64 {
        self.fallback_clock_conversions.load(Ordering::Relaxed)
    }

    #[pyo3(signature = (axis, direction, speed_mm_s, max_travel_mm, endstop_id, endstop_mcu))]
    #[allow(clippy::too_many_arguments)]
    fn home_axis_start(
        &self,
        py: Python<'_>,
        axis: u8,
        direction: f64,
        speed_mm_s: f64,
        max_travel_mm: f64,
        endstop_id: u8,
        endstop_mcu: u32,
    ) -> PyResult<()> {
        use crate::planner::HomeDripParams;

        if axis > 2 {
            return Err(PyRuntimeError::new_err(format!(
                "home_axis: axis {axis} out of range (0=X, 1=Y, 2=Z)"
            )));
        }

        let guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
        let planner = guard
            .as_ref()
            .ok_or_else(|| PyRuntimeError::new_err("home_axis: planner not initialized"))?;

        let (all_axis_keys, _axis_mcu, axis_key) = {
            let configs = self
                .mcu_axis_configs
                .lock()
                .unwrap_or_else(|p| p.into_inner());

            let all_keys = drip_cohort_participants(&configs);
            let found_mcu = configs
                .iter()
                .find(|cfg| cfg.axes.iter().any(|&a| a == axis as usize))
                .map(|cfg| cfg.mcu_id);

            let mcu = found_mcu.ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "home_axis: axis {axis} not found in mcu_axis_configs \
                     (init_planner not called?)"
                ))
            })?;
            let key = crate::pump::AxisKey { mcu_id: mcu, axis };
            (all_keys, mcu, key)
        };

        let cohort: u64 = {
            use std::sync::atomic::AtomicU64;
            static SEQ: AtomicU64 = AtomicU64::new(1);
            SEQ.fetch_add(1, Ordering::Relaxed)
        };

        let start_pos = *self.commanded_pos.lock().unwrap_or_else(|p| p.into_inner());

        {
            let mut latched = self
                .latched_drive_fault
                .lock()
                .unwrap_or_else(|p| p.into_inner());
            latched.remove(&axis_key.mcu_id);
        }

        planner.drain_pending().map_err(|e| {
            self.finish_homing();
            planner_err(e)
        })?;

        {
            let drain = self.drain.clone();
            py.detach(|| drain.wait_drained(DRAIN_TIMEOUT))
                .map_err(PyRuntimeError::new_err)?;
        }

        let homing_host_now = {
            let router = self.router.lock().unwrap_or_else(|p| p.into_inner());
            router.host_now_secs()
        };

        {
            let rebases: Vec<(crate::pump::AxisKey, u64, f64)> = {
                let router = self.router.lock().unwrap_or_else(|p| p.into_inner());
                let store = self
                    .motion_history
                    .lock()
                    .unwrap_or_else(|p| p.into_inner());
                all_axis_keys
                    .iter()
                    .map(|&key| {
                        let clock = router
                            .host_time_to_mcu_clock(
                                mcu_handle_from_raw(key.mcu_id),
                                homing_host_now,
                            )
                            .map_err(|e| {
                                PyRuntimeError::new_err(format!(
                                    "home_axis history rebase: clock projection failed for \
                                     mcu {} axis {}: {e:?}",
                                    key.mcu_id, key.axis
                                ))
                            })?;
                        let position = if usize::from(key.axis) < SPATIAL_AXES {
                            start_pos[usize::from(key.axis)]
                        } else {
                            store.final_position(key).unwrap_or(0.0)
                        };
                        Ok((key, clock, position))
                    })
                    .collect::<PyResult<Vec<_>>>()?
            };
            let mut store = self
                .motion_history
                .lock()
                .unwrap_or_else(|p| p.into_inner());
            for (key, clock, position) in rebases {
                store.rebase_axis(key, clock, position);
            }
        }

        let window_start_clock_in_drip_piece_era = {
            let router = self.router.lock().unwrap_or_else(|p| p.into_inner());
            router
                .host_time_to_mcu_clock(mcu_handle_from_raw(axis_key.mcu_id), homing_host_now)
                .map_err(|e| {
                    PyRuntimeError::new_err(format!(
                        "home_axis: cannot project arm-time clock for axis mcu {}: {e:?}",
                        axis_key.mcu_id
                    ))
                })?
        };

        {
            let mut cohort_guard = self
                .active_drip_cohort
                .lock()
                .unwrap_or_else(|p| p.into_inner());
            *cohort_guard = Some(cohort);
        }

        let pump_tx = self
            .pump_tx
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone()
            .ok_or_else(|| PyRuntimeError::new_err("home_axis: pump not started"))?;

        pump_tx
            .send(crate::pump::PumpMsg::DripArm(crate::pump::DripArm {
                cohort,
                participants: all_axis_keys.clone(),
                timeout: Duration::from_secs(5),
            }))
            .map_err(|_| PyRuntimeError::new_err("home_axis: pump channel closed"))?;

        let (result_tx, result_rx) =
            crossbeam_channel::bounded::<Result<([f64; 3], [f64; 3], u64), String>>(1);

        {
            let mut run = self.homing_run.lock().unwrap_or_else(|p| p.into_inner());
            *run = Some(HomingRun {
                cohort,
                endstop_id,
                endstop_mcu,
                axis,
                axis_key,
                all_axis_keys: all_axis_keys.clone(),
                window_start_clock: window_start_clock_in_drip_piece_era,
                notify: result_tx,
            });
        }

        let home_pos_4 = [start_pos[0], start_pos[1], start_pos[2], 0.0];

        let (planner_done_tx, planner_done_rx) =
            crossbeam_channel::bounded::<Result<(), String>>(1);
        planner
            .home_drip(HomeDripParams {
                home_pos: home_pos_4,
                start: start_pos,
                axis,
                direction,
                speed_mm_s,
                max_travel_mm,
                cohort,
                participants: all_axis_keys.clone(),
                notify: planner_done_tx,
            })
            .map_err(|e| {
                self.finish_homing();
                planner_err(e)
            })?;

        let dispatch = py.detach(|| {
            planner_done_rx
                .recv_timeout(Duration::from_secs(5))
                .map_err(|_| "home_axis: planner timed out dispatching homing move".to_owned())
                .and_then(|r| r)
        });
        if let Err(e) = dispatch {
            self.finish_homing();
            return Err(PyRuntimeError::new_err(e));
        }

        *self.homing_result.lock().unwrap_or_else(|p| p.into_inner()) = Some(result_rx);

        let pending = self
            .pending_trip
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .take();
        if let Some((p_mcu, p_endstop, p_clock)) = pending {
            if p_mcu == endstop_mcu && p_endstop == endstop_id {
                tracing::warn!(
                    subsystem = "trip-relay",
                    event = "early_trip_consumed",
                    mcu = p_mcu,
                    endstop_id = p_endstop,
                    trip_clock = p_clock,
                    "dispatching buffered early trip"
                );
                dispatch_endstop_trip(&self.trip_deps(), p_mcu, p_endstop, p_clock);
            }
        }
        Ok(())
    }

    fn motion_drained(&self) -> bool {
        self.drain.drained()
    }

    fn home_axis_poll(&self) -> PyResult<Option<([f64; 3], [f64; 3], u64)>> {
        let rx = {
            let guard = self.homing_result.lock().unwrap_or_else(|p| p.into_inner());
            match guard.as_ref() {
                Some(rx) => rx.clone(),
                None => {
                    return Err(PyRuntimeError::new_err(
                        "home_axis_poll: no homing in progress",
                    ));
                }
            }
        };
        match rx.try_recv() {
            Err(crossbeam_channel::TryRecvError::Empty) => Ok(None),
            Err(crossbeam_channel::TryRecvError::Disconnected) => {
                self.finish_homing();
                Err(PyRuntimeError::new_err(
                    "home_axis_poll: homing result channel closed",
                ))
            }
            Ok(result) => {
                self.finish_homing();
                let (trip_pos, final_pos, trip_clock) = result.map_err(PyRuntimeError::new_err)?;
                *self.commanded_pos.lock().unwrap_or_else(|p| p.into_inner()) = final_pos;
                self.reanchor_after_trip(final_pos)?;
                Ok(Some((trip_pos, final_pos, trip_clock)))
            }
        }
    }

    fn arm_remote_trigger(&self, mcu_handle: u32, trsync_oid: u32, endstop_id: u8) -> PyResult<()> {
        {
            let armed = self
                .remote_triggers
                .lock()
                .unwrap_or_else(|p| p.into_inner());
            if armed.contains_key(&endstop_id) {
                return Err(PyRuntimeError::new_err(format!(
                    "arm_remote_trigger: endstop_id {endstop_id} is already armed"
                )));
            }
        }
        let host_io = self
            .mcus
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .get(&mcu_handle)
            .and_then(|c| c.host_io.as_ref().map(Arc::clone))
            .ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "arm_remote_trigger: mcu {mcu_handle} has no serial transport"
                ))
            })?;
        let deps = self.trip_deps();
        *self.pending_trip.lock().unwrap_or_else(|p| p.into_inner()) = None;
        let router = Arc::clone(&self.router);
        let fired = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let id = host_io
            .register_frame_interceptor(
                "trsync_state",
                Some(trsync_oid),
                Box::new(move |params| {
                    let decision = crate::remote_trigger::relay_decision(
                        params.try_get_u32("can_trigger"),
                        fired.load(Ordering::SeqCst),
                    );
                    if decision != crate::remote_trigger::RelayAction::Fire {
                        return;
                    }
                    fired.store(true, Ordering::SeqCst);
                    let clock32 = params.try_get_u32("clock").unwrap_or(0);
                    let reference = router
                        .lock()
                        .unwrap_or_else(|p| p.into_inner())
                        .compute_ack_clock(host_rt::passthrough_queue::McuHandle::from_raw(
                            mcu_handle,
                        ))
                        .unwrap_or(0);
                    let clock64 = crate::remote_trigger::relay_trip_clock(clock32, reference);
                    tracing::info!(
                        subsystem = "trip-relay",
                        event = "remote_trigger_fired",
                        mcu = mcu_handle,
                        endstop_id,
                        trsync_oid,
                        clock32,
                        clock64,
                        reason = params.try_get_u32("trigger_reason"),
                        "remote trsync terminal report — dispatching endstop trip"
                    );
                    dispatch_endstop_trip(&deps, mcu_handle, endstop_id, clock64);
                }),
            )
            .map_err(|e| {
                PyRuntimeError::new_err(format!(
                    "arm_remote_trigger: interceptor registration failed: {e:?}"
                ))
            })?;
        self.remote_triggers
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .insert(endstop_id, (mcu_handle, id));
        Ok(())
    }

    fn disarm_remote_trigger(&self, endstop_id: u8) -> PyResult<()> {
        let entry = self
            .remote_triggers
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .remove(&endstop_id);
        let Some((mcu_handle, id)) = entry else {
            return Err(PyRuntimeError::new_err(format!(
                "disarm_remote_trigger: endstop_id {endstop_id} is not armed"
            )));
        };
        let host_io = self
            .mcus
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .get(&mcu_handle)
            .and_then(|c| c.host_io.as_ref().map(Arc::clone));
        match host_io {
            Some(io) => io.unregister_frame_interceptor(id).map_err(|e| {
                PyRuntimeError::new_err(format!("disarm_remote_trigger: unregister failed: {e:?}"))
            }),
            // MCU detached: its reactor (and the interceptor with it) is
            // already gone. Disarm runs on cleanup paths — don't mask the
            // original error.
            None => Ok(()),
        }
    }

    fn home_abort(&self, py: Python<'_>) {
        struct AbortContext {
            all_axis_keys: Vec<crate::pump::AxisKey>,
            cohort: u64,
            axis_key: crate::pump::AxisKey,
            axis: u8,
        }

        let ctx = {
            let guard = self.homing_run.lock().unwrap_or_else(|p| p.into_inner());
            guard.as_ref().map(|r| AbortContext {
                all_axis_keys: r.all_axis_keys.clone(),
                cohort: r.cohort,
                axis_key: r.axis_key,
                axis: r.axis,
            })
        };

        let Some(ctx) = ctx else {
            self.finish_homing();
            return;
        };

        if let Some(tx) = self
            .pump_tx
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone()
        {
            let _ = tx.send(crate::pump::PumpMsg::Flush(ctx.all_axis_keys));
            let _ = tx.send(crate::pump::PumpMsg::DripDisarm(ctx.cohort));
            let (ack_tx, ack_rx) = std::sync::mpsc::sync_channel(1);
            let _ = tx.send(crate::pump::PumpMsg::Barrier(ack_tx));
            let barrier = py.detach(move || ack_rx.recv_timeout(std::time::Duration::from_secs(1)));
            if barrier.is_err() {
                tracing::error!(
                    "home_abort: pump did not acknowledge the flush barrier — \
                     commanded_pos is STALE; a firmware restart is required"
                );
                self.finish_homing();
                return;
            }
        }

        self.finish_homing();

        let final_motor_pos =
            crate::homing::trajectory_final_position(ctx.axis_key, &self.motion_history);

        let final_motor_pos = match final_motor_pos {
            Ok(p) => p,
            Err(e) => {
                tracing::error!(
                    "home_abort: cannot reconcile position after aborted homing move \
                     (trajectory store empty or missing for axis {:?}): {e} — \
                     commanded_pos is STALE; a firmware restart is required to \
                     recover a consistent position",
                    ctx.axis_key
                );
                return;
            }
        };

        let drain = self.drain.clone();
        let drain_result = py.detach(|| drain.wait_drained(DRAIN_TIMEOUT));
        if let Err(e) = drain_result {
            tracing::error!(
                "home_abort: drain timed out after aborted homing move — \
                 commanded_pos is STALE; a firmware restart is required: {e}"
            );
            return;
        }

        let configs = self
            .mcu_axis_configs
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        let kinematics = configs
            .iter()
            .find(|c| c.mcu_id == ctx.axis_key.mcu_id)
            .map_or(1u8, |c| c.kinematics);
        drop(configs);

        let motor_frame =
            trip_position_to_motor_frame(ctx.axis, final_motor_pos, &[], ctx.axis_key.mcu_id);
        let cartesian = KinematicsModule::from_tag(kinematics)
            .expect("build_mcu_configs validated the kinematics tag")
            .inverse(motor_frame);

        let planner_guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
        if let Some(planner) = planner_guard.as_ref() {
            let open_result =
                planner.stream_open(vec![cartesian[0], cartesian[1], cartesian[2], 0.0]);
            if let Err(e) = open_result {
                tracing::error!(
                    "home_abort: runtime_stream_open failed after drain — \
                     commanded_pos is STALE; a firmware restart is required: {e:?}"
                );
                return;
            }
            self.drain.reset();
        }
        drop(planner_guard);

        *self.commanded_pos.lock().unwrap_or_else(|p| p.into_inner()) = cartesian;
        *self.last_g5_pq.lock().unwrap_or_else(|p| p.into_inner()) = None;
    }

    #[pyo3(signature = (source_mcu, clock, host_now))]
    fn motion_state_at_clock(
        &self,
        source_mcu: u32,
        clock: u64,
        host_now: f64,
    ) -> PyResult<std::collections::HashMap<String, (f64, f64, f64)>> {
        const AXIS_NAMES: [&str; 4] = ["x", "y", "z", "e"];
        let configs: Vec<crate::dispatch::McuAxisConfig> = self
            .mcu_axis_configs
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone();
        if configs.is_empty() {
            return Err(PyRuntimeError::new_err(
                "motion_state_at: no axes configured on the engine",
            ));
        }
        let resolved: Vec<(crate::pump::AxisKey, u64, u64)> = {
            let router = self.router.lock().unwrap_or_else(|p| p.into_inner());
            let source_handle = crate::types::mcu_handle_from_raw(source_mcu);
            let mut acc = Vec::new();
            for cfg in &configs {
                let target_handle = crate::types::mcu_handle_from_raw(cfg.mcu_id);
                let axis_clock = crate::motion_history::clock_between_mcus(
                    &router,
                    source_handle,
                    target_handle,
                    clock,
                )
                .map_err(PyRuntimeError::new_err)?;
                let now_clock = router
                    .host_time_to_mcu_clock(target_handle, host_now)
                    .map_err(|e| {
                        PyRuntimeError::new_err(format!(
                            "motion_state_at: clock unsynced for mcu {}: {e:?}",
                            cfg.mcu_id
                        ))
                    })?;
                for &axis in &cfg.axes {
                    let key = crate::pump::AxisKey {
                        mcu_id: cfg.mcu_id,
                        axis: axis as u8,
                    };
                    acc.push((key, axis_clock, now_clock));
                }
            }
            acc
        };
        let store = self
            .motion_history
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        let mut out = std::collections::HashMap::new();
        for (key, axis_clock, now_clock) in resolved {
            let st = match store.state_at_clock(key, axis_clock, Some(now_clock)) {
                Ok(st) => st,
                Err(crate::motion_history::HistoryError::NoHistoryForAxis(_)) => continue,
                Err(e) => return Err(PyRuntimeError::new_err(e.to_string())),
            };
            let name = AXIS_NAMES.get(key.axis as usize).ok_or_else(|| {
                PyRuntimeError::new_err(format!("motion_state_at: unnamed axis {}", key.axis))
            })?;
            out.insert(
                (*name).to_string(),
                (st.position, st.velocity, st.acceleration),
            );
        }
        Ok(out)
    }

    #[pyo3(signature = (timeout_s=0.25))]
    fn query_motor_positions(
        &self,
        py: Python<'_>,
        timeout_s: f64,
    ) -> PyResult<HashMap<String, (f64, f64)>> {
        let timeout = std::time::Duration::from_secs_f64(timeout_s.max(0.0));
        py.detach(|| collect_motor_positions_inner(&self.mcu_axis_configs, &self.mcus, timeout))
            .map_err(PyRuntimeError::new_err)
    }

    fn live_motor_positions(&self) -> std::collections::HashMap<String, (f64, f64)> {
        self.live_position_cache
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .0
            .clone()
    }
}

impl Drop for PyMotionEngine {
    // Backstop for the true-process-exit path: SIGTERM → request_exit → the
    // klippy loop breaks → Py_Finalize → pyo3 drops the engine (if collected).
    // The primary release stays the explicit `klippy:disconnect` → `shutdown()`
    // call so it runs even under `gc.disable()` on the in-process restart loop.
    // `shutdown()` is idempotent, so this never double-tears-down.
    fn drop(&mut self) {
        self.shutdown();
    }
}

struct FlushWait {
    rx: crossbeam_channel::Receiver<Option<std::time::Instant>>,
    deadline: Option<std::time::Instant>,
}

#[derive(Clone)]
pub(crate) struct TripDeps {
    homing_run: Arc<Mutex<Option<HomingRun>>>,
    pending_trip: Arc<Mutex<Option<(u32, u8, u64)>>>,
    active_drip_cohort: Arc<Mutex<Option<u64>>>,
    pump_tx: Arc<Mutex<Option<std::sync::mpsc::Sender<crate::pump::PumpMsg>>>>,
    mcus: Arc<Mutex<HashMap<u32, McuConnection>>>,
    router: Arc<Mutex<PassthroughRouter>>,
    motion_history: Arc<Mutex<crate::motion_history::HistoryStore>>,
    mcu_axis_configs: Arc<Mutex<Vec<McuAxisConfig>>>,
}

impl PyMotionEngine {
    pub(crate) fn trip_deps(&self) -> TripDeps {
        TripDeps {
            homing_run: Arc::clone(&self.homing_run),
            pending_trip: Arc::clone(&self.pending_trip),
            active_drip_cohort: Arc::clone(&self.active_drip_cohort),
            pump_tx: Arc::clone(&self.pump_tx),
            mcus: Arc::clone(&self.mcus),
            router: Arc::clone(&self.router),
            motion_history: Arc::clone(&self.motion_history),
            mcu_axis_configs: Arc::clone(&self.mcu_axis_configs),
        }
    }
}

impl PyMotionEngine {
    fn finish_homing(&self) {
        *self
            .active_drip_cohort
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = None;
        *self.homing_run.lock().unwrap_or_else(|p| p.into_inner()) = None;
        *self.homing_result.lock().unwrap_or_else(|p| p.into_inner()) = None;
    }

    fn reanchor_after_trip(&self, stop_pos: [f64; 3]) -> PyResult<()> {
        let guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
        match guard.as_ref() {
            Some(planner) => planner
                .reset(vec![stop_pos[0], stop_pos[1], stop_pos[2], 0.0])
                .map_err(planner_err),
            None => Ok(()),
        }
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

            use host_rt::mcu_call::McuCall as _;
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

            let axis = run.axis;
            let axis_key = run.axis_key;
            let kinematics = configs
                .iter()
                .find(|c| c.mcu_id == axis_key.mcu_id)
                .map_or(1u8, |c| c.kinematics);
            let reconstruct_cartesian = |source_mcu: u32, clock: u64| -> Result<[f64; 3], String> {
                let motor_pos = crate::homing::reconstruct_axis_position(
                    source_mcu,
                    clock,
                    axis_key,
                    &router_arc,
                    &history_arc,
                    run.window_start_clock,
                )?;
                let motor_frame =
                    trip_position_to_motor_frame(axis, motor_pos, &configs, axis_key.mcu_id);
                Ok(KinematicsModule::from_tag(kinematics)
                    .map_err(|e| e.to_string())?
                    .inverse(motor_frame))
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
    ) {
        self.mcus.lock().unwrap_or_else(|p| p.into_inner()).insert(
            raw,
            McuConnection {
                label: label.to_owned(),
                serial_path: String::new(),
                baud: 0,
                host_io: None,
                runtime_rx_priority: None,
                runtime_rx_bulk: None,
                runtime_caps: None,
                identify_caps: 0,
                mcu_transport_supported: true,
                ethercat_socket: Some(socket_path.to_owned()),
                endpoint_process: Some(child),
                endpoint_conn: Some(Arc::new(conn)),
            },
        );
        self.nominal_clock_freqs
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .insert(raw, ETHERCAT_CLOCK_FREQ_HZ);
    }
}

impl PyMotionEngine {
    fn e_followers(&self, de: f64) -> PyResult<Vec<(usize, f64)>> {
        if de.abs() > 0.0 {
            let cfg = self
                .planner_config
                .lock()
                .unwrap_or_else(|p| p.into_inner());
            let axis_index = cfg.axis_registry.axis_index("e").map_err(|_| {
                PyRuntimeError::new_err(
                    "E word on a move but no [axis e] is declared — declare the \
                     follower axis or stop sending E",
                )
            })?;
            Ok(vec![(axis_index, de)])
        } else {
            Ok(vec![])
        }
    }
}

#[cfg(test)]
mod tests;

#[cfg(test)]
mod place_motor_response_tests;

#[cfg(test)]
mod require_events_dir_tests {
    use super::require_events_dir_for_mcu_transport;
    use std::path::Path;

    #[test]
    fn non_native_no_events_dir_is_ok() {
        assert!(
            require_events_dir_for_mcu_transport(false, None, "mcu-stock").is_ok(),
            "non-native MCU must not require events_dir"
        );
    }

    #[test]
    fn non_native_with_events_dir_is_ok() {
        assert!(
            require_events_dir_for_mcu_transport(
                false,
                Some(Path::new("/tmp/kalico-events")),
                "mcu-stock",
            )
            .is_ok(),
            "non-native MCU must be Ok regardless of events_dir"
        );
    }

    #[test]
    fn native_with_events_dir_is_ok() {
        assert!(
            require_events_dir_for_mcu_transport(
                true,
                Some(Path::new("/tmp/kalico-events")),
                "mcu-h7",
            )
            .is_ok(),
            "native MCU must be Ok when events_dir is set"
        );
    }

    #[test]
    fn native_no_events_dir_is_err_containing_label() {
        let result = require_events_dir_for_mcu_transport(true, None, "mcu-h7");
        assert!(
            result.is_err(),
            "native MCU without events_dir must return Err"
        );
        let msg = result.unwrap_err();
        assert!(
            msg.contains("mcu-h7"),
            "error message must contain the MCU label; got: {msg}"
        );
        assert!(
            msg.contains("init_logging"),
            "error message must mention init_logging; got: {msg}"
        );
    }

    #[test]
    fn native_no_events_dir_err_mentions_mculog_discard() {
        let result = require_events_dir_for_mcu_transport(true, None, "mcu-f4");
        let msg = result.unwrap_err();
        assert!(
            msg.contains("McuLog") || msg.contains("discarded"),
            "error message must explain McuLog discard; got: {msg}"
        );
    }
}

#[cfg(test)]
mod resolve_motion_caps_tests {
    use super::resolve_motion_caps;
    use crate::dispatch::McuCaps;
    use mcu_protocol::messages::RuntimeCapsResponse;

    #[test]
    fn some_caps_returns_ok_with_correct_value() {
        let caps = Some(RuntimeCapsResponse {
            total_piece_memory: 62 * 1024,
        });
        let result = resolve_motion_caps(caps, "octopus", 1);
        assert_eq!(
            result,
            Ok(McuCaps {
                total_piece_memory: 62 * 1024
            })
        );
    }

    #[test]
    fn none_caps_returns_err_containing_label_and_handle() {
        let result = resolve_motion_caps(None, "f446", 7);
        assert!(result.is_err(), "expected Err for None caps");
        let msg = result.unwrap_err();
        assert!(
            msg.contains("f446"),
            "error message should contain the MCU label; got: {msg}"
        );
        assert!(
            msg.contains('7'),
            "error message should contain the handle; got: {msg}"
        );
    }
}

#[cfg(test)]
mod ethercat_endpoint_tests {
    use super::{handshake_ethercat_endpoint, poll_socket_ready, spawn_ethercat_endpoint};
    use std::io::{Read, Write};
    use std::time::{Duration, Instant};

    #[test]
    fn spawn_nonexistent_binary_errors_with_binary_path() {
        let result = spawn_ethercat_endpoint(
            "/nonexistent/binary/kalico-ec",
            "eth0",
            "/tmp/test.sock",
            1.0,
            40.0,
            false,
            None,
            30.0,
            None,
            None,
        );
        assert!(result.is_err(), "expected Err for nonexistent binary");
        let msg = result.unwrap_err();
        assert!(
            msg.contains("/nonexistent/binary/kalico-ec"),
            "error message should contain the binary path; got: {msg}"
        );
        assert!(
            msg.contains("spawn"),
            "error message should indicate a spawn failure; got: {msg}"
        );
    }

    #[test]
    fn poll_socket_ready_detects_early_child_death() {
        let mut child = std::process::Command::new("sh")
            .args(["-c", "exit 3"])
            .spawn()
            .expect("sh must be available");

        // Give the process time to exit so try_wait will see it on the first
        // poll iteration (poll_socket_ready already does this internally, but
        // a brief spin here makes the test deterministic on loaded CI runners).
        let waited = {
            let start = Instant::now();
            loop {
                if child.try_wait().unwrap().is_some() {
                    break start.elapsed();
                }
                std::thread::sleep(Duration::from_millis(5));
                if start.elapsed() > Duration::from_secs(2) {
                    panic!("child did not exit within 2 s");
                }
            }
        };
        let _ = waited;

        let socket_path = "/tmp/kalico_test_socket_that_will_never_exist_a1b2c3d4";
        let deadline = Instant::now() + Duration::from_secs(30);
        let start = Instant::now();
        let result = poll_socket_ready(socket_path, deadline, &mut child);
        let elapsed = start.elapsed();

        assert!(result.is_err(), "expected Err on early child death");
        let msg = result.unwrap_err();
        assert!(
            msg.contains("exit") || msg.contains("exited"),
            "error message should mention exit status; got: {msg}"
        );
        assert!(
            elapsed < Duration::from_secs(5),
            "poll_socket_ready should return promptly on child death, not burn the deadline; \
             elapsed={elapsed:?}"
        );
    }

    fn encode_claim_handshake_reply(correlation_id: u32) -> Vec<u8> {
        use mcu_protocol::codec::Encode as _;
        use mcu_protocol::messages::{ClaimHandshakeReply, MessageKind, SlaveState, SlaveStatus};
        use mcu_transport::frame::{CHANNEL_CONTROL, encode_frame};
        use mcu_transport::wire_helpers::{MESSAGE_VERSION_DEFAULT, encode_message_header};

        let reply = ClaimHandshakeReply {
            slave_statuses: vec![SlaveStatus {
                slave_idx: 0,
                state: SlaveState::Ok,
                fault_code: 0,
            }],
        };
        let mut payload = encode_message_header(
            MessageKind::ClaimHandshakeReply,
            MESSAGE_VERSION_DEFAULT,
            correlation_id,
        )
        .to_vec();
        reply.encode(&mut payload);
        encode_frame(CHANNEL_CONTROL, &payload)
    }

    fn extract_correlation_id(buf: &[u8]) -> u32 {
        use mcu_transport::demux::{Demuxer, Frame};
        use mcu_transport::wire_helpers::decode_message_header;

        let mut demux = Demuxer::new();
        let (frames, _) = demux.feed_slice(buf);
        for f in frames {
            if let Frame::Kalico { payload, .. } = f {
                if let Some((hdr, _)) = decode_message_header(&payload) {
                    return hdr.correlation_id;
                }
            }
        }
        0
    }

    #[test]
    fn handshake_retries_past_stale_socket_file() {
        use std::os::unix::net::UnixListener;

        // Use pid + thread-id to avoid collisions when tests run in parallel.
        let path = format!(
            "/tmp/kalico_test_stale_{}_handshake.sock",
            std::process::id()
        );
        let _ = std::fs::remove_file(&path);

        {
            let _listener = UnixListener::bind(&path)
                .unwrap_or_else(|e| panic!("bind for stale-file setup failed: {e}"));
        }
        assert!(
            std::path::Path::new(&path).exists(),
            "UnixListener drop must leave the socket file — test precondition violated"
        );

        let (tx, rx) = std::sync::mpsc::channel::<()>();
        let path_bg = path.clone();
        let bg = std::thread::spawn(move || {
            let _ = std::fs::remove_file(&path_bg);
            let listener = UnixListener::bind(&path_bg)
                .unwrap_or_else(|e| panic!("background listener bind failed: {e}"));
            let _ = tx.send(());
            if let Ok((mut stream, _)) = listener.accept() {
                let mut buf = [0u8; 4096];
                if let Ok(n) = stream.read(&mut buf) {
                    let cid = extract_correlation_id(&buf[..n]);
                    let reply = encode_claim_handshake_reply(cid);
                    let _ = stream.write_all(&reply);
                    // Shutdown the write half — sends a clean FIN so the
                    // foreground's mcu_call read loop exits on EOF (Closed)
                    // *after* it has already matched the correlated reply frame
                    // and returned Ok.  Without this, dropping the stream under
                    // parallel load can race with the foreground's read.
                    let _ = stream.shutdown(std::net::Shutdown::Write);
                    // Block on a final drain read so we don't release the fd
                    // (and any kernel-buffered data) until the foreground has
                    // consumed everything.
                    let _ = stream.read(&mut buf);
                }
            }
        });

        rx.recv_timeout(Duration::from_secs(5))
            .expect("background listener must signal within 5 s");

        let deadline = Instant::now() + Duration::from_secs(5);
        let result = handshake_ethercat_endpoint(&path, deadline);
        let _ = std::fs::remove_file(&path);

        let succeeded = result.is_ok();
        // Drop the McuSerialConn before joining so the foreground side closes,
        // unblocking the background thread's drain read.
        drop(result);
        let _ = bg.join();

        assert!(succeeded, "handshake must succeed once listener is up");
    }

    #[test]
    fn handshake_connect_refused_is_not_immediately_fatal() {
        use std::os::unix::net::UnixListener;
        use std::sync::Arc;
        use std::sync::atomic::{AtomicBool, Ordering};

        let path = format!(
            "/tmp/kalico_test_refused_{}_handshake.sock",
            std::process::id()
        );
        let _ = std::fs::remove_file(&path);

        {
            let _l = UnixListener::bind(&path).unwrap_or_else(|e| panic!("bind failed: {e}"));
        }

        let tried = Arc::new(AtomicBool::new(false));
        let tried_bg = Arc::clone(&tried);

        let (stop_tx, stop_rx) = std::sync::mpsc::channel::<()>();

        let path_hs = path.clone();
        let hs = std::thread::spawn(move || {
            tried_bg.store(true, Ordering::SeqCst);
            let deadline = Instant::now() + Duration::from_secs(4);
            handshake_ethercat_endpoint(&path_hs, deadline)
        });

        while !tried.load(Ordering::SeqCst) {
            std::thread::yield_now();
        }
        std::thread::sleep(Duration::from_millis(100));

        let _ = std::fs::remove_file(&path);
        let listener =
            UnixListener::bind(&path).unwrap_or_else(|e| panic!("late listener bind failed: {e}"));

        let path_lt = path.clone();
        let lt = std::thread::spawn(move || {
            let _ = stop_rx; // keep channel alive
            if let Ok((mut stream, _)) = listener.accept() {
                let mut buf = [0u8; 4096];
                if let Ok(n) = stream.read(&mut buf) {
                    let cid = extract_correlation_id(&buf[..n]);
                    let _ = stream.write_all(&encode_claim_handshake_reply(cid));
                    let _ = stream.shutdown(std::net::Shutdown::Write);
                    let _ = stream.read(&mut buf);
                }
            }
            let _ = std::fs::remove_file(&path_lt);
        });

        let result = hs.join().expect("handshake thread must not panic");

        let error_msg = match &result {
            Ok(_) => None,
            Err(e) => Some(format!("{e:?}")),
        };

        // Drop the McuSerialConn first so the listener thread's drain read
        // sees EOF and exits. If the handshake never connected at all, the
        // listener is still parked in accept() — unblock it with a throwaway
        // connection so lt.join() cannot hang the test harness.
        let _ = stop_tx.send(());
        drop(result);
        let _ = std::os::unix::net::UnixStream::connect(&path);
        let _ = lt.join();

        if let Some(msg) = error_msg {
            assert!(
                !msg.to_ascii_lowercase().contains("connection refused"),
                "handshake must retry past ConnectionRefused, not fail immediately; got: {msg}"
            );
        }
    }
}

#[cfg(test)]
mod kinematics_calls_tests;

#[cfg(test)]
mod submit_nudge_validation_tests {
    #[test]
    fn multi_bit_mask_is_rejected_by_stepper_sel() {
        assert!(runtime::piece_ring::stepper_sel_from_mask(0b0000_0011).is_err());
        assert!(runtime::piece_ring::stepper_sel_from_mask(0b0000_0010).is_ok());
        assert!(runtime::piece_ring::stepper_sel_from_mask(0).is_ok());
    }
}
