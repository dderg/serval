use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use host_rt::mcu_serial_conn::McuSerialConn;

use super::abort_after_tracing_appender_drains;
use super::state::EthercatDrive;

/// How long the host waits for klippy to consume the latched endpoint-death
/// cause (clean shutdown) before the watchdog forces a last-resort abort. Sized
/// well above the `DRIVE_FAULT_POLL_PERIOD` (1 s) so a healthy reactor always
/// shuts down cleanly first; the abort only fires if the reactor is wedged.
const ENDPOINT_DEATH_SHUTDOWN_GRACE: Duration = Duration::from_secs(5);

/// Latch an EtherCAT-endpoint-death cause for klippy to surface as the shutdown
/// reason (first cause wins), and log it. Deliberately does NOT abort here: the
/// host shuts down cleanly via `ethercat_node._poll_drive_fault` →
/// `invoke_shutdown` so the operator sees the real cause and runs
/// `FIRMWARE_RESTART` (no silent auto-restart). Returns `true` when this call
/// latched the first cause, so the caller arms the safety watchdog exactly once.
pub(crate) fn report_ethercat_endpoint_death(
    latch: &Arc<Mutex<HashMap<u32, String>>>,
    mcu_id: u32,
    reason: &str,
) -> bool {
    let code = runtime::error::FaultCode::EthercatEndpointDied.as_i32();
    let message = format!("EtherCAT endpoint died mid-session (fault {code}): {reason}");
    let mut guard = latch.lock().unwrap_or_else(|p| p.into_inner());
    // First cause wins for BOTH the latched (operator-surfaced) message and the
    // log: a later writer (e.g. the supervisor after the pump already latched)
    // must not overwrite the original cause.
    if let std::collections::hash_map::Entry::Vacant(slot) = guard.entry(mcu_id) {
        slot.insert(message);
        tracing::error!(
            subsystem = "ethercat",
            event = "endpoint_death",
            mcu_id,
            fault_code = code,
            reason,
            "EtherCAT endpoint died mid-session — latched for klippy; clean shutdown, no abort"
        );
        true
    } else {
        false
    }
}

/// Safety backstop for the clean-shutdown path: if klippy has not consumed the
/// latched endpoint-death cause within the grace (i.e. the reactor never ran the
/// shutdown — wedged/CPU-starved), force a last-resort abort so the machine still
/// stops. The normal path consumes the latch within one poll period, so this only
/// fires on a double failure (endpoint dead AND reactor stuck).
pub(crate) fn arm_endpoint_death_watchdog(latch: Arc<Mutex<HashMap<u32, String>>>, mcu_id: u32) {
    let _ = std::thread::Builder::new()
        .name(format!("ec-death-watchdog-{mcu_id}"))
        .spawn(move || {
            std::thread::sleep(ENDPOINT_DEATH_SHUTDOWN_GRACE);
            let unhandled = latch
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .contains_key(&mcu_id);
            if unhandled {
                tracing::error!(
                    subsystem = "ethercat",
                    event = "endpoint_death_watchdog_abort",
                    mcu_id,
                    grace_secs = ENDPOINT_DEATH_SHUTDOWN_GRACE.as_secs(),
                    "klippy did not act on the latched EtherCAT endpoint death within the grace \
                     — aborting as a last-resort safety stop"
                );
                abort_after_tracing_appender_drains();
            }
        });
}

#[derive(Debug)]
pub(crate) enum EndpointClaimError {
    DriveOffline { slave_idx: u8, fault_code: u16 },
    DriveFault { slave_idx: u8, fault_code: u16 },
    Protocol(String),
}

pub(crate) fn message_for_claim_error(
    label: &str,
    interface: &str,
    e: &EndpointClaimError,
) -> String {
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

fn push_drive_flags(args: &mut Vec<String>, d: &EthercatDrive) {
    let (
        _chain_index,
        _axis,
        counts_per_mm,
        rotation_distance,
        ferr,
        max_torque,
        velocity_ff,
        ff_torque_clamp,
        invert_direction,
        dynamics_profile,
    ) = d;
    args.push("--counts-per-mm".into());
    args.push(counts_per_mm.to_string());
    args.push("--rotation-distance".into());
    args.push(rotation_distance.to_string());
    if let Some(ferr) = ferr {
        args.push("--following-error-counts".into());
        args.push(ferr.to_string());
    }
    if let Some(tq) = max_torque {
        args.push("--max-torque-tenth-pct".into());
        args.push(tq.to_string());
    }
    if *velocity_ff {
        args.push("--velocity-ff".into());
    }
    if *invert_direction {
        args.push("--invert".into());
    }
    args.push("--torque-clamp-pct".into());
    args.push(ff_torque_clamp.to_string());
    if let Some(profile) = dynamics_profile {
        args.push("--slave-dynamics-profile".into());
        args.push(profile.to_string());
    }
}

pub(crate) fn endpoint_args(
    interface: &str,
    socket_path: &str,
    cycle_us: u32,
    dynamics_profile: Option<&str>,
    events_dir: Option<&std::path::Path>,
    drives: &[EthercatDrive],
) -> Vec<String> {
    let mut args = vec![
        interface.to_string(),
        "--socket".into(),
        socket_path.to_string(),
        "--cycle-us".into(),
        cycle_us.to_string(),
    ];
    if let Some(p) = dynamics_profile {
        args.push("--dynamics-profile".into());
        args.push(p.to_string());
    }
    if let Some(dir) = events_dir {
        args.push("--events-dir".into());
        args.push(dir.to_string_lossy().into_owned());
    }
    if drives.len() == 1 {
        push_drive_flags(&mut args, &drives[0]);
    } else {
        for d in drives {
            let (chain_index, axis, ..) = d;
            args.push("--slave".into());
            args.push(chain_index.to_string());
            args.push("--axis".into());
            args.push(axis.to_string());
            push_drive_flags(&mut args, d);
        }
    }
    args
}

pub(crate) fn spawn_ethercat_endpoint(
    binary: &str,
    interface: &str,
    socket_path: &str,
    cycle_us: u32,
    dynamics_profile: Option<&str>,
    events_dir: Option<&std::path::Path>,
    drives: &[EthercatDrive],
) -> Result<std::process::Child, String> {
    let args = endpoint_args(
        interface,
        socket_path,
        cycle_us,
        dynamics_profile,
        events_dir,
        drives,
    );
    std::process::Command::new(binary)
        .args(&args)
        .spawn()
        .map_err(|e| format!("spawn {binary}: {e}"))
}

pub(crate) fn poll_socket_ready(
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

pub(crate) fn handshake_ethercat_endpoint(
    socket_path: &str,
    deadline: Instant,
) -> Result<McuSerialConn, EndpointClaimError> {
    use host_rt::mcu_call::McuCall;
    use mcu_protocol::MessageKind;
    use mcu_protocol::codec::{Cursor, Decode};
    use mcu_protocol::messages::ClaimHandshakeReply;

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
