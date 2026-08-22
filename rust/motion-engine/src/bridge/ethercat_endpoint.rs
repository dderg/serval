use crate::lock_ext::LockExt;
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
    let mut guard = latch.lock_ok();
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
            let unhandled = latch.lock_ok().contains_key(&mcu_id);
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

/// What the endpoint answered when asked which executor it runs.
#[derive(Debug)]
pub(crate) enum ReportedExecutor {
    Code(u8),
    Unsupported(String),
}

#[derive(Debug)]
pub(crate) enum EndpointClaimError {
    DriveOffline { slave_idx: u8, fault_code: u16 },
    DriveFault { slave_idx: u8, fault_code: u16 },
    ExecutorMismatch { reported: ReportedExecutor },
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
        EndpointClaimError::ExecutorMismatch { reported } => match reported {
            ReportedExecutor::Code(code) => format!(
                "ethercat {label}: executor mismatch — endpoint reports executor code {code}, \
                 expected {expected} (setpoint ring) — this endpoint still runs a deleted \
                 executor, rebuild rust/ethercat-rt, then FIRMWARE_RESTART",
                expected = ethercat_rt::setpoint::EXECUTOR_SETPOINT_RING
            ),
            ReportedExecutor::Unsupported(detail) => format!(
                "ethercat {label}: executor mismatch — the endpoint could not report its \
                 executor ({detail}); the endpoint binary predates the sample-stream executor \
                 — rebuild rust/ethercat-rt, then FIRMWARE_RESTART"
            ),
        },
        EndpointClaimError::Protocol(s) => {
            format!("ethercat {label}: endpoint protocol error — {s}")
        }
    }
}

fn push_drive_flags(args: &mut Vec<String>, d: &EthercatDrive) {
    args.push("--counts-per-mm".into());
    args.push(d.counts_per_mm.to_string());
    args.push("--rotation-distance".into());
    args.push(d.rotation_distance.to_string());
    if let Some(ferr) = d.following_error_counts {
        args.push("--following-error-counts".into());
        args.push(ferr.to_string());
    }
    if let Some(tq) = d.max_torque_tenth_pct {
        args.push("--max-torque-tenth-pct".into());
        args.push(tq.to_string());
    }
    if d.velocity_ff {
        args.push("--velocity-ff".into());
    }
    if d.invert_direction {
        args.push("--invert".into());
    }
    args.push("--torque-clamp-pct".into());
    args.push(d.ff_max_torque.to_string());
    if let Some(profile) = &d.dynamics_profile {
        args.push("--slave-dynamics-profile".into());
        args.push(profile.to_string());
    }
}

pub(crate) fn endpoint_args(
    interface: &str,
    socket_path: &str,
    cycle_us: u32,
    dynamics_profile: Option<&str>,
    late_tolerance_us: Option<f64>,
    group_delay_us: f64,
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
    if let Some(tol) = late_tolerance_us {
        args.push("--late-tolerance-us".into());
        args.push(tol.to_string());
    }
    args.push("--group-delay-us".into());
    args.push(group_delay_us.to_string());
    if let Some(dir) = events_dir {
        args.push("--events-dir".into());
        args.push(dir.to_string_lossy().into_owned());
    }
    if drives.len() == 1 {
        push_drive_flags(&mut args, &drives[0]);
    } else {
        for d in drives {
            args.push("--slave".into());
            args.push(d.chain_index.to_string());
            args.push("--axis".into());
            args.push(d.axis.to_string());
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
    late_tolerance_us: Option<f64>,
    group_delay_us: f64,
    events_dir: Option<&std::path::Path>,
    drives: &[EthercatDrive],
) -> Result<std::process::Child, String> {
    let args = endpoint_args(
        interface,
        socket_path,
        cycle_us,
        dynamics_profile,
        late_tolerance_us,
        group_delay_us,
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

/// The endpoint's DC-cycle setpoint grid as reported at claim time. Retained on
/// the per-endpoint `McuConnection` so the pump can map trajectory clocks onto
/// absolute grid indices without re-querying.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct SampleGrid {
    pub(crate) cycle_ticks: u32,
    pub(crate) ring_depth_cycles: u32,
    pub(crate) grid_index: u64,
    pub(crate) grid_clock: u64,
}

/// Ask the endpoint which executor it runs and refuse the claim on any answer
/// that is not the setpoint ring. An endpoint that does not understand
/// `QuerySampleGrid` is a mismatch too: its binary predates the sample stream.
pub(crate) fn verify_sample_grid(
    conn: &McuSerialConn,
    deadline: Instant,
) -> Result<SampleGrid, EndpointClaimError> {
    use host_rt::mcu_call::McuCall;
    use mcu_protocol::MessageKind;
    use mcu_protocol::codec::{Cursor, Decode};
    use mcu_protocol::messages::SampleGridResponse;

    let mismatch = |reported| EndpointClaimError::ExecutorMismatch { reported };

    let remaining = deadline.saturating_duration_since(Instant::now());
    let (kind, body) = conn
        .mcu_call(MessageKind::QuerySampleGrid, Vec::new(), remaining)
        .map_err(|e| {
            mismatch(ReportedExecutor::Unsupported(format!(
                "QuerySampleGrid call failed: {e:?}"
            )))
        })?;

    if kind != MessageKind::SampleGridResponse {
        return Err(mismatch(ReportedExecutor::Unsupported(format!(
            "expected SampleGridResponse (0x{:04x}), got 0x{:04x}",
            MessageKind::SampleGridResponse.as_u16(),
            kind.as_u16(),
        ))));
    }

    let reply = SampleGridResponse::decode_from(&mut Cursor::new(&body)).map_err(|e| {
        mismatch(ReportedExecutor::Unsupported(format!(
            "decode SampleGridResponse: {e:?}"
        )))
    })?;

    if reply.executor != ethercat_rt::setpoint::EXECUTOR_SETPOINT_RING {
        return Err(mismatch(ReportedExecutor::Code(reply.executor)));
    }

    Ok(SampleGrid {
        cycle_ticks: reply.cycle_ticks,
        ring_depth_cycles: reply.ring_depth_cycles,
        grid_index: reply.grid_index,
        grid_clock: reply.grid_clock,
    })
}

/// Build the pump's host-side setpoint filler for a claimed endpoint.
/// Everything the filler needs is what the endpoint itself was launched with —
/// the drives' command scale, the dynamics profile (so the host computes the
/// very same torque feedforward), and the DC grid the endpoint just reported —
/// so a node that cannot produce a filler is a claim failure.
pub(crate) fn build_ring_filler(
    grid: SampleGrid,
    dynamics_profile: Option<&str>,
    drives: &[EthercatDrive],
) -> Result<crate::pump::RingFiller, String> {
    use ethercat_rt::setpoint_fill::{ChainFiller, LaneSpec};

    if grid.cycle_ticks == 0 {
        return Err("endpoint reported a zero-length DC cycle".to_owned());
    }
    let interval_ns = u64::from(grid.cycle_ticks);
    let per_slot: Vec<Option<String>> = drives.iter().map(|d| d.dynamics_profile.clone()).collect();
    let dynamics = ethercat_rt::dynamics::chain_model_from_profiles(
        dynamics_profile,
        &per_slot,
        drives.len(),
    )?;
    let ff_lead_ns = match dynamics.as_ref() {
        Some(model) => model.ff_lead_ns(),
        None => vec![0u64; drives.len()],
    };
    let mut specs: Vec<LaneSpec> = Vec::with_capacity(drives.len());
    for (drive, &ff_lead_ns) in drives.iter().zip(&ff_lead_ns) {
        specs.push(LaneSpec {
            axis: u8::try_from(drive.axis)
                .map_err(|_| format!("drive axis {} exceeds the wire's u8", drive.axis))?,
            cmd_counts_per_mm: if drive.invert_direction {
                -drive.counts_per_mm
            } else {
                drive.counts_per_mm
            },
            ff_lead_ns,
        });
    }
    let lead_cycles = (crate::pump::DRIP_WINDOW_SECS * 1e9 / interval_ns as f64).ceil() as u64;
    let mut filler = ChainFiller::new(&specs, dynamics, interval_ns, lead_cycles);
    filler
        .observe_grid(grid.grid_index, grid.grid_clock)
        .map_err(|e| format!("claim-time sample grid rejected: {e:?}"))?;
    Ok(std::sync::Arc::new(std::sync::Mutex::new(filler)))
}
