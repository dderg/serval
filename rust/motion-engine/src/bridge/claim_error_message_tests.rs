use super::{EndpointClaimError, ReportedExecutor, message_for_claim_error};
use host_rt::transport::TransportError;

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

#[test]
fn executor_mismatch_code_blames_the_stale_endpoint_executor() {
    let msg = message_for_claim_error(
        "node_x",
        "eth0",
        &EndpointClaimError::ExecutorMismatch {
            reported: ReportedExecutor::Code(0),
        },
    );
    assert_eq!(
        msg,
        "ethercat node_x: executor mismatch — endpoint reports executor code 0, expected 1 \
         (setpoint ring) — this endpoint still runs a deleted executor, rebuild \
         rust/ethercat-rt, then FIRMWARE_RESTART"
    );
}

#[test]
fn executor_mismatch_unsupported_blames_the_stale_endpoint_binary() {
    let msg = message_for_claim_error(
        "node_x",
        "eth0",
        &EndpointClaimError::ExecutorMismatch {
            reported: ReportedExecutor::Unsupported(
                "expected SampleGridResponse (0x0065), got 0x0061".into(),
            ),
        },
    );
    assert_eq!(
        msg,
        "ethercat node_x: executor mismatch — the endpoint could not report its executor \
         (expected SampleGridResponse (0x0065), got 0x0061); the endpoint binary predates the \
         sample-stream executor — rebuild rust/ethercat-rt, then FIRMWARE_RESTART"
    );
}

#[test]
fn a_timed_out_call_names_the_call_not_a_stale_binary() {
    let msg = message_for_claim_error(
        "node_x",
        "eth0",
        &EndpointClaimError::Transport {
            call: "QuerySampleGrid",
            cause: TransportError::Timeout,
        },
    );
    assert_eq!(
        msg,
        "ethercat node_x: endpoint on eth0 did not answer QuerySampleGrid before the claim \
         deadline — the endpoint process is up but not servicing control frames (RT-starved, \
         wedged, or a binary that ignores QuerySampleGrid); check the endpoint's stderr and \
         rebuild rust/ethercat-rt, then FIRMWARE_RESTART"
    );
}

#[test]
fn a_closed_socket_blames_the_endpoint_exit() {
    let msg = message_for_claim_error(
        "node_x",
        "eth0",
        &EndpointClaimError::Transport {
            call: "ClaimHandshake",
            cause: TransportError::Closed,
        },
    );
    assert_eq!(
        msg,
        "ethercat node_x: endpoint on eth0 closed the control socket during ClaimHandshake — \
         the endpoint exited before answering; check its stderr for the bringup failure, then \
         FIRMWARE_RESTART"
    );
}

#[test]
fn an_io_failure_carries_the_os_error() {
    let msg = message_for_claim_error(
        "node_x",
        "eth0",
        &EndpointClaimError::Transport {
            call: "QuerySampleGrid",
            cause: TransportError::Io(std::io::Error::from(std::io::ErrorKind::BrokenPipe)),
        },
    );
    assert!(
        msg.starts_with(
            "ethercat node_x: control-socket I/O error on eth0 during QuerySampleGrid — "
        ),
        "got: {msg}"
    );
    assert!(msg.ends_with(", then FIRMWARE_RESTART"), "got: {msg}");
}
