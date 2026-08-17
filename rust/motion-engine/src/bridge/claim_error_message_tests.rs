use super::{EndpointClaimError, Executor, ReportedExecutor, message_for_claim_error};

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
fn executor_mismatch_names_both_sides_and_the_node() {
    let msg = message_for_claim_error(
        "node_x",
        "eth0",
        &EndpointClaimError::ExecutorMismatch {
            requested: Executor::SetpointRing,
            reported: ReportedExecutor::Known(Executor::Piece),
        },
    );
    assert_eq!(
        msg,
        "ethercat node_x: executor mismatch — host requested 'setpoint_ring', endpoint \
         reports 'piece' — set executor= on [ethercat_node node_x] to match the endpoint's \
         --executor, then FIRMWARE_RESTART"
    );
}

#[test]
fn executor_mismatch_unsupported_blames_the_stale_endpoint_binary() {
    let msg = message_for_claim_error(
        "node_x",
        "eth0",
        &EndpointClaimError::ExecutorMismatch {
            requested: Executor::Piece,
            reported: ReportedExecutor::Unsupported("QuerySampleGrid call failed: Timeout".into()),
        },
    );
    assert_eq!(
        msg,
        "ethercat node_x: executor mismatch — host requested 'piece' but the endpoint could \
         not report its executor (QuerySampleGrid call failed: Timeout); the endpoint binary \
         predates the sample-stream executor — rebuild rust/ethercat-rt, then FIRMWARE_RESTART"
    );
}

#[test]
fn executor_mismatch_unknown_code_blames_the_newer_endpoint() {
    let msg = message_for_claim_error(
        "node_x",
        "eth0",
        &EndpointClaimError::ExecutorMismatch {
            requested: Executor::Piece,
            reported: ReportedExecutor::UnknownCode(7),
        },
    );
    assert_eq!(
        msg,
        "ethercat node_x: executor mismatch — host requested 'piece', endpoint reports \
         unknown executor code 7 — the endpoint binary is newer than this host, rebuild \
         both, then FIRMWARE_RESTART"
    );
}
