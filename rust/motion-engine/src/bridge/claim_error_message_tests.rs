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
