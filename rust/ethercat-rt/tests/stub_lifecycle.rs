mod common;

use common::{handshake, spawn_stub_unclaimed, wait_for_exit};
use std::time::{Duration, Instant};

use mcu_protocol::messages::SlaveState;

#[test]
fn stub_claim_succeeds_and_disconnect_terminates_process() {
    let (mut guard, conn, path) = spawn_stub_unclaimed("claim", &["--slave", "0"]);
    let reply = handshake(&conn);

    assert_eq!(
        reply.slave_statuses.len(),
        1,
        "handshake reply must contain exactly 1 slave status, got {}",
        reply.slave_statuses.len()
    );
    assert_eq!(
        reply.slave_statuses[0].state,
        SlaveState::Ok,
        "slave 0 state must be Ok, got {:?}",
        reply.slave_statuses[0].state
    );

    drop(conn);

    let mut child = guard.defuse();
    let _status = wait_for_exit(&mut child, Instant::now() + Duration::from_secs(3));

    let _ = std::fs::remove_file(&path);
}

#[test]
fn stub_fail_bringup_propagates_offline_error() {
    let (mut guard, conn, path) = spawn_stub_unclaimed(
        "fail-bringup",
        &["--slave", "0", "--fail-bringup", "slave=1"],
    );
    let reply = handshake(&conn);

    assert_eq!(
        reply.slave_statuses.len(),
        1,
        "handshake reply must contain exactly 1 slave status, got {}",
        reply.slave_statuses.len()
    );
    assert_eq!(
        reply.slave_statuses[0].state,
        SlaveState::Offline,
        "slave status state must be Offline for --fail-bringup slave=1, got {:?}",
        reply.slave_statuses[0].state
    );
    assert_eq!(
        reply.slave_statuses[0].slave_idx, 1,
        "slave_idx must be 1, got {}",
        reply.slave_statuses[0].slave_idx
    );

    drop(conn);

    let mut child = guard.defuse();
    let status = wait_for_exit(&mut child, Instant::now() + Duration::from_secs(3));

    assert!(
        !status.success(),
        "stub must exit with non-zero status after --fail-bringup, got {status:?}"
    );

    let _ = std::fs::remove_file(&path);
}
