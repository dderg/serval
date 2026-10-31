//! Shared harness for the endpoint integration tests: spawn the drive-off
//! stub, claim it, and clean the child up however the test ends.
#![allow(dead_code)]

use std::process::{Child, Command, ExitStatus};
use std::thread;
use std::time::{Duration, Instant};

use host_rt::mcu_call::McuCall;
use host_rt::mcu_serial_conn::McuSerialConn;
use mcu_protocol::codec::{Cursor, Decode};
use mcu_protocol::messages::{ClaimHandshakeReply, MessageKind};

pub const STUB_BIN: &str = env!("CARGO_BIN_EXE_ethercat-rt-stub");

/// Default stub topology: two slots, one per axis — enough for a paired
/// capture, and what the launcher emits for a two-drive node.
const DEFAULT_SLAVES: [&str; 8] = ["--slave", "0", "--axis", "0", "--slave", "1", "--axis", "1"];

const DEADLINE: Duration = Duration::from_secs(5);
const POLL: Duration = Duration::from_millis(10);

/// Kills the stub when the test ends, however it ends — a leaked endpoint
/// holds its socket and fails every later test on the same path.
pub struct ChildGuard {
    child: Option<Child>,
}

impl ChildGuard {
    pub fn new(child: Child) -> Self {
        Self { child: Some(child) }
    }

    /// Consume without killing — the caller takes responsibility for the child.
    pub fn defuse(&mut self) -> Child {
        self.child.take().expect("already defused")
    }
}

impl Drop for ChildGuard {
    fn drop(&mut self) {
        if let Some(mut c) = self.child.take() {
            let _ = c.kill();
            let _ = c.wait();
        }
    }
}

pub fn socket_path(tag: &str) -> String {
    format!("/tmp/kalico-stub-{}-{}.sock", tag, std::process::id())
}

pub fn wait_for_socket(path: &str, deadline: Instant) {
    while !std::path::Path::new(path).exists() {
        assert!(
            Instant::now() < deadline,
            "stub socket {path:?} did not appear within deadline"
        );
        thread::sleep(POLL);
    }
}

pub fn wait_for_exit(child: &mut Child, deadline: Instant) -> ExitStatus {
    loop {
        match child.try_wait().expect("try_wait must not fail") {
            Some(status) => return status,
            None => {
                assert!(
                    Instant::now() < deadline,
                    "stub process did not exit within deadline — orphan process"
                );
                thread::sleep(POLL);
            }
        }
    }
}

pub fn connect_until(path: &str, deadline: Instant) -> McuSerialConn {
    loop {
        match McuSerialConn::connect(path) {
            Ok(connection) => return connection,
            Err(_) if Instant::now() < deadline => thread::sleep(POLL),
            Err(error) => panic!("connect to {path} failed: {error}"),
        }
    }
}

pub fn handshake(conn: &McuSerialConn) -> ClaimHandshakeReply {
    let (kind, body) = conn
        .mcu_call(MessageKind::ClaimHandshake, Vec::new(), DEADLINE)
        .expect("ClaimHandshake mcu_call must succeed");
    assert_eq!(
        kind,
        MessageKind::ClaimHandshakeReply,
        "expected ClaimHandshakeReply (0x{:04x}), got kind 0x{:04x}",
        MessageKind::ClaimHandshakeReply.as_u16(),
        kind.as_u16(),
    );
    ClaimHandshakeReply::decode_from(&mut Cursor::new(&body))
        .expect("ClaimHandshakeReply must decode from response body")
}

/// Spawn the stub on a fresh socket and connect, without claiming it.
pub fn spawn_stub_unclaimed(tag: &str, extra_args: &[&str]) -> (ChildGuard, McuSerialConn, String) {
    let path = socket_path(tag);
    let _ = std::fs::remove_file(&path);
    let topology: &[&str] = if extra_args.iter().any(|a| *a == "--slave") {
        &[]
    } else {
        &DEFAULT_SLAVES
    };
    let child = Command::new(STUB_BIN)
        .args(["--socket", &path])
        .args(topology)
        .args(extra_args)
        .spawn()
        .expect("stub binary must spawn");
    let guard = ChildGuard::new(child);
    wait_for_socket(&path, Instant::now() + DEADLINE);
    let conn = connect_until(&path, Instant::now() + DEADLINE);
    (guard, conn, path)
}

/// Spawn, connect and complete the claim handshake — the state every test
/// that talks to a live endpoint starts from.
pub fn spawn_and_claim(tag: &str, extra_args: &[&str]) -> (ChildGuard, McuSerialConn, String) {
    let (guard, conn, path) = spawn_stub_unclaimed(tag, extra_args);
    let _reply = handshake(&conn);
    (guard, conn, path)
}
