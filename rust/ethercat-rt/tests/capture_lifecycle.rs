mod common;

use std::fs;
use std::io::Read as IoRead;
use std::process::{Child, Command};
use std::thread;
use std::time::{Duration, Instant};

use host_rt::mcu_call::McuCall;
use host_rt::mcu_serial_conn::McuSerialConn;
use mcu_protocol::codec::{Cursor, Decode, Encode};
use mcu_protocol::messages::{
    CaptureDrive, ClaimHandshakeReply, MessageKind, StartCapture, StartCaptureResponse,
    StopCapture, StopCaptureResponse,
};

use ethercat_rt::capture::{
    record_size, ERR_CAPTURE_ACTIVE, ERR_CAPTURE_FILE, ERR_CAPTURE_NOT_ACTIVE,
};

const STUB_BIN: &str = env!("CARGO_BIN_EXE_ethercat-rt-stub");

struct ChildGuard {
    child: Option<Child>,
}

impl ChildGuard {
    fn new(child: Child) -> Self {
        Self { child: Some(child) }
    }

    fn defuse(&mut self) -> Child {
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

fn socket_path(tag: &str) -> String {
    format!("/tmp/kalico-cap-{}-{}.sock", tag, std::process::id())
}

fn capture_file(tag: &str) -> String {
    format!("/tmp/kalico-capture-it-{}-{}.scap", tag, std::process::id())
}

fn wait_for_socket(path: &str, deadline: Instant) {
    loop {
        if std::path::Path::new(path).exists() {
            return;
        }
        assert!(
            Instant::now() < deadline,
            "stub socket {path:?} did not appear within deadline"
        );
        thread::sleep(Duration::from_millis(10));
    }
}

fn do_handshake(conn: &McuSerialConn) -> ClaimHandshakeReply {
    let (kind, body) = conn
        .mcu_call(
            MessageKind::ClaimHandshake,
            Vec::new(),
            Duration::from_secs(5),
        )
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

fn spawn_and_claim(tag: &str) -> (ChildGuard, McuSerialConn, String) {
    let path = socket_path(tag);
    let _ = fs::remove_file(&path);

    let child = Command::new(STUB_BIN)
        .args(["--socket", &path])
        .spawn()
        .expect("stub binary must spawn");
    let guard = ChildGuard::new(child);

    wait_for_socket(&path, Instant::now() + Duration::from_secs(5));

    let conn = common::connect_until(&path, Instant::now() + Duration::from_secs(5));
    let _reply = do_handshake(&conn);

    (guard, conn, path)
}

fn start_capture_drives(conn: &McuSerialConn, path: &str, drives: Vec<CaptureDrive>) -> i32 {
    let body = StartCapture {
        path: path.to_owned(),
        started_utc: "2026-06-10T12:00:00Z".to_owned(),
        drives,
    }
    .encoded_to_vec();
    let (kind, resp) = conn
        .mcu_call(MessageKind::StartCapture, body, Duration::from_secs(5))
        .expect("StartCapture call must succeed");
    assert_eq!(
        kind,
        MessageKind::StartCaptureResponse,
        "expected StartCaptureResponse (0x{:04x}), got 0x{:04x}",
        MessageKind::StartCaptureResponse.as_u16(),
        kind.as_u16()
    );
    StartCaptureResponse::decode_from(&mut Cursor::new(&resp))
        .expect("StartCaptureResponse must decode")
        .result
}

fn start_capture(conn: &McuSerialConn, path: &str) -> i32 {
    start_capture_drives(
        conn,
        path,
        vec![CaptureDrive {
            slot: 0,
            name: "x".to_owned(),
        }],
    )
}

fn stop_capture(conn: &McuSerialConn) -> StopCaptureResponse {
    let body = StopCapture.encoded_to_vec();
    let (kind, resp) = conn
        .mcu_call(MessageKind::StopCapture, body, Duration::from_secs(5))
        .expect("StopCapture call must succeed");
    assert_eq!(
        kind,
        MessageKind::StopCaptureResponse,
        "expected StopCaptureResponse (0x{:04x}), got 0x{:04x}",
        MessageKind::StopCaptureResponse.as_u16(),
        kind.as_u16()
    );
    StopCaptureResponse::decode_from(&mut Cursor::new(&resp))
        .expect("StopCaptureResponse must decode")
}

#[test]
fn capture_start_records_stop_produces_consistent_file() {
    let (mut guard, conn, sock) = spawn_and_claim("cap-basic");
    let path = capture_file("basic");
    let _ = fs::remove_file(&path);

    let rc = start_capture(&conn, &path);
    assert_eq!(rc, 0, "StartCapture must return 0, got {rc}");

    thread::sleep(Duration::from_millis(500));

    let resp = stop_capture(&conn);
    assert_eq!(
        resp.result, 0,
        "StopCapture result must be 0, got {}",
        resp.result
    );
    assert!(
        resp.samples > 100,
        "expected >100 samples after 500 ms, got {}",
        resp.samples
    );
    assert_eq!(
        resp.overflow_cycle,
        StopCaptureResponse::NO_OVERFLOW,
        "expected no overflow, got overflow_cycle={}",
        resp.overflow_cycle
    );

    let mut file = fs::File::open(&path).expect("capture file must exist after stop");
    let mut contents = Vec::new();
    file.read_to_end(&mut contents)
        .expect("capture file must be readable");

    let newline_pos = contents
        .iter()
        .position(|&b| b == b'\n')
        .expect("capture file must contain a header newline");
    let header = std::str::from_utf8(&contents[..newline_pos]).expect("header must be valid UTF-8");
    assert!(
        header.contains("\"version\":2"),
        "header must contain \"version\":2; header={header:?}"
    );
    let rsize = record_size(1);
    assert!(
        header.contains(&format!("\"record_size\":{rsize}")),
        "header must contain \"record_size\":{rsize}; header={header:?}"
    );

    let body = &contents[newline_pos + 1..];
    assert_eq!(
        body.len() % rsize,
        0,
        "body length {} is not a multiple of record_size {}",
        body.len(),
        rsize
    );
    let file_records = body.len() / rsize;
    assert_eq!(
        file_records, resp.samples as usize,
        "file record count {file_records} must equal samples {} from StopCaptureResponse",
        resp.samples
    );

    let _ = fs::remove_file(&path);
    drop(conn);
    let _ = guard.defuse().wait();
    let _ = fs::remove_file(&sock);
}

#[test]
fn double_start_rejected_without_killing_first_capture() {
    let (mut guard, conn, sock) = spawn_and_claim("cap-dbl");
    let path1 = capture_file("dbl-1");
    let path2 = capture_file("dbl-2");
    let _ = fs::remove_file(&path1);
    let _ = fs::remove_file(&path2);

    let rc1 = start_capture(&conn, &path1);
    assert_eq!(rc1, 0, "first StartCapture must return 0, got {rc1}");

    let rc2 = start_capture(&conn, &path2);
    assert_eq!(
        rc2, ERR_CAPTURE_ACTIVE,
        "second StartCapture must return ERR_CAPTURE_ACTIVE ({ERR_CAPTURE_ACTIVE}), got {rc2}"
    );

    let resp = stop_capture(&conn);
    assert_eq!(
        resp.result, 0,
        "StopCapture after double-start must return 0, got {}",
        resp.result
    );

    assert!(
        !std::path::Path::new(&path2).exists(),
        "second capture file {path2:?} must NOT exist after rejected start"
    );

    let _ = fs::remove_file(&path1);
    drop(conn);
    let _ = guard.defuse().wait();
    let _ = fs::remove_file(&sock);
}

#[test]
fn stop_without_start_rejected() {
    let (mut guard, conn, sock) = spawn_and_claim("cap-no-start");

    let resp = stop_capture(&conn);
    assert_eq!(
        resp.result,
        ERR_CAPTURE_NOT_ACTIVE,
        "StopCapture without active capture must return ERR_CAPTURE_NOT_ACTIVE ({ERR_CAPTURE_NOT_ACTIVE}), got {}",
        resp.result
    );
    assert_eq!(
        resp.samples, 0,
        "samples must be 0 when no capture was active, got {}",
        resp.samples
    );

    drop(conn);
    let _ = guard.defuse().wait();
    let _ = fs::remove_file(&sock);
}

#[test]
fn unwritable_path_reports_file_error() {
    let (mut guard, conn, sock) = spawn_and_claim("cap-bad-path");

    let rc = start_capture(&conn, "/dev/null/nope/x.scap");
    assert_eq!(
        rc,
        ERR_CAPTURE_FILE,
        "StartCapture with unwritable path must return ERR_CAPTURE_FILE ({ERR_CAPTURE_FILE}), got {rc}"
    );

    drop(conn);
    let _ = guard.defuse().wait();
    let _ = fs::remove_file(&sock);
}

#[test]
fn two_drive_capture_writes_distinct_blocks_per_record() {
    let (mut guard, conn, sock) = spawn_and_claim("cap-2drv");
    let path = capture_file("2drv");
    let _ = fs::remove_file(&path);

    let rc = start_capture_drives(
        &conn,
        &path,
        vec![
            CaptureDrive {
                slot: 0,
                name: "a".to_owned(),
            },
            CaptureDrive {
                slot: 1,
                name: "b".to_owned(),
            },
        ],
    );
    assert_eq!(rc, 0, "two-drive StartCapture must return 0, got {rc}");

    thread::sleep(Duration::from_millis(300));
    let resp = stop_capture(&conn);
    assert_eq!(
        resp.result, 0,
        "StopCapture result must be 0, got {}",
        resp.result
    );

    let mut contents = Vec::new();
    fs::File::open(&path)
        .expect("capture file must exist after stop")
        .read_to_end(&mut contents)
        .expect("capture file must be readable");

    let newline_pos = contents
        .iter()
        .position(|&b| b == b'\n')
        .expect("capture file must contain a header newline");
    let header = std::str::from_utf8(&contents[..newline_pos]).expect("header must be valid UTF-8");
    let rsize = record_size(2);
    assert!(
        header.contains(&format!("\"record_size\":{rsize}")),
        "header must declare record_size {rsize} for two drives; header={header:?}"
    );
    assert!(
        header.contains("\"name\":\"a\"") && header.contains("\"name\":\"b\""),
        "header drives must list both names; header={header:?}"
    );

    let body = &contents[newline_pos + 1..];
    assert!(
        body.len() >= rsize,
        "expected at least one full record, got {} bytes",
        body.len()
    );
    assert_eq!(
        body.len() % rsize,
        0,
        "body {} is not a multiple of record_size {rsize}",
        body.len()
    );

    let prefix = record_size(0);
    let block = record_size(1) - prefix;
    let rec0 = &body[..rsize];
    let block_a = &rec0[prefix..prefix + block];
    let block_b = &rec0[prefix + block..prefix + 2 * block];
    assert_ne!(
        block_a, block_b,
        "the two drive blocks must hold distinct per-slot samples, not one copied sample"
    );

    let _ = fs::remove_file(&path);
    drop(conn);
    let _ = guard.defuse().wait();
    let _ = fs::remove_file(&sock);
}

#[test]
fn rejected_second_start_keeps_running_capture_stride() {
    let (mut guard, conn, sock) = spawn_and_claim("cap-stride");
    let path1 = capture_file("stride-1");
    let path2 = capture_file("stride-2");
    let _ = fs::remove_file(&path1);
    let _ = fs::remove_file(&path2);

    let rc1 = start_capture(&conn, &path1);
    assert_eq!(
        rc1, 0,
        "first one-drive StartCapture must return 0, got {rc1}"
    );

    let rc2 = start_capture_drives(
        &conn,
        &path2,
        vec![
            CaptureDrive {
                slot: 0,
                name: "a".to_owned(),
            },
            CaptureDrive {
                slot: 1,
                name: "b".to_owned(),
            },
        ],
    );
    assert_eq!(
        rc2, ERR_CAPTURE_ACTIVE,
        "second StartCapture must be rejected while active, got {rc2}"
    );

    thread::sleep(Duration::from_millis(200));
    let resp = stop_capture(&conn);
    assert_eq!(
        resp.result, 0,
        "StopCapture must return 0, got {}",
        resp.result
    );

    let mut contents = Vec::new();
    fs::File::open(&path1)
        .expect("first capture file must exist")
        .read_to_end(&mut contents)
        .expect("first capture file must be readable");
    let newline_pos = contents
        .iter()
        .position(|&b| b == b'\n')
        .expect("capture file must contain a header newline");
    let body = &contents[newline_pos + 1..];
    let rsize = record_size(1);
    assert_eq!(
        body.len() % rsize,
        0,
        "running one-drive capture must keep stride record_size(1)={rsize}; a rejected \
         two-drive start must not change it (body={} bytes)",
        body.len()
    );

    assert!(
        !std::path::Path::new(&path2).exists(),
        "rejected second capture file {path2:?} must not exist"
    );

    let _ = fs::remove_file(&path1);
    drop(conn);
    let _ = guard.defuse().wait();
    let _ = fs::remove_file(&sock);
}
