//! Live tap contract: one scap-v2 header line per connection, then raw
//! capture records for as long as the client stays connected. The stream a
//! client reassembles must decode with the dashboard's own scap parser —
//! that parser is the consumer this socket exists for.

use std::io::Read;
use std::os::unix::net::UnixStream;
use std::path::PathBuf;
use std::time::{Duration, Instant};

use super::*;
use crate::capture::{CaptureRecord, DriveSample};

fn temp_socket(label: &str) -> PathBuf {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    std::env::temp_dir().join(format!(
        "ec_live_tap_{label}_{}_{nanos}.sock",
        std::process::id()
    ))
}

fn two_drive_config() -> Vec<CaptureDriveConfig> {
    (0..2)
        .map(|slot| CaptureDriveConfig {
            slot,
            name: format!("slot{slot}"),
            counts_per_mm: 3276.8,
            rotation_distance: 40.0,
            invert: slot == 1,
        })
        .collect()
}

fn record(cycle_index: u64, ferr0: i32, ferr1: i32) -> CaptureRecord {
    let mut r = CaptureRecord::new(cycle_index, crate::capture::FLAG_MOTION_ACTIVE);
    r.drive_count = 2;
    r.drives[0] = DriveSample {
        following_error: ferr0,
        torque_actual: 100,
        ..DriveSample::default()
    };
    r.drives[1] = DriveSample {
        following_error: ferr1,
        torque_actual: -100,
        ..DriveSample::default()
    };
    r
}

fn wait_until(deadline_ms: u64, mut cond: impl FnMut() -> bool) -> bool {
    let deadline = Instant::now() + Duration::from_millis(deadline_ms);
    while Instant::now() < deadline {
        if cond() {
            return true;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    false
}

fn read_header_line(stream: &mut UnixStream) -> Vec<u8> {
    let mut header = Vec::new();
    let mut byte = [0u8; 1];
    loop {
        stream.read_exact(&mut byte).expect("header byte");
        header.push(byte[0]);
        if byte[0] == b'\n' {
            return header;
        }
    }
}

fn read_exact_with_deadline(stream: &mut UnixStream, n: usize) -> Vec<u8> {
    let mut buf = vec![0u8; n];
    stream
        .set_read_timeout(Some(Duration::from_secs(5)))
        .unwrap();
    let mut filled = 0;
    while filled < n {
        let got = stream.read(&mut buf[filled..]).expect("record bytes");
        assert!(got > 0, "tap closed the stream mid-record");
        filled += got;
    }
    buf
}

#[test]
fn client_disconnect_stops_the_flow_and_reconnect_gets_a_fresh_header() {
    let path = temp_socket("reconnect");
    let tap =
        LiveTap::spawn(path.to_str().unwrap(), two_drive_config(), 250_000).expect("spawn tap");

    let mut first = UnixStream::connect(&path).expect("connect");
    let record_size = {
        let header = read_header_line(&mut first);
        let parsed: serde_json::Value =
            serde_json::from_slice(&header[..header.len() - 1]).unwrap();
        parsed["record_size"].as_u64().unwrap() as usize
    };
    assert!(wait_until(2000, || tap.has_subscriber()));
    drop(first);

    assert!(
        wait_until(2000, || {
            tap.push(record(1, 0, 0));
            !tap.has_subscriber()
        }),
        "a write to the dead client must clear the subscriber flag"
    );

    let mut second = UnixStream::connect(&path).expect("reconnect");
    let header = read_header_line(&mut second);
    let parsed: serde_json::Value =
        serde_json::from_slice(&header[..header.len() - 1]).expect("fresh header");
    assert_eq!(
        parsed["record_size"].as_u64().unwrap() as usize,
        record_size
    );
    assert!(wait_until(2000, || tap.has_subscriber()));

    tap.push(record(42, 7, -7));
    let bytes = read_exact_with_deadline(&mut second, record_size);
    assert_eq!(
        u64::from_le_bytes(bytes[..8].try_into().unwrap()),
        42,
        "records pushed before the reconnect must not leak into the new session"
    );
}

#[test]
fn overflow_is_dropped_not_blocking() {
    let path = temp_socket("overflow");
    let tap =
        LiveTap::spawn(path.to_str().unwrap(), two_drive_config(), 250_000).expect("spawn tap");
    let started = Instant::now();
    for i in 0..(LIVE_TAP_RING_CAPACITY as u64 + 500) {
        tap.push(record(i, 0, 0));
    }
    assert!(
        started.elapsed() < Duration::from_secs(1),
        "push must never block the caller, even with no consumer draining"
    );
}

#[test]
fn drop_joins_the_service_thread() {
    let path = temp_socket("shutdown");
    let tap =
        LiveTap::spawn(path.to_str().unwrap(), two_drive_config(), 250_000).expect("spawn tap");
    drop(tap);
}
