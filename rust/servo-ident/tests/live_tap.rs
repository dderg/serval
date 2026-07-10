//! Live tap consumer contract tests against a fake tap: a unix socket
//! server speaking the frozen wire contract (one scap v2 header line, then
//! raw fixed-size records), fed header and record bytes split out of the
//! committed capture fixture. Covers the attach-now cursor handshake,
//! incremental since_cycle polls, gap surfacing for cycle_index jumps and
//! stale cursors, the unreachable-socket report, the idle hang-up that
//! turns the RT-side tap off, and the HTTP route in front of it all.

use std::io::{Read, Write};
use std::net::TcpStream;
use std::os::unix::net::UnixListener;
use std::path::PathBuf;
use std::sync::mpsc;
use std::time::{Duration, Instant};

use flate2::read::GzDecoder;
use serde_json::Value;

use servo_ident::live_stream::LiveTap;
use servo_ident::scap::{Scap, FLAG_MOTION_ACTIVE};
use servo_ident::{http, serve};

const FIXTURE: &str = "cal_p880_s550_i2273_20260710_151516.scap.gz";
const DEADLINE: Duration = Duration::from_secs(10);
const LONG_IDLE: Duration = Duration::from_secs(60);

struct Fixture {
    header_line: Vec<u8>,
    records: Vec<Vec<u8>>,
    cap: Scap,
}

fn fixture() -> Fixture {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../test/fixtures/servo_captures")
        .join(FIXTURE);
    let gz = std::fs::read(&path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    let mut bytes = Vec::new();
    GzDecoder::new(&gz[..])
        .read_to_end(&mut bytes)
        .expect("fixture gunzips");
    let cap = Scap::from_bytes(&bytes).expect("fixture parses");
    let nl = bytes.iter().position(|&b| b == b'\n').unwrap();
    let header_line = bytes[..=nl].to_vec();
    let records: Vec<Vec<u8>> = bytes[nl + 1..]
        .chunks_exact(cap.header.record_size)
        .map(<[u8]>::to_vec)
        .collect();
    Fixture {
        header_line,
        records,
        cap,
    }
}

fn sock_dir(label: &str) -> PathBuf {
    let stamp = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .subsec_nanos();
    let dir = std::env::temp_dir().join(format!("tap_{label}_{}_{stamp}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

fn poll_until(tap: &LiveTap, since: Option<u64>, pred: impl Fn(&Value) -> bool) -> Value {
    let start = Instant::now();
    loop {
        let v = tap.poll(since);
        if pred(&v) {
            return v;
        }
        assert!(
            start.elapsed() < DEADLINE,
            "timed out polling the live tap; last response: {v}"
        );
        std::thread::sleep(Duration::from_millis(5));
    }
}

fn i64s(v: &Value) -> Vec<i64> {
    v.as_array()
        .unwrap_or_else(|| panic!("expected an array, got {v}"))
        .iter()
        .map(|x| x.as_i64().unwrap())
        .collect()
}

fn ferr_len_is(v: &Value, drive: &str, n: usize) -> bool {
    v["drives"][drive]["ferr"]
        .as_array()
        .is_some_and(|a| a.len() == n)
}

#[test]
fn attach_now_returns_cursor_without_samples_then_polls_stream_new_records() {
    let fx = fixture();
    let dir = sock_dir("attach");
    let sock = dir.join("tap.sock");
    let listener = UnixListener::bind(&sock).unwrap();
    let cycles = fx.cap.read_i64(0, "cycle_index").unwrap();

    let (more_tx, more_rx) = mpsc::channel::<()>();
    let (hold_tx, hold_rx) = mpsc::channel::<()>();
    let header = fx.header_line.clone();
    let first: Vec<u8> = fx.records[..500].concat();
    let second: Vec<u8> = fx.records[500..700].concat();
    let server = std::thread::spawn(move || {
        let (mut s, _) = listener.accept().unwrap();
        s.write_all(&header).unwrap();
        s.write_all(&first).unwrap();
        more_rx.recv().unwrap();
        s.write_all(&second).unwrap();
        hold_rx.recv().ok();
    });

    let tap = LiveTap::new(sock, LONG_IDLE);
    let cursor = cycles[499] as u64;
    let attach = poll_until(&tap, None, |v| {
        v["status"] == "streaming" && v["next_cycle"].as_u64() == Some(cursor)
    });
    assert!(
        attach.get("drives").is_none(),
        "attach-now must carry no samples: {attach}"
    );
    assert_eq!(attach["fs_hz"].as_f64().unwrap(), fx.cap.fs());
    assert_eq!(attach["cycle_ns"].as_u64().unwrap(), fx.cap.header.cycle_ns);
    let names: Vec<&str> = attach["drive_names"]
        .as_array()
        .unwrap()
        .iter()
        .map(|n| n.as_str().unwrap())
        .collect();
    assert_eq!(names, fx.cap.drive_names());

    more_tx.send(()).unwrap();
    let drive0 = fx.cap.drive_names()[0].clone();
    let batch = poll_until(&tap, Some(cursor), |v| ferr_len_is(v, &drive0, 200));
    assert_eq!(batch["first_cycle"].as_u64(), Some(cycles[500] as u64));
    assert_eq!(batch["next_cycle"].as_u64(), Some(cycles[699] as u64));
    assert_eq!(batch["stride"].as_u64(), Some(1));
    assert!(batch.get("gaps").is_none(), "{batch}");
    for (idx, name) in fx.cap.drive_names().iter().enumerate() {
        let sign = if fx.cap.header.drives[idx].invert {
            -1
        } else {
            1
        };
        let host_frame = |v: &[i64]| -> Vec<i64> { v.iter().map(|&x| sign * x).collect() };
        assert_eq!(
            i64s(&batch["drives"][name.as_str()]["ferr"]),
            host_frame(&fx.cap.read_i64(idx, "following_error").unwrap()[500..700]),
            "ferr for {name} must be host-frame (invert applied)"
        );
        assert_eq!(
            i64s(&batch["drives"][name.as_str()]["torque"]),
            host_frame(&fx.cap.read_i64(idx, "torque_actual").unwrap()[500..700]),
            "torque for {name} must be host-frame (invert applied)"
        );
    }
    let expected_moving: Vec<bool> = fx.cap.read_i64(0, "flags").unwrap()[500..700]
        .iter()
        .map(|f| f & FLAG_MOTION_ACTIVE != 0)
        .collect();
    let got_moving: Vec<bool> = batch["moving"]
        .as_array()
        .unwrap()
        .iter()
        .map(|b| b.as_bool().unwrap())
        .collect();
    assert_eq!(got_moving, expected_moving);

    hold_tx.send(()).unwrap();
    server.join().unwrap();
    std::fs::remove_dir_all(&dir).ok();
}

#[test]
fn a_response_never_spans_a_cycle_jump_and_the_next_poll_resumes_after_it() {
    let fx = fixture();
    let dir = sock_dir("gaps");
    let sock = dir.join("tap.sock");
    let listener = UnixListener::bind(&sock).unwrap();
    let cycles = fx.cap.read_i64(0, "cycle_index").unwrap();

    let (hold_tx, hold_rx) = mpsc::channel::<()>();
    let header = fx.header_line.clone();
    let stream_bytes: Vec<u8> = fx.records[..10]
        .iter()
        .chain(&fx.records[20..30])
        .flatten()
        .copied()
        .collect();
    let server = std::thread::spawn(move || {
        let (mut s, _) = listener.accept().unwrap();
        s.write_all(&header).unwrap();
        s.write_all(&stream_bytes).unwrap();
        hold_rx.recv().ok();
    });

    let tap = LiveTap::new(sock, LONG_IDLE);
    poll_until(&tap, None, |v| {
        v["status"] == "streaming" && v["next_cycle"].as_u64() == Some(cycles[29] as u64)
    });

    let drive0 = fx.cap.drive_names()[0].clone();
    let all_ferr = fx.cap.read_i64(0, "following_error").unwrap();
    let stale_cursor = cycles[0] as u64 - 3;
    let pre_gap = tap.poll(Some(stale_cursor));
    assert_eq!(
        pre_gap["first_cycle"].as_u64(),
        Some(cycles[0] as u64),
        "a cursor older than the ring starts at the oldest held record: {pre_gap}"
    );
    assert_eq!(
        pre_gap["next_cycle"].as_u64(),
        Some(cycles[9] as u64),
        "the response must stop at the last record before the jump: {pre_gap}"
    );
    assert_eq!(
        i64s(&pre_gap["drives"][drive0.as_str()]["ferr"]),
        all_ferr[..10].to_vec()
    );
    assert!(pre_gap.get("gaps").is_none(), "{pre_gap}");

    let post_gap = tap.poll(Some(cycles[9] as u64));
    assert_eq!(post_gap["first_cycle"].as_u64(), Some(cycles[20] as u64));
    assert_eq!(post_gap["next_cycle"].as_u64(), Some(cycles[29] as u64));
    assert_eq!(
        i64s(&post_gap["drives"][drive0.as_str()]["ferr"]),
        all_ferr[20..30].to_vec()
    );

    hold_tx.send(()).unwrap();
    server.join().unwrap();
    std::fs::remove_dir_all(&dir).ok();
}

#[test]
fn unreachable_socket_reports_the_connect_error_as_reason() {
    let dir = sock_dir("unreach");
    let tap = LiveTap::new(dir.join("absent.sock"), LONG_IDLE);
    assert_eq!(tap.poll(None)["status"], "connecting");
    let v = poll_until(&tap, None, |v| v["status"] == "unreachable");
    let reason = v["reason"].as_str().unwrap();
    assert!(
        reason.contains("connect") && reason.contains("absent.sock"),
        "reason must name the connect failure and the socket path: {reason}"
    );
    std::fs::remove_dir_all(&dir).ok();
}

#[test]
fn reader_hangs_up_after_the_idle_timeout_and_the_next_poll_reconnects() {
    let fx = fixture();
    let dir = sock_dir("idle");
    let sock = dir.join("tap.sock");
    let listener = UnixListener::bind(&sock).unwrap();
    let cycles = fx.cap.read_i64(0, "cycle_index").unwrap();

    let header = fx.header_line.clone();
    let batches = vec![fx.records[..10].concat(), fx.records[10..20].concat()];
    let (disc_tx, disc_rx) = mpsc::channel::<()>();
    std::thread::spawn(move || {
        for batch in batches {
            let (mut s, _) = listener.accept().unwrap();
            s.write_all(&header).unwrap();
            s.write_all(&batch).unwrap();
            let mut sink = [0u8; 64];
            while matches!(s.read(&mut sink), Ok(n) if n > 0) {}
            let _ = disc_tx.send(());
        }
    });

    let tap = LiveTap::new(sock, Duration::from_millis(200));
    poll_until(&tap, None, |v| {
        v["status"] == "streaming" && v["next_cycle"].as_u64() == Some(cycles[9] as u64)
    });
    disc_rx
        .recv_timeout(DEADLINE)
        .expect("the reader must hang up once polling stops for the idle timeout");
    poll_until(&tap, None, |v| {
        v["status"] == "streaming" && v["next_cycle"].as_u64() == Some(cycles[19] as u64)
    });
    std::fs::remove_dir_all(&dir).ok();
}

struct HttpResult {
    status: u16,
    body: String,
}

fn request(port: u16, path: &str) -> HttpResult {
    let mut stream = TcpStream::connect(("127.0.0.1", port)).expect("connect");
    stream.set_read_timeout(Some(DEADLINE)).unwrap();
    let req = format!(
        "GET {path} HTTP/1.1\r\nHost: 127.0.0.1\r\nContent-Length: 0\r\nConnection: close\r\n\r\n"
    );
    stream.write_all(req.as_bytes()).unwrap();
    let mut raw = Vec::new();
    stream.read_to_end(&mut raw).unwrap();
    let text = String::from_utf8_lossy(&raw);
    let mut parts = text.splitn(2, "\r\n\r\n");
    let head = parts.next().unwrap_or("");
    let body = parts.next().unwrap_or("").to_string();
    let status: u16 = head
        .lines()
        .next()
        .and_then(|l| l.split_whitespace().nth(1))
        .and_then(|s| s.parse().ok())
        .expect("status line");
    HttpResult { status, body }
}

#[test]
fn http_route_serves_live_tap_and_leaves_existing_routes_intact() {
    let dir = sock_dir("http");
    let captures_root = dir.clone();
    let tap = LiveTap::new(dir.join("absent.sock"), LONG_IDLE);
    let listener = http::bind("127.0.0.1", 0).unwrap();
    let port = listener.local_addr().unwrap().port();
    std::thread::spawn(move || {
        http::run(listener, move |req| {
            serve::handle_with_live_tap(&captures_root, &tap, req)
        });
    });

    let bad = request(port, "/api/live_tap?since_cycle=abc");
    assert_eq!(bad.status, 400, "{}", bad.body);

    let start = Instant::now();
    loop {
        let r = request(port, "/api/live_tap");
        assert_eq!(r.status, 200, "{}", r.body);
        let v: Value = serde_json::from_str(&r.body).unwrap();
        if v["status"] == "unreachable" {
            assert!(v["reason"].as_str().unwrap().contains("connect"));
            break;
        }
        assert_eq!(v["status"], "connecting");
        assert!(start.elapsed() < DEADLINE, "never reached unreachable: {v}");
        std::thread::sleep(Duration::from_millis(5));
    }

    let live = request(port, "/api/live");
    assert_eq!(live.status, 200, "{}", live.body);
    let v: Value = serde_json::from_str(&live.body).unwrap();
    assert!(v["capture"].is_null());
    std::fs::remove_dir_all(&dir).ok();
}
