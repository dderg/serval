use super::{
    endpoint_args, handshake_ethercat_endpoint, poll_socket_ready, slot_for_axis,
    spawn_ethercat_endpoint,
};
use std::io::{Read, Write};
use std::time::{Duration, Instant};

#[test]
fn slot_for_axis_maps_hits_and_misses() {
    let slot_axes = [2usize, 5, 7];
    assert_eq!(slot_for_axis(&slot_axes, 2), Some(0));
    assert_eq!(slot_for_axis(&slot_axes, 5), Some(1));
    assert_eq!(slot_for_axis(&slot_axes, 7), Some(2));
    assert_eq!(slot_for_axis(&slot_axes, 0), None);
    assert_eq!(slot_for_axis(&slot_axes, 3), None);
    assert_eq!(slot_for_axis(&[], 0), None);
}

#[test]
fn endpoint_args_single_drive_uses_legacy_form() {
    let args = endpoint_args(
        "eth0",
        "/tmp/x.sock",
        250,
        None,
        None,
        &[(
            1,
            0,
            3276.8,
            40.0,
            Some(8192),
            Some(500),
            false,
            30.0,
            false,
            None,
        )],
    );
    assert!(!args.iter().any(|a| a == "--slave"));
    assert!(!args.iter().any(|a| a == "--axis"));
    let cycle: Vec<&String> = args
        .iter()
        .enumerate()
        .filter(|(i, a)| *a == "--cycle-us" && args.get(i + 1).is_some())
        .map(|(i, _)| &args[i + 1])
        .collect();
    assert_eq!(cycle, vec!["250"]);
    assert!(args.iter().any(|a| a == "--counts-per-mm"));
    assert!(args.iter().any(|a| a == "--following-error-counts"));
    assert!(!args.iter().any(|a| a == "--velocity-ff"));
    assert!(!args.iter().any(|a| a == "--invert"));
    assert!(args.iter().any(|a| a == "--torque-clamp-pct"));
}

#[test]
fn endpoint_args_per_drive_ff_flags() {
    let args = endpoint_args(
        "eth0",
        "/tmp/x.sock",
        250,
        None,
        None,
        &[
            (0, 0, 1000.0, 50.0, None, None, true, 25.0, false, None),
            (1, 2, 2000.0, 40.0, None, None, false, 60.0, true, None),
        ],
    );
    let clamps: Vec<&String> = args
        .iter()
        .enumerate()
        .filter(|(i, a)| *a == "--torque-clamp-pct" && args.get(i + 1).is_some())
        .map(|(i, _)| &args[i + 1])
        .collect();
    assert_eq!(clamps, vec!["25", "60"]);
    let velocity_ff_count = args.iter().filter(|a| *a == "--velocity-ff").count();
    assert_eq!(velocity_ff_count, 1);
    let invert_count = args.iter().filter(|a| *a == "--invert").count();
    assert_eq!(invert_count, 1);
}

#[test]
fn endpoint_args_multi_drive_emits_slave_and_axis_groups() {
    let args = endpoint_args(
        "eth0",
        "/tmp/x.sock",
        250,
        None,
        None,
        &[
            (0, 0, 1000.0, 50.0, None, None, false, 30.0, false, None),
            (
                1,
                2,
                2000.0,
                40.0,
                Some(4096),
                None,
                false,
                30.0,
                false,
                None,
            ),
        ],
    );
    let slave_positions: Vec<&String> = args
        .iter()
        .enumerate()
        .filter(|(i, a)| *a == "--slave" && args.get(i + 1).is_some())
        .map(|(i, _)| &args[i + 1])
        .collect();
    assert_eq!(slave_positions, vec!["0", "1"]);
    let axes: Vec<&String> = args
        .iter()
        .enumerate()
        .filter(|(i, a)| *a == "--axis" && args.get(i + 1).is_some())
        .map(|(i, _)| &args[i + 1])
        .collect();
    assert_eq!(axes, vec!["0", "2"]);
}

#[test]
fn endpoint_args_emits_per_slave_dynamics_profile() {
    let args = endpoint_args(
        "eth0",
        "/tmp/x.sock",
        250,
        None,
        None,
        &[
            (
                0,
                0,
                1000.0,
                50.0,
                None,
                None,
                false,
                30.0,
                false,
                Some("/cfg/x.toml".into()),
            ),
            (
                1,
                2,
                2000.0,
                40.0,
                None,
                None,
                false,
                30.0,
                false,
                Some("/cfg/y.toml".into()),
            ),
        ],
    );
    let profiles: Vec<&String> = args
        .iter()
        .enumerate()
        .filter(|(i, a)| *a == "--slave-dynamics-profile" && args.get(i + 1).is_some())
        .map(|(i, _)| &args[i + 1])
        .collect();
    assert_eq!(profiles, vec!["/cfg/x.toml", "/cfg/y.toml"]);
    assert!(!args.iter().any(|a| a == "--dynamics-profile"));
}

#[test]
fn spawn_nonexistent_binary_errors_with_binary_path() {
    let result = spawn_ethercat_endpoint(
        "/nonexistent/binary/kalico-ec",
        "eth0",
        "/tmp/test.sock",
        250,
        None,
        None,
        &[],
    );
    assert!(result.is_err(), "expected Err for nonexistent binary");
    let msg = result.unwrap_err();
    assert!(
        msg.contains("/nonexistent/binary/kalico-ec"),
        "error message should contain the binary path; got: {msg}"
    );
    assert!(
        msg.contains("spawn"),
        "error message should indicate a spawn failure; got: {msg}"
    );
}

#[test]
fn poll_socket_ready_detects_early_child_death() {
    let mut child = std::process::Command::new("sh")
        .args(["-c", "exit 3"])
        .spawn()
        .expect("sh must be available");

    let waited = {
        let start = Instant::now();
        loop {
            if child.try_wait().unwrap().is_some() {
                break start.elapsed();
            }
            std::thread::sleep(Duration::from_millis(5));
            if start.elapsed() > Duration::from_secs(2) {
                panic!("child did not exit within 2 s");
            }
        }
    };
    let _ = waited;

    let socket_path = "/tmp/kalico_test_socket_that_will_never_exist_a1b2c3d4";
    let deadline = Instant::now() + Duration::from_secs(30);
    let start = Instant::now();
    let result = poll_socket_ready(socket_path, deadline, &mut child);
    let elapsed = start.elapsed();

    assert!(result.is_err(), "expected Err on early child death");
    let msg = result.unwrap_err();
    assert!(
        msg.contains("exit") || msg.contains("exited"),
        "error message should mention exit status; got: {msg}"
    );
    assert!(
        elapsed < Duration::from_secs(5),
        "poll_socket_ready should return promptly on child death, not burn the deadline; \
         elapsed={elapsed:?}"
    );
}

fn encode_claim_handshake_reply(correlation_id: u32) -> Vec<u8> {
    use mcu_protocol::codec::Encode as _;
    use mcu_protocol::messages::{ClaimHandshakeReply, MessageKind, SlaveState, SlaveStatus};
    use mcu_transport::frame::{CHANNEL_CONTROL, encode_frame};
    use mcu_transport::wire_helpers::{MESSAGE_VERSION_DEFAULT, encode_message_header};

    let reply = ClaimHandshakeReply {
        slave_statuses: vec![SlaveStatus {
            slave_idx: 0,
            state: SlaveState::Ok,
            fault_code: 0,
        }],
    };
    let mut payload = encode_message_header(
        MessageKind::ClaimHandshakeReply,
        MESSAGE_VERSION_DEFAULT,
        correlation_id,
    )
    .to_vec();
    reply.encode(&mut payload);
    encode_frame(CHANNEL_CONTROL, &payload)
}

fn extract_correlation_id(buf: &[u8]) -> u32 {
    use mcu_transport::demux::{Demuxer, Frame};
    use mcu_transport::wire_helpers::decode_message_header;

    let mut demux = Demuxer::new();
    let (frames, _) = demux.feed_slice(buf);
    for f in frames {
        if let Frame::Kalico { payload, .. } = f {
            if let Some((hdr, _)) = decode_message_header(&payload) {
                return hdr.correlation_id;
            }
        }
    }
    0
}

#[test]
fn handshake_retries_past_stale_socket_file() {
    use std::os::unix::net::UnixListener;

    let path = format!(
        "/tmp/kalico_test_stale_{}_handshake.sock",
        std::process::id()
    );
    let _ = std::fs::remove_file(&path);

    {
        let _listener = UnixListener::bind(&path)
            .unwrap_or_else(|e| panic!("bind for stale-file setup failed: {e}"));
    }
    assert!(
        std::path::Path::new(&path).exists(),
        "UnixListener drop must leave the socket file — test precondition violated"
    );

    let (tx, rx) = std::sync::mpsc::channel::<()>();
    let path_bg = path.clone();
    let bg = std::thread::spawn(move || {
        let _ = std::fs::remove_file(&path_bg);
        let listener = UnixListener::bind(&path_bg)
            .unwrap_or_else(|e| panic!("background listener bind failed: {e}"));
        let _ = tx.send(());
        if let Ok((mut stream, _)) = listener.accept() {
            let mut buf = [0u8; 4096];
            if let Ok(n) = stream.read(&mut buf) {
                let cid = extract_correlation_id(&buf[..n]);
                let reply = encode_claim_handshake_reply(cid);
                let _ = stream.write_all(&reply);
                let _ = stream.shutdown(std::net::Shutdown::Write);
                let _ = stream.read(&mut buf);
            }
        }
    });

    rx.recv_timeout(Duration::from_secs(5))
        .expect("background listener must signal within 5 s");

    let deadline = Instant::now() + Duration::from_secs(5);
    let result = handshake_ethercat_endpoint(&path, deadline);
    let _ = std::fs::remove_file(&path);

    let succeeded = result.is_ok();
    drop(result);
    let _ = bg.join();

    assert!(succeeded, "handshake must succeed once listener is up");
}

#[test]
fn handshake_connect_refused_is_not_immediately_fatal() {
    use std::os::unix::net::UnixListener;
    use std::sync::Arc;
    use std::sync::atomic::{AtomicBool, Ordering};

    let path = format!(
        "/tmp/kalico_test_refused_{}_handshake.sock",
        std::process::id()
    );
    let _ = std::fs::remove_file(&path);

    {
        let _l = UnixListener::bind(&path).unwrap_or_else(|e| panic!("bind failed: {e}"));
    }

    let tried = Arc::new(AtomicBool::new(false));
    let tried_bg = Arc::clone(&tried);

    let (stop_tx, stop_rx) = std::sync::mpsc::channel::<()>();

    let path_hs = path.clone();
    let hs = std::thread::spawn(move || {
        tried_bg.store(true, Ordering::SeqCst);
        let deadline = Instant::now() + Duration::from_secs(4);
        handshake_ethercat_endpoint(&path_hs, deadline)
    });

    while !tried.load(Ordering::SeqCst) {
        std::thread::yield_now();
    }
    std::thread::sleep(Duration::from_millis(100));

    let _ = std::fs::remove_file(&path);
    let listener =
        UnixListener::bind(&path).unwrap_or_else(|e| panic!("late listener bind failed: {e}"));

    let path_lt = path.clone();
    let lt = std::thread::spawn(move || {
        let _ = stop_rx; // keep channel alive
        if let Ok((mut stream, _)) = listener.accept() {
            let mut buf = [0u8; 4096];
            if let Ok(n) = stream.read(&mut buf) {
                let cid = extract_correlation_id(&buf[..n]);
                let _ = stream.write_all(&encode_claim_handshake_reply(cid));
                let _ = stream.shutdown(std::net::Shutdown::Write);
                let _ = stream.read(&mut buf);
            }
        }
        let _ = std::fs::remove_file(&path_lt);
    });

    let result = hs.join().expect("handshake thread must not panic");

    let error_msg = match &result {
        Ok(_) => None,
        Err(e) => Some(format!("{e:?}")),
    };

    let _ = stop_tx.send(());
    drop(result);
    let _ = std::os::unix::net::UnixStream::connect(&path);
    let _ = lt.join();

    if let Some(msg) = error_msg {
        assert!(
            !msg.to_ascii_lowercase().contains("connection refused"),
            "handshake must retry past ConnectionRefused, not fail immediately; got: {msg}"
        );
    }
}
