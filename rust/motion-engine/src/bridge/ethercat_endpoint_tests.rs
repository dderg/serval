use super::{
    EndpointClaimError, EthercatDrive, endpoint_args, handshake_ethercat_endpoint,
    poll_socket_ready, slots_for_axis, spawn_ethercat_endpoint, verify_sample_grid,
};
use std::io::{Read, Write};
use std::time::{Duration, Instant};

/// A drive with unremarkable defaults; each test overrides only the fields it
/// exercises via struct-update syntax.
fn drive() -> EthercatDrive {
    EthercatDrive {
        chain_index: 0,
        axis: 0,
        counts_per_mm: 1000.0,
        rotation_distance: 40.0,
        following_error_counts: None,
        max_torque_tenth_pct: None,
        velocity_ff: false,
        ff_max_torque: 30.0,
        invert_direction: false,
        dynamics_profile: None,
    }
}

#[test]
fn slots_for_axis_maps_hits_and_misses() {
    let slot_axes = [2usize, 5, 7];
    assert_eq!(slots_for_axis(&slot_axes, 2), vec![0]);
    assert_eq!(slots_for_axis(&slot_axes, 5), vec![1]);
    assert_eq!(slots_for_axis(&slot_axes, 7), vec![2]);
    assert_eq!(slots_for_axis(&slot_axes, 0), Vec::<u8>::new());
    assert_eq!(slots_for_axis(&[], 0), Vec::<u8>::new());
}

#[test]
fn slots_for_axis_returns_every_awd_slot_in_order() {
    let slot_axes = [0usize, 0, 1, 1];
    assert_eq!(slots_for_axis(&slot_axes, 0), vec![0, 1]);
    assert_eq!(slots_for_axis(&slot_axes, 1), vec![2, 3]);
}

#[test]
fn endpoint_args_single_drive_uses_legacy_form() {
    let args = endpoint_args(
        "eth0",
        "/tmp/x.sock",
        250,
        None,
        None,
        250.0,
        None,
        &[EthercatDrive {
            chain_index: 1,
            counts_per_mm: 3276.8,
            following_error_counts: Some(8192),
            max_torque_tenth_pct: Some(500),
            ..drive()
        }],
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
        250.0,
        None,
        &[
            EthercatDrive {
                rotation_distance: 50.0,
                velocity_ff: true,
                ff_max_torque: 25.0,
                ..drive()
            },
            EthercatDrive {
                chain_index: 1,
                axis: 2,
                counts_per_mm: 2000.0,
                ff_max_torque: 60.0,
                invert_direction: true,
                ..drive()
            },
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
        250.0,
        None,
        &[
            EthercatDrive {
                rotation_distance: 50.0,
                ..drive()
            },
            EthercatDrive {
                chain_index: 1,
                axis: 2,
                counts_per_mm: 2000.0,
                following_error_counts: Some(4096),
                ..drive()
            },
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
        250.0,
        None,
        &[
            EthercatDrive {
                rotation_distance: 50.0,
                dynamics_profile: Some("/cfg/x.toml".into()),
                ..drive()
            },
            EthercatDrive {
                chain_index: 1,
                axis: 2,
                counts_per_mm: 2000.0,
                dynamics_profile: Some("/cfg/y.toml".into()),
                ..drive()
            },
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

fn args_for() -> Vec<String> {
    endpoint_args(
        "eth0",
        "/tmp/x.sock",
        250,
        None,
        None,
        250.0,
        None,
        &[drive()],
    )
}

#[test]
fn endpoint_args_emits_the_full_argv_for_a_single_drive() {
    assert_eq!(
        args_for(),
        vec![
            "eth0",
            "--socket",
            "/tmp/x.sock",
            "--cycle-us",
            "250",
            "--group-delay-us",
            "250",
            "--counts-per-mm",
            "1000",
            "--rotation-distance",
            "40",
            "--torque-clamp-pct",
            "30",
        ]
    );
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
        250.0,
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

fn serve_one_grid_reply(
    path: &str,
    reply_of: Option<fn(u32) -> Vec<u8>>,
) -> std::thread::JoinHandle<()> {
    use std::os::unix::net::UnixListener;

    let _ = std::fs::remove_file(path);
    let listener = UnixListener::bind(path).unwrap_or_else(|e| panic!("bind {path}: {e}"));
    std::thread::spawn(move || {
        if let Ok((mut stream, _)) = listener.accept() {
            let mut buf = [0u8; 4096];
            if let Ok(n) = stream.read(&mut buf) {
                if let Some(reply_of) = reply_of {
                    let cid = extract_correlation_id(&buf[..n]);
                    let _ = stream.write_all(&reply_of(cid));
                }
            }
            let _ = stream.read(&mut buf);
        }
    })
}

fn grid_reply_with_zero_depth(cid: u32) -> Vec<u8> {
    ethercat_rt::wire::sample_grid_response_frame(
        cid,
        ethercat_rt::setpoint::EXECUTOR_SETPOINT_RING,
        250_000,
        0,
        (42, 10_500_000),
    )
}

#[test]
fn a_zero_deep_ring_is_refused_at_claim_time() {
    let path = format!("/tmp/kalico_test_zero_depth_{}.sock", std::process::id());
    let server = serve_one_grid_reply(&path, Some(grid_reply_with_zero_depth));

    let conn = host_rt::mcu_serial_conn::McuSerialConn::connect(&path)
        .unwrap_or_else(|e| panic!("connect {path}: {e}"));
    let err = verify_sample_grid(&conn, Instant::now() + Duration::from_secs(5))
        .expect_err("a zero-deep ring must fail the claim");
    drop(conn);
    let _ = server.join();
    let _ = std::fs::remove_file(&path);

    match err {
        EndpointClaimError::Protocol(detail) => assert!(
            detail.contains("zero cycles"),
            "the message must name the empty ring; got: {detail}"
        ),
        other => panic!("expected a protocol error, got {other:?}"),
    }
}

#[test]
fn a_silent_endpoint_reports_a_timeout_not_a_stale_binary() {
    let path = format!("/tmp/kalico_test_grid_silent_{}.sock", std::process::id());
    let server = serve_one_grid_reply(&path, None);

    let conn = host_rt::mcu_serial_conn::McuSerialConn::connect(&path)
        .unwrap_or_else(|e| panic!("connect {path}: {e}"));
    let err = verify_sample_grid(&conn, Instant::now() + Duration::from_millis(200))
        .expect_err("an unanswered QuerySampleGrid must fail the claim");
    drop(conn);
    let _ = server.join();
    let _ = std::fs::remove_file(&path);

    match err {
        EndpointClaimError::Transport { call, cause } => {
            assert_eq!(call, "QuerySampleGrid");
            assert!(
                matches!(cause, host_rt::transport::TransportError::Timeout),
                "expected a timeout, got {cause:?}"
            );
        }
        other => panic!("expected a transport error, got {other:?}"),
    }
}
