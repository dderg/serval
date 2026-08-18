use std::process::{Child, Command};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use ethercat_rt::stream_halt::ERR_PIECES_WHILE_HALTED;
use host_rt::mcu_call::McuCall;
use host_rt::mcu_serial_conn::McuSerialConn;
use mcu_protocol::codec::{Cursor, Decode, Encode};
use mcu_protocol::messages::{
    ArmSensorlessEndstop, ArmSensorlessEndstopResponse, ClaimHandshakeReply, LaneRun, MessageKind,
    PushSampleRuns, PushSampleRunsResponse, ResumeStreamResponse, SampleGridResponse, SdoWrite,
    SdoWriteResponse, SetTorque, SetTorqueResponse, SetpointSample, LANE_RUN_FLAG_REANCHOR,
    LANE_RUN_FLAG_TAIL,
};

const STUB_BIN: &str = env!("CARGO_BIN_EXE_ethercat-rt-stub");
const TORQUE_ACTUAL_INDEX: u16 = 0x6077;

struct ChildGuard {
    child: Option<Child>,
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
    format!("/tmp/kalico-sl-{}-{}.sock", tag, std::process::id())
}

fn wait_for_socket(path: &str, deadline: Instant) {
    while !std::path::Path::new(path).exists() {
        assert!(
            Instant::now() < deadline,
            "stub socket {path:?} did not appear within deadline"
        );
        thread::sleep(Duration::from_millis(10));
    }
}

fn do_handshake(conn: &McuSerialConn) {
    let (kind, body) = conn
        .mcu_call(
            MessageKind::ClaimHandshake,
            Vec::new(),
            Duration::from_secs(5),
        )
        .expect("ClaimHandshake mcu_call must succeed");
    assert_eq!(kind, MessageKind::ClaimHandshakeReply);
    ClaimHandshakeReply::decode_from(&mut Cursor::new(&body))
        .expect("ClaimHandshakeReply must decode");
}

fn arm_sensorless(conn: &McuSerialConn, endstop_id: u8, torque_trip_tenth_pct: u16, enable: bool) {
    let body = ArmSensorlessEndstop {
        slot: 0,
        endstop_id,
        torque_trip_tenth_pct,
        enable: u8::from(enable),
    }
    .encoded_to_vec();
    let (kind, resp) = conn
        .mcu_call(
            MessageKind::ArmSensorlessEndstop,
            body,
            Duration::from_secs(5),
        )
        .expect("ArmSensorlessEndstop call must succeed");
    assert_eq!(kind, MessageKind::ArmSensorlessEndstopResponse);
    let r = ArmSensorlessEndstopResponse::decode(&resp).expect("response decodes");
    assert_eq!(r.result, 0, "arm must be accepted");
}

fn inject_torque(conn: &McuSerialConn, value: i64) {
    let body = SdoWrite {
        slot: 0,
        index: TORQUE_ACTUAL_INDEX,
        subindex: 0,
        size: 2,
        value,
    }
    .encoded_to_vec();
    let (kind, resp) = conn
        .mcu_call(MessageKind::SdoWrite, body, Duration::from_secs(5))
        .expect("SdoWrite call must succeed");
    assert_eq!(kind, MessageKind::SdoWriteResponse);
    let r = SdoWriteResponse::decode(&resp).expect("response decodes");
    assert_eq!(r.result, 0, "torque injection write must succeed");
}

fn spawn_stub(tag: &str) -> (ChildGuard, McuSerialConn) {
    let socket = socket_path(tag);
    let _ = std::fs::remove_file(&socket);
    let child = Command::new(STUB_BIN)
        .args(["--socket", &socket])
        .spawn()
        .expect("spawn ethercat-rt-stub");
    let guard = ChildGuard { child: Some(child) };
    wait_for_socket(&socket, Instant::now() + Duration::from_secs(5));
    let conn = McuSerialConn::connect(&socket).expect("connect to stub socket");
    do_handshake(&conn);
    (guard, conn)
}

/// One anchored, final sample run a few cycles ahead of the endpoint's live
/// grid index — the smallest thing the pump can put in the ring.
fn push_one_run(conn: &McuSerialConn) -> i32 {
    let (kind, resp) = conn
        .mcu_call(
            MessageKind::QuerySampleGrid,
            Vec::new(),
            Duration::from_secs(5),
        )
        .expect("QuerySampleGrid call must succeed");
    assert_eq!(kind, MessageKind::SampleGridResponse);
    let grid = SampleGridResponse::decode(&resp).expect("SampleGridResponse must decode");
    let lane = LaneRun {
        axis_idx: 0,
        flags: LANE_RUN_FLAG_REANCHOR | LANE_RUN_FLAG_TAIL,
        origin_mm_q16: 0,
        start_index: grid.grid_index + 5,
        interval_ticks: grid.cycle_ticks,
        sample_count: 1,
        samples: vec![SetpointSample {
            pos_counts: 0,
            vel_ff: 0,
            torque_ff: 0,
            acc_mm_s2: 0.0,
        }],
    };
    let body = PushSampleRuns { lanes: vec![lane] }.encoded_to_vec();
    let (_, resp) = conn
        .mcu_call(MessageKind::PushSampleRuns, body, Duration::from_secs(5))
        .expect("PushSampleRuns call must succeed");
    PushSampleRunsResponse::decode(&resp)
        .expect("PushSampleRunsResponse must decode")
        .result
}

fn send_resume_stream(conn: &McuSerialConn) -> i32 {
    let (kind, resp) = conn
        .mcu_call(
            MessageKind::ResumeStream,
            Vec::new(),
            Duration::from_secs(5),
        )
        .expect("ResumeStream call must succeed");
    assert_eq!(kind, MessageKind::ResumeStreamResponse);
    ResumeStreamResponse::decode(&resp)
        .expect("ResumeStreamResponse must decode")
        .result
}

fn enable_torque(conn: &McuSerialConn) {
    let body = SetTorque {
        value: 1,
        execute_at_ns: ethercat_rt::clock::monotonic_ns() + 50_000_000,
    }
    .encoded_to_vec();
    let (kind, resp) = conn
        .mcu_call(MessageKind::SetTorque, body, Duration::from_secs(5))
        .expect("SetTorque call must succeed");
    assert_eq!(kind, MessageKind::SetTorqueResponse);
    let r = SetTorqueResponse::decode(&resp)
        .expect("SetTorqueResponse must decode")
        .result;
    assert_eq!(r, 0, "torque enable must return 0, got {r}");
}

#[test]
fn trip_halts_stream_until_resume() {
    let (_guard, conn) = spawn_stub("trip-halt");
    enable_torque(&conn);

    let fired = Arc::new(AtomicBool::new(false));
    let fired_w = Arc::clone(&fired);
    conn.attach_endstop_trip_callback(Arc::new(move |_endstop_id: u8, _trip_clock: u64| {
        fired_w.store(true, Ordering::SeqCst);
    }));

    let r = push_one_run(&conn);
    assert_eq!(r, 0, "push before the trip must be accepted, got {r}");

    arm_sensorless(&conn, 4, 500, true);
    inject_torque(&conn, 600);

    let deadline = Instant::now() + Duration::from_secs(3);
    while !fired.load(Ordering::SeqCst) {
        assert!(
            Instant::now() < deadline,
            "armed torque cross did not produce an EndstopTrip"
        );
        thread::sleep(Duration::from_millis(2));
    }

    let r = push_one_run(&conn);
    assert_eq!(
        r, ERR_PIECES_WHILE_HALTED,
        "push after the trip must be rejected until ResumeStream, got {r}"
    );

    let r = send_resume_stream(&conn);
    assert_eq!(r, 0, "ResumeStream after the trip must return 0, got {r}");

    let r = push_one_run(&conn);
    assert_eq!(r, 0, "push after ResumeStream must be accepted, got {r}");
}

#[test]
fn armed_torque_cross_emits_endstop_trip() {
    let (_guard, conn) = spawn_stub("trip");

    let fired = Arc::new(AtomicBool::new(false));
    let trip_id = Arc::new(AtomicU64::new(u64::MAX));
    let fired_w = Arc::clone(&fired);
    let trip_id_w = Arc::clone(&trip_id);
    conn.attach_endstop_trip_callback(Arc::new(move |endstop_id: u8, _trip_clock: u64| {
        trip_id_w.store(u64::from(endstop_id), Ordering::SeqCst);
        fired_w.store(true, Ordering::SeqCst);
    }));

    arm_sensorless(&conn, 4, 500, true);
    inject_torque(&conn, 600);

    let deadline = Instant::now() + Duration::from_secs(3);
    while !fired.load(Ordering::SeqCst) {
        assert!(
            Instant::now() < deadline,
            "armed torque cross did not produce an EndstopTrip"
        );
        thread::sleep(Duration::from_millis(2));
    }
    assert_eq!(
        trip_id.load(Ordering::SeqCst),
        4,
        "trip carries the armed endstop_id"
    );
}

#[test]
fn below_threshold_does_not_trip_and_disarm_silences() {
    let (_guard, conn) = spawn_stub("quiet");

    let fired = Arc::new(AtomicBool::new(false));
    let fired_w = Arc::clone(&fired);
    conn.attach_endstop_trip_callback(Arc::new(move |_endstop_id: u8, _trip_clock: u64| {
        fired_w.store(true, Ordering::SeqCst);
    }));

    arm_sensorless(&conn, 5, 500, true);
    inject_torque(&conn, 200);
    thread::sleep(Duration::from_millis(50));
    assert!(
        !fired.load(Ordering::SeqCst),
        "below-threshold torque must not trip"
    );

    arm_sensorless(&conn, 5, 0, false);
    inject_torque(&conn, 5_000);
    thread::sleep(Duration::from_millis(50));
    assert!(
        !fired.load(Ordering::SeqCst),
        "a disarmed endstop must stay silent"
    );
}
