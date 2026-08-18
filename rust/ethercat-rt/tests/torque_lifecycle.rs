use std::process::{Child, Command};
use std::thread;
use std::time::{Duration, Instant};

use ethercat_rt::stream_halt::{ERR_PIECES_WHILE_HALTED, ERR_RESUME_STREAM_NOT_HALTED};
use ethercat_rt::torque::{ERR_BAD_TORQUE_STATE, ERR_PIECES_WHILE_FAULTED};
use host_rt::mcu_call::McuCall;
use host_rt::mcu_serial_conn::McuSerialConn;
use mcu_protocol::codec::{Cursor, Decode, Encode};
use mcu_protocol::messages::{
    ClaimHandshakeReply, DriveLimitEntry, LaneRun, MessageKind, PushSampleRuns,
    PushSampleRunsResponse, RestoreDriveLimits, RestoreDriveLimitsResponse, ResumeStreamResponse,
    SampleGridResponse, SetDriveLimits, SetDriveLimitsResponse, SetTorque, SetTorqueResponse,
    SetpointSample, StopResponse, LANE_RUN_FLAG_REANCHOR, LANE_RUN_FLAG_TAIL,
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
    format!("/tmp/kalico-tq-{}-{}.sock", tag, std::process::id())
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

fn wait_for_exit(child: &mut Child, deadline: Instant) -> std::process::ExitStatus {
    loop {
        match child.try_wait().expect("try_wait must not fail") {
            Some(status) => return status,
            None => {
                assert!(
                    Instant::now() < deadline,
                    "stub process did not exit within deadline — orphan process"
                );
                thread::sleep(Duration::from_millis(10));
            }
        }
    }
}

fn send_stop(conn: &McuSerialConn) -> (i32, u64) {
    let (kind, resp) = conn
        .mcu_call(MessageKind::Stop, Vec::new(), Duration::from_secs(5))
        .expect("Stop call must succeed");
    assert_eq!(
        kind,
        MessageKind::StopResponse,
        "expected StopResponse, got 0x{:04x}",
        kind.as_u16()
    );
    let r = StopResponse::decode(&resp).expect("StopResponse must decode");
    (r.result, r.discard_clock)
}

fn send_resume_stream(conn: &McuSerialConn) -> i32 {
    let (kind, resp) = conn
        .mcu_call(
            MessageKind::ResumeStream,
            Vec::new(),
            Duration::from_secs(5),
        )
        .expect("ResumeStream call must succeed");
    assert_eq!(
        kind,
        MessageKind::ResumeStreamResponse,
        "expected ResumeStreamResponse, got 0x{:04x}",
        kind.as_u16()
    );
    ResumeStreamResponse::decode(&resp)
        .expect("ResumeStreamResponse must decode")
        .result
}

fn set_torque(conn: &McuSerialConn, value: bool, execute_at_ns: u64) -> i32 {
    let body = SetTorque {
        value: u8::from(value),
        execute_at_ns,
    }
    .encoded_to_vec();
    let (kind, resp) = conn
        .mcu_call(MessageKind::SetTorque, body, Duration::from_secs(5))
        .expect("SetTorque call must succeed");
    assert_eq!(
        kind,
        MessageKind::SetTorqueResponse,
        "expected SetTorqueResponse, got 0x{:04x}",
        kind.as_u16()
    );
    SetTorqueResponse::decode(&resp)
        .expect("SetTorqueResponse must decode")
        .result
}

fn now_ns() -> u64 {
    ethercat_rt::clock::monotonic_ns()
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

fn spawn_and_claim(tag: &str, extra_args: &[&str]) -> (ChildGuard, McuSerialConn, String) {
    let path = socket_path(tag);
    let _ = std::fs::remove_file(&path);

    let child = Command::new(STUB_BIN)
        .args(["--socket", &path])
        .args(extra_args)
        .spawn()
        .expect("stub binary must spawn");
    let guard = ChildGuard::new(child);

    wait_for_socket(&path, Instant::now() + Duration::from_secs(5));

    let conn = McuSerialConn::connect(&path).expect("McuSerialConn::connect must succeed");
    let _reply = do_handshake(&conn);

    (guard, conn, path)
}

#[test]
fn enable_acks_disable_schedules_and_parks() {
    let (mut guard, conn, path) = spawn_and_claim("tq-parks", &[]);

    let result = set_torque(&conn, true, now_ns() + 50_000_000);
    assert_eq!(result, 0, "enable must return 0, got {result}");

    let disable_at = now_ns() + 200_000_000;
    let result = set_torque(&conn, false, disable_at);
    assert_eq!(
        result, 0,
        "scheduled disable must return 0 immediately, got {result}"
    );

    thread::sleep(Duration::from_millis(400));

    let result = set_torque(&conn, true, now_ns() + 50_000_000);
    assert_eq!(
        result, 0,
        "re-enable after scheduled disable executed must return 0 (gate Parked), got {result}"
    );

    drop(conn);
    let _ = guard.defuse().wait();
    let _ = std::fs::remove_file(&path);
}

#[test]
fn double_enable_rejects_and_exits() {
    let (mut guard, conn, path) = spawn_and_claim("tq-dbl-en", &[]);

    let r1 = set_torque(&conn, true, now_ns() + 50_000_000);
    assert_eq!(r1, 0, "first enable must return 0, got {r1}");

    let r2 = set_torque(&conn, true, now_ns() + 50_000_000);
    assert_eq!(r2, -312, "double enable must return -312, got {r2}");

    let mut child = guard.defuse();
    wait_for_exit(&mut child, Instant::now() + Duration::from_secs(4));
    let _ = std::fs::remove_file(&path);
}

#[test]
fn disable_in_past_executes_immediately() {
    let (mut guard, conn, path) = spawn_and_claim("tq-past", &[]);

    let r1 = set_torque(&conn, true, now_ns() + 50_000_000);
    assert_eq!(r1, 0, "enable must return 0, got {r1}");

    let r2 = set_torque(&conn, false, 1);
    assert_eq!(r2, 0, "disable with past not-before must ack 0, got {r2}");

    thread::sleep(Duration::from_millis(200));

    let r3 = set_torque(&conn, true, now_ns() + 50_000_000);
    assert_eq!(
        r3, 0,
        "re-enable after past disable executed must return 0 (gate Parked), got {r3}"
    );

    drop(conn);
    let _ = guard.defuse().wait();
    let _ = std::fs::remove_file(&path);
}

#[test]
fn reenable_with_pending_disable_cancels_it() {
    let (mut guard, conn, path) = spawn_and_claim("tq-cancel", &[]);

    let r1 = set_torque(&conn, true, now_ns() + 50_000_000);
    assert_eq!(r1, 0, "initial enable must return 0, got {r1}");

    let cancel_at = now_ns() + 500_000_000;
    let r2 = set_torque(&conn, false, cancel_at);
    assert_eq!(r2, 0, "scheduling disable must return 0, got {r2}");

    let r3 = set_torque(&conn, true, now_ns() + 50_000_000);
    assert_eq!(
        r3, 0,
        "re-enable with pending disable must return 0 (cancel), got {r3}"
    );

    thread::sleep(Duration::from_millis(700));

    let r4 = set_torque(&conn, true, now_ns() + 50_000_000);
    assert_eq!(
        r4, -312,
        "enable while still Enabled must return -312 (cancelled disable did not fire), got {r4}"
    );

    let mut child = guard.defuse();
    wait_for_exit(&mut child, Instant::now() + Duration::from_secs(4));
    let _ = std::fs::remove_file(&path);
}

#[test]
fn sample_runs_while_parked_fault_exits() {
    let (mut guard, conn, path) = spawn_and_claim("tq-pcs-park", &[]);

    push_one_run(&conn);

    let mut child = guard.defuse();
    wait_for_exit(&mut child, Instant::now() + Duration::from_secs(5));
    let _ = std::fs::remove_file(&path);
}

#[test]
fn fail_enable_flag_returns_310_and_exits() {
    let (mut guard, conn, path) = spawn_and_claim("tq-fail-en", &["--fail-enable"]);

    let result = set_torque(&conn, true, now_ns() + 50_000_000);
    assert_eq!(result, -310, "--fail-enable must return -310, got {result}");

    let mut child = guard.defuse();
    wait_for_exit(&mut child, Instant::now() + Duration::from_secs(4));
    let _ = std::fs::remove_file(&path);
}

#[test]
fn stop_while_parked_succeeds_and_keeps_session() {
    let (mut guard, conn, path) = spawn_and_claim("stop-parked", &[]);

    let t0 = now_ns();
    let (result, discard_clock) = send_stop(&conn);
    let t1 = now_ns();
    assert_eq!(result, 0, "Stop while parked must return 0, got {result}");
    assert!(
        discard_clock >= t0 && discard_clock <= t1,
        "discard_clock {discard_clock} outside [{t0}, {t1}]"
    );

    let r = set_torque(&conn, true, now_ns() + 50_000_000);
    assert_eq!(
        r, 0,
        "enable after Stop must return 0 (session alive), got {r}"
    );

    drop(conn);
    let _ = guard.defuse().wait();
    let _ = std::fs::remove_file(&path);
}

fn set_drive_limits(conn: &McuSerialConn, counts: u32, tenth_pct: u16) -> i32 {
    let body = SetDriveLimits {
        drives: vec![DriveLimitEntry {
            slot: 0,
            following_error_counts: counts,
            max_torque_tenth_pct: tenth_pct,
        }],
    }
    .encoded_to_vec();
    let (kind, resp) = conn
        .mcu_call(MessageKind::SetDriveLimits, body, Duration::from_secs(5))
        .expect("SetDriveLimits call must succeed");
    assert_eq!(kind, MessageKind::SetDriveLimitsResponse);
    SetDriveLimitsResponse::decode(&resp)
        .expect("decode")
        .result
}

fn restore_drive_limits(conn: &McuSerialConn) -> i32 {
    let (kind, resp) = conn
        .mcu_call(
            MessageKind::RestoreDriveLimits,
            RestoreDriveLimits { slot_mask: 1 }.encoded_to_vec(),
            Duration::from_secs(5),
        )
        .expect("RestoreDriveLimits call must succeed");
    assert_eq!(kind, MessageKind::RestoreDriveLimitsResponse);
    RestoreDriveLimitsResponse::decode(&resp)
        .expect("decode")
        .result
}

#[test]
fn drive_limits_set_and_restore_round_trip() {
    let (mut guard, conn, path) = spawn_and_claim("limits-rt", &[]);
    assert_eq!(set_drive_limits(&conn, 8192, 500), 0);
    assert_eq!(restore_drive_limits(&conn), 0);
    assert_eq!(set_drive_limits(&conn, 4096, 300), 0);
    drop(conn);
    let _ = guard.defuse().wait();
    let _ = std::fs::remove_file(&path);
}

#[test]
fn simulated_drive_fault_parks_keeps_serving_and_recovers() {
    let (mut guard, conn, path) =
        spawn_and_claim("drive-fault", &["--drive-fault-after-cycles", "1"]);

    let r = set_torque(&conn, true, now_ns() + 50_000_000);
    assert_eq!(r, 0);
    push_one_run(&conn);

    let fault_deadline = Instant::now() + Duration::from_secs(5);
    loop {
        assert!(
            Instant::now() < fault_deadline,
            "stub never simulated the drive fault"
        );
        let result = push_one_run(&conn);
        match result {
            0 => thread::sleep(Duration::from_millis(20)),
            ERR_PIECES_WHILE_FAULTED => break,
            other => panic!("unexpected PushSampleRuns result while polling for fault: {other}"),
        }
    }

    let r = set_torque(&conn, true, now_ns() + 50_000_000);
    assert_eq!(
        r, 0,
        "enable from Faulted must run the ladder and return 0, got {r}"
    );

    drop(conn);
    let _ = guard.defuse().wait();
    let _ = std::fs::remove_file(&path);
}

#[test]
fn stop_halts_stream_until_resume() {
    let (mut guard, conn, path) = spawn_and_claim("halt-resume", &[]);

    let r = set_torque(&conn, true, now_ns() + 50_000_000);
    assert_eq!(r, 0, "enable must return 0, got {r}");

    let r = push_one_run(&conn);
    assert_eq!(r, 0, "push before Stop must be accepted, got {r}");

    let (result, _clock) = send_stop(&conn);
    assert_eq!(result, 0, "Stop must return 0, got {result}");

    let r = push_one_run(&conn);
    assert_eq!(
        r, ERR_PIECES_WHILE_HALTED,
        "push while halted must be rejected, got {r}"
    );

    let r = send_resume_stream(&conn);
    assert_eq!(r, 0, "ResumeStream after Stop must return 0, got {r}");

    let r = push_one_run(&conn);
    assert_eq!(r, 0, "push after ResumeStream must be accepted, got {r}");

    drop(conn);
    let _ = guard.defuse().wait();
    let _ = std::fs::remove_file(&path);
}

#[test]
fn resume_stream_without_halt_is_rejected() {
    let (mut guard, conn, path) = spawn_and_claim("resume-nohalt", &[]);

    let r = send_resume_stream(&conn);
    assert_eq!(
        r, ERR_RESUME_STREAM_NOT_HALTED,
        "ResumeStream on an open stream must be rejected, got {r}"
    );

    drop(conn);
    let _ = guard.defuse().wait();
    let _ = std::fs::remove_file(&path);
}

#[test]
fn stop_discards_queued_setpoints_and_keeps_torque() {
    let (mut guard, conn, path) = spawn_and_claim("stop-discard", &[]);

    let r = set_torque(&conn, true, now_ns() + 50_000_000);
    assert_eq!(r, 0, "enable must return 0, got {r}");

    push_one_run(&conn);

    let (result, _clock) = send_stop(&conn);
    assert_eq!(result, 0, "Stop mid-stream must return 0, got {result}");

    thread::sleep(Duration::from_millis(400));

    let r = set_torque(&conn, true, now_ns() + 50_000_000);
    assert_eq!(
        r, ERR_BAD_TORQUE_STATE,
        "re-enable after Stop must reject as already-enabled — Stop must leave \
         torque on so the homing retract that follows it can move, got {r}"
    );

    drop(conn);
    let _ = guard.defuse().wait();
    let _ = std::fs::remove_file(&path);
}

#[test]
fn disable_while_parked_is_idempotent() {
    let (mut guard, conn, path) = spawn_and_claim("disable-parked", &[]);

    let r = set_torque(&conn, false, 0);
    assert_eq!(
        r, 0,
        "disable with torque already off must be a no-op, got {r}"
    );

    thread::sleep(Duration::from_millis(200));

    let r = set_torque(&conn, true, now_ns() + 50_000_000);
    assert_eq!(
        r, 0,
        "enable after idempotent disable must return 0, got {r}"
    );

    drop(conn);
    let _ = guard.defuse().wait();
    let _ = std::fs::remove_file(&path);
}
