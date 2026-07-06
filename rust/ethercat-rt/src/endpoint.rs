#![allow(unsafe_code)]

use std::ffi::CString;
use std::ops::ControlFlow;
use std::sync::atomic::{AtomicBool, Ordering};

use crate::buzz::BuzzOsc;
use crate::capture::{
    any_slot_out_of_range, Capture, CaptureConfig, CaptureDriveConfig, CaptureRecord, DriveSample,
    PendingStart, PendingStop, ERR_CAPTURE_BAD_DRIVE_LIST, FLAG_MOTION_ACTIVE, FLAG_TORQUE_ENABLED,
};
use crate::claim::{
    all_slaves_reply, eval_wkc, single_slave_reply, wait_for_claim, wait_for_claim_pumping,
    WkcDecision,
};
use crate::cli::Args;
use crate::clock::{monotonic_ns, raw_from_monotonic_ns};
use crate::curves::{AxisRing, AXIS_RING_CAPACITY, ENGINE_STATE_FAULT};
use crate::dynamics::{clamp_torque, DynamicsModel};
use crate::ffi;
use crate::mailbox::{MailboxReply, MailboxRequest, MailboxWorker, WorkerScheduling};
use crate::push_plan::plan_bundle;
use crate::scale::{mm_to_counts, CountMap};
use crate::sdo::SdoBus;
use crate::seed_home::ERR_SEED_HOME_STREAMING;
use crate::sensorless::{SensorlessBank, ERR_ARM_SENSORLESS_BAD_THRESHOLD};
use crate::server::FrameServer;
use crate::stream_halt::StreamHalt;
use crate::torque::{
    CommandAction, TickAction, TorqueGate, TorqueState, ERR_ENABLE_FAILED, ERR_PIECES_WHILE_FAULTED,
};
use crate::wire::{
    arm_sensorless_endstop_response_frame, claim_handshake_reply_frame, endstop_trip_frame,
    identify_response_frame, motor_state_empty_frame, motor_state_response_frame_multi,
    push_pieces_response_frame_multi, resonance_buzz_response_frame,
    restore_drive_limits_response_frame, resume_stream_response_frame, runtime_caps_response_frame,
    sdo_read_response_frame, sdo_write_response_frame, seed_servo_home_response_frame,
    set_drive_limits_response_frame, set_torque_response_frame, start_capture_response_frame,
    status_heartbeat_frame, stop_capture_response_frame, stop_response_frame, Command,
};
use mcu_protocol::messages::{
    ArmSensorlessEndstop, PushPieces, ResonanceBuzz, SdoRead, SdoReadResponse, SdoWrite,
    SdoWriteResponse, SetDriveLimits, SetTorque, SlaveState, StartCapture, StopCaptureResponse,
    ERR_SDO_TRANSPORT, ERR_SDO_UNSUPPORTED_SIZE,
};

static SIGTERM_RECEIVED: AtomicBool = AtomicBool::new(false);

/// Below the DC thread (default 80) so the cycle always preempts mailbox
/// work, and below Linux threaded-IRQ handlers (50) so NIC frame delivery
/// preempts the master's receive busy-poll.
const MAILBOX_RT_PRIO: i32 = 40;

/// A commanded target moving faster than this (2 m/s) is physically impossible
/// for these axes — a trajectory discontinuity, the signature the drive latches
/// as Er87.1. Scaled by the cycle time into a per-cycle count bound so it means
/// the same velocity at any DC rate. Log the offending command so the jump is
/// visible the cycle it happens, not inferred.
const TARGET_JUMP_LOG_MM_S: f64 = 2000.0;

extern "C" fn on_sigterm(_: libc::c_int) {
    SIGTERM_RECEIVED.store(true, Ordering::Release);
}

/// Emit each slave's EtherCAT AL state so a failed bringup shows which slave is
/// stuck and where (al_state: 0x01=Init 0x02=PreOp 0x04=SafeOp 0x08=Op,
/// +0x10=error bit; al_code carries the reason, e.g. 0x001b SM watchdog,
/// 0x001a/0x002c/0x0030 DC sync). Routed through the structured pipeline so a
/// flaky multi-restart connect is diagnosable instead of inferred.
fn log_al_states(num_slaves: usize, stage: &str) {
    for s in 0..num_slaves {
        let mut al_state = 0u16;
        let mut al_code = 0u16;
        unsafe { ffi::ec_rt_al_status(s as std::os::raw::c_int, &mut al_state, &mut al_code) };
        tracing::warn!(
            subsystem = "ethercat",
            event = "al_state",
            stage,
            slot = s,
            al_state,
            al_code,
            "slave AL state at bringup stage"
        );
    }
}

struct FfiSdoBus;

fn ffi_sdo_error(abort: u32) -> i32 {
    if abort == 0 {
        return ERR_SDO_TRANSPORT;
    }
    debug_assert!(
        abort < 0x8000_0000,
        "CoE abort code 0x{abort:08x} would collide with local error codes as i32"
    );
    abort as i32
}

impl SdoBus for FfiSdoBus {
    fn read(&mut self, slot: u8, index: u16, subindex: u8) -> Result<(u8, [u8; 4]), i32> {
        let mut buf = [0u8; 8];
        let mut size: std::os::raw::c_int = buf.len() as std::os::raw::c_int;
        let mut abort: u32 = 0;
        let rc = unsafe {
            ffi::ec_rt_sdo_read(
                i32::from(slot),
                index,
                subindex,
                buf.as_mut_ptr(),
                &mut size,
                &mut abort,
            )
        };
        if rc != 0 {
            return Err(ffi_sdo_error(abort));
        }
        if !(1..=4).contains(&size) {
            return Err(ERR_SDO_UNSUPPORTED_SIZE);
        }
        let mut data = [0u8; 4];
        data[..size as usize].copy_from_slice(&buf[..size as usize]);
        Ok((size as u8, data))
    }

    fn write(&mut self, slot: u8, index: u16, subindex: u8, bytes: &[u8]) -> Result<(), i32> {
        let mut abort: u32 = 0;
        let rc = unsafe {
            ffi::ec_rt_sdo_write(
                i32::from(slot),
                index,
                subindex,
                bytes.as_ptr(),
                bytes.len() as std::os::raw::c_int,
                &mut abort,
            )
        };
        if rc != 0 {
            return Err(ffi_sdo_error(abort));
        }
        Ok(())
    }
}

fn bringup_fail(server: &mut FrameServer, rc: i32) -> ! {
    tracing::error!(
        subsystem = "ethercat",
        event = "bringup_fail",
        rc,
        "bringup failed; sending handshake-fail then exiting"
    );
    let claim_deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
    if let Some(cid) = wait_for_claim(server, claim_deadline, &SIGTERM_RECEIVED, "ec-rt") {
        let reply = single_slave_reply(
            1,
            SlaveState::Offline,
            u16::try_from(rc.unsigned_abs()).unwrap_or(u16::MAX),
        );
        server.respond_and_close(&claim_handshake_reply_frame(cid, &reply));
        tracing::error!(
            subsystem = "ethercat",
            event = "bringup_fail_reported",
            rc,
            "sent offline handshake reply, exiting"
        );
    } else {
        tracing::error!(
            subsystem = "ethercat",
            event = "claim_handshake_timeout",
            "bridge did not send ClaimHandshake within 5 s; aborting"
        );
    }
    std::process::exit(1);
}

pub struct EndpointCtx {
    server: FrameServer,

    num_slaves: usize,
    counts_per_mm: Vec<f64>,
    invert: Vec<bool>,
    cmd_counts_per_mm: Vec<f64>,
    rotation_distance: Vec<f64>,
    slave_axes: Vec<u8>,
    velocity_ff: Vec<bool>,
    torque_clamp_tenths: Vec<i16>,
    ff_lead_ns: Vec<u64>,
    jump_log_counts: Vec<i64>,
    cycle_ns: i64,
    telemetry_period: u64,
    dynamics: Option<DynamicsModel>,
    run_limits: Vec<(u32, u16)>,

    rings: Vec<AxisRing>,
    buzz: BuzzOsc,
    cmaps: Vec<Option<CountMap>>,
    last_counts: Vec<Option<i32>>,
    report_anchor: Vec<Option<(i32, f64)>>,
    last_streamed_target: Vec<Option<i32>>,
    last_sent_retired: u32,
    heartbeat_sent: bool,
    gate: TorqueGate,
    capture: Capture,
    cycle_index: u64,
    mailbox: MailboxWorker,
    pending_starts: Vec<(u32, String, PendingStart)>,
    pending_stops: Vec<(u32, PendingStop)>,
    capture_slots: Vec<u8>,
    prdiv: u64,
    ff_saturation: u32,
    wkc_consecutive: u8,
    latched_drive_err: u16,
    sensorless: SensorlessBank,
    stream_halt: StreamHalt,
}

pub fn bringup(args: Args) -> EndpointCtx {
    let Args {
        ifname,
        socket,
        cycle_us,
        slaves,
        rt_cpu,
        rt_prio,
        mailbox_cpu,
        dynamics,
    } = args;

    let num_slaves = slaves.len();
    let counts_per_mm: Vec<f64> = slaves.iter().map(|s| s.counts_per_mm).collect();
    let invert: Vec<bool> = slaves.iter().map(|s| s.invert).collect();
    let cmd_counts_per_mm: Vec<f64> = slaves
        .iter()
        .map(|s| {
            if s.invert {
                -s.counts_per_mm
            } else {
                s.counts_per_mm
            }
        })
        .collect();
    let rotation_distance: Vec<f64> = slaves.iter().map(|s| s.rotation_distance).collect();
    let slave_positions: Vec<i32> = slaves.iter().map(|s| s.pos).collect();
    let slave_axes: Vec<u8> = slaves.iter().map(|s| s.axis).collect();
    let velocity_ff: Vec<bool> = slaves.iter().map(|s| s.velocity_ff).collect();
    let torque_clamp_tenths: Vec<i16> = slaves.iter().map(|s| s.torque_clamp_tenths).collect();
    let ff_lead_ns: Vec<u64> = slaves
        .iter()
        .map(|s| u64::from(s.ff_lead_cycles) * (cycle_us as u64) * 1000)
        .collect();

    let cycle_ns = cycle_us * 1000;
    let telemetry_period = u64::try_from(cycle_us)
        .map(|u| (500_000u64 / u).max(1))
        .unwrap_or(500);

    let rings: Vec<AxisRing> = (0..num_slaves).map(AxisRing::with_slot).collect();
    let buzz = BuzzOsc::new();
    let cmaps: Vec<Option<CountMap>> = (0..num_slaves).map(|_| None).collect();
    let last_counts: Vec<Option<i32>> = vec![None; num_slaves];
    let jump_log_mm = TARGET_JUMP_LOG_MM_S * cycle_us as f64 / 1_000_000.0;
    let jump_log_counts: Vec<i64> = cmd_counts_per_mm
        .iter()
        .map(|c| (c.abs() * jump_log_mm).round() as i64)
        .collect();
    // Per-slot report frame: (counts, host mm) captured at the homing finalize.
    // Maps the drive's raw encoder counts into the host frame for
    // QueryMotorState — the drive's own coordinate frame is never touched,
    // exactly like a stepper's step counter. The counts side of the pair is the
    // last COMMANDED target, not position_actual: at finalize the ring is empty
    // (pieces retired by time) but the servo may still be settling several mm
    // behind, and anchoring against a lagging actual bakes that transient in as
    // a permanent report offset.
    let report_anchor: Vec<Option<(i32, f64)>> = vec![None; num_slaves];
    let last_streamed_target: Vec<Option<i32>> = vec![None; num_slaves];
    let last_sent_retired: u32 = 0;
    let heartbeat_sent = false;

    let mut server = FrameServer::bind(&socket).expect("bind socket");
    tracing::info!(
        subsystem = "ethercat",
        event = "bringup_start",
        num_slaves,
        cycle_us,
        positions = format!("{slave_positions:?}"),
        counts_per_mm = format!("{counts_per_mm:?}"),
        invert = format!("{invert:?}"),
        velocity_ff = format!("{velocity_ff:?}"),
        dynamics = dynamics.is_some(),
        "endpoint starting bringup"
    );

    unsafe {
        let mut sa: libc::sigaction = std::mem::zeroed();
        sa.sa_sigaction = on_sigterm as *const () as libc::sighandler_t;
        sa.sa_flags = libc::SA_RESTART;
        libc::sigaction(libc::SIGTERM, &sa, std::ptr::null_mut());
    }

    let cif = CString::new(ifname.clone()).expect("ifname must not contain NUL");

    // Single-shot bring-up: sync_slave_clocks converges a non-reference drive's
    // DC clock from any starting offset while the kernel master's FSM keeps
    // retrying the OP transition, and ec_rt_bringup_finish gates on the measured
    // per-slave clock offset (ESC 0x092C) before CiA-402. Releasing the master
    // and retrying would only discard convergence progress and re-roll a fresh
    // random offset, so a failure here is terminal and reported with per-slot
    // AL state and DC offset.
    let run_limits: Vec<(u32, u16)> = {
        let rc = unsafe {
            ffi::ec_rt_bringup_preop(
                cif.as_ptr(),
                cycle_ns,
                rt_cpu,
                rt_prio,
                slave_positions.as_ptr(),
                num_slaves as std::os::raw::c_int,
            )
        };
        tracing::info!(
            subsystem = "ethercat",
            event = "bringup_preop",
            rc,
            num_slaves,
            "PREOP bringup result"
        );
        if rc != 0 {
            log_al_states(num_slaves, "preop_fail");
            unsafe { ffi::ec_rt_shutdown() };
            bringup_fail(&mut server, rc);
        }

        let limits: Result<Vec<(u32, u16)>, i32> = (0..num_slaves)
            .map(|s| {
                let slot = s as std::os::raw::c_int;
                let mut ferr = 0u32;
                let mut tmo = 0u16;
                let mut tq = 0u16;
                let rc = unsafe { ffi::ec_rt_read_limits(slot, &mut ferr, &mut tmo, &mut tq) };
                if rc != 0 {
                    tracing::error!(
                        subsystem = "ethercat",
                        event = "drive_limits_read_fail",
                        slot = s,
                        rc,
                        "SDO read of protection limits failed"
                    );
                    return Err(rc);
                }
                tracing::info!(
                    subsystem = "ethercat",
                    event = "drive_limits",
                    slot = s,
                    ferr_6065h = ferr,
                    timeout_6066h = tmo,
                    torque_6072h = tq,
                    "drive protection limits read at bringup"
                );
                let cli_ferr = slaves[s].following_error_counts;
                let cli_tq = slaves[s].max_torque_tenth_pct;
                let run = (cli_ferr.unwrap_or(ferr), cli_tq.unwrap_or(tq));
                if cli_ferr.is_some() || cli_tq.is_some() {
                    let rc = unsafe { ffi::ec_rt_write_limits(slot, run.0, run.1) };
                    if rc != 0 {
                        tracing::error!(
                            subsystem = "ethercat",
                            event = "drive_limits_write_fail",
                            slot = s,
                            rc,
                            "SDO write of session limits failed"
                        );
                        return Err(rc);
                    }
                    tracing::info!(
                        subsystem = "ethercat",
                        event = "drive_limits_applied",
                        slot = s,
                        ferr_6065h = run.0,
                        torque_6072h = run.1,
                        "session protection limits applied"
                    );
                }
                Ok(run)
            })
            .collect();
        let run_limits = match limits {
            Ok(limits) => limits,
            Err(rc) => {
                unsafe { ffi::ec_rt_shutdown() };
                bringup_fail(&mut server, rc);
            }
        };

        let rc = unsafe { ffi::ec_rt_bringup_finish() };
        tracing::info!(
            subsystem = "ethercat",
            event = "bringup_finish",
            rc,
            "OP bringup result"
        );
        if rc != 0 {
            log_al_states(num_slaves, "finish_fail");
            unsafe { ffi::ec_rt_shutdown() };
            bringup_fail(&mut server, rc);
        }

        run_limits
    };
    log_al_states(num_slaves, "parked");
    tracing::info!(
        subsystem = "ethercat",
        event = "bringup_parked",
        "drives parked (Ready-to-Switch-On, no torque)"
    );

    match wait_for_claim_pumping(
        &mut server,
        std::time::Instant::now() + std::time::Duration::from_secs(5),
        &SIGTERM_RECEIVED,
        "ec-rt",
        &mut || {
            let mut toff = 0i64;
            unsafe { ffi::ec_rt_park_cycle(&mut toff) };
        },
    ) {
        Some(cid) => {
            server.respond(&claim_handshake_reply_frame(
                cid,
                &all_slaves_reply(num_slaves, SlaveState::Ok, 0),
            ));
        }
        None => {
            tracing::error!(
                subsystem = "ethercat",
                event = "claim_handshake_timeout",
                "bridge did not send ClaimHandshake within 5 s; aborting"
            );
            unsafe {
                for s in 0..num_slaves {
                    ffi::ec_rt_disable(s as std::os::raw::c_int);
                }
                ffi::ec_rt_shutdown();
            }
            std::process::exit(1);
        }
    }
    tracing::info!(
        subsystem = "ethercat",
        event = "bringup_handshake_ok",
        "handshake ok, entering DC loop"
    );

    let gate = TorqueGate::new();
    let capture = Capture::new();
    let cycle_index: u64 = 0;
    let mailbox = MailboxWorker::spawn(
        FfiSdoBus,
        |slot, ferr_counts, torque_tenth_pct| unsafe {
            ffi::ec_rt_write_limits(i32::from(slot), ferr_counts, torque_tenth_pct)
        },
        match mailbox_cpu {
            Some(cpu) => WorkerScheduling::RealtimeCompanion {
                cpu,
                priority: MAILBOX_RT_PRIO,
            },
            None => WorkerScheduling::Normal,
        },
    );
    let pending_starts: Vec<(u32, String, PendingStart)> = Vec::new();
    let pending_stops: Vec<(u32, PendingStop)> = Vec::new();
    let capture_slots: Vec<u8> = Vec::new();
    let prdiv = 0u64;
    let ff_saturation = 0u32;
    let wkc_consecutive = 0u8;
    let latched_drive_err: u16 = 0;
    let sensorless = SensorlessBank::new(num_slaves);
    let stream_halt = StreamHalt::default();

    EndpointCtx {
        server,
        num_slaves,
        counts_per_mm,
        invert,
        cmd_counts_per_mm,
        rotation_distance,
        slave_axes,
        velocity_ff,
        torque_clamp_tenths,
        ff_lead_ns,
        jump_log_counts,
        cycle_ns,
        telemetry_period,
        dynamics,
        run_limits,
        rings,
        buzz,
        cmaps,
        last_counts,
        report_anchor,
        last_streamed_target,
        last_sent_retired,
        heartbeat_sent,
        gate,
        capture,
        cycle_index,
        mailbox,
        pending_starts,
        pending_stops,
        capture_slots,
        prdiv,
        ff_saturation,
        wkc_consecutive,
        latched_drive_err,
        sensorless,
        stream_halt,
    }
}

pub fn run(ctx: &mut EndpointCtx) {
    'dc: loop {
        if SIGTERM_RECEIVED.load(Ordering::Acquire) {
            eprintln!("ec-rt: SIGTERM received — disabling drive and exiting");
            tracing::warn!(
                subsystem = "ethercat",
                event = "endpoint_exit",
                reason = "sigterm",
                "endpoint exiting: SIGTERM received — disabling drive"
            );
            break;
        }
        if ctx.server.session_ended() {
            eprintln!("ec-rt: bridge disconnected — disabling drive and exiting");
            tracing::warn!(
                subsystem = "ethercat",
                event = "endpoint_exit",
                reason = "bridge_disconnected",
                "endpoint exiting: bridge (klippy) disconnected — disabling drive (downstream of a host-side abort)"
            );
            break;
        }

        if dispatch_commands(ctx).is_break() {
            break 'dc;
        }
        drain_pending_starts(ctx);
        drain_pending_stops(ctx);
        drain_mailbox_replies(ctx);

        if run_cycle(ctx).is_break() {
            break;
        }
    }

    unsafe {
        for s in 0..ctx.num_slaves {
            ffi::ec_rt_disable(s as std::os::raw::c_int);
        }
        ffi::ec_rt_shutdown();
    }
    eprintln!("ec-rt: shutdown complete");
}

fn dispatch_commands(ctx: &mut EndpointCtx) -> ControlFlow<()> {
    for cmd in ctx.server.poll_commands() {
        match cmd {
            Command::Identify {
                correlation_id,
                proto_version,
            } => {
                ctx.server
                    .respond(&identify_response_frame(correlation_id, proto_version));
            }
            Command::PushPieces {
                correlation_id,
                msg,
            } => {
                handle_push_pieces(ctx, correlation_id, msg);
            }
            Command::QueryRuntimeCaps { correlation_id } => {
                let total: u32 = (AXIS_RING_CAPACITY * ctx.num_slaves * 32) as u32;
                ctx.server
                    .respond(&runtime_caps_response_frame(correlation_id, total));
            }
            Command::SetTorque {
                correlation_id,
                msg,
            } => {
                handle_set_torque(ctx, correlation_id, msg);
            }
            Command::Stop { correlation_id } => {
                let now_ns = monotonic_ns();
                for r in &mut ctx.rings {
                    r.reset();
                }
                for c in &mut ctx.cmaps {
                    *c = None;
                }
                ctx.stream_halt.halt();
                eprintln!("ec-rt: Stop — rings discarded, stream halted, discard_clock={now_ns}");
                ctx.server
                    .respond(&stop_response_frame(correlation_id, 0, now_ns));
            }
            Command::StartCapture {
                correlation_id,
                msg,
            } => {
                handle_start_capture(ctx, correlation_id, msg);
            }
            Command::StopCapture { correlation_id } => {
                let pending = ctx.capture.stop_async();
                ctx.pending_stops.push((correlation_id, pending));
            }
            Command::ResumeStream { correlation_id } => match ctx.stream_halt.resume() {
                Ok(()) => {
                    for r in &mut ctx.rings {
                        r.reset();
                    }
                    for c in &mut ctx.cmaps {
                        *c = None;
                    }
                    eprintln!("ec-rt: ResumeStream — stream reopened");
                    ctx.server
                        .respond(&resume_stream_response_frame(correlation_id, 0));
                }
                Err(code) => {
                    eprintln!("ec-rt: ResumeStream rejected code={code} — stream was not halted");
                    ctx.server
                        .respond(&resume_stream_response_frame(correlation_id, code));
                }
            },
            Command::ClaimHandshake { .. } => {
                eprintln!(
                    "ec-rt: protocol violation: ClaimHandshake after handshake \
                     — ending session"
                );
                return ControlFlow::Break(());
            }
            Command::SetDriveLimits {
                correlation_id,
                msg,
            } => {
                handle_set_drive_limits(ctx, correlation_id, msg);
            }
            Command::RestoreDriveLimits {
                correlation_id,
                slot,
            } => {
                handle_restore_drive_limits(ctx, correlation_id, slot);
            }
            Command::SeedServoHome {
                correlation_id,
                slot,
                home_q16,
            } => {
                handle_seed_servo_home(ctx, correlation_id, slot, home_q16);
            }
            Command::ArmSensorlessEndstop {
                correlation_id,
                msg,
            } => {
                handle_arm_sensorless_endstop(ctx, correlation_id, msg);
            }
            Command::ResonanceBuzz {
                correlation_id,
                msg,
            } => {
                handle_resonance_buzz(ctx, correlation_id, msg);
            }
            Command::SdoRead {
                correlation_id,
                msg,
            } => {
                handle_sdo_read(ctx, correlation_id, msg);
            }
            Command::SdoWrite {
                correlation_id,
                msg,
            } => {
                handle_sdo_write(ctx, correlation_id, msg);
            }
            Command::QueryMotorState { correlation_id } => {
                handle_query_motor_state(ctx, correlation_id);
            }
            Command::Unknown { kind_raw, .. } => {
                eprintln!("ec-rt: ignoring kind 0x{kind_raw:04x}");
            }
        }
    }
    ControlFlow::Continue(())
}

fn handle_push_pieces(ctx: &mut EndpointCtx, correlation_id: u32, msg: PushPieces) {
    let now_ns = monotonic_ns();
    let diags: Vec<(u8, u64)> = msg
        .axes
        .iter()
        .map(|a| {
            let front_start_time = if a.piece_count > 0 && a.pieces_bytes.len() >= 8 {
                u64::from_le_bytes(a.pieces_bytes[0..8].try_into().unwrap_or([0; 8]))
            } else {
                0
            };
            (a.axis_idx, front_start_time)
        })
        .collect();
    let result = if ctx.gate.state() == TorqueState::Faulted {
        ERR_PIECES_WHILE_FAULTED
    } else if let Err(code) = ctx.stream_halt.check_push_allowed() {
        code
    } else {
        match plan_bundle(&msg.axes, &ctx.slave_axes, |slot| ctx.rings[slot].free()) {
            Ok(slots) => {
                for (axis, &slot) in msg.axes.iter().zip(slots.iter()) {
                    ctx.rings[slot].push_from_bytes(axis.piece_count, &axis.pieces_bytes);
                }
                0
            }
            Err(code) => code,
        }
    };
    ctx.server.respond(&push_pieces_response_frame_multi(
        correlation_id,
        result,
        now_ns,
        &diags,
    ));
}

fn handle_set_torque(ctx: &mut EndpointCtx, correlation_id: u32, msg: SetTorque) {
    let num_slaves = ctx.num_slaves;
    match ctx.gate.on_set_torque(msg.value != 0, msg.execute_at_ns) {
        CommandAction::Enable => {
            let mut enable_rc = 0;
            for s in 0..num_slaves {
                let rc = unsafe { ffi::ec_rt_enable(s as std::os::raw::c_int) };
                if rc != 0 {
                    eprintln!("ec-rt: slot {s} CiA402 enable failed rc={rc}");
                    enable_rc = rc;
                    break;
                }
            }
            ctx.gate.enable_finished(enable_rc == 0);
            if enable_rc == 0 {
                eprintln!(
                    "ec-rt: torque enabled (CiA402 operation enabled, {num_slaves} slave(s))"
                );
                ctx.server
                    .respond(&set_torque_response_frame(correlation_id, 0));
            } else {
                eprintln!("ec-rt: CiA402 enable failed rc={enable_rc} — disabling and exiting");
                ctx.server.respond(&set_torque_response_frame(
                    correlation_id,
                    ERR_ENABLE_FAILED,
                ));
                unsafe {
                    for s in 0..num_slaves {
                        ffi::ec_rt_disable(s as std::os::raw::c_int);
                    }
                    ffi::ec_rt_shutdown();
                }
                std::process::exit(1);
            }
        }
        CommandAction::ScheduleDisable => {
            eprintln!(
                "ec-rt: torque disable scheduled at {} (now {})",
                msg.execute_at_ns,
                monotonic_ns()
            );
            ctx.server
                .respond(&set_torque_response_frame(correlation_id, 0));
        }
        CommandAction::Reject { code } => {
            eprintln!(
                "ec-rt: SetTorque rejected code={code} \
                     (value={} execute_at={} now={}) — exiting",
                msg.value,
                msg.execute_at_ns,
                monotonic_ns()
            );
            ctx.server
                .respond(&set_torque_response_frame(correlation_id, code));
            unsafe {
                for s in 0..num_slaves {
                    ffi::ec_rt_disable(s as std::os::raw::c_int);
                }
                ffi::ec_rt_shutdown();
            }
            std::process::exit(1);
        }
    }
}

fn handle_start_capture(ctx: &mut EndpointCtx, correlation_id: u32, msg: StartCapture) {
    let num_slaves = ctx.num_slaves;
    let slots: Vec<u8> = msg.drives.iter().map(|d| d.slot).collect();
    if any_slot_out_of_range(&slots, num_slaves) {
        eprintln!(
            "ec-rt: StartCapture drive slot out of range \
             (num_slaves={num_slaves}) — rejecting"
        );
        ctx.server.respond(&start_capture_response_frame(
            correlation_id,
            ERR_CAPTURE_BAD_DRIVE_LIST,
        ));
    } else {
        let drives: Vec<CaptureDriveConfig> = msg
            .drives
            .iter()
            .map(|d| CaptureDriveConfig {
                slot: d.slot,
                name: d.name.clone(),
                counts_per_mm: ctx.counts_per_mm[d.slot as usize],
                rotation_distance: ctx.rotation_distance[d.slot as usize],
            })
            .collect();
        let pending = ctx.capture.start_async(CaptureConfig {
            path: msg.path.clone(),
            started_utc: msg.started_utc.clone(),
            drives,
            cycle_ns: ctx.cycle_ns,
            started_mono_ns: monotonic_ns(),
        });
        if pending.claimed() {
            ctx.capture_slots = slots;
        }
        ctx.pending_starts.push((correlation_id, msg.path, pending));
    }
}

fn handle_set_drive_limits(ctx: &mut EndpointCtx, correlation_id: u32, msg: SetDriveLimits) {
    let num_slaves = ctx.num_slaves;
    if msg.slot as usize >= num_slaves {
        eprintln!(
            "ec-rt: SetDriveLimits for slot {} but only {num_slaves} slave(s)",
            msg.slot
        );
        ctx.server
            .respond(&set_drive_limits_response_frame(correlation_id, -309));
    } else {
        ctx.mailbox.submit(MailboxRequest::WriteLimits {
            correlation_id,
            slot: msg.slot,
            ferr_counts: msg.following_error_counts,
            torque_tenth_pct: msg.max_torque_tenth_pct,
            restore: false,
        });
    }
}

fn handle_restore_drive_limits(ctx: &mut EndpointCtx, correlation_id: u32, slot: u8) {
    match ctx.run_limits.get(slot as usize) {
        Some(&(ferr_counts, torque_tenth_pct)) => {
            ctx.mailbox.submit(MailboxRequest::WriteLimits {
                correlation_id,
                slot,
                ferr_counts,
                torque_tenth_pct,
                restore: true,
            });
        }
        None => {
            eprintln!(
                "ec-rt: RestoreDriveLimits for slot {slot} but only {} slave(s)",
                ctx.run_limits.len()
            );
            ctx.server
                .respond(&restore_drive_limits_response_frame(correlation_id, -309));
        }
    }
}

fn handle_seed_servo_home(ctx: &mut EndpointCtx, correlation_id: u32, slot: u8, home_q16: i32) {
    if slot as usize >= ctx.counts_per_mm.len() {
        eprintln!(
            "ec-rt: SeedServoHome for slot {slot} but only {} slave(s)",
            ctx.counts_per_mm.len()
        );
        ctx.server
            .respond(&seed_servo_home_response_frame(correlation_id, -309));
    } else if ctx.rings.iter().any(|r| !r.is_empty()) {
        eprintln!("ec-rt: SeedServoHome rejected — motion ring not empty");
        ctx.server.respond(&seed_servo_home_response_frame(
            correlation_id,
            ERR_SEED_HOME_STREAMING,
        ));
    } else {
        let anchor_mm = f64::from(home_q16) / 65536.0;
        let anchor_counts = ctx.last_streamed_target[slot as usize]
            .unwrap_or_else(|| unsafe { ffi::ec_rt_get_position_actual(i32::from(slot)) });
        ctx.report_anchor[slot as usize] = Some((anchor_counts, anchor_mm));
        eprintln!(
            "ec-rt: SeedServoHome slot={slot} report anchor \
             {anchor_counts} counts = {anchor_mm:.4} mm (drive frame untouched)"
        );
        ctx.server
            .respond(&seed_servo_home_response_frame(correlation_id, 0));
    }
}

fn handle_arm_sensorless_endstop(
    ctx: &mut EndpointCtx,
    correlation_id: u32,
    msg: ArmSensorlessEndstop,
) {
    let num_slaves = ctx.num_slaves;
    let result = if msg.slot as usize >= num_slaves {
        eprintln!(
            "ec-rt: ArmSensorlessEndstop for slot {} but only {num_slaves} slave(s)",
            msg.slot
        );
        -309
    } else if msg.enable != 0 {
        if msg.torque_trip_tenth_pct == 0 {
            eprintln!("ec-rt: ArmSensorlessEndstop rejected — zero torque trip threshold");
            ERR_ARM_SENSORLESS_BAD_THRESHOLD
        } else {
            ctx.sensorless
                .arm(msg.slot as usize, msg.endstop_id, msg.torque_trip_tenth_pct);
            eprintln!(
                "ec-rt: sensorless endstop {} armed on slot {} (torque_trip={} 0.1%)",
                msg.endstop_id, msg.slot, msg.torque_trip_tenth_pct
            );
            0
        }
    } else {
        ctx.sensorless.disarm(msg.slot as usize);
        eprintln!(
            "ec-rt: sensorless endstop {} disarmed on slot {}",
            msg.endstop_id, msg.slot
        );
        0
    };
    ctx.server.respond(&arm_sensorless_endstop_response_frame(
        correlation_id,
        result,
    ));
}

fn handle_resonance_buzz(ctx: &mut EndpointCtx, correlation_id: u32, msg: ResonanceBuzz) {
    let rc = if ctx.gate.state() != TorqueState::Enabled {
        eprintln!("ec-rt: ResonanceBuzz rejected — drive not operation-enabled");
        crate::buzz::ERR_BUZZ_NOT_ENABLED
    } else if ctx.rings.iter().any(|r| !r.is_empty()) || ctx.buzz.active() {
        eprintln!("ec-rt: ResonanceBuzz rejected — motion in progress");
        if ctx.buzz.active() {
            crate::buzz::ERR_BUZZ_BUSY
        } else {
            crate::buzz::ERR_BUZZ_STREAMING
        }
    } else {
        let base_counts = unsafe { ffi::ec_rt_get_position_actual(0) };
        let rc = ctx.buzz.arm(
            msg.axis_mask,
            msg.sign_mask,
            msg.freq_start_millihz,
            msg.freq_end_millihz,
            msg.amplitude_nm,
            msg.duration_ms,
            msg.ramp_ms,
            base_counts,
        );
        eprintln!(
            "ec-rt: ResonanceBuzz axis_mask=0x{:02x} sign_mask=0x{:02x} \
             freq={}->{} mHz amplitude={} nm duration={} ms ramp={} ms \
             base_counts={base_counts} rc={rc}",
            msg.axis_mask,
            msg.sign_mask,
            msg.freq_start_millihz,
            msg.freq_end_millihz,
            msg.amplitude_nm,
            msg.duration_ms,
            msg.ramp_ms,
        );
        rc
    };
    ctx.server
        .respond(&resonance_buzz_response_frame(correlation_id, rc));
}

fn handle_sdo_read(ctx: &mut EndpointCtx, correlation_id: u32, msg: SdoRead) {
    let num_slaves = ctx.num_slaves;
    if msg.slot as usize >= num_slaves {
        eprintln!(
            "ec-rt: SdoRead for slot {} but only {num_slaves} slave(s)",
            msg.slot
        );
        ctx.server.respond(&sdo_read_response_frame(
            correlation_id,
            &SdoReadResponse {
                result: -309,
                size: 0,
                data: [0; 4],
            },
        ));
    } else {
        ctx.mailbox.submit(MailboxRequest::SdoRead {
            correlation_id,
            msg,
        });
    }
}

fn handle_sdo_write(ctx: &mut EndpointCtx, correlation_id: u32, msg: SdoWrite) {
    let num_slaves = ctx.num_slaves;
    if msg.slot as usize >= num_slaves {
        eprintln!(
            "ec-rt: SdoWrite for slot {} but only {num_slaves} slave(s)",
            msg.slot
        );
        ctx.server.respond(&sdo_write_response_frame(
            correlation_id,
            &SdoWriteResponse {
                result: -309,
                readback_size: 0,
                readback_data: [0; 4],
            },
        ));
    } else {
        ctx.mailbox.submit(MailboxRequest::SdoWrite {
            correlation_id,
            msg,
        });
    }
}

fn handle_query_motor_state(ctx: &mut EndpointCtx, correlation_id: u32) {
    let num_slaves = ctx.num_slaves;
    let samples: Vec<(u8, f64, f64)> = (0..num_slaves)
        .filter_map(|s| {
            let (anchor_counts, anchor_mm) = ctx.report_anchor[s]?;
            let slot = s as std::os::raw::c_int;
            let (pos_counts, vel_counts_s) = unsafe {
                (
                    ffi::ec_rt_get_position_actual(slot),
                    ffi::ec_rt_get_velocity_actual(slot),
                )
            };
            let delta_counts = i64::from(pos_counts) - i64::from(anchor_counts);
            let pos_mm = anchor_mm + delta_counts as f64 / ctx.cmd_counts_per_mm[s];
            let vel_mm_s = crate::scale::velocity_mm_s(vel_counts_s, ctx.cmd_counts_per_mm[s]);
            Some((s as u8, pos_mm, vel_mm_s))
        })
        .collect();
    if samples.is_empty() {
        ctx.server.respond(&motor_state_empty_frame(correlation_id));
    } else {
        ctx.server
            .respond(&motor_state_response_frame_multi(correlation_id, &samples));
    }
}

fn drain_pending_starts(ctx: &mut EndpointCtx) {
    let mut start_idx = 0;
    while start_idx < ctx.pending_starts.len() {
        match ctx.pending_starts[start_idx].2.try_take() {
            Some(rc) => {
                let (correlation_id, path, pending) = ctx.pending_starts.remove(start_idx);
                if rc != 0 && pending.claimed() {
                    ctx.capture.clear_failed_start();
                }
                eprintln!("ec-rt: StartCapture path={path} rc={rc}");
                ctx.server
                    .respond(&start_capture_response_frame(correlation_id, rc));
            }
            None => start_idx += 1,
        }
    }
}

fn drain_pending_stops(ctx: &mut EndpointCtx) {
    let mut stop_idx = 0;
    while stop_idx < ctx.pending_stops.len() {
        match ctx.pending_stops[stop_idx].1.try_take() {
            Some(out) => {
                let (correlation_id, _) = ctx.pending_stops.remove(stop_idx);
                eprintln!(
                    "ec-rt: StopCapture result={} samples={} overflow={:?}",
                    out.result, out.samples, out.overflow_cycle
                );
                ctx.server.respond(&stop_capture_response_frame(
                    correlation_id,
                    out.result,
                    out.samples,
                    out.overflow_cycle
                        .unwrap_or(StopCaptureResponse::NO_OVERFLOW),
                ));
            }
            None => stop_idx += 1,
        }
    }
}

fn drain_mailbox_replies(ctx: &mut EndpointCtx) {
    while let Some(reply) = ctx.mailbox.try_recv() {
        match reply {
            MailboxReply::SdoRead {
                correlation_id,
                msg,
                resp,
            } => {
                if resp.result != 0 {
                    eprintln!(
                        "ec-rt: SdoRead 0x{:04x}.{} failed result={}",
                        msg.index, msg.subindex, resp.result
                    );
                }
                ctx.server
                    .respond(&sdo_read_response_frame(correlation_id, &resp));
            }
            MailboxReply::SdoWrite {
                correlation_id,
                msg,
                resp,
            } => {
                if resp.result != 0 {
                    eprintln!(
                        "ec-rt: SdoWrite 0x{:04x}.{} value={} size={} failed result={}",
                        msg.index, msg.subindex, msg.value, msg.size, resp.result
                    );
                }
                ctx.server
                    .respond(&sdo_write_response_frame(correlation_id, &resp));
            }
            MailboxReply::WriteLimits {
                correlation_id,
                rc,
                ferr_counts,
                torque_tenth_pct,
                restore,
            } => {
                let what = if restore {
                    "RestoreDriveLimits"
                } else {
                    "SetDriveLimits"
                };
                if rc != 0 {
                    eprintln!(
                        "ec-rt: {what} SDO write failed rc={rc} \
                             ferr={ferr_counts} tq={torque_tenth_pct}"
                    );
                } else {
                    eprintln!("ec-rt: {what} applied ferr={ferr_counts} tq={torque_tenth_pct}");
                }
                let frame = if restore {
                    restore_drive_limits_response_frame(correlation_id, rc)
                } else {
                    set_drive_limits_response_frame(correlation_id, rc)
                };
                ctx.server.respond(&frame);
            }
        }
    }
}

fn run_cycle(ctx: &mut EndpointCtx) -> ControlFlow<()> {
    let next_flush_mono_ns = unsafe { ffi::ec_rt_cycle_time_ns() } + ctx.cycle_ns as u64;
    let apply_time = raw_from_monotonic_ns(next_flush_mono_ns);

    let all_rings_empty = ctx.rings.iter().all(|r| r.is_empty());
    apply_tick_action(ctx, apply_time, all_rings_empty);

    poll_sensorless(ctx, apply_time);

    let (motion_active, all_acc, all_vel) = compute_motion_targets(ctx, apply_time);

    handle_ring_fault(ctx);

    let mut toff = 0i64;
    let wkc = unsafe { ffi::ec_rt_cycle(&mut toff) };

    handle_drive_fault(ctx);

    ctx.cycle_index += 1;
    record_capture_sample(ctx, motion_active, &all_acc, &all_vel);

    if evaluate_wkc(ctx, wkc).is_break() {
        return ControlFlow::Break(());
    }

    emit_heartbeat(ctx);
    emit_periodic_telemetry(ctx, wkc, toff);

    ControlFlow::Continue(())
}

fn apply_tick_action(ctx: &mut EndpointCtx, apply_time: u64, all_rings_empty: bool) {
    match ctx.gate.on_tick(apply_time, all_rings_empty) {
        TickAction::None => {}
        TickAction::ExecuteDisable => {
            eprintln!("ec-rt: scheduled torque disable executing");
            unsafe {
                for s in 0..ctx.num_slaves {
                    ffi::ec_rt_disable(s as std::os::raw::c_int);
                }
            }
            ctx.gate.disable_finished();
            for c in &mut ctx.cmaps {
                *c = None;
            }
        }
        TickAction::Fault { code } => {
            eprintln!(
                "ec-rt: torque-gate fault code={code} — pieces present without torque, exiting"
            );
            let retired: Vec<u32> = ctx.rings.iter().map(|r| r.retired_count()).collect();
            ctx.server.respond(&status_heartbeat_frame(
                ENGINE_STATE_FAULT,
                0,
                &retired,
                ctx.ff_saturation,
            ));
            unsafe {
                for s in 0..ctx.num_slaves {
                    ffi::ec_rt_disable(s as std::os::raw::c_int);
                }
                ffi::ec_rt_shutdown();
            }
            std::process::exit(1);
        }
    }
}

fn poll_sensorless(ctx: &mut EndpointCtx, apply_time: u64) {
    let server = &mut ctx.server;
    let sensorless_tripped = ctx.sensorless.poll(
        |slot| unsafe { ffi::ec_rt_get_torque_actual(slot as i32) },
        |slot, endstop_id, torque| {
            eprintln!(
                "ec-rt: sensorless endstop {endstop_id} tripped on slot {slot} \
                 torque={torque} — local stop, stream halted, trip_clock={apply_time}"
            );
            server.respond(&endstop_trip_frame(endstop_id, apply_time));
        },
    );
    if sensorless_tripped {
        for r in &mut ctx.rings {
            r.reset();
        }
        for c in &mut ctx.cmaps {
            *c = None;
        }
        ctx.stream_halt.halt();
    }
}

fn compute_motion_targets(ctx: &mut EndpointCtx, apply_time: u64) -> (bool, Vec<f32>, Vec<f32>) {
    let num_slaves = ctx.num_slaves;
    let mut motion_active = false;
    // The commanded analytic accel/vel the feedforward path samples are the
    // noise-free, C00.06-independent regressors the identification fit wants,
    // so they outlive the feedforward block to reach the capture record.
    let mut all_acc = vec![0f32; num_slaves];
    let mut all_vel = vec![0f32; num_slaves];
    if ctx.gate.state() == TorqueState::Enabled {
        // The coupled torque model needs every axis' accel/vel before any
        // one slot's feedforward can be computed, so sample all slots first.
        let mut sp_counts: Vec<Option<i32>> = vec![None; num_slaves];
        for s in 0..num_slaves {
            let sampled = if ctx.buzz.active() {
                if s == 0 {
                    let cmd_counts_per_mm0 = ctx.cmd_counts_per_mm[0];
                    ctx.buzz
                        .eval(apply_time)
                        .map(|(rel_mm, vel_mm_s, acc_mm_s2)| {
                            let counts = ctx
                                .buzz
                                .base_counts()
                                .wrapping_add(mm_to_counts(f64::from(rel_mm), cmd_counts_per_mm0));
                            (counts, vel_mm_s, acc_mm_s2)
                        })
                } else {
                    None
                }
            } else if let Some((pos_mm, vel_mm_s, acc_mm_s2)) = ctx.rings[s].sample(apply_time) {
                // Streaming is always relative: each stream anchors the
                // drive's actual position to the host's first commanded
                // value, so a homing set_position (host frame shift) can
                // never yank a drive. The report_anchor covers absolute
                // position queries; the drive frame itself is never used.
                let cpm = ctx.cmd_counts_per_mm[s];
                let map = ctx.cmaps[s].get_or_insert_with(|| {
                    let actual =
                        unsafe { ffi::ec_rt_get_position_actual(s as std::os::raw::c_int) };
                    CountMap::new(cpm, actual, f64::from(pos_mm))
                });
                Some((map.target_counts(f64::from(pos_mm)), vel_mm_s, acc_mm_s2))
            } else {
                if !ctx.buzz.active() {
                    ctx.cmaps[s] = None;
                }
                None
            };
            if let Some((counts, vel_mm_s, acc_mm_s2)) = sampled {
                sp_counts[s] = Some(counts);
                let (ff_vel, ff_acc) = if ctx.ff_lead_ns[s] > 0 && !ctx.buzz.active() {
                    ctx.rings[s].peek_vel_acc(apply_time + ctx.ff_lead_ns[s])
                } else {
                    (vel_mm_s, acc_mm_s2)
                };
                all_vel[s] = ff_vel;
                all_acc[s] = ff_acc;
            }
        }

        // The dynamics profile is fitted in the drive frame (the capture
        // flips each drive's commanded kinematics by its direction sign),
        // so the model must be evaluated on drive-frame vectors — flipping
        // only the output torque by the slot's own sign would negate the
        // off-diagonal coupling terms whenever the drives' inverts differ.
        let drive_dir = |s: usize| ctx.cmd_counts_per_mm.get(s).map_or(1.0, |c| c.signum()) as f32;
        let (acc_drive, vel_drive): (Vec<f32>, Vec<f32>) = (0..num_slaves)
            .map(|s| (drive_dir(s) * all_acc[s], drive_dir(s) * all_vel[s]))
            .unzip();
        for s in 0..num_slaves {
            let slot = s as std::os::raw::c_int;
            if let Some(counts) = sp_counts[s] {
                let vel_offset = if ctx.velocity_ff[s] {
                    (f64::from(all_vel[s]) * ctx.cmd_counts_per_mm[s]).round() as i32
                } else {
                    0
                };
                let torque_offset = match &ctx.dynamics {
                    Some(model) => {
                        let raw = model.torque_ff(s, &acc_drive, &vel_drive);
                        if !raw.is_finite() {
                            eprintln!(
                                "ec-rt: FAULT non-finite torque FF on slot {s} \
                                 (acc={} vel={}) — disabling",
                                all_acc[s], all_vel[s]
                            );
                            let retired: Vec<u32> =
                                ctx.rings.iter().map(|r| r.retired_count()).collect();
                            ctx.server.respond(&status_heartbeat_frame(
                                ENGINE_STATE_FAULT,
                                0,
                                &retired,
                                ctx.ff_saturation,
                            ));
                            unsafe {
                                for d in 0..num_slaves {
                                    ffi::ec_rt_disable(d as std::os::raw::c_int);
                                }
                                ffi::ec_rt_shutdown();
                            }
                            std::process::exit(1);
                        }
                        clamp_torque(raw, ctx.torque_clamp_tenths[s], &mut ctx.ff_saturation)
                    }
                    None => 0,
                };
                if let Some(prev) = ctx.last_counts[s] {
                    let increment = i64::from(counts) - i64::from(prev);
                    if increment.abs() > ctx.jump_log_counts[s] {
                        tracing::warn!(
                            subsystem = "ethercat",
                            event = "target_jump",
                            slot = s,
                            prev_target = prev,
                            new_target = counts,
                            increment,
                            vel_mm_s = f64::from(all_vel[s]),
                            acc_mm_s2 = f64::from(all_acc[s]),
                            invert = ctx.invert[s],
                            "commanded target increment exceeds sane per-cycle bound"
                        );
                    }
                }
                ctx.last_counts[s] = Some(counts);
                ctx.last_streamed_target[s] = Some(counts);
                unsafe {
                    ffi::ec_rt_set_target_position(slot, counts);
                    ffi::ec_rt_set_velocity_offset(slot, vel_offset);
                    ffi::ec_rt_set_torque_offset(slot, torque_offset);
                }
                motion_active = true;
            } else {
                ctx.last_counts[s] = None;
                unsafe {
                    ffi::ec_rt_set_velocity_offset(slot, 0);
                    ffi::ec_rt_set_torque_offset(slot, 0);
                }
            }
        }
    } else {
        for lc in &mut ctx.last_counts {
            *lc = None;
        }
    }
    (motion_active, all_acc, all_vel)
}

fn handle_ring_fault(ctx: &mut EndpointCtx) {
    let ring_fault = ctx
        .rings
        .iter()
        .enumerate()
        .find_map(|(s, r)| r.take_fault().map(|f| (s, f)));
    if let Some((slot, fault_val)) = ring_fault {
        let fault_code_u16 = (fault_val & 0xFFFF) as u16;
        eprintln!(
            "ec-rt: FAULT latched on slot {slot} fault_val=0x{fault_val:08x} \
             code=0x{fault_code_u16:04x} — notifying host via heartbeat"
        );
        tracing::error!(
            subsystem = "ethercat",
            event = "ring_fault_latched",
            slot,
            fault_val,
            fault_code = fault_val as i32,
            "drive ring latched a runtime fault — notifying host via heartbeat"
        );
        let retired: Vec<u32> = ctx.rings.iter().map(|r| r.retired_count()).collect();
        #[cfg(not(feature = "hw"))]
        let current_retired: u32 = retired.iter().sum();
        ctx.server.respond(&status_heartbeat_frame(
            ENGINE_STATE_FAULT,
            fault_code_u16,
            &retired,
            ctx.ff_saturation,
        ));

        #[cfg(feature = "hw")]
        {
            eprintln!("ec-rt: disabling drives (hw safety backstop)");
            unsafe {
                for s in 0..ctx.num_slaves {
                    ffi::ec_rt_disable(s as std::os::raw::c_int);
                }
                ffi::ec_rt_shutdown();
            }
            std::process::exit(1);
        }

        #[cfg(not(feature = "hw"))]
        {
            ctx.last_sent_retired = current_retired;
            ctx.heartbeat_sent = true;
        }
    }
}

fn handle_drive_fault(ctx: &mut EndpointCtx) {
    let num_slaves = ctx.num_slaves;
    let drive_fault = (0..num_slaves).find_map(|s| {
        let e = unsafe { ffi::ec_rt_get_error_code(s as std::os::raw::c_int) };
        if e != 0 {
            Some((s, e))
        } else {
            None
        }
    });
    if let Some((slot, err)) = drive_fault {
        if ctx.gate.state() != TorqueState::Faulted {
            eprintln!(
                "ec-rt: DRIVE FAULT slot {slot} err=0x{err:04x} — parking, reporting via heartbeat"
            );
            for d in 0..num_slaves {
                let mut t = ffi::EcTelemetry::default();
                unsafe { ffi::ec_rt_get_telemetry(d as std::os::raw::c_int, &mut t) };
                let last_cmd = ctx.last_counts[d].unwrap_or(t.target_position);
                tracing::error!(
                    subsystem = "ethercat",
                    event = "drive_fault",
                    faulted_slot = slot,
                    slot = d,
                    axis = ctx.slave_axes[d],
                    invert = ctx.invert[d],
                    err = err,
                    error_code = t.error_code,
                    statusword = t.statusword,
                    target_counts = t.target_position,
                    last_cmd_target = last_cmd,
                    last_increment = i64::from(t.target_position) - i64::from(last_cmd),
                    actual = t.position_actual,
                    following_error = t.following_error,
                    velocity_actual = t.velocity_actual,
                    torque_actual = t.torque_actual,
                    velocity_offset = t.velocity_offset,
                    torque_offset = t.torque_offset,
                    "drive fault — per-slot snapshot"
                );
            }
            ctx.gate.on_drive_fault();
            for r in &mut ctx.rings {
                r.reset();
            }
            for c in &mut ctx.cmaps {
                *c = None;
            }
            ctx.latched_drive_err = err;
            let retired: Vec<u32> = ctx.rings.iter().map(|r| r.retired_count()).collect();
            ctx.server
                .respond(&status_heartbeat_frame(0, err, &retired, ctx.ff_saturation));
            ctx.last_sent_retired = retired.iter().sum();
            ctx.heartbeat_sent = true;
        }
    }
}

fn record_capture_sample(
    ctx: &mut EndpointCtx,
    motion_active: bool,
    all_acc: &[f32],
    all_vel: &[f32],
) {
    if ctx.capture.is_active() {
        let mut flags = 0u8;
        if ctx.gate.state() == TorqueState::Enabled {
            flags |= FLAG_TORQUE_ENABLED;
        }
        if motion_active {
            flags |= FLAG_MOTION_ACTIVE;
        }
        let mut record = CaptureRecord::new(ctx.cycle_index, flags);
        record.drive_count = ctx.capture_slots.len() as u8;
        for (i, &slot) in ctx.capture_slots.iter().enumerate() {
            let mut t = ffi::EcTelemetry::default();
            unsafe { ffi::ec_rt_get_telemetry(i32::from(slot), &mut t) };
            // The commanded kinematics are sampled in planner-stream frame;
            // flip them into the drive frame (as cmd_counts_per_mm's sign
            // does for the target) so they are sign-consistent with the
            // drive-frame position/velocity/torque channels — otherwise an
            // inverted axis fits negative inertia.
            let dir = ctx.cmd_counts_per_mm[usize::from(slot)].signum() as f32;
            record.drives[i] = DriveSample {
                target_counts: t.target_position,
                position_actual: t.position_actual,
                velocity_actual: t.velocity_actual,
                following_error: t.following_error,
                torque_actual: t.torque_actual,
                statusword: t.statusword,
                error_code: t.error_code,
                velocity_offset: t.velocity_offset,
                torque_offset: t.torque_offset,
                accel_cmd: dir * all_acc[usize::from(slot)],
                vel_cmd: dir * all_vel[usize::from(slot)],
            };
        }
        ctx.capture.push(record);
    }
}

fn evaluate_wkc(ctx: &mut EndpointCtx, wkc: i32) -> ControlFlow<()> {
    let expected_wkc = 3 * ctx.num_slaves as i32;
    match eval_wkc(wkc, expected_wkc, &mut ctx.wkc_consecutive) {
        WkcDecision::Good => {}
        WkcDecision::Warn(n) => {
            tracing::warn!(
                subsystem = "ethercat",
                event = "wkc_warn",
                wkc,
                expected_wkc,
                consecutive_bad = n,
                halt_threshold = crate::claim::WKC_CONSECUTIVE_LOSS_LIMIT,
                "working counter below expected; tolerating one bad cycle"
            );
        }
        WkcDecision::Halt => {
            log_al_states(ctx.num_slaves, "bus_lost");
            tracing::error!(
                subsystem = "ethercat",
                event = "bus_lost",
                wkc,
                expected_wkc,
                consecutive_bad = ctx.wkc_consecutive,
                halt_threshold = crate::claim::WKC_CONSECUTIVE_LOSS_LIMIT,
                "bus lost after consecutive bad cycles, halting"
            );
            unsafe { ffi::ec_rt_dump_al_state() };
            return ControlFlow::Break(());
        }
    }
    ControlFlow::Continue(())
}

fn emit_heartbeat(ctx: &mut EndpointCtx) {
    let retired: Vec<u32> = ctx.rings.iter().map(|r| r.retired_count()).collect();
    // Sum is monotonic and changes whenever any slot retires a piece, so it
    // triggers an emit even when a slower axis advances behind the leader.
    let current_retired: u32 = retired.iter().sum();
    let all_empty_now = ctx.rings.iter().all(|r| r.is_empty());
    let should_emit = !ctx.heartbeat_sent || current_retired != ctx.last_sent_retired;
    if should_emit {
        let engine_state: u8 = if all_empty_now { 0 } else { 1 };
        ctx.server.respond(&status_heartbeat_frame(
            engine_state,
            0,
            &retired,
            ctx.ff_saturation,
        ));
        ctx.last_sent_retired = current_retired;
        ctx.heartbeat_sent = true;
        if current_retired != 0 {
            eprintln!("ec-rt: heartbeat retired={retired:?}");
        }
    }
}

fn emit_periodic_telemetry(ctx: &mut EndpointCtx, wkc: i32, toff: i64) {
    ctx.prdiv += 1;
    if ctx.prdiv >= ctx.telemetry_period {
        ctx.prdiv = 0;
        let all_empty_now = ctx.rings.iter().all(|r| r.is_empty());
        for s in 0..ctx.num_slaves {
            let mut t = ffi::EcTelemetry::default();
            unsafe { ffi::ec_rt_get_telemetry(s as std::os::raw::c_int, &mut t) };
            tracing::info!(
                subsystem = "ethercat",
                event = "telemetry",
                slot = s,
                axis = ctx.slave_axes[s],
                invert = ctx.invert[s],
                wkc,
                toff,
                statusword = t.statusword,
                error_code = t.error_code,
                target_counts = t.target_position,
                actual = t.position_actual,
                following_error = t.following_error,
                velocity_actual = t.velocity_actual,
                torque_actual = t.torque_actual,
                velocity_offset = t.velocity_offset,
                torque_offset = t.torque_offset,
                motion = !all_empty_now,
                ff_sat = ctx.ff_saturation,
                framed = ctx.report_anchor[s].is_some(),
                "per-slot drive telemetry"
            );
        }
        if ctx.gate.state() == TorqueState::Faulted {
            let retired: Vec<u32> = ctx.rings.iter().map(|r| r.retired_count()).collect();
            ctx.server.respond(&status_heartbeat_frame(
                0,
                ctx.latched_drive_err,
                &retired,
                ctx.ff_saturation,
            ));
        }
    }
}
