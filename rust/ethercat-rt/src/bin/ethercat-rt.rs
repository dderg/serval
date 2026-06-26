//! Usage: ethercat-rt <ifname> [--socket PATH] [--cycle-us N]
//!        [--counts-per-mm F] [--rotation-distance F] [--rt-cpu N] [--rt-prio N]
//!        [--velocity-ff] [--dynamics-profile PATH] [--torque-clamp-pct F]
#![allow(unsafe_code)]

use std::ffi::CString;
use std::sync::atomic::{AtomicBool, Ordering};

use ethercat_rt::buzz::BuzzOsc;
use ethercat_rt::capture::{
    Capture, CaptureConfig, CaptureRecord, DriveSample, PendingStart, PendingStop,
    FLAG_MOTION_ACTIVE, FLAG_TORQUE_ENABLED,
};
use ethercat_rt::claim::{
    all_slaves_reply, eval_wkc, single_slave_reply, wait_for_claim, wait_for_claim_pumping,
    WkcDecision,
};
use ethercat_rt::cli::parse_slaves;
use ethercat_rt::clock::monotonic_ns;
use ethercat_rt::curves::{AxisRing, AXIS_RING_CAPACITY, ENGINE_STATE_FAULT};
use ethercat_rt::dynamics::{clamp_torque, DynamicsModel};
use ethercat_rt::ffi;
use ethercat_rt::mailbox::{MailboxReply, MailboxRequest, MailboxWorker, WorkerScheduling};
use ethercat_rt::scale::{mm_to_counts, CountMap};
use ethercat_rt::sdo::SdoBus;
use ethercat_rt::seed_home::{
    ERR_SEED_HOME_BUSY, ERR_SEED_HOME_NOT_ENABLED, ERR_SEED_HOME_RESTORE, ERR_SEED_HOME_STREAMING,
};
use ethercat_rt::sensorless::{SensorlessArm, ERR_ARM_SENSORLESS_BAD_THRESHOLD};
use ethercat_rt::server::FrameServer;
use ethercat_rt::torque::{
    CommandAction, TickAction, TorqueGate, TorqueState, ERR_ENABLE_FAILED, ERR_PIECES_WHILE_FAULTED,
};
use ethercat_rt::wire::{
    arm_sensorless_endstop_response_frame, claim_handshake_reply_frame, endstop_trip_frame,
    identify_response_frame, motor_state_empty_frame, motor_state_response_frame_multi,
    push_pieces_response_frame_multi, resonance_buzz_response_frame,
    restore_drive_limits_response_frame, resume_stream_response_frame, runtime_caps_response_frame,
    sdo_read_response_frame, sdo_write_response_frame, seed_servo_home_response_frame,
    set_drive_limits_response_frame, set_torque_response_frame, start_capture_response_frame,
    status_heartbeat_frame, stop_capture_response_frame, stop_response_frame, Command,
};
use mcu_protocol::messages::{
    SdoReadResponse, SdoWriteResponse, SlaveState, StopCaptureResponse, ERR_SDO_TRANSPORT,
    ERR_SDO_UNSUPPORTED_SIZE,
};

static SIGTERM_RECEIVED: AtomicBool = AtomicBool::new(false);

/// Below the DC thread (default 80) so the cycle always preempts mailbox
/// work, and below Linux threaded-IRQ handlers (50) so NIC frame delivery
/// preempts the master's receive busy-poll.
const MAILBOX_RT_PRIO: i32 = 40;

extern "C" fn on_sigterm(_: libc::c_int) {
    SIGTERM_RECEIVED.store(true, Ordering::Release);
}

fn arg_val(args: &[String], key: &str) -> Option<String> {
    args.iter()
        .position(|a| a == key)
        .and_then(|i| args.get(i + 1).cloned())
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

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let ifname = args.get(1).cloned().unwrap_or_else(|| "eth0".into());
    let socket = arg_val(&args, "--socket").unwrap_or_else(|| "/tmp/kalico-ethercat.sock".into());
    let cycle_us: i64 = arg_val(&args, "--cycle-us")
        .and_then(|s| s.parse().ok())
        .unwrap_or(1000);
    let slaves = parse_slaves(&args).unwrap_or_else(|e| {
        eprintln!("ec-rt: bad --slave config: {e}");
        std::process::exit(1);
    });
    let num_slaves = slaves.len();
    let counts_per_mm: Vec<f64> = slaves.iter().map(|s| s.counts_per_mm).collect();
    let rotation_distance: Vec<f64> = slaves.iter().map(|s| s.rotation_distance).collect();
    let slave_positions: Vec<i32> = slaves.iter().map(|s| s.pos).collect();
    let slave_axes: Vec<u8> = slaves.iter().map(|s| s.axis).collect();
    let rt_cpu: i32 = arg_val(&args, "--rt-cpu")
        .and_then(|s| s.parse().ok())
        .unwrap_or(3);
    let rt_prio: i32 = arg_val(&args, "--rt-prio")
        .and_then(|s| s.parse().ok())
        .unwrap_or(80);
    let mailbox_cpu: usize = arg_val(&args, "--mailbox-cpu")
        .and_then(|s| s.parse().ok())
        .unwrap_or(2);
    let velocity_ff = args.iter().any(|a| a == "--velocity-ff");
    let torque_clamp_tenths: i16 = arg_val(&args, "--torque-clamp-pct")
        .and_then(|s| s.parse::<f64>().ok())
        .map(|pct| {
            if !(pct > 0.0 && pct <= 400.0) {
                eprintln!("ec-rt: --torque-clamp-pct {pct} outside (0, 400]");
                std::process::exit(1);
            }
            (pct * 10.0) as i16
        })
        .unwrap_or(300);
    let dynamics = arg_val(&args, "--dynamics-profile").map(|path| {
        let text = std::fs::read_to_string(&path).unwrap_or_else(|e| {
            eprintln!("ec-rt: dynamics profile {path}: {e}");
            std::process::exit(1);
        });
        let model = DynamicsModel::from_toml_str(&text).unwrap_or_else(|e| {
            eprintln!("ec-rt: dynamics profile {path} invalid: {e:?}");
            std::process::exit(1);
        });
        if model.n != num_slaves {
            eprintln!(
                "ec-rt: dynamics profile {path} has {} axes, endpoint drives {num_slaves}",
                model.n
            );
            std::process::exit(1);
        }
        model
    });
    let cycle_ns = cycle_us * 1000;
    let telemetry_period = u64::try_from(cycle_us)
        .map(|u| (500_000u64 / u).max(1))
        .unwrap_or(500);

    let mut rings: Vec<AxisRing> = (0..num_slaves).map(AxisRing::with_slot).collect();
    let mut buzz = BuzzOsc::new();
    let mut cmaps: Vec<Option<CountMap>> = (0..num_slaves).map(|_| None).collect();
    let mut framed = false;
    let mut seed_home_inflight: Option<u32> = None;
    let mut seed_home_slot: u8 = 0;
    let mut seed_home_homing_rc: i32 = 0;
    let mut last_sent_retired: u32 = 0;
    let mut heartbeat_sent = false;

    let mut server = FrameServer::bind(&socket).expect("bind socket");
    eprintln!(
        "ec-rt: socket {socket}, cycle {cycle_us}us, {num_slaves} slave(s) \
         positions={slave_positions:?} counts/mm={counts_per_mm:?} \
         rotation_distance={rotation_distance:?} velocity_ff={velocity_ff} \
         dynamics={} clamp={torque_clamp_tenths}",
        dynamics.is_some()
    );

    unsafe {
        let mut sa: libc::sigaction = std::mem::zeroed();
        sa.sa_sigaction = on_sigterm as *const () as libc::sighandler_t;
        sa.sa_flags = libc::SA_RESTART;
        libc::sigaction(libc::SIGTERM, &sa, std::ptr::null_mut());
    }

    fn bringup_fail(server: &mut FrameServer, rc: i32) -> ! {
        eprintln!("ec-rt: bringup failed rc={rc}, sending handshake-fail then exiting");
        let claim_deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
        if let Some(cid) = wait_for_claim(server, claim_deadline, &SIGTERM_RECEIVED, "ec-rt") {
            let reply = single_slave_reply(
                1,
                SlaveState::Offline,
                u16::try_from(rc.unsigned_abs()).unwrap_or(u16::MAX),
            );
            server.respond_and_close(&claim_handshake_reply_frame(cid, &reply));
            eprintln!("ec-rt: sent offline handshake reply, exiting");
        } else {
            eprintln!("ec-rt: bridge did not send ClaimHandshake within 5 s; aborting");
        }
        std::process::exit(1);
    }

    let cif = CString::new(ifname.clone()).expect("ifname must not contain NUL");
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
    if rc != 0 {
        bringup_fail(&mut server, rc);
    }

    let run_limits: Vec<(u32, u16)> = (0..num_slaves)
        .map(|s| {
            let slot = s as std::os::raw::c_int;
            let mut ferr = 0u32;
            let mut tmo = 0u16;
            let mut tq = 0u16;
            let rc = unsafe { ffi::ec_rt_read_limits(slot, &mut ferr, &mut tmo, &mut tq) };
            if rc != 0 {
                eprintln!(
                    "ec-rt: slot {s}: SDO read of protection limits failed rc={rc} — aborting bringup"
                );
                unsafe { ffi::ec_rt_shutdown() };
                std::process::exit(1);
            }
            eprintln!(
                "ec-rt: slot {s} drive limits at bringup: 6065h={ferr} counts, 6066h={tmo} ms, 6072h={tq} (0.1%)"
            );
            let cli_ferr = slaves[s].following_error_counts;
            let cli_tq = slaves[s].max_torque_tenth_pct;
            let run = (cli_ferr.unwrap_or(ferr), cli_tq.unwrap_or(tq));
            if cli_ferr.is_some() || cli_tq.is_some() {
                let rc = unsafe { ffi::ec_rt_write_limits(slot, run.0, run.1) };
                if rc != 0 {
                    eprintln!(
                        "ec-rt: slot {s}: SDO write of session limits failed rc={rc} — aborting bringup"
                    );
                    unsafe { ffi::ec_rt_shutdown() };
                    std::process::exit(1);
                }
                eprintln!("ec-rt: slot {s} session limits applied: 6065h={} 6072h={}", run.0, run.1);
            }
            run
        })
        .collect();

    let rc = unsafe { ffi::ec_rt_bringup_finish() };
    if rc != 0 {
        bringup_fail(&mut server, rc);
    }
    eprintln!("ec-rt: drives parked (Ready-to-Switch-On, no torque)");

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
            eprintln!("ec-rt: bridge did not send ClaimHandshake within 5 s; aborting");
            unsafe {
                for s in 0..num_slaves {
                    ffi::ec_rt_disable(s as std::os::raw::c_int);
                }
                ffi::ec_rt_shutdown();
            }
            std::process::exit(1);
        }
    }
    eprintln!("ec-rt: handshake ok, entering DC loop");

    let mut gate = TorqueGate::new();
    let mut capture = Capture::new();
    let mut cycle_index: u64 = 0;
    let mailbox = MailboxWorker::spawn(
        FfiSdoBus,
        |slot, ferr_counts, torque_tenth_pct| unsafe {
            ffi::ec_rt_write_limits(i32::from(slot), ferr_counts, torque_tenth_pct)
        },
        WorkerScheduling::RealtimeCompanion {
            cpu: mailbox_cpu,
            priority: MAILBOX_RT_PRIO,
        },
    );
    let mut pending_starts: Vec<(u32, String, PendingStart)> = Vec::new();
    let mut pending_stops: Vec<(u32, PendingStop)> = Vec::new();
    let mut prdiv = 0u64;
    let mut ff_saturation = 0u32;
    let mut wkc_consecutive = 0u8;
    let mut latched_drive_err: u16 = 0;
    let mut sensorless_arm: Option<SensorlessArm> = None;
    'dc: loop {
        if SIGTERM_RECEIVED.load(Ordering::Acquire) {
            eprintln!("ec-rt: SIGTERM received — disabling drive and exiting");
            break;
        }
        if server.session_ended() {
            eprintln!("ec-rt: bridge disconnected — disabling drive and exiting");
            break;
        }

        for cmd in server.poll_commands() {
            match cmd {
                Command::Identify {
                    correlation_id,
                    proto_version,
                } => {
                    server.respond(&identify_response_frame(correlation_id, proto_version));
                }
                Command::PushPieces {
                    correlation_id,
                    msg,
                } => {
                    let now_ns = monotonic_ns();
                    let diags: Vec<(u8, u64)> =
                        msg.axes.iter().map(|a| (a.axis_idx, 0u64)).collect();
                    if gate.state() == TorqueState::Faulted {
                        server.respond(&push_pieces_response_frame_multi(
                            correlation_id,
                            ERR_PIECES_WHILE_FAULTED,
                            now_ns,
                            &diags,
                        ));
                    } else {
                        let mut result = 0i32;
                        let mut diags: Vec<(u8, u64)> = Vec::with_capacity(msg.axes.len());
                        for axis in &msg.axes {
                            let front_start_time =
                                if axis.piece_count > 0 && axis.pieces_bytes.len() >= 8 {
                                    u64::from_le_bytes(
                                        axis.pieces_bytes[0..8].try_into().unwrap_or([0; 8]),
                                    )
                                } else {
                                    0
                                };
                            diags.push((axis.axis_idx, front_start_time));
                            // Single-drive: route to the one ring regardless of the
                            // host's (global) axis_idx, matching pre-multi behaviour
                            // where an EtherCAT node could serve any one axis (X/Y/Z).
                            // Multi-drive: map the global axis_idx to its slave slot
                            // via the per-slave --axis the host passed at claim.
                            let slot = if rings.len() == 1 {
                                Some(0)
                            } else {
                                slave_axes.iter().position(|&a| a == axis.axis_idx)
                            };
                            let Some(slot) = slot.filter(|&s| s < rings.len()) else {
                                eprintln!(
                                    "ec-rt: PushPieces for axis {} not mapped to any of {} slave(s)",
                                    axis.axis_idx,
                                    rings.len()
                                );
                                result = -309;
                                continue;
                            };
                            let pushed =
                                rings[slot].push_from_bytes(axis.piece_count, &axis.pieces_bytes);
                            if pushed != axis.piece_count {
                                result = -309;
                            }
                        }
                        server.respond(&push_pieces_response_frame_multi(
                            correlation_id,
                            result,
                            now_ns,
                            &diags,
                        ));
                    }
                }
                Command::QueryRuntimeCaps { correlation_id } => {
                    let total: u32 = (AXIS_RING_CAPACITY * num_slaves * 32) as u32;
                    server.respond(&runtime_caps_response_frame(correlation_id, total));
                }
                Command::SetTorque {
                    correlation_id,
                    msg,
                } => match gate.on_set_torque(msg.value != 0, msg.execute_at_ns) {
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
                        gate.enable_finished(enable_rc == 0);
                        if enable_rc == 0 {
                            eprintln!("ec-rt: torque enabled (CiA402 operation enabled, {num_slaves} slave(s))");
                            server.respond(&set_torque_response_frame(correlation_id, 0));
                        } else {
                            eprintln!(
                                "ec-rt: CiA402 enable failed rc={enable_rc} — disabling and exiting"
                            );
                            server.respond(&set_torque_response_frame(
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
                        server.respond(&set_torque_response_frame(correlation_id, 0));
                    }
                    CommandAction::Reject { code } => {
                        eprintln!(
                            "ec-rt: SetTorque rejected code={code} \
                                 (value={} execute_at={} now={}) — exiting",
                            msg.value,
                            msg.execute_at_ns,
                            monotonic_ns()
                        );
                        server.respond(&set_torque_response_frame(correlation_id, code));
                        unsafe {
                            for s in 0..num_slaves {
                                ffi::ec_rt_disable(s as std::os::raw::c_int);
                            }
                            ffi::ec_rt_shutdown();
                        }
                        std::process::exit(1);
                    }
                },
                Command::Stop { correlation_id } => {
                    let now_ns = monotonic_ns();
                    for r in &mut rings {
                        r.reset();
                    }
                    for c in &mut cmaps {
                        *c = None;
                    }
                    eprintln!("ec-rt: Stop — rings discarded, discard_clock={now_ns}");
                    server.respond(&stop_response_frame(correlation_id, 0, now_ns));
                }
                Command::StartCapture {
                    correlation_id,
                    msg,
                } => {
                    let pending = capture.start_async(CaptureConfig {
                        path: msg.path.clone(),
                        started_utc: msg.started_utc.clone(),
                        drive_name: msg.drive_name.clone(),
                        cycle_ns,
                        counts_per_mm: counts_per_mm[0],
                        started_mono_ns: monotonic_ns(),
                    });
                    pending_starts.push((correlation_id, msg.path, pending));
                }
                Command::StopCapture { correlation_id } => {
                    pending_stops.push((correlation_id, capture.stop_async()));
                }
                Command::ResumeStream { correlation_id } => {
                    server.respond(&resume_stream_response_frame(correlation_id, 0));
                }
                Command::ClaimHandshake { .. } => {
                    eprintln!(
                        "ec-rt: protocol violation: ClaimHandshake after handshake \
                         — ending session"
                    );
                    break 'dc;
                }
                Command::SetDriveLimits {
                    correlation_id,
                    msg,
                } => {
                    if msg.slot as usize >= num_slaves {
                        eprintln!(
                            "ec-rt: SetDriveLimits for slot {} but only {num_slaves} slave(s)",
                            msg.slot
                        );
                        server.respond(&set_drive_limits_response_frame(correlation_id, -309));
                    } else {
                        mailbox.submit(MailboxRequest::WriteLimits {
                            correlation_id,
                            slot: msg.slot,
                            ferr_counts: msg.following_error_counts,
                            torque_tenth_pct: msg.max_torque_tenth_pct,
                            restore: false,
                        });
                    }
                }
                Command::RestoreDriveLimits {
                    correlation_id,
                    slot,
                } => match run_limits.get(slot as usize) {
                    Some(&(ferr_counts, torque_tenth_pct)) => {
                        mailbox.submit(MailboxRequest::WriteLimits {
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
                            run_limits.len()
                        );
                        server.respond(&restore_drive_limits_response_frame(correlation_id, -309));
                    }
                },
                Command::SeedServoHome {
                    correlation_id,
                    slot,
                    home_q16,
                } => {
                    if slot as usize >= counts_per_mm.len() {
                        eprintln!(
                            "ec-rt: SeedServoHome for slot {slot} but only {} slave(s)",
                            counts_per_mm.len()
                        );
                        server.respond(&seed_servo_home_response_frame(correlation_id, -309));
                    } else if seed_home_inflight.is_some() {
                        eprintln!("ec-rt: SeedServoHome rejected — handshake already in flight");
                        server.respond(&seed_servo_home_response_frame(
                            correlation_id,
                            ERR_SEED_HOME_BUSY,
                        ));
                    } else if gate.state() != TorqueState::Enabled {
                        eprintln!(
                            "ec-rt: SeedServoHome rejected — drive not operation-enabled \
                             (state={:?}); method-35 needs torque on",
                            gate.state()
                        );
                        server.respond(&seed_servo_home_response_frame(
                            correlation_id,
                            ERR_SEED_HOME_NOT_ENABLED,
                        ));
                    } else if rings.iter().any(|r| !r.is_empty()) {
                        eprintln!("ec-rt: SeedServoHome rejected — motion ring not empty");
                        server.respond(&seed_servo_home_response_frame(
                            correlation_id,
                            ERR_SEED_HOME_STREAMING,
                        ));
                    } else {
                        let offset_counts = ((f64::from(home_q16) / 65536.0)
                            * counts_per_mm[slot as usize])
                            .round() as i32;
                        eprintln!(
                            "ec-rt: SeedServoHome slot={slot} home_q16={home_q16} \
                             -> 607Ch={offset_counts} counts; staging method-35 mode switch"
                        );
                        seed_home_inflight = Some(correlation_id);
                        seed_home_slot = slot;
                        mailbox.submit(MailboxRequest::SeedHomeSetup {
                            correlation_id,
                            slot,
                            offset_counts,
                        });
                    }
                }
                Command::ArmSensorlessEndstop {
                    correlation_id,
                    msg,
                } => {
                    let result = if msg.enable != 0 {
                        if msg.slot as usize >= num_slaves {
                            eprintln!(
                                "ec-rt: ArmSensorlessEndstop for slot {} but only {num_slaves} slave(s)",
                                msg.slot
                            );
                            -309
                        } else if msg.torque_trip_tenth_pct == 0 {
                            eprintln!(
                                "ec-rt: ArmSensorlessEndstop rejected — zero torque trip threshold"
                            );
                            ERR_ARM_SENSORLESS_BAD_THRESHOLD
                        } else {
                            sensorless_arm = Some(SensorlessArm::new(
                                msg.slot,
                                msg.endstop_id,
                                msg.torque_trip_tenth_pct,
                            ));
                            eprintln!(
                                "ec-rt: sensorless endstop {} armed (torque_trip={} 0.1%)",
                                msg.endstop_id, msg.torque_trip_tenth_pct
                            );
                            0
                        }
                    } else {
                        sensorless_arm = None;
                        eprintln!("ec-rt: sensorless endstop {} disarmed", msg.endstop_id);
                        0
                    };
                    server.respond(&arm_sensorless_endstop_response_frame(
                        correlation_id,
                        result,
                    ));
                }
                Command::ResonanceBuzz {
                    correlation_id,
                    msg,
                } => {
                    let rc = if gate.state() != TorqueState::Enabled {
                        eprintln!("ec-rt: ResonanceBuzz rejected — drive not operation-enabled");
                        ethercat_rt::buzz::ERR_BUZZ_NOT_ENABLED
                    } else if rings.iter().any(|r| !r.is_empty()) || buzz.active() {
                        eprintln!("ec-rt: ResonanceBuzz rejected — motion in progress");
                        if buzz.active() {
                            ethercat_rt::buzz::ERR_BUZZ_BUSY
                        } else {
                            ethercat_rt::buzz::ERR_BUZZ_STREAMING
                        }
                    } else {
                        let base_counts = if framed {
                            unsafe { ffi::ec_rt_get_position_actual(0) }
                        } else {
                            0
                        };
                        let rc = buzz.arm(
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
                    server.respond(&resonance_buzz_response_frame(correlation_id, rc));
                }
                Command::SdoRead {
                    correlation_id,
                    msg,
                } => {
                    if msg.slot as usize >= num_slaves {
                        eprintln!(
                            "ec-rt: SdoRead for slot {} but only {num_slaves} slave(s)",
                            msg.slot
                        );
                        server.respond(&sdo_read_response_frame(
                            correlation_id,
                            &SdoReadResponse {
                                result: -309,
                                size: 0,
                                data: [0; 4],
                            },
                        ));
                    } else {
                        mailbox.submit(MailboxRequest::SdoRead {
                            correlation_id,
                            msg,
                        });
                    }
                }
                Command::SdoWrite {
                    correlation_id,
                    msg,
                } => {
                    if msg.slot as usize >= num_slaves {
                        eprintln!(
                            "ec-rt: SdoWrite for slot {} but only {num_slaves} slave(s)",
                            msg.slot
                        );
                        server.respond(&sdo_write_response_frame(
                            correlation_id,
                            &SdoWriteResponse {
                                result: -309,
                                readback_size: 0,
                                readback_data: [0; 4],
                            },
                        ));
                    } else {
                        mailbox.submit(MailboxRequest::SdoWrite {
                            correlation_id,
                            msg,
                        });
                    }
                }
                Command::QueryMotorState { correlation_id } => {
                    if framed {
                        let samples: Vec<(u8, f64, f64)> = (0..num_slaves)
                            .map(|s| {
                                let slot = s as std::os::raw::c_int;
                                let (pos_counts, vel_rpm) = unsafe {
                                    (
                                        ffi::ec_rt_get_position_actual(slot),
                                        ffi::ec_rt_get_velocity_actual(slot),
                                    )
                                };
                                let pos_mm = f64::from(pos_counts) / counts_per_mm[s];
                                let vel_mm_s = ethercat_rt::scale::velocity_mm_s(
                                    vel_rpm,
                                    rotation_distance[s],
                                );
                                (s as u8, pos_mm, vel_mm_s)
                            })
                            .collect();
                        server.respond(&motor_state_response_frame_multi(correlation_id, &samples));
                    } else {
                        server.respond(&motor_state_empty_frame(correlation_id));
                    }
                }
                Command::Unknown { kind_raw, .. } => {
                    eprintln!("ec-rt: ignoring kind 0x{kind_raw:04x}");
                }
            }
        }

        let mut start_idx = 0;
        while start_idx < pending_starts.len() {
            match pending_starts[start_idx].2.try_take() {
                Some(rc) => {
                    let (correlation_id, path, pending) = pending_starts.remove(start_idx);
                    if rc != 0 && pending.claimed() {
                        capture.clear_failed_start();
                    }
                    eprintln!("ec-rt: StartCapture path={path} rc={rc}");
                    server.respond(&start_capture_response_frame(correlation_id, rc));
                }
                None => start_idx += 1,
            }
        }

        let mut stop_idx = 0;
        while stop_idx < pending_stops.len() {
            match pending_stops[stop_idx].1.try_take() {
                Some(out) => {
                    let (correlation_id, _) = pending_stops.remove(stop_idx);
                    eprintln!(
                        "ec-rt: StopCapture result={} samples={} overflow={:?}",
                        out.result, out.samples, out.overflow_cycle
                    );
                    server.respond(&stop_capture_response_frame(
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

        while let Some(reply) = mailbox.try_recv() {
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
                    server.respond(&sdo_read_response_frame(correlation_id, &resp));
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
                    server.respond(&sdo_write_response_frame(correlation_id, &resp));
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
                    server.respond(&frame);
                }
                MailboxReply::SeedHomeSetup {
                    correlation_id,
                    rc,
                    offset_counts,
                } => {
                    seed_home_homing_rc = if rc != 0 {
                        eprintln!(
                            "ec-rt: SeedServoHome setup failed rc={rc} (offset={offset_counts}); \
                             restoring CSP and failing"
                        );
                        rc
                    } else {
                        let hrc = unsafe { ffi::ec_rt_run_homing(i32::from(seed_home_slot)) };
                        if hrc != 0 {
                            eprintln!(
                                "ec-rt: SeedServoHome controlword phase failed rc={hrc}; \
                                 restoring CSP and failing"
                            );
                        } else {
                            eprintln!(
                                "ec-rt: SeedServoHome homing attained (607Ch={offset_counts}); \
                                 restoring CSP"
                            );
                        }
                        hrc
                    };
                    mailbox.submit(MailboxRequest::SeedHomeRestore {
                        correlation_id,
                        slot: seed_home_slot,
                    });
                }
                MailboxReply::SeedHomeRestore { correlation_id, rc } => {
                    seed_home_inflight = None;
                    let result = if seed_home_homing_rc != 0 {
                        seed_home_homing_rc
                    } else if rc != 0 {
                        eprintln!("ec-rt: SeedServoHome CSP restore failed rc={rc}");
                        ERR_SEED_HOME_RESTORE
                    } else {
                        framed = true;
                        eprintln!("ec-rt: SeedServoHome complete — framed=true");
                        0
                    };
                    seed_home_homing_rc = 0;
                    server.respond(&seed_servo_home_response_frame(correlation_id, result));
                }
            }
        }

        let now = monotonic_ns();

        let all_rings_empty = rings.iter().all(|r| r.is_empty());
        match gate.on_tick(now, all_rings_empty) {
            TickAction::None => {}
            TickAction::ExecuteDisable => {
                eprintln!("ec-rt: scheduled torque disable executing");
                unsafe {
                    for s in 0..num_slaves {
                        ffi::ec_rt_disable(s as std::os::raw::c_int);
                    }
                }
                gate.disable_finished();
                for c in &mut cmaps {
                    *c = None;
                }
            }
            TickAction::Fault { code } => {
                eprintln!(
                    "ec-rt: torque-gate fault code={code} — pieces present without torque, exiting"
                );
                let retired: Vec<u32> = rings.iter().map(|r| r.retired_count()).collect();
                server.respond(&status_heartbeat_frame(
                    ENGINE_STATE_FAULT,
                    0,
                    &retired,
                    ff_saturation,
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

        if let Some(arm_slot) = sensorless_arm.as_ref().map(SensorlessArm::slot) {
            let torque_actual = unsafe { ffi::ec_rt_get_torque_actual(i32::from(arm_slot)) };
            if let Some(endstop_id) = sensorless_arm
                .as_mut()
                .and_then(|arm| arm.poll(torque_actual))
            {
                for r in &mut rings {
                    r.reset();
                }
                for c in &mut cmaps {
                    *c = None;
                }
                eprintln!(
                    "ec-rt: sensorless endstop {endstop_id} tripped torque={torque_actual} \
                     — local stop, trip_clock={now}"
                );
                server.respond(&endstop_trip_frame(endstop_id, now));
            }
        }

        let mut motion_active = false;
        if gate.state() == TorqueState::Enabled {
            // The coupled torque model needs every axis' accel/vel before any
            // one slot's feedforward can be computed, so sample all slots first.
            let mut sp_counts: Vec<Option<i32>> = vec![None; num_slaves];
            let mut all_acc = vec![0f32; num_slaves];
            let mut all_vel = vec![0f32; num_slaves];
            for s in 0..num_slaves {
                let sampled = if buzz.active() {
                    if s == 0 {
                        buzz.eval(now).map(|(rel_mm, vel_mm_s, acc_mm_s2)| {
                            let counts = buzz
                                .base_counts()
                                .wrapping_add(mm_to_counts(f64::from(rel_mm), counts_per_mm[0]));
                            (counts, vel_mm_s, acc_mm_s2)
                        })
                    } else {
                        None
                    }
                } else if let Some((pos_mm, vel_mm_s, acc_mm_s2)) = rings[s].sample(now) {
                    let counts = if framed {
                        mm_to_counts(f64::from(pos_mm), counts_per_mm[s])
                    } else {
                        let cpm = counts_per_mm[s];
                        let map = cmaps[s].get_or_insert_with(|| {
                            let actual =
                                unsafe { ffi::ec_rt_get_position_actual(s as std::os::raw::c_int) };
                            CountMap::new(cpm, actual, f64::from(pos_mm))
                        });
                        map.target_counts(f64::from(pos_mm))
                    };
                    Some((counts, vel_mm_s, acc_mm_s2))
                } else {
                    if !buzz.active() {
                        cmaps[s] = None;
                    }
                    None
                };
                if let Some((counts, vel_mm_s, acc_mm_s2)) = sampled {
                    sp_counts[s] = Some(counts);
                    all_vel[s] = vel_mm_s;
                    all_acc[s] = acc_mm_s2;
                }
            }

            for s in 0..num_slaves {
                let slot = s as std::os::raw::c_int;
                if let Some(counts) = sp_counts[s] {
                    let vel_offset = if velocity_ff {
                        (f64::from(all_vel[s]) * counts_per_mm[s]).round() as i32
                    } else {
                        0
                    };
                    let torque_offset = match &dynamics {
                        Some(model) => {
                            let raw = model.torque_ff(s, &all_acc, &all_vel);
                            if !raw.is_finite() {
                                eprintln!(
                                    "ec-rt: FAULT non-finite torque FF on slot {s} \
                                     (acc={} vel={}) — disabling",
                                    all_acc[s], all_vel[s]
                                );
                                let retired: Vec<u32> =
                                    rings.iter().map(|r| r.retired_count()).collect();
                                server.respond(&status_heartbeat_frame(
                                    ENGINE_STATE_FAULT,
                                    0,
                                    &retired,
                                    ff_saturation,
                                ));
                                unsafe {
                                    for d in 0..num_slaves {
                                        ffi::ec_rt_disable(d as std::os::raw::c_int);
                                    }
                                    ffi::ec_rt_shutdown();
                                }
                                std::process::exit(1);
                            }
                            clamp_torque(raw, torque_clamp_tenths, &mut ff_saturation)
                        }
                        None => 0,
                    };
                    unsafe {
                        ffi::ec_rt_set_target_position(slot, counts);
                        ffi::ec_rt_set_velocity_offset(slot, vel_offset);
                        ffi::ec_rt_set_torque_offset(slot, torque_offset);
                    }
                    motion_active = true;
                } else {
                    unsafe {
                        ffi::ec_rt_set_velocity_offset(slot, 0);
                        ffi::ec_rt_set_torque_offset(slot, 0);
                    }
                }
            }
        }

        let ring_fault = rings
            .iter()
            .enumerate()
            .find_map(|(s, r)| r.take_fault().map(|f| (s, f)));
        if let Some((slot, fault_val)) = ring_fault {
            let fault_code_u16 = (fault_val & 0xFFFF) as u16;
            eprintln!(
                "ec-rt: FAULT latched on slot {slot} fault_val=0x{fault_val:08x} \
                 code=0x{fault_code_u16:04x} — notifying host via heartbeat"
            );
            let retired: Vec<u32> = rings.iter().map(|r| r.retired_count()).collect();
            #[cfg(not(feature = "hw"))]
            let current_retired: u32 = retired.iter().sum();
            server.respond(&status_heartbeat_frame(
                ENGINE_STATE_FAULT,
                fault_code_u16,
                &retired,
                ff_saturation,
            ));

            #[cfg(feature = "hw")]
            {
                eprintln!("ec-rt: disabling drives (hw safety backstop)");
                unsafe {
                    for s in 0..num_slaves {
                        ffi::ec_rt_disable(s as std::os::raw::c_int);
                    }
                    ffi::ec_rt_shutdown();
                }
                std::process::exit(1);
            }

            #[cfg(not(feature = "hw"))]
            {
                last_sent_retired = current_retired;
                heartbeat_sent = true;
            }
        }

        let mut toff = 0i64;
        let wkc = unsafe { ffi::ec_rt_cycle(&mut toff) };

        let drive_fault = (0..num_slaves).find_map(|s| {
            let e = unsafe { ffi::ec_rt_get_error_code(s as std::os::raw::c_int) };
            if e != 0 {
                Some((s, e))
            } else {
                None
            }
        });
        let drive_err: u16 = drive_fault.map(|(_, e)| e).unwrap_or(0);
        if let Some((slot, err)) = drive_fault {
            if gate.state() != TorqueState::Faulted {
                eprintln!(
                    "ec-rt: DRIVE FAULT slot {slot} err=0x{err:04x} — parking, reporting via heartbeat"
                );
                gate.on_drive_fault();
                for r in &mut rings {
                    r.reset();
                }
                for c in &mut cmaps {
                    *c = None;
                }
                latched_drive_err = err;
                let retired: Vec<u32> = rings.iter().map(|r| r.retired_count()).collect();
                server.respond(&status_heartbeat_frame(0, err, &retired, ff_saturation));
                last_sent_retired = retired.iter().sum();
                heartbeat_sent = true;
            }
        }

        cycle_index += 1;
        if capture.is_active() {
            let mut t = ffi::EcTelemetry::default();
            unsafe { ffi::ec_rt_get_telemetry(0, &mut t) };
            let mut flags = 0u8;
            if gate.state() == TorqueState::Enabled {
                flags |= FLAG_TORQUE_ENABLED;
            }
            if motion_active {
                flags |= FLAG_MOTION_ACTIVE;
            }
            capture.push(CaptureRecord {
                cycle_index,
                flags,
                drive: DriveSample {
                    target_counts: t.target_position,
                    position_actual: t.position_actual,
                    velocity_actual: t.velocity_actual,
                    following_error: t.following_error,
                    torque_actual: t.torque_actual,
                    statusword: t.statusword,
                    error_code: t.error_code,
                    velocity_offset: t.velocity_offset,
                    torque_offset: t.torque_offset,
                },
            });
        }

        let expected_wkc = 3 * num_slaves as i32;
        match eval_wkc(wkc, expected_wkc, &mut wkc_consecutive) {
            WkcDecision::Good => {}
            WkcDecision::Warn(n) => {
                eprintln!(
                    "ec-rt: WARNING — working counter {wkc} (expected {expected_wkc}), \
                     consecutive_bad={n}; tolerating (USB-NIC frame loss); \
                     halt threshold={}",
                    ethercat_rt::claim::WKC_CONSECUTIVE_LOSS_LIMIT
                );
            }
            WkcDecision::Halt => {
                let mut al_state = 0u16;
                let mut al_code = 0u16;
                unsafe { ffi::ec_rt_al_status(0, &mut al_state, &mut al_code) };
                eprintln!(
                    "ec-rt: working counter {wkc} (expected {expected_wkc}), \
                     consecutive_bad={wkc_consecutive} — bus lost after \
                     {} consecutive bad cycles, halting \
                     (slot 0 AL state=0x{al_state:02x} status_code=0x{al_code:04x}; \
                     0x001b=SM watchdog, 0x001a/0x002c/0x0030=DC sync)",
                    ethercat_rt::claim::WKC_CONSECUTIVE_LOSS_LIMIT
                );
                unsafe { ffi::ec_rt_dump_al_state() };
                break;
            }
        }

        let retired: Vec<u32> = rings.iter().map(|r| r.retired_count()).collect();
        // Sum is monotonic and changes whenever any slot retires a piece, so it
        // triggers an emit even when a slower axis advances behind the leader.
        let current_retired: u32 = retired.iter().sum();
        let all_empty_now = rings.iter().all(|r| r.is_empty());
        let should_emit = !heartbeat_sent || current_retired != last_sent_retired;
        if should_emit {
            let engine_state: u8 = if all_empty_now { 0 } else { 1 };
            server.respond(&status_heartbeat_frame(
                engine_state,
                0,
                &retired,
                ff_saturation,
            ));
            last_sent_retired = current_retired;
            heartbeat_sent = true;
            if current_retired != 0 {
                eprintln!("ec-rt: heartbeat retired={retired:?}");
            }
        }

        prdiv += 1;
        if prdiv >= telemetry_period {
            prdiv = 0;
            let (sw, pos, ferr, tq_act) = unsafe {
                (
                    ffi::ec_rt_get_statusword(0),
                    ffi::ec_rt_get_position_actual(0),
                    ffi::ec_rt_get_following_error(0),
                    ffi::ec_rt_get_torque_actual(0),
                )
            };
            eprintln!(
                "ec-rt: wkc={wkc} sw=0x{sw:04x} err=0x{drive_err:04x} pos={pos} ferr={ferr} toff={toff} \
                 any_motion={} retired={retired:?} tq_act={tq_act} ff_sat={ff_saturation} framed={framed}",
                !all_empty_now as u8,
            );
            if gate.state() == TorqueState::Faulted {
                server.respond(&status_heartbeat_frame(
                    0,
                    latched_drive_err,
                    &retired,
                    ff_saturation,
                ));
            }
        }
    }

    unsafe {
        for s in 0..num_slaves {
            ffi::ec_rt_disable(s as std::os::raw::c_int);
        }
        ffi::ec_rt_shutdown();
    }
    eprintln!("ec-rt: shutdown complete");
}
