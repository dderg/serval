#![allow(unsafe_code)]

use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::sleep;
use std::time::Duration;

use ethercat_rt::capture::{
    Capture, CaptureConfig, CaptureDriveConfig, CaptureRecord, DriveSample, FLAG_MOTION_ACTIVE,
    FLAG_TORQUE_ENABLED,
};
use ethercat_rt::claim::{parse_fail_bringup, single_slave_reply, wait_for_claim};
use ethercat_rt::clock::monotonic_ns;
use ethercat_rt::sdo::{execute_sdo_read, execute_sdo_write, DictObject, DictSdoBus};
use ethercat_rt::sensorless::{SensorlessBank, ERR_ARM_SENSORLESS_BAD_THRESHOLD};
use ethercat_rt::server::FrameServer;
use ethercat_rt::setpoint::{
    Played, RunHeader, SetpointEntry, SetpointRing, EXECUTOR_SETPOINT_RING, RING_DEPTH_CYCLES,
};
use ethercat_rt::stream_halt::StreamHalt;
use ethercat_rt::torque::{
    CommandAction, TickAction, TorqueGate, TorqueState, ERR_ENABLE_FAILED, ERR_PIECES_WHILE_FAULTED,
};
use ethercat_rt::wire::{
    arm_sensorless_endstop_response_frame, claim_handshake_reply_frame, endstop_trip_frame,
    identify_response_frame, push_sample_runs_response_frame, resonance_buzz_response_frame,
    restore_drive_limits_response_frame, resume_stream_response_frame, runtime_caps_response_frame,
    sample_grid_response_frame, sdo_read_response_frame, sdo_write_response_frame,
    seed_servo_home_response_frame, set_diff_damper_response_frame, set_diff_trim_response_frame,
    set_drive_limits_response_frame, set_dynamics_model_response_frame, set_ff_lead_response_frame,
    set_strain_comp_response_frame, set_torque_response_frame, start_capture_response_frame,
    status_heartbeat_frame, stepper_suppress_response_frame, stop_capture_response_frame,
    stop_response_frame, Command, ENGINE_STATE_FAULT,
};
use mcu_protocol::messages::{
    SdoReadResponse, SlaveState, StopCaptureResponse, LANE_RUN_FLAG_REANCHOR, LANE_RUN_FLAG_TAIL,
};

static SIGTERM_RECEIVED: AtomicBool = AtomicBool::new(false);

const STUB_CYCLE_NS: i64 = 1_000_000;
const STUB_COUNTS_PER_MM: f64 = 3_276.8;
const STUB_ROTATION_DISTANCE: f64 = 40.0;

extern "C" fn on_sigterm(_: libc::c_int) {
    SIGTERM_RECEIVED.store(true, Ordering::Release);
}

fn arg_val(args: &[String], key: &str) -> Option<String> {
    args.iter()
        .position(|a| a == key)
        .and_then(|i| args.get(i + 1).cloned())
}

/// The stub's sample grid: indices count whole `STUB_CYCLE_NS` periods from
/// the base latched at loop entry, so the host can address ring cycles the
/// same way it addresses a real endpoint's DC grid.
fn grid_index_now(base_ns: u64) -> u64 {
    (monotonic_ns() - base_ns) / STUB_CYCLE_NS as u64
}

fn grid_now(base_ns: u64) -> (u64, u64) {
    let index = grid_index_now(base_ns);
    (index, base_ns + index * STUB_CYCLE_NS as u64)
}

/// The ring's exclusive playback cursor in the stub grid's clock domain — the
/// same map `grid_now` echoes, so the host retires a run at its `end_clock`.
fn playback_clock(base_ns: u64, ring: &SetpointRing) -> u64 {
    ring.played_cursor()
        .map_or(0, |index| base_ns + index * STUB_CYCLE_NS as u64)
}

const STUB_PROBE_COUNTER_INDEX: u16 = 0x5FFF;
const TXPDO_TORQUE_ACTUAL_INDEX: u16 = 0x6077;

fn stub_object_dictionary() -> DictSdoBus {
    DictSdoBus::new([
        (
            (0x2002, 0),
            DictObject {
                size: 2,
                value: [100, 0, 0, 0],
                read_only: false,
                unsigned_clamp_max: None,
            },
        ),
        (
            (0x2003, 0),
            DictObject {
                size: 2,
                value: [0, 0, 0, 0],
                read_only: false,
                unsigned_clamp_max: Some(500),
            },
        ),
        (
            (0x2010, 1),
            DictObject {
                size: 4,
                value: [0; 4],
                read_only: false,
                unsigned_clamp_max: None,
            },
        ),
        (
            (0x6041, 0),
            DictObject {
                size: 2,
                value: [0x37, 0x02, 0, 0],
                read_only: true,
                unsigned_clamp_max: None,
            },
        ),
        (
            (0x6077, 0),
            DictObject {
                size: 2,
                value: [0, 0, 0, 0],
                read_only: false,
                unsigned_clamp_max: None,
            },
        ),
    ])
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let socket = arg_val(&args, "--socket").unwrap_or_else(|| "/tmp/kalico-ethercat.sock".into());

    let fail_slave: Option<u8> = match parse_fail_bringup(&args) {
        Ok(v) => v,
        Err(msg) => {
            eprintln!("ec-rt-stub: {msg}");
            eprintln!("Usage: ethercat-rt-stub [--socket PATH] [--fail-bringup slave=N]");
            std::process::exit(2);
        }
    };

    let fail_enable = args.iter().any(|a| a == "--fail-enable");
    let drive_fault_after: Option<u32> =
        arg_val(&args, "--drive-fault-after-cycles").and_then(|s| s.parse().ok());

    let mut ring = SetpointRing::new(0, STUB_CYCLE_NS as u32);
    let mut gate = TorqueGate::new();
    let mut capture = Capture::new();
    let mut capture_drive_count: usize = 0;
    let mut cycle_index: u64 = 0;
    let mut sdo_bus = stub_object_dictionary();
    let mut last_sent_retired: u32 = 0;
    let mut heartbeat_sent = false;
    let mut played_cycles: u32 = 0;
    let mut drive_fault_fired = false;
    let mut stored_limits: Option<(u32, u16)> = None;
    let mut sensorless = SensorlessBank::new(1);
    let mut stream_halt = StreamHalt::default();
    let mut suppressed = false;
    let mut sim_torque: i16 = 0;

    let mut server = FrameServer::bind(&socket).expect("bind socket");
    eprintln!("ec-rt-stub: socket {socket} (NO HARDWARE)");

    // SAFETY: on_sigterm only touches a static AtomicBool; SA_RESTART (and no
    // SA_RESETHAND) keeps a second SIGTERM on the clean-shutdown path too.
    unsafe {
        let mut sa: libc::sigaction = std::mem::zeroed();
        sa.sa_sigaction = on_sigterm as *const () as libc::sighandler_t;
        sa.sa_flags = libc::SA_RESTART;
        libc::sigaction(libc::SIGTERM, &sa, std::ptr::null_mut());
    }

    let claim_deadline = std::time::Instant::now() + Duration::from_secs(10);
    let cid = match wait_for_claim(&mut server, claim_deadline, &SIGTERM_RECEIVED, "ec-rt-stub") {
        Some(id) => id,
        None => {
            eprintln!("ec-rt-stub: bridge did not send ClaimHandshake within 10 s; aborting");
            std::process::exit(1);
        }
    };

    if let Some(slave_idx) = fail_slave {
        let reply = single_slave_reply(slave_idx, SlaveState::Offline, 0);
        server.respond_and_close(&claim_handshake_reply_frame(cid, &reply));
        eprintln!("ec-rt-stub: --fail-bringup: sent Offline for slave {slave_idx}, exiting");
        std::process::exit(1);
    }

    server.respond(&claim_handshake_reply_frame(
        cid,
        &single_slave_reply(1, SlaveState::Ok, 0),
    ));
    eprintln!("ec-rt-stub: handshake ok, entering stub loop");
    let grid_base_ns = monotonic_ns();

    'session: loop {
        if SIGTERM_RECEIVED.load(Ordering::Acquire) {
            eprintln!("ec-rt-stub: SIGTERM received — exiting");
            break;
        }
        if server.session_ended() {
            eprintln!("ec-rt-stub: bridge disconnected — exiting");
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
                Command::QueryRuntimeCaps { correlation_id } => {
                    server.respond(&runtime_caps_response_frame(correlation_id));
                }
                Command::QueryMotorState { .. } => {}
                Command::Stop { correlation_id } => {
                    let now_ns = monotonic_ns();
                    ring.reset();
                    stream_halt.halt();
                    eprintln!(
                        "ec-rt-stub: Stop — ring discarded, stream halted, \
                         discard_clock={now_ns}"
                    );
                    server.respond(&stop_response_frame(correlation_id, 0, now_ns));
                }
                Command::ResumeStream { correlation_id } => match stream_halt.resume() {
                    Ok(()) => {
                        ring.reset();
                        suppressed = false;
                        eprintln!("ec-rt-stub: ResumeStream — stream reopened");
                        server.respond(&resume_stream_response_frame(correlation_id, 0));
                    }
                    Err(code) => {
                        eprintln!(
                            "ec-rt-stub: ResumeStream rejected code={code} — stream was \
                             not halted"
                        );
                        server.respond(&resume_stream_response_frame(correlation_id, code));
                    }
                },
                Command::StepperSuppress {
                    correlation_id,
                    msg,
                } => {
                    if msg.motor == 0xFF && msg.stepper == 0xFF && msg.engage == 0 {
                        suppressed = false;
                        eprintln!("ec-rt-stub: StepperSuppress — all slots released");
                    } else {
                        suppressed = msg.engage != 0;
                        eprintln!(
                            "ec-rt-stub: StepperSuppress axis={} stepper={} engage={}",
                            msg.motor, msg.stepper, msg.engage
                        );
                    }
                    server.respond(&stepper_suppress_response_frame(
                        correlation_id,
                        monotonic_ns() as u32,
                    ));
                }
                Command::ClaimHandshake { .. } => {
                    eprintln!(
                        "ec-rt-stub: protocol violation: ClaimHandshake after handshake \
                         — ending session"
                    );
                    break 'session;
                }
                Command::SetTorque {
                    correlation_id,
                    msg,
                } => match gate.on_set_torque(msg.value != 0, msg.execute_at_ns) {
                    CommandAction::Enable => {
                        let ok = !fail_enable;
                        gate.enable_finished(ok);
                        if ok {
                            eprintln!("ec-rt-stub: torque enabled (simulated)");
                            server.respond(&set_torque_response_frame(correlation_id, 0));
                        } else {
                            eprintln!("ec-rt-stub: simulated enable failure — exiting");
                            server.respond(&set_torque_response_frame(
                                correlation_id,
                                ERR_ENABLE_FAILED,
                            ));
                            std::process::exit(1);
                        }
                    }
                    CommandAction::ScheduleDisable => {
                        eprintln!(
                            "ec-rt-stub: torque disable scheduled at {} (now {})",
                            msg.execute_at_ns,
                            monotonic_ns()
                        );
                        server.respond(&set_torque_response_frame(correlation_id, 0));
                    }
                    CommandAction::Reject { code } => {
                        eprintln!("ec-rt-stub: SetTorque rejected code={code} — exiting");
                        server.respond(&set_torque_response_frame(correlation_id, code));
                        std::process::exit(1);
                    }
                },
                Command::SetDriveLimits {
                    correlation_id,
                    msg,
                } => {
                    stored_limits = msg
                        .drives
                        .last()
                        .map(|d| (d.following_error_counts, d.max_torque_tenth_pct));
                    eprintln!("ec-rt-stub: SetDriveLimits drives={:?}", msg.drives);
                    server.respond(&set_drive_limits_response_frame(correlation_id, 0));
                }
                Command::RestoreDriveLimits {
                    correlation_id,
                    slot_mask,
                } => {
                    eprintln!(
                        "ec-rt-stub: RestoreDriveLimits slot_mask={slot_mask:#x} \
                         stored={stored_limits:?}"
                    );
                    server.respond(&restore_drive_limits_response_frame(correlation_id, 0));
                }
                Command::ArmSensorlessEndstop {
                    correlation_id,
                    msg,
                } => {
                    let result = if msg.enable != 0 {
                        if msg.torque_trip_tenth_pct == 0 {
                            ERR_ARM_SENSORLESS_BAD_THRESHOLD
                        } else {
                            sensorless.arm(0, msg.endstop_id, msg.torque_trip_tenth_pct, None);
                            eprintln!(
                                "ec-rt-stub: sensorless endstop {} armed (torque_trip={} 0.1%)",
                                msg.endstop_id, msg.torque_trip_tenth_pct
                            );
                            0
                        }
                    } else {
                        sensorless.disarm(0);
                        eprintln!("ec-rt-stub: sensorless endstop {} disarmed", msg.endstop_id);
                        0
                    };
                    server.respond(&arm_sensorless_endstop_response_frame(
                        correlation_id,
                        result,
                    ));
                }
                Command::SeedServoHome {
                    correlation_id,
                    slot,
                    home_q16,
                } => {
                    eprintln!("ec-rt-stub: SeedServoHome slot={slot} home_q16={home_q16}");
                    server.respond(&seed_servo_home_response_frame(correlation_id, 0));
                }
                Command::ResonanceBuzz {
                    correlation_id,
                    msg,
                } => {
                    eprintln!(
                        "ec-rt-stub: ResonanceBuzz axis_mask=0x{:02x} freq={}->{} mHz",
                        msg.axis_mask, msg.freq_start_millihz, msg.freq_end_millihz,
                    );
                    server.respond(&resonance_buzz_response_frame(correlation_id, 0));
                }
                Command::SdoRead {
                    correlation_id,
                    msg,
                } => {
                    let resp = if msg.index == STUB_PROBE_COUNTER_INDEX {
                        SdoReadResponse {
                            result: 0,
                            size: 4,
                            data: sdo_bus.read_count.to_le_bytes(),
                        }
                    } else {
                        execute_sdo_read(&mut sdo_bus, &msg)
                    };
                    if resp.result != 0 {
                        eprintln!(
                            "ec-rt-stub: SdoRead 0x{:04x}.{} failed result={}",
                            msg.index, msg.subindex, resp.result
                        );
                    }
                    server.respond(&sdo_read_response_frame(correlation_id, &resp));
                }
                Command::SdoWrite {
                    correlation_id,
                    msg,
                } => {
                    let resp = execute_sdo_write(&mut sdo_bus, &msg);
                    if msg.index == TXPDO_TORQUE_ACTUAL_INDEX && resp.result == 0 {
                        sim_torque = i16::try_from(
                            msg.value.clamp(i64::from(i16::MIN), i64::from(i16::MAX)),
                        )
                        .unwrap_or(0);
                    }
                    eprintln!(
                        "ec-rt-stub: SdoWrite 0x{:04x}.{} value={} size={} -> result={}",
                        msg.index, msg.subindex, msg.value, msg.size, resp.result
                    );
                    server.respond(&sdo_write_response_frame(correlation_id, &resp));
                }
                Command::StartCapture {
                    correlation_id,
                    msg,
                } => {
                    let drives: Vec<CaptureDriveConfig> = msg
                        .drives
                        .iter()
                        .map(|d| CaptureDriveConfig {
                            slot: d.slot,
                            name: d.name.clone(),
                            counts_per_mm: STUB_COUNTS_PER_MM,
                            rotation_distance: STUB_ROTATION_DISTANCE,
                            invert: false,
                        })
                        .collect();
                    let drive_count = drives.len();
                    let rc = capture.start(CaptureConfig {
                        path: msg.path.clone(),
                        started_utc: msg.started_utc.clone(),
                        drives,
                        cycle_ns: STUB_CYCLE_NS,
                        started_mono_ns: monotonic_ns(),
                    });
                    if rc == 0 {
                        capture_drive_count = drive_count;
                    }
                    eprintln!("ec-rt-stub: StartCapture path={} rc={rc}", msg.path);
                    server.respond(&start_capture_response_frame(correlation_id, rc));
                }
                Command::StopCapture { correlation_id } => {
                    let out = capture.stop();
                    eprintln!(
                        "ec-rt-stub: StopCapture result={} samples={} overflow={:?}",
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
                Command::SetDiffDamper {
                    correlation_id,
                    msg,
                } => {
                    eprintln!(
                        "ec-rt-stub: SetDiffDamper slots=({},{}) gain_milli={}",
                        msg.slot_a, msg.slot_b, msg.gain_milli
                    );
                    server.respond(&set_diff_damper_response_frame(correlation_id, 0));
                }
                Command::SetDiffTrim {
                    correlation_id,
                    msg,
                } => {
                    eprintln!(
                        "ec-rt-stub: SetDiffTrim slots=({},{}) gain_micro={}",
                        msg.slot_a, msg.slot_b, msg.gain_micro
                    );
                    server.respond(&set_diff_trim_response_frame(correlation_id, 0));
                }
                Command::SetStrainComp {
                    correlation_id,
                    prepared,
                } => {
                    eprintln!(
                        "ec-rt-stub: SetStrainComp slots=({},{}) grid={}x{}",
                        prepared.slot_a, prepared.slot_b, prepared.nx, prepared.ny
                    );
                    server.respond(&set_strain_comp_response_frame(correlation_id, 0));
                }
                Command::SetDynamicsModel {
                    correlation_id,
                    msg,
                } => {
                    eprintln!(
                        "ec-rt-stub: SetDynamicsModel slots={} modes={}",
                        msg.slots_count, msg.modes_count
                    );
                    server.respond(&set_dynamics_model_response_frame(correlation_id, 0));
                }
                Command::SetFfLead {
                    correlation_id,
                    msg,
                } => {
                    eprintln!(
                        "ec-rt-stub: SetFfLead slot={} lead_ns={}",
                        msg.slot, msg.lead_ns
                    );
                    server.respond(&set_ff_lead_response_frame(correlation_id, 0));
                }
                Command::Unknown { kind_raw, .. } => {
                    eprintln!("ec-rt-stub: ignoring kind 0x{kind_raw:04x}");
                }
                Command::PushSampleRuns {
                    correlation_id,
                    msg,
                } => {
                    let now_ns = monotonic_ns();
                    let mut result = if gate.state() == TorqueState::Faulted {
                        ERR_PIECES_WHILE_FAULTED
                    } else {
                        stream_halt.check_push_allowed().err().unwrap_or(0)
                    };
                    if result == 0 {
                        for lane in &msg.lanes {
                            let header = RunHeader {
                                start_index: lane.start_index,
                                interval_ticks: lane.interval_ticks,
                                origin_mm: f64::from(lane.origin_mm_q16) / 65536.0,
                                anchor: lane.flags & LANE_RUN_FLAG_REANCHOR != 0,
                                final_run: lane.flags & LANE_RUN_FLAG_TAIL != 0,
                            };
                            let entries: Vec<SetpointEntry> = lane
                                .samples
                                .iter()
                                .map(|s| SetpointEntry {
                                    pos_counts: s.pos_counts,
                                    vel_ff: s.vel_ff,
                                    torque_ff: s.torque_ff,
                                    acc_mm_s2: s.acc_mm_s2,
                                })
                                .collect();
                            eprintln!(
                                "ec-rt-stub: PushSampleRuns axis={} start_index={} count={} \
                                 flags=0x{:02x}",
                                lane.axis_idx,
                                lane.start_index,
                                entries.len(),
                                lane.flags
                            );
                            if let Err(fault) = ring.fill(&header, &entries) {
                                eprintln!(
                                    "ec-rt-stub: sample run rejected — {} ({})",
                                    fault.as_str(),
                                    fault.code()
                                );
                                result = fault.code();
                            }
                        }
                    }
                    let lanes: Vec<(u8, u32)> = msg
                        .lanes
                        .iter()
                        .map(|lane| (lane.axis_idx, ring.free() as u32))
                        .collect();
                    server.respond(&push_sample_runs_response_frame(
                        correlation_id,
                        result,
                        now_ns,
                        grid_now(grid_base_ns),
                        &lanes,
                    ));
                }
                Command::QuerySampleGrid { correlation_id } => {
                    server.respond(&sample_grid_response_frame(
                        correlation_id,
                        EXECUTOR_SETPOINT_RING,
                        STUB_CYCLE_NS as u32,
                        RING_DEPTH_CYCLES as u32,
                        grid_now(grid_base_ns),
                    ));
                }
            }
        }

        let now = monotonic_ns();

        match gate.on_tick(now, ring.is_empty()) {
            TickAction::None => {}
            TickAction::ExecuteDisable => {
                eprintln!("ec-rt-stub: scheduled torque disable executed");
                gate.disable_finished();
            }
            TickAction::Fault { code } => {
                eprintln!("ec-rt-stub: torque-gate fault code={code} — exiting");
                server.respond(&status_heartbeat_frame(
                    ENGINE_STATE_FAULT,
                    0,
                    &[ring.played_count()],
                    &[playback_clock(grid_base_ns, &ring)],
                    0,
                ));
                std::process::exit(1);
            }
        }

        let sensorless_tripped = sensorless.poll(
            now,
            |_slot| sim_torque,
            |_slot| 0,
            |_slot, endstop_id, torque, contact_clock| {
                eprintln!(
                    "ec-rt-stub: sensorless endstop {endstop_id} tripped torque={torque} \
                     — local stop, stream halted, trip_clock={contact_clock}"
                );
                server.respond(&endstop_trip_frame(endstop_id, contact_clock));
            },
        );
        if sensorless_tripped {
            ring.reset();
            stream_halt.halt();
        }

        let played = if gate.state() == TorqueState::Enabled {
            match ring.play(grid_index_now(grid_base_ns)) {
                Played::Entry(entry) if !suppressed => Some(entry),
                _ => None,
            }
        } else {
            None
        };
        let motion_active = played.is_some();

        if let Some(entry) = played {
            played_cycles += 1;
            if !drive_fault_fired {
                if let Some(threshold) = drive_fault_after {
                    if played_cycles >= threshold {
                        drive_fault_fired = true;
                        gate.on_drive_fault();
                        ring.reset();
                        eprintln!(
                            "ec-rt-stub: drive fault simulated after {played_cycles} cycles \
                             (last target {} counts)",
                            entry.pos_counts
                        );
                        server.respond(&status_heartbeat_frame(
                            0,
                            0x8611,
                            &[ring.played_count()],
                            &[playback_clock(grid_base_ns, &ring)],
                            0,
                        ));
                        last_sent_retired = ring.played_count();
                        heartbeat_sent = true;
                    }
                }
            }
        }

        cycle_index += 1;
        if capture.is_active() {
            #[allow(clippy::cast_possible_truncation)]
            let pos = ((cycle_index % 100_000) * 10) as i32;
            let mut flags = 0u8;
            if gate.state() == TorqueState::Enabled {
                flags |= FLAG_TORQUE_ENABLED;
            }
            if motion_active {
                flags |= FLAG_MOTION_ACTIVE;
            }
            let sim = DriveSample {
                target_counts: pos,
                position_actual: pos - 3,
                velocity_actual: 0,
                following_error: 3,
                torque_actual: 100,
                statusword: 0x0627,
                error_code: 0,
                velocity_offset: 0,
                torque_offset: 0,
                accel_cmd: 0.0,
                vel_cmd: 0.0,
                pin_res_re: 0.0,
                pin_res_im: 0.0,
            };
            let mut record = CaptureRecord::new(cycle_index, flags);
            record.drive_count = capture_drive_count as u8;
            for (i, d) in record.drives[..capture_drive_count].iter_mut().enumerate() {
                *d = sim;
                d.position_actual += i32::try_from(i).expect("drive index fits i32");
            }
            capture.push(record);
        }

        if let Some(fault_val) = ring.take_fault() {
            let fault_code_u16 = (fault_val & 0xFFFF) as u16;
            eprintln!(
                "ec-rt-stub: FAULT latched fault_val=0x{fault_val:08x} code=0x{fault_code_u16:04x} \
                 — propagating to host via heartbeat, host must shut down"
            );
            let current_played = ring.played_count();
            server.respond(&status_heartbeat_frame(
                ENGINE_STATE_FAULT,
                fault_code_u16,
                &[current_played],
                &[playback_clock(grid_base_ns, &ring)],
                0,
            ));
            last_sent_retired = current_played;
            heartbeat_sent = true;
        }

        let current_played = ring.played_count();
        let should_emit = !heartbeat_sent || current_played != last_sent_retired;
        if should_emit {
            let engine_state: u8 = if ring.is_empty() { 0 } else { 1 };
            server.respond(&status_heartbeat_frame(
                engine_state,
                0,
                &[current_played],
                &[playback_clock(grid_base_ns, &ring)],
                0,
            ));
            last_sent_retired = current_played;
            heartbeat_sent = true;
            if current_played != 0 {
                eprintln!("ec-rt-stub: heartbeat played_count={current_played}");
            }
        }

        sleep(Duration::from_millis(1));
    }
}
