#![allow(unsafe_code)]

use std::ffi::CString;
use std::sync::atomic::Ordering;

use super::drive::{DriveChain, FfiDriveChain};
use super::{EndpointCtx, SIGTERM_RECEIVED};
use crate::buzz::BuzzOsc;
use crate::capture::{Capture, PendingStart, PendingStop};
use crate::claim::{all_slaves_reply, single_slave_reply, wait_for_claim, wait_for_claim_pumping};
use crate::cli::{Args, SlaveCfg};
use crate::curves::AxisRing;
use crate::damper::DiffDamperBank;
use crate::ffi;
use crate::live_tap::{self, LiveTap};
use crate::mailbox::{MailboxWorker, WorkerScheduling};
use crate::scale::CountMap;
use crate::sdo::SdoBus;
use crate::sensorless::SensorlessBank;
use crate::server::FrameServer;
use crate::stream_halt::StreamHalt;
use crate::torque::TorqueGate;
use crate::trim::DiffTrimBank;
use crate::wire::claim_handshake_reply_frame;
use mcu_protocol::messages::{SlaveState, ERR_SDO_TRANSPORT, ERR_SDO_UNSUPPORTED_SIZE};

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
pub(super) fn log_al_states(num_slaves: usize, stage: &str) {
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

struct SlaveColumns {
    counts_per_mm: Vec<f64>,
    invert: Vec<bool>,
    cmd_counts_per_mm: Vec<f64>,
    rotation_distance: Vec<f64>,
    positions: Vec<i32>,
    axes: Vec<u8>,
    velocity_ff: Vec<bool>,
    torque_clamp_tenths: Vec<i16>,
    ff_lead_ns: Vec<u64>,
    jump_log_counts: Vec<i64>,
}

impl SlaveColumns {
    fn from(slaves: &[SlaveCfg], cycle_us: i64) -> Self {
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
        let jump_log_mm = TARGET_JUMP_LOG_MM_S * cycle_us as f64 / 1_000_000.0;
        let jump_log_counts: Vec<i64> = cmd_counts_per_mm
            .iter()
            .map(|c| (c.abs() * jump_log_mm).round() as i64)
            .collect();
        SlaveColumns {
            counts_per_mm: slaves.iter().map(|s| s.counts_per_mm).collect(),
            invert: slaves.iter().map(|s| s.invert).collect(),
            cmd_counts_per_mm,
            rotation_distance: slaves.iter().map(|s| s.rotation_distance).collect(),
            positions: slaves.iter().map(|s| s.pos).collect(),
            axes: slaves.iter().map(|s| s.axis).collect(),
            velocity_ff: slaves.iter().map(|s| s.velocity_ff).collect(),
            torque_clamp_tenths: slaves.iter().map(|s| s.torque_clamp_tenths).collect(),
            ff_lead_ns: slaves
                .iter()
                .map(|s| u64::from(s.ff_lead_cycles) * (cycle_us as u64) * 1000)
                .collect(),
            jump_log_counts,
        }
    }
}

fn read_and_apply_limits(slaves: &[SlaveCfg]) -> Result<Vec<(u32, u16)>, i32> {
    slaves
        .iter()
        .enumerate()
        .map(|(s, cfg)| {
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
            let cli_ferr = cfg.following_error_counts;
            let cli_tq = cfg.max_torque_tenth_pct;
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
        .collect()
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
        late_tolerance_ns,
    } = args;

    let num_slaves = slaves.len();
    let mut drive: Box<dyn DriveChain> = Box::new(FfiDriveChain);
    let columns = SlaveColumns::from(&slaves, cycle_us);

    let cycle_ns = cycle_us * 1000;
    let telemetry_period = u64::try_from(cycle_us)
        .map(|u| (500_000u64 / u).max(1))
        .unwrap_or(500);

    let rings: Vec<AxisRing> = (0..num_slaves).map(AxisRing::with_slot).collect();
    let buzz = BuzzOsc::new();
    let damper = DiffDamperBank::new(cycle_ns);
    let trim = DiffTrimBank::new(cycle_ns);
    let comp = crate::strain_comp::StrainCompBank::new(cycle_ns);
    let cmaps: Vec<Option<CountMap>> = (0..num_slaves).map(|_| None).collect();
    let last_counts: Vec<Option<i32>> = vec![None; num_slaves];
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

    // The capture-io and live-tap thread spawns and their record-channel
    // buffers are multi-millisecond stalls under mlockall(MCL_FUTURE); they
    // must happen before ec_rt_bringup_preop, while no drive is DC-synced
    // and no park cycle is being pumped on this thread (claim-time
    // Capture::new stalled the park loop past the sync watchdog and halted
    // the bus at every claim, bench 2026-07-06).
    let capture = Capture::new();
    let live_tap = LiveTap::spawn(
        &format!("{socket}.live"),
        live_tap::slot_configs(
            &columns.counts_per_mm,
            &columns.rotation_distance,
            &columns.invert,
        ),
        cycle_ns,
    )
    .expect("bind live tap socket");
    let tap_slots: Vec<u8> = (0..num_slaves as u8).collect();

    let mut server = FrameServer::bind(&socket).expect("bind socket");
    tracing::info!(
        subsystem = "ethercat",
        event = "bringup_start",
        num_slaves,
        cycle_us,
        positions = format!("{:?}", columns.positions),
        counts_per_mm = format!("{:?}", columns.counts_per_mm),
        invert = format!("{:?}", columns.invert),
        velocity_ff = format!("{:?}", columns.velocity_ff),
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
                columns.positions.as_ptr(),
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

        let run_limits = match read_and_apply_limits(&slaves) {
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
            drive.shutdown_and_exit(num_slaves);
        }
    }
    tracing::info!(
        subsystem = "ethercat",
        event = "bringup_handshake_ok",
        "handshake ok, entering DC loop"
    );

    let gate = TorqueGate::new();
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

    let SlaveColumns {
        counts_per_mm,
        invert,
        cmd_counts_per_mm,
        rotation_distance,
        positions: _,
        axes: slave_axes,
        velocity_ff,
        torque_clamp_tenths,
        ff_lead_ns,
        jump_log_counts,
    } = columns;

    EndpointCtx {
        server,
        drive,
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
        damper,
        trim,
        comp,
        cmaps,
        last_counts,
        last_written_offset: vec![0; num_slaves],
        report_anchor,
        last_streamed_target,
        last_sent_retired,
        heartbeat_sent,
        gate,
        capture,
        live_tap,
        tap_slots,
        cycle_index,
        mailbox,
        pending_starts,
        pending_stops,
        pending_seed: None,
        capture_slots,
        prdiv,
        ff_saturation,
        wkc_consecutive,
        latched_drive_err,
        sensorless,
        stream_halt,
        late_tolerance_ns,
        timing_armed: false,
        baseline_reanchor_count: 0,
        late_frames: 0,
        late_max_ns: i64::MIN,
        last_dispatch_ns: 0,
        last_pre_work_ns: 0,
        prev_exchange_ns: 0,
    }
}
