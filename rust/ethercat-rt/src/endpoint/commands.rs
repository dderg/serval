use std::ops::ControlFlow;

use super::{discard_motion, EndpointCtx};
use crate::capture::{
    any_slot_out_of_range, CaptureConfig, CaptureDriveConfig, ERR_CAPTURE_BAD_DRIVE_LIST,
};
use crate::clock::monotonic_ns;
use crate::curves::AXIS_RING_CAPACITY;
use crate::dynamics::{DynamicsModel, ERR_DYNAMICS_BAD_DIM, ERR_DYNAMICS_REJECTED};
use crate::mailbox::{LimitEntry, MailboxReply, MailboxRequest};
use crate::push_plan::plan_bundle;
use crate::sensorless::{ERR_ARM_SENSORLESS_AMBIGUOUS_PAIR, ERR_ARM_SENSORLESS_BAD_THRESHOLD};
use crate::strain_comp::ERR_COMP_BAD_LANE;
use crate::torque::{CommandAction, TorqueState, ERR_ENABLE_FAILED, ERR_PIECES_WHILE_FAULTED};
use crate::wire::{
    arm_sensorless_endstop_response_frame, identify_response_frame, motor_state_empty_frame,
    motor_state_response_frame_multi, push_pieces_response_frame_multi,
    resonance_buzz_response_frame, restore_drive_limits_response_frame,
    resume_stream_response_frame, runtime_caps_response_frame, sdo_read_response_frame,
    sdo_write_response_frame, seed_servo_home_response_frame, set_diff_damper_response_frame,
    set_diff_trim_response_frame, set_drive_limits_response_frame,
    set_dynamics_model_response_frame, set_ff_lead_response_frame, set_strain_comp_response_frame,
    set_torque_response_frame, start_capture_response_frame, stop_capture_response_frame,
    stop_response_frame, Command,
};
use mcu_protocol::messages::{
    ArmSensorlessEndstop, PushPieces, ResonanceBuzz, SdoRead, SdoReadResponse, SdoWrite,
    SdoWriteResponse, SetDiffDamper, SetDiffTrim, SetDriveLimits, SetDynamicsModel, SetFfLead,
    SetTorque, StartCapture, StopCaptureResponse,
};

/// Command execution shares the RT thread with the DC exchange, so it must
/// fit in the post-send slack; a jog-start piece burst measured >500 us and
/// skipped whole cycles. Pieces arrive with ~95 ms of lead, so commands left
/// in the queue when the budget runs out simply carry to the next cycle.
const DISPATCH_BUDGET_NS: u128 = 100_000;

fn command_name(cmd: &Command) -> &'static str {
    match cmd {
        Command::Identify { .. } => "Identify",
        Command::PushPieces { .. } => "PushPieces",
        Command::QueryRuntimeCaps { .. } => "QueryRuntimeCaps",
        Command::SetTorque { .. } => "SetTorque",
        Command::Stop { .. } => "Stop",
        Command::StartCapture { .. } => "StartCapture",
        Command::StopCapture { .. } => "StopCapture",
        Command::ResumeStream { .. } => "ResumeStream",
        Command::ClaimHandshake { .. } => "ClaimHandshake",
        Command::SetDriveLimits { .. } => "SetDriveLimits",
        Command::RestoreDriveLimits { .. } => "RestoreDriveLimits",
        Command::SeedServoHome { .. } => "SeedServoHome",
        Command::ArmSensorlessEndstop { .. } => "ArmSensorlessEndstop",
        Command::ResonanceBuzz { .. } => "ResonanceBuzz",
        Command::SetDiffDamper { .. } => "SetDiffDamper",
        Command::SetDiffTrim { .. } => "SetDiffTrim",
        Command::SetStrainComp { .. } => "SetStrainComp",
        Command::SetDynamicsModel { .. } => "SetDynamicsModel",
        Command::SetFfLead { .. } => "SetFfLead",
        Command::SdoRead { .. } => "SdoRead",
        Command::SdoWrite { .. } => "SdoWrite",
        Command::QueryMotorState { .. } => "QueryMotorState",
        Command::Unknown { .. } => "Unknown",
    }
}

pub(super) fn dispatch_commands(ctx: &mut EndpointCtx) -> ControlFlow<()> {
    let started = std::time::Instant::now();
    ctx.server.pump();
    let pump_ns = started.elapsed().as_nanos();
    if pump_ns > DISPATCH_BUDGET_NS {
        tracing::warn!(
            subsystem = "ethercat",
            event = "slow_pump",
            pump_ns = pump_ns as i64,
            "socket read + frame decode exceeded the dispatch budget on the \
             RT thread"
        );
    }
    while let Some(cmd) = ctx.server.pop_command() {
        let name = command_name(&cmd);
        let cmd_started = std::time::Instant::now();
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
                // Capacity is per distinct axis, not per slave: an AWD axis
                // fans its pieces out to every slot claiming it, so extra
                // slots add no per-axis headroom. The host divides this by
                // the axis count to size its per-axis window.
                let mut distinct_axes: Vec<u8> = ctx.slave_axes.clone();
                distinct_axes.sort_unstable();
                distinct_axes.dedup();
                let total: u32 = (AXIS_RING_CAPACITY
                    * distinct_axes.len()
                    * runtime::piece_ring::PIECE_ENTRY_BYTES)
                    as u32;
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
                discard_motion(ctx);
                ctx.stream_halt.halt();
                crate::rt_eprintln!(
                    "ec-rt: Stop — rings discarded, stream halted, discard_clock={now_ns}"
                );
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
                    discard_motion(ctx);
                    crate::rt_eprintln!("ec-rt: ResumeStream — stream reopened");
                    ctx.server
                        .respond(&resume_stream_response_frame(correlation_id, 0));
                }
                Err(code) => {
                    crate::rt_eprintln!(
                        "ec-rt: ResumeStream rejected code={code} — stream was not halted"
                    );
                    ctx.server
                        .respond(&resume_stream_response_frame(correlation_id, code));
                }
            },
            Command::ClaimHandshake { .. } => {
                crate::rt_eprintln!(
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
                slot_mask,
            } => {
                handle_restore_drive_limits(ctx, correlation_id, slot_mask);
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
            Command::SetDiffDamper {
                correlation_id,
                msg,
            } => {
                handle_set_diff_damper(ctx, correlation_id, msg);
            }
            Command::SetDiffTrim {
                correlation_id,
                msg,
            } => {
                handle_set_diff_trim(ctx, correlation_id, msg);
            }
            Command::SetStrainComp {
                correlation_id,
                prepared,
            } => {
                handle_set_strain_comp(ctx, correlation_id, prepared);
            }
            Command::SetDynamicsModel {
                correlation_id,
                msg,
            } => {
                handle_set_dynamics_model(ctx, correlation_id, msg);
            }
            Command::SetFfLead {
                correlation_id,
                msg,
            } => {
                handle_set_ff_lead(ctx, correlation_id, msg);
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
                crate::rt_eprintln!("ec-rt: ignoring kind 0x{kind_raw:04x}");
            }
        }
        let cmd_ns = cmd_started.elapsed().as_nanos();
        // SetTorque's enable path is the CiA402 walk: hundreds of ms of wall
        // time, but its internal exchanges stay on the DC grid, so it never
        // misses a latch — warning on it would bury the real offenders.
        if cmd_ns > DISPATCH_BUDGET_NS && name != "SetTorque" {
            tracing::warn!(
                subsystem = "ethercat",
                event = "slow_command",
                command = name,
                cmd_ns = cmd_ns as i64,
                "a single command exceeded the dispatch budget on the RT \
                 thread — this is the stall the budget cannot split"
            );
        }
        if started.elapsed().as_nanos() > DISPATCH_BUDGET_NS {
            break;
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
                for (axis, axis_slots) in msg.axes.iter().zip(slots.iter()) {
                    for &slot in axis_slots {
                        ctx.rings[slot].push_from_bytes(axis.piece_count, &axis.pieces_bytes);
                    }
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

pub(super) fn handle_set_torque(ctx: &mut EndpointCtx, correlation_id: u32, msg: SetTorque) {
    match ctx.gate.on_set_torque(msg.value != 0, msg.execute_at_ns) {
        CommandAction::Enable => {
            let enable_rc = ctx.drive.enable_all();
            ctx.gate.enable_finished(enable_rc == 0);
            if enable_rc == 0 {
                crate::rt_eprintln!("ec-rt: torque enabled (CiA402 operation enabled)");
                ctx.server
                    .respond(&set_torque_response_frame(correlation_id, 0));
            } else {
                crate::rt_eprintln!(
                    "ec-rt: CiA402 enable failed rc={enable_rc} — disabling and exiting"
                );
                ctx.server.respond(&set_torque_response_frame(
                    correlation_id,
                    ERR_ENABLE_FAILED,
                ));
                ctx.drive.shutdown_and_exit();
            }
        }
        CommandAction::ScheduleDisable => {
            crate::rt_eprintln!(
                "ec-rt: torque disable scheduled at {} (now {})",
                msg.execute_at_ns,
                monotonic_ns()
            );
            ctx.server
                .respond(&set_torque_response_frame(correlation_id, 0));
        }
        CommandAction::Reject { code } => {
            crate::rt_eprintln!(
                "ec-rt: SetTorque rejected code={code} \
                     (value={} execute_at={} now={}) — exiting",
                msg.value,
                msg.execute_at_ns,
                monotonic_ns()
            );
            ctx.server
                .respond(&set_torque_response_frame(correlation_id, code));
            ctx.drive.shutdown_and_exit();
        }
    }
}

fn handle_start_capture(ctx: &mut EndpointCtx, correlation_id: u32, msg: StartCapture) {
    let num_slaves = ctx.num_slaves;
    let slots: Vec<u8> = msg.drives.iter().map(|d| d.slot).collect();
    if any_slot_out_of_range(&slots, num_slaves) {
        crate::rt_eprintln!(
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
                invert: ctx.invert[d.slot as usize],
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
    if msg.drives.is_empty() || msg.drives.iter().any(|d| d.slot as usize >= num_slaves) {
        crate::rt_eprintln!(
            "ec-rt: SetDriveLimits for slots {:?} but only {num_slaves} slave(s)",
            msg.drives.iter().map(|d| d.slot).collect::<Vec<_>>()
        );
        ctx.server
            .respond(&set_drive_limits_response_frame(correlation_id, -309));
    } else {
        ctx.mailbox.submit(MailboxRequest::WriteLimits {
            correlation_id,
            entries: msg
                .drives
                .iter()
                .map(|d| LimitEntry {
                    slot: d.slot,
                    ferr_counts: d.following_error_counts,
                    torque_tenth_pct: d.max_torque_tenth_pct,
                })
                .collect(),
            restore: false,
        });
    }
}

pub(super) fn handle_set_ff_lead(ctx: &mut EndpointCtx, correlation_id: u32, msg: SetFfLead) {
    let num_slaves = ctx.num_slaves;
    if msg.slot as usize >= num_slaves {
        crate::rt_eprintln!(
            "ec-rt: SetFfLead for slot {} but only {num_slaves} slave(s)",
            msg.slot
        );
        ctx.server
            .respond(&set_ff_lead_response_frame(correlation_id, -309));
        return;
    }
    ctx.ff_lead_ns[msg.slot as usize] = msg.lead_ns;
    crate::rt_eprintln!(
        "ec-rt: SetFfLead slot={} lead_ns={} rc=0",
        msg.slot,
        msg.lead_ns
    );
    tracing::info!(
        subsystem = "ethercat",
        event = "set_ff_lead",
        slot = msg.slot,
        lead_ns = msg.lead_ns,
        "feedforward lead updated"
    );
    ctx.server
        .respond(&set_ff_lead_response_frame(correlation_id, 0));
}

fn handle_restore_drive_limits(ctx: &mut EndpointCtx, correlation_id: u32, slot_mask: u32) {
    let num_slaves = ctx.run_limits.len();
    if slot_mask == 0 || slot_mask >> num_slaves != 0 {
        crate::rt_eprintln!(
            "ec-rt: RestoreDriveLimits slot_mask={slot_mask:#x} but only {num_slaves} slave(s)"
        );
        ctx.server
            .respond(&restore_drive_limits_response_frame(correlation_id, -309));
        return;
    }
    let entries = ctx
        .run_limits
        .iter()
        .enumerate()
        .filter(|(slot, _)| slot_mask & (1 << slot) != 0)
        .map(|(slot, &(ferr_counts, torque_tenth_pct))| LimitEntry {
            slot: slot as u8,
            ferr_counts,
            torque_tenth_pct,
        })
        .collect();
    ctx.mailbox.submit(MailboxRequest::WriteLimits {
        correlation_id,
        entries,
        restore: true,
    });
}

const ERR_SEED_HOME_STREAMING: i32 = -826;
const SEED_DRAIN_TIMEOUT_NS: i64 = 2_000_000_000;

pub(super) struct PendingSeed {
    pub(super) correlation_id: u32,
    pub(super) slot: u8,
    pub(super) home_q16: i32,
    pub(super) deadline_cycle: u64,
}

/// The host's wait_moves is a wall-clock estimate; the endpoint retires the
/// last pieces up to the drip lead later, so a seed arriving with the ring
/// still draining is the normal homing race, not an error. Defer it until the
/// rings empty; only a ring that stays occupied past the timeout fails.
pub(super) fn handle_seed_servo_home(
    ctx: &mut EndpointCtx,
    correlation_id: u32,
    slot: u8,
    home_q16: i32,
) {
    if slot as usize >= ctx.counts_per_mm.len() {
        crate::rt_eprintln!(
            "ec-rt: SeedServoHome for slot {slot} but only {} slave(s)",
            ctx.counts_per_mm.len()
        );
        ctx.server
            .respond(&seed_servo_home_response_frame(correlation_id, -309));
    } else if ctx.rings.iter().any(|r| !r.is_empty()) {
        if ctx.pending_seed.is_some() {
            crate::rt_eprintln!("ec-rt: SeedServoHome rejected — a seed is already pending");
            ctx.server.respond(&seed_servo_home_response_frame(
                correlation_id,
                ERR_SEED_HOME_STREAMING,
            ));
            return;
        }
        let drain_cycles = (SEED_DRAIN_TIMEOUT_NS / ctx.cycle_ns).max(1) as u64;
        crate::rt_eprintln!(
            "ec-rt: SeedServoHome slot={slot} deferred until the motion ring drains"
        );
        ctx.pending_seed = Some(PendingSeed {
            correlation_id,
            slot,
            home_q16,
            deadline_cycle: ctx.cycle_index + drain_cycles,
        });
    } else {
        complete_seed(ctx, correlation_id, slot, home_q16);
    }
}

fn complete_seed(ctx: &mut EndpointCtx, correlation_id: u32, slot: u8, home_q16: i32) {
    let anchor_mm = f64::from(home_q16) / 65536.0;
    let anchor_counts = ctx.last_streamed_target[slot as usize]
        .unwrap_or_else(|| ctx.drive.position_actual(usize::from(slot)));
    ctx.report_anchor[slot as usize] = Some((anchor_counts, anchor_mm));
    crate::rt_eprintln!(
        "ec-rt: SeedServoHome slot={slot} report anchor \
         {anchor_counts} counts = {anchor_mm:.4} mm (drive frame untouched)"
    );
    ctx.server
        .respond(&seed_servo_home_response_frame(correlation_id, 0));
}

pub(super) fn drain_pending_seed(ctx: &mut EndpointCtx) {
    let Some(seed) = &ctx.pending_seed else {
        return;
    };
    if ctx.rings.iter().all(|r| r.is_empty()) {
        let seed = ctx.pending_seed.take().expect("checked above");
        complete_seed(ctx, seed.correlation_id, seed.slot, seed.home_q16);
    } else if ctx.cycle_index >= seed.deadline_cycle {
        let seed = ctx.pending_seed.take().expect("checked above");
        crate::rt_eprintln!(
            "ec-rt: SeedServoHome slot={} rejected — motion ring still not \
             empty after the drain timeout",
            seed.slot
        );
        ctx.server.respond(&seed_servo_home_response_frame(
            seed.correlation_id,
            ERR_SEED_HOME_STREAMING,
        ));
    }
}

fn handle_arm_sensorless_endstop(
    ctx: &mut EndpointCtx,
    correlation_id: u32,
    msg: ArmSensorlessEndstop,
) {
    let num_slaves = ctx.num_slaves;
    let result = if msg.slot as usize >= num_slaves {
        crate::rt_eprintln!(
            "ec-rt: ArmSensorlessEndstop for slot {} but only {num_slaves} slave(s)",
            msg.slot
        );
        -309
    } else if msg.enable != 0 {
        if msg.torque_trip_tenth_pct == 0 {
            crate::rt_eprintln!(
                "ec-rt: ArmSensorlessEndstop rejected — zero torque trip threshold"
            );
            ERR_ARM_SENSORLESS_BAD_THRESHOLD
        } else {
            // A belt-pair slot trips on the pair's common-mode torque: the
            // crash pushes both rotors the same mechanical way while the
            // pair's standing fight (and the differential damper's
            // injection) is antisymmetric and cancels out of the average.
            let slot = msg.slot as usize;
            let partners: Vec<usize> = ctx
                .slave_axes
                .iter()
                .enumerate()
                .filter(|&(s, &axis)| s != slot && axis == ctx.slave_axes[slot])
                .map(|(s, _)| s)
                .collect();
            match partners[..] {
                [] | [_] => {
                    let partner = partners.first().copied();
                    ctx.sensorless
                        .arm(slot, msg.endstop_id, msg.torque_trip_tenth_pct, partner);
                    crate::rt_eprintln!(
                        "ec-rt: sensorless endstop {} armed on slot {} \
                         (torque_trip={} 0.1% partner={partner:?})",
                        msg.endstop_id,
                        msg.slot,
                        msg.torque_trip_tenth_pct
                    );
                    0
                }
                _ => {
                    crate::rt_eprintln!(
                        "ec-rt: ArmSensorlessEndstop rejected — slot {slot} shares \
                         axis {} with {} other slots, need at most one partner \
                         (slave_axes={:?})",
                        ctx.slave_axes[slot],
                        partners.len(),
                        ctx.slave_axes
                    );
                    ERR_ARM_SENSORLESS_AMBIGUOUS_PAIR
                }
            }
        }
    } else {
        ctx.sensorless.disarm(msg.slot as usize);
        crate::rt_eprintln!(
            "ec-rt: sensorless endstop {} disarmed on slot {}",
            msg.endstop_id,
            msg.slot
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
        crate::rt_eprintln!("ec-rt: ResonanceBuzz rejected — drive not operation-enabled");
        crate::buzz::ERR_BUZZ_NOT_ENABLED
    } else if ctx.rings.iter().any(|r| !r.is_empty()) || ctx.buzz.active() {
        crate::rt_eprintln!("ec-rt: ResonanceBuzz rejected — motion in progress");
        if ctx.buzz.active() {
            crate::buzz::ERR_BUZZ_BUSY
        } else {
            crate::buzz::ERR_BUZZ_STREAMING
        }
    } else {
        let mut base_counts = [0i32; crate::buzz::MAX_BUZZ_SLOTS];
        for (slot, base) in base_counts.iter_mut().enumerate().take(ctx.num_slaves) {
            if msg.axis_mask & (1 << slot) != 0 {
                *base = ctx.drive.position_actual(slot);
            }
        }
        let rc = ctx.buzz.arm(
            ctx.num_slaves as u8,
            msg.axis_mask,
            msg.sign_mask,
            msg.freq_start_millihz,
            msg.freq_end_millihz,
            msg.amplitude_nm,
            msg.duration_ms,
            msg.ramp_ms,
            base_counts,
        );
        crate::rt_eprintln!(
            "ec-rt: ResonanceBuzz axis_mask=0x{:02x} sign_mask=0x{:02x} \
             freq={}->{} mHz amplitude={} nm duration={} ms ramp={} ms \
             base_counts={base_counts:?} rc={rc}",
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

fn handle_set_diff_damper(ctx: &mut EndpointCtx, correlation_id: u32, msg: SetDiffDamper) {
    let rc = {
        ctx.damper.set(
            ctx.num_slaves,
            msg.slot_a,
            msg.slot_b,
            msg.gain_milli,
            msg.clamp_tenths,
            msg.lpf_millihz,
            msg.lead_us,
        )
    };
    crate::rt_eprintln!(
        "ec-rt: SetDiffDamper slots=({},{}) gain_milli={} clamp={} 0.1% \
         lpf={} mHz lead={} us rc={rc}",
        msg.slot_a,
        msg.slot_b,
        msg.gain_milli,
        msg.clamp_tenths,
        msg.lpf_millihz,
        msg.lead_us,
    );
    tracing::info!(
        subsystem = "ethercat",
        event = "set_diff_damper",
        slot_a = msg.slot_a,
        slot_b = msg.slot_b,
        gain_milli = msg.gain_milli,
        clamp_tenths = msg.clamp_tenths,
        lpf_millihz = msg.lpf_millihz,
        lead_us = msg.lead_us,
        rc,
        "differential damper reconfigured"
    );
    ctx.server
        .respond(&set_diff_damper_response_frame(correlation_id, rc));
}

fn handle_set_diff_trim(ctx: &mut EndpointCtx, correlation_id: u32, msg: SetDiffTrim) {
    let rc = {
        ctx.trim.set(
            ctx.num_slaves,
            msg.slot_a,
            msg.slot_b,
            msg.gain_micro,
            msg.clamp_um,
            msg.lpf_millihz,
            msg.settle_ms,
        )
    };
    crate::rt_eprintln!(
        "ec-rt: SetDiffTrim slots=({},{}) gain_micro={} clamp={} um lpf={} mHz \
         settle={} ms rc={rc}",
        msg.slot_a,
        msg.slot_b,
        msg.gain_micro,
        msg.clamp_um,
        msg.lpf_millihz,
        msg.settle_ms,
    );
    tracing::info!(
        subsystem = "ethercat",
        event = "set_diff_trim",
        slot_a = msg.slot_a,
        slot_b = msg.slot_b,
        gain_micro = msg.gain_micro,
        clamp_um = msg.clamp_um,
        lpf_millihz = msg.lpf_millihz,
        settle_ms = msg.settle_ms,
        rc,
        "differential trim reconfigured"
    );
    ctx.server
        .respond(&set_diff_trim_response_frame(correlation_id, rc));
}

fn handle_set_strain_comp(
    ctx: &mut EndpointCtx,
    correlation_id: u32,
    prepared: crate::strain_comp::PreparedStrainComp,
) {
    let (slot_a, slot_b) = (prepared.slot_a, prepared.slot_b);
    let (lane_a, lane_b) = (prepared.lane_a, prepared.lane_b);
    let (kinematics, nx, ny) = (prepared.kinematics, prepared.nx, prepared.ny);
    let (x0, y0, dx, dy) = (prepared.x0, prepared.y0, prepared.dx, prepared.dy);
    let wire_values = prepared.wire_values;
    let lane_missing = |lane: u8| !ctx.slave_axes.iter().any(|&a| a == lane);
    let rc = if nx > 0 && ny > 0 && (lane_missing(lane_a) || lane_missing(lane_b)) {
        crate::rt_eprintln!(
            "ec-rt: SetStrainComp lanes ({}, {}) not present in slave_axes {:?}",
            lane_a,
            lane_b,
            ctx.slave_axes
        );
        ERR_COMP_BAD_LANE
    } else {
        ctx.comp.install(ctx.num_slaves, prepared)
    };
    crate::rt_eprintln!(
        "ec-rt: SetStrainComp slots=({},{}) lanes=({},{}) kin={} grid={}x{} \
         origin=({}, {}) spacing=({}, {}) values={} rc={rc}",
        slot_a,
        slot_b,
        lane_a,
        lane_b,
        kinematics,
        nx,
        ny,
        x0,
        y0,
        dx,
        dy,
        wire_values,
    );
    tracing::info!(
        subsystem = "ethercat",
        event = "set_strain_comp",
        slot_a,
        slot_b,
        nx,
        ny,
        values = wire_values,
        rc,
        "strain compensation map reconfigured"
    );
    ctx.server
        .respond(&set_strain_comp_response_frame(correlation_id, rc));
}

pub(super) fn handle_set_dynamics_model(
    ctx: &mut EndpointCtx,
    correlation_id: u32,
    msg: SetDynamicsModel,
) {
    let slots = msg.slots_count as usize;
    let modes = msg.modes_count as usize;
    let dims_consistent = msg.frame.len() == modes * slots
        && msg.mass.len() == modes
        && msg.viscous.len() == modes
        && msg.coulomb.len() == modes
        && msg.compliance.len() == modes
        && msg.pin_mass.len() == modes
        && msg.pin_zeta.len() == modes;
    let rc = if slots != ctx.num_slaves || !dims_consistent {
        crate::rt_eprintln!(
            "ec-rt: SetDynamicsModel slots_count={} modes_count={} \
             frame_len={} mass_len={} does not match {} slaves",
            msg.slots_count,
            msg.modes_count,
            msg.frame.len(),
            msg.mass.len(),
            ctx.num_slaves,
        );
        ERR_DYNAMICS_BAD_DIM
    } else {
        let pairs: Vec<crate::dynamics::PairSpec> = msg
            .pairs
            .iter()
            .map(|pair| crate::dynamics::PairSpec {
                first: pair.first as usize,
                second: pair.second as usize,
                direction_split: pair.direction_split,
            })
            .collect();
        match DynamicsModel::from_parts(
            slots,
            modes,
            &msg.frame,
            &msg.mass,
            &msg.viscous,
            &msg.coulomb,
            &msg.compliance,
            &msg.pin_mass,
            &msg.pin_zeta,
            f64::from(msg.pin_lead_us),
            &pairs,
        ) {
            Ok(model) => {
                // Size and reset the per-mode pin-rotor oscillator state to
                // the freshly installed model; a model swap re-anchors it.
                ctx.pin = super::cycle::PinState::build(&model, ctx.cycle_ns);
                ctx.dynamics = Some(model);
                0
            }
            Err(e) => {
                crate::rt_eprintln!(
                    "ec-rt: SetDynamicsModel rejected: {e:?} — keeping previous model"
                );
                ERR_DYNAMICS_REJECTED
            }
        }
    };
    crate::rt_eprintln!(
        "ec-rt: SetDynamicsModel slots={} modes={} rc={rc}",
        msg.slots_count,
        msg.modes_count,
    );
    tracing::info!(
        subsystem = "ethercat",
        event = "set_dynamics_model",
        slots_count = msg.slots_count,
        modes_count = msg.modes_count,
        rc,
        "dynamics feedforward model reconfigured"
    );
    ctx.server
        .respond(&set_dynamics_model_response_frame(correlation_id, rc));
}

fn handle_sdo_read(ctx: &mut EndpointCtx, correlation_id: u32, msg: SdoRead) {
    let num_slaves = ctx.num_slaves;
    if msg.slot as usize >= num_slaves {
        crate::rt_eprintln!(
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
        crate::rt_eprintln!(
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
            let (pos_counts, vel_counts_s) =
                (ctx.drive.position_actual(s), ctx.drive.velocity_actual(s));
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

pub(super) fn drain_pending_starts(ctx: &mut EndpointCtx) {
    let mut start_idx = 0;
    while start_idx < ctx.pending_starts.len() {
        match ctx.pending_starts[start_idx].2.try_take() {
            Some(rc) => {
                let (correlation_id, path, pending) = ctx.pending_starts.remove(start_idx);
                if rc != 0 && pending.claimed() {
                    ctx.capture.clear_failed_start();
                }
                crate::rt_eprintln!("ec-rt: StartCapture path={path} rc={rc}");
                ctx.server
                    .respond(&start_capture_response_frame(correlation_id, rc));
            }
            None => start_idx += 1,
        }
    }
}

pub(super) fn drain_pending_stops(ctx: &mut EndpointCtx) {
    let mut stop_idx = 0;
    while stop_idx < ctx.pending_stops.len() {
        match ctx.pending_stops[stop_idx].1.try_take() {
            Some(out) => {
                let (correlation_id, _) = ctx.pending_stops.remove(stop_idx);
                crate::rt_eprintln!(
                    "ec-rt: StopCapture result={} samples={} overflow={:?}",
                    out.result,
                    out.samples,
                    out.overflow_cycle
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

pub(super) fn drain_mailbox_replies(ctx: &mut EndpointCtx) {
    while let Some(reply) = ctx.mailbox.try_recv() {
        match reply {
            MailboxReply::SdoRead {
                correlation_id,
                msg,
                resp,
            } => {
                if resp.result != 0 {
                    crate::rt_eprintln!(
                        "ec-rt: SdoRead 0x{:04x}.{} failed result={}",
                        msg.index,
                        msg.subindex,
                        resp.result
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
                    crate::rt_eprintln!(
                        "ec-rt: SdoWrite 0x{:04x}.{} value={} size={} failed result={}",
                        msg.index,
                        msg.subindex,
                        msg.value,
                        msg.size,
                        resp.result
                    );
                }
                ctx.server
                    .respond(&sdo_write_response_frame(correlation_id, &resp));
            }
            MailboxReply::WriteLimits {
                correlation_id,
                rc,
                entries,
                restore,
            } => {
                let what = if restore {
                    "RestoreDriveLimits"
                } else {
                    "SetDriveLimits"
                };
                if rc != 0 {
                    crate::rt_eprintln!("ec-rt: {what} SDO write failed rc={rc} {entries:?}");
                } else {
                    crate::rt_eprintln!("ec-rt: {what} applied {entries:?}");
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
