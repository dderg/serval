use std::ops::ControlFlow;

use super::{discard_motion, EndpointCtx};
use crate::capture::{
    any_slot_out_of_range, CaptureConfig, CaptureDriveConfig, ERR_CAPTURE_BAD_DRIVE_LIST,
};
use crate::clock::monotonic_ns;
use crate::curves::AXIS_RING_CAPACITY;
use crate::dynamics::{DynamicsModel, ERR_DYNAMICS_BAD_DIM, ERR_DYNAMICS_REJECTED};
use crate::mailbox::{MailboxReply, MailboxRequest};
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
    set_dynamics_model_response_frame, set_strain_comp_response_frame, set_torque_response_frame,
    start_capture_response_frame, stop_capture_response_frame, stop_response_frame, Command,
};
use mcu_protocol::messages::{
    ArmSensorlessEndstop, PushPieces, ResonanceBuzz, SdoRead, SdoReadResponse, SdoWrite,
    SdoWriteResponse, SetDiffDamper, SetDiffTrim, SetDriveLimits, SetDynamicsModel, SetStrainComp,
    SetTorque, StartCapture, StopCaptureResponse,
};

pub(super) fn dispatch_commands(ctx: &mut EndpointCtx) -> ControlFlow<()> {
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
                // Capacity is per distinct axis, not per slave: an AWD axis
                // fans its pieces out to every slot claiming it, so extra
                // slots add no per-axis headroom. The host divides this by
                // the axis count to size its per-axis window.
                let mut distinct_axes: Vec<u8> = ctx.slave_axes.clone();
                distinct_axes.sort_unstable();
                distinct_axes.dedup();
                let total: u32 = (AXIS_RING_CAPACITY * distinct_axes.len() * 32) as u32;
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
                    discard_motion(ctx);
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
                msg,
            } => {
                handle_set_strain_comp(ctx, correlation_id, msg);
            }
            Command::SetDynamicsModel {
                correlation_id,
                msg,
            } => {
                handle_set_dynamics_model(ctx, correlation_id, msg);
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

fn handle_set_torque(ctx: &mut EndpointCtx, correlation_id: u32, msg: SetTorque) {
    let num_slaves = ctx.num_slaves;
    match ctx.gate.on_set_torque(msg.value != 0, msg.execute_at_ns) {
        CommandAction::Enable => {
            let mut enable_rc = 0;
            for s in 0..num_slaves {
                let rc = ctx.drive.enable(s);
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
                ctx.drive.shutdown_and_exit(num_slaves);
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
            ctx.drive.shutdown_and_exit(num_slaves);
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

const ERR_SEED_HOME_STREAMING: i32 = -826;

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
            .unwrap_or_else(|| ctx.drive.position_actual(usize::from(slot)));
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
                    eprintln!(
                        "ec-rt: sensorless endstop {} armed on slot {} \
                         (torque_trip={} 0.1% partner={partner:?})",
                        msg.endstop_id, msg.slot, msg.torque_trip_tenth_pct
                    );
                    0
                }
                _ => {
                    eprintln!(
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
        eprintln!(
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
    eprintln!(
        "ec-rt: SetDiffDamper slots=({},{}) gain_milli={} clamp={} 0.1% \
         lpf={} mHz lead={} us rc={rc}",
        msg.slot_a, msg.slot_b, msg.gain_milli, msg.clamp_tenths, msg.lpf_millihz, msg.lead_us,
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
        )
    };
    eprintln!(
        "ec-rt: SetDiffTrim slots=({},{}) gain_micro={} clamp={} um lpf={} mHz rc={rc}",
        msg.slot_a, msg.slot_b, msg.gain_micro, msg.clamp_um, msg.lpf_millihz,
    );
    tracing::info!(
        subsystem = "ethercat",
        event = "set_diff_trim",
        slot_a = msg.slot_a,
        slot_b = msg.slot_b,
        gain_micro = msg.gain_micro,
        clamp_um = msg.clamp_um,
        lpf_millihz = msg.lpf_millihz,
        rc,
        "differential trim reconfigured"
    );
    ctx.server
        .respond(&set_diff_trim_response_frame(correlation_id, rc));
}

fn handle_set_strain_comp(ctx: &mut EndpointCtx, correlation_id: u32, msg: SetStrainComp) {
    let lane_missing = |lane: u8| !ctx.slave_axes.iter().any(|&a| a == lane);
    let rc = if msg.nx > 0 && msg.ny > 0 && (lane_missing(msg.lane_a) || lane_missing(msg.lane_b)) {
        eprintln!(
            "ec-rt: SetStrainComp lanes ({}, {}) not present in slave_axes {:?}",
            msg.lane_a, msg.lane_b, ctx.slave_axes
        );
        ERR_COMP_BAD_LANE
    } else {
        let values: Vec<i16> = msg
            .values_um
            .iter()
            .map(|&v| v.clamp(i32::from(i16::MIN), i32::from(i16::MAX)) as i16)
            .collect();
        ctx.comp.set(
            ctx.num_slaves,
            msg.slot_a,
            msg.slot_b,
            msg.lane_a,
            msg.lane_b,
            msg.kinematics,
            msg.nx,
            msg.ny,
            f64::from(msg.x0),
            f64::from(msg.y0),
            f64::from(msg.dx),
            f64::from(msg.dy),
            &values,
        )
    };
    eprintln!(
        "ec-rt: SetStrainComp slots=({},{}) lanes=({},{}) kin={} grid={}x{} \
         origin=({}, {}) spacing=({}, {}) values={} rc={rc}",
        msg.slot_a,
        msg.slot_b,
        msg.lane_a,
        msg.lane_b,
        msg.kinematics,
        msg.nx,
        msg.ny,
        msg.x0,
        msg.y0,
        msg.dx,
        msg.dy,
        msg.values_um.len(),
    );
    tracing::info!(
        subsystem = "ethercat",
        event = "set_strain_comp",
        slot_a = msg.slot_a,
        slot_b = msg.slot_b,
        nx = msg.nx,
        ny = msg.ny,
        values = msg.values_um.len(),
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
        && msg.coulomb.len() == modes;
    let rc = if slots != ctx.num_slaves || !dims_consistent {
        eprintln!(
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
            &pairs,
        ) {
            Ok(model) => {
                ctx.dynamics = Some(model);
                0
            }
            Err(e) => {
                eprintln!("ec-rt: SetDynamicsModel rejected: {e:?} — keeping previous model");
                ERR_DYNAMICS_REJECTED
            }
        }
    };
    eprintln!(
        "ec-rt: SetDynamicsModel slots={} modes={} rc={rc}",
        msg.slots_count, msg.modes_count,
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
                eprintln!("ec-rt: StartCapture path={path} rc={rc}");
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

pub(super) fn drain_mailbox_replies(ctx: &mut EndpointCtx) {
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
