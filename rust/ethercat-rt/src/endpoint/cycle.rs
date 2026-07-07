use std::ops::ControlFlow;

use super::bringup::log_al_states;
use super::EndpointCtx;
use crate::capture::{CaptureRecord, DriveSample, FLAG_MOTION_ACTIVE, FLAG_TORQUE_ENABLED};
use crate::claim::{eval_wkc, WkcDecision};
use crate::clock::raw_from_monotonic_ns;
use crate::curves::ENGINE_STATE_FAULT;
use crate::dynamics::clamp_torque;
use crate::ffi;
use crate::scale::{mm_to_counts, CountMap};
use crate::torque::{TickAction, TorqueState};
use crate::wire::{endstop_trip_frame, status_heartbeat_frame};

pub(super) fn run_cycle(ctx: &mut EndpointCtx) -> ControlFlow<()> {
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
