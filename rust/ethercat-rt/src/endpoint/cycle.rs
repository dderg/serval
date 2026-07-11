use std::ops::ControlFlow;

#[cfg(feature = "hw")]
use super::bringup::log_al_states;
use super::{abort_sync, discard_motion, respond_fault_heartbeat, EndpointCtx};
use crate::capture::{CaptureRecord, DriveSample, FLAG_MOTION_ACTIVE, FLAG_TORQUE_ENABLED};
use crate::claim::{eval_wkc, WkcDecision};
use crate::clock::raw_from_monotonic_ns;
use crate::curves::ENGINE_STATE_FAULT;
use crate::dynamics::clamp_torque;
use crate::scale::{mm_to_counts, CountMap};
use crate::sync::{SyncStep, ERR_SYNC_ABORTED};
use crate::torque::{TickAction, TorqueState};
use crate::wire::{endstop_trip_frame, status_heartbeat_frame, sync_pair_response_frame};
use mcu_protocol::messages::SyncPairResponse;

macro_rules! log_slot_drive_telemetry {
    ($level:ident, $event:literal, $msg:literal, $ctx:expr, $slot:expr, $t:expr,
     pre: { $($pre:tt)* }, mid: { $($mid:tt)* }, post: { $($post:tt)* }) => {
        tracing::$level!(
            subsystem = "ethercat",
            event = $event,
            $($pre)*
            slot = $slot,
            axis = $ctx.slave_axes[$slot],
            invert = $ctx.invert[$slot],
            $($mid)*
            statusword = $t.statusword,
            error_code = $t.error_code,
            target_counts = $t.target_position,
            actual = $t.position_actual,
            following_error = $t.following_error,
            velocity_actual = $t.velocity_actual,
            torque_actual = $t.torque_actual,
            velocity_offset = $t.velocity_offset,
            torque_offset = $t.torque_offset,
            $($post)*
            $msg
        )
    };
}

pub(super) fn run_cycle(ctx: &mut EndpointCtx) -> ControlFlow<()> {
    let next_flush_mono_ns = ctx.drive.cycle_time_ns() + ctx.cycle_ns as u64;
    let apply_time = raw_from_monotonic_ns(next_flush_mono_ns);

    let all_rings_empty = ctx.rings.iter().all(|r| r.is_empty());
    apply_tick_action(ctx, apply_time, all_rings_empty);

    poll_sensorless(ctx, apply_time);

    let (motion_active, all_acc, all_vel) = compute_motion_targets(ctx, apply_time);

    poll_sync(ctx, apply_time);

    handle_ring_fault(ctx);

    let (wkc, toff) = ctx.drive.cycle();

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

pub(super) fn apply_tick_action(ctx: &mut EndpointCtx, apply_time: u64, all_rings_empty: bool) {
    match ctx.gate.on_tick(apply_time, all_rings_empty) {
        TickAction::None => {}
        TickAction::ExecuteDisable => {
            eprintln!("ec-rt: scheduled torque disable executing");
            ctx.drive.disable_all(ctx.num_slaves);
            ctx.gate.disable_finished();
            for c in &mut ctx.cmaps {
                *c = None;
            }
            for t in &mut ctx.last_streamed_target {
                *t = None;
            }
            for lc in &mut ctx.last_counts {
                *lc = None;
            }
        }
        TickAction::Fault { code } => {
            eprintln!(
                "ec-rt: torque-gate fault code={code} — pieces present without torque, exiting"
            );
            respond_fault_heartbeat(ctx, ENGINE_STATE_FAULT, 0);
            ctx.drive.shutdown_and_exit(ctx.num_slaves);
        }
    }
}

fn poll_sensorless(ctx: &mut EndpointCtx, apply_time: u64) {
    let server = &mut ctx.server;
    let sensorless_tripped = ctx.sensorless.poll(
        |slot| ctx.drive.torque_actual(slot),
        |slot, endstop_id, torque| {
            eprintln!(
                "ec-rt: sensorless endstop {endstop_id} tripped on slot {slot} \
                 torque={torque} — local stop, stream halted, trip_clock={apply_time}"
            );
            server.respond(&endstop_trip_frame(endstop_id, apply_time));
        },
    );
    if sensorless_tripped {
        discard_motion(ctx);
        ctx.stream_halt.halt();
    }
}

pub(super) fn compute_motion_targets(
    ctx: &mut EndpointCtx,
    apply_time: u64,
) -> (bool, Vec<f32>, Vec<f32>) {
    let num_slaves = ctx.num_slaves;
    let mut motion_active = false;
    // The commanded analytic accel/vel the feedforward path samples are the
    // noise-free, C00.06-independent regressors the identification fit wants,
    // so they outlive the feedforward block to reach the capture record.
    let mut all_acc = vec![0f32; num_slaves];
    let mut all_vel = vec![0f32; num_slaves];
    if ctx.gate.state() != TorqueState::Enabled && ctx.buzz.active() {
        ctx.buzz.clear();
        eprintln!("ec-rt: buzz cleared — torque gate left Enabled mid-buzz");
    }
    if ctx.gate.state() == TorqueState::Enabled {
        let sp_counts = sample_slot_targets(ctx, apply_time, &mut all_acc, &mut all_vel);
        motion_active = emit_slot_commands(ctx, &sp_counts, &all_acc, &all_vel);
    } else {
        for lc in &mut ctx.last_counts {
            *lc = None;
        }
    }
    (motion_active, all_acc, all_vel)
}

// The coupled torque model needs every axis' accel/vel before any
// one slot's feedforward can be computed, so sample all slots first.
fn sample_slot_targets(
    ctx: &mut EndpointCtx,
    apply_time: u64,
    all_acc: &mut [f32],
    all_vel: &mut [f32],
) -> Vec<Option<i32>> {
    let num_slaves = ctx.num_slaves;
    let mut sp_counts: Vec<Option<i32>> = vec![None; num_slaves];
    let buzz_was_active = ctx.buzz.active();
    let buzz_sample = if buzz_was_active {
        ctx.buzz.eval(apply_time)
    } else {
        None
    };
    for s in 0..num_slaves {
        let sampled = if buzz_was_active {
            buzz_sample.and_then(|(rel_mm, vel_mm_s, acc_mm_s2)| {
                if !ctx.buzz.drives_slot(s) {
                    return None;
                }
                let sign = ctx.buzz.slot_sign(s);
                let counts = ctx.buzz.base_counts(s).wrapping_add(mm_to_counts(
                    f64::from(sign * rel_mm),
                    ctx.cmd_counts_per_mm[s],
                ));
                Some((counts, sign * vel_mm_s, sign * acc_mm_s2))
            })
        } else if let Some((pos_mm, vel_mm_s, acc_mm_s2)) = ctx.rings[s].sample(apply_time) {
            // Streaming is always relative: the stream anchors the host's
            // first commanded mm value onto the drive's commanded-counts
            // frame, so a homing set_position (host frame shift) can never
            // yank a drive. The report_anchor covers absolute position
            // queries; the drive frame itself is never used. The counts
            // side of the anchor is the last COMMANDED target, not
            // position_actual: at a homing trip both drives of a belt pair
            // are elastically wound forward by their (unequal) following
            // errors, and anchoring each at its own strained actual bakes
            // that differential in as a permanent commanded offset — the
            // pair then holds belt tension forever. Anchoring on commanded
            // counts keeps the frame continuous across Stop/ResumeStream,
            // so the retract releases the wind-up instead of freezing it.
            // position_actual is the fallback only where no commanded frame
            // exists: the first stream after torque enable, a drive fault,
            // or a sync coast — the rotor genuinely moved uncommanded there.
            let cpm = ctx.cmd_counts_per_mm[s];
            let commanded_anchor = ctx.last_streamed_target[s];
            let map = ctx.cmaps[s].get_or_insert_with(|| {
                let anchor_counts =
                    commanded_anchor.unwrap_or_else(|| ctx.drive.position_actual(s));
                CountMap::new(cpm, anchor_counts, f64::from(pos_mm))
            });
            Some((map.target_counts(f64::from(pos_mm)), vel_mm_s, acc_mm_s2))
        } else {
            None
        };
        if let Some((counts, vel_mm_s, acc_mm_s2)) = sampled {
            sp_counts[s] = Some(counts);
            let (ff_vel, ff_acc) = if ctx.ff_lead_ns[s] > 0 && !buzz_was_active {
                ctx.rings[s].peek_vel_acc(apply_time + ctx.ff_lead_ns[s])
            } else {
                (vel_mm_s, acc_mm_s2)
            };
            all_vel[s] = ff_vel;
            all_acc[s] = ff_acc;
        }
    }
    sp_counts
}

fn emit_slot_commands(
    ctx: &mut EndpointCtx,
    sp_counts: &[Option<i32>],
    all_acc: &[f32],
    all_vel: &[f32],
) -> bool {
    let num_slaves = ctx.num_slaves;
    let mut motion_active = false;
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
        if let Some(counts) = sp_counts[s] {
            let vel_offset = if ctx.velocity_ff[s] {
                (f64::from(all_vel[s]) * ctx.cmd_counts_per_mm[s]).round() as i32
            } else {
                0
            };
            let raw_ff = ctx
                .dynamics
                .as_ref()
                .map(|model| model.torque_ff(s, &acc_drive, &vel_drive));
            let torque_offset = match raw_ff {
                Some(raw) => {
                    if !raw.is_finite() {
                        fault_non_finite_torque(ctx, s, all_acc[s], all_vel[s]);
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
            ctx.drive.set_target_position(s, counts);
            ctx.drive.set_velocity_offset(s, vel_offset);
            ctx.drive.set_torque_offset(s, torque_offset);
            motion_active = true;
        } else {
            ctx.drive.set_velocity_offset(s, 0);
            ctx.drive.set_torque_offset(s, 0);
        }
    }
    motion_active
}

fn poll_sync(ctx: &mut EndpointCtx, apply_time: u64) {
    let Some(mut run) = ctx.sync.take() else {
        return;
    };
    let inputs = crate::sync::SyncInputs {
        now_ns: apply_time,
        torque_primary: ctx.drive.torque_actual(run.primary),
        torque_secondary: ctx.drive.torque_actual(run.secondary),
        velocity_secondary: ctx.drive.velocity_actual(run.secondary),
        position_secondary: ctx.drive.position_actual(run.secondary),
    };
    match run.machine.poll(&inputs) {
        SyncStep::Idle => ctx.sync = Some(run),
        SyncStep::SetPrimaryTarget(counts) => {
            ctx.drive.set_target_position(run.primary, counts);
            ctx.sync = Some(run);
        }
        SyncStep::DisableSecondary => {
            eprintln!(
                "ec-rt: sync: coasting secondary slot {} (pos={})",
                run.secondary, inputs.position_secondary
            );
            ctx.drive.disable(run.secondary);
            run.secondary_disabled = true;
            ctx.sync = Some(run);
        }
        SyncStep::EnableSecondary => {
            let rc = ctx.drive.enable(run.secondary);
            if rc != 0 {
                eprintln!(
                    "ec-rt: sync: re-enable of slot {} failed rc={rc} — parking",
                    run.secondary
                );
                ctx.sync = Some(run);
                abort_sync(ctx, ERR_SYNC_ABORTED);
                ctx.drive.shutdown_and_exit(ctx.num_slaves);
            }
            run.secondary_disabled = false;
            let settled = ctx.drive.position_actual(run.secondary);
            eprintln!(
                "ec-rt: sync: secondary slot {} re-enabled at {settled}",
                run.secondary
            );
            run.machine.enable_finished(settled);
            ctx.sync = Some(run);
        }
        SyncStep::Done(report) => finalize_sync(ctx, &run, &report),
    }
}

fn finalize_sync(ctx: &mut EndpointCtx, run: &super::SyncRun, report: &crate::sync::SyncReport) {
    if report.secondary_reseeded {
        ctx.last_counts[run.secondary] = None;
        ctx.last_streamed_target[run.secondary] = None;
        ctx.cmaps[run.secondary] = None;
        // The rotor turned by the released delta while the axis stood still,
        // so the same host-frame mm now maps to counts+delta.
        if let Some((anchor_counts, anchor_mm)) = ctx.report_anchor[run.secondary] {
            ctx.report_anchor[run.secondary] = Some((
                anchor_counts.wrapping_add(report.released_delta_counts),
                anchor_mm,
            ));
        }
    }
    tracing::info!(
        subsystem = "ethercat",
        event = "pair_sync_done",
        result = report.result,
        primary = run.primary,
        secondary = run.secondary,
        torque_baseline_primary = report.torque_baseline_primary,
        torque_baseline_secondary = report.torque_baseline_secondary,
        torque_released = report.torque_released,
        torque_dithered = report.torque_dithered,
        torque_final_primary = report.torque_final_primary,
        torque_final_secondary = report.torque_final_secondary,
        released_delta_counts = report.released_delta_counts,
        "belt pair sync finished"
    );
    eprintln!(
        "ec-rt: sync done result={} baseline=({}, {}) released={} dithered={} \
         final=({}, {}) delta={} counts",
        report.result,
        report.torque_baseline_primary,
        report.torque_baseline_secondary,
        report.torque_released,
        report.torque_dithered,
        report.torque_final_primary,
        report.torque_final_secondary,
        report.released_delta_counts
    );
    let resp = SyncPairResponse {
        result: report.result,
        primary_slot: run.primary as u8,
        secondary_slot: run.secondary as u8,
        torque_baseline_primary: report.torque_baseline_primary,
        torque_baseline_secondary: report.torque_baseline_secondary,
        torque_released: report.torque_released,
        torque_dithered: report.torque_dithered,
        torque_final_primary: report.torque_final_primary,
        torque_final_secondary: report.torque_final_secondary,
        released_delta_counts: report.released_delta_counts,
    };
    ctx.server
        .respond(&sync_pair_response_frame(run.correlation_id, &resp));
}

fn fault_non_finite_torque(ctx: &mut EndpointCtx, slot: usize, acc: f32, vel: f32) -> ! {
    eprintln!(
        "ec-rt: FAULT non-finite torque FF on slot {slot} \
         (acc={acc} vel={vel}) — disabling"
    );
    respond_fault_heartbeat(ctx, ENGINE_STATE_FAULT, 0);
    ctx.drive.shutdown_and_exit(ctx.num_slaves);
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
        let retired = respond_fault_heartbeat(ctx, ENGINE_STATE_FAULT, fault_code_u16);

        #[cfg(feature = "hw")]
        {
            let _ = retired;
            eprintln!("ec-rt: disabling drives (hw safety backstop)");
            ctx.drive.shutdown_and_exit(ctx.num_slaves);
        }

        #[cfg(not(feature = "hw"))]
        {
            ctx.last_sent_retired = retired.iter().sum();
            ctx.heartbeat_sent = true;
        }
    }
}

fn handle_drive_fault(ctx: &mut EndpointCtx) {
    let num_slaves = ctx.num_slaves;
    let drive_fault = (0..num_slaves).find_map(|s| {
        let e = ctx.drive.error_code(s);
        if e != 0 {
            Some((s, e))
        } else {
            None
        }
    });
    if let Some((slot, err)) = drive_fault {
        if ctx.gate.state() != TorqueState::Faulted {
            abort_sync(ctx, ERR_SYNC_ABORTED);
            eprintln!(
                "ec-rt: DRIVE FAULT slot {slot} err=0x{err:04x} — parking, reporting via heartbeat"
            );
            for d in 0..num_slaves {
                let t = ctx.drive.telemetry(d);
                let last_cmd = ctx.last_counts[d].unwrap_or(t.target_position);
                log_slot_drive_telemetry!(
                    error, "drive_fault", "drive fault — per-slot snapshot", ctx, d, t,
                    pre: { faulted_slot = slot, },
                    mid: { err = err, },
                    post: {
                        last_cmd_target = last_cmd,
                        last_increment = i64::from(t.target_position) - i64::from(last_cmd),
                    }
                );
            }
            ctx.gate.on_drive_fault();
            discard_motion(ctx);
            for t in &mut ctx.last_streamed_target {
                *t = None;
            }
            ctx.latched_drive_err = err;
            let retired = respond_fault_heartbeat(ctx, 0, err);
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
            let t = ctx.drive.telemetry(usize::from(slot));
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
            #[cfg(feature = "hw")]
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
            ctx.drive.dump_al_state();
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
            let t = ctx.drive.telemetry(s);
            log_slot_drive_telemetry!(
                info, "telemetry", "per-slot drive telemetry", ctx, s, t,
                pre: {},
                mid: { wkc, toff, },
                post: {
                    motion = !all_empty_now,
                    ff_sat = ctx.ff_saturation,
                    framed = ctx.report_anchor[s].is_some(),
                }
            );
        }
        if ctx.gate.state() == TorqueState::Faulted {
            let latched_drive_err = ctx.latched_drive_err;
            respond_fault_heartbeat(ctx, 0, latched_drive_err);
        }
    }
}
