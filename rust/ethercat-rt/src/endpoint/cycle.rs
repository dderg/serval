use std::ops::ControlFlow;

#[cfg(feature = "hw")]
use super::bringup::log_al_states;
use super::{discard_motion, respond_fault_heartbeat, EndpointCtx};
use crate::capture::{CaptureRecord, DriveSample, FLAG_MOTION_ACTIVE, FLAG_TORQUE_ENABLED};
use crate::claim::{eval_wkc, WkcDecision};
use crate::clock::raw_from_monotonic_ns;
use crate::curves::ENGINE_STATE_FAULT;
use crate::dynamics::{clamp_torque, DynamicsModel};
use crate::scale::{mm_to_counts, CountMap};
use crate::torque::{TickAction, TorqueState};
use crate::wire::{endstop_trip_frame, status_heartbeat_frame};

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
    let cycle_start = std::time::Instant::now();
    let next_flush_mono_ns = ctx.drive.cycle_time_ns() + ctx.cycle_ns as u64;
    let apply_time = raw_from_monotonic_ns(next_flush_mono_ns);

    let all_rings_empty = ctx.rings.iter().all(|r| r.is_empty());
    apply_tick_action(ctx, apply_time, all_rings_empty);

    poll_sensorless(ctx, apply_time);

    let sample_time = apply_time + ctx.group_delay_ns;
    let (motion_active, all_acc, all_vel) = compute_motion_targets(ctx, sample_time);

    ctx.sensorless
        .record_commanded(apply_time, |slot| ctx.last_counts[slot]);

    handle_ring_fault(ctx);

    let exchange = std::time::Instant::now();
    ctx.last_pre_cycle_ns = (exchange - cycle_start).as_nanos() as i64;
    ctx.last_inter_exchange_ns = ctx
        .prev_exchange_return
        .map_or(0, |t| (exchange - t).as_nanos() as i64);
    ctx.pre_cycle_max_ns = ctx.pre_cycle_max_ns.max(ctx.last_pre_cycle_ns);
    ctx.inter_exchange_max_ns = ctx.inter_exchange_max_ns.max(ctx.last_inter_exchange_ns);
    let (wkc, toff) = ctx.drive.cycle();
    let exchange_return = std::time::Instant::now();
    let exchange_ns = (exchange_return - exchange).as_nanos() as i64;
    ctx.prev_exchange_return = Some(exchange_return);

    let (wake_late_ns, recv_ns, process_ns, send_ns) = ctx.drive.cycle_stage_ns();
    ctx.last_wake_late_ns = wake_late_ns;
    ctx.last_recv_ns = recv_ns;
    ctx.last_process_ns = process_ns;
    ctx.last_send_ns = send_ns;
    ctx.wake_late_max_ns = ctx.wake_late_max_ns.max(wake_late_ns);
    ctx.recv_max_ns = ctx.recv_max_ns.max(recv_ns);
    ctx.process_max_ns = ctx.process_max_ns.max(process_ns);
    ctx.send_max_ns = ctx.send_max_ns.max(send_ns);
    ctx.last_lateness_ns = toff;
    police_frame_timing(ctx, toff);
    ctx.prev_exchange_ns = exchange_ns;
    let fault_start = std::time::Instant::now();
    handle_drive_fault(ctx);

    ctx.cycle_index += 1;
    let capture_start = std::time::Instant::now();
    record_capture_sample(ctx, motion_active, &all_acc, &all_vel);
    let wkc_start = std::time::Instant::now();

    if evaluate_wkc(ctx, wkc).is_break() {
        return ControlFlow::Break(());
    }

    let heartbeat_start = std::time::Instant::now();
    emit_heartbeat(ctx);
    let telemetry_start = std::time::Instant::now();
    emit_periodic_telemetry(ctx, wkc, toff);
    let post_end = std::time::Instant::now();

    ctx.last_fault_ns = (capture_start - fault_start).as_nanos() as i64;
    ctx.last_capture_ns = (wkc_start - capture_start).as_nanos() as i64;
    ctx.last_wkc_ns = (heartbeat_start - wkc_start).as_nanos() as i64;
    ctx.last_heartbeat_ns = (telemetry_start - heartbeat_start).as_nanos() as i64;
    ctx.last_telemetry_ns = (post_end - telemetry_start).as_nanos() as i64;
    ctx.fault_max_ns = ctx.fault_max_ns.max(ctx.last_fault_ns);
    ctx.capture_max_ns = ctx.capture_max_ns.max(ctx.last_capture_ns);
    ctx.wkc_max_ns = ctx.wkc_max_ns.max(ctx.last_wkc_ns);
    ctx.heartbeat_max_ns = ctx.heartbeat_max_ns.max(ctx.last_heartbeat_ns);
    ctx.telemetry_max_ns = ctx.telemetry_max_ns.max(ctx.last_telemetry_ns);

    ctx.last_post_cycle_ns = (post_end - exchange_return).as_nanos() as i64;
    ctx.post_cycle_max_ns = ctx.post_cycle_max_ns.max(ctx.last_post_cycle_ns);

    ControlFlow::Continue(())
}

pub(super) fn apply_tick_action(ctx: &mut EndpointCtx, apply_time: u64, all_rings_empty: bool) {
    match ctx.gate.on_tick(apply_time, all_rings_empty) {
        TickAction::None => {}
        TickAction::ExecuteDisable => {
            eprintln!("ec-rt: scheduled torque disable executing");
            ctx.drive.disable_all();
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
            ctx.drive.shutdown_and_exit();
        }
    }
}

fn poll_sensorless(ctx: &mut EndpointCtx, apply_time: u64) {
    let server = &mut ctx.server;
    let drive = &ctx.drive;
    let cmd_counts_per_mm = &ctx.cmd_counts_per_mm;
    let sensorless_tripped = ctx.sensorless.poll(
        apply_time,
        |slot| {
            let dir = cmd_counts_per_mm[slot].signum() as i32;
            (dir * i32::from(drive.torque_actual(slot))) as i16
        },
        |slot| drive.position_actual(slot),
        |slot, endstop_id, torque, contact_clock| {
            let windup_ns = apply_time.saturating_sub(contact_clock);
            crate::rt_eprintln!(
                "ec-rt: sensorless endstop {endstop_id} tripped on slot {slot} \
                 torque={torque} — local stop, stream halted, \
                 contact_clock={contact_clock} ({windup_ns} ns before the trip)"
            );
            tracing::info!(
                subsystem = "ethercat",
                event = "sensorless_trip",
                slot,
                endstop_id,
                torque,
                trip_clock = apply_time,
                contact_clock,
                windup_ns,
                "sensorless endstop tripped — reporting commanded-crossing clock"
            );
            server.respond(&endstop_trip_frame(endstop_id, contact_clock));
        },
    );
    if sensorless_tripped {
        discard_motion(ctx);
        ctx.stream_halt.halt();
    }
}

pub(super) fn compute_motion_targets(
    ctx: &mut EndpointCtx,
    sample_time: u64,
) -> (bool, Vec<f32>, Vec<f32>) {
    let num_slaves = ctx.num_slaves;
    let mut motion_active = false;
    // The commanded analytic accel/vel the feedforward path samples are the
    // noise-free, C00.06-independent regressors the identification fit wants,
    // so they outlive the feedforward block to reach the capture record.
    let mut all_acc = vec![0f32; num_slaves];
    let mut all_vel = vec![0f32; num_slaves];
    // Jerk/snap feed only the compliance correction: the corrected rotor
    // trajectory x+c·a needs v+c·j on 60B1h and a+c·s in the torque model.
    let mut all_jrk = vec![0f32; num_slaves];
    let mut all_snp = vec![0f32; num_slaves];
    if ctx.gate.state() != TorqueState::Enabled && ctx.buzz.active() {
        ctx.buzz.clear();
        crate::rt_eprintln!("ec-rt: buzz cleared — torque gate left Enabled mid-buzz");
    }
    if ctx.gate.state() == TorqueState::Enabled {
        let mut lane_mm = vec![None; num_slaves];
        let sp_counts = sample_slot_targets(
            ctx,
            sample_time,
            &mut all_acc,
            &mut all_vel,
            &mut all_jrk,
            &mut all_snp,
            &mut lane_mm,
        );
        motion_active = emit_slot_commands(
            ctx, &sp_counts, &lane_mm, &all_acc, &all_vel, &all_jrk, &all_snp,
        );
    } else {
        ctx.damper.reset_filters();
        ctx.trim.reset();
        ctx.comp.reset_applied();
        ctx.pin.reset();
        for lc in &mut ctx.last_counts {
            *lc = None;
        }
        for off in &mut ctx.last_written_offset {
            *off = 0;
        }
    }
    (motion_active, all_acc, all_vel)
}

// The coupled torque model needs every axis' accel/vel before any
// one slot's feedforward can be computed, so sample all slots first.
#[allow(clippy::too_many_arguments)]
fn sample_slot_targets(
    ctx: &mut EndpointCtx,
    sample_time: u64,
    all_acc: &mut [f32],
    all_vel: &mut [f32],
    all_jrk: &mut [f32],
    all_snp: &mut [f32],
    lane_mm: &mut [Option<f64>],
) -> Vec<Option<i32>> {
    let num_slaves = ctx.num_slaves;
    let mut sp_counts: Vec<Option<i32>> = vec![None; num_slaves];
    let buzz_was_active = ctx.buzz.active();
    let buzz_sample = if buzz_was_active {
        ctx.buzz.eval(sample_time)
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
        } else if let Some((pos_mm, vel_mm_s, acc_mm_s2)) = ctx.rings[s].sample(sample_time) {
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
            lane_mm[s] = Some(f64::from(pos_mm));
            Some((map.target_counts(f64::from(pos_mm)), vel_mm_s, acc_mm_s2))
        } else {
            None
        };
        if let Some((counts, vel_mm_s, acc_mm_s2)) = sampled {
            sp_counts[s] = Some(counts);
            // A buzz has no ring piece behind it, so its jerk/snap are
            // unknown — leave them zero; the compliance correction is
            // gated off during a buzz anyway.
            let (ff_vel, ff_acc, ff_jrk, ff_snp) = if buzz_was_active {
                (vel_mm_s, acc_mm_s2, 0.0, 0.0)
            } else if ctx.ff_lead_ns[s] > 0 {
                ctx.rings[s].peek_kin(sample_time + ctx.ff_lead_ns[s])
            } else {
                let (jrk, snp) = ctx.rings[s].jerk_snap(sample_time);
                (vel_mm_s, acc_mm_s2, jrk, snp)
            };
            all_vel[s] = ff_vel;
            all_acc[s] = ff_acc;
            all_jrk[s] = ff_jrk;
            all_snp[s] = ff_snp;
        }
    }
    sp_counts
}

// The damper is feedback, not feedforward: it differentiates the pair's raw
// encoder positions from the previous exchange's input image. It must NOT
// read 606Ch — the drive's velocity estimate carries an estimator lag that
// pushes the feedback past 90 degrees inside the very band the damper
// targets, turning it into a pump; the bank's lead term compensates the
// remaining transport and torque-path lag instead.
fn damper_torque_tenths(ctx: &mut EndpointCtx) -> Vec<f32> {
    let mut host_frame = vec![0f32; ctx.num_slaves];
    if !ctx.damper.active() {
        return host_frame;
    }
    let pos_mm: Vec<f64> = (0..ctx.num_slaves)
        .map(|s| f64::from(ctx.drive.position_actual(s)) / ctx.cmd_counts_per_mm[s])
        .collect();
    ctx.damper.accumulate(&pos_mm, &mut host_frame);
    for (s, torque) in host_frame.iter_mut().enumerate() {
        *torque *= ctx.cmd_counts_per_mm[s].signum() as f32;
    }
    host_frame
}

// The trim integrates the pair's mechanical-frame differential torque into
// an antisymmetric target offset, but ONLY at commanded standstill: during
// motion a differential torque is legitimate (feedforward, direction- and
// load-dependent inner-loop effort). A slot is quiescent when its ring is
// empty — pieces land in the ring at least the feedforward lead before
// their start, so an empty ring also proves no lead-window torque is being
// commanded — no buzz is running, and its strain-comp pair is not mid-ramp
// (the ramp changes the differential torque by design). The offset
// deliberately stays OUT of last_counts / last_streamed_target: command
// anchors live in the raw stream frame, so a sync or re-anchor never bakes
// a live trim offset in.
const TRIM_STATE_LOG_CYCLES: u64 = 4000;

fn trim_quiescent_slots(ctx: &EndpointCtx) -> Vec<bool> {
    if ctx.buzz.active() {
        return vec![false; ctx.num_slaves];
    }
    let mut quiescent: Vec<bool> = ctx.rings.iter().map(|r| r.is_empty()).collect();
    for (slot_a, slot_b, applied_mm, target_mm, _bias_mm) in ctx.comp.snapshot() {
        if (applied_mm - target_mm).abs() > f64::EPSILON {
            quiescent[slot_a] = false;
            quiescent[slot_b] = false;
        }
    }
    quiescent
}

fn trim_offset_counts(ctx: &mut EndpointCtx) -> Vec<i32> {
    let mut counts = vec![0i32; ctx.num_slaves];
    if !ctx.trim.active() {
        return counts;
    }
    let torque_mech: Vec<f64> = (0..ctx.num_slaves)
        .map(|s| f64::from(ctx.drive.torque_actual(s)) * ctx.cmd_counts_per_mm[s].signum())
        .collect();
    let quiescent = trim_quiescent_slots(ctx);
    let mut offset_mm = vec![0f64; ctx.num_slaves];
    ctx.trim.update(&torque_mech, &quiescent, &mut offset_mm);
    if let Some((slot_a, slot_b)) = ctx.trim.drain_clamp_warning() {
        tracing::warn!(
            subsystem = "ethercat",
            event = "diff_trim_clamped",
            slot_a,
            slot_b,
            "differential trim offset hit its clamp — residual pair fight \
             exceeds the trim's authority"
        );
    }
    if ctx.cycle_index % TRIM_STATE_LOG_CYCLES == 0 {
        for (slot_a, slot_b, offset_mm, filt_tenths, integrating) in ctx.trim.snapshot() {
            tracing::info!(
                subsystem = "ethercat",
                event = "diff_trim_state",
                slot_a,
                slot_b,
                offset_um = (offset_mm * 1e3).round() as i64,
                diff_torque_tenths = filt_tenths.round() as i64,
                quiescent_a = quiescent[slot_a],
                quiescent_b = quiescent[slot_b],
                integrating,
                "differential trim state"
            );
        }
    }
    for s in 0..ctx.num_slaves {
        counts[s] = (offset_mm[s] * ctx.cmd_counts_per_mm[s]).round() as i32;
    }
    counts
}

// The strain compensation is feedforward on the COMMANDED carriage position:
// a per-pair antisymmetric offset interpolated from the calibrated map. Like
// the trim it stays out of last_counts / last_streamed_target so a sync or
// re-anchor never bakes a live offset in.
fn comp_offset_counts(ctx: &mut EndpointCtx, lane_mm: &[Option<f64>]) -> Vec<i32> {
    let mut counts = vec![0i32; ctx.num_slaves];
    if !ctx.comp.active() {
        return counts;
    }
    let mut offset_mm = vec![0f64; ctx.num_slaves];
    ctx.comp.update(lane_mm, &ctx.slave_axes, &mut offset_mm);
    for s in 0..ctx.num_slaves {
        counts[s] = (offset_mm[s] * ctx.cmd_counts_per_mm[s]).round() as i32;
    }
    if ctx.cycle_index % TRIM_STATE_LOG_CYCLES == 0 {
        for (slot_a, slot_b, applied_mm, target_mm, bias_mm) in ctx.comp.snapshot() {
            tracing::info!(
                subsystem = "ethercat",
                event = "strain_comp_state",
                slot_a,
                slot_b,
                applied_um = (applied_mm * 1e3).round() as i64,
                target_um = (target_mm * 1e3).round() as i64,
                anchor_bias_um = (bias_mm * 1e3).round() as i64,
                "strain compensation state"
            );
        }
    }
    counts
}

#[allow(clippy::too_many_arguments)]
fn emit_slot_commands(
    ctx: &mut EndpointCtx,
    sp_counts: &[Option<i32>],
    lane_mm: &[Option<f64>],
    all_acc: &[f32],
    all_vel: &[f32],
    all_jrk: &[f32],
    all_snp: &[f32],
) -> bool {
    let num_slaves = ctx.num_slaves;
    let mut motion_active = false;
    let damper_tenths = damper_torque_tenths(ctx);
    let trim_counts = trim_offset_counts(ctx);
    let comp_counts = comp_offset_counts(ctx, lane_mm);
    // The dynamics profile is fitted in the drive frame (the capture
    // flips each drive's commanded kinematics by its direction sign),
    // so the model must be evaluated on drive-frame vectors — flipping
    // only the output torque by the slot's own sign would negate the
    // off-diagonal coupling terms whenever the drives' inverts differ.
    let dirs: Vec<f32> = (0..num_slaves)
        .map(|s| ctx.cmd_counts_per_mm.get(s).map_or(1.0, |c| c.signum()) as f32)
        .collect();
    let dir_mul = |v: &[f32]| -> Vec<f32> { v.iter().zip(&dirs).map(|(x, d)| x * d).collect() };
    let acc_drive = dir_mul(all_acc);
    let vel_drive = dir_mul(all_vel);
    // Coulomb is a mode-space quantity, so a buzz on any slot flips the sign
    // of a mode velocity that other slots share; drop Coulomb for every slot
    // whenever the buzz drives any slot this cycle.
    let buzz_active = ctx.buzz.active();
    // Belt-compliance correction: the rotor leads the trajectory by
    // G·accel so the carriage — one belt stretch behind — lands exactly
    // on it. The corrected rotor kinematics are v+G·j and a+G·s, so the
    // velocity offset and the torque model see the corrected vectors.
    // Skipped during a buzz: its micrometre excitation has no ring piece
    // (jerk/snap unknown) and must stay a pure position wiggle.
    let compliance = ctx
        .dynamics
        .as_ref()
        .filter(|m| m.has_compliance() && !buzz_active)
        .map(|model| {
            let jrk_drive = dir_mul(all_jrk);
            let snp_drive = dir_mul(all_snp);
            let mut pos_lead = vec![0f32; num_slaves];
            let mut vel_extra = vec![0f32; num_slaves];
            let mut acc_extra = vec![0f32; num_slaves];
            model.compliance_apply(&acc_drive, &mut pos_lead);
            model.compliance_apply(&jrk_drive, &mut vel_extra);
            model.compliance_apply(&snp_drive, &mut acc_extra);
            (pos_lead, vel_extra, acc_extra)
        });
    let (acc_ff, vel_ff): (Vec<f32>, Vec<f32>) = match &compliance {
        Some((_, vel_extra, acc_extra)) => (
            acc_drive
                .iter()
                .zip(acc_extra)
                .map(|(a, e)| a + e)
                .collect(),
            vel_drive
                .iter()
                .zip(vel_extra)
                .map(|(v, e)| v + e)
                .collect(),
        ),
        None => (acc_drive.clone(), vel_drive.clone()),
    };
    // Pin-rotor (mode A) torque hold: advance each pinned mode's predicted-
    // deflection oscillator once this cycle and refill the slot-torque
    // buffer. Runs through a buzz — the analytic buzz accel already rides in
    // acc_drive (all_acc carries it for every driven slot), so the pinned
    // modes see forcing = trajectory accel + buzz accel and keep the pin
    // torque and residual demod live; only the compliance position-lead (B)
    // stays suppressed during a buzz. Re-anchored on the motion-inactive→
    // active edge; costs nothing when no mode is pinned.
    if ctx.pin.active() {
        let streaming = sp_counts.iter().any(Option::is_some);
        let ferr_drive: Vec<f32> = (0..num_slaves)
            .map(|s| {
                (f64::from(ctx.drive.telemetry(s).following_error) / ctx.counts_per_mm[s]) as f32
            })
            .collect();
        ctx.pin.step(&acc_drive, &ferr_drive, streaming);
    }
    for s in 0..num_slaves {
        if let Some(counts) = sp_counts[s] {
            let vel_offset = if ctx.velocity_ff[s] {
                // Drive-frame extra × cmd cpm needs the drive sign folded
                // back in: extra·dir·cpm = extra·|cpm|.
                let extra = compliance
                    .as_ref()
                    .map_or(0.0, |(_, vel_extra, _)| vel_extra[s] * dirs[s]);
                ((f64::from(all_vel[s]) + f64::from(extra)) * ctx.cmd_counts_per_mm[s]).round()
                    as i32
            } else {
                0
            };
            let raw_ff = ctx.dynamics.as_ref().map(|model| {
                let base = if buzz_active {
                    model.torque_ff_without_coulomb(s, &acc_ff, &vel_ff)
                } else {
                    model.torque_ff(s, &acc_ff, &vel_ff)
                };
                base + ctx.pin.slot_torque_at(s)
            });
            let torque_offset = match raw_ff {
                Some(raw) => {
                    if !raw.is_finite() {
                        fault_non_finite_torque(ctx, s, all_acc[s], all_vel[s]);
                    }
                    clamp_torque(
                        raw + damper_tenths[s],
                        ctx.torque_clamp_tenths[s],
                        &mut ctx.ff_saturation,
                    )
                }
                None => damper_tenths[s].round() as i16,
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
            let lead_counts = match &compliance {
                Some((pos_lead, _, _)) => {
                    if !pos_lead[s].is_finite() {
                        fault_non_finite_torque(ctx, s, all_acc[s], all_vel[s]);
                    }
                    mm_to_counts(f64::from(pos_lead[s] * dirs[s]), ctx.cmd_counts_per_mm[s])
                }
                None => 0i32,
            };
            let offset = trim_counts[s]
                .wrapping_add(comp_counts[s])
                .wrapping_add(lead_counts);
            ctx.drive
                .set_target_position(s, counts.wrapping_add(offset));
            ctx.last_written_offset[s] = offset;
            ctx.drive.set_velocity_offset(s, vel_offset);
            ctx.drive.set_torque_offset(s, torque_offset);
            motion_active = true;
        } else {
            ctx.drive.set_velocity_offset(s, 0);
            ctx.drive
                .set_torque_offset(s, damper_tenths[s].round() as i16);
            // A held slot follows the compensation and trim too: the
            // stiffness probe steps offsets at standstill, a map ramping
            // while parked must move the held target now so the next stream
            // doesn't, and the trim does all of its integrating exactly
            // here — at commanded standstill. The base is the drive's own
            // output-image target (always seeded — by enable if nothing
            // ever streamed) minus the offsets baked into the last write;
            // last_counts is no good here, Stop and ResumeStream clear it
            // while the drive keeps holding.
            if ctx.comp.active() || ctx.trim.active() {
                let offset = trim_counts[s].wrapping_add(comp_counts[s]);
                let base = ctx
                    .drive
                    .telemetry(s)
                    .target_position
                    .wrapping_sub(ctx.last_written_offset[s]);
                ctx.drive.set_target_position(s, base.wrapping_add(offset));
                ctx.last_written_offset[s] = offset;
            }
        }
    }
    motion_active
}

fn fault_non_finite_torque(ctx: &mut EndpointCtx, slot: usize, acc: f32, vel: f32) -> ! {
    crate::rt_eprintln!(
        "ec-rt: FAULT non-finite torque FF on slot {slot} \
         (acc={acc} vel={vel}) — disabling"
    );
    respond_fault_heartbeat(ctx, ENGINE_STATE_FAULT, 0);
    ctx.drive.shutdown_and_exit();
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
            ctx.drive.shutdown_and_exit();
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
            crate::rt_eprintln!(
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

pub(super) const FRAME_LATE_FAULT_CODE: u16 = 0xFE10;
pub(super) const CYCLE_SKIP_FAULT_CODE: u16 = 0xFE11;

/// The cycle reports each frame's lateness relative to the SYNC0 latch (wake
/// grid + half cycle — overrun skips stay on the grid, so the phase holds all
/// run); positive means the drives latched last cycle's target. A skip means
/// whole cycles went by with no frame at all — the drives coasted on a stale
/// target — so it faults whenever a tolerance is set, regardless of the
/// lateness measured on the cycle that finally ran.
///
/// The first cycle only arms: the gap between the last bringup exchange and
/// the loop's first wake always costs one catch-up skip, which is benign —
/// nothing is armed yet and the grid phase is preserved.
pub(super) fn police_frame_timing(ctx: &mut EndpointCtx, lateness_ns: i64) {
    let reanchors = ctx.drive.reanchor_count();
    if !ctx.timing_armed {
        ctx.timing_armed = true;
        ctx.baseline_reanchor_count = reanchors;
        return;
    }
    let reanchored = reanchors != ctx.baseline_reanchor_count;
    if reanchored {
        ctx.skip_count_policed = ctx
            .skip_count_policed
            .wrapping_add(reanchors.wrapping_sub(ctx.baseline_reanchor_count));
        ctx.baseline_reanchor_count = reanchors;
        tracing::error!(
            subsystem = "ethercat",
            event = "cycle_skip",
            total = reanchors,
            behind_ns = ctx.drive.last_reanchor_behind_ns(),
            dispatch_ns = ctx.last_dispatch_ns,
            pre_work_ns = ctx.last_pre_work_ns,
            prev_exchange_ns = ctx.prev_exchange_ns,
            wake_late_ns = ctx.last_wake_late_ns,
            recv_ns = ctx.last_recv_ns,
            process_ns = ctx.last_process_ns,
            send_ns = ctx.last_send_ns,
            pre_cycle_ns = ctx.last_pre_cycle_ns,
            post_cycle_ns = ctx.last_post_cycle_ns,
            inter_exchange_ns = ctx.last_inter_exchange_ns,
            fault_ns = ctx.last_fault_ns,
            capture_ns = ctx.last_capture_ns,
            wkc_ns = ctx.last_wkc_ns,
            heartbeat_ns = ctx.last_heartbeat_ns,
            telemetry_ns = ctx.last_telemetry_ns,
            "cycle overran a full period and skipped forward on the grid — \
             the drives coasted on a stale target for the missed cycles \
             (behind_ns is the true stall magnitude; inter_exchange_ns spans \
             every non-exchange nanosecond since the previous exchange, \
             pre_cycle/post_cycle are its measured sub-spans)"
        );
    }
    if lateness_ns > 0 {
        ctx.late_frames += 1;
        ctx.late_frames_total = ctx.late_frames_total.wrapping_add(1);
        ctx.late_max_ns = ctx.late_max_ns.max(lateness_ns);
    }
    let Some(tolerance_ns) = ctx.late_tolerance_ns else {
        return;
    };
    if ctx.gate.state() == TorqueState::Faulted {
        return;
    }
    let code = if reanchored {
        CYCLE_SKIP_FAULT_CODE
    } else if lateness_ns > tolerance_ns {
        FRAME_LATE_FAULT_CODE
    } else {
        return;
    };
    crate::rt_eprintln!(
        "ec-rt: FRAME TIMING FAULT lateness={lateness_ns} ns \
         tolerance={tolerance_ns} ns reanchored={reanchored} — parking, \
         reporting via heartbeat"
    );
    tracing::error!(
        subsystem = "ethercat",
        event = "frame_timing_fault",
        lateness_ns,
        tolerance_ns,
        reanchored,
        code,
        dispatch_ns = ctx.last_dispatch_ns,
        pre_work_ns = ctx.last_pre_work_ns,
        prev_exchange_ns = ctx.prev_exchange_ns,
        wake_late_ns = ctx.last_wake_late_ns,
        recv_ns = ctx.last_recv_ns,
        process_ns = ctx.last_process_ns,
        send_ns = ctx.last_send_ns,
        pre_cycle_ns = ctx.last_pre_cycle_ns,
        post_cycle_ns = ctx.last_post_cycle_ns,
        inter_exchange_ns = ctx.last_inter_exchange_ns,
        fault_ns = ctx.last_fault_ns,
        capture_ns = ctx.last_capture_ns,
        wkc_ns = ctx.last_wkc_ns,
        heartbeat_ns = ctx.last_heartbeat_ns,
        telemetry_ns = ctx.last_telemetry_ns,
        "frame timing exceeded the configured late tolerance — parking"
    );
    ctx.gate.on_drive_fault();
    discard_motion(ctx);
    for t in &mut ctx.last_streamed_target {
        *t = None;
    }
    ctx.latched_drive_err = code;
    let retired = respond_fault_heartbeat(ctx, 0, code);
    ctx.last_sent_retired = retired.iter().sum();
    ctx.heartbeat_sent = true;
}

fn record_capture_sample(
    ctx: &mut EndpointCtx,
    motion_active: bool,
    all_acc: &[f32],
    all_vel: &[f32],
) {
    let want_file = ctx.capture.is_active();
    let want_tap = ctx.live_tap.has_subscriber();
    if !want_file && !want_tap {
        return;
    }
    let mut flags = 0u8;
    if ctx.gate.state() == TorqueState::Enabled {
        flags |= FLAG_TORQUE_ENABLED;
    }
    if motion_active {
        flags |= FLAG_MOTION_ACTIVE;
    }
    let skip_count = ctx.skip_count_policed;
    let late_frames = ctx.late_frames_total;
    let frame_lateness_ns = ctx
        .last_lateness_ns
        .clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32;
    let EndpointCtx {
        drive,
        cmd_counts_per_mm,
        capture_slots,
        tap_slots,
        capture,
        live_tap,
        cycle_index,
        pin,
        ..
    } = ctx;
    let build = |slots: &[u8]| {
        let mut record = CaptureRecord::new(*cycle_index, flags);
        record.skip_count = skip_count;
        record.late_frames = late_frames;
        record.frame_lateness_ns = frame_lateness_ns;
        record.drive_count = slots.len() as u8;
        for (i, &slot) in slots.iter().enumerate() {
            let t = drive.telemetry(usize::from(slot));
            // The commanded kinematics are sampled in planner-stream frame;
            // flip them into the drive frame (as cmd_counts_per_mm's sign
            // does for the target) so they are sign-consistent with the
            // drive-frame position/velocity/torque channels — otherwise an
            // inverted axis fits negative inertia.
            let dir = cmd_counts_per_mm[usize::from(slot)].signum() as f32;
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
                pin_res_re: pin.residual_for_slot(usize::from(slot)).0,
                pin_res_im: pin.residual_for_slot(usize::from(slot)).1,
            };
        }
        record
    };
    if want_file {
        let record = build(capture_slots);
        capture.push(record);
    }
    if want_tap {
        let record = build(tap_slots);
        live_tap.push(record);
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
            crate::rt_eprintln!("ec-rt: heartbeat retired={retired:?}");
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
        if ctx.late_frames > 0 {
            tracing::warn!(
                subsystem = "ethercat",
                event = "frame_late",
                count = ctx.late_frames,
                max_lateness_ns = ctx.late_max_ns,
                "frames finished sending after the nominal SYNC0 latch since \
                 the last telemetry beat — drives held stale targets"
            );
            ctx.late_frames = 0;
            ctx.late_max_ns = i64::MIN;
        }
        let nivcsw = crate::thread_prio::thread_nonvoluntary_ctx_switches();
        tracing::info!(
            subsystem = "ethercat",
            event = "cycle_stage_max",
            wake_late_max_ns = ctx.wake_late_max_ns,
            recv_max_ns = ctx.recv_max_ns,
            process_max_ns = ctx.process_max_ns,
            send_max_ns = ctx.send_max_ns,
            pre_cycle_max_ns = ctx.pre_cycle_max_ns,
            post_cycle_max_ns = ctx.post_cycle_max_ns,
            inter_exchange_max_ns = ctx.inter_exchange_max_ns,
            fault_max_ns = ctx.fault_max_ns,
            capture_max_ns = ctx.capture_max_ns,
            wkc_max_ns = ctx.wkc_max_ns,
            heartbeat_max_ns = ctx.heartbeat_max_ns,
            telemetry_max_ns = ctx.telemetry_max_ns,
            nonvoluntary_ctx_switches = nivcsw - ctx.last_nivcsw,
            "worst exchange stage durations since the last telemetry beat"
        );
        ctx.last_nivcsw = nivcsw;
        ctx.wake_late_max_ns = i64::MIN;
        ctx.recv_max_ns = i64::MIN;
        ctx.process_max_ns = i64::MIN;
        ctx.send_max_ns = i64::MIN;
        ctx.pre_cycle_max_ns = i64::MIN;
        ctx.post_cycle_max_ns = i64::MIN;
        ctx.inter_exchange_max_ns = i64::MIN;
        ctx.fault_max_ns = i64::MIN;
        ctx.capture_max_ns = i64::MIN;
        ctx.wkc_max_ns = i64::MIN;
        ctx.heartbeat_max_ns = i64::MIN;
        ctx.telemetry_max_ns = i64::MIN;
        if ctx.gate.state() == TorqueState::Faulted {
            let latched_drive_err = ctx.latched_drive_err;
            respond_fault_heartbeat(ctx, 0, latched_drive_err);
        }
        crate::obs::emit_dropped_line_report();
    }
}

/// Exact discrete transition of the damped-oscillator state x = (d, ḋ) for
///   d̈ = -ω² d - 2ζω ḋ - u
/// over `dt` with a zero-order-held forcing `u`. Returns the homogeneous
/// state-transition matrix Ad (row-major `[a11, a12, a21, a22]`) and the
/// forcing gain Bd = A⁻¹(Ad - I)·[0, -1]ᵀ, so `x' = Ad·x + Bd·u`. `dt <= 0`
/// yields the identity with zero gain (used for a zero predictor lead).
fn osc_zoh(omega: f64, zeta: f64, dt: f64) -> ([f64; 4], [f64; 2]) {
    if dt <= 0.0 {
        return ([1.0, 0.0, 0.0, 1.0], [0.0, 0.0]);
    }
    let s = zeta * omega; // decay rate
    let wd = omega * (1.0 - zeta * zeta).sqrt(); // damped frequency
    let e = libm::exp(-s * dt);
    let (sin, cos) = libm::sincos(wd * dt);
    let a11 = e * (cos + s / wd * sin);
    let a12 = e * (sin / wd);
    let a21 = e * (-(omega * omega) / wd * sin);
    let a22 = e * (cos - s / wd * sin);
    // Bd = A⁻¹(Ad - I)·B with B = [0, -1]ᵀ and A⁻¹ = [[-2s/ω², -1/ω²], [1, 0]].
    // (Ad - I)·B = [-a12, 1 - a22]ᵀ.
    let w2 = omega * omega;
    let c1 = -a12;
    let c2 = 1.0 - a22;
    let b1 = (-2.0 * s / w2) * c1 + (-1.0 / w2) * c2;
    let b2 = c1;
    ([a11, a12, a21, a22], [b1, b2])
}

/// One pinned mode's precomputed oscillator and running state.
struct PinMode {
    /// Mode index in the dynamics model. The capture block for the slot with
    /// this index carries the mode's residual (see `residual_for_slot`).
    mode: usize,
    pin_mass: f32,
    neg_omega2: f32,     // -ω_b²
    two_zeta_omega: f32, // 2ζω_b
    frame_row: Vec<f32>, // length n_slots: projects slots↔mode
    // DC-cycle transition x' = Ad·x + Bd·a_cmd (ZOH forcing).
    a11: f32,
    a12: f32,
    a21: f32,
    a22: f32,
    b1: f32,
    b2: f32,
    // Homogeneous predictor-lead rotation (no forcing over the lead).
    l11: f32,
    l12: f32,
    l21: f32,
    l22: f32,
    dphase: f32, // ω_d·dt phasor advance per cycle
    // Predicted belt deflection (mm) and its rate (mm/s).
    d: f32,
    v: f32,
    // Residual demodulator: predictor phasor angle and the low-passed
    // in-phase/quadrature components of the mode-projected following error.
    theta: f32,
    res_re: f32,
    res_im: f32,
}

/// Per-mode pin-rotor (mode A) oscillator bank. All transition coefficients
/// are precomputed at model install; the hot path only advances the fixed
/// state and writes into the preallocated slot-torque buffer.
#[derive(Default)]
pub(super) struct PinState {
    modes: Vec<PinMode>,
    lp_alpha: f32,         // residual low-pass coefficient dt/τ
    slot_torque: Vec<f32>, // reused each cycle (sized at build)
    prev_streaming: bool,
}

impl PinState {
    /// Residual demodulator low-pass time constant.
    const RES_TAU_S: f64 = 0.25;

    /// Build the bank for `model`, precomputing every pinned mode's exact
    /// transition coefficients from ω_b, ζ, the DC cycle, and the lead.
    pub(super) fn build(model: &DynamicsModel, cycle_ns: i64) -> Self {
        let n_slots = model.n_slots;
        let dt = cycle_ns as f64 * 1e-9;
        // Pin lead is broadcast identically across slots.
        let lead_s = model.pin_lead_us.first().copied().unwrap_or(0.0) * 1e-6;
        let mut modes = Vec::new();
        for k in 0..model.n_modes {
            if !model.pin_active(k) {
                continue;
            }
            // pin_active(k) guarantees compliance(k) > 0.
            let omega = 1.0 / f64::from(model.compliance(k)).sqrt();
            let zeta = f64::from(model.pin_zeta[k]);
            let (ad, bd) = osc_zoh(omega, zeta, dt);
            let (ld, _) = osc_zoh(omega, zeta, lead_s);
            let wd = omega * (1.0 - zeta * zeta).sqrt();
            modes.push(PinMode {
                mode: k,
                pin_mass: model.pin_mass[k],
                neg_omega2: -(omega * omega) as f32,
                two_zeta_omega: (2.0 * zeta * omega) as f32,
                frame_row: model.frame_row(k).to_vec(),
                a11: ad[0] as f32,
                a12: ad[1] as f32,
                a21: ad[2] as f32,
                a22: ad[3] as f32,
                b1: bd[0] as f32,
                b2: bd[1] as f32,
                l11: ld[0] as f32,
                l12: ld[1] as f32,
                l21: ld[2] as f32,
                l22: ld[3] as f32,
                dphase: (wd * dt) as f32,
                d: 0.0,
                v: 0.0,
                theta: 0.0,
                res_re: 0.0,
                res_im: 0.0,
            });
        }
        let lp_alpha = if modes.is_empty() {
            0.0
        } else {
            (dt / Self::RES_TAU_S) as f32
        };
        Self {
            modes,
            lp_alpha,
            slot_torque: vec![0.0; n_slots],
            prev_streaming: false,
        }
    }

    /// True when at least one mode runs in pin-rotor mode.
    pub(super) fn active(&self) -> bool {
        !self.modes.is_empty()
    }

    /// Restart every oscillator clean — used on a model swap, torque
    /// disable, or a discard that re-anchors the commanded frame. A buzz no
    /// longer resets the bank: pinned modes run through it (see `step`).
    pub(super) fn reset(&mut self) {
        for m in &mut self.modes {
            m.d = 0.0;
            m.v = 0.0;
            m.theta = 0.0;
            m.res_re = 0.0;
            m.res_im = 0.0;
        }
        self.prev_streaming = false;
    }

    /// Advance every pinned mode one DC cycle and refill the slot-torque
    /// buffer. `acc_drive`/`ferr_drive` are drive-frame per-slot commanded
    /// acceleration (mm/s²) and following error (mm). During a buzz the
    /// analytic buzz acceleration already rides in `acc_drive` (the endpoint
    /// writes the buzz oscillator's exact `accel_rel` sample into the
    /// commanded-accel regressor for every driven slot). That analytic accel
    /// is `a_buzz = -ω(t)²·x_buzz` plus the chirp-rate/envelope terms, which
    /// are negligible for the ≤10 Hz/s sweeps used here — so the pinned modes
    /// keep integrating with forcing = trajectory accel (zero while buzzing —
    /// no ring piece) + buzz accel, and the pin torque and residual
    /// demodulator stay live. The state re-anchors on the
    /// motion-inactive→active edge (which a buzz onset is, since a buzz only
    /// arms from standstill).
    pub(super) fn step(&mut self, acc_drive: &[f32], ferr_drive: &[f32], streaming: bool) {
        for t in &mut self.slot_torque {
            *t = 0.0;
        }
        if streaming && !self.prev_streaming {
            for m in &mut self.modes {
                m.d = 0.0;
                m.v = 0.0;
                m.theta = 0.0;
            }
        }
        self.prev_streaming = streaming;
        let alpha = self.lp_alpha;
        for m in &mut self.modes {
            let mut a_cmd = 0.0f32;
            for (f, a) in m.frame_row.iter().zip(acc_drive) {
                a_cmd += f * a;
            }
            // Predictor lead: rotate the current state forward, no forcing.
            let d_lead = m.l11 * m.d + m.l12 * m.v;
            let v_lead = m.l21 * m.d + m.l22 * m.v;
            let tau_pin = m.pin_mass * (m.neg_omega2 * d_lead - m.two_zeta_omega * v_lead - a_cmd);
            for (t, f) in self.slot_torque.iter_mut().zip(&m.frame_row) {
                *t += f * tau_pin;
            }
            // Residual: mode-projected following error demodulated against the
            // predictor phasor (cos θ, -sin θ), 1st-order low-pass.
            let mut ferr_mode = 0.0f32;
            for (f, e) in m.frame_row.iter().zip(ferr_drive) {
                ferr_mode += f * e;
            }
            let (sin, cos) = m.theta.sin_cos();
            m.res_re += alpha * (ferr_mode * cos - m.res_re);
            m.res_im += alpha * (ferr_mode * -sin - m.res_im);
            // Advance one DC cycle with ZOH forcing, then the phasor.
            let d_next = m.a11 * m.d + m.a12 * m.v + m.b1 * a_cmd;
            let v_next = m.a21 * m.d + m.a22 * m.v + m.b2 * a_cmd;
            m.d = d_next;
            m.v = v_next;
            m.theta += m.dphase;
            if m.theta >= std::f32::consts::TAU {
                m.theta -= std::f32::consts::TAU;
            }
        }
    }

    /// Pin torque contribution for `slot` (0 when no mode is pinned).
    pub(super) fn slot_torque_at(&self, slot: usize) -> f32 {
        self.slot_torque.get(slot).copied().unwrap_or(0.0)
    }

    /// Residual (in-phase, quadrature) for the pinned mode whose index equals
    /// `slot`; (0, 0) when that mode is not pinned. The capture stream carries
    /// each mode's residual on the slot block of the same index.
    pub(super) fn residual_for_slot(&self, slot: usize) -> (f32, f32) {
        for m in &self.modes {
            if m.mode == slot {
                return (m.res_re, m.res_im);
            }
        }
        (0.0, 0.0)
    }
}
