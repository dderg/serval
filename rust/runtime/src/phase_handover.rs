use core::sync::atomic::Ordering;

use crate::fault_helpers;
use crate::phase_lut::coil_for_phase;
use crate::phase_spi::{
    self, CHOPCONF_ADDR, GCONF_ADDR, GCONF_DIRECT_MODE, GCONF_EN_PWM, MSCNT_ADDR,
};
use crate::state::SharedState;
use crate::stepping_state::{AxisState, StepMode, StepperRef};

pub const PHASE_PERIOD: i32 = 1024;
pub const PHASE_MASK: i32 = PHASE_PERIOD - 1;
const PHASE_HALF_PERIOD: i32 = PHASE_PERIOD >> 1;

/// TMC5160 XDIRECT register address.
pub const XDIRECT_ADDR: u8 = 0x2D;
/// CHOPCONF `toff` occupies bits [3:0]; zero means the chopper is off and the
/// charge-pump bootstrap would drain under `direct_mode`.
const CHOPCONF_TOFF_MASK: u32 = 0x0F;

/// `set_axis_mode_group` selectors. `2` begins the phase→pulse rotor walk;
/// the host then polls `get_phase_state` and sends `0` to finalize.
pub const MODE_PULSE_FINALIZE: u8 = 0;
pub const MODE_PHASE_ENTER: u8 = 1;
pub const MODE_BEGIN_EXIT_WALK: u8 = 2;

/// XDIRECT register value matching the ISR's on-wire datagram: `coil_B` in
/// bits [24:16], `coil_A` in bits [8:0] (both signed 9-bit).
#[allow(clippy::cast_sign_loss)]
fn pack_xdirect(coil_a: i16, coil_b: i16) -> u32 {
    ((u32::from(coil_b as u16) & 0x1FF) << 16) | (u32::from(coil_a as u16) & 0x1FF)
}

pub struct PhaseQuery {
    pub axis_idx: u8,
    pub mode: u8,
    pub phase: u16,
    pub settled: bool,
}

pub fn shortest_phase_delta(current_phase: u16, target_phase: u16) -> i32 {
    let raw = (i32::from(target_phase) - i32::from(current_phase)).rem_euclid(PHASE_PERIOD);
    if raw > PHASE_HALF_PERIOD {
        raw - PHASE_PERIOD
    } else {
        raw
    }
}

pub fn find_stepper(
    axes: &[Option<AxisState>],
    stepper_oid: u8,
) -> Option<(usize, &AxisState, &StepperRef)> {
    for (axis_idx, axis_opt) in axes.iter().enumerate() {
        let Some(axis) = axis_opt else { continue };
        for stepper in &axis.steppers {
            if stepper.stepper_oid == stepper_oid {
                return Some((axis_idx, axis, stepper));
            }
        }
    }
    None
}

/// Resolve the C-side `phase_motors[]` slot for `stepper` on `axis_idx`.
///
/// `stepper` is the j-th phase-stepped (`tmc_cs_oid.is_some()`) stepper on its
/// axis; the slot is the j-th `phase_slot_idx` entry whose value is `axis_idx`.
/// Single source of truth shared by the TIM5 dispatch path and the phase-mode
/// handover so a preload write never targets a different physical driver than
/// the one the ISR drives. Returns `None` if no slot maps (caller fails loud).
#[allow(clippy::cast_possible_truncation)]
pub fn motor_idx_for(
    shared: &SharedState,
    axis_idx: usize,
    axis: &AxisState,
    stepper: &StepperRef,
) -> Option<u8> {
    let mut j: usize = 0;
    for earlier in &axis.steppers {
        if core::ptr::eq(earlier as *const _, stepper as *const _) {
            break;
        }
        if earlier.tmc_cs_oid.is_some() {
            j += 1;
        }
    }
    let phase_motor_count = shared.phase_motor_count.load(Ordering::Acquire) as usize;
    let mut match_count: usize = 0;
    for m in 0..phase_motor_count.min(crate::state::MAX_STEPPER_OIDS) {
        // SAFETY: `m < phase_motor_count.min(MAX_STEPPER_OIDS)`, so
        // `m < MAX_STEPPER_OIDS == phase_slot_idx.len()`.
        #[allow(clippy::indexing_slicing)]
        let slot = shared.phase_slot_idx[m].load(Ordering::Acquire);
        if slot as usize == axis_idx {
            if match_count == j {
                return Some(m as u8);
            }
            match_count += 1;
        }
    }
    None
}

#[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
fn phase_of(last_step_count: i32, offset: i32) -> u16 {
    (last_step_count.wrapping_add(offset) & PHASE_MASK) as u16
}

pub fn jog_to(
    axes: &[Option<AxisState>],
    shared: &SharedState,
    stepper_oid: u8,
    target_phase: u16,
    max_microsteps_per_sample: u16,
) -> i32 {
    if i32::from(target_phase) >= PHASE_PERIOD {
        crate::fault_helpers::raise_jog_parameters_invalid(shared);
        return -1;
    }
    if max_microsteps_per_sample == 0 || max_microsteps_per_sample > 256 {
        crate::fault_helpers::raise_jog_parameters_invalid(shared);
        return -1;
    }
    let Some((_, axis, stepper)) = find_stepper(axes, stepper_oid) else {
        crate::fault_helpers::raise_jog_parameters_invalid(shared);
        return -1;
    };
    if axis.mode.load(Ordering::Acquire) != StepMode::Phase as u8 {
        return -3;
    }
    let pending_target = stepper.phase_offset_target.load(Ordering::Acquire);
    let pending_phase = phase_of(axis.last_step_count, pending_target);
    let delta = shortest_phase_delta(pending_phase, target_phase);
    stepper
        .phase_offset_target
        .store(pending_target.wrapping_add(delta), Ordering::Release);
    shared
        .max_phase_offset_ramp_per_sample
        .store(max_microsteps_per_sample, Ordering::Release);
    0
}

pub fn align_to(axes: &[Option<AxisState>], stepper_oid: u8, target_phase: u16) -> i32 {
    if i32::from(target_phase) >= PHASE_PERIOD {
        return -1;
    }
    let motion_active = axes
        .iter()
        .any(|a| a.as_ref().map_or(false, |ax| ax.armed.is_some()));
    if motion_active {
        return -2;
    }
    let Some((_, axis, stepper)) = find_stepper(axes, stepper_oid) else {
        return -1;
    };
    let current = stepper.phase_offset_microsteps.load(Ordering::Acquire);
    let current_phase = phase_of(axis.last_step_count, current);
    let new_offset = current.wrapping_add(shortest_phase_delta(current_phase, target_phase));
    stepper
        .phase_offset_microsteps
        .store(new_offset, Ordering::Release);
    stepper
        .phase_offset_target
        .store(new_offset, Ordering::Release);
    0
}

pub fn query(axes: &[Option<AxisState>], stepper_oid: u8) -> Option<PhaseQuery> {
    let (axis_idx, axis, stepper) = find_stepper(axes, stepper_oid)?;
    let current = stepper.phase_offset_microsteps.load(Ordering::Acquire);
    let target = stepper.phase_offset_target.load(Ordering::Acquire);
    #[allow(clippy::cast_possible_truncation)]
    Some(PhaseQuery {
        axis_idx: axis_idx as u8,
        mode: axis.mode.load(Ordering::Acquire),
        phase: phase_of(axis.last_step_count, current),
        settled: current == target,
    })
}

use crate::error::FaultCode;
use crate::stepping_state::MAX_AXES;

fn group_axis_mask(axes: &[Option<AxisState>], oids: &[u8]) -> u8 {
    let mut mask = 0u8;
    for &oid in oids {
        if let Some((ax_i, _, _)) = find_stepper(axes, oid) {
            if ax_i < MAX_AXES {
                mask |= 1u8 << ax_i;
            }
        }
    }
    mask
}

fn set_group_handover(axes: &[Option<AxisState>], mask: u8, value: bool) {
    for ax_i in 0..MAX_AXES {
        if mask & (1u8 << ax_i) == 0 {
            continue;
        }
        if let Some(axis) = axes.get(ax_i).and_then(|a| a.as_ref()) {
            axis.handover_in_progress.store(value, Ordering::Release);
        }
    }
}

fn reset_group_step_queues(mask: u8) {
    for ax_i in 0..MAX_AXES {
        if mask & (1u8 << ax_i) != 0 {
            reset_axis_step_queue(ax_i);
        }
    }
}

fn store_group_mode(axes: &[Option<AxisState>], mask: u8, mode: StepMode) {
    for ax_i in 0..MAX_AXES {
        if mask & (1u8 << ax_i) == 0 {
            continue;
        }
        if let Some(axis) = axes.get(ax_i).and_then(|a| a.as_ref()) {
            axis.mode.store(mode as u8, Ordering::Release);
        }
    }
}

fn reset_axis_step_queue(axis_idx: usize) {
    #[cfg(not(any(test, feature = "host")))]
    {
        #![allow(unsafe_code)]
        use crate::step_queue::{StepQueue, step_queues};
        // SAFETY: `axis_idx < MAX_AXES` (caller iterates the group mask, whose
        // bits index real configured axes); `step_queues` is the C-owned
        // per-axis ring array sized to MAX_AXES.
        unsafe {
            let q = step_queues.get().cast::<StepQueue>().add(axis_idx);
            core::ptr::write_volatile(&mut (*q).head, 0);
            core::ptr::write_volatile(&mut (*q).tail, 0);
        }
    }
    #[cfg(any(test, feature = "host"))]
    let _ = axis_idx;
}

fn motion_active(axes: &[Option<AxisState>]) -> bool {
    axes.iter()
        .any(|a| a.as_ref().map_or(false, |ax| ax.armed.is_some()))
}

/// Enter phase stepping for every stepper named in `oids` (one handover group)
/// entirely on the MCU: validate, suppress ISR writes, then per stepper assert
/// `toff>0`, set `direct_mode`, read the live MSCNT, preload XDIRECT to that
/// electrical angle, and align the step-generator offset — then reset the
/// group's step queues, re-enable writes, and flip every group axis to Phase.
///
/// Synchronous and transactional: the whole group is validated before any SPI
/// write, so a precondition failure never leaves a half-entered group. Returns
/// `0` on success or a negative [`FaultCode`].
pub fn enter_group(
    axes: &[Option<AxisState>],
    shared: &SharedState,
    axis_idx: u8,
    oids: &[u8],
) -> i32 {
    if motion_active(axes) {
        return FaultCode::MotionInProgress.as_i32();
    }
    for &oid in oids {
        let Some((ax_i, axis, stepper)) = find_stepper(axes, oid) else {
            fault_helpers::raise_phase_enter_precondition_failed(shared, axis_idx as usize, oid);
            return FaultCode::PhaseEnterPreconditionFailed.as_i32();
        };
        if stepper.tmc_cs_oid.is_none() {
            fault_helpers::raise_phase_enter_precondition_failed(shared, ax_i, oid);
            return FaultCode::PhaseEnterPreconditionFailed.as_i32();
        }
        if motor_idx_for(shared, ax_i, axis, stepper).is_none() {
            fault_helpers::raise_phase_motor_unmapped(shared, ax_i, oid);
            return FaultCode::PhaseMotorUnmapped.as_i32();
        }
    }

    let mask = group_axis_mask(axes, oids);
    set_group_handover(axes, mask, true);
    phase_spi::disable_writes();

    let rc = enter_group_spi(axes, shared, oids);
    if rc != 0 {
        phase_spi::enable_writes();
        set_group_handover(axes, mask, false);
        return rc;
    }

    reset_group_step_queues(mask);
    phase_spi::enable_writes();
    store_group_mode(axes, mask, StepMode::Phase);
    set_group_handover(axes, mask, false);
    0
}

// Restore the pre-enter GCONF (clearing `direct_mode`) on the first `upto`
// steppers of the group — best-effort rollback so a mid-group write failure
// never leaves a chip latched in direct_mode while the axis stays Pulse.
fn rollback_enter(axes: &[Option<AxisState>], shared: &SharedState, oids: &[u8], upto: usize) {
    for &oid in oids.iter().take(upto) {
        if let Some((ax_i, axis, stepper)) = find_stepper(axes, oid) {
            if let Some(motor_idx) = motor_idx_for(shared, ax_i, axis, stepper) {
                let restore = stepper.phase_enter_gconf.load(Ordering::Acquire);
                let _ = phase_spi::write_register(motor_idx, GCONF_ADDR, restore);
            }
        }
    }
}

fn enter_group_spi(axes: &[Option<AxisState>], shared: &SharedState, oids: &[u8]) -> i32 {
    // Pass 1 — reads only: validate `toff>0` and cache GCONF + MSCNT for every
    // stepper. No chip state changes here, so a failure needs no rollback.
    for &oid in oids {
        let Some((ax_i, axis, stepper)) = find_stepper(axes, oid) else {
            return FaultCode::PhaseEnterPreconditionFailed.as_i32();
        };
        let Some(motor_idx) = motor_idx_for(shared, ax_i, axis, stepper) else {
            return FaultCode::PhaseMotorUnmapped.as_i32();
        };

        let Some(chopconf) = phase_spi::read_register(motor_idx, CHOPCONF_ADDR) else {
            fault_helpers::raise_mscnt_read_timeout(shared, motor_idx);
            return FaultCode::MscntReadTimeout.as_i32();
        };
        if chopconf & CHOPCONF_TOFF_MASK == 0 {
            fault_helpers::raise_phase_enter_precondition_failed(shared, ax_i, oid);
            return FaultCode::PhaseEnterPreconditionFailed.as_i32();
        }

        let Some(gconf0) = phase_spi::read_register(motor_idx, GCONF_ADDR) else {
            fault_helpers::raise_mscnt_read_timeout(shared, motor_idx);
            return FaultCode::MscntReadTimeout.as_i32();
        };
        stepper.phase_enter_gconf.store(gconf0, Ordering::Release);

        let Some(raw_mscnt) = phase_spi::read_register(motor_idx, MSCNT_ADDR) else {
            fault_helpers::raise_mscnt_read_timeout(shared, motor_idx);
            return FaultCode::MscntReadTimeout.as_i32();
        };
        #[allow(clippy::cast_possible_truncation)]
        let mscnt = (raw_mscnt & 0x3FF) as u16;
        stepper
            .phase_enter_mscnt
            .store(i32::from(mscnt), Ordering::Release);
    }

    // Pass 2 — commit writes: set `direct_mode`, preload XDIRECT, align. On any
    // failure at stepper `k`, roll back steppers `0..=k` before returning.
    for (k, &oid) in oids.iter().enumerate() {
        let Some((ax_i, axis, stepper)) = find_stepper(axes, oid) else {
            return FaultCode::PhaseEnterPreconditionFailed.as_i32();
        };
        let Some(motor_idx) = motor_idx_for(shared, ax_i, axis, stepper) else {
            return FaultCode::PhaseMotorUnmapped.as_i32();
        };
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let mscnt = (stepper.phase_enter_mscnt.load(Ordering::Acquire) & PHASE_MASK) as u16;

        let Some(verified) = phase_spi::rmw_register(
            motor_idx,
            GCONF_ADDR,
            GCONF_DIRECT_MODE | GCONF_EN_PWM,
            GCONF_DIRECT_MODE,
        ) else {
            rollback_enter(axes, shared, oids, k);
            fault_helpers::raise_mscnt_read_timeout(shared, motor_idx);
            return FaultCode::MscntReadTimeout.as_i32();
        };
        if verified & GCONF_DIRECT_MODE == 0 {
            rollback_enter(axes, shared, oids, k + 1);
            fault_helpers::raise_gconf_verify_failed(shared, motor_idx, verified);
            return FaultCode::GconfVerifyFailed.as_i32();
        }

        let (coil_a, coil_b) = coil_for_phase(mscnt);
        if !phase_spi::write_register(motor_idx, XDIRECT_ADDR, pack_xdirect(coil_a, coil_b)) {
            rollback_enter(axes, shared, oids, k + 1);
            fault_helpers::raise_mscnt_read_timeout(shared, motor_idx);
            return FaultCode::MscntReadTimeout.as_i32();
        }

        align_to(axes, oid, mscnt);
        let offset = stepper.phase_offset_microsteps.load(Ordering::Acquire);
        stepper
            .last_phase_target
            .store(axis.last_step_count.wrapping_add(offset), Ordering::Release);
    }
    0
}

/// Begin the phase→pulse rotor walk: jog every group stepper back toward its
/// cached enter-MSCNT (the chip's frozen microstep angle) at one microstep per
/// sample. The axes stay in Phase mode so the ISR keeps driving XDIRECT through
/// the walk; the host polls `get_phase_state` and then calls
/// [`exit_finalize_group`]. Returns `0` or a negative [`FaultCode`].
pub fn exit_begin_group(axes: &[Option<AxisState>], shared: &SharedState, oids: &[u8]) -> i32 {
    for &oid in oids {
        let Some((ax_i, axis, stepper)) = find_stepper(axes, oid) else {
            fault_helpers::raise_phase_exit_desync(shared, 0, 0);
            return FaultCode::PhaseExitDesync.as_i32();
        };
        if axis.mode.load(Ordering::Acquire) != StepMode::Phase as u8 {
            fault_helpers::raise_phase_exit_desync(shared, ax_i, 0);
            return FaultCode::PhaseExitDesync.as_i32();
        }
        let cached = stepper.phase_enter_mscnt.load(Ordering::Acquire);
        if cached < 0 {
            fault_helpers::raise_phase_exit_desync(shared, ax_i, 0);
            return FaultCode::PhaseExitDesync.as_i32();
        }
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let target = (cached & PHASE_MASK) as u16;
        let rc = jog_to(axes, shared, oid, target, 1);
        if rc != 0 {
            return rc;
        }
    }
    set_group_handover(axes, group_axis_mask(axes, oids), true);
    0
}

/// Finalize the phase→pulse handover after the walk settled: verify each
/// stepper sits on its cached enter-MSCNT, clear `direct_mode` (restoring the
/// pre-enter GCONF so `en_pwm_mode` reconverges with the host cache), reset the
/// step queues, and flip every group axis to Pulse. Returns `0` or a negative
/// [`FaultCode`]; fails loud on any desync rather than clearing `direct_mode`
/// mid-walk (which would lurch the rotor).
pub fn exit_finalize_group(axes: &[Option<AxisState>], shared: &SharedState, oids: &[u8]) -> i32 {
    for &oid in oids {
        let (Some(q), Some((_, _, stepper))) = (query(axes, oid), find_stepper(axes, oid)) else {
            fault_helpers::raise_phase_exit_desync(shared, 0, 0);
            return FaultCode::PhaseExitDesync.as_i32();
        };
        let cached = stepper.phase_enter_mscnt.load(Ordering::Acquire);
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let target = (cached & PHASE_MASK) as u16;
        if cached < 0 || !q.settled || q.phase != target {
            fault_helpers::raise_phase_exit_desync(shared, q.axis_idx as usize, q.phase);
            return FaultCode::PhaseExitDesync.as_i32();
        }
    }

    let mask = group_axis_mask(axes, oids);
    phase_spi::disable_writes();

    let rc = exit_finalize_spi(axes, shared, oids);
    phase_spi::enable_writes();
    if rc != 0 {
        return rc;
    }

    reset_group_step_queues(mask);
    store_group_mode(axes, mask, StepMode::Pulse);
    set_group_handover(axes, mask, false);
    0
}

fn exit_finalize_spi(axes: &[Option<AxisState>], shared: &SharedState, oids: &[u8]) -> i32 {
    for &oid in oids {
        let Some((ax_i, axis, stepper)) = find_stepper(axes, oid) else {
            return FaultCode::PhaseExitDesync.as_i32();
        };
        let Some(motor_idx) = motor_idx_for(shared, ax_i, axis, stepper) else {
            return FaultCode::PhaseMotorUnmapped.as_i32();
        };

        let cached = stepper.phase_enter_mscnt.load(Ordering::Acquire);
        let Some(raw_mscnt) = phase_spi::read_register(motor_idx, MSCNT_ADDR) else {
            fault_helpers::raise_mscnt_read_timeout(shared, motor_idx);
            return FaultCode::MscntReadTimeout.as_i32();
        };
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        if i32::from((raw_mscnt & 0x3FF) as u16) != (cached & PHASE_MASK) {
            #[allow(clippy::cast_possible_truncation)]
            fault_helpers::raise_phase_exit_desync(shared, ax_i, (raw_mscnt & 0x3FF) as u16);
            return FaultCode::PhaseExitDesync.as_i32();
        }

        let restore = stepper.phase_enter_gconf.load(Ordering::Acquire);
        if !phase_spi::write_register(motor_idx, GCONF_ADDR, restore) {
            fault_helpers::raise_mscnt_read_timeout(shared, motor_idx);
            return FaultCode::MscntReadTimeout.as_i32();
        }
        let Some(verified) = phase_spi::read_register(motor_idx, GCONF_ADDR) else {
            fault_helpers::raise_mscnt_read_timeout(shared, motor_idx);
            return FaultCode::MscntReadTimeout.as_i32();
        };
        if verified & GCONF_DIRECT_MODE != 0 {
            fault_helpers::raise_gconf_verify_failed(shared, motor_idx, verified);
            return FaultCode::GconfVerifyFailed.as_i32();
        }
    }
    0
}

#[cfg(test)]
mod tests;
