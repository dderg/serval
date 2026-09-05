#![allow(unsafe_code)]

use core::sync::atomic::Ordering;

use crate::fault_helpers::raise_position_count_overflow;
use crate::phase_lut::{PHASE_LUT, PHASE_LUT_SIZE};
use crate::state::SharedState;
use crate::stepping_state::AxisState;

const _: () = assert!(
    0x3FF < PHASE_LUT_SIZE,
    "PHASE_LUT_SIZE must be > 0x3FF (1023) for the phase-mask indexing in write_phase_coils to be infallible",
);

// FFI declaration for the C-side SPI write function.
//
// Gate: fire the real C SPI write on bare-metal MCU firmware
// (`not(any(test, feature = "host"))`) AND on Linux-MCU firmware
// (`feature = "mcu-linux"`).  The `mcu-linux` feature implies `host`, so a
// plain `not(any(test, feature = "host"))` would NOT catch a Linux firmware
// build — `mcu-linux` is the explicit discriminator.
#[cfg(any(not(any(test, feature = "host")), feature = "mcu-linux"))]
unsafe extern "C" {
    fn phase_stepping_write_xdirect(motor_idx: u8, coil_a: i16, coil_b: i16);
}

pub const DISPLACEMENT_THRESHOLD_MM: f32 = 1e-4;

#[cfg(feature = "motion-module-stepper")]
pub(crate) fn ramp_phase_offset(stepper: &crate::stepping_state::StepperRef, max_per_sample: i32) {
    if max_per_sample == 0 {
        return;
    }
    let current = stepper.phase_offset_microsteps.load(Ordering::Acquire);
    let target = stepper.phase_offset_target.load(Ordering::Acquire);
    if current == target {
        return;
    }
    let delta = target.wrapping_sub(current);
    let step = if delta.abs() <= max_per_sample {
        delta
    } else if delta > 0 {
        max_per_sample
    } else {
        -max_per_sample
    };
    stepper
        .phase_offset_microsteps
        .store(current.wrapping_add(step), Ordering::Release);
}

/// Slot index of the `tmc_rank`-th TMC stepper mapped to `axis_idx`.
#[allow(clippy::cast_possible_truncation)] // slot index < MAX_STEPPER_OIDS
fn phase_motor_slot(shared: &SharedState, axis_idx: usize, tmc_rank: usize) -> Option<u8> {
    let mapped = (shared.phase_motor_count.load(Ordering::Acquire) as usize)
        .min(crate::state::MAX_STEPPER_OIDS);
    shared
        .phase_slot_idx
        .iter()
        .take(mapped)
        .enumerate()
        .filter(|(_, slot)| slot.load(Ordering::Acquire) as usize == axis_idx)
        .nth(tmc_rank)
        .map(|(motor_idx, _)| motor_idx as u8)
}

pub fn write_phase_coils(
    axis_idx: usize,
    axis: &AxisState,
    shared: &SharedState,
    buzz_offset: i32,
) {
    let base = axis.last_step_count;
    let mut tmc_rank = 0usize;

    for stepper in &axis.steppers {
        let phase_offset = stepper.phase_offset_microsteps.load(Ordering::Acquire);
        let target_stepper = base.wrapping_add(phase_offset).wrapping_add(buzz_offset);
        let prev_stepper = stepper.last_phase_target.load(Ordering::Acquire);
        let delta_stepper = target_stepper.wrapping_sub(prev_stepper);
        stepper
            .last_phase_target
            .store(target_stepper, Ordering::Release);

        if stepper.tmc_cs_oid.is_some() {
            #[allow(clippy::cast_sign_loss)]
            let phase = (target_stepper as u32) & 0x3FF;
            #[allow(clippy::indexing_slicing)] // infallible: phase < PHASE_LUT_SIZE by construction
            let (coil_a, coil_b) = PHASE_LUT[phase as usize];

            let Some(motor_idx) = phase_motor_slot(shared, axis_idx, tmc_rank) else {
                crate::fault_helpers::raise_phase_motor_unmapped(
                    shared,
                    axis_idx,
                    stepper.stepper_oid,
                );
                return;
            };
            tmc_rank += 1;

            #[cfg(all(any(test, feature = "host"), not(feature = "mcu-linux")))]
            crate::test_xdirect_capture::record(motor_idx, coil_a, coil_b);

            // SAFETY: `phase_stepping_write_xdirect` accepts any
            // (motor_idx, coil_a, coil_b) triple; motor_idx is a found slot.
            // coil_a/coil_b are PHASE_LUT values, always within i16 range.
            #[cfg(any(not(any(test, feature = "host")), feature = "mcu-linux"))]
            unsafe {
                phase_stepping_write_xdirect(motor_idx, coil_a, coil_b);
            }
        }

        let prev = stepper.position_count.load(Ordering::Acquire);
        let Some(next) = prev.checked_add(delta_stepper) else {
            raise_position_count_overflow(shared, axis_idx);
            return;
        };
        stepper.position_count.store(next, Ordering::Release);
    }
}

#[cfg(test)]
mod tests;
