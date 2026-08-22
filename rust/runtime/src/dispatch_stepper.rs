#![allow(unsafe_code)]

use core::sync::atomic::Ordering;

use crate::fault_helpers::raise_position_count_overflow;
use crate::phase_lut::{PHASE_LUT, PHASE_LUT_SIZE};
use crate::state::SharedState;
use crate::stepping_state::AxisConfig;

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

#[cfg(not(any(test, feature = "host")))]
unsafe extern "C" {
    fn kalico_kick_step_output(axis_idx: u8, cycle_abs: u32);
}

#[inline]
#[cfg(not(any(test, feature = "host")))]
pub(crate) fn kick_per_axis_timer_foreground(axis_idx: usize, cycle_abs: u32) {
    // SAFETY: writes only a timer compare register and an owned-mask bit,
    // guarded by the same runtime IRQ save/restore used by the ISR path.
    unsafe {
        let flags = crate::state::runtime_irq_save();
        kalico_kick_step_output(axis_idx as u8, cycle_abs);
        crate::state::runtime_irq_restore(flags);
    }
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

pub fn write_phase_coils(
    axis_idx: usize,
    axis: &AxisConfig,
    shared: &SharedState,
    buzz_offset: i32,
) {
    let base = axis.last_step_count;

    for stepper in &axis.steppers {
        let phase_offset = stepper.phase_offset_microsteps.load(Ordering::Acquire);
        let target_stepper = base.wrapping_add(phase_offset).wrapping_add(buzz_offset);
        let prev_stepper = stepper.last_phase_target.load(Ordering::Acquire);
        let delta_stepper = target_stepper.wrapping_sub(prev_stepper);
        stepper
            .last_phase_target
            .store(target_stepper, Ordering::Release);

        #[allow(clippy::cast_sign_loss)]
        let phase = (target_stepper as u32) & 0x3FF;
        #[allow(clippy::indexing_slicing)] // infallible: phase < PHASE_LUT_SIZE by construction
        let (coil_a, coil_b) = PHASE_LUT[phase as usize];

        stepper.last_coil_A.store(coil_a, Ordering::Release);
        stepper.last_coil_B.store(coil_b, Ordering::Release);

        if stepper.tmc_cs_oid.is_some() {
            let phase_motor_count = shared.phase_motor_count.load(Ordering::Acquire) as usize;
            let mut found_motor_idx: Option<u8> = None;
            {
                let mut j: usize = 0;
                for earlier in &axis.steppers {
                    if core::ptr::eq(earlier as *const _, stepper as *const _) {
                        break;
                    }
                    if earlier.tmc_cs_oid.is_some() {
                        j += 1;
                    }
                }
                let mut match_count: usize = 0;
                for m in 0..phase_motor_count.min(crate::state::MAX_STEPPER_OIDS) {
                    // SAFETY: `m < phase_motor_count.min(MAX_STEPPER_OIDS)`, so
                    // `m < MAX_STEPPER_OIDS == phase_slot_idx.len()`.
                    #[allow(clippy::indexing_slicing)]
                    let slot = shared.phase_slot_idx[m].load(Ordering::Acquire);
                    if slot as usize == axis_idx {
                        if match_count == j {
                            #[allow(clippy::cast_possible_truncation)]
                            {
                                found_motor_idx = Some(m as u8);
                            }
                            break;
                        }
                        match_count += 1;
                    }
                }
            }

            let Some(motor_idx) = found_motor_idx else {
                crate::fault_helpers::raise_phase_motor_unmapped(
                    shared,
                    axis_idx,
                    stepper.stepper_oid,
                );
                return;
            };

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
