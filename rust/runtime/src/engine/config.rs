use core::sync::atomic::Ordering;

use crate::error::{RUNTIME_ERR_INVALID_ARG, RUNTIME_OK};
use crate::state::SharedState;
use crate::stepping_state::{AxisState, MAX_AXES, StepMode, StepperBindingRust, TMC_CS_OID_NONE};

use super::Engine;

impl Engine {
    pub fn configure_axis(
        &mut self,
        axis_idx: u8,
        mode: StepMode,
        microstep_distance: f32,
        bindings: &[StepperBindingRust],
    ) -> i32 {
        if (axis_idx as usize) >= MAX_AXES {
            return RUNTIME_ERR_INVALID_ARG;
        }
        if !microstep_distance.is_finite() || microstep_distance <= 0.0 {
            return RUNTIME_ERR_INVALID_ARG;
        }

        let idx = axis_idx as usize;
        // SAFETY: `idx < MAX_AXES` is guaranteed by the bounds check above.
        // `stepping_axes` has exactly `MAX_AXES` elements.
        #[allow(clippy::indexing_slicing)]
        let axis = self.stepping_axes[idx].get_or_insert_with(AxisState::new_unconfigured);

        axis.microstep_distance = microstep_distance;
        axis.reset_isr_cache();
        axis.steppers.clear();
        for b in bindings {
            let tmc_cs_oid = if b.tmc_cs_oid == TMC_CS_OID_NONE {
                None
            } else {
                Some(b.tmc_cs_oid)
            };
            let stepper = crate::stepping_state::StepperRef::new(b.stepper_oid, tmc_cs_oid);
            let _ = axis.steppers.push(stepper);
        }
        axis.mode.store(mode as u8, Ordering::Release);

        if idx + 1 > self.num_axes as usize {
            #[allow(clippy::cast_possible_truncation)]
            {
                self.num_axes = (idx + 1) as u8;
            }
        }

        RUNTIME_OK
    }

    pub fn set_axis_step_budget(&mut self, axis_idx: u8, max_steps_per_sample: u32) -> i32 {
        if max_steps_per_sample == 0
            || max_steps_per_sample > crate::sub_sample_timing::MAX_STEPS_PER_SAMPLE as u32
        {
            return RUNTIME_ERR_INVALID_ARG;
        }
        let Some(axis) = self
            .stepping_axes
            .get_mut(axis_idx as usize)
            .and_then(|s| s.as_mut())
        else {
            return RUNTIME_ERR_INVALID_ARG;
        };
        axis.max_steps_per_sample = max_steps_per_sample;
        RUNTIME_OK
    }

    pub fn configure_kinematics(&mut self, k_xy: f32) -> i32 {
        if !k_xy.is_finite() || k_xy <= 0.0 {
            return -1;
        }
        0
    }

    pub fn configure_pressure_advance(&mut self, advance_accel: f32, advance_decel: f32) -> i32 {
        if !advance_accel.is_finite() || !advance_decel.is_finite() {
            return -1;
        }
        if advance_accel < 0.0 || advance_decel < 0.0 {
            return -1;
        }
        0
    }

    pub fn set_axis_mode(&mut self, axis_idx: u8, new_mode_byte: u8) -> i32 {
        if (axis_idx as usize) >= MAX_AXES {
            return -1;
        }
        let new_mode = match new_mode_byte {
            0 => StepMode::Pulse,
            1 => StepMode::Phase,
            _ => return -1,
        };
        #[cfg(feature = "sample-stepping")]
        {
            if self
                .sample_lanes
                .iter()
                .any(crate::sample_exec::SampleLane::has_pending_samples)
            {
                return -2;
            }
            if let Some(lane) = self.sample_lanes.get_mut(axis_idx as usize) {
                lane.reset_for_mode_switch();
            }
        }
        #[cfg(not(any(test, feature = "host")))]
        {
            #[allow(unsafe_code)]
            {
                use crate::step_queue::{StepQueue, step_queues};
                unsafe {
                    let q = step_queues.get().cast::<StepQueue>().add(axis_idx as usize);
                    core::ptr::write_volatile(&mut (*q).head, 0);
                    core::ptr::write_volatile(&mut (*q).tail, 0);
                }
            }
        }
        let Some(axis) = self
            .stepping_axes
            .get_mut(axis_idx as usize)
            .and_then(|s| s.as_mut())
        else {
            return -1;
        };
        match new_mode {
            StepMode::Phase => {
                use core::sync::atomic::Ordering;
                for stepper in &axis.steppers {
                    let offset = stepper.phase_offset_microsteps.load(Ordering::Acquire);
                    let target = axis.last_step_count.wrapping_add(offset);
                    stepper.last_phase_target.store(target, Ordering::Release);
                }
            }
            StepMode::Pulse => {}
        }
        axis.mode.store(new_mode as u8, Ordering::Release);
        0
    }

    pub fn set_stepper_offset(
        &mut self,
        shared: &SharedState,
        stepper_idx: u8,
        delta_microsteps: i32,
        max_microsteps_per_sample: u16,
    ) -> i32 {
        use core::sync::atomic::Ordering;
        if delta_microsteps == 0 {
            return 0;
        }
        if max_microsteps_per_sample == 0 || max_microsteps_per_sample > 256 {
            crate::fault_helpers::raise_jog_parameters_invalid(shared);
            return -1;
        }
        let mut remaining = stepper_idx as usize;
        for axis_opt in &mut self.stepping_axes {
            let Some(axis) = axis_opt.as_mut() else {
                continue;
            };
            if remaining < axis.steppers.len() {
                #[allow(clippy::indexing_slicing)]
                let stepper = &axis.steppers[remaining];
                let new_target = stepper
                    .phase_offset_target
                    .load(Ordering::Acquire)
                    .wrapping_add(delta_microsteps);
                stepper
                    .phase_offset_target
                    .store(new_target, Ordering::Release);
                shared
                    .max_phase_offset_ramp_per_sample
                    .store(max_microsteps_per_sample, Ordering::Release);
                return 0;
            }
            remaining -= axis.steppers.len();
        }
        crate::fault_helpers::raise_jog_parameters_invalid(shared);
        -1
    }
}
