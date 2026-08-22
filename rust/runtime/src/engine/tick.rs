#[cfg(feature = "motion-module-stepper")]
use core::sync::atomic::Ordering;

use crate::state::SharedState;

use super::Engine;

impl Engine {
    #[cfg(feature = "motion-module-stepper")]
    fn phase_slew_dispatch(&mut self, i: usize, shared: &SharedState) -> bool {
        use crate::stepping_state::StepMode;

        let Some(axis) = self.stepping_axes.get_mut(i).and_then(|s| s.as_mut()) else {
            return false;
        };
        if axis.mode.load(Ordering::Acquire) != StepMode::Phase as u8 {
            return false;
        }
        let pending = axis.steppers.iter().any(|s| {
            s.phase_offset_microsteps.load(Ordering::Acquire)
                != s.phase_offset_target.load(Ordering::Acquire)
        });
        if !pending {
            return false;
        }
        let max_ramp = i32::from(
            shared
                .max_phase_offset_ramp_per_sample
                .load(Ordering::Acquire),
        );
        for stepper in &axis.steppers {
            crate::dispatch_stepper::ramp_phase_offset(stepper, max_ramp);
        }
        crate::dispatch_stepper::write_phase_coils(i, axis, shared, 0);
        true
    }

    #[cfg(not(feature = "motion-module-stepper"))]
    fn phase_slew_dispatch(&mut self, _i: usize, _shared: &SharedState) -> bool {
        false
    }

    #[cfg_attr(not(feature = "sample-stepping"), allow(unused_variables))]
    pub fn tick(&mut self, now: u64, shared: &SharedState) -> bool {
        #[cfg(feature = "sample-stepping")]
        self.sample_take_halt_request(shared);

        let mut active = false;

        for i in 0..(self.num_axes as usize) {
            #[cfg(feature = "sample-stepping")]
            if self.sample_lane_anchored(i) {
                if self.sample_dispatch(i, now, shared) {
                    active = true;
                }
                continue;
            }
            if self.phase_slew_dispatch(i, shared) {
                active = true;
            }
        }

        active
    }
}
