use core::sync::atomic::Ordering;

use crate::piece_ring::PieceEntry;
use crate::state::SharedState;
use crate::stepping_state::{AxisState, StepMode};

use super::{Engine, SharedFaultSink};

fn idle_phase_slew_pending(axis: &AxisState) -> bool {
    if axis.mode.load(Ordering::Acquire) != StepMode::Phase as u8 {
        return false;
    }
    axis.steppers.iter().any(|s| {
        s.phase_offset_microsteps.load(Ordering::Acquire)
            != s.phase_offset_target.load(Ordering::Acquire)
    })
}

impl Engine {
    pub fn tick(&mut self, now: u64, shared: &SharedState, storage: &mut [PieceEntry]) -> bool {
        let refill_fault = crate::buzz_stream::take_refill_fault();
        if refill_fault != 0 {
            shared.last_error.store(refill_fault, Ordering::Release);
        }

        #[cfg(feature = "motion-module-stepper")]
        use crate::dispatch_stepper::dispatch_axis;

        #[cfg(feature = "motion-module-stepper")]
        #[cfg(any(test, feature = "host"))]
        let get_queue = |i: usize| {
            self.test_queue_ptrs
                .get(i)
                .copied()
                .unwrap_or(core::ptr::null_mut())
        };
        #[cfg(feature = "motion-module-stepper")]
        #[cfg(not(any(test, feature = "host")))]
        let get_queue = |i: usize| crate::step_queue::queue_for_axis(i);

        #[cfg(feature = "motion-module-stepper")]
        let sample_period_sec = if self.sample_period_cycles == 0 || self.cycles_per_second == 0.0 {
            0.0_f32
        } else {
            self.sample_period_cycles as f32 / self.cycles_per_second
        };

        #[cfg(feature = "motion-module-stepper")]
        #[allow(clippy::cast_possible_truncation)]
        let now_lo = now as u32;

        let mut active = false;

        for i in 0..(self.num_axes as usize) {
            let (p_end, v_end, p_sample_start, overlay_just_armed) = {
                let Some(axis) = self.stepping_axes.get_mut(i).and_then(|s| s.as_mut()) else {
                    continue;
                };
                let cps = self.cycles_per_second;
                let fault = SharedFaultSink { shared };
                let mut just_armed = false;
                match crate::motion_core::get_position_and_velocity_armed(
                    &mut axis.armed,
                    &mut axis.ring,
                    storage,
                    now,
                    self.sample_period_cycles,
                    cps,
                    i,
                    &fault,
                    &mut just_armed,
                ) {
                    Some((p_end, v_end)) => {
                        active = true;
                        let is_overlay = axis.ring.peek(storage).map_or(0, |p| p.motor_mask) != 0;
                        if is_overlay {
                            if just_armed {
                                axis.overlay_last_p = 0.0;
                            }
                            let p_sample_start = axis.overlay_last_p;
                            axis.overlay_last_p = p_end;
                            (p_end, v_end, p_sample_start, just_armed)
                        } else {
                            let p_sample_start = axis.p_prev;
                            axis.p_prev = p_end;
                            axis.v_prev = v_end;
                            (p_end, v_end, p_sample_start, false)
                        }
                    }
                    None => {
                        if !idle_phase_slew_pending(axis) {
                            continue;
                        }
                        active = true;
                        (axis.p_prev, 0.0, axis.p_prev, false)
                    }
                }
            };

            #[cfg(feature = "motion-module-stepper")]
            {
                let Some(axis) = self.stepping_axes.get_mut(i).and_then(|s| s.as_mut()) else {
                    continue;
                };
                let active_mask = axis.ring.peek(storage).map_or(0, |p| p.motor_mask);
                let queue_ptr = get_queue(i);
                dispatch_axis(
                    i,
                    axis,
                    active_mask,
                    queue_ptr,
                    shared,
                    p_end,
                    v_end,
                    p_sample_start,
                    sample_period_sec,
                    now_lo,
                    self.cycles_per_second,
                    overlay_just_armed,
                );
            }

            #[cfg(not(feature = "motion-module-stepper"))]
            {
                let _ = (p_end, v_end, p_sample_start, overlay_just_armed);
            }
        }

        active
    }
}
