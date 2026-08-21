use crate::state::SharedState;

use super::Engine;

impl Engine {
    pub fn phase_jog_to(
        &self,
        shared: &SharedState,
        stepper_oid: u8,
        target_phase: u16,
        max_microsteps_per_sample: u16,
    ) -> i32 {
        crate::phase_handover::jog_to(
            &self.stepping_axes,
            shared,
            stepper_oid,
            target_phase,
            max_microsteps_per_sample,
        )
    }

    /// A lane frozen on a trip hold, or an anchored lane whose rings have
    /// drained to an idle zero-order hold, plays no queued sample and holds
    /// the axis still; aligning the phase offset under it is exactly the
    /// handover's job. Only undrained queued runs mean the walk would race
    /// live motion. Refusing on the idle hold shut the mcu down on every
    /// phase-mode entry.
    pub fn phase_align_to(&self, stepper_oid: u8, target_phase: u16) -> i32 {
        #[cfg(feature = "sample-stepping")]
        if self
            .sample_lanes
            .iter()
            .any(crate::sample_exec::SampleLane::has_pending_samples)
        {
            return -2;
        }
        crate::phase_handover::align_to(&self.stepping_axes, stepper_oid, target_phase)
    }

    pub fn phase_state(&self, stepper_oid: u8) -> Option<crate::phase_handover::PhaseQuery> {
        crate::phase_handover::query(&self.stepping_axes, stepper_oid)
    }

    /// Adopt the classic executor's step count as this axis's position at a
    /// mode switch: both executors drive the same motor, so the incoming
    /// mode must start from the count the outgoing one physically reached.
    /// A stale count would shift the phase readout the moment the host's
    /// transport seed arrived, dragging the coils away from the aligned
    /// preload.
    pub fn seed_axis_count(&mut self, axis_idx: u8, count: i32) -> i32 {
        use core::sync::atomic::Ordering;
        let Some(axis) = self
            .stepping_axes
            .get_mut(axis_idx as usize)
            .and_then(|s| s.as_mut())
        else {
            return -1;
        };
        axis.last_step_count = count;
        axis.p_prev = count as f32 * axis.microstep_distance;
        axis.v_prev = 0.0;
        for stepper in &axis.steppers {
            stepper.position_count.store(count, Ordering::Release);
            stepper.last_phase_target.store(count, Ordering::Release);
        }
        0
    }

    pub fn seed_position(&mut self, xyz: [f32; 3]) {
        use core::sync::atomic::Ordering;
        let motor_positions = [xyz[0], xyz[1], xyz[2], 0.0_f32, 0.0, 0.0, 0.0, 0.0];

        for (i, axis_opt) in self.stepping_axes.iter_mut().enumerate() {
            let Some(axis) = axis_opt.as_mut() else {
                continue;
            };
            let axis_pos_mm = motor_positions.get(i).copied().unwrap_or(0.0);
            let microstep_distance = axis.microstep_distance;
            if !microstep_distance.is_finite() || microstep_distance <= 0.0 {
                continue;
            }
            #[allow(clippy::cast_possible_truncation)]
            let seed_steps =
                libm::round(f64::from(axis_pos_mm) / f64::from(microstep_distance)) as i32;
            axis.last_step_count = seed_steps;
            axis.p_prev = axis_pos_mm;
            axis.v_prev = 0.0;
            for stepper in &axis.steppers {
                stepper.position_count.store(seed_steps, Ordering::Release);
                stepper
                    .last_phase_target
                    .store(seed_steps, Ordering::Release);
            }
        }
    }
}
