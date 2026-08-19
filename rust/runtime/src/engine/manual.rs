use core::sync::atomic::Ordering;

use crate::buzz::AxisExcitation;
use crate::state::SharedState;
use crate::stepping_state::{MAX_AXES, StepMode};

use super::Engine;

impl Engine {
    fn validate_excitations_idle(
        &self,
        excitations: &heapless::Vec<AxisExcitation, MAX_AXES>,
        shared: &SharedState,
    ) -> Result<(), i32> {
        for ex in excitations {
            if ex.axis_idx >= crate::step_queue::N_AXIS_STEP_QUEUES {
                crate::fault_helpers::raise_jog_parameters_invalid(shared);
                return Err(-1);
            }
            #[cfg(feature = "sample-stepping")]
            if self.sample_lane_anchored(ex.axis_idx) {
                crate::fault_helpers::raise_buzz_axis_conflict(shared, ex.axis_idx);
                return Err(-1);
            }
        }
        Ok(())
    }

    fn arm_excitation_streams(
        &self,
        excitations: &heapless::Vec<AxisExcitation, MAX_AXES>,
        now_cycle: u32,
    ) {
        let cps = f64::from(self.cycles_per_second);
        for ex in excitations {
            let Some(axis) = self.stepping_axes.get(ex.axis_idx).and_then(|s| s.as_ref()) else {
                continue;
            };
            let params = ex.into_params(
                f64::from(axis.p_prev),
                f64::from(axis.microstep_distance),
                cps,
                now_cycle,
            );
            if axis.mode.load(Ordering::Acquire) == StepMode::Phase as u8 {
                let cfg = crate::buzz_xdirect::XdirectConfig::new(
                    axis.microstep_distance,
                    crate::buzz_xdirect::DEFAULT_XDIRECT_UPDATE_HZ,
                );
                crate::buzz_stream::arm_axis_xdirect(ex.axis_idx, params, cfg);
            } else if params.mu != 0.0 {
                crate::buzz_stream::arm_axis_sweep(ex.axis_idx, params);
            } else {
                crate::buzz_stream::arm_axis(ex.axis_idx, params);
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn resonance_buzz(
        &mut self,
        shared: &SharedState,
        axis_mask: u8,
        sign_mask: u8,
        freq_start_millihz: u32,
        freq_end_millihz: u32,
        amplitude_nm: u32,
        duration_ms: u32,
        ramp_ms: u32,
        now_cycle: u32,
    ) -> i32 {
        let rc = self.buzz.arm(
            self.num_axes,
            axis_mask,
            sign_mask,
            freq_start_millihz,
            freq_end_millihz,
            amplitude_nm,
            duration_ms,
            ramp_ms,
        );
        if rc != 0 {
            return rc;
        }
        if !self.buzz.has_pending() {
            return 0;
        }
        let excitations = self.buzz.take_excitations();
        if excitations.is_empty() {
            for i in 0..crate::step_queue::N_AXIS_STEP_QUEUES {
                crate::buzz_stream::clear_axis(i);
            }
            return 0;
        }
        if let Err(rc) = self.validate_excitations_idle(&excitations, shared) {
            return rc;
        }
        self.arm_excitation_streams(&excitations, now_cycle);
        #[cfg(not(any(test, feature = "host")))]
        #[allow(unsafe_code)]
        unsafe {
            crate::buzz_stream::refill_foreground_all(
                now_cycle,
                crate::step_queue::queue_for_axis,
                crate::dispatch_stepper::kick_per_axis_timer_foreground,
            );
        }
        0
    }

    pub fn emit_xdirect_buzz(&self, axis_idx: usize, offset_steps: i32, shared: &SharedState) {
        if let Some(Some(axis)) = self.stepping_axes.get(axis_idx) {
            crate::dispatch_stepper::write_phase_coils(axis_idx, axis, shared, offset_steps);
        }
    }

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
