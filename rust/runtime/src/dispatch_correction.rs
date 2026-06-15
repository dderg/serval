#![allow(unsafe_code)]

use core::sync::atomic::Ordering;

use crate::fault_helpers::{
    raise_position_count_overflow, raise_step_queue_overflow, raise_steps_per_sample_exceeded,
    raise_unknown_step_mode,
};
use crate::log_codes::{
    EVENT_MOTION_CORRECTION_DRAINED, EVENT_MOTION_CORRECTION_START, SUBSYSTEM_MOTION,
};
use crate::piece_ring::PieceEntry;
use crate::state::SharedState;
use crate::step_queue::{StepEntry, StepQueue, peek as queue_peek, push as queue_push};
use crate::stepping_state::{AxisState, CORRECTION_MOTOR_NONE, StepMode};
use crate::sub_sample_timing::{
    MAX_STEPS_PER_SAMPLE, StepTimeInputs, StepTimingResult, compute_step_times,
};

const LOG_LEVEL_DEBUG: u8 = 1;

#[cfg(any(not(any(test, feature = "host")), feature = "mcu-linux"))]
unsafe extern "C" {
    fn event_log_emit(level: u8, subsystem: u8, event: u16, code: u16, arg0: u32, arg1: u32);
}

pub(crate) fn emit_correction_event(event: u16, arg0: u32, arg1: u32) {
    #[cfg(any(not(any(test, feature = "host")), feature = "mcu-linux"))]
    // SAFETY: event_log_emit is a pure C logging sink; no aliasing or
    // ownership constraints on its arguments.
    unsafe {
        event_log_emit(LOG_LEVEL_DEBUG, SUBSYSTEM_MOTION, event, 0, arg0, arg1);
    }
    #[cfg(not(any(not(any(test, feature = "host")), feature = "mcu-linux")))]
    {
        let _ = (LOG_LEVEL_DEBUG, SUBSYSTEM_MOTION, event, arg0, arg1);
    }
}

pub(crate) fn emit_correction_start(axis_idx: u8, motor_idx: u8) {
    emit_correction_event(
        EVENT_MOTION_CORRECTION_START,
        u32::from(axis_idx),
        u32::from(motor_idx),
    );
}

#[allow(clippy::too_many_arguments)]
pub fn tick_correction(
    axis_idx: usize,
    axis: &mut AxisState,
    queue_ptr: *mut StepQueue,
    shared: &SharedState,
    storage: &mut [PieceEntry],
    now: u64,
    sample_period_cycles: u32,
    sample_period_sec: f32,
    sample_start_cycles: u32,
    cycles_per_second: f32,
    fault: &impl crate::fault_sink::FaultSink,
) -> bool {
    if !axis.correction_active() {
        return false;
    }
    let motor_idx = axis.correction_motor_idx;
    let eval = crate::motion_core::get_position_and_velocity(
        &mut axis.correction_armed,
        &mut axis.correction_ring,
        storage,
        now,
        sample_period_cycles,
        cycles_per_second,
        axis_idx,
        fault,
    );
    let Some((c_end, _v_end)) = eval else {
        if !axis.correction_active() {
            finish_stream(axis_idx, axis);
        }
        return true;
    };
    let c_prev = axis.correction_p_prev;
    axis.correction_p_prev = c_end;

    match axis.mode.load(Ordering::Acquire) {
        m if m == StepMode::Pulse as u8 => emit_correction_steps(
            axis_idx,
            axis,
            queue_ptr,
            shared,
            c_end,
            c_prev,
            sample_period_sec,
            sample_start_cycles,
            cycles_per_second,
            motor_idx,
        ),
        m if m == StepMode::Phase as u8 => advance_phase_target(axis, c_end, motor_idx),
        other => raise_unknown_step_mode(shared, axis_idx, other),
    }
    true
}

#[allow(clippy::too_many_arguments)]
fn emit_correction_steps(
    axis_idx: usize,
    axis: &mut AxisState,
    queue_ptr: *mut StepQueue,
    shared: &SharedState,
    c_end: f32,
    c_prev: f32,
    sample_period_sec: f32,
    sample_start_cycles: u32,
    cycles_per_second: f32,
    motor_idx: u8,
) {
    if queue_ptr.is_null() {
        return;
    }
    let microstep_distance = axis.microstep_distance;
    if !microstep_distance.is_finite() || microstep_distance == 0.0 {
        return;
    }

    let prev_step_count = axis.correction_last_step_count;
    #[allow(clippy::cast_possible_truncation)]
    let target_step_count = libm::roundf(c_end / microstep_distance) as i32;
    let signed_steps = target_step_count.wrapping_sub(prev_step_count);
    axis.correction_last_step_count = target_step_count;

    if signed_steps == 0 {
        return;
    }
    let abs_steps = signed_steps.unsigned_abs();
    if abs_steps > MAX_STEPS_PER_SAMPLE as u32 {
        axis.correction_last_step_count = prev_step_count;
        raise_steps_per_sample_exceeded(shared, axis_idx, abs_steps);
        return;
    }

    let inputs = StepTimeInputs {
        p_start: c_prev,
        p_end: c_end,
        prev_step_count,
        target_step_count,
        microstep_distance,
        sample_period_sec,
        sample_start_cycles,
        cycles_per_second,
        displacement_threshold: crate::dispatch_stepper::DISPLACEMENT_THRESHOLD_MM,
    };

    let result = compute_step_times(&inputs);
    let times = match result {
        StepTimingResult::SecantSlope(t) | StepTimingResult::Uniform(t) => t,
        StepTimingResult::NoSteps => return,
    };

    let dir: i8 = if signed_steps > 0 { 1 } else { -1 };

    // SAFETY: sole consumer at same NVIC priority — cannot race with peek.
    let was_empty = unsafe { queue_peek(queue_ptr) }.is_none();
    let first_cycle_abs = times.first().copied();

    let mut steps_committed: i32 = 0;
    for cycle_abs in times.iter().copied() {
        let entry = StepEntry {
            cycle_abs,
            dir,
            stepper_sel: motor_idx + 1,
            _pad: [0; 2],
        };
        // SAFETY: `queue_ptr` is supplied by the TIM5 ISR, sole producer.
        let push_res = unsafe { queue_push(queue_ptr, entry) };
        if push_res.is_err() {
            let committed_delta = steps_committed * i32::from(dir);
            commit_motor_position_count(axis, axis_idx, shared, motor_idx, committed_delta);
            if was_empty && steps_committed > 0 {
                if let Some(wt) = first_cycle_abs {
                    crate::dispatch_stepper::kick_per_axis_timer(axis_idx, wt);
                }
            }
            raise_step_queue_overflow(shared, axis_idx);
            axis.correction_last_step_count = prev_step_count + committed_delta;
            return;
        }
        steps_committed += 1;
    }

    if was_empty && steps_committed > 0 {
        if let Some(wt) = first_cycle_abs {
            crate::dispatch_stepper::kick_per_axis_timer(axis_idx, wt);
        }
    }

    commit_motor_position_count(axis, axis_idx, shared, motor_idx, signed_steps);
}

fn commit_motor_position_count(
    axis: &AxisState,
    axis_idx: usize,
    shared: &SharedState,
    motor_idx: u8,
    delta: i32,
) {
    if delta == 0 {
        return;
    }
    let Some(stepper) = axis.steppers.get(motor_idx as usize) else {
        return;
    };
    let prev = stepper.position_count.load(Ordering::Acquire);
    let Some(next) = prev.checked_add(delta) else {
        raise_position_count_overflow(shared, axis_idx);
        return;
    };
    stepper.position_count.store(next, Ordering::Release);
}

fn advance_phase_target(axis: &mut AxisState, c_end: f32, motor_idx: u8) {
    let microstep_distance = axis.microstep_distance;
    if !microstep_distance.is_finite() || microstep_distance == 0.0 {
        return;
    }
    #[allow(clippy::cast_possible_truncation)]
    let scratch_steps = libm::roundf(c_end / microstep_distance) as i32;
    let delta = scratch_steps.wrapping_sub(axis.correction_last_step_count);
    if delta == 0 {
        return;
    }
    axis.correction_last_step_count = scratch_steps;
    let Some(stepper) = axis.steppers.get(motor_idx as usize) else {
        return;
    };
    let new_target = stepper
        .phase_offset_target
        .load(Ordering::Acquire)
        .wrapping_add(delta);
    stepper
        .phase_offset_target
        .store(new_target, Ordering::Release);
}

fn finish_stream(axis_idx: usize, axis: &mut AxisState) {
    emit_correction_event(
        EVENT_MOTION_CORRECTION_DRAINED,
        axis_idx as u32,
        axis.correction_last_step_count.unsigned_abs(),
    );
    axis.correction_motor_idx = CORRECTION_MOTOR_NONE;
    axis.correction_last_step_count = 0;
    axis.correction_p_prev = 0.0;
}

#[cfg(test)]
mod tests;
