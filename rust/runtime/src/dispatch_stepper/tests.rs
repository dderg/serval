#![allow(clippy::indexing_slicing)]

use super::{DISPLACEMENT_THRESHOLD_MM, commit_position_count_masked, dispatch_axis};
use crate::state::SharedState;
use crate::step_queue::StepQueue;
use crate::stepping_state::{AxisConfig, StepMode, StepperRef};
use core::sync::atomic::{AtomicI16, AtomicI32, AtomicU8, Ordering};
use heapless::Vec;

fn make_stepper() -> StepperRef {
    StepperRef {
        stepper_oid: 0,
        position_count: AtomicI32::new(0),
        overlay_step_frame: AtomicI32::new(0),
        tmc_cs_oid: None,
        last_coil_A: AtomicI16::new(0),
        last_coil_B: AtomicI16::new(0),
        phase_offset_microsteps: AtomicI32::new(0),
        phase_offset_target: AtomicI32::new(0),
        last_phase_target: AtomicI32::new(0),
    }
}

fn make_axis(mode: StepMode, microstep_distance: f32) -> AxisConfig {
    let mut steppers: Vec<StepperRef, 4> = Vec::new();
    let _ = steppers.push(make_stepper());
    AxisConfig {
        mode: AtomicU8::new(mode as u8),
        steppers,
        microstep_distance,
        ..AxisConfig::new_unconfigured()
    }
}

#[test]
fn commit_masked_scopes_position_count() {
    let shared = SharedState::new();
    let mut axis = make_axis(StepMode::Pulse, 0.0125);
    let _ = axis.steppers.push(make_stepper());

    commit_position_count_masked(&axis, 0, &shared, 0, 5);
    assert_eq!(axis.steppers[0].position_count.load(Ordering::Acquire), 5);
    assert_eq!(axis.steppers[1].position_count.load(Ordering::Acquire), 5);

    commit_position_count_masked(&axis, 0, &shared, 0b10, 3);
    assert_eq!(axis.steppers[0].position_count.load(Ordering::Acquire), 5);
    assert_eq!(axis.steppers[1].position_count.load(Ordering::Acquire), 8);
}

#[test]
fn dispatch_pulse_honors_motor_mask() {
    use crate::error::FaultCode;

    {
        let shared = SharedState::new();
        let mut q = StepQueue::new();
        let mut axis = make_axis(StepMode::Pulse, 0.0125);
        let _ = axis.steppers.push(make_stepper());

        let q_ptr: *mut StepQueue = &mut q;
        dispatch_axis(
            0,
            &mut axis,
            /* motor_mask */ 0b10,
            q_ptr,
            &shared,
            /* p_end */ 0.05,
            /* v_end */ 2000.0,
            /* p_sample_start */ 0.0,
            /* sample_period_sec */ 25e-6,
            /* sample_start_cycles */ 1_000,
            /* cycles_per_second */ 520_000_000.0,
            /* overlay_just_armed */ false,
        );

        let enq = q.tail.wrapping_sub(q.head);
        assert_eq!(enq, 4, "expected 4 step entries, got {enq}");
        for i in q.head..q.tail {
            let entry = q.buf[(i % crate::step_queue::STEP_QUEUE_DEPTH as u16) as usize];
            assert_eq!(entry.stepper_sel, 2, "single-bit mask 0b10 => sel 2");
        }
        assert_eq!(
            axis.steppers[0].position_count.load(Ordering::Acquire),
            0,
            "motor 0 must not advance under mask 0b10"
        );
        assert_eq!(
            axis.steppers[1].position_count.load(Ordering::Acquire),
            4,
            "only motor 1 advances under mask 0b10"
        );
        assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
    }

    {
        let shared = SharedState::new();
        let mut q = StepQueue::new();
        let mut axis = make_axis(StepMode::Pulse, 0.0125);
        let _ = axis.steppers.push(make_stepper());

        let q_ptr: *mut StepQueue = &mut q;
        let axis_idx: usize = 1;
        dispatch_axis(
            axis_idx,
            &mut axis,
            /* motor_mask */ 0b11,
            q_ptr,
            &shared,
            /* p_end */ 0.05,
            /* v_end */ 2000.0,
            /* p_sample_start */ 0.0,
            /* sample_period_sec */ 25e-6,
            /* sample_start_cycles */ 1_000,
            /* cycles_per_second */ 520_000_000.0,
            /* overlay_just_armed */ false,
        );

        assert_eq!(q.tail, q.head, "no steps for a multi-bit mask");
        assert_eq!(
            shared.last_error.load(Ordering::Acquire),
            FaultCode::MultiMotorMask.as_i32(),
            "multi-bit mask must raise MultiMotorMask"
        );
        let detail = shared.fault_detail.load(Ordering::Acquire);
        let expected_detail = ((axis_idx as u32 & 0xFF) << 16) | 0b11;
        assert_eq!(detail, expected_detail);
        assert_eq!(axis.steppers[0].position_count.load(Ordering::Acquire), 0);
        assert_eq!(axis.steppers[1].position_count.load(Ordering::Acquire), 0);
    }
}

#[test]
fn pulse_zero_motion_no_steps_scheduled() {
    let shared = SharedState::new();
    let mut q = StepQueue::new();
    let mut axis = make_axis(StepMode::Pulse, 0.0125);

    let q_ptr: *mut StepQueue = &mut q;
    dispatch_axis(
        0,
        &mut axis,
        0,
        q_ptr,
        &shared,
        /* p_end */ 0.0,
        /* v_end */ 0.0,
        /* p_sample_start */ 0.0,
        /* sample_period_sec */ 25e-6,
        /* sample_start_cycles */ 0,
        /* cycles_per_second */ 520_000_000.0,
        /* overlay_just_armed */ false,
    );

    assert_eq!(q.tail, q.head, "no steps should be enqueued");
    assert_eq!(axis.last_step_count, 0);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "no fault should latch"
    );
}

#[test]
fn pulse_positive_motion_enqueues_n_steps() {
    let shared = SharedState::new();
    let mut q = StepQueue::new();
    let mut axis = make_axis(StepMode::Pulse, 0.0125);

    let q_ptr: *mut StepQueue = &mut q;
    dispatch_axis(
        0,
        &mut axis,
        0,
        q_ptr,
        &shared,
        /* p_end */ 0.05,
        /* v_end */ 2000.0,
        /* p_sample_start */ 0.0,
        /* sample_period_sec */ 25e-6,
        /* sample_start_cycles */ 1_000,
        /* cycles_per_second */ 520_000_000.0,
        /* overlay_just_armed */ false,
    );

    let enq = q.tail.wrapping_sub(q.head);
    assert_eq!(enq, 4, "expected 4 step entries, got {enq}");
    assert_eq!(axis.last_step_count, 4);
    assert_eq!(axis.steppers[0].position_count.load(Ordering::Acquire), 4);
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
}

#[test]
fn pulse_below_displacement_threshold_uses_uniform_fallback() {
    let shared = SharedState::new();
    let mut q = StepQueue::new();
    let mut axis = make_axis(StepMode::Pulse, 0.0125);

    axis.last_step_count = -2;
    let tiny = DISPLACEMENT_THRESHOLD_MM / 10.0;

    let q_ptr: *mut StepQueue = &mut q;
    dispatch_axis(
        0,
        &mut axis,
        0,
        q_ptr,
        &shared,
        /* p_end */ tiny,
        /* v_end */ 0.0,
        /* p_sample_start */ -tiny,
        /* sample_period_sec */ 25e-6,
        /* sample_start_cycles */ 0,
        /* cycles_per_second */ 520_000_000.0,
        /* overlay_just_armed */ false,
    );

    let enq = q.tail.wrapping_sub(q.head);
    assert_eq!(enq, 2);
    assert_eq!(axis.last_step_count, 0);
}

#[test]
fn phase_mode_updates_coil_state_no_queue_writes() {
    let shared = SharedState::new();
    let mut q = StepQueue::new();
    let mut axis = make_axis(StepMode::Phase, 0.0125);

    let q_ptr: *mut StepQueue = &mut q;
    dispatch_axis(
        0,
        &mut axis,
        0,
        q_ptr,
        &shared,
        /* p_end */ 256.0 * 0.0125,
        /* v_end */ 0.0,
        /* p_sample_start */ 0.0,
        /* sample_period_sec */ 25e-6,
        /* sample_start_cycles */ 0,
        /* cycles_per_second */ 520_000_000.0,
        /* overlay_just_armed */ false,
    );

    assert_eq!(q.tail, q.head, "phase mode must not enqueue step pulses");
    assert_eq!(axis.last_step_count, 256);
    assert_eq!(axis.steppers[0].last_coil_A.load(Ordering::Acquire), 0);
    assert_eq!(axis.steppers[0].last_coil_B.load(Ordering::Acquire), 248);
    assert_eq!(
        axis.steppers[0].last_phase_target.load(Ordering::Acquire),
        256
    );
    assert_eq!(axis.steppers[0].position_count.load(Ordering::Acquire), 256);
}

#[test]
fn phase_mode_ramps_offset_toward_target_at_max_per_sample() {
    let shared = SharedState::new();
    let mut q = StepQueue::new();
    let mut axis = make_axis(StepMode::Phase, 0.0125);
    axis.steppers[0]
        .phase_offset_target
        .store(10, Ordering::Release);
    shared
        .max_phase_offset_ramp_per_sample
        .store(4, Ordering::Release);

    let q_ptr: *mut StepQueue = &mut q;
    for expected in [4_i32, 8, 10] {
        dispatch_axis(
            0,
            &mut axis,
            0,
            q_ptr,
            &shared,
            /* p_end */ 256.0 * 0.0125,
            /* v_end */ 0.0,
            /* p_sample_start */ 0.0,
            /* sample_period_sec */ 25e-6,
            /* sample_start_cycles */ 0,
            /* cycles_per_second */ 520_000_000.0,
            /* overlay_just_armed */ false,
        );
        assert_eq!(
            axis.steppers[0]
                .phase_offset_microsteps
                .load(Ordering::Acquire),
            expected,
            "ramp should advance to {expected}",
        );
    }
}

#[test]
fn phase_mode_ramp_disabled_when_max_per_sample_is_zero() {
    let shared = SharedState::new();
    let mut q = StepQueue::new();
    let mut axis = make_axis(StepMode::Phase, 0.0125);
    axis.steppers[0]
        .phase_offset_microsteps
        .store(3, Ordering::Release);
    axis.steppers[0]
        .phase_offset_target
        .store(99, Ordering::Release);

    let q_ptr: *mut StepQueue = &mut q;
    dispatch_axis(
        0,
        &mut axis,
        0,
        q_ptr,
        &shared,
        /* p_end */ 256.0 * 0.0125,
        /* v_end */ 0.0,
        /* p_sample_start */ 0.0,
        /* sample_period_sec */ 25e-6,
        /* sample_start_cycles */ 0,
        /* cycles_per_second */ 520_000_000.0,
        /* overlay_just_armed */ false,
    );

    assert_eq!(
        axis.steppers[0]
            .phase_offset_microsteps
            .load(Ordering::Acquire),
        3,
        "ramp should be a no-op when max_per_sample == 0",
    );
}

#[test]
fn phase_mode_honors_phase_offset() {
    let shared = SharedState::new();
    let mut q = StepQueue::new();
    let mut axis = make_axis(StepMode::Phase, 0.0125);
    axis.steppers[0]
        .phase_offset_microsteps
        .store(7, Ordering::Release);

    let q_ptr: *mut StepQueue = &mut q;
    dispatch_axis(
        0,
        &mut axis,
        0,
        q_ptr,
        &shared,
        /* p_end */ 256.0 * 0.0125,
        /* v_end */ 0.0,
        /* p_sample_start */ 0.0,
        /* sample_period_sec */ 25e-6,
        /* sample_start_cycles */ 0,
        /* cycles_per_second */ 520_000_000.0,
        /* overlay_just_armed */ false,
    );

    assert_eq!(
        axis.steppers[0].last_phase_target.load(Ordering::Acquire),
        263
    );
    assert_eq!(axis.steppers[0].position_count.load(Ordering::Acquire), 263);
}

#[test]
fn unknown_step_mode_raises_fault() {
    use crate::error::FaultCode;

    let shared = SharedState::new();
    let mut q = StepQueue::new();

    let raw_mode: u8 = 0x42;
    let mut steppers: heapless::Vec<StepperRef, 4> = heapless::Vec::new();
    let _ = steppers.push(make_stepper());
    let mut axis = AxisConfig {
        mode: AtomicU8::new(raw_mode),
        steppers,
        microstep_distance: 0.0125,
        ..AxisConfig::new_unconfigured()
    };

    let q_ptr: *mut StepQueue = &mut q;
    let axis_idx: usize = 2;
    dispatch_axis(
        axis_idx,
        &mut axis,
        0,
        q_ptr,
        &shared,
        /* p_end */ 1.0,
        /* v_end */ 0.0,
        /* p_sample_start */ 0.0,
        /* sample_period_sec */ 25e-6,
        /* sample_start_cycles */ 0,
        /* cycles_per_second */ 520_000_000.0,
        /* overlay_just_armed */ false,
    );

    assert_eq!(
        q.tail, q.head,
        "no steps should be enqueued for unknown mode"
    );

    let last_err = shared.last_error.load(Ordering::Acquire);
    assert_eq!(
        last_err,
        FaultCode::UnknownStepMode.as_i32(),
        "expected UnknownStepMode fault code, got {last_err}"
    );

    let detail = shared.fault_detail.load(Ordering::Acquire);
    let expected_detail = ((axis_idx as u32 & 0xFF) << 16) | u32::from(raw_mode);
    assert_eq!(
        detail, expected_detail,
        "fault_detail should encode (axis_idx << 16) | mode"
    );
}

#[test]
fn overlay_arm_emits_zero_steps_and_seeds_frame_to_zero() {
    let mstep: f32 = 0.01;
    let mut axis = {
        let mut steppers: heapless::Vec<StepperRef, 4> = heapless::Vec::new();
        let _ = steppers.push(make_stepper());
        let _ = steppers.push(make_stepper());
        AxisConfig {
            mode: AtomicU8::new(StepMode::Pulse as u8),
            steppers,
            microstep_distance: mstep,
            ..AxisConfig::new_unconfigured()
        }
    };

    axis.steppers[1]
        .overlay_step_frame
        .store(999, Ordering::Release);

    let shared = SharedState::new();
    let mut q = StepQueue::new();
    let q_ptr: *mut StepQueue = &mut q;

    dispatch_axis(
        0,
        &mut axis,
        /* motor_mask */ 0b10,
        q_ptr,
        &shared,
        /* p_end */ 0.0,
        /* v_end */ 0.0,
        /* p_sample_start */ 0.0,
        /* sample_period_sec */ 25e-6,
        /* sample_start_cycles */ 1_000,
        /* cycles_per_second */ 520_000_000.0,
        /* overlay_just_armed */ true,
    );

    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "arm tick must not raise any fault"
    );
    assert_eq!(q.tail, q.head, "arm tick must enqueue zero steps");
    assert_eq!(
        axis.steppers[1].overlay_step_frame.load(Ordering::Acquire),
        0,
        "overlay_step_frame must be 0 after arm so full Δ plays from here"
    );
}

#[test]
fn overlay_on_phase_axis_applies_phase_offset() {
    let shared = SharedState::new();
    let mut q = StepQueue::new();
    let msd = 0.0125_f32;
    let mut axis = make_axis(StepMode::Phase, msd);

    let q_ptr: *mut StepQueue = &mut q;
    let axis_idx: usize = 1;
    let motor_mask: u8 = 0b01;
    let overlay_msteps: i32 = 5;
    let p_end = overlay_msteps as f32 * msd;

    dispatch_axis(
        axis_idx,
        &mut axis,
        motor_mask,
        q_ptr,
        &shared,
        p_end,
        /* v_end */ 0.0,
        /* p_sample_start */ 0.0,
        /* sample_period_sec */ 25e-6,
        /* sample_start_cycles */ 0,
        /* cycles_per_second */ 520_000_000.0,
        /* overlay_just_armed */ true,
    );

    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "overlay on phase axis must not fault"
    );
    assert_eq!(
        axis.steppers[0]
            .phase_offset_microsteps
            .load(Ordering::Acquire),
        overlay_msteps,
    );
    assert_eq!(
        axis.steppers[0].phase_offset_target.load(Ordering::Acquire),
        overlay_msteps,
    );
    assert_eq!(
        axis.steppers[0].overlay_step_frame.load(Ordering::Acquire),
        overlay_msteps,
    );
    assert_eq!(
        axis.steppers[0].position_count.load(Ordering::Acquire),
        overlay_msteps,
    );
    assert_eq!(q.tail, q.head, "no steps must be enqueued in phase mode");
}

#[test]
fn overlay_on_phase_axis_accumulates_across_samples() {
    let shared = SharedState::new();
    let mut q = StepQueue::new();
    let msd = 0.0125_f32;
    let mut axis = make_axis(StepMode::Phase, msd);

    let q_ptr: *mut StepQueue = &mut q;
    let axis_idx: usize = 0;
    let motor_mask: u8 = 0b01;

    dispatch_axis(
        axis_idx,
        &mut axis,
        motor_mask,
        q_ptr,
        &shared,
        3.0 * msd,
        0.0,
        0.0,
        25e-6,
        0,
        520_000_000.0,
        /* overlay_just_armed */ true,
    );
    assert_eq!(
        axis.steppers[0]
            .phase_offset_microsteps
            .load(Ordering::Acquire),
        3,
    );

    dispatch_axis(
        axis_idx,
        &mut axis,
        motor_mask,
        q_ptr,
        &shared,
        7.0 * msd,
        0.0,
        0.0,
        25e-6,
        0,
        520_000_000.0,
        /* overlay_just_armed */ false,
    );
    assert_eq!(
        axis.steppers[0]
            .phase_offset_microsteps
            .load(Ordering::Acquire),
        7,
        "second sample moves offset from 3 to 7 (delta 4 added to 3)"
    );

    dispatch_axis(
        axis_idx,
        &mut axis,
        motor_mask,
        q_ptr,
        &shared,
        2.0 * msd,
        0.0,
        0.0,
        25e-6,
        0,
        520_000_000.0,
        /* overlay_just_armed */ true,
    );
    assert_eq!(
        axis.steppers[0]
            .phase_offset_microsteps
            .load(Ordering::Acquire),
        9,
        "new overlay armed: delta from 0 baseline, adds 2 to existing 7"
    );
}
