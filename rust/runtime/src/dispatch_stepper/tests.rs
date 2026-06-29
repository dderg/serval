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
            assert_eq!(entry.stepper_sel(), 2, "single-bit mask 0b10 => sel 2");
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

// ---------------------------------------------------------------------------
// WI-7: phase-stepping SPI→DMA conversion.
//
// The per-bus DMA double-buffer, the cursor-walk FSM and the 5-byte XDIRECT
// packing live in C (`src/stm32/phase_stepping_spi.c`) and are not reachable
// from a Rust unit test. These models mirror that C contract so the packing
// golden vector, the motor→bus topology fan-out and the per-bus transition
// table are checked at the spec level. The packed commit-status decode is the
// real Rust seam and is exercised against `raise_phase_dma` in fault_helpers.
// ---------------------------------------------------------------------------

fn pack_xdirect_model(coil_a: i16, coil_b: i16) -> [u8; 5] {
    let ua = coil_a as u16;
    let ub = coil_b as u16;
    [
        0xAD,
        ((ub >> 8) & 0x01) as u8,
        (ub & 0xFF) as u8,
        ((ua >> 8) & 0x01) as u8,
        (ua & 0xFF) as u8,
    ]
}

#[test]
fn xdirect_packing_golden_vector() {
    assert_eq!(pack_xdirect_model(0, 0), [0xAD, 0x00, 0x00, 0x00, 0x00]);

    assert_eq!(
        pack_xdirect_model(171, -256),
        [0xAD, 0x01, 0x00, 0x00, 0xAB],
        "coil_b=-256 sets B sign bit, low byte 0; coil_a=171 → A low 0xAB"
    );

    assert_eq!(
        pack_xdirect_model(256, 255),
        [0xAD, 0x00, 0xFF, 0x01, 0x00],
        "coil_a=256 sets A sign bit; coil_b=255 stays in low byte"
    );

    assert_eq!(
        pack_xdirect_model(-1, -1),
        [0xAD, 0x01, 0xFF, 0x01, 0xFF],
        "negative coils: sign bit set, low byte 0xFF"
    );
}

#[test]
fn xdirect_packing_orders_b_before_a() {
    let d = pack_xdirect_model(0x00, 0x55);
    assert_eq!(d[0], 0xAD, "datagram opens with write|XDIRECT");
    assert_eq!((d[1], d[2]), (0x00, 0x55), "bytes 1..3 carry coil_B");
    assert_eq!((d[3], d[4]), (0x00, 0x00), "bytes 3..5 carry coil_A");
}

fn build_bus_sequences<const NB: usize>(regs: &[(u8, u8)]) -> [Vec<u8, 16>; NB] {
    let mut buses: [Vec<u8, 16>; NB] = core::array::from_fn(|_| Vec::new());
    for &(motor, bus) in regs {
        let _ = buses[bus as usize].push(motor);
    }
    buses
}

#[test]
fn topology_fan_out_single_bus_single_motor() {
    let buses = build_bus_sequences::<1>(&[(0, 0)]);
    assert_eq!(buses[0].as_slice(), &[0]);
}

#[test]
fn topology_fan_out_four_motors_one_bus_bench() {
    let buses = build_bus_sequences::<1>(&[(0, 0), (1, 0), (2, 0), (3, 0)]);
    assert_eq!(
        buses[0].as_slice(),
        &[0, 1, 2, 3],
        "bench: one bus drains all four motors in registration order"
    );
}

#[test]
fn topology_fan_out_two_by_two() {
    let buses = build_bus_sequences::<2>(&[(0, 0), (1, 0), (2, 1), (3, 1)]);
    assert_eq!(buses[0].as_slice(), &[0, 1]);
    assert_eq!(buses[1].as_slice(), &[2, 3]);
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
enum CsAction {
    Low(u8),
    High(u8),
}

#[derive(Debug, PartialEq, Eq)]
enum BusFsm {
    Idle,
    Armed(usize),
}

struct BusModel {
    seq: Vec<u8, 16>,
    state: BusFsm,
    transcript: Vec<CsAction, 64>,
}

impl BusModel {
    fn new(seq: &[u8]) -> Self {
        let mut s = Vec::new();
        for &m in seq {
            let _ = s.push(m);
        }
        Self {
            seq: s,
            state: BusFsm::Idle,
            transcript: Vec::new(),
        }
    }

    fn kick(&mut self) -> bool {
        if self.state != BusFsm::Idle {
            return false; // OVERRUN: prior batch not drained
        }
        if self.seq.is_empty() {
            return true;
        }
        self.state = BusFsm::Armed(0);
        let _ = self.transcript.push(CsAction::Low(self.seq[0]));
        true
    }

    fn on_transfer_complete(&mut self) {
        let BusFsm::Armed(cursor) = self.state else {
            return;
        };
        let _ = self.transcript.push(CsAction::High(self.seq[cursor]));
        let next = cursor + 1;
        if next < self.seq.len() {
            self.state = BusFsm::Armed(next);
            let _ = self.transcript.push(CsAction::Low(self.seq[next]));
        } else {
            self.state = BusFsm::Idle;
        }
    }

    fn run_to_drain(&mut self) {
        let mut guard = 0;
        while self.state != BusFsm::Idle && guard < 64 {
            self.on_transfer_complete();
            guard += 1;
        }
    }

    /// Commit-tick decision mirroring the C `phase_stepping_commit_tick`
    /// false-overrun discriminator. A busy bus whose FINAL transfer is
    /// hardware-complete (its TC merely queued behind the equal-priority tick)
    /// is finalized inline and re-armed — NOT reported as an overrun. Any other
    /// busy state (mid-walk, or final transfer still in flight) is a genuine
    /// overrun. Returns `false` on overrun, `true` otherwise.
    fn commit(&mut self, final_tc_hw_done: bool) -> bool {
        match self.state {
            BusFsm::Idle => {
                self.kick();
                true
            }
            BusFsm::Armed(cursor) => {
                let on_last = cursor + 1 == self.seq.len();
                if on_last && final_tc_hw_done {
                    let _ = self.transcript.push(CsAction::High(self.seq[cursor]));
                    self.state = BusFsm::Idle;
                    self.kick();
                    true
                } else {
                    false
                }
            }
        }
    }
}

#[test]
fn bus_fsm_single_motor_idle_armed_idle() {
    let mut bus = BusModel::new(&[0]);
    assert!(bus.kick());
    assert_eq!(bus.state, BusFsm::Armed(0));
    bus.on_transfer_complete();
    assert_eq!(bus.state, BusFsm::Idle);
    assert_eq!(
        bus.transcript.as_slice(),
        &[CsAction::Low(0), CsAction::High(0)]
    );
}

#[test]
fn bus_fsm_walks_four_motors_cs_high_precedes_next_cs_low() {
    let seq = [0u8, 1, 2, 3];
    let mut bus = BusModel::new(&seq);
    assert!(bus.kick());
    bus.run_to_drain();
    assert_eq!(bus.state, BusFsm::Idle);

    let expected = [
        CsAction::Low(0),
        CsAction::High(0),
        CsAction::Low(1),
        CsAction::High(1),
        CsAction::Low(2),
        CsAction::High(2),
        CsAction::Low(3),
        CsAction::High(3),
    ];
    assert_eq!(bus.transcript.as_slice(), &expected);

    let pos = |needle: CsAction| bus.transcript.iter().position(|a| *a == needle);
    for i in 0..seq.len() - 1 {
        let high_i = pos(CsAction::High(seq[i]));
        let low_next = pos(CsAction::Low(seq[i + 1]));
        assert!(
            high_i.is_some(),
            "CS-high({}) must be in transcript",
            seq[i]
        );
        assert!(
            low_next.is_some(),
            "CS-low({}) must be in transcript",
            seq[i + 1]
        );
        assert!(
            high_i < low_next,
            "CS-high({}) must strictly precede CS-low({})",
            seq[i],
            seq[i + 1]
        );
    }
}

#[test]
fn bus_fsm_kick_while_busy_reports_overrun() {
    let mut bus = BusModel::new(&[0, 1, 2, 3]);
    assert!(bus.kick(), "first kick from Idle arms the bus");
    assert!(
        !bus.kick(),
        "second kick while still Armed must report OVERRUN, not advance"
    );
    assert_eq!(
        bus.state,
        BusFsm::Armed(0),
        "an overrun kick does not advance the cursor"
    );
}

#[test]
fn commit_finalizes_pending_final_tc_instead_of_overrun() {
    let mut bus = BusModel::new(&[0, 1]);
    assert!(bus.kick());
    bus.on_transfer_complete();
    assert_eq!(bus.state, BusFsm::Armed(1), "now armed on the final motor");
    assert!(
        bus.commit(true),
        "final transfer hardware-done with TC merely pending must NOT fault"
    );
    assert_eq!(
        bus.state,
        BusFsm::Armed(0),
        "inline-finalize releases the done batch and re-arms a fresh one"
    );
}

#[test]
fn commit_reports_overrun_for_genuinely_unfinished_batch() {
    let mut mid = BusModel::new(&[0, 1]);
    assert!(mid.kick());
    assert!(
        !mid.commit(true),
        "mid-walk (cursor not on last motor) is a genuine overrun"
    );

    let mut last_in_flight = BusModel::new(&[0, 1]);
    assert!(last_in_flight.kick());
    last_in_flight.on_transfer_complete();
    assert!(
        !last_in_flight.commit(false),
        "final transfer still in flight (hw not done) is a genuine overrun"
    );
}

#[cfg(feature = "motion-module-stepper")]
#[test]
fn phase_dma_invalid_kind_decodes_to_internal_invariant() {
    use crate::error::FaultCode;
    use crate::fault_helpers::raise_phase_dma;

    let shared = SharedState::new();
    let bus_id = 3u32;
    let bogus_kind = 0x7Fu32;
    let status = (bus_id << 8) | bogus_kind;
    raise_phase_dma(&shared, status);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::InternalInvariant.as_i32(),
        "an unknown kind must surface as InternalInvariant, not a phantom overrun"
    );
    assert_eq!(shared.fault_detail.load(Ordering::Acquire), status);
}

#[cfg(feature = "motion-module-stepper")]
#[test]
fn phase_dma_status_decode_round_trips() {
    use crate::error::FaultCode;
    use crate::fault_helpers::{
        PHASE_DMA_KIND_FEIF, PHASE_DMA_KIND_OVERRUN, PHASE_DMA_KIND_TEIF, PHASE_DMA_KIND_UNDERRUN,
        raise_phase_dma,
    };

    let cases = [
        (PHASE_DMA_KIND_OVERRUN, FaultCode::PhaseDmaOverrun),
        (PHASE_DMA_KIND_TEIF, FaultCode::PhaseDmaTransferErr),
        (PHASE_DMA_KIND_FEIF, FaultCode::PhaseDmaFifoErr),
        (PHASE_DMA_KIND_UNDERRUN, FaultCode::PhaseDmaUnderrun),
    ];
    for (kind, fault) in cases {
        let shared = SharedState::new();
        let bus_id = 2u32;
        let status = (bus_id << 8) | kind;
        raise_phase_dma(&shared, status);
        assert_eq!(shared.last_error.load(Ordering::Acquire), fault.as_i32());
        assert_eq!(shared.fault_detail.load(Ordering::Acquire), status);
    }

    let clean = SharedState::new();
    raise_phase_dma(&clean, 0);
    assert_eq!(clean.last_error.load(Ordering::Acquire), 0);
}
