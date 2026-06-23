#![allow(
    clippy::unwrap_used,
    clippy::indexing_slicing,
    clippy::integer_division
)]

use core::sync::atomic::Ordering;

use crate::engine::Engine;
use crate::error::RUNTIME_OK;
use crate::piece_ring::PieceEntry;
use crate::state::SharedState;
use crate::step_queue::{StepQueue, pop as queue_pop};
use crate::stepping_state::{MAX_AXES, StepMode, StepperBindingRust, TMC_CS_OID_NONE};

const TEST_TOTAL_RING_PIECES: usize = 256;
const TICK_CLOCK_FREQ: u32 = 520_000_000;
const TICK_SAMPLE_RATE: u32 = 40_000;
const TICK_CYCLES: u64 = (TICK_CLOCK_FREQ / TICK_SAMPLE_RATE) as u64;

fn engine_with_z_axis(mode: StepMode) -> (Engine, Vec<PieceEntry>) {
    let mut engine = Engine::default();
    let storage = vec![
        PieceEntry {
            start_time: 0,
            coeffs: [0.0; 4],
            duration: 0.0,
            motor_mask: 0,
            _reserved: [0; 3]
        };
        TEST_TOTAL_RING_PIECES
    ];
    let bindings = [
        StepperBindingRust {
            stepper_oid: 10,
            tmc_cs_oid: TMC_CS_OID_NONE,
            _pad: [0; 2],
        },
        StepperBindingRust {
            stepper_oid: 11,
            tmc_cs_oid: TMC_CS_OID_NONE,
            _pad: [0; 2],
        },
        StepperBindingRust {
            stepper_oid: 12,
            tmc_cs_oid: TMC_CS_OID_NONE,
            _pad: [0; 2],
        },
    ];
    let rc = engine.configure_axis(2, mode, 0.00125, 64, &bindings, TEST_TOTAL_RING_PIECES);
    assert_eq!(rc, RUNTIME_OK);
    (engine, storage)
}

#[test]
fn motor_state_reads_seeded_position() {
    let (mut engine, _) = engine_with_z_axis(StepMode::Pulse);
    engine.seed_position([12.5, -3.0, 7.0]);
    assert_eq!(engine.motor_state(2), Some((7.0, 0.0)));
    assert!(engine.motor_state(0).is_none());
    assert!(engine.motor_state(7).is_none());
}

fn tickable_z_engine() -> (Engine, Vec<PieceEntry>) {
    let mut engine = Engine::new(TICK_CLOCK_FREQ, TICK_SAMPLE_RATE);
    let storage = vec![
        PieceEntry {
            start_time: 0,
            coeffs: [0.0; 4],
            duration: 0.0,
            motor_mask: 0,
            _reserved: [0; 3]
        };
        TEST_TOTAL_RING_PIECES
    ];
    let bindings = [
        StepperBindingRust {
            stepper_oid: 10,
            tmc_cs_oid: TMC_CS_OID_NONE,
            _pad: [0; 2],
        },
        StepperBindingRust {
            stepper_oid: 11,
            tmc_cs_oid: TMC_CS_OID_NONE,
            _pad: [0; 2],
        },
        StepperBindingRust {
            stepper_oid: 12,
            tmc_cs_oid: TMC_CS_OID_NONE,
            _pad: [0; 2],
        },
    ];
    let rc = engine.configure_axis(
        2,
        StepMode::Pulse,
        0.00125,
        64,
        &bindings,
        TEST_TOTAL_RING_PIECES,
    );
    assert_eq!(rc, RUNTIME_OK);
    (engine, storage)
}

fn moving_piece(start_time: u64, delta_mm: f32, motor_mask: u8) -> PieceEntry {
    PieceEntry {
        start_time,
        coeffs: [0.0, 0.0, delta_mm, delta_mm],
        duration: 0.01,
        motor_mask,
        _reserved: [0; 3],
    }
}

#[allow(unsafe_code)]
fn drain_through_piece(
    engine: &mut Engine,
    shared: &SharedState,
    storage: &mut [PieceEntry],
    q: &mut StepQueue,
    start: u64,
) {
    let ticks = (0.01 * TICK_CLOCK_FREQ as f32) as u64 / TICK_CYCLES + 2;
    for n in 0..=ticks {
        engine.tick(start + n * TICK_CYCLES, shared, storage);
        let q_ptr: *mut StepQueue = q;
        while unsafe { queue_pop(q_ptr) }.is_some() {}
    }
}

#[test]
fn overlay_uses_own_step_frame_not_axis_frame() {
    let (mut engine, mut storage) = tickable_z_engine();
    let mut q = StepQueue::new();
    let mut qs: [*mut StepQueue; MAX_AXES] = [core::ptr::null_mut(); MAX_AXES];
    qs[2] = &mut q;
    engine.test_install_step_queues(qs);
    let shared = SharedState::new();

    let normal_start = TICK_CYCLES;
    assert_eq!(
        engine.push_pieces(2, &[moving_piece(normal_start, 0.0125, 0)], &mut storage),
        RUNTIME_OK
    );
    drain_through_piece(&mut engine, &shared, &mut storage, &mut q, normal_start);
    let p_after_normal = engine.motor_state(2).unwrap().0;
    assert!(
        (p_after_normal - 0.0125).abs() < 1e-4,
        "normal piece must advance p_prev"
    );
    let axis_frame_after_normal = engine.stepping_axes[2].as_ref().unwrap().last_step_count;
    let stepper1_after_normal = engine.stepping_axes[2].as_ref().unwrap().steppers[1]
        .position_count
        .load(Ordering::Acquire);

    let overlay_start = normal_start + 200 * TICK_CYCLES;
    let overlay = PieceEntry {
        start_time: overlay_start,
        coeffs: [0.0, 0.01, 0.01, 0.01],
        duration: 0.01,
        motor_mask: 0b0000_0010,
        _reserved: [0; 3],
    };
    assert_eq!(engine.push_pieces(2, &[overlay], &mut storage), RUNTIME_OK);
    drain_through_piece(&mut engine, &shared, &mut storage, &mut q, overlay_start);

    assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
    assert_eq!(
        engine.stepping_axes[2].as_ref().unwrap().last_step_count,
        axis_frame_after_normal,
        "overlay piece must NOT perturb the axis curve frame"
    );
    assert_eq!(
        engine.motor_state(2).unwrap().0,
        p_after_normal,
        "overlay piece must NOT advance p_prev"
    );
    let stepper1_after_overlay = engine.stepping_axes[2].as_ref().unwrap().steppers[1]
        .position_count
        .load(Ordering::Acquire);
    assert_ne!(
        stepper1_after_overlay, stepper1_after_normal,
        "overlay must still step its targeted motor"
    );
}

struct LateArmResult {
    first_sample_steps: i32,
    cumulative_steps: i32,
}

struct OverlayHarness {
    engine: Engine,
    storage: Vec<PieceEntry>,
    shared: SharedState,
    q: Box<StepQueue>,
    overlay_axis: usize,
    next_start: u64,
    last_motor_idx: usize,
    last_signed_steps: i32,
}

impl OverlayHarness {
    fn new_single_motor(mstep_mm: f32) -> Self {
        let axis = 2usize;
        let mut engine = Engine::new(TICK_CLOCK_FREQ, TICK_SAMPLE_RATE);
        let storage = vec![
            PieceEntry {
                start_time: 0,
                coeffs: [0.0; 4],
                duration: 0.0,
                motor_mask: 0,
                _reserved: [0; 3]
            };
            TEST_TOTAL_RING_PIECES
        ];
        let bindings = [
            StepperBindingRust {
                stepper_oid: 10,
                tmc_cs_oid: TMC_CS_OID_NONE,
                _pad: [0; 2],
            },
            StepperBindingRust {
                stepper_oid: 11,
                tmc_cs_oid: TMC_CS_OID_NONE,
                _pad: [0; 2],
            },
        ];
        let rc = engine.configure_axis(
            axis as u8,
            StepMode::Pulse,
            mstep_mm,
            64,
            &bindings,
            TEST_TOTAL_RING_PIECES,
        );
        assert_eq!(rc, RUNTIME_OK);

        let mut q = Box::new(StepQueue::new());
        let mut qs: [*mut StepQueue; MAX_AXES] = [core::ptr::null_mut(); MAX_AXES];
        qs[axis] = q.as_mut();
        engine.test_install_step_queues(qs);

        Self {
            engine,
            storage,
            shared: SharedState::new(),
            q,
            overlay_axis: axis,
            next_start: TICK_CYCLES,
            last_motor_idx: 1,
            last_signed_steps: 0,
        }
    }

    fn arm_overlay_piece(&mut self, motor_idx: usize, delta_mm: f32) {
        let mask: u8 = 1u8 << motor_idx;
        let piece = moving_piece(self.next_start, delta_mm, mask);
        assert_eq!(
            self.engine
                .push_pieces(self.overlay_axis as u8, &[piece], &mut self.storage),
            RUNTIME_OK
        );
        self.last_motor_idx = motor_idx;
    }

    #[allow(unsafe_code)]
    fn run_piece_collect_signed_steps(&mut self) -> i32 {
        let before = self.position_count(self.last_motor_idx);
        let ticks = (0.01 * TICK_CLOCK_FREQ as f32) as u64 / TICK_CYCLES + 2;
        for n in 0..=ticks {
            self.engine.tick(
                self.next_start + n * TICK_CYCLES,
                &self.shared,
                &mut self.storage,
            );
            let q_ptr: *mut StepQueue = self.q.as_mut();
            while unsafe { queue_pop(q_ptr) }.is_some() {}
        }
        self.next_start += (ticks + 1) * TICK_CYCLES;
        assert_eq!(self.shared.last_error.load(Ordering::Acquire), 0);
        let after = self.position_count(self.last_motor_idx);
        self.last_signed_steps = after - before;
        self.last_signed_steps
    }

    fn position_count(&self, motor_idx: usize) -> i32 {
        self.engine.stepping_axes[self.overlay_axis]
            .as_ref()
            .unwrap()
            .steppers[motor_idx]
            .position_count
            .load(Ordering::Acquire)
    }

    fn p_prev(&self) -> f32 {
        self.engine.stepping_axes[self.overlay_axis]
            .as_ref()
            .unwrap()
            .p_prev
    }

    #[allow(unsafe_code)]
    fn run_overlay_piece_armed_late(
        &mut self,
        motor_idx: usize,
        delta_mm: f32,
        late_by_fraction: f32,
    ) -> LateArmResult {
        let duration_sec: f32 = 0.01;
        let duration_cycles = (duration_sec * TICK_CLOCK_FREQ as f32) as u64;
        let late_cycles = (late_by_fraction * duration_cycles as f32) as u64;
        let scheduled_start = self.next_start.saturating_sub(late_cycles);

        let mask: u8 = 1u8 << motor_idx;
        let piece = PieceEntry {
            start_time: scheduled_start,
            coeffs: [0.0, 0.0, delta_mm, delta_mm],
            duration: duration_sec,
            motor_mask: mask,
            _reserved: [0; 3],
        };
        assert_eq!(
            self.engine
                .push_pieces(self.overlay_axis as u8, &[piece], &mut self.storage),
            RUNTIME_OK
        );
        self.last_motor_idx = motor_idx;

        let before = self.position_count(motor_idx);

        self.engine
            .tick(self.next_start, &self.shared, &mut self.storage);
        let q_ptr: *mut StepQueue = self.q.as_mut();
        while unsafe { queue_pop(q_ptr) }.is_some() {}
        let first_sample_steps = self.position_count(motor_idx) - before;

        let ticks = duration_cycles / TICK_CYCLES + 2;
        for n in 1..=ticks {
            self.engine.tick(
                self.next_start + n * TICK_CYCLES,
                &self.shared,
                &mut self.storage,
            );
            while unsafe { queue_pop(q_ptr) }.is_some() {}
        }
        self.next_start += (ticks + 1) * TICK_CYCLES;

        assert_eq!(self.shared.last_error.load(Ordering::Acquire), 0);
        let cumulative_steps = self.position_count(motor_idx) - before;
        LateArmResult {
            first_sample_steps,
            cumulative_steps,
        }
    }
}

#[test]
fn overlay_piece_resets_frame_at_arm_and_emits_round_delta() {
    let mut h = OverlayHarness::new_single_motor(0.01);
    h.arm_overlay_piece(1, 0.50);
    let s1 = h.run_piece_collect_signed_steps();
    assert_eq!(s1, 50);
    h.arm_overlay_piece(1, 0.50);
    let s2 = h.run_piece_collect_signed_steps();
    assert_eq!(s2, 50, "second piece must reset frame and emit +50, not 0");
    assert_eq!(h.position_count(1), 100);
    assert_eq!(h.p_prev(), 0.0);
}

#[test]
fn symmetric_buzz_nets_position_count_to_zero() {
    let mut h = OverlayHarness::new_single_motor(0.01);
    h.arm_overlay_piece(1, 0.50);
    h.run_piece_collect_signed_steps();
    h.arm_overlay_piece(1, -0.50);
    h.run_piece_collect_signed_steps();
    assert_eq!(h.position_count(1), 0);
    assert_eq!(h.p_prev(), 0.0);
}

fn configure_pulse_axis(engine: &mut Engine, axis: usize, mstep: f32) {
    let bindings = [StepperBindingRust {
        stepper_oid: 10,
        tmc_cs_oid: TMC_CS_OID_NONE,
        _pad: [0; 2],
    }];
    assert_eq!(
        engine.configure_axis(
            axis as u8,
            StepMode::Pulse,
            mstep,
            64,
            &bindings,
            TEST_TOTAL_RING_PIECES
        ),
        RUNTIME_OK
    );
}

#[test]
fn resonance_buzz_arm_activates_per_axis_stream() {
    crate::buzz_stream::reset_for_test();
    let axis = 2usize;
    let mut engine = Engine::new(TICK_CLOCK_FREQ, TICK_SAMPLE_RATE);
    configure_pulse_axis(&mut engine, axis, 0.01);
    let shared = SharedState::new();

    assert_eq!(
        engine.resonance_buzz(&shared, 1u8 << axis, 0, 100_000, 100_000, 100_000, 20, 2, 0),
        0
    );
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
    assert!(crate::buzz_stream::axis_active(axis));
    assert!(!crate::buzz_stream::axis_active(0));
    crate::buzz_stream::reset_for_test();
}

#[test]
fn resonance_buzz_disarm_form_clears_streams() {
    crate::buzz_stream::reset_for_test();
    let axis = 2usize;
    let mut engine = Engine::new(TICK_CLOCK_FREQ, TICK_SAMPLE_RATE);
    configure_pulse_axis(&mut engine, axis, 0.01);
    let shared = SharedState::new();

    assert_eq!(
        engine.resonance_buzz(&shared, 1u8 << axis, 0, 100_000, 100_000, 100_000, 20, 2, 0),
        0
    );
    assert!(crate::buzz_stream::axis_active(axis));
    assert_eq!(
        engine.resonance_buzz(&shared, 1u8 << axis, 0, 100_000, 100_000, 0, 20, 2, 0),
        0
    );
    assert!(!crate::buzz_stream::axis_active(axis));
    crate::buzz_stream::reset_for_test();
}

#[test]
fn resonance_buzz_conflicts_with_armed_piece_on_same_axis() {
    use crate::error::FaultCode;
    crate::buzz_stream::reset_for_test();
    let axis = 2usize;
    let mut engine = Engine::new(TICK_CLOCK_FREQ, TICK_SAMPLE_RATE);
    configure_pulse_axis(&mut engine, axis, 0.01);
    let shared = SharedState::new();

    engine.stepping_axes[axis].as_mut().unwrap().armed = Some(crate::motion_core::ArmedPiece {
        mono_coeffs: [0.0; 4],
        vel_coeffs: [0.0; 3],
        piece_start_cycles: 0,
        piece_end_cycles: 0,
    });

    assert_eq!(
        engine.resonance_buzz(&shared, 1u8 << axis, 0, 100_000, 100_000, 100_000, 20, 2, 0),
        -1
    );
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::BuzzAxisConflict.as_i32()
    );
    assert!(!crate::buzz_stream::axis_active(axis));
    crate::buzz_stream::reset_for_test();
}

#[test]
fn resonance_buzz_conflicts_with_queued_piece_on_same_axis() {
    use crate::error::FaultCode;
    crate::buzz_stream::reset_for_test();
    let axis = 2usize;
    let mut engine = Engine::new(TICK_CLOCK_FREQ, TICK_SAMPLE_RATE);
    configure_pulse_axis(&mut engine, axis, 0.01);
    let mut storage = vec![moving_piece(0, 0.0, 0); TEST_TOTAL_RING_PIECES];
    let shared = SharedState::new();

    assert_eq!(
        engine.push_pieces(axis as u8, &[moving_piece(1_000, 0.0125, 0)], &mut storage),
        RUNTIME_OK
    );
    assert!(engine.stepping_axes[axis].as_ref().unwrap().armed.is_none());

    assert_eq!(
        engine.resonance_buzz(&shared, 1u8 << axis, 0, 100_000, 100_000, 100_000, 20, 2, 0),
        -1
    );
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::BuzzAxisConflict.as_i32()
    );
    assert!(!crate::buzz_stream::axis_active(axis));
    crate::buzz_stream::reset_for_test();
}

#[test]
fn push_pieces_rejected_while_buzz_active_on_axis() {
    crate::buzz_stream::reset_for_test();
    let axis = 2usize;
    let mut engine = Engine::new(TICK_CLOCK_FREQ, TICK_SAMPLE_RATE);
    configure_pulse_axis(&mut engine, axis, 0.01);
    let mut storage = vec![moving_piece(0, 0.0, 0); TEST_TOTAL_RING_PIECES];
    let shared = SharedState::new();

    assert_eq!(
        engine.resonance_buzz(&shared, 1u8 << axis, 0, 100_000, 100_000, 100_000, 20, 2, 0),
        0
    );
    assert!(crate::buzz_stream::axis_active(axis));
    assert_eq!(
        engine.push_pieces(axis as u8, &[moving_piece(1_000, 0.0125, 0)], &mut storage),
        crate::error::RUNTIME_ERR_INVALID_ARG
    );
    crate::buzz_stream::reset_for_test();
}

#[test]
fn resonance_buzz_rejects_axis_without_step_queue() {
    crate::buzz_stream::reset_for_test();
    let first_axis_without_queue = crate::step_queue::N_AXIS_STEP_QUEUES;
    let axis = first_axis_without_queue;
    let mut engine = Engine::new(TICK_CLOCK_FREQ, TICK_SAMPLE_RATE);
    configure_pulse_axis(&mut engine, axis, 0.01);
    let shared = SharedState::new();

    assert_eq!(
        engine.resonance_buzz(&shared, 1u8 << axis, 0, 100_000, 100_000, 100_000, 20, 2, 0),
        -1
    );
    assert!(!crate::buzz_stream::axis_active(0));
    crate::buzz_stream::reset_for_test();
}

#[test]
fn resonance_buzz_skips_axis_unconfigured_on_this_mcu() {
    crate::buzz_stream::reset_for_test();
    let mut engine = Engine::new(TICK_CLOCK_FREQ, TICK_SAMPLE_RATE);
    configure_pulse_axis(&mut engine, 2, 0.01);
    let shared = SharedState::new();
    assert_eq!(
        engine.resonance_buzz(&shared, 0b001, 0, 100_000, 100_000, 100_000, 20, 2, 0),
        0,
        "an unconfigured-here axis must be ignored, not rejected"
    );
    assert!(!crate::buzz_stream::axis_active(0));
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "no fault latched"
    );
    crate::buzz_stream::reset_for_test();
}

#[test]
fn resonance_buzz_routes_phase_mode_axis_to_xdirect() {
    crate::buzz_stream::reset_for_test();
    let mut engine = Engine::new(TICK_CLOCK_FREQ, TICK_SAMPLE_RATE);
    configure_pulse_axis(&mut engine, 0, 0.01);
    engine.stepping_axes[0]
        .as_ref()
        .unwrap()
        .mode
        .store(StepMode::Phase as u8, Ordering::Release);
    let shared = SharedState::new();
    assert_eq!(
        engine.resonance_buzz(&shared, 0b001, 0, 100_000, 100_000, 100_000, 20, 2, 0),
        0,
        "buzz on a Phase-mode axis must arm via XDIRECT, not fault"
    );
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0, "no fault");
    assert!(crate::buzz_stream::axis_active(0));
    assert!(
        crate::buzz_stream::is_xdirect(0),
        "phase-mode axis must be marked an XDIRECT buzz stream"
    );
    crate::buzz_stream::reset_for_test();
}

#[test]
fn resonance_buzz_routes_swept_pulse_axis_to_staircase() {
    crate::buzz_stream::reset_for_test();
    let mut engine = Engine::new(TICK_CLOCK_FREQ, TICK_SAMPLE_RATE);
    configure_pulse_axis(&mut engine, 0, 0.01);
    let shared = SharedState::new();
    assert_eq!(
        engine.resonance_buzz(&shared, 0b001, 0, 5_000, 60_000, 300_000, 200, 20, 0),
        0,
        "swept buzz on a Pulse axis must arm, not fault"
    );
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0, "no fault");
    assert!(crate::buzz_stream::axis_active(0));
    assert!(
        !crate::buzz_stream::is_xdirect(0),
        "pulse axis is not XDIRECT"
    );
    assert!(
        crate::buzz_stream::is_sweep(0),
        "swept pulse axis must run the staircase generator"
    );
    crate::buzz_stream::reset_for_test();
}

#[test]
fn resonance_buzz_routes_fixed_tone_pulse_axis_to_plain_tone() {
    crate::buzz_stream::reset_for_test();
    let mut engine = Engine::new(TICK_CLOCK_FREQ, TICK_SAMPLE_RATE);
    configure_pulse_axis(&mut engine, 0, 0.01);
    let shared = SharedState::new();
    assert_eq!(
        engine.resonance_buzz(&shared, 0b001, 0, 50_000, 50_000, 100_000, 100, 10, 0),
        0
    );
    assert!(crate::buzz_stream::axis_active(0));
    assert!(
        !crate::buzz_stream::is_sweep(0),
        "fixed tone is not a sweep"
    );
    assert!(!crate::buzz_stream::is_xdirect(0));
    crate::buzz_stream::reset_for_test();
}

#[test]
fn overlay_multi_piece_no_sample_exceeds_max_steps() {
    use crate::sub_sample_timing::MAX_STEPS_PER_SAMPLE;

    let mstep: f32 = 0.01;
    let axis_idx = 2usize;
    let mask: u8 = 0b0000_0010;

    let mut engine = Engine::new(TICK_CLOCK_FREQ, TICK_SAMPLE_RATE);
    let mut storage = vec![
        PieceEntry {
            start_time: 0,
            coeffs: [0.0; 4],
            duration: 0.0,
            motor_mask: 0,
            _reserved: [0; 3],
        };
        TEST_TOTAL_RING_PIECES
    ];
    let bindings = [
        StepperBindingRust {
            stepper_oid: 10,
            tmc_cs_oid: TMC_CS_OID_NONE,
            _pad: [0; 2],
        },
        StepperBindingRust {
            stepper_oid: 11,
            tmc_cs_oid: TMC_CS_OID_NONE,
            _pad: [0; 2],
        },
    ];
    assert_eq!(
        engine.configure_axis(
            axis_idx as u8,
            StepMode::Pulse,
            mstep,
            64,
            &bindings,
            TEST_TOTAL_RING_PIECES
        ),
        RUNTIME_OK
    );

    let mut q = StepQueue::new();
    let mut qs: [*mut StepQueue; MAX_AXES] = [core::ptr::null_mut(); MAX_AXES];
    qs[axis_idx] = &mut q;
    engine.test_install_step_queues(qs);
    let shared = SharedState::new();

    let mk_piece = |start: u64, span: f32, dur: f32| PieceEntry {
        start_time: start,
        coeffs: [0.0_f32, 0.0, span, span],
        duration: dur,
        motor_mask: mask,
        _reserved: [0; 3],
    };

    let accel_dur = 0.008_f32;
    let cruise_dur = 0.084_f32;
    let decel_dur = 0.008_f32;
    let t0 = TICK_CYCLES;
    let accel_cycles = (accel_dur * TICK_CLOCK_FREQ as f32) as u64;
    let cruise_cycles = (cruise_dur * TICK_CLOCK_FREQ as f32) as u64;
    let decel_cycles = (decel_dur * TICK_CLOCK_FREQ as f32) as u64;

    let accel_piece = mk_piece(t0, 0.08, accel_dur);
    let cruise_piece = mk_piece(t0 + accel_cycles, 0.84, cruise_dur);
    let decel_piece = mk_piece(t0 + accel_cycles + cruise_cycles, 0.08, decel_dur);

    assert_eq!(
        engine.push_pieces(
            axis_idx as u8,
            &[accel_piece, cruise_piece, decel_piece],
            &mut storage
        ),
        RUNTIME_OK
    );

    let total_cycles = accel_cycles + cruise_cycles + decel_cycles;
    let ticks = total_cycles / TICK_CYCLES + 4;

    let mut max_steps_in_sample: u32 = 0;
    let stepper_before = engine.stepping_axes[axis_idx].as_ref().unwrap().steppers[1]
        .position_count
        .load(Ordering::Acquire);

    for n in 0..=ticks {
        let before = engine.stepping_axes[axis_idx].as_ref().unwrap().steppers[1]
            .position_count
            .load(Ordering::Acquire);
        engine.tick(t0 + n * TICK_CYCLES, &shared, &mut storage);
        let after = engine.stepping_axes[axis_idx].as_ref().unwrap().steppers[1]
            .position_count
            .load(Ordering::Acquire);
        let steps_this_sample = (after - before).unsigned_abs();
        if steps_this_sample > max_steps_in_sample {
            max_steps_in_sample = steps_this_sample;
        }
        let q_ptr: *mut StepQueue = &mut q;
        #[allow(unsafe_code)]
        while unsafe { queue_pop(q_ptr) }.is_some() {}
    }

    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "no -310 StepsPerSampleExceeded fault must fire for relativized overlay pieces"
    );
    assert!(
        max_steps_in_sample <= MAX_STEPS_PER_SAMPLE as u32,
        "max steps in any single sample must be ≤ {MAX_STEPS_PER_SAMPLE}, got {max_steps_in_sample}"
    );

    let stepper_after = engine.stepping_axes[axis_idx].as_ref().unwrap().steppers[1]
        .position_count
        .load(Ordering::Acquire);
    let total_steps = stepper_after - stepper_before;
    assert_eq!(
        total_steps, 100,
        "total steps across all 3 pieces must equal 100 (1mm / 0.01mm/step), got {total_steps}"
    );
}
