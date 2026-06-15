#![allow(
    clippy::unwrap_used,
    clippy::indexing_slicing,
    clippy::integer_division
)]

use core::sync::atomic::Ordering;

use crate::engine::Engine;
use crate::error::KALICO_OK;
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
    assert_eq!(rc, KALICO_OK);
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
    assert_eq!(rc, KALICO_OK);
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
        // SAFETY: host test queue, sole consumer here.
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
        KALICO_OK
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
    assert_eq!(engine.push_pieces(2, &[overlay], &mut storage), KALICO_OK);
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
        assert_eq!(rc, KALICO_OK);

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
            KALICO_OK
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
            // SAFETY: host test queue, sole consumer here.
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
