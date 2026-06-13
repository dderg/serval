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
use crate::stepping_state::{
    CORRECTION_MOTOR_NONE, MAX_AXES, StepMode, StepperBindingRust, TMC_CS_OID_NONE,
};

const CLOCK_FREQ: u32 = 520_000_000;
const SAMPLE_RATE: u32 = 40_000;
const TICK_CYCLES: u64 = (CLOCK_FREQ / SAMPLE_RATE) as u64;
const TEST_TOTAL_RING_PIECES: usize = 256;
const MICROSTEP_MM: f32 = 0.00125;

fn blank_storage() -> Vec<PieceEntry> {
    vec![
        PieceEntry {
            start_time: 0,
            coeffs: [0.0; 4],
            duration: 0.0,
            _reserved: 0
        };
        TEST_TOTAL_RING_PIECES
    ]
}

fn engine_with_z_axis(mode: StepMode) -> Engine {
    let mut engine = Engine::new(CLOCK_FREQ, SAMPLE_RATE);
    let tmc = |oid: u8| match mode {
        StepMode::Pulse => TMC_CS_OID_NONE,
        StepMode::Phase => oid,
    };
    let bindings = [
        StepperBindingRust {
            stepper_oid: 10,
            tmc_cs_oid: tmc(20),
            _pad: [0; 2],
        },
        StepperBindingRust {
            stepper_oid: 11,
            tmc_cs_oid: tmc(21),
            _pad: [0; 2],
        },
        StepperBindingRust {
            stepper_oid: 12,
            tmc_cs_oid: tmc(22),
            _pad: [0; 2],
        },
    ];
    let rc = engine.configure_axis(2, mode, MICROSTEP_MM, 64, &bindings, TEST_TOTAL_RING_PIECES);
    assert_eq!(rc, KALICO_OK);
    engine
}

fn smooth_piece(start_time: u64, delta_mm: f32, duration: f32) -> PieceEntry {
    PieceEntry {
        start_time,
        coeffs: [0.0, 0.0, delta_mm, delta_mm],
        duration,
        _reserved: 0,
    }
}

fn install_z_queue(engine: &mut Engine, q: &mut StepQueue) {
    let mut qs: [*mut StepQueue; MAX_AXES] = [core::ptr::null_mut(); MAX_AXES];
    qs[2] = q;
    engine.test_install_step_queues(qs);
}

fn tick_through_stream(
    engine: &mut Engine,
    shared: &SharedState,
    storage: &mut [PieceEntry],
    start: u64,
    duration: f32,
    q: &mut StepQueue,
    drained: &mut Vec<crate::step_queue::StepEntry>,
) {
    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    let stream_ticks = (duration * CLOCK_FREQ as f32) as u64 / TICK_CYCLES + 2;
    for n in 0..=stream_ticks {
        engine.tick(start + n * TICK_CYCLES, shared, storage);
        let q_ptr: *mut StepQueue = q;
        // SAFETY: host test queue, sole consumer here.
        while let Some(entry) = unsafe { queue_pop(q_ptr) } {
            drained.push(entry);
        }
    }
}

#[test]
fn pulse_correction_steps_only_selected_stepper() {
    let mut engine = engine_with_z_axis(StepMode::Pulse);
    let mut storage = blank_storage();
    let mut q = StepQueue::new();
    install_z_queue(&mut engine, &mut q);
    let shared = SharedState::new();

    let start = TICK_CYCLES;
    let piece = smooth_piece(start, 0.0125, 0.01);
    assert_eq!(
        engine.write_correction_piece(2, 0, 0, piece, &mut storage),
        KALICO_OK
    );
    assert_eq!(engine.commit_correction(2, 1, 1), KALICO_OK);

    let mut drained = Vec::new();
    tick_through_stream(
        &mut engine,
        &shared,
        &mut storage,
        start,
        0.01,
        &mut q,
        &mut drained,
    );

    assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
    assert_eq!(drained.len(), 10, "expected 10 microsteps");
    for entry in &drained {
        assert_eq!(entry.stepper_sel, 2, "steps must route to stepper index 1");
        assert_eq!(entry.dir, 1);
    }
    let axis = engine.stepping_axes[2].as_ref().unwrap();
    assert_eq!(axis.last_step_count, 0, "main tracker must stay untouched");
    assert_eq!(axis.steppers[1].position_count.load(Ordering::Acquire), 10);
    assert_eq!(axis.steppers[0].position_count.load(Ordering::Acquire), 0);
    assert_eq!(axis.steppers[2].position_count.load(Ordering::Acquire), 0);
}

#[test]
fn pulse_correction_stream_end_resets_relative_frame() {
    let mut engine = engine_with_z_axis(StepMode::Pulse);
    let mut storage = blank_storage();
    let mut q = StepQueue::new();
    install_z_queue(&mut engine, &mut q);
    let shared = SharedState::new();

    let start = TICK_CYCLES;
    engine.write_correction_piece(2, 0, 0, smooth_piece(start, 0.0125, 0.01), &mut storage);
    assert_eq!(engine.commit_correction(2, 1, 1), KALICO_OK);
    let mut drained = Vec::new();
    tick_through_stream(
        &mut engine,
        &shared,
        &mut storage,
        start,
        0.01,
        &mut q,
        &mut drained,
    );

    {
        let axis = engine.stepping_axes[2].as_ref().unwrap();
        assert!(!axis.correction_active());
        assert_eq!(axis.correction_last_step_count, 0);
        assert_eq!(axis.correction_motor_idx, CORRECTION_MOTOR_NONE);
    }

    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    let second_start = start + (0.01 * CLOCK_FREQ as f32) as u64 + 4 * TICK_CYCLES;
    engine.write_correction_piece(
        2,
        0,
        0,
        smooth_piece(second_start, 0.0125, 0.01),
        &mut storage,
    );
    assert_eq!(engine.commit_correction(2, 0, 1), KALICO_OK);
    let mut drained2 = Vec::new();
    tick_through_stream(
        &mut engine,
        &shared,
        &mut storage,
        second_start,
        0.01,
        &mut q,
        &mut drained2,
    );
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
    assert_eq!(drained2.len(), 10);
    for entry in &drained2 {
        assert_eq!(entry.stepper_sel, 1, "second stream routes to stepper 0");
    }
}

#[test]
fn phase_correction_moves_only_selected_offset_target() {
    let mut engine = engine_with_z_axis(StepMode::Phase);
    let mut storage = blank_storage();
    let mut q = StepQueue::new();
    install_z_queue(&mut engine, &mut q);
    let shared = SharedState::new();
    shared.phase_motor_count.store(3, Ordering::Release);
    shared.phase_slot_idx[0].store(2, Ordering::Release);
    shared.phase_slot_idx[1].store(2, Ordering::Release);
    shared.phase_slot_idx[2].store(2, Ordering::Release);

    let start = TICK_CYCLES;
    engine.write_correction_piece(2, 0, 0, smooth_piece(start, 0.0125, 0.01), &mut storage);
    assert_eq!(engine.commit_correction(2, 0, 1), KALICO_OK);
    let mut drained = Vec::new();
    tick_through_stream(
        &mut engine,
        &shared,
        &mut storage,
        start,
        0.01,
        &mut q,
        &mut drained,
    );

    assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
    assert!(drained.is_empty(), "phase mode must not enqueue pulses");
    let axis = engine.stepping_axes[2].as_ref().unwrap();
    assert_eq!(
        axis.steppers[0].phase_offset_target.load(Ordering::Acquire),
        10
    );
    assert_eq!(
        axis.steppers[1].phase_offset_target.load(Ordering::Acquire),
        0
    );
    assert_eq!(
        axis.steppers[2].phase_offset_target.load(Ordering::Acquire),
        0
    );
    assert_eq!(axis.p_prev, 0.0, "main p_prev must stay untouched");
}

#[test]
fn correction_does_not_mark_axis_position() {
    let mut engine = engine_with_z_axis(StepMode::Pulse);
    let mut storage = blank_storage();
    let mut q = StepQueue::new();
    install_z_queue(&mut engine, &mut q);
    let shared = SharedState::new();

    let (p_prev_before, last_count_before, v_prev_before) = {
        let axis = engine.stepping_axes[2].as_ref().unwrap();
        (axis.p_prev, axis.last_step_count, axis.v_prev)
    };

    let start = TICK_CYCLES;
    engine.write_correction_piece(2, 0, 0, smooth_piece(start, 0.0125, 0.01), &mut storage);
    assert_eq!(engine.commit_correction(2, 2, 1), KALICO_OK);
    let mut drained = Vec::new();
    tick_through_stream(
        &mut engine,
        &shared,
        &mut storage,
        start,
        0.01,
        &mut q,
        &mut drained,
    );

    let axis = engine.stepping_axes[2].as_ref().unwrap();
    assert_eq!(axis.p_prev, p_prev_before);
    assert_eq!(axis.last_step_count, last_count_before);
    assert_eq!(axis.v_prev, v_prev_before);
    assert!(!drained.is_empty());
    for entry in &drained {
        assert_eq!(entry.stepper_sel, 3);
    }
}
