#![allow(clippy::unwrap_used)]

use crate::engine::Engine;
use crate::error::{
    KALICO_ERR_CORRECTION_IN_PROGRESS, KALICO_ERR_INVALID_ARG, KALICO_ERR_MOTION_IN_PROGRESS,
    KALICO_OK,
};
use crate::piece_ring::PieceEntry;
use crate::stepping_state::{
    CORRECTION_MOTOR_NONE, CORRECTION_RING_DEPTH, StepMode, StepperBindingRust, TMC_CS_OID_NONE,
};

const TEST_TOTAL_RING_PIECES: usize = 256;

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

fn one_piece(start_time: u64) -> PieceEntry {
    PieceEntry {
        start_time,
        coeffs: [0.0, 0.5, 1.0, 1.5],
        duration: 0.5,
        motor_mask: 0,
        _reserved: [0; 3],
    }
}

#[test]
fn motor_state_reads_seeded_position() {
    let (mut engine, _) = engine_with_z_axis(StepMode::Pulse);
    engine.seed_position([12.5, -3.0, 7.0]);
    assert_eq!(engine.motor_state(2), Some((7.0, 0.0)));
    assert!(engine.motor_state(0).is_none());
    assert!(engine.motor_state(7).is_none());
}

#[test]
fn configure_axis_allocates_correction_ring() {
    let (engine, _) = engine_with_z_axis(StepMode::Pulse);
    let axis = engine.stepping_axes[2].as_ref().unwrap();
    assert_eq!(axis.correction_ring.ring_depth, CORRECTION_RING_DEPTH);
    assert!(axis.correction_ring.ring_offset >= axis.ring.ring_offset + axis.ring.ring_depth);
}

#[test]
fn commit_correction_rejects_bad_motor_idx() {
    let (mut engine, mut storage) = engine_with_z_axis(StepMode::Pulse);
    assert_eq!(
        engine.write_correction_piece(2, 0, 0, one_piece(1000), &mut storage),
        KALICO_OK
    );
    assert_eq!(engine.commit_correction(2, 3, 1), KALICO_ERR_INVALID_ARG);
}

#[test]
fn commit_correction_rejects_busy_axis() {
    let (mut engine, mut storage) = engine_with_z_axis(StepMode::Pulse);
    assert_eq!(
        engine.push_pieces(2, &[one_piece(1000)], &mut storage),
        KALICO_OK
    );
    engine.write_correction_piece(2, 0, 0, one_piece(1000), &mut storage);
    assert_eq!(
        engine.commit_correction(2, 1, 1),
        KALICO_ERR_MOTION_IN_PROGRESS
    );
}

#[test]
fn commit_correction_rejects_second_stream_other_motor() {
    let (mut engine, mut storage) = engine_with_z_axis(StepMode::Pulse);
    engine.write_correction_piece(2, 0, 0, one_piece(1000), &mut storage);
    assert_eq!(engine.commit_correction(2, 1, 1), KALICO_OK);
    engine.write_correction_piece(2, 1, 0, one_piece(2000), &mut storage);
    assert_eq!(
        engine.commit_correction(2, 2, 2),
        KALICO_ERR_CORRECTION_IN_PROGRESS
    );
}

#[test]
fn commit_correction_allows_streaming_same_motor() {
    let (mut engine, mut storage) = engine_with_z_axis(StepMode::Pulse);
    engine.write_correction_piece(2, 0, 0, one_piece(1000), &mut storage);
    assert_eq!(engine.commit_correction(2, 1, 1), KALICO_OK);
    engine.write_correction_piece(2, 1, 0, one_piece(2000), &mut storage);
    assert_eq!(engine.commit_correction(2, 1, 2), KALICO_OK);
}

#[test]
fn normal_commit_rejected_while_correction_active() {
    let (mut engine, mut storage) = engine_with_z_axis(StepMode::Pulse);
    engine.write_correction_piece(2, 0, 0, one_piece(1000), &mut storage);
    assert_eq!(engine.commit_correction(2, 1, 1), KALICO_OK);
    assert_eq!(
        engine.guard_normal_commit(2),
        KALICO_ERR_CORRECTION_IN_PROGRESS
    );
}

#[test]
fn commit_correction_seeds_motor_tracking_state() {
    let (mut engine, mut storage) = engine_with_z_axis(StepMode::Pulse);
    assert_eq!(
        engine.stepping_axes[2]
            .as_ref()
            .unwrap()
            .correction_motor_idx,
        CORRECTION_MOTOR_NONE
    );
    engine.write_correction_piece(2, 0, 0, one_piece(1000), &mut storage);
    assert_eq!(engine.commit_correction(2, 1, 1), KALICO_OK);
    let axis = engine.stepping_axes[2].as_ref().unwrap();
    assert_eq!(axis.correction_motor_idx, 1);
    assert!(axis.correction_active());
}
