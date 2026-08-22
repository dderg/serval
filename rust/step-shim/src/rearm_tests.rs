//! The mcu's classic stepper spends two scheduler events per step and re-arms
//! `stepper_load_next` from the pending unstep. A run whose first step lands
//! inside that window is loaded behind it — `motion.step_load_late`, then
//! "Stepper too far in past". The shim owes the caller that distance.

use super::{MotorConfig, ShimError, StepEncoder, StepFrame, StepShim};
use runtime::piece_ring::PieceEntry;

const CYCLES_PER_SECOND: f64 = 1_000_000.0;
const SAMPLE_RATE_HZ: f32 = 10_000.0;
const SAMPLE_CYCLES: u64 = 100;
const MICROSTEP: f32 = 0.01;
const OID: u32 = 7;
const MIN_REARM_CYCLES: u64 = 200;

fn cfg(min_rearm_cycles: u64) -> MotorConfig {
    MotorConfig {
        oid: OID,
        microstep_distance: MICROSTEP,
        invert_dir: false,
        max_steps_per_sample: 16,
        sample_rate_hz: SAMPLE_RATE_HZ,
        cycles_per_second: CYCLES_PER_SECOND,
        min_rearm_cycles,
        encoder: StepEncoder::Classic {
            max_error_ticks: super::compress::DEFAULT_MAX_ERROR_TICKS,
        },
    }
}

fn linear_piece(start_time: u64, from_mm: f32, to_mm: f32, duration: f32) -> PieceEntry {
    let mut entry = PieceEntry::zeroed();
    entry.start_time = start_time;
    entry.duration = duration;
    entry.coeff_count = 2;
    entry.coeffs[0] = 0.5 * (from_mm + to_mm);
    entry.coeffs[1] = 0.5 * (to_mm - from_mm);
    entry
}

/// One microstep out and straight back, a sample apart: the reversal splits
/// the stream into two runs whose steps are one sample period apart — inside
/// the mcu's two-pulse re-arm window.
fn reversal_pieces(start: u64) -> Vec<PieceEntry> {
    let sample_secs = SAMPLE_CYCLES as f32 / CYCLES_PER_SECOND as f32;
    vec![
        linear_piece(start, 0.0, MICROSTEP, sample_secs),
        linear_piece(start + SAMPLE_CYCLES, MICROSTEP, 0.0, sample_secs),
    ]
}

fn drain_all(shim: &mut StepShim, start: u64) -> Result<Vec<StepFrame>, ShimError> {
    let mut frames = shim.drain(start + 8 * SAMPLE_CYCLES)?;
    frames.extend(shim.finish(0)?);
    Ok(frames)
}

#[test]
fn a_reversal_inside_the_re_arm_window_is_refused() {
    let start = 10_000;
    let mut shim = StepShim::new(vec![cfg(MIN_REARM_CYCLES)], 16);
    shim.push_pieces(0, &reversal_pieces(start)).unwrap();

    let err = drain_all(&mut shim, start).expect_err("the mcu cannot re-arm this fast");
    match err {
        ShimError::StepTooSoon {
            motor,
            first,
            committed,
            min_rearm,
        } => {
            assert_eq!(motor, 0);
            assert_eq!(min_rearm, MIN_REARM_CYCLES);
            assert!(
                first > committed && first - committed < MIN_REARM_CYCLES,
                "the refused run must be the one inside the window: {first} vs {committed}"
            );
        }
        other => panic!("expected StepTooSoon, got {other}"),
    }
}

/// The same reversal on a both-edge driver, which configures zero pulse ticks
/// and so owes nothing beyond strict monotonicity.
#[test]
fn a_reversal_is_emitted_when_the_mcu_needs_no_re_arm() {
    let start = 10_000;
    let mut shim = StepShim::new(vec![cfg(0)], 16);
    shim.push_pieces(0, &reversal_pieces(start)).unwrap();

    let frames = drain_all(&mut shim, start).expect("a zero re-arm mcu takes this stream");
    let dir_changes = frames
        .iter()
        .filter(|f| matches!(f, StepFrame::SetNextStepDir { .. }))
        .count();
    assert_eq!(
        dir_changes, 2,
        "the stream must actually reverse: {frames:?}"
    );
}
