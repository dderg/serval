use super::{MotorSampler, PendingStep};
use crate::ring::PieceRing;
use crate::{MotorConfig, ShimError};
use runtime::piece_ring::PieceEntry;

const CYCLES_PER_SECOND: f64 = 1_000_000.0;
const SAMPLE_RATE_HZ: f32 = 10_000.0;
const SAMPLE_PERIOD_CYCLES: u64 = 100;
const MICROSTEP: f32 = 0.01;

fn cfg(max_steps_per_sample: u32) -> MotorConfig {
    MotorConfig {
        oid: 3,
        microstep_distance: MICROSTEP,
        invert_dir: false,
        max_steps_per_sample,
        sample_rate_hz: SAMPLE_RATE_HZ,
        cycles_per_second: CYCLES_PER_SECOND,
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

fn sample_all(
    cfg: &MotorConfig,
    pieces: &[PieceEntry],
    up_to_clock: u64,
) -> Result<Vec<PendingStep>, ShimError> {
    let mut ring = PieceRing::new(8);
    for piece in pieces {
        ring.push(0, *piece).unwrap();
    }
    let mut sampler = MotorSampler::new(cfg);
    let mut out = Vec::new();
    sampler.sample(0, cfg, &mut ring, up_to_clock, &mut out)?;
    Ok(out)
}

#[test]
fn constant_velocity_gives_uniform_step_times() {
    let cfg = cfg(16);
    let piece = linear_piece(1_000, 0.0, 1.0, 0.01);
    let steps = sample_all(&cfg, &[piece], u64::MAX).unwrap();

    assert_eq!(steps.len(), 100);
    assert!(steps.iter().all(|s| s.dir == 1 && s.advance == 1));

    let intervals: Vec<u64> = steps.windows(2).map(|w| w[1].clock - w[0].clock).collect();
    assert!(
        intervals
            .iter()
            .all(|d| d.abs_diff(SAMPLE_PERIOD_CYCLES) <= 1),
        "non-uniform intervals: {intervals:?}"
    );
    assert_eq!(steps[0].clock, 1_000 + SAMPLE_PERIOD_CYCLES / 2);
}

#[test]
fn step_clocks_are_strictly_increasing_across_pieces() {
    let cfg = cfg(16);
    let pieces = [
        linear_piece(1_000, 0.0, 1.0, 0.01),
        linear_piece(11_000, 1.0, 3.0, 0.01),
    ];
    let steps = sample_all(&cfg, &pieces, u64::MAX).unwrap();
    assert!(steps.windows(2).all(|w| w[1].clock > w[0].clock));
    assert_eq!(steps.len(), 300);
}

#[test]
fn direction_reversal_flips_sampled_dir() {
    let cfg = cfg(16);
    let pieces = [
        linear_piece(1_000, 0.0, 1.0, 0.01),
        linear_piece(11_000, 1.0, 0.0, 0.01),
    ];
    let steps = sample_all(&cfg, &pieces, u64::MAX).unwrap();

    let flips = steps.windows(2).filter(|w| w[0].dir != w[1].dir).count();
    assert_eq!(flips, 1);
    assert_eq!(steps.first().unwrap().dir, 1);
    assert_eq!(steps.last().unwrap().dir, 0);
    assert_eq!(steps.last().unwrap().advance, -1);
}

#[test]
fn invert_dir_swaps_the_wire_dir_bit() {
    let mut cfg = cfg(16);
    cfg.invert_dir = true;
    let steps = sample_all(&cfg, &[linear_piece(1_000, 0.0, 1.0, 0.01)], u64::MAX).unwrap();
    assert!(steps.iter().all(|s| s.dir == 0 && s.advance == 1));
}

#[test]
fn step_rate_cap_fails_loud() {
    let cfg = cfg(2);
    let err = sample_all(&cfg, &[linear_piece(1_000, 0.0, 5.0, 0.01)], u64::MAX).unwrap_err();
    match err {
        ShimError::StepRateExceeded { motor, steps, cap } => {
            assert_eq!(motor, 0);
            assert_eq!(cap, 2);
            assert!(steps > 2, "expected a burst above the cap, got {steps}");
        }
        other => panic!("expected StepRateExceeded, got {other:?}"),
    }
}

#[test]
fn sampling_stops_at_the_clock_budget() {
    let cfg = cfg(16);
    let piece = linear_piece(1_000, 0.0, 1.0, 0.01);
    let steps = sample_all(&cfg, &[piece], 3_000).unwrap();
    assert_eq!(steps.len(), 20);
    assert!(steps.iter().all(|s| s.clock <= 3_000));
}

#[test]
fn piece_retires_only_after_sampling_passes_its_end() {
    let cfg = cfg(16);
    let mut ring = PieceRing::new(4);
    ring.push(0, linear_piece(1_000, 0.0, 1.0, 0.01)).unwrap();
    ring.push(0, linear_piece(11_000, 1.0, 2.0, 0.01)).unwrap();
    let mut sampler = MotorSampler::new(&cfg);
    let mut out = Vec::new();

    sampler
        .sample(0, &cfg, &mut ring, 10_900, &mut out)
        .unwrap();
    assert_eq!(ring.retired(), 0);

    sampler
        .sample(0, &cfg, &mut ring, 11_100, &mut out)
        .unwrap();
    assert_eq!(ring.retired(), 1);

    sampler
        .sample(0, &cfg, &mut ring, u64::MAX, &mut out)
        .unwrap();
    assert_eq!(ring.retired(), 2);
    assert_eq!(ring.len(), 0);
}
