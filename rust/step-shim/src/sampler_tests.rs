use super::{MotorSampler, PendingStep};
use crate::ring::PieceRing;
use crate::{MotorConfig, ShimError, StepEncoder};
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
        min_rearm_cycles: 0,
        encoder: StepEncoder::Classic {
            max_error_ticks: crate::compress::DEFAULT_MAX_ERROR_TICKS,
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
fn tangential_half_step_does_not_emit_a_zero_width_pulse_pair() {
    let cfg = cfg(16);
    let pieces = [
        linear_piece(1_000, 0.0, 0.5 * MICROSTEP, 0.0001),
        linear_piece(1_100, 0.5 * MICROSTEP, 0.0, 0.0001),
    ];
    let steps = sample_all(&cfg, &pieces, u64::MAX).unwrap();
    assert!(steps.is_empty());
}

#[test]
fn continued_half_step_crossing_emits_once_at_the_boundary() {
    let cfg = cfg(16);
    let pieces = [
        linear_piece(1_000, 0.0, 0.5 * MICROSTEP, 0.0001),
        linear_piece(1_100, 0.5 * MICROSTEP, MICROSTEP, 0.0001),
    ];
    let steps = sample_all(&cfg, &pieces, u64::MAX).unwrap();

    assert_eq!(steps.len(), 1);
    assert_eq!(steps[0].clock, 1_100);
    assert_eq!(steps[0].advance, 1);
}

#[test]
fn large_step_count_tangency_keeps_the_previous_quantized_position() {
    let mut cfg = cfg(50);
    cfg.microstep_distance = 0.000_690_468_75;
    cfg.cycles_per_second = 64_000_000.0;
    let previous_count = 903_348;
    let previous_position = previous_count as f32 * cfg.microstep_distance;
    let tangent_position = (previous_count as f32 + 0.5) * cfg.microstep_distance;
    let first = linear_piece(64_000, previous_position, tangent_position, 0.0001);
    let second = linear_piece(
        first.end_time(cfg.cycles_per_second as f32),
        tangent_position,
        previous_position,
        0.0001,
    );
    let mut ring = PieceRing::new(8);
    ring.push(0, first).unwrap();
    ring.push(0, second).unwrap();
    let mut sampler = MotorSampler::new(&cfg);
    sampler.reset_to(previous_count, &cfg, 0);
    let mut steps = Vec::new();

    sampler
        .sample(0, &cfg, &mut ring, u64::MAX, &mut steps)
        .unwrap();

    assert_eq!(steps.len(), 2);
    assert_eq!([steps[0].advance, steps[1].advance], [1, -1]);
    assert!(steps[0].clock < steps[1].clock);
    assert_eq!(sampler.step_count(), previous_count);
}

#[test]
fn large_step_count_direction_reversal_keeps_monotonic_clocks() {
    let mut cfg = cfg(50);
    cfg.microstep_distance = 0.000_690_468_75;
    cfg.cycles_per_second = 64_000_000.0;
    let previous_count = 8_498_429;
    let previous_position = previous_count as f32 * cfg.microstep_distance;
    let peak_position = (previous_count + 2) as f32 * cfg.microstep_distance;
    let forward = linear_piece(64_000, previous_position, peak_position, 0.0001);
    let reverse = linear_piece(
        forward.end_time(cfg.cycles_per_second as f32),
        peak_position,
        previous_position,
        0.0001,
    );
    let mut ring = PieceRing::new(8);
    ring.push(0, forward).unwrap();
    ring.push(0, reverse).unwrap();
    let mut sampler = MotorSampler::new(&cfg);
    sampler.reset_to(previous_count, &cfg, 0);
    let mut steps = Vec::new();

    sampler
        .sample(0, &cfg, &mut ring, u64::MAX, &mut steps)
        .unwrap();

    assert!(
        steps
            .windows(2)
            .all(|steps| steps[0].clock < steps[1].clock)
    );
    assert_eq!(sampler.step_count(), previous_count);
}

#[test]
fn large_step_count_retraction_stays_after_the_previous_pulse() {
    let mut cfg = cfg(50);
    cfg.microstep_distance = 0.000_690_468_75;
    cfg.cycles_per_second = 64_000_000.0;
    let sample_start = 188_324_244_698;
    let previous_clock = 188_324_245_473;
    let sample_clock = 188_324_251_098;
    let piece = linear_piece(188_324_246_322, 5_867.9, 5_867.9, 0.01);
    let armed = runtime::motion_core::arm_piece(&piece, cfg.cycles_per_second as f32);
    let mut sampler = MotorSampler::new(&cfg);
    sampler.p_prev = 5_867.901;
    sampler.step_count = 8_498_431;
    sampler.prev_sample = sample_start;
    sampler.last_step_clock = Some(previous_clock);
    let mut steps = Vec::new();

    sampler
        .emit_sample(0, &cfg, &armed, sample_clock, &mut steps)
        .unwrap();

    assert!(steps.iter().all(|step| step.clock > previous_clock));
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

fn overlay_piece(start_time: u64, span_mm: f32, duration: f32) -> PieceEntry {
    let mut entry = linear_piece(start_time, 0.0, span_mm, duration);
    entry.motor_mask = 0b0000_0001;
    entry
}

#[test]
fn overlay_piece_steps_relative_to_its_own_zero() {
    let cfg = cfg(16);
    let mut ring = PieceRing::new(4);
    ring.push(0, linear_piece(1_000, 0.0, 1.0, 0.01)).unwrap();
    ring.push(0, overlay_piece(11_000, 0.5, 0.01)).unwrap();
    let mut sampler = MotorSampler::new(&cfg);
    let mut out = Vec::new();
    sampler
        .sample(0, &cfg, &mut ring, u64::MAX, &mut out)
        .unwrap();

    let lane_steps = (1.0 / MICROSTEP) as usize;
    let overlay_steps = (0.5 / MICROSTEP) as usize;
    assert_eq!(out.len(), lane_steps + overlay_steps);
    assert!(
        out.iter().all(|s| s.advance == 1),
        "the overlay must not walk the lane back to its own zero first"
    );
    assert_eq!(
        sampler.step_count(),
        lane_steps as i64,
        "an overlay run must leave the lane's absolute frame untouched"
    );
}

#[test]
fn consecutive_overlay_pieces_each_restart_at_zero() {
    let cfg = cfg(16);
    let mut ring = PieceRing::new(4);
    ring.push(0, overlay_piece(1_000, 0.2, 0.01)).unwrap();
    ring.push(0, overlay_piece(11_000, 0.6, 0.01)).unwrap();
    ring.push(0, overlay_piece(21_000, 0.2, 0.01)).unwrap();
    let mut sampler = MotorSampler::new(&cfg);
    let mut out = Vec::new();
    sampler
        .sample(0, &cfg, &mut ring, u64::MAX, &mut out)
        .unwrap();

    assert_eq!(out.len(), ((0.2 + 0.6 + 0.2) / MICROSTEP) as usize);
    assert!(out.iter().all(|s| s.advance == 1));
    assert_eq!(sampler.step_count(), 0);
}
