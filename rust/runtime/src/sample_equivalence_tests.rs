#![allow(
    clippy::panic,
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::integer_division
)]

// Go/no-go evidence for the sample-stream executor: it must reproduce the
// piece executor's LUT phase index stream over the same lowered trajectory.
//
// Both executors run against one trapezoid, expressed as three Chebyshev
// pieces:
//
//   piece path   Clenshaw at the sample window's END, then the
//                residual-accumulating microstep quantizer, then `& 0x3FF`.
//   sample path  the same Clenshaw values sampled at the run clocks and
//                rounded to LUT quanta by a host-side encoder, then this
//                crate's linear interpolation and rounding, then `& 0x3FF`.
//
// The two executors evaluate different instants inside one tick: the piece
// path targets `now + period`, the sample path targets `now`. The comparison
// shifts by that one tick so the instants line up; everything left is
// quantization and interpolation.

use crate::motion_core::{ArmedPiece, arm_piece};
use crate::phase_lut::PHASE_LUT_SIZE;
use crate::piece_ring::PieceEntry;
use crate::sample_exec::{LaneOutput, SampleLane};
use crate::sample_run::{SAMPLE_RUN_COUNT_MAX, encode_deltas};
use crate::state::SharedState;
use crate::sub_sample_timing::quantize_step_delta;

const CLOCK_HZ: f32 = 520_000_000.0;
const CLOCK_TICKS: u32 = 520_000_000;
const TICK_HZ: u32 = 40_000;
#[allow(clippy::integer_division)]
const PERIOD: u32 = CLOCK_TICKS / TICK_HZ;
const ANCHOR: u64 = 1_000_000;
/// 40 mm/rev belt on a 200-step motor at 256 microsteps.
const MICROSTEP_MM: f32 = 0.2 / 256.0;
const LUT_MASK: i32 = PHASE_LUT_SIZE as i32 - 1;

const _: () = assert!(PERIOD * TICK_HZ == CLOCK_TICKS);

struct Leg {
    duration: f32,
    accel: f32,
}

/// Chebyshev coefficients of `p0 + v0*s + a*s²/2` on `u ∈ [-1, 1]`, where
/// `s = duration*(u+1)/2`.
fn constant_accel_piece(start_time: u64, p0: f32, v0: f32, leg: &Leg) -> PieceEntry {
    let mut entry = PieceEntry::zeroed();
    entry.start_time = start_time;
    entry.duration = leg.duration;
    entry.coeff_count = 3;
    let ramp = v0 * leg.duration / 2.0;
    let curve = leg.accel * leg.duration * leg.duration / 8.0;
    entry.coeffs[0] = p0 + ramp + 1.5 * curve;
    entry.coeffs[1] = ramp + 2.0 * curve;
    entry.coeffs[2] = 0.5 * curve;
    entry
}

fn trapezoid() -> Vec<PieceEntry> {
    let legs = [
        Leg {
            duration: 0.010,
            accel: 8_000.0,
        },
        Leg {
            duration: 0.010,
            accel: 0.0,
        },
        Leg {
            duration: 0.010,
            accel: -8_000.0,
        },
    ];
    let mut pieces = Vec::new();
    let mut start_time = ANCHOR;
    let mut position = 0.0_f32;
    let mut velocity = 0.0_f32;
    for leg in &legs {
        pieces.push(constant_accel_piece(start_time, position, velocity, leg));
        position += velocity * leg.duration + 0.5 * leg.accel * leg.duration * leg.duration;
        velocity += leg.accel * leg.duration;
        start_time += (leg.duration * CLOCK_HZ) as u64;
    }
    pieces
}

/// The ISR's own evaluation: arm the piece covering `clock`, Clenshaw at it.
/// Positions past the last piece hold its endpoint, exactly as an idle axis
/// holds `p_prev`.
fn armed_for(pieces: &[PieceEntry], clock: u64) -> ArmedPiece {
    let mut chosen = &pieces[0];
    for piece in pieces {
        if clock >= piece.start_time {
            chosen = piece;
        }
    }
    arm_piece(chosen, CLOCK_HZ)
}

fn eval_at(pieces: &[PieceEntry], clock: u64) -> f32 {
    let armed = armed_for(pieces, clock);
    let clamped = clock.min(armed.piece_end_cycles);
    armed.eval_pos_vel(clamped).0
}

fn trajectory_end(pieces: &[PieceEntry]) -> u64 {
    let last = pieces.last().expect("trapezoid has legs");
    last.start_time + (last.duration * CLOCK_HZ) as u64
}

/// The piece executor: Clenshaw at the window end, residual-accumulating
/// quantizer, LUT index. Mirrors `dispatch_phase`.
fn piece_lut_stream(pieces: &[PieceEntry], ticks: usize) -> Vec<i32> {
    let mut step_phase = 0.0_f32;
    let mut step_count = 0_i32;
    let mut previous = 0.0_f32;
    let mut stream = Vec::with_capacity(ticks);
    for tick in 0..ticks {
        let window_end = ANCHOR + (tick as u64 + 1) * u64::from(PERIOD);
        let position = eval_at(pieces, window_end);
        let step_phase_end = step_phase + (position - previous);
        let delta = quantize_step_delta(step_phase_end, MICROSTEP_MM);
        step_count = step_count.wrapping_add(delta);
        step_phase = step_phase_end - delta as f32 * MICROSTEP_MM;
        previous = position;
        stream.push(step_count & LUT_MASK);
    }
    stream
}

/// The host-side sample encoder: the lowered trajectory rounded to LUT quanta
/// at each run clock.
fn encode_samples(pieces: &[PieceEntry], interval: u32, count: usize) -> Vec<i32> {
    (0..count)
        .map(|index| {
            let clock = ANCHOR + (index as u64) * u64::from(interval);
            libm::roundf(eval_at(pieces, clock) / MICROSTEP_MM) as i32
        })
        .collect()
}

/// The sample executor driven at the tick rate, fed just-in-time so the ring
/// depth is exercised rather than sidestepped.
fn sample_lut_stream(samples: &[i32], interval: u32, ticks: usize) -> Vec<i32> {
    let shared = SharedState::new();
    let mut lane = SampleLane::new();
    lane.anchor(0, ANCHOR, samples[0]).expect("anchor accepted");
    let mut fed = 0usize;
    let mut base = samples[0];
    let mut stream = Vec::with_capacity(ticks);
    for tick in 0..ticks {
        let now = ANCHOR + (tick as u64) * u64::from(PERIOD);
        while fed < samples.len() {
            let end = (fed + SAMPLE_RUN_COUNT_MAX).min(samples.len());
            let chunk = &samples[fed..end];
            let mut wire = [0u8; 256];
            let written = encode_deltas(base, chunk, &mut wire).expect("encodes");
            #[allow(clippy::cast_possible_truncation)]
            let count = chunk.len() as u8;
            if lane
                .push_run(now, interval, count, &wire[..written])
                .is_err()
            {
                break;
            }
            base = *chunk.last().expect("non-empty chunk");
            fed = end;
        }
        match lane.tick(now, &shared, 0) {
            LaneOutput::Position(position) => stream.push(position & LUT_MASK),
            LaneOutput::Unanchored => panic!("lane went unanchored mid-stream"),
        }
    }
    assert_eq!(
        shared
            .last_error
            .load(core::sync::atomic::Ordering::Acquire),
        0,
        "sample executor latched a fault while replaying the trajectory"
    );
    stream
}

/// Shortest signed distance between two LUT indices, so a wrap through 1023→0
/// is not read as a 1023-quantum jump.
fn lut_distance(a: i32, b: i32) -> i32 {
    let half = PHASE_LUT_SIZE as i32 / 2;
    (a - b + half).rem_euclid(PHASE_LUT_SIZE as i32) - half
}

/// The two streams evaluate instants one tick apart, so the sample stream is
/// read one tick ahead.
fn max_divergence(piece: &[i32], sample: &[i32]) -> i32 {
    let mut worst = 0;
    for (index, expected) in piece.iter().copied().enumerate() {
        let Some(&actual) = sample.get(index + 1) else {
            break;
        };
        worst = worst.max(lut_distance(actual, expected).abs());
    }
    worst
}

fn ticks_for(pieces: &[PieceEntry]) -> usize {
    #[allow(clippy::cast_possible_truncation)]
    let span = (trajectory_end(pieces) - ANCHOR) as usize;
    span / (PERIOD as usize)
}

/// A comparison over a trajectory that never crosses a quantum would pass
/// trivially. This trapezoid sweeps two whole electrical cycles.
fn assert_stream_sweeps_the_lut(samples: &[i32], piece: &[i32]) {
    let total = samples.last().copied().unwrap_or(0);
    assert!(
        total >= 2 * PHASE_LUT_SIZE as i32,
        "trajectory only spans {total} quanta; the comparison would be vacuous"
    );
    let distinct: std::collections::BTreeSet<i32> = piece.iter().copied().collect();
    assert!(
        distinct.len() > PHASE_LUT_SIZE / 2,
        "piece stream only visited {} LUT phases",
        distinct.len()
    );
}

#[test]
fn at_the_tick_rate_the_two_executors_agree_exactly() {
    let pieces = trapezoid();
    let ticks = ticks_for(&pieces);
    let piece = piece_lut_stream(&pieces, ticks);
    let samples = encode_samples(&pieces, PERIOD, ticks + 2);
    let sample = sample_lut_stream(&samples, PERIOD, ticks + 1);
    assert_stream_sweeps_the_lut(&samples, &piece);
    let worst = max_divergence(&piece, &sample);
    assert_eq!(
        worst, 0,
        "matched-rate streams must be bit-identical, diverged by {worst} quanta"
    );
}

/// One sample per four ticks: three of every four ticks land strictly between
/// samples, where the executor blends linearly while the piece path evaluates
/// the true Chebyshev curve. On a sample instant the blend degenerates to the
/// sample itself and the two agree exactly; in between, the curvature the
/// chord misses can carry the rounded blend one quantum either way, and no
/// further, because the chord's error over one sample interval of this
/// trajectory stays below half a quantum plus the rounding step.
#[test]
fn between_samples_interpolation_stays_inside_one_quantum() {
    let pieces = trapezoid();
    let interval = PERIOD * 4;
    let ticks = ticks_for(&pieces);
    let piece = piece_lut_stream(&pieces, ticks);
    #[allow(clippy::integer_division)]
    let sample_count = ticks / 4 + 2;
    let samples = encode_samples(&pieces, interval, sample_count);
    let sample = sample_lut_stream(&samples, interval, ticks + 1);
    assert_stream_sweeps_the_lut(&samples, &piece);

    let mut between_samples_divergences = 0usize;
    for (index, expected) in piece.iter().copied().enumerate() {
        let Some(&actual) = sample.get(index + 1) else {
            break;
        };
        let divergence = lut_distance(actual, expected).abs();
        if (index + 1) % 4 == 0 {
            assert_eq!(
                divergence, 0,
                "tick {index} lands on a sample clock and must match exactly"
            );
        } else {
            assert!(
                divergence <= 1,
                "tick {index} diverged by {divergence} quanta, bound is 1"
            );
            between_samples_divergences += usize::from(divergence != 0);
        }
    }
    assert!(
        between_samples_divergences > 0,
        "no tick diverged at all, so the interpolation bound is untested"
    );
}

#[test]
fn the_matched_rate_stream_ends_on_the_same_position_as_the_piece_stream() {
    let pieces = trapezoid();
    let ticks = ticks_for(&pieces);
    let piece = piece_lut_stream(&pieces, ticks);
    let samples = encode_samples(&pieces, PERIOD, ticks + 2);
    let sample = sample_lut_stream(&samples, PERIOD, ticks + 1);
    assert_eq!(
        piece.last().copied(),
        sample.last().copied(),
        "the two executors parked the coils on different LUT phases"
    );
}

#[test]
fn the_encoder_round_trips_through_the_wire_without_losing_a_quantum() {
    let pieces = trapezoid();
    let samples = encode_samples(&pieces, PERIOD, 400);
    let mut base = samples[0];
    let mut decoded = Vec::with_capacity(samples.len());
    for chunk in samples.chunks(SAMPLE_RUN_COUNT_MAX) {
        let mut wire = [0u8; 256];
        let written = encode_deltas(base, chunk, &mut wire).expect("encodes");
        let mut out = vec![0i32; chunk.len()];
        crate::sample_run::decode_deltas(base, &wire[..written], chunk.len(), &mut out)
            .expect("decodes");
        base = *chunk.last().expect("non-empty chunk");
        decoded.extend_from_slice(&out);
    }
    assert_eq!(decoded, samples);
}
