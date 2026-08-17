use super::compress::StepMove;
use super::{MotorConfig, ShimError, StepEncoder, StepFrame, StepShim};
use runtime::piece_ring::PieceEntry;

const CYCLES_PER_SECOND: f64 = 1_000_000.0;
const OID: u32 = 7;

fn cfg() -> MotorConfig {
    MotorConfig {
        oid: OID,
        microstep_distance: 0.01,
        invert_dir: false,
        max_steps_per_sample: 16,
        sample_rate_hz: 10_000.0,
        cycles_per_second: CYCLES_PER_SECOND,
        min_rearm_cycles: 0,
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

fn queue_step_count(frames: &[StepFrame]) -> u32 {
    frames
        .iter()
        .map(|f| match f {
            StepFrame::QueueStep { count, .. } => u32::from(*count),
            _ => 0,
        })
        .sum()
}

/// A re-anchoring volley's reset clock and the clock its first step lands on.
/// The reset heads the volley, so it must sit as close to that first step as
/// the protocol allows — never back at the clock the piece stream began on.
fn reset_and_first_step(frames: &[StepFrame]) -> (u64, u64) {
    let StepFrame::ResetStepClock { clock, .. } = frames[0] else {
        panic!(
            "a re-anchoring volley must open with reset_step_clock: {:?}",
            frames[0]
        );
    };
    let first = frames
        .iter()
        .find_map(|f| match f {
            StepFrame::QueueStep { interval, .. } => Some(u64::from(*interval)),
            _ => None,
        })
        .expect("the volley must carry steps");
    (u64::from(clock), u64::from(clock) + first)
}

/// The classic encoder keeps the pre-change stream shape: the same anchor,
/// the same dir latch, and a cursor the queue_step span arithmetic walks to
/// exactly the clock the shim reports as emitted.
#[test]
fn classic_stream_matches_the_pre_change_expectations() {
    let mut shim = StepShim::new(vec![cfg()], 8);
    shim.push_pieces(0, &[linear_piece(1_000, 0.0, 1.0, 0.01)])
        .unwrap();
    let frames = shim.drain(u64::MAX).unwrap();

    let (reset, first_step) = reset_and_first_step(&frames);
    assert_eq!(reset + 1, first_step);
    assert!(reset >= 1_000, "reset {reset} predates the piece it opens");
    assert_eq!(frames[1], StepFrame::SetNextStepDir { oid: OID, dir: 1 });
    let mut cursor = 0u64;
    let mut steps = 0u32;
    for frame in &frames {
        match frame {
            StepFrame::ResetStepClock { clock, .. } => cursor = u64::from(*clock),
            StepFrame::SetNextStepDir { .. } => {}
            StepFrame::QueueStep {
                interval,
                count,
                add,
                ..
            } => {
                cursor = StepMove {
                    interval: *interval,
                    count: *count,
                    add: *add,
                }
                .last_clock(cursor);
                steps += u32::from(*count);
            }
            other => panic!("a classic motor must only emit queue_step, got {other:?}"),
        }
    }
    assert_eq!(steps, 100);
    assert_eq!(cursor, shim.emitted_clock(0));
}

/// A high-precision motor emits only queue_step_hp frames, and walking them
/// with the encoder's first_step/last_step offsets lands the cursor exactly
/// where the shim says the run ended — across drains, so the carry-out of
/// one compress call seeds the next.
#[test]
fn hp_frames_reconstruct_to_the_shim_emitted_cursor() {
    let mut config = cfg();
    config.encoder = StepEncoder::HighPrecision;
    let mut shim = StepShim::new(vec![config], 8);
    shim.push_pieces(
        0,
        &[
            linear_piece(1_000, 0.0, 1.0, 0.01),
            linear_piece(11_000, 1.0, 2.0, 0.01),
        ],
    )
    .unwrap();

    let mut cursor = 0u64;
    let mut hp_moves = 0;
    let mut steps = 0u32;
    for frames in [shim.drain(6_000).unwrap(), shim.drain(u64::MAX).unwrap()] {
        for frame in &frames {
            match frame {
                StepFrame::ResetStepClock { clock, .. } => cursor = u64::from(*clock),
                StepFrame::SetNextStepDir { .. } => {}
                StepFrame::QueueStepHp {
                    first_step,
                    last_step,
                    count,
                    ..
                } => {
                    assert!(*first_step > 0, "steps must advance");
                    cursor += *last_step;
                    hp_moves += 1;
                    steps += u32::from(*count);
                }
                StepFrame::QueueStep { .. } => {
                    panic!("an hp motor must not emit classic queue_step")
                }
            }
        }
    }
    assert!(hp_moves > 0, "the hp encoder must produce hp moves");
    assert_eq!(steps, 200);
    assert_eq!(cursor, shim.emitted_clock(0));
}

#[test]
fn first_emission_resets_the_step_clock_then_sets_dir() {
    let mut shim = StepShim::new(vec![cfg()], 8);
    shim.push_pieces(0, &[linear_piece(1_000, 0.0, 1.0, 0.01)])
        .unwrap();
    let frames = shim.drain(u64::MAX).unwrap();

    let (reset, first_step) = reset_and_first_step(&frames);
    assert_eq!(reset + 1, first_step);
    assert!(reset >= 1_000, "reset {reset} predates the piece it opens");
    assert!(
        matches!(frames[0], StepFrame::ResetStepClock { oid: OID, .. }),
        "{:?}",
        frames[0]
    );
    assert_eq!(frames[1], StepFrame::SetNextStepDir { oid: OID, dir: 1 });
    assert!(matches!(frames[2], StepFrame::QueueStep { .. }));
    assert_eq!(queue_step_count(&frames), 100);
    assert_eq!(
        frames
            .iter()
            .filter(|f| matches!(f, StepFrame::ResetStepClock { .. }))
            .count(),
        1
    );
}

#[test]
fn second_drain_does_not_reset_the_step_clock_again() {
    let mut shim = StepShim::new(vec![cfg()], 8);
    shim.push_pieces(
        0,
        &[
            linear_piece(1_000, 0.0, 1.0, 0.01),
            linear_piece(11_000, 1.0, 2.0, 0.01),
        ],
    )
    .unwrap();

    let first = shim.drain(6_000).unwrap();
    let second = shim.drain(u64::MAX).unwrap();
    assert_eq!(
        first
            .iter()
            .filter(|f| matches!(f, StepFrame::ResetStepClock { .. }))
            .count(),
        1
    );
    assert!(
        second
            .iter()
            .all(|f| !matches!(f, StepFrame::ResetStepClock { .. }))
    );
    assert_eq!(
        queue_step_count(&first) + queue_step_count(&second),
        200,
        "every sampled step must reach the wire exactly once"
    );
}

#[test]
fn direction_reversal_emits_set_next_step_dir() {
    let mut shim = StepShim::new(vec![cfg()], 8);
    shim.push_pieces(
        0,
        &[
            linear_piece(1_000, 0.0, 1.0, 0.01),
            linear_piece(11_000, 1.0, 0.0, 0.01),
        ],
    )
    .unwrap();
    let frames = shim.drain(u64::MAX).unwrap();

    let dirs: Vec<u8> = frames
        .iter()
        .filter_map(|f| match f {
            StepFrame::SetNextStepDir { dir, .. } => Some(*dir),
            _ => None,
        })
        .collect();
    assert_eq!(dirs, vec![1, 0]);

    let reverse_at = frames
        .iter()
        .position(|f| matches!(f, StepFrame::SetNextStepDir { dir: 0, .. }))
        .unwrap();
    assert!(matches!(
        frames[reverse_at + 1],
        StepFrame::QueueStep { .. }
    ));
}

#[test]
fn ring_full_fails_loud() {
    let mut shim = StepShim::new(vec![cfg()], 2);
    let pieces = [
        linear_piece(1_000, 0.0, 1.0, 0.01),
        linear_piece(11_000, 1.0, 2.0, 0.01),
        linear_piece(21_000, 2.0, 3.0, 0.01),
    ];
    let err = shim.push_pieces(0, &pieces).unwrap_err();
    assert!(matches!(err, ShimError::RingFull { motor: 0 }));
    assert_eq!(shim.ring_depth(), 2);
}

#[test]
fn non_contiguous_piece_fails_loud() {
    let mut shim = StepShim::new(vec![cfg()], 8);
    shim.push_pieces(0, &[linear_piece(1_000, 0.0, 1.0, 0.01)])
        .unwrap();
    let err = shim
        .push_pieces(0, &[linear_piece(12_345, 1.0, 2.0, 0.01)])
        .unwrap_err();
    match err {
        ShimError::PieceGap {
            motor,
            expected,
            got,
            ..
        } => {
            assert_eq!((motor, expected, got), (0, 11_000, 12_345));
        }
        other => panic!("expected PieceGap, got {other:?}"),
    }
}

#[test]
fn a_seam_within_the_clock_domain_skew_is_accepted() {
    for offset in [-16_i64, -1, 0, 1, 16] {
        let mut shim = StepShim::new(vec![cfg()], 8);
        shim.push_pieces(0, &[linear_piece(1_000, 0.0, 1.0, 0.01)])
            .unwrap();
        let start = (11_000_i64 + offset) as u64;
        shim.push_pieces(0, &[linear_piece(start, 1.0, 2.0, 0.01)])
            .unwrap_or_else(|e| panic!("seam skew of {offset} cycles must be tolerated: {e:?}"));
    }
}

/// The bound the seam tolerance is built on: whatever the piece length and
/// whatever the clock, `end_time`'s f32 round trip never lands further from
/// the instant the producer projected than `projection_slack_cycles` allows.
#[test]
fn the_projection_slack_bounds_the_f32_round_trip() {
    let mut worst_ratio = 0.0_f64;
    for freq in [1_000_000.0_f64, 72_000_000.0, 71_999_983.66, 550_000_000.0] {
        #[allow(clippy::cast_possible_truncation)]
        let freq32 = freq as f32;
        let mut duration = 1e-4_f64;
        while duration < 32.0 {
            for step in 0..97_u32 {
                let d = duration * (1.0 + f64::from(step) / 97.0);
                #[allow(clippy::cast_possible_truncation)]
                let piece = linear_piece(869_400_000_000, 0.0, 1.0, d as f32);
                let end = piece.end_time(freq32);
                #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
                let projected = piece.start_time + (d * freq).round() as u64;
                let span = end - piece.start_time;
                let slack = super::projection_slack_cycles(span);
                let seen = end.abs_diff(projected);
                assert!(
                    seen <= slack,
                    "a {d} s piece at {freq} Hz reprojects {seen} cycles off, past the \
                     {slack}-cycle bound the seam check trusts"
                );
                worst_ratio = worst_ratio.max(seen as f64 / slack as f64);
            }
            duration *= 1.7;
        }
    }
    assert!(
        worst_ratio > 0.25,
        "the bound must stay tight enough to catch real breaks; worst observed use \
         was only {worst_ratio} of it"
    );
}

/// The tolerance is the flat producer allowance plus the piece's own f32
/// slack, and it is still a hard edge: one cycle past it fails loud.
#[test]
fn the_seam_tolerance_scales_with_the_piece_that_projected_it() {
    let long = linear_piece(1_000, 0.0, 1.0, 4.0);
    #[allow(clippy::cast_possible_truncation)]
    let end = long.end_time(CYCLES_PER_SECOND as f32);
    let tolerance =
        super::MAX_SEAM_SKEW_CYCLES + super::projection_slack_cycles(end - long.start_time);
    assert!(
        tolerance > super::MAX_SEAM_SKEW_CYCLES,
        "a 4 s piece must buy more slack than a flat tolerance gives"
    );

    for (offset, must_pass) in [(tolerance, true), (tolerance + 1, false)] {
        let mut shim = StepShim::new(vec![cfg()], 8);
        shim.push_pieces(0, &[long]).unwrap();
        let next = linear_piece(end + offset, 1.0, 2.0, 0.01);
        let pushed = shim.push_pieces(0, &[next]);
        assert_eq!(
            pushed.is_ok(),
            must_pass,
            "seam {offset} cycles out of a {tolerance}-cycle tolerance: {pushed:?}"
        );
    }
}

#[test]
fn repeated_overlapping_seams_do_not_accumulate_lost_steps() {
    let mut shim = StepShim::new(vec![cfg()], 64);
    let mut start = 1_000_u64;
    for i in 0..20 {
        let from = i as f32;
        shim.push_pieces(0, &[linear_piece(start, from, from + 1.0, 0.01)])
            .unwrap();
        start = start + 10_000 - 16;
    }
    let frames = shim.drain(u64::MAX).unwrap();
    let steps = queue_step_count(&frames);
    assert!(
        (1_999..=2_000).contains(&steps),
        "a 16-cycle overlap per seam must not lose a step per seam: {steps}"
    );
}

#[test]
fn retired_counts_are_monotonic_across_drains() {
    let mut shim = StepShim::new(vec![cfg()], 8);
    shim.push_pieces(
        0,
        &[
            linear_piece(1_000, 0.0, 1.0, 0.01),
            linear_piece(11_000, 1.0, 2.0, 0.01),
            linear_piece(21_000, 2.0, 3.0, 0.01),
        ],
    )
    .unwrap();

    let mut observed = Vec::new();
    for budget in [5_000_u64, 11_100, 21_100, u64::MAX] {
        shim.drain(budget).unwrap();
        observed.push(shim.retired_counts()[0]);
    }
    assert!(
        observed.windows(2).all(|w| w[1] >= w[0]),
        "retired regressed: {observed:?}"
    );
    assert_eq!(observed, vec![0, 1, 2, 3]);
}

#[test]
fn step_rate_cap_fails_loud_through_drain() {
    let mut config = cfg();
    config.max_steps_per_sample = 2;
    let mut shim = StepShim::new(vec![config], 8);
    shim.push_pieces(0, &[linear_piece(1_000, 0.0, 5.0, 0.01)])
        .unwrap();
    let err = shim.drain(u64::MAX).unwrap_err();
    assert!(matches!(err, ShimError::StepRateExceeded { cap: 2, .. }));
}

#[test]
fn halt_returns_executed_steps_and_frees_ring_credit() {
    let mut shim = StepShim::new(vec![cfg()], 8);
    shim.push_pieces(
        0,
        &[
            linear_piece(1_000, 0.0, 1.0, 0.01),
            linear_piece(11_000, 1.0, 2.0, 0.01),
        ],
    )
    .unwrap();
    shim.drain(u64::MAX).unwrap();

    let (executed, _) = shim.halt_at(0, u64::MAX).unwrap();
    assert_eq!(executed, 200);
    assert_eq!(shim.retired_counts(), vec![2]);
}

#[test]
fn halt_with_executed_count_uses_the_external_seed() {
    let mut shim = StepShim::new(vec![cfg()], 8);
    shim.push_pieces(0, &[linear_piece(1_000, 0.0, 1.0, 0.01)])
        .unwrap();
    shim.drain(u64::MAX).unwrap();

    let expected = shim.expected_halt_count(0, u64::MAX);
    let (derived, _) = shim
        .halt_at_with_executed(0, 20_000, 37)
        .expect("the external count can reseed a drained shim");
    assert_eq!(derived, expected);

    shim.push_pieces(0, &[linear_piece(50_000, 0.37, 0.47, 0.01)])
        .unwrap();
    shim.drain(u64::MAX).unwrap();
    assert_eq!(shim.halt_at(0, u64::MAX).unwrap().0, 47);
}

#[test]
fn halt_discards_queued_work_and_re_resets_the_step_clock() {
    let mut shim = StepShim::new(vec![cfg()], 8);
    shim.push_pieces(0, &[linear_piece(1_000, 0.0, 1.0, 0.01)])
        .unwrap();
    shim.drain(3_000).unwrap();

    let (executed, _) = shim.halt_at(0, u64::MAX).unwrap();
    assert!(executed > 0);

    shim.reset_position(0, 200);
    shim.push_pieces(0, &[linear_piece(50_000, 2.0, 3.0, 0.01)])
        .unwrap();
    let frames = shim.drain(u64::MAX).unwrap();
    let (reset, first_step) = reset_and_first_step(&frames);
    assert_eq!(reset + 1, first_step);
    assert!(
        reset >= 50_000,
        "reset {reset} predates the piece the halted stream resumed on"
    );
}

#[test]
fn reset_position_reseeds_the_step_counter() {
    let mut shim = StepShim::new(vec![cfg()], 8);
    shim.reset_position(0, -400);
    shim.push_pieces(0, &[linear_piece(1_000, -4.0, -3.0, 0.01)])
        .unwrap();
    let frames = shim.drain(u64::MAX).unwrap();
    assert_eq!(queue_step_count(&frames), 100);
    assert_eq!(shim.halt_at(0, u64::MAX).unwrap().0, -300);
}

#[test]
fn a_cut_inside_a_piece_does_not_replay_steps_before_the_cut() {
    let mut shim = StepShim::new(vec![cfg()], 8);
    shim.push_pieces(0, &[linear_piece(1_000, 0.0, 0.1, 0.010)])
        .unwrap();
    let first = shim.drain(6_000).unwrap();
    let before = queue_step_count(&first);
    assert!(before > 0, "the first drain must emit steps to cut inside");

    let cut_at = 6_000;
    shim.halt_at(0, cut_at).unwrap();
    shim.set_motor_cycles_per_second(0, CYCLES_PER_SECOND * 1.004);
    shim.push_pieces(0, &[linear_piece(1_000, 0.0, 0.1, 0.010)])
        .unwrap();
    let after = shim.drain(11_000).unwrap();

    let mut cursor = 0u64;
    for frame in &after {
        match frame {
            StepFrame::ResetStepClock { clock, .. } => cursor = u64::from(*clock),
            StepFrame::SetNextStepDir { .. } => {}
            StepFrame::QueueStep {
                interval,
                count,
                add,
                ..
            } => {
                let mv = crate::compress::StepMove {
                    interval: *interval,
                    count: *count,
                    add: *add,
                };
                for nth in 1..=*count {
                    assert!(
                        mv.step_clock(cursor, nth) > cut_at,
                        "a step before the cut clock was replayed after the cut"
                    );
                }
                cursor = mv.last_clock(cursor);
            }
            StepFrame::QueueStepHp { .. } => {
                panic!("the classic cut replay cannot walk an hp frame")
            }
        }
    }
}

#[test]
fn re_arming_after_a_cut_emits_the_catch_up_delta_exactly_once() {
    let mut shim = StepShim::new(vec![cfg()], 8);
    shim.push_pieces(0, &[linear_piece(1_000, 0.0, 0.1, 0.010)])
        .unwrap();
    let emitted_before = queue_step_count(&shim.drain(6_000).unwrap());

    let (executed, _) = shim.halt_at(0, 6_000).unwrap();
    assert_eq!(
        executed, emitted_before as i64,
        "halt must report exactly the steps already emitted"
    );

    shim.push_pieces(0, &[linear_piece(6_000, 0.05, 0.1, 0.005)])
        .unwrap();
    let after = queue_step_count(&shim.drain(20_000).unwrap());

    let total = emitted_before + after;
    let expected = (0.1_f32 / 0.01) as u32;
    assert_eq!(
        total, expected,
        "a cut must neither lose nor duplicate motion: {emitted_before} + {after}"
    );
}

const IDLE_CYCLES_PER_SECOND: f64 = 72_000_000.0;

fn idle_cfg() -> MotorConfig {
    MotorConfig {
        oid: OID,
        microstep_distance: 0.01,
        invert_dir: false,
        max_steps_per_sample: 16,
        sample_rate_hz: 1_000.0,
        cycles_per_second: IDLE_CYCLES_PER_SECOND,
        min_rearm_cycles: 0,
        encoder: StepEncoder::Classic {
            max_error_ticks: super::compress::DEFAULT_MAX_ERROR_TICKS,
        },
    }
}

fn reset_clocks(frames: &[StepFrame]) -> Vec<u64> {
    frames
        .iter()
        .filter_map(|f| match *f {
            StepFrame::ResetStepClock { clock, .. } => Some(u64::from(clock)),
            _ => None,
        })
        .collect()
}

/// The step clocks the mcu will execute, walked exactly the way the mcu
/// stepper walks them: an anchor from `reset_step_clock`, then every
/// `queue_step` interval accumulated onto it.
fn replayed_step_clocks(frames: &[StepFrame]) -> Vec<u64> {
    let mut cursor = 0u64;
    let mut clocks = Vec::new();
    for frame in frames {
        match *frame {
            StepFrame::ResetStepClock { clock, .. } => cursor = u64::from(clock),
            StepFrame::SetNextStepDir { .. } => {}
            StepFrame::QueueStep {
                interval,
                count,
                add,
                ..
            } => {
                let mv = StepMove {
                    interval,
                    count,
                    add,
                };
                clocks.extend((1..=count).map(|nth| mv.step_clock(cursor, nth)));
                cursor = mv.last_clock(cursor);
            }
            StepFrame::QueueStepHp { .. } => {
                panic!("the classic clock replay cannot walk an hp frame")
            }
        }
    }
    clocks
}

/// A print-shaped lane: it steps, holds while the other axes print, then
/// steps again in the same direction.
fn drain_across_hold(hold_secs: f32) -> (StepShim, Result<Vec<StepFrame>, ShimError>) {
    #[allow(clippy::cast_possible_truncation)]
    let cps = IDLE_CYCLES_PER_SECOND as f32;
    let mut shim = StepShim::new(vec![idle_cfg()], 8);
    let lift = linear_piece(72_000, 0.0, 1.0, 0.05);
    let hold = linear_piece(lift.end_time(cps), 1.0, 1.0, hold_secs);
    let resume = linear_piece(hold.end_time(cps), 1.0, 2.0, 0.05);
    let end = resume.end_time(cps);
    shim.push_pieces(0, &[lift, hold, resume]).unwrap();

    let frames = shim.drain(hold.start_time).and_then(|mut frames| {
        frames.extend(shim.drain(end)?);
        Ok(frames)
    });
    (shim, frames)
}

#[test]
fn a_hold_inside_the_encoder_window_keeps_the_original_anchor() {
    let (shim, frames) = drain_across_hold(11.0);
    let frames = frames.expect("an 11 s hold is 792 Mticks, inside the 805 Mtick window");

    assert_eq!(reset_clocks(&frames).len(), 1);
    assert_eq!(queue_step_count(&frames), 200);
    assert_eq!(shim.commanded_steps(0), 200);
}

/// A lane parked past the encoder's reach cannot be encoded from its old
/// anchor. Re-anchoring it is time-only: the direction the mcu holds and the
/// step counter both survive, and every step still lands where it was
/// sampled.
#[test]
fn a_hold_past_the_encoder_window_re_anchors_the_step_clock() {
    let (shim, frames) = drain_across_hold(12.0);
    let frames = frames.expect("a hold past the window must re-anchor, not fail");

    let resets = reset_clocks(&frames);
    assert_eq!(resets.len(), 2, "the parked lane must be re-anchored once");
    assert_eq!(queue_step_count(&frames), 200);
    assert_eq!(shim.commanded_steps(0), 200);
    assert_eq!(
        frames
            .iter()
            .filter(|f| matches!(f, StepFrame::SetNextStepDir { .. }))
            .count(),
        1,
        "re-anchoring is time-only: the mcu's direction latch is untouched"
    );

    let clocks = replayed_step_clocks(&frames);
    assert_eq!(clocks.len(), 200);
    assert!(
        clocks.windows(2).all(|w| w[0] < w[1]),
        "the re-anchored stream must stay monotonic: {:?}",
        &clocks[98..102]
    );
    let resume_start = resets[1];
    assert!(
        clocks[100] > resume_start && clocks[100] - resume_start < 72_000,
        "the first step after the re-anchor must land where it was sampled, \
         not at the anchor: anchor {resume_start}, step {}",
        clocks[100]
    );
    assert!(
        clocks[199] - clocks[0] > 12 * 72_000_000,
        "the 12 s hold must survive the re-anchor as real elapsed time"
    );
}

/// A trip reconciles the mcu's executed step count against the host's own
/// count. Re-anchoring the clock must leave that count alone.
#[test]
fn a_trip_after_a_re_anchor_still_reports_every_executed_step() {
    let (mut shim, frames) = drain_across_hold(12.0);
    frames.expect("a hold past the window must re-anchor, not fail");

    let (executed, tail) = shim.halt_at(0, u64::MAX).unwrap();
    assert_eq!(executed, 200);
    assert!(
        tail.is_empty(),
        "every sampled step was already on the wire: {tail:?}"
    );
}
