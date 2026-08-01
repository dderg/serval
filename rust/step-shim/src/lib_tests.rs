use super::{MotorConfig, ShimError, StepFrame, StepShim};
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

#[test]
fn first_emission_resets_the_step_clock_then_sets_dir() {
    let mut shim = StepShim::new(vec![cfg()], 8);
    shim.push_pieces(0, &[linear_piece(1_000, 0.0, 1.0, 0.01)])
        .unwrap();
    let frames = shim.drain(u64::MAX).unwrap();

    assert_eq!(
        frames[0],
        StepFrame::ResetStepClock {
            oid: OID,
            clock: 1_000
        }
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
    assert_eq!(
        frames[0],
        StepFrame::ResetStepClock {
            oid: OID,
            clock: 50_000
        }
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
