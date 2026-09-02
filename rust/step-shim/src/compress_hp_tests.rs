use super::*;

fn reference_walk(m: StepMoveHp) -> Vec<u64> {
    let (mut interval, mut add, add2, shift, mut low) = if m.shift <= 0 {
        let scale = 1_i64 << (-m.shift as u32);
        (
            i64::from(m.interval) * scale,
            i64::from(m.add) * scale,
            i64::from(m.add2) * scale,
            0_u8,
            0_i64,
        )
    } else {
        let extra_shift = if m.shift > 8 {
            (16 - m.shift) as u32
        } else {
            (8 - m.shift) as u32
        };
        let shift = if m.shift > 8 { 16 } else { 8 };
        let scale = 1_i64 << extra_shift;
        (
            i64::from(m.interval) * scale,
            i64::from(m.add) * scale,
            i64::from(m.add2) * scale,
            shift,
            1_i64 << (shift - 1),
        )
    };
    let mut time = 0_i64;
    let mut out = Vec::with_capacity(m.count as usize);
    for _ in 0..m.count {
        let sum = interval + low;
        time += if shift == 0 { sum } else { sum >> shift };
        if shift != 0 {
            low = sum & ((1_i64 << shift) - 1);
        }
        out.push(time as u64);
        interval += add;
        add += add2;
    }
    out
}

fn reconstruct(moves: &[StepMoveHp], last_step_clock: u64) -> Vec<u64> {
    let mut out = Vec::new();
    let mut cursor = last_step_clock;
    for m in moves {
        let offsets = mcu_walk_offsets(m).expect("compressor emitted a valid move");
        out.extend(offsets.iter().map(|offset| cursor + offset));
        cursor += m.last_step;
    }
    out
}

fn assert_within_windows(steps: &[u64], last_step_clock: u64, moves: &[StepMoveHp]) {
    let got = reconstruct(moves, last_step_clock);
    assert_eq!(got.len(), steps.len(), "reconstructed step count differs");
    assert!(
        got.last() <= steps.last(),
        "terminal encoded step must not pass the final requested clock"
    );
    let mut cursor = last_step_clock;
    let mut input_pos = 0usize;
    for m in moves {
        let offsets = mcu_walk_offsets(m).unwrap();
        assert_eq!(m.first_step, offsets[0]);
        assert_eq!(m.last_step, *offsets.last().unwrap());
        for (offset, &actual) in offsets.iter().enumerate() {
            let index = input_pos + offset;
            let point = minmax_point(steps, index, input_pos, cursor);
            let actual = actual as i64;
            assert!(
                actual >= point.minp && actual <= point.maxp,
                "step {index}: reconstructed offset {actual}, requested offset {}, window {}:{}",
                steps[index] - cursor,
                point.minp,
                point.maxp
            );
        }
        cursor += m.last_step;
        input_pos += usize::from(m.count);
    }
}

fn constant_interval(interval: u64, count: usize, base: u64) -> Vec<u64> {
    (1..=count)
        .map(|index| base + interval * index as u64)
        .collect()
}

#[test]
fn mcu_walk_matches_reference_fixed_point_emulator() {
    let moves = [
        StepMoveHp {
            interval: 17_321,
            count: 9,
            add: -37,
            add2: 3,
            shift: 0,
            first_step: 0,
            last_step: 0,
        },
        StepMoveHp {
            interval: 1_003,
            count: 11,
            add: 23,
            add2: -2,
            shift: 5,
            first_step: 0,
            last_step: 0,
        },
        StepMoveHp {
            interval: 98,
            count: 7,
            add: -4,
            add2: 1,
            shift: -3,
            first_step: 0,
            last_step: 0,
        },
        StepMoveHp {
            interval: 12_345,
            count: 8,
            add: -12,
            add2: 2,
            shift: 12,
            first_step: 0,
            last_step: 0,
        },
    ];
    for m in moves {
        assert_eq!(mcu_walk_offsets(&m).unwrap(), reference_walk(m));
    }
}

#[test]
fn constant_velocity_compresses_many_steps_per_move() {
    let steps = constant_interval(5_000, 1_000, 0);
    let (moves, covered, carry) = compress_hp(&mut HpScratch::new(), &steps, 0, 0).unwrap();
    assert_eq!(covered, steps.len());
    assert_eq!(carry, 0);
    assert!(
        moves.len() <= 4,
        "expected a few moves, got {}",
        moves.len()
    );
    assert!(moves.iter().any(|m| m.count >= 256));
    assert_within_windows(&steps, 0, &moves);
}

#[test]
fn accelerating_ramp_uses_quadratic_wire_parameters() {
    let mut steps = Vec::with_capacity(2_000);
    let mut clock = 0_u64;
    for index in 0..2_000_u64 {
        let interval = 20_000 - (19_800 * index / 1_999);
        clock += interval;
        steps.push(clock);
    }
    let (moves, covered, _) = compress_hp(&mut HpScratch::new(), &steps, 0, 0).unwrap();
    assert_eq!(covered, steps.len());
    assert!(
        moves.iter().any(|m| m.add2 != 0 || m.shift > 0),
        "{moves:?}"
    );
    assert_within_windows(&steps, 0, &moves);
}

#[test]
fn jerk_profile_stays_inside_every_local_window() {
    let mut steps = Vec::with_capacity(2_400);
    let mut clock = 0_u64;
    for index in 0..2_400_u64 {
        let t = index as i64 - 1_200;
        let interval = 1_200 + (t * t * t / 2_000_000).unsigned_abs();
        clock += interval.max(200);
        steps.push(clock);
    }
    let (moves, covered, _) = compress_hp(&mut HpScratch::new(), &steps, 0, 0).unwrap();
    assert_eq!(covered, steps.len());
    assert_within_windows(&steps, 0, &moves);
}

#[test]
fn next_expected_interval_preserves_batch_junction_window() {
    let first = constant_interval(1_000, 700, 10_000);
    let (first_moves, first_covered, carry) =
        compress_hp(&mut HpScratch::new(), &first, 10_000, 1_000).unwrap();
    assert_eq!(first_covered, first.len());
    let first_end = reconstruct(&first_moves, 10_000).last().copied().unwrap();

    let second = constant_interval(1_001, 700, first.last().copied().unwrap());

    let (second_moves, second_covered, _) =
        compress_hp(&mut HpScratch::new(), &second, first_end, carry).unwrap();
    assert_eq!(second_covered, second.len());
    assert_within_windows(&second, first_end, &second_moves);
}
#[test]
fn terminal_error_window_never_extends_past_the_requested_clock() {
    let steps = [100, 200];
    let terminal = minmax_point(&steps, 1, 0, 0);
    assert_eq!(terminal.maxp, 200);
}

/// Steps a tick or two apart leave no room for the three-tick error floor: a
/// window reaching back past the previous step let the encoder emit a move
/// whose first step fires on the clock the mcu had already stepped on.
#[test]
fn crowded_steps_never_decode_onto_the_same_clock() {
    let steps = [1_u64, 2, 3, 43, 44, 45, 46, 47];
    let (moves, covered, _) = compress_hp(&mut HpScratch::new(), &steps, 0, 2).unwrap();

    assert_eq!(covered, steps.len());
    let clocks = reconstruct(&moves, 0);
    let mut previous = 0_u64;
    for (index, &clock) in clocks.iter().enumerate() {
        assert!(
            clock > previous,
            "decoded step {index} at {clock} does not advance past {previous}: {moves:?}"
        );
        previous = clock;
    }
    assert_within_windows(&steps, 0, &moves);
}

#[test]
fn a_one_tick_gap_leaves_no_error_allowance_at_all() {
    let steps = [10_u64, 11, 12];
    let crowded = minmax_point(&steps, 1, 0, 0);
    assert_eq!((crowded.minp, crowded.maxp), (11, 11));

    let roomy = [10_u64, 210, 410];
    let spaced = minmax_point(&roomy, 1, 0, 0);
    assert_eq!((spaced.minp, spaced.maxp), (207, 213));
}

#[test]
fn terminal_step_never_crosses_an_unseen_direction_boundary() {
    let mut scratch = HpScratch::new();
    for initial_interval in 70_u64..300 {
        for delta in -2_i64..=2 {
            let mut clock = 10_000_u64;
            let mut steps = Vec::new();
            for index in 0..32_i64 {
                let interval = (initial_interval as i64 + delta * index).max(1) as u64;
                clock += interval;
                steps.push(clock);
            }
            let Ok((moves, covered, _)) = compress_hp(&mut scratch, &steps, 10_000, 0) else {
                continue;
            };
            assert_eq!(covered, steps.len());
            let encoded_end = reconstruct(&moves, 10_000).last().copied().unwrap();
            assert!(
                encoded_end <= clock,
                "initial_interval={initial_interval} delta={delta}: encoded {encoded_end}, requested {clock}"
            );
        }
    }
}

#[test]
fn degenerate_inputs_are_explicit() {
    let (single, covered, _) = compress_hp(&mut HpScratch::new(), &[900], 400, 0).unwrap();
    assert_eq!(covered, 1);
    assert_eq!(single.len(), 1);
    assert_eq!(single[0].count, 1);

    let (two, covered, _) = compress_hp(&mut HpScratch::new(), &[1_000, 2_000], 0, 0).unwrap();
    assert_eq!(covered, 2);
    assert_within_windows(&[1_000, 2_000], 0, &two);

    let error = compress_hp(&mut HpScratch::new(), &[], 0, 0).unwrap_err();
    assert!(error.detail.contains("empty input"));
}

/// The compressor's least-squares and error-window scratch is owned by the
/// caller and reused across runs; a reused buffer must encode exactly what a
/// freshly allocated one does.
#[test]
fn a_reused_scratch_encodes_the_same_wire_as_a_fresh_one() {
    let runs = [
        constant_interval(400, 1_000, 0),
        constant_interval(37, 91, 5_000),
        (0..600).map(|i| 10_000 + i * i + 7 * i).collect::<Vec<_>>(),
    ];
    let mut scratch = HpScratch::new();
    for _ in 0..3 {
        for steps in &runs {
            let anchor = steps[0] - 1;
            let reused = compress_hp(&mut scratch, steps, anchor, 0).unwrap();
            let fresh = compress_hp(&mut HpScratch::new(), steps, anchor, 0).unwrap();
            assert_eq!(reused, fresh);
        }
    }
}
