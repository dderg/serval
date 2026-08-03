use super::*;

fn reconstruct(moves: &[StepMove], last_step_clock: u64) -> Vec<u64> {
    let mut out = Vec::new();
    let mut lsc = last_step_clock;
    for m in moves {
        for n in 1..=m.count {
            out.push(m.step_clock(lsc, n));
        }
        lsc = m.last_clock(lsc);
    }
    out
}

fn assert_within_band(steps: &[u64], last_step_clock: u64, max_error: u32, moves: &[StepMove]) {
    let got = reconstruct(moves, last_step_clock);
    assert_eq!(got.len(), steps.len(), "step count mismatch");
    let mut lsc = last_step_clock;
    let mut i = 0usize;
    let mut prev_emitted = last_step_clock;
    for m in moves {
        for n in 1..=m.count {
            let want = steps[i];
            let have = got[i];
            assert!(
                have <= want,
                "step {i}: reconstructed {have} is later than requested {want}"
            );
            assert!(
                have > prev_emitted,
                "step {i}: reconstructed {have} not after previous {prev_emitted}"
            );
            let prevpoint = if n == 1 { lsc } else { steps[i - 1] };
            let band = u64::from(max_error).min((want - prevpoint) / 2);
            assert!(
                want - have <= band,
                "step {i}: reconstructed {have} is {} ticks before requested {want}, band {band}",
                want - have
            );
            prev_emitted = have;
            i += 1;
        }
        lsc = m.last_clock(lsc);
    }
}

fn ramp(start_interval: i64, add: i64, count: usize, base: u64) -> Vec<u64> {
    let mut clock = base;
    let mut interval = start_interval;
    let mut out = Vec::with_capacity(count);
    for _ in 0..count {
        clock += interval as u64;
        out.push(clock);
        interval += add;
    }
    out
}

#[test]
fn constant_interval_becomes_single_move() {
    let steps = ramp(1000, 0, 500, 0);
    let (moves, consumed) = compress(&steps, 0).unwrap();
    assert_eq!(consumed, steps.len());
    assert_eq!(moves.len(), 1);
    assert_eq!(
        moves[0],
        StepMove {
            interval: 1000,
            count: 500,
            add: 0
        }
    );
    assert_within_band(&steps, 0, DEFAULT_MAX_ERROR_TICKS, &moves);
}

#[test]
fn constant_interval_offset_from_last_step_clock() {
    let base = 7_000_000u64;
    let steps = ramp(250, 0, 64, base);
    let (moves, consumed) = compress(&steps, base).unwrap();
    assert_eq!(consumed, steps.len());
    assert_eq!(moves.len(), 1);
    assert_eq!(moves[0].interval, 250);
    assert_eq!(moves[0].count, 64);
    assert_eq!(moves[0].add, 0);
    assert_within_band(&steps, base, DEFAULT_MAX_ERROR_TICKS, &moves);
}

#[test]
fn linear_ramp_uses_add() {
    let steps = ramp(4000, -7, 400, 0);
    let (moves, consumed) = compress(&steps, 0).unwrap();
    assert_eq!(consumed, steps.len());
    assert!(
        moves.iter().any(|m| m.add != 0),
        "expected an add term, got {moves:?}"
    );
    assert!(moves.len() < 8, "ramp compressed poorly: {moves:?}");
    assert_within_band(&steps, 0, DEFAULT_MAX_ERROR_TICKS, &moves);
}

#[test]
fn acceleration_ramp_reconstructs_within_band() {
    let steps = ramp(200, 3, 900, 12_345);
    let (moves, consumed) = compress(&steps, 12_345).unwrap();
    assert_eq!(consumed, steps.len());
    assert_within_band(&steps, 12_345, DEFAULT_MAX_ERROR_TICKS, &moves);
}

#[test]
fn quadratic_profile_reconstructs_within_band() {
    let mut steps = Vec::new();
    let mut clock = 0u64;
    for i in 0..2000u64 {
        let interval = 300 + (i * i) / 900;
        clock += interval;
        steps.push(clock);
    }
    let (moves, consumed) = compress(&steps, 0).unwrap();
    assert_eq!(consumed, steps.len());
    assert_within_band(&steps, 0, DEFAULT_MAX_ERROR_TICKS, &moves);
}

#[test]
fn jittered_steps_reconstruct_within_band() {
    let mut steps = Vec::new();
    let mut clock = 500u64;
    let mut jitter = 1u64;
    for _ in 0..1500 {
        jitter = (jitter * 1103515245 + 12345) % 2048;
        clock += 900 + jitter;
        steps.push(clock);
    }
    let (moves, consumed) = compress(&steps, 0).unwrap();
    assert_eq!(consumed, steps.len());
    assert_within_band(&steps, 0, DEFAULT_MAX_ERROR_TICKS, &moves);
}

#[test]
fn tight_max_error_still_reconstructs_within_band() {
    let steps = ramp(1200, -2, 600, 0);
    let (moves, consumed) = compress_with_max_error(&steps, 0, 32).unwrap();
    assert_eq!(consumed, steps.len());
    assert_within_band(&steps, 0, 32, &moves);
}

#[test]
fn single_step_is_one_move() {
    let (moves, consumed) = compress(&[900], 400).unwrap();
    assert_eq!(consumed, 1);
    assert_eq!(
        moves,
        vec![StepMove {
            interval: 500,
            count: 1,
            add: 0
        }]
    );
}

#[test]
fn empty_input_yields_nothing() {
    let (moves, consumed) = compress(&[], 0).unwrap();
    assert!(moves.is_empty());
    assert_eq!(consumed, 0);
}

#[test]
fn move_count_capped_at_u16_max() {
    let steps = ramp(4, 0, 70_000, 0);
    let (moves, consumed) = compress(&steps, 0).unwrap();
    assert_eq!(consumed, steps.len());
    assert!(moves.len() >= 2);
    assert!(moves.iter().all(|m| m.count >= 1));
    assert_within_band(&steps, 0, DEFAULT_MAX_ERROR_TICKS, &moves);
}

#[test]
fn non_monotonic_input_errors() {
    let err = compress(&[1000, 2000, 1999, 3000], 0).unwrap_err();
    assert!(err.detail.contains("not after previous"), "{}", err.detail);
}

#[test]
fn duplicate_step_clock_errors() {
    let err = compress(&[1000, 1000], 0).unwrap_err();
    assert!(err.detail.contains("not after previous"), "{}", err.detail);
}

#[test]
fn step_at_or_before_last_step_clock_errors() {
    assert!(compress(&[1000, 2000], 1000).is_err());
    assert!(compress(&[900], 1000).is_err());
}

#[test]
fn unrepresentable_first_interval_errors() {
    let far = (3u64 << 28) + 1;
    let err = compress(&[far], 0).unwrap_err();
    assert!(err.detail.contains("not representable"), "{}", err.detail);
}

#[test]
fn far_future_gap_stops_after_consuming_prefix() {
    let mut steps = ramp(1000, 0, 10, 0);
    let last = *steps.last().unwrap();
    steps.push(last + (3 << 28));
    let (moves, consumed) = compress(&steps, 0).unwrap();
    assert_eq!(consumed, 10);
    let reconstructed = reconstruct(&moves, 0);
    assert_eq!(reconstructed.len(), 10);
    assert_within_band(&steps[..10], 0, DEFAULT_MAX_ERROR_TICKS, &moves);
}

#[test]
fn step_clock_matches_recurrence() {
    let m = StepMove {
        interval: 1000,
        count: 5,
        add: -10,
    };
    let mut clock = 400u64;
    let mut interval = 1000i64;
    for n in 1..=m.count {
        clock += interval as u64;
        assert_eq!(m.step_clock(400, n), clock);
        interval += i64::from(m.add);
    }
    assert_eq!(m.last_clock(400), clock);
}
