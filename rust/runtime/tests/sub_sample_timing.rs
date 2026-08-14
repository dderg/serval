#![allow(
    clippy::ref_as_ptr,
    clippy::float_cmp,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::too_many_lines,
    clippy::uninlined_format_args,
    clippy::doc_markdown
)]

use runtime::sub_sample_timing::{
    MAX_STEPS_PER_SAMPLE, StepTimeInputs, StepTimingResult, compute_step_times,
};

const CYCLES_PER_SEC: f32 = 520_000_000.0;
const SAMPLE_PERIOD_SEC: f32 = 25e-6;
const SAMPLE_PERIOD_CYCLES: u32 = 13_000;

const _: () = assert!(MAX_STEPS_PER_SAMPLE >= 16);

#[test]
fn step_times_in_sample_for_constant_velocity() {
    let inputs = StepTimeInputs {
        p_start: 0.0,
        p_end: 1.0,
        prev_step_count: 0,
        target_step_count: 4,
        microstep_distance: 0.25,
        sample_period_sec: SAMPLE_PERIOD_SEC,
        sample_start_cycles: 0,
        cycles_per_second: CYCLES_PER_SEC,
        displacement_threshold: 1e-3,
    };

    let result = compute_step_times(&inputs);
    let times = match result {
        StepTimingResult::SecantSlope(v) => v,
        other => panic!("expected SecantSlope, got {other:?}"),
    };

    assert_eq!(times.len(), 4, "expected exactly 4 step times");

    for k in 0..4u32 {
        let half_step_threshold_cycles =
            ((2 * k + 1) as u64 * SAMPLE_PERIOD_CYCLES as u64 / 8u64) as u32;
        let expected = half_step_threshold_cycles;
        let got = times[k as usize];
        let drift = if got > expected {
            got - expected
        } else {
            expected - got
        };
        assert!(
            drift < 10,
            "step {k}: drift {drift} cycles >= 10 (got {got}, expected {expected})"
        );
    }
}

#[test]
fn step_times_within_sample_for_decelerating() {
    let inputs = StepTimeInputs {
        p_start: 0.0,
        p_end: 1.0,
        prev_step_count: 0,
        target_step_count: 4,
        microstep_distance: 0.25,
        sample_period_sec: SAMPLE_PERIOD_SEC,
        sample_start_cycles: 0,
        cycles_per_second: CYCLES_PER_SEC,
        displacement_threshold: 1e-3,
    };

    let result = compute_step_times(&inputs);
    let times = match result {
        StepTimingResult::SecantSlope(v) => v,
        other => panic!("expected SecantSlope, got {other:?}"),
    };

    assert_eq!(times.len(), 4);
    for (k, &t) in times.iter().enumerate() {
        assert!(
            t <= SAMPLE_PERIOD_CYCLES,
            "step {k} time {t} exceeds sample period {SAMPLE_PERIOD_CYCLES}"
        );
    }
}

#[test]
fn falls_back_to_uniform_when_displacement_too_small() {
    let inputs = StepTimeInputs {
        p_start: 0.0,
        p_end: 1e-4,
        prev_step_count: 0,
        target_step_count: 3,
        microstep_distance: 1e-4 / 3.0,
        sample_period_sec: SAMPLE_PERIOD_SEC,
        sample_start_cycles: 0,
        cycles_per_second: CYCLES_PER_SEC,
        displacement_threshold: 1e-3,
    };

    let result = compute_step_times(&inputs);
    let times = match result {
        StepTimingResult::Uniform(v) => v,
        other => panic!("expected Uniform, got {other:?}"),
    };

    assert_eq!(times.len(), 3);
    let n = 3u64;
    for k in 0..n {
        let expected = (SAMPLE_PERIOD_CYCLES as u64 * (k + 1) / (n + 1)) as u32;
        assert_eq!(
            times[k as usize], expected,
            "uniform step {k}: expected {expected}, got {}",
            times[k as usize]
        );
    }
}

#[test]
fn large_absolute_step_counts_keep_distinct_clocks() {
    let inputs = StepTimeInputs {
        p_start: 5_792.073,
        p_end: 5_792.074_7,
        prev_step_count: 8_388_609,
        target_step_count: 8_388_611,
        microstep_distance: 0.000_690_468_75,
        sample_period_sec: 0.000_1,
        sample_start_cycles: 2_766_266_336,
        cycles_per_second: 64_000_000.0,
        displacement_threshold: 1e-5,
    };

    let times = match compute_step_times(&inputs) {
        StepTimingResult::SecantSlope(times) => times,
        other => panic!("expected SecantSlope, got {other:?}"),
    };

    assert_eq!(times.len(), 2);
    assert!(times[0] < times[1], "step clocks must increase: {times:?}");
}
