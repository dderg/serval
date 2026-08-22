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
    MAX_STEPS_PER_SAMPLE, QuadStepTimeInputs, StepTimeInputs, StepTimingResult, compute_step_times,
    compute_step_times_quadratic,
};

const CYCLES_PER_SEC: f32 = 520_000_000.0;
const SAMPLE_PERIOD_SEC: f32 = 25e-6;
const SAMPLE_PERIOD_CYCLES: u32 = 13_000;

const _: () = assert!(MAX_STEPS_PER_SAMPLE >= 16);

fn quad_inputs(
    p_start: f32,
    p_end: f32,
    v_start: f32,
    v_end: f32,
    step_delta: i32,
    microstep_distance: f32,
) -> QuadStepTimeInputs {
    QuadStepTimeInputs {
        p_start,
        p_end,
        v_start,
        v_end,
        step_delta,
        microstep_distance,
        sample_period_sec: SAMPLE_PERIOD_SEC,
        sample_start_cycles: 0,
        cycles_per_second: CYCLES_PER_SEC,
        displacement_threshold: 1e-3,
    }
}

fn classic_inputs(
    p_start: f32,
    p_end: f32,
    step_delta: i32,
    microstep_distance: f32,
) -> StepTimeInputs {
    StepTimeInputs {
        p_start,
        p_end,
        step_delta,
        microstep_distance,
        sample_period_sec: SAMPLE_PERIOD_SEC,
        sample_start_cycles: 0,
        cycles_per_second: CYCLES_PER_SEC,
        displacement_threshold: 1e-3,
    }
}

/// Analytic crossing time of the quadratic model
/// 0.5·accel·t² + v_start·t − target = 0 in the numerically stable form.
fn expected_crossing_time(v_start: f64, accel: f64, target: f64) -> f64 {
    let disc = v_start * v_start + 2.0 * accel * target;
    2.0 * target / (v_start + disc.sqrt().copysign(v_start))
}

#[test]
fn zero_accel_matches_secant_exactly() {
    let v_start = 40_000.0;
    let v_end = v_start;
    let p_end = 0.5 * (v_start + v_end) * SAMPLE_PERIOD_SEC;
    let microstep_distance = 0.2;

    let quad = quad_inputs(0.0, p_end, v_start, v_end, 5, microstep_distance);
    let classic = classic_inputs(0.0, p_end, 5, microstep_distance);

    let quad_times = match compute_step_times_quadratic(&quad) {
        StepTimingResult::SecantSlope(t) => t,
        other => panic!("expected SecantSlope, got {other:?}"),
    };
    let classic_times = match compute_step_times(&classic) {
        StepTimingResult::SecantSlope(t) => t,
        other => panic!("expected SecantSlope, got {other:?}"),
    };

    assert_eq!(quad_times.len(), 5);
    assert_eq!(quad_times, classic_times);
}

#[test]
fn strong_accel_matches_analytic_crossing_times() {
    let v_start = 10_000.0;
    let v_end = 400_000.0;
    let p_end = 0.5 * (v_start + v_end) * SAMPLE_PERIOD_SEC;
    let quad = quad_inputs(0.0, p_end, v_start, v_end, 3, 1.0);

    let times = match compute_step_times_quadratic(&quad) {
        StepTimingResult::SecantSlope(t) => t,
        other => panic!("expected SecantSlope, got {other:?}"),
    };

    assert_eq!(times.len(), 3);
    let accel = (f64::from(v_end) - f64::from(v_start)) / f64::from(SAMPLE_PERIOD_SEC);
    let tolerance = 1e-3 * f64::from(SAMPLE_PERIOD_SEC);
    for (k, &t) in times.iter().enumerate() {
        let target = k as f64 + 0.5;
        let expected = expected_crossing_time(f64::from(v_start), accel, target);
        let got = f64::from(t) / f64::from(CYCLES_PER_SEC);
        let drift = (got - expected).abs();
        assert!(
            drift < tolerance,
            "step {k}: drift {drift} s >= tolerance {tolerance} s (got {got}, expected {expected})"
        );
        assert!(
            got < f64::from(SAMPLE_PERIOD_SEC),
            "step {k}: time {got} s exceeds sample period"
        );
    }
    for pair in times.windows(2) {
        assert!(pair[0] < pair[1], "non-increasing step times: {times:?}");
    }
}

#[test]
fn deceleration_to_near_zero_keeps_roots_ordered() {
    let v_start = 10_000.0;
    let v_end = 1.0;
    let p_end = 0.5 * (v_start + v_end) * SAMPLE_PERIOD_SEC;
    let quad = quad_inputs(0.0, p_end, v_start, v_end, 5, 0.02);

    let times = match compute_step_times_quadratic(&quad) {
        StepTimingResult::SecantSlope(t) => t,
        other => panic!("expected SecantSlope, got {other:?}"),
    };

    assert_eq!(times.len(), 5);
    for (k, &t) in times.iter().enumerate() {
        assert!(
            t > 0 && t <= SAMPLE_PERIOD_CYCLES,
            "step {k} time {t} outside (0, {SAMPLE_PERIOD_CYCLES}]"
        );
    }
    for pair in times.windows(2) {
        assert!(pair[0] < pair[1], "non-increasing step times: {times:?}");
    }
}

#[test]
fn small_displacement_falls_back_to_uniform_like_secant() {
    let microstep_distance = 1e-4 / 3.0;
    let v_start = 4.0;
    let v_end = v_start;
    let p_end = 0.5 * (v_start + v_end) * SAMPLE_PERIOD_SEC;

    let quad = quad_inputs(0.0, p_end, v_start, v_end, 3, microstep_distance);
    let classic = classic_inputs(0.0, p_end, 3, microstep_distance);

    let quad_times = match compute_step_times_quadratic(&quad) {
        StepTimingResult::Uniform(t) => t,
        other => panic!("expected Uniform, got {other:?}"),
    };
    let classic_times = match compute_step_times(&classic) {
        StepTimingResult::Uniform(t) => t,
        other => panic!("expected Uniform, got {other:?}"),
    };

    assert_eq!(quad_times.len(), 3);
    assert_eq!(quad_times, classic_times);
}

#[test]
fn reverse_motion_matches_forward_times() {
    let v_start = -10_000.0;
    let v_end = -400_000.0;
    let p_end = 0.5 * (v_start + v_end) * SAMPLE_PERIOD_SEC;
    let quad = quad_inputs(0.0, p_end, v_start, v_end, -3, 1.0);

    let times = match compute_step_times_quadratic(&quad) {
        StepTimingResult::SecantSlope(t) => t,
        other => panic!("expected SecantSlope, got {other:?}"),
    };

    assert_eq!(times.len(), 3);
    let accel_magnitude = (400_000.0 - 10_000.0) / f64::from(SAMPLE_PERIOD_SEC);
    let tolerance = 1e-3 * f64::from(SAMPLE_PERIOD_SEC);
    for (k, &t) in times.iter().enumerate() {
        let target = k as f64 + 0.5;
        let expected = expected_crossing_time(10_000.0, accel_magnitude, target);
        let got = f64::from(t) / f64::from(CYCLES_PER_SEC);
        let drift = (got - expected).abs();
        assert!(
            drift < tolerance,
            "step {k}: drift {drift} s >= tolerance {tolerance} s (got {got}, expected {expected})"
        );
        assert!(
            got < f64::from(SAMPLE_PERIOD_SEC),
            "step {k}: time {got} s exceeds sample period"
        );
    }
    for pair in times.windows(2) {
        assert!(pair[0] < pair[1], "non-increasing step times: {times:?}");
    }
}
