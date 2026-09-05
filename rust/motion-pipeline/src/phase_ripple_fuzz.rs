use nurbs::bezier::{BezierPiece, bezier_pieces_to_nurbs};
use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;
use trajectory::{AdvanceModel, NonlinearAdvance};

use crate::lowering::FitTol;
use crate::shaper::{TrackSignal, apply_nonlinear_advance_to_track, fit_axis_from_signal};

#[derive(Clone, Debug)]
struct CruiseRamp {
    start: f64,
    boundary: f64,
    end: f64,
    position: f64,
    velocity: f64,
    acceleration: f64,
}

impl TrackSignal for CruiseRamp {
    fn eval(&self, t: f64) -> f64 {
        let ramp_time = (t - self.boundary).max(0.0);
        self.position
            + self.velocity * (t - self.start)
            + 0.5 * self.acceleration * ramp_time * ramp_time
    }

    fn deriv(&self, t: f64) -> f64 {
        self.velocity + self.acceleration * (t - self.boundary).max(0.0)
    }

    fn second_deriv(&self, t: f64) -> f64 {
        if t <= self.boundary {
            0.0
        } else {
            self.acceleration
        }
    }
}

fn cruise_ramp() -> impl Strategy<Value = CruiseRamp> {
    (
        prop_oneof![Just(0.0), Just(1.0), Just(16.0), 0.0f64..120.0],
        prop_oneof![
            Just(0.299_987_113_843_926_7),
            (-5i32..1, 0.25f64..1.0).prop_map(|(exponent, factor)| factor * 2.0f64.powi(exponent)),
        ],
        0.25f64..2.0,
        (-2i32..7, 0.5f64..1.0),
        any::<bool>(),
        any::<bool>(),
        0.1f64..0.8,
        -20.0f64..20.0,
    )
        .prop_map(
            |(
                start,
                cruise_duration,
                ramp_ratio,
                (exponent, factor),
                negative,
                decelerating,
                change,
                position,
            )| {
                let boundary = start + cruise_duration;
                let end = boundary + cruise_duration * ramp_ratio;
                let velocity = factor * 2.0f64.powi(exponent) * if negative { -1.0 } else { 1.0 };
                let acceleration =
                    velocity * change * if decelerating { -1.0 } else { 1.0 } / (end - boundary);
                CruiseRamp {
                    start,
                    boundary,
                    end,
                    position,
                    velocity,
                    acceleration,
                }
            },
        )
}

fn config() -> ProptestConfig {
    ProptestConfig {
        cases: std::env::var("PROPTEST_CASES")
            .map(|value| value.parse().expect("PROPTEST_CASES must be an integer"))
            .unwrap_or(128),
        failure_persistence: Some(Box::new(FileFailurePersistence::Direct(
            "proptest-regressions/phase_ripple_fuzz.txt",
        ))),
        ..ProptestConfig::default()
    }
}

fn split_bernstein(coefficients: &[f64], fraction: f64) -> (Vec<f64>, Vec<f64>) {
    let degree = coefficients.len() - 1;
    let mut work = coefficients.to_vec();
    let mut left = vec![work[0]];
    let mut right = vec![work[degree]];
    for remaining in (1..=degree).rev() {
        for index in 0..remaining {
            work[index] = (1.0 - fraction) * work[index] + fraction * work[index + 1];
        }
        left.push(work[0]);
        right.push(work[remaining - 1]);
    }
    right.reverse();
    (left, right)
}

fn subdivided_bernstein_bound(coefficients: &[f64], tolerance: f64, depth: u32) -> f64 {
    if coefficients.iter().any(|value| !value.is_finite()) {
        return f64::INFINITY;
    }
    let bound = coefficients
        .iter()
        .fold(0.0f64, |bound, value| bound.max(value.abs()));
    if bound <= tolerance || depth == 0 {
        return bound;
    }
    let (left, right) = split_bernstein(coefficients, 0.5);
    subdivided_bernstein_bound(&left, tolerance, depth - 1).max(subdivided_bernstein_bound(
        &right,
        tolerance,
        depth - 1,
    ))
}

fn assert_flat_cruise(
    output: &nurbs::ScalarNurbs,
    signal: &CruiseRamp,
    advance_position_scale: f64,
) -> Result<(), TestCaseError> {
    let refined = nurbs::knot::refined_to_full_multiplicity(output);
    let degree = refined.degree() as usize;
    let knots = refined.knots();
    let controls = refined.control_points();
    let mut covered_until = signal.start;
    for span in degree..controls.len() {
        let start = knots[span];
        let end = knots[span + 1];
        if start == end || start >= signal.boundary {
            continue;
        }
        prop_assert_eq!(start, covered_until);
        let cruise_end = end.min(signal.boundary);
        prop_assert!(cruise_end > start);
        let fit_duration = end - start;
        let position_scale = signal.position.abs()
            + signal.velocity.abs() * (signal.end - signal.start)
            + advance_position_scale;
        let sample_roundoff = f64::EPSILON
            * (position_scale + signal.velocity.abs() * signal.end.abs().max(signal.start.abs()));
        let velocity_roundoff = 512.0 * sample_roundoff / fit_duration;
        let acceleration_roundoff = 512.0 * sample_roundoff / fit_duration.powi(2);
        let velocity: Vec<_> = controls[span - degree..=span]
            .windows(2)
            .map(|pair| degree as f64 * (pair[1] - pair[0]) / fit_duration)
            .collect();
        let mut acceleration: Vec<_> = velocity
            .windows(2)
            .map(|pair| (degree - 1) as f64 * (pair[1] - pair[0]) / fit_duration)
            .collect();
        let mut velocity_error: Vec<_> = velocity
            .iter()
            .map(|value| value - signal.velocity)
            .collect();
        if cruise_end < end {
            let fraction = (cruise_end - start) / fit_duration;
            velocity_error = split_bernstein(&velocity_error, fraction).0;
            acceleration = split_bernstein(&acceleration, fraction).0;
        }
        let velocity_error_bound =
            subdivided_bernstein_bound(&velocity_error, velocity_roundoff, 8);
        let acceleration_bound =
            subdivided_bernstein_bound(&acceleration, acceleration_roundoff, 8);
        prop_assert!(
            velocity_error_bound <= velocity_roundoff,
            "cruise velocity ripple: bound={velocity_error_bound:e}, roundoff={velocity_roundoff:e}, signal={signal:?}, span=[{start}, {end}]"
        );
        prop_assert!(
            acceleration_bound <= acceleration_roundoff,
            "cruise acceleration ripple: bound={acceleration_bound:e}, roundoff={acceleration_roundoff:e}, signal={signal:?}, span=[{start}, {end}]"
        );
        covered_until = cruise_end;
    }
    prop_assert_eq!(covered_until, signal.boundary);
    Ok(())
}

proptest! {
    #![proptest_config(config())]

    #[test]
    fn shared_fitter_does_not_import_next_phase_acceleration(
        axis in 0usize..4,
        signal in cruise_ramp(),
    ) {
        let output = fit_axis_from_signal(
            axis,
            signal.start,
            signal.end,
            &[signal.boundary],
            &signal,
            FitTol { pos_mm: 0.005, accel_mm_s2: 50.0 },
            "phase_ripple_fuzz",
        ).map_err(|error| TestCaseError::fail(format!("{signal:?}: {error:?}")))?;
        assert_flat_cruise(&output, &signal, 0.0)?;
    }

    #[test]
    fn nonlinear_advance_does_not_import_next_phase_acceleration(
        axis in 0usize..4,
        signal in cruise_ramp(),
        reciprocal in any::<bool>(),
        zero_gain in any::<bool>(),
        linear_advance in 0.0f64..0.08,
        nonlinear_offset in 0.001f64..0.12,
        knee_ratio in 0.25f64..4.0,
    ) {
        let advance = NonlinearAdvance {
            model: if reciprocal { AdvanceModel::Reciprocal } else { AdvanceModel::Tanh },
            linear_advance: if zero_gain { 0.0 } else { linear_advance },
            nonlinear_offset: if zero_gain { 0.0 } else { nonlinear_offset },
            linearization_velocity: signal.velocity.abs() * knee_ratio,
        };
        let track = bezier_pieces_to_nurbs(&[
            BezierPiece {
                u_start: signal.start,
                u_end: signal.boundary,
                coeffs: vec![signal.position, signal.velocity, 0.0],
            },
            BezierPiece {
                u_start: signal.boundary,
                u_end: signal.end,
                coeffs: vec![signal.eval(signal.boundary), signal.velocity, 0.5 * signal.acceleration],
            },
        ]);
        let output = apply_nonlinear_advance_to_track(
            axis, &track, advance, FitTol { pos_mm: 0.005, accel_mm_s2: 50.0 },
        ).map_err(|error| TestCaseError::fail(format!("{signal:?}, {advance:?}: {error:?}")))?;
        let advance_position_scale = advance.linear_advance * signal.velocity.abs() * 1.8
            + advance.nonlinear_offset;
        assert_flat_cruise(&output, &signal, advance_position_scale)?;
    }
}
