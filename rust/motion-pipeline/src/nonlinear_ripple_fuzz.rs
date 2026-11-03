use crate::lowering::FitTol;
use nurbs::bezier::{BezierPiece, bezier_pieces_to_nurbs, extract_bezier_pieces};
use proptest::prelude::*;
use proptest::test_runner::{Config, FileFailurePersistence};
use trajectory::{AdvanceModel, NonlinearAdvance};

fn config() -> Config {
    Config {
        cases: std::env::var("PROPTEST_CASES")
            .ok()
            .and_then(|value| value.parse().ok())
            .unwrap_or(128),
        failure_persistence: Some(Box::new(FileFailurePersistence::Direct(
            "proptest-regressions/nonlinear_ripple_fuzz.txt",
        ))),
        ..Config::default()
    }
}

fn value(coefficients: &[f64], x: f64) -> f64 {
    coefficients
        .iter()
        .rev()
        .fold(0.0, |sum, c| sum.mul_add(x, *c))
}

fn derivative(coefficients: &[f64]) -> Vec<f64> {
    coefficients
        .iter()
        .enumerate()
        .skip(1)
        .map(|(n, c)| n as f64 * c)
        .collect()
}

fn bernstein_derivative(controls: &[f64]) -> Vec<f64> {
    if controls.len() <= 1 {
        return vec![0.0];
    }
    let degree = controls.len() - 1;
    controls
        .windows(2)
        .map(|pair| degree as f64 * (pair[1] - pair[0]))
        .collect()
}

fn bernstein_value(controls: &[f64], u: f64) -> f64 {
    let mut work = controls.to_vec();
    for remaining in (1..work.len()).rev() {
        for i in 0..remaining {
            work[i] = (1.0 - u).mul_add(work[i], u * work[i + 1]);
        }
    }
    work[0]
}

fn roots(coefficients: &[f64], low: f64, high: f64) -> Vec<f64> {
    let mut degree = coefficients.len();
    while degree > 0 && coefficients[degree - 1] == 0.0 {
        degree -= 1;
    }
    let coefficients = &coefficients[..degree];
    if degree <= 1 {
        return Vec::new();
    }
    if degree == 2 {
        let root = -coefficients[0] / coefficients[1];
        return if root > low && root < high {
            vec![root]
        } else {
            Vec::new()
        };
    }
    let mut partition = vec![low];
    partition.extend(roots(&derivative(coefficients), low, high));
    partition.push(high);
    let mut result = Vec::new();
    for bounds in partition.windows(2) {
        let (mut left, mut right) = (bounds[0], bounds[1]);
        let left_value = value(coefficients, left);
        let right_value = value(coefficients, right);
        if left_value == 0.0 && left > low {
            result.push(left);
        }
        if left_value * right_value >= 0.0 {
            continue;
        }
        for _ in 0..56 {
            let midpoint = 0.5 * (left + right);
            if value(coefficients, midpoint).is_sign_positive() == left_value.is_sign_positive() {
                left = midpoint;
            } else {
                right = midpoint;
            }
        }
        result.push(0.5 * (left + right));
    }
    result
}

fn law(model: AdvanceModel, z: f64) -> (f64, f64, f64) {
    match model {
        AdvanceModel::Tanh => {
            let t = libm::tanh(z);
            let sech_squared = 1.0 - t * t;
            (
                t,
                -2.0 * t * sech_squared,
                2.0 * sech_squared * (3.0 * t * t - 1.0),
            )
        }
        AdvanceModel::Reciprocal => {
            let denominator = 1.0 + z.abs();
            (
                z / denominator,
                -2.0 * z.signum() / denominator.powi(3),
                6.0 / denominator.powi(4),
            )
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct Ramp {
    axis: usize,
    reciprocal: bool,
    reverse: bool,
    decelerate: bool,
    crosses_zero: bool,
    initial_fraction: f64,
    velocity_sweep: f64,
    velocity_scale: f64,
    duration: f64,
    origin: f64,
    offset: f64,
    linear_gain: f64,
}

/// Between the law's acceleration extrema the refit's jerk keeps the law's
/// sign. An extremum the refit coalesced onto a piece edge — closer to it
/// than the refit's resolution floor — is judged on that edge: no span
/// shorter than the floor exists to carry its reversal.
fn ramp_invents_no_acceleration_reversal(ramp: Ramp) -> Result<(), TestCaseError> {
    let Ramp {
        axis,
        reciprocal,
        reverse,
        decelerate,
        crosses_zero,
        initial_fraction,
        velocity_sweep,
        velocity_scale,
        duration,
        origin,
        offset,
        linear_gain,
    } = ramp;
    let model = if reciprocal {
        AdvanceModel::Reciprocal
    } else {
        AdvanceModel::Tanh
    };
    let direction = if reverse { -1.0 } else { 1.0 };
    let low = if crosses_zero {
        -initial_fraction * velocity_sweep
    } else {
        initial_fraction
    };
    let high = velocity_sweep;
    let (start_z, end_z) = if decelerate { (high, low) } else { (low, high) };
    let initial_velocity = direction * start_z * velocity_scale;
    let end = origin + duration;
    let duration = end - origin;
    let acceleration = direction * (end_z - start_z) * velocity_scale / duration;
    let track = bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: origin,
        u_end: end,
        coeffs: vec![0.0, initial_velocity, 0.5 * acceleration],
    }]);
    let advance = NonlinearAdvance {
        model,
        linear_advance: linear_gain,
        nonlinear_offset: offset,
        linearization_velocity: velocity_scale,
    };
    let tolerance = FitTol::absolute(1e-3, 50.0);
    let output = crate::shaper::apply_nonlinear_advance_to_track(axis, &track, advance, tolerance);
    prop_assert!(
        output.is_ok(),
        "axis={axis} advance={advance:?}: {output:?}"
    );
    let output = nurbs::knot::refined_to_full_multiplicity(&output.unwrap());
    let coalesced_onto_edge =
        crate::shaper::nonlinear_advance_resolution_floor(axis, &track, advance, tolerance)
            .expect("the refit that succeeded planned its seeds");
    let pieces = extract_bezier_pieces(&output);
    let degree = output.degree() as usize;
    let controls: Vec<&[f64]> = (degree..output.control_points().len())
        .filter(|&span| output.knots()[span] < output.knots()[span + 1])
        .map(|span| &output.control_points()[span - degree..=span])
        .collect();
    let extremum_z = (1.0 / 3.0f64.sqrt()).atanh();
    let critical_velocities = if reciprocal {
        vec![0.0]
    } else {
        vec![-extremum_z * velocity_scale, extremum_z * velocity_scale]
    };
    let critical_times: Vec<f64> = critical_velocities
        .iter()
        .map(|velocity| origin + (velocity - initial_velocity) / acceleration)
        .filter(|time| *time > origin && *time < end)
        .collect();
    let mut previous: Option<(f64, f64, f64, f64, f64)> = None;
    for (index, piece) in pieces.iter().enumerate() {
        let h = piece.u_end - piece.u_start;
        let normalized = controls[index];
        let velocity = bernstein_derivative(normalized);
        let acceleration_poly = bernstein_derivative(&velocity);
        let jerk = bernstein_derivative(&acceleration_poly);
        let snap = bernstein_derivative(&jerk);
        let snap = BezierPiece::from_bernstein(&snap, 0.0, 1.0).coeffs;
        let degree = normalized.len() as f64;
        let position_scale = piece
            .coeffs
            .iter()
            .enumerate()
            .map(|(power, coefficient)| (coefficient * h.powi(power as i32)).abs())
            .sum::<f64>();
        let position_roundoff = 128.0 * f64::EPSILON * degree.powi(3) * position_scale;
        let acceleration_roundoff = position_roundoff / (h * h);
        let jerk_roundoff = position_roundoff * degree / (h * h * h);
        let mut partitions = vec![0.0, 1.0];
        partitions.extend(
            critical_times
                .iter()
                .filter(|time| {
                    **time - piece.u_start >= coalesced_onto_edge
                        && piece.u_end - **time >= coalesced_onto_edge
                })
                .map(|time| (time - piece.u_start) / h),
        );
        partitions.sort_by(f64::total_cmp);
        for interval in partitions.windows(2) {
            let mid_time = piece.u_start + h * (interval[0] + interval[1]) * 0.5;
            let z = (initial_velocity + acceleration * (mid_time - origin)) / velocity_scale;
            let expected_direction = (law(model, z).2 * acceleration).signum();
            let mut witnesses = vec![interval[0], interval[1]];
            witnesses.extend(roots(&snap, interval[0], interval[1]));
            for u in witnesses {
                let actual_jerk = bernstein_value(&jerk, u) / h.powi(3);
                prop_assert!(
                    actual_jerk.is_finite() && expected_direction * actual_jerk >= -jerk_roundoff,
                    "axis={axis} model={model:?} piece={index} u={u} jerk={actual_jerk} direction={expected_direction} roundoff={jerk_roundoff} range={interval:?}"
                );
            }
        }
        for sample in 1..256 {
            let u = 0.5 * (1.0 - libm::cos(std::f64::consts::PI * sample as f64 / 256.0));
            let time = piece.u_start + h * u;
            let elapsed = time - origin;
            let v = initial_velocity + acceleration * elapsed;
            let (shape, curvature, _) = law(model, v / velocity_scale);
            let expected_position = initial_velocity * elapsed
                + 0.5 * acceleration * elapsed * elapsed
                + linear_gain * v
                + offset * shape;
            let expected_acceleration =
                acceleration + offset * curvature * (acceleration / velocity_scale).powi(2);
            let position_error = (bernstein_value(normalized, u) - expected_position).abs();
            let acceleration_error =
                (bernstein_value(&acceleration_poly, u) / h.powi(2) - expected_acceleration).abs();
            prop_assert!(
                position_error <= tolerance.pos_mm + position_roundoff,
                "axis={axis} model={model:?} piece={index} u={u} position error={position_error}"
            );
            prop_assert!(
                acceleration_error <= tolerance.accel_mm_s2 + acceleration_roundoff,
                "axis={axis} model={model:?} piece={index} u={u} acceleration error={acceleration_error}"
            );
        }
        let start_acceleration = bernstein_value(&acceleration_poly, 0.0) / h.powi(2);
        if let Some((
            previous_end,
            previous_acceleration,
            previous_roundoff,
            previous_velocity,
            previous_velocity_roundoff,
        )) = previous
        {
            prop_assert_eq!(previous_end, piece.u_start);
            let velocity_step = bernstein_value(&velocity, 0.0) / h - previous_velocity;
            prop_assert!(
                velocity_step.abs() <= previous_velocity_roundoff + position_roundoff / h,
                "axis={axis} model={model:?} join={index} velocity step={velocity_step}"
            );
            let is_physical_transition = critical_times
                .iter()
                .any(|time| (*time - piece.u_start).abs() < coalesced_onto_edge);
            if !is_physical_transition {
                let z =
                    (initial_velocity + acceleration * (piece.u_start - origin)) / velocity_scale;
                let expected_direction = (law(model, z).2 * acceleration).signum();
                let step = expected_direction * (start_acceleration - previous_acceleration);
                prop_assert!(
                    step >= -(previous_roundoff + acceleration_roundoff),
                    "axis={axis} model={model:?} join={index} backward acceleration step={step} roundoff={}",
                    previous_roundoff + acceleration_roundoff
                );
            }
        }
        previous = Some((
            piece.u_end,
            bernstein_value(&acceleration_poly, 1.0) / h.powi(2),
            acceleration_roundoff,
            bernstein_value(&velocity, 1.0) / h,
            position_roundoff / h,
        ));
    }
    Ok(())
}

proptest! {
    #![proptest_config(config())]

    #[test]
    fn nonlinear_ramps_do_not_invent_acceleration_reversals(
        axis in 0usize..5,
        reciprocal in any::<bool>(),
        reverse in any::<bool>(),
        decelerate in any::<bool>(),
        crosses_zero in any::<bool>(),
        initial_fraction in 0.025f64..0.5,
        velocity_sweep in 2.0f64..7.0,
        velocity_scale in 0.5f64..8.0,
        duration in 0.02f64..0.2,
        origin in 0.0f64..2.0,
        offset in 0.02f64..0.12,
        linear_gain in 0.0f64..0.06,
    ) {
        ramp_invents_no_acceleration_reversal(Ramp {
            axis,
            reciprocal,
            reverse,
            decelerate,
            crosses_zero,
            initial_fraction,
            velocity_sweep,
            velocity_scale,
            duration,
            origin,
            offset,
            linear_gain,
        })?;
    }
}

fn ramp_with_its_first_extremum_at(extremum_after_start_s: f64) -> Ramp {
    let extremum_z = (1.0 / 3.0f64.sqrt()).atanh();
    let (velocity_sweep, duration) = (4.220_492_511_721_14, 0.02);
    let sweep_rate = velocity_sweep / duration;
    Ramp {
        axis: 0,
        reciprocal: false,
        reverse: false,
        decelerate: false,
        crosses_zero: true,
        initial_fraction: (extremum_z + extremum_after_start_s * sweep_rate)
            / (velocity_sweep * (1.0 - extremum_after_start_s / duration)),
        velocity_sweep,
        velocity_scale: 3.449_412_995_193_711,
        duration,
        origin: 0.0,
        offset: 0.02,
        linear_gain: 0.0,
    }
}

/// #589: the ramp starts 19 ns before the tanh law's acceleration extremum,
/// inside the refit's coalescing floor. The extremum's seed coalesced onto
/// the start and the span's acceleration trend went undeclared, so the
/// accepted rung rippled 166 µs deep at both ends.
#[test]
fn a_ramp_starting_just_before_its_acceleration_extremum_keeps_it() {
    ramp_invents_no_acceleration_reversal(Ramp {
        axis: 0,
        reciprocal: false,
        reverse: false,
        decelerate: false,
        crosses_zero: true,
        initial_fraction: 0.156_020_564_132_454_05,
        velocity_sweep: 4.220_492_511_721_14,
        velocity_scale: 3.449_412_995_193_711,
        duration: 0.02,
        origin: 0.0,
        offset: 0.02,
        linear_gain: 0.0,
    })
    .unwrap();
}

#[test]
fn an_acceleration_extremum_near_the_ramp_start_is_kept_on_either_side_of_the_floor() {
    for extremum_after_start_s in [
        -1e-6, -1e-8, 0.0, 1e-9, 5e-9, 1e-8, 1.916e-8, 3e-8, 1e-7, 1e-6, 1e-5, 1e-4,
    ] {
        ramp_invents_no_acceleration_reversal(ramp_with_its_first_extremum_at(
            extremum_after_start_s,
        ))
        .unwrap_or_else(|error| panic!("extremum {extremum_after_start_s:e} s in: {error}"));
    }
}
