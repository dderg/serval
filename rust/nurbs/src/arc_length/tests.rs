use super::*;

#[test]
fn integrate_constant_returns_length_times_constant() {
    let result = integrate_arc_length(|_u: f64| 2.0_f64, 0.0, 1.0, 5);
    assert!((result - 2.0).abs() < 1e-12);
}

#[test]
fn integrate_linear_matches_closed_form() {
    let result = integrate_arc_length(|u: f64| u, 0.0, 1.0, 5);
    assert!((result - 0.5).abs() < 1e-12);
}

#[test]
fn integrate_quadratic_matches_closed_form() {
    let result = integrate_arc_length(|u: f64| u * u, 0.0, 1.0, 5);
    assert!((result - 1.0 / 3.0).abs() < 1e-12);
}

fn chord(curve: &crate::VectorNurbs<3>) -> f64 {
    let knots = curve.knots();
    let start = crate::eval::vector_eval(curve, knots[0]);
    let end = crate::eval::vector_eval(curve, knots[knots.len() - 1]);
    let d = [end[0] - start[0], end[1] - start[1], end[2] - start[2]];
    (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
}

#[test]
fn collinear_quartic_across_a_c0_knot_measures_the_chord() {
    let heading = [1.0_f64, -0.5, 0.25];
    let mut cps = vec![[0.0_f64, 0.0, 0.0]];
    for step in 0..8 {
        let advance = 0.5 + 0.15 * f64::from(step);
        let prev = cps[cps.len() - 1];
        cps.push([
            prev[0] + heading[0] * advance,
            prev[1] + heading[1] * advance,
            prev[2] + heading[2] * advance,
        ]);
    }
    let curve = crate::VectorNurbs::<3>::try_new(
        4,
        vec![
            0.0, 0.0, 0.0, 0.0, 0.0, 0.4, 0.4, 0.4, 0.4, 1.9, 1.9, 1.9, 1.9, 1.9,
        ],
        cps,
    )
    .unwrap();

    let straight = chord(&curve);
    let length = path_arc_length(&curve);
    assert!(
        (length - straight).abs() <= 1e-13 * straight,
        "collinear control points: length {length} vs chord {straight}"
    );
}

/// 5-point Gauss-Legendre over the single knot span, stable to 1e-15 from 8 to
/// 256 panels.
const CONVERGED_CUBIC_ARC_LENGTH: f64 = 5.757_958_783_394_672;

#[test]
fn single_span_cubic_refines_past_the_first_level() {
    let end = 0.787_703_227_103_625;
    let curve = crate::VectorNurbs::<3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, end, end, end, end],
        vec![
            [0.0, 0.0, 0.0],
            [
                1.366_922_748_251_334_9,
                -0.955_560_871_555_292_6,
                -0.203_225_439_910_827_53,
            ],
            [
                3.271_567_085_379_173_6,
                -1.516_902_631_138_470_1,
                0.568_759_023_982_981_2,
            ],
            [
                5.209_463_407_618_529,
                -1.633_935_571_535_171,
                1.384_952_413_073_341_7,
            ],
        ],
    )
    .unwrap();

    let length = path_arc_length(&curve);
    assert!(
        (length - CONVERGED_CUBIC_ARC_LENGTH).abs() <= 1e-12 * CONVERGED_CUBIC_ARC_LENGTH,
        "length {length} vs converged {CONVERGED_CUBIC_ARC_LENGTH}"
    );
}
