use super::*;

#[test]
fn evaluate_constant_polynomial_is_constant() {
    let p = BezierPiece {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![3.5],
    };
    assert_eq!(p.evaluate(0.0), 3.5);
    assert_eq!(p.evaluate(0.5), 3.5);
    assert_eq!(p.evaluate(1.0), 3.5);
}

#[test]
fn evaluate_linear_polynomial() {
    let p = BezierPiece {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![1.0, 2.0],
    };
    assert_eq!(p.evaluate(0.0), 1.0);
    assert_eq!(p.evaluate(0.5), 2.0);
    assert_eq!(p.evaluate(1.0), 3.0);
}

#[test]
fn evaluate_uses_shifted_basis() {
    let p = BezierPiece {
        u_start: 5.0,
        u_end: 7.0,
        coeffs: vec![1.0, 2.0],
    };
    assert_eq!(p.evaluate(5.0), 1.0);
    assert_eq!(p.evaluate(6.0), 3.0);
    assert_eq!(p.evaluate(7.0), 5.0);
}

#[test]
fn bernstein_round_trip_preserves_polynomial() {
    let monom = BezierPiece {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![1.0, 2.0, 3.0],
    };
    let bernstein = monom.to_bernstein();
    let back = BezierPiece::from_bernstein(&bernstein, 0.0, 1.0);

    for u in [0.0, 0.25, 0.5, 0.75, 1.0] {
        let exp = monom.evaluate(u);
        let got = back.evaluate(u);
        assert!((exp - got).abs() < 1e-12, "u={u}: exp={exp}, got={got}");
    }
}

#[test]
fn cubic_bernstein_round_trip_on_shifted_support() {
    let p = BezierPiece {
        u_start: 1.0,
        u_end: 3.0,
        coeffs: vec![1.0, -2.0, 3.0, -4.0],
    };
    let bern = p.to_bernstein();
    let back = BezierPiece::from_bernstein(&bern, 1.0, 3.0);
    for u in [1.0, 1.5, 2.0, 2.5, 3.0] {
        let exp = p.evaluate(u);
        let got = back.evaluate(u);
        assert!((exp - got).abs() < 1e-12, "u={u}: exp={exp}, got={got}");
    }
}

#[test]
fn from_bernstein_to_monomial_for_known_case() {
    let p = BezierPiece::from_bernstein(&[0.0_f64, 1.0], 0.0, 1.0);
    assert!((p.coeffs[0] - 0.0).abs() < 1e-12);
    assert!((p.coeffs[1] - 1.0).abs() < 1e-12);
}

#[test]
fn from_bernstein_derivatives_ignore_the_offset_of_the_control_points() {
    let offset = 300.0;
    let h = 1.0e-6;
    let local: Vec<f64> = (0..14)
        .map(|i| {
            let u = f64::from(i) / 13.0;
            3.0e-4 * u + 1.0e-8 * u * u
        })
        .collect();
    let far: Vec<f64> = local.iter().map(|b| b + offset).collect();
    let exactly_local: Vec<f64> = far.iter().map(|b| b - offset).collect();
    let near_piece = BezierPiece::from_bernstein(&exactly_local, 0.0, h);
    let far_piece = BezierPiece::from_bernstein(&far, 0.0, h);
    assert_eq!(far_piece.coeffs[0] - offset, near_piece.coeffs[0]);
    for k in 1..far_piece.coeffs.len() {
        assert_eq!(
            far_piece.coeffs[k], near_piece.coeffs[k],
            "monomial coefficient {k} depends on where the piece sits"
        );
    }
}

use crate::ScalarNurbs;

#[test]
fn extract_single_bezier_piece_from_clamped_curve() {
    let curve =
        ScalarNurbs::try_new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![0.0, 1.0, 4.0]).unwrap();

    let pieces = extract_bezier_pieces(&curve);
    assert_eq!(pieces.len(), 1);
    let p = &pieces[0];
    assert_eq!(p.u_start, 0.0);
    assert_eq!(p.u_end, 1.0);
    assert_eq!(p.degree(), 2);
    for u in [0.0, 0.25, 0.5, 0.75, 1.0] {
        let exp = crate::eval::eval(&curve, u);
        let got = p.evaluate(u);
        assert!((exp - got).abs() < 1e-12, "u={u}: exp={exp}, got={got}");
    }
}

#[test]
fn extract_two_bezier_pieces_from_curve_with_interior_knot() {
    let curve = ScalarNurbs::try_new(
        2,
        vec![0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0],
        vec![0.0, 1.0, 2.0, 3.0],
    )
    .unwrap();

    let pieces = extract_bezier_pieces(&curve);
    assert_eq!(pieces.len(), 2);
    assert_eq!(pieces[0].u_start, 0.0);
    assert_eq!(pieces[0].u_end, 0.5);
    assert_eq!(pieces[1].u_start, 0.5);
    assert_eq!(pieces[1].u_end, 1.0);
    let mid_left = pieces[0].evaluate(0.5);
    let mid_right = pieces[1].evaluate(0.5);
    assert!((mid_left - mid_right).abs() < 1e-12);
    for u in [0.0, 0.25, 0.5] {
        let exp = crate::eval::eval(&curve, u);
        let got = pieces[0].evaluate(u);
        assert!((exp - got).abs() < 1e-12);
    }
    for u in [0.5, 0.75, 1.0] {
        let exp = crate::eval::eval(&curve, u);
        let got = pieces[1].evaluate(u);
        assert!((exp - got).abs() < 1e-12);
    }
}

#[test]
fn split_piece_at_preserves_evaluation_on_each_side() {
    let original = BezierPiece {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![1.0, 2.0],
    };
    let (left, right) = split_piece_at(&original, 0.4);

    assert_eq!(left.u_start, 0.0);
    assert_eq!(left.u_end, 0.4);
    assert_eq!(right.u_start, 0.4);
    assert_eq!(right.u_end, 1.0);

    for u in [0.0, 0.2, 0.4] {
        let exp = original.evaluate(u);
        let got = left.evaluate(u);
        assert!((exp - got).abs() < 1e-12);
    }
    for u in [0.4, 0.7, 1.0] {
        let exp = original.evaluate(u);
        let got = right.evaluate(u);
        assert!((exp - got).abs() < 1e-12);
    }
}

#[test]
fn bezier_pieces_to_nurbs_round_trips_extraction() {
    let original = ScalarNurbs::try_new(
        2,
        vec![0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0],
        vec![0.0, 1.0, 2.0, 3.0],
    )
    .unwrap();

    let pieces = extract_bezier_pieces(&original);
    let recomposed = bezier_pieces_to_nurbs(&pieces);

    for u in [0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0] {
        let exp = crate::eval::eval(&original, u);
        let got = crate::eval::eval(&recomposed, u);
        assert!((exp - got).abs() < 1e-10, "u={u}: exp={exp}, got={got}");
    }
}

/// The conversion before its binomial ratios came from a Pascal row: each
/// ratio of two exact integers rounds once either way, so the controls
/// must match it bit for bit.
fn reference_to_bernstein(piece: &BezierPiece) -> Vec<f64> {
    let d = piece.degree();
    let h = piece.u_end - piece.u_start;
    let mut h_pow = 1.0;
    let normalized: Vec<f64> = piece
        .coeffs
        .iter()
        .map(|c| {
            let v = *c * h_pow;
            h_pow *= h;
            v
        })
        .collect();
    (0..=d)
        .map(|k| {
            (0..=k).fold(0.0, |acc, i| {
                acc + (binomial(k, i) as f64 / binomial(d, i) as f64) * normalized[i]
            })
        })
        .collect()
}

#[test]
fn bernstein_controls_match_the_per_term_binomials_bit_for_bit() {
    let mut state = 0x1234_5678_9abc_def1_u64;
    let mut next = || {
        state = state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (state >> 11) as f64 / (1_u64 << 53) as f64 * 2.0 - 1.0
    };
    for degree in 0..=50 {
        for _ in 0..20 {
            let piece = BezierPiece {
                u_start: next(),
                u_end: 1.5 + next(),
                coeffs: (0..=degree).map(|_| next() * 1e3).collect(),
            };
            let bits = |controls: Vec<f64>| -> Vec<u64> {
                controls.into_iter().map(f64::to_bits).collect()
            };
            assert_eq!(
                bits(piece.to_bernstein()),
                bits(reference_to_bernstein(&piece)),
                "degree {degree}"
            );
        }
    }
}
