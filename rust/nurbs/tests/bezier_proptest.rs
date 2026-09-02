use nurbs::ScalarNurbs;
use nurbs::bezier::{BezierPiece, bezier_pieces_to_nurbs, extract_bezier_pieces, split_piece_at};
use nurbs::chebyshev::taylor_shift;
use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;

fn choose(n: usize, k: usize) -> f64 {
    if k > n {
        return 0.0;
    }
    let mut result = 1.0_f64;
    for i in 0..k.min(n - k) {
        result = result * ((n - i) as f64) / ((i + 1) as f64);
    }
    result
}

/// Every intermediate of a monomial or Bernstein evaluation of `coeffs` over a
/// support of width `span` is bounded by `sum |c_i| span^i`.
fn monomial_reach(coeffs: &[f64], span: f64) -> f64 {
    coeffs.iter().rev().fold(0.0, |acc, c| acc * span + c.abs())
}

fn bernstein_sum(bernstein: &[f64], s: f64) -> f64 {
    let d = bernstein.len() - 1;
    let mut acc = 0.0;
    for (k, b) in bernstein.iter().enumerate() {
        acc += b * choose(d, k) * s.powi(k as i32) * (1.0 - s).powi((d - k) as i32);
    }
    acc
}

fn arb_piece() -> impl Strategy<Value = BezierPiece> {
    (1usize..=5, -2.0..2.0_f64, 0.25..4.0_f64).prop_flat_map(|(d, u_start, h)| {
        prop::collection::vec(-4.0..4.0_f64, d + 1).prop_map(move |coeffs| BezierPiece {
            u_start,
            u_end: u_start + h,
            coeffs,
        })
    })
}

fn arb_bernstein() -> impl Strategy<Value = (Vec<f64>, f64, f64)> {
    (1usize..=5, -2.0..2.0_f64, 0.25..4.0_f64).prop_flat_map(|(d, u_start, h)| {
        prop::collection::vec(-4.0..4.0_f64, d + 1)
            .prop_map(move |bernstein| (bernstein, u_start, u_start + h))
    })
}

/// Clamped curve on [0, 1] with `interior.len()` distinct interior breakpoints,
/// each at the generated multiplicity (never above the degree, so the curve
/// stays continuous).
fn arb_curve() -> impl Strategy<Value = ScalarNurbs> {
    (1u8..=5, 0usize..=3).prop_flat_map(|(p, interior_count)| {
        let offsets = prop::collection::vec(-0.3..0.3_f64, interior_count);
        let multiplicities = prop::collection::vec(1usize..=(p as usize), interior_count);
        (offsets, multiplicities).prop_flat_map(move |(offsets, multiplicities)| {
            let pad = p as usize + 1;
            let mut knots = vec![0.0; pad];
            for (i, (offset, multiplicity)) in offsets.iter().zip(multiplicities.iter()).enumerate()
            {
                let position = ((i + 1) as f64 + offset) / (interior_count + 1) as f64;
                for _ in 0..*multiplicity {
                    knots.push(position);
                }
            }
            knots.extend(std::iter::repeat_n(1.0, pad));
            let n = knots.len() - pad;
            prop::collection::vec(-10.0..10.0_f64, n)
                .prop_map(move |cps| ScalarNurbs::try_new(p, knots.clone(), cps).unwrap())
        })
    })
}

fn arb_unit_samples() -> impl Strategy<Value = Vec<f64>> {
    prop::collection::vec(0.0..=1.0_f64, 64)
}

fn distinct_knots(curve: &ScalarNurbs) -> Vec<f64> {
    let mut distinct: Vec<f64> = Vec::new();
    for k in curve.knots() {
        if distinct.last() != Some(k) {
            distinct.push(*k);
        }
    }
    distinct
}

fn control_point_scale(curve: &ScalarNurbs) -> f64 {
    curve
        .control_points()
        .iter()
        .fold(1.0_f64, |acc, c| acc.max(c.abs()))
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 384,
        failure_persistence: Some(Box::new(FileFailurePersistence::Direct(
            "proptest-regressions/bezier.txt",
        ))),
        ..ProptestConfig::default()
    })]

    #[test]
    fn extract_then_recompose_preserves_evaluation(
        curve in arb_curve(),
        samples in arb_unit_samples(),
    ) {
        let pieces = extract_bezier_pieces(&curve);
        let recomposed = bezier_pieces_to_nurbs(&pieces);
        let scale = control_point_scale(&curve);
        let budget = 1e-9 * scale;

        let mut probes = samples;
        probes.extend(distinct_knots(&curve));
        for u in probes {
            let before = nurbs::eval::eval(&curve.as_view(), u);
            let after = nurbs::eval::eval(&recomposed.as_view(), u);
            prop_assert!(
                (before - after).abs() <= budget,
                "u={u}: {before} vs {after} (budget {budget}), knots {:?}",
                curve.knots()
            );
        }
    }

    #[test]
    fn extracted_pieces_tile_the_knot_span(curve in arb_curve()) {
        let pieces = extract_bezier_pieces(&curve);
        let breaks = distinct_knots(&curve);
        let p = curve.degree() as usize;

        prop_assert_eq!(pieces.len(), breaks.len() - 1);
        for (piece, window) in pieces.iter().zip(breaks.windows(2)) {
            prop_assert_eq!(piece.u_start, window[0]);
            prop_assert_eq!(piece.u_end, window[1]);
            prop_assert_eq!(piece.degree(), p);
        }
        for pair in pieces.windows(2) {
            prop_assert_eq!(pair[0].u_end, pair[1].u_start);
        }
    }

    #[test]
    fn recomposed_curve_has_bezier_knot_structure(curve in arb_curve()) {
        let pieces = extract_bezier_pieces(&curve);
        let recomposed = bezier_pieces_to_nurbs(&pieces);
        let p = curve.degree() as usize;
        let knots = recomposed.knots();

        prop_assert_eq!(recomposed.degree(), curve.degree());
        prop_assert_eq!(knots.len(), recomposed.control_points().len() + p + 1);
        prop_assert_eq!(distinct_knots(&recomposed), distinct_knots(&curve));
        for interior in distinct_knots(&recomposed)
            .into_iter()
            .skip(1)
            .take(pieces.len() - 1)
        {
            let multiplicity = knots.iter().filter(|k| **k == interior).count();
            prop_assert_eq!(multiplicity, p, "interior knot {} multiplicity", interior);
        }
        prop_assert_eq!(knots.iter().filter(|k| **k == knots[0]).count(), p + 1);
        let last = knots[knots.len() - 1];
        prop_assert_eq!(knots.iter().filter(|k| **k == last).count(), p + 1);
    }

    #[test]
    fn to_bernstein_matches_the_bernstein_basis_sum(
        piece in arb_piece(),
        fractions in prop::collection::vec(0.0..=1.0_f64, 32),
    ) {
        let bernstein = piece.to_bernstein();
        let h = piece.u_end - piece.u_start;
        let reach = monomial_reach(&piece.coeffs, h);
        let budget = 32.0 * (piece.coeffs.len() as f64) * f64::EPSILON * reach;

        prop_assert_eq!(bernstein.len(), piece.coeffs.len());
        for s in fractions {
            let want = piece.evaluate(piece.u_start + s * h);
            let got = bernstein_sum(&bernstein, s);
            prop_assert!(
                (got - want).abs() <= budget,
                "s={s}: {got} vs {want} (budget {budget}), coeffs {:?}",
                piece.coeffs
            );
        }
    }

    #[test]
    fn from_bernstein_matches_the_bernstein_basis_sum(
        (bernstein, u_start, u_end) in arb_bernstein(),
        fractions in prop::collection::vec(0.0..=1.0_f64, 32),
    ) {
        let piece = BezierPiece::from_bernstein(&bernstein, u_start, u_end);
        prop_assert_eq!(piece.u_start, u_start);
        prop_assert_eq!(piece.u_end, u_end);
        prop_assert_eq!(piece.coeffs.len(), bernstein.len());

        let d = bernstein.len() - 1;
        let bernstein_reach: f64 = bernstein.iter().map(|b| b.abs()).sum();
        let inverse_gain = 3.0_f64.powi(d as i32);
        let budget = 32.0 * f64::EPSILON * inverse_gain * bernstein_reach;
        let h = u_end - u_start;
        for s in fractions {
            let want = bernstein_sum(&bernstein, s);
            let got = piece.evaluate(u_start + s * h);
            prop_assert!(
                (got - want).abs() <= budget,
                "s={s}: {got} vs {want} (budget {budget}), bernstein {bernstein:?}"
            );
        }
    }

    #[test]
    fn bernstein_round_trip_preserves_the_polynomial(
        piece in arb_piece(),
        fractions in prop::collection::vec(0.0..=1.0_f64, 32),
    ) {
        let bernstein = piece.to_bernstein();
        let back = BezierPiece::from_bernstein(&bernstein, piece.u_start, piece.u_end);
        prop_assert_eq!(back.coeffs.len(), piece.coeffs.len());
        prop_assert_eq!(back.u_start, piece.u_start);
        prop_assert_eq!(back.u_end, piece.u_end);

        let h = piece.u_end - piece.u_start;
        let d = piece.degree();
        let reach = monomial_reach(&piece.coeffs, h);
        let budget = 32.0 * f64::EPSILON * 3.0_f64.powi(d as i32) * reach;
        for s in fractions {
            let u = piece.u_start + s * h;
            let want = piece.evaluate(u);
            let got = back.evaluate(u);
            prop_assert!(
                (got - want).abs() <= budget,
                "s={s}: {got} vs {want} (budget {budget}), coeffs {:?}",
                piece.coeffs
            );
        }
    }

    #[test]
    fn split_halves_reproduce_the_parent_polynomial(
        piece in arb_piece(),
        split_fraction in 0.05..0.95_f64,
        fractions in prop::collection::vec(0.0..=1.0_f64, 32),
    ) {
        let h = piece.u_end - piece.u_start;
        let u_split = piece.u_start + split_fraction * h;
        let (left, right) = split_piece_at(&piece, u_split);

        prop_assert_eq!(left.u_start, piece.u_start);
        prop_assert_eq!(left.u_end, u_split);
        prop_assert_eq!(right.u_start, u_split);
        prop_assert_eq!(right.u_end, piece.u_end);
        prop_assert_eq!(left.degree(), piece.degree());
        prop_assert_eq!(right.degree(), piece.degree());

        let reach = monomial_reach(&piece.coeffs, h);
        let budget = 32.0 * (piece.coeffs.len() as f64) * f64::EPSILON * reach;
        for s in fractions {
            let u_left = left.u_start + s * (left.u_end - left.u_start);
            let want_left = piece.evaluate(u_left);
            let got_left = left.evaluate(u_left);
            prop_assert!(
                (got_left - want_left).abs() <= budget,
                "left s={s}: {got_left} vs {want_left} (budget {budget})"
            );

            let u_right = right.u_start + s * (right.u_end - right.u_start);
            let want_right = piece.evaluate(u_right);
            let got_right = right.evaluate(u_right);
            prop_assert!(
                (got_right - want_right).abs() <= budget,
                "right s={s}: {got_right} vs {want_right} (budget {budget})"
            );
        }
    }

    #[test]
    fn split_right_coefficients_agree_with_the_taylor_shift(
        piece in arb_piece(),
        split_fraction in 0.05..0.95_f64,
    ) {
        let h = piece.u_end - piece.u_start;
        let u_split = piece.u_start + split_fraction * h;
        let (_, right) = split_piece_at(&piece, u_split);
        let shifted = taylor_shift(&piece.coeffs, u_split - piece.u_start);

        let delta = u_split - piece.u_start;
        for (i, (from_split, from_shift)) in right.coeffs.iter().zip(&shifted).enumerate() {
            let term_reach: f64 = piece
                .coeffs
                .iter()
                .enumerate()
                .skip(i)
                .map(|(k, c)| c.abs() * choose(k, i) * delta.powi((k - i) as i32))
                .sum();
            let budget = 32.0 * f64::EPSILON * term_reach;
            prop_assert!(
                (from_split - from_shift).abs() <= budget,
                "coeff {i}: {from_split} vs {from_shift} (budget {budget})"
            );
        }
    }
}
