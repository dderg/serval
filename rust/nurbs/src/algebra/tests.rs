use super::*;
use crate::eval::eval;

#[test]
fn convolve_linear_input_with_constant_kernel_yields_correct_integral() {
    let x =
        crate::ScalarNurbs::<f64>::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0]).unwrap();
    let kernel = PiecewisePolynomialKernel::single_poly(vec![1.0_f64], (-0.25, 0.25));

    let y = convolve(&x, &kernel).unwrap();

    let val = eval(&y.as_view(), 0.5);
    assert!((val - 0.25).abs() < 1e-10, "y(0.5) = {val}, expected 0.25");
}

#[test]
fn breakpoint_sort_handles_nan_without_panicking() {
    let mut out_breaks = [0.0_f64, f64::NAN, 1.0];
    out_breaks.sort_by(|a, b| <f64 as crate::Float>::total_cmp(*a, *b));
    assert_eq!(out_breaks.len(), 3);
}

#[test]
fn convolve_constant_input_with_constant_kernel_gives_triangle() {
    let x =
        crate::ScalarNurbs::<f64>::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![2.0, 2.0]).unwrap();
    let kernel = PiecewisePolynomialKernel::single_poly(vec![3.0_f64], (-0.5, 0.5));

    let y = convolve(&x, &kernel).unwrap();

    let val = eval(&y.as_view(), 0.5);
    assert!((val - 6.0).abs() < 1e-10, "y(0.5) = {val}, expected 6");

    let val_lo = eval(&y.as_view(), -0.5);
    assert!(val_lo.abs() < 1e-10, "y(-0.5) = {val_lo}, expected 0");

    let val_hi = eval(&y.as_view(), 1.5);
    assert!(val_hi.abs() < 1e-10, "y(1.5) = {val_hi}, expected 0");
}

#[test]
fn integrate_product_constant_input_constant_kernel_yields_linear_result() {
    let x = crate::bezier::BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![2.0],
    };
    let w = crate::bezier::BezierPiece::<f64> {
        u_start: -0.5,
        u_end: 0.5,
        coeffs: vec![3.0],
    };

    let contribution = integrate_product_piece(&x, &w, 0.5, 1.0);

    assert!((contribution.coeffs[0] - 6.0).abs() < 1e-10);
    assert!((contribution.coeffs[1] - (-6.0)).abs() < 1e-10);
}

#[test]
fn single_poly_kernel_constructs_one_piece() {
    let k = PiecewisePolynomialKernel::single_poly(vec![1.0, 0.5_f64], (-1.0, 1.0));
    assert_eq!(k.pieces.len(), 1);
    assert_eq!(k.pieces[0].u_start, -1.0);
    assert_eq!(k.pieces[0].u_end, 1.0);
    assert_eq!(k.pieces[0].coeffs, vec![1.0, 0.5]);
}

#[test]
fn kernel_support_returns_endpoints() {
    let k = PiecewisePolynomialKernel::single_poly(vec![1.0_f64], (-0.5, 0.5));
    assert_eq!(k.support(), (-0.5, 0.5));
}

#[test]
fn knot_remove_redundant_simplifies_overproduct() {
    let a =
        crate::ScalarNurbs::<f64>::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0]).unwrap();
    let b = a.clone();
    let mut c = multiply(&a, &b).unwrap();
    let initial_knot_count = c.knots().len();

    knot_remove_redundant(&mut c, 1e-10);

    assert_eq!(c.knots().len(), initial_knot_count);
    for u in [0.0, 0.5, 1.0] {
        let exp = eval(&a.as_view(), u) * eval(&b.as_view(), u);
        let got = eval(&c.as_view(), u);
        assert!((exp - got).abs() < 1e-10);
    }
}

#[test]
fn multiply_curves_with_different_interior_knots() {
    let a = crate::ScalarNurbs::<f64>::try_new(
        2,
        vec![0.0, 0.0, 0.0, 0.4, 1.0, 1.0, 1.0],
        vec![0.0, 1.0, 2.0, 3.0],
    )
    .unwrap();
    let b = crate::ScalarNurbs::<f64>::try_new(
        2,
        vec![0.0, 0.0, 0.0, 0.7, 1.0, 1.0, 1.0],
        vec![1.0, 2.0, 0.0, 1.0],
    )
    .unwrap();
    let c = multiply(&a, &b).unwrap();
    assert_eq!(c.degree(), 4);
    for u in [0.0, 0.2, 0.4, 0.5, 0.7, 0.9, 1.0] {
        let exp = eval(&a.as_view(), u) * eval(&b.as_view(), u);
        let got = eval(&c.as_view(), u);
        assert!((exp - got).abs() < 1e-10, "u={u}: exp={exp}, got={got}");
    }
}

#[test]
fn multiply_two_linear_curves_gives_quadratic() {
    let a =
        crate::ScalarNurbs::<f64>::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0]).unwrap();
    let b =
        crate::ScalarNurbs::<f64>::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![1.0, 3.0]).unwrap();
    let c = multiply(&a, &b).unwrap();
    assert_eq!(c.degree(), 2);
    for u in [0.0, 0.25, 0.5, 0.75, 1.0] {
        let exp = eval(&a.as_view(), u) * eval(&b.as_view(), u);
        let got = eval(&c.as_view(), u);
        assert!((exp - got).abs() < 1e-12, "u={u}: exp={exp}, got={got}");
    }
}

#[test]
fn scalar_multiply_doubles_evaluation() {
    let curve = crate::ScalarNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0]).unwrap();
    let doubled = scalar_multiply(&curve, 2.0_f64);
    assert!((eval(&doubled.as_view(), 0.5_f64) - 1.0).abs() < 1e-12);
}

#[test]
fn add_two_compatible_curves() {
    let a = crate::ScalarNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0]).unwrap();
    let b = crate::ScalarNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![2.0, 3.0]).unwrap();
    let sum = add(&a, &b).unwrap();
    assert!((eval(&sum.as_view(), 0.5_f64) - 3.0).abs() < 1e-12);
}

#[test]
fn add_rejects_mismatched_degree() {
    let a = crate::ScalarNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0]).unwrap();
    let b = crate::ScalarNurbs::try_new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![0.0, 0.5, 1.0])
        .unwrap();
    let result = add(&a, &b);
    assert!(matches!(result, Err(crate::AlgebraError::KnotMismatch)));
}

#[test]
fn from_pieces_accepts_contiguous_kernel() {
    let pieces = vec![
        crate::bezier::BezierPiece {
            u_start: -0.5,
            u_end: 0.0,
            coeffs: vec![1.0_f64],
        },
        crate::bezier::BezierPiece {
            u_start: 0.0,
            u_end: 0.5,
            coeffs: vec![2.0_f64],
        },
    ];
    let k = PiecewisePolynomialKernel::from_pieces(pieces).unwrap();
    assert_eq!(k.pieces.len(), 2);
    assert_eq!(k.support(), (-0.5, 0.5));
}

#[test]
fn from_pieces_rejects_non_contiguous() {
    let pieces = vec![
        crate::bezier::BezierPiece {
            u_start: -0.5_f64,
            u_end: 0.0,
            coeffs: vec![1.0],
        },
        crate::bezier::BezierPiece {
            u_start: 0.1,
            u_end: 0.5,
            coeffs: vec![2.0],
        },
    ];
    let result = PiecewisePolynomialKernel::from_pieces(pieces);
    assert!(matches!(result, Err(AlgebraError::SupportMismatch)));
}

#[test]
fn from_pieces_rejects_empty() {
    let result = PiecewisePolynomialKernel::<f64>::from_pieces(vec![]);
    assert!(matches!(result, Err(AlgebraError::SupportMismatch)));
}

#[test]
fn pascal_shift_round_trip_preserves_polynomial() {
    let coeffs = vec![1.0, 2.0, 3.0, -1.5_f64];
    let shift = 0.7;
    let absolute = pascal_shift_to_absolute(&coeffs, shift);
    let back = absolute_to_pascal_shift(&absolute, shift);
    for i in 0..coeffs.len() {
        assert!(
            (back[i] - coeffs[i]).abs() < 1e-12,
            "coeff[{i}]: original {} != round-tripped {}",
            coeffs[i],
            back[i],
        );
    }
}

#[test]
fn single_poly_from_absolute_constructs_kernel_with_correct_polynomial() {
    let k = PiecewisePolynomialKernel::single_poly_from_absolute(vec![1.0_f64, 2.0], (0.5, 1.5));
    assert_eq!(k.pieces.len(), 1);
    assert_eq!(k.pieces[0].u_start, 0.5);
    assert_eq!(k.pieces[0].u_end, 1.5);
    assert!((k.pieces[0].coeffs[0] - 2.0).abs() < 1e-12);
    assert!((k.pieces[0].coeffs[1] - 2.0).abs() < 1e-12);
    let val_at_one = k.pieces[0].evaluate(1.0);
    assert!((val_at_one - 3.0).abs() < 1e-12);
}

#[test]
fn single_poly_from_absolute_round_trips_via_evaluate() {
    let k =
        PiecewisePolynomialKernel::single_poly_from_absolute(vec![1.0_f64, -2.0, 3.0], (-0.5, 0.5));
    for t in [-0.5_f64, 0.0, 0.25, 0.5] {
        let absolute_val = 1.0 - 2.0 * t + 3.0 * t * t;
        let pascal_val = k.pieces[0].evaluate(t);
        assert!(
            (absolute_val - pascal_val).abs() < 1e-12,
            "t={t}: absolute={absolute_val}, pascal={pascal_val}"
        );
    }
}

#[test]
fn multiply_regression_proptest_shrunk_failing_input() {
    let a = crate::ScalarNurbs::<f64>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 0.1, 0.55, 1.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 0.0, 0.181_828_016_839_598_23, 0.0, 0.0],
    )
    .unwrap();
    let b = crate::ScalarNurbs::<f64>::try_new(
        1,
        vec![0.0, 0.0, 0.1, 1.0, 1.0],
        vec![0.0, 4.267_190_258_636_853, 0.0],
    )
    .unwrap();
    let c = multiply(&a, &b).unwrap();
    let exp = eval(&a.as_view(), 0.1) * eval(&b.as_view(), 0.1);
    let got = eval(&c.as_view(), 0.1);
    assert!(
        (exp - got).abs() < 1e-10,
        "u=0.1: pointwise={exp}, multiply={got} (regression)"
    );
}

#[test]
fn multiply_quadratic_x_linear_gives_cubic() {
    let a = crate::ScalarNurbs::<f64>::try_new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0],
    )
    .unwrap();
    let b =
        crate::ScalarNurbs::<f64>::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0]).unwrap();
    let c = multiply(&a, &b).unwrap();
    assert_eq!(c.degree(), 3);
    for u in [0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0] {
        let exp = u * u * u;
        let got = eval(&c.as_view(), u);
        assert!(
            (exp - got).abs() < 1e-12,
            "u={u}: u^3={exp}, multiply={got}"
        );
    }
}

#[test]
fn convolve_multi_piece_input_with_c0_kink_preserves_natural_multiplicity() {
    let x = crate::ScalarNurbs::<f64>::try_new(
        2,
        vec![0.0, 0.0, 0.0, 0.3, 0.3, 1.0, 1.0, 1.0],
        vec![0.0, 1.0, 4.0, 0.5, 0.2],
    )
    .unwrap();
    let kernel = PiecewisePolynomialKernel::single_poly(vec![1.0_f64], (-0.1, 0.1));

    let y = convolve(&x, &kernel).unwrap();

    assert_eq!(y.degree(), 3, "output degree");

    let p = y.degree() as usize;
    let interior = &y.knots()[p + 1..y.knots().len() - p - 1];
    let mult_at_02 = interior.iter().filter(|k| (**k - 0.2).abs() < 1e-9).count();
    let mult_at_04 = interior.iter().filter(|k| (**k - 0.4).abs() < 1e-9).count();
    assert_eq!(
        mult_at_02, 2,
        "expected μ_y(0.2) = m_x = 2 (kink image), got {mult_at_02}; full interior = {interior:?}",
    );
    assert_eq!(
        mult_at_04, 2,
        "expected μ_y(0.4) = m_x = 2 (kink image), got {mult_at_04}; full interior = {interior:?}",
    );

    let exp_02 = 0.8 / 3.0 + 5.2 / 27.0;
    let exp_04 = 0.6 + 2.56 / 147.0;
    let got_02 = eval(&y.as_view(), 0.2);
    let got_04 = eval(&y.as_view(), 0.4);
    assert!(
        (got_02 - exp_02).abs() < 1e-10,
        "y(0.2): expected {exp_02}, got {got_02}, diff {}",
        (got_02 - exp_02).abs(),
    );
    assert!(
        (got_04 - exp_04).abs() < 1e-10,
        "y(0.4): expected {exp_04}, got {got_04}, diff {}",
        (got_04 - exp_04).abs(),
    );

    let exp_05 = 0.4 + 8.32 / 147.0;
    let got_05 = eval(&y.as_view(), 0.5);
    assert!(
        (got_05 - exp_05).abs() < 1e-10,
        "y(0.5): expected {exp_05}, got {got_05}, diff {}",
        (got_05 - exp_05).abs(),
    );
}

#[test]
fn add_with_knot_union_identical_knots_fast_path() {
    let a = crate::ScalarNurbs::try_new(1, vec![0.0_f64, 0.0, 1.0, 1.0], vec![0.0, 1.0]).unwrap();
    let b = crate::ScalarNurbs::try_new(1, vec![0.0_f64, 0.0, 1.0, 1.0], vec![2.0, 3.0]).unwrap();
    let sum = add_with_knot_union(&a, &b).unwrap();
    assert!(
        (eval(&sum.as_view(), 0.0_f64) - 2.0).abs() < 1e-12,
        "fast-path u=0"
    );
    assert!(
        (eval(&sum.as_view(), 0.5_f64) - 3.0).abs() < 1e-12,
        "fast-path u=0.5"
    );
    assert!(
        (eval(&sum.as_view(), 1.0_f64) - 4.0).abs() < 1e-12,
        "fast-path u=1"
    );
}

#[test]
fn add_with_knot_union_mismatched_knots_union_path() {
    use crate::bezier::{BezierPiece, bezier_pieces_to_nurbs};

    let a = bezier_pieces_to_nurbs(&[
        BezierPiece::<f64> {
            u_start: 0.0,
            u_end: 0.5,
            coeffs: vec![0.0, 10.0],
        },
        BezierPiece::<f64> {
            u_start: 0.5,
            u_end: 1.0,
            coeffs: vec![5.0, 10.0],
        },
    ]);
    let b = crate::ScalarNurbs::try_new(1, vec![0.0_f64, 0.0, 1.0, 1.0], vec![20.0, 20.0]).unwrap();

    let sum = add_with_knot_union(&a, &b).unwrap();
    let cases = [
        (0.0_f64, 20.0),
        (0.25, 22.5),
        (0.5, 25.0),
        (0.75, 27.5),
        (1.0, 30.0),
    ];
    for (u, expected) in cases {
        let got = eval(&sum.as_view(), u);
        assert!(
            (got - expected).abs() < 1e-10,
            "union-path u={u}: expected {expected}, got {got}",
        );
    }
}

#[test]
fn add_with_knot_union_doc_example() {
    use crate::ScalarNurbs;
    let x =
        ScalarNurbs::try_new(1, vec![0.0_f64, 0.0, 0.5, 1.0, 1.0], vec![0.0, 5.0, 10.0]).unwrap();
    let y = ScalarNurbs::try_new(1, vec![0.0_f64, 0.0, 1.0, 1.0], vec![20.0, 20.0]).unwrap();
    let sum = add_with_knot_union(&x, &y).unwrap();
    let v0 = crate::eval::eval(&sum.as_view(), 0.0_f64);
    let v1 = crate::eval::eval(&sum.as_view(), 1.0_f64);
    assert!((v0 - 20.0).abs() < 1e-12);
    assert!((v1 - 30.0).abs() < 1e-12);
}

#[test]
fn add_with_knot_union_rejects_degree_mismatch() {
    let a = crate::ScalarNurbs::try_new(1, vec![0.0_f64, 0.0, 1.0, 1.0], vec![0.0, 1.0]).unwrap();
    let b = crate::ScalarNurbs::try_new(
        2,
        vec![0.0_f64, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.5, 1.0],
    )
    .unwrap();
    let result = add_with_knot_union(&a, &b);
    assert!(
        matches!(result, Err(crate::AlgebraError::KnotMismatch)),
        "expected KnotMismatch, got {result:?}",
    );
}
