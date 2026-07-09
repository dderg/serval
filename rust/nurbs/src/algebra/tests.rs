use super::*;
use crate::eval::eval;

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
    let result = PiecewisePolynomialKernel::from_pieces(vec![]);
    assert!(matches!(result, Err(AlgebraError::SupportMismatch)));
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
        BezierPiece {
            u_start: 0.0,
            u_end: 0.5,
            coeffs: vec![0.0, 10.0],
        },
        BezierPiece {
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

#[test]
fn second_moment_integrates_a_single_polynomial_piece_exactly() {
    let uniform = PiecewisePolynomialKernel::single_poly_from_absolute(vec![0.5], (-1.0, 1.0));
    assert!((uniform.second_moment() - 1.0 / 3.0).abs() < 1e-12);

    let parabola =
        PiecewisePolynomialKernel::single_poly_from_absolute(vec![0.75, 0.0, -0.75], (-1.0, 1.0));
    assert!((parabola.second_moment() - 0.2).abs() < 1e-12);
}

#[test]
fn second_moment_sums_over_pieces() {
    let split = PiecewisePolynomialKernel::from_pieces(vec![
        crate::bezier::BezierPiece {
            u_start: -1.0,
            u_end: 0.25,
            coeffs: vec![0.5],
        },
        crate::bezier::BezierPiece {
            u_start: 0.25,
            u_end: 1.0,
            coeffs: vec![0.5],
        },
    ])
    .unwrap();
    assert!((split.second_moment() - 1.0 / 3.0).abs() < 1e-12);
}
