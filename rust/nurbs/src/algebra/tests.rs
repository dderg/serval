use super::*;

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

fn boxed(height: f64, support: (f64, f64)) -> PiecewisePolynomialKernel {
    PiecewisePolynomialKernel::single_poly_from_absolute(vec![height], support)
}

#[test]
fn one_kernel_varies_by_its_total_variation_jumps_included() {
    let parabola =
        PiecewisePolynomialKernel::single_poly_from_absolute(vec![0.75, 0.0, -0.75], (-1.0, 1.0));
    let variation = PiecewisePolynomialKernel::directional_variation(&[&parabola]);
    assert!((variation - 1.5).abs() < 1e-5, "{variation}");
    let flat = boxed(0.5, (-1.0, 1.0));
    assert!((PiecewisePolynomialKernel::directional_variation(&[&flat]) - 1.0).abs() < 1e-12);
}

#[test]
fn a_bimodal_kernel_varies_by_more_than_twice_its_peak() {
    let two_humps = PiecewisePolynomialKernel::from_pieces(vec![
        crate::bezier::BezierPiece {
            u_start: -1.0,
            u_end: 0.0,
            coeffs: vec![1.0],
        },
        crate::bezier::BezierPiece {
            u_start: 0.0,
            u_end: 0.5,
            coeffs: vec![0.25],
        },
        crate::bezier::BezierPiece {
            u_start: 0.5,
            u_end: 1.0,
            coeffs: vec![1.0],
        },
    ])
    .unwrap();
    let variation = PiecewisePolynomialKernel::directional_variation(&[&two_humps]);
    assert!((variation - 3.5).abs() < 1e-12, "{variation}");
}

#[test]
fn identical_kernels_vary_together_like_one() {
    let parabola =
        PiecewisePolynomialKernel::single_poly_from_absolute(vec![0.75, 0.0, -0.75], (-1.0, 1.0));
    let alone = PiecewisePolynomialKernel::directional_variation(&[&parabola]);
    let together = PiecewisePolynomialKernel::directional_variation(&[&parabola, &parabola]);
    assert!(
        (together - alone).abs() < 1e-9 * alone,
        "{together} vs {alone}"
    );
}

#[test]
fn kernels_whose_swings_fall_apart_vary_along_the_diagonal_by_more_than_either() {
    let early = boxed(1.0, (-1.0, 0.0));
    let late = boxed(1.0, (0.0, 1.0));
    let variation = PiecewisePolynomialKernel::directional_variation(&[&early, &late]);
    let diagonal = 1.0 + 2.0_f64.sqrt();
    assert!(
        (variation - diagonal).abs() < 1e-9,
        "{variation} vs {diagonal}"
    );
}

#[test]
fn a_box_kernel_keeps_its_tail_moments_and_its_edges_as_variation() {
    let reach = 0.02;
    let height = 1.0 / (2.0 * reach);
    let kernel = PiecewisePolynomialKernel::single_poly(vec![height], (-reach, reach));
    for ahead in [true, false] {
        let past = 0.005;
        let left = reach - past;
        let tail = kernel.tail(past, ahead);
        let close = |got: f64, want: f64| (got - want).abs() <= 1e-5 * want;
        assert!(close(tail.mass, height * left), "{tail:?}");
        assert!(
            close(tail.first_moment_s, height * left * left / 2.0),
            "{tail:?}"
        );
        assert!(
            close(tail.half_second_moment_s2, height * left.powi(3) / 6.0),
            "{tail:?}"
        );
        assert!(close(tail.variation_per_s, 2.0 * height), "{tail:?}");
        let empty = kernel.tail(reach, ahead);
        assert_eq!(
            empty,
            KernelTail {
                mass: 0.0,
                first_moment_s: 0.0,
                half_second_moment_s2: 0.0,
                variation_per_s: 0.0,
            }
        );
    }
}
