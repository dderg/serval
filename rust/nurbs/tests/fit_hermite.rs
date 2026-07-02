#![allow(clippy::cast_lossless, clippy::cast_possible_wrap)]

use nurbs::algebra::fit_hermite_c1_clamped;
use nurbs::bezier::BezierPiece;

fn d2_at(piece: &BezierPiece<f64>, u: f64) -> f64 {
    piece.differentiate().differentiate().evaluate(u)
}

#[test]
fn clamped_fit_pins_both_boundary_second_derivatives() {
    let pieces: Vec<[BezierPiece<f64>; 1]> = vec![[BezierPiece {
        u_start: 0.0,
        u_end: 2.0,
        coeffs: vec![0.0, 0.0, 0.0, 1.0],
    }]];

    let d2_start_pin = 6.0_f64;
    let d2_end_pin = 10.0_f64;

    let result =
        fit_hermite_c1_clamped::<1>(&pieces, 2.0, 5, [d2_start_pin], Some([d2_end_pin])).unwrap();

    let first = &result[0][0];
    let last = result[0].last().unwrap();

    let got_start = d2_at(first, first.u_start);
    let got_end = d2_at(last, last.u_end);

    assert!(
        (got_start - d2_start_pin).abs() < 1e-6,
        "d2 at start: expected {d2_start_pin}, got {got_start}"
    );
    assert!(
        (got_end - d2_end_pin).abs() < 1e-6,
        "d2 at end: expected {d2_end_pin}, got {got_end}"
    );
}

#[test]
fn clamped_fit_position_residual_within_tolerance() {
    let pieces: Vec<[BezierPiece<f64>; 1]> = (0..3)
        .map(|i| {
            let s = i as f64;
            [BezierPiece {
                u_start: s,
                u_end: s + 1.0,
                coeffs: vec![s * s + 0.5 * s, 2.0 * s + 0.5, 1.0],
            }]
        })
        .collect();

    let tol = 0.01;
    let result =
        fit_hermite_c1_clamped::<1>(&pieces, tol, 5, [2.0_f64], Some([2.0_f64])).unwrap();

    for fitted in &result[0] {
        let n = 40;
        let step = (fitted.u_end - fitted.u_start) / n as f64;
        for i in 0..=n {
            let u = fitted.u_start + i as f64 * step;
            let ref_val = pieces
                .iter()
                .find(|p| p[0].u_start <= u + 1e-12 && u <= p[0].u_end + 1e-12)
                .map(|p| p[0].evaluate(u))
                .unwrap_or_else(|| {
                    pieces.last().unwrap()[0].evaluate(pieces.last().unwrap()[0].u_end)
                });
            let fit_val = fitted.evaluate(u);
            assert!(
                (ref_val - fit_val).abs() <= tol + 1e-10,
                "at u={u}: residual {} exceeds tolerance {tol}",
                (ref_val - fit_val).abs()
            );
        }
    }
}

#[test]
fn clamped_fit_preserves_c1_at_interior_knots() {
    let pieces: Vec<[BezierPiece<f64>; 1]> = (0..4)
        .map(|i| {
            let s = i as f64;
            [BezierPiece {
                u_start: s,
                u_end: s + 1.0,
                coeffs: vec![s * s, 2.0 * s, 1.0],
            }]
        })
        .collect();

    let result =
        fit_hermite_c1_clamped::<1>(&pieces, 0.5, 5, [2.0_f64], Some([10.0_f64])).unwrap();

    for window in result[0].windows(2) {
        let left = &window[0];
        let right = &window[1];
        let left_val = left.evaluate(left.u_end);
        let right_val = right.evaluate(right.u_start);
        assert!(
            (left_val - right_val).abs() < 1e-9,
            "C0 violated at interior knot {}: {left_val} vs {right_val}",
            left.u_end
        );
        let left_d = left.differentiate().evaluate(left.u_end);
        let right_d = right.differentiate().evaluate(right.u_start);
        assert!(
            (left_d - right_d).abs() < 1e-7,
            "C1 violated at interior knot {}: {left_d} vs {right_d}",
            left.u_end
        );
    }
}

#[test]
fn clamped_fit_2d_pins_both_axes() {
    let pieces: Vec<[BezierPiece<f64>; 2]> = vec![[
        BezierPiece {
            u_start: 0.0,
            u_end: 2.0,
            coeffs: vec![0.0, 0.0, 1.0],
        },
        BezierPiece {
            u_start: 0.0,
            u_end: 2.0,
            coeffs: vec![0.0, 2.0],
        },
    ]];

    let result = fit_hermite_c1_clamped::<2>(
        &pieces,
        0.05,
        5,
        [2.0_f64, 0.0_f64],
        Some([2.0_f64, 0.0_f64]),
    )
    .unwrap();

    let x_first = &result[0][0];
    let x_last = result[0].last().unwrap();
    let y_first = &result[1][0];
    let y_last = result[1].last().unwrap();

    assert!(
        (d2_at(x_first, x_first.u_start) - 2.0).abs() < 1e-6,
        "x d2 at start: got {}",
        d2_at(x_first, x_first.u_start)
    );
    assert!(
        (d2_at(x_last, x_last.u_end) - 2.0).abs() < 1e-6,
        "x d2 at end: got {}",
        d2_at(x_last, x_last.u_end)
    );
    assert!(
        d2_at(y_first, y_first.u_start).abs() < 1e-6,
        "y d2 at start: got {}",
        d2_at(y_first, y_first.u_start)
    );
    assert!(
        d2_at(y_last, y_last.u_end).abs() < 1e-6,
        "y d2 at end: got {}",
        d2_at(y_last, y_last.u_end)
    );
}

#[test]
fn adversarial_nonpolynomial_asymmetric_pins() {
    let n = 8;
    let two_pi = std::f64::consts::TAU;

    let pieces: Vec<[BezierPiece<f64>; 1]> = (0..n)
        .map(|i| {
            let u0 = i as f64 * two_pi / n as f64;
            let u1 = (i + 1) as f64 * two_pi / n as f64;
            let h = u1 - u0;
            let f0 = u0.sin();
            let df0 = u0.cos();
            let f1 = u1.sin();
            let df1 = u1.cos();
            let det = h * h * h * h;
            let c2 = (3.0 * h * h * (f1 - f0 - df0 * h) - h * h * h * (df1 - df0)) / det;
            let c3 = (h * h * (df1 - df0) - 2.0 * h * (f1 - f0 - df0 * h)) / det;
            [BezierPiece {
                u_start: u0,
                u_end: u1,
                coeffs: vec![f0, df0, c2, c3, 0.0, 0.0, 0.0],
            }]
        })
        .collect();

    let d2_start_pin = 0.5_f64;
    let d2_end_pin = -0.8_f64;

    let tol = 0.15;
    let result =
        fit_hermite_c1_clamped::<1>(&pieces, tol, 5, [d2_start_pin], Some([d2_end_pin]))
            .expect("adversarial clamped fit must succeed");

    let first = &result[0][0];
    let last = result[0].last().unwrap();
    let got_start = d2_at(first, first.u_start);
    let got_end = d2_at(last, last.u_end);
    assert!(
        (got_start - d2_start_pin).abs() < 1e-6,
        "ADVERSARIAL: d2 at global start: expected {d2_start_pin}, got {got_start}"
    );
    assert!(
        (got_end - d2_end_pin).abs() < 1e-6,
        "ADVERSARIAL: d2 at global end: expected {d2_end_pin}, got {got_end}"
    );

    for window in result[0].windows(2) {
        let left = &window[0];
        let right = &window[1];
        let left_val = left.evaluate(left.u_end);
        let right_val = right.evaluate(right.u_start);
        assert!(
            (left_val - right_val).abs() < 1e-9,
            "ADVERSARIAL: C0 violated at interior knot {}: {} vs {}",
            left.u_end,
            left_val,
            right_val
        );
        let left_d = left.differentiate().evaluate(left.u_end);
        let right_d = right.differentiate().evaluate(right.u_start);
        assert!(
            (left_d - right_d).abs() < 1e-7,
            "ADVERSARIAL: C1 violated at interior knot {}: {} vs {}",
            left.u_end,
            left_d,
            right_d
        );
    }

    for fitted_piece in &result[0] {
        let n_samples = 40;
        let step = (fitted_piece.u_end - fitted_piece.u_start) / n_samples as f64;
        for i in 0..=n_samples {
            let u = fitted_piece.u_start + i as f64 * step;
            let ref_val = pieces
                .iter()
                .find(|p| p[0].u_start <= u + 1e-12 && u <= p[0].u_end + 1e-12)
                .map(|p| p[0].evaluate(u))
                .unwrap_or_else(|| {
                    pieces.last().unwrap()[0].evaluate(pieces.last().unwrap()[0].u_end)
                });
            let fit_val = fitted_piece.evaluate(u);
            assert!(
                (ref_val - fit_val).abs() <= tol + 1e-10,
                "ADVERSARIAL: at u={u}: residual {} exceeds tolerance {tol}",
                (ref_val - fit_val).abs()
            );
        }
    }

    assert!(
        d2_start_pin.abs() > 0.1 && d2_end_pin.abs() > 0.1,
        "pins are non-trivial by construction"
    );
    assert!(
        (d2_start_pin - d2_end_pin).abs() > 0.5,
        "pins are asymmetric by construction"
    );

    assert_eq!(
        first.coeffs.len(),
        6,
        "ADVERSARIAL: first (start-pinned) piece must be degree-5 (6 coeffs), got {}",
        first.coeffs.len()
    );
    assert_eq!(
        last.coeffs.len(),
        6,
        "ADVERSARIAL: last (end-pinned) piece must be degree-5 (6 coeffs), got {}",
        last.coeffs.len()
    );
}
