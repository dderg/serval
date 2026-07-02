#[cfg(feature = "host")]
#[derive(Debug, Clone, PartialEq)]
pub enum FitError {
    ToleranceNotReached { achieved_mm: f64, at_degree: u8 },
    DegenerateInput { reason: &'static str },
}

#[cfg(feature = "host")]
pub fn fit_hermite_c1_clamped<const D: usize>(
    pieces: &[[crate::bezier::BezierPiece<f64>; D]],
    tolerance_mm: f64,
    target_degree: u8,
    d2_start: [f64; D],
    d2_end: Option<[f64; D]>,
) -> Result<[Vec<crate::bezier::BezierPiece<f64>>; D], FitError> {
    if pieces.is_empty() {
        return Err(FitError::DegenerateInput {
            reason: "fit_hermite_c1_clamped: empty input",
        });
    }
    if !tolerance_mm.is_finite() || tolerance_mm <= 0.0 {
        return Err(FitError::DegenerateInput {
            reason: "fit_hermite_c1_clamped: tolerance must be finite and positive",
        });
    }
    if target_degree < 4 {
        return Err(FitError::DegenerateInput {
            reason: "fit_hermite_c1_clamped: target_degree must be >= 4 when d2 pins are supplied",
        });
    }

    for w in pieces.windows(2) {
        for axis in 0..D {
            if (w[0][axis].u_end - w[1][axis].u_start).abs() > 1e-12 {
                return Err(FitError::DegenerateInput {
                    reason: "fit_hermite_c1_clamped: non-contiguous input pieces",
                });
            }
        }
    }

    let n = pieces.len();
    let mut result: [Vec<crate::bezier::BezierPiece<f64>>; D] = std::array::from_fn(|_| Vec::new());

    hermite_fit_recursive_clamped::<D>(
        pieces,
        0,
        n,
        tolerance_mm,
        target_degree,
        Some(&d2_start),
        d2_end.as_ref(),
        &mut result,
    )?;

    Ok(result)
}

#[cfg(feature = "host")]
#[allow(clippy::too_many_arguments)]
fn hermite_fit_recursive_clamped<const D: usize>(
    pieces: &[[crate::bezier::BezierPiece<f64>; D]],
    lo: usize,
    hi: usize,
    tolerance_mm: f64,
    target_degree: u8,
    d2_start: Option<&[f64; D]>,
    d2_end: Option<&[f64; D]>,
    result: &mut [Vec<crate::bezier::BezierPiece<f64>>; D],
) -> Result<(), FitError> {
    debug_assert!(lo < hi);

    let global_lo = pieces[0][0].u_start;
    let global_hi = pieces[pieces.len() - 1][0].u_end;
    let this_lo = pieces[lo][0].u_start;
    let this_hi = pieces[hi - 1][0].u_end;

    let pin_start = if (this_lo - global_lo).abs() < 1e-12 {
        d2_start
    } else {
        None
    };
    let pin_end = if (this_hi - global_hi).abs() < 1e-12 {
        d2_end
    } else {
        None
    };

    let mut candidate =
        hermite_fit_one_piece_clamped::<D>(pieces, lo, hi, target_degree, pin_start, pin_end);
    let max_residual = hermite_check_residual::<D>(pieces, lo, hi, &candidate, target_degree);

    if max_residual <= tolerance_mm {
        for axis in 0..D {
            candidate[axis].u_start = pieces[lo][axis].u_start;
            candidate[axis].u_end = pieces[hi - 1][axis].u_end;
            result[axis].push(candidate[axis].clone());
        }
        return Ok(());
    }

    if hi - lo == 1 {
        return Err(FitError::ToleranceNotReached {
            achieved_mm: max_residual,
            at_degree: target_degree,
        });
    }

    let mid = lo + (hi - lo) / 2;

    hermite_fit_recursive_clamped::<D>(
        pieces,
        lo,
        mid,
        tolerance_mm,
        target_degree,
        d2_start,
        None,
        result,
    )?;
    hermite_fit_recursive_clamped::<D>(
        pieces,
        mid,
        hi,
        tolerance_mm,
        target_degree,
        None,
        d2_end,
        result,
    )?;

    Ok(())
}

#[cfg(feature = "host")]
#[allow(clippy::too_many_arguments)]
fn hermite_fit_one_piece_clamped<const D: usize>(
    pieces: &[[crate::bezier::BezierPiece<f64>; D]],
    lo: usize,
    hi: usize,
    target_degree: u8,
    pin_start: Option<&[f64; D]>,
    pin_end: Option<&[f64; D]>,
) -> [crate::bezier::BezierPiece<f64>; D] {
    let u_lo = pieces[lo][0].u_start;
    let u_hi = pieces[hi - 1][0].u_end;
    let h = u_hi - u_lo;
    let d = target_degree as usize;

    let constraints: Vec<(f64, f64, f64, f64)> = (0..D)
        .map(|axis| {
            let f_lo = pieces[lo][axis].evaluate(u_lo);
            let df_lo = pieces[lo][axis].differentiate().evaluate(u_lo);
            let f_hi = pieces[hi - 1][axis].evaluate(u_hi);
            let df_hi = pieces[hi - 1][axis].differentiate().evaluate(u_hi);
            (f_lo, df_lo, f_hi, df_hi)
        })
        .collect();

    if h.abs() < 1e-300 {
        return std::array::from_fn(|axis| {
            let (f_lo, df_lo, f_hi, df_hi) = constraints[axis];
            hermite_construct_poly(f_lo, df_lo, f_hi, df_hi, u_lo, h, d, 0.0)
        });
    }

    match (pin_start, pin_end) {
        (Some(d2s), Some(d2e)) => std::array::from_fn(|axis| {
            let (f_lo, df_lo, f_hi, df_hi) = constraints[axis];
            hermite_construct_poly_both_clamped(
                f_lo, df_lo, f_hi, df_hi, u_lo, h, d2s[axis], d2e[axis],
            )
        }),
        (Some(d2s), None) => std::array::from_fn(|axis| {
            let (f_lo, df_lo, f_hi, df_hi) = constraints[axis];
            hermite_construct_poly(f_lo, df_lo, f_hi, df_hi, u_lo, h, d, d2s[axis] * 0.5)
        }),
        (None, Some(d2e)) => {
            if d <= 3 {
                return std::array::from_fn(|axis| {
                    let (f_lo, df_lo, f_hi, df_hi) = constraints[axis];
                    hermite_construct_poly(f_lo, df_lo, f_hi, df_hi, u_lo, h, d, 0.0)
                });
            }
            let n_check = 4 * (d + 1);
            let sample_u: Vec<f64> = (0..=n_check)
                .map(|i| u_lo + (u_hi - u_lo) * (i as f64 / n_check as f64))
                .collect();
            let sample_piece_idx: Vec<usize> = sample_u
                .iter()
                .map(|&u| hermite_find_piece_at(pieces, lo, hi, u))
                .collect();

            let cand_0: Vec<crate::bezier::BezierPiece<f64>> = (0..D)
                .map(|axis| {
                    let (f_lo, df_lo, f_hi, df_hi) = constraints[axis];
                    hermite_construct_poly_end_clamped(
                        f_lo, df_lo, f_hi, df_hi, u_lo, h, d, d2e[axis], 0.0,
                    )
                })
                .collect();
            let cand_1: Vec<crate::bezier::BezierPiece<f64>> = (0..D)
                .map(|axis| {
                    let (f_lo, df_lo, f_hi, df_hi) = constraints[axis];
                    hermite_construct_poly_end_clamped(
                        f_lo, df_lo, f_hi, df_hi, u_lo, h, d, d2e[axis], 1.0,
                    )
                })
                .collect();

            let mut a_vals: Vec<f64> = Vec::new();
            let mut b_vals: Vec<f64> = Vec::new();
            for (si, &u) in sample_u.iter().enumerate() {
                let pidx = sample_piece_idx[si];
                for axis in 0..D {
                    let ref_val = pieces[pidx][axis].evaluate(u);
                    let p0 = cand_0[axis].evaluate(u);
                    let p1 = cand_1[axis].evaluate(u);
                    a_vals.push(ref_val - p0);
                    b_vals.push(p1 - p0);
                }
            }
            let optimal_c2 = minimax_1d(&a_vals, &b_vals);

            std::array::from_fn(|axis| {
                let (f_lo, df_lo, f_hi, df_hi) = constraints[axis];
                hermite_construct_poly_end_clamped(
                    f_lo, df_lo, f_hi, df_hi, u_lo, h, d, d2e[axis], optimal_c2,
                )
            })
        }
        (None, None) => hermite_fit_one_piece::<D>(pieces, lo, hi, target_degree),
    }
}

#[cfg(feature = "host")]
#[allow(clippy::cast_possible_wrap)]
fn hermite_construct_poly_both_clamped(
    f_lo: f64,
    df_lo: f64,
    f_hi: f64,
    df_hi: f64,
    u_lo: f64,
    h: f64,
    d2_lo: f64,
    d2_hi: f64,
) -> crate::bezier::BezierPiece<f64> {
    let c0 = f_lo;
    let c1 = df_lo;
    let c2 = d2_lo * 0.5;

    if h.abs() < 1e-300 {
        return crate::bezier::BezierPiece {
            u_start: u_lo,
            u_end: u_lo + h,
            coeffs: vec![c0, c1, c2, 0.0, 0.0, 0.0],
        };
    }

    let h2 = h * h;
    let h3 = h2 * h;
    let h4 = h3 * h;
    let h5 = h4 * h;

    let p_res = f_hi - c0 - c1 * h - c2 * h2;
    let v_res = df_hi - c1 - 2.0 * c2 * h;
    let a_res = d2_hi - 2.0 * c2;

    let rhs0 = p_res;
    let rhs1 = v_res * h;
    let rhs2 = a_res * h2;

    let cramer_det = 2.0;
    let q = (20.0 * rhs0 - 8.0 * rhs1 + rhs2) / cramer_det;
    let r = (-30.0 * rhs0 + 14.0 * rhs1 - 2.0 * rhs2) / cramer_det;
    let s = (12.0 * rhs0 - 6.0 * rhs1 + rhs2) / cramer_det;

    let c3 = q / h3;
    let c4 = r / h4;
    let c5 = s / h5;

    crate::bezier::BezierPiece {
        u_start: u_lo,
        u_end: u_lo + h,
        coeffs: vec![c0, c1, c2, c3, c4, c5],
    }
}

#[cfg(feature = "host")]
#[allow(clippy::too_many_arguments, clippy::cast_possible_wrap)]
fn hermite_construct_poly_end_clamped(
    f_lo: f64,
    df_lo: f64,
    f_hi: f64,
    df_hi: f64,
    u_lo: f64,
    h: f64,
    d: usize,
    d2_hi: f64,
    c2_val: f64,
) -> crate::bezier::BezierPiece<f64> {
    if d < 4 || h.abs() < 1e-300 {
        let mut coeffs = vec![0.0f64; d + 1];
        coeffs[0] = f_lo;
        if d >= 1 {
            coeffs[1] = df_lo;
        }
        return crate::bezier::BezierPiece {
            u_start: u_lo,
            u_end: u_lo + h,
            coeffs,
        };
    }

    let mut coeffs = vec![0.0f64; d + 1];
    coeffs[0] = f_lo;
    coeffs[1] = df_lo;
    coeffs[2] = c2_val;

    let mut pos_residual = f_hi - f_lo - df_lo * h - c2_val * h * h;
    let mut vel_residual = df_hi - df_lo - 2.0 * c2_val * h;
    let mut acc_residual = d2_hi - 2.0 * c2_val;

    let mut h_pow = h * h * h;
    let mut h_pow_d = h * h;
    let mut h_pow_dd = h;
    for k in 3..d.saturating_sub(2) {
        pos_residual -= coeffs[k] * h_pow;
        vel_residual -= (k as f64) * coeffs[k] * h_pow_d;
        acc_residual -= (k as f64) * ((k - 1) as f64) * coeffs[k] * h_pow_dd;
        h_pow *= h;
        h_pow_d *= h;
        h_pow_dd *= h;
    }

    let a = (d - 2) as f64;
    let b = (d - 1) as f64;
    let e = d as f64;

    let rhs0 = pos_residual;
    let rhs1 = vel_residual * h;
    let rhs2 = acc_residual * h * h;

    let m00 = 1.0_f64;
    let m01 = 1.0_f64;
    let m02 = 1.0_f64;
    let m10 = a;
    let m11 = b;
    let m12 = e;
    let m20 = a * (a - 1.0);
    let m21 = b * (b - 1.0);
    let m22 = e * (e - 1.0);

    let det = m00 * (m11 * m22 - m12 * m21) - m01 * (m10 * m22 - m12 * m20)
        + m02 * (m10 * m21 - m11 * m20);

    let h_a = h.powi((d - 2) as i32);
    let h_b = h_a * h;
    let h_e = h_b * h;

    if det.abs() < 1e-300 {
        return crate::bezier::BezierPiece {
            u_start: u_lo,
            u_end: u_lo + h,
            coeffs,
        };
    }

    let q = (rhs0 * (m11 * m22 - m12 * m21) - rhs1 * (m01 * m22 - m02 * m21)
        + rhs2 * (m01 * m12 - m02 * m11))
        / det;
    let r = (m00 * (rhs1 * m22 - rhs2 * m12) - rhs0 * (m10 * m22 - m12 * m20)
        + m02 * (m10 * rhs2 - rhs1 * m20))
        / det;
    let s = (m00 * (m11 * rhs2 - rhs1 * m21) - m01 * (m10 * rhs2 - rhs1 * m20)
        + rhs0 * (m10 * m21 - m11 * m20))
        / det;

    coeffs[d - 2] = q / h_a;
    coeffs[d - 1] = r / h_b;
    coeffs[d] = s / h_e;

    crate::bezier::BezierPiece {
        u_start: u_lo,
        u_end: u_lo + h,
        coeffs,
    }
}

#[cfg(feature = "host")]
fn hermite_fit_one_piece<const D: usize>(
    pieces: &[[crate::bezier::BezierPiece<f64>; D]],
    lo: usize,
    hi: usize,
    target_degree: u8,
) -> [crate::bezier::BezierPiece<f64>; D] {
    let u_lo = pieces[lo][0].u_start;
    let u_hi = pieces[hi - 1][0].u_end;
    let h = u_hi - u_lo;
    let d = target_degree as usize;

    let constraints: Vec<(f64, f64, f64, f64)> = (0..D)
        .map(|axis| {
            let f_lo = pieces[lo][axis].evaluate(u_lo);
            let df_lo = pieces[lo][axis].differentiate().evaluate(u_lo);
            let f_hi = pieces[hi - 1][axis].evaluate(u_hi);
            let df_hi = pieces[hi - 1][axis].differentiate().evaluate(u_hi);
            (f_lo, df_lo, f_hi, df_hi)
        })
        .collect();

    if d <= 3 || h.abs() < 1e-300 {
        return std::array::from_fn(|axis| {
            let (f_lo, df_lo, f_hi, df_hi) = constraints[axis];
            hermite_construct_poly(f_lo, df_lo, f_hi, df_hi, u_lo, h, d, 0.0)
        });
    }

    let n_check = 4 * (d + 1);
    let mut sample_u: Vec<f64> = Vec::with_capacity(n_check + 1);
    let mut sample_piece_idx: Vec<usize> = Vec::with_capacity(n_check + 1);
    for i in 0..=n_check {
        let t = i as f64 / n_check as f64;
        let u = u_lo + (u_hi - u_lo) * t;
        sample_u.push(u);
        sample_piece_idx.push(hermite_find_piece_at(pieces, lo, hi, u));
    }

    let cand_0: Vec<crate::bezier::BezierPiece<f64>> = (0..D)
        .map(|axis| {
            let (f_lo, df_lo, f_hi, df_hi) = constraints[axis];
            hermite_construct_poly(f_lo, df_lo, f_hi, df_hi, u_lo, h, d, 0.0)
        })
        .collect();
    let cand_1: Vec<crate::bezier::BezierPiece<f64>> = (0..D)
        .map(|axis| {
            let (f_lo, df_lo, f_hi, df_hi) = constraints[axis];
            hermite_construct_poly(f_lo, df_lo, f_hi, df_hi, u_lo, h, d, 1.0)
        })
        .collect();

    let mut a_vals: Vec<f64> = Vec::new();
    let mut b_vals: Vec<f64> = Vec::new();
    for (si, &u) in sample_u.iter().enumerate() {
        let pidx = sample_piece_idx[si];
        for axis in 0..D {
            let ref_val = pieces[pidx][axis].evaluate(u);
            let p0 = cand_0[axis].evaluate(u);
            let p1 = cand_1[axis].evaluate(u);
            a_vals.push(ref_val - p0);
            b_vals.push(p1 - p0);
        }
    }

    let optimal_c2 = minimax_1d(&a_vals, &b_vals);

    std::array::from_fn(|axis| {
        let (f_lo, df_lo, f_hi, df_hi) = constraints[axis];
        hermite_construct_poly(f_lo, df_lo, f_hi, df_hi, u_lo, h, d, optimal_c2)
    })
}

#[cfg(feature = "host")]
fn minimax_1d(a: &[f64], b: &[f64]) -> f64 {
    debug_assert_eq!(a.len(), b.len());

    let max_b = b.iter().map(|v| v.abs()).fold(0.0_f64, f64::max);
    if max_b < 1e-30 {
        return 0.0;
    }

    let eval_max_err = |x: f64| -> f64 {
        a.iter()
            .zip(b.iter())
            .map(|(&ai, &bi)| (ai - bi * x).abs())
            .fold(0.0_f64, f64::max)
    };

    let mut candidates: Vec<f64> = Vec::new();
    candidates.push(0.0);
    let n = a.len();
    for i in 0..n {
        if b[i].abs() > 1e-30 {
            candidates.push(a[i] / b[i]);
        }
    }
    for i in 0..n {
        for j in 0..n {
            let denom = b[i] + b[j];
            if denom.abs() > 1e-30 {
                candidates.push((a[i] + a[j]) / denom);
            }
            let denom2 = b[i] - b[j];
            if denom2.abs() > 1e-30 {
                candidates.push((a[i] - a[j]) / denom2);
            }
        }
    }

    let mut best_x = 0.0;
    let mut best_err = eval_max_err(0.0);
    for x in candidates {
        if !x.is_finite() {
            continue;
        }
        let err = eval_max_err(x);
        if err < best_err {
            best_err = err;
            best_x = x;
        }
    }

    best_x
}

#[cfg(feature = "host")]
#[allow(clippy::too_many_arguments, clippy::cast_possible_wrap)]
fn hermite_construct_poly(
    f_lo: f64,
    df_lo: f64,
    f_hi: f64,
    df_hi: f64,
    u_lo: f64,
    h: f64,
    d: usize,
    c2_val: f64,
) -> crate::bezier::BezierPiece<f64> {
    let mut coeffs = vec![0.0f64; d + 1];

    coeffs[0] = f_lo;
    coeffs[1] = df_lo;

    if d >= 4 {
        coeffs[2] = c2_val;
    }

    let mut pos_residual = f_hi - coeffs[0] - coeffs[1] * h;
    let mut vel_residual = df_hi - coeffs[1];

    let mut h_pow = h * h;
    let mut h_pow_deriv = h;
    for k in 2..d.saturating_sub(1) {
        pos_residual -= coeffs[k] * h_pow;
        vel_residual -= (k as f64) * coeffs[k] * h_pow_deriv;
        h_pow *= h;
        h_pow_deriv *= h;
    }

    let h_dm2 = h.powi(d as i32 - 2);
    let h_dm1 = h_dm2 * h;
    let h_d = h_dm1 * h;
    let det = h.powi(2 * d as i32 - 2);

    if det.abs() < 1e-300 {
        return crate::bezier::BezierPiece {
            u_start: u_lo,
            u_end: u_lo + h,
            coeffs,
        };
    }

    let d_f = d as f64;
    let dm1_f = (d - 1) as f64;
    let c_dm1 = (pos_residual * d_f * h_dm1 - h_d * vel_residual) / det;
    let c_d = (h_dm1 * vel_residual - dm1_f * h_dm2 * pos_residual) / det;

    coeffs[d - 1] = c_dm1;
    coeffs[d] = c_d;

    crate::bezier::BezierPiece {
        u_start: u_lo,
        u_end: u_lo + h,
        coeffs,
    }
}

#[cfg(feature = "host")]
fn hermite_check_residual<const D: usize>(
    pieces: &[[crate::bezier::BezierPiece<f64>; D]],
    lo: usize,
    hi: usize,
    candidate: &[crate::bezier::BezierPiece<f64>; D],
    target_degree: u8,
) -> f64 {
    let n_check = 4 * (target_degree as usize + 1);
    let u_lo = pieces[lo][0].u_start;
    let u_hi = pieces[hi - 1][0].u_end;
    let mut max_err = 0.0_f64;

    for i in 0..=n_check {
        let t = i as f64 / n_check as f64;
        let u = u_lo + (u_hi - u_lo) * t;
        let piece_idx = hermite_find_piece_at(pieces, lo, hi, u);

        for axis in 0..D {
            let ref_val = pieces[piece_idx][axis].evaluate(u);
            let fit_val = candidate[axis].evaluate(u);
            let err = (ref_val - fit_val).abs();
            if err > max_err {
                max_err = err;
            }
        }
    }

    max_err
}

#[cfg(feature = "host")]
fn hermite_find_piece_at<const D: usize>(
    pieces: &[[crate::bezier::BezierPiece<f64>; D]],
    lo: usize,
    hi: usize,
    u: f64,
) -> usize {
    for i in lo..hi {
        if u <= pieces[i][0].u_end + 1e-12 {
            return i;
        }
    }
    hi - 1
}
