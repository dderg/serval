use crate::bezier::binomial;
use crate::{AlgebraError, Float};

mod hermite;
#[cfg(feature = "host")]
pub use hermite::{fit_hermite_c1_clamped, FitError};

#[cfg(feature = "host")]
pub fn scalar_multiply<T: Float>(
    curve: &crate::ScalarNurbs<T>,
    scalar: T,
) -> crate::ScalarNurbs<T> {
    let new_cps: Vec<T> = curve.control_points().iter().map(|c| *c * scalar).collect();
    crate::ScalarNurbs::try_new(curve.degree(), curve.knots().to_vec(), new_cps)
        .expect("scalar_multiply preserves invariants")
}

#[cfg(feature = "host")]
pub fn add<T: Float>(
    a: &crate::ScalarNurbs<T>,
    b: &crate::ScalarNurbs<T>,
) -> Result<crate::ScalarNurbs<T>, AlgebraError> {
    if a.degree() != b.degree() {
        return Err(AlgebraError::KnotMismatch);
    }
    if a.knots() != b.knots() {
        return Err(AlgebraError::KnotMismatch);
    }
    let new_cps: Vec<T> = a
        .control_points()
        .iter()
        .zip(b.control_points().iter())
        .map(|(x, y)| *x + *y)
        .collect();
    crate::ScalarNurbs::try_new(a.degree(), a.knots().to_vec(), new_cps)
        .map_err(|_| AlgebraError::KnotMismatch)
}

#[cfg(feature = "host")]
pub fn add_with_knot_union<T: Float>(
    a: &crate::ScalarNurbs<T>,
    b: &crate::ScalarNurbs<T>,
) -> Result<crate::ScalarNurbs<T>, AlgebraError> {
    if a.degree() != b.degree() {
        return Err(AlgebraError::KnotMismatch);
    }

    if a.knots() == b.knots() {
        return add(a, b);
    }

    let a_pieces = crate::bezier::extract_bezier_pieces(a);
    let b_pieces = crate::bezier::extract_bezier_pieces(b);

    if a_pieces.is_empty() || b_pieces.is_empty() {
        return Err(AlgebraError::KnotMismatch);
    }

    let a_start = a_pieces[0].u_start;
    let a_end = a_pieces[a_pieces.len() - 1].u_end;
    let b_start = b_pieces[0].u_start;
    let b_end = b_pieces[b_pieces.len() - 1].u_end;
    let domain_tol = T::from_f64(1e-12);
    if (a_start - b_start).abs() > domain_tol || (a_end - b_end).abs() > domain_tol {
        return Err(AlgebraError::SupportMismatch);
    }

    let breakpoints = union_breakpoints(&a_pieces, &b_pieces);
    let a_refined = refine_pieces_to_breakpoints(&a_pieces, &breakpoints);
    let b_refined = refine_pieces_to_breakpoints(&b_pieces, &breakpoints);

    assert_eq!(
        a_refined.len(),
        b_refined.len(),
        "add_with_knot_union: refine produced mismatched piece counts \
         (a_refined={}, b_refined={}); this is an internal invariant violation",
        a_refined.len(),
        b_refined.len(),
    );

    let sum_pieces: Vec<crate::bezier::BezierPiece<T>> = a_refined
        .iter()
        .zip(b_refined.iter())
        .map(|(ap, bp)| {
            assert_eq!(
                ap.coeffs.len(),
                bp.coeffs.len(),
                "add_with_knot_union: CP count mismatch after union refine \
                 (ap.coeffs={}, bp.coeffs={})",
                ap.coeffs.len(),
                bp.coeffs.len(),
            );
            let coeffs: Vec<T> = ap
                .coeffs
                .iter()
                .zip(bp.coeffs.iter())
                .map(|(ac, bc)| *ac + *bc)
                .collect();
            crate::bezier::BezierPiece {
                u_start: ap.u_start,
                u_end: ap.u_end,
                coeffs,
            }
        })
        .collect();

    Ok(crate::bezier::bezier_pieces_to_nurbs(&sum_pieces))
}

#[cfg(feature = "host")]
#[derive(Debug, Clone)]
pub struct PiecewisePolynomialKernel<T: Float> {
    pub pieces: Vec<crate::bezier::BezierPiece<T>>,
}

#[cfg(feature = "host")]
impl<T: Float> PiecewisePolynomialKernel<T> {
    pub fn single_poly(coeffs: Vec<T>, support: (T, T)) -> Self {
        let piece = crate::bezier::BezierPiece {
            u_start: support.0,
            u_end: support.1,
            coeffs,
        };
        Self {
            pieces: vec![piece],
        }
    }

    #[allow(clippy::needless_pass_by_value)]
    pub fn single_poly_from_absolute(coeffs: Vec<T>, support: (T, T)) -> Self {
        let shifted = absolute_to_pascal_shift(&coeffs, support.0);
        Self::single_poly(shifted, support)
    }

    pub fn support(&self) -> (T, T) {
        (
            self.pieces.first().unwrap().u_start,
            self.pieces.last().unwrap().u_end,
        )
    }

    pub fn from_pieces(pieces: Vec<crate::bezier::BezierPiece<T>>) -> Result<Self, AlgebraError> {
        if pieces.is_empty() {
            return Err(AlgebraError::SupportMismatch);
        }
        for w in pieces.windows(2) {
            if w[0].u_end != w[1].u_start {
                return Err(AlgebraError::SupportMismatch);
            }
        }
        Ok(Self { pieces })
    }
}

#[cfg(feature = "host")]
pub fn multiply<T: Float>(
    a: &crate::ScalarNurbs<T>,
    b: &crate::ScalarNurbs<T>,
) -> Result<crate::ScalarNurbs<T>, AlgebraError> {
    let a_mults = collect_interior_multiplicities(a);
    let b_mults = collect_interior_multiplicities(b);

    let a_pieces = crate::bezier::extract_bezier_pieces(a);
    let b_pieces = crate::bezier::extract_bezier_pieces(b);

    let breakpoints = union_breakpoints(&a_pieces, &b_pieces);
    let a_refined = refine_pieces_to_breakpoints(&a_pieces, &breakpoints);
    let b_refined = refine_pieces_to_breakpoints(&b_pieces, &breakpoints);
    debug_assert_eq!(a_refined.len(), b_refined.len());

    let mut out_pieces = Vec::with_capacity(a_refined.len());
    for (a_p, b_p) in a_refined.iter().zip(b_refined.iter()) {
        let coeffs = poly_multiply(&a_p.coeffs, &b_p.coeffs);
        out_pieces.push(crate::bezier::BezierPiece {
            u_start: a_p.u_start,
            u_end: a_p.u_end,
            coeffs,
        });
    }

    let mut result = crate::bezier::bezier_pieces_to_nurbs(&out_pieces);

    let d_a = a.degree() as usize;
    let d_b = b.degree() as usize;
    let p = result.degree() as usize;
    let interior_breakpoints = collect_interior_breakpoints(&result);
    let targets: Vec<(T, usize)> = interior_breakpoints
        .into_iter()
        .map(|u| {
            let m_a = a_mults
                .iter()
                .find(|(uu, _)| *uu == u)
                .map_or(0, |(_, m)| *m);
            let m_b = b_mults
                .iter()
                .find(|(uu, _)| *uu == u)
                .map_or(0, |(_, m)| *m);
            let target = morken_multiplicity(d_a, m_a, d_b, m_b);
            debug_assert!(
                target <= p,
                "Mørken target {target} exceeds product degree {p}"
            );
            (u, target)
        })
        .collect();

    knot_remove_to_morken_targets(&mut result, &targets, T::from_f64(1e-12));
    Ok(result)
}

#[cfg(feature = "host")]
fn morken_multiplicity(d_a: usize, m_a: usize, d_b: usize, m_b: usize) -> usize {
    match (m_a > 0, m_b > 0) {
        (true, true) => (d_a + m_b).max(d_b + m_a),
        (false, true) => d_a + m_b,
        (true, false) => d_b + m_a,
        (false, false) => 0,
    }
}

#[cfg(feature = "host")]
fn collect_interior_multiplicities<T: Float>(curve: &crate::ScalarNurbs<T>) -> Vec<(T, usize)> {
    let p = curve.degree() as usize;
    let knots = curve.knots();
    if knots.len() <= 2 * (p + 1) {
        return Vec::new();
    }
    let interior_slice = &knots[p + 1..knots.len() - p - 1];
    let mut out: Vec<(T, usize)> = Vec::new();
    for &k in interior_slice {
        if let Some(entry) = out.iter_mut().find(|(u, _)| *u == k) {
            entry.1 += 1;
        } else {
            out.push((k, 1));
        }
    }
    out
}

#[cfg(feature = "host")]
fn collect_interior_breakpoints<T: Float>(curve: &crate::ScalarNurbs<T>) -> Vec<T> {
    collect_interior_multiplicities(curve)
        .into_iter()
        .map(|(u, _)| u)
        .collect()
}

#[cfg(feature = "host")]
fn knot_remove_to_morken_targets<T: Float>(
    curve: &mut crate::ScalarNurbs<T>,
    target_mults: &[(T, usize)],
    tol: T,
) {
    for &(u, target) in target_mults {
        let current = curve.knots().iter().filter(|k| **k == u).count();
        if current > target {
            let n_to_remove = current - target;
            let (new_curve, _actually_removed) =
                crate::knot::remove_knot(curve, u, n_to_remove, tol);
            *curve = new_curve;
        }
    }
}

#[cfg(feature = "host")]
fn union_breakpoints<T: Float>(
    a: &[crate::bezier::BezierPiece<T>],
    b: &[crate::bezier::BezierPiece<T>],
) -> Vec<T> {
    let mut breaks: Vec<T> = Vec::new();
    let push_unique = |u: T, breaks: &mut Vec<T>| {
        if !breaks.contains(&u) {
            breaks.push(u);
        }
    };
    for piece in a {
        push_unique(piece.u_start, &mut breaks);
        push_unique(piece.u_end, &mut breaks);
    }
    for piece in b {
        push_unique(piece.u_start, &mut breaks);
        push_unique(piece.u_end, &mut breaks);
    }
    breaks.sort_by(|x, y| T::total_cmp(*x, *y));
    breaks
}

#[cfg(feature = "host")]
fn refine_pieces_to_breakpoints<T: Float>(
    pieces: &[crate::bezier::BezierPiece<T>],
    breakpoints: &[T],
) -> Vec<crate::bezier::BezierPiece<T>> {
    let mut result: Vec<crate::bezier::BezierPiece<T>> = Vec::new();
    for piece in pieces {
        let mut current = piece.clone();
        let mut interior: Vec<T> = breakpoints
            .iter()
            .filter(|&&b| b > current.u_start && b < current.u_end)
            .copied()
            .collect();
        interior.sort_by(|x, y| T::total_cmp(*x, *y));
        for u in interior {
            let (left, right) = crate::bezier::split_piece_at(&current, u);
            result.push(left);
            current = right;
        }
        result.push(current);
    }
    result
}

#[cfg(feature = "host")]
pub fn convolve<T: Float>(
    curve: &crate::ScalarNurbs<T>,
    kernel: &PiecewisePolynomialKernel<T>,
) -> Result<crate::ScalarNurbs<T>, AlgebraError> {
    let x_pieces = crate::bezier::extract_bezier_pieces(curve);
    let w_pieces = &kernel.pieces;

    let x_breaks: Vec<T> = {
        let mut v: Vec<T> = Vec::new();
        for p in &x_pieces {
            if !v.contains(&p.u_start) {
                v.push(p.u_start);
            }
        }
        v.push(x_pieces.last().unwrap().u_end);
        v
    };
    let w_breaks: Vec<T> = {
        let mut v: Vec<T> = Vec::new();
        for p in w_pieces {
            if !v.contains(&p.u_start) {
                v.push(p.u_start);
            }
        }
        v.push(w_pieces.last().unwrap().u_end);
        v
    };
    let mut out_breaks: Vec<T> = Vec::new();
    for xb in &x_breaks {
        for wb in &w_breaks {
            let s = *xb + *wb;
            if !out_breaks.contains(&s) {
                out_breaks.push(s);
            }
        }
    }
    out_breaks.sort_by(|a, b| T::total_cmp(*a, *b));

    let degree = x_pieces[0].degree() + w_pieces[0].degree() + 1;

    let mut out_pieces: Vec<crate::bezier::BezierPiece<T>> =
        Vec::with_capacity(out_breaks.len() - 1);
    for win in out_breaks.windows(2) {
        let alpha = win[0];
        let beta = win[1];
        let mut accum = crate::bezier::BezierPiece::<T>::zero(alpha, beta, degree);

        for x_p in &x_pieces {
            for w_p in w_pieces {
                let u_mid = (alpha + beta) * T::from_f64(0.5);
                let s_lo = (x_p.u_start).max(u_mid - w_p.u_end);
                let s_hi = (x_p.u_end).min(u_mid - w_p.u_start);
                if s_lo >= s_hi {
                    continue;
                }

                let contribution = integrate_product_piece(x_p, w_p, alpha, beta);
                accum = (&accum + &contribution).expect("same-support accumulation");
            }
        }
        out_pieces.push(accum);
    }

    let mut result = crate::bezier::bezier_pieces_to_nurbs(&out_pieces);
    knot_remove_redundant(&mut result, T::from_f64(1e-12));
    Ok(result)
}

#[cfg(feature = "host")]
pub fn compose_vector_piece<const D: usize>(
    outer: &[&crate::bezier::BezierPiece<f64>; D],
    inner: &crate::bezier::BezierPiece<f64>,
) -> Result<[crate::bezier::BezierPiece<f64>; D], AlgebraError> {
    const ENDPOINT_TOL: f64 = 1e-9;
    let inner_at_start = inner.evaluate(inner.u_start);
    let inner_at_end = inner.evaluate(inner.u_end);
    for outer_axis in outer {
        if (outer_axis.u_start - inner_at_start).abs() > ENDPOINT_TOL
            || (outer_axis.u_end - inner_at_end).abs() > ENDPOINT_TOL
        {
            return Err(AlgebraError::SupportMismatch);
        }
    }

    let pieces: Vec<crate::bezier::BezierPiece<f64>> = outer
        .iter()
        .map(|outer_axis| {
            let d_outer = outer_axis.degree();

            let mut shifted_inner = inner.coeffs.clone();
            if shifted_inner.is_empty() {
                shifted_inner.push(-outer_axis.u_start);
            } else {
                shifted_inner[0] -= outer_axis.u_start;
            }

            let mut powers: Vec<Vec<f64>> = Vec::with_capacity(d_outer + 1);
            powers.push(vec![1.0]);
            for i in 1..=d_outer {
                let next = poly_multiply(&powers[i - 1], &shifted_inner);
                powers.push(next);
            }

            let d_inner = inner.degree();
            let result_len = d_outer * d_inner + 1;
            let mut result_coeffs = vec![0.0_f64; result_len];
            for (i, c_outer) in outer_axis.coeffs.iter().enumerate() {
                let pow = &powers[i];
                for (k, p_k) in pow.iter().enumerate() {
                    result_coeffs[k] += *c_outer * *p_k;
                }
            }

            crate::bezier::BezierPiece {
                u_start: inner.u_start,
                u_end: inner.u_end,
                coeffs: result_coeffs,
            }
        })
        .collect();

    pieces.try_into().map_err(|_: Vec<_>| {
        AlgebraError::NotImplemented("compose_vector_piece: array length mismatch (unreachable)")
    })
}

#[cfg(feature = "host")]
fn poly_multiply<T: Float>(a: &[T], b: &[T]) -> Vec<T> {
    let mut out = vec![T::ZERO; a.len() + b.len() - 1];
    for (i, ai) in a.iter().enumerate() {
        for (j, bj) in b.iter().enumerate() {
            out[i + j] = out[i + j] + *ai * *bj;
        }
    }
    out
}

#[cfg(feature = "host")]
fn integrate_product_piece<T: Float>(
    x: &crate::bezier::BezierPiece<T>,
    w: &crate::bezier::BezierPiece<T>,
    alpha: T,
    beta: T,
) -> crate::bezier::BezierPiece<T> {
    let d_x = x.degree();
    let d_w = w.degree();
    let out_degree = d_x + d_w + 1;

    let u_mid = (alpha + beta) * T::from_f64(0.5);
    let lo_branch_curve = u_mid - w.u_end > x.u_start;
    let hi_branch_curve = u_mid - w.u_start < x.u_end;

    let x_abs_r = pascal_shift_to_absolute(&x.coeffs, x.u_start - alpha);
    let w_abs_z = pascal_shift_to_absolute(&w.coeffs, w.u_start);

    let (r_lo_c, r_lo_v): (T, T) = if lo_branch_curve {
        (-w.u_end, T::ONE)
    } else {
        (x.u_start - alpha, T::ZERO)
    };
    let (r_hi_c, r_hi_v): (T, T) = if hi_branch_curve {
        (-w.u_start, T::ONE)
    } else {
        (x.u_end - alpha, T::ZERO)
    };

    let max_m = d_w;
    let max_n = d_x + d_w;
    let mut integrand = vec![vec![T::ZERO; max_n + 1]; max_m + 1];

    for j in 0..=d_w {
        for l in 0..=j {
            let m = j - l;
            let sign = if l % 2 == 0 { T::ONE } else { -T::ONE };
            let c_jl = T::from_f64(binomial(j, l) as f64);
            let coef = sign * c_jl * w_abs_z[j];
            for i in 0..=d_x {
                let n = l + i;
                integrand[m][n] = integrand[m][n] + coef * x_abs_r[i];
            }
        }
    }

    let mut y_v = vec![T::ZERO; out_degree + 1];
    for m in 0..=max_m {
        for n in 0..=max_n {
            if integrand[m][n] == T::ZERO {
                continue;
            }
            let inv = integrand[m][n] / T::from_f64((n + 1) as f64);
            let hi_pow = power_of_linear(r_hi_c, r_hi_v, n + 1);
            let lo_pow = power_of_linear(r_lo_c, r_lo_v, n + 1);
            for k in 0..hi_pow.len() {
                let target = k + m;
                if target <= out_degree {
                    y_v[target] = y_v[target] + inv * (hi_pow[k] - lo_pow[k]);
                }
            }
        }
    }

    crate::bezier::BezierPiece {
        u_start: alpha,
        u_end: beta,
        coeffs: y_v,
    }
}

#[cfg(feature = "host")]
fn power_of_linear<T: Float>(c: T, a: T, p: usize) -> Vec<T> {
    let mut out = vec![T::ZERO; p + 1];
    let mut c_pow = vec![T::ONE; p + 1];
    let mut a_pow = vec![T::ONE; p + 1];
    for k in 1..=p {
        c_pow[k] = c_pow[k - 1] * c;
        a_pow[k] = a_pow[k - 1] * a;
    }
    for k in 0..=p {
        let bin = T::from_f64(binomial(p, k) as f64);
        out[k] = bin * c_pow[p - k] * a_pow[k];
    }
    out
}

#[cfg(feature = "host")]
fn pascal_shift_to_absolute<T: Float>(shifted: &[T], shift: T) -> Vec<T> {
    let d = shifted.len() - 1;
    let mut out = vec![T::ZERO; d + 1];
    for k in 0..=d {
        let exp = power_of_linear(-shift, T::ONE, k);
        for n in 0..exp.len() {
            out[n] = out[n] + shifted[k] * exp[n];
        }
    }
    out
}

#[cfg(feature = "host")]
fn absolute_to_pascal_shift<T: Float>(absolute: &[T], shift: T) -> Vec<T> {
    let d = absolute.len() - 1;
    let mut out = vec![T::ZERO; d + 1];
    let mut shift_pow = vec![T::ONE; d + 1];
    for k in 1..=d {
        shift_pow[k] = shift_pow[k - 1] * shift;
    }
    for n in 0..=d {
        for k in 0..=n {
            let bin = T::from_f64(binomial(n, k) as f64);
            out[k] = out[k] + absolute[n] * bin * shift_pow[n - k];
        }
    }
    out
}

#[cfg(feature = "host")]
pub fn restrict_to_domain<T: Float>(
    curve: &crate::ScalarNurbs<T>,
    t_lo: T,
    t_hi: T,
) -> Result<crate::ScalarNurbs<T>, AlgebraError> {
    use crate::bezier::{bezier_pieces_to_nurbs, extract_bezier_pieces, split_piece_at};

    if t_lo >= t_hi {
        return Err(AlgebraError::SupportMismatch);
    }

    let pieces = extract_bezier_pieces(curve);
    let mut result = Vec::new();

    for piece in &pieces {
        if piece.u_end <= t_lo || piece.u_start >= t_hi {
            continue;
        }

        let mut p = piece.clone();

        if p.u_start < t_lo {
            let (_, right) = split_piece_at(&p, t_lo);
            p = right;
        }

        if p.u_end > t_hi {
            let (left, _) = split_piece_at(&p, t_hi);
            p = left;
        }

        result.push(p);
    }

    if result.is_empty() {
        return Err(AlgebraError::SupportMismatch);
    }

    Ok(bezier_pieces_to_nurbs(&result))
}

#[cfg(feature = "host")]
pub(crate) fn knot_remove_redundant<T: Float>(curve: &mut crate::ScalarNurbs<T>, tol: T) {
    let p = curve.degree() as usize;
    loop {
        let knots: Vec<T> = curve.knots().to_vec();
        let interior: Vec<T> = {
            let mut seen: Vec<T> = Vec::new();
            for &k in &knots[p + 1..knots.len() - p - 1] {
                if !seen.contains(&k) {
                    seen.push(k);
                }
            }
            seen
        };

        let mut removed_any = false;
        for u in interior {
            let (new_curve, count) = crate::knot::remove_knot(curve, u, 1, tol);
            if count > 0 {
                *curve = new_curve;
                removed_any = true;
            }
        }
        if !removed_any {
            break;
        }
    }
}

#[cfg(all(test, feature = "host"))]
#[allow(clippy::float_cmp)]
mod tests;
