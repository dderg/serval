#![allow(unsafe_code)]

use crate::{MAX_DEGREE, WORKSPACE_SIZE};

pub(crate) use crate::knot::find_knot_span;

#[inline]
pub(crate) fn de_boor_inner(cps: &[f64], knots: &[f64], degree: u8, u: f64) -> f64 {
    debug_assert!((degree as usize) <= MAX_DEGREE);
    let p = degree as usize;
    let n = cps.len();
    let k = find_knot_span(knots, p, n, u);

    debug_assert!(k >= p && k < n, "find_knot_span invariant: k ∈ [p, n-1]");
    debug_assert!(knots.len() == n + p + 1, "knots len == n + p + 1");

    let mut d = [0.0; WORKSPACE_SIZE];
    for j in 0..=p {
        unsafe { *d.get_unchecked_mut(j) = *cps.get_unchecked(k - p + j) };
    }

    for r in 1..=p {
        for j in (r..=p).rev() {
            let knot_lo = unsafe { *knots.get_unchecked(k - p + j) };
            let knot_hi = unsafe { *knots.get_unchecked(k + 1 + j - r) };
            let denom = knot_hi - knot_lo;
            let alpha = if denom > 0.0 {
                (u - knot_lo) / denom
            } else {
                0.0
            };
            let dj = unsafe { *d.get_unchecked(j) };
            let djm1 = unsafe { *d.get_unchecked(j - 1) };
            unsafe { *d.get_unchecked_mut(j) = crate::fmadd(dj - djm1, alpha, djm1) };
        }
    }

    unsafe { *d.get_unchecked(p) }
}

#[inline]
pub fn eval(curve: &crate::ScalarNurbs, u: f64) -> f64 {
    debug_assert!((curve.degree() as usize) <= MAX_DEGREE);
    de_boor_inner(curve.control_points(), curve.knots(), curve.degree(), u)
}

#[inline]
pub fn vector_eval<const N: usize>(curve: &crate::VectorNurbs<N>, u: f64) -> [f64; N] {
    debug_assert!((curve.degree() as usize) <= MAX_DEGREE);
    let p = curve.degree() as usize;
    let knots = curve.knots();
    let cps = curve.control_points();
    let n = cps.len();
    let k = find_knot_span(knots, p, n, u);

    let mut d_axes: [[f64; WORKSPACE_SIZE]; N] = [[0.0; WORKSPACE_SIZE]; N];

    debug_assert!(k >= p && k < n, "find_knot_span invariant: k ∈ [p, n-1]");
    debug_assert!(knots.len() == n + p + 1, "knots len == n + p + 1");

    for j in 0..=p {
        let cp = unsafe { cps.get_unchecked(k - p + j) };
        for axis in 0..N {
            unsafe { *d_axes[axis].get_unchecked_mut(j) = cp[axis] };
        }
    }

    for r in 1..=p {
        for j in (r..=p).rev() {
            let knot_lo = unsafe { *knots.get_unchecked(k - p + j) };
            let knot_hi = unsafe { *knots.get_unchecked(k + 1 + j - r) };
            let denom = knot_hi - knot_lo;
            let alpha = if denom > 0.0 {
                (u - knot_lo) / denom
            } else {
                0.0
            };
            for axis in 0..N {
                let dj = unsafe { *d_axes[axis].get_unchecked(j) };
                let djm1 = unsafe { *d_axes[axis].get_unchecked(j - 1) };
                unsafe {
                    *d_axes[axis].get_unchecked_mut(j) = crate::fmadd(dj - djm1, alpha, djm1)
                };
            }
        }
    }

    let mut result = [0.0; N];
    for axis in 0..N {
        result[axis] = unsafe { *d_axes[axis].get_unchecked(p) };
    }
    result
}

#[must_use]
pub fn derivative(curve: &crate::ScalarNurbs) -> crate::ScalarNurbs {
    let p = curve.degree();
    assert!(p >= 1, "derivative requires degree >= 1");

    let cps = curve.control_points();
    let knots = curve.knots();
    let new_degree = p - 1;
    let new_n = cps.len() - 1;

    let p_t = f64::from(p);

    let mut new_cps: Vec<f64> = Vec::with_capacity(new_n);
    for i in 0..new_n {
        let denom = knots[i + p as usize + 1] - knots[i + 1];
        let q = if denom > 0.0 {
            p_t * (cps[i + 1] - cps[i]) / denom
        } else {
            0.0
        };
        new_cps.push(q);
    }

    let new_knots: Vec<f64> = knots[1..knots.len() - 1].to_vec();

    crate::ScalarNurbs::try_new(new_degree, new_knots, new_cps)
        .expect("degree-lowered NURBS satisfies invariants by construction")
}

#[must_use]
pub fn vector_derivative<const N: usize>(curve: &crate::VectorNurbs<N>) -> crate::VectorNurbs<N> {
    let p = curve.degree();
    assert!(p >= 1, "derivative requires degree >= 1");

    let cps = curve.control_points();
    let knots = curve.knots();
    let new_degree = p - 1;
    let new_n = cps.len() - 1;
    let p_t = f64::from(p);

    let mut new_cps: Vec<[f64; N]> = Vec::with_capacity(new_n);
    for i in 0..new_n {
        let denom = knots[i + p as usize + 1] - knots[i + 1];
        let mut q = [0.0; N];
        if denom > 0.0 {
            for axis in 0..N {
                q[axis] = p_t * (cps[i + 1][axis] - cps[i][axis]) / denom;
            }
        }
        new_cps.push(q);
    }

    let new_knots: Vec<f64> = knots[1..knots.len() - 1].to_vec();

    crate::VectorNurbs::try_new(new_degree, new_knots, new_cps)
        .expect("degree-lowered NURBS satisfies invariants by construction")
}

/// Derivatives `C(u), C'(u), …, C^(order)(u)` into `out[..=order]` from one
/// knot-span lookup. Each order is de Boor on the local hodograph, so the
/// control-point differences are formed before any basis weight touches
/// them and a curve carrying a large offset keeps small derivatives exact;
/// `out[0]` is bit-identical to [`eval`]. Orders past the degree are zero.
/// At a knot the right-hand piece is differentiated, matching [`eval`]'s
/// span choice.
pub fn eval_derivatives(
    cps: &[f64],
    knots: &[f64],
    degree: u8,
    u: f64,
    order: usize,
    out: &mut [f64],
) {
    debug_assert!((degree as usize) <= MAX_DEGREE);
    debug_assert!(knots.len() == cps.len() + (degree as usize) + 1);
    assert!(
        out.len() > order,
        "derivative output holds orders 0..={order}"
    );
    let p = degree as usize;
    let n = cps.len();
    let span = find_knot_span(knots, p, n, u);
    out[..=order].fill(0.0);

    let mut hodograph = [0.0; WORKSPACE_SIZE];
    hodograph[..=p].copy_from_slice(&cps[span - p..=span]);
    for k in 0..=order.min(p) {
        let reduced = p - k;
        if k > 0 {
            let factor = (reduced + 1) as f64;
            for j in 0..=reduced {
                let denominator = knots[span + j + 1] - knots[span - p + j + k];
                hodograph[j] = if denominator > 0.0 {
                    factor * (hodograph[j + 1] - hodograph[j]) / denominator
                } else {
                    0.0
                };
            }
        }
        let mut d = hodograph;
        for r in 1..=reduced {
            for j in (r..=reduced).rev() {
                let knot_lo = knots[span - p + k + j];
                let knot_hi = knots[span + 1 + j - r];
                let denominator = knot_hi - knot_lo;
                let alpha = if denominator > 0.0 {
                    (u - knot_lo) / denominator
                } else {
                    0.0
                };
                d[j] = crate::fmadd(d[j] - d[j - 1], alpha, d[j - 1]);
            }
        }
        out[k] = d[reduced];
    }
}

#[cfg(test)]
mod tests;
