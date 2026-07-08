#![allow(unsafe_code)]

use crate::MIN_PARAMETRIC_SPEED;
use crate::view::{NurbsView, VectorNurbsView};
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
            unsafe { *d.get_unchecked_mut(j) = (dj - djm1).mul_add(alpha, djm1) };
        }
    }

    unsafe { *d.get_unchecked(p) }
}

#[inline]
pub fn eval<V: NurbsView>(curve: &V, u: f64) -> f64 {
    debug_assert!((curve.degree() as usize) <= MAX_DEGREE);
    de_boor_inner(curve.control_points(), curve.knots(), curve.degree(), u)
}

#[inline]
pub fn vector_eval<V: VectorNurbsView<N>, const N: usize>(curve: &V, u: f64) -> [f64; N] {
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
                unsafe { *d_axes[axis].get_unchecked_mut(j) = (dj - djm1).mul_add(alpha, djm1) };
            }
        }
    }

    let mut result = [0.0; N];
    for axis in 0..N {
        result[axis] = unsafe { *d_axes[axis].get_unchecked(p) };
    }
    result
}

#[inline]
pub fn eval_polynomial_with_derivative(
    cps: &[f64],
    knots: &[f64],
    degree: u8,
    u: f64,
) -> (f64, f64) {
    debug_assert!((degree as usize) <= MAX_DEGREE);
    debug_assert!(knots.len() == cps.len() + (degree as usize) + 1);

    if degree == 0 {
        let p = 0;
        let n = cps.len();
        let k = find_knot_span(knots, p, n, u);
        debug_assert!(k < n);
        return (unsafe { *cps.get_unchecked(k) }, 0.0);
    }

    let p = degree as usize;
    let n = cps.len();
    let k = find_knot_span(knots, p, n, u);

    debug_assert!(k >= p && k < n, "find_knot_span invariant: k ∈ [p, n-1]");
    debug_assert!(knots.len() == n + p + 1, "knots len == n + p + 1");

    let mut d = [0.0; WORKSPACE_SIZE];
    let mut dd = [0.0; WORKSPACE_SIZE];
    for j in 0..=p {
        unsafe { *d.get_unchecked_mut(j) = *cps.get_unchecked(k - p + j) };
    }

    for r in 1..=p {
        for j in (r..=p).rev() {
            let lo = unsafe { *knots.get_unchecked(k - p + j) };
            let hi = unsafe { *knots.get_unchecked(k + 1 + j - r) };
            let denom = hi - lo;
            let old_d_jm1 = unsafe { *d.get_unchecked(j - 1) };
            let old_d_j = unsafe { *d.get_unchecked(j) };
            let old_dd_jm1 = unsafe { *dd.get_unchecked(j - 1) };
            let old_dd_j = unsafe { *dd.get_unchecked(j) };
            if denom > 0.0 {
                let inv_denom = 1.0 / denom;
                let alpha = (u - lo) * inv_denom;
                let one_minus_alpha = 1.0 - alpha;
                unsafe {
                    *dd.get_unchecked_mut(j) = one_minus_alpha * old_dd_jm1
                        + alpha * old_dd_j
                        + (old_d_j - old_d_jm1) * inv_denom;
                    *d.get_unchecked_mut(j) = (old_d_j - old_d_jm1).mul_add(alpha, old_d_jm1);
                }
            } else {
                unsafe {
                    *d.get_unchecked_mut(j) = old_d_jm1;
                    *dd.get_unchecked_mut(j) = old_dd_jm1;
                }
            }
        }
    }

    unsafe { (*d.get_unchecked(p), *dd.get_unchecked(p)) }
}

#[inline]
pub fn eval_polynomial(cps: &[f64], knots: &[f64], degree: u8, u: f64) -> f64 {
    debug_assert!((degree as usize) <= MAX_DEGREE);
    debug_assert!(knots.len() == cps.len() + (degree as usize) + 1);
    de_boor_inner(cps, knots, degree, u)
}

#[inline]
pub fn eval_derivative(cps: &[f64], knots: &[f64], degree: u8, u: f64) -> f64 {
    debug_assert!((degree as usize) <= MAX_DEGREE);
    if degree == 0 {
        return 0.0;
    }
    let p = degree as usize;
    let n = cps.len();
    if n < 2 || knots.len() < n + p + 1 {
        return 0.0;
    }
    let new_p = p - 1;
    let new_n = n - 1;
    let lowered_knots = &knots[1..n + p];

    let k = find_knot_span(lowered_knots, new_p, new_n, u);

    debug_assert!(
        k >= new_p && k < new_n,
        "find_knot_span invariant on lowered knots"
    );

    let mut d = [0.0; WORKSPACE_SIZE];
    let p_t = f64::from(degree);
    for j in 0..=new_p {
        let i = k - new_p + j;
        let denom = unsafe { *knots.get_unchecked(i + p + 1) - *knots.get_unchecked(i + 1) };
        unsafe {
            *d.get_unchecked_mut(j) = if denom > 0.0 {
                p_t * (*cps.get_unchecked(i + 1) - *cps.get_unchecked(i)) / denom
            } else {
                0.0
            };
        }
    }

    for r in 1..=new_p {
        for j in (r..=new_p).rev() {
            let knot_lo = unsafe { *lowered_knots.get_unchecked(k - new_p + j) };
            let knot_hi = unsafe { *lowered_knots.get_unchecked(k + 1 + j - r) };
            let denom = knot_hi - knot_lo;
            let alpha = if denom > 0.0 {
                (u - knot_lo) / denom
            } else {
                0.0
            };
            let dj = unsafe { *d.get_unchecked(j) };
            let djm1 = unsafe { *d.get_unchecked(j - 1) };
            unsafe { *d.get_unchecked_mut(j) = (dj - djm1).mul_add(alpha, djm1) };
        }
    }

    unsafe { *d.get_unchecked(new_p) }
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

pub fn curvature_from_derivs<const N: usize>(
    first_deriv: &crate::VectorNurbs<N>,
    second_deriv: &crate::VectorNurbs<N>,
    u: f64,
) -> f64 {
    let r_prime = vector_eval(&first_deriv.as_view(), u);
    let r_double = vector_eval(&second_deriv.as_view(), u);

    assert!(N == 3, "curvature_from_derivs requires N == 3");

    let cx = r_prime[1] * r_double[2] - r_prime[2] * r_double[1];
    let cy = r_prime[2] * r_double[0] - r_prime[0] * r_double[2];
    let cz = r_prime[0] * r_double[1] - r_prime[1] * r_double[0];
    let cross_norm = (cx * cx + cy * cy + cz * cz).sqrt();

    let speed_sq = r_prime[0] * r_prime[0] + r_prime[1] * r_prime[1] + r_prime[2] * r_prime[2];
    let speed = speed_sq.sqrt();
    let speed_cubed = speed * speed * speed;

    let floor = MIN_PARAMETRIC_SPEED;
    cross_norm / speed_cubed.max(floor)
}

#[cfg(test)]
mod tests;
