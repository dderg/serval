use nurbs::bezier::BezierPiece;
use nurbs::chebyshev::{
    chebyshev_to_monomial_tau, monomial_u_to_chebyshev, taylor_shift,
    truncate_chebyshev_c2_anchored,
};

use super::FitTol;

/// Interior probe nodes `cos(kπ/8)` on u ∈ [−1, 1] — endpoints are matched
/// exactly by construction, so all probing is interior.
pub(crate) const LADDER_PROBES_U: [f64; 7] = [
    -0.923_879_532_511_286_7,
    -std::f64::consts::FRAC_1_SQRT_2,
    -0.382_683_432_365_089_8,
    0.0,
    0.382_683_432_365_089_8,
    std::f64::consts::FRAC_1_SQRT_2,
    0.923_879_532_511_286_7,
];

/// `(1 − u²)³` — triple zeros at ±1, so adding it preserves endpoint p/v/a.
const BUMP6: [f64; 7] = [1.0, 0.0, -3.0, 0.0, 3.0, 0.0, -1.0];
/// `u·(1 − u²)³`.
const BUMP7: [f64; 8] = [0.0, 1.0, 0.0, -3.0, 0.0, 3.0, 0.0, -1.0];

/// Post-fit Chebyshev truncation budgets: position at a tenth of the fit
/// tolerance, endpoint velocity/acceleration bounded so collapsing a piece can
/// never step the seam feedforward.
pub(crate) const FIT_TRUNC_POS_FACTOR: f64 = 0.1;
pub(crate) const FIT_TRUNC_VEL_MM_S: f64 = 0.05;
pub(crate) const FIT_TRUNC_ACC_MM_S2: f64 = 0.25;

pub(crate) fn eval_mono(c: &[f64], x: f64) -> f64 {
    c.iter().rev().fold(0.0, |acc, &ck| acc * x + ck)
}

pub(crate) fn eval_mono_dd(c: &[f64], x: f64) -> f64 {
    let mut acc = 0.0;
    for (k, &ck) in c.iter().enumerate().skip(2).rev() {
        acc = acc * x + (k * (k - 1)) as f64 * ck;
    }
    acc
}

/// Monomial-in-u quintic matching `(p, v, a)` — time-domain derivatives — at
/// both ends of a span of duration `h`. Fitting in u keeps the coefficients
/// O(piece amplitude): the conditioning win over monomial-τ.
pub(crate) fn quintic_in_u(sa: (f64, f64, f64), sb: (f64, f64, f64), h: f64) -> Vec<f64> {
    let s = 0.5 * h;
    let q = quintic_hermite_coeffs(
        sa.0,
        sa.1 * s,
        sa.2 * s * s,
        sb.0,
        sb.1 * s,
        sb.2 * s * s,
        2.0,
    );
    taylor_shift(&q, 1.0)
}

/// Monomial coefficients `c0..c5` of the quintic matching `(s0, v0, a0)` at `τ = 0`
/// and `(s1, v1, a1)` at `τ = h`.
pub(super) fn quintic_hermite_coeffs(
    s0: f64,
    v0: f64,
    a0: f64,
    s1: f64,
    v1: f64,
    a1: f64,
    h: f64,
) -> [f64; 6] {
    let ds = s1 - s0;
    let h2 = h * h;
    let h3 = h2 * h;
    let c3 = (20.0 * ds - (8.0 * v1 + 12.0 * v0) * h - (3.0 * a0 - a1) * h2) / (2.0 * h3);
    let c4 =
        (-30.0 * ds + (14.0 * v1 + 16.0 * v0) * h + (3.0 * a0 - 2.0 * a1) * h2) / (2.0 * h3 * h);
    let c5 = (12.0 * ds - 6.0 * (v1 + v0) * h - (a0 - a1) * h2) / (2.0 * h3 * h2);
    [s0, v0, 0.5 * a0, c3, c4, c5]
}

/// Degree-`degree` ladder candidate: the quintic base plus `(1−u²)³`-shaped
/// corrections whose coefficients come from interior residuals (u = 0 for
/// degree 6; u = ±½ with exact 27/64 denominators for degree 7).
pub(crate) fn ladder_candidate(
    base: &[f64],
    degree: usize,
    truth_p: &dyn Fn(f64) -> f64,
) -> Vec<f64> {
    let mut c = base.to_vec();
    match degree {
        5 => {}
        6 => {
            let r0 = truth_p(0.0) - eval_mono(base, 0.0);
            c.resize(7, 0.0);
            for (ci, &w) in c.iter_mut().zip(&BUMP6) {
                *ci += r0 * w;
            }
        }
        7 => {
            let rp = truth_p(0.5) - eval_mono(base, 0.5);
            let rm = truth_p(-0.5) - eval_mono(base, -0.5);
            let q0 = (rp + rm) * (32.0 / 27.0);
            let q1 = (rp - rm) * (64.0 / 27.0);
            c.resize(8, 0.0);
            for (ci, &w) in c.iter_mut().zip(&BUMP6) {
                *ci += q0 * w;
            }
            for (ci, &w) in c.iter_mut().zip(&BUMP7) {
                *ci += q1 * w;
            }
        }
        _ => panic!("ladder degree {degree} outside 5..=7"),
    }
    c
}

fn candidate_ok(
    mono_u: &[f64],
    h: f64,
    tol: FitTol,
    extra_probes_u: &[f64],
    truth_p: &dyn Fn(f64) -> f64,
    truth_a: &dyn Fn(f64) -> f64,
) -> bool {
    let dd_scale = (2.0 / h) * (2.0 / h);
    let probe_ok = |u: f64| {
        (eval_mono(mono_u, u) - truth_p(u)).abs() <= tol.pos_mm
            && (eval_mono_dd(mono_u, u) * dd_scale - truth_a(u)).abs() <= tol.accel_mm_s2
    };
    LADDER_PROBES_U.iter().copied().all(probe_ok) && extra_probes_u.iter().copied().all(probe_ok)
}

/// Endpoint acceleration reads the wire's f32 coefficients with weight
/// `k²(k²−1)/3 · (2/h)²` — on a short piece a degree-6/7 coefficient's f32
/// rounding alone steps the seam accel by tens of mm/s². Below this span the
/// ladder stops at the quintic (whose position error already scales as h⁶).
pub(crate) const MIN_HIGH_DEGREE_SPAN_S: f64 = 5e-4;

pub(crate) fn ladder_degrees(h: f64) -> &'static [usize] {
    if h < MIN_HIGH_DEGREE_SPAN_S {
        &[5]
    } else {
        &[5, 6, 7]
    }
}

/// First ladder degree (5 → 6 → 7, span-capped) whose interior position and
/// acceleration residuals pass; `None` asks the caller to bisect.
pub(crate) fn ladder_fit(
    base: &[f64],
    h: f64,
    tol: FitTol,
    extra_probes_u: &[f64],
    truth_p: &dyn Fn(f64) -> f64,
    truth_a: &dyn Fn(f64) -> f64,
) -> Option<Vec<f64>> {
    ladder_degrees(h).iter().find_map(|&degree| {
        let c = ladder_candidate(base, degree, truth_p);
        candidate_ok(&c, h, tol, extra_probes_u, truth_p, truth_a).then_some(c)
    })
}

/// Chebyshev-truncate an accepted monomial-in-u fit to its true degree, then
/// back to monomial-τ for the NURBS carrier. `h` is the span duration the fit
/// was built with (not recomputed from the bounds — they may live on a shifted
/// timeline, and the truncation weights must match the fit's own scaling).
pub(crate) fn truncated_piece(
    mono_u: &[f64],
    u_start: f64,
    u_end: f64,
    h: f64,
    pos_budget_mm: f64,
    deriv_scale: f64,
) -> BezierPiece {
    let cheb = truncate_chebyshev_c2_anchored(
        &monomial_u_to_chebyshev(mono_u),
        h,
        pos_budget_mm,
        FIT_TRUNC_VEL_MM_S * deriv_scale,
        FIT_TRUNC_ACC_MM_S2 * deriv_scale,
    );
    BezierPiece {
        u_start,
        u_end,
        coeffs: chebyshev_to_monomial_tau(&cheb, h),
    }
}
