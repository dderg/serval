use nurbs::bezier::BezierPiece;
use trajectory::NonlinearAdvance;

use crate::lowering::FitTol;

/// Bisection floor: below `lowering::MIN_PHASE_PIECE_S` the Bézier round trip
/// corrupts derivatives, so a span that still misses the fit budget there is
/// reported instead of subdivided further — at any physical acceleration the
/// Taylor remainder over such a span is orders of magnitude under the
/// budgets, so hitting this is a bug, not a tuning problem.
const MIN_SPLIT_SPAN_S: f64 = 1e-6;
const MAX_SPLIT_DEPTH: u32 = 22;

/// A piece whose composed advance still misses the fit budget at the
/// bisection floor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct AdvanceFitError {
    pub u_start: f64,
    pub span_s: f64,
}

impl std::fmt::Display for AdvanceFitError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "nonlinear advance composition missed the fit budget at t={} over a {}s span",
            self.u_start, self.span_s
        )
    }
}

/// Interior probes for the accept test; endpoints are matched exactly by the
/// Hermite correction, and the Taylor center pins the midpoint, so the error
/// peaks between them.
const PROBES_U: [f64; 4] = [0.15, 0.38, 0.62, 0.85];

/// Support width of the chain's smoothing kernel — the one the compile rule
/// (`NonlinearAdvanceNeedsKernel`) guarantees downstream of a pre-kernel
/// advance stage.
pub(crate) fn downstream_kernel_width(chain: &trajectory::CompiledChain) -> f64 {
    chain
        .stages
        .iter()
        .find_map(|stage| match stage {
            trajectory::ChainStage::SmoothKernel(kernel) => {
                let (lo, hi) = kernel.support();
                Some(hi - lo)
            }
            _ => None,
        })
        .expect("chain with a nonlinear advance must carry a smoothing kernel")
}

/// `y = x + a(ẋ)` applied in closed form to monomial-in-`τ` pieces.
///
/// Per piece the advance law is expanded to second order about the midpoint
/// velocity — polynomial composition, exact algebra — then a quintic Hermite
/// correction pins the exact `(y, ẏ, ÿ)` at both ends, so adjacent pieces
/// weld C² wherever the input does. Interior probes check the Taylor
/// remainder against the fit budgets; a piece that misses bisects until it
/// fits, and errors out loudly at the span floor. Output pieces share one
/// uniform degree.
///
/// `kernel_width_s` is the support width of the downstream smoothing kernel
/// (`NonlinearAdvanceNeedsKernel` guarantees one behind a pre-kernel
/// advance); it sizes the acceptance budget for pieces whose composed target
/// acceleration is too extreme for the absolute budget to be meaningful.
pub(crate) fn apply_nonlinear_advance_pieces(
    pieces: &[BezierPiece],
    adv: NonlinearAdvance,
    tol: FitTol,
    kernel_width_s: f64,
) -> Result<Vec<BezierPiece>, AdvanceFitError> {
    assert!(
        kernel_width_s > 0.0,
        "nonlinear advance composes pre-kernel; a downstream kernel is guaranteed"
    );
    let waiver = AccelWaiver::for_kernel(tol, kernel_width_s);
    let in_len = pieces
        .iter()
        .map(|p| p.coeffs.len())
        .max()
        .expect("non-empty");
    // p is deg L-1, dv² is deg 2L-4, the Hermite correction is deg 5.
    let out_len = (2 * in_len).saturating_sub(3).max(6);
    assert!(
        out_len <= nurbs::MAX_DEGREE + 1,
        "advance composition degree {} exceeds MAX_DEGREE",
        out_len - 1
    );
    let mut out = Vec::with_capacity(pieces.len());
    for piece in pieces {
        let mut coeffs = piece.coeffs.clone();
        coeffs.resize(in_len, 0.0);
        compose_recursive(
            &coeffs,
            piece.u_start,
            piece.u_end,
            adv,
            tol,
            waiver,
            out_len,
            0,
            &mut out,
        )?;
    }
    Ok(out)
}
/// Fallback acceptance for pieces whose composed target acceleration dwarfs
/// the absolute budget (`ÿ = a + a''(v)a² + a'(v)j` reaches 1e8 mm/s² on
/// jerk-unlimited retract ramps, where ±50 mm/s² absolute means ppb relative
/// accuracy and bisection runs to the floor). Such a piece is accepted on a
/// *tightened, analytically bounded* position budget instead: the downstream
/// kernel turns a position error `e` confined to one piece into at most
/// `16·max|e|/width²` of smoothed acceleration error (the triangle kernel's
/// second derivative is four delta taps of weight `(2/width)²`), so holding
/// `max|e| ≤ accel_budget·width²/64` keeps this piece's contribution under a
/// quarter of the acceleration budget after smoothing. `max|e|` is bounded
/// rigorously, not probed: the Taylor remainder is `≤ M₃/6·dv_max³` (plus a
/// `Δa''/2·dv_max²` term when a reciprocal piece straddles rest, where its
/// `a''` jumps), and the quintic correction is a polynomial we hold the
/// coefficients of, so its Bernstein bound is exact.
#[derive(Clone, Copy)]
struct AccelWaiver {
    pos_mm: f64,
    rel: f64,
}

const ACCEL_REL_TOL: f64 = 1e-3;

impl AccelWaiver {
    fn for_kernel(tol: FitTol, kernel_width_s: f64) -> Self {
        Self {
            pos_mm: (tol.accel_mm_s2 * kernel_width_s * kernel_width_s / 64.0).min(tol.pos_mm),
            rel: ACCEL_REL_TOL,
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn compose_recursive(
    coeffs: &[f64],
    u_start: f64,
    u_end: f64,
    adv: NonlinearAdvance,
    tol: FitTol,
    waiver: AccelWaiver,
    out_len: usize,
    depth: u32,
    out: &mut Vec<BezierPiece>,
) -> Result<(), AdvanceFitError> {
    let h = u_end - u_start;
    assert!(h > 0.0, "nonlinear advance: non-positive piece span {h}");
    let candidate = composed_candidate(coeffs, h, adv, out_len);
    if candidate_fits(coeffs, &candidate, h, adv, tol, waiver) {
        out.push(BezierPiece {
            u_start,
            u_end,
            coeffs: candidate.coeffs,
        });
        return Ok(());
    }
    if h <= 2.0 * MIN_SPLIT_SPAN_S || depth >= MAX_SPLIT_DEPTH {
        return Err(AdvanceFitError { u_start, span_s: h });
    }
    let mid = u_start + 0.5 * h;
    let right = shifted_monomial(coeffs, 0.5 * h);
    compose_recursive(
        coeffs,
        u_start,
        mid,
        adv,
        tol,
        waiver,
        out_len,
        depth + 1,
        out,
    )?;
    compose_recursive(
        &right,
        mid,
        u_end,
        adv,
        tol,
        waiver,
        out_len,
        depth + 1,
        out,
    )
}

/// The composed candidate plus the ingredients of the rigorous position
/// residual bound: `dv_max` (Bernstein bound of `v(τ) − v_mid`), the
/// midpoint velocity, and the Bernstein bound of the quintic correction.
struct Candidate {
    coeffs: Vec<f64>,
    v_mid: f64,
    dv_max: f64,
    correction_max: f64,
}

/// Quadratic Taylor of the advance about the midpoint velocity, plus the
/// quintic Hermite correction that restores exact endpoint `(y, ẏ, ÿ)`.
fn composed_candidate(coeffs: &[f64], h: f64, adv: NonlinearAdvance, out_len: usize) -> Candidate {
    let (_, v_mid, _, _) = state_at(coeffs, 0.5 * h);
    let a0 = adv.advance(v_mid);
    let a1 = adv.slope(v_mid);
    let a2 = adv.curvature(v_mid);

    let mut q = vec![0.0; out_len];
    q[..coeffs.len()].copy_from_slice(coeffs);
    q[0] += a0;
    // dv(τ) = v(τ) − v_mid, as a monomial polynomial.
    let mut dv = vec![0.0; coeffs.len().max(2) - 1];
    for (i, d) in dv.iter_mut().enumerate() {
        *d = coeffs.get(i + 1).map_or(0.0, |c| (i + 1) as f64 * c);
    }
    dv[0] -= v_mid;
    for (i, &d) in dv.iter().enumerate() {
        q[i] = a1.mul_add(d, q[i]);
    }
    for (i, &di) in dv.iter().enumerate() {
        for (j, &dj) in dv.iter().enumerate() {
            q[i + j] = (0.5 * a2 * di).mul_add(dj, q[i + j]);
        }
    }

    let (r0, r0v, r0a) = endpoint_residual(coeffs, &q, 0.0, adv);
    let (r1, r1v, r1a) = endpoint_residual(coeffs, &q, h, adv);
    let mut c = [0.0_f64; 6];
    c[0] = r0;
    c[1] = r0v;
    c[2] = 0.5 * r0a;
    let p_gap = r1 - (r0 + h * (r0v + h * 0.5 * r0a));
    let v_gap = r1v - (r0v + h * r0a);
    let a_gap = r1a - r0a;
    let h2 = h * h;
    c[3] = (10.0 * p_gap - 4.0 * v_gap * h + 0.5 * a_gap * h2) / (h2 * h);
    c[4] = (-15.0 * p_gap + 7.0 * v_gap * h - a_gap * h2) / (h2 * h2);
    c[5] = (6.0 * p_gap - 3.0 * v_gap * h + 0.5 * a_gap * h2) / (h2 * h2 * h);
    for (qk, ck) in q.iter_mut().zip(c.iter()) {
        *qk += ck;
    }
    Candidate {
        coeffs: q,
        v_mid,
        dv_max: bernstein_max_abs(&dv, h),
        correction_max: bernstein_max_abs(&c, h),
    }
}

/// Rigorous `max |p(τ)|` bound on `[0, h]`: the Bernstein coefficients of a
/// polynomial bound it by their convex hull, `b_k = Σ_i C(k,i)/C(n,i)·a_i·hⁱ`.
fn bernstein_max_abs(coeffs: &[f64], h: f64) -> f64 {
    let n = coeffs.len() - 1;
    let mut worst = 0.0_f64;
    for k in 0..=n {
        let mut b = 0.0_f64;
        let mut ratio = 1.0_f64;
        let mut h_pow = 1.0_f64;
        for (i, &a) in coeffs.iter().enumerate().take(k + 1) {
            if i > 0 {
                ratio *= (k + 1 - i) as f64 / (n + 1 - i) as f64;
                h_pow *= h;
            }
            b = (a * ratio).mul_add(h_pow, b);
        }
        worst = worst.max(b.abs());
    }
    worst
}

fn endpoint_residual(
    coeffs: &[f64],
    q: &[f64],
    tau: f64,
    adv: NonlinearAdvance,
) -> (f64, f64, f64) {
    let (y, yv, ya) = exact_output(coeffs, tau, adv);
    let (qp, qv, qa, _) = state_at(q, tau);
    (y - qp, yv - qv, ya - qa)
}

fn candidate_fits(
    coeffs: &[f64],
    candidate: &Candidate,
    h: f64,
    adv: NonlinearAdvance,
    tol: FitTol,
    waiver: AccelWaiver,
) -> bool {
    let mut plain = true;
    let mut accel_waivable = true;
    for &u in &PROBES_U {
        let tau = u * h;
        let (y, _, ya) = exact_output(coeffs, tau, adv);
        let (qp, _, qa, _) = state_at(&candidate.coeffs, tau);
        assert!(
            y.is_finite() && qp.is_finite(),
            "nonlinear advance: non-finite sample at tau {tau}"
        );
        let pos_err = (y - qp).abs();
        let acc_err = (ya - qa).abs();
        plain &= pos_err <= tol.pos_mm && acc_err <= tol.accel_mm_s2;
        accel_waivable &= acc_err <= waiver.rel.mul_add(ya.abs(), tol.accel_mm_s2);
    }
    if plain {
        return true;
    }
    if !accel_waivable {
        return false;
    }
    let dv = candidate.dv_max;
    let mut remainder = adv.third_derivative_bound() / 6.0 * dv * dv * dv;
    if candidate.v_mid.abs() <= dv {
        remainder += 0.5 * adv.rest_curvature_jump() * dv * dv;
    }
    remainder + candidate.correction_max <= waiver.pos_mm
}

/// The advance law's exact `(y, ẏ, ÿ)` from the input piece's state:
/// `y = p + a(v)`, `ẏ = v + a'(v)·acc`, `ÿ = acc + a''(v)·acc² + a'(v)·jerk`.
fn exact_output(coeffs: &[f64], tau: f64, adv: NonlinearAdvance) -> (f64, f64, f64) {
    let (p, v, acc, jerk) = state_at(coeffs, tau);
    let slope = adv.slope(v);
    (
        p + adv.advance(v),
        slope.mul_add(acc, v),
        adv.curvature(v)
            .mul_add(acc * acc, slope.mul_add(jerk, acc)),
    )
}

/// Position and first three derivatives of a monomial piece at `tau`, by the
/// synthetic-division fold (Taylor coefficients scale by `k!`).
fn state_at(coeffs: &[f64], tau: f64) -> (f64, f64, f64, f64) {
    let (mut p, mut v, mut a, mut j) = (0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64);
    for &c in coeffs.iter().rev() {
        j = nurbs::fmadd(j, tau, a);
        a = nurbs::fmadd(a, tau, v);
        v = nurbs::fmadd(v, tau, p);
        p = nurbs::fmadd(p, tau, c);
    }
    (p, v, 2.0 * a, 6.0 * j)
}

/// Taylor shift: coefficients of the same polynomial about `τ' = τ − dt`.
fn shifted_monomial(coeffs: &[f64], dt: f64) -> Vec<f64> {
    let mut c = coeffs.to_vec();
    let n = c.len();
    for k in 0..n.saturating_sub(1) {
        for j in (k..n - 1).rev() {
            c[j] = nurbs::fmadd(dt, c[j + 1], c[j]);
        }
    }
    c
}

#[cfg(test)]
mod tests;
