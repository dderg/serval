//! Chebyshev-basis conversions for piece polynomials on u ∈ [−1, 1], where
//! u = 2·(t − t_start)/duration − 1 (equivalently τ = (h/2)·(u + 1) for local
//! time τ ∈ [0, h]).
//!
//! The monomial-in-u → Chebyshev step is built from u·T_k = (T_{k−1} + T_{k+1})/2,
//! whose coefficients are dyadic rationals — exact in f64. A degree-k monomial
//! input therefore yields exactly-zero Chebyshev coefficients above index k,
//! which the enqueue path relies on to recover the true piece degree.

/// `q(x) = p(x + dt)`, same length as the input.
pub fn taylor_shift(coeffs: &[f64], dt: f64) -> Vec<f64> {
    let mut c = coeffs.to_vec();
    let n = c.len();
    for i in 0..n.saturating_sub(1) {
        for j in (i..n - 1).rev() {
            c[j] += dt * c[j + 1];
        }
    }
    c
}

/// Chebyshev coefficients of `u · f(u)` given those of `f(u)`.
fn multiply_by_u(c: &[f64]) -> Vec<f64> {
    let mut out = vec![0.0; c.len() + 1];
    if let Some(&c0) = c.first() {
        out[1] += c0;
    }
    for (k, &ck) in c.iter().enumerate().skip(1) {
        out[k - 1] += 0.5 * ck;
        out[k + 1] += 0.5 * ck;
    }
    out
}

/// Monomial coefficients in u ∈ [−1, 1] → Chebyshev coefficients (exact
/// dyadic table; zero tails preserved exactly).
pub fn monomial_u_to_chebyshev(coeffs: &[f64]) -> Vec<f64> {
    let mut out = vec![0.0; coeffs.len().max(1)];
    let mut u_pow_cheb = vec![1.0];
    for (k, &ck) in coeffs.iter().enumerate() {
        for (o, &t) in out.iter_mut().zip(&u_pow_cheb) {
            *o += ck * t;
        }
        if k + 1 < coeffs.len() {
            u_pow_cheb = multiply_by_u(&u_pow_cheb);
        }
    }
    out
}

/// Inverse of [`monomial_u_to_chebyshev`] (integer T_k table).
pub fn chebyshev_to_monomial_u(a: &[f64]) -> Vec<f64> {
    let n = a.len().max(1);
    let mut out = vec![0.0; n];
    // T_k in monomial-u via T_{k+1} = 2u·T_k − T_{k−1} (integer coefficients).
    let mut t_prev = vec![1.0];
    let mut t_cur = vec![0.0, 1.0];
    for (k, &ak) in a.iter().enumerate() {
        let t_k: &[f64] = match k {
            0 => &t_prev,
            _ => &t_cur,
        };
        for (o, &t) in out.iter_mut().zip(t_k) {
            *o += ak * t;
        }
        if k >= 1 && k + 1 < a.len() {
            let mut t_next = vec![0.0; k + 2];
            for (j, &t) in t_cur.iter().enumerate() {
                t_next[j + 1] += 2.0 * t;
            }
            for (j, &t) in t_prev.iter().enumerate() {
                t_next[j] -= t;
            }
            t_prev = core::mem::replace(&mut t_cur, t_next);
        }
    }
    out
}

/// Monomial coefficients in local time τ ∈ [0, h] → Chebyshev coefficients on
/// u ∈ [−1, 1] under τ = (h/2)·(u + 1).
pub fn monomial_tau_to_chebyshev(coeffs: &[f64], h: f64) -> Vec<f64> {
    let s = 0.5 * h;
    let mut pow = 1.0;
    let scaled: Vec<f64> = coeffs
        .iter()
        .map(|&c| {
            let v = c * pow;
            pow *= s;
            v
        })
        .collect();
    monomial_u_to_chebyshev(&taylor_shift(&scaled, 1.0))
}

/// Inverse of [`monomial_tau_to_chebyshev`].
pub fn chebyshev_to_monomial_tau(a: &[f64], h: f64) -> Vec<f64> {
    let in_u = chebyshev_to_monomial_u(a);
    let shifted = taylor_shift(&in_u, -1.0);
    let s = 2.0 / h;
    let mut pow = 1.0;
    shifted
        .iter()
        .map(|&c| {
            let v = c * pow;
            pow *= s;
            v
        })
        .collect()
}

/// Drop trailing Chebyshev coefficients while the exact sup-norm bound of the
/// dropped tail (Σ|dropped|, since |T_k| ≤ 1) stays within `tol`. Always keeps
/// at least one coefficient.
pub fn truncate_chebyshev(a: &[f64], tol: f64) -> Vec<f64> {
    let mut n = a.len().max(1);
    let mut dropped = 0.0;
    while n > 1 {
        let c = a[n - 1].abs();
        if dropped + c <= tol {
            dropped += c;
            n -= 1;
        } else {
            break;
        }
    }
    a.get(..n).map_or_else(|| vec![0.0], <[f64]>::to_vec)
}

/// Endpoint-derivative magnitudes of `T_k` on `u ∈ [−1, 1]`:
/// `|T_k(±1)| = 1`, `|T_k′(±1)| = k²`, `|T_k″(±1)| = k²(k²−1)/3`.
fn endpoint_weights(k: usize) -> (f64, f64) {
    let k2 = (k * k) as f64;
    (k2, k2 * (k2 - 1.0) / 3.0)
}

/// [`truncate_chebyshev`] with the dropped tail additionally bounded in
/// endpoint velocity and acceleration (time-domain units, piece duration `h`):
/// dropping `a_k` moves the seam accel by up to `|a_k|·k²(k²−1)/3·(2/h)²`,
/// which for a short piece dwarfs the positional effect — a sup-norm-only
/// budget would silently break C² seams.
pub fn truncate_chebyshev_c2(
    a: &[f64],
    h: f64,
    pos_tol: f64,
    vel_tol: f64,
    acc_tol: f64,
) -> Vec<f64> {
    let du_dt = 2.0 / h;
    let mut n = a.len().max(1);
    let (mut pos, mut vel, mut acc) = (0.0, 0.0, 0.0);
    while n > 1 {
        let k = n - 1;
        let c = a[k].abs();
        let (w_v, w_a) = endpoint_weights(k);
        let pos_next = pos + c;
        let vel_next = vel + c * w_v * du_dt;
        let acc_next = acc + c * w_a * du_dt * du_dt;
        if pos_next <= pos_tol && vel_next <= vel_tol && acc_next <= acc_tol {
            (pos, vel, acc) = (pos_next, vel_next, acc_next);
            n -= 1;
        } else {
            break;
        }
    }
    a.get(..n).map_or_else(|| vec![0.0], <[f64]>::to_vec)
}

/// Series endpoint values `(f(−1), f(1))`: `T_k(±1) = (±1)^k`.
pub fn endpoint_positions(a: &[f64]) -> (f64, f64) {
    let mut start = 0.0;
    let mut end = 0.0;
    let mut sign = 1.0;
    for &ak in a {
        end += ak;
        start += sign * ak;
        sign = -sign;
    }
    (start, end)
}

/// [`truncate_chebyshev_c2`] plus an exact endpoint-position re-anchor: after
/// dropping the tail, `a_0`/`a_1` are adjusted so both endpoint positions
/// match the input series exactly. `T_0″ = T_1″ = 0`, so the endpoint
/// acceleration is untouched, and the velocity shift stays inside the dropped
/// tail's own budget. Without this, downstream C⁰ welding (the NURBS carrier
/// shares boundary control points) turns a position seam mismatch `δp` into
/// `O(δp/h²)` of acceleration error on a short neighbor.
pub fn truncate_chebyshev_c2_anchored(
    a: &[f64],
    h: f64,
    pos_tol: f64,
    vel_tol: f64,
    acc_tol: f64,
) -> Vec<f64> {
    let mut t = truncate_chebyshev_c2(a, h, pos_tol, vel_tol, acc_tol);
    if t.len() == a.len() {
        return t;
    }
    let (full_start, full_end) = endpoint_positions(a);
    let (cut_start, cut_end) = endpoint_positions(&t);
    let ds = full_start - cut_start;
    let de = full_end - cut_end;
    if ds == 0.0 && de == 0.0 {
        return t;
    }
    if t.len() < 2 {
        t.resize(2, 0.0);
    }
    t[0] += 0.5 * (de + ds);
    t[1] += 0.5 * (de - ds);
    t
}

#[cfg(test)]
mod tests;
