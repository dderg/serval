//! f32 Chebyshev evaluation for the ISR hot path: series on u ∈ [−1, 1].

/// Clenshaw evaluation of `Σ a_k·T_k(u)` — ~2·len FMAs, no table.
#[inline]
pub fn clenshaw(a: &[f32], u: f32) -> f32 {
    let Some((&a0, rest)) = a.split_first() else {
        return 0.0;
    };
    let mut b1 = 0.0_f32;
    let mut b2 = 0.0_f32;
    for &ak in rest.iter().rev() {
        let b0 = ak + 2.0 * u * b1 - b2;
        b2 = b1;
        b1 = b0;
    }
    a0 + u * b1 - b2
}

/// Chebyshev series of `du_dt · d/du Σ a_k·T_k(u)` written into `out`:
/// `d_{n−2} = 2(n−1)·a_{n−1}`, `d_j = d_{j+2} + 2(j+1)·a_{j+1}`, `d_0` halved.
/// Coefficients of `out` beyond the series are zeroed. Returns the coefficient
/// count of the derivative series (≥ 1).
#[inline]
#[allow(clippy::cast_precision_loss)]
pub fn derivative_series(a: &[f32], du_dt: f32, out: &mut [f32]) -> usize {
    for o in out.iter_mut() {
        *o = 0.0;
    }
    let n = a.len();
    if n <= 1 {
        return 1;
    }
    debug_assert!(out.len() >= n - 1, "derivative output buffer too small");
    let a_last = a.get(n - 1).copied().unwrap_or(0.0);
    if let Some(s) = out.get_mut(n - 2) {
        *s = 2.0 * ((n - 1) as f32) * a_last;
    }
    if n >= 3 {
        for j in (0..=(n - 3)).rev() {
            let d_j2 = out.get(j + 2).copied().unwrap_or(0.0);
            let a_j1 = a.get(j + 1).copied().unwrap_or(0.0);
            if let Some(s) = out.get_mut(j) {
                *s = d_j2 + 2.0 * ((j + 1) as f32) * a_j1;
            }
        }
    }
    if let Some(s) = out.get_mut(0) {
        *s *= 0.5;
    }
    for o in out.iter_mut().take(n - 1) {
        *o *= du_dt;
    }
    n - 1
}

#[cfg(test)]
mod tests;
