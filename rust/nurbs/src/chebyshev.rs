//! Monomial piece-polynomial basis shifts on the Chebyshev reference domain
//! u ∈ [−1, 1], where u = 2·(t − t_start)/duration − 1 (equivalently
//! τ = (h/2)·(u + 1) for local time τ ∈ [0, h]).

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

#[cfg(test)]
mod tests;
