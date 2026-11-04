use crate::AlgebraError;
use crate::bezier::binomial;

const VARIATION_STEPS: usize = 512;
const MOMENT_GRID_STEPS: usize = 1024;
const SHARE_SEARCH_STEPS: u32 = 48;

#[derive(Debug, Clone)]
pub struct PiecewisePolynomialKernel {
    pub pieces: Vec<crate::bezier::BezierPiece>,
}

impl PiecewisePolynomialKernel {
    pub fn single_poly(coeffs: Vec<f64>, support: (f64, f64)) -> Self {
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
    pub fn single_poly_from_absolute(coeffs: Vec<f64>, support: (f64, f64)) -> Self {
        let shifted = absolute_to_pascal_shift(&coeffs, support.0);
        Self::single_poly(shifted, support)
    }

    pub fn support(&self) -> (f64, f64) {
        (
            self.pieces.first().unwrap().u_start,
            self.pieces.last().unwrap().u_end,
        )
    }

    #[must_use]
    pub fn second_moment(&self) -> f64 {
        self.pieces
            .iter()
            .map(|piece| {
                let u0 = piece.u_start;
                let h = piece.u_end - piece.u_start;
                piece
                    .coeffs
                    .iter()
                    .enumerate()
                    .map(|(k, &c)| {
                        let k = k as i32;
                        c * (u0 * u0 * h.powi(k + 1) / f64::from(k + 1)
                            + 2.0 * u0 * h.powi(k + 2) / f64::from(k + 2)
                            + h.powi(k + 3) / f64::from(k + 3))
                    })
                    .sum::<f64>()
            })
            .sum()
    }

    /// `(∫|w|, ∫|u|·|w(u)| du)` over the support, by the midpoint rule on
    /// [`MOMENT_GRID_STEPS`] steps per piece.
    #[must_use]
    pub fn absolute_moments(&self) -> (f64, f64) {
        self.pieces
            .iter()
            .fold((0.0, 0.0), |(mass, moment), piece| {
                let h = (piece.u_end - piece.u_start) / MOMENT_GRID_STEPS as f64;
                (0..MOMENT_GRID_STEPS).fold((mass, moment), |(mass, moment), step| {
                    let u = piece.u_start + (step as f64 + 0.5) * h;
                    let weight = piece.evaluate(u).abs() * h;
                    (mass + weight, moment + u.abs() * weight)
                })
            })
    }

    pub fn from_pieces(pieces: Vec<crate::bezier::BezierPiece>) -> Result<Self, AlgebraError> {
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

    /// The jerk per unit of input acceleration a set of kernels, one per
    /// axis, lets the shaped signal reach along the worst direction: an
    /// input acceleration `a(u)` in the unit disk leaves a shaped jerk along
    /// the unit `d` of `Σᵢ dᵢ ∫ aᵢ(u)·wᵢ′(t − u) du`, at most
    /// `∫ |(dᵢ·wᵢ′(u))ᵢ| du` with `a` aligned to that vector throughout, a
    /// jump of a kernel counting as an impulse in its slope. For one kernel
    /// it is the kernel's total variation, `2·max w` for a unimodal one; a
    /// multimodal kernel's lobes each take a fresh swing of the input.
    ///
    /// The variation is concave in the squared direction `sᵢ = dᵢ²` over the
    /// simplex, so a golden-section search per share finds its largest.
    /// Slopes are integrated by the midpoint rule on [`VARIATION_STEPS`]
    /// steps between the kernels' breakpoints.
    #[must_use]
    pub fn directional_variation(kernels: &[&Self]) -> f64 {
        assert!(
            !kernels.is_empty(),
            "a directional variation needs a kernel"
        );
        let mut breaks: Vec<f64> = kernels
            .iter()
            .flat_map(|kernel| kernel.pieces.iter().flat_map(|p| [p.u_start, p.u_end]))
            .collect();
        breaks.sort_by(f64::total_cmp);
        breaks.dedup();
        let slopes: Vec<Vec<crate::bezier::BezierPiece>> = kernels
            .iter()
            .map(|kernel| kernel.pieces.iter().map(|p| p.differentiate()).collect())
            .collect();
        let mut weights = Vec::new();
        let mut squares = Vec::new();
        for &at in &breaks {
            weights.push(1.0);
            squares.extend(kernels.iter().map(|kernel| {
                (kernel.value_beside(at, true) - kernel.value_beside(at, false)).powi(2)
            }));
        }
        for span in breaks.windows(2) {
            let h = (span[1] - span[0]) / VARIATION_STEPS as f64;
            for step in 0..VARIATION_STEPS {
                let u = span[0] + (step as f64 + 0.5) * h;
                weights.push(h);
                squares.extend(slopes.iter().map(|pieces| piece_value(pieces, u).powi(2)));
            }
        }
        let variation = |shares: &[f64]| {
            weights
                .iter()
                .zip(squares.chunks_exact(kernels.len()))
                .map(|(weight, square)| {
                    weight
                        * square
                            .iter()
                            .zip(shares)
                            .map(|(q, s)| q * s)
                            .sum::<f64>()
                            .sqrt()
                })
                .sum::<f64>()
        };
        largest_from(&mut vec![0.0; kernels.len()], 0, 1.0, &variation)
    }

    /// `w` just right (`right`) or left of `u`, 0 outside the support.
    fn value_beside(&self, u: f64, right: bool) -> f64 {
        self.pieces
            .iter()
            .find(|p| {
                if right {
                    p.u_start <= u && u < p.u_end
                } else {
                    p.u_start < u && u <= p.u_end
                }
            })
            .map_or(0.0, |p| p.evaluate(u))
    }
}

fn piece_value(pieces: &[crate::bezier::BezierPiece], u: f64) -> f64 {
    pieces
        .iter()
        .find(|p| p.u_start <= u && u < p.u_end)
        .map_or(0.0, |p| p.evaluate(u))
}

/// The largest `f` over the shares from `shares[axis]` on, which sum to
/// `left`, the earlier shares held: `f` is concave there, and so is its
/// largest over the later shares as a function of this one.
fn largest_from(shares: &mut [f64], axis: usize, left: f64, f: &dyn Fn(&[f64]) -> f64) -> f64 {
    if axis + 1 == shares.len() {
        shares[axis] = left;
        return f(shares);
    }
    let mut at = |share: f64| {
        shares[axis] = share;
        largest_from(shares, axis + 1, left - share, f)
    };
    let golden = 0.5 * (5.0_f64.sqrt() - 1.0);
    let mut best = at(0.0).max(at(left));
    let (mut lo, mut hi) = (0.0, left);
    let mut inner = (hi - golden * (hi - lo), lo + golden * (hi - lo));
    let mut values = (at(inner.0), at(inner.1));
    for _ in 0..SHARE_SEARCH_STEPS {
        best = best.max(values.0).max(values.1);
        if values.0 < values.1 {
            lo = inner.0;
            inner = (inner.1, lo + golden * (hi - lo));
            values = (values.1, at(inner.1));
        } else {
            hi = inner.1;
            inner = (hi - golden * (hi - lo), inner.0);
            values = (at(inner.0), values.0);
        }
    }
    best.max(values.0).max(values.1)
}

fn absolute_to_pascal_shift(absolute: &[f64], shift: f64) -> Vec<f64> {
    let d = absolute.len() - 1;
    let mut out = vec![0.0; d + 1];
    let mut shift_pow = vec![1.0; d + 1];
    for k in 1..=d {
        shift_pow[k] = shift_pow[k - 1] * shift;
    }
    for n in 0..=d {
        for k in 0..=n {
            let bin = binomial(n, k) as f64;
            out[k] += absolute[n] * bin * shift_pow[n - k];
        }
    }
    out
}

#[cfg(test)]
#[allow(clippy::float_cmp)]
mod tests;
