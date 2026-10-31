use crate::AlgebraError;
use crate::bezier::binomial;

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
