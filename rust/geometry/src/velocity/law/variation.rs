//! How far a rail knot pair's interpolated acceleration moves over a
//! stretch of its time.
//!
//! On a pair the acceleration is a polynomial `p(w) = Σ pₖ·wᵏ` of degree at
//! most four in the pair's variable `w`, which runs monotonically in time:
//! the time share `u`, or `z`, the square root of the time from a departure
//! end. Its total variation is the same in either clock, `∫|p′(w)| dw` over
//! the stretch's `w`. Shifted onto that stretch, `q(y) = p(w₀ + d·y)` with
//! `y ∈ [0, 1]`, the derivative `q′ = Σ βᵢ·Bᵢ,₃` in the cubic Bernstein
//! basis, whose members are non-negative with `∫₀¹ Bᵢ,₃ = ¼`, so
//! `∫|q′| ≤ ¼·Σ|βᵢ|`. Unlike a rate, the bound stays finite at a departure
//! end, where the acceleration rises as the square root of the time.

use taylor_model::{gamma, upper};

use super::defect::{acceleration_bound, quintic_monomials};
use super::{RailPair, RailPairShape};

const DEGREE: usize = 4;

/// Roundings a Bernstein coefficient below carries at most: the pair's
/// coefficients from its knot data, the shift onto the stretch, the powers
/// of its width and the basis change.
const ROUNDINGS: usize = 32;

/// Relative ulps the ends of a stretch in the pair's variable may sit off
/// their exact place, after the square root and the quotients that read
/// them off the time.
const VARIABLE_ULPS: f64 = 8.0;

/// A pair's acceleration as `Σ pₖ·wᵏ` and the clock its variable `w` runs
/// on.
struct AccelerationPolynomial {
    coefficients: [f64; DEGREE + 1],
    variable: Variable,
}

enum Variable {
    /// `w = (τ − t_lo)/h`.
    Share { t_lo: f64, h: f64 },
    /// `w = √(θ/scale)`, `θ = theta_at_lo + rate·(τ − t_lo)`.
    Root {
        t_lo: f64,
        theta_at_lo: f64,
        rate: f64,
        scale: f64,
    },
}

impl Variable {
    fn at(&self, tau: f64) -> f64 {
        match *self {
            Variable::Share { t_lo, h } => (tau - t_lo) / h,
            Variable::Root {
                t_lo,
                theta_at_lo,
                rate,
                scale,
            } => (rate.mul_add(tau - t_lo, theta_at_lo).max(0.0) / scale).sqrt(),
        }
    }
}

impl RailPair {
    fn acceleration_polynomial(&self) -> AccelerationPolynomial {
        let h = self.t_hi - self.t_lo;
        let root_terms = |[c2, c3, c4, c5]: [f64; 4], scale: f64| {
            [c2, 1.5 * c3, 2.0 * c4, 2.5 * c5, 0.0].map(|c| c * scale)
        };
        match self.shape {
            RailPairShape::Quintic { rise, slopes } => {
                let speed = quintic_monomials(0.0, rise, slopes);
                AccelerationPolynomial {
                    coefficients: std::array::from_fn(|power| {
                        (power + 1) as f64 * speed[power + 1] / h
                    }),
                    variable: Variable::Share { t_lo: self.t_lo, h },
                }
            }
            RailPairShape::Departure {
                from_lo,
                coefficients,
            } => {
                let (sign, theta_at_lo) = if from_lo { (1.0, 0.0) } else { (-1.0, h) };
                AccelerationPolynomial {
                    coefficients: root_terms(coefficients, sign / h),
                    variable: Variable::Root {
                        t_lo: self.t_lo,
                        theta_at_lo,
                        rate: sign,
                        scale: h,
                    },
                }
            }
            RailPairShape::Clipped {
                coefficients,
                h: parent,
                theta_lo,
                theta_hi,
                ..
            } => {
                let direction = (theta_hi - theta_lo).signum();
                AccelerationPolynomial {
                    coefficients: root_terms(coefficients, direction / parent),
                    variable: Variable::Root {
                        t_lo: self.t_lo,
                        theta_at_lo: theta_lo,
                        rate: (theta_hi - theta_lo) / h,
                        scale: parent,
                    },
                }
            }
        }
    }

    /// The interpolated tangential acceleration at segment-local time
    /// `tau`, clamped to the pair: the formula
    /// [`super::LawSegment::state_at`] reads inside it, taken at either end
    /// from this pair's side.
    pub fn acceleration_at(&self, tau: f64) -> f64 {
        let polynomial = self.acceleration_polynomial();
        let w = polynomial.variable.at(tau.clamp(self.t_lo, self.t_hi));
        polynomial
            .coefficients
            .iter()
            .rev()
            .fold(0.0, |acc, &c| acc.mul_add(w, c))
    }

    /// An upper bound on `|a|` of the pair's interpolant.
    pub fn acceleration_bound(&self) -> f64 {
        acceleration_bound(self)
    }

    /// An upper bound on the total variation of the interpolated tangential
    /// acceleration over segment-local time `[tau_lo, tau_hi]` within the
    /// pair; zero where they do not meet.
    pub fn acceleration_variation(&self, tau_lo: f64, tau_hi: f64) -> f64 {
        let (lo, hi) = (tau_lo.max(self.t_lo), tau_hi.min(self.t_hi));
        if lo >= hi {
            return 0.0;
        }
        let polynomial = self.acceleration_polynomial();
        let (w_lo, w_hi) = (polynomial.variable.at(lo), polynomial.variable.at(hi));
        let slack = |w: f64| VARIABLE_ULPS * f64::EPSILON * w.abs();
        let from = w_lo.min(w_hi);
        let to = w_lo.max(w_hi);
        derivative_mass(polynomial.coefficients, from - slack(from), to + slack(to))
    }
}

/// An upper bound on `∫|p′|` over `[from, to]`, `p` with `coefficients`.
fn derivative_mass(coefficients: [f64; DEGREE + 1], from: f64, to: f64) -> f64 {
    let width = to - from;
    let signed = derivative_bernstein(coefficients, from, width);
    let magnitude = derivative_bernstein(coefficients.map(f64::abs), from.abs(), width);
    let total: f64 = signed
        .iter()
        .zip(&magnitude)
        .map(|(beta, size)| beta.abs() + gamma(ROUNDINGS) * size)
        .sum();
    upper(total / DEGREE as f64)
}

/// The cubic Bernstein coefficients on `y ∈ [0, 1]` of `d/dy p(from + width·y)`.
fn derivative_bernstein(mut shifted: [f64; DEGREE + 1], from: f64, width: f64) -> [f64; DEGREE] {
    for start in 0..DEGREE {
        for power in (start..DEGREE).rev() {
            shifted[power] = shifted[power + 1].mul_add(from, shifted[power]);
        }
    }
    let mut scale = 1.0;
    let r: [f64; DEGREE] = std::array::from_fn(|power| {
        scale *= width;
        (power + 1) as f64 * shifted[power + 1] * scale
    });
    [
        r[0],
        r[0] + r[1] / 3.0,
        r[0] + 2.0 * r[1] / 3.0 + r[2] / 3.0,
        r[0] + r[1] + r[2] + r[3],
    ]
}

#[cfg(test)]
mod tests;
