use std::ops::{Add, Mul, Sub};

/// A value and its first three derivatives along one variable, carried
/// through arithmetic by the product and chain rules.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(in crate::follower_projection) struct Jet {
    pub(in crate::follower_projection) value: f64,
    pub(in crate::follower_projection) d1: f64,
    pub(in crate::follower_projection) d2: f64,
    pub(in crate::follower_projection) d3: f64,
}

impl Jet {
    pub(in crate::follower_projection) const ZERO: Self = Self::constant(0.0);

    pub(in crate::follower_projection) const fn constant(value: f64) -> Self {
        Self {
            value,
            d1: 0.0,
            d2: 0.0,
            d3: 0.0,
        }
    }

    pub(in crate::follower_projection) const fn variable(value: f64) -> Self {
        Self {
            value,
            d1: 1.0,
            d2: 0.0,
            d3: 0.0,
        }
    }

    /// `Self::variable(value) * factor`, bit for bit, without the products
    /// by the variable's zero derivatives: with `factor.value` positive
    /// those are `+0`, and adding `+0.0` keeps what adding them makes of
    /// each sum.
    pub(in crate::follower_projection) fn variable_times(value: f64, factor: Self) -> Self {
        assert!(
            factor.value > 0.0 && factor.d1.is_finite(),
            "a variable is multiplied by {factor:?}, not a positive jet with a finite slope"
        );
        Self {
            value: value * factor.value,
            d1: factor.value + value * factor.d1,
            d2: (2.0 * factor.d1 + 0.0) + value * factor.d2,
            d3: (3.0 * factor.d2 + 0.0) + value * factor.d3,
        }
    }

    pub(in crate::follower_projection) fn scale(self, factor: f64) -> Self {
        Self {
            value: self.value * factor,
            d1: self.d1 * factor,
            d2: self.d2 * factor,
            d3: self.d3 * factor,
        }
    }

    pub(in crate::follower_projection) fn offset(self, by: f64) -> Self {
        Self {
            value: self.value + by,
            ..self
        }
    }

    pub(in crate::follower_projection) fn recip(self) -> Self {
        let a = self.value;
        let inverse = 1.0 / a;
        let inverse2 = inverse * inverse;
        let (a1, a2, a3) = (self.d1, self.d2, self.d3);
        Self {
            value: inverse,
            d1: -a1 * inverse2,
            d2: (2.0 * a1 * a1 - a * a2) * inverse2 * inverse,
            d3: (6.0 * a * a1 * a2 - a * a * a3 - 6.0 * a1 * a1 * a1) * inverse2 * inverse2,
        }
    }

    /// `Σ coeffs[i]·selfⁱ`, by Horner's rule from the leading coefficient.
    pub(in crate::follower_projection) fn poly(self, coeffs: &[f64]) -> Self {
        let (&leading, lower) = coeffs.split_last().expect("a polynomial has coefficients");
        lower
            .iter()
            .rev()
            .fold(Self::constant(leading), |acc, &coeff| {
                (acc * self).offset(coeff)
            })
    }
}

/// [`Jet::poly`]'s value alone, by the same arithmetic.
pub(in crate::follower_projection) fn horner(x: f64, coeffs: &[f64]) -> f64 {
    let (&leading, lower) = coeffs.split_last().expect("a polynomial has coefficients");
    lower
        .iter()
        .rev()
        .fold(leading, |acc, &coeff| acc * x + coeff)
}

impl Add for Jet {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self {
            value: self.value + other.value,
            d1: self.d1 + other.d1,
            d2: self.d2 + other.d2,
            d3: self.d3 + other.d3,
        }
    }
}

impl Sub for Jet {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        self + other.scale(-1.0)
    }
}

impl Mul for Jet {
    type Output = Self;

    fn mul(self, other: Self) -> Self {
        let (a0, a1, a2, a3) = (self.value, self.d1, self.d2, self.d3);
        let (b0, b1, b2, b3) = (other.value, other.d1, other.d2, other.d3);
        Self {
            value: a0 * b0,
            d1: a1 * b0 + a0 * b1,
            d2: a2 * b0 + 2.0 * a1 * b1 + a0 * b2,
            d3: a3 * b0 + 3.0 * a2 * b1 + 3.0 * a1 * b2 + a0 * b3,
        }
    }
}
