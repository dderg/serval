//! Taylor models: rigorous polynomial enclosures of functions that are not
//! polynomial.
//!
//! A model of a function `f` on a window `[center − radius, center + radius]`
//! is a polynomial `P(x) = Σ cₖ xᵏ` in `x = (t − center) / radius` and a bound
//! `rad` with `|f − P(x)| ≤ rad` for every `|x| ≤ 1`, `P` evaluated exactly.
//! Every operation models the exact result of the operation on the modelled
//! functions, so a signal built from them — a polynomial track through an
//! advance law, a path's speed, an arc's cosine, a rail's defect against its
//! law — is enclosed by the formula its evaluator computes.
//!
//! # The bound
//!
//! An operation's `rad` gathers the operands' bounds as they propagate
//! (`‖a‖·rad_b` for a product, `‖a‖ = Σ|aₖ|` bounding `|P_a|` on `|x| ≤ 1`),
//! the coefficients' rounding (one float operation errs by at most
//! `u = 2⁻⁵³` of its result, a coefficient summed from `n` products by
//! `γ_{2n}` of its terms' magnitudes), the terms of a product past
//! [`TAYLOR_LEN`] (`|x|ᵏ ≤ 1`, so bounded by their magnitudes), and the
//! Lagrange remainder of an elementary function's series. The bounds are
//! themselves sums and products of non-negative floats, each computed by
//! fewer than 2¹² operations, so they err by less than 2⁻⁴⁰ of themselves;
//! [`upper`] inflates them by that, and by the smallest normal float
//! against underflow.

use std::ops::{Add, Mul, Neg};

/// Coefficients a model keeps; a product's higher terms join its bound.
pub const TAYLOR_LEN: usize = 18;

const UNIT_ROUNDING: f64 = f64::EPSILON / 2.0;
const BOUND_ROUNDING: f64 = 1.0 / (1_u64 << 40) as f64;

/// The share of a product's magnitude below which its trailing
/// coefficients join its bound.
const TRIM_SHARE: f64 = 1.0 / (1_u64 << 36) as f64;

/// Every libm elementary function the models read lands within this many
/// ulps of its exact value (musl documents under one for `sin`/`cos` and
/// under three for `tanh`).
const LIBM_ULPS: f64 = 4.0;

/// Disk radii around a real point the tanh remainder is tried at.
/// `|tanh(x + iy)|² = (cosh 2x − c)/(cosh 2x + c)` with `c = cos 2y`: for
/// `c ≥ 0` it is below one, for `c < 0` it falls with `x` from `tan² y`, so
/// on the disk of radius `ρ < π/2` about any real point `|tanh| ≤ max(1,
/// tan ρ)`, and by Cauchy's estimate `|tanh⁽ⁿ⁾/n!| ≤ max(1, tan ρ)·ρ⁻ⁿ` on
/// the real line.
const TANH_DISK_RADII: [f64; 6] = [0.785, 1.0, 1.2, 1.35, 1.45, 1.52];

/// `γₙ = n·u/(1 − n·u)`: the relative error of a float computed by `n`
/// roundings of products and quotients of exact data (Higham, §3.1).
pub fn gamma(roundings: usize) -> f64 {
    let accumulated = roundings as f64 * UNIT_ROUNDING;
    assert!(
        accumulated < 0.5,
        "rounding depth {roundings} defeats the a-priori bound"
    );
    accumulated / (1.0 - accumulated)
}

pub fn upper(bound: f64) -> f64 {
    let inflated = bound * (1.0 + BOUND_ROUNDING) + f64::MIN_POSITIVE;
    if inflated.is_nan() {
        f64::INFINITY
    } else {
        inflated
    }
}

/// An upper bound on `baseⁿ` for a non-negative `base`.
fn powi(base: f64, power: usize) -> f64 {
    upper(base.powi(i32::try_from(power).expect("a series is short")))
}

fn lower(bound: f64) -> f64 {
    bound * (1.0 - BOUND_ROUNDING) - f64::MIN_POSITIVE
}

/// A real number as a float and a bound on its distance from it.
#[derive(Clone, Copy, Debug)]
pub struct Ball {
    pub mid: f64,
    pub rad: f64,
}

impl Ball {
    pub const fn exact(value: f64) -> Self {
        Self {
            mid: value,
            rad: 0.0,
        }
    }

    /// A float that `roundings` multiplications and divisions of exact
    /// data produced: within `γ_n` of its own magnitude.
    pub fn rounded(value: f64, roundings: usize) -> Self {
        Self {
            mid: value,
            rad: upper(gamma(roundings) * value.abs()),
        }
    }

    fn libm(value: f64) -> Self {
        Self {
            mid: value,
            rad: upper(LIBM_ULPS * f64::EPSILON * value.abs()),
        }
    }

    fn div_by_integer(self, divisor: usize) -> Self {
        let divisor = divisor as f64;
        let mid = self.mid / divisor;
        Self {
            mid,
            rad: upper(self.rad / divisor + UNIT_ROUNDING * mid.abs()),
        }
    }
}

impl std::ops::Add for Ball {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        let mid = self.mid + other.mid;
        Self {
            mid,
            rad: upper(self.rad + other.rad + UNIT_ROUNDING * mid.abs()),
        }
    }
}

impl std::ops::Mul for Ball {
    type Output = Self;

    fn mul(self, other: Self) -> Self {
        let mid = self.mid * other.mid;
        Self {
            mid,
            rad: upper(
                self.mid.abs() * other.rad
                    + self.rad * other.mid.abs()
                    + self.rad * other.rad
                    + UNIT_ROUNDING * mid.abs(),
            ),
        }
    }
}

impl std::ops::Neg for Ball {
    type Output = Self;

    fn neg(self) -> Self {
        Self {
            mid: -self.mid,
            rad: self.rad,
        }
    }
}

/// `1/k!` for every series term and the remainder's power.
fn inverse_factorials() -> [Ball; TAYLOR_LEN + 1] {
    let mut out = [Ball::exact(1.0); TAYLOR_LEN + 1];
    for k in 1..=TAYLOR_LEN {
        out[k] = out[k - 1].div_by_integer(k);
    }
    out
}

#[derive(Clone, Copy, Debug)]
pub struct TaylorModel {
    coefficients: [f64; TAYLOR_LEN],
    len: usize,
    rad: f64,
}

impl TaylorModel {
    pub const UNBOUNDED: Self = Self {
        coefficients: [0.0; TAYLOR_LEN],
        len: 0,
        rad: f64::INFINITY,
    };

    pub fn constant(value: Ball) -> Self {
        let mut coefficients = [0.0; TAYLOR_LEN];
        coefficients[0] = value.mid;
        Self {
            coefficients,
            len: 1,
            rad: value.rad,
        }
    }

    /// The polynomial `coefficients` in the window's variable, within `rad`
    /// of the modelled function.
    pub fn from_polynomial(coefficients: &[f64], rad: f64) -> Self {
        assert!(
            coefficients.len() <= TAYLOR_LEN,
            "a {}-coefficient polynomial exceeds a Taylor model",
            coefficients.len()
        );
        let mut kept = [0.0; TAYLOR_LEN];
        kept[..coefficients.len()].copy_from_slice(coefficients);
        Self {
            coefficients: kept,
            len: coefficients.len(),
            rad,
        }
    }

    /// `offset + slope·x`, `slope` exact.
    pub fn affine(offset: Ball, slope: f64) -> Self {
        let mut coefficients = [0.0; TAYLOR_LEN];
        coefficients[0] = offset.mid;
        coefficients[1] = slope;
        Self {
            coefficients,
            len: 2,
            rad: offset.rad,
        }
    }

    pub fn rad(&self) -> f64 {
        self.rad
    }

    pub fn coefficients(&self) -> &[f64] {
        &self.coefficients[..self.len]
    }

    /// Loosens the bound by `extra`, what the model's formula is known to
    /// miss the modelled function by.
    pub fn widen(&mut self, extra: f64) {
        self.rad = upper(self.rad + extra);
    }

    fn magnitude(&self) -> f64 {
        self.coefficients().iter().map(|c| c.abs()).sum()
    }

    /// An upper bound on `|f|` over the window.
    pub fn bound(&self) -> f64 {
        upper(self.magnitude() + self.rad)
    }

    /// The constant term and an upper bound on how far `f` strays from it.
    pub fn spread(&self) -> (f64, f64) {
        let centre = self.coefficients.first().copied().unwrap_or(0.0);
        let others: f64 = self.coefficients().iter().skip(1).map(|c| c.abs()).sum();
        (centre, upper(others + self.rad))
    }

    fn combined(&self, other: &Self, sign: f64) -> Self {
        let len = self.len.max(other.len);
        let mut coefficients = [0.0; TAYLOR_LEN];
        let mut rounding = 0.0;
        for (power, slot) in coefficients.iter_mut().enumerate().take(len) {
            *slot = self.coefficients[power] + sign * other.coefficients[power];
            rounding += slot.abs();
        }
        Self {
            coefficients,
            len,
            rad: upper(self.rad + other.rad + UNIT_ROUNDING * rounding),
        }
    }

    pub fn add(&self, other: &Self) -> Self {
        self.combined(other, 1.0)
    }

    pub fn sub(&self, other: &Self) -> Self {
        self.combined(other, -1.0)
    }

    pub fn add_ball(&self, value: Ball) -> Self {
        let mut out = *self;
        if out.len == 0 {
            out.len = 1;
        }
        out.coefficients[0] += value.mid;
        out.rad = upper(self.rad + value.rad + UNIT_ROUNDING * out.coefficients[0].abs());
        out
    }

    pub fn scale(&self, factor: Ball) -> Self {
        let mut out = *self;
        let mut rounding = 0.0;
        for coefficient in &mut out.coefficients[..out.len] {
            *coefficient *= factor.mid;
            rounding += coefficient.abs();
        }
        out.rad = upper(
            factor.mid.abs() * self.rad
                + factor.rad * (self.magnitude() + self.rad)
                + UNIT_ROUNDING * rounding,
        );
        out
    }

    pub fn mul(&self, other: &Self) -> Self {
        let mut coefficients = [0.0; TAYLOR_LEN];
        let mut truncated = 0.0;
        for (i, &a) in self.coefficients().iter().enumerate() {
            for (j, &b) in other.coefficients().iter().enumerate() {
                let term = a * b;
                if i + j < TAYLOR_LEN {
                    coefficients[i + j] += term;
                } else {
                    truncated += term.abs();
                }
            }
        }
        let len = (self.len + other.len).saturating_sub(1).min(TAYLOR_LEN);
        let (a_magnitude, b_magnitude) = (self.magnitude(), other.magnitude());
        Self {
            coefficients,
            len,
            rad: upper(
                a_magnitude * other.rad
                    + self.rad * b_magnitude
                    + self.rad * other.rad
                    + truncated
                    + gamma(2 * TAYLOR_LEN) * a_magnitude * b_magnitude,
            ),
        }
        .trimmed()
    }

    /// The model with its trailing coefficients below [`TRIM_SHARE`] of its
    /// magnitude moved into the bound, where they cost every later product
    /// a term and move the bound by a share no fit tolerance resolves.
    fn trimmed(mut self) -> Self {
        let negligible = TRIM_SHARE * self.magnitude();
        let mut dropped = 0.0;
        while self.len > 1 && self.coefficients[self.len - 1].abs() <= negligible {
            self.len -= 1;
            dropped += self.coefficients[self.len].abs();
            self.coefficients[self.len] = 0.0;
        }
        self.rad = upper(self.rad + dropped);
        self
    }

    /// `f(self)` for an `f` with `f(c₀ + d) = Σₖ seriesₖ·dᵏ + R` on the
    /// model's range when `n` terms are kept, `|R| ≤ tail(n, reach)` for
    /// `|d| ≤ reach`. The series stops at the first `n` whose tail is
    /// within [`TRIM_SHARE`] of its leading terms, where a product would
    /// trim what the next terms add, so a narrow window costs few products.
    fn compose(&self, series: &[Ball], tail: impl Fn(usize, f64) -> f64) -> Self {
        let mut deviation = *self;
        deviation.coefficients[0] = 0.0;
        let reach = deviation.bound();
        let leading = upper(
            UNIT_ROUNDING * (series[0].mid.abs() + series[1].mid.abs() * reach) + f64::MIN_POSITIVE,
        );
        let tail = |terms: usize| upper(tail(terms, reach));
        let terms = (1..series.len())
            .find(|&terms| tail(terms) <= leading)
            .unwrap_or(series.len());
        let (last, rest) = series[..terms].split_last().expect("a series has terms");
        let mut out = Self::constant(*last);
        for &term in rest.iter().rev() {
            out = out.mul(&deviation).add_ball(term);
        }
        out.rad = upper(out.rad + tail(terms));
        out
    }

    pub fn sin_cos(&self) -> (Self, Self) {
        let (sine, cosine) = libm::sincos(self.coefficients[0]);
        let (sine, cosine) = (Ball::libm(sine), Ball::libm(cosine));
        let inverse = inverse_factorials();
        let cycle = |start: usize, k: usize| match (start + k) % 4 {
            0 => sine,
            1 => cosine,
            2 => sine.neg(),
            _ => cosine.neg(),
        };
        let series = |start: usize| -> [Ball; TAYLOR_LEN] {
            std::array::from_fn(|k| cycle(start, k).mul(inverse[k]))
        };
        let tail = |terms: usize, reach: f64| {
            upper(inverse[terms].mid + inverse[terms].rad) * powi(reach, terms)
        };
        (
            self.compose(&series(0), tail),
            self.compose(&series(1), tail),
        )
    }

    pub fn tanh(&self) -> Self {
        let mut series = [Ball::exact(0.0); TAYLOR_LEN];
        series[0] = Ball::libm(libm::tanh(self.coefficients[0]));
        series[1] = Ball::exact(1.0).add(series[0].mul(series[0]).neg());
        for k in 1..TAYLOR_LEN - 1 {
            let square = (0..=k).fold(Ball::exact(0.0), |sum, i| {
                sum.add(series[i].mul(series[k - i]))
            });
            series[k + 1] = square.neg().div_by_integer(k + 1);
        }
        let disk_bounds = TANH_DISK_RADII.map(|radius| {
            (
                radius,
                upper(libm::tan(radius) * (1.0 + LIBM_ULPS * f64::EPSILON)).max(1.0),
            )
        });
        self.compose(&series, |terms, reach| {
            disk_bounds
                .iter()
                .map(|&(radius, bound)| bound * powi(upper(reach / lower(radius)), terms))
                .fold(f64::INFINITY, f64::min)
        })
    }

    /// `1/f`, or `None` when the model's range reaches zero.
    pub fn reciprocal(&self) -> Option<Self> {
        let (centre, spread) = self.spread();
        let floor = lower(centre.abs() - spread);
        if floor.is_nan() || floor <= 0.0 {
            return None;
        }
        let first = Ball::rounded(1.0 / centre, 1);
        let mut series = [first; TAYLOR_LEN];
        for k in 1..TAYLOR_LEN {
            series[k] = series[k - 1].mul(first.neg());
        }
        Some(self.compose(&series, |terms, reach| {
            powi(upper(reach / floor), terms) * upper(1.0 / floor)
        }))
    }

    /// `√f`, or `None` when the model's range reaches zero.
    pub fn sqrt(&self) -> Option<Self> {
        let (centre, spread) = self.spread();
        let floor = lower(centre - spread);
        if floor.is_nan() || floor <= 0.0 {
            return None;
        }
        let inverse_centre = Ball::rounded(1.0 / centre, 1);
        let mut series = [Ball::rounded(centre.sqrt(), 1); TAYLOR_LEN];
        let mut binomials = [1.0_f64; TAYLOR_LEN + 1];
        for k in 1..=TAYLOR_LEN {
            let ratio = (3.0 - 2.0 * k as f64) / (2.0 * k as f64);
            binomials[k] = upper(binomials[k - 1] * ratio.abs());
            if k < TAYLOR_LEN {
                series[k] = series[k - 1]
                    .mul(Ball::rounded(ratio, 1))
                    .mul(inverse_centre);
            }
        }
        let root = upper(floor.sqrt());
        Some(self.compose(&series, |terms, reach| {
            binomials[terms] * root * powi(upper(reach / floor), terms)
        }))
    }

    /// `∫₀ʸ f` in the model's own variable `y`: exact on the polynomial,
    /// the bound grows by at most `rad·|y| ≤ rad`.
    pub fn integral(&self) -> Self {
        let len = (self.len + 1).min(TAYLOR_LEN);
        let mut coefficients = [0.0; TAYLOR_LEN];
        let mut rounding = 0.0;
        let mut truncated = 0.0;
        for (power, &c) in self.coefficients().iter().enumerate() {
            let term = c / (power + 1) as f64;
            if power + 1 < TAYLOR_LEN {
                coefficients[power + 1] = term;
                rounding += term.abs();
            } else {
                truncated += term.abs();
            }
        }
        Self {
            coefficients,
            len,
            rad: upper(self.rad + truncated + UNIT_ROUNDING * rounding),
        }
    }

    /// `outer(self)`, `outer` a model in a variable this model's function
    /// keeps within `[−1, 1]`; `None` when the range bound reaches past it.
    pub fn compose_model(&self, outer: &Self) -> Option<Self> {
        let (centre, spread) = self.spread();
        if !(upper(centre.abs() + spread) <= 1.0) {
            return None;
        }
        let (last, rest) = outer.coefficients().split_last()?;
        let mut out = Self::constant(Ball::exact(*last));
        for &c in rest.iter().rev() {
            out = out.mul(self).add_ball(Ball::exact(c));
        }
        out.rad = upper(out.rad + outer.rad);
        Some(out)
    }
}

/// The sub-interval a model is built on, and its local variable's map.
#[derive(Clone, Copy, Debug)]
pub struct Window {
    pub lo: f64,
    pub hi: f64,
    pub center: f64,
    pub radius: f64,
}

impl Window {
    pub fn new(lo: f64, hi: f64) -> Self {
        let center = 0.5 * lo + 0.5 * hi;
        Self {
            lo,
            hi,
            center,
            radius: upper((hi - center).max(center - lo)),
        }
    }

    /// `t − origin` on the window.
    pub fn time_since(&self, origin: f64) -> TaylorModel {
        TaylorModel::affine(Ball::rounded(self.center - origin, 1), self.radius)
    }

    pub fn reaches(&self, t: f64, slack: f64) -> bool {
        self.lo - slack <= t && t <= self.hi + slack
    }
}

/// The `order`-th derivative of `Σ cₖ τᵏ` for the model `tau` of `τ`, its
/// falling-factorial scaled coefficients rounded once each.
pub fn derivative_of(coefficients: &[f64], order: usize, tau: &TaylorModel) -> TaylorModel {
    polynomial_of_balls(
        coefficients
            .iter()
            .enumerate()
            .skip(order)
            .map(|(power, &c)| {
                if order == 0 {
                    return Ball::exact(c);
                }
                let falling = ((power + 1 - order)..=power).fold(1.0, |acc, k| acc * k as f64);
                Ball::rounded(c * falling, 1)
            }),
        tau,
    )
}

pub fn polynomial_of_balls(
    coefficients: impl DoubleEndedIterator<Item = Ball>,
    tau: &TaylorModel,
) -> TaylorModel {
    coefficients
        .rev()
        .fold(None, |acc: Option<TaylorModel>, c| {
            Some(match acc {
                None => TaylorModel::constant(c),
                Some(acc) => acc.mul(tau).add_ball(c),
            })
        })
        .unwrap_or_else(|| TaylorModel::constant(Ball::exact(0.0)))
}

#[cfg(test)]
mod tests;
