//! Proves a fitted piece within tolerance of a signal's exact polynomials.
//!
//! On every sub-interval the signal's exact polynomial `S` and the fitted
//! piece `P` are re-expressed over the same local variable `y ∈ [−1, 1]`,
//! and their difference `e = P − S` (for position and acceleration, and
//! velocity where it has a budget) is written in the Bernstein basis. By
//! the convex-hull property `max|e| ≤ maxᵢ|bᵢ|`. A patch whose hull plus
//! rounding term stays within the budget is proven; one whose end
//! coefficients (the error's exact values there) exceed it by more than the
//! rounding term is a miss; anything else is halved by de Casteljau, which
//! tightens the hull quadratically, down to [`HULL_MAX_DEPTH`] levels.
//! A patch still unsettled there is treated as a miss: rejecting a rung is
//! always sound, and the ladder then splits the span, whose halves carry
//! the hull's slack at a quarter of the width.
//!
//! # The rounding term
//!
//! `S`'s own term comes with its coefficients. Composing a polynomial
//! `Σ cₖ xᵏ` into `x = m + r·y` with `|m| + |r| ≤ 1` is a Taylor shift and a
//! scaling, at most `COMPOSE_DEPTH` roundings deep, so it errs by at most
//! `γ·Σ|cₖ|(|m| + |r|)ᵏ`. The subtraction adds `u(|Pₖ| + |Sₖ|)` per
//! coefficient. The Bernstein conversion is the shift `y = 2v − 1` and the
//! `C(i,j)/C(n,j)` sums: run on the coefficients' magnitudes with the shift
//! taken by `+1` it bounds the terms of every `bᵢ`, whose error is `γ` of
//! that. A de Casteljau level forms convex combinations of coefficients no
//! larger than the patch's largest, `n` roundings each. A coefficient error
//! `δₖ` moves `e` by at most `Σ|δₖ|` on `|y| ≤ 1`, and a Bernstein error moves
//! the hull bound by at most its own size, so the terms add.

use super::exact::{EXACT_CAPACITY, ExactPiece, ExactRounding};
use super::taylor_shift;
use crate::lowering::FitTol;

/// How many times a patch the hull cannot settle is halved before it is
/// treated as a miss.
pub(crate) const HULL_MAX_DEPTH: u32 = 6;
const COMPOSE_DEPTH: usize = 3 * EXACT_CAPACITY + 2;
const CONVERSION_DEPTH: usize = 5 * EXACT_CAPACITY + 4;

/// The fitted piece: monomial coefficients in `u ∈ [−1, 1]` over
/// `[t0, t1]`.
#[derive(Clone, Copy)]
pub(crate) struct FittedPiece<'a> {
    pub mono_u: &'a [f64],
    pub t0: f64,
    pub t1: f64,
}

/// What each derivative order may miss by.
#[derive(Clone, Copy)]
pub(crate) struct HullBudget {
    pub tol: FitTol,
    pub velocity: f64,
}

impl HullBudget {
    fn of_order(self, order: usize) -> f64 {
        match order {
            0 => self.tol.pos_mm,
            1 => self.velocity,
            _ => self.tol.accel_mm_s2,
        }
    }
}

/// A coefficient vector beside the bound on its rounding error as a
/// function on `|y| ≤ 1`.
struct Bounded {
    coefficients: [f64; EXACT_CAPACITY],
    len: usize,
    rounding: f64,
}

/// `Σ cₖ (mid + half·y)ᵏ` as coefficients in `y`, with its rounding term.
fn composed(coefficients: &[f64], mid: f64, half: f64) -> Bounded {
    let len = coefficients.len();
    assert!(
        len <= EXACT_CAPACITY,
        "a {len}-coefficient polynomial exceeds the hull"
    );
    let mut shifted = [0.0; EXACT_CAPACITY];
    taylor_shift(coefficients, mid, &mut shifted[..len]);
    let mut scale = 1.0;
    let reach = mid.abs() + half.abs();
    let mut magnitude = 0.0;
    let mut reach_power = 1.0;
    for power in 0..len {
        shifted[power] *= scale;
        magnitude += coefficients[power].abs() * reach_power;
        scale *= half;
        reach_power *= reach;
    }
    Bounded {
        coefficients: shifted,
        len,
        rounding: ExactRounding::gamma(COMPOSE_DEPTH) * magnitude,
    }
}

/// `mono_u`'s `order`-th time derivative, as monomial coefficients in `u`.
fn piece_derivative(mono_u: &[f64], order: usize, h: f64) -> Vec<f64> {
    let rate = 2.0 / h;
    let scale = (0..order).fold(1.0, |acc, _| acc * rate);
    (order..mono_u.len())
        .map(|power| {
            let falling = ((power - order + 1)..=power).fold(1.0, |acc, k| acc * k as f64);
            mono_u[power] * falling * scale
        })
        .collect()
}

/// The Bernstein coefficients on `y ∈ [−1, 1]` of a polynomial in `y`,
/// and the rounding bound of the conversion.
fn bernstein(coefficients: &[f64]) -> (Vec<f64>, f64) {
    let len = coefficients.len();
    let degree = len - 1;
    let mut on_unit = [0.0; EXACT_CAPACITY];
    let mut magnitudes = [0.0; EXACT_CAPACITY];
    taylor_shift(coefficients, -1.0, &mut on_unit[..len]);
    let absolute: Vec<f64> = coefficients.iter().map(|c| c.abs()).collect();
    taylor_shift(&absolute, 1.0, &mut magnitudes[..len]);
    let mut doubling = 1.0;
    for power in 0..len {
        on_unit[power] *= doubling;
        magnitudes[power] *= doubling;
        doubling *= 2.0;
    }
    let mut controls = vec![0.0; len];
    let mut worst_terms = 0.0_f64;
    for (index, control) in controls.iter_mut().enumerate() {
        let mut terms = 0.0;
        for power in 0..=index {
            let weight = binomial(index, power) / binomial(degree, power);
            *control += weight * on_unit[power];
            terms += weight * magnitudes[power];
        }
        worst_terms = worst_terms.max(terms);
    }
    (
        controls,
        ExactRounding::gamma(CONVERSION_DEPTH) * worst_terms,
    )
}

fn binomial(n: usize, k: usize) -> f64 {
    (0..k).fold(1.0, |acc, i| acc * (n - i) as f64 / (i + 1) as f64)
}

/// Halves Bernstein controls at their midpoint.
fn de_casteljau_halves(controls: &[f64]) -> (Vec<f64>, Vec<f64>) {
    let len = controls.len();
    let mut work = controls.to_vec();
    let mut left = Vec::with_capacity(len);
    let mut right = vec![0.0; len];
    left.push(work[0]);
    right[len - 1] = work[len - 1];
    for remaining in (1..len).rev() {
        for index in 0..remaining {
            work[index] = 0.5 * (work[index] + work[index + 1]);
        }
        left.push(work[0]);
        right[remaining - 1] = work[remaining - 1];
    }
    (left, right)
}

/// Where on `[from, to]` the error with these Bernstein controls is not
/// proven within `budget`, or `None` if it is.
fn first_unproven_patch(
    controls: &[f64],
    (from, to): (f64, f64),
    budget: f64,
    rounding: f64,
    level_rounding: f64,
    depth: u32,
) -> Option<f64> {
    let slack = rounding + f64::from(depth) * level_rounding;
    let (peak_index, peak) =
        controls
            .iter()
            .map(|c| c.abs())
            .enumerate()
            .fold((0, 0.0_f64), |worst, (index, value)| {
                if value > worst.1 {
                    (index, value)
                } else {
                    worst
                }
            });
    if !peak.is_finite() {
        return Some(0.5 * (from + to));
    }
    if peak + slack <= budget {
        return None;
    }
    let last = controls.len() - 1;
    if controls[0].abs() - slack > budget {
        return Some(from);
    }
    if controls[last].abs() - slack > budget {
        return Some(to);
    }
    if depth == HULL_MAX_DEPTH || last == 0 {
        return Some(from + (to - from) * peak_index as f64 / last.max(1) as f64);
    }
    let middle = 0.5 * (from + to);
    let (left, right) = de_casteljau_halves(controls);
    first_unproven_patch(
        &left,
        (from, middle),
        budget,
        rounding,
        level_rounding,
        depth + 1,
    )
    .or_else(|| {
        first_unproven_patch(
            &right,
            (middle, to),
            budget,
            rounding,
            level_rounding,
            depth + 1,
        )
    })
}

/// The `u` of a point of `[domain]` where `piece` is not proven within
/// `budget` of the signal whose exact polynomials over that domain are
/// `exact`, or `None` when every order holds its budget everywhere there.
pub(crate) fn first_unproven(
    exact: &[ExactPiece],
    piece: FittedPiece,
    domain: (f64, f64),
    budget: HullBudget,
) -> Option<f64> {
    let h = piece.t1 - piece.t0;
    let u_of = |t: f64| 2.0 * (t - piece.t0) / h - 1.0;
    let orders: &[usize] = if budget.velocity.is_finite() {
        &[0, 1, 2]
    } else {
        &[0, 2]
    };
    let derivatives: Vec<Vec<f64>> = (0..3)
        .map(|order| piece_derivative(piece.mono_u, order, h))
        .collect();
    for sub in exact {
        let (from, to) = (sub.lo.max(domain.0), sub.hi.min(domain.1));
        if from > to {
            continue;
        }
        let middle = 0.5 * (from + to);
        let half = 0.5 * (to - from);
        for &order in orders {
            let fitted = composed(&derivatives[order], u_of(middle), half * (2.0 / h));
            let signal = &sub.pva[order];
            let truth = composed(
                &signal.coefficients[..signal.len],
                (middle - sub.center) / sub.radius,
                half / sub.radius,
            );
            let len = fitted.len.max(truth.len).max(1);
            let mut error = vec![0.0; len];
            let mut subtraction = 0.0;
            for (power, slot) in error.iter_mut().enumerate() {
                let (p, s) = (fitted.coefficients[power], truth.coefficients[power]);
                *slot = p - s;
                subtraction += f64::EPSILON * 0.5 * (p.abs() + s.abs());
            }
            let (controls, conversion) = bernstein(&error);
            let rounding =
                signal.rounding + fitted.rounding + truth.rounding + subtraction + conversion;
            let largest = controls.iter().fold(0.0_f64, |m, c| m.max(c.abs()));
            let level_rounding = ExactRounding::gamma(controls.len()) * largest;
            if let Some(t) = first_unproven_patch(
                &controls,
                (from, to),
                budget.of_order(order),
                rounding,
                level_rounding,
                0,
            ) {
                return Some(u_of(t).clamp(-1.0, 1.0));
            }
        }
    }
    None
}

#[cfg(test)]
#[path = "shaper_hull_tests.rs"]
mod tests;
