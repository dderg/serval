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

use super::TRANSLATION_BINOMIALS;
use super::exact::{EXACT_CAPACITY, ExactPiece, ExactRounding};
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

/// The share of each order's budget an enclosure's bound may take before
/// its window is halved: the hull spends the bound as slack, so a rung
/// within the rest of the budget is still proven, and one in the slack is
/// split, never accepted. A sixteenth leaves rungs 94 % of the budget; a
/// sixty-fourth halved the followers' windows twice more, for 1.7 % more
/// instructions on the P3 kernel profile's dense print.
pub(crate) const ENCLOSURE_BUDGET_SHARE: f64 = 1.0 / 16.0;

/// The derivative order of jerk, which the hull proves only against exact
/// polynomials and only where the budget holds it ([`FitTol::jerk_mm_s3`]).
const JERK: usize = 3;

impl HullBudget {
    fn of_order(self, order: usize) -> f64 {
        match order {
            0 => self.tol.pos_mm,
            1 => self.velocity,
            2 => self.tol.accel_mm_s2,
            JERK => self.tol.jerk_mm_s3,
            _ => panic!("the hull holds no derivative of order {order}"),
        }
    }

    pub(crate) fn enclosure_accuracy(self) -> [f64; 3] {
        [0, 1, 2].map(|order| self.of_order(order) * ENCLOSURE_BUDGET_SHARE)
    }
}

/// `coefficients` re-expanded about `shift`, by repeated Horner passes of
/// separate products and sums: two roundings a step, which the depths
/// above count, where a fused step would cost a library call on targets
/// without a fused multiply-add.
fn unfused_taylor_shift(coefficients: &[f64], shift: f64, shifted: &mut [f64]) {
    shifted.copy_from_slice(coefficients);
    horner_shift_in_place(shifted, |lower, higher| lower + shift * higher);
}

/// The repeated Horner passes of [`unfused_taylor_shift`] in place: pass
/// `lowest` settles `shifted[lowest]` and only touches the slots above it
/// after. Each pass carries the slot it just wrote into the next step.
#[inline]
fn horner_shift_in_place(shifted: &mut [f64], step: impl Fn(f64, f64) -> f64) {
    let len = shifted.len();
    assert!(
        len <= EXACT_CAPACITY,
        "a {len}-coefficient polynomial exceeds the hull"
    );
    let Some(&top) = shifted.last() else {
        return;
    };
    for lowest in 0..len {
        let mut higher = top;
        for slot in shifted[lowest..len - 1].iter_mut().rev() {
            higher = step(*slot, higher);
            *slot = higher;
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
    unfused_taylor_shift(coefficients, mid, &mut shifted[..len]);
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
fn piece_derivative(mono_u: &[f64], order: usize, h: f64) -> Monomials {
    assert!(
        mono_u.len() <= EXACT_CAPACITY,
        "a {}-coefficient piece exceeds the hull",
        mono_u.len()
    );
    let rate = 2.0 / h;
    let scale = (0..order).fold(1.0, |acc, _| acc * rate);
    let mut derivative = Monomials {
        values: [0.0; EXACT_CAPACITY],
        len: mono_u.len().saturating_sub(order),
    };
    for (slot, power) in derivative.values.iter_mut().zip(order..mono_u.len()) {
        let falling = ((power - order + 1)..=power).fold(1.0, |acc, k| acc * k as f64);
        *slot = mono_u[power] * falling * scale;
    }
    derivative
}

/// Monomial coefficients of a polynomial in `u`.
struct Monomials {
    values: [f64; EXACT_CAPACITY],
    len: usize,
}

impl std::ops::Deref for Monomials {
    type Target = [f64];

    fn deref(&self) -> &[f64] {
        &self.values[..self.len]
    }
}

/// The Bernstein coefficients on `y ∈ [−1, 1]` of a polynomial in `y`,
/// and the rounding bound of the conversion. Its Taylor shifts by `∓1`
/// take their products as the exact `−x` and `x`, so they round as
/// [`unfused_taylor_shift`] by `∓1` does.
fn bernstein(coefficients: &[f64]) -> (Controls, f64) {
    let len = coefficients.len();
    let degree = len - 1;
    let mut on_unit = [0.0; EXACT_CAPACITY];
    let mut magnitudes = [0.0; EXACT_CAPACITY];
    let (on_unit, magnitudes) = (&mut on_unit[..len], &mut magnitudes[..len]);
    on_unit.copy_from_slice(coefficients);
    horner_shift_in_place(on_unit, |lower, higher| lower - higher);
    for (magnitude, coefficient) in magnitudes.iter_mut().zip(coefficients) {
        *magnitude = coefficient.abs();
    }
    horner_shift_in_place(magnitudes, |lower, higher| lower + higher);
    let mut doubling = 1.0;
    for power in 0..len {
        on_unit[power] *= doubling;
        magnitudes[power] *= doubling;
        doubling *= 2.0;
    }
    let weights = &bernstein_weights()[degree];
    let mut controls = Controls {
        values: [0.0; EXACT_CAPACITY],
        len,
    };
    let mut worst_terms = 0.0_f64;
    for (index, (control, row)) in controls.values[..len].iter_mut().zip(weights).enumerate() {
        let terms_of = index + 1;
        let mut terms = 0.0;
        for ((&weight, &value), &magnitude) in row[..terms_of]
            .iter()
            .zip(&on_unit[..terms_of])
            .zip(&magnitudes[..terms_of])
        {
            *control += weight * value;
            terms += weight * magnitude;
        }
        worst_terms = worst_terms.max(terms);
    }
    (
        controls,
        ExactRounding::gamma(CONVERSION_DEPTH) * worst_terms,
    )
}

/// Bernstein controls on `y ∈ [−1, 1]`.
struct Controls {
    values: [f64; EXACT_CAPACITY],
    len: usize,
}

impl std::ops::Deref for Controls {
    type Target = [f64];

    fn deref(&self) -> &[f64] {
        &self.values[..self.len]
    }
}

/// The conversion's weights `C(i, p)/C(n, p)` for each degree `n`, row
/// `i ≤ n` and power `p ≤ i`, each one rounded division.
fn bernstein_weights() -> &'static [[[f64; EXACT_CAPACITY]; EXACT_CAPACITY]] {
    static WEIGHTS: std::sync::OnceLock<Vec<[[f64; EXACT_CAPACITY]; EXACT_CAPACITY]>> =
        std::sync::OnceLock::new();
    WEIGHTS.get_or_init(|| {
        (0..EXACT_CAPACITY)
            .map(|degree| {
                let mut weights = [[0.0; EXACT_CAPACITY]; EXACT_CAPACITY];
                for (index, row) in weights.iter_mut().enumerate().take(degree + 1) {
                    for (power, weight) in row.iter_mut().enumerate().take(index + 1) {
                        *weight = TRANSLATION_BINOMIALS[index][power]
                            / TRANSLATION_BINOMIALS[degree][power];
                    }
                }
                weights
            })
            .collect()
    })
}

/// Halves Bernstein controls at their midpoint.
fn de_casteljau_halves(controls: &[f64]) -> (Controls, Controls) {
    let len = controls.len();
    let mut work = [0.0; EXACT_CAPACITY];
    work[..len].copy_from_slice(controls);
    let mut left = Controls {
        values: [0.0; EXACT_CAPACITY],
        len,
    };
    let mut right = Controls {
        values: [0.0; EXACT_CAPACITY],
        len,
    };
    left.values[0] = work[0];
    right.values[len - 1] = work[len - 1];
    for remaining in (1..len).rev() {
        for index in 0..remaining {
            work[index] = 0.5 * (work[index] + work[index + 1]);
        }
        left.values[len - remaining] = work[0];
        right.values[remaining - 1] = work[remaining - 1];
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

/// One order's error `P − S` on one sub-interval of the domain, in the
/// Bernstein basis over `[from, to]`, with its rounding term and the
/// rounding each de Casteljau level adds.
struct ErrorPatch {
    controls: Controls,
    from: f64,
    to: f64,
    rounding: f64,
    level_rounding: f64,
}

/// One order's error `P − S` on one sub-interval of the domain as
/// monomial coefficients in the sub-interval's `y ∈ [−1, 1]`, with the
/// rounding of everything up to them.
struct ErrorPolynomial {
    coefficients: [f64; EXACT_CAPACITY],
    len: usize,
    from: f64,
    to: f64,
    rounding: f64,
}

/// What [`ErrorPolynomial::proven_by_magnitude`] inflates its bound by,
/// against the roundings of the sums it and the hull compute: each is a
/// few dozen operations, under `10⁻¹³` of itself.
const MAGNITUDE_BOUND_INFLATION: f64 = 1e-9;

impl ErrorPolynomial {
    /// Whether the hull would prove the patch at its first look, settled
    /// without the Bernstein conversion. Every Bernstein coefficient of
    /// `yᵏ` on `[−1, 1]` is an average of products of `k` ends `±1`, so
    /// each control of the error lies within `Σ|eₖ|` of zero. The
    /// conversion computes them within its rounding bound
    /// `γ·max ᵢ termsᵢ`, and the terms' Bernstein form of `Σ|eₖ|(2v+1)ᵏ`,
    /// whose monomial coefficients are non-negative, peaks at its last
    /// control `Σ|eₖ|3ᵏ`. So the hull's first test, its largest control
    /// plus the rounding and that conversion bound within the budget,
    /// passes whenever `Σ|eₖ| + 2γΣ|eₖ|3ᵏ` plus the rounding does: a patch
    /// passing this would have been proven, and skipping it changes no
    /// verdict.
    fn proven_by_magnitude(&self, budget: f64) -> bool {
        let (mut monomial, mut tripled, mut power) = (0.0, 0.0, 1.0);
        for coefficient in &self.coefficients[..self.len] {
            monomial += coefficient.abs();
            tripled += coefficient.abs() * power;
            power *= 3.0;
        }
        let conversion = ExactRounding::gamma(CONVERSION_DEPTH) * tripled;
        (monomial + 2.0 * conversion + self.rounding) * (1.0 + MAGNITUDE_BOUND_INFLATION) <= budget
    }

    fn patch(&self) -> ErrorPatch {
        let (controls, conversion) = bernstein(&self.coefficients[..self.len]);
        let largest = controls.iter().fold(0.0_f64, |m, c| m.max(c.abs()));
        ErrorPatch {
            level_rounding: ExactRounding::gamma(controls.len()) * largest,
            controls,
            from: self.from,
            to: self.to,
            rounding: self.rounding + conversion,
        }
    }
}

/// What a fit's error from `sub` in derivative `order` may reach, `None`
/// where it is not held: inside an unshaped leader's square-root onset the
/// acceleration is exempt and the velocity holds the step's allowance.
fn piece_budget(sub: &ExactPiece, order: usize, budget: HullBudget) -> Option<f64> {
    match (order, sub.unshaped_step) {
        (1, Some(step)) => Some(budget.velocity.min(step.allowance())),
        (1, None) => budget.velocity.is_finite().then_some(budget.velocity),
        (2 | JERK, Some(_)) => None,
        _ => Some(budget.of_order(order)),
    }
}

/// Every sub-interval's error polynomial for `order` beside its budget,
/// the fitted piece's derivatives already taken; `held` the budget of each
/// sub-interval, `None` for one whose error is not held.
fn error_polynomials<'a>(
    exact: &'a [ExactPiece],
    piece: FittedPiece<'a>,
    derivative: &'a [f64],
    domain: (f64, f64),
    order: usize,
    held: impl Fn(&ExactPiece) -> Option<f64> + 'a,
) -> impl Iterator<Item = (ErrorPolynomial, f64)> + 'a {
    let h = piece.t1 - piece.t0;
    let u_of = move |t: f64| 2.0 * (t - piece.t0) / h - 1.0;
    exact.iter().filter_map(move |sub| {
        let (from, to) = (sub.lo.max(domain.0), sub.hi.min(domain.1));
        let budget = held(sub)?;
        if from > to {
            return None;
        }
        let middle = 0.5 * (from + to);
        let half = 0.5 * (to - from);
        let fitted = composed(derivative, u_of(middle), half * (2.0 / h));
        let jerk;
        let signal = if order == JERK {
            jerk = sub.jerk();
            &jerk
        } else {
            &sub.pva[order]
        };
        let truth = composed(
            &signal.coefficients[..signal.len],
            (middle - sub.center) / sub.radius,
            half / sub.radius,
        );
        let len = fitted.len.max(truth.len).max(1);
        let mut coefficients = [0.0; EXACT_CAPACITY];
        let mut subtraction = 0.0;
        for (power, slot) in coefficients[..len].iter_mut().enumerate() {
            let (p, s) = (fitted.coefficients[power], truth.coefficients[power]);
            *slot = p - s;
            subtraction += f64::EPSILON * 0.5 * (p.abs() + s.abs());
        }
        Some((
            ErrorPolynomial {
                coefficients,
                len,
                from,
                to,
                rounding: signal.rounding + fitted.rounding + truth.rounding + subtraction,
            },
            budget,
        ))
    })
}

#[cfg(feature = "fit-audit")]
fn derivatives_of(piece: FittedPiece) -> [Monomials; 3] {
    let h = piece.t1 - piece.t0;
    [0, 1, 2].map(|order| piece_derivative(piece.mono_u, order, h))
}

/// Where a piece is not proven: the `u` of the point, and whether it is a
/// jerk miss. A piece follows the jerk of one polynomial at a time: across
/// a break it may miss it however short, and halving the span about the
/// break only narrows its pieces toward the floor, where splitting at the
/// break leaves each side one ([`super::exact::ExactSpan::break_nearest`]).
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Unproven {
    pub u: f64,
    pub misses_jerk: bool,
}

/// Where `piece` is not proven within `budget` on `[domain]` of the
/// signal whose exact polynomials over that domain are `exact`, or `None`
/// when every order holds its budget everywhere there.
pub(crate) fn first_unproven(
    exact: &[ExactPiece],
    piece: FittedPiece,
    domain: (f64, f64),
    budget: HullBudget,
) -> Option<Unproven> {
    let h = piece.t1 - piece.t0;
    let orders = if budget.tol.jerk_mm_s3.is_finite() {
        0..=JERK
    } else {
        0..=2
    };
    for order in orders {
        let derivative = piece_derivative(piece.mono_u, order, h);
        let held = |sub: &ExactPiece| piece_budget(sub, order, budget);
        for (error, limit) in error_polynomials(exact, piece, &derivative, domain, order, held) {
            if error.proven_by_magnitude(limit) {
                continue;
            }
            let patch = error.patch();
            if let Some(t) = first_unproven_patch(
                &patch.controls,
                (patch.from, patch.to),
                limit,
                patch.rounding,
                patch.level_rounding,
                0,
            ) {
                return Some(Unproven {
                    u: (2.0 * (t - piece.t0) / h - 1.0).clamp(-1.0, 1.0),
                    misses_jerk: order == JERK,
                });
            }
        }
    }
    None
}

/// A proven upper bound on a patch's error, refined only as far as it
/// takes to settle it against `budget`: a bound within it is final, one
/// whose end controls already exceed it by the rounding is a proven miss,
/// and anything else is halved down to [`HULL_MAX_DEPTH`] — the same
/// settling the hull accepts a rung by, so a piece it proved is bounded
/// within its budget here.
#[cfg(feature = "fit-audit")]
fn patch_bound(controls: &[f64], patch: &ErrorPatch, budget: f64, depth: u32) -> f64 {
    let slack = patch.rounding + f64::from(depth) * patch.level_rounding;
    let peak = controls.iter().fold(0.0_f64, |m, c| m.max(c.abs()));
    let ends = controls[0].abs().max(controls[controls.len() - 1].abs());
    let bound = peak + slack;
    if !peak.is_finite() {
        return f64::INFINITY;
    }
    if bound <= budget || ends - slack > budget || depth == HULL_MAX_DEPTH || controls.len() == 1 {
        return bound;
    }
    let (left, right) = de_casteljau_halves(controls);
    patch_bound(&left, patch, budget, depth + 1)
        .max(patch_bound(&right, patch, budget, depth + 1))
        .min(bound)
}

/// Proven upper bounds on how far `piece`'s position, velocity and
/// acceleration stray from the signal whose exact polynomials or
/// enclosure over `domain` are `exact`, everywhere on it, each settled
/// against its `budgets`.
#[cfg(feature = "fit-audit")]
pub(crate) fn error_bounds(
    exact: &[ExactPiece],
    piece: FittedPiece,
    domain: (f64, f64),
    budgets: [f64; 3],
) -> [f64; 3] {
    let derivatives = derivatives_of(piece);
    [0, 1, 2].map(|order| {
        let budget = budgets[order];
        let held = |sub: &ExactPiece| sub.bounds(order).then_some(budget);
        error_polynomials(exact, piece, &derivatives[order], domain, order, held)
            .map(|(error, budget)| {
                let patch = error.patch();
                patch_bound(&patch.controls, &patch, budget, 0)
            })
            .fold(0.0_f64, f64::max)
    })
}

#[cfg(test)]
#[path = "shaper_hull_tests.rs"]
mod tests;
