//! The shaped signal's exact polynomial between its breaks.
//!
//! # The construction
//!
//! The shaped signal is `S = x ∗ k` (plus the derivative-gain impulses, which
//! are point masses of `x`). `x` is piecewise polynomial between the table's
//! boundaries `b` (piece seams and the clamp edges `first_t`, `last_t`) and
//! carries point masses there (the table's position and slope jumps, and the
//! gain impulses); `k` is piecewise polynomial between its cuts `c`. Between
//! two consecutive real breaks `b + c` every kernel piece's window
//! `[t − bⱼ, t − aⱼ]` keeps the same pieces and masses, so `S` is one
//! polynomial there. On such a sub-interval pick a float `t_c` strictly
//! inside it and write `t = t_c + w`. For kernel piece `j` let the frozen
//! edges be the floats `f_L = fl(t_c − bⱼ)`, `f_U = fl(t_c − aⱼ)`, and their
//! exact rounding errors `ε_L`, `ε_U` (TwoSum), so the real window edges are
//! `f + ε + w`. The piece's contribution splits exactly into
//!
//! ```text
//! ∫_{f_L}^{f_U} kⱼ(t − s) dμ(s)  +  ∫_{f_U}^{f_U+z} kⱼ(t − s) x(s) ds  −  ∫_{f_L}^{f_L+z′} kⱼ(t − s) x(s) ds
//! ```
//!
//! with `z = w + ε_U`, `z′ = w + ε_L`. In the frozen window,
//! `kⱼ(t − s) = Σ_l c_l (z − (s − f_U))^l` with `c_l` the piece's own
//! coefficients about `aⱼ`, so it is `Σ_r z^r Σ_l c_l C(l,r) (−1)^{l−r} M_{l−r}`
//! with `M_q = ∫ (s − f_U)^q dμ` read from the moment tree, plus the point
//! masses strictly inside the real window. Each sliver lies in one piece of
//! `x` (no break crosses it), whose Taylor series `Y` about the frozen edge
//! meets the kernel piece's series `K` about the matching cut in the Beta
//! integral `∫₀^z K(z − v) Y(v) dv = Σ K_q Y_m q! m!/(q+m+1)! z^{q+m+1}`.
//! Velocity and acceleration are built the same way from the moments of `x′`
//! and `x″` (with the masses' derivatives), exactly as `eval_pva` builds
//! them, so each order is conditioned like the value it bounds instead of
//! being differentiated out of the position.
//!
//! # The rounding term
//!
//! Every coefficient is a sum of products of data — table and kernel
//! coefficients, jumps, and small float differences of times (each a datum
//! carrying one relative rounding). Along any path from the data to a
//! coefficient at most `N` roundings occur, so its error is at most
//! `γ_N · Σ|terms|` with `γ_N = N·u/(1 − N·u)`, `u = 2⁻⁵³` (Higham, §3.1).
//! The `|terms|` sum is bounded by carrying, beside every value, the same
//! expression evaluated on magnitudes: a Taylor shift's magnitude is the
//! shift of the magnitudes by `|shift|`; a moment's is bounded from the
//! moment tree's per-node mass `∫|x⁽ⁿ⁾|` and reach `ρ` as `mass·(|δ| + ρ)^q`,
//! `δ` the node origin's offset (`MomentTree` derives both while it builds).
//! On `|x| ≤ 1` a coefficient error `eₖ` moves the polynomial by at most
//! `Σ|eₖ|`, so each order's rounding term is `2·γ_N·Σₖ magnitudeₖ`, the factor
//! two covering the magnitudes' own rounding. `N` is
//! [`ExactRounding::depth`].
//!
//! # What it covers
//!
//! Real sub-intervals holding a float time; one narrower than an ulp holds
//! no time any evaluation can name, so it is left to its neighbours, whose
//! polynomials are bounded to one ulp past their own breaks.
#![cfg_attr(
    not(test),
    expect(
        dead_code,
        reason = "TODO: the shaped ladder's hull acceptance reads these next"
    )
)]

use std::cmp::Ordering;

use nurbs::algebra::PiecewisePolynomialKernel;
use trajectory::next_toward;

use super::{
    AxisSignalTable, GainImpulse, MOMENT_ORDERS, MOMENT_POWER_CAPACITY, MomentTree,
    TRANSLATION_BINOMIALS, taylor_shift,
};

pub(crate) const EXACT_CAPACITY: usize = MOMENT_POWER_CAPACITY;

fn ordered(left: f64, right: f64) -> Ordering {
    left.partial_cmp(&right)
        .expect("exact polynomial breaks are ordered times, never NaN")
}

/// A real number held exactly as the unevaluated sum of a float and its
/// rounding error.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct RealSum {
    head: f64,
    tail: f64,
}

impl RealSum {
    /// `a + b`, exactly (Knuth's TwoSum).
    pub(crate) fn of(a: f64, b: f64) -> Self {
        let head = a + b;
        let b_part = head - a;
        let a_part = head - b_part;
        Self {
            head,
            tail: (a - a_part) + (b - b_part),
        }
    }

    fn float(value: f64) -> Self {
        Self {
            head: value,
            tail: 0.0,
        }
    }

    /// How `value` orders against this real: the tail is within half an
    /// ulp of the head, so the head decides unless they are equal.
    fn order_of(self, value: f64) -> Ordering {
        ordered(value, self.head).then_with(|| ordered(0.0, self.tail))
    }

    fn cmp(self, other: Self) -> Ordering {
        ordered(self.head, other.head).then_with(|| ordered(self.tail, other.tail))
    }

    fn first_float_above(self) -> f64 {
        if self.tail < 0.0 {
            self.head
        } else {
            next_toward(self.head, f64::INFINITY)
        }
    }

    fn last_float_below(self) -> f64 {
        if self.tail > 0.0 {
            self.head
        } else {
            next_toward(self.head, f64::NEG_INFINITY)
        }
    }
}

/// Polynomial coefficients beside a bound on the magnitude of the terms
/// each was summed from.
#[derive(Clone, Copy, Debug)]
struct Tracked {
    values: [f64; EXACT_CAPACITY],
    magnitudes: [f64; EXACT_CAPACITY],
    len: usize,
}

impl Tracked {
    const ZERO: Self = Self {
        values: [0.0; EXACT_CAPACITY],
        magnitudes: [0.0; EXACT_CAPACITY],
        len: 0,
    };

    fn exact(coefficients: &[f64]) -> Self {
        assert!(coefficients.len() <= EXACT_CAPACITY);
        let mut tracked = Self::ZERO;
        tracked.len = coefficients.len();
        tracked.values[..tracked.len].copy_from_slice(coefficients);
        for (magnitude, value) in tracked.magnitudes.iter_mut().zip(coefficients) {
            *magnitude = value.abs();
        }
        tracked
    }

    fn shifted(&self, shift: f64) -> Self {
        let mut shifted = Self::ZERO;
        shifted.len = self.len;
        taylor_shift(
            &self.values[..self.len],
            shift,
            &mut shifted.values[..self.len],
        );
        taylor_shift(
            &self.magnitudes[..self.len],
            shift.abs(),
            &mut shifted.magnitudes[..self.len],
        );
        shifted
    }

    fn derivative(&self, order: usize) -> Self {
        let mut derivative = Self::ZERO;
        derivative.len = self.len.saturating_sub(order);
        for power in 0..derivative.len {
            let falling = ((power + 1)..=(power + order)).fold(1.0, |acc, k| acc * k as f64);
            derivative.values[power] = self.values[power + order] * falling;
            derivative.magnitudes[power] = self.magnitudes[power + order] * falling;
        }
        derivative
    }

    fn add(&mut self, other: &Self, sign: f64) {
        self.len = self.len.max(other.len);
        for power in 0..other.len {
            self.values[power] += sign * other.values[power];
            self.magnitudes[power] += other.magnitudes[power];
        }
    }

    /// `∫₀^z self(z − v) · other(v) dv` as a polynomial in `z`.
    fn beta_convolution(&self, other: &Self) -> Self {
        let mut product = Self::ZERO;
        if self.len == 0 || other.len == 0 {
            return product;
        }
        product.len = self.len + other.len;
        assert!(
            product.len <= EXACT_CAPACITY,
            "a sliver of degree {} exceeds the exact polynomial capacity",
            product.len - 1
        );
        for kernel_power in 0..self.len {
            for input_power in 0..other.len {
                let beta = BETA[kernel_power][input_power];
                let power = kernel_power + input_power + 1;
                product.values[power] +=
                    beta * self.values[kernel_power] * other.values[input_power];
                product.magnitudes[power] +=
                    beta * self.magnitudes[kernel_power] * other.magnitudes[input_power];
            }
        }
        product
    }
}

/// `BETA[q][m] = q! m! / (q + m + 1)!`, the Beta integral of the sliver.
const BETA: [[f64; EXACT_CAPACITY]; EXACT_CAPACITY] = beta_table();

const fn beta_table() -> [[f64; EXACT_CAPACITY]; EXACT_CAPACITY] {
    let mut table = [[0.0; EXACT_CAPACITY]; EXACT_CAPACITY];
    let mut input_power = 0;
    while input_power < EXACT_CAPACITY {
        table[0][input_power] = 1.0 / (input_power + 1) as f64;
        let mut kernel_power = 1;
        while kernel_power < EXACT_CAPACITY {
            table[kernel_power][input_power] = table[kernel_power - 1][input_power]
                * kernel_power as f64
                / (kernel_power + input_power + 1) as f64;
            kernel_power += 1;
        }
        input_power += 1;
    }
    table
}

struct ExactKernelPiece {
    start: f64,
    end: f64,
    at_start: Tracked,
    at_end: Tracked,
}

/// A kernel prepared for exact polynomials: each piece's series about both
/// of its cuts, and every cut a break can be the image of.
pub(crate) struct ExactKernel {
    pieces: Vec<ExactKernelPiece>,
    cuts: Vec<f64>,
}

impl ExactKernel {
    pub(crate) fn new(kernel: &PiecewisePolynomialKernel) -> Self {
        assert!(
            kernel
                .pieces
                .windows(2)
                .all(|pair| pair[0].u_end == pair[1].u_start),
            "shaper kernel pieces must tile their support"
        );
        let pieces = kernel
            .pieces
            .iter()
            .map(|piece| {
                let at_start = Tracked::exact(&piece.coeffs);
                ExactKernelPiece {
                    start: piece.u_start,
                    end: piece.u_end,
                    at_start,
                    at_end: at_start.shifted(piece.u_end - piece.u_start),
                }
            })
            .collect();
        Self {
            pieces,
            cuts: trajectory::ShapedSignal::kernel_cut_boundaries(kernel).collect(),
        }
    }
}

/// Bounds the roundings any coefficient of an exact polynomial passes
/// through, so the a-priori error term is `γ_N` of its magnitudes.
pub(crate) struct ExactRounding;

impl ExactRounding {
    /// A Taylor shift of up to `EXACT_CAPACITY` coefficients by repeated
    /// synthetic division: each output picks up at most one rounding per
    /// pass and per step down, `2·EXACT_CAPACITY` together.
    const SHIFT: usize = 2 * EXACT_CAPACITY;
    /// A moment translation: the shift's datum and powers (`EXACT_CAPACITY`),
    /// a binomial product (2), and the sum over source powers.
    const TRANSLATION: usize = 2 * EXACT_CAPACITY + 3;
    /// A leaf moment: centring shift, derivative, the edge powers' datum
    /// and powers, the coefficient product and divisor, the sum over the
    /// piece's powers, and its translation.
    const LEAF: usize =
        Self::SHIFT + 1 + (EXACT_CAPACITY + 2) + 2 + EXACT_CAPACITY + Self::TRANSLATION;
    /// The frozen window's combination: the binomial product with a kernel
    /// coefficient (3), its sum, and the shift by the edge's rounding.
    const FROZEN: usize = 3 + EXACT_CAPACITY + Self::SHIFT;
    /// A sliver: the edge series' datum and shift, its derivative, the Beta
    /// product (its own `EXACT_CAPACITY` roundings plus 2), its sum, and the
    /// shift by the edge's rounding. The kernel's own series about its end
    /// adds one more shift.
    const SLIVER: usize =
        1 + 2 * Self::SHIFT + 1 + (EXACT_CAPACITY + 2) + EXACT_CAPACITY + Self::SHIFT;
    /// Scaling into the local variable, and the position offset.
    const FINISH: usize = EXACT_CAPACITY + 2;

    /// `N` for a polynomial read off a moment tree of height `tree_height`,
    /// summing `kernel_pieces` pieces and `point_masses` masses.
    pub(crate) fn depth(tree_height: usize, kernel_pieces: usize, point_masses: usize) -> usize {
        let moment = Self::LEAF
            + tree_height * (Self::TRANSLATION + 1)
            + Self::TRANSLATION
            + 2 * tree_height
            + 4
            + point_masses
            + EXACT_CAPACITY
            + 2;
        (moment + Self::FROZEN).max(Self::SLIVER) + 3 * kernel_pieces + 1 + Self::FINISH
    }

    pub(crate) fn gamma(depth: usize) -> f64 {
        let unit = f64::EPSILON / 2.0;
        let accumulated = depth as f64 * unit;
        assert!(
            accumulated < 0.5,
            "rounding depth {depth} defeats the a-priori bound"
        );
        accumulated / (1.0 - accumulated)
    }
}

/// One derivative order of the signal on a sub-interval, in the local
/// variable `x = (t − center) / radius`, with a bound on how far the
/// computed coefficients' polynomial can be from the exact one on
/// `|x| ≤ 1`.
#[derive(Clone, Copy, Debug)]
pub(crate) struct LocalPolynomial {
    pub coefficients: [f64; EXACT_CAPACITY],
    pub len: usize,
    pub rounding: f64,
}

impl LocalPolynomial {
    #[cfg(test)]
    pub(crate) fn eval(&self, x: f64) -> f64 {
        self.coefficients[..self.len]
            .iter()
            .rev()
            .fold(0.0, |acc, &c| nurbs::fmadd(acc, x, c))
    }

    /// The same polynomial less a constant.
    pub(crate) fn offset_by(mut self, base: f64) -> Self {
        assert!(self.len > 0, "an empty position polynomial has no offset");
        let before = self.coefficients[0];
        self.coefficients[0] -= base;
        self.rounding += f64::EPSILON * (before.abs() + base.abs());
        self
    }
}

/// The signal's exact polynomials on one real sub-interval between breaks.
/// `[lo, hi]` holds the sub-interval and reaches at most an ulp past it,
/// and lies within `radius` of `center`.
#[derive(Clone, Copy, Debug)]
pub(crate) struct ExactPiece {
    pub lo: f64,
    pub hi: f64,
    pub center: f64,
    pub radius: f64,
    pub pva: [LocalPolynomial; 3],
}

impl ExactPiece {
    pub(crate) fn local(&self, t: f64) -> f64 {
        (t - self.center) / self.radius
    }

    #[cfg(test)]
    pub(crate) fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        let x = self.local(t);
        (
            self.pva[0].eval(x),
            self.pva[1].eval(x),
            self.pva[2].eval(x),
        )
    }

    pub(crate) fn offset_by(mut self, base: f64) -> Self {
        self.pva[0] = self.pva[0].offset_by(base);
        self
    }
}

/// The exact convolution behind a shaped signal: the table it integrates
/// (with its moments and jumps), the kernel, the gain impulses, and the
/// constant the signal adds to every position.
#[derive(Clone, Copy)]
pub(crate) struct ExactConvolution<'a> {
    pub table: &'a AxisSignalTable,
    pub kernel: &'a ExactKernel,
    pub impulses: &'a [GainImpulse],
    pub position_offset: f64,
}

/// One point mass of `x` and its derivatives: `weights[m]` multiplies
/// `δ⁽ᵐ⁾(s − at)` in derivative order `order`.
struct PointMass {
    at: f64,
    weights: [[f64; 4]; MOMENT_ORDERS],
}

impl ExactConvolution<'_> {
    /// The signal's exact polynomial on every real sub-interval of
    /// `[lo, hi]` between its breaks that holds a float time, or `None`
    /// where the convolution reads history or lookahead the table does
    /// not hold.
    pub(crate) fn pieces_over(&self, lo: f64, hi: f64) -> Option<Vec<ExactPiece>> {
        assert!(
            lo <= hi,
            "exact polynomial over the empty interval [{lo}, {hi}]"
        );
        let mut breaks = Vec::new();
        let boundaries = self.table.boundaries();
        let (lo_real, hi_real) = (RealSum::float(lo), RealSum::float(hi));
        for &cut in &self.kernel.cuts {
            let first = boundaries.partition_point(|&boundary| {
                RealSum::of(boundary, cut).order_of(lo) != Ordering::Less
            });
            let past = boundaries.partition_point(|&boundary| {
                RealSum::of(boundary, cut).order_of(hi) == Ordering::Greater
            });
            breaks.extend(
                boundaries[first..past.max(first)]
                    .iter()
                    .map(|&boundary| RealSum::of(boundary, cut)),
            );
        }
        breaks.sort_by(|left, right| left.cmp(*right));
        breaks.dedup();
        let points: Vec<RealSum> = std::iter::once(lo_real)
            .chain(breaks)
            .chain(std::iter::once(hi_real))
            .collect();
        let mut pieces = Vec::with_capacity(points.len() - 1);
        for pair in points.windows(2) {
            let first_inside = pair[0].first_float_above();
            let last_inside = pair[1].last_float_below();
            if first_inside > last_inside {
                continue;
            }
            let center = (first_inside + 0.5 * (last_inside - first_inside))
                .clamp(first_inside, last_inside);
            let piece_lo = next_toward(first_inside, f64::NEG_INFINITY).max(lo);
            let piece_hi = next_toward(last_inside, f64::INFINITY).min(hi);
            let radius = next_toward((center - piece_lo).max(piece_hi - center), f64::INFINITY);
            pieces.push(ExactPiece {
                lo: piece_lo,
                hi: piece_hi,
                center,
                radius,
                pva: self.polynomials_at(center, radius)?,
            });
        }
        Some(pieces)
    }

    fn point_masses(&self) -> impl Iterator<Item = PointMass> + '_ {
        let jumps = self
            .table
            .input_jumps()
            .iter()
            .map(|&(at, position_jump, slope_jump)| PointMass {
                at,
                weights: [
                    [0.0; 4],
                    [position_jump, 0.0, 0.0, 0.0],
                    [slope_jump, position_jump, 0.0, 0.0],
                ],
            });
        let impulses = self.impulses.iter().map(|impulse| PointMass {
            at: impulse.at,
            weights: [
                [impulse.delta, impulse.doublet, 0.0, 0.0],
                [0.0, impulse.delta, impulse.doublet, 0.0],
                [0.0, 0.0, impulse.delta, impulse.doublet],
            ],
        });
        jumps.chain(impulses)
    }

    fn polynomials_at(&self, center: f64, radius: f64) -> Option<[LocalPolynomial; 3]> {
        let mut total = [Tracked::ZERO; MOMENT_ORDERS];
        let mut point_masses = 0;
        for piece in &self.kernel.pieces {
            let lower = RealSum::of(center, -piece.end);
            let upper = RealSum::of(center, -piece.start);
            let degree = piece.at_start.len - 1;
            let mut moments = [[0.0; EXACT_CAPACITY]; MOMENT_ORDERS];
            {
                let [position, velocity, acceleration] = &mut moments;
                if !self.table.integrate_moments(
                    lower.head,
                    upper.head,
                    degree,
                    upper.head,
                    [
                        &mut position[..=degree],
                        &mut velocity[..=degree],
                        &mut acceleration[..=degree],
                    ],
                ) {
                    return None;
                }
            }
            let mut magnitudes = self
                .table
                .moment_magnitudes(lower.head, upper.head, degree, upper.head);
            for mass in self.point_masses() {
                if lower.order_of(mass.at) != Ordering::Greater
                    || upper.order_of(mass.at) != Ordering::Less
                {
                    continue;
                }
                point_masses += 1;
                add_point_mass(
                    &mut moments,
                    &mut magnitudes,
                    &mass,
                    mass.at - upper.head,
                    degree,
                );
            }
            let lower_series = self.table.series_past(lower)?;
            let upper_series = self.table.series_past(upper)?;
            for order in 0..MOMENT_ORDERS {
                let mut at_upper =
                    frozen_window(&piece.at_start, &moments[order], &magnitudes[order]);
                at_upper.add(&piece.at_start.beta_convolution(&upper_series[order]), 1.0);
                total[order].add(&at_upper.shifted(upper.tail), 1.0);
                let at_lower = piece.at_end.beta_convolution(&lower_series[order]);
                total[order].add(&at_lower.shifted(lower.tail), -1.0);
            }
        }
        let gamma = ExactRounding::gamma(ExactRounding::depth(
            self.table.moment_tree_height(),
            self.kernel.pieces.len(),
            point_masses,
        ));
        Some(std::array::from_fn(|order| {
            let mut tracked = total[order];
            if order == 0 {
                tracked.len = tracked.len.max(1);
                tracked.values[0] += self.position_offset;
                tracked.magnitudes[0] += self.position_offset.abs();
            }
            let mut scale = 1.0;
            let mut magnitude = 0.0;
            let mut coefficients = [0.0; EXACT_CAPACITY];
            for power in 0..tracked.len {
                coefficients[power] = tracked.values[power] * scale;
                magnitude += tracked.magnitudes[power] * scale;
                scale *= radius;
            }
            assert!(
                coefficients[..tracked.len].iter().all(|c| c.is_finite()),
                "exact polynomial of order {order} at {center} is not finite"
            );
            LocalPolynomial {
                coefficients,
                len: tracked.len,
                rounding: 2.0 * gamma * magnitude,
            }
        }))
    }
}

/// `Σ_r z^r Σ_l c_l C(l, r) (−1)^{l−r} M_{l−r}`: the kernel piece over the
/// frozen window, as a polynomial in the window's motion `z`.
fn frozen_window(kernel: &Tracked, moments: &[f64], magnitudes: &[f64]) -> Tracked {
    let mut frozen = Tracked::ZERO;
    frozen.len = kernel.len;
    for power in 0..kernel.len {
        for kernel_power in power..kernel.len {
            let moment_power = kernel_power - power;
            let binomial = TRANSLATION_BINOMIALS[kernel_power][power];
            let sign = if moment_power % 2 == 0 { 1.0 } else { -1.0 };
            frozen.values[power] +=
                sign * binomial * kernel.values[kernel_power] * moments[moment_power];
            frozen.magnitudes[power] +=
                binomial * kernel.magnitudes[kernel_power] * magnitudes[moment_power];
        }
    }
    frozen
}

/// Adds `∫ (s − origin)^q δ⁽ᵐ⁾(s − at) ds = (−1)^m q!/(q−m)! (at − origin)^{q−m}`
/// for each of the mass's derivative weights.
fn add_point_mass(
    moments: &mut [[f64; EXACT_CAPACITY]; MOMENT_ORDERS],
    magnitudes: &mut [[f64; EXACT_CAPACITY]; MOMENT_ORDERS],
    mass: &PointMass,
    offset: f64,
    degree: usize,
) {
    let mut powers = [1.0; EXACT_CAPACITY];
    for power in 1..=degree {
        powers[power] = powers[power - 1] * offset;
    }
    for order in 0..MOMENT_ORDERS {
        for (derivative, &weight) in mass.weights[order].iter().enumerate() {
            if weight == 0.0 {
                continue;
            }
            let sign = if derivative % 2 == 0 { 1.0 } else { -1.0 };
            for moment_power in derivative..=degree {
                let falling = ((moment_power - derivative + 1)..=moment_power)
                    .fold(1.0, |acc, k| acc * k as f64);
                let term = falling * powers[moment_power - derivative];
                moments[order][moment_power] += sign * weight * term;
                magnitudes[order][moment_power] += weight.abs() * term.abs();
            }
        }
    }
}

/// The input's series past a window edge, as `AxisSignalTable::series_past`
/// returns it: each derivative order about the edge's float head.
type EdgeSeries = [Tracked; MOMENT_ORDERS];

impl AxisSignalTable {
    /// Every time `x` is not one polynomial across: its piece seams and
    /// its clamp edges.
    fn boundaries(&self) -> &[f64] {
        &self.boundaries
    }

    fn moment_tree_height(&self) -> usize {
        self.moment_tree
            .as_ref()
            .expect("exact polynomials read moments that were never built")
            .height()
    }

    /// Bounds on the terms `integrate_moments` sums for each order's
    /// moments over `[lo, hi]` about `origin`: each partial piece, hold and
    /// tree node contributes its mass times `(|δ| + reach)^q`.
    fn moment_magnitudes(
        &self,
        lo: f64,
        hi: f64,
        degree: usize,
        origin: f64,
    ) -> [[f64; MOMENT_POWER_CAPACITY]; MOMENT_ORDERS] {
        let tree = self
            .moment_tree
            .as_ref()
            .expect("moment magnitudes requested before moments were built");
        let moment_len = degree + 1;
        let mut magnitudes = [[0.0; MOMENT_POWER_CAPACITY]; MOMENT_ORDERS];
        let window = |from: f64, to: f64| (0.5 * (from + to) - origin).abs() + 0.5 * (to - from);
        if lo < self.first_t {
            let to = hi.min(self.first_t);
            add_reach_powers(
                self.held_before().abs() * (to - lo),
                window(lo, to),
                &mut magnitudes[0][..moment_len],
            );
        }
        let interior_lo = lo.max(self.first_t);
        let interior_hi = hi.min(self.last_t);
        if interior_hi > interior_lo {
            let first_piece = self.ends.partition_point(|&end| end <= interior_lo);
            let end_piece = self.starts.partition_point(|&start| start < interior_hi);
            let mut full_start = first_piece;
            let mut full_end = end_piece;
            let partial =
                |piece: usize,
                 from: f64,
                 to: f64,
                 magnitudes: &mut [[f64; MOMENT_POWER_CAPACITY]; MOMENT_ORDERS]| {
                    for (order, target) in magnitudes.iter_mut().enumerate() {
                        add_reach_powers(
                            self.piece_magnitude(piece, order) * (to - from),
                            window(from, to),
                            &mut target[..moment_len],
                        );
                    }
                };
            if self.starts[first_piece] < interior_lo {
                full_start += 1;
                partial(
                    first_piece,
                    interior_lo,
                    interior_hi.min(self.ends[first_piece]),
                    &mut magnitudes,
                );
            }
            let right_partial = full_start < full_end && self.ends[full_end - 1] > interior_hi;
            if right_partial {
                full_end -= 1;
            }
            tree.accumulate_magnitudes(full_start, full_end, origin, &mut magnitudes, moment_len);
            if right_partial {
                partial(
                    full_end,
                    self.starts[full_end],
                    interior_hi,
                    &mut magnitudes,
                );
            }
        }
        if hi > self.last_t {
            let from = lo.max(self.last_t);
            add_reach_powers(
                self.held_after().abs() * (hi - from),
                window(from, hi),
                &mut magnitudes[0][..moment_len],
            );
        }
        magnitudes
    }

    /// The Taylor series, about `edge.head`, of every derivative order of
    /// the region of `x` the real `edge` lies inside: a piece, or a clamped
    /// hold. `None` where that region is history or lookahead the table
    /// does not clamp.
    fn series_past(&self, edge: RealSum) -> Option<EdgeSeries> {
        let mut series = [Tracked::ZERO; MOMENT_ORDERS];
        if edge.order_of(self.first_t) == Ordering::Greater {
            if !self.boundary.clamps_before_signal() {
                return None;
            }
            series[0] = Tracked::exact(&[self.held_before()]);
            return Some(series);
        }
        if edge.order_of(self.last_t) == Ordering::Less {
            if !self.lookahead.clamps_past_signal() {
                return None;
            }
            series[0] = Tracked::exact(&[self.held_after()]);
            return Some(series);
        }
        let piece = self
            .starts
            .partition_point(|&start| edge.order_of(start) == Ordering::Less)
            .checked_sub(1)
            .expect("a window edge inside the signal lies past its first piece's start");
        assert!(
            edge.order_of(self.ends[piece]) == Ordering::Greater,
            "a window edge inside the signal lies past its last piece's end"
        );
        let shifted = Tracked::exact(&self.coeffs[piece]).shifted(edge.head - self.starts[piece]);
        for (order, slot) in series.iter_mut().enumerate() {
            *slot = shifted.derivative(order);
        }
        Some(series)
    }
}

impl MomentTree {
    fn height(&self) -> usize {
        self.leaf_count.trailing_zeros() as usize
    }

    /// Adds, for every node `accumulate` would read over `[start, end)`,
    /// the bound `mass·(|δ| + reach)^q` on the terms of its moments about
    /// `origin`.
    fn accumulate_magnitudes(
        &self,
        start: usize,
        end: usize,
        origin: f64,
        targets: &mut [[f64; MOMENT_POWER_CAPACITY]; MOMENT_ORDERS],
        moment_len: usize,
    ) {
        let (mut left, mut right) = (start + self.leaf_count, end + self.leaf_count);
        while left < right {
            if left & 1 != 0 {
                self.accumulate_node_magnitudes(left, origin, targets, moment_len);
                left += 1;
            }
            if right & 1 != 0 {
                right -= 1;
                self.accumulate_node_magnitudes(right, origin, targets, moment_len);
            }
            left /= 2;
            right /= 2;
        }
    }

    fn accumulate_node_magnitudes(
        &self,
        node: usize,
        origin: f64,
        targets: &mut [[f64; MOMENT_POWER_CAPACITY]; MOMENT_ORDERS],
        moment_len: usize,
    ) {
        if !self.origins[node].is_finite() {
            return;
        }
        let reach = (self.origins[node] - origin).abs() + self.reaches[node];
        for (order, target) in targets.iter_mut().enumerate() {
            add_reach_powers(
                self.masses[node * MOMENT_ORDERS + order],
                reach,
                &mut target[..moment_len],
            );
        }
    }
}

/// `targets[q] += mass · reach^q`.
fn add_reach_powers(mass: f64, reach: f64, targets: &mut [f64]) {
    let mut power = mass;
    for target in targets {
        *target += power;
        power *= reach;
    }
}

#[cfg(test)]
#[path = "shaper_exact_tests.rs"]
mod tests;
