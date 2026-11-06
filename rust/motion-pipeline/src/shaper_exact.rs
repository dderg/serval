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
//! [`ExactRounding::depth`]. Where an edge's rounding `ε` is tiny against
//! the radius, the window's shift by it stops at first order and the
//! dropped terms' bound joins the rounding term
//! ([`CentredSum::add_shifted`]).
//!
//! # What it covers
//!
//! Real sub-intervals holding a float time; one narrower than an ulp holds
//! no time any evaluation can name, so it is left to its neighbours, whose
//! polynomials are bounded to one ulp past their own breaks.
use std::cmp::Ordering;

use nurbs::algebra::PiecewisePolynomialKernel;
use trajectory::next_toward;

use crate::follower_projection::UnshapedLeaderStep;

use super::{
    AxisSignalTable, FALLING_FACTORIALS, GainImpulse, MOMENT_ORDERS, MOMENT_POWER_CAPACITY,
    MomentBounds, MomentTree, TRANSLATION_BINOMIALS, taylor_shift,
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

    /// Writes `coefficients`, exact, shifted by `shift` into `shifted`,
    /// whose every slot it sets.
    fn exact_shifted_into(coefficients: &[f64], shift: f64, shifted: &mut Self) {
        let len = coefficients.len();
        assert!(len <= EXACT_CAPACITY);
        let mut magnitudes = [0.0; EXACT_CAPACITY];
        for (magnitude, value) in magnitudes.iter_mut().zip(coefficients) {
            *magnitude = value.abs();
        }
        shifted.truncate_to(len);
        taylor_shift(coefficients, shift, &mut shifted.values[..len]);
        taylor_shift(&magnitudes[..len], shift.abs(), &mut shifted.magnitudes[..len]);
    }

    /// Writes the `order`-th derivative into `derivative`, whose every
    /// slot it sets.
    fn derivative_into(&self, order: usize, derivative: &mut Self) {
        derivative.truncate_to(self.len.saturating_sub(order));
        for power in 0..derivative.len {
            let falling = FALLING_FACTORIALS[power + order][order];
            derivative.values[power] = self.values[power + order] * falling;
            derivative.magnitudes[power] = self.magnitudes[power + order] * falling;
        }
    }

    /// Shortens or lengthens to `len`, zeroing the slots the old length
    /// held past the new one and leaving the rest to the caller to set.
    fn truncate_to(&mut self, len: usize) {
        if len < self.len {
            self.values[len..self.len].fill(0.0);
            self.magnitudes[len..self.len].fill(0.0);
        }
        self.len = len;
    }

    fn add(&mut self, other: &Self, sign: f64) {
        self.len = self.len.max(other.len);
        for power in 0..other.len {
            self.values[power] += sign * other.values[power];
            self.magnitudes[power] += other.magnitudes[power];
        }
    }

    /// Clears to the zero polynomial of length `len`: every slot up to the
    /// longer of the old and new lengths zeroed, so every slot past the
    /// length stays zero as in [`Self::ZERO`].
    fn clear_to(&mut self, len: usize) {
        let reach = self.len.max(len);
        self.values[..reach].fill(0.0);
        self.magnitudes[..reach].fill(0.0);
        self.len = len;
    }

    /// Writes `∫₀^z self(z − v) · other(v) dv`, a polynomial in `z`, into
    /// `product`.
    fn beta_convolution_into(&self, other: &Self, product: &mut Self) {
        if self.len == 0 || other.len == 0 {
            product.clear_to(0);
            return;
        }
        let len = self.len + other.len;
        assert!(
            len <= EXACT_CAPACITY,
            "a sliver of degree {} exceeds the exact polynomial capacity",
            len - 1
        );
        product.clear_to(len);
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
    }
}

/// The derivative order of acceleration, whose slope rounding the exact
/// polynomials bound for the jerk read off them ([`ExactPiece::jerk`]).
const ACCELERATION: usize = 2;

/// A first-order Taylor shift by `ε` truncates `Σⱼ pⱼ·C(j,2)·ε²·ξ^{j−2}` for
/// some `|ξ| ≤ r + |ε|`; up to this `|ε|/r` that is below the coefficients'
/// own rounding, and past it the full shift is cheaper than the slack.
const FIRST_ORDER_SHIFT_LIMIT: f64 = 1.0 / (1u64 << 30) as f64;

/// One derivative order of the signal about a sub-interval's centre, in the
/// centre's offset `w` with `|w| ≤ radius`, summed from contributions each
/// written in its own window's motion `z = w + ε`. `magnitude` bounds the
/// terms' sum on `|w| ≤ radius` and `truncation` the polynomial dropped by
/// first-order shifts there; `slope_magnitude` and `slope_truncation` bound
/// their slopes in `w` likewise, where [`Self::add_slope_bounds`] adds them.
#[derive(Clone, Copy, Debug)]
struct CentredSum {
    values: [f64; EXACT_CAPACITY],
    len: usize,
    magnitude: f64,
    truncation: f64,
    slope_magnitude: f64,
    slope_truncation: f64,
}

impl CentredSum {
    const ZERO: Self = Self {
        values: [0.0; EXACT_CAPACITY],
        len: 0,
        magnitude: 0.0,
        truncation: 0.0,
        slope_magnitude: 0.0,
        slope_truncation: 0.0,
    };

    /// Adds `sign · term(w + shift)`. Shifting `|w|^j` by `shift` gives
    /// `(r + |shift|)^j` on `|w| ≤ r`, so the shifted terms' magnitude is
    /// `Σⱼ mⱼ·(r + |shift|)ʲ` without shifting the magnitudes. A shift
    /// within [`FIRST_ORDER_SHIFT_LIMIT`] of the radius keeps
    /// `pₖ + (k+1)·shift·pₖ₊₁` and adds the remainder's bound
    /// `Σⱼ 2·mⱼ·C(j,2)·shift²·(r + |shift|)^{j−2}` to the truncation, the
    /// factor two covering the magnitudes' rounding.
    fn add_shifted(&mut self, term: &Tracked, shift: f64, sign: f64, radius: f64) {
        let len = term.len;
        self.len = self.len.max(len);
        let reach = radius + shift.abs();
        self.magnitude += term.magnitudes[..len]
            .iter()
            .rev()
            .fold(0.0, |acc, &magnitude| acc * reach + magnitude);
        if shift == 0.0 {
            for (value, &term) in self.values.iter_mut().zip(&term.values[..len]) {
                *value += sign * term;
            }
        } else if shift.abs() <= radius * FIRST_ORDER_SHIFT_LIMIT {
            for power in 0..len {
                let slope = if power + 1 < len {
                    (power + 1) as f64 * shift * term.values[power + 1]
                } else {
                    0.0
                };
                self.values[power] += sign * (term.values[power] + slope);
            }
            let remainder = (2..len).rev().fold(0.0, |acc, power| {
                acc * reach + TRANSLATION_BINOMIALS[power][2] * term.magnitudes[power]
            });
            self.truncation += 2.0 * shift * shift * remainder;
        } else {
            let mut shifted = [0.0; EXACT_CAPACITY];
            taylor_shift(&term.values[..len], shift, &mut shifted[..len]);
            for (value, shifted) in self.values.iter_mut().zip(&shifted[..len]) {
                *value += sign * shifted;
            }
        }
    }
}

impl CentredSum {
    /// Adds the slope bounds of `term(w + shift)` beside
    /// [`Self::add_shifted`]'s. Each coefficient `j` errs by at most `γ·mⱼ`,
    /// so the error's slope on `|w| ≤ r` stays within
    /// `γ·Σⱼ j·mⱼ·(r + |shift|)^{j−1}`: the constant term, which carries the
    /// largest magnitudes, moves no slope. A first-order shift drops a
    /// polynomial whose slope `Σⱼ pⱼ·j·((w+ε)^{j−1} − w^{j−1} − (j−1)·ε·w^{j−2})`
    /// is within `Σⱼ 2·mⱼ·j·C(j−1,2)·ε²·(r + |ε|)^{j−3}`.
    fn add_slope_bounds(&mut self, term: &Tracked, shift: f64, radius: f64) {
        let len = term.len;
        let reach = radius + shift.abs();
        self.slope_magnitude += (1..len).rev().fold(0.0, |acc, power| {
            acc * reach + power as f64 * term.magnitudes[power]
        });
        if shift != 0.0 && shift.abs() <= radius * FIRST_ORDER_SHIFT_LIMIT {
            let remainder = (3..len).rev().fold(0.0, |acc, power| {
                acc * reach
                    + power as f64 * TRANSLATION_BINOMIALS[power - 1][2] * term.magnitudes[power]
            });
            self.slope_truncation += 2.0 * shift * shift * remainder;
        }
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
    narrowest_piece: f64,
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
            narrowest_piece: kernel
                .pieces
                .iter()
                .map(|piece| piece.u_end - piece.u_start)
                .fold(f64::INFINITY, f64::min),
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
        taylor_model::gamma(depth)
    }
}

/// One derivative order of the signal on a sub-interval, in the local
/// variable `x = (t − center) / radius`, with a bound on how far the
/// computed coefficients' polynomial can be from the exact one on
/// `|x| ≤ 1`, and one on how far its time derivative can be from the exact
/// one's there, infinite where nothing bounds it.
#[derive(Clone, Copy, Debug)]
pub(crate) struct LocalPolynomial {
    pub coefficients: [f64; EXACT_CAPACITY],
    pub len: usize,
    pub rounding: f64,
    pub slope_rounding: f64,
}

impl LocalPolynomial {
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
/// and lies within `radius` of `center`. A piece inside an unshaped
/// leader's square-root onset carries the [`UnshapedLeaderStep`] its
/// acceleration is exempt under: it bounds no acceleration there.
#[derive(Clone, Copy, Debug)]
pub(crate) struct ExactPiece {
    pub lo: f64,
    pub hi: f64,
    pub center: f64,
    pub radius: f64,
    pub pva: [LocalPolynomial; 3],
    pub unshaped_step: Option<UnshapedLeaderStep>,
}

impl ExactPiece {
    /// Whether the piece bounds the signal's derivative `order`.
    pub(crate) fn bounds(&self, order: usize) -> bool {
        order < 2 || self.unshaped_step.is_none()
    }

    /// The signal's jerk on the piece, its acceleration polynomial's time
    /// derivative, within the acceleration's slope rounding and the
    /// roundings of the radius powers its coefficients were scaled by and
    /// of the differentiation.
    pub(crate) fn jerk(&self) -> LocalPolynomial {
        let acceleration = &self.pva[2];
        let len = acceleration.len.saturating_sub(1);
        let mut coefficients = [0.0; EXACT_CAPACITY];
        let mut scaling = 0.0;
        for (power, slot) in coefficients[..len].iter_mut().enumerate() {
            let falling = (power + 1) as f64;
            *slot = acceleration.coefficients[power + 1] * falling / self.radius;
            scaling += (power + 4) as f64 * f64::EPSILON * slot.abs();
        }
        LocalPolynomial {
            coefficients,
            len,
            rounding: acceleration.slope_rounding + 2.0 * scaling,
            slope_rounding: f64::INFINITY,
        }
    }

    pub(crate) fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        let x = (t - self.center) / self.radius;
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

/// A span's exact polynomials in time order: every truth sample of its fit
/// is read off them, and the hull proves its pieces against them.
pub(crate) struct ExactSpan {
    pieces: Vec<ExactPiece>,
}

impl ExactSpan {
    pub(crate) fn new(pieces: Vec<ExactPiece>) -> Self {
        assert!(
            pieces
                .windows(2)
                .all(|pair| pair[0].lo <= pair[1].lo && pair[0].hi <= pair[1].hi),
            "exact polynomials of a span are not in time order"
        );
        Self { pieces }
    }

    /// The signal's value and derivatives at `t`, from the sub-interval on
    /// the side of `t` facing `toward`: at a break both neighbours'
    /// polynomials reach `t`, and each is the signal's limit from its own
    /// side. Where breaks crowd within ulps of `t`, the sub-intervals
    /// between them hold no float and no polynomial; the nearest polynomial
    /// facing `toward` is read, as the hull bounds it.
    pub(crate) fn pva(&self, t: f64, toward: f64) -> (f64, f64, f64) {
        self.facing(t, toward).eval_pva(t)
    }

    fn facing(&self, t: f64, toward: f64) -> &ExactPiece {
        let next = next_toward(t, toward);
        let within_span = self
            .pieces
            .first()
            .zip(self.pieces.last())
            .is_some_and(|(first, last)| first.lo <= next && next <= last.hi);
        match ordered(t, toward) {
            Ordering::Less => self
                .pieces
                .get(self.pieces.partition_point(|piece| piece.hi <= t)),
            Ordering::Greater => self
                .pieces
                .partition_point(|piece| piece.lo < t)
                .checked_sub(1)
                .and_then(|index| self.pieces.get(index)),
            Ordering::Equal => panic!("an exact read at {t} faces no side"),
        }
        .filter(|_| within_span)
        .unwrap_or_else(|| panic!("an exact read at {t} lies outside its span"))
    }

    /// The signal's jerk at `t`, read as [`Self::pva`] reads the rest.
    pub(crate) fn jerk(&self, t: f64, toward: f64) -> f64 {
        let piece = self.facing(t, toward);
        piece.jerk().eval((t - piece.center) / piece.radius)
    }

    /// The break inside `(lo, hi)` nearest `t`: the start of the later of
    /// the two polynomials meeting there.
    pub(crate) fn break_nearest(&self, t: f64, (lo, hi): (f64, f64)) -> Option<f64> {
        let first = self.pieces.partition_point(|piece| piece.lo <= lo).max(1);
        let past = self.pieces.partition_point(|piece| piece.lo < hi);
        (first..past)
            .map(|later| self.pieces[later].lo)
            .min_by(|a, b| (a - t).abs().total_cmp(&(b - t).abs()))
    }

    /// How far the polynomials reaching `t` disagree on the position there,
    /// zero where only one does: each is the signal within its own
    /// rounding, and an edge on a break may read either.
    pub(crate) fn position_mismatch_at(&self, t: f64) -> f64 {
        let reaching = &self.pieces[self.pieces.partition_point(|piece| piece.hi < t)..];
        let (low, high) = reaching
            .iter()
            .take_while(|piece| piece.lo <= t)
            .map(|piece| piece.pva[0].eval((t - piece.center) / piece.radius))
            .fold(
                (f64::INFINITY, f64::NEG_INFINITY),
                |(low, high), position| (low.min(position), high.max(position)),
            );
        if low <= high { high - low } else { 0.0 }
    }

    /// The polynomials whose sub-intervals reach into `(lo, hi)`.
    pub(crate) fn over(&self, lo: f64, hi: f64) -> &[ExactPiece] {
        let first = self.pieces.partition_point(|piece| piece.hi <= lo);
        let past = self.pieces.partition_point(|piece| piece.lo < hi);
        &self.pieces[first..past.max(first)]
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
    /// Whether the acceleration polynomials bound their slope's rounding,
    /// for a fit holding a jerk tolerance ([`ExactPiece::jerk`]).
    pub bounds_jerk: bool,
}

/// The chunks one signal's spans have read their exact polynomials off
/// lately, by centre and radius: a span's pieces reach an ulp into the
/// chunks of the sub-intervals beside it ([`ExactConvolution::pieces_over`]),
/// whose own spans read those chunks whole after it.
#[derive(Default)]
pub(crate) struct RecentChunks(std::sync::Mutex<RecentChunkList>);

type RecentChunkList = std::collections::VecDeque<((u64, u64), Option<[LocalPolynomial; 3]>)>;

/// How many chunks [`RecentChunks`] keeps: a span reaches into one on
/// either side.
const RECENT_CHUNKS: usize = 8;

impl RecentChunks {
    fn read(
        &self,
        center: f64,
        radius: f64,
        compute: impl FnOnce() -> Option<[LocalPolynomial; 3]>,
    ) -> Option<[LocalPolynomial; 3]> {
        let key = (center.to_bits(), radius.to_bits());
        let mut recent = self.0.lock().expect("a chunk computation panicked");
        if let Some(&(_, polynomials)) = recent.iter().find(|(held, _)| *held == key) {
            return polynomials;
        }
        let polynomials = compute();
        if recent.len() == RECENT_CHUNKS {
            recent.pop_front();
        }
        recent.push_back((key, polynomials));
        polynomials
    }
}

/// What one span's exact polynomials evaluate in and reuse from chunk to
/// chunk and kernel piece to kernel piece: the input's series past two
/// window edges, one kernel piece's moments and their bounds, and the
/// frozen window and sliver terms each order adds, each read only as far
/// as the evaluation writes it. Each thread keeps one
/// ([`CONVOLUTION_SCRATCH`]) across every evaluation it runs.
struct ConvolutionScratch {
    edge_series: [EdgeSeries; 2],
    moments: [[f64; EXACT_CAPACITY]; MOMENT_ORDERS],
    magnitudes: MomentBounds,
    at_upper: Tracked,
    convolved: Tracked,
}

thread_local! {
    static CONVOLUTION_SCRATCH: std::cell::RefCell<ConvolutionScratch> =
        const {
            std::cell::RefCell::new(ConvolutionScratch {
                edge_series: [[Tracked::ZERO; MOMENT_ORDERS]; 2],
                moments: [[0.0; EXACT_CAPACITY]; MOMENT_ORDERS],
                magnitudes: [[0.0; EXACT_CAPACITY]; MOMENT_ORDERS],
                at_upper: Tracked::ZERO,
                convolved: Tracked::ZERO,
            })
        };
}

/// One point mass of `x` and its derivatives: `weights[m]` multiplies
/// `δ⁽ᵐ⁾(s − at)` in derivative order `order`.
struct PointMass {
    at: f64,
    weights: [[f64; 4]; MOMENT_ORDERS],
}

impl ExactConvolution<'_> {
    /// The signal's exact polynomial on every real sub-interval between its
    /// breaks that reaches into `[lo, hi]` and holds a float time, or
    /// `None` where the convolution reads history or lookahead the table
    /// does not hold. A sub-interval wider than the kernel's narrowest
    /// piece is cut into chunks no wider: the slivers carry the kernel's
    /// series across the chunk, which is as well conditioned as the kernel
    /// only within the kernel's own width. Chunks and their centres follow
    /// from the whole sub-interval, not from `[lo, hi]`, so two spans
    /// meeting inside one read the same polynomial at their seam. Where a
    /// break an ulp inside `lo` or `hi` ends the history or lookahead the
    /// table holds, the float past it reads the table only by rounding: it
    /// is left to its neighbour, whose polynomial reaches an ulp past its
    /// break.
    pub(crate) fn pieces_over(
        &self,
        lo: f64,
        hi: f64,
        recent: &RecentChunks,
    ) -> Option<Vec<ExactPiece>> {
        assert!(
            lo <= hi,
            "exact polynomial over the empty interval [{lo}, {hi}]"
        );
        let mut breaks = Vec::new();
        let boundaries = self.table.boundaries();
        let mut below: Option<RealSum> = None;
        let mut above: Option<RealSum> = None;
        for &cut in &self.kernel.cuts {
            let first = boundaries.partition_point(|&boundary| {
                RealSum::of(boundary, cut).order_of(lo) != Ordering::Less
            });
            let past = boundaries.partition_point(|&boundary| {
                RealSum::of(boundary, cut).order_of(hi) == Ordering::Greater
            });
            if let Some(&boundary) = first.checked_sub(1).and_then(|last| boundaries.get(last)) {
                let candidate = RealSum::of(boundary, cut);
                if below.is_none_or(|below| candidate.cmp(below) == Ordering::Greater) {
                    below = Some(candidate);
                }
            }
            if let Some(&boundary) = boundaries.get(past) {
                let candidate = RealSum::of(boundary, cut);
                if above.is_none_or(|above| candidate.cmp(above) == Ordering::Less) {
                    above = Some(candidate);
                }
            }
            breaks.extend(
                boundaries[first..past.max(first)]
                    .iter()
                    .map(|&boundary| RealSum::of(boundary, cut)),
            );
        }
        breaks.sort_by(|left, right| left.cmp(*right));
        breaks.dedup();
        let points: Vec<RealSum> = std::iter::once(below.unwrap_or(RealSum::float(lo)))
            .chain(breaks)
            .chain(std::iter::once(above.unwrap_or(RealSum::float(hi))))
            .collect();
        let mut pieces = Vec::with_capacity(points.len() - 1);
        for pair in points.windows(2) {
            let first_inside = pair[0].first_float_above();
            let last_inside = pair[1].last_float_below();
            if first_inside > last_inside {
                continue;
            }
            let extent_lo = next_toward(first_inside, f64::NEG_INFINITY);
            let extent_hi = next_toward(last_inside, f64::INFINITY);
            let extent = extent_hi - extent_lo;
            let chunks = (extent / self.kernel.narrowest_piece).ceil().max(1.0);
            let chunk_count = chunks as usize;
            let chunk_edge = |chunk: usize| match chunk {
                0 => extent_lo,
                _ if chunk == chunk_count => extent_hi,
                _ => extent_lo + extent * (chunk as f64 / chunks),
            };
            let chunk_near = |t: f64| ((t - extent_lo) / extent * chunks).max(0.0) as usize;
            let first_chunk = chunk_near(lo).saturating_sub(1);
            let past_chunk = (chunk_near(hi) + 2).min(chunk_count);
            pieces.reserve(past_chunk.saturating_sub(first_chunk));
            for chunk in first_chunk..past_chunk {
                let (chunk_lo, chunk_hi) = (chunk_edge(chunk), chunk_edge(chunk + 1));
                let (piece_lo, piece_hi) = (chunk_lo.max(lo), chunk_hi.min(hi));
                if piece_lo > piece_hi || (piece_lo == piece_hi && lo < hi) {
                    continue;
                }
                let center =
                    (chunk_lo + 0.5 * (chunk_hi - chunk_lo)).clamp(first_inside, last_inside);
                let radius = next_toward((center - chunk_lo).max(chunk_hi - center), f64::INFINITY);
                let Some(pva) = recent.read(center, radius, || {
                    CONVOLUTION_SCRATCH
                        .with_borrow_mut(|scratch| self.polynomials_at(center, radius, scratch))
                }) else {
                    let (first_held, last_held) = (first_inside.max(lo), last_inside.min(hi));
                    if first_held == last_held && (first_held == lo || first_held == hi) {
                        continue;
                    }
                    return None;
                };
                pieces.push(ExactPiece {
                    lo: piece_lo,
                    hi: piece_hi,
                    center,
                    radius,
                    pva,
                    unshaped_step: None,
                });
            }
        }
        if let Some(first) = pieces.first_mut() {
            first.lo = first.lo.min(lo);
        }
        if let Some(last) = pieces.last_mut() {
            last.hi = last.hi.max(hi);
        }
        Some(pieces)
    }

    /// The point masses strictly inside the real window `(lower, upper)`:
    /// both lists are in time order, so the window is two binary searches.
    fn point_masses_within(
        &self,
        lower: RealSum,
        upper: RealSum,
    ) -> impl Iterator<Item = PointMass> + '_ {
        let after_lower = move |at: f64| lower.order_of(at) == Ordering::Greater;
        let before_upper = move |at: f64| upper.order_of(at) == Ordering::Less;
        let jumps = self.table.input_jumps();
        let jump_start = jumps.partition_point(|jump| !after_lower(jump.0));
        let jump_end = jumps.partition_point(|jump| before_upper(jump.0));
        let impulse_start = self
            .impulses
            .partition_point(|impulse| !after_lower(impulse.at));
        let impulse_end = self
            .impulses
            .partition_point(|impulse| before_upper(impulse.at));
        let jump_range = jump_start..jump_end.max(jump_start);
        let impulse_range = impulse_start..impulse_end.max(impulse_start);
        let jumps = jumps[jump_range]
            .iter()
            .map(|&(at, position_jump, slope_jump)| PointMass {
                at,
                weights: [
                    [0.0; 4],
                    [position_jump, 0.0, 0.0, 0.0],
                    [slope_jump, position_jump, 0.0, 0.0],
                ],
            });
        let impulses = self.impulses[impulse_range]
            .iter()
            .map(|impulse| PointMass {
                at: impulse.at,
                weights: [
                    [impulse.delta, impulse.doublet, 0.0, 0.0],
                    [0.0, impulse.delta, impulse.doublet, 0.0],
                    [0.0, 0.0, impulse.delta, impulse.doublet],
                ],
            });
        jumps.chain(impulses)
    }

    fn polynomials_at(
        &self,
        center: f64,
        radius: f64,
        scratch: &mut ConvolutionScratch,
    ) -> Option<[LocalPolynomial; 3]> {
        let mut total = [CentredSum::ZERO; MOMENT_ORDERS];
        let mut point_masses = 0;
        let ConvolutionScratch {
            edge_series,
            moments,
            magnitudes,
            at_upper,
            convolved,
        } = scratch;
        let mut previous_lower: Option<(RealSum, usize)> = None;
        for piece in &self.kernel.pieces {
            let lower = RealSum::of(center, -piece.end);
            let upper = RealSum::of(center, -piece.start);
            let degree = piece.at_start.len - 1;
            for bounds in magnitudes.iter_mut() {
                bounds[..=degree].fill(0.0);
            }
            {
                let [position, velocity, acceleration] = &mut *moments;
                if !self.table.integrate_bounded_moments(
                    lower.head,
                    upper.head,
                    degree,
                    upper.head,
                    [
                        &mut position[..=degree],
                        &mut velocity[..=degree],
                        &mut acceleration[..=degree],
                    ],
                    Some(&mut *magnitudes),
                ) {
                    return None;
                }
            }
            for mass in self.point_masses_within(lower, upper) {
                point_masses += 1;
                add_point_mass(moments, magnitudes, &mass, mass.at - upper.head, degree);
            }
            let upper_slot = match previous_lower {
                Some((edge, slot)) if edge.head == upper.head && edge.tail == upper.tail => slot,
                _ => {
                    let slot = previous_lower.map_or(0, |(_, slot)| 1 - slot);
                    if !self.table.series_past_into(upper, &mut edge_series[slot]) {
                        return None;
                    }
                    slot
                }
            };
            let lower_slot = 1 - upper_slot;
            if !self
                .table
                .series_past_into(lower, &mut edge_series[lower_slot])
            {
                return None;
            }
            previous_lower = Some((lower, lower_slot));
            let (upper_series, lower_series) = (&edge_series[upper_slot], &edge_series[lower_slot]);
            for order in 0..MOMENT_ORDERS {
                frozen_window_into(
                    &piece.at_start,
                    &moments[order],
                    &magnitudes[order],
                    at_upper,
                );
                piece
                    .at_start
                    .beta_convolution_into(&upper_series[order], convolved);
                at_upper.add(convolved, 1.0);
                total[order].add_shifted(at_upper, upper.tail, 1.0, radius);
                if order == ACCELERATION && self.bounds_jerk {
                    total[order].add_slope_bounds(at_upper, upper.tail, radius);
                }
                piece
                    .at_end
                    .beta_convolution_into(&lower_series[order], convolved);
                total[order].add_shifted(convolved, lower.tail, -1.0, radius);
                if order == ACCELERATION && self.bounds_jerk {
                    total[order].add_slope_bounds(convolved, lower.tail, radius);
                }
            }
        }
        let gamma = ExactRounding::gamma(ExactRounding::depth(
            self.table.moment_tree_height(),
            self.kernel.pieces.len(),
            point_masses,
        ));
        Some(std::array::from_fn(|order| {
            let mut sum = total[order];
            if order == 0 {
                sum.len = sum.len.max(1);
                sum.values[0] += self.position_offset;
                sum.magnitude += self.position_offset.abs();
            }
            let mut scale = 1.0;
            let mut coefficients = [0.0; EXACT_CAPACITY];
            for power in 0..sum.len {
                coefficients[power] = sum.values[power] * scale;
                scale *= radius;
            }
            assert!(
                coefficients[..sum.len].iter().all(|c| c.is_finite()),
                "exact polynomial of order {order} at {center} is not finite"
            );
            LocalPolynomial {
                coefficients,
                len: sum.len,
                rounding: 2.0 * gamma * sum.magnitude + sum.truncation,
                slope_rounding: if order == ACCELERATION && self.bounds_jerk {
                    2.0 * gamma * sum.slope_magnitude + sum.slope_truncation
                } else {
                    f64::INFINITY
                },
            }
        }))
    }
}

/// `Σ_r z^r Σ_l c_l C(l, r) (−1)^{l−r} M_{l−r}`: the kernel piece over the
/// frozen window, as a polynomial in the window's motion `z`.
fn frozen_window_into(kernel: &Tracked, moments: &[f64], magnitudes: &[f64], frozen: &mut Tracked) {
    frozen.clear_to(kernel.len);
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
    let len = degree + 1;
    let mut powers = [1.0; EXACT_CAPACITY];
    for power in 1..len {
        powers[power] = powers[power - 1] * offset;
    }
    for ((weights, moments), magnitudes) in mass
        .weights
        .iter()
        .zip(moments.iter_mut())
        .zip(magnitudes.iter_mut())
    {
        for (derivative, &weight) in weights.iter().enumerate() {
            if weight == 0.0 || derivative >= len {
                continue;
            }
            let signed = if derivative % 2 == 0 { weight } else { -weight };
            let scale = weight.abs();
            for power in derivative..len {
                let term = FALLING_FACTORIALS[power][derivative] * powers[power - derivative];
                moments[power] += signed * term;
                magnitudes[power] += scale * term.abs();
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

    /// Bounds the terms a clamped hold of `value` over `[from, to]` adds to
    /// the position moments about `origin`.
    pub(super) fn bound_held(value: f64, (from, to): (f64, f64), origin: f64, targets: &mut [f64]) {
        add_reach_powers(
            value.abs() * (to - from),
            window_reach(from, to, origin),
            targets,
        );
    }

    /// Bounds the terms part `[from, to]` of piece `piece` adds to every
    /// order's moments about `origin`.
    pub(super) fn bound_partial_piece(
        &self,
        piece: usize,
        (from, to): (f64, f64),
        origin: f64,
        bounds: &mut MomentBounds,
        moment_len: usize,
    ) {
        for (order, target) in bounds.iter_mut().enumerate() {
            add_reach_powers(
                self.piece_magnitude(piece, order) * (to - from),
                window_reach(from, to, origin),
                &mut target[..moment_len],
            );
        }
    }

    /// Writes into `series` the Taylor series, about `edge.head`, of every
    /// derivative order of the region of `x` the real `edge` lies inside: a
    /// piece, or a clamped hold. `false` where that region is history or
    /// lookahead the table does not clamp.
    fn series_past_into(&self, edge: RealSum, series: &mut EdgeSeries) -> bool {
        let held = if edge.order_of(self.first_t) == Ordering::Greater {
            if !self.boundary.clamps_before_signal() {
                return false;
            }
            Some(self.held_before())
        } else if edge.order_of(self.last_t) == Ordering::Less {
            if !self.lookahead.clamps_past_signal() {
                return false;
            }
            Some(self.held_after())
        } else {
            None
        };
        if let Some(value) = held {
            let [position, higher @ ..] = series;
            Tracked::exact_shifted_into(&[value], 0.0, position);
            for slot in higher {
                slot.truncate_to(0);
            }
            return true;
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
        let [position, higher @ ..] = series;
        Tracked::exact_shifted_into(
            &self.coeffs[piece],
            edge.head - self.origins[piece],
            position,
        );
        for (order, slot) in (1..).zip(higher.iter_mut()) {
            position.derivative_into(order, slot);
        }
        true
    }
}

impl MomentTree {
    fn height(&self) -> usize {
        self.leaf_count.trailing_zeros() as usize
    }

    /// Adds the bound `mass·(|δ| + reach)^q` on the terms of node `node`'s
    /// moments about `origin`.
    pub(super) fn accumulate_node_magnitudes(
        &self,
        node: usize,
        origin: f64,
        targets: &mut MomentBounds,
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

/// How far from `origin` a point of `[from, to]` lies.
fn window_reach(from: f64, to: f64, origin: f64) -> f64 {
    (0.5 * (from + to) - origin).abs() + 0.5 * (to - from)
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
