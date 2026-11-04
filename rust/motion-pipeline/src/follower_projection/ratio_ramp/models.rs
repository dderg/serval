//! Taylor-model readings of the smoothing on one window: the correction
//! `X` the evaluator sums over the seams inside the window
//! ([`RatioSmoothing::correction`]) and the advance's valley weight
//! ([`RatioSmoothing::valley_weight`]), each the formula the evaluator reads
//! over a stretch of path no bend crosses.
//!
//! The formula is the one both ends of the window's odometer reach read:
//! the width node interval, the seams inside the window and those behind
//! its point, and each valley's side. The bends are where these change
//! ([`RatioSmoothing::bends`]); a reach whose ends read different ones has
//! no model. The bends are located to the float by their own rounding, an
//! ulp or so from where the evaluator's flips, so the ends are read a few
//! ulps inside the reach: the formulas meet to second order at a bend.

use crate::shaper::taylor::{self, Ball, TaylorModel};

use super::{
    AdvanceValley, FALL_TO_REST, KERNEL_HALF_VARIANCE, RISE_FROM_REST, RatioSmoothing,
    SMOOTHED_HALF_SQUARE, SMOOTHED_RAMP, SMOOTHERSTEP, SPAN_LOOKUP_SLACK_MM, WidthField,
};

/// `C(n, k)` for the moments' expansions about a window's centre.
const BINOMIALS: [[f64; 6]; 6] = [
    [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [1.0, 1.0, 0.0, 0.0, 0.0, 0.0],
    [1.0, 2.0, 1.0, 0.0, 0.0, 0.0],
    [1.0, 3.0, 3.0, 1.0, 0.0, 0.0],
    [1.0, 4.0, 6.0, 4.0, 1.0, 0.0],
    [1.0, 5.0, 10.0, 10.0, 5.0, 1.0],
];

/// How many ulps of the path inside its reach the formula of a window is
/// read.
const REACH_INSET_ULPS: f64 = 4.0;

/// The points `reach` is read at for its formula: its ends moved
/// [`REACH_INSET_ULPS`] inside, or its ends themselves where it is no
/// wider than that: a reach that narrow holds only floats a bend may
/// separate.
fn read_points((lo, hi): (f64, f64)) -> (f64, f64) {
    let inset = REACH_INSET_ULPS * f64::EPSILON * lo.abs().max(hi.abs()).max(1.0);
    if hi - lo > 2.0 * inset {
        (lo + inset, hi - inset)
    } else {
        (lo, hi)
    }
}

/// Which formula of the correction a stretch of path reads
/// ([`RatioSmoothing::correction`]): none, or the width's node interval,
/// the seams inside the window, how far those behind its point run, and
/// the table's slope at its back edge.
#[derive(Clone, Copy, PartialEq, Debug)]
enum CorrectionFormula {
    Zero,
    Smoothed {
        interval: WidthInterval,
        seams: (usize, usize),
        behind: usize,
        slope_left: f64,
    },
}

/// How far in the distance from a window's centre the correction's
/// derivatives are read as polynomials.
const SERIES_DEGREE: usize = 3;

/// The correction's Taylor series terms a window reads: its third
/// derivative to [`SERIES_DEGREE`], and one past, which bounds the rest.
const SERIES_LEN: usize = 3 + SERIES_DEGREE + 2;

/// A real number as a float and two bounds on its distance from it: `at`
/// where the series is read at the window's centre, `over` where it is read
/// anywhere on the window's reach — the same floats either way, as the
/// reach only widens. The bounds' own rounding is left to
/// [`Lean::sealed`]: the series arithmetic runs a few hundred of these in a
/// chain, each bound a sum of products of non-negative floats, so the
/// chain's bound errs by far less than the share `taylor::upper` inflates
/// it by once at the end.
#[derive(Clone, Copy)]
struct Lean {
    mid: f64,
    at: f64,
    over: f64,
}

/// What underflow can take from the bounds of a chain of up to `2¹²`
/// operations.
const LEAN_UNDERFLOW: f64 = 4096.0 * f64::MIN_POSITIVE;

impl Lean {
    const ZERO: Self = Self {
        mid: 0.0,
        at: 0.0,
        over: 0.0,
    };

    fn of(ball: Ball) -> Self {
        Self {
            mid: ball.mid,
            at: ball.rad,
            over: ball.rad,
        }
    }

    /// The number read at the window's centre, and over its reach.
    fn sealed(self) -> [Ball; 2] {
        [self.at, self.over].map(|rad| Ball {
            mid: self.mid,
            rad: taylor::upper(rad) + LEAN_UNDERFLOW,
        })
    }

    fn reciprocal(self) -> Option<Self> {
        let magnitude = self.mid.abs();
        if !(magnitude > self.over) {
            return None;
        }
        let mid = 1.0 / self.mid;
        let rounding = f64::EPSILON * mid.abs();
        let spread = |rad: f64| rad / (magnitude * ((magnitude - rad) * (1.0 - f64::EPSILON)));
        Some(Self {
            mid,
            at: spread(self.at) + rounding,
            over: spread(self.over) + rounding,
        })
    }
}

impl std::ops::Add for Lean {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        let mid = self.mid + other.mid;
        let rounding = 0.5 * f64::EPSILON * mid.abs();
        Self {
            mid,
            at: self.at + other.at + rounding,
            over: self.over + other.over + rounding,
        }
    }
}

impl std::ops::Mul for Lean {
    type Output = Self;

    fn mul(self, other: Self) -> Self {
        let mid = self.mid * other.mid;
        let (a, b) = (self.mid.abs(), other.mid.abs());
        let rounding = 0.5 * f64::EPSILON * mid.abs();
        Self {
            mid,
            at: a * other.at + self.at * (b + other.at) + rounding,
            over: a * other.over + self.over * (b + other.over) + rounding,
        }
    }
}

impl std::ops::Neg for Lean {
    type Output = Self;

    fn neg(self) -> Self {
        Self {
            mid: -self.mid,
            ..self
        }
    }
}

/// A truncated Taylor series in the distance from a point of the path.
type Series = [Lean; SERIES_LEN];

/// The Taylor coefficients of `polynomial` about `at`, by repeated
/// Horner passes.
fn series_about(polynomial: &[Ball], at: Lean) -> Series {
    let mut work = [Lean::ZERO; SERIES_LEN];
    for (slot, &c) in work.iter_mut().zip(polynomial) {
        *slot = Lean::of(c);
    }
    if at.mid == 0.0 && at.over == 0.0 {
        return work;
    }
    let n = polynomial.len();
    for lowest in 0..n {
        for power in (lowest + 1..n).rev() {
            work[power - 1] = work[power - 1] + at * work[power];
        }
    }
    work
}

fn series_add(a: &Series, b: &Series) -> Series {
    std::array::from_fn(|k| a[k] + b[k])
}

fn series_mul(a: &Series, b: &Series) -> Series {
    std::array::from_fn(|k| (1..=k).fold(a[0] * b[k], |sum, j| sum + a[j] * b[k - j]))
}

fn series_reciprocal(a: &Series) -> Option<Series> {
    let leading = a[0].reciprocal()?;
    let mut out = [Lean::ZERO; SERIES_LEN];
    out[0] = leading;
    for k in 1..SERIES_LEN {
        let sum = (2..=k).fold(a[1] * out[k - 1], |sum, j| sum + a[j] * out[k - j]);
        out[k] = -(leading * sum);
    }
    Some(out)
}

/// The correction's ingredients as polynomials in the distance from a
/// point of the path ([`SeamMoments::correction`]): the width, the weight
/// of each power of it, and the table's kink behind the point.
struct CorrectionPolynomials {
    width: [Ball; 6],
    weights: [[Ball; 6]; 6],
    kink: [Ball; 3],
    seams: bool,
}

impl CorrectionPolynomials {
    /// The correction's Taylor series about `at` in the distance from the
    /// point; `None` where the width may reach zero there.
    fn series(&self, at: Lean) -> Option<Series> {
        let h = series_about(&self.width, at);
        let weight = |index: usize| series_about(&self.weights[index], at);
        let h_squared_weight = series_mul(&h, &series_mul(&h, &weight(0)));
        if !self.seams {
            return Some(h_squared_weight);
        }
        let growing = series_add(&h_squared_weight, &series_mul(&h, &weight(1)));
        let inverse = series_reciprocal(&h)?;
        let shrinking = series_mul(
            &inverse,
            &series_add(
                &series_mul(
                    &inverse,
                    &series_add(&series_mul(&inverse, &weight(5)), &weight(4)),
                ),
                &weight(3),
            ),
        );
        Some(series_add(
            &series_add(&series_about(&self.kink, at), &growing),
            &series_add(&weight(2), &shrinking),
        ))
    }
}

/// A model of a path function and of its first three path derivatives.
#[derive(Clone, Copy)]
pub(in crate::follower_projection) struct ModelJet {
    pub(in crate::follower_projection) value: TaylorModel,
    pub(in crate::follower_projection) d1: TaylorModel,
    pub(in crate::follower_projection) d2: TaylorModel,
    pub(in crate::follower_projection) d3: TaylorModel,
}

impl ModelJet {
    pub(in crate::follower_projection) fn zero() -> Self {
        let zero = TaylorModel::constant(Ball::exact(0.0));
        Self {
            value: zero,
            d1: zero,
            d2: zero,
            d3: zero,
        }
    }
}

/// Which formula of the width field a point reads: the node it sits on or
/// the two around it ([`WidthField::interval`]).
type WidthInterval = Result<usize, (usize, usize)>;

impl WidthField {
    /// The node interval every point of `reach` reads; `None` where its
    /// ends read different ones or lie past the nodes.
    fn width_interval(&self, reach: (f64, f64)) -> Option<WidthInterval> {
        let (first, last) = (self.nodes.first()?.s, self.nodes.last()?.s);
        if reach.0 < first - SPAN_LOOKUP_SLACK_MM || reach.1 > last + SPAN_LOOKUP_SLACK_MM {
            return None;
        }
        let reach = read_points(reach);
        let interval = self.formula_at(reach.0);
        (self.formula_at(reach.1) == interval).then_some(interval)
    }

    /// The width read on `interval` as a polynomial in the distance from
    /// `centre`.
    fn width_polynomial(&self, interval: WidthInterval, centre: f64) -> [Ball; 6] {
        let mut out = [Ball::exact(0.0); 6];
        let (a, b) = match interval {
            Ok(node) => {
                out[0] = Ball::exact(self.nodes[node].envelope);
                return out;
            }
            Err(pair) => pair,
        };
        let (a, b) = (self.nodes[a], self.nodes[b]);
        let (coefficients, scale, offset): (&[f64], f64, f64) =
            match (a.envelope == 0.0, b.envelope == 0.0) {
                (true, true) => return out,
                (true, false) => (&RISE_FROM_REST, b.envelope, 0.0),
                (false, true) => (&FALL_TO_REST, a.envelope, 0.0),
                (false, false) => (&SMOOTHERSTEP, b.envelope - a.envelope, a.envelope),
            };
        let per_s = Ball::exact(1.0 / (b.s - a.s));
        let at = Ball::rounded(centre - a.s, 1) * per_s;
        let mut exact = [Ball::exact(0.0); 6];
        for (slot, &c) in exact.iter_mut().zip(coefficients) {
            *slot = Ball::exact(c);
        }
        let about = series_about(&exact[..coefficients.len()], Lean::of(at)).map(|c| c.sealed()[0]);
        let mut power = Ball::exact(scale);
        for (slot, &c) in out.iter_mut().zip(&about).take(coefficients.len()) {
            *slot = c * power;
            power = power * per_s;
        }
        out[0] = out[0] + Ball::exact(offset);
        out
    }
}

/// The side of a valley a path point reads its weight on
/// ([`AdvanceValley::weight`]).
#[derive(Clone, Copy, PartialEq)]
enum ValleySide {
    Outside,
    Inside,
    Before,
    After,
}

impl AdvanceValley {
    fn side(&self, s: f64) -> ValleySide {
        if self.reach_start() >= s || self.reach_end() <= s {
            return ValleySide::Outside;
        }
        if s < self.start {
            if self.start - s >= self.before {
                ValleySide::Outside
            } else {
                ValleySide::Before
            }
        } else if s > self.end {
            if s - self.end >= self.after {
                ValleySide::Outside
            } else {
                ValleySide::After
            }
        } else {
            ValleySide::Inside
        }
    }

    /// `[w, w', w'']` on `side` at the model `s`.
    fn weight_model(&self, side: ValleySide, s: &TaylorModel) -> [TaylorModel; 3] {
        let constant = |value: f64| TaylorModel::constant(Ball::exact(value));
        let (inside, width, outward) = match side {
            ValleySide::Outside => return [constant(1.0), constant(0.0), constant(0.0)],
            ValleySide::Inside => return [constant(0.0), constant(0.0), constant(0.0)],
            ValleySide::Before => (
                s.add_ball(Ball::exact(-self.start))
                    .scale(Ball::exact(1.0 / self.before))
                    .add_ball(Ball::exact(1.0)),
                self.before,
                -1.0,
            ),
            ValleySide::After => (
                s.add_ball(Ball::exact(-self.end))
                    .scale(Ball::exact(-1.0 / self.after))
                    .add_ball(Ball::exact(1.0)),
                self.after,
                1.0,
            ),
        };
        let inside_squared = inside.mul(&inside);
        [
            inside_squared
                .mul(&inside)
                .scale(Ball::exact(-1.0))
                .add_ball(Ball::exact(1.0)),
            inside_squared.scale(Ball::rounded(outward * 3.0 / width, 1)),
            inside.scale(Ball::rounded(-6.0 / (width * width), 2)),
        ]
    }
}

impl RatioSmoothing {
    /// The correction `X` and its first three path derivatives on the model
    /// `s` of the path, `reach` its range; `None` where the reach's ends
    /// read different formulas, or the width may reach zero. Each
    /// derivative is its Taylor polynomial about the model's centre, to
    /// [`SERIES_DEGREE`] in the distance `δ` from it, and the Lagrange
    /// remainder past it, bounded by the series read with `δ` the whole
    /// reach of the model: an interval Taylor series.
    pub(in crate::follower_projection) fn correction_model(
        &self,
        s: &TaylorModel,
        reach: (f64, f64),
    ) -> Option<ModelJet> {
        let formula = self.correction_formula(reach)?;
        let (centre, radius) = s.spread();
        let Some(polynomials) = self.correction_polynomials(formula, centre) else {
            return Some(ModelJet::zero());
        };
        let series = polynomials
            .series(Lean {
                mid: 0.0,
                at: 0.0,
                over: radius,
            })?
            .map(Lean::sealed);
        let at_centre = series.map(|[at, _]| at);
        let over_reach = series.map(|[_, over]| over);
        let delta = s.add_ball(Ball::exact(-centre));
        let mut powers = [TaylorModel::constant(Ball::exact(1.0)); SERIES_DEGREE + 1];
        let mut reach_powers = [1.0; SERIES_DEGREE + 2];
        for k in 1..powers.len() {
            powers[k] = powers[k - 1].mul(&delta);
        }
        for k in 1..reach_powers.len() {
            reach_powers[k] = taylor::upper(reach_powers[k - 1] * radius);
        }
        let derivative = |order: usize| {
            let falling = |power: usize| ((power + 1)..=(power + order)).product::<usize>() as f64;
            let mut rounding = 0.0;
            let mut model = TaylorModel::constant(Ball::exact(0.0));
            for (power, model_power) in powers.iter().enumerate() {
                let coefficient = Ball::exact(falling(power)) * at_centre[order + power];
                model = model.add(&model_power.scale(Ball::exact(coefficient.mid)));
                rounding += coefficient.rad * reach_powers[power];
            }
            let past = over_reach[order + SERIES_DEGREE + 1];
            let remainder = falling(SERIES_DEGREE + 1)
                * (past.mid.abs() + past.rad)
                * reach_powers[SERIES_DEGREE + 1];
            model.widen(rounding + remainder);
            model
        };
        Some(ModelJet {
            value: derivative(0),
            d1: derivative(1),
            d2: derivative(2),
            d3: derivative(3),
        })
    }

    /// The formula of the correction every point of `reach` reads; `None`
    /// where its ends read different ones.
    fn correction_formula(&self, reach: (f64, f64)) -> Option<CorrectionFormula> {
        if self.is_inert() {
            return Some(CorrectionFormula::Zero);
        }
        let model_reach = reach;
        let reach = read_points(reach);
        let (h_lo, h_hi) = (self.width.jet(reach.0).value, self.width.jet(reach.1).value);
        if h_lo <= 0.0 && h_hi <= 0.0 {
            return Some(CorrectionFormula::Zero);
        }
        if h_lo <= 0.0 || h_hi <= 0.0 {
            return None;
        }
        let slope_left_of = |first: usize| {
            first
                .checked_sub(1)
                .map_or(self.base_slope.unwrap_or(0.0), |last| {
                    self.seams[last].slope_after
                })
        };
        let first = |at: f64, h: f64| self.seams.partition_point(|seam| seam.s <= at - h);
        let past = |at: f64, h: f64| self.seams.partition_point(|seam| seam.s < at + h);
        let behind = |at: f64| self.seams.partition_point(|seam| seam.s <= at);
        let window = first(reach.0, h_lo)..past(reach.0, h_lo).max(first(reach.0, h_lo));
        if first(reach.1, h_hi) != window.start
            || past(reach.1, h_hi).max(first(reach.1, h_hi)) != window.end
            || behind(reach.0).clamp(window.start, window.end)
                != behind(reach.1).clamp(window.start, window.end)
        {
            return None;
        }
        let behind = behind(reach.0).clamp(window.start, window.end);
        let slope_left = slope_left_of(window.start);
        if window.is_empty() && slope_left == 0.0 {
            return Some(CorrectionFormula::Zero);
        }
        Some(CorrectionFormula::Smoothed {
            interval: self.width.width_interval(model_reach)?,
            seams: (window.start, window.end),
            behind,
            slope_left,
        })
    }

    /// The correction's ingredients by `formula` as polynomials in the
    /// distance from `centre`; `None` for no correction.
    fn correction_polynomials(
        &self,
        formula: CorrectionFormula,
        centre: f64,
    ) -> Option<CorrectionPolynomials> {
        let CorrectionFormula::Smoothed {
            interval,
            seams: (first, past),
            behind: behind_end,
            slope_left,
        } = formula
        else {
            return None;
        };
        let window = first..past;
        let zero = Ball::exact(0.0);
        let mut step = [zero; SMOOTHED_RAMP.len()];
        let mut slope_step = [zero; SMOOTHED_HALF_SQUARE.len()];
        let mut kink = [zero; 3];
        let mut steps = false;
        for (index, seam) in self.seams[window.clone()].iter().enumerate() {
            let offset = Ball::rounded(centre - seam.s, 1);
            let (ratio_step, slope_change) = (Ball::exact(seam.step), Ball::exact(seam.slope_step));
            let mut power = Ball::exact(1.0);
            for k in 0..SMOOTHED_HALF_SQUARE.len() {
                if k < SMOOTHED_RAMP.len() && seam.step != 0.0 {
                    step[k] = step[k] + ratio_step * power;
                }
                if seam.slope_step != 0.0 {
                    slope_step[k] = slope_step[k] + slope_change * power;
                }
                power = power * offset;
            }
            steps |= seam.step != 0.0;
            if window.start + index < behind_end {
                let half_slope = slope_change * Ball::exact(0.5);
                kink[0] = kink[0] + ratio_step * offset + half_slope * offset * offset;
                kink[1] = kink[1] + ratio_step + slope_change * offset;
                kink[2] = kink[2] + half_slope;
            }
        }
        let weight = |index: usize| {
            let mut coefficients = [zero; 6];
            let mut moment = |sums: &[Ball], k: usize, weight: f64| {
                for (power, slot) in coefficients.iter_mut().enumerate().take(k + 1) {
                    let scale = weight * BINOMIALS[k][power];
                    *slot = *slot + Ball::exact(scale) * sums[k - power];
                }
            };
            moment(&slope_step, index, SMOOTHED_HALF_SQUARE[index]);
            if let Some(k) = index.checked_sub(1).filter(|_| steps) {
                moment(&step, k, SMOOTHED_RAMP[k]);
            }
            if index == 0 {
                coefficients[0] =
                    coefficients[0] + Ball::exact(KERNEL_HALF_VARIANCE) * Ball::exact(slope_left);
            }
            coefficients
        };
        Some(CorrectionPolynomials {
            width: self.width.width_polynomial(interval, centre),
            weights: std::array::from_fn(weight),
            kink: kink.map(|term| -term),
            seams: !window.is_empty(),
        })
    }

    /// `[W, W', W'']` of the advance's valley weight on the model `s` of
    /// the path, `reach` its range — `Some(None)` where no valley weighs
    /// it, `W = 1` —; `None` where the reach's ends lie on different sides
    /// of a valley.
    pub(in crate::follower_projection) fn valley_weight_model(
        &self,
        s: &TaylorModel,
        reach: (f64, f64),
    ) -> Option<Option<[TaylorModel; 3]>> {
        let mut total: Option<[TaylorModel; 3]> = None;
        let reach = read_points(reach);
        for valley in self.valleys_over(reach.0, reach.1) {
            let side = valley.side(reach.0);
            if valley.side(reach.1) != side {
                return None;
            }
            if side == ValleySide::Outside {
                continue;
            }
            let [v, v1, v2] = valley.weight_model(side, s);
            total = Some(match total {
                None => [v, v1, v2],
                Some([w, w1, w2]) => [
                    w.mul(&v),
                    w1.mul(&v).add(&w.mul(&v1)),
                    w2.mul(&v)
                        .add(&w1.mul(&v1).scale(Ball::exact(2.0)))
                        .add(&w.mul(&v2)),
                ],
            });
        }
        Some(total)
    }
}
