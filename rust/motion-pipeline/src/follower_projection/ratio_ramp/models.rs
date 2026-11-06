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

use crate::follower_projection::Table;
use crate::shaper::taylor::{self, Ball, TaylorModel};
use geometry::kept_floor;

use super::{
    AdvanceValley, ClusterPiece, ClusterWindow, FALL_TO_REST, KERNEL_HALF_VARIANCE, RISE_FROM_REST,
    RatioSmoothing, SMOOTHED_HALF_SQUARE, SMOOTHERSTEP, SPAN_LOOKUP_SLACK_MM, Seam, SeamCluster,
    SeamView, WidthField,
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

/// How many insets ([`REACH_INSET_ULPS`]) the width at a reach's end spans
/// at least for the correction's formula there to be read inset: a window
/// edge the inset skips then lies where the formulas on its two sides
/// agree to within `(1/16)²` of the step in the ratio, the kernel's
/// smoothing of a step being flat to second order at a window's edges.
const INSET_WIDTHS: f64 = 16.0;

fn reach_inset((lo, hi): (f64, f64)) -> f64 {
    REACH_INSET_ULPS * f64::EPSILON * lo.abs().max(hi.abs()).max(1.0)
}

/// The path distances `[lo, hi]` a window's odometer reads, and which of
/// its ends the odometer reads itself — the distance it starts from, or
/// the one it holds at — rather than reaching only as its model's bound.
#[derive(Clone, Copy, Debug)]
pub(in crate::follower_projection) struct PathReach {
    pub(in crate::follower_projection) lo: f64,
    pub(in crate::follower_projection) hi: f64,
    pub(in crate::follower_projection) exact: (bool, bool),
}

impl PathReach {
    /// The one distance `at`, read itself.
    fn point(at: f64) -> Self {
        Self {
            lo: at,
            hi: at,
            exact: (true, true),
        }
    }

    pub(in crate::follower_projection) fn span(self) -> (f64, f64) {
        (self.lo, self.hi)
    }

    /// The points the reach is read at for its formula: a bound end moved
    /// [`REACH_INSET_ULPS`] inside, an end the odometer reads itself kept,
    /// or both ends themselves where the reach is no wider than two insets:
    /// a reach that narrow holds only floats a bend may separate.
    fn read_points(self) -> (f64, f64) {
        let inset = reach_inset(self.span());
        if self.hi - self.lo <= 2.0 * inset {
            return (self.lo, self.hi);
        }
        let inset_of = |exact: bool| if exact { 0.0 } else { inset };
        (
            self.lo + inset_of(self.exact.0),
            self.hi - inset_of(self.exact.1),
        )
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
    edges: Option<EdgeSpans>,
}

/// The two spans of the table running past a window's edges with seams
/// inside ([`super::edge_spans`]): the slope behind the back edge and past
/// the front one, and the first and last seam's offset from the model's
/// centre with whether each lies behind its point.
struct EdgeSpans {
    slope_left: f64,
    slope_right: f64,
    first: (Ball, bool),
    last: (Ball, bool),
}

impl CorrectionPolynomials {
    /// The correction's Taylor series about `at` in the distance from the
    /// point; `None` where the width may reach zero there.
    fn series(&self, at: Lean) -> Option<Series> {
        if self.width[1..].iter().all(|c| c.mid == 0.0 && c.rad == 0.0) {
            return self.series_of_constant_width(at);
        }
        let h = series_about(&self.width, at);
        let weight = |index: usize| series_about(&self.weights[index], at);
        let h_squared = series_mul(&h, &h);
        let h_squared_weight = series_mul(&h_squared, &weight(0));
        let Some(edges) = &self.edges else {
            return Some(h_squared_weight);
        };
        let growing = series_add(&h_squared_weight, &series_mul(&h, &weight(1)));
        let inverse = series_reciprocal(&h)?;
        let edge = edges.series(&h, &h_squared, &inverse, at);
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
            &series_add(
                &series_add(&series_about(&self.kink, at), &growing),
                &series_add(&weight(2), &shrinking),
            ),
            &edge,
        ))
    }
}

impl CorrectionPolynomials {
    /// [`Self::series`] where the width holds one value over the reach, as
    /// a cluster's does: its powers scale the weights' series instead of
    /// multiplying series of them.
    fn series_of_constant_width(&self, at: Lean) -> Option<Series> {
        let h = Lean::of(self.width[0]);
        let scaled = |series: &Series, factor: Lean| -> Series { series.map(|term| term * factor) };
        let weight = |index: usize| series_about(&self.weights[index], at);
        let h_squared = h * h;
        let h_squared_weight = scaled(&weight(0), h_squared);
        let Some(edges) = &self.edges else {
            return Some(h_squared_weight);
        };
        let growing = series_add(&h_squared_weight, &scaled(&weight(1), h));
        let inverse = h.reciprocal()?;
        let constant = |value: Lean| {
            let mut series = [Lean::ZERO; SERIES_LEN];
            series[0] = value;
            series
        };
        let edge = edges.series(&constant(h), &constant(h_squared), &constant(inverse), at);
        let shrinking = scaled(
            &series_add(
                &scaled(
                    &series_add(&scaled(&weight(5), inverse), &weight(4)),
                    inverse,
                ),
                &weight(3),
            ),
            inverse,
        );
        Some(series_add(
            &series_add(
                &series_add(&series_about(&self.kink, at), &growing),
                &series_add(&weight(2), &shrinking),
            ),
            &edge,
        ))
    }
}

impl EdgeSpans {
    /// `h²·(m_left·back + m_right·front)` as the evaluator's
    /// [`super::edge_spans`] reads it, each edge's `G₂` about the edge it
    /// lies nearer: `h²·P₂(g/h) = 0.0625·g⁴/h² − 0.0125·g⁵/h³` at
    /// `g = h ∓ u`.
    fn series(&self, h: &Series, h_squared: &Series, inverse: &Series, at: Lean) -> Series {
        let inverse_squared = series_mul(inverse, inverse);
        let inverse_cubed = series_mul(&inverse_squared, inverse);
        let off_edge = |g: &Series| {
            let g_squared = series_mul(g, g);
            let g_fourth = series_mul(&g_squared, &g_squared);
            let g_fifth = series_mul(&g_fourth, g);
            series_add(
                &series_scale(&series_mul(&g_fourth, &inverse_squared), 0.0625),
                &series_scale(&series_mul(&g_fifth, &inverse_cubed), -0.0125),
            )
        };
        let complement = |part: &Series| {
            series_add(
                &series_scale(h_squared, KERNEL_HALF_VARIANCE),
                &series_scale(part, -1.0),
            )
        };
        let toward = |(offset, behind): (Ball, bool)| {
            let u = series_about(&[offset, Ball::exact(1.0)], at);
            let sign = if behind { -1.0 } else { 1.0 };
            (off_edge(&series_add(h, &series_scale(&u, sign))), behind)
        };
        let back = match toward(self.first) {
            (behind, true) => behind,
            (ahead, false) => complement(&ahead),
        };
        let front = match toward(self.last) {
            (behind, true) => complement(&behind),
            (ahead, false) => ahead,
        };
        series_add(
            &series_scale(&back, self.slope_left),
            &series_scale(&front, self.slope_right),
        )
    }
}

fn series_scale(a: &Series, factor: f64) -> Series {
    let factor = Lean::of(Ball::exact(factor));
    std::array::from_fn(|k| a[k] * factor)
}

/// The most floats a narrow reach is read at one by one.
const NARROW_FLOATS: usize = 64;

/// `reach` split off from each of its end floats in turn: where the
/// evaluator flips a formula on an end float itself, the float reads one
/// and the rest of the reach the other.
fn end_apart(reach: PathReach) -> [[PathReach; 2]; 2] {
    let PathReach { lo, hi, exact } = reach;
    [
        [
            PathReach::point(lo),
            PathReach {
                lo: trajectory::next_toward(lo, hi),
                hi,
                exact: (false, exact.1),
            },
        ],
        [
            PathReach {
                lo,
                hi: trajectory::next_toward(hi, lo),
                exact: (exact.0, false),
            },
            PathReach::point(hi),
        ],
    ]
}

/// Every float of `reach` where it is no wider than its read points inset
/// ([`PathReach::read_points`]) and holds at most [`NARROW_FLOATS`] of them: a reach
/// that narrow that reads more than one formula reads each at some of its
/// floats, and the evaluator reads no other.
pub(in crate::follower_projection) fn narrow_floats((lo, hi): (f64, f64)) -> Option<Vec<f64>> {
    let inset = reach_inset((lo, hi));
    if hi - lo > 2.0 * inset {
        return None;
    }
    let mut floats = vec![lo];
    while let Some(&last) = floats.last().filter(|&&last| last < hi) {
        if floats.len() == NARROW_FLOATS {
            return None;
        }
        floats.push(trajectory::next_toward(last, f64::INFINITY));
    }
    Some(floats)
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
    fn add(&self, other: &Self) -> Self {
        Self {
            value: self.value.add(&other.value),
            d1: self.d1.add(&other.d1),
            d2: self.d2.add(&other.d2),
            d3: self.d3.add(&other.d3),
        }
    }

    /// The jet widened, model by model, by how far `other` departs from it.
    fn widened_by(mut self, other: &Self) -> Self {
        for (model, other) in [
            (&mut self.value, &other.value),
            (&mut self.d1, &other.d1),
            (&mut self.d2, &other.d2),
            (&mut self.d3, &other.d3),
        ] {
            let departure = other.sub(model).bound();
            model.widen(departure);
        }
        self
    }

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
    fn width_interval(&self, reach: PathReach) -> Option<WidthInterval> {
        let (first, last) = (self.nodes.first()?.s, self.nodes.last()?.s);
        if reach.lo < first - SPAN_LOOKUP_SLACK_MM || reach.hi > last + SPAN_LOOKUP_SLACK_MM {
            return None;
        }
        let reach = reach.read_points();
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

/// The Taylor-model reading of a laid valley, beside its point weight
/// ([`AdvanceValley::weight`]).
trait ValleyModel {
    fn side(&self, s: f64) -> ValleySide;
    fn weight_model(&self, side: ValleySide, s: &TaylorModel) -> [TaylorModel; 3];
}

impl ValleyModel for AdvanceValley {
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

    /// `[w, w', w'']` on `side` at the model `s`, the distance `d` from
    /// the valley flattened by its crawl floor `f` as the evaluator reads
    /// it ([`AdvanceValley::weight`]): `d²/(√(d² + f²) + f) = √(d² + f²) − f`.
    /// Where the root's model reaches zero it is its range instead, held at
    /// `f` from below, which it never passes.
    fn weight_model(&self, side: ValleySide, s: &TaylorModel) -> [TaylorModel; 3] {
        let constant = |value: f64| TaylorModel::constant(Ball::exact(value));
        let (distance, width, floor, outward) = match side {
            ValleySide::Outside => return [constant(1.0), constant(0.0), constant(0.0)],
            ValleySide::Inside => return [constant(0.0), constant(0.0), constant(0.0)],
            ValleySide::Before => (
                s.scale(Ball::exact(-1.0)).add_ball(Ball::exact(self.start)),
                self.before,
                self.floor_before,
                -1.0,
            ),
            ValleySide::After => (
                s.add_ball(Ball::exact(-self.end)),
                self.after,
                self.floor_after,
                1.0,
            ),
        };
        let floor = kept_floor(floor, width);
        let (flattened, slope, bend) = if floor == 0.0 {
            (distance, constant(1.0), constant(0.0))
        } else {
            let squared = distance
                .mul(&distance)
                .add_ball(Ball::rounded(floor * floor, 1));
            let range = || {
                let (centre, radius) = squared.spread();
                let low = floor.max((centre - radius).max(0.0).sqrt());
                let high = taylor::upper(taylor::upper(centre + radius).sqrt()).max(low);
                TaylorModel::constant(Ball {
                    mid: 0.5 * low + 0.5 * high,
                    rad: taylor::upper(0.5 * (high - low)),
                })
            };
            let spread = match squared.sqrt() {
                Some(root) if root.reciprocal().is_some() => root,
                _ => range(),
            };
            let inverse = spread
                .reciprocal()
                .expect("a crawl floor keeps the spread off zero");
            let floor_over = inverse.scale(Ball::exact(floor));
            (
                spread.add_ball(Ball::exact(-floor)),
                distance.mul(&inverse),
                floor_over.mul(&floor_over).mul(&inverse),
            )
        };
        let span = if floor == 0.0 {
            width
        } else {
            width * width / (libm::hypot(width, floor) + floor)
        };
        let per_span = Ball::rounded(1.0 / span, 1);
        let inside = flattened
            .scale(per_span)
            .scale(Ball::exact(-1.0))
            .add_ball(Ball::exact(1.0));
        let inside_squared = inside.mul(&inside);
        [
            inside_squared
                .mul(&inside)
                .scale(Ball::exact(-1.0))
                .add_ball(Ball::exact(1.0)),
            inside_squared
                .mul(&slope)
                .scale(per_span * Ball::exact(3.0 * outward)),
            inside
                .mul(&slope)
                .mul(&slope)
                .scale(per_span * per_span * Ball::exact(-6.0))
                .add(&inside_squared.mul(&bend).scale(per_span * Ball::exact(3.0))),
        ]
    }
}

impl RatioSmoothing {
    /// The correction `X` of `table` and its first three path derivatives
    /// on the model `s` of the path, `reach` its range; `None` where the
    /// reach's ends read different formulas, or the width may reach zero.
    /// Each derivative is its Taylor polynomial about the model's centre,
    /// to [`SERIES_DEGREE`] in the distance `δ` from it, and the Lagrange
    /// remainder past it, bounded by the series read with `δ` the whole
    /// reach of the model: an interval Taylor series.
    pub(in crate::follower_projection) fn correction_model(
        &self,
        table: Table,
        s: &TaylorModel,
        reach: PathReach,
    ) -> Option<ModelJet> {
        let table = self.seam_table(table);
        let (lo, hi) = reach.span();
        let widest = if self.width.nodes.is_empty() {
            0.0
        } else {
            self.width.bounds(lo, hi).0
        };
        let moving = table.moving_view(&self.width, (lo - widest, hi + widest));
        table
            .clusters_reaching(&self.width, (lo, hi))
            .fold(self.slope_model(&moving, s, reach)?, |total, cluster| {
                total.add(&cluster_model(&cluster, s, reach))
            })
            .into()
    }

    /// The moving window's share of the correction ([`super::slope_correction`])
    /// of the table `seams` holds about the reach on the model `s` of the
    /// path.
    fn slope_model(&self, seams: &SeamView, s: &TaylorModel, reach: PathReach) -> Option<ModelJet> {
        if let Some(formula) = self.correction_formula(seams, reach) {
            return self.correction_by(seams, formula, s);
        }
        let formulas = match self.formulas_across_rest(seams, reach) {
            Some(formulas) => formulas,
            None => self.narrow_formulas(seams, reach)?,
        };
        let (first, others) = formulas.split_first()?;
        let mut base = self.correction_by(seams, *first, s)?;
        for &other in others {
            base = base.widened_by(&self.correction_by(seams, other, s)?);
        }
        Some(base)
    }

    /// Every formula a reach too narrow to place its bends in reads, one per
    /// float.
    fn narrow_formulas(
        &self,
        seams: &SeamView,
        reach: PathReach,
    ) -> Option<Vec<CorrectionFormula>> {
        let mut formulas: Vec<CorrectionFormula> = Vec::new();
        for at in narrow_floats(reach.span())? {
            let formula = self.correction_formula(seams, PathReach::point(at))?;
            if !formulas.contains(&formula) {
                formulas.push(formula);
            }
        }
        Some(formulas)
    }

    /// The slope's correction jet on the model `s` of the path by
    /// `formula`.
    fn correction_by(
        &self,
        seams: &SeamView,
        formula: CorrectionFormula,
        s: &TaylorModel,
    ) -> Option<ModelJet> {
        let (centre, radius) = s.spread();
        let Some(polynomials) = self.correction_polynomials(seams, formula, centre) else {
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
    fn correction_formula(&self, seams: &SeamView, reach: PathReach) -> Option<CorrectionFormula> {
        if seams.is_flat() {
            return Some(CorrectionFormula::Zero);
        }
        let read = self.correction_read_points(reach);
        let widths = (self.width.jet(read.0).value, self.width.jet(read.1).value);
        if widths.0 <= 0.0 && widths.1 <= 0.0 {
            return Some(CorrectionFormula::Zero);
        }
        if widths.0 <= 0.0 || widths.1 <= 0.0 {
            return None;
        }
        self.smoothed_formula(seams, read, widths, || self.width.width_interval(reach))
    }

    /// [`PathReach::read_points`] for the correction's formula, a bound end
    /// kept inset only where the width there spans [`INSET_WIDTHS`] insets:
    /// a seam an ulp past the rest node its leaders stand at opens a window
    /// only a few ulps wide, all of it within the inset, which the signal
    /// reads where the leaders stand on the seam.
    fn correction_read_points(&self, reach: PathReach) -> (f64, f64) {
        let (lo, hi) = reach.read_points();
        let inset = reach_inset(reach.span());
        let wide = |s: f64| self.width.jet(s).value >= INSET_WIDTHS * inset;
        (
            if wide(reach.lo) { lo } else { reach.lo },
            if wide(reach.hi) { hi } else { reach.hi },
        )
    }

    /// The formulas of a reach whose width rises from rest, or falls to
    /// it, at one end: the rest reads none, the rest of the reach one
    /// smoothed formula, and the window reads no seams there, or it would
    /// divide by the vanishing width; `None` for any other reach. A reach
    /// whose windows reach no seam reads none throughout, whichever width
    /// nodes it crosses — the rest's own, stacked where the width leaves
    /// zero, among them: only a smoothed formula reads the width's
    /// interval.
    fn formulas_across_rest(
        &self,
        seams: &SeamView,
        reach: PathReach,
    ) -> Option<Vec<CorrectionFormula>> {
        if seams.is_flat() {
            return None;
        }
        let read = self.correction_read_points(reach);
        let widths = (self.width.jet(read.0).value, self.width.jet(read.1).value);
        if (widths.0 <= 0.0) == (widths.1 <= 0.0) {
            return None;
        }
        let moving_end = if widths.0 <= 0.0 { read.1 } else { read.0 };
        let interval = self.width.formula_at(moving_end);
        let Err((a, b)) = interval else {
            return None;
        };
        let within_interval = self.width.nodes[a].s <= read.0 && read.1 <= self.width.nodes[b].s;
        let moving =
            self.smoothed_formula(seams, read, (widths.0.max(0.0), widths.1.max(0.0)), || {
                within_interval.then_some(interval)
            })?;
        match moving {
            CorrectionFormula::Zero => Some(vec![moving]),
            CorrectionFormula::Smoothed {
                seams: (first, past),
                ..
            } => (first == past).then(|| vec![CorrectionFormula::Zero, moving]),
        }
    }

    /// The smoothed formula every point of a reach reads, its ends read at
    /// `reach` with widths `(h_lo, h_hi)` on the width's `interval`; `None`
    /// where its ends' windows differ.
    fn smoothed_formula(
        &self,
        table: &SeamView,
        reach: (f64, f64),
        (h_lo, h_hi): (f64, f64),
        interval: impl FnOnce() -> Option<WidthInterval>,
    ) -> Option<CorrectionFormula> {
        let slope_left_of = |first: usize| {
            first
                .checked_sub(1)
                .map_or(table.slope_before, |last| table.seams[last].slope_after)
        };
        let first = |at: f64, h: f64| table.seams.partition_point(|seam| seam.s <= at - h);
        let past = |at: f64, h: f64| table.seams.partition_point(|seam| seam.s < at + h);
        let behind = |at: f64| table.seams.partition_point(|seam| seam.s <= at);
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
            interval: interval()?,
            seams: (window.start, window.end),
            behind,
            slope_left,
        })
    }

    /// The correction's ingredients by `formula` as polynomials in the
    /// distance from `centre`; `None` for no correction.
    fn correction_polynomials(
        &self,
        table: &SeamView,
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
        let seams = &table.seams[first..past];
        let is_behind = |index: usize| first + index < behind_end;
        let offset_of = |seam: &Seam| Ball::rounded(centre - seam.s, 1);
        let zero = Ball::exact(0.0);
        let mut slope = [zero; SMOOTHED_HALF_SQUARE.len()];
        let mut kink = [zero; 3];
        for (index, pair) in seams.windows(2).enumerate() {
            let (seam, next) = (&pair[0], &pair[1]);
            if seam.slope_after == 0.0 {
                continue;
            }
            let (ahead, behind) = (offset_of(seam), offset_of(next));
            let span_slope = Ball::exact(seam.slope_after);
            let rise = span_slope * Ball::rounded(next.s - seam.s, 1);
            let (mut divided, mut power) = (zero, Ball::exact(1.0));
            for moment in &mut slope[1..] {
                divided = divided * ahead + power;
                power = power * behind;
                *moment = *moment + rise * divided;
            }
            let half = Ball::exact(0.5);
            if is_behind(index + 1) {
                kink[0] = kink[0] + rise * half * (ahead + behind);
                kink[1] = kink[1] + rise;
            } else if is_behind(index) {
                kink[0] = kink[0] + span_slope * half * ahead * ahead;
                kink[1] = kink[1] + span_slope * ahead;
                kink[2] = kink[2] + span_slope * half;
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
            moment(&slope, index, SMOOTHED_HALF_SQUARE[index]);
            if index == 0 && seams.is_empty() {
                coefficients[0] =
                    coefficients[0] + Ball::exact(KERNEL_HALF_VARIANCE) * Ball::exact(slope_left);
            }
            coefficients
        };
        let edges = seams
            .first()
            .zip(seams.last())
            .map(|(first_seam, last_seam)| EdgeSpans {
                slope_left,
                slope_right: last_seam.slope_after,
                first: (offset_of(first_seam), is_behind(0)),
                last: (offset_of(last_seam), is_behind(seams.len() - 1)),
            });
        Some(CorrectionPolynomials {
            width: self.width.width_polynomial(interval, centre),
            weights: std::array::from_fn(weight),
            kink: kink.map(|term| -term),
            edges,
        })
    }

    /// `[W, W', W'']` of the advance's valley weight on the model `s` of
    /// the path, `reach` its range; `None` where the reach's ends lie on
    /// different sides of a valley.
    pub(in crate::follower_projection) fn valley_weight_model(
        &self,
        s: &TaylorModel,
        reach: PathReach,
    ) -> Option<ValleyWeight> {
        if let Some(weight) = self.valley_weight_on(s, reach) {
            return Some(weight);
        }
        let parts = match end_apart(reach).into_iter().find(|parts| {
            parts
                .iter()
                .all(|&part| self.valley_weight_on(s, part).is_some())
        }) {
            Some(parts) => parts.to_vec(),
            None => narrow_floats(reach.span())?
                .into_iter()
                .map(PathReach::point)
                .collect(),
        };
        let mut base: Option<[TaylorModel; 3]> = None;
        for part in parts {
            let weight = self.valley_weight_on(s, part)?.models();
            match &mut base {
                None => base = Some(weight),
                Some(base) => {
                    for (model, other) in base.iter_mut().zip(&weight) {
                        let departure = other.sub(model).bound();
                        model.widen(departure);
                    }
                }
            }
        }
        Some(base.map_or(ValleyWeight::One, ValleyWeight::Model))
    }

    /// [`Self::valley_weight_model`] where every point of `reach` lies on
    /// one side of every valley; `None` where it does not.
    fn valley_weight_on(&self, s: &TaylorModel, reach: PathReach) -> Option<ValleyWeight> {
        let mut total: Option<[TaylorModel; 3]> = None;
        let reach = reach.read_points();
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
        Some(total.map_or(ValleyWeight::One, ValleyWeight::Model))
    }
}

/// The advance's valley weight over a reach: one where no valley weighs
/// it, else `[W, W', W'']`.
pub(in crate::follower_projection) enum ValleyWeight {
    One,
    Model([TaylorModel; 3]),
}

impl ValleyWeight {
    pub(in crate::follower_projection) fn models(self) -> [TaylorModel; 3] {
        match self {
            ValleyWeight::One => {
                [1.0, 0.0, 0.0].map(|value| TaylorModel::constant(Ball::exact(value)))
            }
            ValleyWeight::Model(models) => models,
        }
    }
}

/// One cluster's stationary share of the correction on the model `s` of
/// the path, `reach` its range: the polynomial of every piece the reach
/// reads ([`super::cluster_pieces`]), and nothing where it leaves the
/// cluster's support, the first widened by how far the others depart from
/// it.
fn cluster_model(cluster: &SeamCluster, s: &TaylorModel, reach: PathReach) -> ModelJet {
    let (lo, hi) = reach.span();
    let (support_from, support_to) = cluster.support();
    let first = cluster.pieces.partition_point(|piece| piece.to <= lo);
    let past = cluster
        .pieces
        .partition_point(|piece| piece.from <= hi)
        .max(first);
    let outside = (lo < support_from || hi >= support_to).then(ModelJet::zero);
    cluster.pieces[first..past]
        .iter()
        .map(|piece| piece_model(piece, s))
        .chain(outside)
        .reduce(|hull, model| hull.widened_by(&model))
        .unwrap_or_else(ModelJet::zero)
}

/// A cluster piece's polynomial in `s − from` and its first three path
/// derivatives on the model `s` of the path.
fn piece_model(piece: &ClusterPiece, s: &TaylorModel) -> ModelJet {
    let y = s.add_ball(Ball::exact(-piece.from));
    let derivative = |order: usize| {
        let coefficients = piece.coefficients[order..]
            .iter()
            .zip(&piece.roundings[order..])
            .enumerate()
            .map(|(power, (&mid, &rad))| {
                let falling = ((power + 1)..=(power + order)).product::<usize>() as f64;
                Ball { mid, rad } * Ball::exact(falling)
            });
        taylor::polynomial_of_balls(coefficients, &y)
    };
    ModelJet {
        value: derivative(0),
        d1: derivative(1),
        d2: derivative(2),
        d3: derivative(3),
    }
}

/// The slope's share of a cluster's stationary correction by `window` with
/// half-width `width`, its slopes less the one it leaves on, as the
/// coefficients of its polynomial in the distance from `centre`, each
/// within its bound on the reach `radius` past it: the moments' series
/// read at the centre, the window's width one value throughout, the terms
/// past the polynomial's degree folded into the constant's bound.
pub(super) fn cluster_slope_polynomial(
    seams: &[Seam],
    width: f64,
    window: ClusterWindow,
    centre: f64,
    radius: f64,
) -> [Ball; SMOOTHED_HALF_SQUARE.len()] {
    let zero = [Ball::exact(0.0); SMOOTHED_HALF_SQUARE.len()];
    let Some(polynomials) = slope_polynomials(seams, width, window, centre) else {
        return zero;
    };
    let series = polynomials
        .series_of_constant_width(Lean {
            mid: 0.0,
            at: 0.0,
            over: 0.0,
        })
        .expect("a cluster's width is positive")
        .map(|term| term.sealed()[0]);
    let mut coefficients = zero;
    coefficients.copy_from_slice(&series[..SMOOTHED_HALF_SQUARE.len()]);
    let past_degree = series[SMOOTHED_HALF_SQUARE.len()..]
        .iter()
        .enumerate()
        .map(|(offset, term)| {
            (term.mid.abs() + term.rad) * radius.powi((SMOOTHED_HALF_SQUARE.len() + offset) as i32)
        })
        .sum::<f64>();
    coefficients[0].rad = taylor::upper(coefficients[0].rad + past_degree);
    coefficients
}

/// A cluster's stationary slope correction's ingredients by `window` as
/// polynomials in the distance from `centre`, its slopes less the one it
/// leaves on; `None` where every such slope the window reads is zero, as
/// across a lone seam.
fn slope_polynomials(
    all: &[Seam],
    half_width: f64,
    window: ClusterWindow,
    centre: f64,
) -> Option<CorrectionPolynomials> {
    let out = all[all.len() - 1].slope_after;
    let seams = &all[window.first..window.past];
    let slope_left = window
        .first
        .checked_sub(1)
        .map_or(0.0, |last| all[last].slope_after - out);
    if slope_left == 0.0 && seams.iter().all(|seam| seam.slope_after == out) {
        return None;
    }
    let is_behind = |index: usize| window.first + index < window.behind;
    let offset_of = |seam: &Seam| Ball::rounded(centre - seam.s, 1);
    let zero = Ball::exact(0.0);
    let mut slope = [zero; SMOOTHED_HALF_SQUARE.len()];
    let mut kink = [zero; 3];
    for (index, pair) in seams.windows(2).enumerate() {
        let (seam, next) = (&pair[0], &pair[1]);
        let residual = seam.slope_after - out;
        if residual == 0.0 {
            continue;
        }
        let (ahead, behind) = (offset_of(seam), offset_of(next));
        let span_slope = Ball::exact(residual);
        let rise = span_slope * Ball::rounded(next.s - seam.s, 1);
        let (mut divided, mut power) = (zero, Ball::exact(1.0));
        for moment in &mut slope[1..] {
            divided = divided * ahead + power;
            power = power * behind;
            *moment = *moment + rise * divided;
        }
        let half = Ball::exact(0.5);
        if is_behind(index + 1) {
            kink[0] = kink[0] + rise * half * (ahead + behind);
            kink[1] = kink[1] + rise;
        } else if is_behind(index) {
            kink[0] = kink[0] + span_slope * half * ahead * ahead;
            kink[1] = kink[1] + span_slope * ahead;
            kink[2] = kink[2] + span_slope * half;
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
        moment(&slope, index, SMOOTHED_HALF_SQUARE[index]);
        if index == 0 && seams.is_empty() {
            coefficients[0] =
                coefficients[0] + Ball::exact(KERNEL_HALF_VARIANCE) * Ball::exact(slope_left);
        }
        coefficients
    };
    let edges = seams
        .first()
        .zip(seams.last())
        .map(|(first_seam, last_seam)| EdgeSpans {
            slope_left,
            slope_right: last_seam.slope_after - out,
            first: (offset_of(first_seam), is_behind(0)),
            last: (offset_of(last_seam), is_behind(seams.len() - 1)),
        });
    let mut width = [zero; 6];
    width[0] = Ball::exact(half_width);
    Some(CorrectionPolynomials {
        width,
        weights: std::array::from_fn(weight),
        kink: kink.map(|term| -term),
        edges,
    })
}
