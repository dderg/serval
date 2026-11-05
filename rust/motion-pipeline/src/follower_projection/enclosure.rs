//! Taylor-model enclosures of a follower's projected track: the shaped
//! leaders' polynomial pieces give the path speed `|ẋ|` and its rate, the
//! odometer `s` is its integral anchored at the window centre's own
//! reading, and the span table and its smoothing lay `e(s)`, `r(s)` and
//! `r'(s)` there. The follower reads `p = e(s)`, `v = r·ṡ`,
//! `a = r'·ṡ² + r·s̈`; an advance reads `R = r·W`, `W` the valley weight,
//! and its time derivatives.
//!
//! A window whose odometer range reaches into a neighbouring span — or
//! before the table or past its last span — is modelled by the formula
//! its centre lies in, widened by how far every other formula it reaches
//! strays from it there: the signal is one of them at every point.
//!
//! The odometer anchored at the centre is the exact integral of the
//! speed from there; the evaluator's adaptive quadrature differs from it by
//! its own tolerance, which, like the evaluator's rounding, lies outside
//! the bound.

use crate::shaper::axis_enclosure::{AxisWindows, FORMULA_EDGE_SLACK};
use crate::shaper::taylor::{self, Ball, TaylorModel, Window};
use crate::shaper::{Enclosure, ExactPiece};

use super::ratio_ramp::{PathReach, REACH_INSET_ULPS, ValleyWeight};
use super::{
    AdvancedFollowerSignal, ChainStage, FollowerSignal, LaidRead, RatioSpan, SpanSide, Table,
};

/// What the table and its smoothing lay over a window's reach: `(e, r,
/// r', r'')` as models on the window, and how far other formulas the reach
/// may read depart from them in `(e, r, r')`.
struct LaidModels {
    e: TaylorModel,
    ratio: TaylorModel,
    slope: TaylorModel,
    curvature: TaylorModel,
    departures: Vec<[f64; 3]>,
}

impl LaidModels {
    /// The velocity `r·ṡ` and acceleration `r'·ṡ² + r·s̈` the table lays
    /// along the path at the rate `rate`, widened by how far the other
    /// formulas depart.
    fn along_path(
        &self,
        rate: &TaylorModel,
        rate_squared: &TaylorModel,
        rate_deriv: &TaylorModel,
    ) -> [TaylorModel; 2] {
        let mut along = [
            self.ratio.mul(rate),
            self.slope
                .mul(rate_squared)
                .add(&self.ratio.mul(rate_deriv)),
        ];
        let (rate_bound, rate_deriv_bound) = (rate.bound(), rate_deriv.bound());
        for &[_, ratio, slope] in &self.departures {
            along[0].widen(ratio * rate_bound);
            along[1].widen(slope * rate_bound * rate_bound + ratio * rate_deriv_bound);
        }
        along
    }
}

/// How a stretch of the span table lays extrusion against path distance.
#[derive(Clone, Copy)]
enum TableFormula {
    /// Inside a span: `e = e₀ + r₀·d + ½k·d²`, `d = s − s₀`.
    Span(RatioSpan),
    /// Before the table: the first span's start, at its start ratio.
    Before(RatioSpan),
    /// Past the table: the last span's end extended at its end ratio.
    Past(RatioSpan),
    /// No table laid yet: no extrusion, and no smoothing of it.
    Empty,
}

impl TableFormula {
    /// `(e, r, r')` of the table at the model `s`.
    fn laid(&self, s: &TaylorModel) -> [TaylorModel; 3] {
        match *self {
            TableFormula::Span(span) => {
                let slope = Ball::rounded(span.ratio_slope(), 2);
                let d = s.add_ball(Ball::exact(-span.s0));
                let ratio = d.scale(slope).add_ball(Ball::exact(span.r0));
                let e = d
                    .mul(&d)
                    .scale(slope * Ball::exact(0.5))
                    .add(&d.scale(Ball::exact(span.r0)))
                    .add_ball(Ball::exact(span.e0));
                [e, ratio, TaylorModel::constant(slope)]
            }
            TableFormula::Before(first) => [
                TaylorModel::constant(Ball::exact(first.e0)),
                TaylorModel::constant(Ball::exact(first.r0)),
                TaylorModel::constant(Ball::exact(0.0)),
            ],
            TableFormula::Past(last) => {
                let end = Ball::rounded(last.e_at(last.s1), 4);
                [
                    s.add_ball(Ball::exact(-last.s1))
                        .scale(Ball::exact(last.r1))
                        .add_ball(end),
                    TaylorModel::constant(Ball::exact(last.r1)),
                    TaylorModel::constant(Ball::exact(0.0)),
                ]
            }
            TableFormula::Empty => [(); 3].map(|()| TaylorModel::constant(Ball::exact(0.0))),
        }
    }

    /// The path distances the formula lays.
    fn domain(&self) -> (f64, f64) {
        match *self {
            TableFormula::Span(span) => (span.s0, span.s1),
            TableFormula::Before(first) => (f64::NEG_INFINITY, first.s0),
            TableFormula::Past(last) => (last.s1, f64::INFINITY),
            TableFormula::Empty => (f64::NEG_INFINITY, f64::INFINITY),
        }
    }

    /// How far this formula's `(e, r, r')` stray from `base`'s over the
    /// part of `reach` this formula lays; `None` where it lays none.
    fn departure_from(&self, base: &TableFormula, reach: (f64, f64)) -> Option<[f64; 3]> {
        let (from, to) = self.domain();
        let (lo, hi) = (reach.0.max(from), reach.1.min(to));
        if lo > hi {
            return None;
        }
        let s = TaylorModel::affine(
            Ball::rounded(0.5 * lo + 0.5 * hi, 1),
            taylor::upper(0.5 * (hi - lo)),
        );
        let (own, other) = (self.laid(&s), base.laid(&s));
        Some(std::array::from_fn(|order| {
            own[order].sub(&other[order]).bound()
        }))
    }
}

/// `d²|ẋ|/dt²` of leaders `[ẋᵢ, ẍᵢ, x⃛ᵢ]` at path speed `1/inverse`:
/// `Σ_{i<j}(ẋᵢẍⱼ − ẋⱼẍᵢ)²/|ẋ|³ + ẋ·x⃛/|ẋ|`, as the signal reads it — the
/// turning term from the cross products directly, which a straight path
/// leaves small, not as the difference `|ẍ|² − (ẋ·ẍ/|ẋ|)²` of two large
/// models.
fn leader_speed_second(leaders: &[[TaylorModel; 3]], inverse: &TaylorModel) -> TaylorModel {
    let mut cross_squared = TaylorModel::constant(Ball::exact(0.0));
    let mut velocity_dot_jerk = TaylorModel::constant(Ball::exact(0.0));
    for (index, [v_i, a_i, j_i]) in leaders.iter().enumerate() {
        velocity_dot_jerk = velocity_dot_jerk.add(&v_i.mul(j_i));
        for [v_j, a_j, _] in &leaders[index + 1..] {
            let cross = v_i.mul(a_j).sub(&v_j.mul(a_i));
            cross_squared = cross_squared.add(&cross.mul(&cross));
        }
    }
    let inverse_squared = inverse.mul(inverse);
    cross_squared
        .mul(&inverse_squared)
        .add(&velocity_dot_jerk)
        .mul(inverse)
}

/// A signal's enclosure over its whole support, built once at the
/// accuracy first asked and refined where asked tighter: a piece missing
/// the accuracy a stretch asks tighter than it was enclosed at is enclosed
/// again over its own span at that accuracy, halving afresh from there,
/// and replaced. A piece that still misses it — a window the halving depth
/// stopped on — is enclosed afresh over its overlap with each stretch
/// smaller than it.
#[derive(Default)]
pub(super) struct WholeEnclosure {
    built: std::cell::OnceCell<Option<std::cell::RefCell<Built>>>,
}

/// The pieces built so far, each beside the accuracy it was built at.
struct Built {
    pieces: Vec<ExactPiece>,
    tried: Vec<[f64; 3]>,
}

impl Built {
    fn new(pieces: Vec<ExactPiece>, accuracy: [f64; 3]) -> Self {
        let tried = vec![accuracy; pieces.len()];
        Self { pieces, tried }
    }

    /// The pieces reaching into `[lo, hi]`.
    fn span(&self, (lo, hi): (f64, f64)) -> std::ops::Range<usize> {
        let first = self.pieces.partition_point(|piece| piece.hi < lo);
        let past = self.pieces.partition_point(|piece| piece.lo <= hi);
        first..past.max(first)
    }
}

/// Whether every order `piece` bounds is within `accuracy`.
fn resolves(piece: &ExactPiece, accuracy: [f64; 3]) -> bool {
    let accuracy = piece
        .unshaped_step
        .map_or(accuracy, |step| step.enclosure_accuracy(accuracy));
    piece
        .pva
        .iter()
        .zip(accuracy)
        .enumerate()
        .all(|(order, (model, accuracy))| !piece.bounds(order) || model.rounding <= accuracy)
}

impl WholeEnclosure {
    /// The enclosure over `[lo, hi]` at `accuracy` or as close as the
    /// signal's models get, `build` enclosing a stretch at a given accuracy
    /// and `whole` the support; `None` where the signal has none. Borrowed
    /// from the whole where its pieces there resolve the accuracy.
    pub(super) fn over(
        &self,
        (lo, hi): (f64, f64),
        accuracy: [f64; 3],
        whole: (f64, f64),
        build: impl Fn((f64, f64), [f64; 3]) -> Option<Vec<ExactPiece>>,
    ) -> Option<Enclosure<'_>> {
        let kept = self
            .built
            .get_or_init(|| {
                build(whole, accuracy)
                    .map(|pieces| std::cell::RefCell::new(Built::new(pieces, accuracy)))
            })
            .as_ref()?;
        {
            let mut built = kept.borrow_mut();
            let span = built.span((lo, hi));
            let (mut index, mut end) = (span.start, span.end);
            while index < end {
                let tried_tighter = built.tried[index]
                    .iter()
                    .zip(accuracy)
                    .all(|(&tried, asked)| tried <= asked);
                if tried_tighter || resolves(&built.pieces[index], accuracy) {
                    index += 1;
                    continue;
                }
                let piece = built.pieces[index];
                let refined = build((piece.lo, piece.hi), accuracy)?;
                let added = refined.len();
                built.pieces.splice(index..=index, refined);
                built
                    .tried
                    .splice(index..=index, std::iter::repeat_n(accuracy, added));
                index += added;
                end += added - 1;
            }
        }
        let built = kept.borrow();
        let span = built.span((lo, hi));
        let pieces = &built.pieces;
        let whole_piece = |piece: &ExactPiece| piece.lo >= lo && piece.hi <= hi;
        if pieces[span.clone()]
            .iter()
            .all(|piece| resolves(piece, accuracy) || whole_piece(piece))
        {
            return Some(Enclosure::Held(std::cell::Ref::map(built, |built| {
                &built.pieces[span]
            })));
        }
        let mut out = Vec::with_capacity(span.len());
        for piece in &pieces[span] {
            if resolves(piece, accuracy) || whole_piece(piece) {
                out.push(*piece);
            } else {
                out.extend(build((lo.max(piece.lo), hi.min(piece.hi)), accuracy)?);
            }
        }
        Some(Enclosure::Built(out))
    }
}

/// The share of each model's spread a follower window works to: its
/// enclosure's accuracy is a far coarser share of every model's spread
/// than the rounding the models would otherwise keep terms for.
const FOLLOWER_WORKING_SHARE: f64 = 1.0 / (1_u64 << 24) as f64;

/// A follower's models on one window: what the span table lays along the
/// leaders' path and the raw track's extrude-only motion, each `[p, v, a]`,
/// and what an advance reads of the path's: the velocity laid along it
/// weighted by its valleys and that velocity's first two time derivatives
/// ([`FollowerSignal::ramped_advance_read`]), `[v, a, j]`.
pub(super) struct FollowerModels {
    path: [TaylorModel; 3],
    raw: [TaylorModel; 3],
    read: [TaylorModel; 3],
}

impl FollowerModels {
    /// The follower's own model of derivative `order`.
    fn total(&self, order: usize) -> TaylorModel {
        self.path[order].add(&self.raw[order])
    }
}

/// What a window's models read beyond the follower's own `[p, v, a]`.
#[derive(Clone, Copy, PartialEq)]
pub(super) enum Reading {
    /// The follower alone.
    Follower,
    /// The advance's read velocity and acceleration, its jerk unbounded.
    AdvanceWithoutJerk,
    /// The advance's read velocity, acceleration and jerk.
    Advance,
}

/// The axes a follower reads, window by window: the moving leaders, each
/// beside its index among the follower's leaders, and the raw track.
pub(super) struct FollowerAxisWindows<'a> {
    pub(super) leaders: Vec<(usize, AxisWindows<'a>)>,
    raw: Option<AxisWindows<'a>>,
}

impl<'a> FollowerSignal<'a> {
    /// `table`'s formulas over path distances `[lo, hi]`, the one the
    /// signal reads at `centre` first — at a span boundary, on the side
    /// [`Self::side_at`] reads it.
    fn table_formulas(&self, table: Table, centre: f64, (lo, hi): (f64, f64)) -> Vec<TableFormula> {
        let spans = self.state.spans_of(table);
        let (Some(&first), Some(&last)) = (spans.first(), spans.last()) else {
            return vec![TableFormula::Empty];
        };
        let region_of = |s: f64| {
            if s < first.s0 {
                TableFormula::Before(first)
            } else if s > last.s1 {
                TableFormula::Past(last)
            } else {
                let index = match self.side_at(s) {
                    SpanSide::After => spans.partition_point(|span| span.s1 <= s),
                    SpanSide::Before => spans.partition_point(|span| span.s1 < s),
                };
                TableFormula::Span(spans[index.min(spans.len() - 1)])
            }
        };
        let mut formulas = vec![region_of(centre)];
        if lo < first.s0 {
            formulas.push(TableFormula::Before(first));
        }
        if hi > last.s1 {
            formulas.push(TableFormula::Past(last));
        }
        let from = match self.side_at(lo) {
            SpanSide::After => spans.partition_point(|span| span.s1 <= lo),
            SpanSide::Before => spans.partition_point(|span| span.s1 < lo),
        };
        let read_at_hi = |span: &&RatioSpan| match self.side_at(hi) {
            SpanSide::After => span.s0 <= hi,
            SpanSide::Before => span.s0 < hi,
        };
        formulas.extend(
            spans[from..]
                .iter()
                .take_while(read_at_hi)
                .map(|&span| TableFormula::Span(span)),
        );
        formulas
    }

    /// What `table` and its smoothing lay over `reach` as models on `s`,
    /// `centre` the middle of the model's range; `None` where the
    /// correction has no model.
    fn table_models(
        &self,
        table: Table,
        s: &TaylorModel,
        centre: f64,
        reach: PathReach,
    ) -> Option<LaidModels> {
        let formulas = self.table_formulas(table, centre, reach.span());
        match self.laid_on_floats(table, &formulas, reach.span()) {
            Some(laid) => Some(laid),
            None => self.laid_models(table, s, &formulas, reach),
        }
    }

    /// The follower's models on `window` and what `reading` asks beyond
    /// them, split into what the span table and its smoothing lay along the
    /// leaders' path and the extrude-only motion of the raw track; `None`
    /// where a model is out of reach: an axis past one of its formulas or
    /// without a jerk, an odometer reach across a bend of the smoothing, or
    /// a speed zero the rate divides by. An odometer reaching past the owned
    /// span, where the signal holds its distance, is widened by how far it
    /// reaches.
    fn window_models(&self, window: Window, reading: Reading) -> Option<FollowerModels> {
        let with_jerk = reading == Reading::Advance;
        let windows = self.axis_windows()?;
        let mut leaders = Vec::with_capacity(windows.leaders.len());
        let mut speed_squared = TaylorModel::constant(Ball::exact(0.0));
        let mut power = TaylorModel::constant(Ball::exact(0.0));
        for (_, axis) in &windows.leaders {
            let [velocity, acceleration, jerk] = axis
                .motion_models(window, with_jerk)?
                .map(|model| model.working_to(FOLLOWER_WORKING_SHARE));
            speed_squared = speed_squared.add(&velocity.mul(&velocity));
            power = power.add(&velocity.mul(&acceleration));
            leaders.push([velocity, acceleration, jerk]);
        }
        let rest = match self.rest_at(window.center) {
            Some(rest) if rest.from <= window.lo && window.hi <= rest.to => Some(*rest),
            Some(_) => return None,
            None => None,
        };
        let sliver = self
            .speed_zero_sliver(window.center)
            .filter(|sliver| sliver.holds_window(window.lo, window.hi));
        let (speed, inverse_speed) = match (sliver, rest) {
            (None, None) => {
                let speed = taylor::root_or_range(&speed_squared);
                let inverse = speed.reciprocal()?;
                (speed, Some(inverse))
            }
            _ => (taylor::root_or_range(&speed_squared), None),
        };
        let scale = Ball::exact(self.odometer_scale);
        let rate = speed.scale(scale);
        let along = inverse_speed.as_ref().map(|inverse| power.mul(inverse));
        let along_rest = |order: usize| {
            rest.map(|rest| {
                let direction = rest.direction(window.center);
                windows.leaders.iter().zip(&leaders).fold(
                    TaylorModel::constant(Ball::exact(0.0)),
                    |sum, ((index, _), models)| {
                        sum.add(&models[order].scale(Ball::exact(direction[*index])))
                    },
                )
            })
        };
        let rate_deriv = match (sliver, &along) {
            (_, Some(along)) => along.scale(scale),
            (None, None) => along_rest(1)?.scale(scale),
            (Some(sliver), None) => {
                let (low, high) = (sliver.deriv_at(window.lo), sliver.deriv_at(window.hi));
                if low != high {
                    return None;
                }
                TaylorModel::constant(Ball::exact(low)).scale(scale)
            }
        };
        let mut s = speed
            .integral()
            .scale(Ball::exact(window.radius) * scale)
            .add_ball(Ball::exact(self.s_owned(window.center)));
        let (centre, spread) = s.spread();
        let read = self.read_distances();
        let inside_read =
            |(lo, hi): (f64, f64)| (lo.max(read.0).min(read.1), hi.min(read.1).max(read.0));
        let reach =
            self.reach_inside_crossings(window, inside_read((centre - spread, centre + spread)));
        let held = (centre + spread - self.s_owned_end).max(0.0);
        s.widen(held);
        let reach = inside_read((reach.0, reach.1 + held));
        let reach = PathReach {
            lo: reach.0,
            hi: reach.1,
            exact: (reach.0 == read.0, reach.1 == read.1),
        };
        let laid = self.table_models(Table::Laid, &s, centre, reach)?;
        let raw = match (&windows.raw, self.raw_delta) {
            (Some(raw), Some((_, at_start))) => {
                let [p, v, a, _] = raw.models(window, false)?;
                [p.add_ball(Ball::exact(-at_start)), v, a]
            }
            _ => [(); 3].map(|()| TaylorModel::constant(Ball::exact(0.0))),
        };
        let mut e = laid
            .e
            .add_ball(Ball::exact(self.e_start - self.e_spans_start));
        for &[departure, _, _] in &laid.departures {
            e.widen(departure);
        }
        let rate_squared = rate.mul(&rate);
        let rate_bound = rate.bound();
        let rate_deriv_bound = rate_deriv.bound();
        let [velocity, acceleration] = laid.along_path(&rate, &rate_squared, &rate_deriv);
        let path = [e, velocity, acceleration];
        let read = match reading {
            Reading::Follower => [(); 3].map(|()| TaylorModel::UNBOUNDED),
            Reading::AdvanceWithoutJerk | Reading::Advance => {
                let advance_laid = match self.state.advance_table() {
                    Table::Laid => None,
                    Table::Advance => Some(self.table_models(Table::Advance, &s, centre, reach)?),
                };
                let LaidModels {
                    ratio,
                    slope,
                    curvature,
                    departures,
                    ..
                } = advance_laid.as_ref().unwrap_or(&laid);
                let rate_second = match (with_jerk, &inverse_speed, rest) {
                    (false, _, _) => TaylorModel::UNBOUNDED,
                    (true, Some(inverse), _) => leader_speed_second(&leaders, inverse).scale(scale),
                    (true, None, Some(_)) => along_rest(2)?.scale(scale),
                    (true, None, None) => TaylorModel::constant(Ball::exact(0.0)),
                };
                let ValleyWeight::Model([weight, weight_slope, weight_curvature]) =
                    self.state.smoothing.valley_weight_model(&s, reach)?
                else {
                    let [velocity, acceleration] = match &advance_laid {
                        None => [path[1], path[2]],
                        Some(advance_laid) => {
                            advance_laid.along_path(&rate, &rate_squared, &rate_deriv)
                        }
                    };
                    let jerk = if with_jerk {
                        let mut jerk = ratio
                            .mul(&rate_second)
                            .add(&slope.mul(&rate).mul(&rate_deriv).scale(Ball::exact(3.0)))
                            .add(&curvature.mul(&rate_squared).mul(&rate));
                        let rate_second_bound = rate_second.bound();
                        for &[_, ratio, slope] in departures {
                            jerk.widen(
                                ratio * rate_second_bound
                                    + 3.0 * slope * rate_bound * rate_deriv_bound,
                            );
                        }
                        jerk
                    } else {
                        TaylorModel::UNBOUNDED
                    };
                    return Some(FollowerModels {
                        read: [velocity, acceleration, jerk],
                        path,
                        raw,
                    });
                };
                let weighted = ratio.mul(&weight);
                let weighted_slope = slope.mul(&weight).add(&ratio.mul(&weight_slope));
                let mut read = [
                    weighted.mul(&rate),
                    weighted_slope
                        .mul(&rate_squared)
                        .add(&weighted.mul(&rate_deriv)),
                    if with_jerk {
                        let weighted_curvature = curvature
                            .mul(&weight)
                            .add(&slope.mul(&weight_slope).scale(Ball::exact(2.0)))
                            .add(&ratio.mul(&weight_curvature));
                        weighted
                            .mul(&rate_second)
                            .add(
                                &weighted_slope
                                    .mul(&rate)
                                    .mul(&rate_deriv)
                                    .scale(Ball::exact(3.0)),
                            )
                            .add(&weighted_curvature.mul(&rate_squared).mul(&rate))
                    } else {
                        TaylorModel::UNBOUNDED
                    },
                ];
                let [w, w1, w2] =
                    [&weight, &weight_slope, &weight_curvature].map(TaylorModel::bound);
                let rate_second_bound = rate_second.bound();
                for &[_, ratio, slope] in departures {
                    let weighted_slope = slope * w + ratio * w1;
                    read[0].widen(ratio * w * rate_bound);
                    read[1].widen(
                        weighted_slope * rate_bound * rate_bound + ratio * w * rate_deriv_bound,
                    );
                    read[2].widen(
                        ratio * w * rate_second_bound
                            + 3.0 * weighted_slope * rate_bound * rate_deriv_bound
                            + (2.0 * slope * w1 + ratio * w2)
                                * rate_bound
                                * rate_bound
                                * rate_bound,
                    );
                }
                read
            }
        };
        Some(FollowerModels { path, raw, read })
    }

    /// The path distances the signal reads anywhere on its support: its
    /// odometer only advances from its start, and holds at the owned end.
    /// A window's bound on its odometer reaching past them reads no table
    /// formula there.
    fn read_distances(&self) -> (f64, f64) {
        (self.s_start, self.s_end().min(self.s_owned_end))
    }

    /// What `table` and its smoothing lay over `reach` as models on `s`:
    /// the first of `formulas` with the correction, and how far the others
    /// depart from it; `None` where the correction has no model.
    fn laid_models(
        &self,
        table: Table,
        s: &TaylorModel,
        formulas: &[TableFormula],
        reach: PathReach,
    ) -> Option<LaidModels> {
        let correction = self.state.smoothing.correction_model(table, s, reach)?;
        let [base_e, base_ratio, base_slope] = formulas[0].laid(s);
        Some(LaidModels {
            e: match formulas[0] {
                TableFormula::Empty => base_e,
                _ => base_e.add(&correction.value),
            },
            ratio: base_ratio.add(&correction.d1),
            slope: base_slope.add(&correction.d2),
            curvature: correction.d3,
            departures: formulas[1..]
                .iter()
                .filter_map(|formula| formula.departure_from(&formulas[0], reach.span()))
                .collect(),
        })
    }

    /// What `table` and its smoothing lay over a reach the table lays with
    /// more than one formula yet too narrow to tell them apart, read at
    /// each of its floats: the odometer reads no other, and the table and
    /// its correction change formula together at a seam, where read apart
    /// their departures add up to the ratio's whole step.
    fn laid_on_floats(
        &self,
        table: Table,
        formulas: &[TableFormula],
        reach: (f64, f64),
    ) -> Option<LaidModels> {
        if formulas.len() < 2 {
            return None;
        }
        let reads: Vec<LaidRead> = super::ratio_ramp::narrow_floats(reach)?
            .into_iter()
            .map(|s| self.laid_on(table, s))
            .collect();
        let spanning = |read: fn(&LaidRead) -> f64| {
            let (low, high) = reads
                .iter()
                .map(read)
                .fold((f64::INFINITY, f64::NEG_INFINITY), |(low, high), value| {
                    (low.min(value), high.max(value))
                });
            TaylorModel::constant(Ball {
                mid: 0.5 * low + 0.5 * high,
                rad: taylor::upper(
                    taylor::upper(0.5 * (high - low)) + f64::EPSILON * low.abs().max(high.abs()),
                ),
            })
        };
        Some(LaidModels {
            e: spanning(|read| read.e),
            ratio: spanning(|read| read.ratio),
            slope: spanning(|read| read.slope),
            curvature: spanning(|read| read.curvature),
            departures: Vec::new(),
        })
    }

    /// `reach` held to the side of a ratio crossing a window ends or starts
    /// at: the signal reads the formula before a crossing up to it and the
    /// one after from it on, and a fit piece ending or starting there
    /// holds the limit from inside, so a window ends on the one and starts
    /// on the other, however close its odometer bound reaches across. A
    /// window starting at a crossing may still read the boundary itself,
    /// where the odometer has not moved off it by the float, and the
    /// boundary may read a formula of its own. At a crawl the odometer's
    /// rounding even steps back below it by an ulp past the crossing: a
    /// reach narrow enough to be read at its floats is held
    /// [`REACH_INSET_ULPS`] below it, where a wider one's read points lie
    /// that far inside anyway. A bound wholly across the crossing it starts
    /// or ends at stays whole.
    fn reach_inside_crossings(&self, window: Window, reach: (f64, f64)) -> (f64, f64) {
        let (lo, hi) = reach;
        let crossings = self.ratio_crossings();
        let boundary_at = |t: f64| {
            let index = crossings.partition_point(|&(time, _)| time < t);
            crossings
                .get(index)
                .filter(|&&(time, _)| time == t)
                .map(|&(_, boundary)| boundary)
        };
        let lo = boundary_at(window.lo).map_or(lo, |boundary| {
            let inset = REACH_INSET_ULPS * f64::EPSILON * boundary.abs().max(1.0);
            if hi - boundary > 2.0 * inset {
                lo.max(boundary)
            } else {
                lo.max(boundary - inset)
            }
        });
        let hi = boundary_at(window.hi).map_or(hi, |boundary| {
            hi.min(trajectory::next_toward(boundary, f64::NEG_INFINITY))
        });
        if lo <= hi { (lo, hi) } else { reach }
    }

    /// The follower's enclosure over `[lo, hi]`, a window per stretch
    /// between the signal's grid times and its axes' formula changes;
    /// `None` where an axis has no formula a window reads, or a window no
    /// bounded model.
    pub(super) fn enclosure(
        &self,
        lo: f64,
        hi: f64,
        accuracy: [f64; 3],
    ) -> Option<Vec<ExactPiece>> {
        let breaks = self.breaks_between(lo, hi, &[])?;
        let pieces = enclose_between(
            &breaks,
            accuracy,
            taylor::Signal {
                model: &|window| {
                    self.window_models(window, Reading::Follower)
                        .map(|models| std::array::from_fn(|order| models.total(order)))
                },
                pva_at: &|t| crate::shaper::TrackSignal::eval_pva(self, t),
            },
        )?;
        taylor::debug_assert_holds(
            &pieces,
            |t| crate::shaper::TrackSignal::eval_pva(self, t),
            "follower",
        );
        Some(pieces)
    }

    /// `lo`, the grid times, the ratio crossings, the rests' ends and splits,
    /// the end slivers' inner edges and zero times, the axes' formula changes and `extra`
    /// inside `(lo, hi)` in order, and `hi`; `None` where an axis has no
    /// formula.
    pub(super) fn breaks_between(&self, lo: f64, hi: f64, extra: &[f64]) -> Option<Vec<f64>> {
        let windows = self.axis_windows()?;
        let mut breaks = vec![lo, hi];
        breaks.extend(
            self.grid
                .iter()
                .chain(extra)
                .copied()
                .chain(self.ratio_crossings().iter().map(|&(time, _)| time))
                .chain(self.rests.iter().flat_map(super::rest::Rest::breaks))
                .chain(
                    [self.start_sliver, self.end_sliver]
                        .into_iter()
                        .flatten()
                        .flat_map(|sliver| [sliver.inner, sliver.zero_time]),
                )
                .filter(|&t| t > lo && t < hi),
        );
        for axis in windows
            .leaders
            .iter()
            .map(|(_, axis)| axis)
            .chain(&windows.raw)
        {
            breaks.extend(axis.breaks(lo, hi));
        }
        breaks.sort_by(f64::total_cmp);
        breaks.dedup();
        Some(breaks)
    }

    pub(super) fn axis_windows(&self) -> Option<&FollowerAxisWindows<'a>> {
        self.axis_windows
            .get_or_init(|| {
                Some(FollowerAxisWindows {
                    leaders: self
                        .leaders
                        .axes
                        .iter()
                        .enumerate()
                        .filter_map(|(index, axis)| match AxisWindows::of(axis) {
                            Some(AxisWindows::Hold(_)) => None,
                            windows => Some(windows.map(|windows| (index, windows))),
                        })
                        .collect::<Option<_>>()?,
                    raw: match self.raw_delta {
                        Some((axis, _)) => Some(AxisWindows::of(axis)?),
                        None => None,
                    },
                })
            })
            .as_ref()
    }
}

/// The enclosure of the models `model` reads over each stretch between
/// consecutive `breaks`; `None` where a window has no bounded model.
fn enclose_between(
    breaks: &[f64],
    accuracy: [f64; 3],
    signal: taylor::Signal<'_>,
) -> Option<Vec<ExactPiece>> {
    let mut out = Vec::new();
    for stretch in breaks.windows(2).filter(|stretch| stretch[0] < stretch[1]) {
        taylor::try_enclose((stretch[0], stretch[1]), accuracy, signal, &mut out)?;
    }
    Some(out)
}

impl AdvancedFollowerSignal<'_, '_> {
    /// The advanced follower's enclosure over `[lo, hi]`: the follower's
    /// models less the advance steps behind each window, plus the advance
    /// of the velocity laid along the path weighted by its valleys — never
    /// the extrude-only motion of the raw track, which keeps its own.
    /// `None` where the follower has no bounded model.
    pub(super) fn enclosure(
        &self,
        lo: f64,
        hi: f64,
        accuracy: [f64; 3],
    ) -> Option<Vec<ExactPiece>> {
        let slivers = self.onset_slivers(lo, hi)?;
        let extra: Vec<f64> = self
            .steps
            .iter()
            .map(|step| step.t)
            .chain(slivers.iter().flat_map(|sliver| [sliver.from, sliver.to]))
            .collect();
        let breaks = self.source.breaks_between(lo, hi, &extra)?;
        let mut pieces = Vec::new();
        let pva_at = |t| crate::shaper::TrackSignal::eval_pva(self, t);
        for stretch in breaks.windows(2).filter(|stretch| stretch[0] < stretch[1]) {
            let stretch = (stretch[0], stretch[1]);
            if !slivers
                .iter()
                .any(|sliver| sliver.from <= stretch.0 && stretch.1 <= sliver.to)
            {
                taylor::try_enclose(
                    stretch,
                    accuracy,
                    taylor::Signal {
                        model: &|window| self.advanced_models(window, Reading::Advance),
                        pva_at: &pva_at,
                    },
                    &mut pieces,
                )?;
                continue;
            };
            let first = pieces.len();
            taylor::try_enclose(
                stretch,
                self.unshaped_step.enclosure_accuracy(accuracy),
                taylor::Signal {
                    model: &|window| {
                        let [position, velocity, _] =
                            self.advanced_models(window, Reading::AdvanceWithoutJerk)?;
                        Some([position, velocity, TaylorModel::constant(Ball::exact(0.0))])
                    },
                    pva_at: &pva_at,
                },
                &mut pieces,
            )?;
            for piece in &mut pieces[first..] {
                piece.pva[2].rounding = f64::INFINITY;
                piece.unshaped_step = Some(self.unshaped_step);
            }
        }
        taylor::debug_assert_holds(
            &pieces,
            |t| crate::shaper::TrackSignal::eval_pva(self, t),
            "advanced follower",
        );
        Some(pieces)
    }

    /// The advanced follower's `[p, v, a]` on `window`: the follower's
    /// models less the advance steps behind it, plus the advance of what it
    /// reads of the path — `a` unbounded without the read's jerk.
    fn advanced_models(&self, window: Window, reading: Reading) -> Option<[TaylorModel; 3]> {
        let models = self.source.window_models(window, reading)?;
        let taken: f64 = self
            .steps
            .iter()
            .take_while(|step| step.t < window.center)
            .map(|step| step.step_mm)
            .sum();
        let [position, velocity, acceleration] = std::array::from_fn(|order| models.total(order));
        let position = position.add_ball(Ball::exact(-taken));
        let [read_velocity, read_acceleration, read_jerk] = models.read;
        Some(match self.advance.stage {
            ChainStage::DerivativeGains { k1, k2 } => {
                assert_eq!(*k2, 0.0, "leading acceleration gain is not supported");
                let gain = Ball::exact(*k1);
                [
                    position.add(&read_velocity.scale(gain)),
                    velocity.add(&read_acceleration.scale(gain)),
                    acceleration.add(&read_jerk.scale(gain)),
                ]
            }
            ChainStage::NonlinearAdvance(advance) => crate::shaper::nonlinear_advance_models(
                advance,
                &[position, velocity, acceleration],
                &[read_velocity, read_acceleration, read_jerk],
            ),
            ChainStage::SmoothKernel(_) => unreachable!("a leading advance is not a kernel"),
        })
    }

    /// The slivers of `[lo, hi]` the advance's acceleration is exempt on
    /// ([`UnshapedLeaderStep`]), in time order; `None` where a leader has no
    /// formula. A sliver narrower than a float still takes the one past the
    /// onset, where the jerk no model bounds stands, and slivers no float
    /// lies between — a cap touched and left at once — are one, read from
    /// onsets as far past `[lo, hi]` as the formulas' own edges round.
    pub(super) fn onset_slivers(&self, lo: f64, hi: f64) -> Option<Vec<OnsetSliver>> {
        let windows = self.source.axis_windows()?;
        let gain = super::steepest_advance_slope(self.advance.stage)
            * self.source.ratio_bounds.ratio
            * self.source.odometer_scale;
        let mut slivers: Vec<OnsetSliver> = windows
            .leaders
            .iter()
            .flat_map(|(_, axis)| {
                let pad = FORMULA_EDGE_SLACK * (hi - lo).max(1.0);
                axis.square_root_onsets(lo - pad, hi + pad)
            })
            .map(|onset| {
                let singular_jerk = gain * onset.jerk_coefficient;
                let singular_reach = (singular_jerk / self.accel_tol).powi(2);
                let reach = (onset.far - onset.at).clamp(-singular_reach, singular_reach);
                let end = match onset.at + reach {
                    end if end == onset.at => trajectory::next_toward(onset.at, onset.far),
                    end => end,
                };
                OnsetSliver {
                    from: onset.at.min(end),
                    to: onset.at.max(end),
                    singular_change: 2.0 * singular_jerk * reach.abs().sqrt(),
                }
            })
            .collect();
        slivers.sort_by(|a, b| a.from.total_cmp(&b.from));
        slivers.dedup_by(|later, earlier| {
            if later.from <= trajectory::next_toward(earlier.to, f64::INFINITY) {
                earlier.to = earlier.to.max(later.to);
                earlier.singular_change += later.singular_change;
                true
            } else {
                false
            }
        });
        Some(
            slivers
                .into_iter()
                .filter_map(|sliver| {
                    let (from, to) = (sliver.from.max(lo), sliver.to.min(hi));
                    (from < to).then_some(OnsetSliver { from, to, ..sliver })
                })
                .collect(),
        )
    }
}

/// Where an unshaped leader leaves its curvature cap as a square root of
/// the time, the advance's acceleration takes the leader's jerk
/// `J/√τ` through its gain `G` (steepest slope × ratio × odometer scale).
/// No fit holds that: the sliver `τ < (G·J/accel_tol)²` from the onset —
/// where the singular term alone passes the acceleration tolerance — is
/// exempt from the acceleration bound, the velocity changing across it by
/// `singular_change = 2·G·J·√τ` there ([`UnshapedLeaderStep`]).
#[derive(Clone, Copy, Debug)]
pub(super) struct OnsetSliver {
    pub(super) from: f64,
    pub(super) to: f64,
    pub(super) singular_change: f64,
}
