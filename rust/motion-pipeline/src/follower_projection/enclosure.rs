//! Taylor-model enclosures of a follower's projected track: the shaped
//! leaders' polynomial pieces give the path speed `|ẋ|` and its rate, the
//! odometer `s` is its integral anchored at the window centre's own
//! reading, and the span table lays `e(s)`, `r(s)` and `r'(s)` there. The
//! follower reads `p = e(s)`, `v = r·ṡ`, `a = r'·ṡ² + r·s̈`.
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

use crate::shaper::axis_enclosure::AxisWindows;
use crate::shaper::taylor::{self, Ball, TaylorModel, Window};
use crate::shaper::{Enclosure, ExactPiece};

use super::{AdvancedFollowerSignal, ChainStage, FollowerSignal, RatioSpan};

/// How a stretch of the span table lays extrusion against path distance.
#[derive(Clone, Copy)]
enum TableFormula {
    /// Inside a span: `e = e₀ + r₀·d + ½k·d²`, `d = s − s₀`.
    Span(RatioSpan),
    /// Before the table: the first span's start, at its start ratio.
    Before(RatioSpan),
    /// Past the table: the last span's end extended at its end ratio.
    Past(RatioSpan),
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
        }
    }

    /// The path distances the formula lays.
    fn domain(&self) -> (f64, f64) {
        match *self {
            TableFormula::Span(span) => (span.s0, span.s1),
            TableFormula::Before(first) => (f64::NEG_INFINITY, first.s0),
            TableFormula::Past(last) => (last.s1, f64::INFINITY),
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

/// Whether every order of `piece` is within `accuracy`.
fn resolves(piece: &ExactPiece, accuracy: [f64; 3]) -> bool {
    piece
        .pva
        .iter()
        .zip(accuracy)
        .all(|(order, accuracy)| order.rounding <= accuracy)
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
/// leaders' path, which an advance reads, and the raw track's extrude-only
/// motion, which it does not; each `[p, v, a, j]`.
pub(super) struct FollowerModels {
    path: [TaylorModel; 4],
    raw: [TaylorModel; 4],
}

impl FollowerModels {
    /// The follower's own model of derivative `order`.
    fn total(&self, order: usize) -> TaylorModel {
        self.path[order].add(&self.raw[order])
    }
}

/// The axes a follower reads, window by window.
pub(super) struct FollowerAxisWindows<'a> {
    leaders: Vec<AxisWindows<'a>>,
    raw: Option<AxisWindows<'a>>,
}

impl<'a> FollowerSignal<'a> {
    /// The table's formulas over path distances `[lo, hi]`, the one laid at
    /// `centre` first; `None` before an empty table.
    fn table_formulas(&self, centre: f64, (lo, hi): (f64, f64)) -> Option<Vec<TableFormula>> {
        let spans = &self.state.spans;
        let (first, last) = (*spans.first()?, *spans.last()?);
        let region_of = |s: f64| {
            if s < first.s0 {
                TableFormula::Before(first)
            } else if s > last.s1 {
                TableFormula::Past(last)
            } else {
                let index = spans
                    .partition_point(|span| span.s1 < s)
                    .min(spans.len() - 1);
                TableFormula::Span(spans[index])
            }
        };
        let mut formulas = vec![region_of(centre)];
        if lo < first.s0 {
            formulas.push(TableFormula::Before(first));
        }
        if hi > last.s1 {
            formulas.push(TableFormula::Past(last));
        }
        let from = spans.partition_point(|span| span.s1 < lo);
        formulas.extend(
            spans[from..]
                .iter()
                .take_while(|span| span.s0 <= hi)
                .map(|&span| TableFormula::Span(span)),
        );
        Some(formulas)
    }

    /// The follower's `[p, v, a, j]` on `window` — `j` only `with_jerk` —
    /// split into what the span table lays along the leaders' path and the
    /// extrude-only motion of the raw track; `None` where a model is out of
    /// reach: an axis past one of its formulas or without a jerk, the
    /// smoothing, or a speed zero the rate divides by. An odometer reaching
    /// past the owned span, where the signal holds its distance, is widened
    /// by how far it reaches.
    fn window_models(&self, window: Window, with_jerk: bool) -> Option<FollowerModels> {
        let windows = self.axis_windows()?;
        let mut leaders = Vec::with_capacity(windows.leaders.len());
        let mut speed_squared = TaylorModel::constant(Ball::exact(0.0));
        let mut power = TaylorModel::constant(Ball::exact(0.0));
        for axis in &windows.leaders {
            let [velocity, acceleration, jerk] = axis
                .motion_models(window, with_jerk)?
                .map(|model| model.working_to(FOLLOWER_WORKING_SHARE));
            speed_squared = speed_squared.add(&velocity.mul(&velocity));
            power = power.add(&velocity.mul(&acceleration));
            leaders.push([velocity, acceleration, jerk]);
        }
        let slivers = (
            self.speed_zero_sliver(window.lo),
            self.speed_zero_sliver(window.hi),
        );
        let (speed, inverse_speed) = match slivers {
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
        let rate_deriv = match (slivers, &along) {
            (_, Some(along)) => along.scale(scale),
            ((Some(lo), Some(hi)), None) if lo.zero_time == hi.zero_time => {
                let side = |t: f64| lo.deriv_at(t);
                let (low, high) = (side(window.lo), side(window.hi));
                if low != high {
                    return None;
                }
                TaylorModel::constant(Ball::exact(low)).scale(scale)
            }
            _ => return None,
        };
        let mut s = speed
            .integral()
            .scale(Ball::exact(window.radius) * scale)
            .add_ball(Ball::exact(self.s_owned(window.center)));
        let (centre, spread) = s.spread();
        let reach = self.reach_inside_crossings(window, (centre - spread, centre + spread));
        if self.state.smoothing.touches(reach.0, reach.1) {
            return None;
        }
        let held = (reach.1 - self.s_owned_end).max(0.0);
        s.widen(held);
        let reach = (reach.0 - held, reach.1 + held);
        let formulas = self.table_formulas(centre, reach)?;
        let rate_second = if with_jerk {
            match &inverse_speed {
                Some(inverse) => leader_speed_second(&leaders, inverse).scale(scale),
                None => TaylorModel::constant(Ball::exact(0.0)),
            }
        } else {
            TaylorModel::UNBOUNDED
        };
        let raw = match (&windows.raw, self.raw_delta) {
            (Some(raw), Some((_, at_start))) => {
                let [p, v, a, j] = raw.models(window, with_jerk)?;
                [p.add_ball(Ball::exact(-at_start)), v, a, j]
            }
            _ => [(); 4].map(|()| TaylorModel::constant(Ball::exact(0.0))),
        };
        let laid = |formula: &TableFormula| {
            let [e, ratio, slope] = formula.laid(&s);
            [
                e.add_ball(Ball::exact(self.e_start - self.e_spans_start)),
                ratio.mul(&rate),
                slope.mul(&rate).mul(&rate).add(&ratio.mul(&rate_deriv)),
                ratio
                    .mul(&rate_second)
                    .add(&slope.mul(&rate).mul(&rate_deriv).scale(Ball::exact(3.0))),
            ]
        };
        let mut path = laid(&formulas[0]);
        let [rate, rate_deriv, rate_second] =
            [&rate, &rate_deriv, &rate_second].map(TaylorModel::bound);
        for formula in &formulas[1..] {
            let Some([e, ratio, slope]) = formula.departure_from(&formulas[0], reach) else {
                continue;
            };
            path[0].widen(e);
            path[1].widen(ratio * rate);
            path[2].widen(slope * rate * rate + ratio * rate_deriv);
            path[3].widen(ratio * rate_second + 3.0 * slope * rate * rate_deriv);
        }
        Some(FollowerModels { path, raw })
    }

    /// `reach` held to the side of a ratio crossing a window ends or starts
    /// at: the signal reads the formula before a crossing up to it and the
    /// one after from it on, and a fit piece ending or starting there
    /// holds the limit from inside, so a window ends on the one and starts
    /// on the other, however close its odometer bound reaches across. A
    /// bound wholly across the crossing it starts or ends at stays whole.
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
            lo.max(trajectory::next_toward(boundary, f64::INFINITY))
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
        let pieces = enclose_between(&breaks, accuracy, &|window| {
            self.window_models(window, false)
                .map(|models| std::array::from_fn(|order| models.total(order)))
        })?;
        taylor::debug_assert_holds(
            &pieces,
            |t| crate::shaper::TrackSignal::eval_pva(self, t),
            "follower",
        );
        Some(pieces)
    }

    /// `lo`, the grid times, the axes' formula changes and `extra` inside
    /// `(lo, hi)` in order, and `hi`; `None` where an axis has no formula.
    fn breaks_between(&self, lo: f64, hi: f64, extra: &[f64]) -> Option<Vec<f64>> {
        let windows = self.axis_windows()?;
        let mut breaks = vec![lo, hi];
        breaks.extend(
            self.grid
                .iter()
                .chain(extra)
                .copied()
                .chain(self.ratio_crossings().iter().map(|&(time, _)| time))
                .filter(|&t| t > lo && t < hi),
        );
        for axis in windows.leaders.iter().chain(&windows.raw) {
            breaks.extend(axis.breaks(lo, hi));
        }
        breaks.sort_by(f64::total_cmp);
        breaks.dedup();
        Some(breaks)
    }

    fn axis_windows(&self) -> Option<&FollowerAxisWindows<'a>> {
        self.axis_windows
            .get_or_init(|| {
                Some(FollowerAxisWindows {
                    leaders: self
                        .leaders
                        .axes
                        .iter()
                        .filter_map(|axis| match AxisWindows::of(axis) {
                            Some(AxisWindows::Hold(_)) => None,
                            windows => Some(windows),
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
    model: &dyn Fn(Window) -> Option<[TaylorModel; 3]>,
) -> Option<Vec<ExactPiece>> {
    let mut out = Vec::new();
    for stretch in breaks.windows(2).filter(|stretch| stretch[0] < stretch[1]) {
        taylor::try_enclose((stretch[0], stretch[1]), accuracy, model, &mut out)?;
    }
    Some(out)
}

impl AdvancedFollowerSignal<'_, '_> {
    /// The advanced follower's enclosure over `[lo, hi]`: the follower's
    /// models less the advance steps behind each window, plus the advance
    /// of the motion laid along the path — never the extrude-only motion of
    /// the raw track, which keeps its own. `None` for an advance reading the
    /// ramped path velocity — its valley weights are the smoothing's — or
    /// where the follower has no bounded model.
    pub(super) fn enclosure(
        &self,
        lo: f64,
        hi: f64,
        accuracy: [f64; 3],
    ) -> Option<Vec<ExactPiece>> {
        if self.advance.reads_path_only() {
            return None;
        }
        let step_times: Vec<f64> = self.steps.iter().map(|step| step.t).collect();
        let breaks = self.source.breaks_between(lo, hi, &step_times)?;
        let pieces = enclose_between(&breaks, accuracy, &|window: Window| {
            let models = self.source.window_models(window, true)?;
            let taken: f64 = self
                .steps
                .iter()
                .take_while(|step| step.t < window.center)
                .map(|step| step.step_mm)
                .sum();
            let [position, velocity, acceleration] =
                std::array::from_fn(|order| models.total(order));
            let position = position.add_ball(Ball::exact(-taken));
            let [_, read_velocity, read_acceleration, read_jerk] = models.path;
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
        })?;
        taylor::debug_assert_holds(
            &pieces,
            |t| crate::shaper::TrackSignal::eval_pva(self, t),
            "advanced follower",
        );
        Some(pieces)
    }
}
