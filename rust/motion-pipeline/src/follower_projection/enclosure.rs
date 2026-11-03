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

use crate::shaper::ExactPiece;
use crate::shaper::axis_enclosure::AxisWindows;
use crate::shaper::taylor::{self, Ball, TaylorModel, Window};

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

/// `d²|ẋ|/dt²` of leaders `[ẋᵢ, ẍᵢ, x⃛ᵢ]` at path speed `speed`:
/// `Σ_{i<j}(ẋᵢẍⱼ − ẋⱼẍᵢ)²/|ẋ|³ + ẋ·x⃛/|ẋ|`, as the signal reads it.
fn leader_speed_second(leaders: &[[TaylorModel; 3]], speed: &TaylorModel) -> Option<TaylorModel> {
    let inverse = speed.reciprocal()?;
    let mut cross_squared = TaylorModel::constant(Ball::exact(0.0));
    let mut velocity_dot_jerk = TaylorModel::constant(Ball::exact(0.0));
    for (index, [v_i, a_i, j_i]) in leaders.iter().enumerate() {
        velocity_dot_jerk = velocity_dot_jerk.add(&v_i.mul(j_i));
        for [v_j, a_j, _] in &leaders[index + 1..] {
            let cross = v_i.mul(a_j).sub(&v_j.mul(a_i));
            cross_squared = cross_squared.add(&cross.mul(&cross));
        }
    }
    let inverse_cubed = inverse.mul(&inverse).mul(&inverse);
    Some(
        cross_squared
            .mul(&inverse_cubed)
            .add(&velocity_dot_jerk.mul(&inverse)),
    )
}

/// A signal's enclosure over its whole support, built once at the
/// tightest accuracy asked so far: a tighter enclosure proves a looser
/// request as well, so it is rebuilt only for a tighter one, at the
/// tightest of both.
#[derive(Default)]
pub(super) struct WholeEnclosure {
    built: std::cell::RefCell<Option<([f64; 3], Option<Vec<ExactPiece>>)>>,
}

impl WholeEnclosure {
    /// The pieces of the whole enclosure over `[lo, hi]` at `accuracy` or
    /// tighter, `build` enclosing the whole support at a given accuracy.
    pub(super) fn over(
        &self,
        (lo, hi): (f64, f64),
        accuracy: [f64; 3],
        build: impl FnOnce([f64; 3]) -> Option<Vec<ExactPiece>>,
    ) -> Option<Vec<ExactPiece>> {
        let stale = match &*self.built.borrow() {
            Some((built, _)) => built
                .iter()
                .zip(accuracy)
                .any(|(&built, asked)| built > asked),
            None => true,
        };
        if stale {
            let tightest = match &*self.built.borrow() {
                Some((built, _)) => std::array::from_fn(|order| built[order].min(accuracy[order])),
                None => accuracy,
            };
            let whole = build(tightest);
            *self.built.borrow_mut() = Some((tightest, whole));
        }
        let built = self.built.borrow();
        let whole = built.as_ref().and_then(|(_, whole)| whole.as_ref())?;
        let first = whole.partition_point(|piece| piece.hi < lo);
        let past = whole.partition_point(|piece| piece.lo <= hi);
        Some(whole[first..past.max(first)].to_vec())
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

    /// The follower's `[p, v, a, j]` on `window` — `j` only `with_jerk` —,
    /// `None` where a model is out of reach: an axis past one of its
    /// formulas or without a jerk, the smoothing, or a speed zero the rate
    /// divides by. An odometer reaching past the owned span, where the
    /// signal holds its distance, is widened by how far it reaches.
    fn window_models(&self, window: Window, with_jerk: bool) -> Option<[TaylorModel; 4]> {
        let windows = self.axis_windows()?;
        let mut leaders = Vec::with_capacity(windows.leaders.len());
        let mut speed_squared = TaylorModel::constant(Ball::exact(0.0));
        let mut power = TaylorModel::constant(Ball::exact(0.0));
        for axis in &windows.leaders {
            let [_, velocity, acceleration, jerk] = axis.models(window, with_jerk)?;
            speed_squared = speed_squared.add(&velocity.mul(&velocity));
            power = power.add(&velocity.mul(&acceleration));
            leaders.push([velocity, acceleration, jerk]);
        }
        let speed = taylor::root_or_range(&speed_squared);
        let scale = Ball::exact(self.odometer_scale);
        let rate = speed.scale(scale);
        let sliver = |t: f64| self.speed_zero_sliver(t);
        let rate_deriv = match (sliver(window.lo), sliver(window.hi)) {
            (None, None) => power.mul(&speed.reciprocal()?).scale(scale),
            (Some(lo), Some(hi)) if lo.zero_time == hi.zero_time => {
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
        let reach = (centre - spread, centre + spread);
        if self.state.smoothing.touches(reach.0, reach.1) {
            return None;
        }
        let held = (reach.1 - self.s_owned_end).max(0.0);
        s.widen(held);
        let reach = (reach.0 - held, reach.1 + held);
        let formulas = self.table_formulas(centre, reach)?;
        let rate_second = if with_jerk {
            match (sliver(window.lo), sliver(window.hi)) {
                (None, None) => leader_speed_second(&leaders, &speed)?.scale(scale),
                _ => TaylorModel::constant(Ball::exact(0.0)),
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
        let follower = |formula: &TableFormula| {
            let [e, ratio, slope] = formula.laid(&s);
            [
                e.add_ball(Ball::exact(self.e_start - self.e_spans_start))
                    .add(&raw[0]),
                ratio.mul(&rate).add(&raw[1]),
                slope
                    .mul(&rate)
                    .mul(&rate)
                    .add(&ratio.mul(&rate_deriv))
                    .add(&raw[2]),
                ratio
                    .mul(&rate_second)
                    .add(&slope.mul(&rate).mul(&rate_deriv).scale(Ball::exact(3.0)))
                    .add(&raw[3]),
            ]
        };
        let mut models = follower(&formulas[0]);
        let [rate, rate_deriv, rate_second] =
            [&rate, &rate_deriv, &rate_second].map(TaylorModel::bound);
        for formula in &formulas[1..] {
            let Some([e, ratio, slope]) = formula.departure_from(&formulas[0], reach) else {
                continue;
            };
            models[0].widen(e);
            models[1].widen(ratio * rate);
            models[2].widen(slope * rate * rate + ratio * rate_deriv);
            models[3].widen(ratio * rate_second + 3.0 * slope * rate * rate_deriv);
        }
        Some(models)
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
        enclose_between(&breaks, accuracy, &|window| {
            self.window_models(window, false)
                .map(|[p, v, a, _]| [p, v, a])
        })
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
                        .map(|axis| AxisWindows::of(axis))
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
    /// models with jerk, less the advance steps behind each window, through
    /// the advance. `None` for an advance reading the ramped path velocity
    /// — its valley weights are the smoothing's — or where the follower has
    /// no bounded model.
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
        enclose_between(&breaks, accuracy, &|window: Window| {
            let [position, velocity, acceleration, jerk] =
                self.source.window_models(window, true)?;
            let taken: f64 = self
                .steps
                .iter()
                .take_while(|step| step.t < window.center)
                .map(|step| step.step_mm)
                .sum();
            let position = position.add_ball(Ball::exact(-taken));
            Some(match self.advance.stage {
                ChainStage::DerivativeGains { k1, k2 } => {
                    assert_eq!(*k2, 0.0, "leading acceleration gain is not supported");
                    let gain = Ball::exact(*k1);
                    [
                        position.add(&velocity.scale(gain)),
                        velocity.add(&acceleration.scale(gain)),
                        acceleration.add(&jerk.scale(gain)),
                    ]
                }
                ChainStage::NonlinearAdvance(advance) => crate::shaper::nonlinear_advance_models(
                    advance,
                    &[position, velocity, acceleration, jerk],
                ),
                ChainStage::SmoothKernel(_) => unreachable!("a leading advance is not a kernel"),
            })
        })
    }
}
