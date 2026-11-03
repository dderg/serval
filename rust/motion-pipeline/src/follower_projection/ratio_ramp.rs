use std::collections::VecDeque;

use trajectory::ContinuousSegment;

use super::{
    RatioSpan, SPAN_LOOKUP_SLACK_MM, SPAN_MIN_LEN_MM, ShapedLeaders, axis_grid, integrate,
};

/// `∫₋₁¹ (W(x) − x₊) dx`, the material the smoothed slope step adds.
const SLOPE_STEP_EXCESS: f64 = 0.1;
/// `max |c|` of the slope-step part per unit `Δm·h`: `W(0) − 0` plus the
/// compensation bump's peak `β·15/8`.
const SLOPE_STEP_PEAK: f64 = 3.0 / 16.0 + SLOPE_STEP_EXCESS * 15.0 / 8.0;
/// `max |dc/dx|` of the slope-step part per unit `Δm`: `|w − H| ≤ 1/2`
/// plus the compensation bump's steepest slope `β·60·√3/18`.
const SLOPE_STEP_PEAK_SLOPE: f64 = 0.5 + SLOPE_STEP_EXCESS * 60.0 * 0.096_225_044_864_937_63;
/// `max w'(x)`: a smoothstep over `[−h, h]` climbs at most `0.75/h`.
const SMOOTHSTEP_PEAK_SLOPE: f64 = 0.75;

/// Which half of a ramp's window carries the bump that hands back the
/// material a smoothed slope step moves.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum CompensationSide {
    Before,
    After,
}

impl CompensationSide {
    /// The half where the bump pulls the ratio towards the inside of the
    /// range the table spans there: a convex slope step is rounded up, so
    /// its excess comes back off the higher side; a concave one is rounded
    /// down, so its deficit goes back onto the lower side. Neither then
    /// leaves the range — a ramp into travel never retracts, a ramp up to
    /// a line's flow never overshoots it.
    pub(super) fn inward(slope_step: f64, before_edge: f64, after_edge: f64) -> Self {
        if (after_edge > before_edge) == (slope_step > 0.0) {
            Self::After
        } else {
            Self::Before
        }
    }
}

/// One seam of the extrusion-ratio table, where the ratio steps by `step`
/// and its path slope by `slope_step`, spread over `[seam − h, seam + h]`
/// as a correction added on top of the table. With `x = (s − seam)/h`:
///
/// `c(x) = Δ·(w(x) − H(x)) + Δm·h·(W(x) − x₊ − β·B(x))`
///
/// - `w = ½ + x(3 − x²)/4` is the smoothstep from 0 to 1 over the window
///   and `H` the unit step at the seam, so the first term turns the ratio
///   step into a C1 ramp from one side's ratio to the other's, meeting at
///   their mean on the seam. `w − ½` is odd, so the term lays no material.
/// - `W = ∫₋₁ˣ w` smooths the slope step the same way, one order up. It
///   moves `β·Δm·h²` of material (`β = 1/10`); the C1 bump
///   `B = 30y²(1 − y)²` of unit area on one half of the window hands it
///   back, on the half [`CompensationSide::inward`] picks.
///
/// Every term and its slope vanish at both window ends, and the jumps at
/// the seam cancel the table's own, so the corrected ratio is C1 and the
/// extrusion past the window is the table's exactly: [`Self::extrusion`]
/// is identically zero outside the window.
#[derive(Debug, Clone, Copy)]
pub(super) struct SeamRamp {
    seam: f64,
    half_width: f64,
    step: f64,
    slope_step: f64,
    compensation: CompensationSide,
}

impl SeamRamp {
    pub(super) fn new(
        seam: f64,
        half_width: f64,
        (step, slope_step): (f64, f64),
        compensation: CompensationSide,
    ) -> Self {
        assert!(
            half_width.is_finite() && half_width > 0.0,
            "a ratio ramp needs a positive half-width, got {half_width}"
        );
        assert!(
            step.is_finite() && slope_step.is_finite(),
            "a ratio ramp at {seam} smooths a non-finite step {step} / slope step {slope_step}"
        );
        Self {
            seam,
            half_width,
            step,
            slope_step,
            compensation,
        }
    }

    pub(super) fn start(&self) -> f64 {
        self.seam - self.half_width
    }

    pub(super) fn end(&self) -> f64 {
        self.seam + self.half_width
    }

    fn local(&self, s: f64) -> Option<f64> {
        (s > self.start() && s < self.end()).then(|| (s - self.seam) / self.half_width)
    }

    /// The compensation bump and its first two derivatives and running
    /// integral in `x`, on its own half of the window.
    fn bump(&self, x: f64) -> Bump {
        let y = match self.compensation {
            CompensationSide::Before if x < 0.0 => 1.0 + x,
            CompensationSide::After if x >= 0.0 => x,
            CompensationSide::Before => return Bump::flat(1.0),
            CompensationSide::After => return Bump::flat(0.0),
        };
        let rest = 1.0 - y;
        Bump {
            value: 30.0 * y * y * rest * rest,
            slope: 60.0 * y * rest * (1.0 - 2.0 * y),
            curvature: 60.0 * (1.0 - 6.0 * y + 6.0 * y * y),
            integral: y * y * y * (10.0 - 15.0 * y + 6.0 * y * y),
        }
    }

    pub(super) fn ratio(&self, s: f64) -> f64 {
        let Some(x) = self.local(s) else {
            return 0.0;
        };
        let (heaviside, ramp) = unit_step(x);
        self.step * (smoothstep(x) - heaviside)
            + self.slope_step
                * self.half_width
                * (smoothstep_integral(x) - ramp - SLOPE_STEP_EXCESS * self.bump(x).value)
    }

    pub(super) fn slope(&self, s: f64) -> f64 {
        let Some(x) = self.local(s) else {
            return 0.0;
        };
        let (heaviside, _) = unit_step(x);
        self.step * smoothstep_slope(x) / self.half_width
            + self.slope_step * (smoothstep(x) - heaviside - SLOPE_STEP_EXCESS * self.bump(x).slope)
    }

    pub(super) fn curvature(&self, s: f64) -> f64 {
        let Some(x) = self.local(s) else {
            return 0.0;
        };
        let h = self.half_width;
        self.step * smoothstep_curvature(x) / (h * h)
            + self.slope_step * (smoothstep_slope(x) - SLOPE_STEP_EXCESS * self.bump(x).curvature)
                / h
    }

    /// The correction's extrusion from the window start up to `s`: zero
    /// before the window and, the ramp laying no material, past it too.
    pub(super) fn extrusion(&self, s: f64) -> f64 {
        let Some(x) = self.local(s) else {
            return 0.0;
        };
        let (_, ramp) = unit_step(x);
        let h = self.half_width;
        h * (self.step * (smoothstep_integral(x) - ramp)
            + self.slope_step
                * h
                * (smoothstep_double_integral(x)
                    - 0.5 * ramp * ramp
                    - SLOPE_STEP_EXCESS * self.bump(x).integral))
    }

    pub(super) fn ratio_bound(&self) -> f64 {
        0.5 * self.step.abs() + SLOPE_STEP_PEAK * self.slope_step.abs() * self.half_width
    }

    pub(super) fn slope_bound(&self) -> f64 {
        SMOOTHSTEP_PEAK_SLOPE * self.step.abs() / self.half_width
            + SLOPE_STEP_PEAK_SLOPE * self.slope_step.abs()
    }
}

/// One segment's shaped leaders as the ramp layout reads them: where on the
/// span table it starts, the time it covers, and whether its raw leaders
/// rest along it (an extrude-only move or a dwell).
pub(super) struct RampLeg<'a> {
    pub(super) shaped: &'a ContinuousSegment,
    pub(super) leaders: &'a [usize],
    pub(super) support: (f64, f64),
    pub(super) start_s: f64,
    pub(super) rests: bool,
}

/// A segment whose projection waits for every ramp that can reach into it.
#[derive(Debug, Clone, Copy)]
pub(super) struct PendingSegment {
    pub(super) t_end: f64,
    pub(super) s_start: f64,
    pub(super) s_length: f64,
    table_end: f64,
}

/// The ramps laid over a follower's span table, and how far the table has
/// been laid.
///
/// Each seam of the table where the ratio or its slope steps gets one
/// ramp of half-width `h = min(reach, ℓ₋/2, ℓ₊/2)`: `ℓ±` are the spans on
/// either side, so neighbouring ramps never overlap and each keeps its
/// own exact material, and `reach = (v + |a|·τ/2)·τ/2` with the path speed
/// `v` and its rate `a` the leaders cross the seam with on both sides (see
/// [`Crossing`]). At constant speed the ramp spans `v·τ` of path, crossed
/// in τ; around a reversal, where the path speed passes through zero while
/// its rate does not, it still spans the `a·τ²/4` the leaders turn around
/// in; a seam crossed at rest gets no ramp, as an advance vanishes there.
///
/// A ramp reaches up to half the span after its seam, so a segment's
/// ratios are final only once the span after it is laid: up to the middle
/// of the last span, or everything at a stream rest (see
/// [`Self::close_at`]).
#[derive(Debug, Default)]
pub(super) struct SeamRamps {
    ramps: Vec<SeamRamp>,
    valleys: Vec<AdvanceValley>,
    resting: bool,
    examined_through: f64,
    floor: f64,
    reach_into_next: Option<Crossing>,
    backlog: VecDeque<PendingSegment>,
}

impl SeamRamps {
    pub(super) fn clear(&mut self) {
        *self = Self::default();
    }

    pub(super) fn is_empty(&self) -> bool {
        self.ramps.is_empty()
    }

    pub(super) fn has_backlog(&self) -> bool {
        !self.backlog.is_empty()
    }

    /// Where on the path the last segment still waiting on its ramps ends.
    pub(super) fn backlog_end(&self) -> Option<f64> {
        self.backlog
            .back()
            .map(|segment| segment.s_start + segment.s_length)
    }

    /// No ramp may reach back past `s`: the leaders rest there, or the
    /// stream drained there and what precedes it is already committed. A
    /// rest valley still open closes there, with nothing after it.
    pub(super) fn close_at(&mut self, s: f64) {
        self.floor = self.floor.max(s);
        self.reach_into_next = None;
        self.close_rest(s, 0.0);
    }

    /// A rest valley opens with no end: the advance reads nothing from its
    /// start on until a leg that moves again closes it.
    fn close_rest(&mut self, end: f64, after: f64) {
        if !std::mem::take(&mut self.resting) {
            return;
        }
        let valley = self
            .valleys
            .last_mut()
            .expect("an open rest valley is the last laid");
        valley.end = end.max(valley.start);
        valley.after = after;
    }

    /// Lays the ramps of every seam the table gained with `leg`'s spans:
    /// the seam it opens on and those inside it.
    pub(super) fn lay(&mut self, spans: &[RatioSpan], leg: &RampLeg, ramp_time: f64) {
        let laid_any = spans.last().is_some_and(|last| last.s1 > leg.start_s);
        if !laid_any {
            self.close_at(leg.start_s);
            return;
        }
        let leaders = ShapedLeaders::of(leg.shaped, leg.leaders);
        let (t_start, t_end) = leg.support;
        self.lay_rest_valley(spans, leg, &leaders, ramp_time);
        let first = spans
            .partition_point(|span| span.s0 <= self.examined_through)
            .max(1);
        for index in first..spans.len() {
            let (before, after) = (&spans[index - 1], &spans[index]);
            let seam = after.s0;
            self.examined_through = seam;
            let step = after.r0 - before.r1;
            let slope_step = after.ratio_slope() - before.ratio_slope();
            let opens_leg = seam <= leg.start_s;
            if step == 0.0 && slope_step == 0.0 && !opens_leg {
                continue;
            }
            let crossing = if opens_leg {
                let opening = Crossing::at(&leaders, t_start);
                self.reach_into_next
                    .map_or(opening, |closing| closing.common(opening))
            } else {
                let t = time_at_arc(&leaders, leg, seam - leg.start_s);
                Crossing::at(
                    &leaders,
                    trajectory::next_toward(t, f64::NEG_INFINITY).max(t_start),
                )
                .common(Crossing::at(
                    &leaders,
                    trajectory::next_toward(t, f64::INFINITY).min(t_end),
                ))
            };
            let half_width = crossing
                .reach(ramp_time)
                .min(0.5 * (before.s1 - before.s0))
                .min(0.5 * (after.s1 - after.s0))
                .min(seam - self.floor);
            if half_width <= SPAN_MIN_LEN_MM {
                continue;
            }
            if opens_leg && !self.resting && crossing.turns_around() && !self.closes_a_rest_at(seam)
            {
                self.valleys.push(AdvanceValley {
                    start: seam,
                    end: seam,
                    before: half_width,
                    after: half_width,
                });
            }
            if step == 0.0 && slope_step == 0.0 {
                continue;
            }
            let compensation = CompensationSide::inward(
                slope_step,
                before.ratio_at(seam - half_width),
                after.ratio_at(seam + half_width),
            );
            self.ramps.push(SeamRamp::new(
                seam,
                half_width,
                (step, slope_step),
                compensation,
            ));
        }
        self.reach_into_next = Some(Crossing::at(&leaders, t_end));
    }

    pub(super) fn queue(&mut self, t_end: f64, (s_start, s_length): (f64, f64), table_end: f64) {
        self.backlog.push_back(PendingSegment {
            t_end,
            s_start,
            s_length,
            table_end,
        });
    }

    /// The queued segments no future ramp can reach into, in order.
    pub(super) fn take_final(&mut self, spans: &[RatioSpan]) -> Vec<PendingSegment> {
        let final_through = spans
            .last()
            .map_or(self.floor, |last| self.floor.max(0.5 * (last.s0 + last.s1)));
        let ready = self
            .backlog
            .iter()
            .take_while(|segment| {
                segment.table_end <= self.floor
                    || segment.table_end + SPAN_LOOKUP_SLACK_MM <= final_through
            })
            .count();
        self.backlog.drain(..ready).collect()
    }

    fn at(&self, s: f64) -> impl Iterator<Item = &SeamRamp> {
        let first = self.ramps.partition_point(|ramp| ramp.end() <= s);
        self.ramps[first..]
            .iter()
            .take_while(move |ramp| ramp.start() < s)
    }

    pub(super) fn overlapping(&self, lo: f64, hi: f64) -> impl Iterator<Item = &SeamRamp> {
        let first = self.ramps.partition_point(|ramp| ramp.end() <= lo);
        self.ramps[first..]
            .iter()
            .take_while(move |ramp| ramp.start() < hi)
    }

    pub(super) fn ratio(&self, s: f64) -> f64 {
        self.at(s).map(|ramp| ramp.ratio(s)).sum()
    }

    pub(super) fn slope(&self, s: f64) -> f64 {
        self.at(s).map(|ramp| ramp.slope(s)).sum()
    }

    pub(super) fn curvature(&self, s: f64) -> f64 {
        self.at(s).map(|ramp| ramp.curvature(s)).sum()
    }

    pub(super) fn extrusion(&self, s: f64) -> f64 {
        self.at(s).map(|ramp| ramp.extrusion(s)).sum()
    }

    pub(super) fn prune_before(&mut self, s: f64) {
        let drop = self.ramps.partition_point(|ramp| ramp.end() < s);
        self.ramps.drain(..drop);
        let drop = self
            .valleys
            .partition_point(|valley| valley.reach_end() < s);
        self.valleys.drain(..drop);
    }

    /// A leg whose raw leaders rest opens a rest valley where it starts,
    /// unless one is open already; the first leg that moves again closes it
    /// where it starts. Each side reaches as a ramp at that seam would.
    fn lay_rest_valley(
        &mut self,
        spans: &[RatioSpan],
        leg: &RampLeg,
        leaders: &ShapedLeaders,
        ramp_time: f64,
    ) {
        if leg.rests == self.resting {
            return;
        }
        let opening = Crossing::at(leaders, leg.support.0);
        let crossing = self
            .reach_into_next
            .map_or(opening, |closing| closing.common(opening));
        let seam = leg.start_s;
        let span_before = spans
            .iter()
            .rev()
            .find(|span| span.s1 <= seam + SPAN_LOOKUP_SLACK_MM && span.s0 < seam);
        let span_after = spans
            .iter()
            .find(|span| span.s0 >= seam - SPAN_LOOKUP_SLACK_MM);
        let reach = crossing.reach(ramp_time);
        if leg.rests {
            let before = span_before.map_or(0.0, |span| {
                reach.min(0.5 * (span.s1 - span.s0)).min(seam - self.floor)
            });
            self.valleys.push(AdvanceValley {
                start: seam,
                end: f64::INFINITY,
                before: before.max(0.0),
                after: 0.0,
            });
            self.resting = true;
        } else {
            let after = span_after.map_or(0.0, |span| reach.min(0.5 * (span.s1 - span.s0)));
            self.close_rest(seam, after);
        }
    }

    fn closes_a_rest_at(&self, seam: f64) -> bool {
        self.valleys
            .last()
            .is_some_and(|valley| valley.end == seam && valley.end > valley.start)
    }

    /// The weight `W` the advance reads the ratio with at `s`, with its
    /// first two path derivatives: 1 away from every valley.
    pub(super) fn valley_weight(&self, s: f64) -> (f64, f64, f64) {
        let first = self
            .valleys
            .partition_point(|valley| valley.reach_end() <= s);
        self.valleys[first..]
            .iter()
            .take_while(|valley| valley.reach_start() < s)
            .fold((1.0, 0.0, 0.0), |(w, w1, w2), valley| {
                let (v, v1, v2) = valley.weight(s);
                (w * v, w1 * v + w * v1, w2 * v + 2.0 * w1 * v1 + w * v2)
            })
    }

    /// Where the valleys overlapping `[lo, hi]` start and end bending.
    pub(super) fn valley_ends(&self, lo: f64, hi: f64) -> impl Iterator<Item = f64> + '_ {
        let first = self
            .valleys
            .partition_point(|valley| valley.reach_end() <= lo);
        self.valleys[first..]
            .iter()
            .take_while(move |valley| valley.reach_start() < hi)
            .flat_map(|valley| {
                [
                    valley.reach_start(),
                    valley.start,
                    valley.end,
                    valley.reach_end(),
                ]
            })
    }
}

/// A stretch of path where a ratio-ramped advance reads the laid ratio
/// weighted down to zero: `W = 0` on `[start, end]` and `W = 3x² − 2x³`
/// within `before`/`after` of it, `x` the distance out over that width, so
/// the weight leaves zero with zero slope and meets 1 with zero slope. The
/// laid ratio, and so the material, is untouched; only the lead the
/// advance builds is released over the path before and rebuilt after.
///
/// - A turnaround: the leaders' path speed passes through zero at a seam
///   while its rate does not, so the rate steps from `−a` to `+a`, and an
///   advance reading `r·ṡ` would step the follower's velocity by `2·k·r·a`
///   whatever the ratio's own shape. The valley is that seam, each side as
///   wide as a ramp there.
/// - A rest: an extrude-only move or a dwell, along which only the shaped
///   kernel spill moves the leaders — the tail of the move before, the lead
///   of the move after, and the fit's back-and-forth near the rest point,
///   whose own turnarounds and short idle spans an advance would turn into
///   steps. The valley is the rest's whole arc.
#[derive(Debug, Clone, Copy)]
struct AdvanceValley {
    start: f64,
    end: f64,
    before: f64,
    after: f64,
}

impl AdvanceValley {
    fn reach_start(&self) -> f64 {
        self.start - self.before
    }

    fn reach_end(&self) -> f64 {
        self.end + self.after
    }

    fn weight(&self, s: f64) -> (f64, f64, f64) {
        let (distance, width, outward) = if s < self.start {
            (self.start - s, self.before, -1.0)
        } else if s > self.end {
            (s - self.end, self.after, 1.0)
        } else {
            return (0.0, 0.0, 0.0);
        };
        if distance >= width {
            return (1.0, 0.0, 0.0);
        }
        let x = distance / width;
        (
            x * x * (3.0 - 2.0 * x),
            outward * 6.0 * x * (1.0 - x) / width,
            (6.0 - 12.0 * x) / (width * width),
        )
    }
}

/// The time a turnaround's rate takes to build the speed the leaders cross
/// its seam with: the shaped leaders' velocities there are zero to their
/// fit's rounding, far under the rate times this.
const TURNAROUND_SPEED_TIME_S: f64 = 1e-6;

/// How the leaders cross a seam: their path speed and the magnitude of its
/// rate.
#[derive(Debug, Clone, Copy)]
struct Crossing {
    speed: f64,
    rate: f64,
}

impl Crossing {
    fn at(leaders: &ShapedLeaders, t: f64) -> Self {
        let speed = leaders.speed(t);
        let rate = if speed > 0.0 {
            leaders.speed_deriv(t, speed).abs()
        } else {
            leaders.accel_norm(t)
        };
        Self { speed, rate }
    }

    /// What both sides of a seam share. A rate on one side only — a leader
    /// starting from rest with an acceleration step, as an unshaped z does
    /// — leaves the seam crossed at rest: an advance steps the follower's
    /// velocity there by `k·r·Δa` whether the ratio is ramped or not, so a
    /// ramp would only carry the ratio step's material into the move.
    fn common(self, other: Self) -> Self {
        Self {
            speed: self.speed.min(other.speed),
            rate: self.rate.min(other.rate),
        }
    }

    fn reach(self, ramp_time: f64) -> f64 {
        (self.speed + 0.5 * self.rate * ramp_time) * 0.5 * ramp_time
    }

    /// The leaders turn around at the seam: they cross it at a speed their
    /// rate builds within [`TURNAROUND_SPEED_TIME_S`], the rounding of an
    /// exact zero, while that rate itself is no rounding.
    fn turns_around(self) -> bool {
        self.rate > 0.0 && self.speed <= self.rate * TURNAROUND_SPEED_TIME_S
    }
}

/// When the leaders have covered `arc` of path into `leg`.
fn time_at_arc(leaders: &ShapedLeaders, leg: &RampLeg, arc: f64) -> f64 {
    let speed = |t: f64| leaders.speed(t);
    let (t_start, t_end) = leg.support;
    let mut grid = axis_grid(leg.shaped, leg.leaders);
    grid.retain(|&t| t > t_start && t < t_end);
    grid.insert(0, t_start);
    grid.push(t_end);
    let mut covered = 0.0;
    for window in grid.windows(2) {
        let piece = integrate(&speed, window[0], window[1]);
        if covered + piece >= arc {
            let (mut lo, mut hi) = (window[0], window[1]);
            loop {
                let mid = 0.5 * lo + 0.5 * hi;
                if mid <= lo || mid >= hi {
                    return hi;
                }
                if covered + integrate(&speed, window[0], mid) < arc {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
        }
        covered += piece;
    }
    t_end
}

struct Bump {
    value: f64,
    slope: f64,
    curvature: f64,
    integral: f64,
}

impl Bump {
    fn flat(integral: f64) -> Self {
        Self {
            value: 0.0,
            slope: 0.0,
            curvature: 0.0,
            integral,
        }
    }
}

/// `H(x)` and `x₊`, taking the seam itself with the side after it as the
/// span table does.
fn unit_step(x: f64) -> (f64, f64) {
    if x >= 0.0 { (1.0, x) } else { (0.0, 0.0) }
}

fn smoothstep(x: f64) -> f64 {
    0.5 + 0.25 * x * (3.0 - x * x)
}

fn smoothstep_slope(x: f64) -> f64 {
    0.75 * (1.0 - x * x)
}

fn smoothstep_curvature(x: f64) -> f64 {
    -1.5 * x
}

fn smoothstep_integral(x: f64) -> f64 {
    let x2 = x * x;
    0.5 * (x + 1.0) + (6.0 * x2 - x2 * x2 - 5.0) / 16.0
}

fn smoothstep_double_integral(x: f64) -> f64 {
    let x2 = x * x;
    let rise = x + 1.0;
    0.25 * rise * rise + (2.0 * x2 * x - 0.2 * x2 * x2 * x - 5.0 * x - 3.2) / 16.0
}

#[cfg(test)]
mod tests;
