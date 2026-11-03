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
/// span table it starts and the time it covers.
pub(super) struct RampLeg<'a> {
    pub(super) shaped: &'a ContinuousSegment,
    pub(super) leaders: &'a [usize],
    pub(super) support: (f64, f64),
    pub(super) start_s: f64,
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
/// `v` and its rate `a` where the leaders cross the seam, taken on the
/// side that reaches further. At constant speed the ramp spans `v·τ` of
/// path, crossed in τ; around a reversal, where the path speed passes
/// through zero while its rate does not, it still spans the `a·τ²/4` the
/// leaders turn around in; a seam crossed at rest gets no ramp, as an
/// advance vanishes there.
///
/// A ramp reaches up to half the span after its seam, so a segment's
/// ratios are final only once the span after it is laid: up to the middle
/// of the last span, or everything at a stream rest (see
/// [`Self::close_at`]).
#[derive(Debug, Default)]
pub(super) struct SeamRamps {
    ramps: Vec<SeamRamp>,
    examined_through: f64,
    floor: f64,
    reach_into_next: Option<f64>,
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
    /// stream drained there and what precedes it is already committed.
    pub(super) fn close_at(&mut self, s: f64) {
        self.floor = self.floor.max(s);
        self.reach_into_next = None;
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
        let first = spans
            .partition_point(|span| span.s0 <= self.examined_through)
            .max(1);
        for index in first..spans.len() {
            let (before, after) = (&spans[index - 1], &spans[index]);
            let seam = after.s0;
            self.examined_through = seam;
            let step = after.r0 - before.r1;
            let slope_step = after.ratio_slope() - before.ratio_slope();
            if step == 0.0 && slope_step == 0.0 {
                continue;
            }
            let reach = if seam <= leg.start_s {
                self.reach_into_next
                    .unwrap_or(0.0)
                    .max(reach_at(&leaders, &[t_start], ramp_time))
            } else {
                let crossing = time_at_arc(&leaders, leg, seam - leg.start_s);
                reach_at(
                    &leaders,
                    &[
                        trajectory::next_toward(crossing, f64::NEG_INFINITY).max(t_start),
                        trajectory::next_toward(crossing, f64::INFINITY).min(t_end),
                    ],
                    ramp_time,
                )
            };
            let half_width = reach
                .min(0.5 * (before.s1 - before.s0))
                .min(0.5 * (after.s1 - after.s0))
                .min(seam - self.floor);
            if half_width <= SPAN_MIN_LEN_MM {
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
        self.reach_into_next = Some(reach_at(&leaders, &[t_end], ramp_time));
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
    }
}

/// How far a ramp at a seam the leaders cross at `times` reaches, read on
/// the side that reaches further.
fn reach_at(leaders: &ShapedLeaders, times: &[f64], ramp_time: f64) -> f64 {
    times
        .iter()
        .map(|&t| {
            let speed = leaders.speed(t);
            let rate = if speed > 0.0 {
                leaders.speed_deriv(t, speed).abs()
            } else {
                leaders.accel_norm(t)
            };
            (speed + 0.5 * rate * ramp_time) * 0.5 * ramp_time
        })
        .fold(0.0, f64::max)
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
