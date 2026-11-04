use std::collections::VecDeque;

use geometry::{
    ADVANCE_VALLEY_SPEED_MM_S, RATIO_RAMP_MAX_TIME_S, RATIO_RAMP_MIN_TIME_S, RatioRampRule,
    RatioStep,
};
use trajectory::ContinuousSegment;

use super::{
    FloorLeaders, LeaderReads, RatioSpan, SPAN_LOOKUP_SLACK_MM, SPAN_MIN_LEN_MM, ShapedLeaders,
    axis_grid, integrate,
};

mod jet;
mod models;

pub(super) use jet::Jet;
use jet::{horner, horner_with_slope};
pub(super) use models::{ValleyWeight, narrow_floats};

/// The steepest the width envelope climbs per unit of path away from a
/// node (see [`WidthField`]).
const ENVELOPE_SLOPE: f64 = 0.25;
/// `max |h'|` the width reaches: `3·ENVELOPE_SLOPE` rising off a rest,
/// `1.875·ENVELOPE_SLOPE` between two moving nodes.
pub(super) const WIDTH_SLOPE_BOUND: f64 = 3.0 * ENVELOPE_SLOPE;
/// Width nodes per ramp time: one every half of the shortest ramp.
const WIDTH_NODES_PER_RAMP_TIME: f64 = 2.0;
const NODE_SPACING_S: f64 = RATIO_RAMP_MIN_TIME_S / WIDTH_NODES_PER_RAMP_TIME;
/// How far apart on the path two moving width nodes lie at least, in
/// units of the narrower of their targets: the smootherstep between nodes
/// `L` apart that differ by at most `ENVELOPE_SLOPE·L` bends `h` by up to
/// `h''' = 60·ENVELOPE_SLOPE/L²`, which the smoothed ratio's curvature
/// carries, so an advance turns it into extruder acceleration. Nodes most
/// of a window's width apart keep that bend within the window's own; short
/// of a whole width, since at cruise on the shortest ramp a width is one
/// time tick of path exactly, and a tie there would hang the choice on how
/// the path's rounding falls.
const NODE_SPACING_PER_WIDTH: f64 = 0.75;
/// How far along the path a seam holds the width its ramp asks for, in
/// units of that width `V·τ/2`: a window that wide reaches the seam from
/// up to one width away, and the nodes bracketing that reach lie within
/// one more.
const SEAM_WIDTH_PLATEAU_WIDTHS: f64 = 2.0;
/// How fast the ramp time a seam asks for falls off in time away from it:
/// at a steady speed the width target `v·τ/2` then falls off along the
/// path no faster than [`ENVELOPE_SLOPE`].
const RAMP_TIME_SLOPE: f64 = 2.0 * ENVELOPE_SLOPE;
/// How far in time a seam raises the ramp time around it: it holds its own
/// within one ramp time of the seam, then falls off.
const RAMP_TIME_REACH_S: f64 =
    RATIO_RAMP_MAX_TIME_S + (RATIO_RAMP_MAX_TIME_S - RATIO_RAMP_MIN_TIME_S) / RAMP_TIME_SLOPE;
/// How far in time the seams whose steps a window may gather with one lie
/// from it: half the longest ramp.
const CLUSTER_REACH_S: f64 = 0.5 * RATIO_RAMP_MAX_TIME_S;
/// How long after a node the leaders must be laid before its target is
/// known: every seam that raises its ramp time or its width, and every
/// seam those gather, lies within.
const TARGET_DELAY_S: f64 = RAMP_TIME_REACH_S + CLUSTER_REACH_S;
/// The ramp time over which a turnaround or crawl valley releases and
/// rebuilds the advance's lead, the longest, whatever the step there; and
/// the furthest a rest valley reaches past its spill ([`RestSide::width`]).
const VALLEY_RAMP_TIME_S: f64 = RATIO_RAMP_MAX_TIME_S;
/// A node's target below which the leaders count as resting there: the
/// fit's back-and-forth around a rest point, far under any ramp.
const REST_TARGET_MM: f64 = SPAN_LOOKUP_SLACK_MM;
const SMOOTHERSTEP: [f64; 6] = [0.0, 0.0, 0.0, 10.0, -15.0, 6.0];
const SMOOTHERSTEP_PEAK_CURVATURE: f64 = 5.773_502_691_896_258;
/// `ψ(x) = 1 − (1 − x)³`: zero with slope 3 at a rest, flat to second
/// order at the moving node.
const RISE_FROM_REST: [f64; 4] = [0.0, 3.0, -3.0, 1.0];
const FALL_TO_REST: [f64; 4] = [1.0, 0.0, 0.0, -1.0];
const REST_PEAK_CURVATURE: f64 = 6.0;
/// `K(u) = ¾(1 − u²)` on `[−1, 1]`: its peak, `∫|u|K` and `½∫u²K`.
const KERNEL_PEAK: f64 = 0.75;
const KERNEL_ABS_MOMENT: f64 = 0.375;
const KERNEL_HALF_VARIANCE: f64 = 0.1;
/// `∫₋₁ˣ ∫₋₁ʸ K`: a unit ramp `x₊` smoothed by the kernel.
const SMOOTHED_RAMP: [f64; 5] = [0.1875, 0.5, 0.375, 0.0, -0.0625];
/// `∫₋₁ˣ` of [`SMOOTHED_RAMP`]: `x₊²/2` smoothed by the kernel.
const SMOOTHED_HALF_SQUARE: [f64; 6] = [0.05, 0.1875, 0.25, 0.125, 0.0, -0.0125];
/// [`SMOOTHED_HALF_SQUARE`] about the window's edge, `P₂(w − 1)` in powers
/// of `w = 1 + x`: a fourfold root there.
const SMOOTHED_HALF_SQUARE_OFF_EDGE: [f64; 6] = [0.0, 0.0, 0.0, 0.0, 0.0625, -0.0125];
const WIDTH_BISECTIONS: usize = 64;
/// How close in ulps a Newton step on a window edge settles before the
/// float the edge flips at is bracketed and bisected for.
const NEWTON_SETTLED_ULPS: f64 = 4.0;
/// Speed probes per leader knot interval a rest leg is searched with for
/// where the shaped leaders come to rest.
const REST_PROBES_PER_INTERVAL: usize = 8;
const REST_REFINEMENTS: usize = 64;

/// One segment's shaped leaders as the smoothing reads them: where on the
/// span table it starts, the time it covers, and whether its raw leaders
/// rest along it (an extrude-only move or a dwell).
pub(super) struct RampLeg<'a> {
    pub(super) shaped: &'a ContinuousSegment,
    pub(super) leaders: &'a [usize],
    pub(super) shaped_leaders: &'a ShapedLeaders<'a>,
    pub(super) support: (f64, f64),
    pub(super) start_s: f64,
    pub(super) rests: bool,
}

/// A segment whose projection waits for every seam and width node that can
/// reach into it, with what its ingest read off its leaders.
#[derive(Debug)]
pub(super) struct PendingSegment {
    pub(super) t_end: f64,
    pub(super) s_start: f64,
    pub(super) s_length: f64,
    pub(super) reads: LeaderReads,
    table_end: f64,
}

/// Where the span table's ratio or its slope steps.
#[derive(Debug, Clone, Copy)]
struct Seam {
    s: f64,
    step: f64,
    slope_after: f64,
}

impl Seam {
    /// The seam where `after` follows `before`, if the ratio or its slope
    /// steps there.
    fn between(before: &RatioSpan, after: &RatioSpan) -> Option<Self> {
        let step = after.r0 - before.r1;
        let slope_after = after.ratio_slope();
        (step != 0.0 || slope_after != before.ratio_slope()).then_some(Self {
            s: after.s0,
            step,
            slope_after,
        })
    }
}

/// The seams inside one window as [`RatioSmoothing::correction`] sums
/// them, each at `u = s − σ`: the moments `Σ Δ·uᵏ` of their ratio steps,
/// the moments `Σ m·(u_aᵏ − u_bᵏ)` of the table's slope over each span
/// between two of them, and the table's own kink behind `s` they carry,
/// with its two derivatives.
///
/// With `a`, `b` the coefficients of [`SMOOTHED_RAMP`] and
/// [`SMOOTHED_HALF_SQUARE`], `h·G₁(u/h) = Σₖ aₖ·uᵏ·h¹⁻ᵏ − u₊` and
/// `h²·G₂(u/h) = Σₖ bₖ·uᵏ·h²⁻ᵏ − u₊²/2`. A ratio step adds `Δ·h·G₁`; the
/// slope `m` over a span adds `m·h²·(G₂(x_a) − G₂(x_b))`, its ends'
/// `x = u/h`, which sums to the slope steps' `Δm·h²·G₂` and the slope
/// behind the window's `0.1·h²·m`. So the correction is the moments
/// weighted by powers of `h`, and a moment's path derivative is
/// `(Σ c·uᵏ)' = k·Σ c·uᵏ⁻¹`: each seam costs a few products whatever the
/// jets of `h`. A span inside the window enters as its rise `m·(u_a − u_b)`
/// times `(u_aᵏ − u_bᵏ)/(u_a − u_b)`, never as two slope steps of opposite
/// sign: a corner blend nanometres long steps the slope by millions, and
/// those would cancel only to an ulp of `m·h²`. The two spans running past
/// the window's edges are [`edge_spans`].
#[derive(Default)]
struct SeamMoments {
    step: [f64; SMOOTHED_RAMP.len()],
    slope: [f64; SMOOTHED_HALF_SQUARE.len()],
    behind: [f64; 3],
    steps: bool,
}

impl SeamMoments {
    fn add_step(&mut self, u: f64, step: f64) {
        if step == 0.0 {
            return;
        }
        self.steps = true;
        let mut power = 1.0;
        for moment in &mut self.step {
            *moment += step * power;
            power *= u;
        }
        if u >= 0.0 {
            self.behind[0] += step * u;
            self.behind[1] += step;
        }
    }

    /// The span of slope `slope` and `length` between the seams at
    /// `u_a > u_b`. A seam at `s` lies behind it, as the table reads the
    /// span it opens there.
    fn add_span(&mut self, (u_a, u_b): (f64, f64), slope: f64, length: f64) {
        if slope == 0.0 {
            return;
        }
        let rise = slope * length;
        let mut divided_difference = 0.0;
        let mut power_b = 1.0;
        for moment in &mut self.slope[1..] {
            divided_difference = divided_difference * u_a + power_b;
            power_b *= u_b;
            *moment += rise * divided_difference;
        }
        if u_b >= 0.0 {
            self.behind[0] += rise * 0.5 * (u_a + u_b);
            self.behind[1] += rise;
        } else if u_a >= 0.0 {
            self.behind[0] += 0.5 * slope * u_a * u_a;
            self.behind[1] += slope * u_a;
            self.behind[2] += slope;
        }
    }

    /// The moment `moments[k]` with its path derivatives.
    fn moment(moments: &[f64], k: usize) -> Jet {
        let derivative = |order: usize| {
            if order > k {
                return 0.0;
            }
            let falling: usize = (k + 1 - order..=k).product();
            falling as f64 * moments[k - order]
        };
        Jet {
            value: moments[k],
            d1: derivative(1),
            d2: derivative(2),
            d3: derivative(3),
        }
    }

    /// `X` but for [`edge_spans`] with the width `h` and its inverse: the
    /// moments of power `k` of the slope weigh with `h²⁻ᵏ`, of the ratio
    /// steps with `h¹⁻ᵏ`; `h⁰` is no product.
    fn correction(&self, h: Jet, inverse: Jet) -> Jet {
        let inverse_squared = inverse * inverse;
        let powers_of_h = [
            Some(h * h),
            Some(h),
            None,
            Some(inverse),
            Some(inverse_squared),
            Some(inverse_squared * inverse),
        ];
        let kink = Jet {
            value: -self.behind[0],
            d1: -self.behind[1],
            d2: -self.behind[2],
            d3: 0.0,
        };
        powers_of_h
            .iter()
            .enumerate()
            .fold(kink, |total, (index, &power)| {
                let slope_term =
                    Self::moment(&self.slope, index).scale(SMOOTHED_HALF_SQUARE[index]);
                let weight = match index.checked_sub(1) {
                    Some(k) if self.steps => {
                        slope_term + Self::moment(&self.step, k).scale(SMOOTHED_RAMP[k])
                    }
                    _ => slope_term,
                };
                total + power.map_or(weight, |power| power * weight)
            })
    }
}

/// What the two spans of the table running past the window's edges add to
/// `X`: `slope_left` from behind the back edge up to the first seam inside,
/// at `u_first`, `m·h²·(G₂(1) − G₂(x))`, and `slope_right` from the last,
/// at `u_last`, on past the front edge, `m·h²·G₂(x)`. Each `G₂` is read
/// about the edge it lies nearer, `P₂(x)` as `P₂(w − 1)` with `w = 1 + x`
/// and `0.1 − G₂(x)` as `P₂(−x)` with `w = 1 − x`: a span that stops short
/// of `s` is as steep as its rise over `w·h`, and `P₂`'s fourfold root
/// there keeps the product of the two within an ulp of the rise. No seam
/// inside: one span over the whole window, `0.1·h²·m`.
fn edge_spans(h: Jet, inverse: Jet, slope_left: f64, ends: Option<(f64, f64, f64)>) -> Jet {
    let Some((u_first, u_last, slope_right)) = ends else {
        return (h * h).scale(KERNEL_HALF_VARIANCE * slope_left);
    };
    let back = if slope_left == 0.0 {
        Jet::ZERO
    } else {
        half_square_about_edges(h, inverse, u_first)
            .1
            .scale(slope_left)
    };
    let front = if slope_right == 0.0 {
        Jet::ZERO
    } else {
        half_square_about_edges(h, inverse, u_last)
            .0
            .scale(slope_right)
    };
    h * h * (back + front)
}

/// `(G₂(x), 0.1 − G₂(x))` at `x = u/h`, the one off the nearer edge read
/// about it (see [`edge_spans`]).
fn half_square_about_edges(h: Jet, inverse: Jet, u: f64) -> (Jet, Jet) {
    let u = Jet::variable(u);
    let complement = |part: Jet| part.scale(-1.0).offset(KERNEL_HALF_VARIANCE);
    if u.value < 0.0 {
        let ahead = ((h + u) * inverse).poly(&SMOOTHED_HALF_SQUARE_OFF_EDGE);
        (ahead, complement(ahead))
    } else {
        let behind = ((h + u.scale(-1.0)) * inverse).poly(&SMOOTHED_HALF_SQUARE_OFF_EDGE);
        (complement(behind), behind)
    }
}

/// One node of the width field: its path position, the half-width its
/// leaders ask for there, and the envelope the field takes.
#[derive(Debug, Clone, Copy)]
struct WidthNode {
    s: f64,
    target: f64,
    envelope: f64,
}

/// Where and when the leaders are at one tick of the time grid, and half
/// the path they cover between the ticks on either side once the next is
/// laid.
#[derive(Debug, Clone, Copy)]
struct TimeSample {
    s: f64,
    t: f64,
    base: Option<f64>,
}

/// The width field's time ticks since the leaders last stood still: every
/// [`NODE_SPACING_S`] from `origin`, and the next one to sample.
#[derive(Debug, Clone, Copy)]
struct SampleGrid {
    origin: f64,
    next: i64,
}

impl SampleGrid {
    fn time(&self, tick: i64) -> f64 {
        self.origin + tick as f64 * NODE_SPACING_S
    }
}

/// What may still become a node, in path order: a time sample, or a rest.
#[derive(Debug, Clone, Copy)]
enum Candidate {
    Time(TimeSample),
    Rest(f64),
}

impl Candidate {
    fn s(&self) -> f64 {
        match *self {
            Self::Time(sample) => sample.s,
            Self::Rest(s) => s,
        }
    }
}

/// A ratio step as its ramp time is derived: where and when the leaders
/// cross it, how fast and under what acceleration, how far the ratio moves
/// there — its step and the change along the span it opens, which a window
/// may gather as one step where that span is short — and the largest ratio
/// about it.
#[derive(Debug, Clone, Copy)]
struct TimedSeam {
    t: f64,
    s: f64,
    crossing: Crossing,
    step: f64,
    peak_ratio: f64,
    ramp_time: Option<f64>,
}

/// The last moving node the width field kept: when and where, its ramp
/// time and its target.
#[derive(Debug, Clone, Copy)]
struct KeptNode {
    t: f64,
    s: f64,
    ramp_time: f64,
    target: f64,
}

/// How the leaders cross the last half of the longest ramp of a laid leg
/// ending at `until`.
#[derive(Debug, Clone, Copy)]
struct LegTail {
    until: f64,
    crossing: Crossing,
}

/// The part of `support` a ramp laid at `t` may reach: half the longest
/// ramp on either side.
fn ramp_reach_within(t: f64, (start, end): (f64, f64)) -> (f64, f64) {
    (
        (t - CLUSTER_REACH_S).clamp(start, end),
        (t + CLUSTER_REACH_S).clamp(start, end),
    )
}

/// The smoothing's half-width `h(s)` along the path.
///
/// Each ratio step asks for its own ramp time `τ_σ`
/// ([`RatioRampRule::ramp_time`]) from the fastest and most accelerating
/// the leaders are over its ramp and the steps within half the longest ramp
/// of it, which one window may gather. The ramp time at `t` is the largest
/// of the seams around, each holding `τ_σ` within `τ_σ` of its seam and
/// falling off by half the time beyond, never under the shortest ramp.
///
/// The rule bounds a ramp whose window is `H_σ = V·τ_σ/2` wide wherever the
/// seam is inside it, `V` the fastest the leaders cross it at. The leaders'
/// own speed alone would not lay that: where they brake hard next to the
/// step, or slow into a corner a few millimetres off it, the targets fall
/// off faster than the envelope may, and the slow node's cone narrows the
/// window below `H_σ` exactly where the leaders are fast. So each seam
/// also asks for its width ([`Self::seam_width_at`]): every node within
/// [`SEAM_WIDTH_PLATEAU_WIDTHS`] widths of it on the path targets at least
/// `H_σ`, falling off by [`ENVELOPE_SLOPE`] beyond, so only a rest narrows
/// a seam's window below its width.
///
/// The leaders are sampled every half of the shortest ramp in time. A
/// sample becomes a node once every seam that sets its ramp time `τ` or its
/// width is laid, and only half its own ramp time after the last one and no
/// nearer on the path than [`NODE_SPACING_PER_WIDTH`] times the narrower of
/// the two targets: the smootherstep between two of them bends `h` no
/// sharper than a window's own width allows. A node's own target is half
/// the path the leaders cover in `τ` around it — `v·τ/2` at cruise,
/// `a·τ²/8` at a turnaround, the accelerating path on both sides at a slow
/// corner — and zero at a rest: where the leaders cross a leg seam at rest,
/// at a floor, and at every sample inside a dwell, each of which is kept.
/// The envelope is the largest `ENVELOPE_SLOPE`-Lipschitz field under every
/// target, and `h` blends neighbouring envelope values by a smootherstep
/// (flat to second order at each node, so `h` is C2), or rises off a rest
/// node as `ψ`, linearly at first: a window shrinks to nothing at a rest at
/// most as fast as the distance to it, so it never reaches across.
///
/// `|h'| ≤ WIDTH_SLOPE_BOUND < 1` everywhere, which keeps the smoothed
/// ratio a convex combination of the table's (see [`RatioSmoothing`]).
#[derive(Debug, Default)]
struct WidthField {
    nodes: Vec<WidthNode>,
    candidates: VecDeque<Candidate>,
    seams: Vec<TimedSeam>,
    /// How many of the first `seams` a floor closed: no leg after it is
    /// reached by their ramps.
    closed_seams: usize,
    rule: Option<RatioRampRule>,
    grid: Option<SampleGrid>,
    pending: Option<usize>,
    before_pending: f64,
    last_kept: Option<KeptNode>,
    final_prefix: usize,
}

impl WidthField {
    /// Samples the times `leg` covers on the grid the stream laid since the
    /// leaders last stood still: a leg they open from a standstill — the
    /// stream's start, a drain or a dwell — lays a fresh one from its own
    /// start, so the motion after a rest is sampled as it would be planned
    /// from rest. A standstill is the leaders covering less than
    /// [`REST_TARGET_MM`] over a node spacing, whether at speed or by
    /// accelerating: the shaped leaders' fit leaves a dwell's end a rounding
    /// off an exact zero (1e-14 mm/s, 1e-9 mm/s²). Ticks in a time gap
    /// before `leg`, legal only at rest, sample its start.
    fn extend(&mut self, leaders: &ShapedLeaders, leg: &RampLeg, table_end: f64) {
        let grid = axis_grid(leg.shaped, leg.leaders);
        let (t0, t1) = (grid[0], grid[grid.len() - 1]);
        let opens_from_standstill = leaders.speed(t0) <= REST_SPEED_MM_S
            && leaders.accel_norm(t0) <= REST_SPEED_MM_S / NODE_SPACING_S;
        let mut ticks = match self.grid {
            Some(ticks) if !opens_from_standstill => ticks,
            _ => SampleGrid {
                origin: t0,
                next: 0,
            },
        };
        if self.nodes.is_empty() && self.candidates.is_empty() {
            self.place_rest(leg.start_s);
        }
        let speed = |t: f64| leaders.speed(t);
        let (mut knot, mut at, mut covered) = (1, t0, 0.0);
        loop {
            let t = ticks.time(ticks.next);
            if t >= t1 {
                break;
            }
            if t > t0 {
                while knot < grid.len() - 1 && grid[knot] < t {
                    covered += integrate(&speed, at, grid[knot]);
                    at = grid[knot];
                    knot += 1;
                }
                covered += integrate(&speed, at, t);
                at = t;
            }
            self.place_sample((leg.start_s + covered).min(table_end), t.max(t0));
            ticks.next += 1;
        }
        self.grid = Some(ticks);
    }

    fn place_sample(&mut self, s: f64, t: f64) {
        if let Some(pending) = self.pending {
            self.resolve(pending, s);
        } else {
            self.before_pending = s;
        }
        self.candidates
            .push_back(Candidate::Time(TimeSample { s, t, base: None }));
        self.pending = Some(self.candidates.len() - 1);
    }

    /// The pending sample's base: half the path between the samples on
    /// either side of it, `next` the one after.
    fn resolve(&mut self, pending: usize, next: f64) {
        let Candidate::Time(sample) = &mut self.candidates[pending] else {
            unreachable!("the pending candidate is a time sample");
        };
        let base = 0.5 * (next.max(sample.s) - self.before_pending);
        sample.base = Some(if base <= REST_TARGET_MM { 0.0 } else { base });
        self.before_pending = sample.s;
    }

    fn add_seam(&mut self, seam: TimedSeam) {
        assert!(
            self.seams.last().is_none_or(|last| seam.t >= last.t),
            "a ratio step at t={} is laid behind the last one at t={}",
            seam.t,
            self.seams.last().map_or(f64::NAN, |last| last.t)
        );
        self.seams.push(seam);
    }

    /// Widens every open seam whose ramp reaches into the leg over
    /// `support` by how the leaders cross what it reaches there. Its ramp
    /// time is not derived yet: the leaders are laid only up to `support`'s
    /// start, and [`Self::assign`] derives none within half the longest ramp
    /// of that.
    fn widen_seams_into(&mut self, leaders: &ShapedLeaders, support: (f64, f64)) {
        let first = self
            .seams
            .partition_point(|seam| seam.t + CLUSTER_REACH_S <= support.0)
            .max(self.closed_seams);
        for seam in &mut self.seams[first..] {
            assert!(
                seam.ramp_time.is_none(),
                "the ratio step at t={} has its ramp time derived before the leg from t={} its \
                 ramp reaches into is laid",
                seam.t,
                support.0
            );
            seam.crossing = seam
                .crossing
                .widest(Crossing::over(leaders, ramp_reach_within(seam.t, support)));
        }
    }

    /// The ramp time seam `index` asks for, with the steps it gathers.
    fn seam_ramp_time(&mut self, index: usize) -> f64 {
        if let Some(ramp_time) = self.seams[index].ramp_time {
            return ramp_time;
        }
        let seam = self.seams[index];
        let (step, peak_ratio) = self
            .seams
            .iter()
            .filter(|other| (other.t - seam.t).abs() <= CLUSTER_REACH_S)
            .fold((0.0, 0.0_f64), |(step, peak), other| {
                (step + other.step.abs(), peak.max(other.peak_ratio))
            });
        let rule = self.rule.expect("a seam is laid under a ramp rule");
        let ramp_time = rule.ramp_time(&RatioStep {
            ratio_step: step,
            peak_ratio,
            speed_mm_s: seam.crossing.speed,
            accel_mm_s2: seam.crossing.rate,
            jerk_mm_s3: seam.crossing.jerk,
        });
        self.seams[index].ramp_time = Some(ramp_time);
        ramp_time
    }

    /// The seams whose ramp time or width reaches the time `t`.
    fn seams_reaching(&self, t: f64) -> std::ops::Range<usize> {
        let first = self
            .seams
            .partition_point(|seam| seam.t < t - RAMP_TIME_REACH_S);
        let last = self
            .seams
            .partition_point(|seam| seam.t <= t + RAMP_TIME_REACH_S);
        first..last
    }

    /// The ramp time at `t`: the largest a seam around asks for there.
    fn ramp_time_at(&mut self, t: f64) -> f64 {
        self.seams_reaching(t)
            .fold(RATIO_RAMP_MIN_TIME_S, |ramp_time, index| {
                let seam_ramp_time = self.seam_ramp_time(index);
                let beyond_plateau = ((self.seams[index].t - t).abs() - seam_ramp_time).max(0.0);
                ramp_time.max(seam_ramp_time - RAMP_TIME_SLOPE * beyond_plateau)
            })
    }

    /// The width the seams around ask for at the node at `t`, `s`: each its
    /// ramp's `V·τ/2` within [`SEAM_WIDTH_PLATEAU_WIDTHS`] of it on the
    /// path, falling off by [`ENVELOPE_SLOPE`] beyond (see [`WidthField`]).
    fn seam_width_at(&mut self, t: f64, s: f64) -> f64 {
        self.seams_reaching(t).fold(0.0, |width, index| {
            let ramp_time = self.seam_ramp_time(index);
            let seam = self.seams[index];
            let seam_width = 0.5 * seam.crossing.speed * ramp_time;
            let beyond_plateau =
                ((s - seam.s).abs() - SEAM_WIDTH_PLATEAU_WIDTHS * seam_width).max(0.0);
            width.max(seam_width - ENVELOPE_SLOPE * beyond_plateau)
        })
    }

    /// Turns every candidate at or before `through` whose base is known
    /// into a node or drops it, in order.
    fn assign(&mut self, through: f64) {
        while let Some(&candidate) = self.candidates.front() {
            match candidate {
                Candidate::Rest(s) => {
                    self.push_rest(s);
                    self.last_kept = None;
                }
                Candidate::Time(sample) => {
                    let Some(base) = sample.base.filter(|_| sample.t <= through) else {
                        break;
                    };
                    if base == 0.0 {
                        self.push_rest(sample.s);
                        self.last_kept = None;
                    } else {
                        self.keep_if_due(sample, base);
                    }
                }
            }
            self.candidates.pop_front();
            self.pending = self.pending.and_then(|pending| pending.checked_sub(1));
        }
        let needed_from = match self.candidates.front() {
            Some(Candidate::Time(sample)) => sample.t,
            _ => through,
        } - TARGET_DELAY_S;
        let drop = self.seams.partition_point(|seam| seam.t < needed_from);
        self.seams.drain(..drop);
        self.closed_seams = self.closed_seams.saturating_sub(drop);
    }

    /// Makes `sample` a node of target `base` scaled to the ramp time
    /// there, or the width the seams around ask for if wider, once it lies
    /// far enough past the last node kept in time and along the path.
    fn keep_if_due(&mut self, sample: TimeSample, base: f64) {
        let ramp_time = self.ramp_time_at(sample.t);
        let target =
            (base * ramp_time / RATIO_RAMP_MIN_TIME_S).max(self.seam_width_at(sample.t, sample.s));
        let due = self.last_kept.is_none_or(|last| {
            sample.t >= last.t + 0.5 * last.ramp_time - 0.5 * NODE_SPACING_S
                && sample.s - last.s >= NODE_SPACING_PER_WIDTH * target.min(last.target)
        });
        if due {
            self.push(sample.s, target);
            self.last_kept = Some(KeptNode {
                t: sample.t,
                s: sample.s,
                ramp_time,
                target,
            });
        }
    }

    /// A floor at `s`: the leaders rest there and nothing after reaches
    /// back, so the sample still waiting on the path ahead takes `s` as it,
    /// and every candidate becomes a node or is dropped.
    fn close(&mut self, s: f64) {
        if let Some(pending) = self.pending.take() {
            self.resolve(pending, s);
        }
        self.place_rest(s);
        self.assign(f64::INFINITY);
        self.closed_seams = self.seams.len();
    }

    /// Every ratio step laid has its node, as at a floor: a new rule
    /// derives only the ramps still to come.
    fn is_settled(&self) -> bool {
        self.candidates.is_empty() && self.seams.is_empty()
    }

    fn place_rest(&mut self, s: f64) {
        self.candidates.push_back(Candidate::Rest(s));
    }

    /// A rest past a node already at zero width keeps its own place rather
    /// than stacking on it ([`Self::push`]): `h` is zero between the two
    /// either way, and a leg seam the leaders cross at rest needs its rest
    /// node on the seam itself, or the window there, however narrow,
    /// straddles the seam and reads half its ratio step.
    ///
    /// Leaders that leave a rest and come back to one within a node spacing
    /// leave no time sample in between, and `h` would be zero all along the
    /// path they cover: a ratio step there would step the follower's
    /// velocity. A node halfway along it, its target half that path as a
    /// sample's base, opens the widest window the rests' cones allow.
    fn push_rest(&mut self, s: f64) {
        let Some(&last) = self.nodes.last() else {
            return self.push(s, 0.0);
        };
        if last.envelope != 0.0 {
            return self.push(s, 0.0);
        }
        if s - last.s > REST_TARGET_MM {
            self.push(0.5 * (last.s + s), 0.5 * (s - last.s));
            self.push(s, 0.0);
        } else if s > last.s {
            self.nodes.push(WidthNode {
                s,
                target: 0.0,
                envelope: 0.0,
            });
        }
    }

    /// A node within [`REST_TARGET_MM`] past the last one stacks with it:
    /// the leaders cover no more between them than the fit's back-and-forth
    /// around a rest, and a node a rounding short of a rest would take the
    /// rest's cone as a sliver of an envelope that the smootherstep
    /// flattens onto, narrowing every window short of the rest to as much.
    /// The stack sits where the leaders rest: a rest pulls the moving nodes
    /// stacked short of it onto itself, as the window must close exactly
    /// there whatever the leaders do after.
    fn push(&mut self, s: f64, target: f64) {
        let s = match self.nodes.last() {
            None => s,
            Some(last) => {
                assert!(
                    s >= last.s - SPAN_LOOKUP_SLACK_MM,
                    "a ratio ramp width node at {s} mm falls behind the last one at {} mm",
                    last.s
                );
                if s > last.s + REST_TARGET_MM {
                    s
                } else if target == 0.0 && s > last.s {
                    self.pull_moving_stack_onto_rest(s);
                    s
                } else {
                    last.s
                }
            }
        };
        self.nodes.push(WidthNode {
            s,
            target,
            envelope: target,
        });
        self.settle(self.nodes.len() - 1);
    }

    fn pull_moving_stack_onto_rest(&mut self, rest: f64) {
        let stacked_at = self.nodes.last().expect("a stack to pull").s;
        let moving = self
            .nodes
            .iter()
            .rev()
            .take_while(|node| node.s == stacked_at && node.envelope > 0.0)
            .count();
        let first_pulled = self.nodes.len() - moving;
        assert!(
            moving == 0 || first_pulled >= self.final_prefix,
            "a rest at {rest} mm would pull the final ratio ramp width node at {stacked_at} mm \
             onto itself"
        );
        for node in &mut self.nodes[first_pulled..] {
            node.s = rest;
        }
    }

    fn cone(&self, from: usize, to: usize) -> f64 {
        self.nodes[from].envelope + ENVELOPE_SLOPE * (self.nodes[to].s - self.nodes[from].s).abs()
    }

    fn settle(&mut self, index: usize) {
        let mut envelope = self.nodes[index].envelope.min(self.nodes[index].target);
        if index > 0 {
            envelope = envelope.min(self.cone(index - 1, index));
        }
        self.nodes[index].envelope = envelope;
        for back in (0..index).rev() {
            let lowered = self.cone(back + 1, back);
            if lowered >= self.nodes[back].envelope {
                break;
            }
            self.nodes[back].envelope = lowered;
        }
    }

    /// A node is final once no later node can lower it: every node still
    /// to come lies past the frontier, the first candidate, and its cone is
    /// at least the slope times the distance there.
    fn is_final(&self, index: usize) -> bool {
        let node = self.nodes[index];
        let frontier = self.candidates.front().map_or_else(
            || self.nodes.last().map_or(node.s, |last| last.s),
            Candidate::s,
        );
        node.envelope <= ENVELOPE_SLOPE * (frontier - node.s)
    }

    /// The width is final up to the last node of the final prefix.
    fn final_through(&mut self) -> Option<f64> {
        while self.final_prefix < self.nodes.len() && self.is_final(self.final_prefix) {
            self.final_prefix += 1;
        }
        self.final_prefix
            .checked_sub(1)
            .map(|last| self.nodes[last].s)
    }

    /// The node at `s`, or the two around it. A query within the span
    /// lookup slack past either end node reads that node, as the span table
    /// extends its own ends.
    fn interval(&self, s: f64) -> Result<usize, (usize, usize)> {
        let (first, last) = (self.nodes[0].s, self.nodes[self.nodes.len() - 1].s);
        assert!(
            s >= first - SPAN_LOOKUP_SLACK_MM && s <= last + SPAN_LOOKUP_SLACK_MM,
            "the ratio ramp width is asked for at {s} mm, outside its nodes [{first}, {last}]"
        );
        let after = self.nodes.partition_point(|node| node.s < s);
        if after == self.nodes.len() {
            return Ok(after - 1);
        }
        if self.nodes[after].s == s || after == 0 {
            return Ok(after);
        }
        Err((after - 1, after))
    }

    /// The formula of the width at `s`: on a node that has a node past it,
    /// the interval that node opens, whose derivatives it takes from its
    /// start; else [`Self::interval`]'s.
    fn formula_at(&self, s: f64) -> Result<usize, (usize, usize)> {
        match self.interval(s) {
            Ok(node)
                if self.nodes[node].s == s
                    && self.nodes.get(node + 1).is_some_and(|next| next.s > s) =>
            {
                Err((node, node + 1))
            }
            interval => interval,
        }
    }

    fn jet(&self, s: f64) -> Jet {
        let (a, b) = match self.formula_at(s) {
            Ok(node) => return Jet::constant(self.nodes[node].envelope),
            Err(interval) => interval,
        };
        let (a, b) = (self.nodes[a], self.nodes[b]);
        let x = Jet::variable(s).offset(-a.s).scale(1.0 / (b.s - a.s));
        match (a.envelope == 0.0, b.envelope == 0.0) {
            (true, true) => Jet::ZERO,
            (true, false) => x.poly(&RISE_FROM_REST).scale(b.envelope),
            (false, true) => x.poly(&FALL_TO_REST).scale(a.envelope),
            (false, false) => x
                .poly(&SMOOTHERSTEP)
                .scale(b.envelope - a.envelope)
                .offset(a.envelope),
        }
    }

    /// [`Self::jet`]'s value alone, by the same arithmetic.
    fn width(&self, s: f64) -> f64 {
        match self.interval(s) {
            Ok(node) => self.nodes[node].envelope,
            Err((a, _)) => self.width_after(a, s),
        }
    }

    /// The width at `s`, strictly between node `a` and the next.
    fn width_after(&self, a: usize, s: f64) -> f64 {
        let (a, b) = (self.nodes[a], self.nodes[a + 1]);
        let x = (s - a.s) * (1.0 / (b.s - a.s));
        match (a.envelope == 0.0, b.envelope == 0.0) {
            (true, true) => 0.0,
            (true, false) => horner(x, &RISE_FROM_REST) * b.envelope,
            (false, true) => horner(x, &FALL_TO_REST) * a.envelope,
            (false, false) => horner(x, &SMOOTHERSTEP) * (b.envelope - a.envelope) + a.envelope,
        }
    }

    /// [`Self::width_after`], by the same arithmetic, with the width's
    /// slope along the path.
    fn width_and_slope_after(&self, a: usize, s: f64) -> (f64, f64) {
        let (a, b) = (self.nodes[a], self.nodes[a + 1]);
        let per_s = 1.0 / (b.s - a.s);
        let x = (s - a.s) * per_s;
        let scaled = |coeffs: &[f64], scale: f64, offset: f64| {
            let (value, slope) = horner_with_slope(x, coeffs);
            (value * scale + offset, slope * scale * per_s)
        };
        match (a.envelope == 0.0, b.envelope == 0.0) {
            (true, true) => (0.0, 0.0),
            (true, false) => scaled(&RISE_FROM_REST, b.envelope, 0.0),
            (false, true) => scaled(&FALL_TO_REST, a.envelope, 0.0),
            (false, false) => scaled(&SMOOTHERSTEP, b.envelope - a.envelope, a.envelope),
        }
    }

    /// A reader of [`Self::width`] for a search whose queries close in on
    /// one point: it tries the node interval of its last query first.
    fn searcher(&self) -> WidthSearch<'_> {
        WidthSearch {
            field: self,
            last: std::cell::Cell::new(None),
        }
    }

    /// The nodes bounding `[lo, hi]` and every node between: the last node
    /// at or before `lo` through the first at or past `hi`. On nodes stacked
    /// at a rest the first at `hi == lo` comes before the last at `lo`, and
    /// the last at `lo` bounds both ends.
    fn covering(&self, lo: f64, hi: f64) -> &[WidthNode] {
        assert!(
            lo <= hi,
            "the ratio ramp width is bounded over [{lo}, {hi}] mm, a reversed stretch"
        );
        let first = self
            .nodes
            .partition_point(|node| node.s <= lo)
            .saturating_sub(1);
        let last = self.nodes.partition_point(|node| node.s < hi).max(first);
        &self.nodes[first..=last.min(self.nodes.len() - 1)]
    }

    /// `(max h, max |h''|)` over `[lo, hi]`.
    fn bounds(&self, lo: f64, hi: f64) -> (f64, f64) {
        let nodes = self.covering(lo, hi);
        let widest = nodes.iter().map(|node| node.envelope).fold(0.0, f64::max);
        let curvature = nodes
            .windows(2)
            .filter(|pair| pair[1].s > pair[0].s)
            .map(|pair| {
                let span = pair[1].s - pair[0].s;
                let (a, b) = (pair[0].envelope, pair[1].envelope);
                if a == 0.0 || b == 0.0 {
                    REST_PEAK_CURVATURE * a.max(b) / (span * span)
                } else {
                    SMOOTHERSTEP_PEAK_CURVATURE * (b - a).abs() / (span * span)
                }
            })
            .fold(0.0, f64::max);
        (widest, curvature)
    }

    fn prune_before(&mut self, s: f64) {
        let keep = self
            .nodes
            .partition_point(|node| node.s <= s)
            .saturating_sub(1)
            .min(self.final_prefix.saturating_sub(1));
        self.nodes.drain(..keep);
        self.final_prefix -= keep;
    }
}

/// See [`WidthField::searcher`].
struct WidthSearch<'a> {
    field: &'a WidthField,
    last: std::cell::Cell<Option<usize>>,
}

impl WidthSearch<'_> {
    /// [`WidthField::width`]: strictly inside the last query's node
    /// interval, the interval search would find it again.
    fn width(&self, s: f64) -> f64 {
        let nodes = &self.field.nodes;
        if let Some(a) = self.last.get() {
            if nodes[a].s < s && s < nodes[a + 1].s {
                return self.field.width_after(a, s);
            }
        }
        match self.field.interval(s) {
            Ok(node) => nodes[node].envelope,
            Err((a, _)) => {
                self.last.set(Some(a));
                self.field.width_after(a, s)
            }
        }
    }

    /// [`Self::width`] with its slope along the path, read as flat on a
    /// node.
    fn width_and_slope(&self, s: f64) -> (f64, f64) {
        let nodes = &self.field.nodes;
        if let Some(a) = self.last.get() {
            if nodes[a].s < s && s < nodes[a + 1].s {
                return self.field.width_and_slope_after(a, s);
            }
        }
        match self.field.interval(s) {
            Ok(node) => (nodes[node].envelope, 0.0),
            Err((a, _)) => {
                self.last.set(Some(a));
                self.field.width_and_slope_after(a, s)
            }
        }
    }
}

/// The extrusion ratio a ratio-ramped follower lays: the span table's
/// ratio `r(s)` smoothed along the path by the kernel `K(u) = ¾(1 − u²)`
/// of a half-width `h(s)` the leaders' speed sets ([`WidthField`]), so
/// that every step of the table — in the ratio or in its slope, at a seam
/// or across a cluster of seams however short — is spread over the path
/// the leaders cover in about `τ` around it, and an advance applied to it
/// emits no position or velocity step.
///
/// The smoothing acts on the cumulative extrusion `E` (`E' = r`):
///
/// `Ẽ(s) = ∫ E(s − h(s)·u)·K(u) du`,  `r̃ = Ẽ' = ∫ r(s − h·u)·(1 − h'·u)·K(u) du`
///
/// - Exact material: a symmetric kernel reproduces a linear `E`, so
///   `Ẽ = E` wherever the ratio is constant across the window — after
///   every transition and at every rest, where `h = 0`. Along a sloped
///   stretch the track leads by `0.1·h²·r'` and gives it back where the
///   slope ends.
/// - Range: `|h'| < 1` keeps the weights `(1 − h'u)·K(u)` nonnegative and
///   of unit sum, so `r̃` stays within the ratios the table spans across
///   the window.
/// - C1: `K` vanishes at its ends and `h` is C2, so `r̃'` is continuous;
///   `r̃''` steps where a seam enters or leaves a window.
/// - Span-independent: `Ẽ` is a functional of `E` alone and `h` of the
///   leaders' motion alone, so cutting a span into pieces of equal ratio
///   changes nothing beyond rounding.
///
/// With the table's seams `σᵢ` (ratio step `Δᵢ`, slope step `Δmᵢ`) and
/// `x = (s − σ)/h`, the correction `X = Ẽ − E` is
///
/// `X = 0.1·h²·m₋ + Σ_{|x|<1} Δ·h·G₁(x) + Δm·h²·G₂(x)`
///
/// with `m₋` the table's slope at `s − h` and `G₁ = P₁ − x₊`,
/// `G₂ = P₂ − x₊²/2` the kernel's smoothing of a ramp and a parabola
/// ([`SMOOTHED_RAMP`], [`SMOOTHED_HALF_SQUARE`]). Every `Gᵢ` and its first
/// two derivatives vanish at `x = −1`, and `G₂ → 0.1` at `x = 1` where the
/// seam's slope step joins `m₋`, so `X` is C2 in `s`: the ratio `r + X'`
/// is C1.
///
/// Bounds, `λ = WIDTH_SLOPE_BOUND`: a ratio step `Δ` crossed at path speed
/// `v` adds `k·r̃'·v²` to an advanced extruder velocity, with
///
/// `|r̃'| ≤ (1 + λ)²·(max|r'| + ¾·|Δ|/h) + 0.375·|h''|·(r_max − r_min)`,
///
/// `h ≥ h(σ)/(1 + λ)` wherever the seam is inside a window. At cruise
/// `h = v·τ/2` and `h' = 0`, so the step adds `1.5·k·|Δ|·v/τ` to the
/// extruder velocity and `6·k·|Δ|·v/τ²` to its acceleration, whether the
/// step is one seam or a cluster of seams shorter than the window.
#[derive(Debug, Default)]
pub(super) struct RatioSmoothing {
    width: WidthField,
    seams: Vec<Seam>,
    base_slope: Option<f64>,
    valleys: Vec<AdvanceValley>,
    open_rest: Option<OpenRest>,
    examined_through: f64,
    floor: f64,
    taken_through: f64,
    reach_into_next: Option<Crossing>,
    laid_tails: VecDeque<LegTail>,
    backlog: VecDeque<PendingSegment>,
}

impl RatioSmoothing {
    pub(super) fn clear(&mut self) {
        *self = Self::default();
    }

    /// Nothing to smooth: the table has no seam and no slope.
    pub(super) fn is_inert(&self) -> bool {
        self.seams.is_empty() && self.base_slope.unwrap_or(0.0) == 0.0
    }

    /// No window may reach back past `s`: the leaders rest there, or the
    /// stream drained there and what precedes it is already committed. A
    /// rest valley still open closes there, with nothing after it.
    pub(super) fn close_at(&mut self, s: f64) {
        self.floor = self.floor.max(s);
        self.reach_into_next = None;
        self.laid_tails.clear();
        self.close_rest(s, 0.0);
        self.width.close(s);
    }

    /// A rest valley opens with no end: the advance reads nothing from its
    /// start on until a leg that moves again closes it.
    fn close_rest(&mut self, end: f64, after: f64) {
        if self.open_rest.take().is_none() {
            return;
        }
        let valley = self
            .valleys
            .last_mut()
            .expect("an open rest valley is the last laid");
        valley.end = end.max(valley.start);
        valley.after = after;
    }

    /// Takes in `leg`'s spans: the width nodes it covers, the seams the
    /// table gained with the ramp time `rule` derives for each, and the
    /// advance valleys its opening seam and its slowest point inside need.
    pub(super) fn lay(&mut self, spans: &[RatioSpan], leg: &RampLeg, rule: &RatioRampRule) {
        assert!(
            self.width.rule.is_none_or(|laid| laid == *rule) || self.width.is_settled(),
            "a follower's ratio ramp rule changed from {:?} to {rule:?} with ratio steps still \
             waiting on their ramp times",
            self.width.rule
        );
        self.width.rule = Some(*rule);
        let leaders = leg.shaped_leaders;
        let table_end = spans.last().map_or(leg.start_s, |last| last.s1);
        if table_end <= leg.start_s {
            self.width.extend(leaders, leg, table_end);
            self.close_at(leg.start_s);
            return;
        }
        if self.base_slope.is_none() {
            self.base_slope = Some(spans[0].ratio_slope());
        }
        let (t_start, t_end) = leg.support;
        let opening = Crossing::valley_at(leaders, t_start);
        let crossing = self
            .reach_into_next
            .map_or(opening, |closing| closing.common(opening));
        let seam = leg.start_s;
        let reach = crossing.reach(VALLEY_RAMP_TIME_S).min(seam - self.floor);
        if reach <= SPAN_MIN_LEN_MM {
            self.width.place_rest(seam);
        }
        self.width.extend(leaders, leg, table_end);
        let closes_a_rest = self.lay_rest_valley(spans, leg, rule, crossing, table_end);
        if reach > SPAN_MIN_LEN_MM
            && self.open_rest.is_none()
            && !closes_a_rest
            && crossing.needs_valley(rule, peak_ratio_within(spans, seam, reach))
        {
            self.lay_valley(seam, reach, crossing.crawl_floor());
        }
        let slowest = (!leg.rests && self.open_rest.is_none())
            .then(|| slowest_crawl(leaders, leg, table_end))
            .flatten();
        if let Some((turn, turn_crossing)) = slowest {
            let reach = turn_crossing
                .reach(VALLEY_RAMP_TIME_S)
                .min(turn - self.floor);
            if reach > SPAN_MIN_LEN_MM
                && turn_crossing.needs_valley(rule, peak_ratio_within(spans, turn, reach))
            {
                self.lay_valley(turn, reach, turn_crossing.crawl_floor());
            }
        }
        let new_seams = self.record_seams(spans);
        self.width.widen_seams_into(leaders, leg.support);
        self.time_seams(&spans[new_seams.start.saturating_sub(1)..], leaders, leg);
        self.keep_tail(leaders, leg);
        self.width.assign(t_end - TARGET_DELAY_S);
        self.reach_into_next = Some(Crossing::valley_at(leaders, t_end));
    }

    /// A valley at `turn`, unless the last one laid sits there already: a
    /// crawl at a seam is the slowest point of the leg before or after it
    /// too.
    fn lay_valley(&mut self, turn: f64, reach: f64, floor: f64) {
        let laid_there = self
            .valleys
            .last()
            .is_some_and(|valley| (valley.end - turn).abs() <= SPAN_LOOKUP_SLACK_MM);
        if laid_there {
            return;
        }
        self.valleys.push(AdvanceValley {
            start: turn,
            end: turn,
            before: reach.min(self.back_reach_limit(turn)),
            after: reach,
            floor,
        });
    }

    /// Records the seams of the spans not yet examined, and returns the
    /// range of those spans.
    fn record_seams(&mut self, spans: &[RatioSpan]) -> std::ops::Range<usize> {
        let first = spans
            .partition_point(|span| span.s0 <= self.examined_through)
            .max(1);
        for index in first..spans.len() {
            let (before, after) = (&spans[index - 1], &spans[index]);
            self.examined_through = after.s0;
            if let Some(seam) = Seam::between(before, after) {
                self.seams.push(seam);
            }
        }
        first..spans.len()
    }

    /// Hands the width field the seams between consecutive `spans`, timed
    /// where the leaders of `leg` cross them, with how they cross the ramp
    /// on either side: within `leg`, and over the tails of the legs before
    /// it the ramp reaches back into.
    fn time_seams(&mut self, spans: &[RatioSpan], leaders: &ShapedLeaders, leg: &RampLeg) {
        let path = LegPath::of(leaders, leg);
        for pair in spans.windows(2) {
            let (before, after) = (&pair[0], &pair[1]);
            let Some(seam) = Seam::between(before, after) else {
                continue;
            };
            let t = path.time_at(after.s0);
            let crossing = self
                .laid_tails
                .iter()
                .filter(|tail| tail.until > t - CLUSTER_REACH_S)
                .fold(
                    Crossing::over(leaders, ramp_reach_within(t, leg.support)),
                    |widest, tail| widest.widest(tail.crossing),
                );
            self.width.add_seam(TimedSeam {
                s: after.s0,
                t,
                crossing,
                step: seam.step.abs() + (after.r1 - after.r0).abs(),
                peak_ratio: before.r1.abs().max(after.r0.abs()).max(after.r1.abs()),
                ramp_time: None,
            });
        }
    }

    /// Remembers how the leaders of `leg` cross its last half of the longest
    /// ramp, which a ramp laid on a later leg may reach back into.
    fn keep_tail(&mut self, leaders: &ShapedLeaders, leg: &RampLeg) {
        let (t_start, t_end) = leg.support;
        self.laid_tails
            .retain(|tail| tail.until > t_end - CLUSTER_REACH_S);
        self.laid_tails.push_back(LegTail {
            until: t_end,
            crossing: Crossing::over(leaders, ((t_end - CLUSTER_REACH_S).max(t_start), t_end)),
        });
    }

    pub(super) fn queue(
        &mut self,
        t_end: f64,
        (s_start, s_length): (f64, f64),
        table_end: f64,
        reads: LeaderReads,
    ) {
        self.backlog.push_back(PendingSegment {
            t_end,
            s_start,
            s_length,
            reads,
            table_end,
        });
    }

    /// The queued segments no node, seam or valley still to come can reach
    /// into, in order.
    pub(super) fn take_final(&mut self, spans: &[RatioSpan]) -> Vec<PendingSegment> {
        let table_end = spans.last().map_or(self.floor, |last| last.s1);
        let final_through = self
            .final_through(table_end)
            .min(table_end - self.unlaid_valley_reach(table_end))
            .max(self.floor);
        let ready = self
            .backlog
            .iter()
            .take_while(|segment| {
                segment.table_end <= self.floor
                    || segment.table_end + SPAN_LOOKUP_SLACK_MM <= final_through
            })
            .count();
        let drained: Vec<PendingSegment> = self.backlog.drain(..ready).collect();
        if let Some(last) = drained.last() {
            self.taken_through = self
                .taken_through
                .max(last.table_end)
                .max(last.s_start + last.s_length);
        }
        drained
    }

    /// How far back from `seam` a valley laid there may reach: not past a
    /// floor, and not into a segment already taken for projection, whose
    /// advance read the weight before the valley existed. The reserve
    /// [`Self::unlaid_valley_reach`] keeps this from binding where the
    /// leaders' reach changes slowly; a seam whose crossing reaches much
    /// further than the one the reserve was taken at (hard braking into a
    /// rest right after a cruise) releases the advance over what is left.
    fn back_reach_limit(&self, seam: f64) -> f64 {
        (seam - self.floor.max(self.taken_through)).max(0.0)
    }

    /// How far back from the table's end a valley the next leg may open
    /// there reaches: a rest or a turnaround releases the advance over the
    /// leaders' reach at that seam, which is no further than the reach of
    /// the crossing the last leg closed on.
    fn unlaid_valley_reach(&self, table_end: f64) -> f64 {
        self.reach_into_next.map_or(0.0, |closing| {
            closing
                .reach(VALLEY_RAMP_TIME_S)
                .min(table_end - self.floor)
        })
    }

    /// How far the smoothed ratio is final: its width there is, and every
    /// window reaches no further than the table.
    fn final_through(&mut self, table_end: f64) -> f64 {
        let Some(width_final) = self.width.final_through() else {
            return self.floor;
        };
        let width = self.width.searcher();
        let fits = |s: f64| s + width.width(s) <= table_end;
        let mut hi = width_final.min(table_end);
        if fits(hi) {
            return hi;
        }
        let mut lo = self.width.nodes[0].s.max(self.floor);
        if !fits(lo) {
            return self.floor;
        }
        for _ in 0..WIDTH_BISECTIONS {
            let mid = 0.5 * (lo + hi);
            if mid == lo || mid == hi {
                break;
            }
            if fits(mid) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        lo
    }

    /// `X = Ẽ − E` at `s` with its first three path derivatives: the
    /// extrusion, ratio, slope and curvature the smoothing adds to the
    /// table's, summed over the seams in the window and the spans between
    /// them by their moments ([`SeamMoments`]), and over the spans running
    /// past its edges ([`edge_spans`]).
    pub(super) fn correction(&self, s: f64) -> Jet {
        if self.is_inert() {
            return Jet::ZERO;
        }
        let h = self.width.jet(s);
        if h.value <= 0.0 {
            return Jet::ZERO;
        }
        let (lo, hi) = (s - h.value, s + h.value);
        let first = self.seams.partition_point(|seam| seam.s <= lo);
        let slope_left = first
            .checked_sub(1)
            .map_or(self.base_slope.unwrap_or(0.0), |last| {
                self.seams[last].slope_after
            });
        let window = self.seams_near(lo, hi);
        let mut moments = SeamMoments::default();
        for seam in window {
            moments.add_step(s - seam.s, seam.step);
        }
        for pair in window.windows(2) {
            let (seam, next) = (pair[0], pair[1]);
            moments.add_span((s - seam.s, s - next.s), seam.slope_after, next.s - seam.s);
        }
        let inverse = h.recip();
        let ends = window
            .first()
            .zip(window.last())
            .map(|(first_seam, last_seam)| {
                (s - first_seam.s, s - last_seam.s, last_seam.slope_after)
            });
        moments.correction(h, inverse) + edge_spans(h, inverse, slope_left, ends)
    }

    fn seams_near(&self, lo: f64, hi: f64) -> &[Seam] {
        let first = self.seams.partition_point(|seam| seam.s <= lo);
        let last = self.seams.partition_point(|seam| seam.s < hi);
        &self.seams[first..last.max(first)]
    }

    /// The table's slope just past `s`, as the seams laid so far leave it.
    fn slope_left_of(&self, s: f64) -> f64 {
        self.seams
            .partition_point(|seam| seam.s <= s)
            .checked_sub(1)
            .map_or(self.base_slope.unwrap_or(0.0), |last| {
                self.seams[last].slope_after
            })
    }

    /// Whether the smoothing changes anything over `[lo, hi]`: the table
    /// slopes where the windows there open, or a window there reaches a
    /// seam. `s ± h(s)` grows along the path (`|h'| < 1`), so a seam
    /// outside the stretch is reached from the stretch's end nearer it, if
    /// at all, and one inside from itself, unless the leaders rest on it,
    /// where no window reaches it. Only the width over the stretch decides,
    /// never what is laid past it.
    pub(super) fn touches(&self, lo: f64, hi: f64) -> bool {
        if self.is_inert() {
            return false;
        }
        let (back, ahead) = (lo - self.reach_back(lo), hi + self.reach_back(hi));
        if self.slope_left_of(back) != 0.0 {
            return true;
        }
        let (widest, _) = self.width.bounds(lo, hi);
        self.seams_near(lo - widest, hi + widest)
            .iter()
            .any(|seam| {
                if seam.s < lo {
                    seam.s > back
                } else if seam.s > hi {
                    seam.s < ahead
                } else {
                    self.reach_back(seam.s) > 0.0
                }
            })
    }

    /// `(max |r̃|, max |r̃'|)` over `[lo, hi]`, see [`RatioSmoothing`].
    pub(super) fn ratio_bounds(&self, spans: &[RatioSpan], lo: f64, hi: f64) -> (f64, f64) {
        let (widest, curvature) = self.width.bounds(lo, hi);
        let (reach_lo, reach_hi) = (
            lo - widest - SPAN_LOOKUP_SLACK_MM,
            hi + widest + SPAN_LOOKUP_SLACK_MM,
        );
        let ridden = spans
            .iter()
            .filter(|span| span.s1 >= reach_lo && span.s0 <= reach_hi);
        let (low, high, slope) = ridden.fold(
            (f64::INFINITY, f64::NEG_INFINITY, 0.0_f64),
            |(low, high, slope), span| {
                (
                    low.min(span.r0.min(span.r1)),
                    high.max(span.r0.max(span.r1)),
                    slope.max(span.ratio_slope().abs()),
                )
            },
        );
        let (low, high) = match spans.last() {
            Some(last) if reach_hi > last.s1 => (low.min(last.r1), high.max(last.r1)),
            _ => (low, high),
        };
        if low > high {
            return (0.0, 0.0);
        }
        let stretch = (1.0 + WIDTH_SLOPE_BOUND) * (1.0 + WIDTH_SLOPE_BOUND);
        let steps = self
            .seams_near(reach_lo, reach_hi)
            .iter()
            .map(|seam| {
                let narrowest = self.narrowest_window_over(seam.s);
                if narrowest > 0.0 {
                    KERNEL_PEAK * seam.step.abs() / narrowest
                } else {
                    0.0
                }
            })
            .sum::<f64>();
        (
            low.abs().max(high.abs()),
            stretch * (slope + steps) + KERNEL_ABS_MOMENT * curvature * (high - low),
        )
    }

    /// A lower bound of `h(s)` wherever `seam` is inside the window at `s`:
    /// `h(s) > h(seam)/(1 + λ)` by the width's slope bound, and `h(s)`
    /// exceeds the distance to the seam, so a seam past the nodes is no
    /// nearer than the last of them.
    fn narrowest_window_over(&self, seam: f64) -> f64 {
        let nodes = &self.width.nodes;
        let (first, last) = (nodes[0].s, nodes[nodes.len() - 1].s);
        if seam < first {
            first - seam
        } else if seam > last {
            seam - last
        } else {
            self.width.width(seam) / (1.0 + WIDTH_SLOPE_BOUND)
        }
    }

    /// Where over `[lo, hi]` the smoothed ratio's curvature, or the
    /// advance's read weight, bends: where each seam enters and leaves a
    /// window, the width nodes, and the valley ends. A stretch the smoothing
    /// does not touch bends only at valley ends, whatever the smoothing
    /// holds elsewhere. A window edge lies on its side of the seam clamped
    /// to the nodes, so one that side puts outside `[lo, hi]` is not
    /// searched for.
    pub(super) fn bends(&self, lo: f64, hi: f64) -> Vec<f64> {
        let mut bends: Vec<f64> = self.valley_ends(lo, hi).collect();
        if !self.touches(lo, hi) {
            return bends;
        }
        let (widest, _) = self.width.bounds(lo, hi);
        bends.extend(self.width.covering(lo, hi).iter().map(|node| node.s));
        let (first, last) = (
            self.width.nodes[0].s,
            self.width.nodes[self.width.nodes.len() - 1].s,
        );
        for seam in self.seams_near(lo - widest, hi + widest) {
            let anchor = seam.s.clamp(first, last);
            if anchor >= lo {
                bends.push(self.window_edge(seam.s, -1.0, widest));
            }
            if anchor <= hi {
                bends.push(self.window_edge(seam.s, 1.0, widest));
            }
        }
        bends
    }

    /// The `s` on `side` of `seam` where `s ∓ h(s) = seam`: `s ± h(s)` is
    /// increasing since `|h'| < 1`. A safeguarded Newton closes in on it,
    /// and the float it flips at is bracketed from there and bisected for.
    fn window_edge(&self, seam: f64, side: f64, widest: f64) -> f64 {
        let width = self.width.searcher();
        let reach = |s: f64| (s - seam) * side - width.width(s);
        let (first, last) = (
            self.width.nodes[0].s,
            self.width.nodes[self.width.nodes.len() - 1].s,
        );
        let mut near = seam.clamp(first, last);
        let mut far = (seam + side * (widest / (1.0 - WIDTH_SLOPE_BOUND))).clamp(first, last);
        if reach(far) < 0.0 || reach(near) >= 0.0 {
            return near;
        }
        let strictly_between =
            |s: f64, near: f64, far: f64| (s - near) * side > 0.0 && (far - s) * side > 0.0;
        let mut s = near;
        for _ in 0..WIDTH_BISECTIONS {
            let (h, slope) = width.width_and_slope(s);
            let reached = (s - seam) * side - h;
            if reached < 0.0 {
                near = s;
            } else {
                far = s;
            }
            let newton = s - reached / (side - slope);
            let ulp = (trajectory::next_toward(s, f64::INFINITY) - s).abs();
            if (newton - s).abs() <= NEWTON_SETTLED_ULPS * ulp {
                break;
            }
            s = if strictly_between(newton, near, far) {
                newton
            } else {
                0.5 * (near + far)
            };
        }
        let s_reached = s == far;
        let mut step = trajectory::next_toward(s, if s_reached { near } else { far }) - s;
        loop {
            let probe = s + step;
            if !strictly_between(probe, near, far) {
                break;
            }
            let probe_reached = reach(probe) >= 0.0;
            if probe_reached {
                far = probe;
            } else {
                near = probe;
            }
            if probe_reached != s_reached {
                break;
            }
            step *= 2.0;
        }
        for _ in 0..WIDTH_BISECTIONS {
            let mid = 0.5 * (near + far);
            if mid == near || mid == far {
                break;
            }
            if reach(mid) < 0.0 {
                near = mid;
            } else {
                far = mid;
            }
        }
        far
    }

    /// How far back from `s` its window averages the table.
    pub(super) fn reach_back(&self, s: f64) -> f64 {
        match (self.width.nodes.first(), self.width.nodes.last()) {
            (Some(first), Some(last)) if s >= first.s => self.width.width(s.min(last.s)),
            _ => 0.0,
        }
    }

    pub(super) fn prune_before(&mut self, s: f64) {
        self.valleys.retain(|valley| valley.reach_end() >= s);
        let (Some(first), Some(last)) = (self.width.nodes.first(), self.width.nodes.last()) else {
            return;
        };
        let s = s.min(last.s);
        if first.s > s {
            return;
        }
        self.width.prune_before(s);
        let back_edge = s - self.width.width(s);
        let drop = self.seams.partition_point(|seam| seam.s <= back_edge);
        if drop > 0 {
            self.base_slope = Some(self.seams[drop - 1].slope_after);
            self.seams.drain(..drop);
        }
    }

    /// Within a leg whose raw leaders rest, the shaped leaders rest where
    /// their speed is down to [`REST_SPEED_MM_S`], and the table steps the
    /// ratio from the leg's tail to its lead where their distance from the
    /// rest point puts the rest ([`super::FollowerState::ingest`]), which
    /// the odometer can reach while they still creep toward it. The width
    /// rests there all the same, so the ratio step stays a step that any
    /// weight above zero hands the advance as a position step. A rest
    /// valley takes in both: it opens at the first, unless one is open
    /// already, and the first leg that moves again closes it at the last.
    /// Each side spans the shaped spill between that rest and the raw seam,
    /// over which the advance builds or releases its lead from the rest out,
    /// and reaches past the seam only where the lead or a ratio step near
    /// the rest needs it ([`RestSide::width`]). A rest leg the leaders cross
    /// without coming to rest opens none. Returns whether `leg` closes a
    /// rest valley.
    fn lay_rest_valley(
        &mut self,
        spans: &[RatioSpan],
        leg: &RampLeg,
        rule: &RatioRampRule,
        crossing: Crossing,
        table_end: f64,
    ) -> bool {
        let seam = leg.start_s;
        let leaders = leg.shaped_leaders;
        let (t_start, t_end) = leg.support;
        if !leg.rests {
            let Some(open) = self.open_rest else {
                return false;
            };
            let side = RestSide {
                rest: open.end,
                seam,
                outward: 1.0,
                crossing,
                spill: open.departure,
                beyond: Crossing::over(leaders, (t_start, (t_start + CLUSTER_REACH_S).min(t_end))),
            };
            self.close_rest(open.end, side.width(spans, rule, table_end));
            return true;
        }
        let Some(resting) = resting_stretch(leaders, leg, REST_SPEED_MM_S, table_end) else {
            return false;
        };
        let (rest_start, rest_end) = ratio_steps_within(spans, seam, table_end).map_or(
            (resting.first_s, resting.last_s),
            |(first_step, last_step)| {
                (
                    resting.first_s.min(first_step),
                    resting.last_s.max(last_step),
                )
            },
        );
        if self.open_rest.is_none() {
            let side = RestSide {
                rest: rest_start,
                seam,
                outward: -1.0,
                crossing,
                spill: Spill::over(leaders, (t_start, resting.first_t)),
                beyond: self
                    .laid_tails
                    .iter()
                    .fold(crossing, |widest, tail| widest.widest(tail.crossing)),
            };
            let before = side
                .width(spans, rule, table_end)
                .min(rest_start - seam + self.back_reach_limit(seam));
            self.valleys.push(AdvanceValley {
                start: rest_start,
                end: f64::INFINITY,
                before,
                after: 0.0,
                floor: 0.0,
            });
        }
        self.open_rest = Some(OpenRest {
            end: rest_end,
            departure: Spill::over(leaders, (resting.last_t, t_end)),
        });
        false
    }

    /// The weight `W` the advance reads the ratio with at `s`, with its
    /// first two path derivatives: 1 away from every valley. Valleys reach
    /// as far as their seams' crossings, so neighbours may overlap in any
    /// order; their weights multiply.
    pub(super) fn valley_weight(&self, s: f64) -> (f64, f64, f64) {
        self.valleys_over(s, s)
            .fold((1.0, 0.0, 0.0), |(w, w1, w2), valley| {
                let (v, v1, v2) = valley.weight(s);
                (w * v, w1 * v + w * v1, w2 * v + 2.0 * w1 * v1 + w * v2)
            })
    }

    fn valleys_over(&self, lo: f64, hi: f64) -> impl Iterator<Item = &AdvanceValley> + '_ {
        self.valleys
            .iter()
            .filter(move |valley| valley.reach_start() < hi && valley.reach_end() > lo)
    }

    /// Where the valleys overlapping `[lo, hi]` start and end bending.
    fn valley_ends(&self, lo: f64, hi: f64) -> impl Iterator<Item = f64> + '_ {
        self.valleys_over(lo, hi).flat_map(|valley| {
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
/// weighted down to zero: `W = 0` on `[start, end]`, where the leaders'
/// path speed is zero or a crawl, and `W = ψ(x) = 1 − (1 − x)³` within
/// `before`/`after` of it, meeting 1 flat to second order. `x` is the
/// distance `d` out over that width, both flattened over the valley's
/// `floor` `f` as `√(d² + f²) − f`: with no floor, where the leaders rest
/// or turn around, the weight rises off zero with a slope their zero speed
/// makes harmless; across a crawl at speed `v` and acceleration `a`, the
/// floor is `v²/a`, about where the acceleration takes over from the
/// crawl, and the weight leaves zero flat, since `W'·ṡ²` of opposite signs
/// either side would step the advanced velocity. The laid ratio, and so the
/// material, is untouched; only the lead the advance builds is released
/// over the path before and rebuilt after.
///
/// Rising from a rest the leaders leave at acceleration `a`, the lead
/// `k·r·W·ṡ` catches up with the advance's `k·r·ṡ` at a rate of at most
/// `1.653·k·r·a` (the peak of `d/du (u·ψ(u²))`): the advance's natural
/// rate, overshot by what catching up from zero takes, and over a shaped
/// start whose acceleration builds from zero, much less. A crawl's floor
/// keeps that, since the leaders cover it within about `v/a`.
///
/// - A turnaround: the leaders' path speed passes through zero at a seam
///   while its rate does not, so the rate steps from `−a` to `+a`, and an
///   advance reading `r·ṡ` would step the follower's velocity by `2·k·r·a`
///   whatever the ratio's own shape. The valley is that seam, each side
///   reaching as far as the leaders turn around in half the ramp time.
/// - A crawl: the leaders' slowest point, under
///   [`ADVANCE_VALLEY_SPEED_MM_S`], at a seam or inside a leg
///   ([`slowest_crawl`]). The path speed's second derivative carries the
///   tangent's turning there, `a⊥²/v`, which an advance would turn into
///   follower acceleration however the planner bounds the path's own. Only
///   where that leaves the follower's limit ([`Crossing::needs_valley`]):
///   releasing the lead asks for some too.
/// - A rest: an extrude-only move or a dwell, where the shaped leaders come
///   to rest between the kernel spill of the move before and of the move
///   after, and the fit's back-and-forth near the rest point has its own
///   turnarounds and short idle spans an advance would turn into steps. The
///   valley is the stretch where the leaders rest, each side spanning the
///   spill, and past the raw seam only where the lead or a ratio step near
///   the rest needs it ([`RestSide::width`]).
#[derive(Debug, Clone, Copy)]
struct AdvanceValley {
    start: f64,
    end: f64,
    before: f64,
    after: f64,
    floor: f64,
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
        let floor = self.floor.min(width);
        let flattened = |d: f64| d * d / (libm::hypot(d, floor) + floor);
        let spread = libm::hypot(distance, floor);
        let slope = distance / spread;
        let bend = floor / spread * (floor / spread) / spread;
        let span = flattened(width);
        let inside = 1.0 - flattened(distance) / span;
        (
            1.0 - inside * inside * inside,
            outward * 3.0 * inside * inside * slope / span,
            -6.0 * inside * slope * slope / (span * span) + 3.0 * inside * inside * bend / span,
        )
    }
}

/// A rest valley laid up to the last rest leg so far: where the leaders
/// last rest, and the spill they leave it over.
#[derive(Debug, Clone, Copy)]
struct OpenRest {
    end: f64,
    departure: Spill,
}

/// The shaped spill between a raw seam and where the leaders rest beside
/// it: how long they take over it, and how they cross it.
#[derive(Debug, Clone, Copy)]
struct Spill {
    duration: f64,
    crossing: Crossing,
}

impl Spill {
    fn over(leaders: &ShapedLeaders, (from, until): (f64, f64)) -> Self {
        Self {
            duration: until - from,
            crossing: Crossing::over(leaders, (from, until)),
        }
    }
}

/// One side of a rest valley: where the leaders rest and the raw seam
/// beside it, `outward` from the rest along the path (−1 before it, 1
/// after), how they cross that seam, the spill between, and how they
/// cross the path past the seam the side may reach.
#[derive(Debug, Clone, Copy)]
struct RestSide {
    rest: f64,
    seam: f64,
    outward: f64,
    crossing: Crossing,
    spill: Spill,
    beyond: Crossing,
}

impl RestSide {
    /// How far from the rest the side spans: the spill, and past the seam
    /// - as far as the longest ramp reaches where a ratio step within that
    ///   reach lies so near the rest that the window the width field narrows
    ///   to there ramps it beyond `rule`'s limits
    ///   ([`Self::narrows_a_step`]), or where the table does not reach that
    ///   far yet: the valley keeps the advance from reading it;
    /// - else nowhere where the shortest ramp `rule` allows for releasing or
    ///   building the lead of an advance on the ratio there, a step of that
    ///   whole ratio to the advance, fits within the spill, which then
    ///   carries the whole lead up to the rest;
    /// - else as far as the leaders cover at the seam in half that ramp,
    ///   read as they cross the whole reach. A spill that takes no time — a
    ///   leader stopping without a kernel, its acceleration stepping at the
    ///   rest — always reaches out.
    fn width(self, spans: &[RatioSpan], rule: &RatioRampRule, table_end: f64) -> f64 {
        let spill_mm = (self.seam - self.rest).abs();
        let longest = self.crossing.reach(VALLEY_RAMP_TIME_S);
        let far = self.seam + self.outward * longest;
        let reach = (self.rest.min(far), self.rest.max(far));
        if far > table_end || self.narrows_a_step(spans, rule, reach) {
            return spill_mm + longest;
        }
        let ratio = peak_ratio_within(spans, self.seam, spill_mm + longest);
        let lead_ramp_time = |leaders: Crossing| {
            rule.ramp_time(&RatioStep {
                ratio_step: ratio,
                peak_ratio: ratio,
                speed_mm_s: leaders.speed,
                accel_mm_s2: leaders.rate,
                jerk_mm_s3: leaders.jerk,
            })
        };
        let over_spill = self.crossing.widest(self.spill.crossing);
        if lead_ramp_time(over_spill) <= self.spill.duration {
            return spill_mm;
        }
        spill_mm
            + self
                .crossing
                .reach(lead_ramp_time(over_spill.widest(self.beyond)))
    }

    /// Whether the table steps its ratio or its slope inside `(from, to)`
    /// so near the rest that the width field, whose envelope falls off
    /// toward the rest by [`ENVELOPE_SLOPE`], ramps the step over less time
    /// than `rule`'s limits allow at the fastest and most accelerating the
    /// leaders cross the side.
    fn narrows_a_step(
        self,
        spans: &[RatioSpan],
        rule: &RatioRampRule,
        (from, to): (f64, f64),
    ) -> bool {
        let leaders = self
            .crossing
            .widest(self.spill.crossing)
            .widest(self.beyond);
        if leaders.speed == 0.0 {
            return false;
        }
        let first = spans.partition_point(|span| span.s0 <= from).max(1);
        spans[first - 1..]
            .windows(2)
            .take_while(|pair| pair[1].s0 < to)
            .any(|pair| {
                let (before, after) = (&pair[0], &pair[1]);
                let Some(seam) = Seam::between(before, after) else {
                    return false;
                };
                let window = ENVELOPE_SLOPE * (seam.s - self.rest).abs();
                let (velocity, accel) = rule.demand(
                    2.0 * window / leaders.speed,
                    &RatioStep {
                        ratio_step: seam.step.abs() + (after.r1 - after.r0).abs(),
                        peak_ratio: before.r1.abs().max(after.r0.abs()).max(after.r1.abs()),
                        speed_mm_s: leaders.speed,
                        accel_mm_s2: leaders.rate,
                        jerk_mm_s3: leaders.jerk,
                    },
                );
                velocity > rule.max_velocity_mm_s || accel > rule.max_accel_mm_s2
            })
    }
}

/// The speed under which the leaders rest: they cover no more than
/// [`REST_TARGET_MM`], the width field's rest, between two width nodes.
const REST_SPEED_MM_S: f64 = REST_TARGET_MM / NODE_SPACING_S;

/// The path the leaders of a leg cover from its start, by time. The
/// integral over each whole knot interval is taken once and the running
/// sum kept, so a covered path sums the same terms in the same order as
/// summing every interval afresh.
struct LegPath<'l, 'a> {
    leaders: &'l ShapedLeaders<'a>,
    grid: Vec<f64>,
    start_s: f64,
    whole: std::cell::RefCell<Vec<f64>>,
}

impl<'l, 'a> LegPath<'l, 'a> {
    fn of(leaders: &'l ShapedLeaders<'a>, leg: &RampLeg) -> Self {
        Self {
            leaders,
            grid: axis_grid(leg.shaped, leg.leaders),
            start_s: leg.start_s,
            whole: std::cell::RefCell::new(vec![std::iter::empty::<f64>().sum()]),
        }
    }

    fn speed(&self, t: f64) -> f64 {
        self.leaders.speed(t)
    }

    /// The path covered by `t`: every knot interval ending by it whole,
    /// and the part of the next one up to it.
    fn covered(&self, t: f64) -> f64 {
        let grid = &self.grid;
        let mut whole_intervals = grid[1..].partition_point(|&end| end <= t);
        while whole_intervals > 0 && grid[whole_intervals - 1] >= t {
            whole_intervals -= 1;
        }
        let mut whole = self.whole.borrow_mut();
        while whole.len() <= whole_intervals {
            let index = whole.len() - 1;
            let sum =
                whole[index] + integrate(&|time| self.speed(time), grid[index], grid[index + 1]);
            whole.push(sum);
        }
        let through_whole = whole[whole_intervals];
        match grid.get(whole_intervals + 1) {
            Some(_) if grid[whole_intervals] < t => {
                through_whole + integrate(&|time| self.speed(time), grid[whole_intervals], t)
            }
            _ => through_whole,
        }
    }

    /// When the leaders reach path `s`: the leg's start for a seam laid
    /// there, else the time the path they cover reaches `s`.
    fn time_at(&self, s: f64) -> f64 {
        let grid = &self.grid;
        if s <= self.start_s + SPAN_LOOKUP_SLACK_MM {
            return grid[0];
        }
        let (mut lo, mut hi) = (grid[0], grid[grid.len() - 1]);
        for _ in 0..WIDTH_BISECTIONS {
            let mid = 0.5 * (lo + hi);
            if mid == lo || mid == hi {
                break;
            }
            if self.start_s + self.covered(mid) < s {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        hi
    }

    /// Whether the leaders provably stay at `speed` or faster over the
    /// whole leg, knot interval by knot interval.
    fn outruns(&self, speed: f64) -> bool {
        self.grid.windows(2).all(|pair| {
            self.leaders
                .speed_floor(
                    pair[0],
                    0.5 * (pair[0] + pair[1]),
                    pair[1],
                    FloorLeaders::Cached,
                )
                .is_some_and(|floor| floor >= speed)
        })
    }
}

/// The shaped leaders' speed along a leg, probed
/// [`REST_PROBES_PER_INTERVAL`] times per knot interval.
struct LegSpeed<'l, 'a> {
    path: LegPath<'l, 'a>,
    probes: Vec<f64>,
    table_end: f64,
}

impl<'l, 'a> LegSpeed<'l, 'a> {
    fn over(path: LegPath<'l, 'a>, table_end: f64) -> Self {
        let grid = &path.grid;
        let probes = grid
            .windows(2)
            .flat_map(|pair| {
                (0..REST_PROBES_PER_INTERVAL).map(move |index| {
                    pair[0] + (pair[1] - pair[0]) * index as f64 / REST_PROBES_PER_INTERVAL as f64
                })
            })
            .chain(std::iter::once(grid[grid.len() - 1]))
            .collect();
        Self {
            path,
            probes,
            table_end,
        }
    }

    fn at(&self, t: f64) -> f64 {
        self.path.speed(t)
    }

    /// The probes either side of probe `index`.
    fn around(&self, index: usize) -> (f64, f64) {
        (
            self.probes[index.saturating_sub(1)],
            self.probes[(index + 1).min(self.probes.len() - 1)],
        )
    }

    /// The slowest time between the probes either side of probe `index`.
    fn slowest_near(&self, index: usize) -> f64 {
        let (mut lo, mut hi) = self.around(index);
        for _ in 0..REST_REFINEMENTS {
            let (left, right) = (lo + (hi - lo) / 3.0, hi - (hi - lo) / 3.0);
            if self.at(left) <= self.at(right) {
                hi = right;
            } else {
                lo = left;
            }
        }
        0.5 * (lo + hi)
    }

    /// Whether the leaders provably stay faster than `rest_speed` between
    /// the probes either side of probe `index`.
    fn outruns_rest_near(&self, index: usize, rest_speed: f64) -> bool {
        let (lo, hi) = self.around(index);
        self.path
            .leaders
            .speed_floor(lo, self.probes[index], hi, FloorLeaders::Every)
            .is_some_and(|floor| floor > rest_speed)
    }

    fn path_at(&self, t: f64) -> f64 {
        (self.path.start_s + self.path.covered(t)).min(self.table_end)
    }
}

/// Where and when the shaped leaders of a rest leg first and last rest.
#[derive(Debug, Clone, Copy)]
struct Resting {
    first_t: f64,
    first_s: f64,
    last_t: f64,
    last_s: f64,
}

/// Where along `leg` the shaped leaders rest, first to last: the first and
/// the last time their speed is down to `rest_speed`, the minimum of each
/// probe interval refined so that a turnaround, at rest for an instant
/// only, is found.
fn resting_stretch(
    leaders: &ShapedLeaders,
    leg: &RampLeg,
    rest_speed: f64,
    table_end: f64,
) -> Option<Resting> {
    let speed = LegSpeed::over(LegPath::of(leaders, leg), table_end);
    let resting = |t: f64| speed.at(t) <= rest_speed;
    let rest_time = |index: usize| {
        let probe = speed.probes[index];
        if resting(probe) {
            return Some(probe);
        }
        if speed.outruns_rest_near(index, rest_speed) {
            return None;
        }
        let slowest = speed.slowest_near(index);
        resting(slowest).then_some(slowest)
    };
    let first = (0..speed.probes.len()).find_map(rest_time)?;
    let last = (0..speed.probes.len())
        .rev()
        .find_map(rest_time)
        .expect("a first rest is also a last");
    let edge = |moving: f64, rest: f64| {
        let (mut moving, mut rest) = (moving, rest);
        for _ in 0..REST_REFINEMENTS {
            let mid = 0.5 * (moving + rest);
            if resting(mid) {
                rest = mid;
            } else {
                moving = mid;
            }
        }
        rest
    };
    let grid = &speed.path.grid;
    let first_t = edge(grid[0], first);
    let last_t = edge(grid[grid.len() - 1], last);
    Some(Resting {
        first_t,
        first_s: speed.path_at(first_t),
        last_t,
        last_s: speed.path_at(last_t),
    })
}

/// The first and the last seam in `[from, to)` where the table's ratio
/// itself steps.
fn ratio_steps_within(spans: &[RatioSpan], from: f64, to: f64) -> Option<(f64, f64)> {
    let first_after = spans.partition_point(|span| span.s0 < from).max(1);
    let mut steps = spans[first_after - 1..]
        .windows(2)
        .take_while(|pair| pair[1].s0 < to)
        .filter_map(|pair| Seam::between(&pair[0], &pair[1]))
        .filter(|seam| seam.step != 0.0)
        .map(|seam| seam.s);
    let first = steps.next()?;
    Some((first, steps.last().unwrap_or(first)))
}

/// Where inside `leg`, off its seams, the shaped leaders are slowest, if
/// they crawl there ([`Crossing::crawls`]): the path there and how a valley
/// there reads them. Shaping spreads a raw reversal over the kernels'
/// support, so where the move into it brakes unlike the move out of it
/// launches, the shaped velocity passes through zero off the raw seam,
/// inside either move, where the seam's crossing does not see it; where the
/// axes are shaped unlike each other it passes a hair beside zero instead.
/// A leg whose every knot interval, or whose slowest probe, provably stays
/// faster has none, and one slowest at an end leaves it to that seam's
/// crossing.
fn slowest_crawl(
    leaders: &ShapedLeaders,
    leg: &RampLeg,
    table_end: f64,
) -> Option<(f64, Crossing)> {
    let path = LegPath::of(leaders, leg);
    if path.outruns(ADVANCE_VALLEY_SPEED_MM_S) {
        return None;
    }
    let speed = LegSpeed::over(path, table_end);
    let (slowest_probe, slowest) = speed
        .probes
        .iter()
        .map(|&t| speed.at(t))
        .enumerate()
        .min_by(|left, right| left.1.total_cmp(&right.1))
        .expect("a leg has probes");
    if slowest >= ADVANCE_VALLEY_SPEED_MM_S
        && speed.outruns_rest_near(slowest_probe, ADVANCE_VALLEY_SPEED_MM_S)
    {
        return None;
    }
    let turn = speed.slowest_near(slowest_probe);
    let last_probe = speed.probes.len() - 1;
    let at_an_end = [0, last_probe]
        .into_iter()
        .any(|end| slowest_probe == end && speed.at(turn) >= slowest);
    if at_an_end {
        return None;
    }
    let crossing = Crossing::valley_at(leaders, turn);
    crossing.crawls().then(|| (speed.path_at(turn), crossing))
}

/// The largest extrusion ratio `spans` lay within `reach` of `s`.
fn peak_ratio_within(spans: &[RatioSpan], s: f64, reach: f64) -> f64 {
    spans
        .iter()
        .filter(|span| span.s1 >= s - reach && span.s0 <= s + reach)
        .map(|span| span.r0.abs().max(span.r1.abs()))
        .fold(0.0, f64::max)
}

/// The time a turnaround's rate takes to build the speed the leaders cross
/// its seam with: the shaped leaders' velocities there are zero to their
/// fit's rounding, far under the rate times this.
const TURNAROUND_SPEED_TIME_S: f64 = 1e-6;
/// Probes a ramp's crossing is searched with for the leaders' fastest
/// speed, acceleration and jerk.
const CROSSING_PROBES: usize = 16;

/// How the leaders cross a seam: their path speed, the magnitude of its
/// rate, and how steeply that rate changes.
#[derive(Debug, Clone, Copy)]
struct Crossing {
    speed: f64,
    rate: f64,
    jerk: f64,
}

impl Crossing {
    /// The leaders' path speed at `t` and their path acceleration, signed,
    /// as they leave `t`. Where the speed is a zero's rounding — at rest,
    /// or at a turnaround, under what [`TURNAROUND_SPEED_TIME_S`] of the
    /// acceleration builds — its direction is rounding too, and so is the
    /// tangential acceleration read along it: the speed leaves a zero at
    /// the whole acceleration.
    fn speed_and_path_accel(leaders: &ShapedLeaders, t: f64) -> (f64, f64) {
        let motion = leaders.motion(t);
        let speed = motion.speed();
        let accel = motion.accel_norm();
        if speed > accel * TURNAROUND_SPEED_TIME_S {
            (speed, motion.speed_deriv(speed))
        } else {
            (speed, accel)
        }
    }

    /// How a valley at `t` reads the leaders crossing it, with the jerk
    /// [`Self::needs_valley`] weighs: the path speed's second rate at `t`,
    /// which carries the tangent's turning `a⊥²/v` through a crawl. Under
    /// [`ADVANCE_VALLEY_SPEED_MM_S`] they leave under the whole of their
    /// acceleration, mostly across the path where they nearly reverse, not
    /// the path speed's rate at its minimum.
    fn valley_at(leaders: &ShapedLeaders, t: f64) -> Self {
        let motion = leaders.motion(t);
        let speed = motion.speed();
        let accel = motion.accel_norm();
        if speed <= accel * TURNAROUND_SPEED_TIME_S {
            return Self {
                speed,
                rate: accel,
                jerk: 0.0,
            };
        }
        let rate = if speed < ADVANCE_VALLEY_SPEED_MM_S {
            accel
        } else {
            motion.speed_deriv(speed).abs()
        };
        let jerk = motion
            .speed_second(speed)
            .unwrap_or_else(|| leaders.speed_second(t, speed))
            .abs();
        Self { speed, rate, jerk }
    }

    /// The fastest and the most accelerating the leaders are over `window`,
    /// and the steepest their path acceleration changes there.
    ///
    /// The jerk is the largest mean over the intervals between probes, the
    /// change in path acceleration across one over its length `Δt`. The
    /// leaders are a fit, which holds their acceleration within its
    /// tolerance `A` but leaves their jerk free: a sliver piece may carry
    /// any, and where the fit lays one depends on how the shaper batched
    /// it. The mean is within `2A/Δt` of the leaders' own whatever pieces
    /// the fit lays. A window of no length spans no interval and meets no
    /// jerk.
    fn over(leaders: &ShapedLeaders, (from, until): (f64, f64)) -> Self {
        let probe_at = |index: usize| from + (until - from) * index as f64 / CROSSING_PROBES as f64;
        let (speed, path_accel) = Self::speed_and_path_accel(leaders, from);
        let mut widest = Self {
            speed,
            rate: path_accel.abs(),
            jerk: 0.0,
        };
        let mut previous = (from, path_accel);
        for index in 1..=CROSSING_PROBES {
            let at = probe_at(index);
            let (speed, path_accel) = Self::speed_and_path_accel(leaders, at);
            let jerk = if at > previous.0 {
                (path_accel - previous.1).abs() / (at - previous.0)
            } else {
                0.0
            };
            widest = widest.widest(Self {
                speed,
                rate: path_accel.abs(),
                jerk,
            });
            previous = (at, path_accel);
        }
        widest
    }

    fn widest(self, other: Self) -> Self {
        Self {
            speed: self.speed.max(other.speed),
            rate: self.rate.max(other.rate),
            jerk: self.jerk.max(other.jerk),
        }
    }

    /// What both sides of a seam share. A rate on one side only — a leader
    /// starting from rest with an acceleration step, as an unshaped z does
    /// — leaves the seam crossed at rest: an advance steps the follower's
    /// velocity there by `k·r·Δa` whether the ratio is smoothed or not, so
    /// a window would only carry the ratio step's material into the move.
    fn common(self, other: Self) -> Self {
        Self {
            speed: self.speed.min(other.speed),
            rate: self.rate.min(other.rate),
            jerk: self.jerk.min(other.jerk),
        }
    }

    /// How far the leaders reach from the seam in half the ramp time.
    fn reach(self, ramp_time: f64) -> f64 {
        (self.speed + 0.5 * self.rate * ramp_time) * 0.5 * ramp_time
    }

    /// The leaders turn around at the seam: they cross it at a speed their
    /// rate builds within [`TURNAROUND_SPEED_TIME_S`], the rounding of an
    /// exact zero, while that rate itself is no rounding.
    fn turns_around(self) -> bool {
        self.rate > 0.0 && self.speed <= self.rate * TURNAROUND_SPEED_TIME_S
    }

    /// The leaders turn around, or crawl under
    /// [`ADVANCE_VALLEY_SPEED_MM_S`].
    fn crawls(self) -> bool {
        self.turns_around() || self.speed < ADVANCE_VALLEY_SPEED_MM_S
    }

    /// An advance of `rule`'s slope releases its lead around where the
    /// leaders turn around, or crawl where reading the path speed through
    /// at `ratio` would ask the follower for more than its acceleration
    /// limit: `r·(a + k·s⃛)`, the path speed's second rate carrying the
    /// tangent's turning `a⊥²/v`. Elsewhere releasing the lead over the
    /// crawl's short reach would ask more of the follower than the crawl.
    fn needs_valley(self, rule: &RatioRampRule, ratio: f64) -> bool {
        self.turns_around()
            || (self.crawls()
                && ratio * (self.rate + rule.advance_slope_s * self.jerk) > rule.max_accel_mm_s2)
    }

    /// The floor a valley here keeps flat (see [`AdvanceValley`]): `v²/a`,
    /// about where the acceleration takes over from the crawl, and none
    /// where the leaders turn around.
    fn crawl_floor(self) -> f64 {
        if self.turns_around() || self.speed == 0.0 {
            0.0
        } else {
            self.speed * self.speed / self.rate
        }
    }
}

#[cfg(test)]
mod tests;
