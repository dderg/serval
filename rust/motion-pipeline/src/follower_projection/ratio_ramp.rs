use std::collections::VecDeque;

use trajectory::ContinuousSegment;

use super::{
    RatioSpan, SPAN_LOOKUP_SLACK_MM, SPAN_MIN_LEN_MM, ShapedLeaders, axis_grid, integrate,
};

mod jet;
pub(super) use jet::Jet;

/// The steepest the width envelope climbs per unit of path away from a
/// node (see [`WidthField`]).
const ENVELOPE_SLOPE: f64 = 0.25;
/// `max |h'|` the width reaches: `3·ENVELOPE_SLOPE` rising off a rest,
/// `1.875·ENVELOPE_SLOPE` between two moving nodes.
pub(super) const WIDTH_SLOPE_BOUND: f64 = 3.0 * ENVELOPE_SLOPE;
/// Width nodes per ramp time: one every `τ/2`.
const WIDTH_NODES_PER_RAMP_TIME: f64 = 2.0;
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
const WIDTH_BISECTIONS: usize = 64;
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
    pub(super) support: (f64, f64),
    pub(super) start_s: f64,
    pub(super) rests: bool,
}

/// A segment whose projection waits for every seam and width node that can
/// reach into it.
#[derive(Debug, Clone, Copy)]
pub(super) struct PendingSegment {
    pub(super) t_end: f64,
    pub(super) s_start: f64,
    pub(super) s_length: f64,
    table_end: f64,
}

/// Where the span table's ratio or its slope steps.
#[derive(Debug, Clone, Copy)]
struct Seam {
    s: f64,
    step: f64,
    slope_step: f64,
    slope_after: f64,
}

/// One node of the width field: its path position, the half-width its
/// leaders ask for there once known, and the envelope the field takes.
#[derive(Debug, Clone, Copy)]
struct WidthNode {
    s: f64,
    target: Option<f64>,
    envelope: f64,
}

/// The smoothing's half-width `h(s)` along the path.
///
/// Nodes sit every `τ/2` in time. A node's target is half the path the
/// leaders cover in the `τ` around it — `v·τ/2` at cruise, `a·τ²/8` at a
/// turnaround, the accelerating path on both sides at a slow corner — and
/// zero at a rest: where the leaders cross a leg seam at rest, at a floor,
/// and at every node inside a dwell. The envelope is the largest
/// `ENVELOPE_SLOPE`-Lipschitz field under every target, and `h` blends
/// neighbouring envelope values by a smootherstep (flat to second order at
/// each node, so `h` is C2), or rises off a rest node as `ψ`, linearly at
/// first: a window shrinks to nothing at a rest at most as fast as the
/// distance to it, so it never reaches across.
///
/// `|h'| ≤ WIDTH_SLOPE_BOUND < 1` everywhere, which keeps the smoothed
/// ratio a convex combination of the table's (see [`RatioSmoothing`]).
#[derive(Debug, Default)]
struct WidthField {
    nodes: Vec<WidthNode>,
    spacing: f64,
    next_time: Option<i64>,
    pending: Option<usize>,
    before_pending: f64,
    final_prefix: usize,
}

impl WidthField {
    fn retime(&mut self, ramp_time: f64) {
        let spacing = ramp_time / WIDTH_NODES_PER_RAMP_TIME;
        if self.spacing == spacing {
            return;
        }
        assert!(
            self.pending.is_none(),
            "a ratio ramp time changed from {} s to {ramp_time} s away from a floor",
            self.spacing * WIDTH_NODES_PER_RAMP_TIME
        );
        let last_rest = self.nodes.last().map(|node| node.s);
        *self = Self {
            spacing,
            ..Self::default()
        };
        if let Some(s) = last_rest {
            self.place_rest(s);
        }
    }

    /// Places the time nodes `leg` covers, the times in a gap before it
    /// (legal only at rest) at its start.
    fn extend(&mut self, leaders: &ShapedLeaders, leg: &RampLeg, table_end: f64) {
        let grid = axis_grid(leg.shaped, leg.leaders);
        let (t0, t1) = (grid[0], grid[grid.len() - 1]);
        let mut index = self
            .next_time
            .unwrap_or_else(|| (t0 / self.spacing).ceil() as i64);
        if self.nodes.is_empty() {
            self.place_rest(leg.start_s);
        }
        let speed = |t: f64| leaders.speed(t);
        let (mut knot, mut at, mut covered) = (1, t0, 0.0);
        loop {
            let t = index as f64 * self.spacing;
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
            self.place_time_node((leg.start_s + covered).min(table_end));
            index += 1;
        }
        self.next_time = Some(index);
    }

    fn place_time_node(&mut self, s: f64) {
        if let Some(pending) = self.pending {
            self.resolve(pending, s);
            self.before_pending = self.nodes[pending].s;
        } else {
            self.before_pending = s;
        }
        self.push(s, None);
        self.pending = Some(self.nodes.len() - 1);
    }

    /// The pending node's target: half the path between the time nodes on
    /// either side of it, `next` the one after.
    fn resolve(&mut self, pending: usize, next: f64) {
        let target = 0.5 * (next.max(self.nodes[pending].s) - self.before_pending);
        self.nodes[pending].target = Some(if target <= REST_TARGET_MM {
            0.0
        } else {
            target
        });
        self.settle(pending);
    }

    /// A floor at `s`: the leaders rest there and nothing after reaches
    /// back, so the node still waiting on the path ahead takes `s` as it.
    fn close(&mut self, s: f64) {
        if let Some(pending) = self.pending.take() {
            self.resolve(pending, s);
        }
        self.place_rest(s);
    }

    fn place_rest(&mut self, s: f64) {
        if self
            .nodes
            .last()
            .is_some_and(|last| last.s >= s && last.envelope == 0.0)
        {
            return;
        }
        self.push(s, Some(0.0));
    }

    /// A node within [`REST_TARGET_MM`] past the last one stacks on it: the
    /// leaders cover no more between them than the fit's back-and-forth
    /// around a rest, and a node a rounding short of a rest would take the
    /// rest's cone as a sliver of an envelope that the smootherstep
    /// flattens onto, narrowing every window short of the rest to as much.
    fn push(&mut self, s: f64, target: Option<f64>) {
        let s = self.nodes.last().map_or(s, |last| {
            assert!(
                s >= last.s - SPAN_LOOKUP_SLACK_MM,
                "a ratio ramp width node at {s} mm falls behind the last one at {} mm",
                last.s
            );
            if s <= last.s + REST_TARGET_MM {
                last.s
            } else {
                s
            }
        });
        self.nodes.push(WidthNode {
            s,
            target,
            envelope: target.unwrap_or(f64::INFINITY),
        });
        self.settle(self.nodes.len() - 1);
    }

    fn cone(&self, from: usize, to: usize) -> f64 {
        self.nodes[from].envelope + ENVELOPE_SLOPE * (self.nodes[to].s - self.nodes[from].s).abs()
    }

    fn settle(&mut self, index: usize) {
        let mut envelope = self.nodes[index]
            .envelope
            .min(self.nodes[index].target.unwrap_or(f64::INFINITY));
        if index > 0 {
            envelope = envelope.min(self.cone(index - 1, index));
        }
        if index + 1 < self.nodes.len() {
            envelope = envelope.min(self.cone(index + 1, index));
        }
        self.nodes[index].envelope = envelope;
        for back in (0..index).rev() {
            let lowered = self.cone(back + 1, back);
            if lowered >= self.nodes[back].envelope {
                break;
            }
            self.nodes[back].envelope = lowered;
        }
        for ahead in index + 1..self.nodes.len() {
            let lowered = self.cone(ahead - 1, ahead);
            if lowered >= self.nodes[ahead].envelope {
                break;
            }
            self.nodes[ahead].envelope = lowered;
        }
    }

    /// A node is final once no later node can lower it: every node still to
    /// come lies past the frontier, and its cone is at least the slope
    /// times the distance there.
    fn is_final(&self, index: usize) -> bool {
        let node = self.nodes[index];
        let frontier = self.pending.map_or_else(
            || self.nodes.last().map_or(node.s, |last| last.s),
            |p| self.nodes[p].s,
        );
        node.target.is_some() && node.envelope <= ENVELOPE_SLOPE * (frontier - node.s)
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

    fn jet(&self, s: f64) -> Jet {
        let (a, b) = match self.interval(s) {
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

    fn width(&self, s: f64) -> f64 {
        self.jet(s).value
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
        self.pending = self.pending.map(|pending| pending - keep);
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
    resting: bool,
    rest_end: f64,
    examined_through: f64,
    floor: f64,
    taken_through: f64,
    reach_into_next: Option<Crossing>,
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

    pub(super) fn has_backlog(&self) -> bool {
        !self.backlog.is_empty()
    }

    /// Where on the path the last segment still waiting on the smoothing
    /// ends.
    pub(super) fn backlog_end(&self) -> Option<f64> {
        self.backlog
            .back()
            .map(|segment| segment.s_start + segment.s_length)
    }

    /// No window may reach back past `s`: the leaders rest there, or the
    /// stream drained there and what precedes it is already committed. A
    /// rest valley still open closes there, with nothing after it.
    pub(super) fn close_at(&mut self, s: f64) {
        self.floor = self.floor.max(s);
        self.reach_into_next = None;
        self.close_rest(s, 0.0);
        if self.width.spacing > 0.0 {
            self.width.close(s);
        }
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

    /// Takes in `leg`'s spans: the width nodes it covers, the seams the
    /// table gained, and the advance valleys its opening seam and a
    /// turnaround inside it need.
    pub(super) fn lay(&mut self, spans: &[RatioSpan], leg: &RampLeg, ramp_time: f64) {
        self.width.retime(ramp_time);
        let leaders = ShapedLeaders::of(leg.shaped, leg.leaders);
        let table_end = spans.last().map_or(leg.start_s, |last| last.s1);
        if table_end <= leg.start_s {
            self.width.extend(&leaders, leg, table_end);
            self.close_at(leg.start_s);
            return;
        }
        if self.base_slope.is_none() {
            self.base_slope = Some(spans[0].ratio_slope());
        }
        let (t_start, t_end) = leg.support;
        let opening = Crossing::at(&leaders, t_start);
        let crossing = self
            .reach_into_next
            .map_or(opening, |closing| closing.common(opening));
        let seam = leg.start_s;
        let reach = crossing.reach(ramp_time).min(seam - self.floor);
        if reach <= SPAN_MIN_LEN_MM {
            self.width.place_rest(seam);
        }
        self.width.extend(&leaders, leg, table_end);
        let closes_a_rest = self.lay_rest_valley(leg, &leaders, crossing, ramp_time, table_end);
        if reach > SPAN_MIN_LEN_MM && !self.resting && !closes_a_rest && crossing.turns_around() {
            self.lay_turnaround_valley(seam, reach);
        }
        let turn_inside = (!leg.rests && !self.resting)
            .then(|| turnaround_inside(&leaders, leg, table_end))
            .flatten();
        if let Some((turn, turn_crossing)) = turn_inside {
            let reach = turn_crossing.reach(ramp_time).min(turn - self.floor);
            if reach > SPAN_MIN_LEN_MM {
                self.lay_turnaround_valley(turn, reach);
            }
        }
        self.record_seams(spans);
        self.reach_into_next = Some(Crossing::at(&leaders, t_end));
    }

    fn lay_turnaround_valley(&mut self, turn: f64, reach: f64) {
        self.valleys.push(AdvanceValley {
            start: turn,
            end: turn,
            before: reach.min(self.back_reach_limit(turn)),
            after: reach,
        });
    }

    fn record_seams(&mut self, spans: &[RatioSpan]) {
        let first = spans
            .partition_point(|span| span.s0 <= self.examined_through)
            .max(1);
        for index in first..spans.len() {
            let (before, after) = (&spans[index - 1], &spans[index]);
            self.examined_through = after.s0;
            let step = after.r0 - before.r1;
            let slope_step = after.ratio_slope() - before.ratio_slope();
            if step == 0.0 && slope_step == 0.0 {
                continue;
            }
            self.seams.push(Seam {
                s: after.s0,
                step,
                slope_step,
                slope_after: after.ratio_slope(),
            });
        }
    }

    pub(super) fn queue(&mut self, t_end: f64, (s_start, s_length): (f64, f64), table_end: f64) {
        self.backlog.push_back(PendingSegment {
            t_end,
            s_start,
            s_length,
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
                .reach(WIDTH_NODES_PER_RAMP_TIME * self.width.spacing)
                .min(table_end - self.floor)
        })
    }

    /// How far the smoothed ratio is final: its width there is, and every
    /// window reaches no further than the table.
    fn final_through(&mut self, table_end: f64) -> f64 {
        let Some(width_final) = self.width.final_through() else {
            return self.floor;
        };
        let fits = |s: f64| s + self.width.width(s) <= table_end;
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
    /// table's.
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
        let squared = h * h;
        let inverse = h.recip();
        let position = Jet::variable(s);
        self.seams[first..]
            .iter()
            .take_while(|seam| seam.s < hi)
            .fold(
                squared.scale(KERNEL_HALF_VARIANCE * slope_left),
                |total, seam| {
                    let x = position.offset(-seam.s) * inverse;
                    let (ramp, half_square) = if x.value >= 0.0 {
                        (x, (x * x).scale(0.5))
                    } else {
                        (Jet::ZERO, Jet::ZERO)
                    };
                    let g1 = x.poly(&SMOOTHED_RAMP) - ramp;
                    let g2 = x.poly(&SMOOTHED_HALF_SQUARE) - half_square;
                    total + (h * g1).scale(seam.step) + (squared * g2).scale(seam.slope_step)
                },
            )
    }

    fn seams_near(&self, lo: f64, hi: f64) -> &[Seam] {
        let first = self.seams.partition_point(|seam| seam.s <= lo);
        let last = self.seams.partition_point(|seam| seam.s < hi);
        &self.seams[first..last.max(first)]
    }

    /// Whether the smoothing changes anything over `[lo, hi]`.
    pub(super) fn touches(&self, lo: f64, hi: f64) -> bool {
        if self.is_inert() {
            return false;
        }
        let (widest, _) = self.width.bounds(lo, hi);
        self.base_slope.unwrap_or(0.0) != 0.0
            || !self.seams_near(lo - widest, hi + widest).is_empty()
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
    /// window, the width nodes, and the valley ends.
    pub(super) fn bends(&self, lo: f64, hi: f64) -> Vec<f64> {
        let mut bends: Vec<f64> = self.valley_ends(lo, hi).collect();
        if self.is_inert() {
            return bends;
        }
        let (widest, _) = self.width.bounds(lo, hi);
        bends.extend(self.width.covering(lo, hi).iter().map(|node| node.s));
        for seam in self.seams_near(lo - widest, hi + widest) {
            bends.push(self.window_edge(seam.s, -1.0, widest));
            bends.push(self.window_edge(seam.s, 1.0, widest));
        }
        bends
    }

    /// The `s` on `side` of `seam` where `s ∓ h(s) = seam`: `s ± h(s)` is
    /// increasing since `|h'| < 1`.
    fn window_edge(&self, seam: f64, side: f64, widest: f64) -> f64 {
        let reach = |s: f64| (s - seam) * side - self.width.width(s);
        let (first, last) = (
            self.width.nodes[0].s,
            self.width.nodes[self.width.nodes.len() - 1].s,
        );
        let mut near = seam.clamp(first, last);
        let mut far = (seam + side * (widest / (1.0 - WIDTH_SLOPE_BOUND))).clamp(first, last);
        if reach(far) < 0.0 || reach(near) >= 0.0 {
            return near;
        }
        for _ in 0..WIDTH_BISECTIONS {
            let mid = 0.5 * (near + far);
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
    /// their speed is down to [`rest_speed`]: a rest valley opens where they
    /// first come to rest, unless one is open already, and the first leg
    /// that moves again closes it where they last rested. Each side spans
    /// the shaped spill between that rest and the raw seam plus as far as
    /// the leaders cover in half the ramp time at the seam, so the advance
    /// keeps its lead through the spill and builds or releases it from the
    /// rest out. A rest leg the leaders cross without coming to rest opens
    /// none. Returns whether `leg` closes a rest valley.
    fn lay_rest_valley(
        &mut self,
        leg: &RampLeg,
        leaders: &ShapedLeaders,
        crossing: Crossing,
        ramp_time: f64,
        table_end: f64,
    ) -> bool {
        let seam = leg.start_s;
        let reach = crossing.reach(ramp_time);
        if !leg.rests {
            if !self.resting {
                return false;
            }
            let rest_end = self.rest_end;
            self.close_rest(rest_end, seam - rest_end + reach);
            return true;
        }
        let Some((rest_start, rest_end)) =
            resting_stretch(leaders, leg, rest_speed(ramp_time), table_end)
        else {
            return false;
        };
        if !self.resting {
            self.valleys.push(AdvanceValley {
                start: rest_start,
                end: f64::INFINITY,
                before: rest_start - seam + reach.min(self.back_reach_limit(seam)),
                after: 0.0,
            });
            self.resting = true;
        }
        self.rest_end = rest_end;
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
/// path speed is zero, and `W = ψ(x) = 1 − (1 − x)³` within
/// `before`/`after` of it, `x` the distance out over that width, so the
/// weight rises off the rest with a slope the leaders' zero speed makes
/// harmless and meets 1 flat to second order. The laid ratio, and so the
/// material, is untouched; only the lead the advance builds is released
/// over the path before and rebuilt after.
///
/// Rising from a rest the leaders leave at acceleration `a`, the lead
/// `k·r·W·ṡ` catches up with the advance's `k·r·ṡ` at a rate of at most
/// `1.653·k·r·a` (the peak of `d/du (u·ψ(u²))`): the advance's natural
/// rate, overshot by what catching up from zero takes, and over a shaped
/// start whose acceleration builds from zero, much less.
///
/// - A turnaround: the leaders' path speed passes through zero at a seam
///   while its rate does not, so the rate steps from `−a` to `+a`, and an
///   advance reading `r·ṡ` would step the follower's velocity by `2·k·r·a`
///   whatever the ratio's own shape. The valley is that seam, each side
///   reaching as far as the leaders turn around in half the ramp time.
/// - A rest: an extrude-only move or a dwell, where the shaped leaders come
///   to rest between the kernel spill of the move before and of the move
///   after, and the fit's back-and-forth near the rest point has its own
///   turnarounds and short idle spans an advance would turn into steps. The
///   valley is the stretch where the leaders rest.
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
        let inside = 1.0 - distance / width;
        (
            1.0 - inside * inside * inside,
            outward * 3.0 * inside * inside / width,
            -6.0 * inside / (width * width),
        )
    }
}

/// The speed under which the leaders rest: they cover no more than
/// [`REST_TARGET_MM`], the width field's rest, in half the ramp time.
fn rest_speed(ramp_time: f64) -> f64 {
    REST_TARGET_MM / (0.5 * ramp_time)
}

/// The shaped leaders' speed along a leg, probed
/// [`REST_PROBES_PER_INTERVAL`] times per knot interval.
struct LegSpeed<'l, 'a> {
    leaders: &'l ShapedLeaders<'a>,
    grid: Vec<f64>,
    probes: Vec<f64>,
    start_s: f64,
    table_end: f64,
}

impl<'l, 'a> LegSpeed<'l, 'a> {
    fn of(leaders: &'l ShapedLeaders<'a>, leg: &RampLeg, table_end: f64) -> Self {
        let grid = axis_grid(leg.shaped, leg.leaders);
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
            leaders,
            grid,
            probes,
            start_s: leg.start_s,
            table_end,
        }
    }

    fn at(&self, t: f64) -> f64 {
        self.leaders.speed(t)
    }

    /// The slowest time between the probes either side of probe `index`.
    fn slowest_near(&self, index: usize) -> f64 {
        let (mut lo, mut hi) = (
            self.probes[index.saturating_sub(1)],
            self.probes[(index + 1).min(self.probes.len() - 1)],
        );
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

    fn path_at(&self, t: f64) -> f64 {
        let covered = self
            .grid
            .windows(2)
            .take_while(|pair| pair[0] < t)
            .map(|pair| integrate(&|time| self.at(time), pair[0], pair[1].min(t)))
            .sum::<f64>();
        (self.start_s + covered).min(self.table_end)
    }
}

/// Where along `leg` the shaped leaders rest, first to last: the path at
/// the first and the last time their speed is down to `rest_speed`, the
/// minimum of each probe interval refined so that a turnaround, at rest
/// for an instant only, is found.
fn resting_stretch(
    leaders: &ShapedLeaders,
    leg: &RampLeg,
    rest_speed: f64,
    table_end: f64,
) -> Option<(f64, f64)> {
    let speed = LegSpeed::of(leaders, leg, table_end);
    let resting = |t: f64| speed.at(t) <= rest_speed;
    let rest_time = |index: usize| {
        if resting(speed.probes[index]) {
            return Some(speed.probes[index]);
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
    let first = edge(speed.grid[0], first);
    let last = edge(speed.grid[speed.grid.len() - 1], last);
    Some((speed.path_at(first), speed.path_at(last)))
}

/// Where inside `leg`, off its seams, the shaped leaders turn around: the
/// path there and how they cross it. Shaping spreads a raw reversal over
/// the kernels' support, so where the move into it brakes unlike the move
/// out of it launches, the shaped velocity passes through zero off the raw
/// seam, inside either move, where the seam's crossing does not see it.
/// Such a leg's velocity reverses from one end to the other, and the
/// turnaround is its slowest point.
fn turnaround_inside(
    leaders: &ShapedLeaders,
    leg: &RampLeg,
    table_end: f64,
) -> Option<(f64, Crossing)> {
    let (t_start, t_end) = leg.support;
    let reverses = (0..leaders.axes.len())
        .map(|index| leaders.pva(index, t_start).1 * leaders.pva(index, t_end).1)
        .sum::<f64>()
        < 0.0;
    if !reverses
        || Crossing::at(leaders, t_start).turns_around()
        || Crossing::at(leaders, t_end).turns_around()
    {
        return None;
    }
    let speed = LegSpeed::of(leaders, leg, table_end);
    let (slowest_probe, _) = speed
        .probes
        .iter()
        .map(|&t| speed.at(t))
        .enumerate()
        .min_by(|left, right| left.1.total_cmp(&right.1))
        .expect("a leg has probes");
    let turn = speed.slowest_near(slowest_probe);
    let crossing = Crossing::at(leaders, turn);
    crossing
        .turns_around()
        .then(|| (speed.path_at(turn), crossing))
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
    /// velocity there by `k·r·Δa` whether the ratio is smoothed or not, so
    /// a window would only carry the ratio step's material into the move.
    fn common(self, other: Self) -> Self {
        Self {
            speed: self.speed.min(other.speed),
            rate: self.rate.min(other.rate),
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
}

#[cfg(test)]
mod tests;
