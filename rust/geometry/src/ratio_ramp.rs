//! How long a follower's ratio ramp spreads an extrusion-ratio step.
//!
//! The projection smooths the extrusion ratio along the path with the
//! kernel `K(x) = ¾(1 − x²)` of half-width `h = v·τ/2` at leader speed `v`.
//! `∫₋₁ˣ K = 3u² − 2u³` with `u = (x + 1)/2`, so a ratio step `Δr` reaches
//! the follower as the smoothstep `r̃ = r_b + Δr·S(t/τ)` over about `τ` in
//! time, `S′ ≤ 3/2`, `|S″| ≤ 6`. An advance of slope `k` reads the flow
//! `f = r̃·v` as `f + k·ḟ`; with the leaders at speed `v` and acceleration
//! `a` across the step, what the ramp adds to the follower is
//!
//! - velocity: `Δr·((v + k·a)·S + k·v·S′/τ)`, at most
//!   `Δr·(v + k|a|)·m(p)` with `p = k·v/((v + k|a|)·τ)` and
//!   `m(p) = max_u (S + 6p·u(1 − u))`, reached at
//!   `u* = ((1 − 2p) + √(1 + 4p²))/2`: an overshoot of `m(p) − 1` times the
//!   step over the advanced flow after it;
//! - acceleration: `Δr·((v + 2k·a)·S′/τ + k·v·S″/τ²)`, at most
//!   `Δr·(v + 2k|a|)/τ·g(p)` with `p = k·v/((v + 2k|a|)·τ)` and
//!   `g(p) = max_u |S′ + p·S″|`: `6p` for `p ≥ ½` (the ends of the ramp,
//!   `|S″| = 6`), `3/2 + 6p²` below (inside it, at `u = ½ − p`).
//!
//! The rest of the advanced flow, `r̃·a` and `k·r̃·ȧ`, is the path's own
//! demand, which the ramp's adds to: the larger ratio about the step times
//! `|a| + k·|ȧ|`. At a steady speed the bounds are exact:
//! `τ ≥ √(6kΔ/A)` wherever `τ ≤ 2k`, with `Δ = |Δr|·v`, and `τ ≥ 3Δ/(2A)`
//! without an advance.
//!
//! A sloped span of ratio slope `m` longer than its ends' windows is
//! averaged by a window that follows the width along the path. That
//! window leads the table's extrusion by `0.1·m·h²`, so the smoothed ratio
//! carries `0.1·m·(h²)′`, its slope `0.1·m·(h²)″` and its curvature
//! `0.1·m·(h²)‴` wherever the width bends, and the advance turns them into
//! `0.1·m·((h²)″·(v² + 3k·v·a) + (h²)‴·k·v³)` of acceleration and
//! `0.1·m·(h²)″·k·v²` of velocity beyond the table's own slope
//! ([`RatioRampRule::demand`]), with `|(h²)″|`, `h·|(h²)‴|` and, over a
//! gap `L` between width nodes, `L·|(h²)‴|` at most what the width field's
//! node shape allows ([`WIDTH_SQUARE_SLOPE_BOUND`],
//! [`WIDTH_SQUARE_CURVATURE_BOUND`], [`width_square_curvature_per_gap`]),
//! `h = v·τ/2`, at most half the span, and `L` at least the path the
//! leaders cover in [`RATIO_RAMP_NODE_SPACING_S`].
//!
//! The ramp time is the shortest in
//! [`RATIO_RAMP_MIN_TIME_S`, `RATIO_RAMP_MAX_TIME_S`] whose addition fits
//! within what the path's own demand leaves of the follower's limits, and
//! never within less than [`RAMP_HEADROOM_FLOOR`] of them: the ramp's
//! addition vanishes with its step, so a small step takes a short ramp even
//! where the path alone runs at the limits. Holding the path within the
//! limits is the planner's: where even the longest ramp does not fit, or a
//! rest of the leaders leaves the ramp less path than it takes
//! ([`rest_room_reach_mm`]), it lowers the leader speed at the step
//! ([`RatioRampRule::step_speed_cap`]).

mod valley;

pub use valley::{
    AdvanceValley, CRAWL_SIDE_WIDEST_MM, CrawlSide, RebuildsWithin, SideWeight,
    TURNAROUND_SPEED_TIME_S, VALLEY_LEAD_BUDGET_S, VALLEY_RAMP_TIME_S, ValleyKind, coupled_bend,
    crawl_floor, crawl_side_width, crawl_step_window, kept_floor, leader_reach, side_weight,
    turns_around, valley_reach, widest_crawl_reach, widest_crawl_side_within,
};

/// The shortest ratio ramp.
pub const RATIO_RAMP_MIN_TIME_S: f64 = 0.005;
/// The longest ratio ramp: past it the planner slows the step instead of
/// smearing the flow over more path.
pub const RATIO_RAMP_MAX_TIME_S: f64 = 0.03;
/// The shaped path speed under which a ratio-ramped advance releases its
/// lead around the leaders' slowest point instead of reading the path
/// speed through it. Near a reversal the path speed's second derivative
/// carries the tangent's turning `a⊥²/v`, which grows without bound as the
/// speed minimum `v` falls, whatever the planner bounds of the path's own
/// acceleration and jerk; under this speed the lead the advance holds,
/// `k·r·v`, is too small to matter to the flow.
pub const ADVANCE_VALLEY_SPEED_MM_S: f64 = 2.0;
/// The steepest a ratio ramp's half-width climbs per unit of path away
/// from a node of its width field, a rest of the leaders among them: the
/// window `d` along the path from a rest is at most `ENVELOPE_SLOPE·d`
/// wide.
pub const RATIO_RAMP_ENVELOPE_SLOPE: f64 = 0.25;
/// How far apart on the path two moving width nodes lie at least, in
/// units of the narrower of their targets: the smootherstep between nodes
/// `L` apart that differ by at most `ENVELOPE_SLOPE·L` bends `h` by up to
/// `h''' = 60·ENVELOPE_SLOPE/L²`, which the smoothed ratio's curvature
/// carries, so an advance turns it into extruder acceleration. Nodes most
/// of a window's width apart keep that bend within the window's own; short
/// of a whole width, since at cruise on the shortest ramp a width is one
/// time tick of path exactly, and a tie there would hang the choice on how
/// the path's rounding falls.
pub const RATIO_RAMP_NODE_SPACING_PER_WIDTH: f64 = 0.75;
/// `½∫u²K`: a ratio of slope `m` averaged by the kernel over a window of
/// half-width `h` leads the table's extrusion by this times `m·h²`.
const KERNEL_HALF_VARIANCE: f64 = 0.1;
/// The smootherstep's third derivative at its ends, its largest.
const SMOOTHERSTEP_PEAK_JERK: f64 = 60.0;
/// The widest a moving width node is, in units of the path to its
/// neighbour: the narrower of the two at most `1/NODE_SPACING_PER_WIDTH`
/// of it, the wider [`RATIO_RAMP_ENVELOPE_SLOPE`] of it more.
const WIDEST_NODE_PER_GAP: f64 =
    1.0 / RATIO_RAMP_NODE_SPACING_PER_WIDTH + RATIO_RAMP_ENVELOPE_SLOPE;
/// The most the width field bends `h²` per unit of path squared,
/// `max |(h²)″| = max 2·|h′² + h·h″|`: between two moving nodes `L` apart,
/// the smootherstep `h = h_a + Δ·S(x)` with `|Δ| ≤ ENVELOPE_SLOPE·L` and `h`
/// at most [`WIDEST_NODE_PER_GAP`]` ·L` peaks at `4.4447` where the wider
/// node falls by the most to the narrower, `x ≈ 0.2` from the wider; a rise
/// off a rest, `h = H·ψ`, `H ≤ ENVELOPE_SLOPE·L`, reaches `9/8` there.
pub const WIDTH_SQUARE_SLOPE_BOUND: f64 = 4.445;
/// The least time between two width nodes the leaders move between: half
/// the shortest ramp.
pub const RATIO_RAMP_NODE_SPACING_S: f64 = RATIO_RAMP_MIN_TIME_S / 2.0;
/// The most the width field bends `h²`'s slope against the path `L` between
/// the nodes either side, `max L·|(h²)‴| = max 2L·|3h′h″ + h·h‴|`, over
/// widths `h` at most `widths_per_gap·L`. Between moving nodes,
/// `h = h_a + Δ·S(x)` with `|Δ| ≤ ENVELOPE_SLOPE·L` makes it
/// `2·|3(Δ/L)²·S′S″ + (h/L)·(Δ/L)·S‴|`, which peaks at a node, where
/// `S′ = S″ = 0` and `S‴ = 60`, as `2·60·ENVELOPE_SLOPE·h/L` wherever that
/// passes `2.51`; `h` is at most [`WIDEST_NODE_PER_GAP`]` ·L`. A rise off a
/// rest over `L`, `h = H·ψ(x)` with `ψ = 1 − (1 − x)³` and
/// `H ≤ ENVELOPE_SLOPE·L`, bends it by at most
/// `6·ENVELOPE_SLOPE²·max|ψ′ψ″| = 6.75`, at the rest.
#[must_use]
pub fn width_square_curvature_per_gap(widths_per_gap: f64) -> f64 {
    let node = 2.0
        * RATIO_RAMP_ENVELOPE_SLOPE
        * SMOOTHERSTEP_PEAK_JERK
        * widths_per_gap.min(WIDEST_NODE_PER_GAP);
    node.max(REST_RISE_SQUARE_CURVATURE_PER_GAP)
}
/// `max |ψ′·ψ″| = 3u²·6u` at the rest, `u = 1`.
const REST_RISE_PEAK_SLOPE_BEND: f64 = 18.0;
const REST_RISE_SQUARE_CURVATURE_PER_GAP: f64 =
    6.0 * RATIO_RAMP_ENVELOPE_SLOPE * RATIO_RAMP_ENVELOPE_SLOPE * REST_RISE_PEAK_SLOPE_BEND;
const WIDTH_SQUARE_CURVATURE_PER_GAP_BOUND: f64 =
    2.0 * WIDEST_NODE_PER_GAP * RATIO_RAMP_ENVELOPE_SLOPE * SMOOTHERSTEP_PEAK_JERK;
/// The most the width field bends `h²`'s slope against the width there,
/// `max h·|(h²)‴|`: [`width_square_curvature_per_gap`] at the widest
/// node a gap allows; a rise off a rest bends `h²` far less against its
/// own width.
pub const WIDTH_SQUARE_CURVATURE_BOUND: f64 =
    WIDEST_NODE_PER_GAP * WIDTH_SQUARE_CURVATURE_PER_GAP_BOUND;
/// The share of the path between a ratio step and a rest of the leaders
/// that the step's ramp takes on that side. A window `u` past the step,
/// under the rest's cone `c·(d − u)`, reaches back to the step only while
/// `u ≤ c·(d − u)`: out to `c/(1 + c)` of the way, where a window of that
/// half-width laid freely would end too.
const REST_ROOM_RAMP_SHARE: f64 = RATIO_RAMP_ENVELOPE_SLOPE / (1.0 + RATIO_RAMP_ENVELOPE_SLOPE);

const BISECTIONS: usize = 64;
/// The share of each follower limit a ratio ramp may always add to the
/// path's own demand, however near the limit that already runs. With no
/// floor the headroom closes as the path's demand reaches the limit, and
/// the ramp time leaps from the shortest to the longest for any step there,
/// however small.
const RAMP_HEADROOM_FLOOR: f64 = 0.25;

/// How far along the path a rest of the leaders narrows the ramp of a
/// ratio step they cross at up to `speed`: past it, the step's share of
/// the room holds the longest ramp.
#[must_use]
pub fn rest_room_reach_mm(speed: f64) -> f64 {
    0.5 * speed * RATIO_RAMP_MAX_TIME_S / REST_ROOM_RAMP_SHARE
}

/// The longest ramp a ratio step crossed at `speed`, `room_mm` along the
/// path from a rest of the leaders, gets: the half of it on the rest's side
/// laid over the step's share of the room, and never over
/// [`RATIO_RAMP_MAX_TIME_S`].
#[must_use]
pub fn longest_ramp_time(speed: f64, room_mm: f64) -> f64 {
    assert!(
        room_mm >= 0.0,
        "a ratio step lies {room_mm} mm from the leaders' nearest rest"
    );
    RATIO_RAMP_MAX_TIME_S.min(rest_room_ramp_time(speed, room_mm))
}

fn rest_room_ramp_time(speed: f64, room_mm: f64) -> f64 {
    if speed > 0.0 {
        2.0 * REST_ROOM_RAMP_SHARE * room_mm / speed
    } else {
        f64::INFINITY
    }
}

/// What the longest ramp across a ratio step of `ratio_step` adds to the
/// follower's acceleration per unit of the leaders' acceleration where they
/// cross the step at rest, an advance of slope `advance_slope_s` reading
/// it: `Δr·2k/τ_max·g(0)`, the part of the ramp's acceleration
/// ([`RatioRampRule::demand`]) no slower crossing lowers.
#[must_use]
pub fn ramp_accel_at_rest_per_accel(advance_slope_s: f64, ratio_step: f64) -> f64 {
    ratio_step * 2.0 * advance_slope_s / RATIO_RAMP_MAX_TIME_S * acceleration_gain(0.0)
}

/// What a follower's ratio ramp is held to: the advance's slope, the
/// follower motor's velocity and acceleration limits, and the jerk gain of
/// the kernel smoothing the advanced follower, infinite without one.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RatioRampRule {
    pub advance_slope_s: f64,
    pub max_velocity_mm_s: f64,
    pub max_accel_mm_s2: f64,
    pub kernel_jerk_per_accel_hz: f64,
}

/// A ratio step as the leaders cross it: the step, the larger ratio on
/// either side, the spans either side ([`RatioSlide`]), and the magnitudes
/// of the leaders' speed, acceleration and jerk there.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RatioStep {
    pub ratio_step: f64,
    pub peak_ratio: f64,
    pub slides: [RatioSlide; 2],
    pub speed_mm_s: f64,
    pub accel_mm_s2: f64,
    pub jerk_mm_s3: f64,
}

impl RatioStep {
    /// The steepest ratio slope the moving window carries beside the step
    /// on a ramp of `tau` seconds at `speed_mm_s`
    /// ([`RatioSlide::moving_slope`]).
    fn moving_slope(&self, speed_mm_s: f64, tau: f64) -> f64 {
        self.slides
            .iter()
            .map(|slide| slide.moving_slope(speed_mm_s * tau))
            .fold(0.0, f64::max)
    }
}

/// A run of the ratio table beside a ratio step: its ratio slope per
/// millimetre and the path it slopes over.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RatioSlide {
    pub slope: f64,
    pub length_mm: f64,
}

impl RatioSlide {
    /// The flat run of a step's side that has none.
    pub const FLAT: Self = Self {
        slope: 0.0,
        length_mm: f64::INFINITY,
    };

    /// The ratio slope the moving window carries over the run where the
    /// step's window is `ramp_path_mm/2` wide, `ramp_path_mm = v·τ`: all of
    /// it where the run is at least that long, none where it is at most
    /// half that, and in between a share rising linearly with the length,
    /// so a ramp time derived from it moves continuously with the run. The
    /// run joins its ends' cluster where their windows overlap across it;
    /// a run that leaves them apart is at least as long as the narrower
    /// window is wide twice, so the step at its narrower end reads all of
    /// its slope.
    #[must_use]
    pub fn moving_slope(&self, ramp_path_mm: f64) -> f64 {
        let half = 0.5 * ramp_path_mm;
        let share = if half > 0.0 {
            ((self.length_mm - half) / half).clamp(0.0, 1.0)
        } else {
            1.0
        };
        self.slope.abs() * share
    }

    /// The most `v·|(h²)‴|` along the run where the moving window lies
    /// within it at `speed_mm_s` on a ramp of `tau` seconds: at most
    /// `v·h·|(h²)‴|/h` at the ramp's width `h = v·τ/2`, and at most
    /// `v·L·|(h²)‴|/L` over a gap between width nodes, at least the path
    /// `L = v·`[`RATIO_RAMP_NODE_SPACING_S`] the leaders cover between two,
    /// with `h` at most half the run ([`width_square_curvature_per_gap`]):
    /// so a ramp a rest narrows bends `h²` no sharper than its nodes'
    /// spacing allows, and a short run's window no sharper than its width
    /// allows.
    fn width_square_curvature_per_speed(&self, speed_mm_s: f64, tau: f64) -> f64 {
        let widths_per_gap = 0.5 * self.length_mm / (speed_mm_s * RATIO_RAMP_NODE_SPACING_S);
        (2.0 * WIDTH_SQUARE_CURVATURE_BOUND / tau)
            .min(width_square_curvature_per_gap(widths_per_gap) / RATIO_RAMP_NODE_SPACING_S)
    }
}

/// A ratio step as the planner lets the leaders cross it: `step` at their
/// shaped speed there, `rest_room_mm` along the path from their nearest
/// rest, their shaped speed at most `edge_speed_mm_s` half the
/// [`longest_ramp_time`] either side of it, and their own advanced flow
/// there at most `own_flow_mm_s`, which the planner can bound tighter than
/// the step's speed and acceleration alone do.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StepCrossing {
    pub step: RatioStep,
    pub rest_room_mm: f64,
    pub edge_speed_mm_s: f64,
    pub own_flow_mm_s: f64,
}

/// `max_u (S(u) + 6p·u(1 − u))` with `S(u) = 3u² − 2u³`.
fn velocity_overshoot(p: f64) -> f64 {
    let u = 0.5 * ((1.0 - 2.0 * p) + (1.0 + 4.0 * p * p).sqrt());
    u * u * (3.0 - 2.0 * u) + 6.0 * p * u * (1.0 - u)
}

/// `max_u |S′(u) + p·S″(u)|`.
fn acceleration_gain(p: f64) -> f64 {
    if p >= 0.5 { 6.0 * p } else { 1.5 + 6.0 * p * p }
}

/// `k·v/(w·τ)`, zero where the leaders neither move nor accelerate.
fn ramp_ratio(k: f64, speed: f64, weight: f64, tau: f64) -> f64 {
    if weight > 0.0 {
        k * speed / (weight * tau)
    } else {
        0.0
    }
}

impl RatioRampRule {
    /// The follower's peak velocity and acceleration over a ramp of `tau`
    /// seconds across `step`: the leaders' own advanced flow plus what the
    /// ramp adds to it.
    #[must_use]
    pub fn demand(&self, tau: f64, step: &RatioStep) -> (f64, f64) {
        let (own_velocity, own_accel) = self.own_demand(step);
        let (ramp_velocity, ramp_accel) = self.ramp_demand(tau, step);
        (own_velocity + ramp_velocity, own_accel + ramp_accel)
    }

    /// The advanced flow at the larger ratio about `step`, ramp or not.
    fn own_demand(&self, step: &RatioStep) -> (f64, f64) {
        let k = self.advance_slope_s;
        let (v, a, j) = (
            step.speed_mm_s,
            step.accel_mm_s2.abs(),
            step.jerk_mm_s3.abs(),
        );
        (step.peak_ratio * (v + k * a), step.peak_ratio * (a + k * j))
    }

    /// What a ramp of `tau` seconds adds to the follower's peak velocity
    /// and acceleration across `step`: proportional to the step and the
    /// moving window's slope bend, and shrinking as the ramp lengthens.
    fn ramp_demand(&self, tau: f64, step: &RatioStep) -> (f64, f64) {
        (self.ramp_velocity(tau, step), self.ramp_accel(tau, step))
    }

    fn ramp_velocity(&self, tau: f64, step: &RatioStep) -> f64 {
        let k = self.advance_slope_s;
        let (v, a) = (step.speed_mm_s, step.accel_mm_s2.abs());
        let advanced = v + k * a;
        let overshoot = step.ratio_step
            * (velocity_overshoot(ramp_ratio(k, v, advanced, tau)) - 1.0)
            * advanced;
        let bend =
            KERNEL_HALF_VARIANCE * step.moving_slope(v, tau) * WIDTH_SQUARE_SLOPE_BOUND * k * v * v;
        overshoot + bend
    }

    /// The ramp's own acceleration, and the moving window's slope bend
    /// `0.1·m·v·((h²)″·(v + 3k·a) + k·v·(h²)‴·v)` along the slide that
    /// bends it the most ([`RatioSlide::width_square_curvature_per_speed`]).
    fn ramp_accel(&self, tau: f64, step: &RatioStep) -> f64 {
        let k = self.advance_slope_s;
        let (v, a) = (step.speed_mm_s, step.accel_mm_s2.abs());
        let rate = v + 2.0 * k * a;
        let ramp = step.ratio_step * rate / tau * acceleration_gain(ramp_ratio(k, v, rate, tau));
        let bend = step
            .slides
            .iter()
            .map(|slide| {
                KERNEL_HALF_VARIANCE
                    * slide.moving_slope(v * tau)
                    * v
                    * (WIDTH_SQUARE_SLOPE_BOUND * (v + 3.0 * k * a)
                        + k * v * slide.width_square_curvature_per_speed(v, tau))
            })
            .fold(0.0, f64::max);
        ramp + bend
    }

    /// Whether the leaders crossing a ratio step as `crossing` lays out keep
    /// the follower within its limits over the longest ramp the step's rest
    /// room leaves it. The projection lays the step's window `V·τ/2` wide at
    /// the fastest speed `V` the leaders cross the ramp at, its edges', and
    /// a rest's cone narrows that window to the step's share of the room,
    /// so the room's ramp is the one it leaves at the edges' speed. The
    /// leaders' own flow, and the velocity the ramp adds to it, are read at
    /// the step: the overshoot `k·r̃′·v²` peaks inside the ramp, where the
    /// smoothed ratio climbs fastest. Without a follower kernel the
    /// acceleration the ramp adds is read at the ramp's edges, where the
    /// smoothed ratio's curvature steps and the advance turns it into
    /// `k·r̃″·v³`; a follower kernel spreads that step over its own width,
    /// so with one it is read at the step too.
    #[must_use]
    pub fn crossing_fits(&self, crossing: &StepCrossing) -> bool {
        let step = &crossing.step;
        assert!(
            crossing.edge_speed_mm_s >= step.speed_mm_s,
            "the leaders cross a ratio step at {} mm/s and its ramp's edges at {} mm/s",
            step.speed_mm_s,
            crossing.edge_speed_mm_s
        );
        let tau = longest_ramp_time(crossing.edge_speed_mm_s, crossing.rest_room_mm);
        let accel_read_at = if self.kernel_jerk_per_accel_hz.is_infinite() {
            RatioStep {
                speed_mm_s: crossing.edge_speed_mm_s,
                ..*step
            }
        } else {
            *step
        };
        let (own_velocity, own_accel) = self.own_demand(step);
        own_velocity.min(crossing.own_flow_mm_s) + self.ramp_velocity(tau, step)
            <= self.max_velocity_mm_s
            && own_accel + self.ramp_accel(tau, &accel_read_at) <= self.max_accel_mm_s2
    }

    /// How much a ramp across `step` may add to the follower's velocity and
    /// acceleration: what the leaders' own flow leaves of each limit, and
    /// never under [`RAMP_HEADROOM_FLOOR`] of it.
    fn ramp_headroom(&self, step: &RatioStep) -> (f64, f64) {
        let (own_velocity, own_accel) = self.own_demand(step);
        let headroom = |limit: f64, own: f64| (limit - own).max(RAMP_HEADROOM_FLOOR * limit);
        (
            headroom(self.max_velocity_mm_s, own_velocity),
            headroom(self.max_accel_mm_s2, own_accel),
        )
    }

    /// The ramp time for `step`: the shortest whose demand fits within the
    /// ramp's headroom ([`Self::ramp_headroom`]), or the longest ramp where
    /// none does. Its demand is continuous in the ramp time and every input
    /// and falls as the ramp lengthens, and the headroom is continuous and
    /// positive, so the ramp time is continuous in its inputs and the
    /// shortest as the step and the slopes vanish.
    #[must_use]
    pub fn ramp_time(&self, step: &RatioStep) -> f64 {
        assert!(
            [
                step.ratio_step,
                step.peak_ratio,
                step.speed_mm_s,
                step.accel_mm_s2
            ]
            .iter()
            .all(|value| value.is_finite())
                && step.ratio_step >= 0.0
                && step.speed_mm_s >= 0.0
                && step
                    .slides
                    .iter()
                    .all(|slide| slide.slope.is_finite() && slide.length_mm >= 0.0),
            "a ratio ramp is asked for an ill-formed step {step:?}"
        );
        let (velocity_headroom, accel_headroom) = self.ramp_headroom(step);
        let fits = |tau: f64| {
            let (velocity, accel) = self.ramp_demand(tau, step);
            velocity <= velocity_headroom && accel <= accel_headroom
        };
        if fits(RATIO_RAMP_MIN_TIME_S) {
            return RATIO_RAMP_MIN_TIME_S;
        }
        if !fits(RATIO_RAMP_MAX_TIME_S) {
            return RATIO_RAMP_MAX_TIME_S;
        }
        let (mut short, mut long) = (RATIO_RAMP_MIN_TIME_S, RATIO_RAMP_MAX_TIME_S);
        for _ in 0..BISECTIONS {
            let mid = 0.5 * (short + long);
            if fits(mid) {
                long = mid;
            } else {
                short = mid;
            }
        }
        long
    }

    /// The largest planned speed up to `fastest_mm_s` at which the leaders
    /// cross a ratio step, as `crossing_at` lays the crossing out at it,
    /// within the follower's limits ([`Self::crossing_fits`]): infinite
    /// where even `fastest_mm_s` does, zero where even a crossing at rest
    /// does not. Every demand grows with the speed, so bisection finds it.
    #[must_use]
    pub fn step_speed_cap(
        &self,
        fastest_mm_s: f64,
        crossing_at: impl Fn(f64) -> StepCrossing,
    ) -> f64 {
        assert!(
            fastest_mm_s.is_finite() && fastest_mm_s >= 0.0,
            "the leaders cross a ratio step at up to {fastest_mm_s} mm/s"
        );
        let fits = |speed: f64| self.crossing_fits(&crossing_at(speed));
        if fits(fastest_mm_s) {
            return f64::INFINITY;
        }
        if !fits(0.0) {
            return 0.0;
        }
        let (mut slow, mut fast) = (0.0, fastest_mm_s);
        for _ in 0..BISECTIONS {
            let mid = 0.5 * (slow + fast);
            if fits(mid) {
                slow = mid;
            } else {
                fast = mid;
            }
        }
        slow
    }
}

#[cfg(test)]
mod tests;
