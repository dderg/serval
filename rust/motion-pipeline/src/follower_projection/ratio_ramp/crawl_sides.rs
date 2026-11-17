use std::cell::OnceCell;
use std::collections::VecDeque;

use geometry::{
    ADVANCE_VALLEY_SPEED_MM_S, CrawlSide, RatioRampRule, VALLEY_LEAD_BUDGET_S, crawl_side_width,
    crawl_step_window, kept_floor, side_weight, turns_around,
};

use super::super::{RatioSpan, SPAN_LOOKUP_SLACK_MM, Table};
use super::{Crossing, NODE_SPACING_S, RampLeg, RatioSmoothing, Seam, StepRamps};

/// The fewest intervals per leg the side trail reads the leaders at; a
/// longer leg gets one per [`NODE_SPACING_S`]. The most the leaders can be
/// between two probes, `(v₁ + v₂ + A·Δt)/2`, is a side's crossing speed
/// ([`crawl_side`]), so the spacing keeps it within `A·Δt/2` of the
/// probes' own.
const TRAIL_PROBES_PER_LEG: usize = 16;

/// How far from its crawl, in step windows, a crawl valley owns ratio
/// steps ([`StepWindows::own_near`]): within two, [`crawl_step_window`]'s
/// share holds a step's flow to the shortest ramp's at the crawl's speed.
const OWNED_STEP_WINDOWS: f64 = 2.0;

/// How far from its crawl, in step windows, a crawl valley holds the
/// advance at zero: an owned run's farthest seam, and that seam's own
/// window. A window is a sixth of the leaders' reach, so the stretch lies
/// within half the reach, which leaders crossing the crawl at its speed
/// and rate leave within half of [`geometry::VALLEY_RAMP_TIME_S`], 15 ms.
const ZERO_REACH_WINDOWS: f64 = OWNED_STEP_WINDOWS + 1.0;

/// The leaders at one probe of the side trail: the leg it reads, where on
/// the path, how fast, and the path speed's rate `s̈`, signed, as they
/// leave the probe ([`Crossing::speed_and_path_accel`]).
#[derive(Debug, Clone, Copy, PartialEq)]
struct TrailProbe {
    leg: u64,
    t: f64,
    s: f64,
    speed: f64,
    path_accel: f64,
}

/// The most the leaders ask along a stretch of the side trail: how fast,
/// `|s̈|`, and the range of `s̈` away from the crawl.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct SideMotion {
    pub(super) fastest: f64,
    pub(super) path_accel: f64,
    pub(super) outward: (f64, f64),
}

/// The leaders along the path, probed leg by leg, as far back as a crawl
/// valley's side may still reach: what its rebuild leaves of the
/// follower's limit is read off it ([`crawl_side`]).
#[derive(Debug, Default)]
pub(super) struct SideTrail {
    probes: VecDeque<TrailProbe>,
    legs: u64,
}

impl SideTrail {
    /// Probes the leaders of `leg` over its support, the path read off the
    /// odometer that summed its arc and no further than `table_end`.
    pub(super) fn record(&mut self, leg: &RampLeg, table_end: f64) {
        let (t_start, t_end) = leg.support;
        self.legs += 1;
        let intervals =
            TRAIL_PROBES_PER_LEG.max(((t_end - t_start) / NODE_SPACING_S).ceil() as usize);
        for index in 0..=intervals {
            let t = if index == intervals {
                t_end
            } else {
                t_start + (t_end - t_start) * index as f64 / intervals as f64
            };
            let s = (leg.start_s + leg.odometer.covered_until(t)).min(table_end);
            let (speed, path_accel) = Crossing::speed_and_path_accel(leg.shaped_leaders, t);
            self.probes.push_back(TrailProbe {
                leg: self.legs,
                t,
                s,
                speed,
                path_accel,
            });
        }
    }

    /// The furthest path probed.
    pub(super) fn through(&self) -> f64 {
        self.probes
            .back()
            .map_or(f64::NEG_INFINITY, |probe| probe.s)
    }

    /// How many legs the trail has probed.
    pub(super) fn legs(&self) -> u64 {
        self.legs
    }

    /// The trail as it stood once its first `legs` legs were probed.
    pub(super) fn as_of(&self, legs: u64) -> Self {
        Self {
            probes: self
                .probes
                .iter()
                .filter(|probe| probe.leg <= legs)
                .copied()
                .collect(),
            legs,
        }
    }

    /// The path the leaders had reached [`VALLEY_LEAD_BUDGET_S`] before the
    /// last probe at or before `s`, at the latest probe no later: no side of
    /// a crawl valley at `s` reaches back past it. `None` where the trail
    /// does not cover that long.
    pub(super) fn lead_budget_back_from(&self, s: f64) -> Option<f64> {
        let at = self
            .probes
            .partition_point(|probe| probe.s <= s)
            .checked_sub(1)?;
        let since = self.probes[at].t - VALLEY_LEAD_BUDGET_S;
        let index = self.probes.partition_point(|probe| probe.t <= since);
        (index > 0).then(|| self.probes[index - 1].s)
    }

    /// Forgets the probes no side reads any more: those wholly before both
    /// the path `s` and the time [`VALLEY_LEAD_BUDGET_S`] before the last.
    pub(super) fn forget_before(&mut self, s: f64) {
        let Some(last) = self.probes.back().copied() else {
            return;
        };
        let since = last.t - VALLEY_LEAD_BUDGET_S;
        while self
            .probes
            .get(1)
            .is_some_and(|next| next.s <= s && next.t <= since)
        {
            self.probes.pop_front();
        }
    }

    /// The most the leaders ask between the probes that bracket the path
    /// `[lo, hi]` of a side reaching `side` from its crawl.
    fn over(&self, (lo, hi): (f64, f64), side: Side) -> SideMotion {
        let first = self
            .probes
            .partition_point(|probe| probe.s < lo)
            .saturating_sub(1);
        let last = self.probes.partition_point(|probe| probe.s <= hi);
        let bracket = self.probes.range(first..(last + 1).min(self.probes.len()));
        let (fastest, path_accel, outward) = bracket.fold(
            (0.0_f64, 0.0_f64, (f64::INFINITY, f64::NEG_INFINITY)),
            |(fastest, path_accel, (lo, hi)), probe| {
                let out = probe.outward(side);
                (
                    fastest.max(probe.speed),
                    path_accel.max(probe.path_accel.abs()),
                    (lo.min(out), hi.max(out)),
                )
            },
        );
        SideMotion {
            fastest,
            path_accel,
            outward: if outward.0 <= outward.1 {
                outward
            } else {
                (0.0, 0.0)
            },
        }
    }

    /// The intervals between neighbouring probes of one leg on the path
    /// `[lo, hi]` of a side reaching `side` from its crawl, and next to
    /// them across its ends, each with the mean `s⃛` over it: the change of
    /// `s̈` over the interval's length. The leaders are a fit, which holds
    /// their acceleration within its tolerance `A` but leaves their jerk
    /// free, so a sliver piece may carry any; the mean is within `2A/Δt` of
    /// the leaders' own whatever pieces the fit lays (as [`Crossing::over`]
    /// reads it). Legs' supports overlap, so an interval spans one leg's
    /// probes only.
    fn intervals_within(
        &self,
        (lo, hi): (f64, f64),
        side: Side,
    ) -> impl Iterator<Item = Interval<'_>> + Clone {
        let first = self.probes.partition_point(|probe| probe.s < lo);
        let last = self.probes.partition_point(|probe| probe.s <= hi);
        let (first, last) = match side {
            Side::After => (first.saturating_sub(1), last),
            Side::Before => (first, (last + 1).min(self.probes.len())),
        };
        (first.saturating_sub(1)..last.min(self.probes.len().saturating_sub(1))).filter_map(
            |index| {
                let (from, to) = (&self.probes[index], &self.probes[index + 1]);
                (from.leg == to.leg && to.t > from.t).then(|| {
                    (
                        from,
                        to,
                        (to.path_accel - from.path_accel) / (to.t - from.t),
                    )
                })
            },
        )
    }

    /// `interval` read across at least `span` of the leaders' time about
    /// its middle, between the probes of any leg nearest outside it. The
    /// fit holds `s̈` within its tolerance `A` and leaves the jerk free, so
    /// over an interval of a sliver leg, 1e-5 mm long, the mean `Δs̈/Δt`
    /// reads a sliver piece's jerk, within only `2A/Δt` of the leaders'
    /// own; across `span` that error is `2A/span`. An interval as long
    /// already, or one the trail has no probe beyond, reads as it is.
    fn read_across<'a>(&'a self, interval: Interval<'a>, span: f64) -> Interval<'a> {
        let (from, to, _) = interval;
        if to.t - from.t >= span {
            return interval;
        }
        let middle = 0.5 * (from.t + to.t);
        let start = self
            .probes
            .iter()
            .filter(|probe| probe.t <= middle - 0.5 * span)
            .max_by(|left, right| left.t.total_cmp(&right.t));
        let end = self
            .probes
            .iter()
            .filter(|probe| probe.t >= middle + 0.5 * span)
            .min_by(|left, right| left.t.total_cmp(&right.t));
        let (Some(start), Some(end)) = (start, end) else {
            return interval;
        };
        (
            start,
            end,
            (end.path_accel - start.path_accel) / (end.t - start.t),
        )
    }
}

/// Two neighbouring probes of one leg and the mean `s⃛` between them
/// ([`SideTrail::intervals_within`]).
type Interval<'a> = (&'a TrailProbe, &'a TrailProbe, f64);

/// Whether the leaders may crawl between the probes of `interval`, under
/// [`ADVANCE_VALLEY_SPEED_MM_S`]: the slowest they can be there, each
/// probe's speed less what the larger `|s̈|` takes off toward the other,
/// is `(v₁ + v₂ − |s̈|·Δt)/2`.
fn may_crawl((from, to, _): Interval) -> bool {
    let decline = from.path_accel.abs().max(to.path_accel.abs()) * (to.t - from.t).abs();
    0.5 * (from.speed + to.speed - decline) < ADVANCE_VALLEY_SPEED_MM_S
}

impl TrailProbe {
    /// The path acceleration away from the crawl a side reaching `side`
    /// reads here: `s̈` after it, `−s̈` before; where the leaders rest or
    /// turn around their speed rises off zero either way.
    fn outward(&self, side: Side) -> f64 {
        if turns_around(self.speed, self.path_accel.abs()) {
            return self.path_accel.abs();
        }
        match side {
            Side::After => self.path_accel,
            Side::Before => -self.path_accel,
        }
    }
}

/// Which way from its crawl a valley's side reaches.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) enum Side {
    Before,
    After,
}

/// Why a side's width is not laid yet.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) enum Unlaid {
    /// The side may reach past the path probed or the table laid.
    Waits,
}

/// Where a crawl valley's side leaves its zero stretch: the path `s` of
/// the stretch's edge, how the leaders cross it, and the floor the side
/// keeps flat there.
#[derive(Debug, Clone, Copy)]
pub(super) struct Edge {
    pub(super) s: f64,
    pub(super) crossing: Crossing,
    pub(super) floor: f64,
}

impl Edge {
    /// The edge at the crawl `turn` itself, with the crawl's floor
    /// ([`Crossing::crawl_floor`]).
    pub(super) fn at_crawl(turn: f64, crossing: Crossing) -> Self {
        Self {
            s: turn,
            crossing,
            floor: crossing.crawl_floor(),
        }
    }
}

/// What a crawl valley's sides read as laid: the span table up to `table`,
/// infinite where the leaders rest past the trail; the leaders' trail; and
/// the smoothed ratio, final up to `smoothing`
/// ([`RatioSmoothing::final_through`]).
#[derive(Debug, Clone, Copy)]
pub(super) struct SideReads<'a> {
    pub(super) table: f64,
    pub(super) trail: &'a SideTrail,
    pub(super) smoothing: f64,
}

/// The width of the `side` of the crawl valley leaving its zero stretch at
/// `edge`: the narrowest [`crawl_side_width`] lays where, over every
/// interval between neighbouring probes of the trail over it, the
/// leaders' own demand there and the rebuild `k·|W″·ṡ³ + 3W′·ṡ·s̈|` there
/// ([`geometry::RebuildsWithin::fits_between`], within the bound over the
/// whole side, [`CrawlSide::rebuild`]), both at `r`, fit the follower's
/// limit; `r` the largest ratio the smoothed advance table, laid as
/// `advance`, reads over the part of the interval within the side
/// ([`RatioSmoothing::peak_ratio_in`]), and the leaders read off the
/// trail, never under the leaders' `reach`; `None` where none reaching no
/// further than `limit` fits. The probes say nothing of the leaders
/// between them but how fast `s̈` lets their speed change, which past the
/// edge of a zero stretch the leaders cross at speed is where the rebuild
/// bends hardest. The turning guard proves the same sum cell by cell for
/// the widest side the lead budget covers, so a side no wider fits here.
/// With an infinite table end the leaders rest past the trail, and the
/// side reads it as far as it goes; else a side whose ratio the smoothing
/// reads past the table's end waits for the spans laid there
/// ([`RatioSmoothing::reads_within`]). A side waits too until the
/// smoothed ratio over it is final: a step's ramp, and whether the side
/// budgets it, follows the width at the step, which nodes still to come
/// may move. It fails loudly where the leaders leave the follower no room
/// in any interval: it never narrows a side to
/// make it fit. An interval whose own reading passes the limit is read
/// again across the width field's node spacing first
/// ([`SideTrail::read_across`]): a sliver leg's mean reads the fit, not
/// the leaders. Where that still passes the limit across an interval the
/// leaders may crawl in (a reversal inside the side), it asks `|s̈|` alone:
/// the turning there is that crawl's, whose own valley holds the weight
/// at zero over it, as the turning guard asks nothing of the turning
/// term below [`ADVANCE_VALLEY_SPEED_MM_S`].
///
/// The advance reads `r·(s̈ + k·Φ·s⃛)`, `Φ` the product of the valleys'
/// weights, at most this side's `W(d)` at a distance `d` from the crawl:
/// `|s̈ + k·Φ·s⃛|` is linear in `Φ`, so at most `|s̈|` or `|s̈ + k·W·s⃛|`,
/// read signed at the corners of each interval's `s̈` and `W`, with its
/// mean `s⃛` ([`SideTrail::intervals_within`]).
///
/// A ratio step the valley does not own ([`StepWindows`]) ramps inside
/// the side: `R = W·r̃` adds the ramp's flow `r̃′·ṡ²`, its advance
/// `k·W·(r̃″·ṡ³ + 3r̃′·ṡ·s̈)`, and `2k·W′·r̃′·ṡ³` where it meets the rebuild.
/// Each interval takes the first two at the largest weight its probes
/// read and the bounds of the step clusters' `|r̃′|`, `|r̃″|` over it
/// ([`RatioSmoothing::step_ramps`]), at the fastest the leaders may cross
/// it, `(v₁ + v₂ + A·Δt)/2`, from the room the rebuild gets; the third
/// reads as more `s̈`, `(2/3)·|r̃′|·ṡ²/r` more, in the rebuild's
/// `3W′·ṡ·s̈`. The advanced velocity `r̃·ṡ + k·(W·(r̃′·ṡ² + r̃·s̈) + W′·r̃·ṡ²)`
/// fits the follower's velocity limit the same way, `W′·r̃·ṡ²` being the
/// lead the rebuild adds ([`side_slope_within`]). The side widens until
/// all of it fits.
///
/// A follower behind its own kernel keeps the leaders' reach: the turning
/// guard holds kernel-free followers only and leaves no room for its
/// sides, so a wider one would reach past what the projection holds back.
pub(super) fn crawl_side(
    side: Side,
    edge: Edge,
    reach: f64,
    (smoothing, (laid, advance), reads): (
        &RatioSmoothing,
        (&[RatioSpan], &[RatioSpan]),
        SideReads<'_>,
    ),
    steps: &StepWindows,
    rule: &RatioRampRule,
    limit: f64,
) -> Result<Option<f64>, Unlaid> {
    if rule.kernel_jerk_per_accel_hz.is_finite() {
        return Ok(Some(reach));
    }
    let trail = reads.trail;
    let slope = rule.advance_slope_s;
    let Edge {
        s: turn,
        crossing,
        floor,
    } = edge;
    let mut waits = false;
    let width = crawl_side_width(reach, |width| {
        if width > limit {
            return true;
        }
        let window = match side {
            Side::Before => (turn - width, turn),
            Side::After => (turn, turn + width),
        };
        if reads.table.is_finite()
            && (window.1 > trail.through().min(reads.table)
                || !smoothing.reads_within(Table::Advance, window.1, reads.table))
        {
            waits = true;
            return true;
        }
        let window = (window.0, window.1.min(trail.through()));
        if window.1 > reads.smoothing {
            waits = true;
            return true;
        }
        let motion = trail.over(window, side);
        let distance_of = |probe: &TrailProbe| match side {
            Side::Before => turn - probe.s,
            Side::After => probe.s - turn,
        };
        let own_over = |(from, to, path_jerk): Interval| {
            let weights = [from, to].map(|probe| side_weight(distance_of(probe), width, floor).0);
            [from.path_accel, to.path_accel]
                .into_iter()
                .flat_map(|accel| {
                    weights
                        .map(|weight| accel.abs().max((accel + slope * weight * path_jerk).abs()))
                })
                .fold(0.0, f64::max)
        };
        let clipped = |(from, to, _): Interval| {
            (
                from.s.min(to.s).max(window.0),
                from.s.max(to.s).min(window.1),
            )
        };
        let ratio_over = |interval: Interval| {
            let (lo, hi) = clipped(interval);
            if lo > hi {
                return 0.0;
            }
            smoothing.peak_ratio_in(Table::Advance, advance, lo, hi)
        };
        let ramps_over = |interval: Interval| {
            let (lo, hi) = clipped(interval);
            if lo > hi {
                return StepRamps::default();
            }
            smoothing.step_ramps((lo, hi), &|table, s| steps.owns(table, s))
        };
        let accel = motion.path_accel.max(crossing.rate);
        let crossing_speed =
            |(from, to, _): Interval| 0.5 * (from.speed + to.speed + accel * (to.t - from.t).abs());
        let velocity_fits = |interval: Interval, ratio: f64| {
            let (from, to, _) = interval;
            let (lo, hi) = clipped(interval);
            let flow_ratio = smoothing.peak_ratio_in(Table::Laid, laid, lo, hi);
            let ramps = ramps_over(interval);
            let speed = crossing_speed(interval);
            let interval_accel = from.path_accel.abs().max(to.path_accel.abs());
            let (near, far) = {
                let distances = [from, to].map(|probe| distance_of(probe));
                (
                    distances[0].min(distances[1]),
                    distances[0].max(distances[1]),
                )
            };
            let (weight, steepest) = side_slope_within((near, far), width, floor);
            let lead = weight * (ramps.slope * speed * speed + ratio * interval_accel)
                + steepest * ratio * speed * speed;
            flow_ratio * speed + slope * lead <= rule.max_velocity_mm_s
        };
        let rebuilds = OnceCell::new();
        let rebuild_fits = |interval: Interval, ratio: f64, own: f64| {
            let (whole, within) = rebuilds.get_or_init(|| {
                let leaders = CrawlSide {
                    speed: crossing.speed,
                    accel,
                    fastest: motion.fastest.max(crossing.speed),
                    floor: (floor, floor),
                    outward: (
                        motion.outward.0.clamp(-accel, accel),
                        motion.outward.1.clamp(-accel, accel),
                    ),
                    signed_from: 0.0,
                };
                (leaders.rebuild(width), leaders.rebuilds_within(width))
            });
            let (from, to, _) = interval;
            let ramps = ramps_over(interval);
            let interval_accel = from.path_accel.abs().max(to.path_accel.abs());
            let speed = crossing_speed(interval);
            let weight = [from, to]
                .map(|probe| side_weight(distance_of(probe), width, floor).0)
                .into_iter()
                .fold(0.0, f64::max);
            let ramp_ask = ramps.flow * speed * speed
                + slope
                    * weight
                    * speed
                    * (ramps.curvature * speed * speed + 3.0 * ramps.slope * interval_accel);
            let room = ((rule.max_accel_mm_s2 - ramp_ask) / ratio - own) / slope;
            let lead_accel = interval_accel + 2.0 / 3.0 * ramps.slope / ratio * speed * speed;
            let readings = [from, to].map(|probe| (distance_of(probe), probe.speed));
            let (near, far) = if readings[0].0 <= readings[1].0 {
                (readings[0], readings[1])
            } else {
                (readings[1], readings[0])
            };
            (ramps.slope == 0.0 && *whole <= room)
                || within.fits_between(near, far, (accel, lead_accel), room)
        };
        trail
            .intervals_within(window, side)
            .fold(true, |fits, interval| {
                let ratio = ratio_over(interval);
                let interval = if ratio * own_over(interval) >= rule.max_accel_mm_s2 {
                    trail.read_across(interval, NODE_SPACING_S)
                } else {
                    interval
                };
                let own = match own_over(interval) {
                    own if ratio * own >= rule.max_accel_mm_s2 && may_crawl(interval) => {
                        interval.0.path_accel.abs().max(interval.1.path_accel.abs())
                    }
                    own => own,
                };
                assert!(
                    ratio * own < rule.max_accel_mm_s2,
                    "follower projection: the leaders ask {} mm/s² of the follower between \
                     s={} and s={} within {width} mm {side:?} the crawl at s={turn} \
                     ({crossing:?}, {motion:?}, ratio {ratio}), past its limit of {} mm/s² \
                     before its valley rebuilds the lead",
                    ratio * own,
                    interval.0.s,
                    interval.1.s,
                    rule.max_accel_mm_s2
                );
                fits && (ratio == 0.0
                    || slope == 0.0
                    || (rebuild_fits(interval, ratio, own) && velocity_fits(interval, ratio)))
            })
    });
    if waits {
        return Err(Unlaid::Waits);
    }
    Ok(width.filter(|&width| width <= limit))
}

/// `(max W, max |W′|)` over the distances `[near, far]` from the zero
/// stretch of a side `width` wide whose floor is `floor`
/// ([`geometry::side_weight`]): `W` rises with the distance, and
/// `|W′| = 3ι²σ/w̃` is at most the share ahead `ι` read at `near` times
/// the slope `σ` read at `far`.
fn side_slope_within((near, far): (f64, f64), width: f64, floor: f64) -> (f64, f64) {
    let (near, far) = (near.clamp(0.0, width), far.clamp(0.0, width));
    if far == 0.0 || near == width {
        return (side_weight(far, width, floor).0, 0.0);
    }
    let kept = kept_floor(floor, width);
    let ahead = libm::cbrt(1.0 - side_weight(near, width, floor).0);
    let rising = if kept == 0.0 {
        1.0
    } else {
        far / libm::hypot(far, kept)
    };
    let span = width * width / (libm::hypot(width, kept) + kept);
    (
        side_weight(far, width, floor).0,
        3.0 * ahead * ahead * rising / span,
    )
}

/// The ratio steps a crawl valley holds the weight at zero around: each a
/// step within [`OWNED_STEP_WINDOWS`] of the crawl whose window is kept
/// within `window` of it ([`geometry::crawl_step_window`]), with every
/// seam of the sloped run of spans either side of it, which the smoothing
/// may join into one cluster ramped over the narrowest of their windows
/// ([`SeamTable::joins`]), the whole run within that reach too; the
/// valley's zero stretch spans the crawl and every owned run's windows,
/// within [`ZERO_REACH_WINDOWS`] of the crawl. Inside it the advance reads
/// none of a ramp; only its flow `r̃·ṡ` reaches the follower, whose
/// acceleration `r̃·s̈ + r̃′·ṡ²` takes `|r̃′| ≤ ¾·|Δ|/h` from a step over a
/// window of half-width `h`, besides the table's own slope. A narrower
/// window the width field lays is one the ramp rule budgets. The leaders'
/// path acceleration across the stretch is at most `A`, so `ṡ² ≤ v² + 2A·d`
/// a distance `d` from the crawl they cross at `v`. A step further out
/// ramps inside a side, whose width budgets its ramp ([`crawl_side`]):
/// owned, it would move the side past the next step, and on a slicer's
/// outline, a step every line, that runs on along the print.
pub(super) struct StepWindows {
    turn: f64,
    crossing: Crossing,
    pub(super) window: f64,
    guarded: bool,
    pub(super) owned: Vec<OwnedStep>,
}

/// A ratio step a crawl valley holds the weight at zero around: which
/// table it steps, where, and the seams of the sloped run of spans either
/// side of it, its own among them, whose windows the valley caps with it.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct OwnedStep {
    pub(super) table: Table,
    pub(super) s: f64,
    pub(super) run: Vec<f64>,
}

impl OwnedStep {
    /// The step `seam` of `spans` at their index `at`, with the seams of
    /// the sloped spans running from it either way.
    fn of(table: Table, spans: &[RatioSpan], at: usize, seam: &Seam) -> Self {
        let mut first = at;
        while first > 1 && spans[first - 1].ratio_slope() != 0.0 {
            first -= 1;
        }
        let mut past = at;
        while past + 1 < spans.len() && spans[past].ratio_slope() != 0.0 {
            past += 1;
        }
        let run = (first..=past)
            .filter(|&index| Seam::between(&spans[index - 1], &spans[index]).is_some())
            .map(|index| spans[index].s0)
            .collect();
        Self {
            table,
            s: seam.s,
            run,
        }
    }

    /// The first and last seam of its run.
    fn extent(&self) -> (f64, f64) {
        (self.run[0], self.run[self.run.len() - 1])
    }
}

impl StepWindows {
    /// No step owned yet around the crawl at `turn` the leaders cross as
    /// `crossing`, `reach` from it; a follower the turning guard does not
    /// hold owns none.
    pub(super) fn around(turn: f64, crossing: Crossing, reach: f64, guarded: bool) -> Self {
        Self {
            turn,
            crossing,
            window: crawl_step_window(reach),
            guarded,
            owned: Vec::new(),
        }
    }

    /// Owns every ratio step of either table whose run lies within
    /// [`OWNED_STEP_WINDOWS`] of the crawl, where the projection may still
    /// cap every seam of the run: `near` for a step within one window of
    /// the crawl, `farther` past it.
    pub(super) fn own_near(
        &mut self,
        (laid, advance): (&[RatioSpan], &[RatioSpan]),
        near: &dyn Fn(f64) -> bool,
        farther: &dyn Fn(f64) -> bool,
    ) {
        if !self.guarded {
            return;
        }
        let reach = OWNED_STEP_WINDOWS * self.window;
        let (lo, hi) = (self.turn - reach, self.turn + reach);
        for (table, spans) in [(Table::Laid, laid), (Table::Advance, advance)] {
            let first = spans
                .partition_point(|span| span.s1 < lo)
                .saturating_sub(1)
                .max(1);
            let last = spans.partition_point(|span| span.s0 <= hi);
            for at in first..last {
                let Some(seam) = Seam::between(&spans[at - 1], &spans[at]) else {
                    continue;
                };
                if seam.step == 0.0 || seam.s < lo || seam.s > hi {
                    continue;
                }
                let owned = OwnedStep::of(table, spans, at, &seam);
                let (first_seam, last_seam) = owned.extent();
                let cappable = if (seam.s - self.turn).abs() <= self.window {
                    near
                } else {
                    farther
                };
                if first_seam >= lo && last_seam <= hi && owned.run.iter().all(|&s| cappable(s)) {
                    self.owned.push(owned);
                }
            }
        }
    }

    /// Whether `table` has a seam at `s` the valley owns.
    pub(super) fn owns(&self, table: Table, s: f64) -> bool {
        self.owned
            .iter()
            .any(|owned| owned.table == table && owned.run.contains(&s))
    }

    /// Where the valley holds the weight at zero: the crawl, and every
    /// owned run's windows.
    pub(super) fn zero_stretch(&self) -> (f64, f64) {
        let zero = self
            .owned
            .iter()
            .fold((self.turn, self.turn), |(lo, hi), owned| {
                let (first, last) = owned.extent();
                (lo.min(first - self.window), hi.max(last + self.window))
            });
        let bound = ZERO_REACH_WINDOWS * self.window + SPAN_LOOKUP_SLACK_MM;
        assert!(
            self.turn - zero.0 <= bound && zero.1 - self.turn <= bound,
            "follower projection: the crawl valley at s={} holds the advance at zero over \
             [{}, {}], past {ZERO_REACH_WINDOWS} of its {} mm step windows",
            self.turn,
            zero.0,
            zero.1,
            self.window
        );
        zero
    }

    /// The most path acceleration the leaders take across the zero
    /// stretch, or at the crawl.
    fn path_accel(&self, trail: &SideTrail) -> f64 {
        trail
            .over(self.zero_stretch(), Side::After)
            .path_accel
            .max(self.crossing.rate)
    }

    /// `v² + 2A·d`: the most `ṡ²` a `distance` from the crawl.
    fn speed_squared_within(&self, path_accel: f64, distance: f64) -> f64 {
        self.crossing.speed * self.crossing.speed + 2.0 * path_accel * distance
    }

    /// Where the sides leave the zero stretch: the crawl itself where no
    /// owned window moves the edge, else the stretch's edge, which the
    /// leaders cross at most as fast as the stretch's path acceleration `A`
    /// takes them from the crawl, `v_e`. The leaders move there, so the
    /// side leaves zero flat over the floor `v_e²/A` whatever the crawl's.
    pub(super) fn edges(&self, trail: &SideTrail, table_end: f64) -> Result<(Edge, Edge), Unlaid> {
        let at_crawl = Edge::at_crawl(self.turn, self.crossing);
        if self.owned.is_empty() {
            return Ok((at_crawl, at_crawl));
        }
        let zero = self.zero_stretch();
        if zero.1 > trail.through() && table_end.is_finite() {
            return Err(Unlaid::Waits);
        }
        let path_accel = self.path_accel(trail);
        let edge = |s: f64| {
            let distance = (s - self.turn).abs();
            if distance == 0.0 {
                return at_crawl;
            }
            let speed_squared = self.speed_squared_within(path_accel, distance);
            Edge {
                s,
                crossing: Crossing {
                    speed: speed_squared.sqrt(),
                    rate: path_accel,
                    jerk: self.crossing.jerk,
                },
                floor: speed_squared / path_accel,
            }
        };
        Ok((edge(zero.0), edge(zero.1)))
    }

    /// Fails loudly where the flow over the zero stretch asks more than the
    /// follower's limit ([`Self::flow`]).
    pub(super) fn assert_flow_fits(
        &self,
        (smoothing, trail): (&RatioSmoothing, &SideTrail),
        laid: &[RatioSpan],
        rule: &RatioRampRule,
    ) {
        if self.owned.is_empty() {
            return;
        }
        let zero = self.zero_stretch();
        let path_accel = self.path_accel(trail);
        let flow = self.flow(smoothing, laid, path_accel);
        assert!(
            flow <= rule.max_accel_mm_s2,
            "follower projection: the ratio steps {:?} the crawl valley at s={} ({:?}) holds at \
             zero over [{}, {}] ask {flow} mm/s² of the follower's flow at a path acceleration \
             of {path_accel} mm/s², past its limit of {} mm/s²",
            self.owned,
            self.turn,
            self.crossing,
            zero.0,
            zero.1,
            rule.max_accel_mm_s2
        );
    }

    /// What the flow over the zero stretch asks of the follower at the path
    /// acceleration `A` across it: `r·A` at the largest ratio the smoothed
    /// laid table, laid as `laid`, reads there
    /// ([`RatioSmoothing::peak_ratio_in`]), and `¾·|Δ|·ṡ²/w` for each ratio
    /// step `Δ` of an owned run of the laid table capped to `w`, `ṡ²` read
    /// as far from the crawl as the step's own window reaches: a step ramps
    /// over its window alone, and the run's other seams only change the
    /// slope. The table's own slopes, which a window of any width averages
    /// to no steeper, are the planner's to hold.
    fn flow(&self, smoothing: &RatioSmoothing, laid: &[RatioSpan], path_accel: f64) -> f64 {
        let zero = self.zero_stretch();
        let ramps: f64 = self
            .laid_steps(laid)
            .map(|(_, s, step)| {
                let reaching = (s - self.turn).abs() + self.window;
                0.75 * step * self.speed_squared_within(path_accel, reaching) / self.window
            })
            .sum();
        smoothing.peak_ratio_in(Table::Laid, laid, zero.0, zero.1) * path_accel + ramps
    }

    /// Each seam of the owned runs of the laid table, laid as `laid`, that
    /// steps its ratio, once, with the owned step whose run holds it first
    /// and the step's size.
    fn laid_steps<'a>(
        &'a self,
        laid: &'a [RatioSpan],
    ) -> impl Iterator<Item = (&'a OwnedStep, f64, f64)> + 'a {
        let laid_runs = self
            .owned
            .iter()
            .enumerate()
            .filter(|(_, owned)| owned.table == Table::Laid);
        laid_runs
            .flat_map(move |(index, owned)| {
                owned.run.iter().copied().filter_map(move |s| {
                    let earlier = self.owned[..index]
                        .iter()
                        .any(|earlier| earlier.table == Table::Laid && earlier.run.contains(&s));
                    (!earlier).then_some((owned, s))
                })
            })
            .filter_map(move |(owned, s)| {
                let at = laid.partition_point(|span| span.s0 < s);
                assert!(
                    at > 0 && laid.get(at).is_some_and(|span| span.s0 == s),
                    "follower projection: a crawl valley owns a seam at s={s} the laid table no \
                     longer has"
                );
                let step = Seam::between(&laid[at - 1], &laid[at]).map_or(0.0, |seam| seam.step);
                (step != 0.0).then_some((owned, s, step.abs()))
            })
    }
}
