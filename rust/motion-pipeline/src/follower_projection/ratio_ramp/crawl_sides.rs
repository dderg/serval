use std::collections::VecDeque;

use geometry::{
    CrawlSide, RatioRampRule, VALLEY_LEAD_BUDGET_S, crawl_side_width, side_weight, turns_around,
};

use super::super::RatioSpan;
use super::{Crossing, RampLeg};

/// Probes per leg the side trail reads the leaders at, both ends included
/// once more.
const TRAIL_PROBES_PER_LEG: usize = 16;

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
        for index in 0..=TRAIL_PROBES_PER_LEG {
            let t = if index == TRAIL_PROBES_PER_LEG {
                t_end
            } else {
                t_start + (t_end - t_start) * index as f64 / TRAIL_PROBES_PER_LEG as f64
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

    /// The path the leaders had reached [`VALLEY_LEAD_BUDGET_S`] before the
    /// last probe, at the latest probe no later: no crawl valley still to
    /// lay reaches back past it. `None` before the trail covers that long.
    pub(super) fn lead_budget_back(&self) -> Option<f64> {
        let last = self.probes.back()?;
        let since = last.t - VALLEY_LEAD_BUDGET_S;
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

    /// The largest `read` of each probe on the path `[lo, hi]` of a side
    /// reaching `side` from its crawl, and of the probe next to them across
    /// the crawl, with the mean `s⃛` over each interval of its leg it
    /// bounds: the change of `s̈` over the interval's length. The leaders
    /// are a fit, which holds their acceleration within its tolerance `A`
    /// but leaves their jerk free, so a sliver piece may carry any; the
    /// mean is within `2A/Δt` of the leaders' own whatever pieces the fit
    /// lays (as [`Crossing::over`] reads it). Legs' supports overlap, so an
    /// interval spans one leg's probes only.
    fn most_within(
        &self,
        (lo, hi): (f64, f64),
        side: Side,
        read: impl Fn(&TrailProbe, f64) -> f64,
    ) -> f64 {
        let first = self.probes.partition_point(|probe| probe.s < lo);
        let last = self.probes.partition_point(|probe| probe.s <= hi);
        let (first, last) = match side {
            Side::After => (first.saturating_sub(1), last),
            Side::Before => (first, (last + 1).min(self.probes.len())),
        };
        let mean_jerk = |from: &TrailProbe, to: &TrailProbe| {
            (from.leg == to.leg && to.t > from.t)
                .then(|| (to.path_accel - from.path_accel) / (to.t - from.t))
        };
        let mut most = 0.0_f64;
        for index in first..last {
            let probe = &self.probes[index];
            let jerks = [
                index
                    .checked_sub(1)
                    .and_then(|previous| mean_jerk(&self.probes[previous], probe)),
                self.probes
                    .get(index + 1)
                    .and_then(|next| mean_jerk(probe, next)),
            ];
            for jerk in jerks.into_iter().flatten() {
                most = most.max(read(probe, jerk));
            }
        }
        most
    }
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

/// The width of the `side` of the crawl valley at `turn` the leaders cross
/// as `crossing`: the narrowest [`crawl_side_width`] lays where, at every
/// probe of the trail over it, the leaders' own demand there and the
/// rebuild `k·|W″·ṡ³ + 3W′·ṡ·s̈|` there ([`geometry::RebuildsWithin`] at the probe,
/// within the bound over the whole side, [`CrawlSide::rebuild`]), both at
/// `r`, fit the follower's limit; `r` the largest ratio laid within it and
/// the leaders read off the trail, never under the leaders' `reach`; `None`
/// where none reaching no further than `limit` fits. The turning guard
/// proves the same sum cell by cell for the widest side the lead budget
/// covers, so a side no wider fits here. With an infinite `table_end` the leaders rest past the
/// trail, and the side reads it as far as it goes. It fails loudly where
/// the leaders leave the follower no room: it never narrows a side to make
/// it fit.
///
/// The advance reads `r·(s̈ + k·Φ·s⃛)`, `Φ` the product of the valleys'
/// weights, at most this side's `W(d)` at a distance `d` from the crawl:
/// `|s̈ + k·Φ·s⃛|` is linear in `Φ`, so at most `|s̈|` or `|s̈ + k·W·s⃛|`,
/// read signed at each probe ([`SideTrail::most_within`]).
///
/// A follower behind its own kernel keeps the leaders' reach: the turning
/// guard holds kernel-free followers only and leaves no room for its
/// sides, so a wider one would reach past what the projection holds back.
pub(super) fn crawl_side(
    side: Side,
    (turn, crossing): (f64, Crossing),
    reach: f64,
    (trail, spans, table_end): (&SideTrail, &[RatioSpan], f64),
    rule: &RatioRampRule,
    limit: f64,
) -> Result<Option<f64>, Unlaid> {
    if rule.kernel_jerk_per_accel_hz.is_finite() {
        return Ok(Some(reach));
    }
    let slope = rule.advance_slope_s;
    let floor = crossing.crawl_floor();
    let mut waits = false;
    let width = crawl_side_width(reach, |width| {
        if width > limit {
            return true;
        }
        let window = match side {
            Side::Before => (turn - width, turn),
            Side::After => (turn, turn + width),
        };
        if window.1 > trail.through().min(table_end) && table_end.is_finite() {
            waits = true;
            return true;
        }
        let window = (window.0, window.1.min(trail.through()));
        let ratio = peak_ratio_over(spans, window);
        let motion = trail.over(window, side);
        let distance_of = |probe: &TrailProbe| match side {
            Side::Before => turn - probe.s,
            Side::After => probe.s - turn,
        };
        let own_at = |probe: &TrailProbe, path_jerk: f64| {
            let weight = side_weight(distance_of(probe), width, floor).0;
            probe
                .path_accel
                .abs()
                .max((probe.path_accel + slope * weight * path_jerk).abs())
        };
        let own = trail.most_within(window, side, own_at);
        let room = rule.max_accel_mm_s2 - ratio * own;
        assert!(
            room > 0.0,
            "follower projection: the leaders ask {} mm/s² of the follower within {width} mm \
             {side:?} the crawl at s={turn} ({crossing:?}, {motion:?}, ratio {ratio}), past its \
             limit of {} mm/s² before its valley rebuilds the lead",
            ratio * own,
            rule.max_accel_mm_s2
        );
        let accel = motion.path_accel.max(crossing.rate);
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
        let whole = leaders.rebuild(width);
        let within = leaders.rebuilds_within(width);
        let asked = trail.most_within(window, side, |probe, path_jerk| {
            let distance = distance_of(probe).max(0.0);
            let rebuild = within
                .at((distance, distance), probe.speed, probe.path_accel.abs())
                .min(whole);
            own_at(probe, path_jerk) + slope * rebuild
        });
        ratio * asked <= rule.max_accel_mm_s2
    });
    if waits {
        return Err(Unlaid::Waits);
    }
    Ok(width.filter(|&width| width <= limit))
}

/// The largest extrusion ratio `spans` lay over the path `[lo, hi]`.
fn peak_ratio_over(spans: &[RatioSpan], (lo, hi): (f64, f64)) -> f64 {
    spans
        .iter()
        .filter(|span| span.s1 >= lo && span.s0 <= hi)
        .map(|span| span.r0.abs().max(span.r1.abs()))
        .fold(0.0, f64::max)
}
