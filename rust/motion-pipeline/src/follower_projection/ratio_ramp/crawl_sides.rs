use std::cell::OnceCell;
use std::collections::VecDeque;

use geometry::{
    CrawlSide, RatioRampRule, VALLEY_LEAD_BUDGET_S, crawl_side_width, crawl_step_window,
    side_weight, turns_around,
};

use super::super::{RatioSpan, Table};
use super::{Crossing, RampLeg, RatioSmoothing, Seam};

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
}

/// Two neighbouring probes of one leg and the mean `s⃛` between them
/// ([`SideTrail::intervals_within`]).
type Interval<'a> = (&'a TrailProbe, &'a TrailProbe, f64);

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
/// With an infinite `table_end` the leaders rest past the trail, and the
/// side reads it as far as it goes; else a side whose ratio the smoothing
/// reads past `table_end` waits for the spans laid there
/// ([`RatioSmoothing::reads_within`]). It fails loudly where the leaders
/// leave the follower no room in any interval: it never narrows a side to
/// make it fit.
///
/// The advance reads `r·(s̈ + k·Φ·s⃛)`, `Φ` the product of the valleys'
/// weights, at most this side's `W(d)` at a distance `d` from the crawl:
/// `|s̈ + k·Φ·s⃛|` is linear in `Φ`, so at most `|s̈|` or `|s̈ + k·W·s⃛|`,
/// read signed at the corners of each interval's `s̈` and `W`, with its
/// mean `s⃛` ([`SideTrail::intervals_within`]).
///
/// A follower behind its own kernel keeps the leaders' reach: the turning
/// guard holds kernel-free followers only and leaves no room for its
/// sides, so a wider one would reach past what the projection holds back.
pub(super) fn crawl_side(
    side: Side,
    edge: Edge,
    reach: f64,
    (smoothing, advance, table_end): (&RatioSmoothing, &[RatioSpan], f64),
    rule: &RatioRampRule,
    limit: f64,
) -> Result<Option<f64>, Unlaid> {
    if rule.kernel_jerk_per_accel_hz.is_finite() {
        return Ok(Some(reach));
    }
    let trail = &smoothing.trail;
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
        if table_end.is_finite()
            && (window.1 > trail.through().min(table_end)
                || !smoothing.reads_within(Table::Advance, window.1, table_end))
        {
            waits = true;
            return true;
        }
        let window = (window.0, window.1.min(trail.through()));
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
        let ratio_over = |(from, to, _): Interval| {
            let (lo, hi) = (
                from.s.min(to.s).max(window.0),
                from.s.max(to.s).min(window.1),
            );
            if lo > hi {
                return 0.0;
            }
            smoothing.peak_ratio_in(Table::Advance, advance, lo, hi)
        };
        let accel = motion.path_accel.max(crossing.rate);
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
            let room = (rule.max_accel_mm_s2 / ratio - own) / slope;
            let (from, to, _) = interval;
            let readings = [from, to].map(|probe| (distance_of(probe), probe.speed));
            let (near, far) = if readings[0].0 <= readings[1].0 {
                (readings[0], readings[1])
            } else {
                (readings[1], readings[0])
            };
            let interval_accel = from.path_accel.abs().max(to.path_accel.abs());
            *whole <= room || within.fits_between(near, far, (accel, interval_accel), room)
        };
        trail
            .intervals_within(window, side)
            .fold(true, |fits, interval| {
                let (ratio, own) = (ratio_over(interval), own_over(interval));
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
                fits && (ratio == 0.0 || slope == 0.0 || rebuild_fits(interval, ratio, own))
            })
    });
    if waits {
        return Err(Unlaid::Waits);
    }
    Ok(width.filter(|&width| width <= limit))
}

/// The ratio steps a crawl valley holds the weight at zero around: each
/// step's window is kept within `window` of it
/// ([`geometry::crawl_step_window`]), with every seam of the sloped run of
/// spans either side of it, which the smoothing may join into one cluster
/// ramped over the narrowest of their windows ([`SeamTable::joins`]), and
/// the valley's zero stretch spans the crawl and every owned run's
/// windows. Inside it the advance reads none of a ramp; only its flow
/// `r̃·ṡ` reaches the follower, whose acceleration `r̃·s̈ + r̃′·ṡ²` takes
/// `|r̃′| ≤ ¾·|Δ|/h` from a step over a window of half-width `h`, besides
/// the table's own slope. A narrower window the width field lays is one
/// the ramp rule budgets. The leaders' path acceleration across the
/// stretch is at most `A`, so `ṡ² ≤ v² + 2A·d` a distance `d` from the
/// crawl they cross at `v`.
pub(super) struct StepWindows {
    turn: f64,
    crossing: Crossing,
    pub(super) window: f64,
    guarded: bool,
    pub(super) owned: Vec<OwnedStep>,
}

/// A ratio step a crawl valley holds the weight at zero around: which
/// table it steps, where, by how much, and the seams of the sloped run of
/// spans either side of it, its own among them, whose windows the valley
/// caps with it.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct OwnedStep {
    pub(super) table: Table,
    pub(super) s: f64,
    step: f64,
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
            step: seam.step.abs(),
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

    /// Owns every ratio step of either table on the path `[lo, hi]` not
    /// owned yet, with its run, where `untaken` holds for every seam of the
    /// run: the projection has read none of their ramps yet. Whether there
    /// was one.
    pub(super) fn own_within(
        &mut self,
        (laid, advance): (&[RatioSpan], &[RatioSpan]),
        (lo, hi): (f64, f64),
        untaken: &dyn Fn(f64) -> bool,
    ) -> bool {
        if !self.guarded {
            return false;
        }
        let before = self.owned.len();
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
                let owned_already = self
                    .owned
                    .iter()
                    .any(|owned| owned.table == table && owned.s == seam.s);
                if seam.step == 0.0 || seam.s < lo || seam.s > hi || owned_already {
                    continue;
                }
                let owned = OwnedStep::of(table, spans, at, &seam);
                if owned.run.iter().all(|&s| untaken(s)) {
                    self.owned.push(owned);
                }
            }
        }
        self.owned.len() > before
    }

    /// Where the valley holds the weight at zero: the crawl, and every
    /// owned run's windows.
    pub(super) fn zero_stretch(&self) -> (f64, f64) {
        self.owned
            .iter()
            .fold((self.turn, self.turn), |(lo, hi), owned| {
                let (first, last) = owned.extent();
                (lo.min(first - self.window), hi.max(last + self.window))
            })
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
    /// follower's limit: `r·A` at the largest ratio the smoothed laid
    /// table, laid as `laid`, reads there ([`RatioSmoothing::peak_ratio_in`]),
    /// and `¾·|Δ|·ṡ²/window` for each owned step of the laid table, `ṡ²`
    /// read as far from the crawl as its run's windows reach. The table's
    /// own slopes, which a window of any width averages to no steeper, are
    /// the planner's to hold.
    pub(super) fn assert_flow_fits(
        &self,
        smoothing: &RatioSmoothing,
        laid: &[RatioSpan],
        rule: &RatioRampRule,
    ) {
        if self.owned.is_empty() {
            return;
        }
        let zero = self.zero_stretch();
        let path_accel = self.path_accel(&smoothing.trail);
        let reaching = |(first, last): (f64, f64)| {
            let farthest = (first - self.turn).abs().max((last - self.turn).abs());
            self.speed_squared_within(path_accel, farthest + self.window)
        };
        let ramps: f64 = self
            .owned
            .iter()
            .filter(|owned| owned.table == Table::Laid)
            .map(|owned| 0.75 * owned.step * reaching(owned.extent()) / self.window)
            .sum();
        let flow = smoothing.peak_ratio_in(Table::Laid, laid, zero.0, zero.1) * path_accel + ramps;
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
}
