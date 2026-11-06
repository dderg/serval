use std::collections::VecDeque;

use geometry::{
    ADVANCE_VALLEY_SPEED_MM_S, CrawlSide, RatioRampRule, VALLEY_LEAD_BUDGET_S, crawl_side_width,
};

use super::super::RatioSpan;
use super::{Crossing, LegPath};

/// Probes per leg the side trail reads the leaders at, both ends included
/// once more.
const TRAIL_PROBES_PER_LEG: usize = 16;

/// The leaders at one probe of the side trail: where on the path, how
/// fast, `|s̈|`, and `|v̂·j| + a⊥²/max(ṡ, ADVANCE_VALLEY_SPEED_MM_S)`, the
/// path speed's second rate an advance reads (its turning part through a
/// crawl read at the speed a valley leaves it reading).
#[derive(Debug, Clone, Copy, PartialEq)]
struct TrailProbe {
    t: f64,
    s: f64,
    speed: f64,
    path_accel: f64,
    path_jerk: f64,
}

/// The most the leaders ask along a stretch of the side trail.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct SideMotion {
    pub(super) fastest: f64,
    pub(super) path_accel: f64,
    pub(super) path_jerk: f64,
}

/// The leaders along the path, probed leg by leg, as far back as a crawl
/// valley's side may still reach: what its rebuild leaves of the
/// follower's limit is read off it ([`crawl_side`]).
#[derive(Debug, Default)]
pub(super) struct SideTrail {
    probes: VecDeque<TrailProbe>,
}

impl SideTrail {
    /// Probes the leaders of the leg `path` follows over `support`, the
    /// path read no further than `table_end`.
    pub(super) fn record(&mut self, path: &LegPath, (t_start, t_end): (f64, f64), table_end: f64) {
        for index in 0..=TRAIL_PROBES_PER_LEG {
            let t = if index == TRAIL_PROBES_PER_LEG {
                t_end
            } else {
                t_start + (t_end - t_start) * index as f64 / TRAIL_PROBES_PER_LEG as f64
            };
            let s = (path.start_s + path.covered(t)).min(table_end);
            let (states, leaders) = path.leaders.pvaj_states(t);
            let (speed, path_accel, path_jerk) = path_derivatives(&states[..leaders]);
            self.probes.push_back(TrailProbe {
                t,
                s,
                speed,
                path_accel,
                path_jerk,
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
    /// `[lo, hi]`.
    fn over(&self, (lo, hi): (f64, f64)) -> SideMotion {
        let first = self
            .probes
            .partition_point(|probe| probe.s < lo)
            .saturating_sub(1);
        let last = self.probes.partition_point(|probe| probe.s <= hi);
        self.probes
            .range(first..(last + 1).min(self.probes.len()))
            .fold(
                SideMotion {
                    fastest: 0.0,
                    path_accel: 0.0,
                    path_jerk: 0.0,
                },
                |most, probe| SideMotion {
                    fastest: most.fastest.max(probe.speed),
                    path_accel: most.path_accel.max(probe.path_accel),
                    path_jerk: most.path_jerk.max(probe.path_jerk),
                },
            )
    }
}

/// The leaders' path speed `ṡ`, `|s̈|` and `|v̂·j| + a⊥²/max(ṡ, 2 mm/s)`
/// from their states, `a⊥²` summed through Lagrange's identity as
/// [`super::super::speed_second_of`] does. At rest the whole acceleration
/// leaves along the path.
fn path_derivatives(states: &[(f64, f64, f64, f64)]) -> (f64, f64, f64) {
    let norm = |pick: fn(&(f64, f64, f64, f64)) -> f64| {
        states
            .iter()
            .map(|state| pick(state) * pick(state))
            .sum::<f64>()
            .sqrt()
    };
    let speed = norm(|state| state.1);
    if speed == 0.0 {
        return (0.0, norm(|state| state.2), norm(|state| state.3));
    }
    let along = |pick: fn(&(f64, f64, f64, f64)) -> f64| {
        states
            .iter()
            .map(|state| state.1 / speed * pick(state))
            .sum::<f64>()
    };
    let mut normal_squared = 0.0;
    for (index, &(_, v_i, a_i, _)) in states.iter().enumerate() {
        for &(_, v_j, a_j, _) in &states[index + 1..] {
            let normal = v_i / speed * a_j - v_j / speed * a_i;
            normal_squared += normal * normal;
        }
    }
    (
        speed,
        along(|state| state.2).abs(),
        along(|state| state.3).abs() + normal_squared / speed.max(ADVANCE_VALLEY_SPEED_MM_S),
    )
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
/// as `crossing`: the narrowest [`crawl_side_width`] lays where the rebuild
/// `k·r·|W″·ṡ³ + 3W′·ṡ·s̈|` over it ([`CrawlSide::rebuild`]) fits what the
/// leaders' own demand there leaves of the follower's limit,
/// `A_e − r·(|s̈| + k·|s⃛|)`, `r` the largest ratio laid within it and the
/// leaders read off the trail, never under the leaders' `reach`. A side
/// reaches no further back than `limit`. It fails loudly where the leaders
/// leave the follower no room, and where no width within the limits fits:
/// it never narrows a side to make it fit.
pub(super) fn crawl_side(
    side: Side,
    (turn, crossing): (f64, Crossing),
    reach: f64,
    (trail, spans, table_end): (&SideTrail, &[RatioSpan], f64),
    rule: &RatioRampRule,
    limit: f64,
) -> Result<f64, Unlaid> {
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
        if window.1 > trail.through().min(table_end) {
            waits = true;
            return true;
        }
        let ratio = peak_ratio_over(spans, window);
        let motion = trail.over(window);
        let room = rule.max_accel_mm_s2 - ratio * (motion.path_accel + slope * motion.path_jerk);
        assert!(
            room > 0.0,
            "follower projection: the leaders ask {} mm/s² of the follower within {width} mm \
             {side:?} the crawl at s={turn} ({crossing:?}, {motion:?}, ratio {ratio}), past its \
             limit of {} mm/s² before its valley rebuilds the lead",
            rule.max_accel_mm_s2 - room,
            rule.max_accel_mm_s2
        );
        let leaders = CrawlSide {
            speed: crossing.speed,
            accel: motion.path_accel.max(crossing.rate),
            fastest: motion.fastest.max(crossing.speed),
            floor: (floor, floor),
        };
        slope * ratio * leaders.rebuild(width) <= room
    });
    if waits {
        return Err(Unlaid::Waits);
    }
    let width = width.unwrap_or_else(|| {
        panic!(
            "follower projection: no width of the {side:?} side of the crawl valley at s={turn} \
             ({crossing:?}) fits its rebuild within the follower's limit"
        )
    });
    assert!(
        width <= limit,
        "follower projection: the {side:?} side of the crawl valley at s={turn} ({crossing:?}) \
         needs {width} mm to fit its rebuild, past the {limit} mm it may reach"
    );
    Ok(width)
}

/// The largest extrusion ratio `spans` lay over the path `[lo, hi]`.
fn peak_ratio_over(spans: &[RatioSpan], (lo, hi): (f64, f64)) -> f64 {
    spans
        .iter()
        .filter(|span| span.s1 >= lo && span.s0 <= hi)
        .map(|span| span.r0.abs().max(span.r1.abs()))
        .fold(0.0, f64::max)
}
