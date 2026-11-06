use std::cell::OnceCell;
use std::collections::VecDeque;

use geometry::{
    ADVANCE_VALLEY_SPEED_MM_S, CrawlSide, RebuildsWithin, VALLEY_LEAD_BUDGET_S, turns_around,
    valley_reach, widest_crawl_side_within,
};

use super::screen::ShapedBox;
use super::shadow::{CellMotion, norm};

/// How far the fitted leaders the follower projection reads its valleys
/// off may stray from the shaped motion the guard bounds: in position by
/// the fit's tolerance, in acceleration by its acceleration tolerance, and
/// in speed by `2·√(pos·accel)`, the fastest an error held within `pos`
/// whose rate changes no faster than `accel` can change (Landau's
/// inequality on a half-line).
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct FitMiss {
    pub position: f64,
    pub speed: f64,
    pub accel: f64,
}

impl FitMiss {
    pub fn of(pos_mm: f64, accel_mm_s2: f64) -> Self {
        Self {
            position: pos_mm,
            speed: 2.0 * (pos_mm * accel_mm_s2).sqrt(),
            accel: accel_mm_s2,
        }
    }

    /// The shaped speed under which the fitted leaders may crawl.
    pub fn crawl_speed(&self) -> f64 {
        ADVANCE_VALLEY_SPEED_MM_S + self.speed
    }
}

/// The shaped motion's speed and acceleration ranges across one grid
/// cell, how fast its heading turns at most, `|a × v|/|v|²`, and the range
/// of its path acceleration `s̈`: [`CellMotion::tangential_accel`] where
/// the cell is no slower than [`ADVANCE_VALLEY_SPEED_MM_S`], within the
/// acceleration's size elsewhere.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct CellBounds {
    pub speed: (f64, f64),
    pub accel: (f64, f64),
    pub turn: f64,
    pub tangential: (f64, f64),
}

impl CellBounds {
    pub fn of_motion(motion: &CellMotion) -> Self {
        let speed = (motion.speed_floor.max(0.0), motion.speed_ceiling);
        let strongest = motion.accel_ceiling;
        Self {
            speed,
            accel: (motion.accel_floor, strongest),
            turn: turn_rate(motion.cross_ceiling, strongest, speed.0),
            tangential: if speed.0 >= ADVANCE_VALLEY_SPEED_MM_S {
                motion.tangential_accel
            } else {
                (-strongest, strongest)
            },
        }
    }

    pub fn of_box(shaped: &ShapedBox) -> Self {
        let strongest = norm(shaped.strongest);
        Self {
            speed: (shaped.speed_floor, norm(shaped.fastest)),
            accel: (0.0, strongest),
            turn: turn_rate(shaped.cross_ceiling(), strongest, shaped.speed_floor),
            tangential: (-strongest, strongest),
        }
    }

    pub fn union(self, other: Self) -> Self {
        let hull = |a: (f64, f64), b: (f64, f64)| (a.0.min(b.0), a.1.max(b.1));
        Self {
            speed: hull(self.speed, other.speed),
            accel: hull(self.accel, other.accel),
            turn: self.turn.max(other.turn),
            tangential: hull(self.tangential, other.tangential),
        }
    }

    /// The bounds both `self` and `other` hold, each of the same motion
    /// across the cell.
    pub fn meet(self, other: Self) -> Self {
        let overlap = |what: &str, a: (f64, f64), b: (f64, f64)| {
            let (lo, hi) = (a.0.max(b.0), a.1.min(b.1));
            assert!(
                lo <= hi + 1e-9 * hi.abs().max(1.0),
                "turning guard: two bounds on a cell's {what} miss each other: {a:?} and {b:?}"
            );
            (lo.min(hi), hi)
        };
        Self {
            speed: overlap("speed", self.speed, other.speed),
            accel: overlap("acceleration", self.accel, other.accel),
            turn: self.turn.min(other.turn),
            tangential: overlap("path acceleration", self.tangential, other.tangential),
        }
    }

    /// The range of the fitted leaders' path acceleration across the cell:
    /// with their velocity within `δ` of the shaped `v` and their
    /// acceleration within `α` of it, their direction strays by at most
    /// `2δ/|v|`, so `s̈` by at most `α + 2δ·(|a| + α)/|v|`; where the shaped
    /// speed may fall to `2δ`, within the acceleration's size.
    fn fitted_tangential(&self, fit: FitMiss) -> (f64, f64) {
        let strongest = self.accel.1 + fit.accel;
        if self.speed.0 <= 2.0 * fit.speed {
            return (-strongest, strongest);
        }
        let stray = fit.accel + 2.0 * fit.speed * strongest / self.speed.0;
        (
            (self.tangential.0 - stray).max(-strongest),
            (self.tangential.1 + stray).min(strongest),
        )
    }
}

/// `|a × v|/|v|²` at most, with `|a × v|` within `cross`, `|a|` within
/// `accel` and `|v|` at least `speed`.
fn turn_rate(cross: f64, accel: f64, speed: f64) -> f64 {
    if speed > 0.0 {
        (cross / (speed * speed)).min(accel / speed)
    } else {
        f64::INFINITY
    }
}

/// The cells of the guard's grid scanned for crawls, in order, with sums
/// over them to each: the path the shaped motion covers at least and at
/// most, and the terms the fitted leaders' odometer strays by
/// ([`CellTrack::drift`]).
#[derive(Debug, Default)]
pub(super) struct CellTrack {
    base: i64,
    cells: VecDeque<TrackedCell>,
    before_base: Odometer,
}

#[derive(Debug, Clone, Copy)]
struct TrackedCell {
    bounds: CellBounds,
    fast: bool,
    through: Odometer,
}

/// Sums over tracked cells: the least and most path covered; the turning
/// `Σ |a × v|/v²·h` and the speed miss's share `Σ δ²/(2(v − δ))·h` of the
/// cells faster than twice the fit's speed miss `δ`, `Σ δ·h` of the
/// others; and how many runs of fast cells start.
#[derive(Debug, Clone, Copy, Default)]
struct Odometer {
    floor: f64,
    ceiling: f64,
    turning: f64,
    stray: f64,
    fast_runs: u32,
}

impl Odometer {
    fn since(self, before: Self) -> Self {
        Self {
            floor: (self.floor - before.floor).max(0.0),
            ceiling: (self.ceiling - before.ceiling).max(0.0),
            turning: (self.turning - before.turning).max(0.0),
            stray: (self.stray - before.stray).max(0.0),
            fast_runs: self.fast_runs - before.fast_runs,
        }
    }
}

impl CellTrack {
    /// One past the last cell tracked.
    pub fn end(&self) -> i64 {
        self.base + self.cells.len() as i64
    }

    pub fn base(&self) -> i64 {
        self.base
    }

    fn index(&self, n: i64) -> usize {
        usize::try_from(n - self.base)
            .ok()
            .filter(|&index| index < self.cells.len())
            .unwrap_or_else(|| {
                panic!(
                    "turning guard: cell {n} is not tracked, only [{}, {})",
                    self.base,
                    self.end()
                )
            })
    }

    pub fn get(&self, n: i64) -> CellBounds {
        self.cells[self.index(n)].bounds
    }

    pub fn push(&mut self, n: i64, bounds: CellBounds, h: f64, fit: FitMiss) {
        if self.cells.is_empty() {
            self.base = n;
        }
        assert_eq!(n, self.end(), "turning guard: cells are tracked in order");
        let previous = self.cells.back();
        let before = previous.map_or(self.before_base, |cell| cell.through);
        let speed = bounds.speed.0;
        let fast = speed > 2.0 * fit.speed;
        let through = Odometer {
            floor: before.floor + speed * h,
            ceiling: before.ceiling + bounds.speed.1 * h,
            turning: before.turning + if fast { bounds.turn * h } else { 0.0 },
            stray: before.stray
                + if fast {
                    fit.speed * fit.speed / (2.0 * (speed - fit.speed)) * h
                } else {
                    fit.speed * h
                },
            fast_runs: before.fast_runs
                + u32::from(fast && !previous.is_some_and(|cell| cell.fast)),
        };
        self.cells.push_back(TrackedCell {
            bounds,
            fast,
            through,
        });
    }

    pub fn truncate_from(&mut self, n: i64) {
        let keep = usize::try_from(n - self.base).unwrap_or(0);
        self.cells.truncate(keep);
    }

    pub fn trim_before(&mut self, n: i64) {
        while self.base < n {
            let Some(cell) = self.cells.pop_front() else {
                break;
            };
            self.before_base = cell.through;
            self.base += 1;
        }
    }

    pub fn clear(&mut self) {
        self.cells.clear();
        self.before_base = Odometer::default();
    }

    /// The sums over the cells `from..=to`, none where the range is empty.
    fn over(&self, from: i64, to: i64) -> Odometer {
        if to < from {
            return Odometer::default();
        }
        let before = if from == self.base {
            self.before_base
        } else {
            self.cells[self.index(from - 1)].through
        };
        self.cells[self.index(to)].through.since(before)
    }

    /// The least and most path the shaped motion covers across the cells
    /// `from..=to`, none where the range is empty.
    pub fn path(&self, from: i64, to: i64) -> (f64, f64) {
        let over = self.over(from, to);
        (over.floor, over.ceiling)
    }

    /// How far the fitted leaders' odometer may stray from the shaped
    /// path's over the cells `from..=to`. Where the shaped speed `v`
    /// outruns the speed miss `δ`, `|v + u| ≤ |v| + u·v̂ + |u|²/(2(|v| − δ))`
    /// and `≥ |v| + u·v̂`, and `∫u·v̂ = [e·v̂] − ∫e·dv̂/dt` with the position
    /// miss `|e| ≤ p`: within `p·(2 + Θ)` over a run of such cells whose
    /// heading turns `Θ = ∫|dv̂/dt| = ∫|a × v|/v²`; elsewhere within `δ` a
    /// second. The time times `δ` bounds it too.
    pub fn drift(&self, from: i64, to: i64, h: f64, fit: FitMiss) -> f64 {
        if to < from {
            return 0.0;
        }
        let over = self.over(from, to);
        let first_continues_a_run = from > self.base
            && self.cells[self.index(from)].fast
            && self.cells[self.index(from - 1)].fast;
        let runs = over.fast_runs + u32::from(first_continues_a_run);
        let linear = fit.speed * (to + 1 - from) as f64 * h;
        linear.min(fit.position * (2.0 * f64::from(runs) + over.turning) + over.stray)
    }
}

/// Where along a run of crawling cells the follower projection may lay
/// its advance valley: its zero stretch spans the crawls it finds there,
/// at seams and at the legs' slowest points, all within the cells
/// `cells`; the projection reads their speed and rate off the fitted
/// leaders, which set the leaders' reach, the least each side spans
/// ([`geometry::valley_reach`]), and the floor ([`geometry::crawl_floor`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct Slot {
    pub cells: (i64, i64),
    pub width: (f64, f64),
    pub floor: (f64, f64),
    pub speed: f64,
}

impl Slot {
    /// The slot over the cells `from..=to`, `None` where the fitted
    /// leaders cannot crawl in any of them.
    pub fn over(track: &CellTrack, (from, to): (i64, i64), fit: FitMiss) -> Option<Self> {
        let mut slot: Option<Self> = None;
        for n in from..=to {
            let cell = track.get(n);
            let speed = (
                (cell.speed.0 - fit.speed).max(0.0),
                (cell.speed.1 + fit.speed).min(ADVANCE_VALLEY_SPEED_MM_S),
            );
            if speed.0 >= ADVANCE_VALLEY_SPEED_MM_S {
                continue;
            }
            let rate = (
                (cell.accel.0 - fit.accel).max(0.0),
                cell.accel.1 + fit.accel,
            );
            let width = (valley_reach(speed.0, rate.0), valley_reach(speed.1, rate.1));
            let floor = (
                if speed.0 == 0.0 || turns_around(speed.0, rate.1) {
                    0.0
                } else {
                    speed.0 * speed.0 / rate.1
                },
                if rate.0 == 0.0 {
                    f64::INFINITY
                } else {
                    speed.1 * speed.1 / rate.0
                },
            );
            let here = Self {
                cells: (n, n),
                width,
                floor,
                speed: speed.1,
            };
            slot = Some(slot.map_or(here, |slot| Self {
                cells: (slot.cells.0, n),
                width: (slot.width.0.min(width.0), slot.width.1.max(width.1)),
                floor: (slot.floor.0.min(floor.0), slot.floor.1.max(floor.1)),
                speed: slot.speed.max(here.speed),
            }));
        }
        slot
    }

    /// The slot beside cells where the leaders rest: the projection lays
    /// the side of the rest valley there from an edge inside it, which
    /// keeps no floor.
    pub fn beside_rest(self) -> Self {
        Self {
            floor: (0.0, self.floor.1),
            ..self
        }
    }

    /// The path range from a point in cell `n` to the slot's centre, the
    /// fitted leaders' odometer straying from the shaped path's by at most
    /// [`CellTrack::drift`] over the cells between. The odometer never
    /// runs backwards, so the least reads only the cells between `n` and
    /// the run: the crawl lies inside it.
    fn distance(&self, track: &CellTrack, n: i64, h: f64, fit: FitMiss) -> (f64, f64) {
        self.distance_along(
            n,
            |from, to| track.path(from, to),
            |from, to| track.drift(from, to, h, fit),
        )
    }

    /// [`Self::distance`] with the least and most path the shaped motion
    /// covers across cells read off `path`, and the odometer's stray off
    /// `drift`.
    fn distance_along(
        &self,
        n: i64,
        path: impl Fn(i64, i64) -> (f64, f64),
        drift: impl Fn(i64, i64) -> f64,
    ) -> (f64, f64) {
        let (m0, m1) = self.cells;
        let most_drift = drift(n.min(m0), n.max(m1));
        let (least, most) = if n < m0 {
            (path(n + 1, m0 - 1).0 - drift(n + 1, m0 - 1), path(n, m1).1)
        } else if n > m1 {
            (path(m1 + 1, n - 1).0 - drift(m1 + 1, n - 1), path(m0, n).1)
        } else {
            (0.0, path(n.min(m0), n.max(m1)).1)
        };
        (least.max(0.0), most + most_drift)
    }

    /// The fitted leaders over the cells `cells` of one side of the slot's
    /// valley, as its rebuild reads them ([`CrawlSide`]), off the cells'
    /// `bounds` in order: within the run, where the crawl may lie anywhere,
    /// the path acceleration away from it either way; past the run's
    /// furthest reach from the crawl, its sign read off the bounds of the
    /// cells beyond the run ([`CellBounds::fitted_tangential`]).
    fn leaders(
        &self,
        track: &CellTrack,
        (cells, bounds): ((i64, i64), &[CellBounds]),
        (h, fit): (f64, FitMiss),
    ) -> CrawlSide {
        let (fastest, strongest) = bounds.iter().fold((0.0_f64, 0.0_f64), |(v, a), cell| {
            (v.max(cell.speed.1), a.max(cell.accel.1))
        });
        let accel = strongest + fit.accel;
        let either_way = CrawlSide::either_way(
            self.speed,
            accel,
            (fastest + fit.speed).max(self.speed),
            self.floor,
        );
        let (m0, m1) = self.cells;
        let (beyond, sign) = if cells.0 < m0 {
            ((cells.0, m0 - 1), -1.0)
        } else if cells.1 > m1 {
            ((m1 + 1, cells.1), 1.0)
        } else {
            return either_way;
        };
        let index = |n: i64| usize::try_from(n - cells.0).expect("a cell of the side");
        let outward = bounds[index(beyond.0)..=index(beyond.1)]
            .iter()
            .map(|cell| {
                let (lo, hi) = cell.fitted_tangential(fit);
                if sign > 0.0 { (lo, hi) } else { (-hi, -lo) }
            })
            .fold((accel, -accel), |(lo, hi), (cell_lo, cell_hi)| {
                (lo.min(cell_lo), hi.max(cell_hi))
            });
        CrawlSide {
            outward: (
                outward.0.clamp(-accel, accel),
                outward.1.clamp(-accel, accel),
            ),
            signed_from: track.path(m0, m1).1 + track.drift(m0, m1, h, fit),
            ..either_way
        }
    }
}

/// Which way from its crawl, or rest, a valley side reaches along the
/// path: its rebuild's `k·r·W′·ṡ²` takes from the advanced flow before
/// it and adds to it after.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Toward {
    Before,
    After,
}

/// One side of a crawl valley as the guard leaves room for it: the widest
/// the follower projection lays it ([`geometry::crawl_side_width`] picks a
/// grid width no wider wherever this one fits), which way it reaches, the
/// cells it may reach from wherever in the run the crawl lies, and how its
/// fitted leaders cross each of them. The most `|W″·ṡ³ + 3W′·ṡ·s̈|` and
/// `W′·ṡ²` its rebuild asks in a cell are read off them once a check
/// needs them, which an advance of slope `k` at ratio `r` turns into
/// `k·r` times as much follower acceleration and velocity.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct Reserve {
    pub width: f64,
    pub toward: Toward,
    pub cells: (i64, i64),
    leaders: Option<SideLeaders>,
}

/// The fitted leaders across a crawl valley side: as its rebuild reads
/// them, cell by cell, joined over every [`REBUILD_SCREEN_CELLS`] of them
/// and over all its cells, each of which bounds what the rebuild asks in
/// any cell it joins ([`RebuildsWithin`] grows as the distance, speed and
/// path acceleration ranges widen); and the bounds over the whole side
/// ([`CrawlSide::rebuild`], [`CrawlSide::lead_velocity`]) once read.
#[derive(Debug, Clone, PartialEq)]
struct SideLeaders {
    side: CrawlSide,
    within: RebuildsWithin,
    cells: Vec<CellLeaders>,
    blocks: Vec<CellBlock>,
    joined: [f64; 2],
    whole: [OnceCell<f64>; 2],
}

/// What a side's rebuild asks of the leaders: `|W″·ṡ³ + 3W′·ṡ·s̈|`, or
/// `W′·ṡ²`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Ask {
    Rebuild,
    Lead,
}

impl Ask {
    const BOTH: [Self; 2] = [Self::Rebuild, Self::Lead];

    fn index(self) -> usize {
        match self {
            Self::Rebuild => 0,
            Self::Lead => 1,
        }
    }
}

/// How many neighbouring cells of a side one [`CellBlock`] joins.
const REBUILD_SCREEN_CELLS: usize = 32;

/// The fitted leaders over neighbouring cells of a crawl valley side,
/// joined, and what [`RebuildsWithin`] bounds each of the rebuild's asks by
/// in any of them once read.
#[derive(Debug, Clone, PartialEq)]
struct CellBlock {
    leaders: Leaders,
    within: [OnceCell<f64>; 2],
}

/// The ranges of the fitted leaders' distance from the crawl, of their
/// speed and of their path acceleration away from it, `o`, over some cells
/// of a crawl valley side.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Leaders {
    distance: (f64, f64),
    speed: (f64, f64),
    outward: (f64, f64),
}

impl Leaders {
    fn joined(cells: &[CellLeaders]) -> Self {
        let hull = |a: (f64, f64), b: (f64, f64)| (a.0.min(b.0), a.1.max(b.1));
        cells
            .iter()
            .map(|cell| cell.leaders)
            .reduce(|joined, cell| Self {
                distance: hull(joined.distance, cell.distance),
                speed: hull(joined.speed, cell.speed),
                outward: hull(joined.outward, cell.outward),
            })
            .expect("turning guard: a crawl side joins at least one cell")
    }

    fn ask(&self, within: &RebuildsWithin, ask: Ask) -> f64 {
        match ask {
            Ask::Rebuild => within.at(self.distance, self.speed, self.outward),
            Ask::Lead => within.lead_velocity_at(self.distance, self.speed.1),
        }
    }
}

/// The fitted leaders in one cell of a crawl valley side, and what
/// [`RebuildsWithin`] bounds each of the rebuild's asks by there once
/// read.
#[derive(Debug, Clone, PartialEq)]
struct CellLeaders {
    leaders: Leaders,
    within: [OnceCell<f64>; 2],
}

impl Reserve {
    /// A side that leaves no room over the cells `cells`: the projection
    /// lays it where the weight held at zero, or a side reading no motion
    /// past it, needs none.
    pub fn none(cells: (i64, i64), toward: Toward) -> Self {
        assert!(
            cells.0 <= cells.1,
            "turning guard: a crawl side spans at least one cell, not {cells:?}"
        );
        Self {
            width: 0.0,
            toward,
            cells,
            leaders: None,
        }
    }

    /// The side `width` wide reaching `toward` over the cells `cells` of
    /// the `slot`'s valley: in each, the least of the bounds over the whole
    /// side ([`CrawlSide::rebuild`], [`CrawlSide::lead_velocity`]) and the
    /// ones the cell's own fitted leaders set at their distance from the
    /// crawl ([`RebuildsWithin`]). Past the run, a cell's shaped motion is
    /// bounded by both the track and its `shadow` convolution, which a box
    /// over a whole move leaves far looser: that sets the path to the run,
    /// so the distance, the speed and path acceleration there, and its sign
    /// away from the crawl, in each cell and over the whole side. A box
    /// keeps the fastest and strongest motion anywhere along its move, which
    /// slowing the move's end into the crawl leaves as it is while it
    /// narrows the side. Within the run the crawl may lie either way.
    fn over(
        slot: &Slot,
        track: &CellTrack,
        (cells, width, toward): ((i64, i64), f64, Toward),
        (h, fit): (f64, FitMiss),
        shadow: &dyn Fn(i64) -> CellBounds,
    ) -> Self {
        let (m0, m1) = slot.cells;
        let beyond_the_run = |n: i64| n < m0 || n > m1;
        let bounds: Vec<CellBounds> = (cells.0..=cells.1)
            .map(|n| {
                let tracked = track.get(n);
                if beyond_the_run(n) {
                    tracked.meet(shadow(n))
                } else {
                    tracked
                }
            })
            .collect();
        let side = slot.leaders(track, (cells, &bounds), (h, fit));
        let covered: Vec<(f64, f64)> = std::iter::once((0.0, 0.0))
            .chain(bounds.iter().scan((0.0, 0.0), |(least, most), cell| {
                *least += cell.speed.0 * h;
                *most += cell.speed.1 * h;
                Some((*least, *most))
            }))
            .collect();
        let index = |n: i64| usize::try_from(n - cells.0).expect("a cell of the side");
        let path = |from: i64, to: i64| {
            if to < from {
                return (0.0, 0.0);
            }
            let (before, through) = (covered[index(from)], covered[index(to) + 1]);
            (through.0 - before.0, through.1 - before.1)
        };
        let crossing: Vec<CellLeaders> = (cells.0..=cells.1)
            .zip(&bounds)
            .map(|(n, cell)| {
                let strongest = cell.accel.1 + fit.accel;
                let outward = if !beyond_the_run(n) {
                    (-strongest, strongest)
                } else if n < m0 {
                    let (lo, hi) = cell.fitted_tangential(fit);
                    (-hi, -lo)
                } else {
                    cell.fitted_tangential(fit)
                };
                CellLeaders {
                    leaders: Leaders {
                        distance: slot
                            .distance_along(n, path, |from, to| track.drift(from, to, h, fit)),
                        speed: (
                            (cell.speed.0 - fit.speed).max(0.0),
                            cell.speed.1 + fit.speed,
                        ),
                        outward,
                    },
                    within: Default::default(),
                }
            })
            .collect();
        let within = side.rebuilds_within(width);
        let blocks = crossing
            .chunks(REBUILD_SCREEN_CELLS)
            .map(|cells| CellBlock {
                leaders: Leaders::joined(cells),
                within: Default::default(),
            })
            .collect();
        let joined = Leaders::joined(&crossing);
        Self {
            width,
            toward,
            cells,
            leaders: Some(SideLeaders {
                side,
                joined: Ask::BOTH.map(|ask| joined.ask(&within, ask)),
                within,
                cells: crossing,
                blocks,
                whole: Default::default(),
            }),
        }
    }

    /// The most the rebuild asks in cell `n`, one of the side's, where
    /// `binds` holds for a bound on it; `None` where a bound it does not
    /// hold for shows the rebuild cannot bind. `binds` holds for every
    /// bound past one it holds for.
    pub fn rebuild_where(&self, n: i64, binds: impl Fn(f64) -> bool) -> Option<f64> {
        self.ask_where(n, Ask::Rebuild, binds)
    }

    /// The most `W′·ṡ²` the rebuild asks in cell `n`, one of the side's,
    /// where `binds` holds for a bound on it, as [`Self::rebuild_where`].
    pub fn lead_velocity_where(&self, n: i64, binds: impl Fn(f64) -> bool) -> Option<f64> {
        self.ask_where(n, Ask::Lead, binds)
    }

    /// The bound on what the rebuild asks of the leaders in cell `n` as
    /// `ask` reads it, over all the side's cells, over the block of them
    /// holding `n`, and over `n` alone, each read only where `binds` held
    /// for the last; at most the bound over the whole side.
    fn ask_where(&self, n: i64, ask: Ask, binds: impl Fn(f64) -> bool) -> Option<f64> {
        assert!(
            self.cells.0 <= n && n <= self.cells.1,
            "turning guard: cell {n} lies outside the crawl side over {:?}",
            self.cells
        );
        let Some(leaders) = &self.leaders else {
            return binds(0.0).then_some(0.0);
        };
        let at = ask.index();
        if !binds(leaders.joined[at]) {
            return None;
        }
        let index = usize::try_from(n - self.cells.0).expect("a cell of the side");
        let block = &leaders.blocks[index / REBUILD_SCREEN_CELLS];
        let screened = *block.within[at].get_or_init(|| block.leaders.ask(&leaders.within, ask));
        if !binds(screened) {
            return None;
        }
        let cell = &leaders.cells[index];
        let within = *cell.within[at].get_or_init(|| cell.leaders.ask(&leaders.within, ask));
        binds(within).then(|| {
            within.min(*leaders.whole[at].get_or_init(|| match ask {
                Ask::Rebuild => leaders.side.rebuild(self.width),
                Ask::Lead => leaders.side.lead_velocity(self.width),
            }))
        })
    }
}

/// A run of grid cells where the shaped speed may fall under the fitted
/// leaders' crawl ([`FitMiss::crawl_speed`]), the one valley the follower
/// projection lays across the crawls it finds along it, and the room its
/// sides leave for their rebuilds: the after side once the motion
/// [`VALLEY_LEAD_BUDGET_S`] past the run is tracked.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct Crawl {
    pub slot: Slot,
    /// The source line of the move the run starts in.
    pub line: u32,
    pub before: Reserve,
    pub after: Option<Reserve>,
    covered_after: CoveredAfter,
}

/// The least path the fitted leaders cover from the end of a run to each
/// cell tracked past it, read through cell `through`: the most of it over
/// those cells, and over those within [`VALLEY_LEAD_BUDGET_S`] of the run.
#[derive(Debug, Clone, Copy, PartialEq)]
struct CoveredAfter {
    through: i64,
    most: f64,
    within_budget: f64,
}

impl CoveredAfter {
    fn from_run_end(last: i64) -> Self {
        Self {
            through: last,
            most: 0.0,
            within_budget: 0.0,
        }
    }
}

/// Why a crawl's valley side does not fit within the
/// [`VALLEY_LEAD_BUDGET_S`] of motion beside it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct Overreach {
    /// The widest the leaders' reach makes the side.
    pub width: f64,
    /// The least path the leaders cover in the budget beside the crawl.
    pub covered: f64,
    /// The widest the reach would be with the leaders' rate at the crawl
    /// at the fit's tolerance alone.
    pub width_at_rest: f64,
}

impl Crawl {
    pub fn cells(&self) -> (i64, i64) {
        self.slot.cells
    }

    /// The cells its sides may reach: past the run without end while the
    /// after side waits.
    pub fn reach(&self) -> (i64, i64) {
        (
            self.before.cells.0,
            self.after.as_ref().map_or(i64::MAX, |after| after.cells.1),
        )
    }

    /// Whether its sides may reach a cell of `from..=to`.
    pub fn reaches(&self, (from, to): (i64, i64)) -> bool {
        let (first, last) = self.reach();
        first <= to && from <= last
    }

    /// The crawl over the slot, with room left for its before side: as
    /// wide as the widest grid width the leaders' least path in the
    /// [`VALLEY_LEAD_BUDGET_S`] before the run covers, and reaching every
    /// cell that may lie within it of the run. The projection reaches no
    /// further back than where the leaders last rested, nor than the
    /// segment's table, which starts at the start of cell
    /// `table_start + 1`; where either lies inside the budget or within the
    /// widest reach of the slot, the projection holds the weight at zero
    /// back to there wherever no side fits, and no room is left
    /// ([`Reserve::none`]). A run the leaders come to rest past
    /// (`into_rest`) holds the edge of the rest valley the projection lays
    /// there, whose before side reaches back past an earlier rest as far as
    /// its rebuild needs, and fails loudly where no side within the path
    /// the projection still holds fits: room is left for it whatever lies
    /// before, over the budget and no further back than the table.
    pub fn found(
        slot: Slot,
        line: u32,
        track: &CellTrack,
        (table_start, into_rest): (i64, bool),
        (h, fit): (f64, FitMiss),
        shadow: &dyn Fn(i64) -> CellBounds,
    ) -> Result<Self, Overreach> {
        let (first, last) = slot.cells;
        let budget_from = first - (VALLEY_LEAD_BUDGET_S / h).floor() as i64;
        let within_reach = |n: i64| {
            n >= budget_from
                || track.path(n + 1, first - 1).1 + track.drift(n + 1, last, h, fit) <= slot.width.1
        };
        let rested = (track.base()..first)
            .rev()
            .take_while(|&n| within_reach(n))
            .any(|n| track.get(n).speed.1 == 0.0);
        let table_within_reach = table_start >= budget_from
            || (table_start + 1 >= track.base() && within_reach(table_start));
        if !into_rest && (rested || table_within_reach) {
            return Ok(Self {
                slot,
                line,
                before: Reserve::none(slot.cells, Toward::Before),
                after: None,
                covered_after: CoveredAfter::from_run_end(last),
            });
        }
        let budget_from = if into_rest {
            budget_from.max(table_start + 1).min(first)
        } else {
            budget_from
        };
        assert!(
            budget_from >= track.base(),
            "turning guard: the crawl at cell {first} reads cells from {budget_from} the track \
             no longer holds (from {})",
            track.base()
        );
        let covered = (budget_from..first)
            .map(|n| track.path(n, first - 1).0 - track.drift(n, first - 1, h, fit))
            .fold(0.0_f64, f64::max);
        let Some(width) = widest_crawl_side_within(slot.width.1, covered) else {
            if table_within_reach {
                return Ok(Self {
                    slot,
                    line,
                    before: Reserve::none(slot.cells, Toward::Before),
                    after: None,
                    covered_after: CoveredAfter::from_run_end(last),
                });
            }
            return Err(Overreach {
                width: slot.width.1,
                covered,
                width_at_rest: valley_reach(slot.speed, fit.accel),
            });
        };
        let mut side_from = first;
        while side_from > budget_from && slot.distance(track, side_from - 1, h, fit).0 < width {
            side_from -= 1;
        }
        let cells = (side_from, last);
        Ok(Self {
            slot,
            line,
            before: Reserve::over(
                &slot,
                track,
                (cells, width, Toward::Before),
                (h, fit),
                shadow,
            ),
            after: None,
            covered_after: CoveredAfter::from_run_end(last),
        })
    }

    /// Waits for the after side again, the cells past the run to be
    /// tracked anew.
    pub fn reopen_after(&mut self) {
        self.after = None;
        self.covered_after = CoveredAfter::from_run_end(self.slot.cells.1);
    }

    /// Forgets what it read of the cells from `n` on, which the track
    /// holds anew.
    pub fn forget_track_from(&mut self, n: i64) {
        if self.covered_after.through >= n {
            self.covered_after = CoveredAfter::from_run_end(self.slot.cells.1);
        }
    }

    /// Reads the least path the leaders cover past the run on through cell
    /// `to`, which the track holds.
    fn cover_after_through(&mut self, track: &CellTrack, to: i64, (h, fit): (f64, FitMiss)) {
        let last = self.slot.cells.1;
        let horizon = last + (VALLEY_LEAD_BUDGET_S / h).floor() as i64;
        assert!(
            self.covered_after.through <= to,
            "turning guard: the crawl at cell {} read the track through cell {} it holds only \
             through {to}",
            self.slot.cells.0,
            self.covered_after.through
        );
        let covered = &mut self.covered_after;
        while covered.through < to {
            covered.through += 1;
            let n = covered.through;
            covered.most = covered
                .most
                .max(track.path(last + 1, n).0 - track.drift(last + 1, n, h, fit));
            if n <= horizon {
                covered.within_budget = covered.most;
            }
        }
    }

    /// Leaves room for the after side: as wide as the widest grid width
    /// the leaders' least path in the [`VALLEY_LEAD_BUDGET_S`] after the
    /// run covers once those cells are tracked; where they cover too
    /// little, the first grid width past the leaders' reach once the path
    /// after the run covers it. Where the segment ends first (`all` its
    /// cells tracked) no room is left ([`Reserve::none`]): past its end the
    /// leaders rest, and the projection, reading no motion there, widens
    /// the side until it fits. `false` while it waits.
    pub fn resolve_after(
        &mut self,
        track: &CellTrack,
        all: bool,
        (h, fit): (f64, FitMiss),
        shadow: &dyn Fn(i64) -> CellBounds,
    ) -> bool {
        let slot = self.slot;
        let (first, last) = slot.cells;
        let horizon = last + (VALLEY_LEAD_BUDGET_S / h).floor() as i64;
        let to = track.end() - 1;
        if to < horizon && !all {
            return false;
        }
        self.cover_after_through(track, to, (h, fit));
        let covered = self.covered_after;
        let within_budget = widest_crawl_side_within(slot.width.1, covered.within_budget);
        let width = match within_budget {
            Some(width) => width,
            None => match widest_crawl_side_within(slot.width.1, covered.most) {
                Some(width) => width,
                None if all => {
                    self.after = Some(Reserve::none(slot.cells, Toward::After));
                    return true;
                }
                None => return false,
            },
        };
        let mut side_to = last;
        while side_to < to && slot.distance(track, side_to + 1, h, fit).0 < width {
            side_to += 1;
        }
        let cells = (first, side_to);
        self.after = Some(Reserve::over(
            &slot,
            track,
            (cells, width, Toward::After),
            (h, fit),
            shadow,
        ));
        true
    }
}

#[cfg(test)]
mod tests;
