use std::collections::VecDeque;

use geometry::{
    ADVANCE_VALLEY_SPEED_MM_S, CrawlSide, VALLEY_LEAD_BUDGET_S, turns_around, valley_reach,
    widest_crawl_side_within,
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

    /// The largest acceleration across the cells `from..=to`.
    fn strongest(&self, from: i64, to: i64) -> f64 {
        (from..=to).fold(0.0, |most: f64, n| most.max(self.get(n).accel.1))
    }

    /// The largest speed across the cells `from..=to`.
    fn fastest(&self, from: i64, to: i64) -> f64 {
        (from..=to).fold(0.0, |most: f64, n| most.max(self.get(n).speed.1))
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

    /// The path range from a point in cell `n` to the slot's centre, the
    /// fitted leaders' odometer straying from the shaped path's by at most
    /// [`CellTrack::drift`] over the cells between. The odometer never
    /// runs backwards, so the least reads only the cells between `n` and
    /// the run: the crawl lies inside it.
    fn distance(&self, track: &CellTrack, n: i64, h: f64, fit: FitMiss) -> (f64, f64) {
        let (m0, m1) = self.cells;
        let most_drift = track.drift(n.min(m0), n.max(m1), h, fit);
        let (least, most) = if n < m0 {
            let between = track.path(n + 1, m0 - 1).0 - track.drift(n + 1, m0 - 1, h, fit);
            (between, track.path(n, m1).1)
        } else if n > m1 {
            let between = track.path(m1 + 1, n - 1).0 - track.drift(m1 + 1, n - 1, h, fit);
            (between, track.path(m0, n).1)
        } else {
            (0.0, track.path(n.min(m0), n.max(m1)).1)
        };
        (least.max(0.0), most + most_drift)
    }

    /// The fitted leaders over the cells `cells` of one side of the slot's
    /// valley, as its rebuild reads them ([`CrawlSide`]): within the run,
    /// where the crawl may lie anywhere, the path acceleration away from it
    /// either way; past the run's furthest reach from the crawl, its sign
    /// read off the `shadow` convolution's bounds of the cells beyond the
    /// run ([`CellBounds::fitted_tangential`]), the track's box bounding
    /// only its size.
    fn leaders(
        &self,
        track: &CellTrack,
        cells: (i64, i64),
        (h, fit): (f64, FitMiss),
        shadow: &dyn Fn(i64) -> CellBounds,
    ) -> CrawlSide {
        let accel = track.strongest(cells.0, cells.1) + fit.accel;
        let either_way = CrawlSide::either_way(
            self.speed,
            accel,
            (track.fastest(cells.0, cells.1) + fit.speed).max(self.speed),
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
        let outward = (beyond.0..=beyond.1)
            .map(|n| {
                let (lo, hi) = shadow(n).fitted_tangential(fit);
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

/// One side of a crawl valley as the guard leaves room for it: the widest
/// the follower projection lays it ([`geometry::crawl_side_width`] picks a
/// grid width no wider wherever this one fits), the cells it may reach
/// from wherever in the run the crawl lies, and for each of them the most
/// `|W″·ṡ³ + 3W′·ṡ·s̈|` its rebuild asks there, which an advance of slope
/// `k` at ratio `r` turns into `k·r` times as much follower acceleration.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct Reserve {
    pub width: f64,
    pub cells: (i64, i64),
    rebuilds: Vec<f64>,
}

impl Reserve {
    /// A side that leaves no room over the cells `cells`: the projection
    /// lays it where the weight held at zero, or a side reading no motion
    /// past it, needs none.
    pub fn none(cells: (i64, i64)) -> Self {
        Self {
            width: 0.0,
            cells,
            rebuilds: vec![0.0; cell_count(cells)],
        }
    }

    /// The side `width` wide over the cells `cells` of the `slot`'s
    /// valley: in each, the least of the bound over the whole side
    /// ([`CrawlSide::rebuild`]) and the one the cell's own fitted leaders
    /// set at their distance from the crawl
    /// ([`CrawlSide::rebuild_within`]).
    fn over(
        slot: &Slot,
        track: &CellTrack,
        (cells, width): ((i64, i64), f64),
        (h, fit): (f64, FitMiss),
        shadow: &dyn Fn(i64) -> CellBounds,
    ) -> Self {
        let side = slot.leaders(track, cells, (h, fit), shadow);
        let whole_side = side.rebuild(width);
        let rebuilds = (cells.0..=cells.1)
            .map(|n| {
                let cell = track.get(n);
                let distance = slot.distance(track, n, h, fit);
                side.rebuild_within(
                    width,
                    distance,
                    cell.speed.1 + fit.speed,
                    cell.accel.1 + fit.accel,
                )
                .min(whole_side)
            })
            .collect();
        Self {
            width,
            cells,
            rebuilds,
        }
    }

    /// The most the rebuild asks in cell `n`, one of the side's.
    pub fn rebuild_at(&self, n: i64) -> f64 {
        assert!(
            self.cells.0 <= n && n <= self.cells.1,
            "turning guard: cell {n} lies outside the crawl side over {:?}",
            self.cells
        );
        self.rebuilds[usize::try_from(n - self.cells.0).expect("a cell of the side")]
    }
}

fn cell_count((from, to): (i64, i64)) -> usize {
    usize::try_from(to - from + 1).expect("a crawl side spans at least one cell")
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
    /// ([`Reserve::none`]).
    pub fn found(
        slot: Slot,
        line: u32,
        track: &CellTrack,
        table_start: i64,
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
        if rested || table_within_reach {
            return Ok(Self {
                slot,
                line,
                before: Reserve::none(slot.cells),
                after: None,
            });
        }
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
            before: Reserve::over(&slot, track, (cells, width), (h, fit), shadow),
            after: None,
        })
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
        let covered_to = |end: i64| {
            (last + 1..=end)
                .map(|n| track.path(last + 1, n).0 - track.drift(last + 1, n, h, fit))
                .fold(0.0_f64, f64::max)
        };
        if to < horizon && !all {
            return false;
        }
        let within_budget = widest_crawl_side_within(slot.width.1, covered_to(horizon.min(to)));
        let width = match within_budget {
            Some(width) => width,
            None => match widest_crawl_side_within(slot.width.1, covered_to(to)) {
                Some(width) => width,
                None if all => {
                    self.after = Some(Reserve::none(slot.cells));
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
            (cells, width),
            (h, fit),
            shadow,
        ));
        true
    }
}

#[cfg(test)]
mod tests;
