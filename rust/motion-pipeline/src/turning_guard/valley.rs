use std::collections::VecDeque;

use geometry::{ADVANCE_VALLEY_SPEED_MM_S, SideWeight, turns_around, valley_reach};

use super::screen::ShapedBox;
use super::shadow::{CellMotion, ValleyRead, norm};

/// The longest a crawl valley's side may take the leaders before its
/// crawl: the share of the pump's lead (`MAX_LEAD_SECS`, 2 s, in
/// motion-core's pump loop) the guard spends holding a move back for the
/// valleys still to come within its reach. A move is proven once the
/// motion this much past it has arrived; a crawl whose side would reach
/// further back is backed off until it does not ([`Crawl::before_side`]).
pub(super) const VALLEY_LEAD_BUDGET_S: f64 = 0.1;

/// The most slots of one cell's valleys whose every subset of laid ones
/// is weighed apart ([`weigh`]); past it each slot's weight is read as 1
/// wherever it multiplies another's slope.
const SUBSETS_WEIGHED_MAX_SLOTS: usize = 6;

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

/// The shaped motion's speed and acceleration ranges across one grid cell.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct CellBounds {
    pub speed: (f64, f64),
    pub accel: (f64, f64),
}

impl CellBounds {
    pub fn of_motion(motion: &CellMotion) -> Self {
        Self {
            speed: (motion.speed_floor.max(0.0), motion.speed_ceiling),
            accel: (motion.accel_floor, motion.accel_ceiling),
        }
    }

    pub fn of_box(shaped: &ShapedBox) -> Self {
        Self {
            speed: (shaped.speed_floor, norm(shaped.fastest)),
            accel: (0.0, norm(shaped.strongest)),
        }
    }

    pub fn union(self, other: Self) -> Self {
        Self {
            speed: (
                self.speed.0.min(other.speed.0),
                self.speed.1.max(other.speed.1),
            ),
            accel: (
                self.accel.0.min(other.accel.0),
                self.accel.1.max(other.accel.1),
            ),
        }
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
/// `Σ A/v·h` and the speed miss's share `Σ δ²/(2(v − δ))·h` of the cells
/// faster than twice the fit's speed miss `δ`, `Σ δ·h` of the others; and
/// how many runs of fast cells start.
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
            turning: before.turning
                + if fast {
                    bounds.accel.1 / speed * h
                } else {
                    0.0
                },
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
    /// miss `|e| ≤ p`: within `p·(2 + Θ)` over a run of such cells turning
    /// `Θ ≤ ∫A/v`; elsewhere within `δ` a second. The time times `δ`
    /// bounds it too.
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
}

/// Where along a run of crawling cells the follower projection may lay
/// its advance valley, and how wide: its zero stretch spans the crawls it
/// finds there, at seams and at the legs' slowest points, all within the
/// cells `cells`; the projection reads their speed and rate off the fitted
/// leaders, which set the sides' width and the floor by the shared rule
/// ([`geometry::valley_reach`], [`geometry::crawl_floor`]).
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

    /// Widens the slot to what the projection lays where it merges the
    /// valley into the `last` crawl's, their sides overlapping: the after
    /// side then as wide as the wider of the two, over either floor.
    pub fn widen_by_merge(&mut self, last: &Self, track: &CellTrack, h: f64, fit: FitMiss) {
        let (gap_from, gap_to) = (last.cells.1 + 1, self.cells.0 - 1);
        let drift = track.drift(last.cells.0, self.cells.1, h, fit);
        let apart = (track.path(gap_from, gap_to).0 - drift).max(0.0);
        if apart < last.width.1 + self.width.1 {
            self.width.1 = self.width.1.max(last.width.1);
            self.floor = (
                self.floor.0.min(last.floor.0),
                self.floor.1.max(last.floor.1),
            );
        }
    }

    /// The path range from a point in cell `n` to the slot's centre, the
    /// fitted leaders' odometer straying from the shaped path's by at most
    /// [`CellTrack::drift`] over the cells between.
    fn distance(&self, track: &CellTrack, n: i64, h: f64, fit: FitMiss) -> (f64, f64) {
        let (m0, m1) = self.cells;
        let (least, most) = if n < m0 {
            (track.path(n + 1, m0 - 1).0, track.path(n, m1).1)
        } else if n > m1 {
            (track.path(m1 + 1, n - 1).0, track.path(m0, n).1)
        } else {
            (0.0, track.path(n.min(m0), n.max(m1)).1)
        };
        let drift = track.drift(n.min(m0), n.max(m1), h, fit);
        ((least - drift).max(0.0), most + drift)
    }

    /// The slot's valley read from cell `n` whose fitted speed lies within
    /// `speed`, `None` where it cannot reach the cell. The leaders reach
    /// the cell's speed from the crawl's within the side: `σ̇² ≤ v² + 2A·d`
    /// with `A` the acceleration between and `d < w`, so a cell faster than
    /// any crawl builds lies outside every side, and one it does lie in
    /// reads a crawl at least `√(σ̇² − 2A·d)` fast, whose side is at least
    /// that crawl's reach ([`geometry::valley_reach`]) wide.
    fn read(
        &self,
        track: &CellTrack,
        n: i64,
        speed: (f64, f64),
        h: f64,
        fit: FitMiss,
    ) -> Option<Factor> {
        let distance = self.distance(track, n, h, fit);
        if distance.0 >= self.width.1 {
            return None;
        }
        let strongest = track.strongest(n.min(self.cells.0), n.max(self.cells.1)) + fit.accel;
        let crawl_sq = speed.0 * speed.0 - 2.0 * strongest * distance.1.min(self.width.1);
        if crawl_sq > self.speed * self.speed {
            return None;
        }
        let narrowest = self
            .width
            .0
            .max(distance.0)
            .max(valley_reach(crawl_sq.max(0.0).sqrt(), 0.0));
        let width = (narrowest.min(self.width.1), self.width.1);
        let side = SideWeight::within(distance, width, self.floor);
        let fastest = speed.1;
        let cubed = fastest * fastest * fastest;
        let leaving_floor = self.floor.1.min(width.1);
        let bend = if leaving_floor == 0.0 {
            0.0
        } else {
            side.bend_scale.1
                * (side.bend.1 * cubed).min(coupled_bend(
                    strongest,
                    leaving_floor,
                    self.speed,
                    width.0,
                    self.floor.1 > width.0,
                ))
        };
        Some(Factor {
            weight: side.weight,
            rebuild: side.slope.1 * fastest,
            fold: side.fold.1 * cubed,
            bend,
        })
    }
}

/// `σ̇³·β` at most: the floor's bend `β = f̃²/(d² + f̃²)^{3/2}` times the
/// cube of the fitted path speed at a distance `d` from the crawl, where
/// the path's acceleration stays within `strongest` (`A`) between. The
/// speed there is `σ̇² ≤ v² + 2A·d` with `v` the crawl's speed; with
/// `p = v²/(A·f̃)` and `x = d/f̃`, `σ̇³·β ≤ A^{3/2}·√f̃·((p + 2x)/(1 + x²))^{3/2}`,
/// and `(p + 2x)/(1 + x²)` peaks over `x` at `(p + √(p² + 4))/2`. Where the
/// weight bends over the floor `f = v²/a` itself (`f̃ = f`), `p = a/A ≤ 1`
/// and the bound is `φ^{3/2}·A^{3/2}·√f`, `φ` the golden ratio; where the
/// side's width `w` caps the floor, `p ≤ v²/(A·w)`. Neither a vanishing
/// floor nor a vanishing distance makes it diverge, which their ranges
/// alone do.
pub(super) fn coupled_bend(
    strongest: f64,
    leaving_floor: f64,
    crawl_speed: f64,
    narrowest: f64,
    width_caps_floor: bool,
) -> f64 {
    let share = if width_caps_floor {
        (crawl_speed * crawl_speed / (strongest * narrowest)).max(1.0)
    } else {
        1.0
    };
    let peak = 0.5 * (share + (share * share + 4.0).sqrt());
    strongest * strongest.sqrt() * leaving_floor.sqrt() * peak * peak.sqrt()
}

/// One slot's valley as a cell reads it: its weight's range, `|W′·σ̇|`,
/// and the fold and bend `W″·σ̇³` reaches below and above zero.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Factor {
    weight: (f64, f64),
    rebuild: f64,
    fold: f64,
    bend: f64,
}

/// The advance reads the product `Φ = Π W_j` of the valleys laid, any
/// subset of the slots' (one the projection finds the path too steep to
/// read through at): `Φ′ = Σ_j W_j′·Π_{i≠j} W_i` and
/// `Φ″ = Σ_j W_j″·Π_{i≠j} W_i + 2·Σ_{j<k} W_j′·W_k′·Π_{i≠j,k} W_i`. Each
/// subset is bounded with the other factors at their largest weight, and
/// the worst subset taken; past [`SUBSETS_WEIGHED_MAX_SLOTS`] every other
/// factor is read at 1.
fn weigh(factors: &[Factor], speed: f64) -> ValleyRead {
    let weight_floor = factors.iter().map(|factor| factor.weight.0).product();
    let mut read = ValleyRead {
        weight_floor,
        ..ValleyRead::NONE
    };
    if factors.len() > SUBSETS_WEIGHED_MAX_SLOTS {
        let read_at_one = vec![1.0; factors.len()];
        widen_by_subset(&mut read, factors, &read_at_one, |_| true, speed);
        return read;
    }
    let largest: Vec<f64> = factors.iter().map(|factor| factor.weight.1).collect();
    for laid in 1..(1_u32 << factors.len()) {
        widen_by_subset(
            &mut read,
            factors,
            &largest,
            |j| laid & (1 << j) != 0,
            speed,
        );
    }
    read
}

/// Widens `read` to the valleys of the subset `laid` of `factors`, the
/// other factors of each product read at `largest`.
fn widen_by_subset(
    read: &mut ValleyRead,
    factors: &[Factor],
    largest: &[f64],
    laid: impl Fn(usize) -> bool,
    speed: f64,
) {
    let others = |skip: &[usize]| -> f64 {
        (0..factors.len())
            .filter(|&i| laid(i) && !skip.contains(&i))
            .map(|i| largest[i])
            .product()
    };
    let (mut rebuild, mut fold, mut bend, mut cross) = (0.0, 0.0, 0.0, 0.0);
    for (j, factor) in factors.iter().enumerate().filter(|&(j, _)| laid(j)) {
        let rest = others(&[j]);
        rebuild += factor.rebuild * rest;
        fold += factor.fold * rest;
        bend += factor.bend * rest;
        for (k, other) in factors
            .iter()
            .enumerate()
            .skip(j + 1)
            .filter(|&(k, _)| laid(k))
        {
            cross += 2.0 * factor.rebuild * other.rebuild * speed * others(&[j, k]);
        }
    }
    read.rebuild = read.rebuild.max(rebuild);
    read.fold = read.fold.max(fold + cross);
    read.bend = read.bend.max(bend + cross);
}

/// A run of grid cells where the shaped speed may fall under the fitted
/// leaders' crawl ([`FitMiss::crawl_speed`]), and the one valley the
/// follower projection lays across the crawls it finds along it: it widens
/// the last crawl valley over any crawl whose sides would overlap it.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct Crawl {
    pub slot: Slot,
    /// The source line of the move the run starts in.
    pub line: u32,
}

/// Why a crawl's valley does not fit before it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct Overreach {
    /// The widest the valley's side may be.
    pub width: f64,
    /// The least path the leaders cover in the [`VALLEY_LEAD_BUDGET_S`]
    /// before the crawl.
    pub covered: f64,
    /// The widest the side would be with the leaders' rate at the crawl
    /// at the fit's tolerance alone.
    pub width_at_rest: f64,
}

impl Crawl {
    pub fn cells(&self) -> (i64, i64) {
        self.slot.cells
    }

    /// Whether its valley may reach a cell of `from..=to`.
    pub fn reaches(&self, track: &CellTrack, (from, to): (i64, i64), h: f64, fit: FitMiss) -> bool {
        let slot = &self.slot;
        let nearest = if to < slot.cells.0 {
            to
        } else if from > slot.cells.1 {
            from
        } else {
            return true;
        };
        slot.distance(track, nearest, h, fit).0 < slot.width.1
    }

    /// Checks that its valley's side fits within the path the leaders
    /// cover in the [`VALLEY_LEAD_BUDGET_S`] before the crawl, walking back
    /// from its first cell until the least path covered reaches the widest
    /// side; where the segment's table starts inside the budget, at the
    /// start of cell `table_start + 1`, the projection narrows the side to
    /// the path since ([`geometry::valley_reach`] taken within the table).
    pub fn before_side(
        &mut self,
        track: &CellTrack,
        table_start: Option<i64>,
        h: f64,
        fit: FitMiss,
    ) -> Result<(), Overreach> {
        let (first, last) = self.slot.cells;
        let widest = self.slot.width.1;
        let budget_cells = (VALLEY_LEAD_BUDGET_S / h).floor() as i64;
        let earliest = table_start.map_or(first - budget_cells, |cell| {
            (first - budget_cells).max(cell + 1)
        });
        assert!(
            earliest >= track.base(),
            "turning guard: the crawl at cell {first} reads cells from {earliest} the track no \
             longer holds (from {})",
            track.base()
        );
        let mut covered = 0.0_f64;
        for n in (earliest..first).rev() {
            let drift = track.drift(n, last, h, fit);
            covered = covered.max(track.path(n, first - 1).0 - drift);
            if covered >= widest {
                return Ok(());
            }
        }
        if let Some(cell) = table_start {
            let drift = track.drift(cell + 1, last, h, fit);
            let shortest = (track.path(cell + 1, first - 1).0 - drift).max(0.0);
            let longest = track.path(cell + 1, last).1 + drift;
            self.slot.width = (
                self.slot.width.0.min(shortest),
                self.slot.width.1.min(longest),
            );
            return Ok(());
        }
        Err(Overreach {
            width: widest,
            covered: covered.max(0.0),
            width_at_rest: valley_reach(self.slot.speed, fit.accel),
        })
    }
}

/// What the valleys of `crawls` weigh the advance by in cell `n`, whose
/// shaped speed lies within `speed`. Where two may reach it, the
/// projection may have merged them, holding the weight at zero there.
pub(super) fn read(
    crawls: &[Crawl],
    track: &CellTrack,
    n: i64,
    speed: (f64, f64),
    h: f64,
    fit: FitMiss,
) -> ValleyRead {
    let fitted = ((speed.0 - fit.speed).max(0.0), speed.1 + fit.speed);
    let factors: Vec<Factor> = crawls
        .iter()
        .filter_map(|crawl| crawl.slot.read(track, n, fitted, h, fit))
        .collect();
    let read = weigh(&factors, fitted.1);
    if factors.len() > 1 {
        ValleyRead {
            weight_floor: 0.0,
            ..read
        }
    } else {
        read
    }
}

#[cfg(test)]
mod tests;
