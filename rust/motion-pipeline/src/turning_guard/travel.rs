use std::collections::VecDeque;

use super::valley::{FitMiss, Slot};

/// The runs of tracked cells whose planned motion a guarded follower
/// extrudes on, where the projection may read a ratio and so lay a crawl
/// valley ([`TravelCrawl`]). Each run keeps the [`Odometer`] reading before
/// its first cell and through its last, and how far along the path the
/// ramps of its ratio steps may reach; runs before the track's base fold
/// into the furthest any of their ramps reaches.
#[derive(Debug)]
pub(super) struct ExtrudingCells {
    runs: VecDeque<ExtrudingRun>,
    trimmed_reach: f64,
}

#[derive(Debug, Clone, Copy)]
struct ExtrudingRun {
    first: i64,
    last: i64,
    before: f64,
    through: f64,
    ramp_mm: f64,
}

impl ExtrudingRun {
    /// The furthest along the [`Odometer`] its ramps reach.
    fn reach(&self) -> f64 {
        self.through + self.ramp_mm
    }
}

/// The least path the fitted leaders cover from the start of the segment
/// through a cell `n`: the least shaped path the crawl track sums through
/// it less the speed miss times the time, a bound on how far the fitted
/// odometer strays ([`super::valley::TrackView::drift`]). Two readings'
/// difference bounds the fitted leaders' path between their cells below.
#[derive(Debug, Clone, Copy)]
pub(super) struct Odometer {
    h: f64,
    fit: FitMiss,
}

impl Odometer {
    pub fn new(h: f64, fit: FitMiss) -> Self {
        Self { h, fit }
    }

    /// The reading through cell `n`, the track summing `floor_through` of
    /// least shaped path through it.
    pub fn through(&self, n: i64, floor_through: f64) -> f64 {
        floor_through - self.fit.speed * n as f64 * self.h
    }
}

impl Default for ExtrudingCells {
    fn default() -> Self {
        Self {
            runs: VecDeque::new(),
            trimmed_reach: f64::NEG_INFINITY,
        }
    }
}

impl ExtrudingCells {
    /// Tracks cell `n`, the next, whose moves' ratio steps ramp up to
    /// `ramp` along the path where a guarded follower extrudes on them, the
    /// odometer reading `before` it and `through` it.
    pub fn push(&mut self, n: i64, ramp: Option<f64>, (before, through): (f64, f64)) {
        let Some(ramp_mm) = ramp else {
            return;
        };
        if let Some(run) = self.runs.back_mut().filter(|run| run.last + 1 == n) {
            run.last = n;
            run.through = through;
            run.ramp_mm = run.ramp_mm.max(ramp_mm);
            return;
        }
        self.runs.push_back(ExtrudingRun {
            first: n,
            last: n,
            before,
            through,
            ramp_mm,
        });
    }

    /// Forgets the cells from `n` on, the odometer reading `before` it.
    /// A run cut short keeps the widest ramp its cells took: a rescan reads
    /// them again.
    pub fn truncate_from(&mut self, n: i64, before: f64) {
        while self.runs.back().is_some_and(|run| run.first >= n) {
            self.runs.pop_back();
        }
        if let Some(run) = self.runs.back_mut().filter(|run| run.last >= n) {
            run.last = n - 1;
            run.through = before;
        }
    }

    /// Folds the runs ending before cell `n`, which no rescan reads again,
    /// into the furthest their ramps reach.
    pub fn trim_before(&mut self, n: i64) {
        while let Some(run) = self.runs.front().filter(|run| run.last < n).copied() {
            self.trimmed_reach = self.trimmed_reach.max(run.reach());
            self.runs.pop_front();
        }
    }

    pub fn clear(&mut self) {
        *self = Self::default();
    }

    /// Whether the follower extrudes on a cell of `start..=end`, or the
    /// ramps of an extrusion before reach within `reach` of `start`, the
    /// odometer reading `before_start` before it.
    pub fn behind(&self, (start, end): (i64, i64), before_start: f64, reach: f64) -> bool {
        let earlier = self.runs.iter().filter(|run| run.first <= end);
        if earlier.clone().any(|run| run.last >= start) {
            return true;
        }
        let furthest = earlier
            .map(ExtrudingRun::reach)
            .fold(self.trimmed_reach, f64::max);
        before_start - furthest <= reach
    }

    /// Whether the ramps of a run starting after cell `end` reach within
    /// `reach` of it, the odometer reading `through_end` through it.
    fn ahead(&self, end: i64, through_end: f64, reach: f64) -> bool {
        self.runs
            .iter()
            .filter(|run| run.first > end)
            .any(|run| run.before - run.ramp_mm - through_end <= reach)
    }
}

/// A run of crawling cells along which the follower extrudes nothing in
/// the path the projection reads behind it: its crawls lay no valley
/// (the projection's `Crossing::needs_valley`) unless the follower extrudes
/// within the leaders' reach ahead too. `through_end` is the
/// [`Odometer`] reading through its last cell; `cleared` the cell through
/// which the track proved the reach ahead extrudes nothing, `None` while
/// it waits, and `through_cleared` the reading through it.
#[derive(Debug, Clone, Copy)]
pub(super) struct TravelCrawl {
    pub slot: Slot,
    pub line: u32,
    through_end: f64,
    pub cleared: Option<i64>,
    through_cleared: f64,
}

/// What the track says of a [`TravelCrawl`]'s reach ahead.
pub(super) enum Ahead {
    Waits,
    Extrudes,
    Clear(i64),
}

impl TravelCrawl {
    pub fn new(slot: Slot, line: u32, through_end: f64) -> Self {
        Self {
            slot,
            line,
            through_end,
            cleared: None,
            through_cleared: through_end,
        }
    }

    pub fn cells(&self) -> (i64, i64) {
        self.slot.cells
    }

    /// Reads the cells tracked past the run through `last`, the odometer
    /// reading `through_last` through it: the ramps of an extrusion there
    /// reach within the leaders' reach; the path covered is past the reach
    /// and `ramp_cap`, the widest ramp a move to come may take; or the
    /// segment ends (`all`) and the leaders rest after it.
    pub fn ahead(
        &self,
        extruding: &ExtrudingCells,
        (last, through_last): (i64, f64),
        ramp_cap: f64,
        all: bool,
    ) -> Ahead {
        let reach = self.slot.width.1;
        if extruding.ahead(self.slot.cells.1, self.through_end, reach) {
            return Ahead::Extrudes;
        }
        if all || through_last - self.through_end > reach + ramp_cap {
            return Ahead::Clear(last);
        }
        Ahead::Waits
    }

    /// Marks the reach ahead clear through cell `cell`, the odometer
    /// reading `through` through it.
    pub fn clear(&mut self, cell: i64, through: f64) {
        self.cleared = Some(cell);
        self.through_cleared = through;
    }

    /// Fails loudly where the ramps of an extrusion starting at cell `n`,
    /// up to `ramp` wide, the odometer reading `before` it, reach within
    /// the leaders' reach of a cleared run: the projection would lay a
    /// valley there the guard left no room for. The leaders' path only
    /// grows, so the path to `n` is at least the path to the cleared cell,
    /// which the clear proved longer than the reach and the ramp cap, and
    /// the guard admits no move whose ramp passes that cap: unreachable
    /// unless one of those proofs is wrong.
    pub fn assert_clear_of(&self, n: i64, before: f64, ramp: f64) {
        let Some(cleared) = self.cleared else {
            return;
        };
        let path = before.max(self.through_cleared) - ramp - self.through_end;
        assert!(
            path > self.slot.width.1,
            "turning guard: the crawl run over cells {:?} (line {}), cleared through cell {cleared}, \
             has an extrusion at cell {n} whose {ramp} mm ramp reaches {path} mm of it, within its \
             {} mm reach",
            self.slot.cells,
            self.line,
            self.slot.width.1
        );
    }
}

#[cfg(test)]
mod tests;
