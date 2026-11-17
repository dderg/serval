use std::collections::VecDeque;

use super::valley::{FitMiss, Slot};

/// The runs of tracked cells whose planned motion a guarded follower
/// extrudes on, each with the least path the track sums through the cell
/// before it and through its last: where the projection reads a ratio it
/// may lay a crawl valley ([`TravelCrawl`]). The last run before the
/// track's base stays, for the crawls still to close after it.
#[derive(Debug, Default)]
pub(super) struct ExtrudingCells {
    runs: VecDeque<ExtrudingRun>,
}

#[derive(Debug, Clone, Copy)]
struct ExtrudingRun {
    first: i64,
    last: i64,
    floor_before: f64,
    floor_through: f64,
}

impl ExtrudingCells {
    /// Tracks cell `n`, the next, the track's least path through it
    /// `floor_through` and through the cell before it `floor_before`.
    pub fn push(&mut self, n: i64, extrudes: bool, (floor_before, floor_through): (f64, f64)) {
        if !extrudes {
            return;
        }
        if let Some(run) = self.runs.back_mut().filter(|run| run.last + 1 == n) {
            run.last = n;
            run.floor_through = floor_through;
            return;
        }
        self.runs.push_back(ExtrudingRun {
            first: n,
            last: n,
            floor_before,
            floor_through,
        });
    }

    /// Forgets the cells from `n` on, the least path through the cell
    /// before it `floor_before`.
    pub fn truncate_from(&mut self, n: i64, floor_before: f64) {
        while self.runs.back().is_some_and(|run| run.first >= n) {
            self.runs.pop_back();
        }
        if let Some(run) = self.runs.back_mut().filter(|run| run.last >= n) {
            run.last = n - 1;
            run.floor_through = floor_before;
        }
    }

    /// Forgets the runs before the last one ending before cell `n`.
    pub fn trim_before(&mut self, n: i64) {
        while self.runs.get(1).is_some_and(|run| run.last < n) {
            self.runs.pop_front();
        }
    }

    pub fn clear(&mut self) {
        self.runs.clear();
    }

    /// Whether the follower extrudes on a cell of `start..=end`, or on one
    /// before whose path to `start` may be no longer than `reach`, the
    /// least path through the cell before `start` `floor_before_start`.
    pub fn behind(
        &self,
        (start, end): (i64, i64),
        floor_before_start: f64,
        reach: f64,
        (h, fit): (f64, FitMiss),
    ) -> bool {
        let Some(run) = self.runs.iter().rev().find(|run| run.first <= end) else {
            return false;
        };
        run.last >= start
            || least_path(floor_before_start - run.floor_through, start - 1 - run.last, (h, fit))
                <= reach
    }

    /// The first run starting after cell `end`: its first cell and the
    /// least path through the cell before it.
    fn first_after(&self, end: i64) -> Option<(i64, f64)> {
        self.runs
            .iter()
            .find(|run| run.first > end)
            .map(|run| (run.first, run.floor_before))
    }
}

/// The least path the fitted leaders cover over `cells` cells across
/// which the track sums `floor` of least shaped path: less the speed miss
/// times their time, a bound on the drift of the fitted odometer
/// ([`super::valley::TrackView::drift`]).
fn least_path(floor: f64, cells: i64, (h, fit): (f64, FitMiss)) -> f64 {
    floor - fit.speed * cells as f64 * h
}

/// A run of crawling cells along which the follower extrudes nothing in
/// the path the projection reads behind it: the run's crawls lay no
/// valley ([`crate::follower_projection`]'s `needs_valley`) unless the
/// follower extrudes within the leaders' reach ahead too. Extruding reads
/// the planned moves' ratio in reach, which covers the ramps of their
/// ratio steps over the fastest the leaders reach about them
/// (`0.5·v·RATIO_RAMP_MAX_TIME_S`). `cleared` is the cell through which
/// the track proved the reach ahead extrudes nothing, `None` while it
/// waits.
#[derive(Debug, Clone, Copy)]
pub(super) struct TravelCrawl {
    pub slot: Slot,
    pub line: u32,
    floor_through_end: f64,
    pub cleared: Option<i64>,
}

/// What the track says of a [`TravelCrawl`]'s reach ahead.
pub(super) enum Ahead {
    Waits,
    Extrudes,
    Clear(i64),
}

impl TravelCrawl {
    pub fn new(slot: Slot, line: u32, floor_through_end: f64) -> Self {
        Self {
            slot,
            line,
            floor_through_end,
            cleared: None,
        }
    }

    pub fn cells(&self) -> (i64, i64) {
        self.slot.cells
    }

    /// Reads the cells tracked past the run, the last `last` with least
    /// path `floor_through_last` through it: the follower extrudes within
    /// the reach ahead, the path covered there is past the reach, or the
    /// segment ends (`all`) and the leaders rest after it.
    pub fn ahead(
        &self,
        extruding: &ExtrudingCells,
        (last, floor_through_last): (i64, f64),
        all: bool,
        (h, fit): (f64, FitMiss),
    ) -> Ahead {
        let end = self.slot.cells.1;
        let reach = self.slot.width.1;
        let path_since_end = |cell: i64, floor: f64| {
            least_path(floor - self.floor_through_end, cell - end, (h, fit))
        };
        if let Some((first, floor_before)) = extruding.first_after(end) {
            if path_since_end(first - 1, floor_before) <= reach {
                return Ahead::Extrudes;
            }
            return Ahead::Clear(first - 1);
        }
        if all || path_since_end(last, floor_through_last) > reach {
            return Ahead::Clear(last);
        }
        Ahead::Waits
    }
}

#[cfg(test)]
mod tests;
