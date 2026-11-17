use super::{Ahead, ExtrudingCells, TravelCrawl};
use crate::turning_guard::valley::{FitMiss, Slot};

const H: f64 = 2.5e-4;
const REACH_MM: f64 = 1.0;
const SPEED_MM_S: f64 = 100.0;
const RUN: (i64, i64) = (100, 110);

fn fit() -> FitMiss {
    FitMiss::of(0.005, 50.0)
}

fn slot() -> Slot {
    Slot {
        cells: RUN,
        width: (REACH_MM, REACH_MM),
        floor: (0.0, 0.0),
        speed: 1.0,
        leaders: (1.0, 0.0),
    }
}

/// The least path the track sums through cell `n` of a cruise at
/// `SPEED_MM_S` from cell 0.
fn floor_through(n: i64) -> f64 {
    SPEED_MM_S * (n + 1) as f64 * H
}

fn extruding_on(cells: impl IntoIterator<Item = i64>) -> ExtrudingCells {
    let mut extruding = ExtrudingCells::default();
    for n in cells {
        extruding.push(n, true, (floor_through(n - 1), floor_through(n)));
    }
    extruding
}

/// The fitted leaders' least path between the ends of `cells` cells
/// apart on the cruise.
fn least_between(cells: i64) -> f64 {
    (SPEED_MM_S - fit().speed) * cells as f64 * H
}

/// The farthest cell before `RUN` whose extrusion is within reach of it,
/// and the first after.
fn reach_cells() -> i64 {
    (0..).find(|&cells| least_between(cells) > REACH_MM).unwrap() - 1
}

#[test]
fn an_extrusion_within_the_reach_behind_a_run_needs_its_valley() {
    let (start, end) = RUN;
    let floor_before_start = floor_through(start - 1);
    let within = start - 1 - reach_cells();
    let past = within - 1;

    assert!(extruding_on([within]).behind(RUN, floor_before_start, REACH_MM, (H, fit())));
    assert!(!extruding_on([past]).behind(RUN, floor_before_start, REACH_MM, (H, fit())));
    assert!(extruding_on([end]).behind(RUN, floor_before_start, REACH_MM, (H, fit())));
    assert!(!ExtrudingCells::default().behind(RUN, floor_before_start, REACH_MM, (H, fit())));
}

#[test]
fn an_extrusion_within_the_reach_ahead_of_a_run_needs_its_valley() {
    let end = RUN.1;
    let travel = TravelCrawl::new(slot(), 1, floor_through(end));
    let within = end + 1 + reach_cells();
    let past = within + 1;
    let last = |n: i64| (n, floor_through(n));

    assert!(matches!(
        travel.ahead(&extruding_on([within]), last(within), false, (H, fit())),
        Ahead::Extrudes
    ));
    assert!(matches!(
        travel.ahead(&extruding_on([past]), last(past), false, (H, fit())),
        Ahead::Clear(cell) if cell == within
    ));
}

#[test]
fn a_run_waits_for_its_reach_ahead_unless_the_segment_ends() {
    let end = RUN.1;
    let travel = TravelCrawl::new(slot(), 1, floor_through(end));
    let short = end + reach_cells();
    let past = short + 2;
    let none = ExtrudingCells::default();
    let last = |n: i64| (n, floor_through(n));

    assert!(matches!(
        travel.ahead(&none, last(short), false, (H, fit())),
        Ahead::Waits
    ));
    assert!(matches!(
        travel.ahead(&none, last(short), true, (H, fit())),
        Ahead::Clear(cell) if cell == short
    ));
    assert!(matches!(
        travel.ahead(&none, last(past), false, (H, fit())),
        Ahead::Clear(cell) if cell == past
    ));
}

#[test]
fn a_rescan_forgets_the_extruding_cells_it_reads_again() {
    let mut extruding = extruding_on([RUN.1 + 5, RUN.1 + 6, RUN.1 + 7]);
    let first = RUN.1 + 6;
    extruding.truncate_from(first, floor_through(first - 1));
    let travel = TravelCrawl::new(slot(), 1, floor_through(RUN.1));
    let last = (first - 1, floor_through(first - 1));

    assert!(matches!(
        travel.ahead(&extruding, last, false, (H, fit())),
        Ahead::Extrudes
    ));

    extruding.truncate_from(RUN.1 + 5, floor_through(RUN.1 + 4));

    assert!(matches!(
        travel.ahead(&extruding, (RUN.1 + 4, floor_through(RUN.1 + 4)), false, (H, fit())),
        Ahead::Waits
    ));
}
