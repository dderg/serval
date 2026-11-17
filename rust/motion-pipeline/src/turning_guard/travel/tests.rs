use super::{Ahead, ExtrudingCells, Odometer, TravelCrawl};
use crate::turning_guard::valley::{FitMiss, Slot};

const H: f64 = 2.5e-4;
const REACH_MM: f64 = 1.0;
const RAMP_MM: f64 = 0.5;
const RAMP_CAP_MM: f64 = 2.0;
const SPEED_MM_S: f64 = 100.0;
const RUN: (i64, i64) = (100, 110);

fn odometer() -> Odometer {
    Odometer::new(H, FitMiss::of(0.005, 50.0))
}

/// The odometer reading through cell `n` of a cruise at `SPEED_MM_S`
/// from cell 0.
fn reading(n: i64) -> f64 {
    odometer().through(n, SPEED_MM_S * (n + 1) as f64 * H)
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

fn extruding_on(cells: impl IntoIterator<Item = i64>) -> ExtrudingCells {
    let mut extruding = ExtrudingCells::default();
    for n in cells {
        extruding.push(n, Some(RAMP_MM), (reading(n - 1), reading(n)));
    }
    extruding
}

/// How many cells of the cruise the fitted leaders take at least to cover
/// `path_mm`.
fn cells_over(path_mm: f64) -> i64 {
    (1..).find(|&cells| reading(cells) - reading(0) > path_mm).unwrap()
}

fn travel() -> TravelCrawl {
    TravelCrawl::new(slot(), 1, reading(RUN.1))
}

#[test]
fn an_extrusion_whose_ramp_reaches_within_the_reach_behind_needs_the_valley() {
    let (start, end) = RUN;
    let before_start = reading(start - 1);
    let past = start - 1 - cells_over(REACH_MM + RAMP_MM);
    let within = past + 1;

    assert!(extruding_on([within]).behind(RUN, before_start, REACH_MM));
    assert!(!extruding_on([past]).behind(RUN, before_start, REACH_MM));
    assert!(extruding_on([end]).behind(RUN, before_start, REACH_MM));
    assert!(!ExtrudingCells::default().behind(RUN, before_start, REACH_MM));
}

#[test]
fn a_run_folded_before_the_track_still_reaches_the_crawls_after_it() {
    let start = RUN.0;
    let within = start - 1 - cells_over(REACH_MM + RAMP_MM) + 1;
    let mut extruding = extruding_on([within]);
    extruding.trim_before(within + 1);

    assert!(extruding.behind(RUN, reading(start - 1), REACH_MM));
}

#[test]
fn an_extrusion_whose_ramp_reaches_within_the_reach_ahead_needs_the_valley() {
    let end = RUN.1;
    let past = end + 1 + cells_over(REACH_MM + RAMP_MM);
    let within = past - 1;
    let last = |n: i64| (n, reading(n));

    assert!(matches!(
        travel().ahead(&extruding_on([within]), last(within), RAMP_CAP_MM, false),
        Ahead::Extrudes
    ));
    assert!(matches!(
        travel().ahead(&extruding_on([past]), last(past), RAMP_CAP_MM, false),
        Ahead::Waits
    ));
}

#[test]
fn a_run_clears_past_its_reach_and_the_widest_ramp_to_come() {
    let end = RUN.1;
    let cleared = end + cells_over(REACH_MM + RAMP_CAP_MM);
    let none = ExtrudingCells::default();
    let last = |n: i64| (n, reading(n));

    assert!(matches!(
        travel().ahead(&none, last(cleared - 1), RAMP_CAP_MM, false),
        Ahead::Waits
    ));
    assert!(matches!(
        travel().ahead(&none, last(cleared - 1), RAMP_CAP_MM, true),
        Ahead::Clear(cell) if cell == cleared - 1
    ));
    assert!(matches!(
        travel().ahead(&none, last(cleared), RAMP_CAP_MM, false),
        Ahead::Clear(cell) if cell == cleared
    ));
}

#[test]
fn a_rescan_forgets_the_extruding_cells_it_reads_again() {
    let first = RUN.1 + 5;
    let mut extruding = extruding_on([first, first + 1, first + 2]);
    extruding.truncate_from(first + 1, reading(first));
    let last = (first, reading(first));

    assert!(matches!(
        travel().ahead(&extruding, last, RAMP_CAP_MM, false),
        Ahead::Extrudes
    ));

    extruding.truncate_from(first, reading(first - 1));

    assert!(matches!(
        travel().ahead(&extruding, (first - 1, reading(first - 1)), RAMP_CAP_MM, false),
        Ahead::Waits
    ));
}

#[test]
#[should_panic(expected = "within its 1 mm reach")]
fn an_extrusion_within_the_reach_of_a_cleared_run_fails_loudly() {
    let mut cleared = travel();
    cleared.cleared = Some(RUN.1 + 1);
    let wide_ramp = 4.0 * RAMP_CAP_MM;
    let n = RUN.1 + cells_over(REACH_MM + RAMP_CAP_MM) + 1;

    cleared.assert_clear_of(n, reading(n - 1), wide_ramp);
}
