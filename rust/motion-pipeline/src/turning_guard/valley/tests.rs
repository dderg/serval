use geometry::{CrawlSide, VALLEY_LEAD_BUDGET_S, widest_crawl_side_within};

use super::{CellBounds, CellTrack, Crawl, FitMiss, Slot};

const H: f64 = 2.5e-4;

fn cell(speed: f64, accel: f64) -> CellBounds {
    CellBounds {
        speed: (speed, speed),
        accel: (accel, accel),
        turn: 0.0,
    }
}

/// A track at `cruise` mm/s but for the cells `crawl` at a crawl, up to
/// cell `end`.
fn track_with_crawl(cruise: f64, crawl: (i64, i64), end: i64, fit: FitMiss) -> CellTrack {
    let mut track = CellTrack::default();
    for n in 0..end {
        let bounds = if (crawl.0..=crawl.1).contains(&n) {
            CellBounds {
                speed: (0.0, 0.5),
                accel: (0.0, 300.0),
                turn: f64::INFINITY,
            }
        } else {
            cell(cruise, 300.0)
        };
        track.push(n, bounds, H, fit);
    }
    track
}

#[test]
fn a_crawl_leaves_room_for_the_widest_grid_side_the_lead_budget_covers_before_it() {
    let fit = FitMiss::of(0.005, 50.0);
    let crawl_cells = (800, 810);
    let track = track_with_crawl(10.0, crawl_cells, 1200, fit);
    let slot = Slot::over(&track, crawl_cells, fit).expect("the run crawls");
    let crawl = Crawl::found(slot, 1, &track, None, H, fit).expect("the side fits the budget");
    let budget_cells = (VALLEY_LEAD_BUDGET_S / H).floor() as i64;
    let covered = (crawl_cells.0 - budget_cells..crawl_cells.0)
        .map(|n| track.path(n, crawl_cells.0 - 1).0 - track.drift(n, crawl_cells.1, H, fit))
        .fold(0.0_f64, f64::max);
    let widest = widest_crawl_side_within(slot.width.1, covered).expect("a grid width fits");
    assert_eq!(crawl.before.width, widest);
    assert!(widest <= covered && widest > slot.width.1);
    let reached_back = track.path(crawl.before.cells.0, crawl_cells.0 - 1).1;
    assert!(
        reached_back >= widest,
        "the side reaches {reached_back} mm of cells back, short of its {widest} mm"
    );
    let expected = CrawlSide {
        speed: slot.speed,
        accel: 300.0 + fit.accel,
        fastest: 10.0 + fit.speed,
        floor: slot.floor,
    }
    .rebuild(widest);
    assert_eq!(crawl.before.rebuild, expected);
}

#[test]
fn a_crawl_s_after_side_waits_for_the_lead_budget_past_its_run() {
    let fit = FitMiss::of(0.005, 50.0);
    let crawl_cells = (800, 810);
    let budget_cells = (VALLEY_LEAD_BUDGET_S / H).floor() as i64;
    let short = track_with_crawl(10.0, crawl_cells, crawl_cells.1 + budget_cells / 4, fit);
    let slot = Slot::over(&short, crawl_cells, fit).expect("the run crawls");
    let mut crawl = Crawl::found(slot, 1, &short, None, H, fit).expect("the side fits the budget");
    assert!(
        !crawl.resolve_after(&short, false, H, fit),
        "the after side settled before its budget's cells were tracked"
    );
    let long = track_with_crawl(10.0, crawl_cells, crawl_cells.1 + budget_cells + 1, fit);
    assert!(crawl.resolve_after(&long, false, H, fit));
    let after = crawl.after.expect("the after side settled");
    assert!(after.cells.0 == crawl_cells.0 && after.cells.1 > crawl_cells.1);
    assert_eq!(crawl.reach(), (crawl.before.cells.0, after.cells.1));
}

#[test]
fn a_crawl_with_a_rest_inside_its_budget_reaches_back_no_further_than_the_rest() {
    let fit = FitMiss::of(0.005, 50.0);
    let crawl_cells = (800, 810);
    let mut track = CellTrack::default();
    for n in 0..900 {
        let bounds = if n == 780 {
            cell(0.0, 0.0)
        } else if (crawl_cells.0..=crawl_cells.1).contains(&n) {
            CellBounds {
                speed: (0.0, 0.5),
                accel: (0.0, 300.0),
                turn: f64::INFINITY,
            }
        } else {
            cell(10.0, 300.0)
        };
        track.push(n, bounds, H, fit);
    }
    let slot = Slot::over(&track, crawl_cells, fit).expect("the run crawls");
    let crawl = Crawl::found(slot, 1, &track, None, H, fit).expect("a rest bounds the side");
    assert!(
        crawl.before.cells.0 > 780,
        "the side reaches past the rest: {crawl:?}"
    );
}
