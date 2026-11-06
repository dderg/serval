use geometry::{CrawlSide, VALLEY_LEAD_BUDGET_S, widest_crawl_side_within};

use super::{CellBounds, CellTrack, Crawl, FitMiss, Reserve, Slot, Toward};

const H: f64 = 2.5e-4;

fn cell(speed: f64, accel: f64) -> CellBounds {
    CellBounds {
        speed: (speed, speed),
        accel: (accel, accel),
        turn: 0.0,
        tangential: (-accel, accel),
    }
}

fn crawling() -> CellBounds {
    CellBounds {
        speed: (0.0, 0.5),
        accel: (0.0, 300.0),
        turn: f64::INFINITY,
        tangential: (-300.0, 300.0),
    }
}

/// A track at `cruise` mm/s but for the cells `crawl` at a crawl, up to
/// cell `end`.
fn track_with_crawl(cruise: f64, crawl: (i64, i64), end: i64, fit: FitMiss) -> CellTrack {
    let mut track = CellTrack::default();
    for n in 0..end {
        let bounds = if (crawl.0..=crawl.1).contains(&n) {
            crawling()
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
    let crawl = Crawl::found(slot, 1, &track, (-1, false), (H, fit), &|n| track.get(n))
        .expect("the side fits the budget");
    let budget_cells = (VALLEY_LEAD_BUDGET_S / H).floor() as i64;
    let covered = (crawl_cells.0 - budget_cells..crawl_cells.0)
        .map(|n| track.path(n, crawl_cells.0 - 1).0 - track.drift(n, crawl_cells.0 - 1, H, fit))
        .fold(0.0_f64, f64::max);
    let widest = widest_crawl_side_within(slot.width.1, covered).expect("a grid width fits");
    assert_eq!(crawl.before.width, widest);
    assert!(widest <= covered && widest > slot.width.1);
    let reached_back = track.path(crawl.before.cells.0, crawl_cells.0 - 1).1;
    assert!(
        reached_back >= widest,
        "the side reaches {reached_back} mm of cells back, short of its {widest} mm"
    );
    let whole_side =
        CrawlSide::either_way(slot.speed, 300.0 + fit.accel, 10.0 + fit.speed, slot.floor)
            .rebuild(widest);
    let (first, last) = crawl.before.cells;
    let most = (first..=last)
        .filter_map(|n| crawl.before.rebuild_where(n, |_| true))
        .fold(0.0, f64::max);
    assert!(
        0.0 < most && most <= whole_side,
        "the side's cells ask up to {most}, past the whole side's {whole_side}"
    );
}

#[test]
fn the_block_screen_leaves_every_cell_s_rebuild_as_its_own_bound_reads_it() {
    let fit = FitMiss::of(0.005, 50.0);
    let crawl_cells = (800, 810);
    let mut track = CellTrack::default();
    for n in 0..1200 {
        let bounds = if (crawl_cells.0..=crawl_cells.1).contains(&n) {
            crawling()
        } else {
            let wave = libm::sin(n as f64 * 0.05);
            cell(10.0 + 4.0 * wave, 300.0 + 250.0 * wave)
        };
        track.push(n, bounds, H, fit);
    }
    let slot = Slot::over(&track, crawl_cells, fit).expect("the run crawls");
    let crawl = Crawl::found(slot, 1, &track, (-1, false), (H, fit), &|n| track.get(n))
        .expect("the side fits the budget");
    let side = &crawl.before;
    let leaders = side.leaders.as_ref().expect("the side leaves room");
    let whole = leaders.side.rebuild(side.width);
    let own: Vec<f64> = leaders
        .cells
        .iter()
        .map(|cell| {
            let at = cell.leaders;
            leaders.within.at(at.distance, at.speed, at.accel)
        })
        .collect();
    let (least, most) = own
        .iter()
        .fold((f64::INFINITY, 0.0_f64), |(lo, hi), &rebuild| {
            (lo.min(rebuild), hi.max(rebuild))
        });
    assert!(
        least < most && own.len() > 2 * super::REBUILD_SCREEN_CELLS,
        "the side's cells read one rebuild, {least}..{most}, over {} cells",
        own.len()
    );
    for step in 0..=16 {
        let threshold = least + (most - least) * f64::from(step) / 16.0;
        for (offset, &rebuild) in own.iter().enumerate() {
            let n = side.cells.0 + offset as i64;
            let expected = (rebuild > threshold).then(|| rebuild.min(whole));
            assert_eq!(
                side.rebuild_where(n, |bound| bound > threshold),
                expected,
                "cell {n} over the threshold {threshold}"
            );
        }
    }
}

#[test]
fn a_crawl_s_after_side_waits_for_the_lead_budget_past_its_run() {
    let fit = FitMiss::of(0.005, 50.0);
    let crawl_cells = (800, 810);
    let budget_cells = (VALLEY_LEAD_BUDGET_S / H).floor() as i64;
    let short = track_with_crawl(10.0, crawl_cells, crawl_cells.1 + budget_cells / 4, fit);
    let slot = Slot::over(&short, crawl_cells, fit).expect("the run crawls");
    let mut crawl = Crawl::found(slot, 1, &short, (-1, false), (H, fit), &|n| short.get(n))
        .expect("the side fits the budget");
    assert!(
        !crawl.resolve_after(&short, false, (H, fit), &|n| short.get(n)),
        "the after side settled before its budget's cells were tracked"
    );
    let long = track_with_crawl(10.0, crawl_cells, crawl_cells.1 + budget_cells + 1, fit);
    assert!(crawl.resolve_after(&long, false, (H, fit), &|n| long.get(n)));
    let after = crawl.after.clone().expect("the after side settled");
    assert!(after.cells.0 == crawl_cells.0 && after.cells.1 > crawl_cells.1);
    assert_eq!(crawl.reach(), (crawl.before.cells.0, after.cells.1));
}

/// The projection holds the weight at zero back to a rest where no before
/// side reaching no further back fits: no room is left for one.
#[test]
fn a_crawl_with_a_rest_inside_its_budget_leaves_no_room_before_it() {
    let fit = FitMiss::of(0.005, 50.0);
    let crawl_cells = (800, 810);
    let track = track_with_rest(crawl_cells, 780, fit);
    let slot = Slot::over(&track, crawl_cells, fit).expect("the run crawls");
    let crawl = Crawl::found(slot, 1, &track, (-1, false), (H, fit), &|n| track.get(n))
        .expect("a rest bounds the side");
    assert_eq!(crawl.before, Reserve::none(crawl_cells, Toward::Before));
}

/// A run the leaders come to rest past holds the edge of the rest valley
/// the projection lays there, whose before side reaches past an earlier
/// rest: room is left for it, as wide as the lead budget covers, over a
/// side keeping no floor.
#[test]
fn a_run_into_a_rest_leaves_room_for_the_rest_valley_s_before_side_past_an_earlier_rest() {
    let fit = FitMiss::of(0.005, 50.0);
    let crawl_cells = (800, 810);
    let mut track = CellTrack::default();
    for n in 0..900 {
        let bounds = if n == 780 {
            cell(0.0, 0.0)
        } else if (crawl_cells.0..=crawl_cells.1).contains(&n) {
            crawling()
        } else {
            cell(10.0, 300.0)
        };
        track.push(n, bounds, H, fit);
    }
    let slot = Slot::over(&track, crawl_cells, fit)
        .expect("the run crawls")
        .beside_rest();
    assert_eq!(slot.floor.0, 0.0, "a rest valley's side keeps no floor");
    let crawl = Crawl::found(slot, 1, &track, (-1, true), (H, fit), &|n| track.get(n))
        .expect("the side fits the budget");
    let budget_cells = (VALLEY_LEAD_BUDGET_S / H).floor() as i64;
    let covered = (crawl_cells.0 - budget_cells..crawl_cells.0)
        .map(|n| track.path(n, crawl_cells.0 - 1).0 - track.drift(n, crawl_cells.0 - 1, H, fit))
        .fold(0.0_f64, f64::max);
    let widest = widest_crawl_side_within(slot.width.1, covered).expect("a grid width fits");
    assert_eq!(crawl.before.width, widest);
    assert!(
        crawl.before.cells.0 < 780,
        "the side's room stops at the earlier rest: {:?}",
        crawl.before.cells
    );
}

/// A rest further back than the lead budget but within the widest the
/// slot's leaders reach bounds the side too.
#[test]
fn a_rest_within_the_slot_s_reach_past_the_budget_bounds_the_side() {
    let fit = FitMiss::of(0.005, 50.0);
    let crawl_cells = (800, 810);
    let budget_cells = (VALLEY_LEAD_BUDGET_S / H).floor() as i64;
    let rest = crawl_cells.0 - budget_cells - 5;
    let track = track_with_rest(crawl_cells, rest, fit);
    let mut slot = Slot::over(&track, crawl_cells, fit).expect("the run crawls");
    let past_rest =
        track.path(rest + 1, crawl_cells.0 - 1).1 + track.drift(rest + 1, crawl_cells.1, H, fit);
    let without = Crawl::found(slot, 1, &track, (-1, false), (H, fit), &|n| track.get(n));
    assert!(
        without.is_err(),
        "a side the slow motion since the rest cannot hold fits: {without:?}"
    );
    slot.width.1 = past_rest;
    let crawl = Crawl::found(slot, 1, &track, (-1, false), (H, fit), &|n| track.get(n))
        .expect("the rest bounds the side");
    assert_eq!(crawl.before, Reserve::none(crawl_cells, Toward::Before));
}

/// A track crawling over the cells `crawl`, resting at cell `rest` and
/// moving at 1 mm/s elsewhere.
fn track_with_rest(crawl: (i64, i64), rest: i64, fit: FitMiss) -> CellTrack {
    let mut track = CellTrack::default();
    for n in 0..900 {
        let bounds = if n == rest {
            cell(0.0, 0.0)
        } else if (crawl.0..=crawl.1).contains(&n) {
            crawling()
        } else {
            cell(1.0, 300.0)
        };
        track.push(n, bounds, H, fit);
    }
    track
}
