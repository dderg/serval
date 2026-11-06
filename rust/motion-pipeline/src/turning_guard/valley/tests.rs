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
        .map(|cell| cell.leaders.ask(&leaders.within, super::Ask::Rebuild))
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

/// The first cell of the move that brakes into the crawl of
/// [`braking_into_a_crawl`].
const BRAKE_CELL: i64 = 600;

/// Cell `n` of a long move braking from 46 to 10 mm/s, then of one braking
/// on into a crawl at 0.5 mm/s over the cells `crawl`, all at `scale` of
/// their speed: the shaped motion itself, and what the track holds, the
/// box over the long move, which keeps its unscaled entry and its
/// 2000 mm/s² junction whatever the scale, and the motion itself where the
/// braking move may crawl.
fn braking_into_a_crawl(scale: f64, crawl: (i64, i64), n: i64) -> (CellBounds, CellBounds) {
    if n >= crawl.0 {
        let crawling = CellBounds {
            speed: (0.0, scale * 0.5),
            accel: (0.0, scale * scale * 300.0),
            turn: f64::INFINITY,
            tangential: (-scale * scale * 300.0, scale * scale * 300.0),
        };
        return (crawling, crawling);
    }
    let (entry, exit, crawl_speed) = (46.0, 10.0, 0.5);
    let along = |from: f64, to: f64, (first, past): (i64, i64)| {
        let span = (past - first) as f64;
        (
            scale * (from + (to - from) * (n - first) as f64 / span),
            scale * scale * (from - to) / (span * H),
        )
    };
    let (speed, accel) = if n < BRAKE_CELL {
        along(entry, exit, (0, BRAKE_CELL))
    } else {
        along(exit, crawl_speed, (BRAKE_CELL, crawl.0))
    };
    let shaped = CellBounds {
        speed: (speed, speed),
        accel: (accel, accel),
        turn: 0.0,
        tangential: (-accel, -accel),
    };
    let tracked = if n < BRAKE_CELL {
        CellBounds {
            speed: (scale * exit, 60.0),
            accel: (0.0, 2000.0),
            turn: 0.0,
            tangential: (-2000.0, 2000.0),
        }
    } else {
        shaped
    };
    (shaped, tracked)
}

/// The most the before side of the crawl past [`braking_into_a_crawl`]
/// at `scale` leaves room for in any of its cells.
fn reserved_braking_at(scale: f64) -> f64 {
    let fit = FitMiss::of(0.005, 50.0);
    let crawl_cells = (800, 810);
    let mut track = CellTrack::default();
    for n in 0..1200 {
        track.push(n, braking_into_a_crawl(scale, crawl_cells, n).1, H, fit);
    }
    let slot = Slot::over(&track, crawl_cells, fit).expect("the run crawls");
    let shaped = |n: i64| braking_into_a_crawl(scale, crawl_cells, n).0;
    let crawl =
        Crawl::found(slot, 1, &track, (-1, false), (H, fit), &shaped).expect("the side fits");
    let (first, last) = crawl.before.cells;
    (first..=last)
        .filter_map(|n| crawl.before.rebuild_where(n, |_| true))
        .fold(0.0, f64::max)
}

/// The guard backs off a move braking into a crawl by slowing it, which
/// narrows the before side the lead budget covers. The room it leaves for
/// the rebuild must not grow with it, or the back-off raises the demand it
/// answers: the cells are read at their own motion, not at the fastest and
/// strongest the box over their move holds (`trident_follower_cache_regression`
/// line 935 on the p1 bowden profile, 5000 to 18540 mm/s² over 19
/// back-offs).
#[test]
fn slowing_the_approach_to_a_crawl_never_grows_the_room_its_valley_side_needs() {
    let reserved: Vec<(f64, f64)> = [1.0, 0.8, 0.6, 0.45, 0.35]
        .into_iter()
        .map(|scale| (scale, reserved_braking_at(scale)))
        .collect();
    for pair in reserved.windows(2) {
        assert!(
            pair[1].1 <= pair[0].1,
            "slowing the approach from {} to {} of its speed grows the reserve from {} to {}: \
             {reserved:?}",
            pair[0].0,
            pair[1].0,
            pair[0].1,
            pair[1].1
        );
    }
}
