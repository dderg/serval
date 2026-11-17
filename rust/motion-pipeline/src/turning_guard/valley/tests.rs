use geometry::{CrawlSide, VALLEY_LEAD_BUDGET_S, widest_crawl_side_within};

use super::{CellBounds, CellTrack, Crawl, FitMiss, Reserve, RunScan, Slot, TrackView};

const H: f64 = 2.5e-4;
/// Shaped ceilings that leave a side's cells to the track's box alone.
const BOXED_ONLY: (f64, f64) = (f64::INFINITY, f64::INFINITY);

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

fn unstored(n: i64) -> CellBounds {
    panic!("a test track stores every cell it tracks, not cell {n}")
}

/// A view of `track`, which stores every cell it tracks.
fn view(track: &CellTrack, fit: FitMiss) -> TrackView<'_> {
    TrackView::new(track, &unstored, (H, fit))
}

/// The slot the run of cells `from..=to` lays, read in order.
fn slot_over(track: &TrackView, (from, to): (i64, i64), fit: FitMiss) -> Option<Slot> {
    let mut run = RunScan::default();
    for n in from..=to {
        run.read(n, track.get(n), fit);
    }
    run.slot()
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
        track.push(n, bounds, (H, fit), false);
    }
    track
}

#[test]
fn a_crawl_leaves_room_for_the_widest_grid_side_the_lead_budget_covers_before_it() {
    let fit = FitMiss::of(0.005, 50.0);
    let crawl_cells = (800, 810);
    let track = track_with_crawl(10.0, crawl_cells, 1200, fit);
    let track = view(&track, fit);
    let slot = slot_over(&track, crawl_cells, fit).expect("the run crawls");
    let crawl = Crawl::found(slot, 1, &track, -1, (H, fit), &|n| track.get(n))
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
        .filter_map(|n| crawl.before.rebuild_where(n, &track, BOXED_ONLY, |_| true))
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
        track.push(n, bounds, (H, fit), false);
    }
    let track = view(&track, fit);
    let slot = slot_over(&track, crawl_cells, fit).expect("the run crawls");
    let crawl = Crawl::found(slot, 1, &track, -1, (H, fit), &|n| track.get(n))
        .expect("the side fits the budget");
    let side = &crawl.before;
    let leaders = side.leaders.as_ref().expect("the side leaves room");
    let whole = leaders.side.rebuild(side.width);
    let own: Vec<f64> = (side.cells.0..=side.cells.1)
        .map(|n| leaders.at(n, side.cells.0, &track).rebuild(&leaders.within))
        .collect();
    let (least, most) = own
        .iter()
        .fold((f64::INFINITY, 0.0_f64), |(lo, hi), &rebuild| {
            (lo.min(rebuild), hi.max(rebuild))
        });
    assert!(
        least < most && own.len() as i64 > 2 * super::REBUILD_SCREEN_CELLS,
        "the side's cells read one rebuild, {least}..{most}, over {} cells",
        own.len()
    );
    for step in 0..=16 {
        let threshold = least + (most - least) * f64::from(step) / 16.0;
        for (offset, &rebuild) in own.iter().enumerate() {
            let n = side.cells.0 + offset as i64;
            let expected = (rebuild > threshold).then(|| rebuild.min(whole));
            assert_eq!(
                side.rebuild_where(n, &track, BOXED_ONLY, |bound| bound > threshold),
                expected,
                "cell {n} over the threshold {threshold}"
            );
        }
    }
}

/// A checked cell whose shaped motion runs slower than its move's box asks
/// the rebuild of its own leaders.
#[test]
fn a_side_cell_asks_the_rebuild_of_its_own_shaped_motion() {
    let fit = FitMiss::of(0.005, 50.0);
    let crawl_cells = (800, 810);
    let track = track_with_crawl(10.0, crawl_cells, 1200, fit);
    let track = view(&track, fit);
    let slot = slot_over(&track, crawl_cells, fit).expect("the run crawls");
    let crawl = Crawl::found(slot, 1, &track, -1, (H, fit), &|n| track.get(n))
        .expect("the side fits the budget");
    let side = &crawl.before;
    let leaders = side.leaders.as_ref().expect("the side leaves room");
    let n = crawl_cells.0 - 1;
    let shaped = (2.0, 60.0);
    let boxed = side
        .rebuild_where(n, &track, BOXED_ONLY, |_| true)
        .expect("a bound");
    let own = side
        .rebuild_where(n, &track, shaped, |_| true)
        .expect("a bound");
    let tracked = leaders.at(n, side.cells.0, &track);
    let expected = super::JoinedLeaders {
        speed: tracked.speed.min(shaped.0 + fit.speed),
        accel: tracked.accel.min(shaped.1 + fit.accel),
        ..tracked
    }
    .rebuild(&leaders.within)
    .min(leaders.side.rebuild(side.width));
    assert!(own < boxed, "{own} against the box's {boxed}");
    assert_eq!(own, expected);
}

#[test]
fn a_crawl_s_after_side_waits_for_the_lead_budget_past_its_run() {
    let fit = FitMiss::of(0.005, 50.0);
    let crawl_cells = (800, 810);
    let budget_cells = (VALLEY_LEAD_BUDGET_S / H).floor() as i64;
    let short = track_with_crawl(10.0, crawl_cells, crawl_cells.1 + budget_cells / 4, fit);
    let short = view(&short, fit);
    let slot = slot_over(&short, crawl_cells, fit).expect("the run crawls");
    let mut crawl = Crawl::found(slot, 1, &short, -1, (H, fit), &|n| short.get(n))
        .expect("the side fits the budget");
    assert!(
        !crawl.resolve_after(&short, false, (H, fit), &|n| short.get(n), None),
        "the after side settled before its budget's cells were tracked"
    );
    let long = track_with_crawl(10.0, crawl_cells, crawl_cells.1 + budget_cells + 1, fit);
    let long = view(&long, fit);
    assert!(crawl.resolve_after(&long, false, (H, fit), &|n| long.get(n), None));
    let after = crawl.after.clone().expect("the after side settled");
    assert!(after.cells.0 == crawl_cells.0 && after.cells.1 > crawl_cells.1);
    assert_eq!(crawl.reach(), (crawl.before.cells.0, after.cells.1));
}

/// The width a crawl's after side resolves at, the leaders past the run at
/// `cruise` mm/s, read through `end`, never narrower than `laid`.
fn after_width(cruise: f64, end: i64, laid: Option<f64>) -> Option<f64> {
    let fit = FitMiss::of(0.005, 50.0);
    let crawl_cells = (800, 810);
    let track = track_with_crawl(cruise, crawl_cells, end, fit);
    let track = view(&track, fit);
    let slot = slot_over(&track, crawl_cells, fit).expect("the run crawls");
    let mut crawl = Crawl::found(slot, 1, &track, -1, (H, fit), &|n| track.get(n))
        .expect("the side fits the budget");
    crawl
        .resolve_after(&track, false, (H, fit), &|n| track.get(n), laid)
        .then(|| crawl.after.expect("the after side settled").width)
}

/// A back-off past a crawl slows the leaders over its after side's budget:
/// the side stays as wide as it was laid, waiting for the path to cover it,
/// instead of narrowing and rebuilding steeper on cells proven against it.
#[test]
fn a_back_off_past_a_crawl_never_narrows_its_after_side() {
    let budget_cells = (VALLEY_LEAD_BUDGET_S / H).floor() as i64;
    let end = 810 + budget_cells + 1;
    let laid = after_width(10.0, end, None).expect("the side settles past the budget");
    let slowed = after_width(5.0, end, None).expect("the side settles past the budget");
    assert!(
        slowed < laid,
        "slower leaders lay {slowed} mm, not under {laid} mm"
    );
    assert_eq!(
        after_width(5.0, end, Some(laid)),
        None,
        "the side settled before the path covered its {laid} mm"
    );
    assert_eq!(after_width(5.0, 4 * end, Some(laid)), Some(laid));
}

/// The projection holds the weight at zero back to a rest where no before
/// side reaching no further back fits: no room is left for one.
#[test]
fn a_crawl_with_a_rest_inside_its_budget_leaves_no_room_before_it() {
    let fit = FitMiss::of(0.005, 50.0);
    let crawl_cells = (800, 810);
    let track = track_with_rest(crawl_cells, 780, fit);
    let track = view(&track, fit);
    let slot = slot_over(&track, crawl_cells, fit).expect("the run crawls");
    let crawl = Crawl::found(slot, 1, &track, -1, (H, fit), &|n| track.get(n))
        .expect("a rest bounds the side");
    assert_eq!(crawl.before, Reserve::none(crawl_cells));
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
    let track = view(&track, fit);
    let mut slot = slot_over(&track, crawl_cells, fit).expect("the run crawls");
    let past_rest =
        track.path(rest + 1, crawl_cells.0 - 1).1 + track.drift(rest + 1, crawl_cells.1, H, fit);
    let without = Crawl::found(slot, 1, &track, -1, (H, fit), &|n| track.get(n));
    assert!(
        without.is_err(),
        "a side the slow motion since the rest cannot hold fits: {without:?}"
    );
    slot.width.1 = past_rest;
    let crawl = Crawl::found(slot, 1, &track, -1, (H, fit), &|n| track.get(n))
        .expect("the rest bounds the side");
    assert_eq!(crawl.before, Reserve::none(crawl_cells));
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
        track.push(n, bounds, (H, fit), false);
    }
    track
}
