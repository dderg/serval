use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;

use super::{
    AdvanceValley, CrawlSide, SideWeight, ValleyKind, coupled_bend, crawl_floor, crawl_side_width,
    kept_floor, turns_around, valley_reach, widest_crawl_side_within,
};

const GOLDEN_RATIO: f64 = 1.618_033_988_749_895;

/// A range around `value` reaching `below` and `above` of it, never under
/// `least`.
fn around(value: f64, below: f64, above: f64, least: f64) -> (f64, f64) {
    ((value - below).max(least), value + above)
}

fn contains((lo, hi): (f64, f64), value: f64, what: &str) -> Result<(), TestCaseError> {
    let slack = 1e-9 * (1.0 + value.abs());
    prop_assert!(
        lo - slack <= value && value <= hi + slack,
        "{what} {value} lies outside [{lo}, {hi}]"
    );
    Ok(())
}

fn crawl_valley(width: f64, floor: f64) -> AdvanceValley {
    AdvanceValley {
        start: 10.0,
        end: 10.0,
        before: width,
        after: width,
        floor_before: floor,
        floor_after: floor,
        kind: ValleyKind::Crawl,
    }
}

/// The leaders leaving a crawl at `speed`, their path acceleration held
/// within `accel` in `pieces` stretches of `durations` at `shares` of it,
/// never past `fastest` nor back through rest; the most
/// `|W″·ṡ³ + 3W′·ṡ·s̈|` they meet on the valley's after side, and the
/// range of the path acceleration they leave under while on it further
/// than `signed_from` from the crawl.
fn realized_rebuild(
    valley: &AdvanceValley,
    (speed, accel, fastest): (f64, f64, f64),
    pieces: &[(f64, f64)],
    signed_from: f64,
) -> (f64, (f64, f64)) {
    let steps_per_piece = 400;
    let (mut s, mut v) = (0.0_f64, speed);
    let mut most = 0.0_f64;
    let mut outward = (f64::INFINITY, f64::NEG_INFINITY);
    for &(share, duration) in pieces {
        let dt = duration / f64::from(steps_per_piece);
        for _ in 0..steps_per_piece {
            let rate = (share * accel).clamp(-v / dt, (fastest - v) / dt);
            let (weight, slope, curvature) = valley.weight(valley.end + s);
            if weight < 1.0 {
                most = most.max((curvature * v * v * v + 3.0 * slope * v * rate).abs());
                if s + v * dt + 0.5 * rate * dt * dt > signed_from {
                    outward = (outward.0.min(rate), outward.1.max(rate));
                }
            }
            s += v * dt + 0.5 * rate * dt * dt;
            v = (v + rate * dt).clamp(0.0, fastest);
        }
    }
    (most, outward)
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 4096,
        failure_persistence: Some(Box::new(FileFailurePersistence::Direct(
            "proptest-regressions/valley_side_weight.txt",
        ))),
        ..ProptestConfig::default()
    })]

    /// Wherever within the ranges a crawl valley lies, its weight, slope
    /// and curvature at the point stay within the side's bounds: the
    /// curvature as `bend_scale·bend − fold` over the ranges' corners.
    #[test]
    fn a_valley_side_bounds_its_weight_wherever_in_the_ranges_the_valley_lies(
        speed in 0.0f64..2.0,
        rate in 1.0f64..50_000.0,
        share in 0.0f64..1.2,
        spreads in prop::array::uniform6(0.0f64..0.3),
        after in any::<bool>(),
    ) {
        let width = valley_reach(speed, rate);
        let floor = crawl_floor(speed, rate);
        let valley = crawl_valley(width, floor);
        let distance = share * width;
        let s = if after { 10.0 + distance } else { 10.0 - distance };
        let (weight, slope, curvature) = valley.weight(s);
        let bounds = SideWeight::within(
            around(distance, spreads[0] * width, spreads[1] * width, 0.0),
            around(width, spreads[2] * width, spreads[3] * width, 1e-3 * width),
            around(floor, spreads[4] * floor, spreads[5] * floor, 0.0),
        );
        contains(bounds.weight, weight, "the weight")?;
        contains(bounds.slope, slope.abs(), "the slope")?;
        let curvature_bounds = (
            bounds.bend_scale.0 * bounds.bend.0 - bounds.fold.1,
            bounds.bend_scale.1 * bounds.bend.1 - bounds.fold.0,
        );
        contains(curvature_bounds, curvature, "the curvature")?;
    }

    /// `σ̇³·β` stays within [`coupled_bend`] wherever the speed builds from
    /// the crawl under the side's acceleration, the floor the crawl's own
    /// or a wider one within the range.
    #[test]
    fn the_coupled_bend_bound_holds_wherever_the_speed_builds_from_the_crawl(
        accel in 1.0f64..60_000.0,
        rate_share in 1e-3f64..1.0,
        speed in 0.0f64..2.0,
        width in 1e-4f64..20.0,
        log_distance in -8.0f64..6.0,
        floor_widening in 1.0f64..50.0,
    ) {
        let floor = crawl_floor(speed, rate_share * accel);
        prop_assume!(floor > 0.0);
        let range = (floor, floor * floor_widening);
        let kept = kept_floor(floor * floor_widening, width);
        let distance = kept * libm::exp(log_distance);
        let speed_sq = speed * speed + 2.0 * accel * distance;
        let spread_sq = distance * distance + kept * kept;
        let asked = speed_sq * speed_sq.sqrt() * kept * kept / (spread_sq * spread_sq.sqrt());
        let bound = coupled_bend(speed, accel, range.1, width);
        prop_assert!(
            asked <= bound * (1.0 + 1e-9),
            "σ̇³β {asked} past the coupled bound {bound}"
        );
    }

    /// The leaders leaving a crawl however they please within the side's
    /// speed and acceleration never meet more rebuild than
    /// [`CrawlSide::rebuild`] bounds, at the width the rule lays or wider:
    /// read either way, or within the range of the path acceleration they
    /// leave under past a distance from the crawl.
    #[test]
    fn a_crawl_side_bounds_the_rebuild_the_leaders_meet_over_it(
        speed in 0.0f64..2.0,
        accel in 1.0f64..20_000.0,
        rate_share in 1e-3f64..1.0,
        fastest_over in 0.0f64..200.0,
        widening in 1.0f64..40.0,
        pieces in prop::collection::vec((-1.0f64..1.0, 1e-4f64..0.02), 1..6),
        signed_share in prop_oneof![Just(0.0), 0.0f64..1.2],
    ) {
        let rate = rate_share * accel;
        let floor = crawl_floor(speed, rate);
        let width = valley_reach(speed, rate) * widening;
        let fastest = speed + fastest_over;
        let signed_from = signed_share * width;
        let either_way = CrawlSide::either_way(speed, accel, fastest, (floor, floor));
        let (realized, outward) = realized_rebuild(
            &crawl_valley(width, floor),
            (speed, accel, fastest),
            &pieces,
            signed_from,
        );
        let bound = either_way.rebuild(width);
        prop_assert!(
            realized <= bound * (1.0 + 1e-6),
            "the leaders meet a rebuild of {realized}, past its bound {bound}"
        );
        let outward = if outward.0 <= outward.1 { outward } else { (0.0, 0.0) };
        let signed = CrawlSide { outward, signed_from, ..either_way }.rebuild(width);
        prop_assert!(
            realized <= signed * (1.0 + 1e-6) + 1e-9 * bound,
            "the leaders leaving under {outward:?} past {signed_from} mm meet a rebuild of \
             {realized}, past its bound {signed}"
        );
        prop_assert!(signed <= bound * (1.0 + 1e-12), "the signed bound {signed} is looser than {bound}");
    }

    /// Read where they are, the leaders leaving a crawl however they
    /// please within the side's speed and acceleration meet no more rebuild
    /// than [`CrawlSide::rebuild_within`] bounds at their own distance,
    /// speed and acceleration, and no more `W′·ṡ²` than
    /// [`RebuildsWithin::lead_velocity_at`] bounds there and
    /// [`CrawlSide::lead_velocity`] over the whole side.
    #[test]
    fn the_rebuild_read_where_the_leaders_are_bounds_what_they_meet_there(
        speed in 0.0f64..2.0,
        accel in 1.0f64..20_000.0,
        rate_share in 1e-3f64..1.0,
        fastest_over in 0.0f64..200.0,
        widening in 1.0f64..40.0,
        pieces in prop::collection::vec((-1.0f64..1.0, 1e-4f64..0.02), 1..6),
        spreads in prop::array::uniform2(0.0f64..0.3),
    ) {
        let rate = rate_share * accel;
        let floor = crawl_floor(speed, rate);
        let width = valley_reach(speed, rate) * widening;
        let fastest = speed + fastest_over;
        let side = CrawlSide::either_way(speed, accel, fastest, (floor, floor));
        let within = side.rebuilds_within(width);
        let whole_velocity = side.lead_velocity(width);
        let valley = crawl_valley(width, floor);
        let steps_per_piece = 400;
        let (mut s, mut v) = (0.0_f64, speed);
        for &(share, duration) in &pieces {
            let dt = duration / f64::from(steps_per_piece);
            for step in 0..steps_per_piece {
                let rate = (share * accel).clamp(-v / dt, (fastest - v) / dt);
                let (weight, slope, curvature) = valley.weight(valley.end + s);
                if weight < 1.0 && step % 10 == 0 {
                    let met = (curvature * v * v * v + 3.0 * slope * v * rate).abs();
                    let distance = around(s, spreads[0] * width, spreads[1] * width, 0.0);
                    let bound = within.at(distance, v, rate.abs());
                    prop_assert!(
                        met <= bound * (1.0 + 1e-6) + 1e-9,
                        "at {s} mm, {v} mm/s and {rate} mm/s² the leaders meet a rebuild of \
                         {met}, past its bound {bound} within {distance:?}"
                    );
                    let lead = slope.abs() * v * v;
                    let lead_bound = within.lead_velocity_at(distance, v);
                    prop_assert!(
                        lead <= lead_bound * (1.0 + 1e-6) + 1e-9,
                        "at {s} mm and {v} mm/s the leaders meet a lead velocity of {lead}, past \
                         its bound {lead_bound} within {distance:?}"
                    );
                    prop_assert!(
                        lead <= whole_velocity * (1.0 + 1e-6) + 1e-9,
                        "at {s} mm and {v} mm/s the leaders meet a lead velocity of {lead}, past \
                         the side's bound {whole_velocity}"
                    );
                }
                s += v * dt + 0.5 * rate * dt * dt;
                v = (v + rate * dt).clamp(0.0, fastest);
            }
        }
    }

    /// The width the rule lays fits the room, and a side read within
    /// looser bounds never lays a narrower one.
    #[test]
    fn a_crawl_side_is_laid_where_its_rebuild_fits_and_looser_bounds_never_narrow_it(
        speed in 0.0f64..2.0,
        accel in 1.0f64..20_000.0,
        rate_share in 1e-3f64..1.0,
        fastest_over in 0.0f64..200.0,
        room in 10.0f64..1e6,
        loosen in prop::array::uniform4(0.0f64..1.0),
        limit_share in 1.0f64..200.0,
        outward_shares in (-1.0f64..1.0, -1.0f64..1.0),
    ) {
        let rate = rate_share * accel;
        let floor = crawl_floor(speed, rate);
        let reach = valley_reach(speed, rate);
        let tight = CrawlSide {
            speed,
            accel,
            fastest: speed + fastest_over,
            floor: (floor, floor),
            outward: (
                accel * outward_shares.0.min(outward_shares.1),
                accel * outward_shares.0.max(outward_shares.1),
            ),
            signed_from: loosen[3] * reach,
        };
        let loose = CrawlSide::either_way(
            speed + loosen[0] * (2.0 - speed),
            accel * (1.0 + loosen[1]),
            tight.fastest * (1.0 + loosen[2]) + loosen[0] * 2.0,
            (if turns_around(speed, rate) { 0.0 } else { floor * (1.0 - loosen[3]) }, f64::INFINITY),
        );
        let loose_reach = valley_reach(loose.speed, loose.accel).max(reach);
        let width = crawl_side_width(reach, |width| tight.rebuild(width) <= room);
        if let Some(width) = width {
            prop_assert!(tight.rebuild(width) <= room, "laid {width} where its rebuild does not fit");
        }
        if let Some(grid) = widest_crawl_side_within(loose_reach, loose_reach * limit_share) {
            if loose.rebuild(grid) <= room {
                let width = width.expect("a side fitting under looser bounds fits under tighter ones");
                prop_assert!(width <= grid, "laid {width}, past the {grid} a looser read fits at");
            }
        }
    }
}

#[test]
fn the_coupled_bend_bound_is_the_golden_ratio_to_the_three_halves_where_the_floor_bends() {
    let (accel, floor) = (4000.0, 3.8e-4);
    let bound = coupled_bend(1.9, accel, floor, 0.18);
    let expected = libm::pow(GOLDEN_RATIO, 1.5) * libm::pow(accel, 1.5) * floor.sqrt();
    assert!(
        (bound - expected).abs() <= 1e-9 * expected,
        "the coupled bound {bound} is not φ^1.5·A^1.5·√f = {expected}"
    );
    let slower = coupled_bend(1.0, accel, floor, 0.18);
    assert!(
        slower < bound,
        "a crawl slower than its floor allows bends {slower}, no less than {bound}"
    );
}

/// A crawl whose path acceleration all but vanishes has a floor `v²/a` of
/// kilometres; its side keeps the floor within a share of its width, and
/// the weight leaves zero past it, so the valley holds zero no further out.
#[test]
fn a_valley_holds_zero_only_within_the_floor_its_side_keeps() {
    let width = 0.0114;
    let valley = crawl_valley(width, crawl_floor(0.76, 3.7e-10));
    let kept = kept_floor(valley.floor_after, width);
    assert!(valley.holds_zero_at(valley.end + kept, 0.0));
    assert!(!valley.holds_zero_at(valley.end + 2.0 * kept, 0.0));
    assert!(!valley.holds_zero_at(valley.start - 2.0 * kept, 0.0));
    assert!(
        valley.weight(valley.end + 2.0 * kept).0 > 0.0,
        "the weight leaves zero past the kept floor"
    );
    assert!(!valley.holds_zero_at(valley.end + 18.8, 0.0));
}

#[test]
fn a_crawl_side_wider_than_its_reach_lies_on_the_grid() {
    let reach = 0.05;
    let width = crawl_side_width(reach, |width| width >= 0.3).expect("a wide enough side fits");
    let widest =
        widest_crawl_side_within(reach, width).expect("the grid has widths past the reach");
    assert_eq!(widest, width, "the laid width is a grid width");
    assert!(width >= 0.3 && width < 0.3 * libm::pow(2.0, 1.0 / 8.0));
}
