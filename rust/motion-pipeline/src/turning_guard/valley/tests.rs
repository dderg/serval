use geometry::{AdvanceValley, SideWeight};

use super::super::test_motion::Rng;
use super::{Factor, coupled_bend, weigh};

const GOLDEN_RATIO: f64 = 1.618_033_988_749_895;

#[test]
fn the_coupled_bend_bound_holds_wherever_the_speed_builds_from_the_crawl() {
    let mut rng = Rng::new(11);
    let mut tightest = 0.0_f64;
    for _ in 0..200_000 {
        let strongest = rng.between(1.0, 60_000.0);
        let rate = strongest * rng.unit();
        let crawl_speed = 2.0 * rng.unit();
        let width = rng.between(1e-4, 2.0);
        let floor = if rate == 0.0 {
            f64::INFINITY
        } else {
            crawl_speed * crawl_speed / rate
        };
        let leaving_floor = floor.min(width);
        if leaving_floor == 0.0 {
            continue;
        }
        let distance = leaving_floor * libm::exp(rng.between(-8.0, 6.0));
        let speed_sq = crawl_speed * crawl_speed + 2.0 * strongest * distance;
        let spread_sq = distance * distance + leaving_floor * leaving_floor;
        let bend = leaving_floor * leaving_floor / (spread_sq * spread_sq.sqrt());
        let asked = speed_sq * speed_sq.sqrt() * bend;
        let bound = coupled_bend(strongest, leaving_floor, crawl_speed, width, floor > width);
        assert!(
            asked <= bound * (1.0 + 1e-12),
            "σ̇³·β = {asked} past the bound {bound} at A={strongest}, a={rate}, v={crawl_speed}, \
             w={width}, d={distance}"
        );
        tightest = tightest.max(asked / bound);
    }
    assert!(
        tightest > 0.99,
        "the coupled bend bound is never within 1% of what it bounds: {tightest}"
    );
}

#[test]
fn the_coupled_bend_bound_is_the_golden_ratio_to_the_three_halves_where_the_floor_bends() {
    let (strongest, floor) = (4000.0, 3.8e-3);
    let bound = coupled_bend(strongest, floor, 1.9, 0.18, false);
    let expected = GOLDEN_RATIO.powf(1.5) * strongest.powf(1.5) * floor.sqrt();
    assert!(
        (bound - expected).abs() <= 1e-12 * expected,
        "{bound} against φ^1.5·A^1.5·√f = {expected}"
    );
    assert!(
        (GOLDEN_RATIO.powf(1.5) - 2.058).abs() < 1e-3,
        "φ^1.5 = {}",
        GOLDEN_RATIO.powf(1.5)
    );
}

#[test]
fn the_weighed_product_bounds_every_subset_of_laid_valleys() {
    let mut rng = Rng::new(5);
    for _ in 0..20_000 {
        let count = 1 + (rng.unit() * 4.0) as usize;
        let s = rng.between(-1.0, 1.0);
        let speed = rng.between(0.0, 60.0);
        let valleys: Vec<AdvanceValley> = (0..count)
            .map(|_| {
                let centre = rng.between(-0.3, 0.3);
                let width = rng.between(0.02, 0.6);
                AdvanceValley {
                    start: centre,
                    end: centre,
                    before: width,
                    after: width,
                    floor: rng.between(0.0, 0.01),
                }
            })
            .collect();
        let factors: Vec<Factor> = valleys
            .iter()
            .map(|valley| {
                let distance = (s - valley.start).abs();
                let side = SideWeight::within(
                    (distance, distance),
                    (valley.before, valley.before),
                    (valley.floor, valley.floor),
                );
                Factor {
                    weight: side.weight,
                    rebuild: side.slope.1 * speed,
                    fold: side.fold.1 * speed * speed * speed,
                    bend: side.bend_scale.1 * side.bend.1 * speed * speed * speed,
                }
            })
            .collect();
        let read = weigh(&factors, speed);
        for laid in 1..(1_u32 << count) {
            let (w, w1, w2) = valleys
                .iter()
                .enumerate()
                .filter(|(j, _)| laid & (1 << j) != 0)
                .fold((1.0, 0.0, 0.0), |(w, w1, w2), (_, valley)| {
                    let (v, v1, v2) = valley.weight(s);
                    (w * v, w1 * v + w * v1, w2 * v + 2.0 * w1 * v1 + w * v2)
                });
            let slack = 1e-9 * (1.0 + read.rebuild + read.fold + read.bend);
            let cubed = speed * speed * speed;
            assert!(
                w >= read.weight_floor - 1e-12
                    && (w1 * speed).abs() <= read.rebuild + slack
                    && w2 * cubed <= read.bend + slack
                    && -w2 * cubed <= read.fold + slack,
                "the laid valleys {laid:b} of {valleys:?} weigh ({w}, {w1}, {w2}) at s={s}, \
                 speed {speed}, outside {read:?}"
            );
        }
    }
}
