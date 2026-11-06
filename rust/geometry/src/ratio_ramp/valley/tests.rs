use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;

use super::{AdvanceValley, SideWeight, ValleyKind, crawl_floor, valley_reach};

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
        let valley = AdvanceValley {
            start: 10.0,
            end: 10.0,
            before: width,
            after: width,
            floor,
            kind: ValleyKind::Crawl,
        };
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
}
