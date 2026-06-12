use super::*;
use crate::Limits;
use crate::topp::path::sample_arclength_grid;

#[test]
fn single_segment_chain_mirrors_arclength_grid() {
    let c = tests_support::line([0.0; 3], [50.0, 0.0, 0.0]);
    let g = sample_arclength_grid(&c, 11).unwrap();
    let chain = ChainGrid::from_segment_grids(vec![g.clone()], vec![lim(300.0)]);
    assert_eq!(chain.s, g.s);
    assert_eq!(chain.h_intervals.len(), 10);
    assert!(chain.junctions.is_empty());
    assert_eq!(chain.segment_ranges, vec![(0, 10)]);
    assert_eq!(chain.geom[3].c_prime, g.c_prime[3]);
    assert!(chain.limits_idx.iter().all(|&i| i == 0));
}

#[test]
fn two_segment_chain_shares_junction_point() {
    let a = tests_support::line([0.0; 3], [40.0, 0.0, 0.0]);
    let b = tests_support::line([40.0, 0.0, 0.0], [100.0, 0.0, 0.0]);
    let ga = sample_arclength_grid(&a, 11).unwrap(); // h = 4
    let gb = sample_arclength_grid(&b, 13).unwrap(); // h = 5
    let chain = ChainGrid::from_segment_grids(vec![ga, gb], vec![lim(300.0), lim(200.0)]);

    assert_eq!(chain.s.len(), 23); // 11 + 13 − 1 shared point
    assert_eq!(chain.h_intervals.len(), 22);
    assert!((chain.s[10] - 40.0).abs() < 1e-9);
    assert!((chain.s[22] - 100.0).abs() < 1e-9);
    assert!((chain.h_intervals[9] - 4.0).abs() < 1e-9);
    assert!((chain.h_intervals[10] - 5.0).abs() < 1e-9);
    assert_eq!(chain.junctions.len(), 1);
    assert_eq!(chain.junctions[0].idx, 10);
    assert_eq!(chain.junctions[0].limits_idx, 1);
    assert_eq!(chain.limits_idx[10], 0); // primary arrays carry the LEFT side
    assert_eq!(chain.segment_ranges, vec![(0, 10), (10, 22)]);
}

#[test]
#[should_panic(expected = "junction spacing ratio")]
fn extreme_spacing_ratio_panics() {
    let a = tests_support::line([0.0; 3], [40.0, 0.0, 0.0]);
    let b = tests_support::line([40.0, 0.0, 0.0], [80.0, 0.0, 0.0]);
    let ga = sample_arclength_grid(&a, 3).unwrap(); // h = 20
    let gb = sample_arclength_grid(&b, 41).unwrap(); // h = 1 → ratio 20 > 16
    let _ = ChainGrid::from_segment_grids(vec![ga, gb], vec![lim(300.0), lim(300.0)]);
}

fn lim(v: f64) -> Limits {
    Limits::axis_boxes([v; 3], [5_000.0; 3], [100_000.0; 3])
}

#[test]
fn chain_carries_per_segment_follower_demands() {
    let a = tests_support::line([0.0; 3], [40.0, 0.0, 0.0]);
    let b = tests_support::line([40.0, 0.0, 0.0], [80.0, 0.0, 0.0]);
    let ga = sample_arclength_grid(&a, 11).unwrap();
    let gb = sample_arclength_grid(&b, 11).unwrap();
    let demand = crate::FollowerDemand {
        axis: 3,
        ratio: 0.05,
        pa_k: 0.0,
    };
    let chain = ChainGrid::try_from_segment_grids_with_followers(
        vec![ga, gb],
        vec![lim_e(300.0), lim_e(300.0)],
        vec![vec![demand], vec![]],
        &[false, false],
    )
    .unwrap();
    let (s0, e0) = chain.segment_ranges[0];
    let (s1, e1) = chain.segment_ranges[1];
    for i in s0..e0 {
        assert_eq!(chain.followers_at(i), &[demand]);
    }
    for i in (e0.max(s1) + 1)..=e1 {
        assert!(chain.followers_at(i).is_empty());
    }
}

#[test]
fn follower_demand_validation_fails_loudly() {
    let a = tests_support::line([0.0; 3], [40.0, 0.0, 0.0]);
    let ga = sample_arclength_grid(&a, 11).unwrap();
    let bad = |d: crate::FollowerDemand| {
        ChainGrid::try_from_segment_grids_with_followers(
            vec![ga.clone()],
            vec![lim_e(300.0)],
            vec![vec![d]],
            &[false],
        )
        .unwrap_err()
    };
    let base = crate::FollowerDemand {
        axis: 3,
        ratio: 0.05,
        pa_k: 0.0,
    };
    bad(crate::FollowerDemand { axis: 1, ..base });
    bad(crate::FollowerDemand { axis: 4, ..base });
    bad(crate::FollowerDemand {
        ratio: f64::NAN,
        ..base
    });
    bad(crate::FollowerDemand { ratio: 0.0, ..base });
    bad(crate::FollowerDemand { pa_k: -1.0, ..base });
}

fn lim_e(v: f64) -> Limits {
    let mut sets: Vec<crate::LimitSet> = Limits::axis_boxes([v; 3], [5_000.0; 3], [100_000.0; 3])
        .sets()
        .to_vec();
    sets.push(crate::LimitSet {
        axes: crate::AxisSet::from_indices(&[3]),
        v_max: 75.0,
        a_max: 1500.0,
        j_max: 3000.0,
    });
    Limits::try_new(&sets, 4).unwrap()
}
