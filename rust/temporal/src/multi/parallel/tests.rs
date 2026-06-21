use super::*;
use crate::Limits;
use crate::multi::joining::ChainState;
use crate::topp::chain::ChainGrid;
use crate::topp::path::sample_arclength_grid;
use nurbs::VectorNurbs;

fn straight() -> VectorNurbs<f64, 3> {
    VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [50.0, 0.0, 0.0]],
    )
    .unwrap()
}

fn limits() -> Limits {
    Limits::axis_boxes([500.0; 3], [5_000.0; 3], [100_000.0; 3])
}

fn straight_chain() -> ChainGrid {
    let grid = sample_arclength_grid(&straight(), 20).unwrap();
    ChainGrid::from_segment_grids(vec![grid], vec![limits()])
}

fn short_straight_chain() -> ChainGrid {
    const TOO_SHORT_FOR_500MMS_DECEL_MM: f64 = 1.0;
    let curve = VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [TOO_SHORT_FOR_500MMS_DECEL_MM, 0.0, 0.0]],
    )
    .unwrap();
    let grid = sample_arclength_grid(&curve, 20).unwrap();
    ChainGrid::from_segment_grids(vec![grid], vec![limits()])
}

#[test]
fn fan_out_processes_all_dirty() {
    let chains: Vec<ChainGrid> = (0..4).map(|_| straight_chain()).collect();
    let mut states: Vec<ChainState> = (0..4)
        .map(|_| ChainState {
            v_start: 0.0,
            v_end: 0.0,
            a_start: None,
            profile: None,
            dirty: true,
        })
        .collect();
    fan_out_solves(&chains, &mut states, 3).unwrap();
    for s in &states {
        assert!(s.profile.is_some());
        assert!(!s.dirty);
    }
}

#[test]
fn pinned_both_endpoints_returns_failed_status_unmodified() {
    let k = (4.0 / 3.0) * (std::f64::consts::SQRT_2 - 1.0);
    let r = 1.0_f64;
    let curved = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [r * k, 0.0, 0.0],
            [r, r * (1.0 - k), 0.0],
            [r, r, 0.0],
        ],
    )
    .unwrap();
    let curved_limits = limits();
    let grid = sample_arclength_grid(&curved, 20).unwrap();
    let chain = ChainGrid::from_segment_grids(vec![grid], vec![curved_limits]);

    let profile = solve_with_boundary_fallback(&chain, 100.0, 0.0, None, true, true)
        .expect("must not return ScheduleError");
    assert!(
        !is_success(profile.status),
        "with both endpoints pinned and an infeasible problem the fallback \
         must return a non-success status, got {:?}",
        profile.status,
    );
}

#[test]
fn pinned_v_start_not_overwritten_by_infeasible_chain0_solve() {
    const PINNED_V_START: f64 = 500.0;
    const PINNED_A_START: f64 = 50.0;

    let chain = short_straight_chain();
    let mut states = vec![ChainState {
        v_start: PINNED_V_START,
        v_end: 0.0,
        a_start: Some(PINNED_A_START),
        profile: None,
        dirty: true,
    }];

    fan_out_solves(&[chain.clone()], &mut states, 1)
        .expect("fan_out_solves must not return BatchError on infeasible profile");

    assert_eq!(
        states[0].v_start, PINNED_V_START,
        "chain 0's pinned v_start must be unchanged after an infeasible solve; \
         got {} instead of {}",
        states[0].v_start, PINNED_V_START,
    );

    fan_out_solves(&[chain], &mut states, 1)
        .expect("second fan_out_solves must not return InvalidEndpointAccel after the first");
}
