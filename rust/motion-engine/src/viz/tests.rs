use super::*;

fn square_waypoints() -> Vec<(f64, f64, f64, f64)> {
    vec![
        (0.0, 0.0, 0.0, 100.0),
        (10.0, 0.0, 0.0, 100.0),
        (10.0, 10.0, 0.0, 100.0),
        (0.0, 10.0, 0.0, 100.0),
        (0.0, 0.0, 0.0, 100.0),
    ]
}

fn default_limits() -> geometry::VelocityLimits {
    geometry::VelocityLimits::try_new(300.0, 3000.0, 5.0).unwrap()
}

#[test]
fn build_moves_from_square() {
    let moves = build_moves(&square_waypoints(), default_limits()).unwrap();
    assert_eq!(moves.len(), 4);
}

#[test]
fn raw_path_has_correct_point_count() {
    let moves = build_moves(&square_waypoints(), default_limits()).unwrap();
    let points = extract_raw_path(&moves);
    assert_eq!(points.len(), 5);
}

#[test]
fn raw_path_starts_at_origin() {
    let moves = build_moves(&square_waypoints(), default_limits()).unwrap();
    let points = extract_raw_path(&moves);
    assert!((points[0].0).abs() < 1e-9);
    assert!((points[0].1).abs() < 1e-9);
}

#[test]
fn fitted_outcome_has_spatial_segments() {
    let moves = build_moves(&square_waypoints(), default_limits()).unwrap();
    let outcome = geometry::fit_chain(&moves, geometry::ChainFitConfig::default()).unwrap();
    let spatial_count = outcome
        .moves
        .iter()
        .filter(|m| m.segment.spatial.is_some())
        .count();
    assert!(spatial_count > 0);
}

#[test]
fn kinematics_emits_finite_position_and_nonnegative_speed() {
    let moves = build_moves(&square_waypoints(), default_limits()).unwrap();
    let outcome = geometry::fit_chain(&moves, geometry::ChainFitConfig::default()).unwrap();
    let profile = geometry::plan_velocity(&outcome, geometry::VelocityConfig::default()).unwrap();
    let kin = sample_kinematics(&outcome, &profile);
    assert!(!kin.x.is_empty());
    assert_eq!(kin.x.len(), kin.y.len());
    assert_eq!(kin.x.len(), kin.v.len());
    for i in 0..kin.x.len() {
        assert!(kin.x[i].is_finite());
        assert!(kin.y[i].is_finite());
        assert!(kin.v[i] >= 0.0);
    }
}

#[test]
fn curved_corner_is_sampled_densely_enough_to_recover_curvature() {
    let moves = build_moves(&square_waypoints(), default_limits()).unwrap();
    let outcome = geometry::fit_chain(&moves, geometry::ChainFitConfig::default()).unwrap();
    let profile = geometry::plan_velocity(&outcome, geometry::VelocityConfig::default()).unwrap();
    let kin = sample_kinematics(&outcome, &profile);
    // The viz recovers centripetal acceleration by differentiating position, so
    // a turn must show up as many interior samples where the path direction
    // rotates (successive position deltas not parallel), not a single jump.
    let mut turning = 0;
    for i in 1..kin.x.len() - 1 {
        let (ax, ay) = (kin.x[i] - kin.x[i - 1], kin.y[i] - kin.y[i - 1]);
        let (bx, by) = (kin.x[i + 1] - kin.x[i], kin.y[i + 1] - kin.y[i]);
        let cross = ax * by - ay * bx;
        if cross.abs() > 1e-9 {
            turning += 1;
        }
    }
    assert!(
        turning > 8,
        "corner collapsed to {turning} turning samples; centripetal \
         acceleration cannot be reconstructed from position"
    );
}

#[test]
fn single_move_skips_fitting() {
    let waypoints = vec![(0.0, 0.0, 0.0, 100.0), (10.0, 0.0, 0.0, 100.0)];
    let moves = build_moves(&waypoints, default_limits()).unwrap();
    let outcome = geometry::fit_chain(&moves, geometry::ChainFitConfig::default()).unwrap();
    assert_eq!(outcome.report.blended, 0);
}

#[test]
fn zero_displacement_filtered() {
    let waypoints = vec![
        (0.0, 0.0, 0.0, 100.0),
        (0.0, 0.0, 0.0, 100.0),
        (10.0, 0.0, 0.0, 100.0),
    ];
    let moves = build_moves(&waypoints, default_limits()).unwrap();
    assert_eq!(moves.len(), 1);
}
