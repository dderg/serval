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
fn fitted_path_has_more_points_than_raw() {
    let moves = build_moves(&square_waypoints(), default_limits()).unwrap();
    let raw_count = extract_raw_path(&moves).len();

    let outcome = geometry::fit_chain(&moves, geometry::ChainFitConfig::default()).unwrap();
    let fitted_count = sample_fitted_path(&outcome).len();
    assert!(fitted_count > raw_count);
}

#[test]
fn velocity_profile_has_samples() {
    let moves = build_moves(&square_waypoints(), default_limits()).unwrap();
    let outcome = geometry::fit_chain(&moves, geometry::ChainFitConfig::default()).unwrap();
    let profile = geometry::plan_velocity(&outcome, geometry::VelocityConfig::default()).unwrap();
    let samples = extract_velocity_profile(&profile);
    assert!(!samples.is_empty());
    for &(s, v) in &samples {
        assert!(s >= 0.0);
        assert!(v >= 0.0);
    }
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
