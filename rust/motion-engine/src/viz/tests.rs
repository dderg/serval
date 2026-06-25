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
fn kinematics_has_samples_with_valid_heading() {
    let moves = build_moves(&square_waypoints(), default_limits()).unwrap();
    let outcome = geometry::fit_chain(&moves, geometry::ChainFitConfig::default()).unwrap();
    let profile = geometry::plan_velocity(&outcome, geometry::VelocityConfig::default()).unwrap();
    let kin = sample_kinematics(&outcome, &profile);
    assert!(!kin.s.is_empty());
    assert_eq!(kin.a_t.len(), kin.s.len());
    for i in 0..kin.s.len() {
        assert!(kin.s[i] >= 0.0);
        assert!(kin.v[i] >= 0.0);
        assert!(kin.a_t[i].is_finite());
        let h_len = (kin.heading_x[i].powi(2) + kin.heading_y[i].powi(2)).sqrt();
        assert!((h_len - 1.0).abs() < 1e-6, "heading not unit: {h_len}");
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
