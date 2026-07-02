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
    geometry::VelocityLimits::try_new(300.0, 3000.0, 5.0, 100_000.0).unwrap()
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

fn eval_piece(p: &[f64; 6], t: f64) -> f64 {
    let z = t - p[0];
    p[2] + p[3] * z + p[4] * z * z + p[5] * z * z * z
}

#[test]
fn trajectory_lowers_to_contiguous_finite_cubics() {
    let moves = build_moves(&square_waypoints(), default_limits()).unwrap();
    let outcome = geometry::fit_chain(&moves, geometry::ChainFitConfig::default()).unwrap();
    let profile = geometry::plan_velocity(&outcome, 1e-7, f64::INFINITY, f64::INFINITY).unwrap();
    let traj = lower_trajectory(&outcome, &profile);
    assert!(!traj.x.is_empty());
    assert_eq!(traj.x.len(), traj.y.len());
    for (i, p) in traj.x.iter().enumerate() {
        assert!(p.iter().all(|c| c.is_finite()));
        assert!(p[1] > p[0], "piece must span a positive time interval");
        if i + 1 < traj.x.len() {
            assert!(
                (traj.x[i + 1][0] - p[1]).abs() < 1e-9,
                "pieces must be contiguous in time"
            );
        }
    }
    assert!((traj.x.last().unwrap()[1] - traj.t_end).abs() < 1e-9);
}

#[test]
fn cubic_pieces_are_position_continuous_at_joins() {
    let moves = build_moves(&square_waypoints(), default_limits()).unwrap();
    let outcome = geometry::fit_chain(&moves, geometry::ChainFitConfig::default()).unwrap();
    let profile = geometry::plan_velocity(&outcome, 1e-7, f64::INFINITY, f64::INFINITY).unwrap();
    let traj = lower_trajectory(&outcome, &profile);
    // Hermite lowering matches position at every join, on both axes.
    for axis in [&traj.x, &traj.y] {
        for w in axis.windows(2) {
            let end = eval_piece(&w[0], w[0][1]);
            let start = eval_piece(&w[1], w[1][0]);
            assert!(
                (end - start).abs() < 1e-6,
                "position jump at piece join: {end} vs {start}"
            );
        }
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
