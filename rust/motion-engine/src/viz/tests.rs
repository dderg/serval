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

fn default_config(limits: geometry::VelocityLimits) -> StreamConfig {
    StreamConfig {
        chain: geometry::ChainFitConfig::default(),
        integration_tol: VELOCITY_INTEGRATION_TOL,
        max_extrude_only_velocity_mm_s: f64::INFINITY,
        max_extrude_only_accel_mm_s2: f64::INFINITY,
        fit_tol_mm: TRAJECTORY_FIT_TOL_MM,
        max_buffer_moves: SNAPSHOT_MAX_BUFFER_MOVES,
        limits,
    }
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
    let limits = default_limits();
    let moves = build_moves(&square_waypoints(), limits).unwrap();
    let (fitted, _) = run_pipeline(&moves, default_config(limits), AxisChainSet::default());
    let spatial_count = fitted
        .iter()
        .filter(|fm| fm.segment.spatial.is_some())
        .count();
    assert!(spatial_count > 0);
}

fn eval_piece(p: &[f64; 6], t: f64) -> f64 {
    let z = t - p[0];
    p[2] + p[3] * z + p[4] * z * z + p[5] * z * z * z
}

#[test]
fn trajectory_lowers_to_contiguous_finite_cubics() {
    let limits = default_limits();
    let moves = build_moves(&square_waypoints(), limits).unwrap();
    let (_, shaped) = run_pipeline(&moves, default_config(limits), AxisChainSet::default());
    let traj = collect_trajectory_pieces(&shaped);
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
    let limits = default_limits();
    let moves = build_moves(&square_waypoints(), limits).unwrap();
    let (_, shaped) = run_pipeline(&moves, default_config(limits), AxisChainSet::default());
    let traj = collect_trajectory_pieces(&shaped);
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
    let limits = default_limits();
    let waypoints = vec![(0.0, 0.0, 0.0, 100.0), (10.0, 0.0, 0.0, 100.0)];
    let moves = build_moves(&waypoints, limits).unwrap();
    let (fitted, _) = run_pipeline(&moves, default_config(limits), AxisChainSet::default());
    assert_eq!(fitted.len(), 1, "single move must pass through unchanged");
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
