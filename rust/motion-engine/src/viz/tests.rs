use super::*;

fn square_waypoints() -> Vec<(f64, f64, f64, f64, f64)> {
    vec![
        (0.0, 0.0, 0.0, 0.0, 100.0),
        (10.0, 0.0, 0.0, 0.0, 100.0),
        (10.0, 10.0, 0.0, 0.0, 100.0),
        (0.0, 10.0, 0.0, 0.0, 100.0),
        (0.0, 0.0, 0.0, 0.0, 100.0),
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
        fit_tol_accel_mm_s2: 50.0,
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

fn eval_piece(p: &[f64], t: f64) -> f64 {
    let z = t - p[0];
    p[2..].iter().rev().fold(0.0, |acc, &c| acc * z + c)
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
    let waypoints = vec![(0.0, 0.0, 0.0, 0.0, 100.0), (10.0, 0.0, 0.0, 0.0, 100.0)];
    let moves = build_moves(&waypoints, limits).unwrap();
    let (fitted, _) = run_pipeline(&moves, default_config(limits), AxisChainSet::default());
    assert_eq!(fitted.len(), 1, "single move must pass through unchanged");
}

#[test]
fn zero_displacement_filtered() {
    let waypoints = vec![
        (0.0, 0.0, 0.0, 0.0, 100.0),
        (0.0, 0.0, 0.0, 0.0, 100.0),
        (10.0, 0.0, 0.0, 0.0, 100.0),
    ];
    let moves = build_moves(&waypoints, default_limits()).unwrap();
    assert_eq!(moves.len(), 1);
}

fn follower_ratio(m: &geometry::Move, axis: usize) -> Option<f64> {
    m.segment
        .followers
        .iter()
        .find(|f| f.axis_index == axis)
        .map(|f| f.ratio)
}

#[test]
fn spatial_move_with_extrusion_sets_follower_ratio() {
    // 10 mm move in X, extruding 2 mm of filament: the E follower ratio is
    // ΔE / Δs on the real extruder axis (index 3).
    let waypoints = vec![(0.0, 0.0, 0.0, 0.0, 100.0), (10.0, 0.0, 0.0, 2.0, 100.0)];
    let moves = build_moves(&waypoints, default_limits()).unwrap();
    assert_eq!(moves.len(), 1);
    let ratio = follower_ratio(&moves[0], EXTRUDER_AXIS).expect("extruder follower present");
    assert!((ratio - 0.2).abs() < 1e-12, "ΔE/Δs should be 2/10 = 0.2");
}

#[test]
fn diagonal_move_extrusion_ratio_uses_spatial_distance() {
    // 3-4-5 triangle: Δs = 5, ΔE = 1, ratio = 0.2.
    let waypoints = vec![(0.0, 0.0, 0.0, 0.0, 100.0), (3.0, 4.0, 0.0, 1.0, 100.0)];
    let moves = build_moves(&waypoints, default_limits()).unwrap();
    let ratio = follower_ratio(&moves[0], EXTRUDER_AXIS).expect("extruder follower present");
    assert!((ratio - 0.2).abs() < 1e-12, "ΔE/Δs should be 1/5 = 0.2");
}

#[test]
fn pure_retract_is_a_non_spatial_follower_move() {
    // E-only retract: no XYZ change, ΔE = -3. It must survive as a virtual
    // (non-spatial) move carrying only the extruder follower.
    let waypoints = vec![(0.0, 0.0, 0.0, 0.0, 100.0), (0.0, 0.0, 0.0, -3.0, 100.0)];
    let moves = build_moves(&waypoints, default_limits()).unwrap();
    assert_eq!(
        moves.len(),
        1,
        "retract must not be filtered as zero motion"
    );
    assert!(
        moves[0].segment.spatial.is_none(),
        "retract has no spatial geometry"
    );
    let ratio = follower_ratio(&moves[0], EXTRUDER_AXIS).expect("extruder follower present");
    assert!(
        (ratio + 1.0).abs() < 1e-12,
        "unit-length virtual path: ratio = -1"
    );
}

#[test]
fn no_extrusion_means_no_follower() {
    let waypoints = vec![(0.0, 0.0, 0.0, 0.0, 100.0), (10.0, 0.0, 0.0, 0.0, 100.0)];
    let moves = build_moves(&waypoints, default_limits()).unwrap();
    assert!(
        follower_ratio(&moves[0], EXTRUDER_AXIS).is_none(),
        "a travel move carries no extruder follower"
    );
}

#[test]
fn extrusion_lowers_to_a_moving_e_track() {
    let limits = default_limits();
    // Extrude the whole way around the square so every axis[3] track moves.
    let waypoints = vec![
        (0.0, 0.0, 0.0, 0.0, 100.0),
        (10.0, 0.0, 0.0, 1.0, 100.0),
        (10.0, 10.0, 0.0, 2.0, 100.0),
    ];
    let moves = build_moves(&waypoints, limits).unwrap();
    let (_, shaped) = run_pipeline(&moves, default_config(limits), AxisChainSet::default());
    let traj = collect_trajectory_pieces(&shaped);
    assert!(!traj.e.is_empty(), "E lane must lower to cubic pieces");
    assert_eq!(
        traj.e.len(),
        traj.x.len(),
        "every seg contributes an E piece"
    );
    let e_start = eval_piece(traj.e.first().unwrap(), traj.e.first().unwrap()[0]);
    let e_end = eval_piece(traj.e.last().unwrap(), traj.e.last().unwrap()[1]);
    assert!(
        (e_end - e_start - 2.0).abs() < 1e-6,
        "E advances by the total 2 mm extruded"
    );
}

#[test]
fn seam_metrics_flag_a_known_discontinuity() {
    // Two X pieces meeting at t=1: left ends at pos=1,vel=0,acc=0; right starts
    // at pos=5 (Δp=4), vel=3 (Δv=3), acc=2*2=4 (Δa=4).
    let traj = TrajectoryPieces {
        x: vec![
            vec![0.0, 1.0, 1.0, 0.0, 0.0, 0.0],
            vec![1.0, 2.0, 5.0, 3.0, 2.0, 0.0],
        ],
        y: Vec::new(),
        z: Vec::new(),
        e: Vec::new(),
        t_end: 2.0,
    };
    let m = seam_metrics(&traj);
    assert!((m.max_dp[0] - 4.0).abs() < 1e-12);
    assert!((m.max_dv[0] - 3.0).abs() < 1e-12);
    assert!((m.max_da[0] - 4.0).abs() < 1e-12);
    let worst = m.worst.first().expect("one interior seam recorded");
    assert_eq!(worst.axis, 0);
    assert!((worst.t - 1.0).abs() < 1e-12);
    assert!((worst.dp - 4.0).abs() < 1e-12);
}

#[test]
fn continuous_pieces_report_no_seam_jumps() {
    let limits = default_limits();
    let moves = build_moves(&square_waypoints(), limits).unwrap();
    let (_, shaped) = run_pipeline(&moves, default_config(limits), AxisChainSet::default());
    let traj = collect_trajectory_pieces(&shaped);
    let m = seam_metrics(&traj);
    // C1 Hermite lowering matches position and velocity at every join.
    for axis in 0..4 {
        assert!(m.max_dp[axis] < 1e-6, "axis {axis} position jump");
        assert!(m.max_dv[axis] < 1e-6, "axis {axis} velocity jump");
    }
}
