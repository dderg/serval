use super::*;
use motion_pipeline::StreamConfig;

fn square_waypoints() -> Vec<waypoints::Waypoint> {
    vec![
        (0.0, 0.0, 0.0, 0.0, 100.0, 3000.0),
        (10.0, 0.0, 0.0, 0.0, 100.0, 3000.0),
        (10.0, 10.0, 0.0, 0.0, 100.0, 3000.0),
        (0.0, 10.0, 0.0, 0.0, 100.0, 3000.0),
        (0.0, 0.0, 0.0, 0.0, 100.0, 3000.0),
    ]
}

fn default_limits() -> geometry::VelocityLimits {
    geometry::VelocityLimits::try_new(
        300.0,
        3000.0,
        geometry::corner_deviation_from_scv(5.0, 3000.0),
        100_000.0,
    )
    .unwrap()
}

fn default_config(limits: geometry::VelocityLimits) -> StreamConfig {
    StreamConfig {
        corner: geometry::CornerFitConfig::default(),
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
    let (fitted, _, _) = run_pipeline(&moves, default_config(limits), AxisChainSet::default());
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
    let (_, shaped, _) = run_pipeline(&moves, default_config(limits), AxisChainSet::default());
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
    let (_, shaped, _) = run_pipeline(&moves, default_config(limits), AxisChainSet::default());
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
    let waypoints = vec![
        (0.0, 0.0, 0.0, 0.0, 100.0, 3000.0),
        (10.0, 0.0, 0.0, 0.0, 100.0, 3000.0),
    ];
    let moves = build_moves(&waypoints, limits).unwrap();
    let (fitted, _, _) = run_pipeline(&moves, default_config(limits), AxisChainSet::default());
    assert_eq!(fitted.len(), 1, "single move must pass through unchanged");
}

#[test]
fn zero_displacement_filtered() {
    let waypoints = vec![
        (0.0, 0.0, 0.0, 0.0, 100.0, 3000.0),
        (0.0, 0.0, 0.0, 0.0, 100.0, 3000.0),
        (10.0, 0.0, 0.0, 0.0, 100.0, 3000.0),
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
    let waypoints = vec![
        (0.0, 0.0, 0.0, 0.0, 100.0, 3000.0),
        (10.0, 0.0, 0.0, 2.0, 100.0, 3000.0),
    ];
    let moves = build_moves(&waypoints, default_limits()).unwrap();
    assert_eq!(moves.len(), 1);
    let ratio = follower_ratio(&moves[0], EXTRUDER_AXIS).expect("extruder follower present");
    assert!((ratio - 0.2).abs() < 1e-12, "ΔE/Δs should be 2/10 = 0.2");
}

#[test]
fn diagonal_move_extrusion_ratio_uses_spatial_distance() {
    // 3-4-5 triangle: Δs = 5, ΔE = 1, ratio = 0.2.
    let waypoints = vec![
        (0.0, 0.0, 0.0, 0.0, 100.0, 3000.0),
        (3.0, 4.0, 0.0, 1.0, 100.0, 3000.0),
    ];
    let moves = build_moves(&waypoints, default_limits()).unwrap();
    let ratio = follower_ratio(&moves[0], EXTRUDER_AXIS).expect("extruder follower present");
    assert!((ratio - 0.2).abs() < 1e-12, "ΔE/Δs should be 1/5 = 0.2");
}

#[test]
fn pure_retract_is_a_non_spatial_follower_move() {
    // E-only retract: no XYZ change, ΔE = -3. It must survive as a virtual
    // (non-spatial) move carrying only the extruder follower.
    let waypoints = vec![
        (0.0, 0.0, 0.0, 0.0, 100.0, 3000.0),
        (0.0, 0.0, 0.0, -3.0, 100.0, 3000.0),
    ];
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
    let waypoints = vec![
        (0.0, 0.0, 0.0, 0.0, 100.0, 3000.0),
        (10.0, 0.0, 0.0, 0.0, 100.0, 3000.0),
    ];
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
        (0.0, 0.0, 0.0, 0.0, 100.0, 3000.0),
        (10.0, 0.0, 0.0, 1.0, 100.0, 3000.0),
        (10.0, 10.0, 0.0, 2.0, 100.0, 3000.0),
    ];
    let moves = build_moves(&waypoints, limits).unwrap();
    let (_, shaped, _) = run_pipeline(&moves, default_config(limits), AxisChainSet::default());
    let traj = collect_trajectory_pieces(&shaped);
    assert!(!traj.e.is_empty(), "E lane must lower to cubic pieces");
    // Every seg contributes to the E track: the E pieces tile the exact same
    // time span as X, gap-free. (Piece *counts* may differ between axes — the
    // fit bisects each axis independently.)
    let (x0, x1) = (traj.x.first().unwrap()[0], traj.x.last().unwrap()[1]);
    let (e0, e1) = (traj.e.first().unwrap()[0], traj.e.last().unwrap()[1]);
    assert!(
        (e0 - x0).abs() < 1e-9 && (e1 - x1).abs() < 1e-9,
        "E track spans the whole trajectory: x=[{x0},{x1}] e=[{e0},{e1}]"
    );
    for w in traj.e.windows(2) {
        assert!(
            (w[1][0] - w[0][1]).abs() < 1e-9,
            "E pieces are gap-free at t={}",
            w[0][1]
        );
    }
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
    let (_, shaped, _) = run_pipeline(&moves, default_config(limits), AxisChainSet::default());
    let traj = collect_trajectory_pieces(&shaped);
    let m = seam_metrics(&traj);
    // C1 Hermite lowering matches position and velocity at every join.
    // Velocity joins carry up to a cap-landing snap kick (the ride pass's
    // CONTACT_SNAP_REL band, 1e-5 relative): a corner blend whose curvature
    // ramp outruns the jerk budget lands on the blend ceiling with the
    // band-sized residual the anchored landing could not absorb.
    for axis in 0..4 {
        assert!(m.max_dp[axis] < 1e-6, "axis {axis} position jump");
        assert!(m.max_dv[axis] < 1e-5, "axis {axis} velocity jump");
    }
}

#[test]
fn snapshot_serializes_to_the_baseline_schema() {
    let snap = pipeline_snapshot(
        &square_waypoints(),
        SnapshotParams {
            max_velocity: 300.0,
            max_accel: 3000.0,
            square_corner_velocity: 5.0,
            corner_deviation: None,
            max_jerk: 100_000.0,
            max_extrude_only_velocity: None,
            max_extrude_only_accel: None,
            max_path_deviation: None,
            max_accel_deviation: None,
            axis_decls: Vec::new(),
            post_processor_decls: Vec::new(),
        },
    )
    .unwrap();
    let json: serde_json::Value = serde_json::to_value(&snap).unwrap();
    for key in [
        "raw_x",
        "raw_y",
        "traj_x_pieces",
        "traj_y_pieces",
        "traj_z_pieces",
        "traj_e_pieces",
        "traj_t_end",
        "traversal_time_s",
        "seam_max_dp",
        "seam_max_dv",
        "seam_max_da",
        "worst_seams",
    ] {
        assert!(json.get(key).is_some(), "missing snapshot key {key}");
    }
}

fn default_axis_snapshot_params() -> SnapshotParams {
    SnapshotParams {
        max_velocity: 300.0,
        max_accel: 3000.0,
        square_corner_velocity: 5.0,
        corner_deviation: None,
        max_jerk: 100_000.0,
        max_extrude_only_velocity: None,
        max_extrude_only_accel: None,
        max_path_deviation: None,
        max_accel_deviation: None,
        axis_decls: Vec::new(),
        post_processor_decls: Vec::new(),
    }
}

fn axis(name: &str, follows: &[&str]) -> planner_config::AxisDecl {
    planner_config::AxisDecl {
        name: name.into(),
        follows: follows.iter().map(|s| s.to_string()).collect(),
        motors: Vec::new(),
        post_processors: Vec::new(),
    }
}

fn pp(name: &str, ty: &str, params: &[(&str, f64)]) -> planner_config::PostProcessorDecl {
    planner_config::PostProcessorDecl {
        name: name.into(),
        ty: ty.into(),
        params: params.iter().map(|(k, v)| ((*k).to_string(), *v)).collect(),
    }
}

#[test]
fn empty_axis_decls_falls_back_to_the_default_four_axis_topology() {
    let snap = pipeline_snapshot(&square_waypoints(), default_axis_snapshot_params()).unwrap();
    assert!(!snap.traj_x_pieces.is_empty());
    assert!(!snap.traj_e_pieces.is_empty());
}

#[test]
fn declaring_only_the_e_axis_still_defaults_x_y_z() {
    // A caller shouldn't have to boilerplate x/y/z just to attach a
    // post-processor to e — merge_axis_decls fills in the other three from
    // the default topology.
    let mut params = default_axis_snapshot_params();
    let mut e = axis("e", &["x", "y", "z"]);
    e.post_processors = vec!["pa".to_string()];
    params.axis_decls = vec![e];
    params.post_processor_decls = vec![pp("pa", "linear_pressure_advance", &[("k", 0.04)])];
    let snap = pipeline_snapshot(&square_waypoints(), params).unwrap();
    assert!(!snap.traj_x_pieces.is_empty());
}

#[test]
fn all_post_processor_types_are_reachable() {
    // Kernels attach to a spatial axis; a bare derivative-gain stage is only
    // legal off the leader axes (pre-kernel gains on leaders are rejected),
    // so linear_pressure_advance rides the e follower, its natural home.
    for (ty, params, axis_name) in [
        ("smooth_bell", [("smooth_time", 0.0200625)].as_slice(), "x"),
        ("smooth_triangle", [("smooth_time", 0.02)].as_slice(), "x"),
        ("smooth_zv", [("frequency_hz", 40.0)].as_slice(), "x"),
        ("smooth_mzv", [("frequency_hz", 40.0)].as_slice(), "x"),
        ("linear_pressure_advance", [("k", 0.04)].as_slice(), "e"),
    ] {
        let mut params_snap = default_axis_snapshot_params();
        let mut carrier = match axis_name {
            "x" => axis("x", &[]),
            _ => axis("e", &["x", "y", "z"]),
        };
        carrier.post_processors = vec!["pp".to_string()];
        params_snap.axis_decls = vec![carrier];
        params_snap.post_processor_decls = vec![pp("pp", ty, params)];
        pipeline_snapshot(&square_waypoints(), params_snap)
            .unwrap_or_else(|e| panic!("post-processor type '{ty}' should compile: {e}"));
    }
}

#[test]
fn mode_inverse_is_reachable_after_a_kernel() {
    pipeline_snapshot(&square_waypoints(), mode_inverse_on_x_params())
        .unwrap_or_else(|e| panic!("mode_inverse after a kernel should compile: {e}"));
}

fn mode_inverse_on_x_params() -> SnapshotParams {
    let mut params = default_axis_snapshot_params();
    let mut x = axis("x", &[]);
    x.post_processors = vec!["slew".to_string(), "belt".to_string()];
    params.axis_decls = vec![x];
    params.post_processor_decls = vec![
        pp("slew", "smooth_bell", &[("smooth_time", 0.0015)]),
        pp(
            "belt",
            "mode_inverse",
            &[("frequency_hz", 40.0), ("damping_ratio", 0.1)],
        ),
    ];
    params
}

fn eval_lane(pieces: &[Vec<f64>], t: f64) -> f64 {
    let idx = pieces
        .partition_point(|p| p[0] <= t)
        .saturating_sub(1)
        .min(pieces.len() - 1);
    eval_piece(&pieces[idx], t.clamp(pieces[idx][0], pieces[idx][1]))
}

fn max_lane_difference(a: &[Vec<f64>], b: &[Vec<f64>], t_end: f64) -> f64 {
    (0..=1000)
        .map(|i| t_end * i as f64 / 1000.0)
        .map(|t| (eval_lane(a, t) - eval_lane(b, t)).abs())
        .fold(0.0, f64::max)
}

#[test]
fn mode_inverse_emits_a_toolhead_signal_distinct_from_the_motor_command() {
    let snap = pipeline_snapshot(&square_waypoints(), mode_inverse_on_x_params()).unwrap();
    let toolhead_x = snap.toolhead_x_pieces.as_ref().expect("toolhead x lane");
    let toolhead_y = snap.toolhead_y_pieces.as_ref().expect("toolhead y lane");
    assert!(!toolhead_x.is_empty());
    assert!(
        max_lane_difference(toolhead_x, &snap.traj_x_pieces, snap.traj_t_end) > 1e-2,
        "x carries motor-side gains, so its motor command must depart from the toolhead signal"
    );
    assert!(
        max_lane_difference(toolhead_y, &snap.traj_y_pieces, snap.traj_t_end) < 1e-6,
        "y has no motor-side stage, so both signals coincide"
    );
    let json: serde_json::Value = serde_json::to_value(&snap).unwrap();
    for key in [
        "toolhead_x_pieces",
        "toolhead_y_pieces",
        "toolhead_z_pieces",
        "toolhead_e_pieces",
    ] {
        assert!(json.get(key).is_some(), "missing snapshot key {key}");
    }
}

#[test]
fn kernel_only_chain_serializes_without_toolhead_lanes() {
    let mut params = default_axis_snapshot_params();
    let mut x = axis("x", &[]);
    x.post_processors = vec!["slew".to_string()];
    params.axis_decls = vec![x];
    params.post_processor_decls = vec![pp("slew", "smooth_bell", &[("smooth_time", 0.0015)])];
    let snap = pipeline_snapshot(&square_waypoints(), params).unwrap();
    assert!(snap.toolhead_x_pieces.is_none());
    let json: serde_json::Value = serde_json::to_value(&snap).unwrap();
    for key in [
        "toolhead_x_pieces",
        "toolhead_y_pieces",
        "toolhead_z_pieces",
        "toolhead_e_pieces",
    ] {
        assert!(
            json.get(key).is_none(),
            "kernel-only snapshot must serialize without {key}"
        );
    }
}

#[test]
fn mode_inverse_without_a_kernel_surfaces_as_an_error() {
    let mut params = default_axis_snapshot_params();
    let mut x = axis("x", &[]);
    x.post_processors = vec!["belt".to_string()];
    params.axis_decls = vec![x];
    params.post_processor_decls = vec![pp(
        "belt",
        "mode_inverse",
        &[("frequency_hz", 40.0), ("damping_ratio", 0.1)],
    )];
    let err = pipeline_snapshot(&square_waypoints(), params).unwrap_err();
    assert!(
        matches!(&err, SnapshotError::InvalidChain(msg) if msg.contains("smoothing kernel")),
        "got: {err}"
    );
}

#[test]
fn composition_conflict_surfaces_as_an_error() {
    let mut params = default_axis_snapshot_params();
    let mut x = axis("x", &[]);
    x.post_processors = vec!["a".to_string(), "b".to_string()];
    params.axis_decls = vec![x];
    params.post_processor_decls = vec![
        pp("a", "smooth_bell", &[("smooth_time", 0.0200625)]),
        pp("b", "smooth_triangle", &[("smooth_time", 0.02)]),
    ];
    let err = pipeline_snapshot(&square_waypoints(), params).unwrap_err();
    assert!(matches!(err, SnapshotError::InvalidChain(_)));
}

#[test]
fn undeclared_axis_reference_surfaces_as_an_error() {
    let mut params = default_axis_snapshot_params();
    let mut x = axis("x", &[]);
    x.post_processors = vec!["nope".to_string()];
    params.axis_decls = vec![x];
    let err = pipeline_snapshot(&square_waypoints(), params).unwrap_err();
    assert!(matches!(err, SnapshotError::InvalidChain(_)));
}

#[test]
fn unknown_axis_name_is_rejected() {
    let mut params = default_axis_snapshot_params();
    params.axis_decls = vec![axis("a", &[])];
    let err = pipeline_snapshot(&square_waypoints(), params).unwrap_err();
    assert!(matches!(err, SnapshotError::InvalidChain(msg) if msg.contains("'a'")));
}

#[test]
fn streaming_final_snapshot_serializes_byte_identical_to_full() {
    let full = pipeline_snapshot(&square_waypoints(), default_axis_snapshot_params()).unwrap();
    let mut partial_count = 0;
    let streamed = pipeline_snapshot_streaming(
        &square_waypoints(),
        default_axis_snapshot_params(),
        2,
        |_| partial_count += 1,
    )
    .unwrap();
    assert!(partial_count > 0);
    assert_eq!(
        serde_json::to_string(&full).unwrap(),
        serde_json::to_string(&streamed).unwrap()
    );
}

#[test]
fn streaming_partials_are_growing_prefixes_with_the_full_raw_path() {
    let mut partials: Vec<(Vec<f64>, Vec<Vec<f64>>)> = Vec::new();
    let streamed = pipeline_snapshot_streaming(
        &square_waypoints(),
        default_axis_snapshot_params(),
        2,
        |snap| partials.push((snap.raw_x.clone(), snap.traj_x_pieces.clone())),
    )
    .unwrap();
    assert!(!partials.is_empty());
    let mut prev_len = 0;
    for (raw_x, x_pieces) in &partials {
        assert_eq!(*raw_x, streamed.raw_x);
        assert!(x_pieces.len() >= prev_len);
        assert!(x_pieces.len() <= streamed.traj_x_pieces.len());
        assert_eq!(*x_pieces, streamed.traj_x_pieces[..x_pieces.len()]);
        prev_len = x_pieces.len();
    }
}

#[test]
fn streaming_partials_carry_toolhead_lanes_when_the_chain_has_motor_side_stages() {
    let mut saw_toolhead_partial = false;
    let streamed =
        pipeline_snapshot_streaming(&square_waypoints(), mode_inverse_on_x_params(), 2, |snap| {
            assert!(snap.toolhead_x_pieces.is_some());
            saw_toolhead_partial = true;
        })
        .unwrap();
    assert!(saw_toolhead_partial);
    assert!(streamed.toolhead_x_pieces.is_some());
}
