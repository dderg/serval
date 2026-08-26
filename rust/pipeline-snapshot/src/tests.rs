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
        f64::INFINITY,
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

fn exact(shaped: &[ContinuousSegment]) -> ExactTrajectory {
    ExactTrajectory::from_segments(shaped).expect("shaped segments carry evaluable exact carriers")
}

const MIN_CARRIER_DURATION_S: f64 = 2e-9;

fn at(traj: &ExactTrajectory, axis: usize, t: f64, side: SampleSide) -> Pvaj {
    traj.eval_axis(axis, t, side)
        .unwrap_or_else(|e| panic!("axis {axis} at t={t} is not evaluable: {e}"))
}

fn position(traj: &ExactTrajectory, axis: usize, t: f64) -> f64 {
    at(traj, axis, t, SampleSide::Right).position
}

fn assert_axis_tiles_the_run(traj: &ExactTrajectory, axis: usize, lane: &str) {
    let rows = traj.rows(axis);
    assert!(!rows.is_empty(), "{lane}: lane must carry rows");
    for (i, row) in rows.iter().enumerate() {
        assert!(
            row.t1 > row.t0,
            "{lane}[{i}]: carrier must span positive time"
        );
        assert!(
            rows.len() == 1 || row.t1 - row.t0 >= MIN_CARRIER_DURATION_S,
            "{lane}[{i}]: carrier spans {:e}s, under device resolution",
            row.t1 - row.t0
        );
        for side in [SampleSide::Left, SampleSide::Right] {
            let state = at(traj, axis, 0.5 * (row.t0 + row.t1), side);
            assert!(
                state.position.is_finite()
                    && state.velocity.is_finite()
                    && state.acceleration.is_finite()
                    && state.jerk.is_finite(),
                "{lane}[{i}]: non-finite carrier state {state:?}"
            );
        }
        if let Some(next) = rows.get(i + 1) {
            assert!(
                (next.t0 - row.t1).abs() < 1e-9,
                "{lane}[{i}]: carriers must be contiguous in time"
            );
        }
    }
}

#[test]
fn trajectory_carriers_tile_the_run_on_every_spatial_axis() {
    let limits = default_limits();
    let moves = build_moves(&square_waypoints(), limits).unwrap();
    let (_, shaped, _) = run_pipeline(&moves, default_config(limits), AxisChainSet::default());
    let traj = exact(&shaped);
    assert_axis_tiles_the_run(&traj, 0, "x");
    assert_axis_tiles_the_run(&traj, 1, "y");
    for (lane, axis) in [("x", 0), ("y", 1)] {
        let rows = traj.rows(axis);
        assert!(
            (rows.first().unwrap().t0 - traj.t_start()).abs() < 1e-9,
            "{lane}: lanes must start together"
        );
        assert!(
            (rows.last().unwrap().t1 - traj.t_end()).abs() < 1e-9,
            "{lane}: lane must run to t_end"
        );
    }
}

#[test]
fn carriers_are_position_continuous_across_their_shared_instant() {
    let limits = default_limits();
    let moves = build_moves(&square_waypoints(), limits).unwrap();
    let (_, shaped, _) = run_pipeline(&moves, default_config(limits), AxisChainSet::default());
    let traj = exact(&shaped);
    for axis in [0, 1] {
        for w in traj.rows(axis).windows(2) {
            let end = at(&traj, axis, w[0].t1, SampleSide::Left).position;
            let start = at(&traj, axis, w[1].t0, SampleSide::Right).position;
            assert!(
                (end - start).abs() < 1e-6,
                "position jump at carrier join: {end} vs {start}"
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
    let traj = exact(&shaped);
    let e_rows = traj.rows(3);
    assert!(!e_rows.is_empty(), "E lane must carry rows");
    // Every seg contributes to the E track: the E carriers tile the exact same
    // time span as X, gap-free. (Carrier *counts* may differ between axes.)
    let x_rows = traj.rows(0);
    let (x0, x1) = (x_rows.first().unwrap().t0, x_rows.last().unwrap().t1);
    let (e0, e1) = (e_rows.first().unwrap().t0, e_rows.last().unwrap().t1);
    assert!(
        (e0 - x0).abs() < 1e-9 && (e1 - x1).abs() < 1e-9,
        "E track spans the whole trajectory: x=[{x0},{x1}] e=[{e0},{e1}]"
    );
    for w in e_rows.windows(2) {
        assert!(
            (w[1].t0 - w[0].t1).abs() < 1e-9,
            "E carriers are gap-free at t={}",
            w[0].t1
        );
    }
    let e_start = position(&traj, 3, e0);
    let e_end = at(&traj, 3, e1, SampleSide::Left).position;
    let n = 20_000;
    let mut path_len = 0.0;
    let mut prev = (position(&traj, 0, x0), position(&traj, 1, x0));
    for i in 1..=n {
        let t = x0 + (x1 - x0) * f64::from(i) / f64::from(n);
        let cur = (position(&traj, 0, t), position(&traj, 1, t));
        path_len += libm::hypot(cur.0 - prev.0, cur.1 - prev.1);
        prev = cur;
    }
    assert!(
        (e_end - e_start - 0.1 * path_len).abs() < 1e-4,
        "E advances at the commanded 0.1 rate over the actual path: \
         {} vs 0.1 × {path_len}",
        e_end - e_start
    );
}

/// One X-axis carrier per `(t0, t1, coefficients-in-local-time)` row, each the
/// Bézier spline that *is* that polynomial: the power-to-Bernstein change of
/// basis is exact, so the carrier commands precisely the named coefficients
/// rather than a fit of them.
pub(crate) fn x_polynomial_trajectory(rows: &[(f64, f64, Vec<f64>)]) -> ExactTrajectory {
    let mut curves = Vec::new();
    let mut lane = Vec::new();
    for (t0, t1, coeffs) in rows {
        let degree = coeffs.len() - 1;
        let mut knots = vec![*t0; degree + 1];
        knots.extend(std::iter::repeat_n(*t1, degree + 1));
        curves.push(SplineCurve {
            degree: degree as u8,
            knots,
            control_points: bernstein_control_points(coeffs, t1 - t0),
        });
        lane.push(CarrierRow {
            t0: *t0,
            t1: *t1,
            carrier: Carrier::Spline {
                curve: curves.len() - 1,
            },
        });
    }
    ExactTrajectory {
        spans: Vec::new(),
        curves,
        axes: [lane, Vec::new(), Vec::new(), Vec::new()],
        t_start: rows.first().map_or(0.0, |row| row.0),
        t_end: rows.iter().fold(0.0_f64, |acc, row| acc.max(row.1)),
        runtime: Default::default(),
    }
}

pub(crate) fn bernstein_control_points(coeffs: &[f64], h: f64) -> Vec<f64> {
    let binomial =
        |n: usize, k: usize| (0..k).fold(1.0, |acc, i| acc * (n - i) as f64 / (i + 1) as f64);
    let degree = coeffs.len() - 1;
    let scaled: Vec<f64> = coeffs
        .iter()
        .enumerate()
        .map(|(k, c)| c * h.powi(k as i32))
        .collect();
    (0..=degree)
        .map(|j| {
            (0..=j)
                .map(|k| binomial(j, k) / binomial(degree, k) * scaled[k])
                .sum()
        })
        .collect()
}

#[test]
fn seam_metrics_flag_a_known_discontinuity() {
    // Two X carriers meeting at t=1: left ends at pos=1,vel=0,acc=0; right
    // starts at pos=5 (Δp=4), vel=3 (Δv=3), acc=2*2=4 (Δa=4).
    let traj = x_polynomial_trajectory(&[(0.0, 1.0, vec![1.0]), (1.0, 2.0, vec![5.0, 3.0, 2.0])]);
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
fn continuous_carriers_report_no_seam_jumps() {
    let limits = default_limits();
    let moves = build_moves(&square_waypoints(), limits).unwrap();
    let (_, shaped, _) = run_pipeline(&moves, default_config(limits), AxisChainSet::default());
    let m = seam_metrics(&exact(&shaped));
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
            max_jerk: f64::INFINITY,
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
        "schema_version",
        "raw_x",
        "raw_y",
        "trajectory",
        "traversal_time_s",
        "seam_max_dp",
        "seam_max_dv",
        "seam_max_da",
        "worst_seams",
    ] {
        assert!(json.get(key).is_some(), "missing snapshot key {key}");
    }
    assert_eq!(json["schema_version"], SNAPSHOT_SCHEMA_VERSION);
    let traj = &json["trajectory"];
    for key in ["spans", "curves", "axes", "t_end"] {
        assert!(traj.get(key).is_some(), "missing trajectory key {key}");
    }
    let axes = traj["axes"].as_array().expect("four axis lanes");
    assert_eq!(axes.len(), 4);
    for row in axes[0].as_array().expect("x lane rows") {
        for key in ["t0", "t1", "carrier"] {
            assert!(row.get(key).is_some(), "missing carrier row key {key}");
        }
        assert!(
            row["carrier"]["kind"].is_string(),
            "every carrier names its kind: {row}"
        );
    }
}

fn square_snapshot() -> Snapshot {
    pipeline_snapshot(&square_waypoints(), default_axis_snapshot_params()).unwrap()
}

/// The carriers a snapshot serializes are the ones the pipeline handed it:
/// analytic rows point at a span in the shared table, spline rows at a curve,
/// and the tables are deduplicated — the x and y lanes of a spatial move name
/// the same span index instead of each carrying a copy.
#[test]
fn serialized_carriers_reference_the_deduplicated_shared_tables() {
    let snap = square_snapshot();
    let json: serde_json::Value = serde_json::to_value(&snap).unwrap();
    let traj = &json["trajectory"];
    let span_count = traj["spans"].as_array().unwrap().len();
    let curve_count = traj["curves"].as_array().unwrap().len();
    let span_refs = |axis: usize| -> Vec<u64> {
        traj["axes"][axis]
            .as_array()
            .unwrap()
            .iter()
            .filter(|row| row["carrier"]["kind"] == "analytic")
            .map(|row| row["carrier"]["span"].as_u64().expect("span index"))
            .collect()
    };
    let x_spans = span_refs(0);
    assert!(!x_spans.is_empty(), "the square is carried analytically");
    for axis in 0..4 {
        for row in traj["axes"][axis].as_array().unwrap() {
            let carrier = &row["carrier"];
            match carrier["kind"].as_str().expect("carrier kind") {
                "analytic" => {
                    assert!((carrier["span"].as_u64().unwrap() as usize) < span_count);
                    assert!((carrier["axis"].as_u64().unwrap() as usize) < 4);
                }
                "spline" | "relative_spline" => {
                    assert!((carrier["curve"].as_u64().unwrap() as usize) < curve_count);
                }
                _ => {}
            }
        }
    }
    let y_spans = span_refs(1);
    assert!(
        x_spans.iter().any(|span| y_spans.contains(span)),
        "x and y of one move must share a span row, not duplicate it"
    );
}

fn path_curvature(traj: &ExactTrajectory, t: f64) -> Option<f64> {
    let x = at(traj, 0, t, SampleSide::Right);
    let y = at(traj, 1, t, SampleSide::Right);
    let speed_squared = x.velocity * x.velocity + y.velocity * y.velocity;
    if speed_squared < 1e-6 {
        return None;
    }
    Some(
        (x.velocity * y.acceleration - y.velocity * x.acceleration).abs()
            / libm::pow(speed_squared, 1.5),
    )
}

/// A corner blend is an arc or a clothoid, and the snapshot keeps it as one:
/// the span carries the geometry verbatim, so evaluating the carrier through
/// the blend reports the curvature that geometry implies — constant `1/r` on an
/// arc, sweeping between the clothoid's own endpoint curvatures. A polynomial
/// refit of the same window would answer for neither.
#[test]
fn curved_analytic_carriers_keep_their_geometry() {
    let snap = square_snapshot();
    let json: serde_json::Value = serde_json::to_value(&snap).unwrap();
    let traj = &json["trajectory"];
    let spans = traj["spans"].as_array().unwrap();
    let kinds: Vec<&str> = spans
        .iter()
        .filter_map(|span| span["spatial"]["kind"].as_str())
        .collect();
    assert!(
        kinds.contains(&"arc") || kinds.contains(&"clothoid"),
        "the square's corners fit to curved geometry, got {kinds:?}"
    );
    let mut checked_rows = 0;
    for row in traj["axes"][0].as_array().unwrap() {
        if row["carrier"]["kind"] != "analytic" {
            continue;
        }
        let spatial = &spans[row["carrier"]["span"].as_u64().unwrap() as usize]["spatial"];
        let (lo, hi) = match spatial["kind"].as_str() {
            Some("arc") => {
                let kappa = 1.0 / spatial["radius"].as_f64().unwrap();
                (kappa, kappa)
            }
            Some("clothoid") => {
                let kappa_0 = spatial["kappa_0"].as_f64().unwrap();
                let kappa_1 = kappa_0
                    + spatial["sigma"].as_f64().unwrap() * spatial["length"].as_f64().unwrap();
                let (lo, hi) = (kappa_0.abs(), kappa_1.abs());
                (lo.min(hi), lo.max(hi))
            }
            _ => continue,
        };
        let (t0, t1) = (row["t0"].as_f64().unwrap(), row["t1"].as_f64().unwrap());
        for i in 1..8 {
            let t = t0 + (t1 - t0) * f64::from(i) / 8.0;
            let Some(kappa) = path_curvature(&snap.trajectory, t) else {
                continue;
            };
            let slack = 1e-6 * (1.0 + hi);
            assert!(
                kappa >= lo - slack && kappa <= hi + slack,
                "curved carrier at t={t} reports curvature {kappa}, geometry allows [{lo}, {hi}]"
            );
            checked_rows += 1;
        }
    }
    assert!(
        checked_rows > 0,
        "no curved analytic carrier was reachable on the x lane"
    );
}

#[test]
fn clothoid_disk_ride_tracks_scalar_acceleration_without_phase_steps() {
    let waypoints = vec![
        (0.0, 0.0, 0.0, 0.0, 300.0, 1000.0),
        (10.0, 0.0, 0.0, 0.0, 300.0, 1000.0),
        (10.0, 10.0, 0.0, 0.0, 300.0, 1000.0),
    ];
    let mut params = default_axis_snapshot_params();
    params.max_velocity = 300.0;
    params.max_accel = 1000.0;
    params.square_corner_velocity = 5.0;
    let snap = pipeline_snapshot(&waypoints, params).unwrap();
    let clothoids: Vec<(f64, f64)> = snap
        .trajectory
        .spans
        .iter()
        .filter_map(|span| {
            matches!(span.spatial, Some(Spatial::Clothoid { .. }))
                .then_some((span.t_start, span.t_end))
        })
        .collect();
    assert_eq!(clothoids.len(), 2);

    let mut minimum = f64::INFINITY;
    let mut maximum = 0.0_f64;
    for &(t0, t1) in &clothoids {
        for i in 0..=256 {
            let t = t0 + (t1 - t0) * f64::from(i) / 256.0;
            let side = if i == 256 {
                SampleSide::Left
            } else {
                SampleSide::Right
            };
            let x = snap.trajectory.eval_axis(0, t, side).unwrap();
            let y = snap.trajectory.eval_axis(1, t, side).unwrap();
            let scalar = libm::hypot(x.acceleration, y.acceleration);
            minimum = minimum.min(scalar);
            maximum = maximum.max(scalar);
        }
    }
    assert!(
        minimum >= 988.5 && maximum <= 1001.5 && maximum - minimum <= 13.0,
        "scalar acceleration left the 1000 mm/s² disk: [{minimum}, {maximum}]"
    );

    for t in snap.trajectory.breakpoints() {
        if !clothoids.iter().any(|&(t0, t1)| t > t0 && t < t1) {
            continue;
        }
        let left_x = snap.trajectory.eval_axis(0, t, SampleSide::Left).unwrap();
        let left_y = snap.trajectory.eval_axis(1, t, SampleSide::Left).unwrap();
        let right_x = snap.trajectory.eval_axis(0, t, SampleSide::Right).unwrap();
        let right_y = snap.trajectory.eval_axis(1, t, SampleSide::Right).unwrap();
        let jump = libm::hypot(
            right_x.acceleration - left_x.acceleration,
            right_y.acceleration - left_y.acceleration,
        );
        assert!(jump < 1e-3, "acceleration stepped by {jump} at t={t}");
    }
}

/// A rest-to-rest corner at a low feed used to fall back to the first-order
/// staircase reconstruction (per-cell constant-acceleration steps), because
/// the apex landing guard scaled its zero-headroom exception with the run's
/// feed ceiling. The disk ride must hold this corner too.
#[test]
fn low_feed_corner_from_rest_rides_the_disk() {
    let waypoints = vec![
        (0.0, 0.0, 0.0, 0.0, 4.5, 1000.0),
        (0.5, 0.0, 0.0, 0.0, 4.5, 1000.0),
        (0.5, 0.5, 0.0, 0.0, 4.5, 1000.0),
    ];
    let mut params = default_axis_snapshot_params();
    params.max_velocity = 4.5;
    params.max_accel = 1000.0;
    params.square_corner_velocity = 5.0;
    let snap = pipeline_snapshot(&waypoints, params).unwrap();
    let clothoids: Vec<(f64, f64)> = snap
        .trajectory
        .spans
        .iter()
        .filter_map(|span| {
            matches!(span.spatial, Some(Spatial::Clothoid { .. }))
                .then_some((span.t_start, span.t_end))
        })
        .collect();
    assert_eq!(clothoids.len(), 2);
    let mut worst = 0.0_f64;
    for &(t0, t1) in &clothoids {
        for i in 1..256 {
            let t = t0 + (t1 - t0) * f64::from(i) / 256.0;
            let x = snap.trajectory.eval_axis(0, t, SampleSide::Right).unwrap();
            let y = snap.trajectory.eval_axis(1, t, SampleSide::Right).unwrap();
            let v = libm::hypot(x.velocity, y.velocity);
            if v < 0.5 || v >= 4.5 * (1.0 - 1e-3) {
                continue;
            }
            let scalar = libm::hypot(x.acceleration, y.acceleration);
            worst = worst.max((1000.0 - scalar).abs());
        }
    }
    assert!(
        worst <= 25.0,
        "low-feed corner left the 1000 mm/s² disk by {worst}"
    );
}

/// The schema is the whole trajectory: a snapshot that has been through JSON
/// evaluates identically to the one that never left memory.
#[test]
fn a_deserialized_snapshot_evaluates_identically() {
    let snap = square_snapshot();
    let text = serde_json::to_string(&snap).unwrap();
    let restored: Snapshot = serde_json::from_str(&text).unwrap();
    assert_eq!(
        restored.trajectory.breakpoints(),
        snap.trajectory.breakpoints()
    );
    let t_end = snap.trajectory.t_end();
    for axis in 0..4 {
        for i in 0..=500 {
            let t = t_end * f64::from(i) / 500.0;
            assert_eq!(
                at(&restored.trajectory, axis, t, SampleSide::Right),
                at(&snap.trajectory, axis, t, SampleSide::Right),
                "axis {axis} at t={t} changed across the schema"
            );
        }
    }
}

#[test]
#[should_panic(expected = "FiniteJerkUnsupported")]
fn finite_max_jerk_is_rejected_by_the_pipeline() {
    let _ = pipeline_snapshot(
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
    );
}

fn default_axis_snapshot_params() -> SnapshotParams {
    SnapshotParams {
        max_velocity: 300.0,
        max_accel: 3000.0,
        square_corner_velocity: 5.0,
        corner_deviation: None,
        max_jerk: f64::INFINITY,
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
    assert!(!snap.trajectory.rows(0).is_empty());
    assert!(!snap.trajectory.rows(3).is_empty());
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
    assert!(!snap.trajectory.rows(0).is_empty());
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
        (
            "tanh_pressure_advance",
            [
                ("linear_advance", 0.02),
                ("nonlinear_offset", 0.05),
                ("linearization_velocity", 20.0),
            ]
            .as_slice(),
            "e",
        ),
        (
            "recipr_pressure_advance",
            [
                ("linear_advance", 0.02),
                ("nonlinear_offset", 0.05),
                ("linearization_velocity", 20.0),
            ]
            .as_slice(),
            "e",
        ),
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
fn linear_advance_speed_ramp_never_retracts() {
    let waypoints = vec![
        (0.0, 0.0, 0.0, 0.0, 10.0, 3000.0),
        (40.0, 0.0, 0.0, 2.0, 10.0, 3000.0),
        (80.0, 0.0, 0.0, 4.0, 30.0, 3000.0),
        (120.0, 0.0, 0.0, 6.0, 80.0, 3000.0),
    ];
    let mut params = default_axis_snapshot_params();
    let mut e = axis("e", &["x", "y", "z"]);
    e.post_processors = vec!["pa".to_string()];
    params.axis_decls = vec![e];
    params.post_processor_decls = vec![pp("pa", "linear_pressure_advance", &[("k", 0.03)])];
    let snap = pipeline_snapshot(&waypoints, params).unwrap();
    let rows = snap.trajectory.rows(3);
    let decel_to_rest_start = 5.82;
    let t_end = rows.last().unwrap().t1.min(decel_to_rest_start);
    let mut previous: Option<(f64, f64)> = None;
    let mut t = 1e-4;
    while t < t_end - 1e-4 {
        let state = snap.trajectory.eval_axis(3, t, SampleSide::Left).unwrap();
        let (p, v) = (state.position, state.velocity);
        if let Some((prev_t, prev_p)) = previous {
            assert!(
                p >= prev_p - 1e-4,
                "extruder retracted from {prev_p} at t={prev_t} to {p} at t={t}"
            );
        }
        assert!(
            v > -0.05,
            "extruder velocity {v} reversed at t={t} on a monotone speed ramp"
        );
        previous = Some((t, p));
        t += 5e-4;
    }
}

#[test]
fn reciprocal_advance_handles_a_fast_corner_endpoint_transition() {
    let waypoints = vec![
        (0.0, 0.0, 0.0, 0.0, 200.0, 3000.0),
        (60.0, 0.0, 0.0, 3.0, 200.0, 3000.0),
        (60.0, 60.0, 0.0, 6.0, 200.0, 3000.0),
    ];
    let mut params = default_axis_snapshot_params();
    let mut e = axis("e", &["x", "y", "z"]);
    e.post_processors = vec!["pa".to_string()];
    params.axis_decls = vec![e];
    params.post_processor_decls = vec![pp(
        "pa",
        "recipr_pressure_advance",
        &[
            ("linear_advance", 0.0),
            ("nonlinear_offset", 0.06),
            ("linearization_velocity", 2.0),
        ],
    )];
    pipeline_snapshot(&waypoints, params)
        .expect("a sub-resolution reciprocal transition must not fail the snapshot");
}

#[test]
fn nonlinear_advance_before_follower_smoothing_preserves_input_seams() {
    let waypoints = vec![
        (0.0, 0.0, 0.0, 0.0, 300.0, 3000.0),
        (40.0, 0.0, 0.0, 2.0, 300.0, 3000.0),
        (40.0, 40.0, 0.0, 4.0, 300.0, 3000.0),
    ];
    let mut params = default_axis_snapshot_params();
    let mut x = axis("x", &[]);
    x.post_processors = vec!["shaper".to_string()];
    let mut y = axis("y", &[]);
    y.post_processors = vec!["shaper".to_string()];
    let mut e = axis("e", &["x", "y", "z"]);
    e.post_processors = vec!["pa".to_string(), "st_e".to_string()];
    params.axis_decls = vec![x, y, e];
    params.post_processor_decls = vec![
        pp("shaper", "smooth_bell", &[("smooth_time", 0.02390625)]),
        pp(
            "pa",
            "tanh_pressure_advance",
            &[
                ("linear_advance", 0.01),
                ("nonlinear_offset", 0.06),
                ("linearization_velocity", 2.0),
            ],
        ),
        pp("st_e", "smooth_bell", &[("smooth_time", 0.02675)]),
    ];
    pipeline_snapshot(&waypoints, params)
        .expect("segment-local nonlinear offsets must preserve projected input seams");
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

fn max_axis_difference(a: &ExactTrajectory, b: &ExactTrajectory, axis: usize) -> f64 {
    let t_end = a.t_end().min(b.t_end());
    (0..=1000)
        .map(|i| t_end * i as f64 / 1000.0)
        .map(|t| (position(a, axis, t) - position(b, axis, t)).abs())
        .fold(0.0, f64::max)
}

#[test]
fn mode_inverse_emits_a_toolhead_signal_distinct_from_the_motor_command() {
    let snap = pipeline_snapshot(&square_waypoints(), mode_inverse_on_x_params()).unwrap();
    let toolhead = snap.toolhead.as_ref().expect("toolhead trajectory");
    assert!(!toolhead.rows(0).is_empty());
    assert!(
        max_axis_difference(toolhead, &snap.trajectory, 0) > 1e-2,
        "x carries motor-side gains, so its motor command must depart from the toolhead signal"
    );
    assert!(
        max_axis_difference(toolhead, &snap.trajectory, 1) < 1e-6,
        "y has no motor-side stage, so both signals coincide"
    );
    let json: serde_json::Value = serde_json::to_value(&snap).unwrap();
    assert!(
        json.get("toolhead").is_some(),
        "missing snapshot key toolhead"
    );
    for key in ["spans", "curves", "axes", "t_end"] {
        assert!(
            json["toolhead"].get(key).is_some(),
            "missing toolhead trajectory key {key}"
        );
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
    assert!(snap.toolhead.is_none());
    let json: serde_json::Value = serde_json::to_value(&snap).unwrap();
    assert!(
        json.get("toolhead").is_none(),
        "kernel-only snapshot must serialize without a toolhead trajectory"
    );
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
    let mut partials: Vec<(Vec<f64>, serde_json::Value)> = Vec::new();
    let streamed = pipeline_snapshot_streaming(
        &square_waypoints(),
        default_axis_snapshot_params(),
        2,
        |snap| {
            partials.push((
                snap.raw_x.clone(),
                serde_json::to_value(snap.trajectory.rows(0)).unwrap(),
            ))
        },
    )
    .unwrap();
    assert!(!partials.is_empty());
    let final_rows = serde_json::to_value(streamed.trajectory.rows(0)).unwrap();
    let final_rows = final_rows.as_array().unwrap();
    let mut prev_len = 0;
    for (raw_x, rows) in &partials {
        assert_eq!(*raw_x, streamed.raw_x);
        let rows = rows.as_array().unwrap();
        assert!(rows.len() >= prev_len);
        assert!(rows.len() <= final_rows.len());
        assert_eq!(rows[..], final_rows[..rows.len()]);
        prev_len = rows.len();
    }
}

#[test]
fn streaming_partials_carry_toolhead_lanes_when_the_chain_has_motor_side_stages() {
    let mut saw_toolhead_partial = false;
    let streamed =
        pipeline_snapshot_streaming(&square_waypoints(), mode_inverse_on_x_params(), 2, |snap| {
            assert!(snap.toolhead.is_some());
            saw_toolhead_partial = true;
        })
        .unwrap();
    assert!(saw_toolhead_partial);
    assert!(streamed.toolhead.is_some());
}

/// The playground's stock G-code (Voron cube layer 5) carries three
/// concentric near-closed circle-wall loops bounded by travels. Their fitted
/// centers must agree across loops and across corner-deviation budgets:
/// raising the budget widens which facets join a run, never how well the
/// shared circle is placed. Regression for the endpoint-anchored fit
/// scattering the loop centers by ~0.25mm at 0.3mm deviation.
#[test]
fn default_gcode_circle_loops_stay_concentric_across_corner_deviation() {
    let text = std::fs::read_to_string(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../snapshots/web/static/default.gcode"
    ))
    .unwrap();
    let mut loop_centers = Vec::new();
    let mut radii_per_cd: Vec<Vec<f64>> = Vec::new();
    for cd in [0.05, 0.3] {
        let waypoints = waypoints::parse_gcode(&text, 300.0, 3000.0).unwrap();
        let limits = geometry::VelocityLimits::try_new(300.0, 3000.0, cd, f64::INFINITY).unwrap();
        let moves = build_moves(&waypoints, limits).unwrap();
        let (fitted, _, _) = run_pipeline(&moves, default_config(limits), AxisChainSet::default());
        let loops: Vec<&geometry::path::Arc> = fitted
            .iter()
            .filter_map(|m| match &m.segment.spatial {
                Some(geometry::path::Segment::Arc(a)) if a.sweep.abs().to_degrees() > 300.0 => {
                    Some(a)
                }
                _ => None,
            })
            .collect();
        assert_eq!(loops.len(), 3, "cd={cd}: expected the three wall loops");
        loop_centers.extend(loops.iter().map(|a| [a.origin[0], a.origin[1]]));
        let mut radii: Vec<f64> = loops.iter().map(|a| a.radius).collect();
        radii.sort_by(f64::total_cmp);
        radii_per_cd.push(radii);
    }
    let mean = loop_centers
        .iter()
        .fold([0.0, 0.0], |s, c| [s[0] + c[0], s[1] + c[1]])
        .map(|v| v / loop_centers.len() as f64);
    for c in &loop_centers {
        let err = libm::hypot(c[0] - mean[0], c[1] - mean[1]);
        assert!(
            err < 0.05,
            "loop center {c:?} strays {err:.4}mm from the shared center {mean:?}"
        );
    }
    for (lo, hi) in radii_per_cd[0].iter().zip(&radii_per_cd[1]) {
        assert!(
            (lo - hi).abs() < 0.025,
            "loop radius changed with the deviation budget: {lo:.4} vs {hi:.4}"
        );
    }
    for w in radii_per_cd[0].windows(2) {
        assert!(
            w[1] - w[0] > 0.3,
            "adjacent wall loops nearly coincide: {:?}",
            radii_per_cd[0]
        );
    }
}
