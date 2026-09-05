use super::*;
use pipeline_snapshot::{Carrier, CarrierRow, SplineCurve};

// -- Exact-carrier fixtures --------------------------------------------------

// A clamped cubic Bezier over [t0, t1] in absolute stream time -- the same
// parameterization the runtime spline carrier uses, so its position and all
// three derivatives are exact closed forms the assertions below can predict
// independently of how the evaluator is implemented.
fn cubic(t0: f64, t1: f64, p: [f64; 4]) -> SplineCurve {
    SplineCurve {
        degree: 3,
        knots: vec![t0, t0, t0, t0, t1, t1, t1, t1],
        control_points: p.to_vec(),
    }
}

fn bezier_pvaj(t0: f64, t1: f64, p: [f64; 4], t: f64) -> (f64, f64, f64, f64) {
    let dt = t1 - t0;
    let u = (t - t0) / dt;
    let w = 1.0 - u;
    let pos = p[0] * w * w * w + 3.0 * p[1] * u * w * w + 3.0 * p[2] * u * u * w + p[3] * u * u * u;
    let vel = 3.0 / dt
        * ((p[1] - p[0]) * w * w + 2.0 * (p[2] - p[1]) * u * w + (p[3] - p[2]) * u * u);
    let acc =
        6.0 / (dt * dt) * ((p[2] - 2.0 * p[1] + p[0]) * w + (p[3] - 2.0 * p[2] + p[1]) * u);
    let jerk = 6.0 / (dt * dt * dt) * (p[3] - 3.0 * p[2] + 3.0 * p[1] - p[0]);
    (pos, vel, acc, jerk)
}

fn spline_row(t0: f64, t1: f64, curve: usize) -> CarrierRow {
    CarrierRow {
        t0,
        t1,
        carrier: Carrier::Spline { curve },
    }
}

fn hold_row(t0: f64, t1: f64, position: f64) -> CarrierRow {
    CarrierRow {
        t0,
        t1,
        carrier: Carrier::Hold { position },
    }
}

fn traj(curves: Vec<SplineCurve>, x: Vec<CarrierRow>, y: Vec<CarrierRow>) -> ExactTrajectory {
    ExactTrajectory::from_parts(Vec::new(), curves, [x, y, Vec::new(), Vec::new()])
}

// x traverses one cubic on [0, 1]; y holds. One carrier per axis, so the whole
// grid lives inside a single exact span.
fn single_cubic() -> ExactTrajectory {
    let cx = cubic(0.0, 1.0, [0.0, 1.0, 5.0, 9.0]);
    traj(
        vec![cx],
        vec![spline_row(0.0, 1.0, 0)],
        vec![hold_row(0.0, 1.0, 2.0)],
    )
}

// -- Exact evaluation on the display grid -----------------------------------

#[test]
fn grid_samples_every_breakpoint_from_both_sides() {
    let cx0 = cubic(0.0, 0.5, [0.0, 0.5, 1.0, 1.5]);
    let cx1 = cubic(0.5, 1.0, [1.5, 2.5, 4.0, 6.0]);
    let t = traj(
        vec![cx0, cx1],
        vec![spline_row(0.0, 0.5, 0), spline_row(0.5, 1.0, 1)],
        vec![hold_row(0.0, 1.0, 0.0)],
    );
    let grid = display_grid(&t);

    let seam: Vec<&GridSample> = grid.iter().filter(|g| g.t == 0.5).collect();
    assert_eq!(seam.len(), 2, "the seam must be sampled from both sides");
    assert!(seam[0].left, "the closing sample belongs to the left carrier");
    assert!(!seam[1].left, "the opening sample belongs to the right one");
    assert_eq!(grid.first().unwrap().t, 0.0);
    assert!(!grid.first().unwrap().left);
    assert_eq!(grid.last().unwrap().t, 1.0);
    assert!(grid.last().unwrap().left);
}

#[test]
fn grid_density_follows_time_not_carrier_count() {
    // One 1 s carrier at a 0.25 ms target sample spacing hits the 512 clamp,
    // and a 1 ms one falls back to the 4-sample floor.
    assert_eq!(samples_in(0.0, 1.0), 512);
    assert_eq!(samples_in(0.0, 1e-3), 4);
    assert_eq!(samples_in(0.0, 2.5e-4), 4);
}

#[test]
fn time_series_jerk_is_the_carriers_own_third_derivative() {
    // A cubic's jerk is a nonzero constant; a numerically differentiated
    // acceleration would smear it at every seam and endpoint instead.
    let t = single_cubic();
    let grid = display_grid(&t);
    let ts = time_series(&t, &grid).unwrap();
    let (_, _, _, expected_jerk) = bezier_pvaj(0.0, 1.0, [0.0, 1.0, 5.0, 9.0], 0.3);
    assert!(expected_jerk.abs() > 1.0);
    for (i, &j) in ts.jx.iter().enumerate() {
        assert!(
            (j - expected_jerk).abs() < 1e-9,
            "sample {i}: jerk {j} != {expected_jerk}"
        );
    }
    assert!(ts.jy.iter().all(|&j| j == 0.0));
}

#[test]
fn time_series_matches_the_closed_form_at_every_sample() {
    let t = single_cubic();
    let grid = display_grid(&t);
    let ts = time_series(&t, &grid).unwrap();
    for i in 0..ts.t.len() {
        let (pos, vel, acc, jerk) = bezier_pvaj(0.0, 1.0, [0.0, 1.0, 5.0, 9.0], ts.t[i]);
        assert!((ts.kin_x[i] - pos).abs() < 1e-9);
        assert!((ts.vx[i] - vel).abs() < 1e-9);
        assert!((ts.ax[i] - acc).abs() < 1e-9);
        assert!((ts.jx[i] - jerk).abs() < 1e-9);
        assert_eq!(ts.kin_y[i], 2.0);
    }
    // Z and E carry no carriers in this case: flat zero tracks, full length.
    assert_eq!(ts.vz.len(), ts.t.len());
    assert!(ts.vz.iter().chain(&ts.az).chain(&ts.jz).all(|&v| v == 0.0));
    assert!(ts.ve.iter().chain(&ts.ae).chain(&ts.je).all(|&v| v == 0.0));
}

#[test]
fn time_series_renders_an_acceleration_step_as_a_step() {
    // Two cubics meeting at t=0.5 with matched position/velocity but different
    // acceleration: the two seam samples must report the two carriers' own
    // values, not one averaged value.
    let p0 = [0.0, 1.0, 2.0, 3.0];
    let p1 = [3.0, 4.0, 6.0, 9.0];
    let t = traj(
        vec![cubic(0.0, 0.5, p0), cubic(0.5, 1.0, p1)],
        vec![spline_row(0.0, 0.5, 0), spline_row(0.5, 1.0, 1)],
        vec![hold_row(0.0, 1.0, 0.0)],
    );
    let grid = display_grid(&t);
    let ts = time_series(&t, &grid).unwrap();
    let seam: Vec<usize> = (0..ts.t.len()).filter(|&i| ts.t[i] == 0.5).collect();
    assert_eq!(seam.len(), 2);
    let left = bezier_pvaj(0.0, 0.5, p0, 0.5).2;
    let right = bezier_pvaj(0.5, 1.0, p1, 0.5).2;
    assert!((ts.ax[seam[0]] - left).abs() < 1e-9);
    assert!((ts.ax[seam[1]] - right).abs() < 1e-9);
    assert!((left - right).abs() > 1.0);
}

// -- Derivative impulses ----------------------------------------------------

#[test]
fn jerk_impulses_measure_the_exact_acceleration_step() {
    let p0 = [0.0, 1.0, 2.0, 3.0];
    let p1 = [3.0, 4.0, 6.0, 9.0];
    let t = traj(
        vec![cubic(0.0, 0.5, p0), cubic(0.5, 1.0, p1)],
        vec![spline_row(0.0, 0.5, 0), spline_row(0.5, 1.0, 1)],
        vec![hold_row(0.0, 1.0, 0.0)],
    );
    let (times, mags) = jerk_impulses(&t, 0.0).unwrap();
    let expected = (bezier_pvaj(0.5, 1.0, p1, 0.5).2 - bezier_pvaj(0.0, 0.5, p0, 0.5).2).abs();
    assert_eq!(times, vec![0.5]);
    assert!((mags[0] - expected).abs() < 1e-9);
}

#[test]
fn accel_impulses_measure_the_exact_velocity_step() {
    // Control points chosen so the two carriers meet in position but not in
    // velocity: a genuine acceleration impulse at the seam.
    let p0 = [0.0, 1.0, 2.0, 3.0];
    let p1 = [3.0, 3.1, 4.0, 6.0];
    let t = traj(
        vec![cubic(0.0, 0.5, p0), cubic(0.5, 1.0, p1)],
        vec![spline_row(0.0, 0.5, 0), spline_row(0.5, 1.0, 1)],
        vec![hold_row(0.0, 1.0, 0.0)],
    );
    let (times, mags) = accel_impulses(&t, 0.0).unwrap();
    let expected = (bezier_pvaj(0.5, 1.0, p1, 0.5).1 - bezier_pvaj(0.0, 0.5, p0, 0.5).1).abs();
    assert!(expected > 1.0);
    assert_eq!(times, vec![0.5]);
    assert!((mags[0] - expected).abs() < 1e-9);
}

#[test]
fn a_continuous_seam_reports_no_impulse() {
    // One curve split into two rows at 0.5 is C-infinity across the seam.
    let cx = cubic(0.0, 1.0, [0.0, 1.0, 5.0, 9.0]);
    let t = traj(
        vec![cx],
        vec![spline_row(0.0, 0.5, 0), spline_row(0.5, 1.0, 0)],
        vec![hold_row(0.0, 1.0, 0.0)],
    );
    let grid = display_grid(&t);
    let ts = time_series(&t, &grid).unwrap();
    let a_peak = ts.a_scalar.iter().copied().fold(0.0_f64, f64::max);
    let v_peak = ts.v_scalar.iter().copied().fold(0.0_f64, f64::max);
    assert!(jerk_impulses(&t, a_peak).unwrap().0.is_empty());
    assert!(accel_impulses(&t, v_peak).unwrap().0.is_empty());
}

// -- End-to-end through the current schema ----------------------------------

#[test]
fn from_json_renders_exact_series_for_a_current_schema_snapshot() {
    let curves = || {
        vec![
            cubic(0.0, 1.0, [0.0, 1.0, 5.0, 9.0]),
            cubic(0.0, 1.0, [0.0, 0.0, 1.0, 3.0]),
        ]
    };
    let both_axes = || {
        traj(
            curves(),
            vec![spline_row(0.0, 1.0, 0)],
            vec![spline_row(0.0, 1.0, 1)],
        )
    };
    let motor = both_axes();
    let toolhead = both_axes();
    let snap = Snapshot {
        schema_version: pipeline_snapshot::SNAPSHOT_SCHEMA_VERSION,
        raw_x: vec![0.0, 9.0],
        raw_y: vec![0.0, 3.0],
        trajectory: motor,
        toolhead: Some(toolhead),
        traversal_time_s: 1.0,
        seam_max_dp: [1e-9, 2e-9, 0.0, 0.0],
        seam_max_dv: [0.0; 4],
        seam_max_da: [0.0; 4],
        worst_seams: Vec::new(),
    };
    let json = serde_json::to_string(&snap).unwrap();

    let data = TrajectoryData::from_json(&json).expect("current-schema snapshot must load");
    assert!(data.point_count() > 100);
    assert_eq!(data.kappa.len(), data.t.len());
    assert_eq!(data.curvature_class.len(), data.t.len());
    assert_eq!(data.jx.len(), data.t.len());
    assert!(data.jx.iter().all(|j| j.abs() > 1.0));
    assert_eq!(data.seam_max_dp, vec![1e-9, 2e-9, 0.0, 0.0]);
    assert_eq!(data.worst_seams_json, "[]");
    assert_eq!(data.traversal_time_s, 1.0);
    assert!(data.has_toolhead());

    // The toolhead lane is the same signal here, so every mirrored series must
    // land on the motor-command one sample for sample.
    assert_eq!(data.toolhead.x, data.kin_x);
    assert_eq!(data.toolhead.jx, data.jx);
    assert_eq!(data.toolhead.j_scalar, data.j_scalar);
    assert_eq!(data.toolhead.a_tang, data.a_tang);
    assert_eq!(data.toolhead.kappa, data.kappa);

    // No cusp/gap anywhere: this case moves throughout and every instant is
    // covered by a carrier.
    assert!(
        data.curvature_class
            .iter()
            .all(|&c| c != CurvatureClass::Cusp.code() && c != CurvatureClass::Gap.code())
    );
}

#[test]
fn a_snapshot_without_xy_carriers_reads_as_gap() {
    let snap = Snapshot {
        schema_version: pipeline_snapshot::SNAPSHOT_SCHEMA_VERSION,
        raw_x: Vec::new(),
        raw_y: Vec::new(),
        trajectory: ExactTrajectory::from_parts(
            Vec::new(),
            Vec::new(),
            [Vec::new(), Vec::new(), Vec::new(), Vec::new()],
        ),
        toolhead: None,
        traversal_time_s: 0.0,
        seam_max_dp: [0.0; 4],
        seam_max_dv: [0.0; 4],
        seam_max_da: [0.0; 4],
        worst_seams: Vec::new(),
    };
    let json = serde_json::to_string(&snap).unwrap();
    let data = TrajectoryData::from_json(&json).unwrap();
    assert!(!data.has_toolhead());
    assert!(
        data.curvature_class
            .iter()
            .all(|&c| c == CurvatureClass::Gap.code())
    );
    assert!(data.jerk_impulse_t.is_empty());
    assert!(data.accel_impulse_t.is_empty());
}

// A real snapshot straight out of the production stages, not a hand-built
// fixture: the viewer must render it with no piece conversion anywhere between
// the shaper's carriers and the plotted series.
fn real_snapshot(
    axis_decls: Vec<pipeline_snapshot::AxisDecl>,
    post_processor_decls: Vec<pipeline_snapshot::PostProcessorDecl>,
) -> Snapshot {
    let gcode = "G90\nM83\nG1 X60 Y0 F9000\nG1 X60 Y60\nG1 X0 Y60\n";
    let waypoints = pipeline_snapshot::waypoints::parse_gcode(gcode, 300.0, 3000.0).unwrap();
    pipeline_snapshot::pipeline_snapshot(
        &waypoints,
        pipeline_snapshot::SnapshotParams {
            max_velocity: 300.0,
            max_accel: 3000.0,
            square_corner_velocity: 5.0,
            corner_deviation: None,
            max_jerk: f64::INFINITY,
            max_extrude_only_velocity: None,
            max_extrude_only_accel: None,
            max_path_deviation: None,
            max_accel_deviation: None,
            axis_decls,
            post_processor_decls,
        },
    )
    .unwrap()
}

#[test]
fn a_real_pipeline_snapshot_renders_exact_series() {
    let snap = real_snapshot(Vec::new(), Vec::new());
    let breakpoints = snap.trajectory.breakpoints();
    let t_end = snap.trajectory.t_end();
    let json = serde_json::to_string(&snap).unwrap();

    let data = TrajectoryData::from_json(&json).expect("a real snapshot must load");
    let n = data.t.len();
    assert!(n > 1000, "a 180 mm three-move case must sample densely");
    for series in [
        &data.kin_x,
        &data.kin_y,
        &data.vx,
        &data.ax,
        &data.jx,
        &data.j_scalar,
        &data.kappa,
        &data.curvature_class,
        &data.a_cent,
        &data.j_tang,
    ] {
        assert_eq!(series.len(), n);
        assert!(series.iter().all(|v| v.is_finite()));
    }

    // The grid lands on the trajectory's own breakpoints, and every interior
    // one carries BOTH neighbouring carriers' own acceleration, so a step is
    // plotted as a step instead of being averaged away.
    for &b in &breakpoints {
        let hit: Vec<f64> = (0..n).filter(|&i| data.t[i] == b).map(|i| data.ax[i]).collect();
        assert!(!hit.is_empty(), "breakpoint {b} is missing from the grid");
        if b <= 0.0 || b >= t_end {
            continue;
        }
        for side in [SampleSide::Left, SampleSide::Right] {
            let exact = snap
                .trajectory
                .eval_axis(AXIS_X, b, side)
                .unwrap()
                .acceleration;
            assert!(
                hit.contains(&exact),
                "breakpoint {b} misses its {side:?} acceleration {exact}"
            );
        }
    }

    // Motion actually happened, and the jerk lane is the carriers' own third
    // derivative rather than a difference of neighbouring accelerations.
    assert!(data.v_scalar.iter().copied().fold(0.0, f64::max) > 100.0);
    assert!(data.j_scalar.iter().copied().fold(0.0, f64::max) > 0.0);
    assert!(data.kin_x.iter().copied().fold(0.0, f64::max) > 59.0);
    assert!(data.kin_y.iter().copied().fold(0.0, f64::max) > 59.0);

    // The two corners bend the path: curvature is rendered, and classified as
    // something other than a straight run somewhere along the way.
    assert!(data.kappa.iter().any(|k| k.abs() > KAPPA_ZERO_EPS));
    assert!(
        data.curvature_class
            .iter()
            .any(|&c| c != CurvatureClass::Zero.code())
    );

    // Every reported impulse sits on an interior breakpoint and carries a
    // finite strength: the exact one-sided jump, no midpoint probing.
    for (times, mags) in [
        (&data.jerk_impulse_t, &data.jerk_impulse_mag),
        (&data.accel_impulse_t, &data.accel_impulse_mag),
    ] {
        assert_eq!(times.len(), mags.len());
        for (&t, &m) in times.iter().zip(mags) {
            assert!(t > 0.0 && t < t_end);
            assert!(breakpoints.contains(&t));
            assert!(m.is_finite() && m > 0.0);
        }
    }
}

// A motor-side chain (bell smoothing + mode inverse on X/Y) makes the motor
// command depart from the toolhead signal, so the snapshot carries both. Both
// must render on the one shared grid the panels overlay.
#[test]
fn a_chained_snapshot_renders_the_toolhead_lane_on_the_shared_grid() {
    let decl = |name: &str, ty: &str, params: &[(&str, f64)]| pipeline_snapshot::PostProcessorDecl {
        name: name.to_string(),
        ty: ty.to_string(),
        params: params
            .iter()
            .map(|(k, v)| ((*k).to_string(), *v))
            .collect(),
    };
    let axis = |name: &str| pipeline_snapshot::AxisDecl {
        name: name.to_string(),
        follows: Vec::new(),
        motors: Vec::new(),
        post_processors: vec!["slew".to_string(), "belt".to_string()],
    };
    let snap = real_snapshot(
        vec![axis("x"), axis("y")],
        vec![
            decl("slew", "smooth_bell", &[("smooth_time", 0.0015)]),
            decl(
                "belt",
                "mode_inverse",
                &[("frequency_hz", 40.0), ("damping_ratio", 0.1)],
            ),
        ],
    );
    assert!(snap.toolhead.is_some(), "a mode_inverse chain must emit one");
    let json = serde_json::to_string(&snap).unwrap();

    let data = TrajectoryData::from_json(&json).expect("a chained snapshot must load");
    assert!(data.has_toolhead());
    let n = data.t.len();
    for series in [
        &data.toolhead.x,
        &data.toolhead.vx,
        &data.toolhead.ax,
        &data.toolhead.jx,
        &data.toolhead.j_scalar,
        &data.toolhead.a_cent,
        &data.toolhead.kappa,
    ] {
        assert_eq!(series.len(), n);
        assert!(series.iter().all(|v| v.is_finite()));
    }
    // The chain is what the two lanes differ by: identical lanes would mean the
    // viewer plotted the motor command twice.
    assert_ne!(data.toolhead.x, data.kin_x);
    assert_ne!(data.toolhead.jx, data.jx);
}

// -- Representation-independent math ----------------------------------------

#[test]
fn frenet_components_split_dot_and_cross() {
    // v = (3, 4) (speed 5), f = (1, 2): tangential = (3+8)/5, normal = |6-4|/5.
    let (tang, norm) = frenet_components(&[3.0], &[4.0], &[1.0], &[2.0]);
    assert!((tang[0] - 2.2).abs() < 1e-12);
    assert!((norm[0] - 0.4).abs() < 1e-12);
}

#[test]
fn frenet_tangential_is_signed_while_braking() {
    // f anti-parallel to v: all tangential, negative; no normal component.
    let (tang, norm) = frenet_components(&[5.0], &[0.0], &[-100.0], &[0.0]);
    assert_eq!(tang[0], -100.0);
    assert_eq!(norm[0], 0.0);
}

#[test]
fn frenet_components_read_zero_when_stopped() {
    let (tang, norm) = frenet_components(&[0.0], &[0.0], &[100.0], &[-50.0]);
    assert_eq!(tang[0], 0.0);
    assert_eq!(norm[0], 0.0);
}

#[test]
fn frenet_components_recover_pure_centripetal_turn() {
    // Circular motion: v = (0, 2), a = (-8, 0) — a ⟂ v, so the whole
    // acceleration is centripetal (|a| = v²/r) and none is tangential.
    let (tang, norm) = frenet_components(&[0.0], &[2.0], &[-8.0], &[0.0]);
    assert_eq!(tang[0], 0.0);
    assert_eq!(norm[0], 8.0);
}

#[test]
fn kappa_is_zero_on_a_straight_line() {
    // Constant velocity, zero accel/jerk -- no curvature regardless of speed.
    let (kappa, dkappa_dt) = kappa_and_dkappa_dt(5.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    assert_eq!(kappa, 0.0);
    assert_eq!(dkappa_dt, 0.0);
}

#[test]
fn kappa_constant_on_circle_with_nonconstant_speed() {
    // Circle of radius R, parameterized by theta(t) = t^2 -- so theta' = 2t
    // is NOT constant, i.e. the tangential speed along the circle varies
    // with t. Curvature must still read as exactly 1/R at every t: kappa is
    // a property of the path's shape, not of how fast it's traversed. If
    // the formula secretly depended on ds/dt this test would fail at one of
    // the two very different speeds checked below.
    let r = 3.0_f64;
    let kappa_at = |t: f64| -> f64 {
        let theta = t * t;
        let (s, c) = libm::sincos(theta);
        let vx = -2.0 * r * t * s;
        let vy = 2.0 * r * t * c;
        let ax = -2.0 * r * s - 4.0 * r * t * t * c;
        let ay = 2.0 * r * c - 4.0 * r * t * t * s;
        let jx = -12.0 * r * t * c + 8.0 * r * t.powi(3) * s;
        let jy = -12.0 * r * t * s - 8.0 * r * t.powi(3) * c;
        kappa_and_dkappa_dt(vx, vy, ax, ay, jx, jy).0
    };
    let k_slow = kappa_at(0.3); // speed = 2*r*0.3 = 1.8*r
    let k_fast = kappa_at(0.9); // speed = 2*r*0.9 = 5.4*r -- 3x faster
    assert!((k_slow - 1.0 / r).abs() < 1e-9);
    assert!((k_fast - 1.0 / r).abs() < 1e-9);
}

#[test]
fn dkappa_ds_constant_on_clothoid() {
    // Euler spiral parameterized directly by arc length (dx/ds = cos(phi),
    // dy/ds = sin(phi), phi = sigma*s^2/2) -- so speed == 1 identically and
    // t IS s here, letting dkappa_dt stand in for dkappa_ds directly.
    // kappa(s) = sigma*s by construction; dkappa/ds must read back as the
    // constant sigma at every s, independent of s.
    let sigma = 0.25_f64;
    let dkappa_ds_at = |s: f64| -> f64 {
        let phi = 0.5 * sigma * s * s;
        let (sp, cp) = libm::sincos(phi);
        let vx = cp;
        let vy = sp;
        let ax = -sp * sigma * s;
        let ay = cp * sigma * s;
        let jx = -cp * sigma * sigma * s * s - sp * sigma;
        let jy = -sp * sigma * sigma * s * s + cp * sigma;
        kappa_and_dkappa_dt(vx, vy, ax, ay, jx, jy).1
    };
    assert!((dkappa_ds_at(0.5) - sigma).abs() < 1e-9);
    assert!((dkappa_ds_at(2.0) - sigma).abs() < 1e-9);
    assert!((dkappa_ds_at(4.0) - sigma).abs() < 1e-9);
}

#[test]
fn classify_window_zero_when_kappa_is_flat_zero() {
    let kappa = vec![0.0, 1e-6, -1e-6, 2e-6];
    let dkappa_ds = vec![0.0, 0.0, 0.0, 0.0];
    assert_eq!(classify_window(&kappa, &dkappa_ds), CurvatureClass::Zero);
}

#[test]
fn classify_window_constant_when_kappa_nonzero_but_steady() {
    let kappa = vec![0.02, 0.021, 0.019, 0.02];
    let dkappa_ds = vec![0.0, 1e-6, -1e-6, 0.0];
    assert_eq!(
        classify_window(&kappa, &dkappa_ds),
        CurvatureClass::Constant
    );
}

#[test]
fn classify_window_linear_when_rate_is_steady_nonzero() {
    let kappa = vec![0.0, 0.01, 0.02, 0.03];
    let dkappa_ds = vec![0.01, 0.0102, 0.0099, 0.0101];
    assert_eq!(classify_window(&kappa, &dkappa_ds), CurvatureClass::Linear);
}

#[test]
fn classify_window_other_when_rate_is_unsteady() {
    let kappa = vec![0.0, 0.05, -0.02, 0.08];
    let dkappa_ds = vec![0.05, -0.07, 0.1, -0.09];
    assert_eq!(classify_window(&kappa, &dkappa_ds), CurvatureClass::Other);
}

#[test]
fn classify_window_ignores_a_handful_of_outliers() {
    // 24 steady samples plus 2 seam-artifact outliers -- the percentile
    // spread must not be blown out by them the way a raw max-min would be.
    let mut dkappa_ds = vec![0.01; 24];
    dkappa_ds[5] = 5.0;
    dkappa_ds[19] = -5.0;
    let kappa: Vec<f64> = (0..24).map(|i| 0.01 * i as f64).collect();
    assert_eq!(classify_window(&kappa, &dkappa_ds), CurvatureClass::Linear);
}

#[test]
fn classify_window_handles_small_windows_with_outliers() {
    // 4-sample window (small, under n=10): 3 steady samples + 1 extreme outlier.
    // Sorted dkappa_ds = [0.01, 0.01, 0.01, 5.0]: trim = 1, so lo=sorted[1],
    // hi=sorted[2], spread=0 -> Linear on the median rate. Without the trim
    // floor the raw min-max spread would read 4.99 and misclassify as Other.
    let dkappa_ds = vec![0.01, 0.01, 5.0, 0.01];
    let kappa = vec![0.01, 0.02, 0.03, 0.04];
    assert_eq!(classify_window(&kappa, &dkappa_ds), CurvatureClass::Linear);
}

#[test]
fn smooth_classes_kills_an_isolated_single_window_flicker() {
    use CurvatureClass::*;
    let raw = vec![Constant, Constant, Other, Constant, Constant];
    let smoothed = smooth_classes(&raw);
    assert_eq!(
        smoothed,
        vec![Constant, Constant, Constant, Constant, Constant]
    );
}

#[test]
fn smooth_classes_keeps_a_sustained_change() {
    use CurvatureClass::*;
    let raw = vec![Constant, Constant, Other, Other, Other, Constant, Constant];
    let smoothed = smooth_classes(&raw);
    assert_eq!(smoothed, raw);
}

#[test]
fn percentile_spread_reads_zero_at_n_equals_three() {
    // At n=3, trim = 1, so lo and hi both index the middle element (the
    // median), forcing spread to always be 0.0 regardless of how extreme the
    // two outer samples are. This is the documented, accepted tradeoff: a
    // 3-sample window errs toward "not enough data to call it anomalous"
    // rather than risking a false positive on extreme outer values.
    let sorted = vec![-100.0, 0.01, 100.0];
    assert_eq!(percentile_spread(&sorted), 0.0);
}

#[test]
fn curvature_series_flags_a_cusp_at_zero_speed() {
    let (_, classes) = curvature_series(
        &[0.0, 0.0],
        &[0.0, 0.0],
        &[1.0, 1.0],
        &[2.0, 2.0],
        &[0.0, 0.0],
        &[0.0, 0.0],
    );
    assert!(classes.iter().all(|&c| c == CurvatureClass::Cusp));
}

#[test]
fn curvature_series_flags_cusp_at_a_speed_too_small_for_kappa_to_be_meaningful() {
    // A speed of 1e-6 clears FRENET_SPEED_FLOOR (1e-9) by three orders of
    // magnitude, but kappa's speed^3 denominator still makes it blow up to
    // an astronomically large value here -- this must still read as Cusp,
    // not as a giant-but-"real" curvature number.
    let (_, classes) = curvature_series(&[1e-6], &[0.0], &[2000.0], &[2.0], &[0.0], &[0.0]);
    assert_eq!(classes[0], CurvatureClass::Cusp);
}

#[test]
fn curvature_series_reads_zero_on_a_straight_run() {
    let n = 50;
    let vx = vec![100.0; n];
    let vy = vec![0.0; n];
    let zeros = vec![0.0; n];
    let (kappa, classes) =
        curvature_series(&vx, &vy, &zeros, &zeros, &zeros, &zeros);
    assert!(kappa.iter().all(|&k| k == 0.0));
    assert_eq!(classes.len(), n);
    assert!(classes.iter().all(|&c| c == CurvatureClass::Zero));
}

#[test]
fn curvature_series_despikes_an_isolated_curved_window() {
    // 24 + 24 + 2 samples: the middle window carries real curvature (well
    // above KAPPA_ZERO_EPS) flanked by two straight windows that agree with
    // each other, so smooth_classes must fold it back to Zero.
    let n = 50;
    let vx = vec![100.0; n];
    let vy = vec![0.0; n];
    let mut ay = vec![0.0; n];
    for a in ay.iter_mut().take(48).skip(24) {
        *a = 5000.0;
    }
    let zeros = vec![0.0; n];
    let (kappa, classes) = curvature_series(&vx, &vy, &zeros, &ay, &zeros, &zeros);
    assert!(kappa[24..48].iter().all(|k| k.abs() > KAPPA_ZERO_EPS));
    assert!(classes.iter().all(|&c| c == CurvatureClass::Zero));
}

#[test]
fn toolhead_series_mirrors_motor_derived_series() {
    let t = single_cubic();
    let grid = display_grid(&t);
    let s = toolhead_series(&t, &grid).unwrap();
    let ts = time_series(&t, &grid).unwrap();

    assert_eq!(s.x, ts.kin_x);
    assert_eq!(s.jx, ts.jx);
    assert_eq!(s.v_scalar, scalar_derivative(&s.vx, &s.vy));
    assert_eq!(s.a_scalar, scalar_derivative(&s.ax, &s.ay));
    assert_eq!(s.j_scalar, scalar_derivative(&s.jx, &s.jy));

    let (a_tang, a_cent) = frenet_components(&s.vx, &s.vy, &s.ax, &s.ay);
    let (j_tang, j_cent) = frenet_components(&s.vx, &s.vy, &s.jx, &s.jy);
    assert_eq!(s.a_tang, a_tang);
    assert_eq!(s.a_cent, a_cent);
    assert_eq!(s.j_tang, j_tang);
    assert_eq!(s.j_cent, j_cent);
    assert_eq!(s.kappa.len(), grid.len());
}

#[test]
fn toolhead_series_is_empty_for_an_empty_grid() {
    let s = toolhead_series(&single_cubic(), &[]).unwrap();
    assert!(s.x.is_empty() && s.jx.is_empty() && s.kappa.is_empty());
}
