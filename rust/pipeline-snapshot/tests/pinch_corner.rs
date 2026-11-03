//! Regressions around curvature pinches: corners whose apex pins the profile
//! exactly on the curvature ceiling (disk rail = 0) and tiny debris features
//! whose curvature spikes collapse the feasible speed at printing feeds.

use std::sync::Arc;

use geometry::path::lowering::PositionProfile;
use motion_pipeline::fit_stage::FitStage;
use motion_pipeline::planner::Planner;
use motion_pipeline::types::PlannedMove;
use motion_pipeline::{BaseItem, Lowerer, PlannedItem, live_config};
use pipeline_snapshot::waypoints::parse_gcode;
use pipeline_snapshot::{CartesianLimits, ExactTrajectory, PlannerConfig, SampleSide, plan_steps};

fn stream_case(
    case_rel_path: &str,
    max_velocity: f64,
    max_accel: f64,
    scv: f64,
) -> Vec<PlannedMove> {
    let path = format!(
        "{}/../../snapshots/cases/{case_rel_path}",
        env!("CARGO_MANIFEST_DIR")
    );
    let text = std::fs::read_to_string(&path).expect("read case gcode");
    let waypoints = parse_gcode(
        &text,
        &CartesianLimits::without_z_caps(max_velocity, max_accel, f64::INFINITY, 0.0),
    )
    .expect("parse");
    let planner_config = PlannerConfig {
        cartesian: CartesianLimits::without_z_caps(
            max_velocity,
            max_accel,
            f64::INFINITY,
            geometry::corner_deviation_from_scv(scv, max_accel),
        ),
        max_extrude_only_velocity: Some(100.0),
        max_extrude_only_accel: Some(1000.0),
        ..PlannerConfig::default()
    };
    let limits = live_config::move_limits(&planner_config).expect("limits");
    let steps = plan_steps(&waypoints, limits).expect("steps");

    let config = live_config::stream_config(&planner_config).expect("stream config");
    let mut fit = FitStage::new(config.corner).into_driver();
    let mut planner = Planner::new(config, Arc::default());
    let mut planned = Vec::new();
    let mut collect = |item| {
        if let PlannedItem::Move(m) = item {
            planned.push(m);
        }
        true
    };
    for step in steps {
        assert!(step.feed(|input| fit.feed(input, &mut |item| planner.feed(item, &mut collect))));
    }
    assert!(fit.finish(&mut |item| planner.feed(item, &mut collect)));
    assert!(planner.finish(&mut collect));

    assert!(!planned.is_empty());
    planned
}

/// Maximum acceleration commanded by the exact continuous carrier.
fn carrier_max_accel(planned: &[PlannedMove]) -> f64 {
    let segments = lower_run(planned);
    let mut max_a = 0.0_f64;
    for seg in &segments {
        let samples = 2_000;
        for k in 0..=samples {
            let t = seg.t_start + (seg.t_end - seg.t_start) * (k as f64 / samples as f64);
            for axis in 0..3 {
                let pva = seg
                    .eval_axis(axis, t)
                    .unwrap_or_else(|e| panic!("axis {axis} at t={t} not evaluable: {e}"));
                max_a = max_a.max(pva.acceleration.abs());
            }
        }
    }
    max_a
}

/// Every exposed carrier has finite state, and consecutive carriers agree in
/// position and velocity when evaluated from both sides of their shared instant.
fn assert_carriers_are_finite_and_continuous(planned: &[PlannedMove]) {
    let traj = ExactTrajectory::from_segments(&lower_run(planned))
        .expect("continuous segments carry evaluable exact carriers");
    for (lane, axis) in [("x", 0), ("y", 1), ("z", 2)] {
        let rows = traj.rows(axis);
        assert!(!rows.is_empty(), "{lane}: lane must carry rows");
        for row in rows {
            assert!(
                row.t1 - row.t0 >= 2e-9,
                "{lane}: carrier spans {:e}s, under device resolution",
                row.t1 - row.t0
            );
        }
        for w in rows.windows(2) {
            let seam = w[0].t1;
            let left = eval(&traj, axis, seam, SampleSide::Left);
            let right = eval(&traj, axis, w[1].t0, SampleSide::Right);
            for (name, before, after) in [
                ("position", left.position, right.position),
                ("velocity", left.velocity, right.velocity),
            ] {
                assert!(
                    before.is_finite() && after.is_finite(),
                    "{lane}: non-finite {name} at t={seam}"
                );
                let jump = (before - after).abs();
                assert!(
                    jump <= 1e-6 * (1.0 + before.abs().max(after.abs())),
                    "{lane}: {name} jumps {jump:e} at t={seam}"
                );
            }
        }
    }
}

fn eval(traj: &ExactTrajectory, axis: usize, t: f64, side: SampleSide) -> pipeline_snapshot::Pvaj {
    traj.eval_axis(axis, t, side)
        .unwrap_or_else(|e| panic!("axis {axis} at t={t} is not evaluable: {e}"))
}

fn lower_run(planned: &[PlannedMove]) -> Vec<trajectory::ContinuousSegment> {
    let mut home = vec![0.0_f64; 4];
    if let Some(seg) = planned
        .iter()
        .find_map(|pm| pm.geometry.segment.spatial.as_ref())
    {
        home[..3].copy_from_slice(&seg.point_at(0.0));
    }
    let mut lowered = Vec::new();
    let mut collect = |item| {
        lowered.push(item);
        true
    };
    let mut lowerer = Lowerer::new(
        trajectory::AxisChainSet::default().rest_support(),
        home,
        0.0,
    );
    for pm in planned {
        let item = PlannedItem::Move(PlannedMove {
            geometry: pm.geometry.clone(),
            velocity: pm.velocity.clone(),
        });
        assert!(lowerer.feed(item, &mut collect), "lowerer rejected input");
    }
    lowered
        .into_iter()
        .filter_map(|item| match item {
            BaseItem::Seg(seg) => Some(seg),
            _ => None,
        })
        .collect()
}

/// The neptune_cube "discontinuity" case under printer limits (v=100,
/// a=1000, scv=8), streamed through the fit + planner stages: its rounded
/// corners are fitted into clothoid-arc-clothoid moves whose apex pins the
/// profile exactly on the curvature ceiling. The backward envelope's flight
/// landing coming off such a pinch used to adopt the ascending cap chord's
/// slope unclamped, teleporting the envelope one super-disk velocity step
/// up; the binding-envelope feasibility gate then rejected the run and the
/// planner failed on ordinary sliced gcode (Diverged at waypoint 253).
#[test]
fn discontinuity_case_plans_under_printer_limits() {
    let planned = stream_case("neptune_cube/discontinuity.gcode", 100.0, 1000.0, 8.0);
    assert!(
        planned.len() > 200,
        "expected the whole file planned, got {}",
        planned.len()
    );
}

/// The clothoid/90_deg_accelerating case (v=300, a=1000, scv=5): the corner's
/// exit clothoid tightens fast enough that its normal acceleration demand
/// `kappa*v^2` alone consumes the whole vector budget, so the profile arrives
/// at the following straight's feed ceiling on the rail. The landing used to
/// resolve inside the straight's first sample window; it now resolves on the
/// clothoid, the executed carrier holds the configured per-axis acceleration
/// budget, and every snapshot row is a finite, continuously joined window.
#[test]
fn cap_starved_clothoid_holds_per_axis_accel_budget() {
    let planned = stream_case("clothoid/90_deg_accelerating.gcode", 300.0, 1000.0, 5.0);
    let max_a = carrier_max_accel(&planned);
    let budget = 1000.0 * (1.0 + 1e-9);
    assert!(max_a <= budget, "carrier accel {max_a:.6} exceeds {budget}");
    assert_carriers_are_finite_and_continuous(&planned);
}

/// The facet_debris/debris_corners case (v=300, a=3000, scv=9): micrometre
/// debris facets fit into clothoids whose curvature-cap descent is steeper
/// than the acceleration rail on both flanks of a one-node notch, so neither
/// pass can land on it tangentially and both fly a hair over the notch
/// bottom. Unbounded raising authority once let the flight build toward the
/// rail across the tiny feature and arrive higher above the notch, and the
/// emission's cap clamp then carved an acceleration cliff into one 12 us
/// sample window; the bankrupt rail-follow's velocity-doubling bound now
/// caps what one substep may build, so the fly-over arrives near the notch
/// bottom while the debris field is still crossed at the disk-planned speed
/// instead of the old frozen crawl (~16.6 mm/s vs ~13.1), the executed
/// carrier holds the configured per-axis acceleration budget, and every
/// snapshot row is a finite, continuously joined window.
#[test]
fn debris_notch_fly_over_holds_per_axis_accel_budget() {
    let planned = stream_case("facet_debris/debris_corners.gcode", 300.0, 3000.0, 9.0);
    let max_a = carrier_max_accel(&planned);
    let budget = 3000.0 * (1.0 + 1e-9);
    assert!(max_a <= budget, "carrier accel {max_a:.6} exceeds {budget}");
    assert_carriers_are_finite_and_continuous(&planned);
}
