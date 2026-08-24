//! Regressions around curvature pinches: corners whose apex pins the profile
//! exactly on the curvature ceiling (disk rail = 0) and tiny debris features
//! whose curvature spikes collapse the feasible speed at printing feeds.

use crossbeam_channel::unbounded;
use geometry::path::lowering::PositionProfile;
use motion_pipeline::fit_stage::FitStage;
use motion_pipeline::planner::Planner;
use motion_pipeline::types::PlannedMove;
use motion_pipeline::{BaseItem, Lowerer, PlannedItem, StreamConfig, StreamInput};
use pipeline_snapshot::waypoints::parse_gcode;
use pipeline_snapshot::{
    SNAPSHOT_MAX_BUFFER_MOVES, TRAJECTORY_FIT_TOL_ACCEL_MM_S2, TRAJECTORY_FIT_TOL_MM,
    VELOCITY_INTEGRATION_TOL, build_moves, collect_trajectory_pieces,
};

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
    let waypoints = parse_gcode(&text, max_velocity, max_accel).expect("parse");
    let corner_deviation = geometry::corner_deviation_from_scv(scv, max_accel);
    let limits =
        geometry::VelocityLimits::try_new(max_velocity, max_accel, corner_deviation, f64::INFINITY)
            .expect("limits");
    let moves = build_moves(&waypoints, limits).expect("moves");

    let config = StreamConfig {
        corner: geometry::CornerFitConfig::default(),
        integration_tol: VELOCITY_INTEGRATION_TOL,
        max_extrude_only_velocity_mm_s: 100.0,
        max_extrude_only_accel_mm_s2: 1000.0,
        fit_tol_mm: TRAJECTORY_FIT_TOL_MM,
        fit_tol_accel_mm_s2: TRAJECTORY_FIT_TOL_ACCEL_MM_S2,
        max_buffer_moves: SNAPSHOT_MAX_BUFFER_MOVES,
        limits,
    };
    let (fitted_tx, fitted_rx) = unbounded();
    let (planned_tx, planned_rx) = unbounded();
    let mut fit = FitStage::new(config.corner).into_driver(fitted_tx);
    let mut planner = Planner::new(config);
    let mut planned: Vec<PlannedMove> = Vec::new();
    let pump = |planner: &mut Planner, planned: &mut Vec<PlannedMove>| {
        while let Ok(item) = fitted_rx.try_recv() {
            assert!(planner.feed(item, &planned_tx), "planner rejected input");
        }
        while let Ok(item) = planned_rx.try_recv() {
            if let PlannedItem::Move(m) = item {
                planned.push(m);
            }
        }
    };
    for m in moves {
        assert!(fit.feed(StreamInput::from(m)));
        pump(&mut planner, &mut planned);
    }
    assert!(fit.finish());
    pump(&mut planner, &mut planned);
    assert!(planner.finish(&planned_tx), "planner failed to finish");
    pump(&mut planner, &mut planned);
    assert!(!planned.is_empty());
    planned
}

fn piece_derivative(coeffs: &[f64], tau: f64, deriv: usize) -> f64 {
    (deriv..coeffs.len())
        .map(|k| {
            let scale: usize = (k - deriv + 1..=k).product();
            coeffs[k] * scale as f64 * tau.powi((k - deriv) as i32)
        })
        .sum()
}

/// Max per-axis acceleration the lowered carrier itself commands — the
/// trajectory the firmware executes, sampled through the segments' own
/// evaluator rather than through the snapshot's polynomial reconstruction.
/// With jerk unbounded the executed acceleration steps, so a reconstruction
/// spanning a step rings above the step's own height; the carrier is what the
/// planner's per-axis budget governs.
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

/// Every snapshot row is a real window carrying finite coefficients, and
/// consecutive rows agree in position and velocity: a lowering that rang
/// through a numerical sliver would report neither.
fn assert_rows_are_finite_and_continuous(planned: &[PlannedMove]) {
    let segments = lower_run(planned);
    let traj = collect_trajectory_pieces(&segments);
    for (lane, pieces) in [("x", &traj.x), ("y", &traj.y), ("z", &traj.z)] {
        assert!(!pieces.is_empty(), "{lane}: lane must carry rows");
        for p in pieces {
            assert!(p.iter().all(|c| c.is_finite()), "{lane}: {p:?}");
            assert!(
                p[1] - p[0] >= 2e-9,
                "{lane}: row spans {:e}s, under device resolution",
                p[1] - p[0]
            );
        }
        for w in pieces.windows(2) {
            let h = w[0][1] - w[0][0];
            for deriv in 0..2 {
                let end = piece_derivative(&w[0][2..], h, deriv);
                let start = piece_derivative(&w[1][2..], 0.0, deriv);
                let jump = (end - start).abs();
                assert!(
                    jump <= 1e-6 * (1.0 + end.abs().max(start.abs())),
                    "{lane}: derivative {deriv} jumps {jump:e} at t={}",
                    w[0][1]
                );
            }
        }
    }
}

fn lower_run(planned: &[PlannedMove]) -> Vec<trajectory::ContinuousSegment> {
    let mut home = vec![0.0_f64; 4];
    if let Some(seg) = planned
        .iter()
        .find_map(|pm| pm.geometry.segment.spatial.as_ref())
    {
        home[..3].copy_from_slice(&seg.point_at(0.0));
    }
    let (lowered_tx, lowered_rx) = unbounded();
    let mut lowerer = Lowerer::new(trajectory::AxisChainSet::default(), home, 0.0);
    for pm in planned {
        let item = PlannedItem::Move(PlannedMove {
            geometry: pm.geometry.clone(),
            velocity: pm.velocity.clone(),
        });
        assert!(lowerer.feed(item, &lowered_tx), "lowerer rejected input");
    }
    drop(lowered_tx);
    lowered_rx
        .into_iter()
        .filter_map(|item| match item {
            BaseItem::Seg(seg) => Some(seg.segment),
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
    assert_rows_are_finite_and_continuous(&planned);
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
    assert_rows_are_finite_and_continuous(&planned);
}
