//! Regressions around curvature pinches: corners whose apex pins the profile
//! exactly on the curvature ceiling (disk rail = 0) and tiny debris features
//! whose curvature spikes are jerk-infeasible at printing speed.

use crossbeam_channel::unbounded;
use geometry::path::lowering::PositionProfile;
use motion_pipeline::fit_stage::FitStage;
use motion_pipeline::planner::Planner;
use motion_pipeline::types::PlannedMove;
use motion_pipeline::{PlannedItem, StreamConfig, StreamInput};
use pipeline_snapshot::waypoints::parse_gcode;
use pipeline_snapshot::{
    SNAPSHOT_MAX_BUFFER_MOVES, TRAJECTORY_FIT_TOL_ACCEL_MM_S2, TRAJECTORY_FIT_TOL_MM,
    VELOCITY_INTEGRATION_TOL, build_moves,
};

fn stream_case(
    case_rel_path: &str,
    max_velocity: f64,
    max_accel: f64,
    scv: f64,
    max_jerk: f64,
) -> Vec<PlannedMove> {
    let path = format!(
        "{}/../../snapshots/cases/{case_rel_path}",
        env!("CARGO_MANIFEST_DIR")
    );
    let text = std::fs::read_to_string(&path).expect("read case gcode");
    let waypoints = parse_gcode(&text, max_velocity, max_accel).expect("parse");
    let corner_deviation = geometry::corner_deviation_from_scv(scv, max_accel);
    let limits =
        geometry::VelocityLimits::try_new(max_velocity, max_accel, corner_deviation, max_jerk)
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

fn piece_extreme(coeffs: &[f64], h: f64, deriv: usize) -> f64 {
    let eval = |tau: f64| -> f64 {
        (deriv..coeffs.len())
            .map(|k| {
                let scale: usize = (k - deriv + 1..=k).product();
                coeffs[k] * scale as f64 * tau.powi((k - deriv) as i32)
            })
            .sum()
    };
    (0..=16)
        .map(|k| eval(h * k as f64 / 16.0).abs())
        .fold(0.0, f64::max)
}

/// Max per-axis (accel, jerk) over every lowered piece of every planned move.
fn lowered_extremes(planned: &[PlannedMove]) -> (f64, f64) {
    let fit_tol = motion_pipeline::lowering::FitTol {
        pos_mm: TRAJECTORY_FIT_TOL_MM,
        vel_mm_s: f64::INFINITY,
        accel_mm_s2: TRAJECTORY_FIT_TOL_ACCEL_MM_S2,
    };
    let chains: Vec<trajectory::CompiledChain> = vec![trajectory::CompiledChain::default(); 4];
    let (mut max_a, mut max_j) = (0.0_f64, 0.0_f64);
    for pm in planned {
        let Some(seg) = pm.geometry.segment.spatial.as_ref() else {
            continue;
        };
        let mut start_pos = vec![0.0_f64; 4];
        start_pos[..3].copy_from_slice(&seg.point_at(0.0));
        let (pieces, _) = motion_pipeline::lowering::lower_move_pieces(
            &pm.geometry,
            &pm.velocity,
            0.0,
            &start_pos,
            fit_tol,
            &chains,
            None,
        )
        .expect("lower");
        for ps in pieces.iter().take(3) {
            for p in ps {
                let h = p.u_end - p.u_start;
                max_a = max_a.max(piece_extreme(&p.coeffs, h, 2));
                max_j = max_j.max(piece_extreme(&p.coeffs, h, 3));
            }
        }
    }
    (max_a, max_j)
}

/// The neptune_cube "discontinuity" case under printer limits (v=100,
/// a=1000, scv=8, j=1e6), streamed through the fit + planner stages: its
/// rounded corners are fitted into clothoid-arc-clothoid moves whose apex
/// pins the profile exactly on the curvature ceiling. The backward
/// envelope's flight landing coming off such a pinch used to adopt the
/// ascending cap chord's slope unclamped, teleporting the envelope one
/// super-disk velocity step up; the binding-envelope feasibility gate then
/// rejected the run and the planner failed on ordinary sliced gcode
/// (Diverged at waypoint 253).
#[test]
fn discontinuity_case_plans_under_printer_limits() {
    let planned = stream_case("neptune_cube/discontinuity.gcode", 100.0, 1000.0, 8.0, 1e6);
    assert!(
        planned.len() > 200,
        "expected the whole file planned, got {}",
        planned.len()
    );
}

/// The clothoid/90_deg_accelerating case (v=300, a=1000, scv=5, j=6e5): the
/// corner's exit clothoid tightens fast enough that its normal jerk demand
/// `sigma*v^3` alone exceeds the vector jerk budget. `effective_jerk` used to
/// freeze the tangential acceleration there, so the profile arrived at the
/// following straight's feed ceiling carrying an acceleration no jerk-limited
/// landing could shed — the snap kick landed inside the straight's first
/// sample window and the quintic reconstruction rang 30% past the
/// acceleration budget. With full *lowering* authority in the already
/// jerk-infeasible regime the landing resolves on the clothoid and every
/// lowered piece stays within budget.
#[test]
fn jerk_starved_clothoid_lowers_within_accel_budget() {
    let planned = stream_case(
        "clothoid/90_deg_accelerating.gcode",
        300.0,
        1000.0,
        5.0,
        600_000.0,
    );
    let (max_a, _) = lowered_extremes(&planned);
    let budget = 1000.0 + TRAJECTORY_FIT_TOL_ACCEL_MM_S2;
    assert!(max_a <= budget, "lowered accel {max_a:.1} exceeds {budget}");
}

/// The facet_debris/debris_corners case (v=300, a=3000, scv=9, j=1e5):
/// micrometre debris facets fit into clothoids whose curvature-cap descent is
/// steeper than the acceleration rail on both flanks of a one-node notch, so
/// neither pass can land on it tangentially and both fly a hair over the
/// notch bottom. Unbounded raising authority in the jerk-starved regime once
/// let the flight build toward the rail across the tiny feature and arrive
/// higher above the notch, and the emission's cap clamp then carved a
/// 100x-jerk cliff into one 12 us sample window; the bankrupt rail-follow's
/// velocity-doubling bound now caps what one substep may build, so the
/// fly-over arrives near the notch bottom while the debris field is still
/// crossed at the disk-planned speed instead of the old frozen crawl
/// (~16.6 mm/s vs ~13.1). The residual snap scales with that fly-over
/// speed, which is why the pinned bound sits above the frozen-era value.
///
/// The jerk bound is the current fly-over residual with headroom, far above
/// the configured 1e5 limit: the notch's curvature spike is jerk-infeasible
/// at any printing speed (the normal jerk of traversing a 3 um kappa=11
/// facet at 16 mm/s exceeds the budget geometrically), so a bounded snap
/// residual at the notch node is the designed outcome, not a bug — this test
/// pins its magnitude so it cannot silently grow two orders again.
#[test]
fn debris_notch_fly_over_keeps_bounded_jerk() {
    let planned = stream_case("facet_debris/debris_corners.gcode", 300.0, 3000.0, 9.0, 1e5);
    let (max_a, max_j) = lowered_extremes(&planned);
    let budget = 3000.0 + TRAJECTORY_FIT_TOL_ACCEL_MM_S2;
    assert!(max_a <= budget, "lowered accel {max_a:.1} exceeds {budget}");
    assert!(
        max_j <= 5e7,
        "lowered jerk {max_j:.3e} exceeds the pinned debris-notch residual"
    );
}
