use super::*;
use crate::fitter::{CornerFitConfig, fit_corners};
use crate::frontend::{ExtruderCaps, MoveContext, VelocityLimits, line_move};
use crate::path::lowering::PositionProfile;
use crate::path::{Arc, CurvatureProfile, Line};
use crate::segment::SourceRange;
use crate::step_capacity::{FollowerStepLoad, McuStepLoad};
use crate::velocity::{VelocityPlanParams, plan_velocity_warm_start, warm_start_params};

const COREXY: [[f64; 3]; 3] = [[1.0, 1.0, 0.0], [1.0, -1.0, 0.0], [0.0, 0.0, 1.0]];
const E_AXIS: usize = 3;
const XY_SHARE: f64 = 1.0 / 1000.0;
const UNCAPPED: (f64, f64) = (1000.0, 20_000.0);

fn mcu(name: &str, lane_s_per_mm: [f64; 3], e_share: f64) -> McuStepLoad {
    McuStepLoad {
        mcu: name.to_string(),
        lane_s_per_mm,
        followers: vec![FollowerStepLoad {
            axis_index: E_AXIS,
            s_per_mm: e_share,
        }],
    }
}

fn corexy(mcus: Vec<McuStepLoad>) -> StepCapacity {
    StepCapacity {
        lane_weights: COREXY,
        mcus,
    }
}

fn one_board() -> StepCapacity {
    corexy(vec![mcu("mcu", [XY_SHARE, XY_SHARE, 0.0], 0.0)])
}

fn line(to: [f64; 3]) -> Segment {
    Segment::Line(Line::try_new([0.0; 3], to).unwrap())
}

fn demand_along(capacity: &StepCapacity, segment: &Segment) -> MotorDemand {
    MotorDemand {
        lane_leans: capacity
            .lane_weights
            .map(|weights| segment.heading_lean_along(weights)),
        extrusion: None,
    }
}

fn capped(capacity: &StepCapacity, demand: MotorDemand, length: f64) -> (f64, f64) {
    step_capacity_capped(capacity, demand, length, 0.0, UNCAPPED).0
}

#[test]
fn a_pure_x_move_on_corexy_steps_both_motors() {
    let capacity = one_board();
    let x = line([100.0, 0.0, 0.0]);
    assert_eq!(demand_along(&capacity, &x).lane_leans, [1.0, 1.0, 0.0]);
    let (ceiling, accel) = capped(&capacity, demand_along(&capacity, &x), 100.0);
    assert!((ceiling - 1.0 / (2.0 * XY_SHARE)).abs() < 1e-9, "{ceiling}");
    assert_eq!(accel, UNCAPPED.1);
}

#[test]
fn a_diagonal_on_corexy_steps_one_motor_at_root_two() {
    let capacity = one_board();
    let diagonal = line([100.0, 100.0, 0.0]);
    let leans = demand_along(&capacity, &diagonal).lane_leans;
    assert!((leans[0] - 2.0_f64.sqrt()).abs() < 1e-12);
    assert!(leans[1].abs() < 1e-12);
    let (ceiling, _) = capped(&capacity, demand_along(&capacity, &diagonal), 141.0);
    let exact = 1.0 / (2.0_f64.sqrt() * XY_SHARE);
    assert!(
        (ceiling - exact).abs() < 1e-9 * exact,
        "{ceiling} vs {exact}"
    );
}

#[test]
fn a_move_within_capacity_is_left_alone() {
    let capacity = corexy(vec![mcu("mcu", [1e-5, 1e-5, 0.0], 0.0)]);
    let x = line([100.0, 0.0, 0.0]);
    let (limits, overload) =
        step_capacity_capped(&capacity, demand_along(&capacity, &x), 100.0, 0.0, UNCAPPED);
    assert_eq!(limits, UNCAPPED);
    assert_eq!(overload, None);
}

#[test]
fn a_binding_cap_reports_the_demand_it_refused() {
    let capacity = one_board();
    let x = line([100.0, 0.0, 0.0]);
    let (_, overload) =
        step_capacity_capped(&capacity, demand_along(&capacity, &x), 100.0, 0.0, UNCAPPED);
    let overload = overload.expect("the cap binds");
    assert_eq!(overload.mcu_index, 0);
    let demand = UNCAPPED.0 * 2.0 * XY_SHARE;
    assert!((overload.demand_over_capacity - demand).abs() < 1e-12);
}

#[test]
fn pressure_advance_spends_capacity_while_accelerating() {
    let (ratio, k, e_share) = (0.05, 0.04, 1.0 / 60.0);
    let capacity = corexy(vec![mcu("toolboard", [0.0; 3], e_share)]);
    let rate = LinearRate {
        per_velocity: ratio,
        per_accel: ratio * k,
    };
    let x = line([100.0, 0.0, 0.0]);
    let demand = MotorDemand {
        extrusion: Some((E_AXIS, rate)),
        ..demand_along(&capacity, &x)
    };
    for length in [0.5, 100.0] {
        let (ceiling, accel) = capped(&capacity, demand, length);
        let used = e_share * rate.at(ceiling, accel);
        assert!(used <= 1.0 + 1e-12, "{used}");
        assert!(ceiling < UNCAPPED.0 || accel < UNCAPPED.1);
    }
    let no_advance = MotorDemand {
        extrusion: Some((
            E_AXIS,
            LinearRate {
                per_velocity: ratio,
                per_accel: 0.0,
            },
        )),
        ..demand
    };
    assert_eq!(
        capped(&capacity, no_advance, 100.0),
        (UNCAPPED.0, UNCAPPED.1),
        "50 mm/s of filament fits a 60 mm/s extruder motor"
    );
}

#[test]
fn every_mcu_is_held_and_the_busiest_is_reported() {
    let capacity = corexy(vec![
        mcu("mcu", [XY_SHARE, XY_SHARE, 0.0], 0.0),
        mcu("toolboard", [0.0, 0.0, 1.0 / 20.0], 0.0),
    ]);
    let climbing = line([100.0, 0.0, 10.0]);
    let demand = demand_along(&capacity, &climbing);
    let ((ceiling, _), overload) =
        step_capacity_capped(&capacity, demand, climbing.s_len(), 0.0, UNCAPPED);
    let used: Vec<f64> = mcu_rates(&capacity, demand)
        .map(|rate| rate.at(ceiling, UNCAPPED.1))
        .collect();
    assert!(used.iter().all(|&share| share <= 1.0 + 1e-12), "{used:?}");
    assert!(
        used.iter().any(|&share| (share - 1.0).abs() < 1e-9),
        "{used:?}"
    );
    assert_eq!(overload.unwrap().mcu_index, 1);
}

#[test]
fn a_curve_is_bounded_by_its_steepest_heading() {
    let capacity = one_board();
    let arc = Segment::Arc(
        Arc::try_new([0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], 2.0, 0.0, 1.5).unwrap(),
    );
    let leans = demand_along(&capacity, &arc).lane_leans;
    let samples = 10_000;
    for (lane, weights) in COREXY.iter().enumerate() {
        let sampled = (0..=samples)
            .map(|i| arc.s_len() * f64::from(i) / f64::from(samples))
            .map(|s| crate::vec3::dot(*weights, arc.heading_at(s)).abs())
            .fold(0.0, f64::max);
        assert!(leans[lane] >= sampled - 1e-12, "lane {lane}");
        assert!(leans[lane] <= sampled + 1e-6, "lane {lane}");
    }
    assert!((leans[1] - 2.0_f64.sqrt()).abs() < 1e-12);
}

#[test]
fn the_planner_never_plans_a_move_past_its_mcu_capacity() {
    let capacity = one_board();
    let limits = VelocityLimits::try_new(1000.0, 20_000.0, 0.0, f64::INFINITY).unwrap();
    let m = line_move(
        [0.0; 3],
        [200.0, 0.0, 0.0],
        0.0,
        MoveContext {
            extruder_axis: E_AXIS,
            feedrate_mm_s: 1000.0,
            limits,
            source: SourceRange {
                start_line: 0,
                end_line: 0,
            },
        },
    )
    .unwrap();
    let fitted = fit_corners(std::slice::from_ref(&m), CornerFitConfig::default()).unwrap();
    let plan = |step_capacity| {
        plan_velocity_warm_start(
            &fitted,
            VelocityPlanParams {
                step_capacity,
                ..warm_start_params(1e-7)
            },
        )
        .unwrap()
    };
    let capped = plan(Some(&capacity));
    let free = plan(None);
    let cap = 1.0 / (2.0 * XY_SHARE);
    assert!(
        (capped.moves[0].peak_v - cap).abs() < 1e-6,
        "{}",
        capped.moves[0].peak_v
    );
    assert!(free.moves[0].peak_v > cap);
    assert!(capped.moves[0].step_capacity_overload.is_some());
    assert_eq!(free.moves[0].step_capacity_overload, None);
}

#[test]
fn an_extrusion_without_extruder_caps_on_a_capped_mcu_fails_loudly() {
    let capacity = corexy(vec![mcu("mcu", [XY_SHARE, XY_SHARE, 0.0], 1.0 / 300.0)]);
    let limits = VelocityLimits::try_new(100.0, 2000.0, 0.0, f64::INFINITY).unwrap();
    let m = line_move(
        [0.0; 3],
        [10.0, 0.0, 0.0],
        0.5,
        MoveContext {
            extruder_axis: E_AXIS,
            feedrate_mm_s: 100.0,
            limits,
            source: SourceRange {
                start_line: 7,
                end_line: 7,
            },
        },
    )
    .unwrap();
    let result = std::panic::catch_unwind(|| {
        MotorDemand::of(&m, m.segment.spatial.as_ref(), 0.05, &capacity)
    });
    assert!(result.is_err());
}

#[test]
fn an_extruding_move_with_caps_carries_its_advance_into_the_demand() {
    let capacity = corexy(vec![mcu("mcu", [XY_SHARE, XY_SHARE, 0.0], 1.0 / 300.0)]);
    let caps = ExtruderCaps {
        axis_index: E_AXIS,
        max_velocity_mm_s: f64::INFINITY,
        max_accel_mm_s2: f64::INFINITY,
        advance_slope_s: 0.03,
        advance_curvature_s2_per_mm: 0.0,
        path_jerk_per_accel_hz: [0.0; 8],
        path_jerk_per_accel_step_hz: [0.0; 8],
        unshaped_leaders: [false; 3],
        own_jerk_per_accel_hz: f64::INFINITY,
        instantaneous_corner_velocity_mm_s: 1.0,
        leader_spill: crate::frontend::LeaderSpillId::UNSHAPED,
        leader_kernel_reach_s: 0.0,
        rest_settle: crate::frontend::RestSettleId::UNSHAPED,
    };
    let limits = VelocityLimits::try_new(100.0, 2000.0, 0.0, f64::INFINITY)
        .unwrap()
        .with_extruder_caps(Some(caps))
        .unwrap();
    let m = line_move(
        [0.0; 3],
        [10.0, 0.0, 0.0],
        0.5,
        MoveContext {
            extruder_axis: E_AXIS,
            feedrate_mm_s: 100.0,
            limits,
            source: SourceRange {
                start_line: 0,
                end_line: 0,
            },
        },
    )
    .unwrap();
    let demand = MotorDemand::of(&m, m.segment.spatial.as_ref(), 0.05, &capacity);
    let (axis, rate) = demand.extrusion.unwrap();
    assert_eq!(axis, E_AXIS);
    assert!(rate.per_velocity >= 0.05);
    assert!((rate.per_accel - rate.per_velocity * 0.03).abs() < 1e-15);
}
