use super::*;
use crate::fitter::{CornerFitConfig, fit_corners};
use crate::frontend::{LeaderSpill, MoveContext, VelocityLimits, line_move};
use crate::path::{Arc, Line};
use crate::segment::SourceRange;
use crate::velocity::{plan_velocity_warm_start, warm_start_params};

const RATIO: f64 = 0.03125;
const X_GAIN: f64 = 1000.0;
const Y_GAIN: f64 = 700.0;
const PLANE_GAIN: f64 = 1200.0;
const GAINS: [f64; 8] = [
    0.0, X_GAIN, Y_GAIN, PLANE_GAIN, 0.0, X_GAIN, Y_GAIN, PLANE_GAIN,
];
const X: [bool; 3] = [true, false, false];
const Y: [bool; 3] = [false, true, false];
const XZ: [bool; 3] = [true, false, true];

fn caps(max_velocity: f64, max_accel: f64, slope: f64) -> ExtruderCaps {
    ExtruderCaps {
        axis_index: 3,
        max_velocity_mm_s: max_velocity,
        max_accel_mm_s2: max_accel,
        advance_slope_s: slope,
        advance_curvature_s2_per_mm: 0.0,
        path_jerk_per_accel_hz: GAINS,
        unshaped_leaders: [false; 3],
        own_jerk_per_accel_hz: f64::INFINITY,
        instantaneous_corner_velocity_mm_s: 1.0,
        leader_spill: LeaderSpill::UNSHAPED,
        leader_kernel_reach_s: 0.0,
    }
}

fn around(ratio_in_reach: f64, axes_in_reach: [bool; 3]) -> ExtrusionBudget {
    ExtrusionBudget {
        ratio_in_reach,
        axes_in_reach,
        rest_holds_s: [0.0; 2],
        rest_room_mm: f64::INFINITY,
    }
}

#[test]
fn ratios_are_budgeted_at_the_quarter_octave_level_above_them() {
    assert_eq!(budgeted_ratio(RATIO), RATIO);
    let level_above = RATIO * libm::pow(2.0, 0.25);
    for ratio in [RATIO * 1.001, RATIO * 1.1, level_above] {
        assert!(
            (budgeted_ratio(ratio) - level_above).abs() < 1e-15,
            "{ratio}"
        );
    }
    assert_eq!(budgeted_ratio(0.0), 0.0);
    assert_eq!(budgeted_ratio(-0.05), 0.0);
}

fn line_x(length: f64) -> Segment {
    Segment::Line(Line::try_new([0.0, 0.0, 0.0], [length, 0.0, 0.0]).unwrap())
}

fn demand_of(caps: &ExtruderCaps, path_gain: f64, kappa_peak: f64) -> Demand {
    Demand {
        ratio: RATIO,
        slope: caps.advance_slope_s,
        curvature: caps.advance_curvature_s2_per_mm,
        path_gain,
        own_gain: caps.own_jerk_per_accel_hz,
        kappa_peak,
        unshaped_share: 0.0,
        ratio_in_reach: RATIO,
    }
}

fn climbing() -> Segment {
    Segment::Line(Line::try_new([0.0; 3], [10.0, 0.0, 1.0]).unwrap())
}

fn unshaped_z(icv: f64) -> ExtruderCaps {
    let mut caps = caps(1000.0, 2000.0, 0.5);
    caps.unshaped_leaders[2] = true;
    caps.instantaneous_corner_velocity_mm_s = icv;
    caps
}

fn velocity_step(caps: &ExtruderCaps, ratio: f64, accel: f64, unshaped_share: f64) -> f64 {
    2.0 * caps.advance_slope_s * ratio * accel * unshaped_share
}

#[test]
fn travel_keeps_its_caps() {
    let limits = (300.0, 20_000.0);
    assert_eq!(
        extruder_capped(
            &line_x(10.0),
            &caps(10.0, 100.0, 0.5),
            &around(0.0, X),
            0.0,
            limits
        ),
        limits
    );
}

#[test]
fn a_straight_extrusion_gets_the_largest_budget_its_advance_jerk_fits() {
    let caps = caps(1000.0, 2000.0, 0.5);
    let (ceiling, accel) = extruder_capped(
        &line_x(50.0),
        &caps,
        &around(RATIO, X),
        0.0,
        (150.0, 3000.0),
    );
    assert_eq!(ceiling, 150.0);
    let exact = 2000.0 / (RATIO * (1.0 + 0.5 * X_GAIN));
    assert!(
        (accel - exact).abs() < 1e-9 * exact,
        "{accel} against {exact}"
    );
}

#[test]
fn only_the_axes_the_path_moves_around_a_segment_lend_their_jerk_gain() {
    let caps = caps(1000.0, 2000.0, 0.5);
    let along_y = Segment::Line(Line::try_new([0.0; 3], [0.0, 50.0, 0.0]).unwrap());
    let (_, alone) = extruder_capped(&along_y, &caps, &around(RATIO, Y), 0.0, (150.0, 3000.0));
    let exact = 2000.0 / (RATIO * (1.0 + 0.5 * Y_GAIN));
    assert!((alone - exact).abs() < 1e-9 * exact);
    let (_, cornering) = extruder_capped(
        &along_y,
        &caps,
        &around(RATIO, [true, true, false]),
        0.0,
        (150.0, 3000.0),
    );
    let exact = 2000.0 / (RATIO * (1.0 + 0.5 * PLANE_GAIN));
    assert!((cornering - exact).abs() < 1e-9 * exact);
}

const PLANE: [bool; 3] = [true, true, false];

#[test]
fn a_curve_also_pays_for_the_tangent_turning_under_its_normal_acceleration() {
    let caps = caps(1000.0, 5000.0, 0.02);
    let arc = Segment::Arc(
        Arc::try_new([0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], 2.0, 0.0, 1.5).unwrap(),
    );
    let kappa = 0.5;
    let (ceiling, accel) =
        extruder_capped(&arc, &caps, &around(RATIO, PLANE), kappa, (300.0, 70_000.0));
    let demand = demand_of(&caps, PLANE_GAIN, kappa);
    assert!(demand.accel(ceiling, accel) <= 5000.0 * (1.0 + 1e-12));
    let straight = extruder_capped(
        &line_x(3.0),
        &caps,
        &around(RATIO, PLANE),
        0.0,
        (300.0, 70_000.0),
    );
    assert!(
        accel < straight.1,
        "turning must cost budget: {accel} vs {}",
        straight.1
    );
}

#[test]
fn the_velocity_cap_gives_way_where_crossing_is_cheaper() {
    let caps = caps(45.0, 1e9, 0.5);
    for length in [0.5, 200.0] {
        let (ceiling, accel) = extruder_capped(
            &line_x(length),
            &caps,
            &around(RATIO, X),
            0.0,
            (150.0, 3000.0),
        );
        let velocity = RATIO * (ceiling + 0.5 * accel);
        assert!(velocity <= 45.0 * (1.0 + 1e-12), "{velocity}");
        assert!(ceiling < 150.0 || accel < 3000.0);
    }
}

#[test]
fn without_an_advance_only_the_base_flow_is_capped() {
    let caps = caps(3.0, 50.0, 0.0);
    let (ceiling, accel) = extruder_capped(
        &line_x(10.0),
        &caps,
        &around(RATIO, X),
        0.0,
        (150.0, 3000.0),
    );
    assert!((ceiling - 3.0 / RATIO).abs() < 1e-9);
    assert!((accel - 50.0 / RATIO).abs() < 1e-9);
}

#[test]
fn an_unshaped_leader_keeps_the_advance_velocity_steps_within_the_instantaneous_corner_velocity() {
    let caps = unshaped_z(0.1);
    let (ceiling, accel) =
        extruder_capped(&climbing(), &caps, &around(RATIO, XZ), 0.0, (150.0, 3000.0));
    assert_eq!(ceiling, 150.0);
    let z_share = 1.0 / 101.0_f64.sqrt();
    let exact = 0.1 / (2.0 * 0.5 * RATIO * z_share);
    assert!(
        (accel - exact).abs() < 1e-9 * exact,
        "{accel} against {exact}"
    );
    assert!(velocity_step(&caps, RATIO, accel, z_share) <= 0.1 * (1.0 + 1e-12));
}

#[test]
fn an_unshaped_leader_spends_the_budget_the_shaped_ones_leave() {
    let caps = unshaped_z(1.0);
    let (_, accel) = extruder_capped(&climbing(), &caps, &around(RATIO, XZ), 0.0, (150.0, 3000.0));
    let exact = 2000.0 / (RATIO * (1.0 + 0.5 * X_GAIN));
    assert!((accel - exact).abs() < 1e-9 * exact);
}

#[test]
fn a_follower_kernel_smooths_an_unshaped_leader_steps_itself() {
    let mut caps = unshaped_z(0.1);
    caps.own_jerk_per_accel_hz = 400.0;
    let (_, accel) = extruder_capped(&climbing(), &caps, &around(RATIO, XZ), 0.0, (150.0, 3000.0));
    let exact = 2000.0 / (RATIO * (1.0 + 0.5 * 400.0));
    assert!((accel - exact).abs() < 1e-9 * exact);
}

#[test]
fn the_planner_holds_an_extruding_move_to_its_cap_and_leaves_travel_alone() {
    let extruder = caps(1000.0, 2000.0, 0.5);
    let limits = VelocityLimits::try_new(150.0, 3000.0, 0.0, f64::INFINITY)
        .unwrap()
        .with_extruder_caps(Some(extruder))
        .unwrap();
    let moves: Vec<_> = [(0.0, 0.0), (RATIO * 50.0, 1.0)]
        .iter()
        .enumerate()
        .map(|(i, &(e_delta, y))| {
            line_move(
                [0.0, y, 0.0],
                [50.0, y, 0.0],
                e_delta,
                MoveContext {
                    extruder_axis: 3,
                    feedrate_mm_s: 150.0,
                    limits,
                    source: SourceRange {
                        start_line: i as u32,
                        end_line: i as u32,
                    },
                },
            )
            .unwrap()
        })
        .collect();
    let peak = |m: &crate::Move| {
        let fitted = fit_corners(std::slice::from_ref(m), CornerFitConfig::default()).unwrap();
        let profile = plan_velocity_warm_start(&fitted, warm_start_params(1e-7)).unwrap();
        profile
            .moves
            .iter()
            .flat_map(|velocity| &velocity.samples)
            .map(|sample| sample.a.abs())
            .fold(0.0, f64::max)
    };
    let cap = 2000.0 / (RATIO * (1.0 + 0.5 * X_GAIN));
    assert!((peak(&moves[1]) - cap).abs() < 1e-6 * cap);
    assert!((peak(&moves[0]) - 3000.0).abs() < 1e-6);
}

#[test]
fn the_planner_caps_the_climbing_travel_after_an_extrusion_on_an_unshaped_leader() {
    let extruder = unshaped_z(1.0);
    let limits = VelocityLimits::try_new(150.0, 3000.0, 0.0, f64::INFINITY)
        .unwrap()
        .with_extruder_caps(Some(extruder))
        .unwrap();
    let context = |line: u32| MoveContext {
        extruder_axis: 3,
        feedrate_mm_s: 150.0,
        limits,
        source: SourceRange {
            start_line: line,
            end_line: line,
        },
    };
    let moves = [
        line_move([0.0; 3], [50.0, 0.0, 0.0], RATIO * 50.0, context(0)).unwrap(),
        line_move([50.0, 0.0, 0.0], [100.0, 0.0, 5.0], 0.0, context(1)).unwrap(),
        line_move([100.0, 0.0, 5.0], [150.0, 0.0, 5.0], 0.0, context(2)).unwrap(),
    ];
    let fitted = fit_corners(&moves, CornerFitConfig::default()).unwrap();
    let profile = plan_velocity_warm_start(&fitted, warm_start_params(1e-7)).unwrap();
    let peak_on = |line: u32| {
        profile
            .moves
            .iter()
            .filter(|velocity| velocity.source.start_line == line)
            .flat_map(|velocity| &velocity.samples)
            .map(|sample| sample.a.abs())
            .fold(0.0, f64::max)
    };
    let cap = 2000.0 / (RATIO * (1.0 + 0.5 * X_GAIN));
    let z_share = 5.0 / 2525.0_f64.sqrt();
    assert!(velocity_step(&extruder, RATIO, cap, z_share) <= 1.0);
    assert!(
        (peak_on(1) - cap).abs() < 1e-6 * cap,
        "{} against {cap}",
        peak_on(1)
    );
    assert!((peak_on(2) - 3000.0).abs() < 1e-6);
}

#[test]
fn the_planner_budgets_a_travel_at_the_extrusion_a_ramp_carries_across_the_blends_beside_it() {
    let mut extruder = caps(1000.0, 2000.0, 0.5);
    extruder.leader_kernel_reach_s = 0.004;
    let limits = VelocityLimits::try_new(150.0, 3000.0, 0.0, f64::INFINITY)
        .unwrap()
        .with_extruder_caps(Some(extruder))
        .unwrap();
    let line = 2.0 * RATIO;
    let pieces = [
        (10.0, line),
        (0.001, RATIO),
        (0.45, 0.0),
        (0.001, RATIO),
        (10.0, line),
    ];
    let mut start = 0.0;
    let moves: Vec<_> = pieces
        .iter()
        .enumerate()
        .map(|(index, &(length, ratio))| {
            let end = start + length;
            let m = line_move(
                [start, 0.0, 0.0],
                [end, 0.0, 0.0],
                ratio * length,
                MoveContext {
                    extruder_axis: 3,
                    feedrate_mm_s: 150.0,
                    limits,
                    source: SourceRange {
                        start_line: index as u32,
                        end_line: index as u32,
                    },
                },
            )
            .unwrap();
            start = end;
            m
        })
        .collect();
    let profile =
        crate::velocity::plan_velocity_stops(&moves, &[false; 5], warm_start_params(1e-7)).unwrap();
    let travel_peak = profile.moves[2]
        .samples
        .iter()
        .map(|sample| sample.a.abs())
        .fold(0.0, f64::max);
    let cap = 2000.0 / (line * (1.0 + 0.5 * X_GAIN));
    assert!(
        (travel_peak - cap).abs() < 1e-6 * cap,
        "{travel_peak} against {cap}"
    );
}
