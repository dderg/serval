use super::*;

#[test]
fn stream_config_accepts_unlimited_jerk() {
    let mut cfg = PlannerConfig::default();
    cfg.cartesian.max_jerk = f64::INFINITY;

    assert!(stream_config(&cfg).is_ok());
}

#[test]
fn stream_config_rejects_finite_jerk() {
    let mut cfg = PlannerConfig::default();
    cfg.cartesian.max_jerk = 100_000.0;

    assert_eq!(
        require_unlimited_config_jerk(cfg.cartesian.max_jerk),
        Err(
            "finite [printer] max_jerk is not supported by the continuous trajectory pipeline; set max_jerk: 0"
        )
    );
    assert!(stream_config(&cfg).is_err());
}

const CARTESIAN: &str = "[printer]\n\
    max_velocity: 1000\n\
    max_accel: 10000\n\
    [kinematics]\n\
    type: cartesian\n\
    axis_x: x\n\
    axis_y: y\n\
    axis_z: z\n\
    x_motors: mx\n\
    y_motors: my\n\
    z_motors: mz\n\
    [motor mx]\n\
    drive: stepper\n\
    [motor my]\n\
    drive: stepper\n\
    [motor mz]\n\
    drive: stepper\n\
    [axis x]\n\
    [axis y]\n\
    [axis z]\n";

const RAMPED_EXTRUDER: &str = "[motor me]\n\
    drive: stepper\n\
    [axis e]\n\
    follows: x, y, z\n\
    motors: me\n\
    post_processors: pa\n\
    ratio_ramp_time: 0.015\n\
    [post_processor pa]\n\
    type: linear_pressure_advance\n\
    k: 0.03\n\
    [extruder]\n\
    axis: e\n\
    max_extrude_only_velocity: 50\n\
    max_extrude_only_accel: 5000\n";

fn line(start: [f64; 3], delta: [f64; 3], de: f64) -> GcodeLine {
    GcodeLine {
        start,
        delta,
        de,
        feedrate_mm_s: 200.0,
        seq: 0,
    }
}

fn base(text: &str) -> GcodeMoveBase {
    let cfg = planner_config::from_doc::planner_config_from_text(text).unwrap();
    GcodeMoveBase::of(&cfg, &extruder_caps(&cfg).unwrap()).unwrap()
}

#[test]
fn a_travel_plans_under_the_extruder_caps_like_the_extrusions_it_joins() {
    let base = base(&format!("{CARTESIAN}{RAMPED_EXTRUDER}"));
    let runtime = planner_config::RuntimeLimits::default();
    let extrusion = base
        .line_move(&runtime, line([0.0; 3], [10.0, 0.0, 0.0], 0.4))
        .unwrap()
        .unwrap();
    let travel = base
        .line_move(&runtime, line([10.0, 0.0, 0.0], [0.0, 10.0, 0.0], 0.0))
        .unwrap()
        .unwrap();
    assert!(extrusion.limits.extruder.is_some());
    assert_eq!(travel.limits.extruder, extrusion.limits.extruder);
}

#[test]
fn without_an_extruder_axis_a_travel_has_no_extruder_caps_and_an_extrusion_is_refused() {
    let base = base(CARTESIAN);
    let runtime = planner_config::RuntimeLimits::default();
    let travel = base
        .line_move(&runtime, line([0.0; 3], [10.0, 0.0, 0.0], 0.0))
        .unwrap()
        .unwrap();
    assert_eq!(travel.limits.extruder, None);
    let err = base
        .line_move(&runtime, line([0.0; 3], [10.0, 0.0, 0.0], 0.4))
        .unwrap_err();
    assert!(err.contains("no [axis e] is declared"), "{err}");
}

#[test]
fn runtime_overrides_clamp_against_the_config_and_keep_the_z_and_extruder_caps() {
    let base = base(&format!("{CARTESIAN}{RAMPED_EXTRUDER}"));
    let mut runtime = planner_config::RuntimeLimits::default();
    runtime.set_velocity_cap(Some(5000.0)).unwrap();
    runtime.set_accel_cap(Some(2000.0)).unwrap();
    let limits = base.limits_under(&runtime).unwrap();
    assert_eq!(limits.max_velocity_mm_s, 1000.0);
    assert_eq!(limits.accel_mm_s2, 2000.0);
    assert_eq!(limits.extruder, base.limits().extruder);
    assert_eq!(
        limits.max_z_velocity_mm_s,
        base.limits().max_z_velocity_mm_s
    );
}

#[test]
fn a_line_too_short_to_move_is_no_move() {
    let base = base(&format!("{CARTESIAN}{RAMPED_EXTRUDER}"));
    let moved = base
        .line_move(
            &planner_config::RuntimeLimits::default(),
            line([0.0; 3], [1e-12, 0.0, 0.0], 0.0),
        )
        .unwrap();
    assert!(moved.is_none());
}

#[test]
fn the_offline_plans_carry_no_step_capacity() {
    let mut cfg = PlannerConfig::default();
    cfg.cartesian.max_jerk = f64::INFINITY;

    assert_eq!(stream_config(&cfg).unwrap().step_capacity, None);
}
