use super::planner_api::gcode_line_move;

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

fn planner_config(text: &str) -> planner_config::PlannerConfig {
    let doc = config_doc::Document::parse(text, "test.cfg").unwrap();
    let (settings, _) = planner_config::from_doc::read_motion_settings(&doc).unwrap();
    planner_config::from_doc::planner_config_from_settings(&settings).unwrap()
}

#[test]
fn a_travel_plans_under_the_extruder_caps_like_the_extrusions_it_joins() {
    let cfg = planner_config(&format!("{CARTESIAN}{RAMPED_EXTRUDER}"));
    let caps = motion_pipeline::live_config::extruder_caps(&cfg).unwrap();
    let extrusion =
        gcode_line_move(&cfg, &caps, [0.0; 3], [10.0, 0.0, 0.0], 0.4, 200.0, 1).unwrap();
    let travel = gcode_line_move(
        &cfg,
        &caps,
        [10.0, 0.0, 0.0],
        [0.0, 10.0, 0.0],
        0.0,
        200.0,
        2,
    )
    .unwrap();
    assert!(extrusion.limits.extruder.is_some());
    assert_eq!(travel.limits.extruder, extrusion.limits.extruder);
}

#[test]
fn without_an_extruder_axis_a_travel_has_no_extruder_caps_and_an_extrusion_is_refused() {
    let cfg = planner_config(CARTESIAN);
    let travel = gcode_line_move(&cfg, &[], [0.0; 3], [10.0, 0.0, 0.0], 0.0, 200.0, 1).unwrap();
    assert_eq!(travel.limits.extruder, None);
    let err = gcode_line_move(&cfg, &[], [0.0; 3], [10.0, 0.0, 0.0], 0.4, 200.0, 2).unwrap_err();
    assert!(err.contains("no [axis e] is declared"), "{err}");
}
