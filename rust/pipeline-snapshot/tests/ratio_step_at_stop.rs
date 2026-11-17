//! #955: the fitter leaves a ratio step of more than a quarter between two
//! extrusions as a stop, but the leaders' kernels carry the motion either
//! side through it, so the leaders cross the step at speed while braking
//! into the stop. The extruder must stay within `[axis e] max_accel` there.

mod corpus;

/// A kernel-free linear advance behind 30 Hz smooth shapers, the print-like
/// fuzz's machine with its seeds' printer acceleration and advance.
fn printer(max_accel_mm_s2: f64, advance_k_s: f64) -> String {
    format!(
        "[printer]
max_velocity: 300
max_accel: {max_accel_mm_s2}
max_z_velocity: 25
max_z_accel: 100
[kinematics]
type: corexy
axis_x: x
axis_y: y
axis_z: z
a_motors: a
b_motors: b
z_motors: z
[axis x]
post_processors: shaper_x
[axis y]
post_processors: shaper_y
[axis z]
[axis e]
follows: x, y, z
motors: extruder
post_processors: pa
max_accel: 2000
[post_processor pa]
type: linear_pressure_advance
k: {advance_k_s}
[post_processor shaper_x]
type: smooth_mzv
frequency_hz: 30
[post_processor shaper_y]
type: smooth_zv
frequency_hz: 30
[motor a]
drive: stepper
[motor b]
drive: stepper
[motor z]
drive: stepper
[motor extruder]
drive: stepper
[extruder]
axis: e
max_extrude_only_velocity: 50
max_extrude_only_accel: 5000
"
    )
}

/// A 12.8 mm line at 0.0147 mm/mm, then 0.06 mm at 0.0202 to the stream's
/// end: a 37 % step the fitter stops at, 0.06 mm short of the last rest.
const STEP_BEFORE_THE_END: &str = "G90
M83
G1 X150 Y150 Z0.2 F30000
G1 X162.773 Y150.000 E0.18784 F5946
G1 X162.834 Y150.000 E0.00123 F1200
";

/// A 13.9 mm line at 0.0113 mm/mm braking into 0.18 mm at 0.0199.
const BRAKE_INTO_A_WIDER_LINE: &str = "G90
M83
G1 X150 Y150 Z0.2 F30000
G1 X151.000 Y150.000 E0.02000 F1200
G1 X164.907 Y150.000 E0.15719 F6539
G1 X165.088 Y150.000 E0.00361 F1200
";

fn assert_within_limits(name: &str, gcode: &str, config_text: &str) {
    let config = std::env::temp_dir().join(format!("ratio_step_at_stop_{name}_{}.cfg", std::process::id()));
    std::fs::write(&config, config_text).expect("temp config");
    let snapshot = corpus::plan(name, gcode, &config).expect("the stream plans");
    let audit = corpus::extruder::extruder_audit(&snapshot, gcode, &config)
        .expect("the config limits its extruder");
    let excesses: Vec<String> = audit.excesses.iter().map(ToString::to_string).collect();
    assert!(
        excesses.is_empty(),
        "{name}: {} extruder excesses: {}",
        excesses.len(),
        excesses.join("; ")
    );
}

#[test]
fn a_step_the_fitter_stops_at_short_of_the_end_keeps_the_extruder_within_its_limit() {
    assert_within_limits(
        "step_before_the_end",
        STEP_BEFORE_THE_END,
        &printer(55888.07962294791, 0.052704665710604226),
    );
}

#[test]
fn braking_into_a_wider_line_the_fitter_stops_at_keeps_the_extruder_within_its_limit() {
    assert_within_limits(
        "brake_into_a_wider_line",
        BRAKE_INTO_A_WIDER_LINE,
        &printer(5082.167503849642, 0.0382562584349927),
    );
}
