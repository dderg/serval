//! #947: the retract after a travel starts while the travel's shaped spill
//! still settles within the rest radius. A retract is an extrude-only move,
//! planned under the extrude-only caps (5000 mm/s² here); `[axis e]`'s
//! 2000 mm/s² holds e only while it follows the toolhead, and the travel
//! extrudes nothing, so the follower adds nothing to the retract.

use pipeline_snapshot::SampleSide;

mod corpus;

const PRINTER: &str = "[printer]
max_velocity: 300
max_accel: 47338.141408059026
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
k: 0.01
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
";

const TRAVEL_THEN_HOP: &str = "G90
M83
G1 X150 Y150 Z0.2 F30000
G1 X151.000 Y150.000 F1200
G1 E-0.5 F2400
G1 Z0.600 F600
G1 X152.000 Y150.000 F30000
G1 Z0.200 F600
G1 E0.5 F2400
";

const EXTRUDE_ONLY_ACCEL_MM_S2: f64 = 5000.0;
const PROBE_STEP_S: f64 = 1e-4;

fn toolhead_speed(trajectory: &pipeline_snapshot::ExactTrajectory, t: f64) -> f64 {
    (0..3)
        .map(|axis| {
            trajectory
                .eval_axis(axis, t, SampleSide::Right)
                .expect("the toolhead is evaluable")
                .velocity
        })
        .fold(0.0, |sum: f64, v| v.mul_add(v, sum))
        .sqrt()
}

#[test]
fn a_retract_while_a_travel_settles_is_held_to_the_extrude_only_caps() {
    let config =
        std::env::temp_dir().join(format!("retract_while_settling_{}.cfg", std::process::id()));
    std::fs::write(&config, PRINTER).expect("temp config");
    let snapshot =
        corpus::plan("retract while settling", TRAVEL_THEN_HOP, &config).expect("the hop plans");
    let trajectory = &snapshot.trajectory;
    let probes = ((trajectory.t_end() - trajectory.t_start()) / PROBE_STEP_S).ceil() as usize;
    let retract_starts = (0..probes)
        .map(|probe| trajectory.t_start() + probe as f64 * PROBE_STEP_S)
        .find(|&t| {
            trajectory
                .eval_axis(3, t, SampleSide::Right)
                .expect("e is evaluable")
                .acceleration
                .abs()
                >= EXTRUDE_ONLY_ACCEL_MM_S2 * (1.0 - 1e-9)
        })
        .expect("the retract reaches the extrude-only acceleration cap");
    assert!(
        toolhead_speed(trajectory, retract_starts) > 0.0,
        "the retract at t={retract_starts} must overlap the travel's settle"
    );
    let audit = corpus::extruder::extruder_audit(&snapshot, TRAVEL_THEN_HOP, &config)
        .expect("the config limits its extruder");
    let excesses: Vec<String> = audit.excesses.iter().map(ToString::to_string).collect();
    assert!(
        excesses.is_empty(),
        "{} extruder excesses: {}",
        excesses.len(),
        excesses.join("; ")
    );
}
