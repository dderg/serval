//! A ratio ramp a rest narrows holds the extruder within its velocity limit.
//! The width field's envelope falls to zero at a rest of the leaders, so a
//! ratio step a few ramp widths short of one is spread over a window
//! narrower than the `V·τ/2` its rule derived. The ramp then reads steeper
//! than the rule's `τ`, and the advance turns it into extruder velocity past
//! the limit; the planner has to slow the toolhead at the step instead.
//!
//! Lines 1191–1213 of `trident_follower_cache_regression` on the kernel-free
//! fast bench config: a travel at 1000 mm/s joins an extrusion 10.4 mm
//! before the stream's end rest, crossed at 485 mm/s. The rule derived
//! τ = 21.7 ms for a 5.28 mm window, the rest's cone left it about half
//! that, and the extruder reached 58.95 mm/s against its 50 mm/s cap.

use corpus::extruder::ExtruderExcess;

mod corpus;

const TRAVEL_INTO_EXTRUSION_BEFORE_REST: &str = "G90
M83
G1 X126.231 Y158.935 F60000
G1 F24766.062
G1 X127.301 Y158.787 E.03805
G1 X128.652 Y158.513 E.0486
G1 X129.585 Y158.272 E.03397
G1 X130.206 Y158.083 E.0229
G1 X141.219 Y147.071 E.54903
G1 X141.721 Y147.875 E.03342
G1 X141.78 Y147.999 E.00484
G1 X141.801 Y148.094 E.00343
G1 X141.787 Y148.243 E.00526
G1 X141.716 Y148.401 E.00613
G1 X141.222 Y149.2 E.03312
G1 X130.22 Y138.198 E.5485
G1 X129.562 Y137.99 E.02435
G1 X128.536 Y137.73 E.03728
G1 X136.438 Y141.452 F60000
G1 F31837.476
G1 X137.653 Y142.428 E.04273
G1 X138.581 Y143.28 E.03454
G1 X139.524 Y144.264 E.03737
G1 X140.333 Y145.219 E.03432
G1 X141.085 Y146.221 E.03436
G1 X141.767 Y147.257 E.03401
G1 X142.047 Y147.72 E.01484
";

/// The fast bench config without its extruder kernel: pressure advance
/// alone on `e`, held to 20000 mm/s² and the extrude-only 50 mm/s.
const KERNEL_FREE_FAST: &str = "[mcu]
serial: /dev/null

[printer]
max_velocity: 2000
max_accel: 50000
max_jerk: 0
corner_deviation: 0.5
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
position_min: 0
position_endstop: 0
position_max: 300
endstop_pin: ^PA9
homing_speed: 20
post_processors: shaper_x

[axis y]
position_min: 0
position_endstop: 0
position_max: 300
endstop_pin: ^PA10
homing_speed: 20
post_processors: shaper_y

[axis z]
position_min: -5
position_endstop: 0
position_max: 250
endstop_pin: ^PA15
homing_speed: 5

[axis e]
follows: x, y, z
motors: extruder
post_processors: pa
max_accel: 20000

[post_processor pa]
type: linear_pressure_advance
k: 0.03

[post_processor shaper_x]
type: smooth_mzv
frequency_hz: 50

[post_processor shaper_y]
type: smooth_zv
frequency_hz: 44

[motor a]
drive: stepper
step_pin: PB12
dir_pin: PB13
microsteps: 16
rotation_distance: 40

[motor b]
drive: stepper
step_pin: PB14
dir_pin: PB15
microsteps: 16
rotation_distance: 40

[motor z]
drive: stepper
step_pin: PB6
dir_pin: PB7
microsteps: 16
rotation_distance: 8

[motor extruder]
drive: stepper
step_pin: PB8
dir_pin: PB9
microsteps: 16
rotation_distance: 7.73

[extruder]
axis: e
nozzle_diameter: 0.400
filament_diameter: 1.750
heater_pin: PA1
sensor_type: temperature_mcu
min_temp: 0
max_temp: 250
min_extrude_temp: 0
control: pid
pid_kp: 30.356
pid_ki: 1.857
pid_kd: 124.081
max_extrude_only_velocity: 50
max_extrude_only_accel: 5000
";

#[test]
fn a_ratio_step_a_rest_narrows_holds_the_extruder_velocity_limit() {
    let config =
        std::env::temp_dir().join(format!("rest_narrowed_ramp_{}.cfg", std::process::id()));
    std::fs::write(&config, KERNEL_FREE_FAST).expect("temp config");
    let snapshot = corpus::plan(
        "rest_narrowed_ramp",
        TRAVEL_INTO_EXTRUSION_BEFORE_REST,
        &config,
    )
    .expect("the excerpt plans motion");
    let audit =
        corpus::extruder::extruder_audit(&snapshot, TRAVEL_INTO_EXTRUSION_BEFORE_REST, &config)
            .expect("the config limits its extruder");
    std::fs::remove_file(&config).expect("temp config removed");
    let over_velocity: Vec<String> = audit
        .excesses
        .iter()
        .filter(|excess| matches!(excess, ExtruderExcess::Velocity { .. }))
        .map(ToString::to_string)
        .collect();
    assert!(
        over_velocity.is_empty(),
        "the ramp a rest narrows leaves the extruder's velocity limit:\n{}",
        over_velocity.join("\n")
    );
}
