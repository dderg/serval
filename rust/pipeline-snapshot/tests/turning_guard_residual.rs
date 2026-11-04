//! A reversal folded by the leaders' kernels: the shaped toolhead turns
//! round a corner at a few mm/s, where the advance's turning term
//! `k·r·a⊥²/|v|` reads the shaped speed, not the planned one. The planner
//! held the planned path's own demand within the extruder's limit, yet the
//! shaped path asked it for up to 33438 mm/s² against 20000; the turning
//! guard now backs the corner's acceleration off until it fits.

mod corpus;

use corpus::extruder::{ExtruderExcess, ToolheadPhase};

/// `tools/sim/gcode/trident_follower_cache_regression.gcode` lines
/// 1191-1213, the last layer's reversal at x≈129, y≈138.
const REVERSAL: &str = "G90
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

/// A fast corexy with a kernel-free linear advance on E: the leaders
/// shaped by a smooth MZV and a smooth ZV, E following them raw.
const FAST_COREXY_WITH_RAW_ADVANCE: &str = "[mcu]
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
fn a_reversal_the_kernels_fold_asks_the_extruder_no_more_than_its_limit() {
    let config = std::env::temp_dir().join(format!(
        "turning_guard_residual_{}.cfg",
        std::process::id()
    ));
    std::fs::write(&config, FAST_COREXY_WITH_RAW_ADVANCE).expect("temp config");
    let snapshot = corpus::plan("reversal", REVERSAL, &config).expect("the excerpt plans");
    let audit = corpus::extruder::extruder_audit(&snapshot, REVERSAL, &config)
        .expect("the config limits the extruder");
    std::fs::remove_file(&config).expect("temp config removed");
    let moving: Vec<&ExtruderExcess> = audit
        .excesses
        .iter()
        .filter(|excess| {
            matches!(
                excess,
                ExtruderExcess::Acceleration {
                    phase: ToolheadPhase::Moving,
                    ..
                }
            )
        })
        .collect();
    assert!(moving.is_empty(), "{moving:#?}");
}
