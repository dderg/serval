//! Where the shaped axes curve into a stop unlike each other, the leaders
//! cover a few micrometres more path than the tail's straight distance from
//! the rest point. Laid at the next move's ratio, that idle arc stepped the
//! ratio while they still moved at several mm/s, and the rest valley opened
//! there: the kernel-free advance stepped the extruder's velocity by up to
//! 0.47 mm/s at a fit join with no unshaped leader near, and the ratio step
//! itself asked for over 100000 mm/s² of a 20000 mm/s² extruder.

mod corpus;

/// The opening lines of each sim print, through its first stops after an
/// extrusion on the kernel-free fast bench config.
const EXCERPTS: [(&str, usize); 2] = [("voron_first_layers", 100), ("voron_dense_top_layers", 200)];

#[test]
fn a_kernel_free_advance_holds_the_extruder_to_its_limits_into_a_curved_stop() {
    let config = corpus::repo().join("tools/bench/configs/fast.cfg");
    for (print, lines) in EXCERPTS {
        let gcode: String =
            std::fs::read_to_string(corpus::repo().join(format!("tools/sim/gcode/{print}.gcode")))
                .expect("read the sim print")
                .lines()
                .take(lines)
                .flat_map(|line| [line, "\n"])
                .collect();
        let snapshot = corpus::plan(print, &gcode, &config).expect("the excerpt plans");
        let audit = corpus::extruder::extruder_audit(&snapshot, &gcode, &config)
            .expect("the config limits the extruder");
        let excesses: Vec<String> = audit.excesses.iter().map(ToString::to_string).collect();
        assert!(
            excesses.is_empty(),
            "{print}, first {lines} lines: {excesses:#?}"
        );
    }
}

/// #946, from the print-like planner fuzz: x shaped at 102 Hz settles long
/// before y at 30 Hz, so the leaders pass closest to the retract's rest
/// point while still moving at about 5 mm/s, 15 µm of path short of where
/// they rest. The ratio stepped there, the rest valley opened there, and
/// its weight's slope stepped the advanced velocity from -1.19 to 0 mm/s.
const UNEVENLY_SHAPED_PRINTER: &str = "[printer]
max_velocity: 300
max_accel: 12164.490773907943
square_corner_velocity: 0
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
max_accel: 11457.535635176524
[post_processor pa]
type: linear_pressure_advance
k: 0.01
[post_processor shaper_x]
type: smooth_mzv
frequency_hz: 102.11524699612416
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

const FAST_SHORT_EXTRUSION_INTO_A_RETRACT: &str = "G90
M83
G1 X151.637 Y149.234 E0.04226 F1200
G1 X152.476 Y149.779 E0.04226 F1200
G1 X152.517 Y149.681 E0.00447 F12809
G1 E-0.5 F2400
G1 Z0.600 F600
";

#[test]
fn the_ratio_steps_where_unevenly_shaped_leaders_rest_not_where_they_pass_the_rest_point() {
    let config =
        std::env::temp_dir().join(format!("unevenly_shaped_printer_{}.cfg", std::process::id()));
    std::fs::write(&config, UNEVENLY_SHAPED_PRINTER).expect("temp config");
    let gcode = FAST_SHORT_EXTRUSION_INTO_A_RETRACT;
    let snapshot = corpus::plan("fast short extrusion into a retract", gcode, &config)
        .expect("the gcode moves");
    let audit = corpus::extruder::extruder_audit(&snapshot, gcode, &config)
        .expect("the config limits the extruder");
    let excesses: Vec<String> = audit.excesses.iter().map(ToString::to_string).collect();
    assert!(excesses.is_empty(), "{}: {excesses:#?}", audit.summary());
}
