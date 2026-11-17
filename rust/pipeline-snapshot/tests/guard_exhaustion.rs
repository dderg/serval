//! #964: stretches of a stock Benchy (the one replayed in #420) on the
//! bowden test config, and a scenario fuzz print, that stopped the turning
//! guard.

use std::path::Path;

mod corpus;

const CONFIG: &str = "rust/motion-core/tests/configs/p1_bowden_ramp.cfg";

/// Plans `gcode` on `config` and holds it to the extruder's limits.
fn plans_within_the_extruder_limits(name: &str, gcode: &str, config: &Path) {
    let snapshot = corpus::plan(name, gcode, config).expect("the G-code plans");
    let audit = corpus::extruder::extruder_audit(&snapshot, gcode, config)
        .expect("the config limits its extruder");
    let excesses: Vec<String> = audit.excesses.iter().map(ToString::to_string).collect();
    assert!(
        excesses.is_empty(),
        "{} extruder excesses: {}",
        excesses.len(),
        excesses.join("; ")
    );
}

/// Plans the Benchy stretch at `path` on the bowden test config.
fn benchy_plans_within_the_extruder_limits(name: &str, path: &str) {
    let gcode = std::fs::read_to_string(source_tree::repo_file(path)).expect("the Benchy stretch");
    plans_within_the_extruder_limits(name, &gcode, &source_tree::repo_file(CONFIG));
}

/// Lines 13747-13851: a cell behind the frozen seam asked 2009 mm/s² of a
/// 2000 mm/s² extruder after 68 back-offs of the line at the seam.
#[test]
fn a_benchy_infill_run_plans_on_the_bowden_test_config() {
    benchy_plans_within_the_extruder_limits(
        "benchy guard exhaustion",
        "rust/motion-core/tests/gcode/benchy_guard_exhaustion.gcode",
    );
}

/// Lines 22148-22204: with each after side kept at the width it was laid
/// at, every back-off scaled the first line, read by a crawl side that held
/// the binding cell but did not bind, until it fell to the floor while the
/// cell 4.9 s on still asked 2005 mm/s² of 2000.
#[test]
fn a_benchy_wall_run_plans_on_the_bowden_test_config() {
    benchy_plans_within_the_extruder_limits(
        "benchy guard side window",
        "rust/motion-core/tests/gcode/benchy_guard_side_window.gcode",
    );
}

/// Lines 28412-28783: the edge cell of a crawl's before side read its
/// rebuild at the 51 mm/s its move's box peaks at, while the shaped motion
/// there ran at 14 mm/s, and still asked 3973 mm/s² of 2000 after 14
/// back-offs: they slow the cell, not the move's feed-limited peak.
#[test]
fn a_benchy_crawl_beside_a_fast_move_plans_on_the_bowden_test_config() {
    benchy_plans_within_the_extruder_limits(
        "benchy guard side box",
        "rust/motion-core/tests/gcode/benchy_guard_side_box.gcode",
    );
}

/// Lines 17878-17923: the cell 0.77 s in asked 2014 mm/s² of 2000 after 68
/// back-offs of the line at the frozen seam. Its window spanned the 4 s
/// every crawl side holding the cell reads, binding or not.
#[test]
fn a_benchy_crawl_among_wide_sides_plans_on_the_bowden_test_config() {
    benchy_plans_within_the_extruder_limits(
        "benchy guard side chase",
        "rust/motion-core/tests/gcode/benchy_guard_side_chase.gcode",
    );
}

/// Lines 76544-76598: a cell behind the frozen seam asked 2002 mm/s² of
/// 2000 after 66 back-offs of the line at the seam, which could not lower
/// what the cell read.
#[test]
fn a_benchy_crawl_behind_the_frozen_seam_plans_on_the_bowden_test_config() {
    benchy_plans_within_the_extruder_limits(
        "benchy guard after width",
        "rust/motion-core/tests/gcode/benchy_guard_after_width.gcode",
    );
}

/// Scenario fuzz seed 1164 (`tools/sim/fuzzing/repros/issue-902`), shrunk:
/// a corexy with no input shaper and a kernel-free advance limited to
/// 20000 mm/s².
const UNSHAPED_PRINTER: &str = "[printer]
max_velocity: 1000
max_accel: 10000
corner_deviation: 0.21
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
[axis y]
[axis z]
[axis e]
follows: x, y, z
motors: extruder
max_accel: 20000
post_processors: pa
[post_processor pa]
type: linear_pressure_advance
k: 0.0508
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

/// A travel ends into a reversing one: the cell at the seam asked
/// 42374 mm/s² of 20000 after 7 back-offs of the first travel, reading
/// the second's acceleration, which no back-off scaled.
const TRAVEL_INTO_A_REVERSAL: &str = "G90
M83
G1 X121.263 Y149.188 Z0.4 F30000
G1 X121.142 Y149.43 E.00913
G1 Z0.6 F600
G1 X112.385 Y161.55 F39240
G1 X169.53 Y95.84 F44520
";

#[test]
fn an_unshaped_travel_into_a_reversal_plans() {
    let config =
        std::env::temp_dir().join(format!("unshaped_printer_{}.cfg", std::process::id()));
    std::fs::write(&config, UNSHAPED_PRINTER).expect("temp config");
    plans_within_the_extruder_limits("travel into a reversal", TRAVEL_INTO_A_REVERSAL, &config);
}
