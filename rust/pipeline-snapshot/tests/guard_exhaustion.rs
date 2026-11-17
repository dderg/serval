//! #964: stretches of a stock Benchy (the one replayed in #420) on the
//! bowden test config that stopped the turning guard.

mod corpus;

const CONFIG: &str = "rust/motion-core/tests/configs/p1_bowden_ramp.cfg";

/// Plans the G-code at `path` on the bowden test config and holds it to the
/// extruder's limits.
fn plans_within_the_extruder_limits(name: &str, path: &str) {
    let gcode = std::fs::read_to_string(source_tree::repo_file(path)).expect("the Benchy stretch");
    let config = source_tree::repo_file(CONFIG);
    let snapshot = corpus::plan(name, &gcode, &config).expect("the Benchy stretch plans");
    let audit = corpus::extruder::extruder_audit(&snapshot, &gcode, &config)
        .expect("the config limits its extruder");
    let excesses: Vec<String> = audit.excesses.iter().map(ToString::to_string).collect();
    assert!(
        excesses.is_empty(),
        "{} extruder excesses: {}",
        excesses.len(),
        excesses.join("; ")
    );
}

/// Lines 13747-13851: a cell behind the frozen seam asked 2009 mm/s² of a
/// 2000 mm/s² extruder after 68 back-offs of the line at the seam.
#[test]
fn a_benchy_infill_run_plans_on_the_bowden_test_config() {
    plans_within_the_extruder_limits(
        "benchy guard exhaustion",
        "rust/motion-core/tests/gcode/benchy_guard_exhaustion.gcode",
    );
}

/// Lines 22148-22204: every back-off scaled the first line, read by a
/// crawl side that held the binding cell but did not bind, until it fell to
/// the floor while the cell 4.9 s on still asked 2005 mm/s² of 2000.
#[test]
fn a_benchy_wall_run_plans_on_the_bowden_test_config() {
    plans_within_the_extruder_limits(
        "benchy guard side window",
        "rust/motion-core/tests/gcode/benchy_guard_side_window.gcode",
    );
}
