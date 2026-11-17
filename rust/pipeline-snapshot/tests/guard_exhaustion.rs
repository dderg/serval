//! #964: 105 lines of a stock Benchy (lines 13747-13851 of the Benchy replayed
//! in #420) on the bowden test config stop the turning guard: a cell it
//! checks asks 2009 mm/s² of a 2000 mm/s² extruder after 68 back-offs of the
//! line whose start seam it has frozen.

mod corpus;

const GCODE: &str = "rust/motion-core/tests/gcode/benchy_guard_exhaustion.gcode";
const CONFIG: &str = "rust/motion-core/tests/configs/p1_bowden_ramp.cfg";

#[test]
fn a_benchy_infill_run_plans_on_the_bowden_test_config() {
    let gcode = std::fs::read_to_string(source_tree::repo_file(GCODE)).expect("the Benchy stretch");
    let config = source_tree::repo_file(CONFIG);
    let snapshot = corpus::plan("benchy guard exhaustion", &gcode, &config)
        .expect("the Benchy stretch plans");
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
