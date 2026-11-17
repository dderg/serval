use pipeline_snapshot::pipeline_snapshot_for_config;
use pipeline_snapshot::waypoints::parse_gcode;
use planner_config::from_doc::planner_config_from_text;

mod corpus;

/// 5 mm extruding reversals at 1000 mm/s on the bench (#904): ten pass,
/// fifteen fail.
const EXTRUDING_REVERSALS: usize = 15;

#[test]
fn extruding_reversals_at_1000_mm_s() {
    let config =
        std::fs::read_to_string(corpus::repo().join(corpus::PRINTER_CONFIG)).expect("printer.cfg");
    let planner = planner_config_from_text(&config).unwrap();
    let mut gcode = String::from("G90\nM83\nG1 X10 Y10 F6000\nG1 X10 Y10 F60000\n");
    for _ in 0..EXTRUDING_REVERSALS {
        gcode.push_str("G1 X15 Y10 E0.2\nG1 X10 Y10 E0.2\n");
    }
    let waypoints = parse_gcode(&gcode, &planner.cartesian).unwrap();
    pipeline_snapshot_for_config(&waypoints, &planner, None).unwrap();
}
