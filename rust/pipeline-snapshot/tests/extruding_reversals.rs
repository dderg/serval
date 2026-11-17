use pipeline_snapshot::pipeline_snapshot_for_config;
use pipeline_snapshot::waypoints::parse_gcode;
use planner_config::from_doc::planner_config_from_text;

mod corpus;

const EXTRUSION_RATIO: f64 = 0.04;

fn plans_extruding_reversals(length_mm: f64, feed_mm_min: u32, count: usize) {
    let config =
        std::fs::read_to_string(corpus::repo().join(corpus::PRINTER_CONFIG)).expect("printer.cfg");
    let planner = planner_config_from_text(&config).unwrap();
    let mut gcode = format!("G90\nM83\nG1 X10 Y10 F6000\nG1 X10 Y10 F{feed_mm_min}\n");
    let (out, e) = (10.0 + length_mm, EXTRUSION_RATIO * length_mm);
    for _ in 0..count {
        gcode.push_str(&format!("G1 X{out} Y10 E{e}\nG1 X10 Y10 E{e}\n"));
    }
    let waypoints = parse_gcode(&gcode, &planner.cartesian).unwrap();
    pipeline_snapshot_for_config(&waypoints, &planner, None).unwrap();
}

/// 5 mm extruding reversals at 1000 mm/s on the bench (#904): ten pass,
/// fifteen fail.
#[test]
fn extruding_reversals_at_1000_mm_s() {
    plans_extruding_reversals(5.0, 60000, 15);
}

/// Gap fill's out-and-back at 150 mm/s on the bench (#904): between two
/// reversals 0.5 mm apart the shaped leaders turn around off each seam of
/// the same leg.
#[test]
fn half_millimetre_extruding_reversals_at_150_mm_s() {
    plans_extruding_reversals(0.5, 9000, 5);
}
