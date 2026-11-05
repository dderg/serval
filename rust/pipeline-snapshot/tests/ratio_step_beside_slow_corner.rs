//! A bowden extruder's ratio steps between extrusion and travel, next to the
//! slow corners its small acceleration budget leaves, and on a travel the
//! leaders brake into, stay within `[axis e]`'s limits.

mod corpus;

const P1_BOWDEN_RAMP: &str = "tools/bench/configs/profiles/p1_bowden_bedslinger_ramp.cfg";

const SWITCH_AFTER_CORNER: &str = "\
G21
G90
M83
SET_KINEMATIC_POSITION X=188.365 Y=138.96 Z=5.4
G1 F24766.062
G1 X186.729 Y138.96 E.05767
G1 X184.845 Y137.075 E.09394
G1 X183.096 Y137.075 E.06163
G1 X165.388 Y137.075 F60000
G1 F24766.062
G1 X163.64 Y137.075 E.06163
G1 X160.12 Y140.595 E.1755
G1 X160.12 Y137.261 E.11754
";

const SWITCH_BEFORE_CORNER: &str = "\
G21
G90
M83
SET_KINEMATIC_POSITION X=160.12 Y=140.595 Z=5.6
G1 F24766.062
G1 X163.64 Y137.075 E.1755
G1 X165.388 Y137.075 E.06163
G1 X183.096 Y137.075 F60000
G1 F24766.062
G1 X184.845 Y137.075 E.06163
G1 X186.595 Y138.826 E.08727
G1 X186.614 Y138.826 E.00067
G1 X188.365 Y137.075 E.08727
";

const TRAVEL_BRAKING_INTO_EXTRUSION: &str = "\
G21
G90
M83
G1 X0 Y0 F3000
G1 X10.00000 Y10.00000
G1 X20.00000 Y20.00000
G1 X40.00000 Y20.00000 E0.665203
G1 X40.00000 Y40.00000 E0.665203
G1 X20.00000 Y40.00000 E0.665203
G1 X20.00000 Y20.00000 E0.665203
G1 X20.00000 Y5.00000
G1 X5.00000 Y5.00000
";

fn assert_within_extruder_limits(name: &str, gcode: &str) {
    let config = corpus::repo().join(P1_BOWDEN_RAMP);
    let snapshot = corpus::plan(name, gcode, &config).expect("the G-code plans");
    let audit = corpus::extruder::extruder_audit(&snapshot, gcode, &config)
        .expect("the P1 bowden extruder has limits");
    let shown: Vec<String> = audit.excesses.iter().take(5).map(|e| format!("{e:?}")).collect();
    assert!(
        audit.excesses.is_empty(),
        "{name}: {} extruder excesses, first: {}",
        audit.excesses.len(),
        shown.join("; ")
    );
}

#[test]
fn a_switch_after_a_slow_corner_holds_the_extruder_limits() {
    assert_within_extruder_limits("switch after corner", SWITCH_AFTER_CORNER);
}

#[test]
fn a_switch_before_a_slow_corner_holds_the_extruder_limits() {
    assert_within_extruder_limits("switch before corner", SWITCH_BEFORE_CORNER);
}

#[test]
fn a_travel_braking_into_an_extrusion_holds_the_extruder_limits() {
    assert_within_extruder_limits("travel into extrusion", TRAVEL_BRAKING_INTO_EXTRUSION);
}
