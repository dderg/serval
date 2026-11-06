//! A bowden extruder's ratio step from an extrusion into a travel with no
//! retraction, on a kernel-free chain whose path jerk the planner budgets at
//! the isolated-switch gain, stays within `[axis e]`'s limits: the step's
//! own advanced demand carries the path jerk the planner allows around it.

mod corpus;

const P1_BOWDEN_RAMP: &str = "tools/bench/configs/profiles/p1_bowden_bedslinger_ramp.cfg";
const DENSE: &str = "tools/sim/gcode/voron_dense_top_layers.gcode";
const SLICE_LINES: std::ops::RangeInclusive<usize> = 3300..=3370;

fn dense_slice() -> String {
    let gcode = std::fs::read_to_string(corpus::repo().join(DENSE)).expect("the dense print reads");
    let lines: Vec<&str> = gcode.lines().collect();
    let body = lines[SLICE_LINES.start() - 1..*SLICE_LINES.end()].join("\n");
    format!(
        "G21\nG90\nM83\nSET_KINEMATIC_POSITION X=120.056 Y=150.717 Z=45\nG1 F25953.656\n{body}\n"
    )
}

#[test]
fn an_extrusion_into_a_travel_holds_the_extruder_limits() {
    let gcode = dense_slice();
    let config = corpus::repo().join(P1_BOWDEN_RAMP);
    let snapshot = corpus::plan("dense ratio step", &gcode, &config).expect("the G-code plans");
    let audit = corpus::extruder::extruder_audit(&snapshot, &gcode, &config)
        .expect("the P1 bowden extruder has limits");
    let shown: Vec<String> = audit
        .excesses
        .iter()
        .take(5)
        .map(|e| format!("{e:?}"))
        .collect();
    assert!(
        audit.excesses.is_empty(),
        "{} extruder excesses, first: {}",
        audit.excesses.len(),
        shown.join("; ")
    );
}
