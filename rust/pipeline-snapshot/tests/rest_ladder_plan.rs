mod corpus;

const RUNG_FROM_REST: &str = "SET_KINEMATIC_POSITION X=40 Y=150 Z=10
G92 E0
G90
M83
G1 X240 Y150 Z50 E38.6 F12000
";

const PRESSURE_ADVANCE_ON_E: &str = "motors: extruder
post_processors: pa, st

[post_processor pa]
type: linear_pressure_advance
k: 0.03

[post_processor st]
type: smooth_triangle
smooth_time: 0.04
";

/// Plans `gcode` under the bench rest ladder's fast-Z limits with pressure
/// advance and a smoothing kernel on E, edited by `edit`.
fn plans_with_pressure_advance(name: &str, gcode: &str, edit: impl Fn(String) -> String) {
    let base = std::fs::read_to_string(corpus::repo().join("tools/bench/configs/rest_ladder.cfg"))
        .expect("rest ladder config");
    assert!(base.contains("motors: extruder\n"));
    let config = edit(base.replacen("motors: extruder\n", PRESSURE_ADVANCE_ON_E, 1));
    let path = std::env::temp_dir().join(format!("{name}_{}.cfg", std::process::id()));
    std::fs::write(&path, config).expect("temp config");
    let snapshot = corpus::plan(name, gcode, &path).expect("waypoints");
    std::fs::remove_file(&path).expect("temp config removed");
    assert!(snapshot.traversal_time_s > 0.0, "{name}");
}

#[test]
fn a_rest_ladder_rung_plans_from_rest_with_pressure_advance_on_e() {
    plans_with_pressure_advance("rung_from_rest", RUNG_FROM_REST, |config| config);
}

#[test]
fn a_z_capped_rung_plans_from_rest_with_pressure_advance_on_e() {
    plans_with_pressure_advance(
        "z_capped_rung",
        &RUNG_FROM_REST.replace("F12000", "F9000"),
        |config| {
            config
                .replace("max_z_velocity: 400", "max_z_velocity: 25")
                .replace("max_z_accel: 15000", "max_z_accel: 100")
        },
    );
}

#[test]
fn the_bench_rest_ladder_plans_with_pressure_advance_on_e() {
    let gcode = std::fs::read_to_string(corpus::repo().join("tools/bench/gcode/rest_ladder.gcode"))
        .expect("rest ladder gcode");
    plans_with_pressure_advance("rest_ladder", &gcode, |config| config);
}
