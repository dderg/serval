use super::*;

fn limits() -> CartesianLimits {
    CartesianLimits::without_z_caps(300.0, 3000.0, f64::INFINITY, 0.0)
}

fn parse(text: &str) -> Vec<Waypoint> {
    parse_gcode(text, &limits()).unwrap()
}

fn parse_err(text: &str) -> WaypointError {
    parse_gcode(text, &limits()).unwrap_err()
}

#[test]
fn absolute_moves_produce_anchor_plus_targets() {
    assert_eq!(
        parse("G90\nG1 X10 Y0 F6000\nG1 X10 Y10\n"),
        vec![
            Waypoint::at([10.0, 0.0, 0.0, 0.0], 100.0),
            Waypoint::at([10.0, 10.0, 0.0, 0.0], 100.0)
        ]
    );
}

#[test]
fn g0_travels_at_max_velocity_regardless_of_feedrate() {
    let wp = parse("G1 X5 F600\nG0 X10\n");
    assert_eq!(wp[0].feedrate, 10.0);
    assert_eq!(wp[1].feedrate, 300.0);
}

#[test]
fn feedrate_persists_across_moves_and_survives_g0() {
    let wp = parse("G1 X5 F600\nG0 X10\nG1 X20\n");
    assert_eq!(wp[2].feedrate, 10.0, "G1 reuses the last G1 feedrate");
}

#[test]
fn relative_mode_accumulates() {
    assert_eq!(
        parse("G91\nG1 X10 F6000\nG1 X10 Y5\n"),
        vec![
            Waypoint::at([10.0, 0.0, 0.0, 0.0], 100.0),
            Waypoint::at([20.0, 5.0, 0.0, 0.0], 100.0)
        ]
    );
}

#[test]
fn absolute_e_needs_declared_mode() {
    assert!(matches!(
        parse_err("G90\nG1 X10 E1 F6000\n"),
        WaypointError::AmbiguousExtruderMode { line: 2 }
    ));
}

#[test]
fn m83_makes_e_relative() {
    let wp = parse("M83\nG1 X10 E1 F6000\nG1 X20 E1\n");
    assert_eq!(wp[0].position[3], 1.0);
    assert_eq!(wp[1].position[3], 2.0);
}

#[test]
fn m82_makes_e_absolute() {
    let wp = parse("M82\nG1 X10 E1 F6000\nG1 X20 E3\n");
    assert_eq!(wp[0].position[3], 1.0);
    assert_eq!(wp[1].position[3], 3.0);
}

#[test]
fn g91_implies_relative_e_when_undeclared() {
    let wp = parse("G91\nG1 X10 E1 F6000\nG1 X10 E1\n");
    assert_eq!(wp[1].position[3], 2.0);
}

#[test]
fn g92_resets_e_without_a_move() {
    let wp = parse("M82\nG1 X10 E5 F6000\nG92 E0\nG1 X20 E1\n");
    assert_eq!(wp.len(), 2);
    assert_eq!(wp[1].position[3], 1.0);
}

#[test]
fn e_only_preamble_folds_into_first_positional_waypoint() {
    assert_eq!(
        parse("M83\nG1 E-2 F1800\nG1 X10 Y0 F6000\n"),
        vec![Waypoint::at([10.0, 0.0, 0.0, -2.0], 100.0)]
    );
}

#[test]
fn retract_after_position_is_a_waypoint() {
    let wp = parse("M83\nG1 X10 F6000\nG1 E-2 F1800\n");
    assert_eq!(wp.len(), 2);
    assert_eq!(wp[1], Waypoint::at([10.0, 0.0, 0.0, -2.0], 30.0));
}

#[test]
fn arcs_are_rejected() {
    assert!(matches!(
        parse_err("G1 X10 F6000\nG2 X20 I5 J0\n"),
        WaypointError::UnsupportedArc { line: 2, major: 2 }
    ));
}

#[test]
fn comments_and_unknown_commands_are_ignored() {
    assert_eq!(
        parse("; header\nM104 S200\nG28\nG1 X10 F6000 ; move\n").len(),
        1
    );
}

#[test]
fn feedrate_only_g1_updates_state_without_a_waypoint() {
    assert_eq!(
        parse("G1 F600\nG1 X10\n"),
        vec![Waypoint::at([10.0, 0.0, 0.0, 0.0], 10.0)]
    );
}

#[test]
fn set_velocity_limit_accel_rides_on_subsequent_waypoints_as_the_runtime_cap() {
    let wp = parse(
        "G1 X10 F6000\nSET_VELOCITY_LIMIT ACCEL=500\nG1 X20\nSET_VELOCITY_LIMIT ACCEL=50000\nG1 X30\n",
    );
    assert_eq!(wp[0].runtime, RuntimeLimits::default());
    assert_eq!(wp[1].runtime.accel, Some(500.0));
    assert_eq!(wp[2].runtime.accel, Some(50_000.0));
}

#[test]
fn set_velocity_limit_rejects_unsupported_params() {
    assert!(matches!(
        parse_err("SET_VELOCITY_LIMIT VELOCITY=100\nG1 X10 F6000\n"),
        WaypointError::UnsupportedVelocityLimitParam { line: 1, ref param } if &**param == "VELOCITY"
    ));
}

#[test]
fn set_velocity_limit_rejects_non_positive_accel() {
    assert!(matches!(
        parse_err("SET_VELOCITY_LIMIT ACCEL=0\nG1 X10 F6000\n"),
        WaypointError::InvalidAccelLimit { line: 1, ref value } if value == "0"
    ));
}

#[test]
fn unknown_extended_commands_are_ignored() {
    assert_eq!(
        parse("EXCLUDE_OBJECT_DEFINE NAME=part1 CENTER=104,100\nG1 X10 F6000\n").len(),
        1
    );
}
