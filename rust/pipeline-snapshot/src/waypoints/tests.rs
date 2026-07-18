use super::*;

#[test]
fn absolute_moves_produce_anchor_plus_targets() {
    let wp = parse_gcode("G90\nG1 X10 Y0 F6000\nG1 X10 Y10\n", 300.0, 3000.0).unwrap();
    assert_eq!(
        wp,
        vec![
            (10.0, 0.0, 0.0, 0.0, 100.0, 3000.0),
            (10.0, 10.0, 0.0, 0.0, 100.0, 3000.0)
        ]
    );
}

#[test]
fn g0_travels_at_max_velocity_regardless_of_feedrate() {
    let wp = parse_gcode("G1 X5 F600\nG0 X10\n", 300.0, 3000.0).unwrap();
    assert_eq!(wp[0].4, 10.0);
    assert_eq!(wp[1].4, 300.0);
}

#[test]
fn feedrate_persists_across_moves_and_survives_g0() {
    let wp = parse_gcode("G1 X5 F600\nG0 X10\nG1 X20\n", 300.0, 3000.0).unwrap();
    assert_eq!(wp[2].4, 10.0, "G1 reuses the last G1 feedrate");
}

#[test]
fn relative_mode_accumulates() {
    let wp = parse_gcode("G91\nG1 X10 F6000\nG1 X10 Y5\n", 300.0, 3000.0).unwrap();
    assert_eq!(
        wp,
        vec![
            (10.0, 0.0, 0.0, 0.0, 100.0, 3000.0),
            (20.0, 5.0, 0.0, 0.0, 100.0, 3000.0)
        ]
    );
}

#[test]
fn absolute_e_needs_declared_mode() {
    let err = parse_gcode("G90\nG1 X10 E1 F6000\n", 300.0, 3000.0).unwrap_err();
    assert!(matches!(
        err,
        WaypointError::AmbiguousExtruderMode { line: 2 }
    ));
}

#[test]
fn m83_makes_e_relative() {
    let wp = parse_gcode("M83\nG1 X10 E1 F6000\nG1 X20 E1\n", 300.0, 3000.0).unwrap();
    assert_eq!(wp[0].3, 1.0);
    assert_eq!(wp[1].3, 2.0);
}

#[test]
fn m82_makes_e_absolute() {
    let wp = parse_gcode("M82\nG1 X10 E1 F6000\nG1 X20 E3\n", 300.0, 3000.0).unwrap();
    assert_eq!(wp[0].3, 1.0);
    assert_eq!(wp[1].3, 3.0);
}

#[test]
fn g91_implies_relative_e_when_undeclared() {
    let wp = parse_gcode("G91\nG1 X10 E1 F6000\nG1 X10 E1\n", 300.0, 3000.0).unwrap();
    assert_eq!(wp[1].3, 2.0);
}

#[test]
fn g92_resets_e_without_a_move() {
    let wp = parse_gcode("M82\nG1 X10 E5 F6000\nG92 E0\nG1 X20 E1\n", 300.0, 3000.0).unwrap();
    assert_eq!(wp.len(), 2);
    assert_eq!(wp[1].3, 1.0);
}

#[test]
fn e_only_preamble_folds_into_first_positional_waypoint() {
    let wp = parse_gcode("M83\nG1 E-2 F1800\nG1 X10 Y0 F6000\n", 300.0, 3000.0).unwrap();
    assert_eq!(wp.len(), 1);
    assert_eq!(wp[0], (10.0, 0.0, 0.0, -2.0, 100.0, 3000.0));
}

#[test]
fn retract_after_position_is_a_waypoint() {
    let wp = parse_gcode("M83\nG1 X10 F6000\nG1 E-2 F1800\n", 300.0, 3000.0).unwrap();
    assert_eq!(wp.len(), 2);
    assert_eq!(wp[1], (10.0, 0.0, 0.0, -2.0, 30.0, 3000.0));
}

#[test]
fn arcs_are_rejected() {
    let err = parse_gcode("G1 X10 F6000\nG2 X20 I5 J0\n", 300.0, 3000.0).unwrap_err();
    assert!(matches!(
        err,
        WaypointError::UnsupportedArc { line: 2, major: 2 }
    ));
}

#[test]
fn comments_and_unknown_commands_are_ignored() {
    let wp = parse_gcode(
        "; header\nM104 S200\nG28\nG1 X10 F6000 ; move\n",
        300.0,
        3000.0,
    )
    .unwrap();
    assert_eq!(wp.len(), 1);
}

#[test]
fn feedrate_only_g1_updates_state_without_a_waypoint() {
    let wp = parse_gcode("G1 F600\nG1 X10\n", 300.0, 3000.0).unwrap();
    assert_eq!(wp, vec![(10.0, 0.0, 0.0, 0.0, 10.0, 3000.0)]);
}

#[test]
fn set_velocity_limit_accel_applies_to_subsequent_moves() {
    let wp = parse_gcode(
        "G1 X10 F6000\nSET_VELOCITY_LIMIT ACCEL=500\nG1 X20\nSET_VELOCITY_LIMIT ACCEL=8000\nG1 X30\n",
        300.0,
        3000.0,
    )
    .unwrap();
    assert_eq!(wp[0].5, 3000.0, "before any override: config max_accel");
    assert_eq!(wp[1].5, 500.0);
    assert_eq!(wp[2].5, 8000.0);
}

#[test]
fn set_velocity_limit_rejects_unsupported_params() {
    let err = parse_gcode(
        "SET_VELOCITY_LIMIT VELOCITY=100\nG1 X10 F6000\n",
        300.0,
        3000.0,
    )
    .unwrap_err();
    assert!(matches!(
        err,
        WaypointError::UnsupportedVelocityLimitParam { line: 1, ref param } if &**param == "VELOCITY"
    ));
}

#[test]
fn set_velocity_limit_rejects_non_positive_accel() {
    let err = parse_gcode("SET_VELOCITY_LIMIT ACCEL=0\nG1 X10 F6000\n", 300.0, 3000.0).unwrap_err();
    assert!(matches!(
        err,
        WaypointError::InvalidAccelLimit { line: 1, ref value } if value == "0"
    ));
}

#[test]
fn unknown_extended_commands_are_ignored() {
    let wp = parse_gcode(
        "EXCLUDE_OBJECT_DEFINE NAME=part1 CENTER=104,100\nG1 X10 F6000\n",
        300.0,
        3000.0,
    )
    .unwrap();
    assert_eq!(wp.len(), 1);
}
