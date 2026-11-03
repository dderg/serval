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
fn g0_is_g1_and_takes_and_keeps_its_feedrate() {
    let wp = parse("G1 X5 F600\nG0 X10\nG0 X20 F1200\nG1 X30\n");
    assert_eq!(
        wp.iter().map(|w| w.feedrate).collect::<Vec<_>>(),
        [10.0, 10.0, 20.0, 20.0]
    );
}

#[test]
fn moves_before_any_feedrate_run_at_klippys_default_speed() {
    let wp = parse("G1 X5\nG1 X10\n");
    assert_eq!(wp[1].feedrate, LIVE_DEFAULT_SPEED_MM_S);
}

#[test]
fn non_positive_feedrate_is_refused_like_klippy() {
    assert!(matches!(
        parse_err("G1 X5 F0\n"),
        WaypointError::InvalidSpeed { line: 1, .. }
    ));
}

#[test]
fn kinematic_feedrate_is_capped_at_the_effective_velocity_but_extrude_only_is_not() {
    let wp =
        parse("M83\nG1 X5 F60000\nG1 X10\nSET_VELOCITY_LIMIT VELOCITY=40\nG1 X20\nG1 E-2 F60000\n");
    assert_eq!(wp[1].feedrate, 300.0);
    assert_eq!(wp[2].feedrate, 40.0);
    assert_eq!(wp[3].feedrate, 1000.0);
}

#[test]
fn z_share_of_a_move_is_capped_at_max_z_velocity_like_check_move() {
    let config = CartesianLimits {
        max_z_velocity: 15.0,
        ..limits()
    };
    let wp = parse_gcode("G1 X0 Y0 Z0 F6000\nG1 X3 Z4\nG1 Z10\n", &config).unwrap();
    assert!((wp[1].feedrate - 15.0 * 5.0 / 4.0).abs() < 1e-12);
    assert_eq!(wp[2].feedrate, 15.0);
}

#[test]
fn a_move_to_the_commanded_position_submits_nothing() {
    let wp = parse("G1 X5 F600\nG1 X5\nG1 X10\n");
    assert_eq!(wp.len(), 2);
}

#[test]
fn a_sub_nanometre_xyz_move_with_extrusion_is_extrude_only_in_place() {
    let wp = parse("M83\nG1 X5 F600\nG1 X5.0000000001 E1 F60000\n");
    assert_eq!(wp[1].position, [5.0, 0.0, 0.0, 1.0]);
    assert_eq!(wp[1].feedrate, 1000.0);
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
fn e_is_absolute_by_default_like_klippy() {
    let wp = parse("G1 X10 E1 F6000\nG1 X20 E3\n");
    assert_eq!(wp[1].position[3], 3.0);
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
fn g91_makes_e_relative_even_after_m82() {
    let wp = parse("M82\nG91\nG1 X10 E1 F6000\nG1 X10 E1\n");
    assert_eq!(wp[1].position[3], 2.0);
}

#[test]
fn g92_moves_the_gcode_origin_not_the_toolhead() {
    let wp = parse("M82\nG1 X10 E5 F6000\nG92 E0\nG1 X20 E1\n");
    assert_eq!(wp.len(), 2);
    assert_eq!(wp[1].position[3], 6.0);
}

#[test]
fn g92_e0_under_relative_extrusion_leaves_later_extrusion_unchanged() {
    let wp = parse("M83\nG1 X10 E1 F6000\nG1 X20 E1\nG92 E0\nG1 X30 E1\n");
    assert_eq!(wp[2].position[3] - wp[1].position[3], 1.0);
}

#[test]
fn g92_on_xyz_offsets_later_absolute_targets() {
    let wp = parse("G1 X10 Y10 F6000\nG92 X0 Y0\nG1 X5 Y5\nG92\nG1 X1\n");
    assert_eq!(wp[1].position[..2], [15.0, 15.0]);
    assert_eq!(wp[2].position[..2], [16.0, 15.0]);
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
fn set_velocity_limit_sets_every_runtime_limit_live_honours() {
    let wp = parse(
        "G1 X10 F6000\n\
         SET_VELOCITY_LIMIT VELOCITY=100 ACCEL=500 SQUARE_CORNER_VELOCITY=8 MINIMUM_CRUISE_RATIO=0.5 ACCEL_TO_DECEL=200\n\
         G1 X20\n\
         SET_VELOCITY_LIMIT CORNER_DEVIATION=0.05\n\
         G1 X30\n",
    );
    assert_eq!(
        wp[1].runtime,
        RuntimeLimits {
            velocity: Some(100.0),
            accel: Some(500.0),
            corner_deviation: Some(geometry::corner_deviation_from_scv(8.0, 3000.0)),
        }
    );
    assert_eq!(wp[2].runtime.corner_deviation, Some(0.05));
    assert_eq!(wp[2].runtime.velocity, Some(100.0));
}

#[test]
fn set_velocity_limit_square_corner_velocity_converts_at_the_config_accel() {
    let wp = parse("G1 X10 F6000\nSET_VELOCITY_LIMIT ACCEL=500 SQUARE_CORNER_VELOCITY=8\nG1 X20\n");
    assert_eq!(
        wp[1].runtime.corner_deviation,
        Some(geometry::corner_deviation_from_scv(8.0, 3000.0))
    );
}

#[test]
fn set_velocity_limit_ignores_unknown_parameters_like_klippy() {
    let wp = parse("G1 X10 F6000\nSET_VELOCITY_LIMIT FOO=1\nG1 X20\n");
    assert_eq!(wp[1].runtime, RuntimeLimits::default());
}

#[test]
fn set_velocity_limit_refuses_both_corner_budget_aliases() {
    assert!(matches!(
        parse_err("SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY=5 CORNER_DEVIATION=0.05\n"),
        WaypointError::CornerBudgetAliases { line: 1 }
    ));
}

#[test]
fn set_velocity_limit_refuses_out_of_range_values_like_klippy() {
    for bad in [
        "SET_VELOCITY_LIMIT ACCEL=0\n",
        "SET_VELOCITY_LIMIT VELOCITY=-1\n",
        "SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY=-1\n",
        "SET_VELOCITY_LIMIT CORNER_DEVIATION=-0.1\n",
        "SET_VELOCITY_LIMIT ACCEL=fast\n",
        "SET_VELOCITY_LIMIT MINIMUM_CRUISE_RATIO=half\n",
    ] {
        assert!(
            matches!(parse_err(bad), WaypointError::InvalidParam { line: 1, .. }),
            "{bad}"
        );
    }
}

#[test]
fn reset_velocity_limit_returns_to_the_config_base() {
    let wp = parse(
        "G1 X10 F6000\nSET_VELOCITY_LIMIT VELOCITY=100 ACCEL=500 CORNER_DEVIATION=0.05\nG1 X20\nRESET_VELOCITY_LIMIT\nG1 X30\n",
    );
    assert_eq!(wp[2].runtime, RuntimeLimits::default());
}

#[test]
fn m204_sets_the_accel_cap_from_s_or_the_lower_of_p_and_t() {
    let wp = parse("G1 X10 F6000\nM204 S700\nG1 X20\nM204 P900 T800\nG1 X30\nM204 P100\nG1 X40\n");
    assert_eq!(wp[1].runtime.accel, Some(700.0));
    assert_eq!(wp[2].runtime.accel, Some(800.0));
    assert_eq!(
        wp[3].runtime.accel,
        Some(800.0),
        "P without T is reported invalid and changes nothing"
    );
    assert!(matches!(
        parse_err("M204 S0\n"),
        WaypointError::InvalidParam { line: 1, .. }
    ));
}

#[test]
fn unknown_extended_commands_are_ignored() {
    assert_eq!(
        parse("EXCLUDE_OBJECT_DEFINE NAME=part1 CENTER=104,100\nG1 X10 F6000\n").len(),
        1
    );
}

fn rests(waypoints: &[Waypoint]) -> Vec<Option<f64>> {
    waypoints.iter().map(|w| w.dwell_before_s).collect()
}

#[test]
fn stop_at_rest_commands_ride_on_the_next_move() {
    let wp = parse(
        "G1 X10 F6000\nG4 P250\nG1 X20\nM400\nG4 P100\nG1 X30\nG1 X40\nM84\n\
         SET_KINEMATIC_POSITION\nG1 X50\nM400\n",
    );
    assert_eq!(rests(&wp), [None, Some(0.25), Some(0.1), None, Some(0.2)]);
}

#[test]
fn a_rest_before_the_anchor_is_where_the_plan_starts() {
    assert_eq!(
        rests(&parse("G4 P500\nG1 X10 F6000\nG1 X20\n")),
        [None, None]
    );
}

#[test]
fn negative_dwell_is_refused_like_klippy() {
    assert!(matches!(
        parse_err("G1 X10 F6000\nG4 P-1\n"),
        WaypointError::InvalidParam { line: 2, .. }
    ));
}

#[test]
fn motors_off_unhomes_until_set_kinematic_position() {
    assert!(matches!(
        parse_err("G1 X10 F6000\nM18\nG1 Y5\n"),
        WaypointError::Unhomed { line: 3, axis: 'Y' }
    ));
    assert_eq!(parse("G1 X10 F6000\nM84\nG1 E1\n").len(), 2);
    assert!(matches!(
        parse_err("G1 X10 F6000\nM84\nSET_KINEMATIC_POSITION CLEAR=Z\nG1 Y5\nG1 Z1\n"),
        WaypointError::Unhomed { line: 5, axis: 'Z' }
    ));
}

#[test]
fn set_kinematic_position_names_the_start_before_the_anchor() {
    let wp = parse("SET_KINEMATIC_POSITION X=5\nG91\nG1 X10 F6000\nG1 X1\n");
    assert_eq!(wp[0].position[0], 15.0);
    assert_eq!(wp[1].position[0], 16.0);
}

#[test]
fn set_kinematic_position_away_from_the_toolhead_is_refused() {
    assert!(matches!(
        parse_err("G1 X10 F6000\nSET_KINEMATIC_POSITION X=0\n"),
        WaypointError::PositionJump { line: 2, .. }
    ));
}
