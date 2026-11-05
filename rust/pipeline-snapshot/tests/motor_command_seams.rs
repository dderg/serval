use std::path::PathBuf;

use pipeline_snapshot::waypoints::parse_gcode;
use pipeline_snapshot::{Snapshot, SnapshotParams, pipeline_snapshot};
use planner_config::from_doc::read_motion_settings;

const SEAM_VELOCITY_ROUNDING_PER_SPEED: f64 = 1e-9;

fn post_processor_cases() -> PathBuf {
    source_tree::repo_file("snapshots/cases/post_processor")
}

fn mode_inverse_snapshot(gcode_name: &str) -> (Snapshot, f64) {
    let config = post_processor_cases().join("mode_inverse.cfg");
    let text = std::fs::read_to_string(&config).expect("read mode_inverse.cfg");
    let doc =
        config_doc::Document::parse(&text, &config.display().to_string()).expect("config parses");
    let (settings, _) = read_motion_settings(&doc).expect("motion settings");
    let gcode =
        std::fs::read_to_string(post_processor_cases().join(gcode_name)).expect("read gcode");
    let waypoints = parse_gcode(&gcode, &settings.cartesian).expect("gcode parses");
    let snapshot = pipeline_snapshot(
        &waypoints,
        SnapshotParams {
            max_velocity: settings.cartesian.max_velocity,
            max_accel: settings.cartesian.max_accel,
            corner: settings.cartesian.corner,
            max_jerk: settings.cartesian.max_jerk,
            max_z_velocity: SnapshotParams::binding_z_cap(
                settings.cartesian.max_z_velocity,
                settings.cartesian.max_velocity,
            ),
            max_z_accel: SnapshotParams::binding_z_cap(
                settings.cartesian.max_z_accel,
                settings.cartesian.max_accel,
            ),
            max_extrude_only_velocity: settings.max_extrude_only_velocity,
            max_extrude_only_accel: settings.max_extrude_only_accel,
            max_path_deviation: Some(settings.fit_tolerance_mm),
            max_accel_deviation: Some(settings.fit_tolerance_accel_mm_s2),
            axis_decls: settings.axes,
            post_processor_decls: settings.post_processors,
        },
    )
    .unwrap_or_else(|error| panic!("{gcode_name}: {error}"));
    (snapshot, settings.cartesian.max_velocity)
}

fn assert_motor_command_seams_are_velocity_continuous(gcode_name: &str) {
    let (snapshot, max_velocity) = mode_inverse_snapshot(gcode_name);
    assert!(
        snapshot.toolhead.is_some(),
        "{gcode_name}: mode_inverse must make the motor command depart from the toolhead"
    );
    let rounding = SEAM_VELOCITY_ROUNDING_PER_SPEED * max_velocity;
    for (axis, name) in [(0, "x"), (1, "y")] {
        let worst = snapshot
            .worst_seams
            .iter()
            .filter(|seam| seam.axis == axis)
            .max_by(|a, b| a.dv.abs().total_cmp(&b.dv.abs()));
        assert!(
            snapshot.seam_max_dv[axis] <= rounding,
            "{gcode_name}: motor command {name} steps {} mm/s across a seam (worst {worst:?}), \
             rounding allows {rounding}",
            snapshot.seam_max_dv[axis]
        );
    }
}

#[test]
fn mode_inverse_corner_motor_command_is_velocity_continuous_at_every_seam() {
    assert_motor_command_seams_are_velocity_continuous("corner.gcode");
}

#[test]
fn mode_inverse_extruding_corner_motor_command_is_velocity_continuous_at_every_seam() {
    assert_motor_command_seams_are_velocity_continuous("extruding_corner.gcode");
}
