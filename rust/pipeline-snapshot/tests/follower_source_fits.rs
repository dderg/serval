use std::path::Path;

use pipeline_snapshot::pipeline_snapshot_for_config;
use pipeline_snapshot::waypoints::parse_gcode;
use planner_config::from_doc::planner_config_from_text;

mod corpus;

fn bench_config_with(replacements: &[(&str, &str)], appended: &str) -> String {
    let mut text = std::fs::read_to_string(corpus::repo().join(corpus::PRINTER_CONFIG)).unwrap();
    for (from, to) in replacements {
        assert!(text.contains(from), "printer.cfg has no `{from}`");
        text = text.replace(from, to);
    }
    text + appended
}

fn plan(config_text: &str, gcode: &str) {
    let planner = planner_config_from_text(config_text).unwrap();
    let waypoints = parse_gcode(gcode, &planner.cartesian).unwrap();
    pipeline_snapshot_for_config(&waypoints, &planner, None).unwrap();
}

#[test]
fn bell_shaped_leaders_carry_a_z_and_e_move() {
    let config = bench_config_with(
        &[
            ("type: smooth_mzv\nfrequency_hz: 50", "type: smooth_bell\nsmooth_time: 0.02"),
            ("type: smooth_zv\nfrequency_hz: 44", "type: smooth_bell\nsmooth_time: 0.025"),
        ],
        "",
    );
    plan(
        &config,
        "G90\nM83\nG1 X40 Y150 Z10 F6000\nG1 X240 Y150 Z50 E38.6 F54000\n",
    );
}

#[test]
fn recipr_advance_rides_unshaped_leaders_through_epochs() {
    let config = corpus::repo().join("snapshots/cases/nonlinear_pa/recipr.cfg");
    let gcode = std::fs::read_to_string(corpus::repo().join("tools/bench/gcode/epochs.gcode"))
        .unwrap()
        .lines()
        .take(28)
        .collect::<Vec<_>>()
        .join("\n");
    plan(&std::fs::read_to_string(Path::new(&config)).unwrap(), &gcode);
}

#[test]
fn mode_inverse_after_the_kernels_turns_a_corner() {
    let config = bench_config_with(
        &[
            ("post_processors: shaper_x\n", "post_processors: shaper_x, inv_x\n"),
            ("post_processors: shaper_y\n", "post_processors: shaper_y, inv_y\n"),
        ],
        "\n[post_processor inv_x]\ntype: mode_inverse\nfrequency_hz: 50\ndamping_ratio: 0.1\n\
         \n[post_processor inv_y]\ntype: mode_inverse\nfrequency_hz: 44\ndamping_ratio: 0.1\n",
    );
    plan(&config, "G1 X0 Y0\nG1 X10 Y0\nG1 X10.872 Y0.038\n");
}
