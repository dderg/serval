//! The x/y shaped-leader fits of a dense top layer under the bench printer's
//! shapers (`tools/bench/printer.cfg`), with an e chain that adds no time
//! kernel of its own, so the planning window is cut by the x/y kernels alone.

use pipeline_snapshot::waypoints::parse_gcode;
use pipeline_snapshot::{AxisDecl, PostProcessorDecl, SnapshotParams, pipeline_snapshot};

const MAX_VELOCITY: f64 = 1000.0;
const MAX_ACCEL: f64 = 10_000.0;

fn dense_top_layer_lines(count: usize) -> String {
    let path = format!(
        "{}/../../tools/sim/gcode/voron_dense_top_layers.gcode",
        env!("CARGO_MANIFEST_DIR")
    );
    let text = std::fs::read_to_string(path).expect("read dense top layer gcode");
    text.lines().take(count).collect::<Vec<_>>().join("\n")
}

fn axis(name: &str, follows: &[&str], post_processors: &[&str]) -> AxisDecl {
    AxisDecl {
        name: name.into(),
        follows: follows.iter().map(|s| (*s).into()).collect(),
        motors: Vec::new(),
        post_processors: post_processors.iter().map(|s| (*s).into()).collect(),
    }
}

fn post_processor(name: &str, ty: &str, params: &[(&str, f64)]) -> PostProcessorDecl {
    PostProcessorDecl {
        name: name.into(),
        ty: ty.into(),
        params: params.iter().map(|(k, v)| ((*k).into(), *v)).collect(),
    }
}

fn bench_shapers_with_unkerneled_e() -> SnapshotParams {
    SnapshotParams {
        max_velocity: MAX_VELOCITY,
        max_accel: MAX_ACCEL,
        corner_deviation: 0.21,
        max_jerk: f64::INFINITY,
        max_extrude_only_velocity: None,
        max_extrude_only_accel: None,
        max_path_deviation: Some(0.005),
        max_accel_deviation: Some(50.0),
        axis_decls: vec![
            axis("x", &[], &["shaper_x"]),
            axis("y", &[], &["shaper_y"]),
            axis("z", &[], &[]),
            axis("e", &["x", "y", "z"], &["pa"]),
        ],
        post_processor_decls: vec![
            post_processor("pa", "linear_pressure_advance", &[("k", 0.0)]),
            post_processor("shaper_x", "smooth_mzv", &[("frequency_hz", 50.0)]),
            post_processor("shaper_y", "smooth_zv", &[("frequency_hz", 44.0)]),
        ],
    }
}

#[test]
fn dense_top_layer_shapes_with_an_unkerneled_extruder() {
    let gcode = dense_top_layer_lines(30);
    let waypoints = parse_gcode(&gcode, MAX_VELOCITY, MAX_ACCEL).expect("parse");
    let snapshot =
        pipeline_snapshot(&waypoints, bench_shapers_with_unkerneled_e()).expect("snapshot");
    assert!(snapshot.traversal_time_s > 0.0);
}
