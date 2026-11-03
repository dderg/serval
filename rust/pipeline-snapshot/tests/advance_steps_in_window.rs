//! An advance before projection turns every follower velocity step into a
//! position step — at a segment join, or inside a segment where the shaped
//! path crosses a ratio-span boundary. A smoothing kernel after the advance
//! carries it; the kernel-free case is refused by configuration and held
//! by motion-core's kernel_free_follower_advance tests.

use pipeline_snapshot::waypoints::parse_gcode;
use pipeline_snapshot::{
    AxisDecl, EXTRUDER_AXIS, PostProcessorDecl, SampleSide, SnapshotParams, pipeline_snapshot,
};

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
        ratio_ramp_time: None,
    }
}

fn post_processor(name: &str, ty: &str, params: &[(&str, f64)]) -> PostProcessorDecl {
    PostProcessorDecl {
        name: name.into(),
        ty: ty.into(),
        params: params.iter().map(|(k, v)| ((*k).into(), *v)).collect(),
    }
}

fn bench_shapers_with_extruder_chain(e_chain: Vec<PostProcessorDecl>) -> SnapshotParams {
    let names = e_chain
        .iter()
        .map(|decl| decl.name.as_str())
        .collect::<Vec<_>>();
    let mut post_processor_decls = vec![
        post_processor("shaper_x", "smooth_mzv", &[("frequency_hz", 50.0)]),
        post_processor("shaper_y", "smooth_zv", &[("frequency_hz", 44.0)]),
    ];
    let axis_decls = vec![
        axis("x", &[], &["shaper_x"]),
        axis("y", &[], &["shaper_y"]),
        axis("z", &[], &[]),
        axis("e", &["x", "y", "z"], &names),
    ];
    post_processor_decls.extend(e_chain);
    SnapshotParams {
        max_velocity: MAX_VELOCITY,
        max_accel: MAX_ACCEL,
        corner_deviation: 0.04,
        max_jerk: f64::INFINITY,
        max_z_velocity: None,
        max_z_accel: None,
        max_extrude_only_velocity: None,
        max_extrude_only_accel: None,
        max_path_deviation: Some(0.005),
        max_accel_deviation: Some(50.0),
        axis_decls,
        post_processor_decls,
    }
}

fn tanh_advance() -> PostProcessorDecl {
    post_processor(
        "pa",
        "tanh_pressure_advance",
        &[
            ("linear_advance", 0.06),
            ("nonlinear_offset", 0.05),
            ("linearization_velocity", 20.0),
        ],
    )
}

fn triangle_smoothing() -> PostProcessorDecl {
    post_processor("st", "smooth_triangle", &[("smooth_time", 0.04)])
}

fn final_extruder_position(e_chain: Vec<PostProcessorDecl>) -> (f64, f64) {
    let waypoints =
        parse_gcode(&dense_top_layer_lines(120), MAX_VELOCITY, MAX_ACCEL).expect("parse");
    let snap = pipeline_snapshot(&waypoints, bench_shapers_with_extruder_chain(e_chain))
        .expect("snapshot");
    let end = snap.trajectory.t_end();
    let position = snap
        .trajectory
        .eval_axis(EXTRUDER_AXIS, end, SampleSide::Left)
        .expect("extruder carrier")
        .position;
    (position, snap.seam_max_dp[EXTRUDER_AXIS])
}

#[test]
fn a_ratio_step_inside_a_segment_is_carried_into_a_following_kernel() {
    let (unadvanced, _) = final_extruder_position(vec![triangle_smoothing()]);
    let (advanced, seam_step) = final_extruder_position(vec![tanh_advance(), triangle_smoothing()]);
    assert!(
        (advanced - unadvanced).abs() < 1e-6,
        "the advance vanishes at rest, so it must not change the extruded total: \
         {advanced} mm with advance, {unadvanced} mm without"
    );
    assert!(
        seam_step < 1e-9,
        "the kernel must smooth the advance's steps, yet the extruder track steps by \
         {seam_step} mm at a seam"
    );
}
