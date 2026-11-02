//! A projected follower whose chain advances it with no smoothing kernel
//! after the advance. Configuration refuses such a chain (planner-config),
//! so these build it directly to hold the pipeline's own runtime backstop:
//! every advance step at speed fails loudly as `AdvanceStep`, and where no
//! step is taken at speed the advance leaves the extruded total alone.

use motion_core::seam_test_harness::{
    collect_shaped_segments_from_script, default_stream_config, parse_gcode_to_moves,
};
use motion_pipeline::{StreamConfig, StreamInput};
use trajectory::{AxisChainSet, CompiledChain, ContinuousSegment, PostProcessorInstance, algos};

const EXTRUDER_AXIS: usize = 3;
const RATIO: f64 = 0.0333;

fn compiled(instances: Vec<PostProcessorInstance>) -> CompiledChain {
    CompiledChain::compile(&instances).expect("chain compiles")
}

fn linear_advance(k: f64) -> Vec<PostProcessorInstance> {
    vec![PostProcessorInstance::new(
        "pa",
        &algos::LinearPressureAdvance,
        vec![k],
    )]
}

fn chains(
    x: Vec<PostProcessorInstance>,
    y: Vec<PostProcessorInstance>,
    e: Vec<PostProcessorInstance>,
) -> AxisChainSet {
    AxisChainSet {
        chains: vec![
            compiled(x),
            compiled(y),
            CompiledChain::default(),
            compiled(e),
        ],
        followers: vec![(EXTRUDER_AXIS, vec![0, 1, 2])],
    }
}

fn bench_shapers(e: Vec<PostProcessorInstance>) -> AxisChainSet {
    chains(
        vec![PostProcessorInstance::new(
            "shx",
            &algos::SmoothMzv,
            vec![50.0],
        )],
        vec![PostProcessorInstance::new(
            "shy",
            &algos::SmoothZv,
            vec![44.0],
        )],
        e,
    )
}

fn config(max_velocity: f64, max_accel: f64, corner_deviation: f64) -> StreamConfig {
    let mut config = default_stream_config();
    config.fit_tol_mm = 0.005;
    config.integration_tol = 1e-7;
    config.max_buffer_moves = 65_536;
    config.limits =
        geometry::VelocityLimits::try_new(max_velocity, max_accel, corner_deviation, f64::INFINITY)
            .expect("test limits are valid");
    config
}

fn final_extruder_position(gcode: &str, config: StreamConfig, chains: AxisChainSet) -> f64 {
    let moves = parse_gcode_to_moves(gcode, config.limits);
    let script = moves.into_iter().map(StreamInput::from).collect();
    let segs: Vec<ContinuousSegment> = collect_shaped_segments_from_script(script, config, chains);
    let last = segs.last().expect("the gcode produces segments");
    last.eval_axis(EXTRUDER_AXIS, last.t_end)
        .expect("extruder carrier")
        .position
}

/// A 20 mm square perimeter at 150 mm/s, entered from `entry`, its sides'
/// extrusion scaled by `flow`, then the moves in `exit`.
fn perimeter_gcode(entry: &[(f64, f64)], flow: [f64; 4], exit: &[(f64, f64)]) -> String {
    let mut gcode = String::from("G90\nM83\nG1 X0 Y0 F9000\n");
    for (x, y) in entry {
        gcode.push_str(&format!("G1 X{x} Y{y}\n"));
    }
    for ((x, y), scale) in [(40.0, 20.0), (40.0, 40.0), (20.0, 40.0), (20.0, 20.0)]
        .into_iter()
        .zip(flow)
    {
        gcode.push_str(&format!("G1 X{x} Y{y} E{:.6}\n", RATIO * 20.0 * scale));
    }
    for (x, y) in exit {
        gcode.push_str(&format!("G1 X{x} Y{y}\n"));
    }
    gcode
}

fn repo_file(path: &str) -> String {
    std::fs::read_to_string(format!("{}/../../{path}", env!("CARGO_MANIFEST_DIR")))
        .unwrap_or_else(|error| panic!("read {path}: {error}"))
}

#[test]
fn pressure_advance_alone_follows_shaped_leaders_away_from_rest() {
    let trident = chains(
        vec![PostProcessorInstance::new(
            "shx",
            &algos::SmoothMzv,
            vec![191.0],
        )],
        vec![PostProcessorInstance::new(
            "shy",
            &algos::SmoothMzv,
            vec![129.4],
        )],
        linear_advance(0.02),
    );
    let gcode = "G90\nM83\nG1 X0 Y0 F3000\nG1 X60 Y0 E2\nG1 X60 Y60 E2\n";
    let e_end = final_extruder_position(gcode, config(1000.0, 10_000.0, 0.04), trident);
    assert!(
        (e_end - 4.0).abs() < 0.05,
        "the extruder must end near the commanded 4 mm, ended at {e_end}"
    );
}

#[test]
#[should_panic(expected = "cannot follow an extrusion-ratio step taken at speed")]
fn a_ratio_step_at_a_join_taken_at_speed_is_refused() {
    let gcode = perimeter_gcode(
        &[(10.0, 10.0), (20.0, 20.0)],
        [1.0, 1.5, 1.0, 1.0],
        &[(20.0, 5.0)],
    );
    final_extruder_position(
        &gcode,
        config(1000.0, 10_000.0, 0.21),
        bench_shapers(linear_advance(0.03)),
    );
}

#[test]
#[should_panic(expected = "cannot follow an extrusion-ratio step taken at speed")]
fn a_ratio_step_inside_a_segment_taken_at_speed_is_refused() {
    let gcode = repo_file("tools/sim/gcode/voron_dense_top_layers.gcode")
        .lines()
        .take(120)
        .collect::<Vec<_>>()
        .join("\n");
    final_extruder_position(
        &gcode,
        config(1000.0, 10_000.0, 0.04),
        bench_shapers(linear_advance(0.06)),
    );
}

#[test]
fn an_advance_conserves_extrusion_between_rests() {
    let gcode = perimeter_gcode(&[(20.0, 20.0)], [1.0; 4], &[]);
    let limits = config(1000.0, 10_000.0, 0.21);
    let unadvanced = final_extruder_position(&gcode, limits, bench_shapers(Vec::new()));
    let advanced = final_extruder_position(&gcode, limits, bench_shapers(linear_advance(0.03)));
    assert!(
        (advanced - unadvanced).abs() < 1e-6,
        "pressure advance vanishes at rest, so it must not change the extruded total: \
         {advanced} mm with advance, {unadvanced} mm without"
    );
}

/// Real slicer flow varies by fractions of a percent between moves, so
/// most joins on a print carry an advance step well inside the fit
/// tolerance. Each one is small; dropped, they add up.
#[test]
fn an_advance_conserves_extrusion_through_sub_tolerance_ratio_steps() {
    let gcode = repo_file("snapshots/cases/neptune_cube/layer_5.gcode");
    let limits = config(
        300.0,
        3000.0,
        geometry::corner_deviation_from_scv(5.0, 3000.0),
    );
    let raw_leaders = |e| chains(Vec::new(), Vec::new(), e);
    let unadvanced = final_extruder_position(&gcode, limits, raw_leaders(Vec::new()));
    let advanced = final_extruder_position(&gcode, limits, raw_leaders(linear_advance(0.03)));
    assert!(
        (advanced - unadvanced).abs() < 1e-9,
        "every advance step a join takes within the fit tolerance must be carried: \
         {advanced} mm with advance, {unadvanced} mm without"
    );
}
