//! A projected follower's advance (pressure advance or any other
//! derivative-gain stage) needs a smoothing kernel in its chain: an advance
//! turns every extrusion-ratio change taken at speed into an instant
//! position step only a kernel can follow. Configuration refuses such a
//! chain; the pipeline refuses it too, at construction and at a chain swap,
//! so a chain set built by hand cannot reach the shaper either.

use motion_core::seam_test_harness::{
    collect_shaped_segments_from_script, default_stream_config, parse_gcode_to_moves,
};
use motion_pipeline::{Control, StreamConfig, StreamInput};
use trajectory::{AxisChainSet, CompiledChain, PostProcessorInstance, algos};

const EXTRUDER_AXIS: usize = 3;

fn compiled(instances: Vec<PostProcessorInstance>) -> CompiledChain {
    CompiledChain::compile(&instances).expect("chain compiles")
}

fn trident_leaders_with(e: Vec<PostProcessorInstance>) -> AxisChainSet {
    AxisChainSet {
        chains: vec![
            compiled(vec![PostProcessorInstance::new(
                "shx",
                &algos::SmoothMzv,
                vec![191.0],
            )]),
            compiled(vec![PostProcessorInstance::new(
                "shy",
                &algos::SmoothMzv,
                vec![129.4],
            )]),
            CompiledChain::default(),
            compiled(e),
        ],
        followers: vec![(EXTRUDER_AXIS, vec![0, 1, 2])],
    }
}

fn linear_advance(k: f64) -> PostProcessorInstance {
    PostProcessorInstance::new("pa", &algos::LinearPressureAdvance, vec![k])
}

fn config() -> StreamConfig {
    let mut config = default_stream_config();
    config.fit_tol_mm = 0.005;
    config.integration_tol = 1e-7;
    config.limits = geometry::VelocityLimits::try_new(1000.0, 10_000.0, 0.04, f64::INFINITY)
        .expect("test limits are valid");
    config
}

fn corner_script() -> Vec<StreamInput> {
    let gcode = "G90\nM83\nG1 X0 Y0 F3000\nG1 X60 Y0 E2\nG1 X60 Y60 E2\n";
    parse_gcode_to_moves(gcode, config().limits)
        .into_iter()
        .map(StreamInput::from)
        .collect()
}

#[test]
#[should_panic(expected = "follows other axes and its chain advances it with no smoothing kernel")]
fn a_follower_advance_without_a_kernel_is_refused_when_the_pipeline_is_built() {
    collect_shaped_segments_from_script(
        corner_script(),
        config(),
        trident_leaders_with(vec![linear_advance(0.02)]),
    );
}

#[test]
#[should_panic(expected = "follows other axes and its chain advances it with no smoothing kernel")]
fn a_follower_advance_without_a_kernel_is_refused_at_a_chain_swap() {
    let mut script = corner_script();
    script.push(StreamInput::Drain);
    script.push(StreamInput::Control(Control::SetAxisChains(
        trident_leaders_with(vec![linear_advance(0.02)]),
    )));
    collect_shaped_segments_from_script(script, config(), trident_leaders_with(Vec::new()));
}
