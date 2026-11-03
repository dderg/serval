//! A projected follower's advance (pressure advance or any other
//! derivative-gain stage) needs a smoothing kernel or a ratio ramp in its
//! chain: an advance turns every extrusion-ratio change taken at speed into
//! an instant position step, which only a kernel can follow and only a ramp
//! can take out. Configuration refuses a chain with neither; the pipeline
//! refuses it too, at construction and at a chain swap, so a chain set built
//! by hand cannot reach the shaper either.

use motion_core::seam_test_harness::{
    collect_shaped_segments_from_script, default_stream_config, parse_gcode_to_moves,
};
use motion_pipeline::{Control, StreamConfig, StreamInput};
use trajectory::{AxisChainSet, CompiledChain, ContinuousSegment, PostProcessorInstance, algos};

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

fn ratio_ramped(e: Vec<PostProcessorInstance>) -> AxisChainSet {
    let mut chains = trident_leaders_with(e);
    chains.chains[EXTRUDER_AXIS] = chains.chains[EXTRUDER_AXIS]
        .clone()
        .with_ratio_ramp_time(0.015);
    chains
}

#[test]
fn a_follower_advance_on_a_ratio_ramp_is_accepted_when_the_pipeline_is_built() {
    let segments = collect_shaped_segments_from_script(
        corner_script(),
        config(),
        ratio_ramped(vec![linear_advance(0.02)]),
    );
    assert!(!segments.is_empty());
}

#[test]
fn a_follower_advance_on_a_ratio_ramp_is_accepted_at_a_chain_swap() {
    let mut script = corner_script();
    script.push(StreamInput::Drain);
    script.push(StreamInput::Control(Control::SetAxisChains(ratio_ramped(
        vec![linear_advance(0.02)],
    ))));
    let segments =
        collect_shaped_segments_from_script(script, config(), trident_leaders_with(Vec::new()));
    assert!(!segments.is_empty());
}

const BOWDEN_RATIO: f64 = 0.0333;
const BOWDEN_ADVANCE: f64 = 0.5;

fn bowden_kernel_chain() -> AxisChainSet {
    AxisChainSet {
        chains: vec![
            compiled(vec![PostProcessorInstance::new(
                "shx",
                &algos::SmoothMzv,
                vec![42.0],
            )]),
            compiled(vec![PostProcessorInstance::new(
                "shy",
                &algos::SmoothMzv,
                vec![36.0],
            )]),
            CompiledChain::default(),
            compiled(vec![
                linear_advance(BOWDEN_ADVANCE),
                PostProcessorInstance::new("st", &algos::SmoothTriangle, vec![0.002]),
            ]),
        ],
        followers: vec![(EXTRUDER_AXIS, vec![0, 1, 2])],
    }
}

fn bowden_corner(fit_tol_mm: f64) -> Vec<ContinuousSegment> {
    let mut config = default_stream_config();
    config.fit_tol_mm = fit_tol_mm;
    config.limits = geometry::VelocityLimits::try_new(
        150.0,
        3000.0,
        geometry::corner_deviation_from_scv(5.0, 3000.0),
        f64::INFINITY,
    )
    .expect("bowden limits are valid");
    let gcode = "G90\nM83\nG1 X0 Y0 F6000\nG1 X60 Y0 E2\nG1 X17.574 Y42.426 E2\n";
    let mut script: Vec<StreamInput> = parse_gcode_to_moves(gcode, config.limits)
        .into_iter()
        .map(StreamInput::from)
        .collect();
    script.push(StreamInput::Drain);
    collect_shaped_segments_from_script(script, config, bowden_kernel_chain())
}

fn axis_at(segments: &[ContinuousSegment], axis: usize, t: f64) -> f64 {
    segments
        .iter()
        .find(|segment| t >= segment.t_start && t <= segment.t_end)
        .expect("the sample lies inside the plan")
        .eval_axis(axis, t)
        .expect("the axis evaluates")
        .position
}

/// An advance turns its source's velocity error `δ'` into `k·δ'` of
/// command, and the melt lag that advance models hands `δ'` to the nozzle
/// flow whole: a bead error of `δ'/(r·v)`, largest where a corner slows.
/// Fitted after the advance, the source's error stays a position error
/// within the follower's budget, which the melt lag filters. The reference
/// plans the same corner at a 25× tighter tolerance; the extruders are
/// compared wherever both lay their bead on the same shaped path.
#[test]
fn an_advance_before_a_kernel_holds_the_command_to_the_position_budget() {
    let fit_tol_mm = 0.005;
    let fitted = bowden_corner(fit_tol_mm);
    let reference = bowden_corner(fit_tol_mm / 25.0);
    let t_end = fitted
        .last()
        .expect("the corner plans")
        .t_end
        .min(reference.last().expect("the corner plans").t_end);
    let samples: u32 = 20_000;
    let shared_path: Vec<f64> = (0..=samples)
        .map(|index| t_end * f64::from(index) / f64::from(samples))
        .filter(|&t| {
            [0, 1]
                .into_iter()
                .all(|axis| axis_at(&fitted, axis, t) == axis_at(&reference, axis, t))
        })
        .collect();
    assert!(
        shared_path.len() * 10 >= samples as usize * 9,
        "the tolerances must leave the shaped path alone: only {} of {samples} samples share it",
        shared_path.len()
    );
    let worst = shared_path
        .iter()
        .map(|&t| {
            (axis_at(&fitted, EXTRUDER_AXIS, t) - axis_at(&reference, EXTRUDER_AXIS, t)).abs()
        })
        .fold(0.0, f64::max);
    let budget = 2.0 * fit_tol_mm * BOWDEN_RATIO;
    assert!(
        worst <= budget,
        "the advanced command leaves the reference by {worst} mm, past the follower's \
         position budget {budget} mm"
    );
}
