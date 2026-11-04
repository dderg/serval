//! A projected follower's advance (pressure advance or any other
//! derivative-gain stage) needs no smoothing kernel in its chain: every
//! follower ramps its extrusion-ratio steps, so an advance turns no ratio
//! change taken at speed into a position step, and the pipeline takes such
//! a chain at construction and at a chain swap.

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
fn a_kernel_free_follower_advance_is_accepted_when_the_pipeline_is_built() {
    let segments = collect_shaped_segments_from_script(
        corner_script(),
        config(),
        trident_leaders_with(vec![linear_advance(0.02)]),
    );
    assert!(!segments.is_empty());
}

#[test]
fn a_kernel_free_follower_advance_is_accepted_at_a_chain_swap() {
    let mut script = corner_script();
    script.push(StreamInput::Drain);
    script.push(StreamInput::Control(Control::SetAxisChains(
        trident_leaders_with(vec![linear_advance(0.02)]),
    )));
    let segments =
        collect_shaped_segments_from_script(script, config(), trident_leaders_with(Vec::new()));
    assert!(!segments.is_empty());
}

const BOWDEN_RATIO: f64 = 0.0333;
const BOWDEN_ADVANCE: f64 = 0.5;

fn bowden_chain(e: Vec<PostProcessorInstance>) -> AxisChainSet {
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
            compiled(e),
        ],
        followers: vec![(EXTRUDER_AXIS, vec![0, 1, 2])],
    }
}

fn bowden_kernel() -> PostProcessorInstance {
    PostProcessorInstance::new("st", &algos::SmoothTriangle, vec![0.002])
}

fn bowden_corner(fit_tol_mm: f64, e: Vec<PostProcessorInstance>) -> Vec<ContinuousSegment> {
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
    collect_shaped_segments_from_script(script, config, bowden_chain(e))
}

fn segment_at(segments: &[ContinuousSegment], t: f64) -> &ContinuousSegment {
    segments
        .iter()
        .find(|segment| t >= segment.t_start && t <= segment.t_end)
        .expect("the sample lies inside the plan")
}

/// At cruise the advance only offsets the extruder by `k·ė`; an advance
/// applied both to the fitted source and to the kernel's input doubles it.
#[test]
fn an_advance_before_a_kernel_offsets_the_cruise_once() {
    let advanced = bowden_corner(0.005, vec![linear_advance(0.05), bowden_kernel()]);
    let plain = bowden_corner(0.005, vec![bowden_kernel()]);
    let cruise = 0.3;
    let plain_state = segment_at(&plain, cruise)
        .eval_axis(EXTRUDER_AXIS, cruise)
        .expect("the extruder evaluates");
    let offset = axis_at(&advanced, EXTRUDER_AXIS, cruise) - plain_state.position;
    let expected = 0.05 * plain_state.velocity;
    assert!(
        (offset - expected).abs() <= 1e-3 * expected,
        "the advance offsets the cruise by {offset} mm, not k·ė = {expected} mm"
    );
}

fn axis_at(segments: &[ContinuousSegment], axis: usize, t: f64) -> f64 {
    segment_at(segments, t)
        .eval_axis(axis, t)
        .expect("the axis evaluates")
        .position
}

fn bowden_advanced_corner(fit_tol_mm: f64) -> Vec<ContinuousSegment> {
    bowden_corner(
        fit_tol_mm,
        vec![linear_advance(BOWDEN_ADVANCE), bowden_kernel()],
    )
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
    let fitted = bowden_advanced_corner(fit_tol_mm);
    let reference = bowden_advanced_corner(fit_tol_mm / 25.0);
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
