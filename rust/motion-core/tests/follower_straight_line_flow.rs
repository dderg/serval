// A straight line of colinear extruding moves has zero corners, so the
// follower projection has zero legitimate corner-cut adaptation: the projected
// extrusion at the terminal rest must equal the commanded total to sub-step
// precision. Any systematic per-junction drift here is an integration error in
// the projection arithmetic, not flow adaptation.

use motion_core::seam_test_harness::{
    collect_shaped_segments_from_script, default_stream_config, parse_gcode_to_moves,
};
use motion_pipeline::StreamInput;

const EXTRUDER_AXIS: usize = 3;
const STEPS_PER_MM: f64 = 2206.9;

fn trident_chain_set() -> trajectory::AxisChainSet {
    let pa = trajectory::PostProcessorInstance::new(
        "pa",
        &trajectory::algos::LinearPressureAdvance,
        vec![0.017],
    );
    let st = trajectory::PostProcessorInstance::new(
        "st",
        &trajectory::algos::SmoothTriangle,
        vec![0.02],
    );
    let e_chain =
        trajectory::CompiledChain::compile(&[pa, st]).expect("pa + smooth_triangle composes");
    let bell =
        trajectory::PostProcessorInstance::new("bell", &trajectory::algos::SmoothBell, vec![0.003]);
    let xy_chain = trajectory::CompiledChain::compile(&[bell]).expect("smooth_bell compiles alone");
    trajectory::AxisChainSet {
        chains: vec![
            xy_chain.clone(),
            xy_chain,
            trajectory::CompiledChain::default(),
            e_chain,
        ],
        followers: vec![(EXTRUDER_AXIS, vec![0, 1, 2])],
    }
}

fn trident_config() -> motion_pipeline::StreamConfig {
    let mut cfg = default_stream_config();
    cfg.limits = geometry::VelocityLimits::try_new(2800.0, 50000.0, 100.0, 4_000_000.0)
        .expect("trident bench limits are valid");
    cfg
}

#[test]
fn colinear_moves_end_at_commanded_extrusion() {
    let mut gcode = String::from("G90\nM83\nG1 X0 Y50 F9000\n");
    for i in 0..200 {
        gcode.push_str(&format!(
            "G1 X{:.1} Y50 E0.6 F30000\n",
            (i + 1) as f64 * 1.4
        ));
    }
    let cfg = trident_config();
    let moves = parse_gcode_to_moves(&gcode, cfg.limits);
    assert_eq!(moves.len(), 200, "print body failed to parse");
    let mut script: Vec<StreamInput> = moves.into_iter().map(StreamInput::from).collect();
    script.push(StreamInput::Drain);

    let segs = collect_shaped_segments_from_script(script, cfg, trident_chain_set());
    let last = segs.last().expect("shaped segments emitted");
    let projected = nurbs::eval::eval(&last.axes[EXTRUDER_AXIS], last.t_end);
    let commanded = 200.0 * 0.6;
    let error_steps = (commanded - projected).abs() * STEPS_PER_MM;
    eprintln!(
        "commanded {commanded} mm, projected {projected:.6} mm, \
         error {error_steps:.2} steps ({:.6} mm)",
        projected - commanded
    );
    assert!(
        error_steps < 1.0,
        "straight-line print drifted {error_steps:.2} steps \
         (projected {projected:.6} mm vs commanded {commanded} mm)"
    );
}
