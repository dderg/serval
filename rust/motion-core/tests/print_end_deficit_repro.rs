// Offline reproduction of the trident bench crash on the second print of a
// session: print 1 runs long enough for the follower projection to accumulate
// a commanded-vs-projected extrusion gap (corner-cut flow adaptation plus
// drain-boundary residue), PRINT_END retracts/parks/M84s, and print 2's G28
// zeroes the extruder's MCU counter and pipeline odometer. Before the fix the
// gap survived `FollowerState::reset_timeline`, so the first post-reset
// extruder track anchored at `0 - carried_deficit` against a counter seeded
// to exactly 0 — a one-shot multi-thousand-step demand the MCU faults
// -310 StepsPerSampleExceeded on (observed live: 6228 steps = 2.82mm at
// 2206.9 steps/mm).

use motion_core::seam_test_harness::{
    collect_shaped_segments_from_script, default_stream_config, parse_gcode_to_moves,
};
use motion_pipeline::{Control, StreamInput};

const EXTRUDER_AXIS: usize = 3;
const STEPS_PER_MM: f64 = 2206.9; // trident: 200*16*37 / 53.65
const SAMPLE_PERIOD_S: f64 = 100e-6;
const MAX_STEPS_PER_SAMPLE: f64 = 16.0;

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
    let corner_deviation = geometry::corner_deviation_from_scv(60.0, 25000.0);
    cfg.limits =
        geometry::VelocityLimits::try_new(2800.0, 25000.0, corner_deviation, f64::INFINITY)
            .expect("trident bench limits are valid");
    cfg
}

// Print 1: 200 fast zigzag extruding lines with pacer drains, then the bench's
// PRINT_END tail (retract, anti-string dash, rear park). This accumulates a
// multi-hundred-step commanded-vs-projected gap by the final rest.
fn print_one_script() -> Vec<StreamInput> {
    let mut gcode = String::from("G90\nM83\nG1 X50 Y50 F9000\n");
    for i in 0..200 {
        let (x, y) = if i % 2 == 0 {
            (250, 50 + i / 2)
        } else {
            (50, 50 + i / 2)
        };
        gcode.push_str(&format!("G1 X{x} Y{y} E0.6 F30000\n"));
    }
    gcode.push_str("G1 E-10.0 F1800\nG1 X70 Y70 F60000\nG1 X150 Y252 F30000\n");
    let moves = parse_gcode_to_moves(&gcode, trident_config().limits);
    assert!(moves.len() > 200, "print body failed to parse");
    let mut script: Vec<StreamInput> = Vec::new();
    for (i, m) in moves.into_iter().enumerate() {
        script.push(m.into());
        if i % 40 == 39 {
            script.push(StreamInput::Drain);
        }
    }
    script.push(StreamInput::Drain);
    script
}

#[test]
#[ignore = "covered by simulator print/reseed scenario"]
fn g28_reseed_after_print_has_no_step_burst() {
    let mut script = print_one_script();
    // Print 2's G28: home_drip opens the stream with the extruder odometer
    // forced to 0 while runtime seed_position zeroes the MCU step counter —
    // the pipeline must emit an extruder track that starts exactly at 0.
    script.push(StreamInput::Control(Control::Reset {
        pos: vec![150.0, 252.0, 5.0, 0.0],
    }));
    let cfg = trident_config();
    let homing = parse_gcode_to_moves("G90\nG1 X150 Y252 F9000\nG1 X300 Y252 F2100\n", cfg.limits);
    assert!(!homing.is_empty(), "homing move failed to parse");
    script.extend(homing.into_iter().map(StreamInput::from));
    script.push(StreamInput::Drain);

    let all = collect_shaped_segments_from_script(script, cfg, trident_chain_set());
    // The Reset restarts the timeline at rest, so the post-reset segments are
    // the trailing run that begins where t_start jumps backwards.
    let restart = all
        .windows(2)
        .position(|w| w[1].t_start < w[0].t_end - 1e-9)
        .map_or(0, |i| i + 1);
    assert!(
        restart > 0,
        "expected the Reset to restart the shaped timeline across {} segments",
        all.len()
    );
    let segs = &all[restart..];
    assert!(!segs.is_empty(), "no segments emitted after the reset");

    // The MCU counter was seeded to exactly 0; sampling walks from 0.
    let mut prev = 0.0f64;
    let mut worst_steps = 0.0f64;
    let mut worst_t = 0.0f64;
    for seg in segs {
        let mut t = seg.t_start;
        while t < seg.t_end {
            let pos = seg
                .eval_axis(EXTRUDER_AXIS, t)
                .expect("extruder axis evaluates inside the segment")
                .position;
            let steps = (pos - prev).abs() * STEPS_PER_MM;
            if steps > worst_steps {
                worst_steps = steps;
                worst_t = t;
            }
            prev = pos;
            t += SAMPLE_PERIOD_S;
        }
    }
    eprintln!(
        "post-reset segments={} worst per-sample delta = {worst_steps:.1} steps at t={worst_t:.4}s",
        segs.len()
    );
    assert!(
        worst_steps <= MAX_STEPS_PER_SAMPLE,
        "extruder track demands {worst_steps:.1} steps in one {SAMPLE_PERIOD_S}s sample \
         at t={worst_t:.4}s after a counter-zeroing reset — the MCU faults -310 above \
         {MAX_STEPS_PER_SAMPLE}"
    );
}
