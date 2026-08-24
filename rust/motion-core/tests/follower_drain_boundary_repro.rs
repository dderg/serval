// Offline reproduction of the mid-print trident crash at pacer Drain
// boundaries: the shaped extruder track (axis 3, projected follower of X/Y/Z)
// keeps decaying for the chains' trailing support after the raw trajectory
// rests — the PA term k*e_dot is still inside the smoothing window at the
// flush instant. Before the fix the lowerer materialized the rest hold only
// when the next move arrived, so a Drain followed by a bare clock jump (a G4
// dwell, the pacer's idle gap) parked the MCU mid-decay and the resumed track
// re-entered at the settled value: a one-sample multi-step burst the MCU
// faults -310 StepsPerSampleExceeded on (observed live: 106 steps; the
// per-sample budget is 16).

use motion_core::seam_test_harness::{
    collect_shaped_segments_from_script, default_stream_config, parse_gcode_to_moves,
};
use motion_pipeline::{Control, StreamInput};

const EXTRUDER_AXIS: usize = 3;
const STEPS_PER_MM: f64 = 2206.9;
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
    cfg.limits = geometry::VelocityLimits::try_new(
        2800.0,
        50000.0,
        geometry::corner_deviation_from_scv(20.0, 50000.0),
        f64::INFINITY,
    )
    .expect("trident bench limits are valid");
    cfg
}

fn zigzag_with_drain_gaps_script() -> Vec<StreamInput> {
    let mut gcode = String::from("G90\nM83\nG1 X50 Y50 F9000\n");
    for i in 0..40 {
        let (x, y) = if i % 2 == 0 {
            (250, 50 + i / 2)
        } else {
            (50, 50 + i / 2)
        };
        gcode.push_str(&format!("G1 X{x} Y{y} E12 F30000\n"));
    }
    let moves = parse_gcode_to_moves(&gcode, trident_config().limits);
    assert!(moves.len() >= 40, "print body failed to parse");
    let mut script: Vec<StreamInput> = Vec::new();
    for (i, m) in moves.into_iter().enumerate() {
        script.push(m.into());
        if i % 10 == 9 {
            script.push(StreamInput::Drain);
            script.push(StreamInput::Control(Control::Dwell { secs: 0.5 }));
        }
    }
    script.push(StreamInput::Drain);
    script
}

#[test]
fn drain_boundaries_leave_extruder_track_step_safe() {
    let segs = collect_shaped_segments_from_script(
        zigzag_with_drain_gaps_script(),
        trident_config(),
        trident_chain_set(),
    );
    assert!(!segs.is_empty(), "no segments emitted");

    let mut prev: Option<f64> = None;
    let mut worst_steps = 0.0f64;
    let mut worst_t = 0.0f64;
    for seg in &segs {
        let mut t = seg.t_start;
        while t < seg.t_end {
            let pos = seg
                .eval_axis(EXTRUDER_AXIS, t)
                .expect("extruder axis evaluates inside the segment")
                .position;
            if let Some(p) = prev {
                let steps = (pos - p).abs() * STEPS_PER_MM;
                if steps > worst_steps {
                    worst_steps = steps;
                    worst_t = t;
                }
            }
            prev = Some(pos);
            t += SAMPLE_PERIOD_S;
        }
    }
    eprintln!(
        "segments={} worst per-sample delta = {worst_steps:.1} steps at t={worst_t:.4}s",
        segs.len()
    );
    assert!(
        worst_steps <= MAX_STEPS_PER_SAMPLE,
        "extruder track demands {worst_steps:.1} steps in one {SAMPLE_PERIOD_S}s sample \
         at t={worst_t:.4}s — the MCU faults -310 above {MAX_STEPS_PER_SAMPLE}"
    );
}
