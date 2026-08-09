// Offline reproduction of the trident bench crash: boot, then `G1 E1 F60`
// trips -310 StepsPerSampleExceeded on the extruder axis (17-19 steps in one
// 100us sample at 2207 steps/mm). Replays the first-ever move being an
// extrude-only move through the streaming pipeline with the bench's PA +
// smooth_triangle chain and scans the shaped extruder track for a per-sample
// position delta the MCU would fault on.

use motion_core::seam_test_harness::{
    collect_shaped_segments_scripted, default_stream_config, parse_gcode_to_moves,
};

const EXTRUDER_AXIS: usize = 3;
const STEPS_PER_MM: f64 = 2206.9; // trident: 200*16*37 / 53.65
const SAMPLE_PERIOD_S: f64 = 100e-6;
const MAX_STEPS_PER_SAMPLE: f64 = 16.0;

fn trident_extruder_chain_set() -> trajectory::AxisChainSet {
    let pa = trajectory::PostProcessorInstance::new(
        "pa",
        &trajectory::algos::LinearPressureAdvance,
        vec![0.017],
    );
    let st = trajectory::PostProcessorInstance::new(
        "st",
        &trajectory::algos::SmoothTriangle,
        vec![0.016],
    );
    let e_chain =
        trajectory::CompiledChain::compile(&[pa, st]).expect("pa + smooth_triangle composes");
    trajectory::AxisChainSet {
        chains: vec![
            trajectory::CompiledChain::default(),
            trajectory::CompiledChain::default(),
            trajectory::CompiledChain::default(),
            e_chain,
        ],
        followers: vec![(EXTRUDER_AXIS, vec![0, 1, 2])],
    }
}

#[test]
fn first_move_extrude_only_has_no_step_burst() {
    let mut cfg = default_stream_config();
    cfg.limits = geometry::VelocityLimits::try_new(2800.0, 50000.0, 0.05, 4_000_000.0)
        .expect("trident bench limits are valid");
    let moves = parse_gcode_to_moves("G1 E0\nG1 E1 F60\n", cfg.limits);
    assert_eq!(moves.len(), 1, "expected exactly one extrude-only move");

    let segs = collect_shaped_segments_scripted(&moves, cfg, trident_extruder_chain_set(), None);
    assert!(!segs.is_empty(), "pipeline emitted no segments");

    let first_seg = segs.first().expect("checked non-empty");
    let track_start = nurbs::eval::eval(&first_seg.axes[EXTRUDER_AXIS], first_seg.t_start);
    eprintln!(
        "track start = {:.7} mm = {:.1} steps above the seeded position",
        track_start,
        track_start * STEPS_PER_MM
    );

    let mut worst_steps = 0.0f64;
    let mut worst_t = 0.0f64;
    let mut prev: Option<f64> = Some(0.0);
    for seg in &segs {
        let mut t = seg.t_start;
        while t < seg.t_end {
            let pos = nurbs::eval::eval(&seg.axes[EXTRUDER_AXIS], t);
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
        "segments={} worst per-sample delta = {:.2} steps at t={:.4}s",
        segs.len(),
        worst_steps,
        worst_t
    );
    assert!(
        worst_steps <= MAX_STEPS_PER_SAMPLE,
        "extruder track demands {worst_steps:.1} steps in one {SAMPLE_PERIOD_S}s sample \
         at t={worst_t:.4}s — the MCU faults -310 above {MAX_STEPS_PER_SAMPLE}"
    );
}
