// Offline replay of a full bench print through the trident PA +
// smooth_triangle extruder chain (no input shaper), scanning the shaped
// extruder track for per-sample step bursts that would fault -310 on the MCU.
// Gcode path comes from KALICO_REPRO_GCODE; skipped when unset.

use motion_core::seam_test_harness::{
    collect_shaped_segments_scripted, default_stream_config, parse_gcode_to_moves,
};

const EXTRUDER_AXIS: usize = 3;
const STEPS_PER_MM: f64 = 2206.9;
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
fn full_print_extruder_track_has_no_step_burst() {
    let Ok(path) = std::env::var("KALICO_REPRO_GCODE") else {
        eprintln!("KALICO_REPRO_GCODE unset — skipping full-print replay");
        return;
    };
    let source = std::fs::read_to_string(&path).expect("gcode file readable");

    let mut cfg = default_stream_config();
    cfg.limits = geometry::VelocityLimits::try_new(2800.0, 50000.0, 5.0, 100_000.0)
        .expect("trident bench limits are valid");
    let moves = parse_gcode_to_moves(&source, cfg.limits);
    eprintln!("parsed {} moves from {path}", moves.len());
    assert!(!moves.is_empty());

    let segs = collect_shaped_segments_scripted(
        &moves,
        cfg,
        trident_extruder_chain_set(),
        std::env::var("KALICO_REPRO_DRAIN_EVERY")
            .ok()
            .and_then(|v| v.parse().ok()),
    );
    eprintln!("pipeline emitted {} segments", segs.len());

    let mut bursts: Vec<(f64, f64, u32)> = Vec::new();
    let mut worst = (0.0f64, 0.0f64, 0u32);
    let mut prev: Option<f64> = None;
    for seg in &segs {
        let mut t = seg.t_start;
        while t < seg.t_end {
            let pos = nurbs::eval::eval(&seg.axes[EXTRUDER_AXIS], t);
            if let Some(p) = prev {
                let steps = (pos - p).abs() * STEPS_PER_MM;
                if steps > worst.0 {
                    worst = (steps, t, seg.source_line);
                }
                if steps > MAX_STEPS_PER_SAMPLE {
                    bursts.push((steps, t, seg.source_line));
                }
            }
            prev = Some(pos);
            t += SAMPLE_PERIOD_S;
        }
    }
    eprintln!(
        "worst per-sample delta = {:.2} steps ({:.2} mm/s) at t={:.4}s (gcode line {})",
        worst.0,
        worst.0 / STEPS_PER_MM / SAMPLE_PERIOD_S,
        worst.1,
        worst.2
    );
    bursts.sort_by(|a, b| b.0.total_cmp(&a.0));
    for (steps, t, line) in bursts.iter().take(20) {
        eprintln!("burst: {steps:.1} steps/sample at t={t:.4}s (gcode line {line})");
    }
    assert!(
        bursts.is_empty(),
        "{} samples exceed {MAX_STEPS_PER_SAMPLE} steps/sample; worst = {:.1}",
        bursts.len(),
        bursts[0].0
    );
}
