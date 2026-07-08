use crate::*;
use crossbeam_channel::unbounded;
use geometry::segment::SourceRange;
use geometry::{ChainFitConfig, MoveContext, VelocityLimits, line_move};
use nurbs::eval::eval;
use trajectory::{AxisChainSet, PostProcessorInstance, ShapedSegment};

fn cfg() -> StreamConfig {
    StreamConfig {
        chain: ChainFitConfig::default(),
        integration_tol: 1e-7,
        max_extrude_only_velocity_mm_s: f64::INFINITY,
        max_extrude_only_accel_mm_s2: f64::INFINITY,
        fit_tol_mm: 1e-3,
        fit_tol_accel_mm_s2: 50.0,
        max_buffer_moves: 64,
        limits: VelocityLimits::try_new(300.0, 5000.0, 5.0, 100_000.0).unwrap(),
    }
}

fn ctx(line_no: u32, feed: f64) -> MoveContext {
    MoveContext {
        extruder_axis: 3,
        feedrate_mm_s: feed,
        limits: VelocityLimits::try_new(300.0, 5000.0, 5.0, 100_000.0).unwrap(),
        source: SourceRange {
            start_line: line_no,
            end_line: line_no,
        },
    }
}

fn line(line_no: u32, start: [f64; 3], end: [f64; 3], e: f64) -> geometry::Move {
    line_move(start, end, e, ctx(line_no, 80.0)).unwrap()
}

fn cfg_bench() -> StreamConfig {
    StreamConfig {
        chain: ChainFitConfig::default(),
        integration_tol: 1e-4,
        max_extrude_only_velocity_mm_s: f64::INFINITY,
        max_extrude_only_accel_mm_s2: f64::INFINITY,
        fit_tol_mm: 0.005,
        fit_tol_accel_mm_s2: 50.0,
        max_buffer_moves: 512,
        limits: VelocityLimits::try_new(100.0, 1000.0, 5.0, 1_000_000.0).unwrap(),
    }
}

fn line_bench(line_no: u32, start: [f64; 3], end: [f64; 3]) -> geometry::Move {
    let ctx = MoveContext {
        extruder_axis: 3,
        feedrate_mm_s: 60.0,
        limits: VelocityLimits::try_new(100.0, 1000.0, 5.0, 1_000_000.0).unwrap(),
        source: SourceRange {
            start_line: line_no,
            end_line: line_no,
        },
    };
    line_move(start, end, 0.0, ctx).unwrap()
}

/// Deterministic synchronous replay: each stage runs to completion over a
/// pre-filled, closed channel, so no stage ever observes a transient-empty
/// input. The output is the full-look-ahead trajectory with exactly one
/// terminal brake-to-rest — the reference the live threaded pipeline
/// approaches as backpressure keeps its channels full.
fn replay(
    config: StreamConfig,
    chains: AxisChainSet,
    home: &[f64],
    t_start: f64,
    moves: &[geometry::Move],
) -> Vec<ShapedSegment> {
    let (raw_tx, raw_rx) = unbounded();
    for m in moves {
        raw_tx.send(m.clone().into()).unwrap();
    }
    drop(raw_tx);

    let (fitted_tx, fitted_rx) = unbounded();
    FitStage::new(config.chain).run(raw_rx, fitted_tx);

    let (planned_tx, planned_rx) = unbounded();
    Planner::new(config).run(fitted_rx, planned_tx);

    let (lowered_tx, lowered_rx) = unbounded();
    run_lowerer(
        planned_rx,
        lowered_tx,
        FitTol {
            pos_mm: config.fit_tol_mm,
            accel_mm_s2: config.fit_tol_accel_mm_s2,
        },
        chains.clone(),
        home.to_vec(),
        t_start,
    );

    let (shaped_tx, shaped_rx) = unbounded();
    Shaper::new(chains).run(lowered_rx, shaped_tx);

    shaped_rx
        .into_iter()
        .filter_map(|item| match item {
            ShapedItem::Seg(seg) => Some(seg),
            ShapedItem::Control(_) => None,
        })
        .collect()
}

fn boundary_speed(prev: &ShapedSegment, next: &ShapedSegment) -> f64 {
    let h = 1e-6;
    let axes = prev.axes.len().min(3);
    let mut v2 = 0.0;
    for axis in 0..axes {
        let a = eval(&prev.axes[axis], prev.t_end - h);
        let b = eval(&next.axes[axis], next.t_start + h);
        let v = (b - a) / (2.0 * h);
        v2 += v * v;
    }
    v2.sqrt()
}

fn assert_time_contiguous(segs: &[ShapedSegment]) {
    for w in segs.windows(2) {
        assert!(
            (w[1].t_start - w[0].t_end).abs() < 1e-9,
            "time gap between segments: {} -> {}",
            w[0].t_end,
            w[1].t_start
        );
    }
}

fn assert_position_contiguous(segs: &[ShapedSegment]) {
    for w in segs.windows(2) {
        for axis in 0..w[0].axes.len() {
            let a = eval(&w[0].axes[axis], w[0].t_end);
            let b = eval(&w[1].axes[axis], w[1].t_start);
            assert!(
                (a - b).abs() < 1e-6,
                "axis {axis} position gap at t={}: {a} vs {b}",
                w[0].t_end
            );
        }
    }
}

// Real first perimeter from a Voron cube print (Neptune bench), as (x, y, e).
// 135° chamfer corners blend; short ~1.3mm chamfers sit between long ~18.6mm edges.
const VORON_PERIMETER: [(f64, f64, f64); 17] = [
    (102.008, 96.308, 0.14859),
    (103.2, 95.814, 0.04756),
    (121.8, 95.814, 0.68571),
    (122.992, 96.308, 0.04756),
    (128.692, 102.008, 0.29718),
    (129.186, 103.2, 0.04756),
    (129.186, 121.8, 0.68571),
    (128.692, 122.992, 0.04756),
    (122.992, 128.692, 0.29718),
    (121.8, 129.186, 0.04756),
    (103.2, 129.186, 0.68571),
    (102.008, 128.692, 0.04756),
    (96.308, 122.992, 0.29718),
    (95.814, 121.8, 0.04756),
    (95.814, 103.2, 0.68571),
    (96.308, 102.008, 0.04756),
    (99.13, 99.186, 0.14711),
];

fn voron_moves() -> (Vec<geometry::Move>, Vec<f64>) {
    let start = [99.158, 99.158, 0.2];
    let mut prev = start;
    let mut moves = Vec::new();
    for (i, (x, y, e)) in VORON_PERIMETER.into_iter().enumerate() {
        let end = [x, y, 0.2];
        moves.push(line(i as u32 + 1, prev, end, e));
        prev = end;
    }
    (moves, vec![start[0], start[1], start[2], 0.0])
}

#[test]
fn voron_cube_perimeter_replays_contiguously() {
    let (moves, home) = voron_moves();
    let segs = replay(cfg(), AxisChainSet::default(), &home, 0.0, &moves);
    assert!(!segs.is_empty());
    assert_time_contiguous(&segs);
    assert_position_contiguous(&segs);
    let last = segs.last().unwrap();
    let (x_end, y_end, _) = VORON_PERIMETER[VORON_PERIMETER.len() - 1];
    assert!((eval(&last.axes[0], last.t_end) - x_end).abs() < 1e-4);
    assert!((eval(&last.axes[1], last.t_end) - y_end).abs() < 1e-4);
}

#[test]
fn cold_run_infill_replays_without_overcommit() {
    // Real infill prefix from cold_run.gcode (Neptune bench) — the path that
    // aborted klippy mid-print with `velocity plan: OverCommitted`. Replaying it
    // must not panic anywhere in the pipeline; the emission boundary invariant
    // (barrier + setback) is what protects the warm-start entry velocity.
    let start = [99.158, 99.158, 0.0];
    let pts: [(f64, f64); 91] = [
        (99.158, 99.158),
        (102.008, 96.308),
        (103.2, 95.814),
        (121.8, 95.814),
        (122.992, 96.308),
        (128.692, 102.008),
        (129.186, 103.2),
        (129.186, 121.8),
        (128.692, 122.992),
        (122.992, 128.692),
        (121.8, 129.186),
        (103.2, 129.186),
        (102.008, 128.692),
        (96.308, 122.992),
        (95.814, 121.8),
        (95.814, 103.2),
        (96.308, 102.008),
        (99.13, 99.186),
        (99.453, 99.51),
        (102.331, 96.631),
        (103.2, 96.271),
        (121.8, 96.271),
        (122.669, 96.631),
        (128.369, 102.331),
        (128.729, 103.2),
        (128.729, 121.8),
        (128.369, 122.669),
        (122.669, 128.369),
        (121.8, 128.729),
        (103.2, 128.729),
        (102.331, 128.369),
        (96.631, 122.669),
        (96.271, 121.8),
        (96.271, 103.2),
        (96.631, 102.331),
        (99.425, 99.538),
        (121.445, 127.05),
        (103.555, 127.05),
        (97.95, 121.445),
        (97.95, 103.555),
        (103.555, 97.95),
        (121.445, 97.95),
        (127.05, 103.555),
        (127.05, 121.445),
        (121.474, 127.022),
        (108.669, 105.367),
        (108.715, 105.339),
        (109.475, 104.986),
        (110.267, 104.714),
        (111.083, 104.525),
        (111.913, 104.422),
        (112.751, 104.404),
        (113.598, 104.474),
        (114.808, 104.736),
        (115.602, 105.018),
        (116.357, 105.378),
        (117.072, 105.814),
        (117.748, 106.33),
        (118.626, 107.202),
        (119.143, 107.866),
        (119.586, 108.577),
        (119.953, 109.33),
        (120.24, 110.116),
        (120.445, 110.928),
        (120.565, 111.756),
        (120.599, 112.593),
        (120.567, 113.226),
        (120.449, 114.051),
        (120.247, 114.864),
        (119.961, 115.651),
        (119.596, 116.404),
        (119.148, 117.127),
        (118.365, 118.085),
        (117.754, 118.664),
        (117.089, 119.174),
        (116.376, 119.612),
        (115.621, 119.974),
        (114.833, 120.256),
        (114.019, 120.456),
        (113.19, 120.57),
        (112.353, 120.598),
        (111.518, 120.54),
        (110.68, 120.393),
        (109.499, 120.023),
        (108.734, 119.671),
        (108.014, 119.244),
        (107.342, 118.745),
        (106.725, 118.179),
        (106.169, 117.553),
        (105.85, 117.108),
        (105.267, 116.145),
    ];
    let mut prev = start;
    let mut moves = Vec::new();
    for (i, (x, y)) in pts.into_iter().enumerate() {
        let end = [x, y, 0.0];
        if dist3(prev, end) < 1e-9 {
            continue;
        }
        moves.push(line_bench(i as u32 + 1, prev, end));
        prev = end;
    }
    let segs = replay(
        cfg_bench(),
        AxisChainSet::default(),
        &[start[0], start[1], start[2], 0.0],
        0.0,
        &moves,
    );
    assert!(!segs.is_empty());
    assert_time_contiguous(&segs);
    assert_position_contiguous(&segs);
}

#[test]
fn collinear_jogs_cruise_through_the_seam() {
    let moves = [
        line(1, [0.0, 0.0, 0.0], [50.0, 0.0, 0.0], 0.0),
        line(2, [50.0, 0.0, 0.0], [100.0, 0.0, 0.0], 0.0),
    ];
    let segs = replay(
        cfg(),
        AxisChainSet::default(),
        &[0.0, 0.0, 0.0],
        0.0,
        &moves,
    );
    assert!(!segs.is_empty());
    let last = segs.last().unwrap();
    assert!((eval(&last.axes[0], last.t_end) - 100.0).abs() < 1e-6);
    // The seam at x=50 is interior; the toolhead must cruise through it.
    for w in segs.windows(2) {
        let x = eval(&w[0].axes[0], w[0].t_end);
        if (x - 50.0).abs() < 1e-6 {
            let v = boundary_speed(&w[0], &w[1]);
            assert!(v > 1.0, "collinear seam stalled: {v} mm/s at x={x}");
        }
    }
}

#[test]
fn blended_corner_is_rounded_not_stopped() {
    // A 90-degree corner is blended (a biclothoid). No interior boundary of the
    // replay may drop to rest — the corner is rounded, not stopped.
    let moves = [
        line(1, [0.0, 0.0, 0.0], [50.0, 0.0, 0.0], 0.0),
        line(2, [50.0, 0.0, 0.0], [50.0, 50.0, 0.0], 0.0),
    ];
    let segs = replay(
        cfg(),
        AxisChainSet::default(),
        &[0.0, 0.0, 0.0],
        0.0,
        &moves,
    );
    assert!(!segs.is_empty());
    for w in segs.windows(2) {
        let v = boundary_speed(&w[0], &w[1]);
        assert!(v > 1.0, "interior boundary stalled at {v} mm/s");
    }
    let last = segs.last().unwrap();
    assert!((eval(&last.axes[0], last.t_end) - 50.0).abs() < 1e-6);
    assert!((eval(&last.axes[1], last.t_end) - 50.0).abs() < 1e-6);
}

#[test]
fn continuous_blended_chain_never_pauses() {
    // A gentle zigzag: every corner is shallow enough to blend, so there is no
    // stop seam anywhere. No interior boundary may drop to rest.
    let pts = [
        [0.0, 0.0, 0.0],
        [20.0, 0.0, 0.0],
        [40.0, 3.0, 0.0],
        [60.0, 0.0, 0.0],
        [80.0, 3.0, 0.0],
        [100.0, 0.0, 0.0],
        [120.0, 3.0, 0.0],
    ];
    let moves: Vec<geometry::Move> = pts
        .windows(2)
        .enumerate()
        .map(|(i, w)| line(i as u32 + 1, w[0], w[1], 0.0))
        .collect();
    let segs = replay(
        cfg(),
        AxisChainSet::default(),
        &[0.0, 0.0, 0.0],
        0.0,
        &moves,
    );
    assert_time_contiguous(&segs);
    for w in segs.windows(2) {
        let v = boundary_speed(&w[0], &w[1]);
        assert!(v > 1.0, "interior boundary stalled at {v} mm/s");
    }
}

#[test]
fn extrusion_is_conserved_and_continuous_across_blends() {
    let moves = [
        line(1, [0.0, 0.0, 0.0], [50.0, 0.0, 0.0], 5.0),
        line(2, [50.0, 0.0, 0.0], [50.0, 50.0, 0.0], 5.0),
        line(3, [50.0, 50.0, 0.0], [100.0, 50.0, 0.0], 5.0),
    ];
    let segs = replay(
        cfg(),
        AxisChainSet::default(),
        &[0.0, 0.0, 0.0, 0.0],
        0.0,
        &moves,
    );
    assert_position_contiguous(&segs);
    let last = segs.last().unwrap();
    assert!(
        (eval(&last.axes[3], last.t_end) - 15.0).abs() < 1e-3,
        "total extrusion must be conserved"
    );
}

fn axis_velocity(seg: &ShapedSegment, axis: usize, t: f64) -> f64 {
    let h = 1e-6;
    (eval(&seg.axes[axis], t + h) - eval(&seg.axes[axis], t - h)) / (2.0 * h)
}

#[test]
fn extrusion_velocity_is_continuous_across_a_ramped_blend() {
    // A shallow corner between two legs at different extrusion ratios (0.20 vs
    // ~0.24, under the gate) blends and cruises near feedrate. The extrusion ramp
    // keeps ė continuous: with the old constant-ratio halves the blend midpoint
    // would step by (Δratio)·v ≈ 3 mm/s at this speed. Every interior E-axis seam
    // stays well under that.
    let moves = [
        line(1, [0.0, 0.0, 0.0], [40.0, 0.0, 0.0], 8.0),
        line(2, [40.0, 0.0, 0.0], [80.0, 4.0, 0.0], 9.65),
    ];
    let segs = replay(
        cfg(),
        AxisChainSet::default(),
        &[0.0, 0.0, 0.0, 0.0],
        0.0,
        &moves,
    );
    assert!(segs.len() > 2, "expected a rounded (multi-piece) corner");
    let mut worst = 0.0_f64;
    for w in segs.windows(2) {
        let spatial = boundary_speed(&w[0], &w[1]);
        assert!(
            spatial > 1.0,
            "corner stopped ({spatial} mm/s): not blended"
        );
        let jump =
            (axis_velocity(&w[1], 3, w[1].t_start) - axis_velocity(&w[0], 3, w[0].t_end)).abs();
        worst = worst.max(jump);
    }
    assert!(
        worst < 1.0,
        "extrusion velocity discontinuous across blend: {worst} mm/s"
    );
    let last = segs.last().unwrap();
    assert!((eval(&last.axes[3], last.t_end) - 17.65).abs() < 1e-3);
}

#[test]
fn abrupt_extrusion_step_rests_at_the_seam() {
    // Legs at 0.10 vs 0.30 extrusion ratio — above the gate. The corner is left
    // unblended, so the toolhead comes to rest across the seam.
    let moves = [
        line(1, [0.0, 0.0, 0.0], [50.0, 0.0, 0.0], 5.0),
        line(2, [50.0, 0.0, 0.0], [50.0, 50.0, 0.0], 15.0),
    ];
    let segs = replay(
        cfg(),
        AxisChainSet::default(),
        &[0.0, 0.0, 0.0, 0.0],
        0.0,
        &moves,
    );
    let min_seam = segs
        .windows(2)
        .map(|w| boundary_speed(&w[0], &w[1]))
        .fold(f64::INFINITY, f64::min);
    assert!(
        min_seam < 1e-2,
        "abrupt extrusion step should rest at the seam, min seam speed {min_seam}"
    );
}

#[test]
fn odometer_accumulates_extrusion_across_emissions() {
    let moves = [
        line(1, [0.0, 0.0, 0.0], [40.0, 0.0, 0.0], 4.0),
        line(2, [40.0, 0.0, 0.0], [80.0, 0.0, 0.0], 4.0),
    ];
    let segs = replay(
        cfg(),
        AxisChainSet::default(),
        &[0.0, 0.0, 0.0, 0.0],
        0.0,
        &moves,
    );
    let last = segs.last().unwrap();
    assert!((eval(&last.axes[3], last.t_end) - 8.0).abs() < 1e-3);
}

#[test]
fn trajectory_time_starts_at_the_pipeline_anchor() {
    let moves = [
        line(1, [0.0, 0.0, 0.0], [30.0, 0.0, 0.0], 0.0),
        line(2, [30.0, 0.0, 0.0], [60.0, 0.0, 0.0], 0.0),
        line(3, [60.0, 0.0, 0.0], [90.0, 0.0, 0.0], 0.0),
    ];
    let segs = replay(
        cfg(),
        AxisChainSet::default(),
        &[0.0, 0.0, 0.0],
        2.0,
        &moves,
    );
    assert_eq!(segs[0].t_start, 2.0);
    assert_time_contiguous(&segs);
}

#[test]
fn drained_prefix_is_invariant_under_append() {
    // Emission finality: everything the pipeline emitted well clear of the
    // brake-to-rest setback must be identical whether or not more moves follow.
    // Compare a short replay against a longer one on the early segments (first
    // half of the short run's timeline — safely inside the final region).
    let mk = |n: usize| -> Vec<geometry::Move> {
        (0..n)
            .map(|i| {
                let x0 = i as f64 * 20.0;
                let y0 = if i % 2 == 0 { 0.0 } else { 3.0 };
                let y1 = if i % 2 == 0 { 3.0 } else { 0.0 };
                line(i as u32 + 1, [x0, y0, 0.0], [x0 + 20.0, y1, 0.0], 0.0)
            })
            .collect()
    };
    let short = replay(
        cfg(),
        AxisChainSet::default(),
        &[0.0, 0.0, 0.0],
        0.0,
        &mk(12),
    );
    let long = replay(
        cfg(),
        AxisChainSet::default(),
        &[0.0, 0.0, 0.0],
        0.0,
        &mk(20),
    );
    let horizon = 0.5 * short.last().unwrap().t_end;
    let mut compared = 0usize;
    for (a, b) in short.iter().zip(&long) {
        if a.t_end > horizon {
            break;
        }
        assert!((a.t_start - b.t_start).abs() < 1e-3);
        assert!((a.t_end - b.t_end).abs() < 1e-3);
        for axis in 0..2 {
            let da = eval(&a.axes[axis], a.t_end);
            let db = eval(&b.axes[axis], b.t_end);
            assert!(
                (da - db).abs() < 1e-9,
                "seg {compared} axis {axis}: {da} vs {db}"
            );
        }
        compared += 1;
    }
    assert!(compared > 0, "nothing inside the comparison horizon");
}

fn smooth_x_chains(frequency_hz: f64) -> AxisChainSet {
    AxisChainSet::spatial(
        trajectory::CompiledChain::compile(&[PostProcessorInstance::new(
            "is",
            &trajectory::algos::SmoothZv,
            vec![frequency_hz],
        )])
        .expect("single post-processor always compiles"),
        trajectory::CompiledChain::default(),
        trajectory::CompiledChain::default(),
    )
}

#[test]
fn smooth_shaper_output_matches_shaped_signal_oracle() {
    let moves = [
        line(1, [0.0, 0.0, 0.0], [80.0, 0.0, 0.0], 0.0),
        line(2, [80.0, 0.0, 0.0], [160.0, 0.0, 0.0], 0.0),
    ];
    let base = replay(
        cfg(),
        AxisChainSet::default(),
        &[0.0, 0.0, 0.0],
        0.0,
        &moves,
    );
    let shaped = replay(cfg(), smooth_x_chains(18.0), &[0.0, 0.0, 0.0], 0.0, &moves);
    assert_eq!(
        base.len() + 1,
        shaped.len(),
        "a kernel chain starting from rest pads a leading hold segment"
    );
    let pad = shaped[1].t_start - base[0].t_start;
    assert!(pad > 0.0, "hold pad must shift the move start forward");
    let shaped = &shaped[1..];

    let oracle_chains = smooth_x_chains(18.0);
    let trajectory::ChainStage::SmoothKernel(kernel) = &oracle_chains.chains[0].stages[0] else {
        panic!("expected smooth kernel");
    };
    let first = base.first().unwrap().t_start;
    let last = base.last().unwrap().t_end;

    for (base_seg, shaped_seg) in base.iter().zip(shaped) {
        let mut breaks: Vec<f64> = Vec::new();
        for seg in &base {
            breaks.push(seg.t_start);
            breaks.extend_from_slice(seg.axes[0].knots());
            breaks.push(seg.t_end);
        }
        let sig = trajectory::ShapedSignal::new_from_evaluator(
            kernel,
            |t| {
                let clamped = t.clamp(first, last);
                base.iter()
                    .find(|seg| clamped >= seg.t_start && clamped <= seg.t_end)
                    .map_or_else(
                        || eval(&base.last().unwrap().axes[0], clamped),
                        |seg| eval(&seg.axes[0], clamped),
                    )
            },
            breaks,
        );
        for frac in [0.1_f64, 0.3, 0.5, 0.7, 0.9] {
            let t = frac.mul_add(base_seg.t_end - base_seg.t_start, base_seg.t_start);
            let got = eval(&shaped_seg.axes[0], t + pad);
            let want = sig.eval(t);
            assert!(
                (got - want).abs() < 5e-2,
                "shaped x at t={t}: got {got}, want {want}"
            );
        }
    }
}

#[test]
fn smooth_shaper_with_wide_support_still_flushes_at_rest() {
    // A 0.5 Hz kernel's support is wider than the whole trajectory; the rest
    // ending lets the shaper clamp and flush instead of waiting forever.
    let moves = [
        line(1, [0.0, 0.0, 0.0], [20.0, 0.0, 0.0], 0.0),
        line(2, [20.0, 0.0, 0.0], [40.0, 0.0, 0.0], 0.0),
    ];
    let segs = replay(cfg(), smooth_x_chains(0.5), &[0.0, 0.0, 0.0], 0.0, &moves);
    assert!(!segs.is_empty(), "rest flush must release held segments");
}

#[test]
fn smooth_shaper_first_emission_after_nonzero_start_time_is_valid() {
    let moves = [line(1, [0.0, 0.0, 0.0], [20.0, 0.0, 0.0], 0.0)];
    let segs = replay(cfg(), smooth_x_chains(18.0), &[0.0, 0.0, 0.0], 5.0, &moves);
    assert_eq!(segs[0].t_start, 5.0);
}

/// A later emit batch whose back convolution window reaches before the stream
/// start must clamp to the stream-start rest (like the first batch does), not
/// die on "needs unavailable history": nothing was trimmed, so the signal
/// before the retained front is the same held rest. Regression for the
/// BEACON_POKE hang, where the first move after `SET_KINEMATIC_POSITION` +
/// `G4 P1000` panicked the shape thread at t = 1.0 - 1ulp.
#[test]
fn smooth_shaper_second_batch_window_before_stream_start_clamps() {
    let chains = smooth_x_chains(18.0);
    let (_, back) = chains.chains[0].max_half_support();
    let back = back.abs();
    let t0 = 1.0;
    let step = 0.4 * back;

    let constant_seg = |t_start: f64, t_end: f64| ShapedSegment {
        axes: (0..3)
            .map(|_| {
                nurbs::bezier::bezier_pieces_to_nurbs(&[nurbs::bezier::BezierPiece {
                    u_start: t_start,
                    u_end: t_end,
                    coeffs: vec![150.0],
                }])
            })
            .collect(),
        followers: vec![],
        t_start,
        t_end,
        motor_mask: 0,
        source_line: 1,
    };

    let (lowered_tx, lowered_rx) = unbounded();
    for i in 0..8 {
        let (a, b) = (i as f64, (i + 1) as f64);
        lowered_tx
            .send(LoweredItem::Seg(LoweredSegment {
                seg: constant_seg(a.mul_add(step, t0), b.mul_add(step, t0)),
                rest_at_end: true,
            }))
            .unwrap();
    }
    drop(lowered_tx);

    let (shaped_tx, shaped_rx) = unbounded();
    Shaper::new(chains).run(lowered_rx, shaped_tx);

    let segs: Vec<ShapedSegment> = shaped_rx
        .into_iter()
        .filter_map(|item| match item {
            ShapedItem::Seg(seg) => Some(seg),
            ShapedItem::Control(_) => None,
        })
        .collect();
    assert_eq!(segs.len(), 8);
    for seg in &segs {
        let mid = 0.5 * (seg.t_start + seg.t_end);
        let got = eval(&seg.axes[0], mid);
        assert!(
            (got - 150.0).abs() < 1e-3,
            "seg [{}, {}]: shaped constant drifted to {got}",
            seg.t_start,
            seg.t_end,
        );
    }
}

const NEPTUNE_SCV25_FILLET: [(f64, f64, f64); 7] = [
    (124.102, 100.688, 0.01272),
    (124.102, 101.679, 0.03333),
    (124.13, 101.951, 0.00921),
    (124.178, 102.09, 0.00494),
    (122.055, 101.676, 0.07276),
    (121.2, 100.821, 0.04068),
    (120.826, 98.898, 0.0659),
];

#[test]
fn arc_run_into_sharp_corner_stays_contiguous_at_high_scv() {
    // Voron cube Z6.2 fillet slice that crashed the Neptune bench mid-print:
    // with arc fitting enabled and square_corner_velocity raised to 25, the
    // fit stage emitted a 0.27mm gap between the corner blend leaving the fitted
    // run and the following long line, tripping the TravelAligningSender
    // contiguity assert.
    let limits = VelocityLimits::try_new(100.0, 1000.0, 25.0, 1_000_000.0).unwrap();
    let config = StreamConfig {
        chain: ChainFitConfig::with_arc_fit(3),
        integration_tol: 1e-4,
        max_extrude_only_velocity_mm_s: f64::INFINITY,
        max_extrude_only_accel_mm_s2: f64::INFINITY,
        fit_tol_mm: 0.005,
        fit_tol_accel_mm_s2: 50.0,
        max_buffer_moves: 512,
        limits,
    };
    let mut prev = [124.155, 100.313, 0.0];
    let mut moves = Vec::new();
    for (i, (x, y, e)) in NEPTUNE_SCV25_FILLET.into_iter().enumerate() {
        let end = [x, y, 0.0];
        let ctx = MoveContext {
            extruder_axis: 3,
            feedrate_mm_s: 80.0,
            limits,
            source: SourceRange {
                start_line: i as u32 + 1,
                end_line: i as u32 + 1,
            },
        };
        moves.push(line_move(prev, end, e, ctx).unwrap());
        prev = end;
    }
    let home = vec![124.155, 100.313, 0.0, 0.0];
    let segs = replay(config, AxisChainSet::default(), &home, 0.0, &moves);
    assert!(!segs.is_empty());
    assert_position_contiguous(&segs);
}

#[test]
fn blends_consuming_a_full_arc_emit_no_degenerate_remainder() {
    // From the same print at SCV 25, one layer up: the corner blends at both
    // ends of a short fitted arc consume its entire length. The remainder
    // (2e-16 mm) must be skipped, not emitted — the planner rejects segments
    // at or below its 1e-9 mm length epsilon.
    let limits = VelocityLimits::try_new(100.0, 1000.0, 25.0, 1_000_000.0).unwrap();
    let config = StreamConfig {
        chain: ChainFitConfig::with_arc_fit(3),
        integration_tol: 1e-4,
        max_extrude_only_velocity_mm_s: f64::INFINITY,
        max_extrude_only_accel_mm_s2: f64::INFINITY,
        fit_tol_mm: 0.005,
        fit_tol_accel_mm_s2: 50.0,
        max_buffer_moves: 512,
        limits,
    };
    let points = [
        (118.225, 104.096, 0.0, 150.0),
        (118.295, 103.844, 0.00878, 98.87),
        (118.314, 103.664, 0.00609, 98.87),
        (118.315, 102.438, 0.04125, 98.87),
        (121.855, 98.898, 0.16839, 98.87),
        (126.102, 98.898, 0.14286, 98.87),
        (118.153, 106.847, 0.37817, 98.87),
    ];
    let mut prev = [114.719, 104.739, 0.0];
    let mut moves = Vec::new();
    for (i, (x, y, e, feed)) in points.into_iter().enumerate() {
        let end = [x, y, 0.0];
        let ctx = MoveContext {
            extruder_axis: 3,
            feedrate_mm_s: feed,
            limits,
            source: SourceRange {
                start_line: i as u32 + 1,
                end_line: i as u32 + 1,
            },
        };
        moves.push(line_move(prev, end, e, ctx).unwrap());
        prev = end;
    }
    let home = vec![114.719, 104.739, 0.0, 0.0];
    let segs = replay(config, AxisChainSet::default(), &home, 0.0, &moves);
    assert!(!segs.is_empty());
    assert_position_contiguous(&segs);
}

fn replay_inputs(
    config: StreamConfig,
    chains: AxisChainSet,
    home: &[f64],
    inputs: Vec<StreamInput>,
) -> Vec<ShapedSegment> {
    let (raw_tx, raw_rx) = unbounded();
    for item in inputs {
        raw_tx.send(item).unwrap();
    }
    drop(raw_tx);
    let (fitted_tx, fitted_rx) = unbounded();
    FitStage::new(config.chain).run(raw_rx, fitted_tx);
    let (planned_tx, planned_rx) = unbounded();
    Planner::new(config).run(fitted_rx, planned_tx);
    let (lowered_tx, lowered_rx) = unbounded();
    run_lowerer(
        planned_rx,
        lowered_tx,
        FitTol {
            pos_mm: config.fit_tol_mm,
            accel_mm_s2: config.fit_tol_accel_mm_s2,
        },
        chains.clone(),
        home.to_vec(),
        0.0,
    );
    let (shaped_tx, shaped_rx) = unbounded();
    Shaper::new(chains).run(lowered_rx, shaped_tx);
    shaped_rx
        .into_iter()
        .filter_map(|item| match item {
            ShapedItem::Seg(seg) => Some(seg),
            ShapedItem::Control(_) => None,
        })
        .collect()
}

#[test]
fn mesh_warp_tracks_across_a_fenced_move_sequence() {
    let z = vec![0.10, 0.00, -0.10, 0.05, 0.00, -0.05, -0.10, 0.00, 0.10];
    let mut mesh = geometry::MeshGrid::new(20.0, 20.0, 100.0, 100.0, 3, 3, z, 0.2).unwrap();
    mesh.zero_at(120.0, 120.0);
    let transform = std::sync::Arc::new(geometry::SurfaceTransform::new(
        mesh,
        geometry::Fade::new(1.0, 10.0, 0.0).unwrap(),
    ));
    let t = transform.clone();

    let waypoints = [
        [120.0, 120.0, 5.0],
        [120.0, 120.0, 0.5],
        [20.0, 20.0, 0.5],
        [220.0, 20.0, 0.5],
        [220.0, 220.0, 0.5],
    ];
    let mut inputs = vec![StreamInput::Control(Control::SetMesh {
        mesh: Some(transform),
        gcode_z_rebase: 5.0,
    })];
    for (i, pair) in waypoints.windows(2).enumerate() {
        inputs.push(StreamInput::Move(
            line_move(pair[0], pair[1], 0.0, ctx(i as u32 + 1, 50.0)).unwrap(),
        ));
        inputs.push(StreamInput::Drain);
    }

    let xy_chain = trajectory::CompiledChain::compile(&[PostProcessorInstance::new(
        "is_xy",
        &trajectory::algos::SmoothMzv,
        vec![50.0],
    )])
    .unwrap();
    let chains = AxisChainSet::spatial(
        xy_chain.clone(),
        xy_chain,
        trajectory::CompiledChain::default(),
    );
    let home = vec![120.0, 120.0, 5.0, 0.0];
    let segs = replay_inputs(cfg_bench(), chains, &home, inputs);
    assert!(!segs.is_empty());

    let last = segs.last().unwrap();
    let t_end = last.t_end;
    let x = eval(&last.axes[0], t_end);
    let y = eval(&last.axes[1], t_end);
    let z_machine = eval(&last.axes[2], t_end);
    let expected = 0.5 + t.correction_at(220.0, 220.0, 0.5);
    assert!(
        (x - 220.0).abs() < 1e-2 && (y - 220.0).abs() < 1e-2,
        "sequence should end at (220,220), got ({x}, {y})"
    );
    assert!(
        (z_machine - expected).abs() < 5e-3,
        "final machine Z {z_machine} should be {expected}"
    );
}
