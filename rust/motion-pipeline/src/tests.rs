use crate::types::*;
use crate::*;
use crossbeam_channel::unbounded;
use geometry::segment::SourceRange;
use geometry::{CornerFitConfig, MoveContext, VelocityLimits, line_move};
use nurbs::eval::eval;
use trajectory::{AxisChainSet, PostProcessorInstance, ShapedSegment};

fn cfg() -> StreamConfig {
    StreamConfig {
        corner: CornerFitConfig::default(),
        integration_tol: 1e-7,
        max_extrude_only_velocity_mm_s: f64::INFINITY,
        max_extrude_only_accel_mm_s2: f64::INFINITY,
        fit_tol_mm: 1e-3,
        fit_tol_accel_mm_s2: 50.0,
        max_buffer_moves: 64,
        limits: VelocityLimits::try_new(
            300.0,
            5000.0,
            geometry::corner_deviation_from_scv(5.0, 5000.0),
            100_000.0,
        )
        .unwrap(),
    }
}

fn ctx(line_no: u32, feed: f64) -> MoveContext {
    MoveContext {
        extruder_axis: 3,
        feedrate_mm_s: feed,
        limits: VelocityLimits::try_new(
            300.0,
            5000.0,
            geometry::corner_deviation_from_scv(5.0, 5000.0),
            100_000.0,
        )
        .unwrap(),
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
        corner: CornerFitConfig::default(),
        integration_tol: 1e-4,
        max_extrude_only_velocity_mm_s: f64::INFINITY,
        max_extrude_only_accel_mm_s2: f64::INFINITY,
        fit_tol_mm: 0.005,
        fit_tol_accel_mm_s2: 50.0,
        max_buffer_moves: 512,
        limits: VelocityLimits::try_new(
            100.0,
            1000.0,
            geometry::corner_deviation_from_scv(5.0, 1000.0),
            1_000_000.0,
        )
        .unwrap(),
    }
}

fn line_bench(line_no: u32, start: [f64; 3], end: [f64; 3]) -> geometry::Move {
    let ctx = MoveContext {
        extruder_axis: 3,
        feedrate_mm_s: 60.0,
        limits: VelocityLimits::try_new(
            100.0,
            1000.0,
            geometry::corner_deviation_from_scv(5.0, 1000.0),
            1_000_000.0,
        )
        .unwrap(),
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
    FitStage::new(config.corner).run(raw_rx, fitted_tx);

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

/// The smooth_zv kernel is mean-centered, so its support is asymmetric
/// (`[-T/2 - mu, T/2 - mu]`, mu < 0): the convolution needs more history than
/// lookahead. Streaming many segments through the shaper forces history
/// trimming; a window/retention computation that assumes symmetric support
/// dies here with a non-finite sample. Regression for the reflected
/// input-window fix.
#[test]
fn asymmetric_kernel_survives_history_trimming_across_many_segments() {
    let chain = trajectory::CompiledChain::compile(&[PostProcessorInstance::new(
        "is",
        &trajectory::algos::SmoothZv,
        vec![130.0],
    )])
    .expect("single post-processor always compiles");
    let chains = AxisChainSet::spatial(
        chain,
        trajectory::CompiledChain::default(),
        trajectory::CompiledChain::default(),
    );
    let moves: Vec<geometry::Move> = (0..40)
        .map(|i| {
            let a = f64::from(i) * 2.0;
            line(i + 1, [a, 0.0, 0.0], [a + 2.0, 0.0, 0.0], 0.0)
        })
        .collect();
    let segs = replay(cfg(), chains, &[0.0, 0.0, 0.0], 0.0, &moves);
    assert!(!segs.is_empty());
    for seg in &segs {
        for curve in &seg.axes {
            assert!(curve.control_points().iter().all(|v| v.is_finite()));
        }
    }
    let last = segs.last().expect("non-empty");
    let final_x = eval(&last.axes[0], last.t_end);
    let shaped_fit_budget = 1e-3;
    assert!(
        (final_x - 80.0).abs() < shaped_fit_budget,
        "final x = {final_x}, t_end = {}",
        last.t_end
    );
}

fn smooth_x_chains(smooth_time: f64) -> AxisChainSet {
    AxisChainSet::spatial(
        trajectory::CompiledChain::compile(&[PostProcessorInstance::new(
            "is",
            &trajectory::algos::SmoothBell,
            vec![smooth_time],
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
    let shaped = replay(
        cfg(),
        smooth_x_chains(0.044583333333333336),
        &[0.0, 0.0, 0.0],
        0.0,
        &moves,
    );
    assert_eq!(
        base.len() + 2,
        shaped.len(),
        "a kernel chain pads a leading hold segment from rest and a trailing \
         settle hold at the drain"
    );
    let pad = shaped[1].t_start - base[0].t_start;
    assert!(pad > 0.0, "hold pad must shift the move start forward");
    let shaped = &shaped[1..shaped.len() - 1];

    let oracle_chains = smooth_x_chains(0.044583333333333336);
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
    let segs = replay(cfg(), smooth_x_chains(1.605), &[0.0, 0.0, 0.0], 0.0, &moves);
    assert!(!segs.is_empty(), "rest flush must release held segments");
}

#[test]
fn smooth_shaper_first_emission_after_nonzero_start_time_is_valid() {
    let moves = [line(1, [0.0, 0.0, 0.0], [20.0, 0.0, 0.0], 0.0)];
    let segs = replay(
        cfg(),
        smooth_x_chains(0.044583333333333336),
        &[0.0, 0.0, 0.0],
        5.0,
        &moves,
    );
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
    let chains = smooth_x_chains(0.044583333333333336);
    let (_, back) = chains.chains[0].max_input_window();
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
        spatial_path: false,
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
    let limits = VelocityLimits::try_new(
        100.0,
        1000.0,
        geometry::corner_deviation_from_scv(25.0, 1000.0),
        1_000_000.0,
    )
    .unwrap();
    let config = StreamConfig {
        corner: CornerFitConfig::default(),
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
    let limits = VelocityLimits::try_new(
        100.0,
        1000.0,
        geometry::corner_deviation_from_scv(25.0, 1000.0),
        1_000_000.0,
    )
    .unwrap();
    let config = StreamConfig {
        corner: CornerFitConfig::default(),
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
    FitStage::new(config.corner).run(raw_rx, fitted_tx);
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
        &trajectory::algos::SmoothBell,
        vec![0.019125],
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

fn xy_shaper_follower_chains(smooth_time: f64) -> AxisChainSet {
    let bell = |name: &str| {
        trajectory::CompiledChain::compile(&[PostProcessorInstance::new(
            name,
            &trajectory::algos::SmoothBell,
            vec![smooth_time],
        )])
        .expect("single post-processor always compiles")
    };
    AxisChainSet {
        chains: vec![
            bell("is_x"),
            bell("is_y"),
            trajectory::CompiledChain::default(),
            trajectory::CompiledChain::default(),
        ],
        followers: vec![(3, vec![0, 1, 2])],
    }
}

fn follower_chains_without_kernels() -> AxisChainSet {
    AxisChainSet {
        chains: vec![trajectory::CompiledChain::default(); 4],
        followers: vec![(3, vec![0, 1, 2])],
    }
}

fn sampled_planar_path_length(segs: &[ShapedSegment]) -> f64 {
    const SAMPLES_PER_SEG: usize = 2000;
    let mut length = 0.0;
    let mut prev: Option<(f64, f64)> = None;
    for seg in segs {
        for i in 0..=SAMPLES_PER_SEG {
            let t = seg.t_start + (seg.t_end - seg.t_start) * i as f64 / SAMPLES_PER_SEG as f64;
            let p = (eval(&seg.axes[0], t), eval(&seg.axes[1], t));
            if let Some(q) = prev {
                length += ((p.0 - q.0).powi(2) + (p.1 - q.1).powi(2)).sqrt();
            }
            prev = Some(p);
        }
    }
    length
}

fn extruder_end(segs: &[ShapedSegment]) -> f64 {
    let last = segs.last().expect("segments emitted");
    eval(&last.axes[3], last.t_end)
}

fn assert_extruder_continuous_and_monotone(segs: &[ShapedSegment]) {
    let mut prev_val: Option<f64> = None;
    for seg in segs {
        for i in 0..=200 {
            let t = seg.t_start + (seg.t_end - seg.t_start) * i as f64 / 200.0;
            let v = eval(&seg.axes[3], t);
            if let Some(p) = prev_val {
                assert!(
                    v >= p - 1e-6,
                    "extruder track regressed from {p} to {v} at t={t}"
                );
                assert!(
                    v - p < 0.05,
                    "extruder track jumped from {p} to {v} at t={t}"
                );
            }
            prev_val = Some(v);
        }
    }
}

/// The follower rides the path's true distance at the commanded rate:
/// through a blended (and further kernel-smoothed) corner it extrudes
/// exactly ratio × the (shorter) actual arc length, so it ends short of the
/// gcode total by the corner-cut distance.
#[test]
fn follower_tracks_shaped_path_distance_through_a_corner() {
    let moves = [
        line(1, [0.0, 0.0, 0.0], [30.0, 0.0, 0.0], 1.5),
        line(2, [30.0, 0.0, 0.0], [30.0, 30.0, 0.0], 1.5),
    ];
    let home = [0.0, 0.0, 0.0, 0.0];

    let raw = replay(cfg(), follower_chains_without_kernels(), &home, 0.0, &moves);
    let raw_len = sampled_planar_path_length(&raw);
    assert!(
        raw_len < 60.0 && (extruder_end(&raw) - 0.05 * raw_len).abs() < 2e-3,
        "without leader kernels the follower rides the fitted arc length: \
         got {} vs 0.05 × {raw_len}",
        extruder_end(&raw)
    );

    let shaped = replay(
        cfg(),
        xy_shaper_follower_chains(0.044583333333333336),
        &home,
        0.0,
        &moves,
    );
    assert_extruder_continuous_and_monotone(&shaped);
    let e_end = extruder_end(&shaped);
    let shaped_len = sampled_planar_path_length(&shaped);
    assert!(
        shaped_len < 60.0 + 1e-6,
        "shaped path cannot be longer than commanded: {shaped_len}"
    );
    assert!(
        (e_end - 0.05 * shaped_len).abs() < 2e-3,
        "extruder must ride the shaped arc length: e_end {e_end} vs \
         0.05 × {shaped_len} = {}",
        0.05 * shaped_len
    );
}

/// An extrude-only move rides no spatial path; its raw track passes through
/// the projection and the totals add up on a straight (cut-free) path.
#[test]
fn extrude_only_move_passes_through_the_projection() {
    let moves = [
        line(1, [0.0, 0.0, 0.0], [20.0, 0.0, 0.0], 1.0),
        line(2, [20.0, 0.0, 0.0], [20.0, 0.0, 0.0], -0.5),
        line(3, [20.0, 0.0, 0.0], [40.0, 0.0, 0.0], 1.0),
    ];
    let home = [0.0, 0.0, 0.0, 0.0];
    let shaped = replay(
        cfg(),
        xy_shaper_follower_chains(0.044583333333333336),
        &home,
        0.0,
        &moves,
    );
    let e_end = extruder_end(&shaped);
    assert!(
        (e_end - 1.5).abs() < 1e-3,
        "straight path with a retract must extrude the commanded total: {e_end}"
    );
}

fn xy_follower_chains_with_optional_inverse(smooth_time: f64, with_inverse: bool) -> AxisChainSet {
    let spatial = |name: &str| {
        let mut instances = vec![PostProcessorInstance::new(
            name,
            &trajectory::algos::SmoothBell,
            vec![smooth_time],
        )];
        if with_inverse {
            instances.push(PostProcessorInstance::new(
                "mi",
                &trajectory::algos::ModeInverse,
                vec![130.0, 0.1],
            ));
        }
        trajectory::CompiledChain::compile(&instances).expect("bell + mode_inverse compiles")
    };
    AxisChainSet {
        chains: vec![
            spatial("is_x"),
            spatial("is_y"),
            trajectory::CompiledChain::default(),
            e_chain(Some(0.03), 0.02),
        ],
        followers: vec![(3, vec![0, 1, 2])],
    }
}

/// A trailing mode-inverse stage produces the motor command; the physical
/// toolhead tracks the kernel output, and the follower must ride the
/// toolhead. Adding the inverse to the leaders therefore must not change the
/// extruder track at all — while visibly changing the leaders' own output.
#[test]
fn follower_projects_onto_toolhead_signal_not_motor_command() {
    let moves = [
        line(1, [0.0, 0.0, 0.0], [20.0, 0.0, 0.0], 1.0),
        line(2, [20.0, 0.0, 0.0], [20.0, 20.0, 0.0], 2.0),
    ];
    let home = [0.0, 0.0, 0.0, 0.0];
    let plain = replay(
        cfg(),
        xy_follower_chains_with_optional_inverse(0.02, false),
        &home,
        0.0,
        &moves,
    );
    let inverted = replay(
        cfg(),
        xy_follower_chains_with_optional_inverse(0.02, true),
        &home,
        0.0,
        &moves,
    );
    assert_eq!(plain.len(), inverted.len());
    let mut max_e_diff: f64 = 0.0;
    let mut max_x_diff: f64 = 0.0;
    for (p, q) in plain.iter().zip(&inverted) {
        for i in 0..=100 {
            let t = p.t_start + (p.t_end - p.t_start) * f64::from(i) / 100.0;
            max_e_diff = max_e_diff.max((eval(&p.axes[3], t) - eval(&q.axes[3], t)).abs());
            max_x_diff = max_x_diff.max((eval(&p.axes[0], t) - eval(&q.axes[0], t)).abs());
        }
    }
    assert!(
        max_e_diff < 1e-9,
        "extruder must ride the toolhead signal, unchanged by the leader's \
         motor-side inverse; diff = {max_e_diff}"
    );
    assert!(
        max_x_diff > 1e-3,
        "sanity: the inverse must visibly counter-drive the leader itself; \
         diff = {max_x_diff}"
    );
}

fn e_chain(k: Option<f64>, e_smooth_time: f64) -> trajectory::CompiledChain {
    let mut instances = Vec::new();
    if let Some(k) = k {
        instances.push(PostProcessorInstance::new(
            "pa",
            &trajectory::algos::LinearPressureAdvance,
            vec![k],
        ));
    }
    if e_smooth_time > 0.0 {
        instances.push(PostProcessorInstance::new(
            "st",
            &trajectory::algos::SmoothBell,
            vec![e_smooth_time],
        ));
    }
    trajectory::CompiledChain::compile(&instances).expect("pa + kernel compiles")
}

fn follower_kernel_chains(
    leader_smooth_time: Option<f64>,
    k: Option<f64>,
    e_smooth_time: f64,
) -> AxisChainSet {
    let mut chains =
        leader_smooth_time.map_or_else(follower_chains_without_kernels, xy_shaper_follower_chains);
    chains.chains[3] = e_chain(k, e_smooth_time);
    chains
}

fn nonlinear_e_chain(
    algo: &'static dyn trajectory::algos::PostProcessorAlgo,
    linear_advance: f64,
    nonlinear_offset: f64,
    linearization_velocity: f64,
    e_smooth_time: f64,
) -> trajectory::CompiledChain {
    let mut instances = vec![PostProcessorInstance::new(
        "nlpa",
        algo,
        vec![linear_advance, nonlinear_offset, linearization_velocity],
    )];
    if e_smooth_time > 0.0 {
        instances.push(PostProcessorInstance::new(
            "st",
            &trajectory::algos::SmoothBell,
            vec![e_smooth_time],
        ));
    }
    trajectory::CompiledChain::compile(&instances).expect("nonlinear pa + kernel compiles")
}

fn sample_extruder(segs: &[ShapedSegment]) -> Vec<(f64, f64)> {
    let mut samples = Vec::new();
    for seg in segs {
        for i in 0..=200 {
            let t = seg.t_start + (seg.t_end - seg.t_start) * i as f64 / 200.0;
            samples.push((t, eval(&seg.axes[3], t)));
        }
    }
    samples
}

fn assert_extruder_has_no_jumps(segs: &[ShapedSegment]) {
    let mut prev: Option<(f64, f64)> = None;
    for (t, v) in sample_extruder(segs) {
        if let Some((_, p)) = prev {
            assert!(
                (v - p).abs() < 0.05,
                "extruder track jumped from {p} to {v} at t={t}"
            );
        }
        prev = Some((t, v));
    }
}

/// With unshaped leaders the projection is a passthrough, so a chain on the
/// follower must land where the same chain lands on the same axis declared
/// as a plain non-follower. The two declarations convolve identical inputs
/// but fit with different budgets (a follower's budget is scaled by its
/// demand ratio, a plain axis has no demand), so they agree to the sum of
/// the two fit budgets, not to bits; with PA they additionally bake it into
/// different fit stages (the lowerer fits the PA-boosted signal, the
/// projection applies PA exactly to the fitted raw track).
#[test]
fn follower_kernel_with_unshaped_leaders_matches_direct_convolution() {
    let moves = [
        line(1, [0.0, 0.0, 0.0], [30.0, 0.0, 0.0], 1.5),
        line(2, [30.0, 0.0, 0.0], [30.0, 30.0, 0.0], 1.5),
    ];
    let home = [0.0, 0.0, 0.0, 0.0];
    for (k, tol) in [(None, 2e-3), (Some(0.04), 2e-2)] {
        let as_follower = follower_kernel_chains(None, k, 0.02675);
        let mut as_plain_axis = as_follower.clone();
        as_plain_axis.followers.clear();

        let follower = replay(cfg(), as_follower, &home, 0.0, &moves);
        let direct = replay(cfg(), as_plain_axis, &home, 0.0, &moves);
        let a = sample_extruder(&follower);
        let b = sample_extruder(&direct);
        assert_eq!(a.len(), b.len(), "same segmentation expected");
        for ((ta, va), (tb, vb)) in a.iter().zip(&b) {
            assert!((ta - tb).abs() < 1e-9);
            assert!(
                (va - vb).abs() < tol,
                "follower chain (k={k:?}) diverged from the direct \
                 convolution at t={ta}: {va} vs {vb}"
            );
        }
    }
}

/// A kernel on the follower smooths the *projected* signal: the track stays
/// continuous and monotone, and the total still rides the shaped arc length
/// (the kernel preserves the endpoint once the stream is at rest).
#[test]
fn follower_kernel_rides_the_projection_through_a_corner() {
    let moves = [
        line(1, [0.0, 0.0, 0.0], [30.0, 0.0, 0.0], 1.5),
        line(2, [30.0, 0.0, 0.0], [30.0, 30.0, 0.0], 1.5),
    ];
    let home = [0.0, 0.0, 0.0, 0.0];
    let shaped = replay(
        cfg(),
        follower_kernel_chains(Some(0.044583333333333336), None, 0.02675),
        &home,
        0.0,
        &moves,
    );
    assert_extruder_continuous_and_monotone(&shaped);
    let e_end = extruder_end(&shaped);
    let shaped_len = sampled_planar_path_length(&shaped);
    assert!(
        (e_end - 0.05 * shaped_len).abs() < 2e-3,
        "kernelled follower must still ride the shaped arc length: e_end \
         {e_end} vs 0.05 × {shaped_len} = {}",
        0.05 * shaped_len
    );
}

/// Full smooth-pressure-advance on a projected follower: PA boosts the
/// projected flow, the follower's own kernel smooths the boosted signal, and
/// once the flow settles (a trailing travel move, where the ratio is zero
/// and PA's velocity term dies) the extruded total is unchanged by PA.
#[test]
fn smooth_pressure_advance_on_follower_preserves_the_projected_total() {
    let moves = [
        line(1, [0.0, 0.0, 0.0], [30.0, 0.0, 0.0], 1.5),
        line(2, [30.0, 0.0, 0.0], [30.0, 30.0, 0.0], 1.5),
        line(3, [30.0, 30.0, 0.0], [0.0, 30.0, 0.0], 0.0),
    ];
    let home = [0.0, 0.0, 0.0, 0.0];
    let with_pa = replay(
        cfg(),
        follower_kernel_chains(Some(0.044583333333333336), Some(0.04), 0.02675),
        &home,
        0.0,
        &moves,
    );
    assert_extruder_has_no_jumps(&with_pa);
    let without_pa = replay(
        cfg(),
        follower_kernel_chains(Some(0.044583333333333336), None, 0.02675),
        &home,
        0.0,
        &moves,
    );
    let (e_pa, e_plain) = (extruder_end(&with_pa), extruder_end(&without_pa));
    assert!(
        (e_pa - e_plain).abs() < 1e-4,
        "PA must not change the settled extruded total: {e_pa} vs {e_plain}"
    );
}

/// Nonlinear PA on the projected follower: matched to the linear model's
/// small-signal slope, each saturating model must fall short of the linear
/// one at cruise while still settling on the same extruded total once the
/// flow stops. `recipr` rises toward the bound more slowly than `tanh`, so
/// it must command the least advance of the three.
#[test]
fn nonlinear_pressure_advance_saturates_against_the_matched_linear_model() {
    let moves = [
        line(1, [0.0, 0.0, 0.0], [30.0, 0.0, 0.0], 1.5),
        line(2, [30.0, 0.0, 0.0], [30.0, 30.0, 0.0], 1.5),
        line(3, [30.0, 30.0, 0.0], [0.0, 30.0, 0.0], 0.0),
    ];
    let home = [0.0, 0.0, 0.0, 0.0];
    let (offset, v_lin, smooth) = (0.05, 2.0, 0.02675);
    let matched_k = offset / v_lin;

    let plain = replay(
        cfg(),
        follower_kernel_chains(None, None, smooth),
        &home,
        0.0,
        &moves,
    );
    let linear = replay(
        cfg(),
        follower_kernel_chains(None, Some(matched_k), smooth),
        &home,
        0.0,
        &moves,
    );
    let saturating = |algo: &'static dyn trajectory::algos::PostProcessorAlgo| {
        let mut chains = follower_kernel_chains(None, None, smooth);
        chains.chains[3] = nonlinear_e_chain(algo, 0.0, offset, v_lin, smooth);
        replay(cfg(), chains, &home, 0.0, &moves)
    };
    let tanh = saturating(&trajectory::algos::TanhPressureAdvance);
    let recipr = saturating(&trajectory::algos::ReciprPressureAdvance);

    let lead = |pa: &[ShapedSegment]| {
        sample_extruder(pa)
            .iter()
            .zip(sample_extruder(&plain))
            .map(|((_, a), (_, b))| a - b)
            .fold(f64::NEG_INFINITY, f64::max)
    };
    let (lead_lin, lead_tanh, lead_recipr) = (lead(&linear), lead(&tanh), lead(&recipr));
    for (name, segs, lead_nl) in [("tanh", &tanh, lead_tanh), ("recipr", &recipr, lead_recipr)] {
        assert_extruder_has_no_jumps(segs);
        assert!(
            lead_nl > 1e-3,
            "{name} PA must still advance the extruder, lead {lead_nl}"
        );
        assert!(
            lead_nl < 0.75 * lead_lin,
            "{name} must advance visibly less than the matched linear model \
             at cruise: {lead_nl} vs {lead_lin}"
        );
        let (e_nl, e_plain) = (extruder_end(segs), extruder_end(&plain));
        assert!(
            (e_nl - e_plain).abs() < 1e-4,
            "{name} PA must not change the settled extruded total: {e_nl} vs {e_plain}"
        );
    }
    assert!(
        lead_recipr < lead_tanh,
        "recipr rises toward the bound more slowly than tanh, so it must lead \
         less: {lead_recipr} vs {lead_tanh}"
    );
}

#[test]
fn derivative_gains_track_transform_matches_analytic_second_derivative() {
    let piece = nurbs::bezier::BezierPiece {
        u_start: 0.0,
        u_end: 0.1,
        coeffs: vec![1.0, 2.0, 3.0, 4.0],
    };
    let track = nurbs::bezier::bezier_pieces_to_nurbs(&[piece]);
    let (k1, k2) = (0.0, 0.002);
    let out = crate::shaper::apply_derivative_gains_to_track(&track, k1, k2);
    for i in 0..=20 {
        let t = 0.1 * i as f64 / 20.0;
        let pos = 1.0 + 2.0 * t + 3.0 * t * t + 4.0 * t * t * t;
        let accel = 6.0 + 24.0 * t;
        let expected = pos + k2 * accel;
        assert!(
            (eval(&out, t) - expected).abs() < 1e-12,
            "track transform must equal x + k2*x'' at t={t}"
        );
    }
}

#[test]
fn derivative_gains_track_transform_combines_both_gains() {
    let piece = nurbs::bezier::BezierPiece {
        u_start: 0.0,
        u_end: 0.1,
        coeffs: vec![1.0, 2.0, 3.0, 4.0],
    };
    let track = nurbs::bezier::bezier_pieces_to_nurbs(&[piece]);
    let (k1, k2) = (0.05, 0.002);
    let out = crate::shaper::apply_derivative_gains_to_track(&track, k1, k2);
    for i in 0..=20 {
        let t = 0.1 * i as f64 / 20.0;
        let pos = 1.0 + 2.0 * t + 3.0 * t * t + 4.0 * t * t * t;
        let vel = 2.0 + 6.0 * t + 12.0 * t * t;
        let accel = 6.0 + 24.0 * t;
        let expected = pos + k1 * vel + k2 * accel;
        assert!(
            (eval(&out, t) - expected).abs() < 1e-12,
            "track transform must equal x + k1*x' + k2*x'' at t={t}"
        );
    }
}

#[test]
fn nonlinear_advance_track_transform_matches_the_advance_law() {
    let piece = nurbs::bezier::BezierPiece {
        u_start: 0.0,
        u_end: 0.1,
        coeffs: vec![1.0, 2.0, 3.0, 4.0],
    };
    let track = nurbs::bezier::bezier_pieces_to_nurbs(&[piece]);
    for model in [
        trajectory::AdvanceModel::Tanh,
        trajectory::AdvanceModel::Reciprocal,
    ] {
        let adv = trajectory::NonlinearAdvance {
            model,
            linear_advance: 0.03,
            nonlinear_offset: 0.08,
            linearization_velocity: 5.0,
        };
        let out = crate::shaper::apply_nonlinear_advance_to_track(3, &track, adv)
            .expect("nonlinear advance refits a polynomial track");
        let mut worst: f64 = 0.0;
        let mut worst_offset_term: f64 = 0.0;
        for i in 0..=20 {
            let t = 0.1 * i as f64 / 20.0;
            let pos = 1.0 + 2.0 * t + 3.0 * t * t + 4.0 * t * t * t;
            let vel = 2.0 + 6.0 * t + 12.0 * t * t;
            let offset_term = adv.advance(vel) - 0.03 * vel;
            let expected = pos + 0.03 * vel + offset_term;
            worst = worst.max((eval(&out, t) - expected).abs());
            worst_offset_term = worst_offset_term.max(offset_term);
        }
        assert!(
            worst < 1e-4,
            "{model:?}: refit of x + a(x') must hold the shaper's position \
             budget, worst {worst}"
        );
        assert!(
            worst_offset_term > 100.0 * worst,
            "{model:?}: the saturating term ({worst_offset_term}) must dominate \
             the fit error ({worst}), otherwise this test would pass on the \
             linear model too"
        );
    }
}

fn eval_axis_at(segs: &[ShapedSegment], axis: usize, t: f64) -> f64 {
    let seg = segs
        .iter()
        .find(|seg| t >= seg.t_start && t <= seg.t_end)
        .expect("t inside emitted trajectory");
    eval(&seg.axes[axis], t)
}

fn extruder_gain_kernel_chains(
    leader_smooth_time: Option<f64>,
    gain_first: bool,
    k1: f64,
    k2: f64,
    e_smooth_time: f64,
) -> AxisChainSet {
    let kernel = e_chain(None, e_smooth_time).stages[0].clone();
    let gain = trajectory::ChainStage::DerivativeGains { k1, k2 };
    let stages = if gain_first {
        vec![gain, kernel]
    } else {
        vec![kernel, gain]
    };
    let mut chains =
        leader_smooth_time.map_or_else(follower_chains_without_kernels, xy_shaper_follower_chains);
    chains.chains[3] = trajectory::CompiledChain { stages };
    chains
}

fn assert_gain_kernel_orders_commute(leader_smooth_time: Option<f64>) {
    // The feedrate step between the collinear moves keeps the fit stage
    // from merging them — the test's tolerances are calibrated to three
    // separate emit windows.
    let moves = [
        line_move([0.0, 0.0, 0.0], [20.0, 0.0, 0.0], 1.0, ctx(1, 80.0)).unwrap(),
        line_move([20.0, 0.0, 0.0], [40.0, 0.0, 0.0], 1.0, ctx(2, 95.0)).unwrap(),
        line_move([40.0, 0.0, 0.0], [60.0, 0.0, 0.0], 1.0, ctx(3, 80.0)).unwrap(),
    ];
    let home = [0.0, 0.0, 0.0, 0.0];
    let (k1, k2) = (0.03, 2e-4);
    let smooth = 0.02675;
    let pre = replay(
        cfg(),
        extruder_gain_kernel_chains(leader_smooth_time, true, k1, k2, smooth),
        &home,
        0.0,
        &moves,
    );
    let post = replay(
        cfg(),
        extruder_gain_kernel_chains(leader_smooth_time, false, k1, k2, smooth),
        &home,
        0.0,
        &moves,
    );
    let plain = replay(
        cfg(),
        extruder_gain_kernel_chains(leader_smooth_time, false, k1, 0.0, smooth),
        &home,
        0.0,
        &moves,
    );
    let t0 = pre
        .first()
        .unwrap()
        .t_start
        .max(post.first().unwrap().t_start);
    let t1 = pre.last().unwrap().t_end.min(post.last().unwrap().t_end);
    let mut max_k2_effect: f64 = 0.0;
    for i in 0..=400 {
        let t = t0 + (t1 - t0) * i as f64 / 400.0;
        let a = eval_axis_at(&pre, 3, t);
        let b = eval_axis_at(&post, 3, t);
        assert!(
            (a - b).abs() < 2e-3,
            "gain-then-kernel and kernel-then-gain must agree (LTI): {a} vs {b} at t={t}"
        );
        max_k2_effect = max_k2_effect.max((b - eval_axis_at(&plain, 3, t)).abs());
    }
    assert!(
        max_k2_effect > 1e-2,
        "k2 term must visibly move the track, got max effect {max_k2_effect}"
    );
}

#[test]
fn derivative_gains_commute_with_kernel_on_a_direct_follower() {
    assert_gain_kernel_orders_commute(None);
}

#[test]
fn derivative_gains_commute_with_kernel_on_a_projected_follower() {
    assert_gain_kernel_orders_commute(Some(0.044583333333333336));
}

fn x_kernel_chains(smooth_time: f64, mode: Option<(f64, f64)>) -> AxisChainSet {
    let mut instances = vec![PostProcessorInstance::new(
        "slew",
        &trajectory::algos::SmoothBell,
        vec![smooth_time],
    )];
    if let Some((frequency_hz, damping_ratio)) = mode {
        instances.push(PostProcessorInstance::new(
            "belt",
            &trajectory::algos::ModeInverse,
            vec![frequency_hz, damping_ratio],
        ));
    }
    let x = trajectory::CompiledChain::compile(&instances).expect("kernel + mode_inverse compiles");
    AxisChainSet {
        chains: vec![
            x,
            trajectory::CompiledChain::default(),
            trajectory::CompiledChain::default(),
            trajectory::CompiledChain::default(),
        ],
        followers: vec![(3, vec![0, 1, 2])],
    }
}

/// Semi-analytic plant simulation: the commanded track drives the 2nd-order
/// mode `z̈ + 2ζω·ż + ω²·z = ω²·x_cmd(t)` (toolhead z behind belt compliance),
/// integrated with RK4 at fine dt over the emitted trajectory.
fn integrate_mode_response(
    cmd: &[ShapedSegment],
    frequency_hz: f64,
    damping_ratio: f64,
    t0: f64,
    t1: f64,
) -> impl Fn(f64) -> f64 {
    let omega = 2.0 * std::f64::consts::PI * frequency_hz;
    let dt = 1e-5;
    let steps = ((t1 - t0) / dt).ceil() as usize;
    let x_cmd = |t: f64| eval_axis_at(cmd, 0, t.clamp(t0, t1));
    let deriv = |t: f64, z: f64, zd: f64| {
        (
            zd,
            omega * omega * (x_cmd(t) - z) - 2.0 * damping_ratio * omega * zd,
        )
    };
    let mut z = x_cmd(t0);
    let mut zd = 0.0;
    let mut trace = Vec::with_capacity(steps + 1);
    trace.push(z);
    for i in 0..steps {
        let t = t0 + i as f64 * dt;
        let (k1z, k1v) = deriv(t, z, zd);
        let (k2z, k2v) = deriv(t + 0.5 * dt, z + 0.5 * dt * k1z, zd + 0.5 * dt * k1v);
        let (k3z, k3v) = deriv(t + 0.5 * dt, z + 0.5 * dt * k2z, zd + 0.5 * dt * k2v);
        let (k4z, k4v) = deriv(t + dt, z + dt * k3z, zd + dt * k3v);
        z += dt / 6.0 * (k1z + 2.0 * k2z + 2.0 * k3z + k4z);
        zd += dt / 6.0 * (k1v + 2.0 * k2v + 2.0 * k3v + k4v);
        trace.push(z);
    }
    move |t: f64| {
        let idx = ((t - t0) / dt).round() as usize;
        trace[idx.min(steps)]
    }
}

/// The plan-inversion contract: driving the belt-compliance mode with the
/// mode_inverse command `x + (2ζ/ω)ẋ + (1/ω²)ẍ` makes the toolhead follow the
/// nominal (kernel-only) path, while driving it with the nominal path directly
/// leaves the resonance ringing.
#[test]
fn mode_inverse_makes_the_oscillator_track_the_nominal_path() {
    let (frequency_hz, damping_ratio) = (30.0, 0.05);
    let smooth_time = 0.0015;
    let moves = [
        line(1, [0.0, 0.0, 0.0], [40.0, 0.0, 0.0], 0.0),
        line(2, [40.0, 0.0, 0.0], [10.0, 0.0, 0.0], 0.0),
    ];
    let home = [0.0, 0.0, 0.0, 0.0];
    let nominal = replay(
        cfg(),
        x_kernel_chains(smooth_time, None),
        &home,
        0.0,
        &moves,
    );
    let inverted = replay(
        cfg(),
        x_kernel_chains(smooth_time, Some((frequency_hz, damping_ratio))),
        &home,
        0.0,
        &moves,
    );
    let t0 = nominal[0].t_start.max(inverted[0].t_start);
    let t1 = nominal
        .last()
        .unwrap()
        .t_end
        .min(inverted.last().unwrap().t_end);
    assert!(t1 - t0 > 0.3, "trajectory too short to ring: {}", t1 - t0);

    let tracked = integrate_mode_response(&inverted, frequency_hz, damping_ratio, t0, t1);
    let ringing = integrate_mode_response(&nominal, frequency_hz, damping_ratio, t0, t1);

    let mut max_tracked: f64 = 0.0;
    let mut max_ringing: f64 = 0.0;
    let n = 4000;
    for i in 0..=n {
        let t = t0 + (t1 - t0) * i as f64 / n as f64;
        let x_nom = eval_axis_at(&nominal, 0, t);
        max_tracked = max_tracked.max((tracked(t) - x_nom).abs());
        max_ringing = max_ringing.max((ringing(t) - x_nom).abs());
    }
    assert!(
        max_tracked < 5e-3,
        "inverted command must make the mode track the nominal path: \
         max residual {max_tracked}"
    );
    assert!(
        max_ringing > 0.05,
        "without inversion the mode must ring visibly: max residual {max_ringing}"
    );
    assert!(
        max_ringing > 20.0 * max_tracked,
        "inversion must suppress the residual by over an order of magnitude: \
         {max_ringing} vs {max_tracked}"
    );
}

/// Replay with explicit stream items (moves, drains, dwells) — the live
/// ingress's shape for M400/G4 sequences, which `replay` cannot express.
fn replay_items(
    config: StreamConfig,
    chains: AxisChainSet,
    home: &[f64],
    items: Vec<StreamInput>,
) -> Vec<ShapedSegment> {
    let (raw_tx, raw_rx) = unbounded();
    for item in items {
        raw_tx.send(item).unwrap();
    }
    drop(raw_tx);

    let (fitted_tx, fitted_rx) = unbounded();
    FitStage::new(config.corner).run(raw_rx, fitted_tx);

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

/// The servo-ident stroke/dwell pattern on the Trident bench chains
/// (smooth_mzv 50Hz on X, smooth_zv 44Hz on Y): full-speed strokes with
/// drains and 1.2s dwells between them. Regression for the shaper dying
/// with "shaping window needs unavailable history" at the second stroke.
#[test]
fn stroke_dwell_stroke_keeps_shaper_history() {
    let chains = AxisChainSet::spatial(
        trajectory::CompiledChain::compile(&[PostProcessorInstance::new(
            "sx",
            &trajectory::algos::SmoothMzv,
            vec![50.0],
        )])
        .expect("compiles"),
        trajectory::CompiledChain::compile(&[PostProcessorInstance::new(
            "sy",
            &trajectory::algos::SmoothZv,
            vec![44.0],
        )])
        .expect("compiles"),
        trajectory::CompiledChain::default(),
    );
    let limits = VelocityLimits::try_new(1000.0, 10000.0, 0.21, 20000.0).unwrap();
    let config = StreamConfig {
        corner: CornerFitConfig::default(),
        integration_tol: 1e-4,
        max_extrude_only_velocity_mm_s: f64::INFINITY,
        max_extrude_only_accel_mm_s2: f64::INFINITY,
        fit_tol_mm: 0.005,
        fit_tol_accel_mm_s2: 50.0,
        max_buffer_moves: 128,
        limits,
    };
    let stroke_ctx = |line_no: u32| MoveContext {
        extruder_axis: 3,
        feedrate_mm_s: 500.0,
        limits,
        source: SourceRange {
            start_line: line_no,
            end_line: line_no,
        },
    };

    let mut items = Vec::new();
    let mut x = 100.0;
    for stroke in 0..4u32 {
        let dir = if stroke % 2 == 0 { 1.0 } else { -1.0 };
        let x_end = x + dir * 60.0;
        items.push(StreamInput::Move(
            line_move(
                [x, 100.0, 10.0],
                [x_end, 100.0, 10.0],
                0.0,
                stroke_ctx(stroke + 1),
            )
            .unwrap(),
        ));
        x = x_end;
        items.push(StreamInput::Drain);
        items.push(StreamInput::Control(Control::Dwell { secs: 1.2 }));
    }

    let segs = replay_items(config, chains, &[100.0, 100.0, 10.0], items);
    assert!(!segs.is_empty());
    // Dwells are legitimate time gaps; across each one the trajectory must
    // hold position exactly.
    for w in segs.windows(2) {
        if (w[1].t_start - w[0].t_end).abs() < 1e-9 {
            continue;
        }
        for axis in 0..w[0].axes.len() {
            let a = eval(&w[0].axes[axis], w[0].t_end);
            let b = eval(&w[1].axes[axis], w[1].t_start);
            // The force-flushed kernel tail may sit a sub-micron shy of
            // rest; anything beyond the fit budget is a real weld.
            assert!(
                (a - b).abs() < 1e-3,
                "axis {axis} moved across the dwell gap at t={}: {a} vs {b}",
                w[0].t_end
            );
        }
    }
    for seg in &segs {
        for curve in &seg.axes {
            assert!(curve.control_points().iter().all(|v| v.is_finite()));
        }
    }
}

/// The beacon rapid-scan path on the corexy_fast world limits (2800 mm/s,
/// 100k accel, bell shapers, corner_deviation covering the kernel share):
/// long passes joined by ~1mm arc-chord turnarounds at 800 mm/s. Regression
/// for the shaped track spiking to ~2e6 mm/s (step-ceiling abort on the
/// bench-shaped worlds).
#[test]
fn beacon_scan_path_shaped_velocity_stays_bounded() {
    let chains = AxisChainSet::spatial(
        trajectory::CompiledChain::compile(&[PostProcessorInstance::new(
            "sx",
            &trajectory::algos::SmoothBell,
            vec![0.019125],
        )])
        .expect("compiles"),
        trajectory::CompiledChain::compile(&[PostProcessorInstance::new(
            "sy",
            &trajectory::algos::SmoothBell,
            vec![0.018238636363636363],
        )])
        .expect("compiles"),
        trajectory::CompiledChain::default(),
    );
    let limits = VelocityLimits::try_new(2800.0, 100000.0, 0.695, 1000000.0).unwrap();
    let config = StreamConfig {
        corner: CornerFitConfig::default(),
        integration_tol: 1e-4,
        max_extrude_only_velocity_mm_s: f64::INFINITY,
        max_extrude_only_accel_mm_s2: f64::INFINITY,
        fit_tol_mm: 0.005,
        fit_tol_accel_mm_s2: 50.0,
        max_buffer_moves: 128,
        limits,
    };
    let pts: Vec<(f64, f64)> = include_str!("../beacon_scan_pts.txt")
        .lines()
        .map(|l| {
            let (x, y) = l.split_once(' ').expect("two floats");
            (x.parse().unwrap(), y.parse().unwrap())
        })
        .collect();
    let mut moves = Vec::new();
    let mut prev = [pts[0].0, pts[0].1, 2.0];
    for (i, &(x, y)) in pts.iter().enumerate().skip(1) {
        let end = [x, y, 2.0];
        let ctx = MoveContext {
            extruder_axis: 3,
            feedrate_mm_s: 800.0,
            limits,
            source: SourceRange {
                start_line: i as u32,
                end_line: i as u32,
            },
        };
        // klippy's Motion.move drops zero-distance moves before submit.
        if let Ok(m) = line_move(prev, end, 0.0, ctx) {
            moves.push(m);
        }
        prev = end;
    }
    // The pacer drains whenever the feed runs dry; inject that cadence so
    // the windowed (streaming) planner shape is exercised, not just the
    // full-lookahead one.
    let mut items = Vec::new();
    for (i, m) in moves.iter().cloned().enumerate() {
        items.push(StreamInput::Move(m));
        if i % 8 == 7 {
            items.push(StreamInput::Drain);
        }
    }
    let segs = replay_items(config, chains, &[pts[0].0, pts[0].1, 2.0], items);
    assert!(!segs.is_empty());
    let mut worst: (f64, f64, usize) = (0.0, 0.0, 0);
    for seg in &segs {
        for axis in 0..2 {
            let h = 1e-5;
            let mut t = seg.t_start;
            while t < seg.t_end - h {
                let v = (eval(&seg.axes[axis], t + h) - eval(&seg.axes[axis], t)) / h;
                if v.abs() > worst.0.abs() {
                    worst = (v, t, axis);
                }
                t += (seg.t_end - seg.t_start) / 64.0;
            }
        }
    }
    assert!(
        worst.0.abs() < 4000.0,
        "shaped velocity spiked to {} mm/s on axis {} at t={}",
        worst.0,
        worst.2,
        worst.1
    );
}
