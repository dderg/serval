use crate::types::*;
use crate::*;
use crossbeam_channel::unbounded;
use geometry::segment::SourceRange;
use geometry::{CornerFitConfig, MoveContext, VelocityLimits, line_move};
use nurbs::eval::eval;
use std::sync::Arc;

use trajectory::{AxisChainSet, ContinuousAxis, ContinuousSegment, PostProcessorInstance};

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
            f64::INFINITY,
        )
        .unwrap(),
    }
}
fn fit_tol(config: StreamConfig) -> FitTol {
    FitTol {
        pos_mm: config.fit_tol_mm,
        accel_mm_s2: config.fit_tol_accel_mm_s2,
    }
}

fn eval_segment_axis(segment: &ContinuousSegment, axis: usize, t: f64) -> f64 {
    segment.eval_axis(axis, t).unwrap().position
}
fn assert_segment_axes_finite(segment: &ContinuousSegment) {
    for (axis, source) in segment.axes.iter().enumerate() {
        match source {
            ContinuousAxis::Spline(curve) | ContinuousAxis::RelativeSpline { curve, .. } => {
                assert!(curve.control_points().iter().all(|v| v.is_finite()));
            }
            ContinuousAxis::PiecewiseRelativeSpline(pieces) => {
                for piece in pieces.iter() {
                    assert!(piece.base_position.is_finite());
                    assert!(piece.curve.control_points().iter().all(|v| v.is_finite()));
                }
            }
            _ => {}
        }
        for t in [
            segment.t_start,
            0.5 * (segment.t_start + segment.t_end),
            segment.t_end,
        ] {
            let value = segment.eval_axis(axis, t).expect("axis evaluates");
            assert!(value.position.is_finite());
            assert!(value.velocity.is_finite());
            assert!(value.acceleration.is_finite());
        }
    }
}
fn axis_breakpoints(axis: &ContinuousAxis) -> (Vec<f64>, usize) {
    match axis {
        ContinuousAxis::Analytic { span, .. } => (
            span.phases
                .iter()
                .flat_map(|phase| [span.t_start + phase.t0, span.t_start + phase.end_time()])
                .collect(),
            3,
        ),
        ContinuousAxis::Spline(curve) | ContinuousAxis::RelativeSpline { curve, .. } => {
            (curve.knots().to_vec(), curve.degree() as usize)
        }
        ContinuousAxis::PiecewiseRelativeSpline(pieces) => (
            pieces
                .iter()
                .flat_map(|piece| piece.curve.knots().iter().copied())
                .collect(),
            pieces
                .iter()
                .map(|piece| piece.curve.degree() as usize)
                .max()
                .expect("a piecewise relative spline has pieces"),
        ),
        ContinuousAxis::Hold { t_start, t_end, .. } => (vec![*t_start, *t_end], 0),
        ContinuousAxis::Nudge(profile) => (profile.breakpoints().to_vec(), 3),
        ContinuousAxis::Buzz { profile, .. } => (profile.breakpoints().to_vec(), 3),
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
            f64::INFINITY,
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
            f64::INFINITY,
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
            f64::INFINITY,
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
) -> Vec<ContinuousSegment> {
    replay_stream(
        config,
        chains,
        home,
        t_start,
        moves.iter().map(|m| m.clone().into()).collect(),
    )
    .into_iter()
    .filter_map(|item| match item {
        TrajectoryItem::Seg(seg) => Some(seg),
        TrajectoryItem::Parked | TrajectoryItem::Control(_) => None,
    })
    .collect()
}

/// [`replay`] without the segment filter — the marker stream the dispatcher
/// sees, park declaration included.
fn replay_stream(
    config: StreamConfig,
    chains: AxisChainSet,
    home: &[f64],
    t_start: f64,
    items: Vec<StreamInput>,
) -> Vec<TrajectoryItem> {
    let (raw_tx, raw_rx) = unbounded();
    for item in items {
        raw_tx.send(item).unwrap();
    }
    drop(raw_tx);

    let (fitted_tx, fitted_rx) = unbounded();
    FitStage::new(config.corner).run(raw_rx, fitted_tx);

    let (planned_tx, planned_rx) = unbounded();
    Planner::new(config).run(fitted_rx, planned_tx);

    let fit_tol = fit_tol(config);
    let (lowered_tx, lowered_rx) = unbounded();
    run_lowerer(
        planned_rx,
        lowered_tx,
        chains.clone(),
        home.to_vec(),
        t_start,
    );

    let (shaped_tx, shaped_rx) = unbounded();
    Shaper::new(chains, fit_tol).run(lowered_rx, shaped_tx);

    shaped_rx.into_iter().collect()
}

fn boundary_speed(prev: &ContinuousSegment, next: &ContinuousSegment) -> f64 {
    let h = 1e-6;
    let axes = prev.axes.len().min(3);
    let mut v2 = 0.0;
    for axis in 0..axes {
        let a = eval_segment_axis(prev, axis, prev.t_end - h);
        let b = eval_segment_axis(next, axis, next.t_start + h);
        let v = (b - a) / (2.0 * h);
        v2 += v * v;
    }
    v2.sqrt()
}

fn assert_time_contiguous(segs: &[ContinuousSegment]) {
    for w in segs.windows(2) {
        assert!(
            (w[1].t_start - w[0].t_end).abs() < 1e-9,
            "time gap between segments: {} -> {}",
            w[0].t_end,
            w[1].t_start
        );
    }
}

fn assert_position_contiguous(segs: &[ContinuousSegment]) {
    for w in segs.windows(2) {
        for axis in 0..w[0].axes.len() {
            let a = eval_segment_axis(&w[0], axis, w[0].t_end);
            let b = eval_segment_axis(&w[1], axis, w[1].t_start);
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
    assert!((eval_segment_axis(last, 0, last.t_end) - x_end).abs() < 1e-4);
    assert!((eval_segment_axis(last, 1, last.t_end) - y_end).abs() < 1e-4);
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
    assert!((eval_segment_axis(last, 0, last.t_end) - 100.0).abs() < 1e-6);
    // The seam at x=50 is interior; the toolhead must cruise through it.
    for w in segs.windows(2) {
        let x = eval_segment_axis(&w[0], 0, w[0].t_end);
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
    assert!((eval_segment_axis(last, 0, last.t_end) - 50.0).abs() < 1e-6);
    assert!((eval_segment_axis(last, 1, last.t_end) - 50.0).abs() < 1e-6);
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
        (eval_segment_axis(last, 3, last.t_end) - 15.0).abs() < 1e-3,
        "total extrusion must be conserved"
    );
}

fn axis_velocity(seg: &ContinuousSegment, axis: usize, t: f64) -> f64 {
    seg.eval_axis(axis, t)
        .unwrap_or_else(|error| panic!("axis {axis} evaluation failed at {t}: {error}"))
        .velocity
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
    assert!((eval_segment_axis(last, 3, last.t_end) - 17.65).abs() < 1e-3);
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
    assert!((eval_segment_axis(last, 3, last.t_end) - 8.0).abs() < 1e-3);
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
            let da = eval_segment_axis(a, axis, a.t_end);
            let db = eval_segment_axis(b, axis, b.t_end);
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
        assert_segment_axes_finite(seg);
    }
    let last = segs.last().expect("non-empty");
    let final_x = eval_segment_axis(last, 0, last.t_end);
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
    let input_degree = base
        .iter()
        .map(|segment| axis_breakpoints(&segment.axes[0]).1)
        .max()
        .expect("non-empty base");

    for (base_seg, shaped_seg) in base.iter().zip(shaped) {
        let mut breaks: Vec<f64> = Vec::new();
        for seg in &base {
            breaks.push(seg.t_start);
            breaks.extend(axis_breakpoints(&seg.axes[0]).0);
            breaks.push(seg.t_end);
        }
        let sig = trajectory::ShapedSignal::new_from_evaluator(
            kernel,
            |t| {
                let clamped = t.clamp(first, last);
                base.iter()
                    .find(|seg| clamped >= seg.t_start && clamped <= seg.t_end)
                    .map_or_else(
                        || eval_segment_axis(base.last().unwrap(), 0, clamped),
                        |seg| eval_segment_axis(seg, 0, clamped),
                    )
            },
            breaks,
            input_degree,
        );
        for frac in [0.1_f64, 0.3, 0.5, 0.7, 0.9] {
            let t = frac.mul_add(base_seg.t_end - base_seg.t_start, base_seg.t_start);
            let got = eval_segment_axis(shaped_seg, 0, t + pad);
            let want = sig.eval(t);
            assert!(
                (got - want).abs() < 5e-2,
                "shaped x at t={t}: got {got}, want {want}"
            );
        }
    }
}

#[test]
fn polynomial_moment_convolution_matches_quadrature() {
    use std::rc::Rc;

    use nurbs::bezier::{BezierPiece, bezier_pieces_to_nurbs};

    let first_t = 300.0;
    let first_end = 300.004;
    let second_start = 300.006;
    let last_t = 300.012;
    let first_track = bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: first_t,
        u_end: first_end,
        coeffs: vec![
            10.0, 4.0, -30.0, 200.0, -1_000.0, 5_000.0, -20_000.0, 40_000.0,
        ],
    }]);
    let held = eval(&first_track, first_end);
    let second_track = bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: second_start,
        u_end: last_t,
        coeffs: vec![held, -3.0, 20.0, -100.0, 400.0, -1_000.0, 2_000.0, -3_000.0],
    }]);
    let kernel = trajectory::build_smooth_mzv_kernel(90.2);
    let kernel_degree = kernel
        .pieces
        .iter()
        .map(|piece| piece.degree())
        .max()
        .unwrap();
    let mut breaks = first_track.knots().to_vec();
    breaks.extend_from_slice(second_track.knots());

    let oracle_table = Rc::new(crate::shaper::AxisSignalTable::from_tracks(
        [&first_track, &second_track],
        first_t,
        last_t,
        true,
        true,
    ));
    let oracle_eval = Rc::clone(&oracle_table);
    let oracle = trajectory::ShapedSignal::new_from_evaluator(
        &kernel,
        move |t| oracle_eval.eval(t),
        breaks.clone(),
        oracle_table.max_degree(),
    );

    let fast_table = Rc::new(
        crate::shaper::AxisSignalTable::from_tracks(
            [&first_track, &second_track],
            first_t,
            last_t,
            true,
            true,
        )
        .with_piece_moments(kernel_degree),
    );
    let fast_eval = Rc::clone(&fast_table);
    let fast_moments = Rc::clone(&fast_table);
    let fast = trajectory::ShapedSignal::new_from_polynomial_evaluator(
        &kernel,
        move |t| fast_eval.eval(t),
        breaks,
        fast_table.max_degree(),
        move |lo, hi, degree, origin, moments| {
            fast_moments.integrate_moments(lo, hi, degree, origin, moments)
        },
    );

    for t in [
        first_t,
        300.001,
        first_end,
        300.005,
        second_start,
        300.009,
        last_t,
    ] {
        let got = fast.eval_pva(t);
        let want = oracle.eval_pva(t);
        assert!(
            (got.0 - want.0).abs() < 1e-10,
            "position at {t}: {got:?} vs {want:?}"
        );
        assert!(
            (got.1 - want.1).abs() < 1e-6,
            "velocity at {t}: {got:?} vs {want:?}"
        );
        assert!(
            (got.2 - want.2).abs() < 1e-3,
            "acceleration at {t}: {got:?} vs {want:?}"
        );
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

    let constant_seg = |t_start: f64, t_end: f64| ContinuousSegment {
        axes: Arc::from(
            (0..3)
                .map(|_| {
                    ContinuousAxis::Spline(Arc::new(nurbs::bezier::bezier_pieces_to_nurbs(&[
                        nurbs::bezier::BezierPiece {
                            u_start: t_start,
                            u_end: t_end,
                            coeffs: vec![150.0],
                        },
                    ])))
                })
                .collect::<Vec<_>>(),
        ),
        followers: Arc::from([]),
        spatial_path: false,
        t_start,
        t_end,
        motor_mask: 0,
        source_line: 1,
        rest_at_end: true,
    };

    let (lowered_tx, lowered_rx) = unbounded();
    for i in 0..8 {
        let (a, b) = (i as f64, (i + 1) as f64);
        lowered_tx
            .send(BaseItem::Seg(BaseSegment {
                segment: constant_seg(a.mul_add(step, t0), b.mul_add(step, t0)),
            }))
            .unwrap();
    }
    drop(lowered_tx);

    let (shaped_tx, shaped_rx) = unbounded();
    Shaper::new(chains, fit_tol(cfg())).run(lowered_rx, shaped_tx);

    let segs: Vec<ContinuousSegment> = shaped_rx
        .into_iter()
        .filter_map(|item| match item {
            TrajectoryItem::Seg(seg) => Some(seg),
            TrajectoryItem::Parked | TrajectoryItem::Control(_) => None,
        })
        .collect();
    assert_eq!(segs.len(), 8);
    for seg in &segs {
        let mid = 0.5 * (seg.t_start + seg.t_end);
        let got = eval_segment_axis(seg, 0, mid);
        assert!(
            (got - 150.0).abs() < 1e-3,
            "seg [{}, {}]: shaped constant drifted to {got}",
            seg.t_start,
            seg.t_end,
        );
        let ContinuousAxis::Spline(curve) = &seg.axes[0] else {
            panic!("a changed smooth-kernel axis must be a spline");
        };
        assert_eq!(
            curve.control_points(),
            &[150.0],
            "stationary shaped axes must retain their constant representation"
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
        f64::INFINITY,
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
        f64::INFINITY,
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
) -> Vec<ContinuousSegment> {
    let (raw_tx, raw_rx) = unbounded();
    for item in inputs {
        raw_tx.send(item).unwrap();
    }
    drop(raw_tx);
    let (fitted_tx, fitted_rx) = unbounded();
    FitStage::new(config.corner).run(raw_rx, fitted_tx);
    let (planned_tx, planned_rx) = unbounded();
    Planner::new(config).run(fitted_rx, planned_tx);
    let fit_tol = fit_tol(config);
    let (lowered_tx, lowered_rx) = unbounded();
    run_lowerer(planned_rx, lowered_tx, chains.clone(), home.to_vec(), 0.0);
    let (shaped_tx, shaped_rx) = unbounded();
    Shaper::new(chains, fit_tol).run(lowered_rx, shaped_tx);
    shaped_rx
        .into_iter()
        .filter_map(|item| match item {
            TrajectoryItem::Seg(seg) => Some(seg),
            TrajectoryItem::Parked | TrajectoryItem::Control(_) => None,
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
    let x = eval_segment_axis(last, 0, t_end);
    let y = eval_segment_axis(last, 1, t_end);
    let z_machine = eval_segment_axis(last, 2, t_end);
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

/// The dispatcher learns a stop is a park from the drain itself. Reading it
/// off the committed track's end derivative cannot work: a trailing
/// derivative-gain stage (pressure advance) leaves the parked extruder's
/// commanded velocity at `k·ë`, nonzero wherever the profile stops with
/// acceleration still applied — which unlimited jerk makes every stop.
#[test]
fn a_drain_declares_the_park_after_its_last_segment() {
    let mut config = cfg();
    config.max_extrude_only_velocity_mm_s = 100.0;
    config.max_extrude_only_accel_mm_s2 = 1000.0;
    config.limits = VelocityLimits::try_new(
        300.0,
        5000.0,
        geometry::corner_deviation_from_scv(5.0, 5000.0),
        f64::INFINITY,
    )
    .unwrap();
    let chains = AxisChainSet {
        chains: vec![
            trajectory::CompiledChain::default(),
            trajectory::CompiledChain::default(),
            trajectory::CompiledChain::default(),
            trajectory::CompiledChain::compile(&[PostProcessorInstance::new(
                "pa",
                &trajectory::algos::LinearPressureAdvance,
                vec![0.05],
            )])
            .expect("single post-processor always compiles"),
        ],
        followers: vec![(3, vec![0, 1, 2])],
    };

    let items = replay_stream(
        config,
        chains,
        &[0.0, 0.0, 0.0, 0.0],
        0.0,
        vec![
            line(1, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 5.0).into(),
            line(2, [0.0, 0.0, 0.0], [10.0, 0.0, 0.0], 1.0).into(),
            line(3, [10.0, 0.0, 0.0], [20.0, 0.0, 0.0], 0.0).into(),
        ],
    );

    assert!(
        matches!(items.last(), Some(TrajectoryItem::Parked)),
        "the drain must declare the park after its last segment"
    );
    assert!(
        items[..items.len() - 1]
            .iter()
            .all(|item| !matches!(item, TrajectoryItem::Parked)),
        "only the drain parks: no marker may precede its segments"
    );
}

fn sampled_planar_path_length(segs: &[ContinuousSegment]) -> f64 {
    const SAMPLES_PER_SEG: usize = 2000;
    let mut length = 0.0;
    let mut prev: Option<(f64, f64)> = None;
    for seg in segs {
        for i in 0..=SAMPLES_PER_SEG {
            let t = seg.t_start + (seg.t_end - seg.t_start) * i as f64 / SAMPLES_PER_SEG as f64;
            let p = (eval_segment_axis(seg, 0, t), eval_segment_axis(seg, 1, t));
            if let Some(q) = prev {
                length += ((p.0 - q.0).powi(2) + (p.1 - q.1).powi(2)).sqrt();
            }
            prev = Some(p);
        }
    }
    length
}

fn extruder_end(segs: &[ContinuousSegment]) -> f64 {
    let last = segs.last().expect("segments emitted");
    eval_segment_axis(last, 3, last.t_end)
}

fn assert_extruder_continuous_and_monotone(segs: &[ContinuousSegment]) {
    let tolerance = fit_tol(cfg()).scaled(
        segs.iter()
            .map(|seg| crate::lowering::follower_tol_scale(&seg.followers, 3))
            .fold(1.0, f64::min),
    );
    let mut prev_val: Option<f64> = None;
    for seg in segs {
        for i in 0..=200 {
            let t = seg.t_start + (seg.t_end - seg.t_start) * i as f64 / 200.0;
            let v = eval_segment_axis(seg, 3, t);
            if let Some(p) = prev_val {
                assert!(
                    v >= p - tolerance.pos_mm,
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
            max_e_diff =
                max_e_diff.max((eval_segment_axis(p, 3, t) - eval_segment_axis(q, 3, t)).abs());
            max_x_diff =
                max_x_diff.max((eval_segment_axis(p, 0, t) - eval_segment_axis(q, 0, t)).abs());
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

/// Issue #405: a rest-hold sized exactly to the follower kernel's support
/// lands `t_end + own_hi` a few ulps past the shaping frontier. The old
/// gate admitted it through a 1e-12 slack the strict `MissingLookahead`
/// check does not share, panicking the shape thread mid-print
/// ("shaping window needs unavailable lookahead"). The gate must be as
/// strict as the check: the segment waits for real lookahead instead.
#[test]
fn follower_frontier_waits_for_exact_kernel_lookahead() {
    let leader = trajectory::CompiledChain::compile(&[PostProcessorInstance::new(
        "leader",
        &trajectory::algos::SmoothZv,
        vec![80.0],
    )])
    .expect("leader chain compiles");
    let follower = trajectory::CompiledChain::compile(&[
        PostProcessorInstance::new("pa", &trajectory::algos::LinearPressureAdvance, vec![0.018]),
        PostProcessorInstance::new("st", &trajectory::algos::SmoothTriangle, vec![0.02]),
    ])
    .expect("follower chain compiles");
    let chains = AxisChainSet {
        chains: vec![
            leader.clone(),
            leader,
            trajectory::CompiledChain::default(),
            follower,
        ],
        followers: vec![(3, vec![0, 1, 2])],
    };
    let direct_hi = chains.direct_forward_support();
    let own_hi = chains.max_follower_own_forward_support();
    let target_end = 20.389_013_601_744_544;
    let frontier_end = target_end + own_hi - 5e-13;
    let buffered_end = frontier_end + direct_hi;
    let last_end = buffered_end + 1e-3;
    assert!(target_end + own_hi > frontier_end);
    assert!(target_end + own_hi <= frontier_end + 1e-12);

    let segment = |t_start: f64, t_end: f64| ContinuousSegment {
        axes: Arc::from(
            (0..4)
                .map(|axis| {
                    ContinuousAxis::Spline(Arc::new(nurbs::bezier::bezier_pieces_to_nurbs(&[
                        nurbs::bezier::BezierPiece {
                            u_start: t_start,
                            u_end: t_end,
                            coeffs: vec![f64::from(axis)],
                        },
                    ])))
                })
                .collect::<Vec<_>>(),
        ),
        followers: Arc::from([]),
        spatial_path: false,
        t_start,
        t_end,
        motor_mask: 0,
        source_line: 1,
        rest_at_end: false,
    };
    let (lowered_tx, lowered_rx) = unbounded();
    for (t_start, t_end) in [
        (target_end - 0.005, target_end),
        (target_end, frontier_end),
        (frontier_end, buffered_end),
        (buffered_end, last_end),
    ] {
        lowered_tx
            .send(BaseItem::Seg(BaseSegment {
                segment: segment(t_start, t_end),
            }))
            .unwrap();
    }
    drop(lowered_tx);

    let (shaped_tx, shaped_rx) = unbounded();
    Shaper::new(chains, fit_tol(cfg())).run(lowered_rx, shaped_tx);
    let emitted = shaped_rx
        .into_iter()
        .filter(|item| matches!(item, TrajectoryItem::Seg(_)))
        .count();
    assert_eq!(emitted, 4);
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

fn sample_extruder(segs: &[ContinuousSegment]) -> Vec<(f64, f64)> {
    let mut samples = Vec::new();
    for seg in segs {
        for i in 0..=200 {
            let t = seg.t_start + (seg.t_end - seg.t_start) * i as f64 / 200.0;
            samples.push((t, eval_segment_axis(seg, 3, t)));
        }
    }
    samples
}

fn assert_extruder_has_no_jumps(segs: &[ContinuousSegment]) {
    let tolerance = fit_tol(cfg()).pos_mm;
    for pair in segs.windows(2) {
        let left = pair[0].eval_axis(3, pair[0].t_end).unwrap();
        let right = pair[1].eval_axis(3, pair[1].t_start).unwrap();
        assert!(
            (left.position - right.position).abs() <= tolerance,
            "extruder seam at {}: left {} vs right {}",
            pair[0].t_end,
            left.position,
            right.position
        );
    }
    for seg in segs {
        for i in 0..=200 {
            let t = seg.t_start + (seg.t_end - seg.t_start) * i as f64 / 200.0;
            let pva = seg.eval_axis(3, t).unwrap();
            assert!(
                [pva.position, pva.velocity, pva.acceleration]
                    .into_iter()
                    .all(f64::is_finite),
                "extruder PVA is non-finite at {t}: {pva:?}"
            );
        }
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

/// Late in a print the follower's cumulative position is tens of
/// millimetres, and a kernel fit that carried it absolutely would spend its
/// whole positional budget on the base — the ladder then bisects toward
/// subpicosecond spans chasing a relative residual it can never resolve.
/// Every emitted target must carry its own base, leaving the fitted curve
/// near zero, while the evaluated track stays absolute and continuous.
#[test]
fn follower_kernel_targets_carry_their_own_base_late_in_a_print() {
    let moves = [
        line(1, [0.0, 0.0, 0.0], [30.0, 0.0, 0.0], 1.5),
        line(2, [30.0, 0.0, 0.0], [30.0, 30.0, 0.0], 1.5),
        line(3, [30.0, 30.0, 0.0], [0.0, 30.0, 0.0], 1.5),
        line(4, [0.0, 30.0, 0.0], [0.0, 0.0, 0.0], 1.5),
    ];
    let start_e = 24.0;
    let home = [0.0, 0.0, 0.0, start_e];
    let shaped = replay(
        cfg(),
        follower_kernel_chains(Some(0.044583333333333336), None, 0.02675),
        &home,
        0.0,
        &moves,
    );
    assert!(
        shaped.len() >= 4,
        "expected several emitted targets, got {}",
        shaped.len()
    );
    for seg in &shaped {
        let ContinuousAxis::PiecewiseRelativeSpline(pieces) = &seg.axes[3] else {
            panic!("a kerneled follower emits a piecewise relative spline");
        };
        assert!(!pieces.is_empty());
        for piece in pieces.iter() {
            let source_extent = piece
                .curve
                .control_points()
                .iter()
                .fold(0.0_f64, |extent, value| extent.max(value.abs()));
            assert!(
                source_extent < 2.0,
                "kernel fit source must stay near zero: extent {source_extent} \
                 over base {} at t={}",
                piece.base_position,
                piece.t_start,
            );
            assert!(
                piece.base_position >= start_e - 1e-6,
                "each piece base must carry the cumulative position: {} at t={}",
                piece.base_position,
                piece.t_start,
            );
        }
    }
    assert_extruder_continuous_and_monotone(&shaped);
    let first = shaped.first().expect("segments emitted");
    let track_start = eval_segment_axis(first, 3, first.t_start);
    assert!(
        (track_start - start_e).abs() <= 0.05 * fit_tol(cfg()).pos_mm,
        "absolute track must still start at the print's cumulative position: \
         {track_start} vs {start_e}"
    );
    assert!(
        extruder_end(&shaped) > start_e + 5.0,
        "the print must extrude on top of its base: {}",
        extruder_end(&shaped)
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

    let lead = |pa: &[ContinuousSegment]| {
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
        let out = crate::shaper::apply_nonlinear_advance_to_track(3, &track, adv, fit_tol(cfg()))
            .expect("nonlinear advance refits a polynomial track");
        let tolerance = fit_tol(cfg());
        let first_derivative = nurbs::eval::derivative(&out);
        let second_derivative = nurbs::eval::derivative(&first_derivative);
        for output_piece in nurbs::bezier::extract_bezier_pieces(&out) {
            let duration = output_piece.u_end - output_piece.u_start;
            for &u in &crate::lowering::LADDER_PROBES_U {
                let t = nurbs::fmadd(0.5 * (u + 1.0), duration, output_piece.u_start);
                let position = 1.0 + 2.0 * t + 3.0 * t * t + 4.0 * t * t * t;
                let velocity = 2.0 + 6.0 * t + 12.0 * t * t;
                let acceleration = 6.0 + 24.0 * t;
                let expected_position = position + adv.advance(velocity);
                let expected_acceleration = acceleration
                    + adv.curvature(velocity) * acceleration * acceleration
                    + adv.slope(velocity) * 24.0;
                let position_error = (eval(&out, t) - expected_position).abs();
                let acceleration_error =
                    (eval(&second_derivative, t) - expected_acceleration).abs();
                assert!(
                    position_error <= tolerance.pos_mm,
                    "{model:?}: position probe u={u} at t={t} exceeds budget: \
                     {position_error} > {}",
                    tolerance.pos_mm
                );
                assert!(
                    acceleration_error <= tolerance.accel_mm_s2,
                    "{model:?}: acceleration probe u={u} at t={t} exceeds budget: \
                     {acceleration_error} > {}",
                    tolerance.accel_mm_s2
                );
            }
        }
        for i in 0..=100 {
            let t = 0.1 * i as f64 / 100.0;
            assert!(eval(&out, t).is_finite());
            assert!(eval(&first_derivative, t).is_finite());
            assert!(eval(&second_derivative, t).is_finite());
        }
    }
}

/// The advance signal joins the track's pieces at shared position and
/// velocity, so the second piece's velocity zero moves away from the raw
/// piece's own root. `Reciprocal` flips its curvature sign there, and that
/// one-sided acceleration is only representable if the seam is seeded from
/// the joined coefficients.
#[test]
fn nonlinear_advance_seeds_the_joined_velocity_zero() {
    let track = nurbs::bezier::bezier_pieces_to_nurbs(&[
        nurbs::bezier::BezierPiece {
            u_start: 0.0,
            u_end: 1.0,
            coeffs: vec![0.0, 1.0, -0.4995],
        },
        nurbs::bezier::BezierPiece {
            u_start: 1.0,
            u_end: 2.0,
            coeffs: vec![0.5005, 50.0, -10.0],
        },
    ]);
    let adv = trajectory::NonlinearAdvance {
        model: trajectory::AdvanceModel::Reciprocal,
        linear_advance: 0.0,
        nonlinear_offset: 0.08,
        linearization_velocity: 5.0,
    };
    let out = crate::shaper::apply_nonlinear_advance_to_track(3, &track, adv, fit_tol(cfg()))
        .expect("the joined velocity zero is seeded, so every span fits");
    let joined = |t: f64| {
        if t <= 1.0 {
            (t * (1.0 - 0.4995 * t), 1.0 - 0.999 * t)
        } else {
            let tau = t - 1.0;
            (0.5005 + tau * (0.001 - 10.0 * tau), 0.001 - 20.0 * tau)
        }
    };
    let tolerance = fit_tol(cfg()).pos_mm;
    let pieces = nurbs::bezier::extract_bezier_pieces(&out);
    for output_piece in &pieces {
        let duration = output_piece.u_end - output_piece.u_start;
        assert!(
            duration > 0.0,
            "no zero-width sliver may reach the fitter, piece at {}",
            output_piece.u_start
        );
        for &u in crate::lowering::LADDER_PROBES_U
            .iter()
            .filter(|u| u.abs() < 1.0)
        {
            let t = nurbs::fmadd(0.5 * (u + 1.0), duration, output_piece.u_start);
            let (position, velocity) = joined(t);
            let error = (eval(&out, t) - (position + adv.advance(velocity))).abs();
            assert!(
                error <= tolerance,
                "joined advance position at t={t}: {error} > {tolerance}"
            );
        }
    }
    let root = 1.0 + 0.001 / 20.0;
    assert!(
        pieces
            .iter()
            .any(|piece| (piece.u_start - root).abs() <= 1e-9),
        "the joined velocity zero must own a piece boundary"
    );
}

#[test]
fn the_ladder_fits_a_resolution_scale_span_without_high_degree_amplification() {
    let t0 = 0.028_682_476_406_763_253;
    let t1 = 0.028_682_534_800_374_4;
    let h = t1 - t0;
    let jerk = 2850.0;
    let position = |t: f64| {
        let d = t - t0;
        0.077_158_500_926_177_32
            + d * (4.348_350_763_482_099 + d * (62.593_446_449_829_32 + d * jerk / 6.0))
    };
    let velocity = |t: f64| {
        let d = t - t0;
        4.348_350_763_482_099 + d * (125.186_892_899_658_64 + d * jerk / 2.0)
    };
    let acceleration = |t: f64| 125.186_892_899_658_64 + (t - t0) * jerk;
    let t_of = |u: f64| nurbs::fmadd(0.5 * (u + 1.0), h, t0);
    let truth_p = |u: f64| position(t_of(u));
    let truth_v = |u: f64| velocity(t_of(u));
    let truth_a = |u: f64| acceleration(t_of(u));
    let tolerance = crate::lowering::FitTol {
        pos_mm: 5e-5,
        accel_mm_s2: 2.5,
    };
    let base = crate::lowering::quintic_in_u(
        (truth_p(-1.0), truth_v(-1.0), truth_a(-1.0)),
        (truth_p(1.0), truth_v(1.0), truth_a(1.0)),
        h,
    );
    let attempt = crate::lowering::ladder_fit(
        &base,
        h,
        tolerance,
        &truth_p,
        &truth_a,
        &truth_v,
        truth_p(1.0) - truth_p(-1.0),
        f64::INFINITY,
        crate::lowering::LadderPolicy {
            endpoint_anchored: true,
            enforce_velocity_sign: true,
            high_degree_span_floor: 0.0,
        },
    );
    let fit = match attempt {
        Ok(fit) => fit,
        Err(failure) => panic!(
            "a resolution-scale span must fit without midpoint shortcuts: u={}, \
             position error {}, acceleration error {}",
            failure.u, failure.position_error, failure.acceleration_error
        ),
    };
    let track =
        nurbs::bezier::bezier_pieces_to_nurbs(&[crate::lowering::exact_piece(&fit, t0, t1, h)]);
    let first_derivative = nurbs::eval::derivative(&track);
    let second_derivative = nurbs::eval::derivative(&first_derivative);
    for &u in &crate::lowering::LADDER_PROBES_U {
        let t = t_of(u);
        let position_error = (eval(&track, t) - position(t)).abs();
        let acceleration_error = (eval(&second_derivative, t) - acceleration(t)).abs();
        assert!(
            position_error <= tolerance.pos_mm,
            "probe u={u}: position error {position_error} > {}",
            tolerance.pos_mm
        );
        assert!(
            acceleration_error <= tolerance.accel_mm_s2,
            "probe u={u}: acceleration error {acceleration_error} > {}, \
             the rung amplified endpoint rounding noise by (2/h)^2",
            tolerance.accel_mm_s2
        );
    }
    for t in [t0, t1] {
        let position_error = (eval(&track, t) - position(t)).abs();
        let velocity_error = (eval(&first_derivative, t) - velocity(t)).abs();
        assert!(
            position_error <= 1e-12,
            "endpoint t={t} must be anchored in position, off by {position_error}"
        );
        assert!(
            velocity_error <= 1e-6,
            "endpoint t={t} must be anchored in velocity, off by {velocity_error}"
        );
    }
}

#[test]
fn a_smooth_cusp_fits_below_the_high_degree_floor_without_bump_corrections() {
    let t0 = 0.028_682_476_406_763_253;
    let seed_span = 2.691_631_367_790_492_4e-7;
    let t1 = t0 + seed_span;
    let cusp_time = t1 + 2.562_573_106_7e-7;
    let cusp_accel = 1200.0;
    let cusp_speed = 3.0e-4;
    let base_speed = 4.348_350_763_482_099;
    let base_position = 0.077_158_500_926_177_32;
    let ramp = |t: f64| (cusp_speed * cusp_speed + (cusp_accel * (t - cusp_time)).powi(2)).sqrt();
    let position = |t: f64| {
        let d = t - cusp_time;
        base_position
            + base_speed * d
            + 0.5
                * (d * ramp(t)
                    + (cusp_speed * cusp_speed / cusp_accel)
                        * (cusp_accel * d / cusp_speed).asinh())
    };
    let velocity = |t: f64| base_speed + ramp(t);
    let acceleration = |t: f64| cusp_accel * cusp_accel * (t - cusp_time) / ramp(t);
    let tolerance = crate::lowering::FitTol {
        pos_mm: 5e-5,
        accel_mm_s2: 1.5,
    };
    let high_degree_span_floor = 3.6e-7;
    let mut span_start = t0;
    let span_end = t1;
    let fit = loop {
        let h = span_end - span_start;
        let t_of = |u: f64| nurbs::fmadd(0.5 * (u + 1.0), h, span_start);
        let truth_p = |u: f64| position(t_of(u));
        let truth_v = |u: f64| velocity(t_of(u));
        let truth_a = |u: f64| acceleration(t_of(u));
        let base = crate::lowering::quintic_in_u(
            (truth_p(-1.0), truth_v(-1.0), truth_a(-1.0)),
            (truth_p(1.0), truth_v(1.0), truth_a(1.0)),
            h,
        );
        let attempt = crate::lowering::ladder_fit(
            &base,
            h,
            tolerance,
            &truth_p,
            &truth_a,
            &truth_v,
            truth_p(1.0) - truth_p(-1.0),
            f64::INFINITY,
            crate::lowering::LadderPolicy {
                endpoint_anchored: true,
                enforce_velocity_sign: true,
                high_degree_span_floor,
            },
        );
        match attempt {
            Ok(fit) => break fit,
            Err(failure) => {
                assert!(
                    h > 2e-8,
                    "the cusp must fit by the 3e-8 decade, still failing at h={h}: \
                     u={}, acceleration error {} > {}",
                    failure.u,
                    failure.acceleration_error,
                    tolerance.accel_mm_s2
                );
                span_start = 0.5 * (span_start + span_end);
            }
        }
    };
    let h = span_end - span_start;
    assert!(
        h < high_degree_span_floor,
        "the span must sit below the high-degree floor for this test to mean anything"
    );
    assert!(
        fit.len() <= 6,
        "below the high-degree floor no bump-corrected rung may be accepted, got degree {}",
        fit.len() - 1
    );
    let track = nurbs::bezier::bezier_pieces_to_nurbs(&[crate::lowering::exact_piece(
        &fit, span_start, span_end, h,
    )]);
    let first_derivative = nurbs::eval::derivative(&track);
    let second_derivative = nurbs::eval::derivative(&first_derivative);
    for &u in &crate::lowering::LADDER_PROBES_U {
        let t = nurbs::fmadd(0.5 * (u + 1.0), h, span_start);
        let position_error = (eval(&track, t) - position(t)).abs();
        let acceleration_error = (eval(&second_derivative, t) - acceleration(t)).abs();
        assert!(
            position_error <= tolerance.pos_mm,
            "probe u={u}: position error {position_error} > {}",
            tolerance.pos_mm
        );
        assert!(
            acceleration_error <= tolerance.accel_mm_s2,
            "probe u={u}: acceleration error {acceleration_error} > {}",
            tolerance.accel_mm_s2
        );
    }
    for t in [span_start, span_end] {
        let position_error = (eval(&track, t) - position(t)).abs();
        let velocity_error = (eval(&first_derivative, t) - velocity(t)).abs();
        assert!(
            position_error <= 1e-12,
            "endpoint t={t} must be anchored in position, off by {position_error}"
        );
        assert!(
            velocity_error <= 1e-6,
            "endpoint t={t} must be anchored in velocity, off by {velocity_error}"
        );
    }
}

/// A constant-acceleration span whose duration is a resolution-scale 6.9e-8
/// riding a carrier near 24 mm: one ulp of the carrier is 8e-4 of the span's
/// own travel. The cubic hands that ulp to the acceleration three times over
/// through `c3` and the delta-built quadratic spends it once, while the
/// left-Taylor quadratic never touches the delta at all and reproduces a
/// constant acceleration exactly — so below the high-degree floor a degree-2
/// rung holds a -3000 mm/s² span of this length, and the travel it carries
/// can only agree with a delta recovered by subtracting absolute endpoints to
/// within the carrier's own rounding.
#[test]
fn a_constant_acceleration_resolution_span_fits_the_anchored_quadratic() {
    let t0 = 0.8300123879637047;
    let t1 = 0.8300124568154544;
    let h = t1 - t0;
    let p_start = 23.994882985793687;
    let v_start = 4.56395463731461;
    let accel = -3000.0;
    let position = |t: f64| {
        let d = t - t0;
        p_start + d * (v_start + 0.5 * accel * d)
    };
    let velocity = |t: f64| v_start + accel * (t - t0);
    let t_of = |u: f64| nurbs::fmadd(0.5 * (u + 1.0), h, t0);
    let truth_p = |u: f64| position(t_of(u));
    let truth_v = |u: f64| velocity(t_of(u));
    let truth_a = |_: f64| accel;
    let tolerance = crate::lowering::FitTol {
        pos_mm: 3e-5,
        accel_mm_s2: 1.5,
    };
    let velocity_budget = 1e-6;
    let base = crate::lowering::quintic_in_u(
        (truth_p(-1.0), truth_v(-1.0), truth_a(-1.0)),
        (truth_p(1.0), truth_v(1.0), truth_a(1.0)),
        h,
    );
    let fit = crate::lowering::ladder_fit(
        &base,
        h,
        tolerance,
        &truth_p,
        &truth_a,
        &truth_v,
        truth_p(1.0) - truth_p(-1.0),
        velocity_budget,
        crate::lowering::LadderPolicy {
            endpoint_anchored: true,
            enforce_velocity_sign: true,
            high_degree_span_floor: 3.6e-7,
        },
    )
    .unwrap_or_else(|failure| {
        panic!(
            "a constant-acceleration resolution span must fit: u={}, position error {}, \
             velocity error {}, acceleration error {} > {}",
            failure.u,
            failure.position_error,
            failure.velocity_error,
            failure.acceleration_error,
            tolerance.accel_mm_s2
        )
    });
    assert_eq!(
        fit.len(),
        3,
        "a degree-2 rung must hold this span inside the accel budget"
    );
    let track =
        nurbs::bezier::bezier_pieces_to_nurbs(&[crate::lowering::exact_piece(&fit, t0, t1, h)]);
    let first_derivative = nurbs::eval::derivative(&track);
    let second_derivative = nurbs::eval::derivative(&first_derivative);
    for &u in &crate::lowering::LADDER_PROBES_U {
        let t = t_of(u);
        let position_error = (eval(&track, t) - position(t)).abs();
        let acceleration_error = (eval(&second_derivative, t) - accel).abs();
        assert!(
            position_error <= tolerance.pos_mm,
            "probe u={u}: position error {position_error} > {}",
            tolerance.pos_mm
        );
        assert!(
            acceleration_error <= tolerance.accel_mm_s2,
            "probe u={u}: acceleration error {acceleration_error} > {}",
            tolerance.accel_mm_s2
        );
    }
    for t in [t0, t1] {
        let position_error = (eval(&track, t) - position(t)).abs();
        assert!(
            position_error <= 1e-12,
            "endpoint t={t} must be anchored in position, off by {position_error}"
        );
    }
    let coefficient_left_velocity = (fit[1] - 2.0 * fit[2]) * (2.0 / h);
    let coefficient_left_error = (coefficient_left_velocity - v_start).abs();
    assert!(
        coefficient_left_error <= 1e-12,
        "the fit must match the left seam velocity exactly, off by {coefficient_left_error}"
    );
    let coefficient_delta = 2.0 * fit[1];
    let subtracted_delta = truth_p(1.0) - truth_p(-1.0);
    let delta_error = (coefficient_delta - subtracted_delta).abs();
    let carrier_rounding = 4.0 * f64::EPSILON * p_start.abs();
    assert!(
        delta_error <= carrier_rounding,
        "the fit must carry the span's travel to within the carrier's rounding, off by \
         {delta_error} > {carrier_rounding}"
    );
    let left_velocity_error = (eval(&first_derivative, t0) - v_start).abs();
    assert!(
        left_velocity_error <= velocity_budget,
        "the left seam velocity must stay inside the validated budget, off by \
         {left_velocity_error} > {velocity_budget}"
    );
    let right_velocity_error = (eval(&first_derivative, t1) - velocity(t1)).abs();
    assert!(
        right_velocity_error <= velocity_budget,
        "the right seam velocity must stay inside the validated budget, off by \
         {right_velocity_error} > {velocity_budget}"
    );
}

fn eval_axis_at(segs: &[ContinuousSegment], axis: usize, t: f64) -> f64 {
    let seg = segs
        .iter()
        .find(|seg| t >= seg.t_start && t <= seg.t_end)
        .expect("t inside emitted trajectory");
    eval_segment_axis(seg, axis, t)
}

/// The exact convolution cut transitions land one ulp away from the segment
/// edges and from each other, and the ModeInverse chain differentiates the
/// fitted track twice. An ulp-wide knot span therefore does not merely fit
/// badly — de Boor divides by the knot spacing and the *neighbouring*
/// full-width span evaluates to ~1e14 mm, which is what drove the mode-inverse
/// residual to 1.5e16. Every emitted knot span must be wide enough to carry a
/// polynomial.
#[test]
fn mode_inverse_cut_transitions_leave_no_degenerate_knot_spans() {
    let (frequency_hz, damping_ratio) = (30.0, 0.05);
    let smooth_time = 0.0015;
    let moves = [
        line(1, [0.0, 0.0, 0.0], [40.0, 0.0, 0.0], 0.0),
        line(2, [40.0, 0.0, 0.0], [10.0, 0.0, 0.0], 0.0),
    ];
    let home = [0.0, 0.0, 0.0, 0.0];
    let inverted = replay(
        cfg(),
        x_kernel_chains(smooth_time, Some((frequency_hz, damping_ratio))),
        &home,
        0.0,
        &moves,
    );
    for seg in &inverted {
        let ContinuousAxis::Spline(curve) = &seg.axes[0] else {
            continue;
        };
        let knots = curve.knots();
        for pair in knots.windows(2) {
            let gap = pair[1] - pair[0];
            let floor = 1e-12_f64.max(8.0 * f64::EPSILON * pair[1].abs());
            assert!(
                gap == 0.0 || gap >= floor,
                "degenerate knot span {pair:?} (gap {gap:e}) in segment \
                 [{}, {}]",
                seg.t_start,
                seg.t_end
            );
        }
        let n = 512;
        for j in 0..=n {
            let t = seg.t_start + (seg.t_end - seg.t_start) * j as f64 / n as f64;
            let pva = seg
                .eval_axis(0, t)
                .expect("mode-inverse command evaluates inside its own segment");
            assert!(
                pva.position.abs() < 1e3
                    && pva.velocity.is_finite()
                    && pva.acceleration.is_finite(),
                "mode-inverse command blew up at t={t}: {pva:?}"
            );
        }
    }
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

#[test]
fn shaped_seeds_carry_every_cut_transition_exactly_once() {
    let kernel = trajectory::build_smooth_mzv_kernel(22.428_571_428_571_43);
    let input_breaks = [
        0.0,
        0.2713748376620639,
        0.7051624188303481,
        3.0128303182171217,
        18.4517596196437,
    ];
    let seeds = crate::shaper::shaped_signal_breakpoints(&kernel, &input_breaks);
    assert!(seeds.windows(2).all(|pair| pair[0] < pair[1]));
    let mut transitions = Vec::new();
    for &input_break in &input_breaks {
        for kernel_break in trajectory::ShapedSignal::kernel_cut_boundaries(&kernel) {
            trajectory::ShapedSignal::output_cut_transitions(
                &kernel,
                input_break,
                kernel_break,
                &mut transitions,
            );
        }
    }
    transitions.sort_by(f64::total_cmp);
    transitions.dedup();
    assert_eq!(seeds, transitions);
    let shifted = transitions
        .iter()
        .filter(|transition| {
            input_breaks.iter().all(|input_break| {
                trajectory::ShapedSignal::kernel_cut_boundaries(&kernel)
                    .all(|kernel_break| **transition != input_break + kernel_break)
            })
        })
        .count();
    assert!(
        shifted > 0,
        "no seed exercises a cancellation-shifted cut alignment"
    );
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
    // The window is the intersection of all three replays — `plain` (k2 = 0)
    // is sampled too, and its emit window is not guaranteed to be the widest.
    let t0 = pre
        .first()
        .unwrap()
        .t_start
        .max(post.first().unwrap().t_start)
        .max(plain.first().unwrap().t_start);
    let t1 = pre
        .last()
        .unwrap()
        .t_end
        .min(post.last().unwrap().t_end)
        .min(plain.last().unwrap().t_end);
    let mut max_k2_effect: f64 = 0.0;
    for i in 0..=400 {
        // Clamped: the last sample's product form lands an ulp past `t1`,
        // which is exactly the final segment's `t_end`.
        let t = (t0 + (t1 - t0) * i as f64 / 400.0).min(t1);
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
    cmd: &[ContinuousSegment],
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
) -> Vec<ContinuousSegment> {
    replay_stream(config, chains, home, 0.0, items)
        .into_iter()
        .filter_map(|item| match item {
            TrajectoryItem::Seg(seg) => Some(seg),
            TrajectoryItem::Parked | TrajectoryItem::Control(_) => None,
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
    let limits = VelocityLimits::try_new(1000.0, 10000.0, 0.21, f64::INFINITY).unwrap();
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
            let a = eval_segment_axis(&w[0], axis, w[0].t_end);
            let b = eval_segment_axis(&w[1], axis, w[1].t_start);
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
        assert_segment_axes_finite(seg);
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
    let limits = VelocityLimits::try_new(2800.0, 100000.0, 0.695, f64::INFINITY).unwrap();
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
                let v = (eval_segment_axis(seg, axis, t + h) - eval_segment_axis(seg, axis, t)) / h;
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

fn worst_seam_jump(segs: &[ContinuousSegment], axis: usize) -> (f64, f64) {
    const PROBE: f64 = 1e-8;
    let mut worst = (0.0, f64::NAN);
    let mut record = |jump: f64, t: f64| {
        if jump > worst.0 {
            worst = (jump, t);
        }
    };
    for seg in segs {
        let (breakpoints, _) = axis_breakpoints(&seg.axes[axis]);
        for t in breakpoints {
            if t - PROBE <= seg.t_start || t + PROBE >= seg.t_end {
                continue;
            }
            let left = eval_segment_axis(seg, axis, t - PROBE);
            let right = eval_segment_axis(seg, axis, t + PROBE);
            record((right - left).abs(), t);
        }
    }
    for pair in segs.windows(2) {
        let left = eval_segment_axis(&pair[0], axis, pair[0].t_end);
        let right = eval_segment_axis(&pair[1], axis, pair[1].t_start);
        record((right - left).abs(), pair[0].t_end);
    }
    worst
}

/// A moving leader carried across a short follower hold, with pressure
/// advance on the projected follower. The zero-extrusion middle move is
/// shorter than the leader kernel's support, so the shaped spans straddling
/// it are exactly where a midpoint constant/quadratic ladder rung used to
/// win: those rungs match `(p, v, a)` at `u = 0` only, so accepting one
/// spends the whole position budget as a step at the span seam. With every
/// accepted rung endpoint-anchored the seams stay C0, and the extruder still
/// lands on the commanded total — pressure advance included, since a
/// derivative gain nets to zero between rest and rest.
#[test]
fn moving_leader_across_a_short_follower_hold_keeps_c0_seams_and_total() {
    let moves = [
        line(1, [0.0, 0.0, 0.0], [20.0, 0.0, 0.0], 1.0),
        line(2, [20.0, 0.0, 0.0], [20.6, 0.0, 0.0], 0.0),
        line(3, [20.6, 0.0, 0.0], [40.6, 0.0, 0.0], 1.0),
    ];
    let home = [0.0, 0.0, 0.0, 0.0];
    let leader_smooth = 0.044583333333333336;

    let with_pa = replay(
        cfg(),
        follower_kernel_chains(Some(leader_smooth), Some(0.04), 0.02675),
        &home,
        0.0,
        &moves,
    );
    let without_pa = replay(
        cfg(),
        follower_kernel_chains(Some(leader_smooth), None, 0.02675),
        &home,
        0.0,
        &moves,
    );
    assert!(with_pa.len() >= 3 && without_pa.len() >= 3);
    for (label, segs) in [("with pa", &with_pa), ("without pa", &without_pa)] {
        for axis in [0, 3] {
            let (jump, t) = worst_seam_jump(segs, axis);
            assert!(
                jump < 1e-5,
                "{label}: axis {axis} seam at t={t} steps by {jump} mm — an \
                 accepted fit spent its budget as an endpoint jump"
            );
        }
    }

    let first = with_pa.first().expect("segments emitted");
    let last = with_pa.last().expect("segments emitted");
    let x_span =
        eval_segment_axis(last, 0, last.t_end) - eval_segment_axis(first, 0, first.t_start);
    assert!(
        (x_span - 40.6).abs() < 1e-2,
        "the leader must traverse the whole path across the hold, got {x_span}"
    );

    let e_with = extruder_end(&with_pa);
    let e_without = extruder_end(&without_pa);
    assert!(
        (e_with - e_without).abs() <= 0.1 * fit_tol(cfg()).pos_mm,
        "pressure advance must not move the total: {e_with} vs {e_without}"
    );
    assert!(
        (e_with - 2.0).abs() < 2e-3,
        "the follower must still deliver the commanded 2.0 mm total, got {e_with}"
    );
}

/// The voron0 bench chain set (`tools/sim/tests/test_voron0_migration.py`):
/// smooth_mzv on the CoreXY leaders at the measured belt frequencies, a
/// smooth_bell on the gear-reduced Z, and a smooth_triangle on the extruder
/// declared as a follower of x/y/z.
fn voron0_chains() -> AxisChainSet {
    let compile = |name: &str,
                   algo: &'static dyn trajectory::algos::PostProcessorAlgo,
                   param: f64| {
        trajectory::CompiledChain::compile(&[PostProcessorInstance::new(name, algo, vec![param])])
            .expect("single post-processor always compiles")
    };
    AxisChainSet {
        chains: vec![
            compile("x_shaping", &trajectory::algos::SmoothMzv, 112.8),
            compile("y_shaping", &trajectory::algos::SmoothMzv, 90.2),
            compile("z_shaping", &trajectory::algos::SmoothBell, 0.025),
            compile("e_smoothing", &trajectory::algos::SmoothTriangle, 0.01),
        ],
        followers: vec![(3, vec![0, 1, 2])],
    }
}

fn voron0_config() -> StreamConfig {
    StreamConfig {
        corner: CornerFitConfig::default(),
        integration_tol: 1e-4,
        max_extrude_only_velocity_mm_s: f64::INFINITY,
        max_extrude_only_accel_mm_s2: f64::INFINITY,
        fit_tol_mm: 0.005,
        fit_tol_accel_mm_s2: 50.0,
        max_buffer_moves: 128,
        limits: VelocityLimits::try_new(600.0, 20000.0, 0.04, f64::INFINITY).unwrap(),
    }
}

/// `SET_KINEMATIC_POSITION X=60 Y=60 Z=20` then the migration test's move
/// sequence: two F18000 travels, an F1200 Z drop, an F6000 extruding
/// diagonal, and the `M400` drain.
fn voron0_stream() -> (Vec<f64>, Vec<StreamInput>) {
    let limits = voron0_config().limits;
    let mv = |line_no: u32, start: [f64; 3], end: [f64; 3], e: f64, feed: f64| {
        StreamInput::Move(
            line_move(
                start,
                end,
                e,
                MoveContext {
                    extruder_axis: 3,
                    feedrate_mm_s: feed,
                    limits,
                    source: SourceRange {
                        start_line: line_no,
                        end_line: line_no,
                    },
                },
            )
            .unwrap(),
        )
    };
    let items = vec![
        mv(1, [60.0, 60.0, 20.0], [100.0, 100.0, 20.0], 0.0, 300.0),
        mv(2, [100.0, 100.0, 20.0], [20.0, 100.0, 20.0], 0.0, 300.0),
        mv(3, [20.0, 100.0, 20.0], [20.0, 100.0, 10.0], 0.0, 20.0),
        mv(4, [20.0, 100.0, 10.0], [60.0, 60.0, 10.0], 2.0, 100.0),
        StreamInput::Drain,
    ];
    (vec![60.0, 60.0, 20.0, 0.0], items)
}

/// Everything ahead of the shaper, run to completion — the lowered stream is
/// the shaper's input and is identical no matter how the shaper consumes it.
fn lower_to_base_items(
    config: StreamConfig,
    chains: &AxisChainSet,
    home: &[f64],
    items: Vec<StreamInput>,
) -> Vec<BaseItem> {
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
    run_lowerer(planned_rx, lowered_tx, chains.clone(), home.to_vec(), 0.0);
    lowered_rx.into_iter().collect()
}

/// `Shaper::run` over a pre-filled closed channel: the loop's `try_recv`
/// burst buffers up to `STAGE_CHANNEL_CAP` lowered segments before each emit,
/// so every emit window covers many segments at once.
fn shape_in_bursts(
    chains: AxisChainSet,
    fit_tol: FitTol,
    items: Vec<BaseItem>,
) -> Vec<TrajectoryItem> {
    let (in_tx, in_rx) = unbounded();
    for item in items {
        in_tx.send(item).unwrap();
    }
    drop(in_tx);
    let (out_tx, out_rx) = unbounded();
    Shaper::new(chains, fit_tol).run(in_rx, out_tx);
    out_rx.into_iter().collect()
}

/// `Shaper::feed` per item — the single-threaded host driver, which forces an
/// emit decision after every single lowered segment.
fn shape_one_at_a_time(
    chains: AxisChainSet,
    fit_tol: FitTol,
    items: Vec<BaseItem>,
) -> Vec<TrajectoryItem> {
    let (out_tx, out_rx) = unbounded();
    let mut shaper = Shaper::new(chains, fit_tol);
    for item in items {
        assert!(shaper.feed(item, &out_tx), "the collector never hangs up");
    }
    shaper.finish(&out_tx);
    drop(out_tx);
    out_rx.into_iter().collect()
}

fn item_kind(item: &TrajectoryItem) -> (&'static str, f64, f64) {
    match item {
        TrajectoryItem::Seg(seg) => ("seg", seg.t_start, seg.t_end),
        TrajectoryItem::Parked => ("parked", 0.0, 0.0),
        TrajectoryItem::Control(_) => ("control", 0.0, 0.0),
    }
}

fn trajectory_segments(items: &[TrajectoryItem]) -> Vec<&ContinuousSegment> {
    items
        .iter()
        .filter_map(|item| match item {
            TrajectoryItem::Seg(seg) => Some(seg),
            _ => None,
        })
        .collect()
}

/// Every knot / phase boundary the emitted tracks carry, summed over all axes
/// — the quantity that explodes when a fit ladder bisects away or when an
/// emit window is re-fitted per batch instead of reused.
fn total_track_breakpoints(items: &[TrajectoryItem]) -> usize {
    trajectory_segments(items)
        .iter()
        .map(|seg| {
            seg.axes
                .iter()
                .map(|axis| axis_breakpoints(axis).0.len())
                .sum::<usize>()
        })
        .sum()
}

/// Chunking the shaper's input must be a scheduling detail, never a semantic
/// one: the burst driver and the one-item-at-a-time driver group the same
/// lowered segments into different emit windows, and the trajectory they
/// produce must have the same structure, the same sampled motion, and the
/// same order of magnitude of fitted pieces.
///
/// Piece count is asserted against a fixed budget rather than a recomputed
/// expectation, so the test cannot drift with the fitter it guards. Batching
/// is not free today, and the cost is not spread evenly: the leaders are
/// exactly batch-invariant because the shaped-leader cache fits each segment
/// once, while the projected follower costs ~2.8x more pieces one-at-a-time
/// than bursted. The follower's post-kernel fit is the only stage without
/// that cache — it reruns on every emit over the committed range alone, so a
/// one-segment commit re-partitions what a wide column would have shared.
/// The per-axis bound pins that attribution; the total bound trips on
/// genuine multiplication.
#[test]
fn voron0_shaper_output_is_independent_of_input_batching() {
    let config = voron0_config();
    let chains = voron0_chains();
    let (home, items) = voron0_stream();

    let burst = shape_in_bursts(
        chains.clone(),
        fit_tol(config),
        lower_to_base_items(config, &chains, &home, items),
    );
    let single = shape_one_at_a_time(
        chains.clone(),
        fit_tol(config),
        lower_to_base_items(config, &chains, &home, voron0_stream().1),
    );

    let burst_segs = trajectory_segments(&burst);
    let single_segs = trajectory_segments(&single);
    assert!(
        !burst_segs.is_empty(),
        "the voron0 sequence must produce a trajectory"
    );

    let burst_kinds: Vec<_> = burst.iter().map(item_kind).collect();
    let single_kinds: Vec<_> = single.iter().map(item_kind).collect();
    assert_eq!(
        burst_kinds.len(),
        single_kinds.len(),
        "batching changed the emitted item count: {} vs {}",
        burst_kinds.len(),
        single_kinds.len()
    );
    for (i, (b, s)) in burst_kinds.iter().zip(&single_kinds).enumerate() {
        assert_eq!(b.0, s.0, "item {i} kind changed with batching");
        assert!(
            (b.1 - s.1).abs() < 1e-12 && (b.2 - s.2).abs() < 1e-12,
            "item {i} time span changed with batching: [{}, {}] vs [{}, {}]",
            b.1,
            b.2,
            s.1,
            s.2
        );
    }

    let budget = 4.0 * config.fit_tol_mm;
    for (i, (b, s)) in burst_segs.iter().zip(&single_segs).enumerate() {
        assert_eq!(b.axes.len(), s.axes.len(), "segment {i} axis count changed");
        for axis in 0..b.axes.len() {
            for k in 0..=8 {
                let t = b.t_start + (b.t_end - b.t_start) * f64::from(k) / 8.0;
                let pb = eval_segment_axis(b, axis, t);
                let ps = eval_segment_axis(s, axis, t);
                assert!(
                    (pb - ps).abs() <= budget,
                    "segment {i} axis {axis} moved with batching at t={t}: {pb} vs {ps}"
                );
            }
        }
        assert_segment_axes_finite(b);
        assert_segment_axes_finite(s);
    }

    let axis_pieces = |items: &[TrajectoryItem], axis: usize| -> usize {
        trajectory_segments(items)
            .iter()
            .map(|seg| axis_breakpoints(&seg.axes[axis]).0.len())
            .sum()
    };

    // Leaders are fitted once into the shaper's shaped-leader cache and
    // reused bit-identically by every later emit, so their piece counts do
    // not depend on how the input was chunked at all: measured 1540/1540 on
    // x and 1296/1296 on y, with z at 712 against 692. Anything beyond a few
    // percent here means that cache stopped being reused.
    for axis in 0..3 {
        let (b, s) = (axis_pieces(&burst, axis), axis_pieces(&single, axis));
        let (lo, hi) = (b.min(s), b.max(s));
        assert!(
            hi <= lo + lo / 10,
            "leader axis {axis} refitted with batching: {b} bursted vs {s} one-at-a-time — \
             the shaped-leader cache is no longer reused across emit windows"
        );
    }

    let burst_pieces = total_track_breakpoints(&burst);
    let single_pieces = total_track_breakpoints(&single);
    // Measured on this sequence (12 segments): 20_126 bursted, 49_410
    // one-at-a-time. The spread is entirely the projected follower, whose
    // post-kernel fit — unlike the leaders' — is redone on every emit over
    // the committed range only, costing 16_578 bursted against 45_882
    // one-at-a-time. The budget is a tripwire with headroom over the worse
    // arm, not a golden number; the ratio bound admits today's follower
    // spread and should tighten once that fit spans the frontier column.
    const PIECE_BUDGET: usize = 80_000;
    assert!(
        burst_pieces <= PIECE_BUDGET && single_pieces <= PIECE_BUDGET,
        "fitted piece count ran away: {burst_pieces} bursted, {single_pieces} one-at-a-time, \
         budget {PIECE_BUDGET}"
    );
    let (lo, hi) = (
        burst_pieces.min(single_pieces),
        burst_pieces.max(single_pieces),
    );
    assert!(
        hi <= 3 * lo,
        "batching multiplied the fitted pieces: {burst_pieces} bursted vs {single_pieces} \
         one-at-a-time"
    );
}
