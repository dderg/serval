use std::time::Duration;

use crossbeam_channel::{bounded, unbounded};
use geometry::segment::SourceRange;
use geometry::{ChainFitConfig, Move, MoveContext, VelocityLimits, line_move};

use super::fitter::Fitter;

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

fn line(line_no: u32, start: [f64; 3], end: [f64; 3], e: f64) -> Move {
    line_move(start, end, e, ctx(line_no, 80.0)).unwrap()
}

/// Pre-fills the input channel and closes it before the fitter runs, so the
/// fitter never observes a transient-empty input: the output is the pure
/// end-of-stream fit.
fn run_fitter(moves: &[Move], config: ChainFitConfig) -> Vec<Move> {
    let (tx, rx) = unbounded();
    let (out_tx, out_rx) = unbounded();
    for m in moves {
        tx.send(m.clone()).unwrap();
    }
    drop(tx);
    Fitter::new(config).run(rx, out_tx);
    out_rx.into_iter().map(|fm| fm.piece).collect()
}

fn half_circle(
    first_line_no: u32,
    start: [f64; 3],
    c: [f64; 2],
    r: f64,
    n: u32,
    a0: f64,
    e_per_facet: f64,
) -> Vec<Move> {
    let mut prev = start;
    (1..=n)
        .map(|i| {
            let a = a0 + std::f64::consts::PI * f64::from(i) / f64::from(n);
            let end = [c[0] + r * a.cos(), c[1] + r * a.sin(), 0.0];
            let m = line(first_line_no + i - 1, prev, end, e_per_facet);
            prev = end;
            m
        })
        .collect()
}

#[test]
fn arc_mode_all_line_input_reconstructs_arc() {
    // Production shape: no non-line moves at all. Straight collinear runs feed
    // into a dense polygonal half-circle and back out; the run seals when the
    // first straight facet breaks the arc fit, and eases into the tangent
    // lines on both sides.
    let mut moves = Vec::new();
    let mut prev = [10.0, 50.0, 0.0];
    for i in 0..4u32 {
        let end = [15.0 + 5.0 * f64::from(i), 50.0, 0.0];
        moves.push(line(i + 1, prev, end, 0.3));
        prev = end;
    }
    let arc = half_circle(5, prev, [50.0, 50.0], 20.0, 400, std::f64::consts::PI, 0.3);
    prev = *geometry::fitter::spatial_end(arc.last().unwrap())
        .as_ref()
        .unwrap();
    moves.extend(arc);
    for i in 0..4u32 {
        let end = [prev[0] + 5.0 + 5.0 * f64::from(i), prev[1], 0.0];
        moves.push(line(405 + i, prev, end, 0.3));
        prev = end;
    }
    let streamed = run_fitter(&moves, ChainFitConfig::with_arc_fit(3));
    assert!(
        streamed
            .iter()
            .any(|m| matches!(m.segment.spatial, Some(geometry::path::Segment::Arc(_)))),
        "expected the half-circle to reconstruct into an arc"
    );
}

fn circle_facets(n: u32, e_of: impl Fn(u32) -> f64) -> Vec<Move> {
    let (r, c) = (20.0_f64, [50.0, 50.0]);
    let mut prev = [c[0] + r, c[1], 0.0];
    (1..=n)
        .map(|i| {
            let a = 2.0 * std::f64::consts::PI * f64::from(i) / f64::from(n + 4);
            let end = [c[0] + r * a.cos(), c[1] + r * a.sin(), 0.0];
            let m = line(i, prev, end, e_of(i));
            prev = end;
            m
        })
        .collect()
}

#[test]
fn extrusion_ratio_step_splits_the_arc() {
    // Same circle, but extrusion per facet doubles halfway: one arc must not
    // absorb both extrusion ratios, so two runs (two Arc pieces) come out.
    let n = 400;
    let moves = circle_facets(n, |i| if i <= n / 2 { 0.3 } else { 0.6 });
    let streamed = run_fitter(&moves, ChainFitConfig::with_arc_fit(3));
    let arcs = streamed
        .iter()
        .filter(|m| matches!(m.segment.spatial, Some(geometry::path::Segment::Arc(_))))
        .count();
    assert_eq!(arcs, 2, "expected the epmm step to split the run");
}

#[test]
fn small_extrusion_drift_splits_the_stream_arc() {
    // A 10% epmm step exceeds the stream fitter's rounding tolerance for a
    // single run: two arcs come out where a looser tolerance would keep one.
    let n = 400;
    let moves = circle_facets(n, |i| if i <= n / 2 { 0.30 } else { 0.33 });
    let streamed = run_fitter(&moves, ChainFitConfig::with_arc_fit(3));
    let arcs = streamed
        .iter()
        .filter(|m| matches!(m.segment.spatial, Some(geometry::path::Segment::Arc(_))))
        .count();
    assert_eq!(arcs, 2, "expected the epmm drift to split the run");
}

#[test]
fn empty_input_flushes_buffered_moves_without_close() {
    let (tx, rx) = bounded::<Move>(64);
    let (out_tx, out_rx) = bounded::<super::FittedMove>(64);
    let fitter = Fitter::new(ChainFitConfig::default());
    let handle = std::thread::spawn(move || fitter.run(rx, out_tx));

    tx.send(line(1, [0.0, 0.0, 0.0], [50.0, 0.0, 0.0], 0.5))
        .unwrap();
    tx.send(line(2, [50.0, 0.0, 0.0], [50.0, 50.0, 0.0], 0.5))
        .unwrap();

    // Without closing the input, the fitter must still emit everything once
    // the input runs empty: trimmed body, two blend halves, trimmed tail body.
    let mut got = Vec::new();
    for _ in 0..4 {
        got.push(
            out_rx
                .recv_timeout(Duration::from_secs(10))
                .expect("fitter held moves on an empty input"),
        );
    }
    assert_eq!(got.len(), 4);
    drop(tx);
    handle.join().unwrap();
}
