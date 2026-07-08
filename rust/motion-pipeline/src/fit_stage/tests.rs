use std::time::Duration;

use crossbeam_channel::{bounded, unbounded};
use geometry::segment::SourceRange;
use geometry::{CornerFitConfig, Move, MoveContext, VelocityLimits, line_move};

use super::FitStage;
use crate::{Control, StreamInput};

fn moves_of(rx: crossbeam_channel::Receiver<StreamInput>) -> Vec<Move> {
    rx.into_iter()
        .filter_map(|item| match item {
            StreamInput::Move(m) => Some(m),
            StreamInput::Drain | StreamInput::Control(_) => None,
        })
        .collect()
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

fn line(line_no: u32, start: [f64; 3], end: [f64; 3], e: f64) -> Move {
    line_move(start, end, e, ctx(line_no, 80.0)).unwrap()
}

/// Pre-fills the input channel and closes it before the fit stage runs, so
/// the fit stage never observes a transient-empty input: the output is the
/// pure end-of-stream fit.
fn run_fit_stage(moves: &[Move], config: CornerFitConfig) -> Vec<Move> {
    let (tx, rx) = unbounded();
    let (out_tx, out_rx) = unbounded();
    for m in moves {
        tx.send(m.clone().into()).unwrap();
    }
    drop(tx);
    FitStage::new(config).run(rx, out_tx);
    moves_of(out_rx)
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
            let end = [c[0] + r * libm::cos(a), c[1] + r * libm::sin(a), 0.0];
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
    let streamed = run_fit_stage(&moves, CornerFitConfig::default());
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
            let end = [c[0] + r * libm::cos(a), c[1] + r * libm::sin(a), 0.0];
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
    let streamed = run_fit_stage(&moves, CornerFitConfig::default());
    let arcs = streamed
        .iter()
        .filter(|m| matches!(m.segment.spatial, Some(geometry::path::Segment::Arc(_))))
        .count();
    assert_eq!(arcs, 2, "expected the epmm step to split the run");
}

#[test]
fn small_extrusion_drift_rides_one_arc_with_a_ramp() {
    // A 10% epmm step sits inside the ramp band: one arc absorbs the whole
    // window and carries a linear extrusion-rate ramp that conserves total E.
    let n = 400;
    let moves = circle_facets(n, |i| if i <= n / 2 { 0.30 } else { 0.33 });
    let streamed = run_fit_stage(&moves, CornerFitConfig::default());
    let arcs: Vec<&Move> = streamed
        .iter()
        .filter(|m| matches!(m.segment.spatial, Some(geometry::path::Segment::Arc(_))))
        .collect();
    assert_eq!(arcs.len(), 1, "expected the drift to ride one ramped arc");
    assert!(
        arcs[0].segment.followers.iter().any(|f| f.is_ramped()),
        "expected the arc to carry the drift as a ratio ramp"
    );
    let total_e = |ms: &[Move]| -> f64 {
        ms.iter()
            .flat_map(|m| {
                let len = m.segment.s_len();
                m.segment.followers.iter().map(move |f| f.delta_over(len))
            })
            .sum()
    };
    let e_in = total_e(&moves);
    let e_out = total_e(&streamed);
    assert!(
        (e_in - e_out).abs() <= 1e-6 * e_in,
        "fitted stream must conserve E: in={e_in} out={e_out}"
    );
}

#[test]
fn squeezed_chamfer_facet_is_consumed_in_the_stream() {
    // A 90° corner whose vertex is a 5µm 45°-45° chamfer between two long
    // legs — well under the junction deviation, so the facet's own corner
    // blends would be microscopic and one blend consumes it instead. The
    // facet's line number must not survive, the stream stays contiguous
    // (asserted inside the stage), and E is conserved.
    let d = 0.005 / std::f64::consts::SQRT_2;
    let moves = vec![
        line(1, [0.0, 0.0, 0.0], [10.0, 0.0, 0.0], 1.0),
        line(2, [10.0, 0.0, 0.0], [10.0 + d, d, 0.0], 0.0005),
        line(3, [10.0 + d, d, 0.0], [10.0 + d, 10.0 + d, 0.0], 1.0),
    ];
    let streamed = run_fit_stage(&moves, CornerFitConfig::default());

    let clothoids = streamed
        .iter()
        .filter(|m| {
            matches!(
                m.segment.spatial,
                Some(geometry::path::Segment::Clothoid(_))
            )
        })
        .count();
    assert_eq!(clothoids, 2, "one blend (two halves) spans the chamfer");
    assert!(
        streamed.iter().all(|m| !matches!(
            m.segment.spatial,
            Some(geometry::path::Segment::Line(_))
        ) || m.source.start_line != 2),
        "the consumed facet must not be emitted as a line"
    );

    let total_e = |ms: &[Move]| -> f64 {
        ms.iter()
            .flat_map(|m| {
                let len = m.segment.s_len();
                m.segment.followers.iter().map(move |f| f.delta_over(len))
            })
            .sum()
    };
    let e_in = total_e(&moves);
    let e_out = total_e(&streamed);
    assert!(
        (e_in - e_out).abs() <= 1e-9,
        "consumption must conserve E: in={e_in} out={e_out}"
    );
}

#[test]
fn drain_flushes_buffered_moves_without_close() {
    let (tx, rx) = bounded::<StreamInput>(64);
    let (out_tx, out_rx) = bounded::<StreamInput>(64);
    let fit_stage = FitStage::new(CornerFitConfig::default());
    let handle = std::thread::spawn(move || fit_stage.run(rx, out_tx));

    tx.send(line(1, [0.0, 0.0, 0.0], [50.0, 0.0, 0.0], 0.5).into())
        .unwrap();
    tx.send(line(2, [50.0, 0.0, 0.0], [50.0, 50.0, 0.0], 0.5).into())
        .unwrap();
    tx.send(StreamInput::Drain).unwrap();

    // Without closing the input, `Drain` must flush everything: trimmed body,
    // two blend halves, trimmed tail body, then the forwarded `Drain`.
    let mut got = Vec::new();
    for _ in 0..4 {
        let item = out_rx
            .recv_timeout(Duration::from_secs(10))
            .expect("fit stage held moves across a drain");
        assert!(matches!(item, StreamInput::Move(_)), "move expected");
        got.push(item);
    }
    assert_eq!(got.len(), 4);
    assert!(matches!(
        out_rx.recv_timeout(Duration::from_secs(10)),
        Ok(StreamInput::Drain)
    ));
    drop(tx);
    handle.join().unwrap();
}

#[test]
fn set_mesh_rebases_the_travel_align_anchor() {
    let (tx, rx) = bounded::<StreamInput>(64);
    let (out_tx, out_rx) = bounded::<StreamInput>(64);
    let fit_stage = FitStage::new(CornerFitConfig::default());
    let handle = std::thread::spawn(move || fit_stage.run(rx, out_tx));

    tx.send(line(1, [0.0, 0.0, 5.0], [10.0, 0.0, 5.0], 0.5).into())
        .unwrap();
    tx.send(StreamInput::Drain).unwrap();
    tx.send(StreamInput::Control(Control::SetMesh {
        mesh: None,
        gcode_z_rebase: 4.9,
    }))
    .unwrap();
    // The compensation travel: from the rebased resting Z back to the
    // pre-swap gcode Z, then a printing move continuing from there. Without
    // the anchor rebase the aligner snaps the travel's start to the stale
    // z=5.0 name and collapses it to a zero-length line (bench crash:
    // "travel align of line 3 failed: ZeroMotion").
    tx.send(line(3, [10.0, 0.0, 4.9], [10.0, 0.0, 5.0], 0.0).into())
        .unwrap();
    tx.send(line(4, [10.0, 0.0, 5.0], [20.0, 0.0, 5.0], 0.5).into())
        .unwrap();
    drop(tx);
    handle.join().unwrap();

    let moves = moves_of(out_rx);
    let travel = moves
        .iter()
        .find(|m| m.source.start_line == 3)
        .expect("compensation travel emitted");
    assert_eq!(
        geometry::fitter::spatial_start(travel),
        Some([10.0, 0.0, 4.9])
    );
    let end = geometry::fitter::spatial_end(travel).unwrap();
    assert!(
        end[2] > 4.95,
        "travel should climb toward the pre-swap z (tail may be blend-trimmed), got {end:?}"
    );
}
