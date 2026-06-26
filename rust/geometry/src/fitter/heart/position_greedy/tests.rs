use super::*;
use crate::fitter::CornerFitConfig;
use crate::frontend::{Move, MoveContext, VelocityLimits, line_move};
use crate::segment::SourceRange;
use std::f64::consts::PI;

fn seg(line_no: u32, a: [f64; 3], b: [f64; 3]) -> Move {
    line_move(
        a,
        b,
        0.0,
        MoveContext {
            extruder_axis: 3,
            feedrate_mm_s: 200.0,
            limits: VelocityLimits::try_new(300.0, 3000.0, 20.0).unwrap(),
            source: SourceRange {
                start_line: line_no,
                end_line: line_no,
            },
        },
    )
    .unwrap()
}

fn faceted_arc(r: f64, n: usize, span: f64) -> Vec<Move> {
    let verts: Vec<[f64; 3]> = (0..=n)
        .map(|k| {
            let phi = (k as f64 / n as f64) * span;
            [r * phi.sin(), r - r * phi.cos(), 0.0]
        })
        .collect();
    verts
        .windows(2)
        .enumerate()
        .map(|(i, w)| seg((i + 1) as u32, w[0], w[1]))
        .collect()
}

fn tol() -> f64 {
    20.0 * 20.0 * (2.0_f64.sqrt() - 1.0) / 3000.0
}

#[test]
fn full_faceted_arc_is_one_span() {
    let chain = faceted_arc(3.0, 12, PI / 2.0);
    let spans = PositionGreedy.arc_spans(&chain, tol(), 3, CornerFitConfig::default());
    assert_eq!(spans, vec![(0, 11)]);
}

#[test]
fn sharp_corner_yields_no_span() {
    let chain = vec![
        seg(1, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
        seg(2, [1.0, 0.0, 0.0], [1.0, 1.0, 0.0]),
        seg(3, [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]),
    ];
    let spans = PositionGreedy.arc_spans(&chain, tol(), 3, CornerFitConfig::default());
    assert!(spans.is_empty());
}
