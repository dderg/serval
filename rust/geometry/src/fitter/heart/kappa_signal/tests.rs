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
            limits: VelocityLimits::try_new(300.0, 3000.0, 20.0, 100_000.0).unwrap(),
            source: SourceRange {
                start_line: line_no,
                end_line: line_no,
            },
        },
    )
    .unwrap()
}

fn chain_from(verts: &[[f64; 3]]) -> Vec<Move> {
    verts
        .windows(2)
        .enumerate()
        .map(|(i, w)| seg((i + 1) as u32, w[0], w[1]))
        .collect()
}

fn faceted_arc(r: f64, n: usize, span: f64) -> Vec<Move> {
    let verts: Vec<[f64; 3]> = (0..=n)
        .map(|k| {
            let phi = (k as f64 / n as f64) * span;
            [r * phi.sin(), r - r * phi.cos(), 0.0]
        })
        .collect();
    chain_from(&verts)
}

fn tol() -> f64 {
    20.0 * 20.0 * (2.0_f64.sqrt() - 1.0) / 3000.0
}

fn jitter(k: usize) -> f64 {
    let h = (k as u64)
        .wrapping_mul(6364136223846793005)
        .wrapping_add(1442695040888963407);
    (((h >> 32) & 0xFFFF) as f64 / 32767.5) - 1.0
}

fn jittered_dense_arc(r: f64, n: usize, span: f64, amp: f64) -> Vec<Move> {
    let center = [0.0, r, 0.0];
    let verts: Vec<[f64; 3]> = (0..=n)
        .map(|k| {
            let phi = (k as f64 / n as f64) * span;
            let base = [r * phi.sin(), r - r * phi.cos(), 0.0];
            if k == 0 || k == n {
                return base;
            }
            let radial = [base[0] - center[0], base[1] - center[1], 0.0];
            let len = (radial[0] * radial[0] + radial[1] * radial[1]).sqrt();
            let push = amp * jitter(k) / len;
            [base[0] + radial[0] * push, base[1] + radial[1] * push, 0.0]
        })
        .collect();
    chain_from(&verts)
}

#[test]
fn uniform_curvature_run_is_one_span() {
    let chain = faceted_arc(3.0, 12, PI / 2.0);
    let spans = KappaSignal.arc_spans(&chain, tol(), 3, CornerFitConfig::default());
    assert_eq!(spans, vec![(0, 11)]);
}

#[test]
fn windowed_slope_recovers_arc_curvature() {
    let verts: Vec<[f64; 3]> = (0..=12)
        .map(|k| {
            let phi = (k as f64 / 12.0) * (PI / 2.0);
            [3.0 * phi.sin(), 3.0 - 3.0 * phi.cos(), 0.0]
        })
        .collect();
    let (s, theta) = turning_signal(&verts, [0.0, 0.0, 1.0]);
    let mut fit = SlopeFit::default();
    for v in 1..s.len() {
        fit.push(s[v], theta[v]);
    }
    assert!(
        (fit.slope() - 1.0 / 3.0).abs() < 5e-3,
        "slope {} should recover 1/r = 1/3",
        fit.slope()
    );
}

#[test]
fn noisy_dense_arc_is_a_single_span() {
    let chain = jittered_dense_arc(3.0, 20, PI / 2.0, 0.08 * tol());
    let spans = KappaSignal.arc_spans(&chain, tol(), 3, CornerFitConfig::default());
    assert_eq!(
        spans.len(),
        1,
        "robust slope fit must not fragment a noisy arc: {spans:?}"
    );
}

#[test]
fn inflection_breaks_the_span() {
    let r = 3.0;
    let m = 8;
    let mut verts: Vec<[f64; 3]> = (0..=m)
        .map(|k| {
            let phi = (k as f64 / m as f64) * (PI / 2.0);
            [r * phi.sin(), r - r * phi.cos(), 0.0]
        })
        .collect();
    for k in 1..=m {
        let psi = (k as f64 / m as f64) * (PI / 2.0);
        verts.push([2.0 * r - r * psi.cos(), r + r * psi.sin(), 0.0]);
    }
    let chain = chain_from(&verts);
    let join = m;
    let spans = KappaSignal.arc_spans(&chain, tol(), 3, CornerFitConfig::default());
    assert!(!spans.is_empty());
    assert!(
        spans.iter().all(|&(a, b)| b < join || a >= join),
        "no span may straddle the inflection at leg {join}: {spans:?}"
    );
}

#[test]
fn sharp_corner_yields_no_span() {
    let chain = vec![
        seg(1, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
        seg(2, [1.0, 0.0, 0.0], [1.0, 1.0, 0.0]),
        seg(3, [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]),
    ];
    let spans = KappaSignal.arc_spans(&chain, tol(), 3, CornerFitConfig::default());
    assert!(spans.is_empty());
}
