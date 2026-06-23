//! Local micro-benchmark: how `plan_velocity` cost and trajectory time scale
//! with `integration_tol`. Run explicitly (release):
//!   cargo test --release -p geometry --test plan_velocity_bench -- --ignored --nocapture

use std::time::Instant;

use geometry::{
    ChainFitConfig, Move, MoveContext, SourceRange, VelocityConfig, VelocityLimits, fit_chain,
    line_move, plan_velocity_warm_start,
};

const MAX_V: f64 = 300.0;
const ACCEL: f64 = 5000.0;
const SCV: f64 = 5.0;

fn ctx(line_no: u32, feedrate_mm_s: f64) -> MoveContext {
    MoveContext {
        extruder_axis: 3,
        feedrate_mm_s,
        limits: VelocityLimits::try_new(MAX_V, ACCEL, SCV).unwrap(),
        source: SourceRange {
            start_line: line_no,
            end_line: line_no,
        },
    }
}

fn line(line_no: u32, feed: f64, start: [f64; 3], end: [f64; 3]) -> Move {
    line_move(start, end, 0.0, ctx(line_no, feed)).expect("line_move")
}

/// A serpentine of short segments with a corner at every vertex — the corner
/// turns become clothoid blends, the shape `plan_velocity` integrates per run.
fn serpentine(n_moves: usize, feed: f64) -> Vec<Move> {
    let mut verts: Vec<[f64; 3]> = Vec::with_capacity(n_moves + 1);
    let dx = 2.0;
    let dy = 1.0;
    let mut x = 0.0;
    for i in 0..=n_moves {
        let y = if i % 2 == 0 { 0.0 } else { dy };
        verts.push([x, y, 0.0]);
        x += dx;
    }
    verts
        .windows(2)
        .enumerate()
        .map(|(i, w)| line(i as u32 + 1, feed, w[0], w[1]))
        .collect()
}

#[test]
#[ignore]
fn plan_velocity_cost_vs_tol() {
    let moves = serpentine(40, 100.0);
    let outcome = fit_chain(&moves, ChainFitConfig::default()).expect("fit_chain");
    println!(
        "fit: in={} out={} segments",
        moves.len(),
        outcome.moves.len()
    );

    for &tol in &[1e-7_f64, 1e-6, 1e-5, 1e-4, 1e-3, 1e-2] {
        let cfg = VelocityConfig {
            integration_tol: tol,
            ..VelocityConfig::default()
        };
        // warm once, then time the median of a few runs
        let mut best = f64::INFINITY;
        let mut traversal = 0.0;
        for _ in 0..5 {
            let clock = Instant::now();
            let profile = plan_velocity_warm_start(&outcome, cfg, 0.0).expect("plan");
            let us = clock.elapsed().as_secs_f64() * 1e6;
            best = best.min(us);
            traversal = profile.report.traversal_time_s;
        }
        println!(
            "tol={:>8.0e}  plan={:>10.1} us   traversal_time={:.9} s",
            tol, best, traversal
        );
    }
}
