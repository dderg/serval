use std::env;
use std::fs;
use std::process;

use _motion_engine::seam_harness::{default_stream_config, parse_gcode_to_moves};
use geometry::path::CurvatureProfile;
use geometry::path::Segment;
use geometry::path::lowering::PositionProfile;
use geometry::{ChainFitConfig, fit_chain};

fn dump_facets(moves: &[geometry::Move]) {
    let lh: Vec<Option<(f64, [f64; 3])>> = moves
        .iter()
        .map(|m| match &m.segment.spatial {
            Some(Segment::Line(l)) => Some((l.s_len(), l.heading_at(0.0))),
            _ => None,
        })
        .collect();
    for i in 0..lh.len() {
        let Some((len, h)) = lh[i] else { continue };
        let turn = lh
            .get(i + 1)
            .and_then(|n| *n)
            .map(|(_, hn)| {
                let d = (h[0] * hn[0] + h[1] * hn[1] + h[2] * hn[2]).clamp(-1.0, 1.0);
                d.acos().to_degrees()
            })
            .unwrap_or(f64::NAN);
        let e_per_mm = moves[i]
            .segment
            .followers
            .iter()
            .find(|f| f.axis_index == 3)
            .map_or(0.0, |f| f.ratio);
        println!("FACET {i} len={len:.3} turn={turn:.2} epmm={e_per_mm:.4}");
    }
}

fn census(label: &str, moves: &[geometry::Move], config: ChainFitConfig) {
    let outcome = fit_chain(moves, config).expect("fit");
    let (mut lines, mut arcs, mut clothoids) = (0, 0, 0);
    let mut max_arc_len = 0.0_f64;
    let mut total_arc_len = 0.0_f64;
    for m in &outcome.moves {
        match &m.segment.spatial {
            Some(Segment::Line(_)) => lines += 1,
            Some(Segment::Arc(a)) => {
                arcs += 1;
                max_arc_len = max_arc_len.max(a.s_len());
                total_arc_len += a.s_len();
            }
            Some(Segment::Clothoid(_)) => clothoids += 1,
            _ => {}
        }
    }
    println!(
        "{label:24}  Line={lines:3} Arc={arcs:3} Clothoid={clothoids:3}  | chains={} blended={} | max_arc={max_arc_len:6.2}mm total_arc={total_arc_len:6.2}mm",
        outcome.report.chains, outcome.report.blended,
    );
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        eprintln!("usage: analyze_arc_fit <in.gcode>");
        process::exit(1);
    }
    let source = fs::read_to_string(&args[1]).unwrap_or_else(|e| {
        eprintln!("cannot read: {e}");
        process::exit(1);
    });
    let limits = default_stream_config().limits;
    let moves = parse_gcode_to_moves(&source, limits);

    census("default (off)", &moves, ChainFitConfig::default());
    for tol in [0.005, 0.05, 0.2, 0.5, 1.0, 2.0, 4.0] {
        census(
            &format!("deviation_tol={tol}mm min_run=3"),
            &moves,
            ChainFitConfig::with_arc_fit(tol, 3),
        );
    }

    if std::env::var("DUMP_FACETS").is_ok() {
        dump_facets(&moves);
    }
}
