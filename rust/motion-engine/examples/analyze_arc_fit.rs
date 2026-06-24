use std::env;
use std::f64::consts::PI;
use std::fs;
use std::process;

use _motion_engine::seam_harness::{default_stream_config, parse_gcode_to_moves};
use geometry::path::CurvatureProfile;
use geometry::path::Segment;
use geometry::path::lowering::PositionProfile;
use geometry::{ChainFitConfig, fit_chain};

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn unit(a: [f64; 3]) -> [f64; 3] {
    let n = dot3(a, a).sqrt();
    if n < 1e-12 {
        [0.0; 3]
    } else {
        [a[0] / n, a[1] / n, a[2] / n]
    }
}

fn analyze_runs(moves: &[geometry::Move]) {
    let mut headings: Vec<([f64; 3], f64)> = Vec::new();
    for m in moves {
        if let Some(Segment::Line(l)) = &m.segment.spatial {
            let s = l.s_len();
            let h = unit(sub(l.point_at(s), l.point_at(0.0)));
            headings.push((h, s));
        } else {
            headings.push(([0.0; 3], -1.0));
        }
    }
    let n = headings.len();
    let mut i = 0;
    println!("\n=== maximal turning runs (wide-open gates: facet=INF, turn<180deg) ===");
    while i + 1 < n {
        if headings[i].1 < 0.0 {
            i += 1;
            continue;
        }
        let mut end = i;
        let mut turning = 0u32;
        let mut total_deg = 0.0;
        let mut sign0: Option<f64> = None;
        let mut reason = "end-of-list";
        let mut max_facet = headings[i].1;
        while end + 1 < n {
            let (ta, sa) = headings[end];
            let (tb, sb) = headings[end + 1];
            if sa < 0.0 || sb < 0.0 {
                reason = "non-line";
                break;
            }
            max_facet = max_facet.max(sb);
            let theta = dot3(ta, tb).clamp(-1.0, 1.0).acos();
            let deg = theta.to_degrees();
            if theta >= std::f64::consts::PI - 1e-6 {
                reason = "reversal(theta>=180)";
                break;
            }
            if theta <= 1e-6 {
                end += 1;
                continue;
            }
            let axis = unit(cross3(ta, tb));
            let sign = match sign0 {
                None => {
                    sign0 = Some(1.0);
                    1.0
                }
                Some(_) => dot3(axis, unit(cross3(headings[i].0, headings[i + 1].0))).signum(),
            };
            if let Some(s0) = sign0 {
                if end > i && sign != s0 {
                    reason = "turn-sign-flip";
                    break;
                }
                let _ = s0;
            }
            total_deg += deg;
            turning += 1;
            end += 1;
        }
        if turning >= 2 {
            println!(
                "run [{i:3}..{end:3}]  facets={:3}  turning_junctions={turning:3}  total_turn={total_deg:6.1}deg  max_facet={max_facet:.3}mm  break={reason}",
                end - i + 1
            );
        }
        i = if end > i { end } else { i + 1 };
    }
}

fn census(label: &str, moves: &[geometry::Move], config: ChainFitConfig) {
    let outcome = fit_chain(moves, config).expect("fit");
    let (mut lines, mut arcs, mut clothoids, mut other) = (0, 0, 0, 0);
    let mut max_arc_len = 0.0_f64;
    let mut max_clothoid_run = 0u32;
    let mut clothoid_run = 0u32;
    for m in &outcome.moves {
        match &m.segment.spatial {
            Some(Segment::Line(_)) => {
                lines += 1;
                clothoid_run = 0;
            }
            Some(Segment::Arc(a)) => {
                arcs += 1;
                max_arc_len = max_arc_len.max(a.s_len());
                clothoid_run = 0;
            }
            Some(Segment::Clothoid(_)) => {
                clothoids += 1;
                clothoid_run += 1;
                max_clothoid_run = max_clothoid_run.max(clothoid_run);
            }
            _ => other += 1,
        }
    }
    println!(
        "{label:28}  Line={lines:3} Arc={arcs:3} Clothoid={clothoids:3} other={other}  | chains={} blended={} | max_arc={max_arc_len:5.2}mm max_clothoid_run={max_clothoid_run}",
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

    census("default (arc_fit=None)", &moves, ChainFitConfig::default());
    census(
        "arc_fit facet=1mm turn=12deg",
        &moves,
        ChainFitConfig::with_arc_fit(1.0, 12f64.to_radians()),
    );
    census(
        "arc_fit facet=2mm turn=90deg",
        &moves,
        ChainFitConfig::with_arc_fit(2.0, 90f64.to_radians()),
    );
    census(
        "arc_fit facet=INF turn=180deg",
        &moves,
        ChainFitConfig::with_arc_fit(f64::INFINITY, PI - 1e-6),
    );

    let mut loose = ChainFitConfig::with_arc_fit(f64::INFINITY, PI - 1e-6);
    loose.cocircular_tol = 1.0;
    census("  + cocircular_tol=1mm", &moves, loose);

    for tol in [0.005, 0.025, 0.1, 1.0] {
        let mut c = ChainFitConfig::with_arc_fit(2.0, 90f64.to_radians());
        c.cocircular_tol = tol;
        census(&format!("facet=2mm turn=90 tol={tol}mm"), &moves, c);
    }

    analyze_runs(&moves);
}
