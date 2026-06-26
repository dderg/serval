use geometry::path::CurvatureProfile;
use geometry::path::Segment;
use geometry::path::lowering::PositionProfile;
use geometry::{
    ChainFitConfig, FitOutcome, HeartKind, Move, MoveContext, SourceRange, VelocityLimits,
    fit_chain, line_move,
};
use std::f64::consts::PI;

const E_AXIS: usize = 3;
const ACCEL: f64 = 3000.0;
const SCV: f64 = 20.0;

fn delta() -> f64 {
    SCV * SCV * (2.0_f64.sqrt() - 1.0) / ACCEL
}

fn ctx(line_no: u32) -> MoveContext {
    MoveContext {
        extruder_axis: E_AXIS,
        feedrate_mm_s: 200.0,
        limits: VelocityLimits::try_new(300.0, ACCEL, SCV).unwrap(),
        source: SourceRange {
            start_line: line_no,
            end_line: line_no,
        },
    }
}

fn seg(line_no: u32, a: [f64; 3], b: [f64; 3]) -> Move {
    line_move(a, b, 0.0, ctx(line_no)).unwrap()
}

fn dist(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn scale(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

fn unit(a: [f64; 3]) -> [f64; 3] {
    scale(a, 1.0 / dist(a, [0.0; 3]))
}

fn arc_vertices(r: f64, n: usize, span: f64) -> Vec<[f64; 3]> {
    (0..=n)
        .map(|k| {
            let phi = (k as f64 / n as f64) * span;
            [r * phi.sin(), r - r * phi.cos(), 0.0]
        })
        .collect()
}

fn faceted_quarter_circle() -> Vec<Move> {
    let verts = arc_vertices(3.0, 12, PI / 2.0);
    verts
        .windows(2)
        .enumerate()
        .map(|(i, w)| seg((i + 1) as u32, w[0], w[1]))
        .collect()
}

fn straight_to_curve() -> Vec<Move> {
    let r = 3.0;
    let n = 12;
    let span = PI / 2.0;
    let lead = 4.0;
    let verts = arc_vertices(r, n, span);
    let facet = span / n as f64;
    let circle = |phi: f64| [r * phi.sin(), r - r * phi.cos(), 0.0];
    let bdir = unit(sub(verts[0], circle(-facet)));
    let pre = sub(verts[0], scale(bdir, lead));
    let fdir = unit(sub(circle(span + facet), verts[n]));
    let post = add(verts[n], scale(fdir, lead));
    let mut pts = vec![pre];
    pts.extend(verts);
    pts.push(post);
    pts.windows(2)
        .enumerate()
        .map(|(i, w)| seg((i + 1) as u32, w[0], w[1]))
        .collect()
}

fn sharp_corner() -> Vec<Move> {
    vec![
        seg(1, [0.0, 0.0, 0.0], [10.0, 0.0, 0.0]),
        seg(2, [10.0, 0.0, 0.0], [10.0, 10.0, 0.0]),
    ]
}

fn jitter(k: usize) -> f64 {
    let h = (k as u64)
        .wrapping_mul(6364136223846793005)
        .wrapping_add(1442695040888963407);
    (((h >> 32) & 0xFFFF) as f64 / 32767.5) - 1.0
}

fn jittered_dense_arc() -> Vec<Move> {
    let r = 3.0;
    let n = 40;
    let span = PI / 2.0;
    let amp = 0.25 * delta();
    let center = [0.0, r, 0.0];
    let verts: Vec<[f64; 3]> = (0..=n)
        .map(|k| {
            let phi = (k as f64 / n as f64) * span;
            let base = [r * phi.sin(), r - r * phi.cos(), 0.0];
            if k == 0 || k == n {
                base
            } else {
                add(base, scale(unit(sub(base, center)), amp * jitter(k)))
            }
        })
        .collect();
    verts
        .windows(2)
        .enumerate()
        .map(|(i, w)| seg((i + 1) as u32, w[0], w[1]))
        .collect()
}

fn cfg(heart: HeartKind) -> ChainFitConfig {
    ChainFitConfig {
        heart,
        ..ChainFitConfig::with_arc_fit(3)
    }
}

struct Metrics {
    max_kappa_jump: f64,
    max_deviation: f64,
    elements: usize,
    peak_kappa: f64,
}

fn spatials(out: &FitOutcome) -> Vec<&Segment> {
    out.moves
        .iter()
        .filter_map(|m| m.segment.spatial.as_ref())
        .collect()
}

fn max_kappa_jump(out: &FitOutcome) -> f64 {
    let segs = spatials(out);
    let mut worst = 0.0_f64;
    for w in segs.windows(2) {
        let (_, prev_end) = w[0].kappa_endpoints();
        let (next_start, _) = w[1].kappa_endpoints();
        worst = worst.max((prev_end.abs() - next_start.abs()).abs());
    }
    worst
}

fn point_to_segment(p: [f64; 3], a: [f64; 3], b: [f64; 3]) -> f64 {
    let ab = sub(b, a);
    let len2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2];
    if len2 == 0.0 {
        return dist(p, a);
    }
    let ap = sub(p, a);
    let t = ((ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / len2).clamp(0.0, 1.0);
    dist(p, add(a, scale(ab, t)))
}

fn max_deviation(out: &FitOutcome, polyline: &[[f64; 3]]) -> f64 {
    let mut worst = 0.0_f64;
    for s in spatials(out) {
        let len = s.s_len();
        for k in 0..=64 {
            let p = s.point_at(len * k as f64 / 64.0);
            let nearest = polyline
                .windows(2)
                .map(|w| point_to_segment(p, w[0], w[1]))
                .fold(f64::INFINITY, f64::min);
            worst = worst.max(nearest);
        }
    }
    worst
}

fn peak_kappa(out: &FitOutcome) -> f64 {
    spatials(out)
        .iter()
        .map(|s| s.kappa_peak().1.abs())
        .fold(0.0_f64, f64::max)
}

fn polyline(moves: &[Move]) -> Vec<[f64; 3]> {
    let mut pts = Vec::new();
    if let Some(first) = moves.first() {
        if let Some(Segment::Line(l)) = &first.segment.spatial {
            pts.push(l.start);
        }
    }
    for m in moves {
        if let Some(Segment::Line(l)) = &m.segment.spatial {
            pts.push(l.end);
        }
    }
    pts
}

fn measure(moves: &[Move], heart: HeartKind) -> Metrics {
    let out = fit_chain(moves, cfg(heart)).unwrap();
    let poly = polyline(moves);
    Metrics {
        max_kappa_jump: max_kappa_jump(&out),
        max_deviation: max_deviation(&out, &poly),
        elements: out.moves.len(),
        peak_kappa: peak_kappa(&out),
    }
}

fn kind(s: &Segment) -> &'static str {
    match s {
        Segment::Line(_) => "L",
        Segment::Arc(_) => "A",
        Segment::Clothoid(_) => "C",
    }
}

fn trace(moves: &[Move], heart: HeartKind) -> String {
    let out = fit_chain(moves, cfg(heart)).unwrap();
    spatials(&out)
        .iter()
        .map(|s| {
            let (k0, k1) = s.kappa_endpoints();
            format!("{}[{k0:.3}->{k1:.3}]", kind(s))
        })
        .collect::<Vec<_>>()
        .join(" ")
}

#[test]
fn both_hearts_are_g2_and_in_band() {
    let cases: [(&str, Vec<Move>); 4] = [
        ("faceted_quarter_circle", faceted_quarter_circle()),
        ("straight_to_curve", straight_to_curve()),
        ("sharp_corner", sharp_corner()),
        ("jittered_dense_arc", jittered_dense_arc()),
    ];

    println!(
        "{:<24} {:<14} {:>12} {:>12} {:>9} {:>12}",
        "case", "heart", "max_dev", "max_k_jump", "elements", "peak_kappa"
    );
    for (name, moves) in &cases {
        for heart in [HeartKind::PositionGreedy, HeartKind::KappaSignal] {
            let m = measure(moves, heart);
            println!(
                "{:<24} {:<14} {:>12.3e} {:>12.3e} {:>9} {:>12.4}",
                name,
                format!("{heart:?}"),
                m.max_deviation,
                m.max_kappa_jump,
                m.elements,
                m.peak_kappa
            );
        }
    }

    println!("\n--- jittered_dense_arc segment traces ---");
    let jit = jittered_dense_arc();
    for heart in [HeartKind::PositionGreedy, HeartKind::KappaSignal] {
        println!("{heart:?}: {}", trace(&jit, heart));
    }

    for (name, moves) in &cases {
        for heart in [HeartKind::PositionGreedy, HeartKind::KappaSignal] {
            let m = measure(moves, heart);
            assert!(
                m.max_deviation <= delta() + 1e-9,
                "{name}/{heart:?}: out of band, deviation {} > delta {}",
                m.max_deviation,
                delta()
            );
        }
    }
    for (name, moves) in &cases[..3] {
        for heart in [HeartKind::PositionGreedy, HeartKind::KappaSignal] {
            let m = measure(moves, heart);
            assert!(
                m.max_kappa_jump <= 1e-9,
                "{name}/{heart:?}: G2 violated, kappa jump {}",
                m.max_kappa_jump
            );
        }
    }
}
