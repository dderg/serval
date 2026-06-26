use crate::fitter::{ChainFitConfig, HeartKind, UnblendReason, fit_chain};
use crate::frontend::{Move, MoveContext, VelocityLimits, line_move};
use crate::path::lowering::PositionProfile;
use crate::path::{Arc, Clothoid, CurvatureProfile, Line, Segment};
use crate::segment::SourceRange;
use std::f64::consts::PI;

const E_AXIS: usize = 3;

fn cfg(heart: HeartKind) -> ChainFitConfig {
    ChainFitConfig {
        heart,
        ..ChainFitConfig::with_arc_fit(3)
    }
}

const HEARTS: [HeartKind; 2] = [HeartKind::PositionGreedy, HeartKind::KappaSignal];

fn ctx(line_no: u32, accel: f64, scv: f64, feed: f64) -> MoveContext {
    MoveContext {
        extruder_axis: E_AXIS,
        feedrate_mm_s: feed,
        limits: VelocityLimits::try_new(300.0, accel, scv).unwrap(),
        source: SourceRange {
            start_line: line_no,
            end_line: line_no,
        },
    }
}

fn seg(line_no: u32, accel: f64, scv: f64, feed: f64, a: [f64; 3], b: [f64; 3], e: f64) -> Move {
    line_move(a, b, e, ctx(line_no, accel, scv, feed)).unwrap()
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
    let n = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();
    [a[0] / n, a[1] / n, a[2] / n]
}

fn arc_vertices(r: f64, n: usize, span: f64) -> Vec<[f64; 3]> {
    (0..=n)
        .map(|k| {
            let phi = (k as f64 / n as f64) * span;
            [r * phi.sin(), r - r * phi.cos(), 0.0]
        })
        .collect()
}

fn faceted_arc(
    r: f64,
    n: usize,
    span: f64,
    accel: f64,
    scv: f64,
    feed: f64,
    e_per_mm: f64,
) -> Vec<Move> {
    let verts = arc_vertices(r, n, span);
    verts
        .windows(2)
        .enumerate()
        .map(|(i, w)| {
            let len = dist(w[0], w[1]);
            seg((i + 1) as u32, accel, scv, feed, w[0], w[1], e_per_mm * len)
        })
        .collect()
}

fn faceted_arc_with_leads(r: f64, n: usize, span: f64, lead: f64, e_per_mm: f64) -> Vec<Move> {
    let verts = arc_vertices(r, n, span);
    let facet = span / n as f64;
    let circle = |phi: f64| [r * phi.sin(), r - r * phi.cos(), 0.0];

    let v0 = verts[0];
    let bdir = unit(sub(v0, circle(-facet)));
    let pre = sub(v0, scale(bdir, lead));

    let vn = verts[n];
    let fdir = unit(sub(circle(span + facet), vn));
    let post = add(vn, scale(fdir, lead));

    let mut pts = vec![pre];
    pts.extend(verts);
    pts.push(post);
    pts.windows(2)
        .enumerate()
        .map(|(i, w)| {
            seg(
                (i + 1) as u32,
                3000.0,
                20.0,
                200.0,
                w[0],
                w[1],
                e_per_mm * dist(w[0], w[1]),
            )
        })
        .collect()
}

fn as_clothoid(m: &Move) -> &Clothoid {
    match &m.segment.spatial {
        Some(Segment::Clothoid(c)) => c,
        other => panic!("expected clothoid, got {other:?}"),
    }
}

fn as_arc(m: &Move) -> &Arc {
    match &m.segment.spatial {
        Some(Segment::Arc(a)) => a,
        other => panic!("expected arc, got {other:?}"),
    }
}

fn as_line(m: &Move) -> &Line {
    match &m.segment.spatial {
        Some(Segment::Line(l)) => l,
        other => panic!("expected line, got {other:?}"),
    }
}

fn total_extrusion(moves: &[Move]) -> f64 {
    moves
        .iter()
        .map(|m| {
            let s = m.segment.s_len();
            m.segment
                .followers
                .iter()
                .filter(|f| f.axis_index == E_AXIS)
                .map(|f| f.ratio * s)
                .sum::<f64>()
        })
        .sum()
}

fn arc_in(moves: &[Move]) -> &Arc {
    moves
        .iter()
        .find_map(|m| match &m.segment.spatial {
            Some(Segment::Arc(a)) => Some(a),
            _ => None,
        })
        .expect("reconstructed arc present")
}

fn has_clothoid(moves: &[Move]) -> bool {
    moves
        .iter()
        .any(|m| matches!(m.segment.spatial, Some(Segment::Clothoid(_))))
}

fn worst_kappa_jump(moves: &[Move]) -> f64 {
    let spatial: Vec<&Segment> = moves
        .iter()
        .filter_map(|m| m.segment.spatial.as_ref())
        .collect();
    let mut worst = 0.0_f64;
    for w in spatial.windows(2) {
        let (_, prev_end) = w[0].kappa_endpoints();
        let (next_start, _) = w[1].kappa_endpoints();
        worst = worst.max((prev_end - next_start).abs());
    }
    worst
}

#[test]
fn faceted_arc_reconstructs_for_both_hearts() {
    for heart in HEARTS {
        let moves = faceted_arc(3.0, 12, PI / 2.0, 3000.0, 20.0, 200.0, 0.5);
        let out = fit_chain(&moves, cfg(heart)).unwrap();
        assert_eq!(out.report.chains, 1, "{heart:?}: one faceted-arc run");
        assert!(out.report.unblended.is_empty(), "{heart:?}");
        let arc = arc_in(&out.moves);
        assert!(
            (arc.radius - 3.0).abs() < 0.05,
            "{heart:?} radius {}",
            arc.radius
        );
        assert!(
            (arc.sweep.abs() - PI / 2.0).abs() < 0.05,
            "{heart:?} sweep {}",
            arc.sweep
        );
        assert!(
            !has_clothoid(&out.moves),
            "{heart:?}: isolated faceted arc has no neighbour lines to ease into"
        );
    }
}

#[test]
fn straight_to_curve_is_g2_and_in_band_for_both_hearts() {
    let accel = 3000.0;
    let scv = 20.0;
    let delta = scv * scv * (2.0_f64.sqrt() - 1.0) / accel;
    for heart in HEARTS {
        let moves = faceted_arc_with_leads(3.0, 12, PI / 2.0, 4.0, 0.0);
        let out = fit_chain(&moves, cfg(heart)).unwrap();
        assert_eq!(out.report.chains, 1, "{heart:?}");
        assert!(out.report.unblended.is_empty(), "{heart:?}");
        assert!(
            has_clothoid(&out.moves),
            "{heart:?}: tangent leads get a curvature easement"
        );
        assert!(
            worst_kappa_jump(&out.moves) <= 1e-9,
            "{heart:?}: kappa jump {}",
            worst_kappa_jump(&out.moves)
        );

        let i = out
            .moves
            .iter()
            .position(|m| matches!(m.segment.spatial, Some(Segment::Arc(_))))
            .unwrap();
        let arc = as_arc(&out.moves[i]);
        let up = as_clothoid(&out.moves[i - 1]);
        let line = as_line(&out.moves[i - 2]);
        let dotp = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        assert!(
            dist(line.point_at(line.s_len()), up.point_at(0.0)) < 1e-6,
            "{heart:?}"
        );
        assert!(up.kappa(0.0).abs() < 1e-9, "{heart:?}");
        assert!(
            dotp(up.heading_at(0.0), line.heading_at(0.0)) > 1.0 - 1e-9,
            "{heart:?}"
        );
        assert!(
            dist(up.point_at(up.s_len()), arc.point_at(0.0)) < 1e-6,
            "{heart:?}"
        );
        let k_ratio = up.kappa(up.s_len()).abs() * arc.radius;
        assert!((k_ratio - 1.0).abs() < 1e-3, "{heart:?} ratio {k_ratio}");

        let t = line.heading_at(0.0);
        let start = line.point_at(line.s_len());
        let mut worst = 0.0_f64;
        for k in 0..=16 {
            let p = up.point_at(up.s_len() * k as f64 / 16.0);
            let d = sub(p, start);
            let along = dotp(d, t);
            let perp = sub(d, scale(t, along));
            let off_line = (perp[0] * perp[0] + perp[1] * perp[1] + perp[2] * perp[2]).sqrt();
            let off_arc = (dist(p, arc.origin) - arc.radius).abs();
            worst = worst.max(off_line.min(off_arc));
        }
        assert!(
            worst <= delta,
            "{heart:?}: easement bulge {worst} past {delta}"
        );
    }
}

#[test]
fn tail_easement_is_symmetric_for_both_hearts() {
    for heart in HEARTS {
        let moves = faceted_arc_with_leads(3.0, 12, PI / 2.0, 4.0, 0.0);
        let out = fit_chain(&moves, cfg(heart)).unwrap();
        let i = out
            .moves
            .iter()
            .position(|m| matches!(m.segment.spatial, Some(Segment::Arc(_))))
            .unwrap();
        let arc = as_arc(&out.moves[i]);
        let down = as_clothoid(&out.moves[i + 1]);
        let line = as_line(&out.moves[i + 2]);
        let dotp = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        let s = arc.s_len();
        assert!(
            dist(arc.point_at(s), down.point_at(0.0)) < 1e-6,
            "{heart:?}"
        );
        let k_ratio = down.kappa(0.0).abs() * arc.radius;
        assert!((k_ratio - 1.0).abs() < 1e-3, "{heart:?} ratio {k_ratio}");
        assert!(
            dist(down.point_at(down.s_len()), line.point_at(0.0)) < 1e-6,
            "{heart:?}"
        );
        assert!(down.kappa(down.s_len()).abs() < 1e-9, "{heart:?}");
        assert!(
            dotp(down.heading_at(down.s_len()), line.heading_at(0.0)) > 1.0 - 1e-9,
            "{heart:?}"
        );
    }
}

#[test]
fn extrusion_conserved_across_run_and_easement() {
    for heart in HEARTS {
        let moves = faceted_arc(3.0, 12, PI / 2.0, 3000.0, 20.0, 200.0, 0.42);
        let before = total_extrusion(&moves);
        let after = total_extrusion(&fit_chain(&moves, cfg(heart)).unwrap().moves);
        assert!(
            (before - after).abs() < 1e-9,
            "{heart:?} run before {before} after {after}"
        );

        let leads = faceted_arc_with_leads(3.0, 12, PI / 2.0, 4.0, 0.42);
        let before = total_extrusion(&leads);
        let after = total_extrusion(&fit_chain(&leads, cfg(heart)).unwrap().moves);
        assert!(
            (before - after).abs() < 1e-9,
            "{heart:?} eased before {before} after {after}"
        );
    }
}

#[test]
fn no_arc_fit_config_never_chains() {
    let moves = faceted_arc(3.0, 12, PI / 2.0, 3000.0, 20.0, 200.0, 0.0);
    let out = fit_chain(&moves, ChainFitConfig::default()).unwrap();
    assert_eq!(out.report.chains, 0);
    assert!(!has_clothoid(&out.moves) || out.report.blended > 0);
}

#[test]
fn sharp_corners_do_not_chain() {
    for heart in HEARTS {
        let moves = vec![
            seg(1, 3000.0, 5.0, 200.0, [0.0, 0.0, 0.0], [0.5, 0.0, 0.0], 0.0),
            seg(2, 3000.0, 5.0, 200.0, [0.5, 0.0, 0.0], [0.5, 0.5, 0.0], 0.0),
            seg(3, 3000.0, 5.0, 200.0, [0.5, 0.5, 0.0], [0.0, 0.5, 0.0], 0.0),
        ];
        let out = fit_chain(&moves, cfg(heart)).unwrap();
        assert_eq!(out.report.chains, 0, "{heart:?}");
    }
}

#[test]
fn long_facets_rejected_by_bulge() {
    for heart in HEARTS {
        let moves = faceted_arc(100.0, 4, 0.4, 3000.0, 20.0, 200.0, 0.0);
        let out = fit_chain(&moves, cfg(heart)).unwrap();
        assert_eq!(out.report.chains, 0, "{heart:?}");
    }
}

#[test]
fn virtual_move_breaks_the_chain() {
    for heart in HEARTS {
        let mut moves = faceted_arc(3.0, 6, PI / 3.0, 3000.0, 20.0, 200.0, 0.0);
        let split = moves.len() / 2;
        let mut virt = moves[split].clone();
        virt.segment.spatial = None;
        virt.segment.virtual_path_mm = Some(0.5);
        moves.insert(split, virt);
        let out = fit_chain(&moves, cfg(heart)).unwrap();
        assert!(out.report.chains <= 2, "{heart:?}");
        assert!(
            out.moves.iter().any(|m| m.segment.spatial.is_none()),
            "{heart:?}: the virtual move survives"
        );
    }
}

#[test]
fn arc_line_corner_blends_g2() {
    let n = 12;
    let verts = arc_vertices(3.0, n, PI / 2.0);
    for heart in HEARTS {
        let mut moves = vec![seg(1, 3000.0, 20.0, 200.0, [0.0, -4.0, 0.0], verts[0], 0.0)];
        moves.extend(verts.windows(2).enumerate().map(|(i, w)| {
            seg(
                (i + 2) as u32,
                3000.0,
                20.0,
                200.0,
                w[0],
                w[1],
                0.5 * dist(w[0], w[1]),
            )
        }));
        moves.push(seg(
            (n + 2) as u32,
            3000.0,
            20.0,
            200.0,
            verts[n],
            add(verts[n], [4.0, 0.0, 0.0]),
            0.0,
        ));
        let out = fit_chain(&moves, cfg(heart)).unwrap();
        assert_eq!(out.report.chains, 1, "{heart:?}");
        assert!(has_clothoid(&out.moves), "{heart:?}");
        let reasons: Vec<UnblendReason> = out.report.unblended.iter().map(|u| u.reason).collect();
        assert!(
            !reasons.contains(&UnblendReason::ArcIncident),
            "{heart:?}: arc-line corner must blend, not rest: {reasons:?}"
        );
        assert!(out.report.blended >= 2, "{heart:?}: {}", out.report.blended);
        assert!(
            worst_kappa_jump(&out.moves) <= 1e-9,
            "{heart:?}: g2 violated {}",
            worst_kappa_jump(&out.moves)
        );
    }
}

#[test]
fn isolated_corner_matches_per_corner() {
    use crate::fitter::fit_corners;
    for heart in HEARTS {
        let moves = vec![
            seg(
                1,
                3000.0,
                5.0,
                100.0,
                [0.0, 0.0, 0.0],
                [10.0, 0.0, 0.0],
                5.0,
            ),
            seg(
                2,
                3000.0,
                5.0,
                100.0,
                [10.0, 0.0, 0.0],
                [10.0, 10.0, 0.0],
                5.0,
            ),
        ];
        let chain = fit_chain(&moves, cfg(heart)).unwrap();
        let corners = fit_corners(&moves, Default::default()).unwrap();
        assert_eq!(chain.report.chains, 0, "{heart:?}");
        assert_eq!(chain.report.blended, corners.report.blended, "{heart:?}");
        assert_eq!(chain.moves.len(), corners.moves.len(), "{heart:?}");
    }
}

#[test]
fn short_chain_passes_through() {
    for heart in HEARTS {
        let one = vec![seg(
            1,
            3000.0,
            5.0,
            100.0,
            [0.0, 0.0, 0.0],
            [10.0, 0.0, 0.0],
            0.0,
        )];
        let out = fit_chain(&one, cfg(heart)).unwrap();
        assert_eq!(out.moves.len(), 1, "{heart:?}");
        assert_eq!(out.report.chains, 0, "{heart:?}");
    }
}
