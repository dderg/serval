use crate::fitter::{ChainFitConfig, fit_chain, fit_corners};
use crate::frontend::{Move, MoveContext, VelocityLimits, line_move};
use crate::path::lowering::PositionProfile;
use crate::path::{Arc, Clothoid, CurvatureProfile, Line, Segment};
use crate::segment::SourceRange;
use crate::velocity::{VelocityConfig, plan_velocity};
use std::f64::consts::PI;

const E_AXIS: usize = 3;

fn cfg() -> ChainFitConfig {
    ChainFitConfig::with_arc_fit(0.02, 3)
}

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

/// A faceted arc bracketed by straight lead-in / lead-out lines. Each lead
/// extends the circle's faceting one step past the boundary vertex, so it meets
/// the arc turning the *sweep* way — a genuine faceted-arc neighbour — then runs
/// straight. This is the geometry a curvature easement is built for; a lead that
/// kinked the other way would inflect and be (correctly) refused.
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

#[test]
fn faceted_arc_reconstructs_to_bare_arc() {
    let moves = faceted_arc(3.0, 12, PI / 2.0, 3000.0, 20.0, 200.0, 0.5);
    let out = fit_chain(&moves, cfg()).unwrap();

    assert_eq!(out.report.chains, 1, "one faceted-arc run reconstructed");
    assert!(
        out.report.unblended.is_empty(),
        "no stop pinned inside the run"
    );
    let arc = arc_in(&out.moves);
    assert!((arc.radius - 3.0).abs() < 0.05, "radius {}", arc.radius);
    assert!(
        (arc.sweep.abs() - PI / 2.0).abs() < 0.05,
        "sweep {}",
        arc.sweep
    );
    assert!(
        !has_clothoid(&out.moves),
        "an isolated faceted arc has no neighbor lines to ease into, so no clothoids"
    );
}

#[test]
fn near_tangent_leads_get_a_curvature_easement() {
    let moves = faceted_arc_with_leads(3.0, 12, PI / 2.0, 4.0, 0.0);
    let out = fit_chain(&moves, cfg()).unwrap();
    assert_eq!(out.report.chains, 1);

    let i = out
        .moves
        .iter()
        .position(|m| matches!(m.segment.spatial, Some(Segment::Arc(_))))
        .unwrap();
    let arc = as_arc(&out.moves[i]);
    let up = as_clothoid(&out.moves[i - 1]);
    let line = as_line(&out.moves[i - 2]);
    let dotp = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

    // C0 + C1 with the (trimmed) line: the spiral starts exactly where the line
    // now ends, tangent, straight.
    assert!(
        dist(line.point_at(line.s_len()), up.point_at(0.0)) < 1e-6,
        "easement starts on the line endpoint — no gap"
    );
    assert!(
        up.kappa(0.0).abs() < 1e-9,
        "easement leaves the line straight"
    );
    assert!(dotp(up.heading_at(0.0), line.heading_at(0.0)) > 1.0 - 1e-9);

    // C2 into the arc: position, tangent, AND curvature all continuous — the
    // spiral hands off at exactly the arc's curvature, so the toolhead neither
    // over- nor under-curves leaving it.
    assert!(dist(up.point_at(up.s_len()), arc.point_at(0.0)) < 1e-6);
    assert!(dotp(up.heading_at(up.s_len()), arc.heading_at(0.0)) > 1.0 - 1e-6);
    let k_ratio = up.kappa(up.s_len()).abs() * arc.radius;
    assert!(
        (k_ratio - 1.0).abs() < 1e-3,
        "easement curvature matches the arc at the handoff, ratio {k_ratio}"
    );
}

#[test]
fn lead_out_gets_a_symmetric_tail_easement() {
    let moves = faceted_arc_with_leads(3.0, 12, PI / 2.0, 4.0, 0.0);
    let out = fit_chain(&moves, cfg()).unwrap();
    let i = out
        .moves
        .iter()
        .position(|m| matches!(m.segment.spatial, Some(Segment::Arc(_))))
        .unwrap();
    let arc = as_arc(&out.moves[i]);
    let down = as_clothoid(&out.moves[i + 1]);
    let line = as_line(&out.moves[i + 2]);
    let dotp = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

    // C2 out of the arc: the tail spiral leaves at exactly the arc's curvature.
    let s = arc.s_len();
    assert!(dist(arc.point_at(s), down.point_at(0.0)) < 1e-6);
    assert!(dotp(arc.heading_at(s), down.heading_at(0.0)) > 1.0 - 1e-6);
    let k_ratio = down.kappa(0.0).abs() * arc.radius;
    assert!(
        (k_ratio - 1.0).abs() < 1e-3,
        "tail easement curvature matches the arc at the handoff, ratio {k_ratio}"
    );

    // C0 + C1 into the (trimmed) line: ends exactly on the line start, tangent.
    assert!(
        dist(down.point_at(down.s_len()), line.point_at(0.0)) < 1e-6,
        "tail easement ends on the line endpoint — no gap"
    );
    assert!(
        down.kappa(down.s_len()).abs() < 1e-9,
        "tail easement reaches the line straight"
    );
    assert!(dotp(down.heading_at(down.s_len()), line.heading_at(0.0)) > 1.0 - 1e-9);
}

#[test]
fn inflecting_lead_is_refused() {
    // A lead-in collinear with the first chord sits on the *outside* of the arc
    // tangent: entering the arc the heading would kink one way then the arc curve
    // back the other (an S). A curvature-continuous spiral cannot absorb that, so
    // the run keeps its bare arc rather than inventing an inflection.
    let r = 3.0;
    let n = 12;
    let span = PI / 2.0;
    let verts = arc_vertices(r, n, span);
    let chord = unit(sub(verts[1], verts[0]));
    let pre = sub(verts[0], scale(chord, 4.0));
    let last_chord = unit(sub(verts[n], verts[n - 1]));
    let post = add(verts[n], scale(last_chord, 4.0));

    let mut pts = vec![pre];
    pts.extend(verts);
    pts.push(post);
    let moves: Vec<Move> = pts
        .windows(2)
        .enumerate()
        .map(|(i, w)| seg((i + 1) as u32, 3000.0, 20.0, 200.0, w[0], w[1], 0.0))
        .collect();

    let out = fit_chain(&moves, cfg()).unwrap();
    assert_eq!(out.report.chains, 1, "the arc itself still reconstructs");
    assert!(
        !has_clothoid(&out.moves),
        "inflecting leads get no easement — the bare arc is kept"
    );
}

#[test]
fn joint_refit_moves_the_circle_within_budget() {
    let delta = 20.0 * 20.0 * (2.0_f64.sqrt() - 1.0) / 3000.0;
    let bare = fit_chain(
        &faceted_arc(3.0, 12, PI / 2.0, 3000.0, 20.0, 200.0, 0.0),
        cfg(),
    )
    .unwrap();
    let bare_origin = arc_in(&bare.moves).origin;

    let out = fit_chain(&faceted_arc_with_leads(3.0, 12, PI / 2.0, 4.0, 0.0), cfg()).unwrap();
    let arc = arc_in(&out.moves);

    // The circle is re-fit (its centre shifts) so the spirals land on it...
    let moved = dist(arc.origin, bare_origin);
    assert!(
        moved > 1e-6,
        "circle re-fit should move the centre, moved {moved}"
    );
    // ...but only within the same junction-deviation budget the bare arc honours.
    assert!(moved < delta, "centre moved {moved} past budget {delta}");

    // Every interior vertex still lies within budget of the moved circle.
    let verts = arc_vertices(3.0, 12, PI / 2.0);
    for v in &verts[2..11] {
        let off = (dist(*v, arc.origin) - arc.radius).abs();
        assert!(
            off <= delta,
            "interior vertex {off} off the moved circle, budget {delta}"
        );
    }
}

#[test]
fn eased_run_conserves_extrusion() {
    let moves = faceted_arc_with_leads(3.0, 12, PI / 2.0, 4.0, 0.42);
    assert!(
        has_clothoid(&fit_chain(&moves, cfg()).unwrap().moves),
        "precondition: this geometry is eased",
    );
    let before = total_extrusion(&moves);
    let out = fit_chain(&moves, cfg()).unwrap();
    let after = total_extrusion(&out.moves);
    // Each spiral carries exactly the filament of the line length it consumed, so
    // trimming the neighbour lines loses nothing.
    assert!(
        (before - after).abs() < 1e-9,
        "before {before} after {after}"
    );
}

#[test]
fn reverse_clothoid_retraces_the_curve() {
    use crate::path::Clothoid;
    let c = Clothoid::try_new(
        [1.0, 2.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        0.1,
        0.05,
        2.0,
    )
    .unwrap();
    let r = super::reverse_clothoid(&c).expect("reversible");
    let l = c.s_len();
    for k in 0..=8 {
        let s = l * k as f64 / 8.0;
        assert!(
            dist(r.point_at(s), c.point_at(l - s)) < 1e-9,
            "position at {s}"
        );
        let hr = r.heading_at(s);
        let hc = c.heading_at(l - s);
        assert!((hr[0] + hc[0]).abs() < 1e-9 && (hr[1] + hc[1]).abs() < 1e-9);
    }
}

#[test]
fn no_arc_fit_config_never_chains() {
    let moves = faceted_arc(3.0, 12, PI / 2.0, 3000.0, 20.0, 200.0, 0.0);
    let out = fit_chain(&moves, ChainFitConfig::default()).unwrap();
    assert_eq!(
        out.report.chains, 0,
        "arc fitting is off by default; even a clean faceted arc must not chain"
    );
}

#[test]
fn sharp_corners_rejected_by_angle_gate() {
    let config = ChainFitConfig::with_arc_fit(0.02, 3);
    let moves = vec![
        seg(1, 3000.0, 5.0, 200.0, [0.0, 0.0, 0.0], [0.5, 0.0, 0.0], 0.0),
        seg(2, 3000.0, 5.0, 200.0, [0.5, 0.0, 0.0], [0.5, 0.5, 0.0], 0.0),
        seg(3, 3000.0, 5.0, 200.0, [0.5, 0.5, 0.0], [0.0, 0.5, 0.0], 0.0),
    ];
    let out = fit_chain(&moves, config).unwrap();
    assert_eq!(out.report.chains, 0, "90 deg corners must not chain");
}

#[test]
fn faceted_arc_within_default_gates_reconstructs() {
    let config = ChainFitConfig::with_arc_fit(0.02, 3);
    let moves = faceted_arc(3.0, 12, PI / 2.0, 3000.0, 20.0, 200.0, 0.0);
    let out = fit_chain(&moves, config).unwrap();
    assert_eq!(out.report.chains, 1, "genuine faceting reconstructs");
}

#[test]
fn long_facets_rejected_by_bulge() {
    let config = ChainFitConfig::with_arc_fit(0.02, 3);
    let moves = faceted_arc(100.0, 4, 0.4, 3000.0, 20.0, 200.0, 0.0);
    let out = fit_chain(&moves, config).unwrap();
    assert_eq!(
        out.report.chains, 0,
        "coarse facets bulge past junction deviation, so the arc is refused"
    );
}

#[test]
fn bare_arc_seam_lands_on_the_boundary_vertices() {
    let r = 3.0;
    let span = PI / 2.0;
    let n = 12;
    let moves = faceted_arc(r, n, span, 3000.0, 20.0, 200.0, 0.0);
    let out = fit_chain(&moves, cfg()).unwrap();
    let arc = arc_in(&out.moves);

    let verts = arc_vertices(r, n, span);
    assert!(dist(arc.point_at(0.0), verts[0]) < 0.02, "arc enters at v0");
    assert!(
        dist(arc.point_at(arc.s_len()), verts[n]) < 0.02,
        "arc exits at the last vertex"
    );
}

#[test]
fn reconstructed_arc_passes_near_interior_vertices() {
    let r = 3.0;
    let span = PI / 2.0;
    let n = 16;
    let moves = faceted_arc(r, n, span, 3000.0, 20.0, 200.0, 0.0);
    let out = fit_chain(&moves, cfg()).unwrap();
    let arc = arc_in(&out.moves);

    let verts = arc_vertices(r, n, span);
    for v in &verts[2..n - 1] {
        let radial = dist(*v, arc.origin);
        assert!(
            (radial - arc.radius).abs() < 0.05,
            "vertex radial {radial} vs {}",
            arc.radius
        );
    }
}

#[test]
fn chain_fit_beats_the_per_corner_sawtooth() {
    let moves = faceted_arc(3.0, 20, PI / 2.0, 3000.0, 45.0, 200.0, 0.3);

    let chain = fit_chain(&moves, cfg()).unwrap();
    let corners = fit_corners(&moves, Default::default()).unwrap();
    assert_eq!(chain.report.chains, 1);

    let cfg = VelocityConfig::default();
    let t_chain = plan_velocity(&chain, cfg).unwrap().report.traversal_time_s;
    let t_corners = plan_velocity(&corners, cfg)
        .unwrap()
        .report
        .traversal_time_s;
    assert!(
        t_chain < t_corners,
        "chain {t_chain} should beat per-corner {t_corners}"
    );
}

#[test]
fn extrusion_is_conserved_across_the_run() {
    let moves = faceted_arc(3.0, 12, PI / 2.0, 3000.0, 20.0, 200.0, 0.42);
    let before = total_extrusion(&moves);
    let out = fit_chain(&moves, cfg()).unwrap();
    let after = total_extrusion(&out.moves);
    assert!(
        (before - after).abs() < 1e-9,
        "before {before} after {after}"
    );
}

#[test]
fn non_cocircular_run_falls_through_to_per_corner() {
    let mut p = [0.0, 0.0, 0.0];
    let mut heading = 0.0_f64;
    let mut moves = Vec::new();
    for k in 0..8u32 {
        heading += if k % 2 == 0 { 0.2 } else { 0.8 };
        let next = [p[0] + heading.cos(), p[1] + heading.sin(), 0.0];
        moves.push(seg(k + 1, 3000.0, 20.0, 200.0, p, next, 0.0));
        p = next;
    }
    let chain = fit_chain(&moves, cfg()).unwrap();
    let corners = fit_corners(&moves, Default::default()).unwrap();
    assert_eq!(
        chain.report.chains, 0,
        "sharply varying curvature is not reconstructed as one circle"
    );
    assert_eq!(chain.moves.len(), corners.moves.len());
}

#[test]
fn turn_reversal_splits_the_run() {
    let mut p = [0.0, 0.0, 0.0];
    let mut ang = 0.0_f64;
    let mut moves = Vec::new();
    for k in 0..12u32 {
        ang += if k < 6 { 0.2 } else { -0.2 };
        let next = [p[0] + ang.cos(), p[1] + ang.sin(), 0.0];
        moves.push(seg(k + 1, 3000.0, 20.0, 200.0, p, next, 0.0));
        p = next;
    }
    let out = fit_chain(&moves, cfg()).unwrap();
    assert!(out.report.chains <= 2);
}

#[test]
fn arc_move_breaks_the_run() {
    let mut moves = faceted_arc(3.0, 6, PI / 3.0, 3000.0, 20.0, 200.0, 0.0);
    let split = moves.len() / 2;
    let mut virt = moves[split].clone();
    virt.segment.spatial = None;
    virt.segment.virtual_path_mm = Some(0.5);
    moves.insert(split, virt);
    let out = fit_chain(&moves, cfg()).unwrap();
    assert!(out.report.chains <= 2);
    assert!(
        out.moves.iter().any(|m| m.segment.spatial.is_none()),
        "the interrupting virtual move must survive, never consumed by a chain"
    );
}

#[test]
fn isolated_corner_is_unchanged_from_per_corner() {
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
    let chain = fit_chain(&moves, cfg()).unwrap();
    let corners = fit_corners(&moves, Default::default()).unwrap();
    assert_eq!(chain.report.chains, 0);
    assert_eq!(chain.report.blended, corners.report.blended);
    assert_eq!(chain.moves.len(), corners.moves.len());
}

#[test]
fn short_chain_passes_through() {
    let one = vec![seg(
        1,
        3000.0,
        5.0,
        100.0,
        [0.0, 0.0, 0.0],
        [10.0, 0.0, 0.0],
        0.0,
    )];
    let out = fit_chain(&one, cfg()).unwrap();
    assert_eq!(out.moves.len(), 1);
    assert_eq!(out.report.chains, 0);
}

#[test]
fn run_velocity_profile_plans_without_error() {
    let moves = faceted_arc(3.0, 12, PI / 2.0, 3000.0, 20.0, 200.0, 0.3);
    let out = fit_chain(&moves, cfg()).unwrap();
    let profile = plan_velocity(&out, VelocityConfig::default()).unwrap();
    assert!(profile.report.traversal_time_s.is_finite() && profile.report.traversal_time_s > 0.0);
}

#[test]
fn coarse_facets_within_budget_still_reconstruct() {
    let r = 4.0;
    let n = 5;
    let span = PI / 3.0;
    let moves = faceted_arc(r, n, span, 3000.0, 20.0, 200.0, 0.0);
    let out = fit_chain(&moves, cfg()).unwrap();
    assert_eq!(out.report.chains, 1);
    let arc = arc_in(&out.moves);
    let verts = arc_vertices(r, n, span);
    let a = verts[n - 1];
    let b = verts[n];
    let d = dist(a, b);
    let last_chord = [(b[0] - a[0]) / d, (b[1] - a[1]) / d, 0.0];
    let h = arc.heading_at(arc.s_len());
    let dotp = h[0] * last_chord[0] + h[1] * last_chord[1];
    assert!(dotp > 0.99, "arc exits along the final chord, dot {dotp}");
}

#[test]
fn non_cocircular_triple_is_rejected_by_the_vertex_tube() {
    let mut p = [0.0, 0.0, 0.0];
    let mut moves = Vec::new();
    for (k, ang) in [0.0_f64, 0.25, 0.95].iter().enumerate() {
        let next = [p[0] + ang.cos(), p[1] + ang.sin(), 0.0];
        moves.push(seg(k as u32 + 1, 3000.0, 20.0, 200.0, p, next, 0.0));
        p = next;
    }
    let out = fit_chain(&moves, cfg()).unwrap();
    assert_eq!(
        out.report.chains, 0,
        "non-co-circular triple must not reconstruct"
    );
}

#[test]
fn easement_deviation_stays_within_junction_deviation() {
    let accel = 3000.0;
    let scv = 20.0;
    let delta = scv * scv * (2.0_f64.sqrt() - 1.0) / accel;
    let moves = faceted_arc_with_leads(3.0, 12, PI / 2.0, 4.0, 0.0);
    let out = fit_chain(&moves, cfg()).unwrap();
    let i = out
        .moves
        .iter()
        .position(|m| matches!(m.segment.spatial, Some(Segment::Arc(_))))
        .unwrap();
    let arc = as_arc(&out.moves[i]);
    let up = as_clothoid(&out.moves[i - 1]);
    let line = as_line(&out.moves[i - 2]);

    // The spiral replaces the line->arc corner; everywhere along it, it must hug
    // either the line (near the start) or the arc circle (near the end) to within
    // the junction-deviation budget.
    let t = line.heading_at(0.0);
    let start = line.point_at(line.s_len());
    let mut worst = 0.0_f64;
    for k in 0..=16 {
        let p = up.point_at(up.s_len() * k as f64 / 16.0);
        let d = sub(p, start);
        let along = d[0] * t[0] + d[1] * t[1] + d[2] * t[2];
        let perp = sub(d, scale(t, along));
        let off_line = (perp[0] * perp[0] + perp[1] * perp[1] + perp[2] * perp[2]).sqrt();
        let off_arc = (dist(p, arc.origin) - arc.radius).abs();
        worst = worst.max(off_line.min(off_arc));
    }
    assert!(
        worst <= delta,
        "easement bulges {worst} off the corner, past junction deviation {delta}"
    );
}
