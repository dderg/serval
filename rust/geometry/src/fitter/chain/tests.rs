use crate::fitter::{ChainFitConfig, fit_chain, fit_corners};
use crate::frontend::{Move, MoveContext, VelocityLimits, line_move};
use crate::path::lowering::PositionProfile;
use crate::path::{Arc, Clothoid, CurvatureProfile, Line, Segment};
use crate::segment::SourceRange;
use crate::velocity::{VelocityConfig, plan_velocity};
use std::f64::consts::PI;

const E_AXIS: usize = 3;

fn cfg() -> ChainFitConfig {
    ChainFitConfig::with_arc_fit(f64::INFINITY, PI)
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

fn find_recon(moves: &[Move]) -> (usize, &Clothoid, &Arc, &Clothoid) {
    let i = moves
        .iter()
        .position(|m| matches!(m.segment.spatial, Some(Segment::Arc(_))))
        .expect("reconstructed arc present");
    (
        i,
        as_clothoid(&moves[i - 1]),
        as_arc(&moves[i]),
        as_clothoid(&moves[i + 1]),
    )
}

#[test]
fn faceted_arc_reconstructs_to_clothoid_arc_clothoid() {
    let moves = faceted_arc(3.0, 12, PI / 2.0, 3000.0, 20.0, 200.0, 0.5);
    let out = fit_chain(&moves, cfg()).unwrap();

    assert_eq!(out.report.chains, 1, "one faceted-arc run reconstructed");
    assert!(
        out.report.unblended.is_empty(),
        "no stop pinned inside the run"
    );
    let (_, up, arc, down) = find_recon(&out.moves);
    assert!(arc.radius > 0.0 && arc.radius.is_finite());
    assert!(up.kappa(0.0).abs() < 1e-9);
    assert!(down.kappa(down.s_len()).abs() < 1e-9);
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
    let config = ChainFitConfig::with_arc_fit(1.0, 12f64.to_radians());
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
    let config = ChainFitConfig::with_arc_fit(1.0, 12f64.to_radians());
    let moves = faceted_arc(3.0, 12, PI / 2.0, 3000.0, 20.0, 200.0, 0.0);
    let out = fit_chain(&moves, config).unwrap();
    assert_eq!(out.report.chains, 1, "genuine faceting reconstructs");
}

#[test]
fn long_facets_rejected_by_length_gate() {
    let config = ChainFitConfig::with_arc_fit(1.0, 60f64.to_radians());
    let moves = faceted_arc(100.0, 4, 0.4, 3000.0, 20.0, 200.0, 0.0);
    let out = fit_chain(&moves, config).unwrap();
    assert_eq!(out.report.chains, 0, "long facets must not chain");
}

#[test]
fn reconstruction_is_g2_and_seam_exact() {
    let moves = faceted_arc(3.0, 12, PI / 2.0, 3000.0, 20.0, 200.0, 0.0);
    let out = fit_chain(&moves, cfg()).unwrap();
    let (i, up, arc, down) = find_recon(&out.moves);
    let kappa_arc = arc.kappa(0.0);

    assert!((up.kappa(up.s_len()) - kappa_arc).abs() < 1e-9);
    assert!((down.kappa(0.0) - kappa_arc).abs() < 1e-9);

    let line_in = as_line(&out.moves[i - 2]);
    let line_out = as_line(&out.moves[i + 2]);
    assert!(dist(line_in.point_at(line_in.s_len()), up.point_at(0.0)) < 1e-6);
    assert!(dist(up.point_at(up.s_len()), arc.point_at(0.0)) < 1e-6);
    assert!(dist(arc.point_at(arc.s_len()), down.point_at(0.0)) < 1e-6);
    assert!(dist(down.point_at(down.s_len()), line_out.point_at(0.0)) < 1e-6);

    let dotp = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    assert!(dotp(up.heading_at(up.s_len()), arc.heading_at(0.0)) > 1.0 - 1e-6);
    assert!(dotp(arc.heading_at(arc.s_len()), down.heading_at(0.0)) > 1.0 - 1e-6);
}

#[test]
fn reconstructed_arc_passes_near_interior_vertices() {
    let r = 3.0;
    let span = PI / 2.0;
    let n = 16;
    let moves = faceted_arc(r, n, span, 3000.0, 20.0, 200.0, 0.0);
    let out = fit_chain(&moves, cfg()).unwrap();
    let (_, _, arc, _) = find_recon(&out.moves);

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
        heading += 0.12 + 0.04 * k as f64;
        let next = [p[0] + heading.cos(), p[1] + heading.sin(), 0.0];
        moves.push(seg(k + 1, 3000.0, 20.0, 200.0, p, next, 0.0));
        p = next;
    }
    let chain = fit_chain(&moves, cfg()).unwrap();
    let corners = fit_corners(&moves, Default::default()).unwrap();
    assert_eq!(
        chain.report.chains, 0,
        "varying-kappa run is not reconstructed"
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
fn coarse_facets_still_reconstruct_landing_on_the_last_chord() {
    let moves = faceted_arc(4.0, 5, PI / 3.0, 3000.0, 20.0, 200.0, 0.0);
    let out = fit_chain(&moves, cfg()).unwrap();
    assert_eq!(out.report.chains, 1);
    let (_, _, _, down) = find_recon(&out.moves);
    let last_chord = as_line(out.moves.last().unwrap()).heading_at(0.0);
    let h = down.heading_at(down.s_len());
    let dotp = h[0] * last_chord[0] + h[1] * last_chord[1] + h[2] * last_chord[2];
    assert!(dotp > 1.0 - 1e-6, "down exits along the last chord heading");
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
fn transition_shift_stays_within_delta() {
    let accel = 3000.0;
    let scv = 20.0;
    let delta = scv * scv * (2.0_f64.sqrt() - 1.0) / accel;
    let moves = faceted_arc(3.0, 12, PI / 2.0, accel, scv, 200.0, 0.0);
    let out = fit_chain(&moves, cfg()).unwrap();
    let (_, up, arc, _) = find_recon(&out.moves);
    let shift = up.s_len() * up.s_len() / (24.0 * arc.radius);
    assert!(
        shift <= delta * 1.001,
        "shift {shift} exceeds delta {delta}"
    );
}
