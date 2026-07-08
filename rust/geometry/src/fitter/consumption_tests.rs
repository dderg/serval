use super::*;
use crate::frontend::MoveContext;
use crate::frontend::line_move;
use crate::path::lowering::PositionProfile;
use crate::path::{Clothoid, Segment};
use crate::segment::SourceRange;
use crate::vec3::{dist, norm, sub};
use std::f64::consts::SQRT_2;

const E_AXIS: usize = 3;
const ACCEL: f64 = 3000.0;
const SCV: f64 = 9.0;

fn ctx_with(line_no: u32, accel: f64, scv: f64) -> MoveContext {
    MoveContext {
        extruder_axis: E_AXIS,
        feedrate_mm_s: 100.0,
        limits: VelocityLimits::try_new(200.0, accel, scv, 100_000.0).unwrap(),
        source: SourceRange {
            start_line: line_no,
            end_line: line_no,
        },
    }
}

fn seg_with(line_no: u32, start: [f64; 3], end: [f64; 3], ratio: f64, accel: f64) -> Move {
    let e = ratio * dist(start, end);
    line_move(start, end, e, ctx_with(line_no, accel, SCV)).unwrap()
}

fn seg(line_no: u32, start: [f64; 3], end: [f64; 3], ratio: f64) -> Move {
    seg_with(line_no, start, end, ratio, ACCEL)
}

fn delta_of(accel: f64) -> f64 {
    SCV * SCV * (SQRT_2 - 1.0) / accel
}

fn as_clothoid(m: &Move) -> &Clothoid {
    match &m.segment.spatial {
        Some(Segment::Clothoid(c)) => c,
        other => panic!("expected clothoid, got {other:?}"),
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
                .map(|f| f.delta_over(s))
                .sum::<f64>()
        })
        .sum()
}

/// A 90° corner whose vertex is replaced by a 45°-45° chamfer facet of
/// length `h`: in along +x, chamfer at 45°, out along +y.
fn chamfered_corner(h: f64, r_in: f64, r_mid: f64, r_out: f64) -> (Move, Move, Move) {
    chamfered_corner_at_accel(h, r_in, r_mid, r_out, ACCEL)
}

fn chamfered_corner_at_accel(
    h: f64,
    r_in: f64,
    r_mid: f64,
    r_out: f64,
    accel: f64,
) -> (Move, Move, Move) {
    let d = h / SQRT_2;
    let v1 = [10.0, 0.0, 0.0];
    let v2 = [10.0 + d, d, 0.0];
    (
        seg_with(1, [0.0, 0.0, 0.0], v1, r_in, accel),
        seg_with(2, v1, v2, r_mid, accel),
        seg_with(3, v2, [10.0 + d, 10.0 + d, 0.0], r_out, accel),
    )
}

/// A 90° corner rounded by `n` equal facets of length `h` each turning
/// 90°/(n+1): in along +x, out along +y, plus the polyline vertices from the
/// inbound line's end to the outbound line's start.
fn faceted_corner(n: usize, h: f64, ratios: &[f64]) -> (Move, Vec<Move>, Move, Vec<[f64; 3]>) {
    assert_eq!(ratios.len(), n);
    let step = std::f64::consts::FRAC_PI_2 / (n as f64 + 1.0);
    let mut vertices = vec![[10.0, 0.0, 0.0]];
    let mut heading = 0.0;
    for _ in 0..n {
        heading += step;
        let p = *vertices.last().unwrap();
        vertices.push([
            p[0] + h * libm::cos(heading),
            p[1] + h * libm::sin(heading),
            0.0,
        ]);
    }
    let a = seg(1, [0.0, 0.0, 0.0], vertices[0], 0.1);
    let mids: Vec<Move> = (0..n)
        .map(|i| seg(2 + i as u32, vertices[i], vertices[i + 1], ratios[i]))
        .collect();
    let last = *vertices.last().unwrap();
    let b = seg(2 + n as u32, last, [last[0], last[1] + 10.0, 0.0], 0.1);
    (a, mids, b, vertices)
}

fn dist_to_segment(p: [f64; 3], s0: [f64; 3], s1: [f64; 3]) -> f64 {
    let d = sub(s1, s0);
    let len_sq = d.iter().map(|x| x * x).sum::<f64>();
    if len_sq <= 0.0 {
        return dist(p, s0);
    }
    let t = ((0..3).map(|i| (p[i] - s0[i]) * d[i]).sum::<f64>() / len_sq).clamp(0.0, 1.0);
    dist(p, [s0[0] + t * d[0], s0[1] + t * d[1], s0[2] + t * d[2]])
}

fn max_dev_from_chain(halves: &[&Clothoid], chain: &[[f64; 3]]) -> f64 {
    let mut worst = 0.0_f64;
    for c in halves {
        for i in 0..=64 {
            let p = c.point_at(c.s_len() * f64::from(i) / 64.0);
            let d = chain
                .windows(2)
                .map(|s| dist_to_segment(p, s[0], s[1]))
                .fold(f64::INFINITY, f64::min);
            worst = worst.max(d);
        }
    }
    worst
}

fn assert_g2_chain(segments: &[&Clothoid]) {
    for w in segments.windows(2) {
        let (a, b) = (w[0], w[1]);
        assert!(
            dist(a.point_at(a.s_len()), b.point_at(0.0)) < 1e-7,
            "segments join in position"
        );
        assert!(
            norm(sub(a.heading_at(a.s_len()), b.heading_at(0.0))) < 1e-7,
            "tangent continuous at the join"
        );
        let (_, ka_end) = a.kappa_endpoints();
        let (kb_start, _) = b.kappa_endpoints();
        assert!(
            (ka_end - kb_start).abs() < 1e-6,
            "curvature continuous at the join: {ka_end} vs {kb_start}"
        );
    }
}

#[test]
fn squeezed_chamfer_is_consumed_within_tolerance() {
    let (a, mid, b) = chamfered_corner(0.05, 0.1, 0.1, 0.1);
    let fc = plan_facet_consumption(&a, &[&mid], &b, CornerFitConfig::default(), 0.0)
        .unwrap()
        .expect("squeezed chamfer must be consumed");
    assert!(fc.trim_in() > 0.0 && fc.trim_out() > 0.0);

    let moves = consumption_moves(&fc, &a, &[&mid], &b).unwrap();
    assert_eq!(moves.len(), 2, "one pair suffices within the tube");
    let segments: Vec<&Clothoid> = moves.iter().map(as_clothoid).collect();

    let line_a = match &a.segment.spatial {
        Some(Segment::Line(l)) => l,
        _ => unreachable!(),
    };
    let line_b = match &b.segment.spatial {
        Some(Segment::Line(l)) => l,
        _ => unreachable!(),
    };
    let v1 = line_a.end;
    let v2 = line_b.start;
    let contact_a = [v1[0] - fc.trim_in(), v1[1], v1[2]];
    let contact_b = [v2[0], v2[1] + fc.trim_out(), v2[2]];
    let first = segments[0];
    let last = segments[segments.len() - 1];
    assert!(
        dist(first.point_at(0.0), contact_a) < 1e-9,
        "starts on line a"
    );
    assert!(
        dist(last.point_at(last.s_len()), contact_b) < 1e-7,
        "ends on line b"
    );

    let (k_start, _) = first.kappa_endpoints();
    let (_, k_end) = last.kappa_endpoints();
    assert!(
        k_start.abs() < 1e-9 && k_end.abs() < 1e-9,
        "kappa-free contacts"
    );
    assert_g2_chain(&segments);

    let dev = max_dev_from_chain(&segments, &[contact_a, v1, v2, contact_b]);
    assert!(
        dev <= delta_of(ACCEL) + 1e-9,
        "deviation {dev} exceeds tolerance {}",
        delta_of(ACCEL)
    );

    let expected_e = 0.1 * (fc.trim_in() + mid.segment.s_len() + fc.trim_out());
    let got_e = total_extrusion(&moves);
    assert!(
        (got_e - expected_e).abs() < 1e-9,
        "blend must carry the replaced spans' extrusion: {got_e} vs {expected_e}"
    );
}

#[test]
fn chamfer_beyond_one_pair_is_consumed_by_a_split_blend() {
    // At this acceleration the junction deviation is ~2.1µm: the 50µm
    // chamfer is far outside what a single curvature bump can hug (it
    // strays roughly the chamfer depth from the tube), but the squeeze gate
    // still holds — its corners' deviation-optimal trims dwarf the facet.
    // The solver must split and hug the chamfer with two pairs.
    let accel = 5000.0;
    let (a, mid, b) = chamfered_corner_at_accel(0.05, 0.1, 0.1, 0.1, accel);
    let fc = plan_facet_consumption(&a, &[&mid], &b, CornerFitConfig::default(), 0.0)
        .unwrap()
        .expect("the split blend must rescue the wide chamfer");

    let moves = consumption_moves(&fc, &a, &[&mid], &b).unwrap();
    assert_eq!(moves.len(), 4, "two pairs hug the chamfer's two corners");
    let segments: Vec<&Clothoid> = moves.iter().map(as_clothoid).collect();
    assert_g2_chain(&segments);

    let line_a = match &a.segment.spatial {
        Some(Segment::Line(l)) => l,
        _ => unreachable!(),
    };
    let line_b = match &b.segment.spatial {
        Some(Segment::Line(l)) => l,
        _ => unreachable!(),
    };
    let v1 = line_a.end;
    let v2 = line_b.start;
    let contact_a = [v1[0] - fc.trim_in(), v1[1], v1[2]];
    let contact_b = [v2[0], v2[1] + fc.trim_out(), v2[2]];
    let dev = max_dev_from_chain(&segments, &[contact_a, v1, v2, contact_b]);
    assert!(
        dev <= delta_of(accel) + 1e-9,
        "deviation {dev} exceeds tolerance {}",
        delta_of(accel)
    );

    let expected_e = 0.1 * (fc.trim_in() + mid.segment.s_len() + fc.trim_out());
    let got_e = total_extrusion(&moves);
    assert!(
        (got_e - expected_e).abs() < 1e-9,
        "blend must carry the replaced spans' extrusion: {got_e} vs {expected_e}"
    );
}

#[test]
fn facet_cluster_is_consumed_by_one_blend() {
    let (a, mids, b, vertices) = faceted_corner(3, 0.02, &[0.1, 0.1, 0.1]);
    let mid_refs: Vec<&Move> = mids.iter().collect();
    let fc = plan_facet_consumption(&a, &mid_refs, &b, CornerFitConfig::default(), 0.0)
        .unwrap()
        .expect("the squeezed facet cluster must be consumed");

    let moves = consumption_moves(&fc, &a, &mid_refs, &b).unwrap();
    let segments: Vec<&Clothoid> = moves.iter().map(as_clothoid).collect();
    assert_g2_chain(&segments);

    let mut chain = Vec::with_capacity(vertices.len() + 2);
    let first = vertices[0];
    let last = *vertices.last().unwrap();
    chain.push([first[0] - fc.trim_in(), first[1], first[2]]);
    chain.extend_from_slice(&vertices);
    chain.push([last[0], last[1] + fc.trim_out(), last[2]]);
    let dev = max_dev_from_chain(&segments, &chain);
    assert!(
        dev <= delta_of(ACCEL) + 1e-9,
        "deviation {dev} exceeds tolerance {}",
        delta_of(ACCEL)
    );

    let consumed_len: f64 = mids.iter().map(|m| m.segment.s_len()).sum();
    let expected_e = 0.1 * (fc.trim_in() + consumed_len + fc.trim_out());
    let got_e = total_extrusion(&moves);
    assert!(
        (got_e - expected_e).abs() < 1e-9,
        "blend must carry the replaced spans' extrusion: {got_e} vs {expected_e}"
    );
}

#[test]
fn cluster_with_a_travel_facet_is_not_consumed() {
    let (a, mids, b, _) = faceted_corner(3, 0.02, &[0.1, 0.0, 0.1]);
    let mid_refs: Vec<&Move> = mids.iter().collect();
    let fc = plan_facet_consumption(&a, &mid_refs, &b, CornerFitConfig::default(), 0.0).unwrap();
    assert!(
        fc.is_none(),
        "a travel facet inside an extruding cluster stays a sharp boundary"
    );
}

#[test]
fn travel_facet_between_extrusions_is_not_consumed() {
    let (a, mid, b) = chamfered_corner(0.05, 0.1, 0.0, 0.1);
    let fc = plan_facet_consumption(&a, &[&mid], &b, CornerFitConfig::default(), 0.0).unwrap();
    assert!(fc.is_none(), "a travel gap between extrusions stays sharp");
}

#[test]
fn extruding_facet_between_travels_is_not_consumed() {
    let (a, mid, b) = chamfered_corner(0.05, 0.0, 0.1, 0.0);
    let fc = plan_facet_consumption(&a, &[&mid], &b, CornerFitConfig::default(), 0.0).unwrap();
    assert!(
        fc.is_none(),
        "an extruding blip between travels stays sharp"
    );
}

#[test]
fn travel_facet_between_travels_is_consumed() {
    let (a, mid, b) = chamfered_corner(0.05, 0.0, 0.0, 0.0);
    let fc = plan_facet_consumption(&a, &[&mid], &b, CornerFitConfig::default(), 0.0).unwrap();
    assert!(fc.is_some(), "an all-travel chamfer may be consumed");
}

#[test]
fn s_jog_is_not_consumed() {
    let d = 0.05 / SQRT_2;
    let v1 = [10.0, 0.0, 0.0];
    let v2 = [10.0 + d, d, 0.0];
    let a = seg(1, [0.0, 0.0, 0.0], v1, 0.1);
    let mid = seg(2, v1, v2, 0.1);
    let b = seg(3, v2, [20.0 + d, d, 0.0], 0.1);
    let fc = plan_facet_consumption(&a, &[&mid], &b, CornerFitConfig::default(), 0.0).unwrap();
    assert!(
        fc.is_none(),
        "opposite-turn jog cannot be one same-sign blend"
    );
}

#[test]
fn roomy_facet_is_not_consumed() {
    let (a, mid, b) = chamfered_corner(5.0, 0.1, 0.1, 0.1);
    let fc = plan_facet_consumption(&a, &[&mid], &b, CornerFitConfig::default(), 0.0).unwrap();
    assert!(
        fc.is_none(),
        "a facet with room for its own blends keeps them"
    );
}
