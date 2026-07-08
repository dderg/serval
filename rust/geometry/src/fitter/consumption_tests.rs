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

fn ctx(line_no: u32) -> MoveContext {
    MoveContext {
        extruder_axis: E_AXIS,
        feedrate_mm_s: 100.0,
        limits: VelocityLimits::try_new(200.0, ACCEL, SCV, 100_000.0).unwrap(),
        source: SourceRange {
            start_line: line_no,
            end_line: line_no,
        },
    }
}

fn seg(line_no: u32, start: [f64; 3], end: [f64; 3], ratio: f64) -> Move {
    let e = ratio * dist(start, end);
    line_move(start, end, e, ctx(line_no)).unwrap()
}

fn delta() -> f64 {
    SCV * SCV * (SQRT_2 - 1.0) / ACCEL
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
    let d = h / SQRT_2;
    let v1 = [10.0, 0.0, 0.0];
    let v2 = [10.0 + d, d, 0.0];
    (
        seg(1, [0.0, 0.0, 0.0], v1, r_in),
        seg(2, v1, v2, r_mid),
        seg(3, v2, [10.0 + d, 10.0 + d, 0.0], r_out),
    )
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

#[test]
fn squeezed_chamfer_is_consumed_within_tolerance() {
    let (a, mid, b) = chamfered_corner(0.05, 0.1, 0.1, 0.1);
    let fc = plan_facet_consumption(&a, &mid, &b, CornerFitConfig::default(), 0.0)
        .unwrap()
        .expect("squeezed chamfer must be consumed");
    assert!(fc.trim_in() > 0.0 && fc.trim_out() > 0.0);

    let halves = consumption_moves(&fc, &a, &mid, &b).unwrap();
    assert_eq!(halves.len(), 2);
    let h1 = as_clothoid(&halves[0]);
    let h2 = as_clothoid(&halves[1]);

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
    assert!(dist(h1.point_at(0.0), contact_a) < 1e-9, "starts on line a");
    assert!(
        dist(h2.point_at(h2.s_len()), contact_b) < 1e-7,
        "ends on line b"
    );

    let (k1s, k1e) = h1.kappa_endpoints();
    let (k2s, k2e) = h2.kappa_endpoints();
    assert!(k1s.abs() < 1e-9 && k2e.abs() < 1e-9, "kappa-free contacts");
    assert!((k1e - k2s).abs() < 1e-7, "curvature continuous at the join");
    assert!(
        dist(h1.point_at(h1.s_len()), h2.point_at(0.0)) < 1e-7,
        "halves join"
    );
    assert!(
        norm(sub(h1.heading_at(h1.s_len()), h2.heading_at(0.0))) < 1e-7,
        "tangent continuous at the join"
    );

    let dev = max_dev_from_chain(&[h1, h2], &[contact_a, v1, v2, contact_b]);
    assert!(
        dev <= delta() + 1e-9,
        "deviation {dev} exceeds tolerance {}",
        delta()
    );

    let expected_e = 0.1 * (fc.trim_in() + mid.segment.s_len() + fc.trim_out());
    let got_e = total_extrusion(&halves);
    assert!(
        (got_e - expected_e).abs() < 1e-9,
        "blend must carry the replaced spans' extrusion: {got_e} vs {expected_e}"
    );
}

#[test]
fn travel_facet_between_extrusions_is_not_consumed() {
    let (a, mid, b) = chamfered_corner(0.05, 0.1, 0.0, 0.1);
    let fc = plan_facet_consumption(&a, &mid, &b, CornerFitConfig::default(), 0.0).unwrap();
    assert!(fc.is_none(), "a travel gap between extrusions stays sharp");
}

#[test]
fn extruding_facet_between_travels_is_not_consumed() {
    let (a, mid, b) = chamfered_corner(0.05, 0.0, 0.1, 0.0);
    let fc = plan_facet_consumption(&a, &mid, &b, CornerFitConfig::default(), 0.0).unwrap();
    assert!(
        fc.is_none(),
        "an extruding blip between travels stays sharp"
    );
}

#[test]
fn travel_facet_between_travels_is_consumed() {
    let (a, mid, b) = chamfered_corner(0.05, 0.0, 0.0, 0.0);
    let fc = plan_facet_consumption(&a, &mid, &b, CornerFitConfig::default(), 0.0).unwrap();
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
    let fc = plan_facet_consumption(&a, &mid, &b, CornerFitConfig::default(), 0.0).unwrap();
    assert!(
        fc.is_none(),
        "opposite-turn jog cannot be one clothoid pair"
    );
}

#[test]
fn roomy_facet_is_not_consumed() {
    let (a, mid, b) = chamfered_corner(5.0, 0.1, 0.1, 0.1);
    let fc = plan_facet_consumption(&a, &mid, &b, CornerFitConfig::default(), 0.0).unwrap();
    assert!(
        fc.is_none(),
        "a facet with room for its own blends keeps them"
    );
}
