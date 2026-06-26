use geometry::path::CurvatureProfile;
use geometry::path::Segment;
use geometry::path::lowering::PositionProfile;
use geometry::{
    ChainFitConfig, HeartKind, Move, MoveContext, SourceRange, VelocityLimits, fit_chain, line_move,
};
use proptest::prelude::*;

const ACCEL: f64 = 3000.0;
const SCV: f64 = 20.0;

fn delta() -> f64 {
    SCV * SCV * (2.0_f64.sqrt() - 1.0) / ACCEL
}

fn ctx(line_no: u32) -> MoveContext {
    MoveContext {
        extruder_axis: 3,
        feedrate_mm_s: 200.0,
        limits: VelocityLimits::try_new(300.0, ACCEL, SCV).unwrap(),
        source: SourceRange {
            start_line: line_no,
            end_line: line_no,
        },
    }
}

fn polyline_moves(steps: &[(f64, f64)]) -> (Vec<Move>, Vec<[f64; 3]>) {
    let mut heading = 0.0_f64;
    let mut pos = [0.0_f64; 3];
    let mut verts = vec![pos];
    let mut moves = Vec::new();
    let mut line_no = 1;
    for &(turn, len) in steps {
        heading += turn;
        let next = [
            pos[0] + len * heading.cos(),
            pos[1] + len * heading.sin(),
            0.0,
        ];
        if let Ok(m) = line_move(pos, next, 0.0, ctx(line_no)) {
            moves.push(m);
            verts.push(next);
            pos = next;
            line_no += 1;
        }
    }
    (moves, verts)
}

fn point_to_segment(p: [f64; 3], a: [f64; 3], b: [f64; 3]) -> f64 {
    let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let len2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2];
    let t = if len2 == 0.0 {
        0.0
    } else {
        ((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1] + (p[2] - a[2]) * ab[2]) / len2
    }
    .clamp(0.0, 1.0);
    let q = [a[0] + t * ab[0], a[1] + t * ab[1], a[2] + t * ab[2]];
    ((p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2) + (p[2] - q[2]).powi(2)).sqrt()
}

fn dist(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

fn check_invariants(
    moves: &[Move],
    verts: &[[f64; 3]],
    heart: HeartKind,
) -> Result<(), TestCaseError> {
    let cfg = ChainFitConfig {
        heart,
        ..ChainFitConfig::with_arc_fit(3)
    };
    let out = fit_chain(moves, cfg).expect("fit_chain must succeed on finite polyline input");

    let spatials: Vec<&Segment> = out
        .moves
        .iter()
        .filter_map(|m| m.segment.spatial.as_ref())
        .collect();
    prop_assert!(!spatials.is_empty());

    let mut prev_end: Option<[f64; 3]> = None;
    let mut worst_dev = 0.0_f64;
    for s in &spatials {
        let len = s.s_len();
        prop_assert!(len.is_finite() && len > 0.0);
        let start = s.point_at(0.0);
        let end = s.point_at(len);
        prop_assert!(start.iter().chain(end.iter()).all(|c| c.is_finite()));
        if let Some(p) = prev_end {
            prop_assert!(
                dist(p, start) <= delta(),
                "C0 gap exceeds the tolerance tube: {:?} -> {:?}",
                p,
                start
            );
        }
        prev_end = Some(end);
        for k in 0..=32 {
            let pt = s.point_at(len * k as f64 / 32.0);
            prop_assert!(pt.iter().all(|c| c.is_finite()));
            let near = verts
                .windows(2)
                .map(|w| point_to_segment(pt, w[0], w[1]))
                .fold(f64::INFINITY, f64::min);
            worst_dev = worst_dev.max(near);
        }
    }
    prop_assert!(
        worst_dev <= 1.5 * delta() + 1e-6,
        "out of band: {worst_dev} exceeds the reconstruct guarantee (residual + sagitta, each <= delta {})",
        delta()
    );
    Ok(())
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(256))]

    #[test]
    fn fit_is_ok_finite_c0_and_in_band(
        steps in prop::collection::vec(
            (-2.6_f64..2.6, 0.5_f64..5.0),
            2..30usize,
        )
    ) {
        let (moves, verts) = polyline_moves(&steps);
        prop_assume!(moves.len() >= 2);
        check_invariants(&moves, &verts, HeartKind::PositionGreedy)?;
        check_invariants(&moves, &verts, HeartKind::KappaSignal)?;
    }
}
