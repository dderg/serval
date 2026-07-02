use super::*;
use crate::frontend::{MoveContext, line_move};
use crate::path::lowering::PositionProfile;
use crate::path::{Clothoid, CurvatureProfile};
use crate::segment::SourceRange;
use crate::vec3::dist;
use std::f64::consts::{PI, SQRT_2};

const E_AXIS: usize = 3;

fn ctx(line_no: u32, accel: f64, scv: f64) -> MoveContext {
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

fn seg(line_no: u32, accel: f64, scv: f64, start: [f64; 3], end: [f64; 3], e: f64) -> Move {
    line_move(start, end, e, ctx(line_no, accel, scv)).unwrap()
}

fn delta_of(accel: f64, scv: f64) -> f64 {
    scv * scv * (SQRT_2 - 1.0) / accel
}

fn as_clothoid(m: &Move) -> &Clothoid {
    match &m.segment.spatial {
        Some(Segment::Clothoid(c)) => c,
        other => panic!("expected clothoid, got {other:?}"),
    }
}

fn as_line(m: &Move) -> &Line {
    match &m.segment.spatial {
        Some(Segment::Line(l)) => l,
        other => panic!("expected line, got {other:?}"),
    }
}

fn approx3(a: [f64; 3], b: [f64; 3], tol: f64) -> bool {
    (0..3).all(|i| (a[i] - b[i]).abs() <= tol)
}

fn total_extrusion(moves: &[Move], axis: usize) -> f64 {
    moves
        .iter()
        .map(|m| {
            let s = m.segment.s_len();
            m.segment
                .followers
                .iter()
                .filter(|f| f.axis_index == axis)
                .map(|f| f.ratio * s)
                .sum::<f64>()
        })
        .sum()
}

fn right_angle_corner(accel: f64, scv: f64, leg: f64, e: f64) -> Vec<Move> {
    vec![
        seg(1, accel, scv, [0.0, 0.0, 0.0], [leg, 0.0, 0.0], e),
        seg(2, accel, scv, [leg, 0.0, 0.0], [leg, leg, 0.0], e),
    ]
}

#[test]
fn right_angle_corner_expands_to_trimmed_lines_and_two_clothoids() {
    let moves = right_angle_corner(3000.0, 5.0, 50.0, 5.0);
    let out = fit_corners(&moves, CornerFitConfig::default()).unwrap();

    assert_eq!(out.report.blended, 1);
    assert!(out.report.unblended.is_empty());
    assert_eq!(out.moves.len(), 4);
    as_line(&out.moves[0]);
    as_clothoid(&out.moves[1]);
    as_clothoid(&out.moves[2]);
    as_line(&out.moves[3]);
}

#[test]
fn blend_is_position_and_heading_continuous() {
    let moves = right_angle_corner(3000.0, 5.0, 50.0, 0.0);
    let out = fit_corners(&moves, CornerFitConfig::default()).unwrap();

    let line_a = as_line(&out.moves[0]);
    let half1 = as_clothoid(&out.moves[1]);
    let half2 = as_clothoid(&out.moves[2]);
    let line_b = as_line(&out.moves[3]);
    let l1 = half1.s_len();
    let l2 = half2.s_len();

    let tol = 1e-6;
    assert!(approx3(line_a.end, half1.point_at(0.0), tol));
    assert!(approx3(half1.point_at(l1), half2.point_at(0.0), tol));
    assert!(approx3(half2.point_at(l2), line_b.start, tol));

    assert!(approx3(line_a.heading_at(0.0), half1.heading_at(0.0), tol));
    assert!(approx3(half1.heading_at(l1), half2.heading_at(0.0), tol));
    assert!(approx3(half2.heading_at(l2), line_b.heading_at(0.0), tol));
}

#[test]
fn blend_is_curvature_continuous() {
    let moves = right_angle_corner(3000.0, 5.0, 50.0, 0.0);
    let out = fit_corners(&moves, CornerFitConfig::default()).unwrap();

    let half1 = as_clothoid(&out.moves[1]);
    let half2 = as_clothoid(&out.moves[2]);
    let (k1_0, k1_l) = half1.kappa_endpoints();
    let (k2_0, k2_l) = half2.kappa_endpoints();

    let tol = 1e-9;
    assert!(k1_0.abs() <= tol, "incoming line meets clothoid at kappa 0");
    assert!((k1_l - k2_0).abs() <= 1e-7, "apex curvature continuous");
    assert!(
        k2_l.abs() <= 1e-7,
        "outgoing line meets clothoid at kappa 0"
    );
    assert!(k1_l > 0.0, "apex curvature is positive");
}

#[test]
fn blend_deviation_equals_delta_when_budget_is_slack() {
    let accel = 3000.0;
    let scv = 5.0;
    let moves = right_angle_corner(accel, scv, 50.0, 0.0);
    let out = fit_corners(&moves, CornerFitConfig::default()).unwrap();

    let vertex = [50.0, 0.0, 0.0];
    let half1 = as_clothoid(&out.moves[1]);
    let apex = half1.point_at(half1.s_len());
    let deviation = dist(vertex, apex);
    let delta = delta_of(accel, scv);

    assert!(
        (deviation - delta).abs() <= 1e-6 * delta,
        "deviation {deviation} should equal delta {delta} when the budget is slack"
    );
}

#[test]
fn blend_deviation_stays_within_delta_at_a_shallow_corner() {
    let accel = 2000.0;
    let scv = 8.0;
    let theta = PI / 6.0;
    let leg = 80.0;
    let mid = [leg, 0.0, 0.0];
    let end = [leg + leg * theta.cos(), leg * theta.sin(), 0.0];
    let moves = vec![
        seg(1, accel, scv, [0.0, 0.0, 0.0], mid, 0.0),
        seg(2, accel, scv, mid, end, 0.0),
    ];
    let out = fit_corners(&moves, CornerFitConfig::default()).unwrap();

    let half1 = as_clothoid(&out.moves[1]);
    let deviation = dist(mid, half1.point_at(half1.s_len()));
    let delta = delta_of(accel, scv);
    assert!(
        deviation <= delta + 1e-9,
        "deviation {deviation} delta {delta}"
    );
}

#[test]
fn collinear_junction_passes_through() {
    let moves = vec![
        seg(1, 3000.0, 5.0, [0.0, 0.0, 0.0], [10.0, 0.0, 0.0], 0.0),
        seg(2, 3000.0, 5.0, [10.0, 0.0, 0.0], [20.0, 0.0, 0.0], 0.0),
    ];
    let out = fit_corners(&moves, CornerFitConfig::default()).unwrap();

    assert_eq!(out.report.blended, 0);
    assert_eq!(out.moves.len(), 2);
    assert_eq!(out.moves, moves);
    assert_eq!(out.report.unblended[0].reason, UnblendReason::Collinear);
}

#[test]
fn near_reversal_is_left_sharp() {
    let moves = vec![
        seg(1, 3000.0, 5.0, [0.0, 0.0, 0.0], [10.0, 0.0, 0.0], 0.0),
        seg(2, 3000.0, 5.0, [10.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0),
    ];
    let out = fit_corners(&moves, CornerFitConfig::default()).unwrap();

    assert_eq!(out.report.blended, 0);
    assert_eq!(out.report.unblended[0].reason, UnblendReason::NearReversal);
}

#[test]
fn zero_square_corner_velocity_is_left_sharp() {
    let moves = right_angle_corner(3000.0, 0.0, 50.0, 0.0);
    let out = fit_corners(&moves, CornerFitConfig::default()).unwrap();

    assert_eq!(out.report.blended, 0);
    assert_eq!(out.report.unblended[0].reason, UnblendReason::ZeroDeviation);
}

#[test]
fn short_leg_tightens_blend_below_delta() {
    let accel = 1000.0;
    let scv = 10.0;
    let leg = 0.05;
    let moves = right_angle_corner(accel, scv, leg, 0.0);
    let out = fit_corners(&moves, CornerFitConfig::default()).unwrap();

    assert_eq!(out.report.blended, 1, "report: {:?}", out.report);
    let line_a = as_line(&out.moves[0]);
    let trim = leg - line_a.s_len();
    let budget = 0.5 * leg;
    assert!(
        (trim - budget).abs() <= 1e-9,
        "budget-bound blend consumes the full half-leg: trim {trim} budget {budget}"
    );

    let vertex = [leg, 0.0, 0.0];
    let half1 = as_clothoid(&out.moves[1]);
    let deviation = dist(vertex, half1.point_at(half1.s_len()));
    assert!(deviation < delta_of(accel, scv));
}

#[test]
fn vanishing_leg_leaves_the_corner_sharp() {
    let leg = 2e-9;
    let moves = right_angle_corner(1000.0, 10.0, leg, 0.0);
    let out = fit_corners(&moves, CornerFitConfig::default()).unwrap();

    assert_eq!(out.report.blended, 0);
    assert_eq!(out.report.unblended[0].reason, UnblendReason::NoBudget);
}

#[test]
fn arc_incident_junction_is_left_sharp() {
    let line = seg(1, 3000.0, 5.0, [0.0, 0.0, 0.0], [10.0, 0.0, 0.0], 0.0);
    let arc = crate::frontend::arc_move(
        [10.0, 0.0, 0.0],
        [15.0, 5.0, 0.0],
        0.0,
        5.0,
        true,
        0.0,
        ctx(2, 3000.0, 5.0),
    )
    .unwrap();
    let moves = vec![line, arc];
    let out = fit_corners(&moves, CornerFitConfig::default()).unwrap();

    assert_eq!(out.report.blended, 0);
    assert_eq!(out.moves.len(), 2);
    assert_eq!(out.report.unblended[0].reason, UnblendReason::ArcIncident);
}

#[test]
fn virtual_move_breaks_the_spatial_chain() {
    let retraction = line_move(
        [10.0, 0.0, 0.0],
        [10.0, 0.0, 0.0],
        -2.0,
        ctx(2, 3000.0, 5.0),
    )
    .unwrap();
    let moves = vec![
        seg(1, 3000.0, 5.0, [0.0, 0.0, 0.0], [10.0, 0.0, 0.0], 1.0),
        retraction,
        seg(3, 3000.0, 5.0, [10.0, 0.0, 0.0], [10.0, 10.0, 0.0], 1.0),
    ];
    let out = fit_corners(&moves, CornerFitConfig::default()).unwrap();

    assert_eq!(out.report.blended, 0);
    assert_eq!(out.moves, moves);
    assert!(
        out.report
            .unblended
            .iter()
            .all(|j| j.reason == UnblendReason::NonSpatial)
    );
}

#[test]
fn extrusion_is_conserved_across_a_blend() {
    let moves = right_angle_corner(3000.0, 5.0, 50.0, 5.0);
    let before = total_extrusion(&moves, E_AXIS);
    let out = fit_corners(&moves, CornerFitConfig::default()).unwrap();
    let after = total_extrusion(&out.moves, E_AXIS);

    assert_eq!(out.report.blended, 1);
    assert!(
        (before - after).abs() <= 1e-9,
        "before {before} after {after}"
    );
}

#[test]
fn extrusion_conserved_when_short_leg_is_consumed_by_two_blends() {
    let accel = 1000.0;
    let scv = 30.0;
    let d = 0.05 / SQRT_2;
    let moves = vec![
        seg(1, accel, scv, [-10.0, 0.0, 0.0], [0.0, 0.0, 0.0], 10.0),
        seg(2, accel, scv, [0.0, 0.0, 0.0], [d, d, 0.0], 0.05),
        seg(3, accel, scv, [d, d, 0.0], [d, d + 10.0, 0.0], 10.0),
    ];
    let before = total_extrusion(&moves, E_AXIS);
    let out = fit_corners(&moves, CornerFitConfig::default()).unwrap();
    let after = total_extrusion(&out.moves, E_AXIS);

    assert_eq!(out.report.blended, 2);
    assert_eq!(out.report.consumed_legs, 1, "report: {:?}", out.report);
    assert!(
        (before - after).abs() <= 1e-9,
        "before {before} after {after}"
    );
}

#[test]
fn non_finite_line_yields_fit_error_with_source_line() {
    let good = seg(1, 3000.0, 5.0, [0.0, 0.0, 0.0], [10.0, 0.0, 0.0], 0.0);
    let bad_line = Line::try_new([10.0, 0.0, 0.0], [f64::NAN, 5.0, 0.0]).unwrap();
    let bad_segment = PathSegment::try_new(Segment::Line(bad_line), Vec::new()).unwrap();
    let bad = Move {
        segment: bad_segment,
        feedrate_mm_s: 100.0,
        limits: VelocityLimits::try_new(200.0, 3000.0, 5.0, 100_000.0).unwrap(),
        source: SourceRange {
            start_line: 2,
            end_line: 2,
        },
    };

    let err = fit_corners(&[good, bad], CornerFitConfig::default()).unwrap_err();
    let FitError::Internal { line_no, source } = err;
    assert_eq!(line_no, 2);
    assert!(matches!(source, GeometryError::DegenerateClothoid { .. }));
}

#[test]
fn travel_corner_blends_without_followers() {
    let moves = right_angle_corner(3000.0, 5.0, 50.0, 0.0);
    let out = fit_corners(&moves, CornerFitConfig::default()).unwrap();

    assert_eq!(out.report.blended, 1);
    assert!(out.moves[1].segment.followers.is_empty());
    assert!(out.moves[2].segment.followers.is_empty());
}

#[test]
fn single_move_is_returned_unchanged() {
    let moves = vec![seg(1, 3000.0, 5.0, [0.0, 0.0, 0.0], [10.0, 0.0, 0.0], 0.0)];
    let out = fit_corners(&moves, CornerFitConfig::default()).unwrap();
    assert_eq!(out.moves, moves);
    assert_eq!(out.report.blended, 0);
}

#[test]
fn empty_input_is_returned_unchanged() {
    let out = fit_corners(&[], CornerFitConfig::default()).unwrap();
    assert!(out.moves.is_empty());
    assert_eq!(out.report, FitReport::default());
}
