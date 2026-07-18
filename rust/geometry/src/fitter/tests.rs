use super::*;
use crate::frontend::{MoveContext, line_move};
use crate::path::lowering::PositionProfile;
use crate::path::{Arc, Clothoid, CurvatureProfile, PathSegment, Segment};
use crate::segment::SourceRange;
use crate::vec3::dist;
use std::f64::consts::{PI, SQRT_2};

const E_AXIS: usize = 3;

fn ctx(line_no: u32, accel: f64, scv: f64) -> MoveContext {
    MoveContext {
        extruder_axis: E_AXIS,
        feedrate_mm_s: 100.0,
        limits: VelocityLimits::try_new(
            200.0,
            accel,
            crate::corner_deviation_from_scv(scv, accel),
            100_000.0,
        )
        .unwrap(),
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
                .map(|f| f.delta_over(s))
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
    let end = [leg + leg * libm::cos(theta), leg * libm::sin(theta), 0.0];
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
    let arc_ctx = ctx(2, 3000.0, 5.0);
    let quarter_arc = Arc::try_new(
        [10.0, 5.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        5.0,
        -std::f64::consts::FRAC_PI_2,
        std::f64::consts::FRAC_PI_2,
    )
    .unwrap();
    let arc = Move {
        segment: PathSegment::try_new(Segment::Arc(quarter_arc), Vec::new()).unwrap(),
        feedrate_mm_s: arc_ctx.feedrate_mm_s,
        limits: arc_ctx.limits,
        source: arc_ctx.source,
    };
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

fn e_follower(m: &Move) -> FollowerDemand {
    *m.segment
        .followers
        .iter()
        .find(|f| f.axis_index == E_AXIS)
        .expect("extruder follower present")
}

#[test]
fn modest_ratio_step_blends_with_a_continuous_extrusion_ramp() {
    // Two legs at slightly different extrusion ratios (0.10 vs 0.12, a 16.7%
    // relative step, under the 25% gate). The corner blends, total E is
    // conserved, and the two clothoid halves meet at one shared ratio so ė is
    // continuous across the blend midpoint.
    let moves = vec![
        seg(1, 3000.0, 5.0, [0.0, 0.0, 0.0], [50.0, 0.0, 0.0], 5.0),
        seg(2, 3000.0, 5.0, [50.0, 0.0, 0.0], [50.0, 50.0, 0.0], 6.0),
    ];
    let before = total_extrusion(&moves, E_AXIS);
    let out = fit_corners(&moves, CornerFitConfig::default()).unwrap();
    assert_eq!(out.report.blended, 1);
    assert!(out.report.unblended.is_empty());
    let after = total_extrusion(&out.moves, E_AXIS);
    assert!(
        (before - after).abs() <= 1e-9,
        "before {before} after {after}"
    );

    let half1 = e_follower(&out.moves[1]);
    let half2 = e_follower(&out.moves[2]);
    assert!(half1.is_ramped() && half2.is_ramped());
    assert!(
        (half1.ratio_end - half2.ratio).abs() <= 1e-12,
        "midpoint ratio discontinuous: {} vs {}",
        half1.ratio_end,
        half2.ratio
    );
}

#[test]
fn abrupt_ratio_step_is_left_unblended_as_a_stop() {
    // 0.10 vs 0.30 — a 66% relative step, above the gate. The corner stays sharp
    // (unblended), which forces the planner to a full stop across the seam.
    let moves = vec![
        seg(1, 3000.0, 5.0, [0.0, 0.0, 0.0], [50.0, 0.0, 0.0], 5.0),
        seg(2, 3000.0, 5.0, [50.0, 0.0, 0.0], [50.0, 50.0, 0.0], 15.0),
    ];
    let out = fit_corners(&moves, CornerFitConfig::default()).unwrap();
    assert_eq!(out.report.blended, 0);
    assert_eq!(out.report.unblended.len(), 1);
    assert_eq!(out.report.unblended[0].reason, UnblendReason::ExtrusionStep);
}

#[test]
fn travel_to_extrude_corner_still_blends_and_ramps_to_zero() {
    // One side is a travel (ratio 0). The relative step is 1.0, but a ramp to or
    // from zero extrusion is desirable, so the gate exempts it and the corner
    // blends. Total E (all on the extruding leg) is conserved.
    let moves = vec![
        seg(1, 3000.0, 5.0, [0.0, 0.0, 0.0], [50.0, 0.0, 0.0], 0.0),
        seg(2, 3000.0, 5.0, [50.0, 0.0, 0.0], [50.0, 50.0, 0.0], 5.0),
    ];
    let before = total_extrusion(&moves, E_AXIS);
    let out = fit_corners(&moves, CornerFitConfig::default()).unwrap();
    assert_eq!(out.report.blended, 1);
    let after = total_extrusion(&out.moves, E_AXIS);
    assert!(
        (before - after).abs() <= 1e-9,
        "before {before} after {after}"
    );
    let half1 = e_follower(&out.moves[1]);
    assert!(
        half1.ratio.abs() <= 1e-12,
        "half1 must start at zero extrusion, got {}",
        half1.ratio
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
fn run_whose_tail_spiral_overclaims_its_neighbor_still_touches_the_run_endpoints() {
    // Slicer polyline from neptune_cube/discontinuity.gcode around Y=105: at
    // accel 10000 / scv 20 the tail easing spiral wants 0.246mm of a 0.313mm
    // neighbor — over the half-line claim limit — which used to strand the
    // arc's tail end ~15µm off the neighbor line.
    let accel = 10_000.0;
    let scv = 20.0;
    let ratio = 0.0337;
    let mk =
        |line_no: u32, a: [f64; 3], b: [f64; 3]| seg(line_no, accel, scv, a, b, ratio * dist(a, b));
    let head = mk(1, [124.414, 103.283, 0.0], [124.414, 104.107, 0.0]);
    let facets = vec![
        mk(2, [124.414, 104.107, 0.0], [124.447, 104.481, 0.0]),
        mk(3, [124.447, 104.481, 0.0], [124.561, 104.756, 0.0]),
        mk(4, [124.561, 104.756, 0.0], [124.786, 105.022, 0.0]),
    ];
    let tail = mk(5, [124.786, 105.022, 0.0], [125.053, 105.185, 0.0]);

    let corner = CornerFitConfig::default();
    let mut fit = RunFit::fit(
        &facets,
        Some(&head),
        Some(&tail),
        CornerFitConfig::default(),
    )
    .unwrap()
    .expect("run reconstructs");
    let first = &facets[0];
    let last = facets.last().unwrap();
    let head_blend = fit.blend_head_with_line(&head, first, corner).unwrap();
    let tail_blend = fit.blend_tail_with_line(last, &tail, corner).unwrap();

    let mut chain: Vec<Move> = Vec::new();
    chain.push(
        trim_line_move(&head, 0.0, fit.head_boundary_trim())
            .unwrap()
            .expect("head neighbor keeps a body"),
    );
    chain.extend(head_blend);
    if let Some(stub) = trim_line_move(first, 0.0, fit.head_consumption()).unwrap() {
        chain.push(stub);
    }
    chain.extend(fit.pieces(first, last).unwrap());
    if let Some(stub) = trim_line_move(last, fit.tail_consumption(), 0.0).unwrap() {
        chain.push(stub);
    }
    chain.extend(tail_blend);
    chain.push(
        trim_line_move(&tail, fit.tail_boundary_trim(), 0.0)
            .unwrap()
            .expect("tail neighbor keeps a body"),
    );

    for pair in chain.windows(2) {
        let end = spatial_end(&pair[0]).unwrap();
        let start = spatial_start(&pair[1]).unwrap();
        let gap = dist(end, start);
        assert!(
            gap <= 1e-6,
            "emitted chain is discontinuous: {end:?} -> {start:?} ({gap:.9}mm gap)"
        );
    }
}

#[test]
fn displaced_spiral_keeps_extrusion_rate_continuous_and_conserves_e() {
    // Slicer polyline from neptune_cube/layer_5.gcode around (119.5, 117.7):
    // the anchored one-end easing slides the head spiral ~0.63mm along a
    // 1.27mm neighbor line while the spiral itself is only ~0.14mm long.
    // Spreading the trimmed footage's E over the spiral spiked `de/ds` 4.4×,
    // which the planner rest-anchored — a full stop mid-perimeter.
    let accel = 10_000.0;
    let scv = 20.0;
    let mk = |line_no: u32, a: [f64; 3], b: [f64; 3], e: f64| seg(line_no, accel, scv, a, b, e);
    let head = mk(1, [120.191, 116.647, 0.0], [119.505, 117.714, 0.0], 0.04268);
    let facets = vec![
        mk(2, [119.505, 117.714, 0.0], [118.91, 118.433, 0.0], 0.03139),
        mk(3, [118.91, 118.433, 0.0], [118.261, 119.065, 0.0], 0.0305),
        mk(4, [118.261, 119.065, 0.0], [117.549, 119.627, 0.0], 0.0305),
        mk(5, [117.549, 119.627, 0.0], [116.728, 120.147, 0.0], 0.03268),
    ];
    let tail = mk(6, [116.728, 120.147, 0.0], [115.574, 120.674, 0.0], 0.04268);

    let corner = CornerFitConfig::default();
    let mut fit = RunFit::fit(
        &facets,
        Some(&head),
        Some(&tail),
        CornerFitConfig::default(),
    )
    .unwrap()
    .expect("run reconstructs");
    let first = &facets[0];
    let last = facets.last().unwrap();
    let head_blend = fit.blend_head_with_line(&head, first, corner).unwrap();
    let tail_blend = fit.blend_tail_with_line(last, &tail, corner).unwrap();

    let mut chain: Vec<Move> = Vec::new();
    chain.push(
        trim_line_move(&head, 0.0, fit.head_boundary_trim())
            .unwrap()
            .expect("head neighbor keeps a body"),
    );
    chain.extend(head_blend);
    if let Some(stub) = trim_line_move(first, 0.0, fit.head_consumption()).unwrap() {
        chain.push(stub);
    }
    chain.extend(fit.pieces(first, last).unwrap());
    if let Some(stub) = trim_line_move(last, fit.tail_consumption(), 0.0).unwrap() {
        chain.push(stub);
    }
    chain.extend(tail_blend);
    chain.push(
        trim_line_move(&tail, fit.tail_boundary_trim(), 0.0)
            .unwrap()
            .expect("tail neighbor keeps a body"),
    );

    let e_rate = |m: &Move, s: f64| {
        let len = m.segment.s_len();
        m.segment
            .followers
            .iter()
            .find(|f| f.axis_index == E_AXIS)
            .map_or(0.0, |f| f.ratio_at(s, len))
    };
    for (i, pair) in chain.windows(2).enumerate() {
        let r_in = e_rate(&pair[0], pair[0].segment.s_len());
        let r_out = e_rate(&pair[1], 0.0);
        let step = (r_out - r_in).abs() / r_in.abs().max(r_out.abs());
        assert!(
            step <= 1e-9,
            "seam {i}: de/ds steps {r_in} -> {r_out} — every seam of the eased \
             construct must be exactly rate-continuous",
        );
    }

    let before = total_extrusion(&facets, E_AXIS)
        + total_extrusion(std::slice::from_ref(&head), E_AXIS)
        + total_extrusion(std::slice::from_ref(&tail), E_AXIS);
    let after = total_extrusion(&chain, E_AXIS);
    assert!(
        (before - after).abs() <= 1e-9,
        "extrusion not conserved: before {before} after {after}"
    );
}

#[test]
fn corner_ramp_beyond_extruder_budget_leaves_the_junction_unblended() {
    // A 17% extrusion-rate step blends under the default (infinite) gate; an
    // extruder accel budget below the ramp's marginal `m·v²` demand rejects
    // the very same ramp in closed form.
    let a = seg(1, 3000.0, 5.0, [0.0, 0.0, 0.0], [10.0, 0.0, 0.0], 0.5);
    let b = seg(2, 3000.0, 5.0, [10.0, 0.0, 0.0], [10.0, 10.0, 0.0], 0.6);

    let open = CornerFitConfig::default();
    assert!(matches!(
        plan_junction_reduced(&a, &b, open, 0.0, 0.0).unwrap(),
        JunctionPlan::Blend(_)
    ));

    let tight = CornerFitConfig {
        ramp_accel_budget_mm_s2: 20.0,
        ..CornerFitConfig::default()
    };
    assert!(matches!(
        plan_junction_reduced(&a, &b, tight, 0.0, 0.0).unwrap(),
        JunctionPlan::Unblended(UnblendReason::ExtrusionRampInfeasible)
    ));
}

#[test]
fn disabled_jerk_limiting_never_rejects_ramps_on_its_own() {
    // `max_jerk = ∞` (jerk limiting off) with a finite extruder budget: the
    // gate charges only the ramp's marginal `m·v²`, so nothing the G-code
    // itself commands (like the unbounded `r·j` under PA) may poison it — a
    // `0·∞ = NaN` there once rejected every ramp and left a no-jerk stream
    // with no fits at all.
    let no_jerk = |line_no: u32, start: [f64; 3], end: [f64; 3], e: f64| {
        let ctx = MoveContext {
            extruder_axis: E_AXIS,
            feedrate_mm_s: 100.0,
            limits: VelocityLimits::try_new(200.0, 3000.0, 5.0, f64::INFINITY).unwrap(),
            source: SourceRange {
                start_line: line_no,
                end_line: line_no,
            },
        };
        line_move(start, end, e, ctx).unwrap()
    };
    let a = no_jerk(1, [0.0, 0.0, 0.0], [10.0, 0.0, 0.0], 0.5);
    let b = no_jerk(2, [10.0, 0.0, 0.0], [10.0, 10.0, 0.0], 0.5);

    let config = CornerFitConfig {
        ramp_accel_budget_mm_s2: 1000.0,
        ..CornerFitConfig::default()
    };
    assert!(matches!(
        plan_junction_reduced(&a, &b, config, 0.0, 0.0).unwrap(),
        JunctionPlan::Blend(_)
    ));
}

#[test]
fn arc_ramp_beyond_extruder_budget_dissolves_the_run() {
    let accel = 10_000.0;
    let scv = 20.0;
    let mk = |line_no: u32, a: [f64; 3], b: [f64; 3], e: f64| seg(line_no, accel, scv, a, b, e);
    let head = mk(1, [120.191, 116.647, 0.0], [119.505, 117.714, 0.0], 0.04268);
    let facets = vec![
        mk(2, [119.505, 117.714, 0.0], [118.91, 118.433, 0.0], 0.03139),
        mk(3, [118.91, 118.433, 0.0], [118.261, 119.065, 0.0], 0.0305),
        mk(4, [118.261, 119.065, 0.0], [117.549, 119.627, 0.0], 0.0305),
        mk(5, [117.549, 119.627, 0.0], [116.728, 120.147, 0.0], 0.03268),
    ];
    let tail = mk(6, [116.728, 120.147, 0.0], [115.574, 120.674, 0.0], 0.04268);

    let tight = CornerFitConfig {
        ramp_accel_budget_mm_s2: 0.01,
        ..CornerFitConfig::default()
    };
    let fit = RunFit::fit(&facets, Some(&head), Some(&tail), tight).unwrap();
    assert!(
        fit.is_none(),
        "an arc ramp the extruder cannot follow must dissolve the run"
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
    let FitError::Internal { line_no, source } = err else {
        panic!("expected internal error, got {err:?}");
    };
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
fn corner_blend_deviation_is_the_raw_budget_regardless_of_accel() {
    let delta = 0.05;
    let vertex = [50.0, 0.0, 0.0];
    let deviation_at = |accel: f64| {
        let limits = VelocityLimits::try_new(200.0, accel, delta, 100_000.0).unwrap();
        let m = |line_no, start, end| {
            line_move(
                start,
                end,
                0.0,
                MoveContext {
                    extruder_axis: E_AXIS,
                    feedrate_mm_s: 100.0,
                    limits,
                    source: SourceRange {
                        start_line: line_no,
                        end_line: line_no,
                    },
                },
            )
            .unwrap()
        };
        let moves = vec![
            m(1, [0.0, 0.0, 0.0], vertex),
            m(2, vertex, [50.0, 50.0, 0.0]),
        ];
        let out = fit_corners(&moves, CornerFitConfig::default()).unwrap();
        assert_eq!(out.report.blended, 1);
        let half1 = as_clothoid(&out.moves[1]);
        dist(vertex, half1.point_at(half1.s_len()))
    };
    let slow = deviation_at(1_000.0);
    let fast = deviation_at(25_000.0);
    assert!((slow - delta).abs() <= 1e-6 * delta, "slow dev {slow}");
    assert!(
        (fast - slow).abs() <= 1e-9,
        "corner rounding must not depend on accel: {slow} vs {fast}"
    );
}

fn merge_pair(prev: &Move, next: &Move) -> Option<Move> {
    merge_collinear_lines(prev, next, &[], CornerFitConfig::default())
}

fn sub_degree_pair() -> (Move, Move) {
    // ~0.25° turn at the shared vertex, 0.2 mm facets — the slicer's
    // width-transition shape that motivates merging.
    let a = seg(1, 10_000.0, 20.0, [0.0, 0.0, 0.0], [0.2, 0.0, 0.0], 0.01);
    let b = seg(
        2,
        10_000.0,
        20.0,
        [0.2, 0.0, 0.0],
        [0.4, 0.0009, 0.0],
        0.011,
    );
    (a, b)
}

#[test]
fn merge_joins_sub_degree_extruding_facets() {
    let (a, b) = sub_degree_pair();
    let m = merge_pair(&a, &b).expect("sub-degree facets merge");
    let line = as_line(&m);
    assert!(approx3(line.start, [0.0, 0.0, 0.0], 1e-12));
    assert!(approx3(line.end, [0.4, 0.0009, 0.0], 1e-12));
    let merged_e = m.segment.followers[0].ratio * m.segment.s_len();
    assert!(
        (merged_e - 0.021).abs() < 1e-9,
        "extrusion preserved: {merged_e}"
    );
    assert_eq!(m.source.start_line, 1);
    assert_eq!(m.source.end_line, 2);
}

#[test]
fn merge_refuses_a_real_corner() {
    let a = seg(1, 10_000.0, 20.0, [0.0, 0.0, 0.0], [5.0, 0.0, 0.0], 0.1);
    let b = seg(2, 10_000.0, 20.0, [5.0, 0.0, 0.0], [5.0, 5.0, 0.0], 0.1);
    assert!(merge_pair(&a, &b).is_none());
}

#[test]
fn merge_refuses_a_shallow_turn_between_long_lines() {
    // 0.5° is under the turn cap, but the vertex of two 20 mm legs sits far
    // outside the corner-deviation budget.
    let theta = 0.5 * PI / 180.0;
    let a = seg(1, 10_000.0, 20.0, [0.0, 0.0, 0.0], [20.0, 0.0, 0.0], 0.1);
    let end = [20.0 + 20.0 * libm::cos(theta), 20.0 * libm::sin(theta), 0.0];
    let b = seg(2, 10_000.0, 20.0, [20.0, 0.0, 0.0], end, 0.1);
    assert!(merge_pair(&a, &b).is_none());
}

#[test]
fn merge_refuses_a_feedrate_step_outside_the_band() {
    let (a, mut b) = sub_degree_pair();
    b.feedrate_mm_s = a.feedrate_mm_s * 1.5;
    assert!(merge_pair(&a, &b).is_none());
}

#[test]
fn merge_takes_the_slower_feedrate() {
    let (a, mut b) = sub_degree_pair();
    b.feedrate_mm_s = a.feedrate_mm_s * 0.95;
    let m = merge_pair(&a, &b).expect("within the feedrate band");
    assert_eq!(m.feedrate_mm_s, b.feedrate_mm_s);
}

#[test]
fn merge_refuses_an_extrusion_ratio_step() {
    let a = seg(1, 10_000.0, 20.0, [0.0, 0.0, 0.0], [0.2, 0.0, 0.0], 0.01);
    let b = seg(2, 10_000.0, 20.0, [0.2, 0.0, 0.0], [0.4, 0.0, 0.0], 0.02);
    assert!(merge_pair(&a, &b).is_none());
}

#[test]
fn merge_refuses_mixing_extrusion_with_travel() {
    let a = seg(1, 10_000.0, 20.0, [0.0, 0.0, 0.0], [0.2, 0.0, 0.0], 0.01);
    let b = seg(2, 10_000.0, 20.0, [0.2, 0.0, 0.0], [0.4, 0.0, 0.0], 0.0);
    assert!(merge_pair(&a, &b).is_none());
}

#[test]
fn merge_joins_travels() {
    let a = seg(1, 10_000.0, 20.0, [0.0, 0.0, 0.0], [0.2, 0.0, 0.0], 0.0);
    let b = seg(2, 10_000.0, 20.0, [0.2, 0.0, 0.0], [0.4, 0.0, 0.0], 0.0);
    let m = merge_pair(&a, &b).expect("collinear travels merge");
    assert!(m.segment.followers.is_empty());
}

#[test]
fn merge_deviation_budget_covers_absorbed_vertices() {
    // A gentle arc of sub-degree facets: each junction merges until the
    // absorbed vertices' sagitta exceeds the corner-deviation budget, so a
    // curve cannot silently flatten into one chord.
    let accel = 10_000.0;
    let scv = 20.0;
    let budget = delta_of(accel, scv);
    let r = 50.0_f64;
    let facet = 0.4;
    let dtheta = facet / r;
    let vertex = |i: usize| {
        let th = dtheta * i as f64;
        [r * libm::sin(th), r * (1.0 - libm::cos(th)), 0.0]
    };
    let mut merged = seg(1, accel, scv, vertex(0), vertex(1), 0.01);
    let mut absorbed: Vec<[f64; 3]> = Vec::new();
    let mut joined = 1;
    for i in 1..40 {
        let next = seg(i as u32 + 1, accel, scv, vertex(i), vertex(i + 1), 0.01);
        match merge_collinear_lines(&merged, &next, &absorbed, CornerFitConfig::default()) {
            Some(m) => {
                absorbed.push(vertex(i));
                merged = m;
                joined += 1;
            }
            None => break,
        }
    }
    assert!(joined > 1, "gentle facets must merge at all");
    let line = as_line(&merged);
    let worst = absorbed
        .iter()
        .map(|v| {
            let t = ((v[0] - line.start[0]) * (line.end[0] - line.start[0])
                + (v[1] - line.start[1]) * (line.end[1] - line.start[1]))
                / (line.s_len() * line.s_len());
            let p = [
                line.start[0] + t * (line.end[0] - line.start[0]),
                line.start[1] + t * (line.end[1] - line.start[1]),
                0.0,
            ];
            dist(*v, p)
        })
        .fold(0.0_f64, f64::max);
    assert!(
        worst <= budget,
        "absorbed vertices stay within the corner budget: {worst} vs {budget}"
    );
    assert!(joined < 40, "the budget must stop a curve from flattening");
}
