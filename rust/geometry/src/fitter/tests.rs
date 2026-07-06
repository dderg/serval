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
    // A 17% extrusion-rate step blends under the default (infinite) gate; a
    // tight extruder accel budget rejects the very same ramp in closed form.
    let a = seg(1, 3000.0, 5.0, [0.0, 0.0, 0.0], [10.0, 0.0, 0.0], 0.5);
    let b = seg(2, 3000.0, 5.0, [10.0, 0.0, 0.0], [10.0, 10.0, 0.0], 0.6);

    let open = CornerFitConfig::default();
    assert!(matches!(
        plan_junction(&a, &b, open).unwrap(),
        JunctionPlan::Blend(_)
    ));

    let tight = CornerFitConfig {
        ramp_gate: FollowerRampGate {
            max_accel_mm_s2: 100.0,
            ..FollowerRampGate::default()
        },
        ..CornerFitConfig::default()
    };
    assert!(matches!(
        plan_junction(&a, &b, tight).unwrap(),
        JunctionPlan::Unblended(UnblendReason::ExtrusionRampInfeasible)
    ));
}

#[test]
fn pressure_advance_amplifies_the_corner_ramp_gate() {
    // The same corner under the same accel budget: fine without pressure
    // advance, infeasible once the gain amplifies the worst-case demand by
    // `k·(r·J + 3·m·v·a)`.
    let a = seg(1, 3000.0, 5.0, [0.0, 0.0, 0.0], [10.0, 0.0, 0.0], 0.5);
    let b = seg(2, 3000.0, 5.0, [10.0, 0.0, 0.0], [10.0, 10.0, 0.0], 0.6);

    let gate = |pa: f64| CornerFitConfig {
        ramp_gate: FollowerRampGate {
            max_accel_mm_s2: 1000.0,
            pressure_advance_s: pa,
            ..FollowerRampGate::default()
        },
        ..CornerFitConfig::default()
    };
    assert!(matches!(
        plan_junction(&a, &b, gate(0.0)).unwrap(),
        JunctionPlan::Blend(_)
    ));
    assert!(matches!(
        plan_junction(&a, &b, gate(0.2)).unwrap(),
        JunctionPlan::Unblended(UnblendReason::ExtrusionRampInfeasible)
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
        ramp_gate: FollowerRampGate {
            max_accel_mm_s2: 10.0,
            ..FollowerRampGate::default()
        },
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
