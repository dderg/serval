use super::lower_profile;
use crate::GeometryError;
use crate::fitter::{CornerFitConfig, FitOutcome, FitReport, fit_corners};
use crate::frontend::{Move, VelocityLimits};
use crate::path::lowering::PositionProfile;
use crate::path::{Arc, CurvatureProfile, Line, PathSegment, Segment};
use crate::segment::{FollowerDemand, SourceRange};
use crate::velocity::{
    MoveVelocity, VelSample, VelocityConfig, VelocityProfile, VelocityReport, plan_velocity,
};

fn limits(max_v: f64, accel: f64) -> VelocityLimits {
    VelocityLimits::try_new(max_v, accel, 5.0).unwrap()
}

fn src(line_no: u32) -> SourceRange {
    SourceRange {
        start_line: line_no,
        end_line: line_no,
    }
}

fn line(start: [f64; 3], end: [f64; 3], feed: f64, max_v: f64, accel: f64, line_no: u32) -> Move {
    let seg = Segment::Line(Line::try_new(start, end).unwrap());
    Move {
        segment: PathSegment::try_new(seg, Vec::new()).unwrap(),
        feedrate_mm_s: feed,
        limits: limits(max_v, accel),
        source: src(line_no),
    }
}

fn planned(moves: Vec<Move>) -> (FitOutcome, VelocityProfile) {
    let out = fit_corners(&moves, CornerFitConfig::default()).unwrap();
    let plan = plan_velocity(&out, VelocityConfig::default()).unwrap();
    (out, plan)
}

fn vmove(length: f64, samples: &[(f64, f64)], accel: f64, line_no: u32) -> MoveVelocity {
    let samples: Vec<VelSample> = samples
        .iter()
        .map(|&(s, v)| VelSample { s, v, a: 0.0 })
        .collect();
    MoveVelocity {
        entry_v: samples.first().map_or(0.0, |x| x.v),
        exit_v: samples.last().map_or(0.0, |x| x.v),
        peak_v: samples.iter().fold(0.0_f64, |a, x| a.max(x.v)),
        samples,
        phases: Vec::new(),
        accel,
        jerk: 1.0e6,
        length,
        source: src(line_no),
    }
}

fn outcome(moves: Vec<Move>) -> FitOutcome {
    FitOutcome {
        moves,
        report: FitReport::default(),
    }
}

fn profile(moves: Vec<MoveVelocity>) -> VelocityProfile {
    VelocityProfile {
        moves,
        report: VelocityReport::default(),
        barrier: 0,
        v_barrier: 0.0,
    }
}

#[test]
fn empty_sequence_lowers_to_nothing() {
    let got = lower_profile(&outcome(Vec::new()), &profile(Vec::new()), 1000.0).unwrap();
    assert!(got.is_empty());
}

#[test]
fn rate_must_be_finite_and_positive() {
    let (out, plan) = planned(vec![line(
        [0.0, 0.0, 0.0],
        [100.0, 0.0, 0.0],
        50.0,
        200.0,
        1000.0,
        1,
    )]);
    for bad in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        assert!(matches!(
            lower_profile(&out, &plan, bad),
            Err(GeometryError::InvalidLowering { .. })
        ));
    }
    assert!(lower_profile(&out, &plan, 1000.0).is_ok());
}

#[test]
fn move_count_mismatch_fails_loud() {
    let out = outcome(vec![line(
        [0.0, 0.0, 0.0],
        [10.0, 0.0, 0.0],
        50.0,
        200.0,
        1000.0,
        1,
    )]);
    assert!(matches!(
        lower_profile(&out, &profile(Vec::new()), 1000.0),
        Err(GeometryError::InvalidLowering { .. })
    ));
}

#[test]
fn source_mismatch_fails_loud() {
    let out = outcome(vec![line(
        [0.0, 0.0, 0.0],
        [10.0, 0.0, 0.0],
        50.0,
        200.0,
        1000.0,
        1,
    )]);
    let plan = profile(vec![vmove(10.0, &[(0.0, 0.0), (10.0, 5.0)], 1000.0, 2)]);
    assert!(matches!(
        lower_profile(&out, &plan, 1000.0),
        Err(GeometryError::InvalidLowering { .. })
    ));
}

#[test]
fn trapezoid_line_positions_and_timing() {
    let rate = 1000.0;
    let (out, plan) = planned(vec![line(
        [0.0, 0.0, 0.0],
        [100.0, 0.0, 0.0],
        50.0,
        200.0,
        1000.0,
        1,
    )]);
    let samples = lower_profile(&out, &plan, rate).unwrap();
    assert!(samples.len() > 2);

    let mut prev_x = -1.0;
    for s in &samples {
        let p = s.position.expect("spatial line lowers to a position");
        assert!(
            p[1].abs() < 1e-12 && p[2].abs() < 1e-12,
            "stays on the line"
        );
        assert!(
            p[0] >= prev_x - 1e-9 && p[0] <= 100.0 + 1e-9,
            "x monotone in [0,100]"
        );
        prev_x = p[0];
    }
    assert!(samples[0].position.unwrap()[0].abs() < 1e-9);
    let last = samples.last().unwrap();
    assert!(
        (last.position.unwrap()[0] - 100.0).abs() < 1e-9,
        "ends at the line end"
    );
    assert!(
        (last.t_s - plan.report.traversal_time_s).abs()
            < 1e-9 * plan.report.traversal_time_s.max(1.0),
        "stream duration equals the planned traversal time"
    );

    let dt = 1.0 / rate;
    for w in samples.windows(2) {
        let step = w[1].t_s - w[0].t_s;
        assert!(step > 0.0, "t strictly ascending");
        assert!(step <= dt + 1e-12, "spacing never exceeds the fixed step");
    }
}

#[test]
fn total_time_matches_report_on_a_blended_corner_chain() {
    let rate = 2000.0;
    let (out, plan) = planned(vec![
        line([0.0, 0.0, 0.0], [50.0, 0.0, 0.0], 100.0, 300.0, 1000.0, 1),
        line([50.0, 0.0, 0.0], [50.0, 50.0, 0.0], 100.0, 300.0, 1000.0, 3),
    ]);
    let samples = lower_profile(&out, &plan, rate).unwrap();

    let last = samples.last().unwrap();
    assert!(
        (last.t_s - plan.report.traversal_time_s).abs()
            < 1e-9 * plan.report.traversal_time_s.max(1.0),
        "two independent time integrators agree"
    );
    for w in samples.windows(2) {
        assert!(
            w[1].t_s > w[0].t_s,
            "t strictly ascending across move seams"
        );
    }
}

#[test]
fn position_is_continuous_across_move_seams() {
    let rate = 2000.0;
    let moves = vec![
        line([0.0, 0.0, 0.0], [50.0, 0.0, 0.0], 100.0, 300.0, 1000.0, 1),
        line([50.0, 0.0, 0.0], [50.0, 50.0, 0.0], 100.0, 300.0, 1000.0, 3),
    ];
    let (out, plan) = planned(moves);
    let peak = plan.moves.iter().fold(0.0_f64, |a, m| a.max(m.peak_v));
    let samples = lower_profile(&out, &plan, rate).unwrap();
    let dt = 1.0 / rate;

    for w in samples.windows(2) {
        let a = w[0].position.unwrap();
        let b = w[1].position.unwrap();
        let step = ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2) + (b[2] - a[2]).powi(2)).sqrt();
        assert!(
            step <= peak * dt + 1e-6,
            "no positional jump at a seam: step {step} exceeds v_max·dt"
        );
    }
}

#[test]
fn followers_advance_monotonically() {
    let rate = 1000.0;
    let seg = PathSegment::try_new(
        Segment::Line(Line::try_new([0.0, 0.0, 0.0], [20.0, 0.0, 0.0]).unwrap()),
        vec![FollowerDemand {
            axis_index: 3,
            ratio: 0.1,
        }],
    )
    .unwrap();
    let m = Move {
        segment: seg,
        feedrate_mm_s: 60.0,
        limits: limits(200.0, 1000.0),
        source: src(1),
    };
    let (out, plan) = planned(vec![m]);
    let samples = lower_profile(&out, &plan, rate).unwrap();

    let mut prev = -1.0;
    for s in &samples {
        let e = s.followers[0];
        assert!(e >= prev - 1e-12, "follower monotone non-decreasing");
        prev = e;
    }
    assert!(
        (samples.last().unwrap().followers[0] - 2.0).abs() < 1e-9,
        "0.1·20mm"
    );
}

#[test]
fn constant_accel_inversion_is_exact_on_a_single_interval() {
    let rate = 1000.0;
    let length = 100.0_f64;
    let accel = 1000.0_f64;
    let v_end = (2.0 * accel * length).sqrt();
    let out = outcome(vec![line(
        [0.0, 0.0, 0.0],
        [length, 0.0, 0.0],
        v_end,
        v_end,
        accel,
        1,
    )]);
    let plan = profile(vec![vmove(
        length,
        &[(0.0, 0.0), (length, v_end)],
        accel,
        1,
    )]);
    let samples = lower_profile(&out, &plan, rate).unwrap();

    for s in &samples {
        let s_recovered = s.position.unwrap()[0];
        let s_expected = (0.5 * accel * s.t_s * s.t_s).min(length);
        assert!(
            (s_recovered - s_expected).abs() < 1e-6,
            "s(t)=½at² at t={}: {s_recovered} vs {s_expected}",
            s.t_s
        );
    }
    assert!((samples.last().unwrap().t_s - v_end / accel).abs() < 1e-9);
}

#[test]
fn stop_node_is_reached_in_finite_time_without_dwell() {
    let rate = 1000.0;
    let length = 50.0_f64;
    let accel = 1000.0_f64;
    let v0 = (2.0 * accel * length).sqrt();
    let out = outcome(vec![line(
        [0.0, 0.0, 0.0],
        [length, 0.0, 0.0],
        v0,
        v0,
        accel,
        1,
    )]);
    let plan = profile(vec![vmove(length, &[(0.0, v0), (length, 0.0)], accel, 1)]);
    let samples = lower_profile(&out, &plan, rate).unwrap();

    let last = samples.last().unwrap();
    assert!(
        (last.t_s - v0 / accel).abs() < 1e-9,
        "constant-decel stop time is finite"
    );
    assert!((last.position.unwrap()[0] - length).abs() < 1e-9);
    for w in samples.windows(2) {
        assert!(w[1].t_s > w[0].t_s, "no dwell — t never repeats");
    }
}

#[test]
fn virtual_path_lowers_with_no_position() {
    let rate = 500.0;
    let seg = PathSegment::try_new_virtual(
        vec![FollowerDemand {
            axis_index: 3,
            ratio: 1.0,
        }],
        2.0,
    )
    .unwrap();
    let m = Move {
        segment: seg,
        feedrate_mm_s: 10.0,
        limits: limits(50.0, 1000.0),
        source: src(1),
    };
    let out = outcome(vec![m]);
    let plan = profile(vec![vmove(
        2.0,
        &[(0.0, 0.0), (1.0, 8.0), (2.0, 0.0)],
        1000.0,
        1,
    )]);
    let samples = lower_profile(&out, &plan, rate).unwrap();

    assert!(
        samples.iter().all(|s| s.position.is_none()),
        "no spatial position"
    );
    let mut prev = -1.0;
    for s in &samples {
        assert!(s.followers[0] >= prev - 1e-12);
        prev = s.followers[0];
    }
    assert!((samples.last().unwrap().followers[0] - 2.0).abs() < 1e-9);
}

fn lower_with_samples(length: f64, samples: &[(f64, f64)]) -> Result<(), GeometryError> {
    let out = outcome(vec![line(
        [0.0, 0.0, 0.0],
        [length, 0.0, 0.0],
        100.0,
        200.0,
        1000.0,
        1,
    )]);
    let plan = profile(vec![vmove(length, samples, 1000.0, 1)]);
    lower_profile(&out, &plan, 1000.0).map(|_| ())
}

#[test]
fn malformed_profile_samples_fail_loud() {
    let does_not_span_segment_end = &[(0.0, 0.0), (50.0, 30.0)];
    assert!(lower_with_samples(100.0, does_not_span_segment_end).is_err());

    let does_not_start_at_s_zero = &[(1.0, 0.0), (100.0, 30.0)];
    assert!(lower_with_samples(100.0, does_not_start_at_s_zero).is_err());

    let non_monotone_arc_length = &[(0.0, 0.0), (50.0, 30.0), (50.0, 30.0), (100.0, 10.0)];
    assert!(lower_with_samples(100.0, non_monotone_arc_length).is_err());

    let negative_velocity = &[(0.0, 0.0), (100.0, -5.0)];
    assert!(lower_with_samples(100.0, negative_velocity).is_err());

    let non_finite_velocity = &[(0.0, 0.0), (100.0, f64::NAN)];
    assert!(lower_with_samples(100.0, non_finite_velocity).is_err());

    let stalled_at_zero_velocity = &[(0.0, 0.0), (100.0, 0.0)];
    assert!(lower_with_samples(100.0, stalled_at_zero_velocity).is_err());

    let single_sample_cannot_form_interval = &[(0.0, 0.0)];
    assert!(lower_with_samples(100.0, single_sample_cannot_form_interval).is_err());
}

#[test]
fn non_finite_anchor_is_rejected() {
    let seg = Segment::Line(Line::try_new([0.0, 0.0, 0.0], [f64::NAN, 0.0, 0.0]).unwrap());
    let m = Move {
        segment: PathSegment::try_new(seg, Vec::new()).unwrap(),
        feedrate_mm_s: 100.0,
        limits: limits(200.0, 1000.0),
        source: src(1),
    };
    let out = outcome(vec![m]);
    let plan = profile(vec![vmove(1.0, &[(0.0, 0.0), (1.0, 5.0)], 1000.0, 1)]);
    assert!(matches!(
        lower_profile(&out, &plan, 1000.0),
        Err(GeometryError::InvalidLowering {
            reason: "spatial anchor is not finite"
        })
    ));
}

#[test]
fn non_finite_segment_length_is_rejected() {
    let arc = Arc {
        origin: [0.0, 0.0, 0.0],
        u: [1.0, 0.0, 0.0],
        v: [0.0, 1.0, 0.0],
        radius: f64::NAN,
        start_angle: 0.0,
        sweep: 1.0,
    };
    assert!(
        !arc.s_len().is_finite(),
        "NaN radius yields a non-finite s_len"
    );
    let m = Move {
        segment: PathSegment::try_new(Segment::Arc(arc), Vec::new()).unwrap(),
        feedrate_mm_s: 100.0,
        limits: limits(200.0, 1000.0),
        source: src(1),
    };
    let out = outcome(vec![m]);
    let plan = profile(vec![vmove(1.0, &[(0.0, 0.0), (1.0, 5.0)], 1000.0, 1)]);
    assert!(matches!(
        lower_profile(&out, &plan, 1000.0),
        Err(GeometryError::InvalidLowering {
            reason: "segment length is not finite and positive"
        })
    ));
}

#[test]
fn seam_samples_match_point_at_at_the_shared_node() {
    let rate = 2000.0;
    let moves = vec![
        line([0.0, 0.0, 0.0], [50.0, 0.0, 0.0], 100.0, 300.0, 1000.0, 1),
        line([50.0, 0.0, 0.0], [50.0, 50.0, 0.0], 100.0, 300.0, 1000.0, 3),
    ];
    let (out, plan) = planned(moves);
    let samples = lower_profile(&out, &plan, rate).unwrap();

    for pair in out.moves.windows(2) {
        let (left, right) = (&pair[0], &pair[1]);
        let node_out = left
            .segment
            .spatial
            .as_ref()
            .unwrap()
            .point_at(left.segment.s_len());
        let node_in = right.segment.spatial.as_ref().unwrap().point_at(0.0);
        let gap = ((node_out[0] - node_in[0]).powi(2)
            + (node_out[1] - node_in[1]).powi(2)
            + (node_out[2] - node_in[2]).powi(2))
        .sqrt();
        assert!(
            gap < 1e-6,
            "geometry is continuous at the shared node: gap {gap}"
        );
        assert!(
            samples.iter().any(|s| s.position.is_some_and(|p| {
                ((p[0] - node_out[0]).powi(2)
                    + (p[1] - node_out[1]).powi(2)
                    + (p[2] - node_out[2]).powi(2))
                .sqrt()
                    < 100.0 / rate
            })),
            "the lowered stream passes through the shared node"
        );
    }
}

#[test]
fn constant_accel_inversion_is_exact_across_multiple_intervals() {
    let rate = 1000.0;
    let half = 50.0_f64;
    let accel = 1000.0_f64;
    let v_mid = (2.0 * accel * half).sqrt();
    let out = outcome(vec![line(
        [0.0, 0.0, 0.0],
        [2.0 * half, 0.0, 0.0],
        v_mid,
        v_mid,
        accel,
        1,
    )]);
    let plan = profile(vec![vmove(
        2.0 * half,
        &[(0.0, 0.0), (half, v_mid), (2.0 * half, 0.0)],
        accel,
        1,
    )]);
    let samples = lower_profile(&out, &plan, rate).unwrap();

    let t_mid = v_mid / accel;
    for s in &samples {
        let s_recovered = s.position.unwrap()[0];
        let s_expected = if s.t_s <= t_mid {
            0.5 * accel * s.t_s * s.t_s
        } else {
            let dt = s.t_s - t_mid;
            half + v_mid * dt - 0.5 * accel * dt * dt
        }
        .min(2.0 * half);
        assert!(
            (s_recovered - s_expected).abs() < 1e-6,
            "piecewise ½at² at t={}: {s_recovered} vs {s_expected}",
            s.t_s
        );
    }
    assert!((samples.last().unwrap().t_s - 2.0 * t_mid).abs() < 1e-9);
}
