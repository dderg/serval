use super::*;

use crate::path::{Line, Segment};

fn limits() -> VelocityLimits {
    VelocityLimits {
        max_velocity_mm_s: 300.0,
        accel_mm_s2: 3000.0,
        corner_deviation_mm: 5.0,
        max_jerk_mm_s3: 100_000.0,
    }
}

fn ctx() -> MoveContext {
    MoveContext {
        extruder_axis: 3,
        feedrate_mm_s: 30.0,
        limits: limits(),
        source: SourceRange {
            start_line: 7,
            end_line: 7,
        },
    }
}

fn assert_close(a: f64, b: f64, tol: f64) {
    assert!((a - b).abs() <= tol, "expected {a} ≈ {b} (tol {tol})");
}

fn spatial(m: &Move) -> &Segment {
    m.segment.spatial.as_ref().expect("spatial segment present")
}

fn line_of(m: &Move) -> &Line {
    match spatial(m) {
        Segment::Line(l) => l,
        other => panic!("expected Line, got {other:?}"),
    }
}

#[test]
fn line_move_builds_line_with_extruder_follower() {
    let m = line_move([0.0, 0.0, 0.0], [10.0, 0.0, 0.0], 1.0, ctx()).unwrap();

    let line = line_of(&m);
    assert_eq!(line.start, [0.0, 0.0, 0.0]);
    assert_eq!(line.end, [10.0, 0.0, 0.0]);

    assert_eq!(m.segment.followers.len(), 1);
    let f = m.segment.followers[0];
    assert_eq!(f.axis_index, 3);
    assert_close(f.ratio * m.segment.s_len(), 1.0, 1e-12);

    assert_close(m.feedrate_mm_s, 30.0, 0.0);
    assert_eq!(m.limits, limits());
    assert_eq!(m.source.start_line, 7);
}

#[test]
fn line_move_travel_has_no_follower() {
    let m = line_move([0.0, 0.0, 0.0], [3.0, 4.0, 0.0], 0.0, ctx()).unwrap();
    assert!(m.segment.followers.is_empty());
    assert_close(m.segment.s_len(), 5.0, 1e-12);
}

#[test]
fn line_move_pure_retraction_is_virtual() {
    let m = line_move([1.0, 2.0, 3.0], [1.0, 2.0, 3.0], -2.0, ctx()).unwrap();
    assert!(m.segment.spatial.is_none());
    assert_eq!(m.segment.virtual_path_mm, Some(2.0));
    assert_eq!(m.segment.followers.len(), 1);
    let f = m.segment.followers[0];
    assert_close(f.ratio, -1.0, 0.0);
    assert_close(f.ratio * m.segment.s_len(), -2.0, 1e-12);
}

#[test]
fn line_move_zero_motion_rejected() {
    let err = line_move([5.0, 5.0, 5.0], [5.0, 5.0, 5.0], 0.0, ctx()).unwrap_err();
    assert_eq!(err, FrontendError::ZeroMotion { line_no: 7 });
}

#[test]
fn line_move_non_finite_coordinate_rejected() {
    let err = line_move([0.0, 0.0, 0.0], [f64::NAN, 0.0, 0.0], 0.0, ctx()).unwrap_err();
    assert_eq!(err, FrontendError::NonFiniteInput { line_no: 7 });

    let err = line_move([0.0, 0.0, 0.0], [f64::INFINITY, 0.0, 0.0], 1.0, ctx()).unwrap_err();
    assert_eq!(err, FrontendError::NonFiniteInput { line_no: 7 });

    let err = line_move([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], f64::NAN, ctx()).unwrap_err();
    assert_eq!(err, FrontendError::NonFiniteInput { line_no: 7 });
}

#[test]
fn invalid_feedrate_rejected_before_geometry() {
    let mut bad = ctx();
    bad.feedrate_mm_s = 0.0;
    let err = line_move([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], 0.0, bad).unwrap_err();
    assert_eq!(err, FrontendError::InvalidFeedrate { line_no: 7 });

    bad.feedrate_mm_s = f64::NAN;
    let err = line_move([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 0.0, bad).unwrap_err();
    assert_eq!(err, FrontendError::InvalidFeedrate { line_no: 7 });
}

#[test]
fn invalid_limits_rejected() {
    for (mv, ac, scv, jerk) in [
        (0.0, 3000.0, 5.0, 100_000.0),
        (300.0, 0.0, 5.0, 100_000.0),
        (300.0, 3000.0, -1.0, 100_000.0),
        (f64::INFINITY, 3000.0, 5.0, 100_000.0),
        (300.0, 3000.0, 5.0, 0.0),
    ] {
        let mut bad = ctx();
        bad.limits = VelocityLimits {
            max_velocity_mm_s: mv,
            accel_mm_s2: ac,
            corner_deviation_mm: scv,
            max_jerk_mm_s3: jerk,
        };
        let err = line_move([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 0.0, bad).unwrap_err();
        assert!(matches!(
            err,
            FrontendError::InvalidLimits { line_no: 7, .. }
        ));
    }
}

#[test]
fn velocity_limits_try_new_validates() {
    assert!(VelocityLimits::try_new(300.0, 3000.0, 5.0, 100_000.0).is_ok());
    assert!(VelocityLimits::try_new(300.0, 3000.0, 0.0, 100_000.0).is_ok());
    assert!(VelocityLimits::try_new(300.0, 3000.0, 5.0, f64::INFINITY).is_ok());
    assert!(VelocityLimits::try_new(0.0, 3000.0, 5.0, 100_000.0).is_err());
    assert!(VelocityLimits::try_new(300.0, -1.0, 5.0, 100_000.0).is_err());
    assert!(VelocityLimits::try_new(300.0, 3000.0, f64::NAN, 100_000.0).is_err());
    assert!(VelocityLimits::try_new(300.0, 3000.0, 5.0, 0.0).is_err());
    assert!(VelocityLimits::try_new(300.0, 3000.0, 5.0, -1.0).is_err());
    assert!(VelocityLimits::try_new(300.0, 3000.0, 5.0, f64::NAN).is_err());
}
