use super::*;

use crate::path::{Line, Segment};

fn limits() -> VelocityLimits {
    VelocityLimits::try_new(300.0, 3000.0, crate::CornerBudget::Shaped(5.0), 100_000.0).unwrap()
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
            corner: CornerBudget::Shaped(scv),
            max_jerk_mm_s3: jerk,
            ..limits()
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
    assert!(
        VelocityLimits::try_new(300.0, 3000.0, crate::CornerBudget::Shaped(5.0), 100_000.0).is_ok()
    );
    assert!(
        VelocityLimits::try_new(300.0, 3000.0, crate::CornerBudget::Shaped(0.0), 100_000.0).is_ok()
    );
    assert!(
        VelocityLimits::try_new(
            300.0,
            3000.0,
            crate::CornerBudget::Shaped(5.0),
            f64::INFINITY
        )
        .is_ok()
    );
    assert!(
        VelocityLimits::try_new(0.0, 3000.0, crate::CornerBudget::Shaped(5.0), 100_000.0).is_err()
    );
    assert!(
        VelocityLimits::try_new(300.0, -1.0, crate::CornerBudget::Shaped(5.0), 100_000.0).is_err()
    );
    assert!(
        VelocityLimits::try_new(
            300.0,
            3000.0,
            crate::CornerBudget::Shaped(f64::NAN),
            100_000.0
        )
        .is_err()
    );
    assert!(VelocityLimits::try_new(300.0, 3000.0, crate::CornerBudget::Shaped(5.0), 0.0).is_err());
    assert!(
        VelocityLimits::try_new(300.0, 3000.0, crate::CornerBudget::Shaped(5.0), -1.0).is_err()
    );
    assert!(
        VelocityLimits::try_new(300.0, 3000.0, crate::CornerBudget::Shaped(5.0), f64::NAN).is_err()
    );
}

#[test]
fn corner_budget_unshaped_cut_is_the_payload_or_the_scv_conversion() {
    assert_eq!(CornerBudget::Shaped(0.03).unshaped_cut_mm(5000.0), 0.03);
    assert_eq!(CornerBudget::OverFloor(0.04).unshaped_cut_mm(5000.0), 0.04);
    assert_eq!(
        CornerBudget::ScvEquivalent(5.0).unshaped_cut_mm(3000.0),
        corner_deviation_from_scv(5.0, 3000.0)
    );
    assert_eq!(
        CornerBudget::ScvEquivalent(5.0).unshaped_cut_mm(6000.0),
        corner_deviation_from_scv(5.0, 6000.0)
    );
}

#[test]
fn corner_budget_validate_rejects_negative_and_non_finite_payloads() {
    for make in [
        CornerBudget::Shaped,
        CornerBudget::OverFloor,
        CornerBudget::ScvEquivalent,
    ] {
        assert!(make(0.0).validate().is_ok());
        assert!(make(0.5).validate().is_ok());
        for bad in [-1e-9, f64::NAN, f64::INFINITY] {
            assert!(make(bad).validate().is_err(), "{:?}", make(bad));
        }
    }
}

#[test]
fn corner_budget_stricter_keeps_the_kind_or_the_smaller_cut() {
    assert_eq!(
        CornerBudget::Shaped(0.02).stricter(CornerBudget::Shaped(0.01), 3000.0),
        CornerBudget::Shaped(0.01)
    );
    assert_eq!(
        CornerBudget::ScvEquivalent(5.0).stricter(CornerBudget::ScvEquivalent(8.0), 3000.0),
        CornerBudget::ScvEquivalent(5.0)
    );
    assert_eq!(
        CornerBudget::OverFloor(0.04).stricter(CornerBudget::ScvEquivalent(5.0), 3000.0),
        CornerBudget::ScvEquivalent(5.0)
    );
    assert_eq!(
        CornerBudget::Shaped(0.001).stricter(CornerBudget::ScvEquivalent(5.0), 3000.0),
        CornerBudget::Shaped(0.001)
    );
}

#[test]
fn velocity_limits_z_caps_validate() {
    let path = VelocityLimits::try_new(300.0, 3000.0, crate::CornerBudget::Shaped(5.0), 100_000.0)
        .unwrap();
    assert_eq!(path.max_z_velocity_mm_s, f64::INFINITY);
    assert_eq!(path.max_z_accel_mm_s2, f64::INFINITY);
    let capped = path.with_z_caps(25.0, 100.0).unwrap();
    assert_eq!(
        (capped.max_z_velocity_mm_s, capped.max_z_accel_mm_s2),
        (25.0, 100.0)
    );
    assert!(path.with_z_caps(f64::INFINITY, f64::INFINITY).is_ok());
    for (velocity, accel) in [
        (0.0, 100.0),
        (25.0, -1.0),
        (25.0, crate::velocity::law::RAIL_ACCELERATION_DEFECT),
        (f64::NAN, 100.0),
        (25.0, f64::NAN),
    ] {
        assert!(path.with_z_caps(velocity, accel).is_err());
    }
}
