use geometry::{CubicSegment, FollowerDemand, GeometryError, SourceRange};
use nurbs::VectorNurbs;

fn valid_cubic_xyz() -> VectorNurbs<f64, 3> {
    VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [3.0, 0.0, 0.0],
        ],
    )
    .expect("valid cubic")
}

fn dummy_source() -> SourceRange {
    SourceRange {
        start_line: 1,
        end_line: 1,
    }
}

#[test]
fn try_new_rejects_non_cubic() {
    let linear = VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
    )
    .expect("valid linear");
    let result = CubicSegment::try_new(linear, vec![], 100.0, dummy_source(), None);
    assert!(matches!(
        result,
        Err(GeometryError::NotSinglePieceCubic { .. })
    ));
}

#[test]
fn try_new_accepts_valid_travel() {
    let result = CubicSegment::try_new(valid_cubic_xyz(), vec![], 100.0, dummy_source(), None);
    assert!(result.is_ok());
}

#[test]
fn try_new_accepts_follower_with_signed_ratio() {
    let result = CubicSegment::try_new(
        valid_cubic_xyz(),
        vec![FollowerDemand::constant(3, -0.05)],
        100.0,
        dummy_source(),
        None,
    );
    assert!(result.is_ok());
}

#[test]
fn try_new_rejects_zero_follower_ratio() {
    let result = CubicSegment::try_new(
        valid_cubic_xyz(),
        vec![FollowerDemand::constant(3, 0.0)],
        100.0,
        dummy_source(),
        None,
    );
    assert!(matches!(
        result,
        Err(GeometryError::FollowerInvariantViolation { .. })
    ));
}

#[test]
fn try_new_rejects_duplicate_follower_axis() {
    let result = CubicSegment::try_new(
        valid_cubic_xyz(),
        vec![
            FollowerDemand::constant(3, 0.1),
            FollowerDemand::constant(3, 0.2),
        ],
        100.0,
        dummy_source(),
        None,
    );
    assert!(matches!(
        result,
        Err(GeometryError::FollowerInvariantViolation { .. })
    ));
}

#[test]
fn try_new_rejects_non_finite_control_point() {
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [f64::NAN, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [3.0, 0.0, 0.0],
        ],
    )
    .expect("VectorNurbs accepts NaN at the type level; CubicSegment::try_new must catch it");
    let result = CubicSegment::try_new(
        xyz,
        vec![],
        100.0,
        SourceRange {
            start_line: 1,
            end_line: 1,
        },
        None,
    );
    assert!(matches!(
        result,
        Err(GeometryError::NotSinglePieceCubic { .. })
    ));
}

#[test]
fn try_new_rejects_non_finite_feedrate() {
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [3.0, 0.0, 0.0],
        ],
    )
    .unwrap();
    let result = CubicSegment::try_new(
        xyz,
        vec![],
        f64::INFINITY,
        SourceRange {
            start_line: 1,
            end_line: 1,
        },
        None,
    );
    assert!(matches!(
        result,
        Err(GeometryError::FollowerInvariantViolation { .. })
    ));
}

#[test]
fn try_new_rejects_non_finite_follower_ratio() {
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [3.0, 0.0, 0.0],
        ],
    )
    .unwrap();
    let result = CubicSegment::try_new(
        xyz,
        vec![FollowerDemand::constant(3, f64::NAN)],
        100.0,
        SourceRange {
            start_line: 1,
            end_line: 1,
        },
        None,
    );
    assert!(matches!(
        result,
        Err(GeometryError::FollowerInvariantViolation { .. })
    ));
}
