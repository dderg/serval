use super::*;

fn linear_3d_curve() -> VectorNurbs<3> {
    VectorNurbs::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [1.0, 2.0, 3.0]],
    )
    .unwrap()
}

#[test]
fn try_new_accepts_valid_linear_3d() {
    let curve = linear_3d_curve();
    assert_eq!(curve.degree(), 1);
    assert_eq!(curve.control_points()[1], [1.0, 2.0, 3.0]);
}

#[test]
fn try_new_rejects_degree_exceeded() {
    let result = VectorNurbs::<3>::try_new(21, vec![0.0; 23], vec![[0.0; 3]; 1]);
    assert!(matches!(
        result,
        Err(crate::ConstructError::DegreeExceeded { .. })
    ));
}

#[test]
fn try_new_rejects_knot_count_mismatch() {
    let result = VectorNurbs::<3>::try_new(1, vec![0.0, 0.0, 1.0], vec![[0.0; 3], [1.0; 3]]);
    assert!(matches!(
        result,
        Err(crate::ConstructError::KnotCountMismatch { .. })
    ));
}

#[test]
fn as_view_provides_borrowed_access() {
    let owned = linear_3d_curve();
    let view = owned.as_view();
    assert_eq!(view.degree(), 1);
    assert_eq!(view.control_points()[1], [1.0, 2.0, 3.0]);
}
