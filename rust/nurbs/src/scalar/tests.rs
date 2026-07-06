use super::*;
use crate::ConstructError;

fn linear_curve() -> ScalarNurbs<f64> {
    ScalarNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0]).unwrap()
}

#[test]
fn try_new_accepts_valid_linear() {
    let curve = linear_curve();
    assert_eq!(curve.degree(), 1);
    assert_eq!(curve.control_points(), &[0.0, 1.0]);
}

#[test]
fn try_new_rejects_degree_exceeded() {
    let result = ScalarNurbs::<f64>::try_new(21, vec![0.0; 23], vec![0.0; 1]);
    assert!(matches!(
        result,
        Err(ConstructError::DegreeExceeded {
            actual: 21,
            max: 20
        })
    ));
}

#[test]
fn try_new_rejects_knot_count_mismatch() {
    let result = ScalarNurbs::<f64>::try_new(
        1,
        vec![0.0, 0.0, 1.0], // 3 knots, but 2 cps + 1 + 1 = 4 expected
        vec![0.0, 1.0],
    );
    assert!(matches!(
        result,
        Err(ConstructError::KnotCountMismatch { .. })
    ));
}

#[test]
fn try_new_rejects_unclamped_start() {
    let result = ScalarNurbs::<f64>::try_new(1, vec![0.0, 0.5, 1.0, 1.0], vec![0.0, 1.0]);
    assert!(matches!(result, Err(ConstructError::KnotsNotClamped)));
}

#[test]
fn try_new_rejects_unclamped_end() {
    let result = ScalarNurbs::<f64>::try_new(1, vec![0.0, 0.0, 0.5, 1.0], vec![0.0, 1.0]);
    assert!(matches!(result, Err(ConstructError::KnotsNotClamped)));
}

#[test]
fn try_new_rejects_non_monotone_knots() {
    let result = ScalarNurbs::<f64>::try_new(
        2,
        vec![0.0, 0.0, 0.0, 0.4, 0.3, 1.0, 1.0, 1.0], // 0.3 < 0.4
        vec![0.0, 0.5, 1.0, 1.5, 2.0],
    );
    assert!(matches!(result, Err(ConstructError::KnotsNotMonotone)));
}

#[test]
fn try_new_rejects_degenerate_knot_range() {
    let result = ScalarNurbs::<f64>::try_new(1, vec![0.0, 0.0, 0.0, 0.0], vec![0.0, 1.0]);
    assert!(matches!(result, Err(ConstructError::DegenerateKnotRange)));
}

#[test]
fn as_view_provides_borrowed_access() {
    let owned = linear_curve();
    let view = owned.as_view();
    assert_eq!(view.degree(), 1);
    assert_eq!(view.knots(), &[0.0, 0.0, 1.0, 1.0]);
    assert_eq!(view.control_points(), &[0.0, 1.0]);
}

#[test]
fn ref_try_new_accepts_valid_data() {
    let knots = [0.0_f64, 0.0, 1.0, 1.0];
    let cps = [0.0_f64, 1.0];
    let r = ScalarNurbsRef::try_new(1, &knots, &cps).unwrap();
    assert_eq!(r.degree(), 1);
}
