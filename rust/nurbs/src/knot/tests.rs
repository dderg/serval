use super::*;
use crate::ScalarNurbs;
use crate::eval::eval;

#[test]
fn try_new_accepts_monotone_knots() {
    let kv = KnotVector::try_new(vec![0.0, 0.0, 0.5, 1.0, 1.0]).unwrap();
    assert_eq!(kv.as_slice(), &[0.0, 0.0, 0.5, 1.0, 1.0]);
}

#[test]
fn try_new_rejects_non_monotone() {
    let result = KnotVector::try_new(vec![0.0, 0.5, 0.3, 1.0]);
    assert!(matches!(result, Err(ConstructError::KnotsNotMonotone)));
}

#[test]
fn try_new_rejects_too_short() {
    let result = KnotVector::try_new(vec![0.0]);
    assert!(matches!(
        result,
        Err(ConstructError::KnotCountMismatch { .. })
    ));
}

#[test]
fn find_knot_span_returns_correct_span() {
    let knots = [0.0_f64, 0.0, 0.5, 1.0, 1.0];
    assert_eq!(find_knot_span(&knots, 1, 3, 0.25), 1);
    assert_eq!(find_knot_span(&knots, 1, 3, 1.0), 2);
    assert_eq!(find_knot_span(&knots, 1, 3, 0.0), 1);
}

#[test]
fn refined_to_full_multiplicity_raises_interior_knots() {
    let curve = ScalarNurbs::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0],
        vec![0.0, 1.0, 2.0, 3.0, 4.0],
    )
    .unwrap();

    let refined = refined_to_full_multiplicity(&curve);

    assert_eq!(
        refined.knots(),
        &[0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 1.0, 1.0, 1.0, 1.0]
    );
    for u in [0.0, 0.25, 0.5, 0.75, 1.0] {
        let before = eval(&curve, u);
        let after = eval(&refined, u);
        assert!(
            (before - after).abs() < 1e-10,
            "u={u}: before={before}, after={after}"
        );
    }
}

#[test]
fn refined_to_full_multiplicity_is_identity_when_interior_knots_already_full() {
    let curve = ScalarNurbs::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 1.0, 1.0, 1.0, 1.0],
        vec![0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
    )
    .unwrap();

    let refined = refined_to_full_multiplicity(&curve);

    assert_eq!(refined.knots(), curve.knots());
    assert_eq!(refined.control_points(), curve.control_points());
}

#[test]
fn refined_to_full_multiplicity_is_identity_when_no_interior_knots() {
    let curve = ScalarNurbs::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![0.0, 1.0, 2.0, 3.0],
    )
    .unwrap();

    let refined = refined_to_full_multiplicity(&curve);

    assert_eq!(refined.knots(), curve.knots());
    assert_eq!(refined.control_points(), curve.control_points());
}

#[test]
fn refinement_preserves_discontinuous_piece_values() {
    let discontinuous = ScalarNurbs::try_new(
        2,
        vec![0.0, 0.0, 0.0, 0.25, 0.5, 0.5, 0.5, 1.0, 1.0, 1.0],
        vec![0.0, 0.5, 1.0, 2.0, 10.0, 11.0, 12.0],
    )
    .unwrap();
    let refined = refined_to_full_multiplicity(&discontinuous);
    let pieces = crate::bezier::extract_bezier_pieces(&refined);
    for piece in pieces {
        for fraction in [0.1, 0.5, 0.9] {
            let t = piece.u_start + fraction * (piece.u_end - piece.u_start);
            assert!((piece.evaluate(t) - crate::eval::eval(&discontinuous, t)).abs() < 1e-12);
        }
    }
    assert_eq!(crate::eval::eval(&refined, 0.5), 10.0);
}
