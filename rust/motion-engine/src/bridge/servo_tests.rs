use super::servo::validate_dynamics_pairs;

#[test]
fn valid_flat_pair_builds_wire_record() {
    let pairs = validate_dynamics_pairs(&[0.5, -0.5], 1, 2, &[0, 1], &[0.1]).unwrap();
    assert_eq!(pairs.len(), 1);
    assert_eq!((pairs[0].first, pairs[0].second), (0, 1));
    assert_eq!(pairs[0].direction_split, 0.1);
}

#[test]
fn rejects_bad_pair_array_shapes_and_coefficients() {
    let odd_slots = validate_dynamics_pairs(&[0.5, 0.5], 1, 2, &[0], &[]).unwrap_err();
    assert!(odd_slots.contains("pairs must be flat"), "{odd_slots}");
    let wrong_coefficients = validate_dynamics_pairs(&[0.5, 0.5], 1, 2, &[0, 1], &[]).unwrap_err();
    assert!(
        wrong_coefficients.contains("one coefficient per pair"),
        "{wrong_coefficients}"
    );
    let boundary = validate_dynamics_pairs(&[0.5, 0.5], 1, 2, &[0, 1], &[0.5]).unwrap_err();
    assert!(boundary.contains("abs < 0.5"), "{boundary}");
}

#[test]
fn rejects_zero_non_parallel_and_overlapping_pair_columns() {
    let zero = validate_dynamics_pairs(&[0.0, 0.0, 1.0], 1, 3, &[0, 1], &[0.1]).unwrap_err();
    assert!(
        zero.contains("first frame column must be nonzero"),
        "{zero}"
    );
    let non_parallel = validate_dynamics_pairs(&[0.5, 0.4], 1, 2, &[0, 1], &[0.1]).unwrap_err();
    assert!(
        non_parallel.contains("exact equal or opposite"),
        "{non_parallel}"
    );
    let overlap =
        validate_dynamics_pairs(&[0.5, 0.5, 0.5], 1, 3, &[0, 1, 1, 2], &[0.1, -0.1]).unwrap_err();
    assert!(overlap.contains("more than one pair"), "{overlap}");
}
