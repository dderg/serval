use super::servo::{buzz_axis_bits, buzz_lanes, validate_dynamics_pairs};

#[test]
fn axis_bits_keep_only_the_endpoint_own_masked_axes() {
    let driven = [0u8, 2, 3];
    let bits = buzz_axis_bits(0b0000_1111, |axis| driven.contains(&axis));
    assert_eq!(bits, 0b0000_1101);
    let narrowed = buzz_axis_bits(0b0000_1111, |axis| axis == 2);
    assert_eq!(narrowed, 0b0000_0100);
    assert_eq!(buzz_axis_bits(0b0000_0010, |axis| axis == 2), 0);
    assert_eq!(buzz_axis_bits(0, |_| true), 0);
}

#[test]
fn axis_bits_ignore_mask_bits_above_the_eight_wire_axes() {
    assert_eq!(buzz_axis_bits(0b1000_0001, |_| true), 0b1000_0001);
}

#[test]
fn phase_lanes_name_one_axis_each_with_the_sign_mask_applied() {
    let lanes = buzz_lanes(0b0000_0101, 0b0000_0100);
    assert_eq!(lanes.len(), 2);
    assert_eq!((lanes[0].axis, lanes[0].sign), (0, 1.0));
    assert_eq!((lanes[1].axis, lanes[1].sign), (2, -1.0));
    assert!(buzz_lanes(0, 0xff).is_empty());
}

#[test]
fn phase_lane_signs_ignore_sign_bits_for_unselected_axes() {
    let lanes = buzz_lanes(0b0000_0010, 0b0000_0101);
    assert_eq!(lanes.len(), 1);
    assert_eq!((lanes[0].axis, lanes[0].sign), (1, 1.0));
}

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
