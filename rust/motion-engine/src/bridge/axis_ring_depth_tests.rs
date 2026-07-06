use super::axis_ring_depth;

#[test]
fn typical_two_axis_mcu_splits_evenly() {
    assert_eq!(axis_ring_depth(1984, 2), 1984 / 2);
}

#[test]
fn single_axis_mcu_gets_full_depth() {
    assert_eq!(axis_ring_depth(1984, 1), 1984);
}

#[test]
fn lower_clamp_keeps_at_least_one() {
    assert_eq!(axis_ring_depth(0, 2), 1);
}

#[test]
fn zero_num_axes_treated_as_one() {
    assert_eq!(axis_ring_depth(1000, 0), 1000);
}
