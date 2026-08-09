use super::axis_ring_depth;

#[test]
fn lower_clamp_keeps_at_least_one() {
    assert_eq!(axis_ring_depth(0, 2), 1);
}

#[test]
fn zero_num_axes_treated_as_one() {
    assert_eq!(axis_ring_depth(1000, 0), 1000);
}
