use super::pipeline_setup::retired_by_axis;

#[test]
fn single_slave_places_retired_at_its_axis() {
    assert_eq!(retired_by_axis(&[2], &[7]), vec![0, 0, 7]);
}

#[test]
fn distinct_axes_map_one_to_one() {
    assert_eq!(retired_by_axis(&[0, 1], &[3, 9]), vec![3, 9]);
}

#[test]
fn awd_axis_reports_the_laggard_slot() {
    assert_eq!(retired_by_axis(&[0, 0, 1, 1], &[5, 3, 8, 8]), vec![3, 8]);
}

#[test]
fn missing_slot_counter_is_skipped() {
    assert_eq!(retired_by_axis(&[0, 1], &[4]), vec![4, 0]);
}
