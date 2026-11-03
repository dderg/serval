use super::*;

const FREQ: f64 = 84_000_000.0;

#[test]
fn a_later_host_time_projects_forward_from_the_reference() {
    assert_eq!(
        project_clock(1_000, 10.0, FREQ, 10.5),
        Ok(1_000 + 42_000_000)
    );
}

#[test]
fn a_host_time_just_before_the_reference_projects_to_the_reference_clock() {
    assert_eq!(project_clock(1_000, 10.0, FREQ, 9.999), Ok(1_000));
}

#[test]
fn a_host_time_a_full_regression_window_before_the_reference_is_still_allowed() {
    let at_the_bound = 10.0 - NEGATIVE_PROJECTION_BOUND_SECS;
    assert_eq!(project_clock(1_000, 10.0, FREQ, at_the_bound), Ok(1_000));
}

#[test]
fn a_host_time_beyond_the_bound_is_refused_with_all_three_numbers() {
    let host = 10.0 - NEGATIVE_PROJECTION_BOUND_SECS - 0.001;
    let error = project_clock(1_000, 10.0, FREQ, host).unwrap_err();
    assert_eq!(error.host_secs, host);
    assert_eq!(error.clock_offset, 10.0);
    assert!(error.delta_secs < -NEGATIVE_PROJECTION_BOUND_SECS);
    let text = error.to_string();
    assert!(text.contains("beyond"), "{text}");
}

#[test]
fn a_zero_host_time_against_a_record_in_a_later_epoch_is_refused() {
    assert!(project_clock(1_000, 123_456.0, FREQ, 0.0).is_err());
}

#[test]
fn a_nan_host_time_is_refused() {
    assert!(project_clock(1_000, 10.0, FREQ, f64::NAN).is_err());
}
