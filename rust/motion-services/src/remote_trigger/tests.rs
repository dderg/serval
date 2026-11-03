use super::{RelayAction, relay_decision, relay_trip_clock};

#[test]
fn non_terminal_report_is_ignored() {
    assert_eq!(relay_decision(Some(1), false), RelayAction::Ignore);
}

#[test]
fn terminal_report_fires() {
    assert_eq!(relay_decision(Some(0), false), RelayAction::Fire);
}

#[test]
fn second_terminal_report_is_ignored() {
    assert_eq!(relay_decision(Some(0), true), RelayAction::Ignore);
}

#[test]
fn malformed_report_without_can_trigger_is_ignored() {
    assert_eq!(relay_decision(None, false), RelayAction::Ignore);
}

#[test]
fn nonzero_report_clock_expands_against_reference() {
    let reference = 0x1_0000_1000;
    let clock32_just_below_reference_low32 = 0x0000_0F00;
    assert_eq!(
        relay_trip_clock(clock32_just_below_reference_low32, reference),
        0x1_0000_0F00
    );
}

#[test]
fn clock32_ahead_of_reference_expands_forward() {
    assert_eq!(relay_trip_clock(0x0000_2000, 0x1_0000_1000), 0x1_0000_2000);
}

#[test]
fn expansion_handles_wrap_boundary() {
    let reference_just_past_wrap = 0x2_0000_0010;
    let clock32_just_before_wrap = 0xFFFF_FF00;
    assert_eq!(
        relay_trip_clock(clock32_just_before_wrap, reference_just_past_wrap),
        0x1_FFFF_FF00
    );
}

#[test]
fn zero_clock_means_host_commanded_trigger_substitute_reference() {
    let host_commanded_trigger_clock = 0;
    let reference = 0x1_0000_1000;
    assert_eq!(
        relay_trip_clock(host_commanded_trigger_clock, reference),
        reference
    );
}

#[test]
#[should_panic(expected = "reference is stale or wrong")]
fn a_missing_reference_clock_is_refused_not_wrapped_into_a_wrong_trip_clock() {
    let _ = relay_trip_clock(0x8000_0000, 0);
}

use super::trip_clock_from_report;

#[test]
fn a_report_with_its_clock_and_a_reference_expands_normally() {
    assert_eq!(
        trip_clock_from_report(3, Some(0x0000_2000), Ok::<u64, ()>(0x1_0000_1000)),
        0x1_0000_2000
    );
}

#[test]
#[should_panic(expected = "mcu 3: trsync_state report carried no clock parameter")]
fn a_report_without_a_clock_parameter_is_refused_not_read_as_a_host_commanded_trip() {
    let _ = trip_clock_from_report(3, None, Ok::<u64, ()>(1_000));
}

#[test]
#[should_panic(expected = "mcu 3: trip relay has no clock estimate")]
fn a_missing_clock_estimate_is_refused_not_replaced_by_a_zero_reference() {
    let _ = trip_clock_from_report(3, Some(0x1000), Err("NoClockEstimate"));
}
