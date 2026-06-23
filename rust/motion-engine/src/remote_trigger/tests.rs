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
