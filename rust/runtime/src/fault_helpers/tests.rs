use super::*;

#[test]
fn position_count_overflow_publishes_code_and_detail() {
    let shared = SharedState::new();
    raise_position_count_overflow(&shared, 1);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::PositionCountOverflow.as_i32()
    );
    assert_eq!(shared.fault_detail.load(Ordering::Acquire), 0x0001_0000);
}

#[test]
fn math_non_finite_publishes_code_and_detail() {
    let shared = SharedState::new();
    raise_math_non_finite(&shared, 3);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::MathNonFinite.as_i32()
    );
    assert_eq!(shared.fault_detail.load(Ordering::Acquire), 0x0003_0000);
}

#[test]
fn phase_mode_not_available_publishes_code_and_detail() {
    let shared = SharedState::new();
    raise_phase_mode_not_available(&shared, 1);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::PhaseModeNotAvailable.as_i32()
    );
    assert_eq!(shared.fault_detail.load(Ordering::Acquire), 0x0001_0000);
}

#[test]
fn jog_parameters_invalid_publishes_code_and_zero_detail() {
    let shared = SharedState::new();
    raise_jog_parameters_invalid(&shared);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::JogParametersInvalid.as_i32()
    );
    assert_eq!(shared.fault_detail.load(Ordering::Acquire), 0);
}

#[test]
fn tick_interval_exceeded_publishes_code_and_saturated_detail() {
    let shared = SharedState::new();
    raise_tick_interval_exceeded(&shared, 42);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::TickIntervalExceeded.as_i32()
    );
    assert_eq!(shared.fault_detail.load(Ordering::Acquire), 42);

    let shared2 = SharedState::new();
    raise_tick_interval_exceeded(&shared2, 0x1_0000);
    assert_eq!(shared2.fault_detail.load(Ordering::Acquire), 0xFFFF);
}

#[test]
fn internal_invariant_publishes_code_axis_and_reason() {
    let shared = SharedState::new();
    raise_internal_invariant(&shared, 2, 7);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::InternalInvariant.as_i32()
    );
    assert_eq!(shared.fault_detail.load(Ordering::Acquire), (2 << 16) | 7);
}

#[test]
fn overlay_unsupported_publishes_code_and_detail() {
    let shared = SharedState::new();
    raise_overlay_unsupported(&shared, 2, 0b0000_0010);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::OverlayUnsupported.as_i32()
    );
    assert_eq!(
        shared.fault_detail.load(Ordering::Acquire),
        (2 << 16) | 0b0000_0010
    );
}

#[test]
fn emit_fault_log_stub_does_not_panic() {
    emit_fault_log(FaultCode::StepsPerSampleExceeded, 0x1_0000);
}
