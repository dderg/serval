#![allow(clippy::expect_used, clippy::unwrap_used)]

use super::*;

#[test]
fn fault_code_as_u16_round_trips_negative_codes() {
    // -160 (InvalidCurveHandle) → as_i16 = -160 → as u16 = 0xFF60.
    // Host sign-extends 0xFF60 back through i16 → -160. clippy::cast_sign_loss
    // is the whole point — the wire format is u16 and the host reverses it.
    let code = FaultCode::InvalidCurveHandle.as_u16();
    #[allow(clippy::cast_sign_loss)]
    let expected = (-160_i16) as u16;
    assert_eq!(code, expected);
}

#[test]
fn fault_code_from_u16_round_trip_positive_zero() {
    assert_eq!(FaultCode::from_u16(0), Some(FaultCode::None));
}

#[test]
fn fault_code_from_u16_sign_wrap_steps_per_sample_exceeded() {
    // -310 as i16 = -310; -310i16 as u16 = 65226 = 0xFECA
    let wire = FaultCode::StepsPerSampleExceeded.as_u16();
    assert_eq!(wire, 0xFECA);
    assert_eq!(
        FaultCode::from_u16(wire),
        Some(FaultCode::StepsPerSampleExceeded)
    );
}

#[test]
fn fault_code_from_u16_sign_wrap_tick_interval_exceeded() {
    // -311 as i16 = -311; -311i16 as u16 = 65225 = 0xFEC9
    let wire = FaultCode::TickIntervalExceeded.as_u16();
    assert_eq!(wire, 0xFEC9);
    assert_eq!(
        FaultCode::from_u16(wire),
        Some(FaultCode::TickIntervalExceeded)
    );
}

#[test]
fn fault_code_from_u16_sign_wrap_host_disconnect() {
    let wire = FaultCode::HostDisconnect.as_u16();
    assert_eq!(FaultCode::from_u16(wire), Some(FaultCode::HostDisconnect));
}

#[test]
fn fault_code_from_u16_unknown_returns_none() {
    assert_eq!(FaultCode::from_u16(0x1234), None);
}

#[test]
fn code_name_steps_per_sample_exceeded() {
    assert_eq!(
        FaultCode::StepsPerSampleExceeded.code_name(),
        "StepsPerSampleExceeded"
    );
}

#[test]
fn code_name_none() {
    assert_eq!(FaultCode::None.code_name(), "None");
}

#[test]
fn code_name_tick_interval_exceeded() {
    assert_eq!(
        FaultCode::TickIntervalExceeded.code_name(),
        "TickIntervalExceeded"
    );
}

#[test]
fn from_u16_round_trip_all_variants() {
    let all_codes = [
        FaultCode::None,
        FaultCode::QueueFull,
        FaultCode::InvalidCurve,
        FaultCode::InvalidHandle,
        FaultCode::InvalidDuration,
        FaultCode::InvalidKinematics,
        FaultCode::NullPtr,
        FaultCode::NotInit,
        FaultCode::FaultLatched,
        FaultCode::Internal,
        FaultCode::StepBurstExceeded,
        FaultCode::ZeroDurationSegment,
        FaultCode::HomingTrip,
        FaultCode::CapabilityMissing,
        FaultCode::NoStep,
        FaultCode::InvalidArg,
        FaultCode::InvalidPhaseAxisCount,
        FaultCode::PhaseBusReentrant,
        FaultCode::PhaseModeNotAvailable,
        FaultCode::CurveLoadInvalid,
        FaultCode::MotionInProgress,
        FaultCode::BadCrc,
        FaultCode::FramingViolation,
        FaultCode::Disconnect,
        FaultCode::ProtocolVersionUnsupported,
        FaultCode::ClockSyncQuality,
        FaultCode::ClockSyncTimeout,
        FaultCode::ArmTimeout,
        FaultCode::ArmRejected,
        FaultCode::CrossMcuDesync,
        FaultCode::Underrun,
        FaultCode::QueueOverrun,
        FaultCode::LivenessStalled,
        FaultCode::TraceOverflow,
        FaultCode::StreamStateViolation,
        FaultCode::SegmentIdNonMonotonic,
        FaultCode::TStartInPast,
        FaultCode::TEndBeforeTStart,
        FaultCode::SegmentTooShort,
        FaultCode::SegmentTooLong,
        FaultCode::InvalidCurveHandle,
        FaultCode::CurveReloadRejected,
        FaultCode::CurveFormatInvalid,
        FaultCode::NanInfOutput,
        FaultCode::BoundaryLoopOverflow,
        FaultCode::InternalInvariant,
        FaultCode::HostDisconnect,
        FaultCode::HostRetransmitExhausted,
        FaultCode::HostDispatcherTimeout,
        FaultCode::EthercatEndpointDied,
        FaultCode::StepQueueOverflow,
        FaultCode::SpiQueueOverflow,
        FaultCode::MathNonFinite,
        FaultCode::SampleRateMisconfigured,
        FaultCode::PositionCountOverflow,
        FaultCode::JogParametersInvalid,
        FaultCode::StepRateExceedsMcuCeiling,
        FaultCode::StepsPerSampleExceeded,
        FaultCode::TickIntervalExceeded,
        FaultCode::PhaseMotorUnmapped,
        FaultCode::OverlayUnsupported,
    ];
    for code in all_codes {
        let wire = code.as_u16();
        let recovered = FaultCode::from_u16(wire)
            .expect("from_u16 must succeed for every known FaultCode variant");
        assert_eq!(recovered, code, "round-trip mismatch for {code:?}");
    }
}
