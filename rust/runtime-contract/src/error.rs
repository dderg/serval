#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaultCode {
    None = 0,

    QueueFull = -1,
    InvalidCurve = -2,
    InvalidHandle = -3,
    InvalidDuration = -4,
    InvalidKinematics = -5,
    NullPtr = -6,
    NotInit = -7,
    FaultLatched = -8,
    Internal = -9,

    BadCrc = -100,
    FramingViolation = -101,
    Disconnect = -102,
    ProtocolVersionUnsupported = -103,

    ClockSyncQuality = -110,
    ClockSyncTimeout = -111,

    ArmTimeout = -120,
    ArmRejected = -121,
    CrossMcuDesync = -122,

    Underrun = -130,
    QueueOverrun = -131,
    LivenessStalled = -132,
    TraceOverflow = -133,

    StreamStateViolation = -140,
    SegmentIdNonMonotonic = -141,

    TStartInPast = -150,
    TEndBeforeTStart = -151,
    SegmentTooShort = -152,
    SegmentTooLong = -153,

    InvalidCurveHandle = -160,
    CurveReloadRejected = -161,
    CurveFormatInvalid = -162,

    NanInfOutput = -170,
    BoundaryLoopOverflow = -171,
    InternalInvariant = -172,

    StepBurstExceeded = -21,
    ZeroDurationSegment = -22,
    HomingTrip = -23,
    CapabilityMissing = -24,
    NoStep = -25,
    InvalidArg = -26,

    InvalidPhaseAxisCount = -27,
    PhaseBusReentrant = -28,

    PhaseModeNotAvailable = -29,
    CurveLoadInvalid = -30,
    MotionInProgress = -31,

    HostDisconnect = -200,
    HostRetransmitExhausted = -201,
    HostDispatcherTimeout = -202,
    EthercatEndpointDied = -203,

    StepQueueOverflow = -300,
    SpiQueueOverflow = -301,
    MathNonFinite = -302,
    SampleRateMisconfigured = -304,
    PositionCountOverflow = -305,
    JogParametersInvalid = -306,
    StepRateExceedsMcuCeiling = -307,
    StepsPerSampleExceeded = -310,
    TickIntervalExceeded = -311,
    PhaseMotorUnmapped = -313,
    OverlayUnsupported = -314,
    SampleRunLate = -317,
    SampleRingUnderrun = -318,
    SampleRingFull = -319,
    SampleLaneUnknown = -320,
    SampleRunRejected = -321,
    SampleBarrierOverflow = -322,
}

impl FaultCode {
    #[inline]
    pub const fn as_i32(self) -> i32 {
        self as i32
    }

    /// Cast to u16 for the `runtime_status` and `runtime_fault` wire formats.
    /// Wraps the negative i32 through i16 then u16 so the host can
    /// sign-extend back to i32 if it wants.
    #[inline]
    #[allow(clippy::cast_sign_loss)] // intentional: negative i16 → u16 wire encoding
    pub const fn as_u16(self) -> u16 {
        (self as i32 as i16) as u16
    }

    /// Reconstruct a [`FaultCode`] from its sign-wrapped `u16` wire encoding.
    ///
    /// # Examples
    ///
    /// ```
    /// # use runtime_contract::error::FaultCode;
    /// assert_eq!(FaultCode::from_u16(0), Some(FaultCode::None));
    /// assert_eq!(FaultCode::from_u16(-310i16 as u16), Some(FaultCode::StepsPerSampleExceeded));
    /// assert_eq!(FaultCode::from_u16(-311i16 as u16), Some(FaultCode::TickIntervalExceeded));
    /// assert_eq!(FaultCode::from_u16(-313i16 as u16), Some(FaultCode::PhaseMotorUnmapped));
    /// assert_eq!(FaultCode::from_u16(-314i16 as u16), Some(FaultCode::OverlayUnsupported));
    /// assert_eq!(FaultCode::from_u16(1), None);
    /// ```
    #[allow(clippy::cast_possible_wrap)] // intentional: sign-extend u16 → i16 → i32
    pub fn from_u16(v: u16) -> Option<Self> {
        let i = i32::from(v as i16);
        Some(match i {
            0 => Self::None,
            -1 => Self::QueueFull,
            -2 => Self::InvalidCurve,
            -3 => Self::InvalidHandle,
            -4 => Self::InvalidDuration,
            -5 => Self::InvalidKinematics,
            -6 => Self::NullPtr,
            -7 => Self::NotInit,
            -8 => Self::FaultLatched,
            -9 => Self::Internal,
            -21 => Self::StepBurstExceeded,
            -22 => Self::ZeroDurationSegment,
            -23 => Self::HomingTrip,
            -24 => Self::CapabilityMissing,
            -25 => Self::NoStep,
            -26 => Self::InvalidArg,
            -27 => Self::InvalidPhaseAxisCount,
            -28 => Self::PhaseBusReentrant,
            -29 => Self::PhaseModeNotAvailable,
            -30 => Self::CurveLoadInvalid,
            -31 => Self::MotionInProgress,
            -100 => Self::BadCrc,
            -101 => Self::FramingViolation,
            -102 => Self::Disconnect,
            -103 => Self::ProtocolVersionUnsupported,
            -110 => Self::ClockSyncQuality,
            -111 => Self::ClockSyncTimeout,
            -120 => Self::ArmTimeout,
            -121 => Self::ArmRejected,
            -122 => Self::CrossMcuDesync,
            -130 => Self::Underrun,
            -131 => Self::QueueOverrun,
            -132 => Self::LivenessStalled,
            -133 => Self::TraceOverflow,
            -140 => Self::StreamStateViolation,
            -141 => Self::SegmentIdNonMonotonic,
            -150 => Self::TStartInPast,
            -151 => Self::TEndBeforeTStart,
            -152 => Self::SegmentTooShort,
            -153 => Self::SegmentTooLong,
            -160 => Self::InvalidCurveHandle,
            -161 => Self::CurveReloadRejected,
            -162 => Self::CurveFormatInvalid,
            -170 => Self::NanInfOutput,
            -171 => Self::BoundaryLoopOverflow,
            -172 => Self::InternalInvariant,
            -200 => Self::HostDisconnect,
            -201 => Self::HostRetransmitExhausted,
            -202 => Self::HostDispatcherTimeout,
            -203 => Self::EthercatEndpointDied,
            -300 => Self::StepQueueOverflow,
            -301 => Self::SpiQueueOverflow,
            -302 => Self::MathNonFinite,
            -304 => Self::SampleRateMisconfigured,
            -305 => Self::PositionCountOverflow,
            -306 => Self::JogParametersInvalid,
            -307 => Self::StepRateExceedsMcuCeiling,
            -310 => Self::StepsPerSampleExceeded,
            -311 => Self::TickIntervalExceeded,
            -313 => Self::PhaseMotorUnmapped,
            -314 => Self::OverlayUnsupported,
            -317 => Self::SampleRunLate,
            -318 => Self::SampleRingUnderrun,
            -319 => Self::SampleRingFull,
            -320 => Self::SampleLaneUnknown,
            -321 => Self::SampleRunRejected,
            -322 => Self::SampleBarrierOverflow,
            _ => return None,
        })
    }

    /// Human-readable variant name for use in structured log output.
    ///
    /// # Examples
    ///
    /// ```
    /// # use runtime_contract::error::FaultCode;
    /// assert_eq!(FaultCode::None.code_name(), "None");
    /// assert_eq!(FaultCode::StepsPerSampleExceeded.code_name(), "StepsPerSampleExceeded");
    /// assert_eq!(FaultCode::TickIntervalExceeded.code_name(), "TickIntervalExceeded");
    /// assert_eq!(FaultCode::OverlayUnsupported.code_name(), "OverlayUnsupported");
    /// ```
    pub fn code_name(self) -> &'static str {
        match self {
            Self::None => "None",
            Self::QueueFull => "QueueFull",
            Self::InvalidCurve => "InvalidCurve",
            Self::InvalidHandle => "InvalidHandle",
            Self::InvalidDuration => "InvalidDuration",
            Self::InvalidKinematics => "InvalidKinematics",
            Self::NullPtr => "NullPtr",
            Self::NotInit => "NotInit",
            Self::FaultLatched => "FaultLatched",
            Self::Internal => "Internal",
            Self::StepBurstExceeded => "StepBurstExceeded",
            Self::ZeroDurationSegment => "ZeroDurationSegment",
            Self::HomingTrip => "HomingTrip",
            Self::CapabilityMissing => "CapabilityMissing",
            Self::NoStep => "NoStep",
            Self::InvalidArg => "InvalidArg",
            Self::InvalidPhaseAxisCount => "InvalidPhaseAxisCount",
            Self::PhaseBusReentrant => "PhaseBusReentrant",
            Self::PhaseModeNotAvailable => "PhaseModeNotAvailable",
            Self::CurveLoadInvalid => "CurveLoadInvalid",
            Self::MotionInProgress => "MotionInProgress",
            Self::BadCrc => "BadCrc",
            Self::FramingViolation => "FramingViolation",
            Self::Disconnect => "Disconnect",
            Self::ProtocolVersionUnsupported => "ProtocolVersionUnsupported",
            Self::ClockSyncQuality => "ClockSyncQuality",
            Self::ClockSyncTimeout => "ClockSyncTimeout",
            Self::ArmTimeout => "ArmTimeout",
            Self::ArmRejected => "ArmRejected",
            Self::CrossMcuDesync => "CrossMcuDesync",
            Self::Underrun => "Underrun",
            Self::QueueOverrun => "QueueOverrun",
            Self::LivenessStalled => "LivenessStalled",
            Self::TraceOverflow => "TraceOverflow",
            Self::StreamStateViolation => "StreamStateViolation",
            Self::SegmentIdNonMonotonic => "SegmentIdNonMonotonic",
            Self::TStartInPast => "TStartInPast",
            Self::TEndBeforeTStart => "TEndBeforeTStart",
            Self::SegmentTooShort => "SegmentTooShort",
            Self::SegmentTooLong => "SegmentTooLong",
            Self::InvalidCurveHandle => "InvalidCurveHandle",
            Self::CurveReloadRejected => "CurveReloadRejected",
            Self::CurveFormatInvalid => "CurveFormatInvalid",
            Self::NanInfOutput => "NanInfOutput",
            Self::BoundaryLoopOverflow => "BoundaryLoopOverflow",
            Self::InternalInvariant => "InternalInvariant",
            Self::HostDisconnect => "HostDisconnect",
            Self::HostRetransmitExhausted => "HostRetransmitExhausted",
            Self::HostDispatcherTimeout => "HostDispatcherTimeout",
            Self::EthercatEndpointDied => "EthercatEndpointDied",
            Self::StepQueueOverflow => "StepQueueOverflow",
            Self::SpiQueueOverflow => "SpiQueueOverflow",
            Self::MathNonFinite => "MathNonFinite",
            Self::SampleRateMisconfigured => "SampleRateMisconfigured",
            Self::PositionCountOverflow => "PositionCountOverflow",
            Self::JogParametersInvalid => "JogParametersInvalid",
            Self::StepRateExceedsMcuCeiling => "StepRateExceedsMcuCeiling",
            Self::StepsPerSampleExceeded => "StepsPerSampleExceeded",
            Self::TickIntervalExceeded => "TickIntervalExceeded",
            Self::PhaseMotorUnmapped => "PhaseMotorUnmapped",
            Self::OverlayUnsupported => "OverlayUnsupported",
            Self::SampleRunLate => "SampleRunLate",
            Self::SampleRingUnderrun => "SampleRingUnderrun",
            Self::SampleRingFull => "SampleRingFull",
            Self::SampleLaneUnknown => "SampleLaneUnknown",
            Self::SampleRunRejected => "SampleRunRejected",
            Self::SampleBarrierOverflow => "SampleBarrierOverflow",
        }
    }
}

#[cfg(test)]
mod tests;
