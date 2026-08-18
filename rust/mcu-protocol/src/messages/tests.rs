use super::roundtrip;
use super::*;

#[test]
fn message_kind_round_trips_via_u16() {
    for &k in &[
        MessageKind::Identify,
        MessageKind::IdentifyResponse,
        MessageKind::ConfigureAxes,
        MessageKind::ConfigureAxesResponse,
        MessageKind::QueryRuntimeCaps,
        MessageKind::RuntimeCapsResponse,
        MessageKind::FaultEvent,
        MessageKind::StatusHeartbeat,
        MessageKind::McuLog,
        MessageKind::ClaimHandshakeReply,
        MessageKind::ClaimHandshake,
        MessageKind::SdoRead,
        MessageKind::SdoReadResponse,
        MessageKind::SdoWrite,
        MessageKind::SdoWriteResponse,
        MessageKind::ResonanceBuzz,
        MessageKind::ResonanceBuzzResponse,
        MessageKind::ArmSensorlessEndstop,
        MessageKind::ArmSensorlessEndstopResponse,
        MessageKind::StepperSuppress,
        MessageKind::StepperSuppressResponse,
    ] {
        assert_eq!(MessageKind::from_u16(k.as_u16()), Some(k));
    }
    assert_eq!(MessageKind::from_u16(0x0010), None); // LoadCurveCubic
    assert_eq!(MessageKind::from_u16(0x0011), None); // LoadCurveResponse
    assert_eq!(MessageKind::from_u16(0x0020), None); // PushSegment
    assert_eq!(MessageKind::from_u16(0x0021), None); // PushSegmentResponse
    assert_eq!(MessageKind::from_u16(0x0050), None); // ResetCurvePool
    assert_eq!(MessageKind::from_u16(0x0051), None); // ResetCurvePoolResponse
    assert_eq!(MessageKind::from_u16(0x0060), None); // PushPieces
    assert_eq!(MessageKind::from_u16(0x0061), None); // PushPiecesResponse
    assert_eq!(MessageKind::from_u16(0x0080), None); // StatusEvent (old)
    assert_eq!(MessageKind::from_u16(0x0081), None); // CreditFreed
    assert_eq!(MessageKind::from_u16(0x0090), None); // ClaimHandshakeReply (old, relocated to 0x0043)
    assert_eq!(MessageKind::from_u16(0x0091), None); // ClaimHandshake (old, relocated to 0x0042)
    assert_eq!(MessageKind::from_u16(0xFFFF), None);
}

#[test]
fn configure_axes_roundtrip() {
    let v = ConfigureAxes {
        kinematics: 0,
        present_mask: 0b1111,
        awd_mask: 0b0011,
        invert_mask: 0b0010,
        steps_per_mm: [80.0, 80.0, 400.0, 415.0],
    };
    assert_eq!(roundtrip(&v), v);
    assert_eq!(v.encoded_to_vec().len(), 20);
    let r = ConfigureAxesResponse { result: -7 };
    assert_eq!(roundtrip(&r), r);
    assert_eq!(r.encoded_to_vec().len(), 4);
}

#[test]
fn resonance_buzz_roundtrip() {
    let v = ResonanceBuzz {
        axis_mask: 0b001,
        sign_mask: 0b010,
        freq_start_millihz: 5_000,
        freq_end_millihz: 300_000,
        amplitude_nm: 4_200,
        duration_ms: 3_000,
        ramp_ms: 300,
    };
    assert_eq!(roundtrip(&v), v);
    assert_eq!(v.encoded_to_vec().len(), 22);
    let r = ResonanceBuzzResponse { result: -1 };
    assert_eq!(roundtrip(&r), r);
    assert_eq!(r.encoded_to_vec().len(), 4);
}

#[test]
fn resonance_buzz_kind_is_not_event() {
    assert!(!MessageKind::ResonanceBuzz.is_event());
    assert!(!MessageKind::ResonanceBuzzResponse.is_event());
}

#[test]
fn set_diff_damper_roundtrip() {
    let v = SetDiffDamper {
        slot_a: 0,
        slot_b: 1,
        gain_milli: 2_500,
        clamp_tenths: 50,
        lpf_millihz: 300_000,
        lead_us: 900,
    };
    assert_eq!(roundtrip(&v), v);
    assert_eq!(v.encoded_to_vec().len(), 14);
    let r = SetDiffDamperResponse { result: -831 };
    assert_eq!(roundtrip(&r), r);
    assert_eq!(r.encoded_to_vec().len(), 4);
}

#[test]
fn set_diff_trim_roundtrip() {
    let v = SetDiffTrim {
        slot_a: 2,
        slot_b: 3,
        gain_micro: 50_000,
        clamp_um: 150,
        lpf_millihz: 25_000,
        settle_ms: 300,
    };
    assert_eq!(roundtrip(&v), v);
    assert_eq!(v.encoded_to_vec().len(), 16);
    let r = SetDiffTrimResponse { result: -851 };
    assert_eq!(roundtrip(&r), r);
    assert_eq!(r.encoded_to_vec().len(), 4);
}

#[test]
fn set_dynamics_model_roundtrip() {
    let v = SetDynamicsModel {
        slots_count: 4,
        modes_count: 2,
        frame: vec![0.25, -0.25, -0.25, -0.25, 0.25, -0.25, 0.25, 0.25],
        mass: vec![0.0123, 0.0119],
        viscous: vec![0.0045, 0.0044],
        coulomb: vec![1.2, 1.1],
        compliance: vec![1.76e-5, 7.0e-6],
        pin_mass: vec![0.02, 0.0],
        pin_zeta: vec![0.05, 0.0],
        pin_lead_us: 250.0,
        pairs: vec![DynamicsPair {
            first: 0,
            second: 1,
            direction_split: -0.125,
        }],
    };
    assert_eq!(roundtrip(&v), v);
    assert_eq!(
        v.encoded_to_vec().len(),
        2 + (8 + 2 + 2 + 2 + 2 + 2 + 2) * 4 + 4 + 1 + 6
    );
    let r = SetDynamicsModelResponse { result: -862 };
    assert_eq!(roundtrip(&r), r);
    assert_eq!(r.encoded_to_vec().len(), 4);
}

#[test]
fn set_dynamics_model_empty_pairs_roundtrip() {
    let v = SetDynamicsModel {
        slots_count: 1,
        modes_count: 1,
        frame: vec![1.0],
        mass: vec![0.01],
        viscous: vec![0.0],
        coulomb: vec![0.0],
        compliance: vec![0.0],
        pin_mass: vec![0.0],
        pin_zeta: vec![0.0],
        pin_lead_us: 0.0,
        pairs: vec![],
    };
    assert_eq!(roundtrip(&v), v);
    assert_eq!(v.encoded_to_vec().last(), Some(&0));
}

#[test]
fn set_dynamics_model_truncated_array_is_decode_error() {
    let v = SetDynamicsModel {
        slots_count: 4,
        modes_count: 2,
        frame: vec![0.25; 8],
        mass: vec![0.01; 2],
        viscous: vec![0.0; 2],
        coulomb: vec![0.0; 2],
        compliance: vec![0.0; 2],
        pin_mass: vec![0.0; 2],
        pin_zeta: vec![0.0; 2],
        pin_lead_us: 0.0,
        pairs: vec![],
    };
    let mut bytes = v.encoded_to_vec();
    bytes.truncate(bytes.len() - 8);
    let mut c = Cursor::new(&bytes);
    assert!(SetDynamicsModel::decode_from(&mut c).is_err());
}

#[test]
fn set_dynamics_model_truncated_pair_tail_is_decode_error() {
    let v = SetDynamicsModel {
        slots_count: 2,
        modes_count: 1,
        frame: vec![0.5, 0.5],
        mass: vec![0.01],
        viscous: vec![0.0],
        coulomb: vec![0.0],
        compliance: vec![0.0],
        pin_mass: vec![0.0],
        pin_zeta: vec![0.0],
        pin_lead_us: 0.0,
        pairs: vec![DynamicsPair {
            first: 0,
            second: 1,
            direction_split: 0.1,
        }],
    };
    let mut bytes = v.encoded_to_vec();
    bytes.pop();
    assert!(SetDynamicsModel::decode(&bytes).is_err());
}

#[test]
fn arm_sensorless_endstop_roundtrip() {
    let v = ArmSensorlessEndstop {
        slot: 1,
        endstop_id: 4,
        torque_trip_tenth_pct: 500,
        enable: 1,
    };
    assert_eq!(roundtrip(&v), v);
    assert_eq!(v.encoded_to_vec().len(), 5);
    let r = ArmSensorlessEndstopResponse { result: -7 };
    assert_eq!(roundtrip(&r), r);
    assert_eq!(r.encoded_to_vec().len(), 4);
}

#[test]
fn arm_sensorless_endstop_kind_is_not_event() {
    assert!(!MessageKind::ArmSensorlessEndstop.is_event());
    assert!(!MessageKind::ArmSensorlessEndstopResponse.is_event());
}

#[test]
fn fault_event_roundtrip() {
    let v = FaultEvent {
        fault_code: 0x0007,
        fault_detail: 0xBAAD_F00D,
        segment_id: 11,
    };
    assert_eq!(roundtrip(&v), v);
    assert_eq!(v.encoded_to_vec().len(), 10);
}

#[test]
fn runtime_caps_response_has_empty_body() {
    let msg = RuntimeCapsResponse {};
    let mut buf = Vec::new();
    msg.encode(&mut buf);
    assert!(buf.is_empty());
    let mut cursor = Cursor::new(&buf);
    assert_eq!(RuntimeCapsResponse::decode_from(&mut cursor).unwrap(), msg);
}

#[test]
fn status_heartbeat_roundtrip_empty() {
    let msg = StatusHeartbeat {
        engine_state: 0,
        fault_code: 0,
        retired_counts: vec![],
        playback_clocks: vec![],
        ff_saturation_count: 0,
    };
    let mut buf = Vec::new();
    msg.encode(&mut buf);
    assert_eq!(buf.len(), 8);
    let mut cursor = Cursor::new(&buf);
    let decoded = StatusHeartbeat::decode_from(&mut cursor).unwrap();
    assert_eq!(decoded.retired_counts.len(), 0);
    assert_eq!(decoded.playback_clocks.len(), 0);
    assert_eq!(decoded.ff_saturation_count, 0);
}

#[test]
fn status_heartbeat_roundtrip_with_axes() {
    let msg = StatusHeartbeat {
        engine_state: 1,
        fault_code: 0,
        retired_counts: vec![42, 42, 10, 5],
        playback_clocks: vec![900, 901, 7, 0],
        ff_saturation_count: 7,
    };
    let mut buf = Vec::new();
    msg.encode(&mut buf);
    assert_eq!(buf.len(), 56);
    let mut cursor = Cursor::new(&buf);
    let decoded = StatusHeartbeat::decode_from(&mut cursor).unwrap();
    assert_eq!(decoded.engine_state, 1);
    assert_eq!(decoded.fault_code, 0);
    assert_eq!(decoded.retired_counts, vec![42, 42, 10, 5]);
    assert_eq!(decoded.playback_clocks, vec![900, 901, 7, 0]);
    assert_eq!(decoded.ff_saturation_count, 7);
}

#[test]
fn status_heartbeat_short_frame_missing_ff_saturation_is_decode_error() {
    let msg = StatusHeartbeat {
        engine_state: 2,
        fault_code: 0,
        retired_counts: vec![99],
        playback_clocks: vec![1234],
        ff_saturation_count: 5,
    };
    let full = msg.encoded_to_vec();
    let truncated = &full[..full.len() - 5];
    assert!(
        StatusHeartbeat::decode(truncated).is_err(),
        "short frame must fail to decode"
    );
}

#[test]
fn status_heartbeat_missing_playback_clocks_is_decode_error() {
    let msg = StatusHeartbeat {
        engine_state: 1,
        fault_code: 0,
        retired_counts: vec![7, 8],
        playback_clocks: vec![100, 200],
        ff_saturation_count: 0,
    };
    let full = msg.encoded_to_vec();
    let without_clocks = {
        let mut bytes = full[..4 + 8].to_vec();
        bytes.extend_from_slice(&0u32.to_le_bytes());
        bytes
    };
    assert!(
        StatusHeartbeat::decode(&without_clocks).is_err(),
        "a heartbeat that stops after the retirement counts must not decode"
    );
}

#[test]
fn set_torque_round_trips() {
    let msg = SetTorque {
        value: 1,
        execute_at_ns: 0xDEAD_BEEF_CAFE_F00D,
    };
    let bytes = msg.encoded_to_vec();
    assert_eq!(bytes.len(), 9, "u8 + u64 = 9 bytes");
    let back = SetTorque::decode(&bytes).expect("decode");
    assert_eq!(back, msg);
}

#[test]
fn set_torque_response_round_trips() {
    let msg = SetTorqueResponse { result: -311 };
    let bytes = msg.encoded_to_vec();
    assert_eq!(bytes.len(), 4, "i32 = 4 bytes");
    let back = SetTorqueResponse::decode(&bytes).expect("decode");
    assert_eq!(back, msg);
}

#[test]
fn set_torque_kinds_have_stable_tags() {
    assert_eq!(MessageKind::SetTorque.as_u16(), 0x0070);
    assert_eq!(MessageKind::SetTorqueResponse.as_u16(), 0x0071);
    assert_eq!(MessageKind::from_u16(0x0070), Some(MessageKind::SetTorque));
    assert_eq!(
        MessageKind::from_u16(0x0071),
        Some(MessageKind::SetTorqueResponse)
    );
    assert!(!MessageKind::SetTorque.is_event());
}

#[test]
fn decode_rejects_trailing_bytes() {
    let v = FaultEvent {
        fault_code: 1,
        fault_detail: 2,
        segment_id: 3,
    };
    let mut bytes = v.encoded_to_vec();
    bytes.push(0xAA);
    match FaultEvent::decode(&bytes) {
        Err(DecodeError::TrailingBytes { remaining: 1 }) => {}
        other => panic!("expected TrailingBytes(1), got {other:?}"),
    }
}

#[test]
fn stop_round_trips_empty_body() {
    let bytes = Stop.encoded_to_vec();
    assert!(bytes.is_empty(), "Stop body is empty");
    let back = Stop::decode(&bytes).expect("decode");
    assert_eq!(back, Stop);
}

#[test]
fn stop_response_round_trips() {
    let msg = StopResponse {
        result: 0,
        discard_clock: 0x0123_4567_89AB_CDEF,
    };
    let bytes = msg.encoded_to_vec();
    assert_eq!(bytes.len(), 12, "i32 + u64 = 12 bytes");
    assert_eq!(StopResponse::decode(&bytes).expect("decode"), msg);
}

#[test]
fn resume_stream_round_trips_empty_body() {
    let bytes = ResumeStream.encoded_to_vec();
    assert!(bytes.is_empty(), "ResumeStream body is empty");
    let back = ResumeStream::decode(&bytes).expect("decode");
    assert_eq!(back, ResumeStream);
}

#[test]
fn resume_stream_response_round_trips() {
    let msg = ResumeStreamResponse { result: -142 };
    let bytes = msg.encoded_to_vec();
    assert_eq!(bytes.len(), 4, "i32 = 4 bytes");
    assert_eq!(ResumeStreamResponse::decode(&bytes).expect("decode"), msg);
}

#[test]
fn resume_stream_kinds_have_stable_tags() {
    assert_eq!(MessageKind::ResumeStream.as_u16(), 0x0078);
    assert_eq!(MessageKind::ResumeStreamResponse.as_u16(), 0x0079);
    assert_eq!(
        MessageKind::from_u16(0x0078),
        Some(MessageKind::ResumeStream)
    );
    assert_eq!(
        MessageKind::from_u16(0x0079),
        Some(MessageKind::ResumeStreamResponse)
    );
    assert!(!MessageKind::ResumeStream.is_event());
}

#[test]
fn stop_kinds_have_stable_tags() {
    assert_eq!(MessageKind::Stop.as_u16(), 0x0072);
    assert_eq!(MessageKind::StopResponse.as_u16(), 0x0073);
    assert_eq!(MessageKind::from_u16(0x0072), Some(MessageKind::Stop));
    assert_eq!(
        MessageKind::from_u16(0x0073),
        Some(MessageKind::StopResponse)
    );
    assert!(!MessageKind::Stop.is_event());
}

#[test]
fn set_drive_limits_round_trips() {
    let msg = SetDriveLimits {
        drives: vec![
            DriveLimitEntry {
                slot: 2,
                following_error_counts: 8192,
                max_torque_tenth_pct: 500,
            },
            DriveLimitEntry {
                slot: 3,
                following_error_counts: 4096,
                max_torque_tenth_pct: 300,
            },
        ],
    };
    let bytes = msg.encoded_to_vec();
    assert_eq!(bytes.len(), 1 + 2 * 7);
    let decoded = SetDriveLimits::decode(&bytes).unwrap();
    assert_eq!(decoded, msg);
}

#[test]
fn set_drive_limits_rejects_truncated_entries() {
    let msg = SetDriveLimits {
        drives: vec![DriveLimitEntry {
            slot: 0,
            following_error_counts: 1,
            max_torque_tenth_pct: 2,
        }],
    };
    let mut bytes = msg.encoded_to_vec();
    bytes.truncate(bytes.len() - 1);
    assert!(SetDriveLimits::decode(&bytes).is_err());
}

#[test]
fn restore_drive_limits_round_trips_with_slot_mask() {
    let msg = RestoreDriveLimits { slot_mask: 0b1010 };
    let bytes = msg.encoded_to_vec();
    assert_eq!(bytes.len(), 4);
    assert_eq!(RestoreDriveLimits::decode(&bytes).unwrap(), msg);
}

#[test]
fn drive_limits_responses_round_trip() {
    let r = SetDriveLimitsResponse { result: -315 };
    assert_eq!(
        SetDriveLimitsResponse::decode(&r.encoded_to_vec()).unwrap(),
        r
    );
    let r = RestoreDriveLimitsResponse { result: 0 };
    assert_eq!(
        RestoreDriveLimitsResponse::decode(&r.encoded_to_vec()).unwrap(),
        r
    );
}

#[test]
fn drive_limits_message_kinds_round_trip() {
    for kind in [
        MessageKind::SetDriveLimits,
        MessageKind::SetDriveLimitsResponse,
        MessageKind::RestoreDriveLimits,
        MessageKind::RestoreDriveLimitsResponse,
    ] {
        assert_eq!(MessageKind::from_u16(kind.as_u16()), Some(kind));
    }
}

#[test]
fn seed_servo_home_round_trips() {
    let msg = SeedServoHome {
        slot: 3,
        home_q16: -123_456,
    };
    let bytes = msg.encoded_to_vec();
    assert_eq!(bytes.len(), 5);
    assert_eq!(SeedServoHome::decode(&bytes).unwrap(), msg);

    let r = SeedServoHomeResponse { result: -801 };
    assert_eq!(
        SeedServoHomeResponse::decode(&r.encoded_to_vec()).unwrap(),
        r
    );

    for kind in [
        MessageKind::SeedServoHome,
        MessageKind::SeedServoHomeResponse,
    ] {
        assert_eq!(MessageKind::from_u16(kind.as_u16()), Some(kind));
    }
}

#[test]
fn endstop_trip_round_trips_and_is_event() {
    let msg = EndstopTrip {
        endstop_id: 3,
        trip_clock: 0x0123_4567_89AB_CDEF,
    };
    let bytes = msg.encoded_to_vec();
    assert_eq!(bytes.len(), 9, "u8 + u64 = 9 bytes");
    assert_eq!(bytes[0], 3);
    assert_eq!(&bytes[1..9], &0x0123_4567_89AB_CDEF_u64.to_le_bytes());
    assert_eq!(EndstopTrip::decode(&bytes).expect("decode"), msg);
    assert!(MessageKind::EndstopTrip.is_event());
    assert_eq!(MessageKind::EndstopTrip.as_u16(), 0x0085);
}

#[test]
fn put_get_str_round_trip() {
    use crate::codec::{Cursor, get_str, put_str};
    let mut buf = Vec::new();
    put_str(&mut buf, "servo_captures/x_20260610.scap");
    put_str(&mut buf, "");
    let mut c = Cursor::new(&buf);
    assert_eq!(get_str(&mut c).unwrap(), "servo_captures/x_20260610.scap");
    assert_eq!(get_str(&mut c).unwrap(), "");
}

#[test]
fn get_str_rejects_truncated_buffer() {
    use crate::codec::{Cursor, get_str};
    let length_prefix_overruns_payload = [10u8, 0, b'a', b'b'];
    let mut c = Cursor::new(&length_prefix_overruns_payload);
    assert!(get_str(&mut c).is_err());
}

#[test]
fn get_str_rejects_invalid_utf8() {
    use crate::codec::{Cursor, get_str};
    let buf = [2u8, 0, 0xff, 0xfe];
    let mut c = Cursor::new(&buf);
    assert!(get_str(&mut c).is_err());
}

#[test]
fn start_capture_round_trip() {
    use crate::messages::{CaptureDrive, StartCapture};
    let msg = StartCapture {
        path: "/home/pi/printer_data/logs/servo_captures/t.scap".into(),
        started_utc: "2026-06-10T12:00:00Z".into(),
        drives: vec![
            CaptureDrive {
                slot: 0,
                name: "x".into(),
            },
            CaptureDrive {
                slot: 2,
                name: "y".into(),
            },
        ],
    };
    let buf = msg.encoded_to_vec();
    assert_eq!(StartCapture::decode(&buf).unwrap(), msg);
}

#[test]
fn start_capture_rejects_empty_drive_list() {
    use crate::codec::DecodeError;
    use crate::messages::StartCapture;
    let msg = StartCapture {
        path: "/tmp/t.scap".into(),
        started_utc: "2026-06-10T12:00:00Z".into(),
        drives: vec![],
    };
    let buf = msg.encoded_to_vec();
    assert!(matches!(
        StartCapture::decode(&buf),
        Err(DecodeError::EmptyArray { .. })
    ));
}

#[test]
fn start_capture_rejects_duplicate_slot() {
    use crate::codec::DecodeError;
    use crate::messages::{CaptureDrive, StartCapture};
    let msg = StartCapture {
        path: "/tmp/t.scap".into(),
        started_utc: "2026-06-10T12:00:00Z".into(),
        drives: vec![
            CaptureDrive {
                slot: 1,
                name: "a".into(),
            },
            CaptureDrive {
                slot: 1,
                name: "b".into(),
            },
        ],
    };
    let buf = msg.encoded_to_vec();
    assert!(matches!(
        StartCapture::decode(&buf),
        Err(DecodeError::DuplicateField { .. })
    ));
}

#[test]
fn stop_capture_response_round_trip() {
    use crate::messages::StopCaptureResponse;
    let msg = StopCaptureResponse {
        result: -323,
        samples: 12_345,
        overflow_cycle: StopCaptureResponse::NO_OVERFLOW,
    };
    let buf = msg.encoded_to_vec();
    assert_eq!(StopCaptureResponse::decode(&buf).unwrap(), msg);
}

#[test]
fn get_str_zero_length_decodes_to_empty() {
    use crate::codec::{Cursor, get_str};
    let buf = [0u8, 0];
    let mut c = Cursor::new(&buf);
    assert_eq!(get_str(&mut c).unwrap(), "");
}

#[test]
fn motor_state_kinds_roundtrip() {
    assert_eq!(
        MessageKind::from_u16(0x0044),
        Some(MessageKind::QueryMotorState)
    );
    assert_eq!(
        MessageKind::from_u16(0x0045),
        Some(MessageKind::MotorStateResponse)
    );
    assert_eq!(MessageKind::QueryMotorState.as_u16(), 0x0044);
    assert_eq!(MessageKind::MotorStateResponse.as_u16(), 0x0045);
}

#[test]
fn motor_state_response_roundtrip() {
    let msg = MotorStateResponse {
        motors: vec![
            MotorSample {
                slot: 0,
                pos_q16: 123 * 65536,
                vel_q16: -45 * 65536,
            },
            MotorSample {
                slot: 2,
                pos_q16: 7,
                vel_q16: 0,
            },
        ],
    };
    let mut buf = Vec::new();
    msg.encode(&mut buf);
    assert_eq!(buf.len(), 1 + 2 * 9);
    let mut c = Cursor::new(&buf);
    let got = MotorStateResponse::decode_from(&mut c).unwrap();
    assert_eq!(got, msg);
}

#[test]
fn motor_state_response_empty_roundtrip() {
    let msg = MotorStateResponse { motors: vec![] };
    let mut buf = Vec::new();
    msg.encode(&mut buf);
    assert_eq!(buf, vec![0u8]);
    let mut c = Cursor::new(&buf);
    assert_eq!(MotorStateResponse::decode_from(&mut c).unwrap(), msg);
}

#[test]
fn capture_message_kinds_round_trip_u16() {
    use crate::messages::MessageKind;
    for (kind, raw) in [
        (MessageKind::StartCapture, 0x0068u16),
        (MessageKind::StartCaptureResponse, 0x0069),
        (MessageKind::StopCapture, 0x006A),
        (MessageKind::StopCaptureResponse, 0x006B),
    ] {
        assert_eq!(kind.as_u16(), raw);
        assert_eq!(MessageKind::from_u16(raw), Some(kind));
        assert!(!kind.is_event());
    }
}

fn lane(axis_idx: u8, sample_count: u16) -> LaneRun {
    LaneRun {
        axis_idx,
        flags: LANE_RUN_FLAG_REANCHOR,
        origin_mm_q16: -3 * 65_536,
        start_index: 9_000 + u64::from(axis_idx),
        interval_ticks: 1_600,
        sample_count,
        samples: (0..sample_count)
            .map(|i| SetpointSample {
                pos_counts: 100 * i32::from(i),
                vel_ff: -50 * i32::from(i),
                torque_ff: -(i as i16),
                acc_mm_s2: 0.25 * f32::from(i),
            })
            .collect(),
    }
}

#[test]
fn push_sample_runs_multi_lane_round_trips() {
    let msg = PushSampleRuns {
        lanes: vec![lane(0, 3), lane(1, 1), lane(2, 2)],
    };
    let buf = msg.encoded_to_vec();
    assert_eq!(
        buf.len(),
        1 + 3 * LANE_RUN_HEADER_LEN + 6 * SETPOINT_SAMPLE_LEN
    );
    assert_eq!(buf[0], 3, "leading lane_count byte");
    assert_eq!(roundtrip(&msg), msg);
}

#[test]
fn push_sample_runs_single_lane_single_sample_round_trips() {
    let msg = PushSampleRuns {
        lanes: vec![lane(4, 1)],
    };
    let buf = msg.encoded_to_vec();
    assert_eq!(buf.len(), 1 + LANE_RUN_HEADER_LEN + SETPOINT_SAMPLE_LEN);
    assert_eq!(roundtrip(&msg), msg);
}

#[test]
fn push_sample_runs_decode_zero_lane_count_is_err() {
    assert_eq!(
        PushSampleRuns::decode(&[0u8]).unwrap_err(),
        DecodeError::EmptyArray {
            field: "PushSampleRuns.lanes"
        }
    );
}

#[test]
fn push_sample_runs_decode_duplicate_axis_is_err() {
    let mut buf = PushSampleRuns {
        lanes: vec![lane(1, 1), lane(2, 1)],
    }
    .encoded_to_vec();
    buf[1 + LANE_RUN_HEADER_LEN + SETPOINT_SAMPLE_LEN] = 1;
    assert_eq!(
        PushSampleRuns::decode(&buf).unwrap_err(),
        DecodeError::DuplicateField {
            field: "PushSampleRuns.axis_idx"
        }
    );
}

#[test]
fn push_sample_runs_decode_zero_sample_count_is_err() {
    let buf = PushSampleRuns {
        lanes: vec![lane(0, 0)],
    }
    .encoded_to_vec();
    assert_eq!(buf.len(), 1 + LANE_RUN_HEADER_LEN);
    assert_eq!(
        PushSampleRuns::decode(&buf).unwrap_err(),
        DecodeError::EmptyArray {
            field: "PushSampleRuns.samples"
        }
    );
}

#[test]
fn push_sample_runs_decode_truncated_is_err() {
    let full = PushSampleRuns {
        lanes: vec![lane(0, 2)],
    }
    .encoded_to_vec();
    assert_eq!(
        PushSampleRuns::decode(&full[..full.len() - 3]).unwrap_err(),
        DecodeError::UnexpectedEof
    );
}

#[test]
fn push_sample_runs_response_round_trips() {
    let msg = PushSampleRunsResponse {
        result: 0,
        arrival_clock: 0x0102_0304_0506_0708,
        grid_index: 1_234_567,
        grid_clock: 0x1112_1314_1516_1718,
        lanes: vec![
            LaneDepth {
                axis_idx: 0,
                free_cycles: 256,
            },
            LaneDepth {
                axis_idx: 3,
                free_cycles: 0,
            },
        ],
    };
    let buf = msg.encoded_to_vec();
    assert_eq!(buf.len(), 4 + 8 + 8 + 8 + 1 + 2 * 5);
    assert_eq!(buf[28], 2, "lane_count byte");
    assert_eq!(roundtrip(&msg), msg);
}

#[test]
fn push_sample_runs_response_decode_zero_lane_count_is_err() {
    let buf = vec![0u8; 4 + 8 + 8 + 8 + 1];
    assert_eq!(
        PushSampleRunsResponse::decode(&buf).unwrap_err(),
        DecodeError::EmptyArray {
            field: "PushSampleRunsResponse.lanes"
        }
    );
}

#[test]
fn sample_grid_response_round_trips() {
    let msg = SampleGridResponse {
        executor: 1,
        cycle_ticks: 1_600,
        ring_depth_cycles: 512,
        grid_index: 77_777,
        grid_clock: 0x2122_2324_2526_2728,
    };
    let buf = msg.encoded_to_vec();
    assert_eq!(buf.len(), 1 + 4 + 4 + 8 + 8);
    assert_eq!(roundtrip(&msg), msg);
}

#[test]
fn query_sample_grid_round_trips() {
    let msg = QuerySampleGrid {};
    assert!(msg.encoded_to_vec().is_empty());
    assert_eq!(roundtrip(&msg), msg);
}

#[test]
fn sample_stream_message_kinds_round_trip_u16() {
    for (kind, raw) in [
        (MessageKind::PushSampleRuns, 0x0062u16),
        (MessageKind::PushSampleRunsResponse, 0x0063),
        (MessageKind::QuerySampleGrid, 0x0064),
        (MessageKind::SampleGridResponse, 0x0065),
    ] {
        assert_eq!(kind.as_u16(), raw);
        assert_eq!(MessageKind::from_u16(raw), Some(kind));
        assert!(!kind.is_event());
    }
}
