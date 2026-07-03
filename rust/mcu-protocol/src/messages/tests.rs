use super::roundtrip;
use super::*;

/// One valid variable-length wire entry: `PIECE_WIRE_HEADER_LEN + 4 * coeff_count`
/// bytes, filled with `fill` except for the `coeff_count` byte at offset 13.
fn one_piece(fill: u8, coeff_count: u8) -> Vec<u8> {
    let mut v = vec![fill; PIECE_WIRE_HEADER_LEN + 4 * coeff_count as usize];
    v[13] = coeff_count;
    v
}

#[test]
fn message_kind_round_trips_via_u16() {
    for &k in &[
        MessageKind::Identify,
        MessageKind::IdentifyResponse,
        MessageKind::ConfigureAxes,
        MessageKind::ConfigureAxesResponse,
        MessageKind::QueryRuntimeCaps,
        MessageKind::RuntimeCapsResponse,
        MessageKind::PushPieces,
        MessageKind::PushPiecesResponse,
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
    ] {
        assert_eq!(MessageKind::from_u16(k.as_u16()), Some(k));
    }
    assert_eq!(MessageKind::from_u16(0x0010), None); // LoadCurveCubic
    assert_eq!(MessageKind::from_u16(0x0011), None); // LoadCurveResponse
    assert_eq!(MessageKind::from_u16(0x0020), None); // PushSegment
    assert_eq!(MessageKind::from_u16(0x0021), None); // PushSegmentResponse
    assert_eq!(MessageKind::from_u16(0x0050), None); // ResetCurvePool
    assert_eq!(MessageKind::from_u16(0x0051), None); // ResetCurvePoolResponse
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
fn runtime_caps_response_new_format() {
    let msg = RuntimeCapsResponse {
        total_piece_memory: 63488,
    };
    let mut buf = Vec::new();
    msg.encode(&mut buf);
    assert_eq!(buf.len(), 4);
    let mut cursor = Cursor::new(&buf);
    let decoded = RuntimeCapsResponse::decode_from(&mut cursor).unwrap();
    assert_eq!(decoded.total_piece_memory, 63488);
}

#[test]
fn status_heartbeat_roundtrip_empty() {
    let msg = StatusHeartbeat {
        engine_state: 0,
        fault_code: 0,
        retired_counts: vec![],
        ff_saturation_count: 0,
    };
    let mut buf = Vec::new();
    msg.encode(&mut buf);
    assert_eq!(buf.len(), 8);
    let mut cursor = Cursor::new(&buf);
    let decoded = StatusHeartbeat::decode_from(&mut cursor).unwrap();
    assert_eq!(decoded.retired_counts.len(), 0);
    assert_eq!(decoded.ff_saturation_count, 0);
}

#[test]
fn status_heartbeat_roundtrip_with_axes() {
    let msg = StatusHeartbeat {
        engine_state: 1,
        fault_code: 0,
        retired_counts: vec![42, 42, 10, 5],
        ff_saturation_count: 7,
    };
    let mut buf = Vec::new();
    msg.encode(&mut buf);
    assert_eq!(buf.len(), 24);
    let mut cursor = Cursor::new(&buf);
    let decoded = StatusHeartbeat::decode_from(&mut cursor).unwrap();
    assert_eq!(decoded.engine_state, 1);
    assert_eq!(decoded.fault_code, 0);
    assert_eq!(decoded.retired_counts, vec![42, 42, 10, 5]);
    assert_eq!(decoded.ff_saturation_count, 7);
}

#[test]
fn status_heartbeat_short_frame_missing_ff_saturation_is_decode_error() {
    let msg = StatusHeartbeat {
        engine_state: 2,
        fault_code: 0,
        retired_counts: vec![99],
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
fn push_pieces_single_axis_round_trips() {
    // axis_count(1) + block header(8) + 1 piece(16 + 4*4 = 32) = 41 bytes.
    let msg = PushPieces::single(2, 1, 41, 5000, one_piece(0xAB, 4));
    let buf = msg.encoded_to_vec();
    assert_eq!(buf.len(), 1 + 8 + 32);
    assert_eq!(buf[0], 1, "leading axis_count byte");
    let decoded = PushPieces::decode(&buf).expect("decode ok");
    assert_eq!(decoded, msg);
    let a = &decoded.axes[0];
    assert_eq!(
        (a.axis_idx, a.piece_count, a.start_slot, a.new_head),
        (2, 1, 41, 5000)
    );
    assert_eq!(a.pieces_bytes, one_piece(0xAB, 4));
}

#[test]
fn push_pieces_three_axes_round_trip() {
    let msg = PushPieces {
        axes: vec![
            AxisPieces {
                axis_idx: 0,
                piece_count: 1,
                start_slot: 0,
                new_head: 1,
                pieces_bytes: one_piece(0x10, 4),
            },
            AxisPieces {
                axis_idx: 1,
                piece_count: 2,
                start_slot: 4,
                new_head: 6,
                pieces_bytes: [one_piece(0x20, 4), one_piece(0x20, 4)].concat(),
            },
            AxisPieces {
                axis_idx: 2,
                piece_count: 1,
                start_slot: 7,
                new_head: 8,
                pieces_bytes: one_piece(0x30, 4),
            },
        ],
    };
    let buf = msg.encoded_to_vec();
    // 1 + (8+32) + (8+64) + (8+32) = 153 bytes.
    assert_eq!(buf.len(), 1 + (8 + 32) + (8 + 64) + (8 + 32));
    assert_eq!(buf[0], 3);
    assert_eq!(PushPieces::decode(&buf).expect("decode ok"), msg);
}

#[test]
fn push_pieces_three_axes_one_piece_fits_frame_budget() {
    let block = |axis| AxisPieces {
        axis_idx: axis,
        piece_count: 1,
        start_slot: 0,
        new_head: 1,
        pieces_bytes: one_piece(0, 4),
    };
    let msg = PushPieces {
        axes: vec![block(0), block(1), block(2)],
    };
    assert!(
        msg.encoded_to_vec().len() <= PIECE_FRAME_PAYLOAD_MAX,
        "3 axes x 1 piece must fit the shared MCU frame budget"
    );
}

#[test]
fn max_pieces_per_axis_is_at_least_one_for_realistic_axis_counts() {
    // `ConfigureAxes::steps_per_mm` fixes the dispatcher at 4 axes — the
    // realistic single-MCU axis count this budget must serve.
    for n in 1..=4u8 {
        assert!(
            max_pieces_per_axis(n) >= 1,
            "axis_count {n} must admit at least one piece per axis within the frame budget"
        );
        // And a frame built to that cap must actually fit, sized at the
        // worst-case (8-coefficient, PIECE_WIRE_MAX_LEN) entry.
        let pc = max_pieces_per_axis(n) as u8;
        let axes = (0..n)
            .map(|axis| AxisPieces {
                axis_idx: axis,
                piece_count: pc,
                start_slot: 0,
                new_head: u32::from(pc),
                pieces_bytes: vec![0; PIECE_WIRE_MAX_LEN * pc as usize],
            })
            .collect();
        assert!(PushPieces { axes }.encoded_to_vec().len() <= PIECE_FRAME_PAYLOAD_MAX);
    }
}

#[test]
fn max_pieces_per_axis_saturates_to_zero_beyond_the_worst_case_budget() {
    // At `PIECE_WIRE_MAX_LEN`-sized (8-coefficient) worst-case entries, 5+
    // axes on one frame can no longer fit even one piece per axis — the
    // documented "too many axes for the buffer" case.
    for n in 5..=6u8 {
        assert_eq!(
            max_pieces_per_axis(n),
            0,
            "axis_count {n} must saturate to 0 under the worst-case entry budget"
        );
    }
}

#[test]
fn push_pieces_decode_axis_count_zero_is_err() {
    assert_eq!(
        PushPieces::decode(&[0u8]).unwrap_err(),
        DecodeError::EmptyArray {
            field: "PushPieces.axes"
        }
    );
}

#[test]
fn push_pieces_decode_duplicate_axis_idx_is_err() {
    // axis_count=2; two complete blocks both axis_idx=1, piece_count=0.
    let mut buf = vec![2u8];
    for _ in 0..2 {
        buf.extend_from_slice(&[1, 0]); // axis_idx=1, piece_count=0
        buf.extend_from_slice(&0u16.to_le_bytes()); // start_slot
        buf.extend_from_slice(&0u32.to_le_bytes()); // new_head
    }
    assert_eq!(
        PushPieces::decode(&buf).unwrap_err(),
        DecodeError::DuplicateField {
            field: "PushPieces.axis_idx"
        }
    );
}

#[test]
fn push_pieces_decode_truncated_is_err() {
    let full = PushPieces::single(0, 1, 0, 0, one_piece(0xCD, 4)).encoded_to_vec();
    assert!(
        matches!(
            PushPieces::decode(&full[..full.len() - 5]),
            Err(DecodeError::UnexpectedEof)
        ),
        "a frame missing coefficient bytes must fail with UnexpectedEof, not read garbage"
    );
}

#[test]
fn push_pieces_decode_bad_coeff_count_is_err() {
    let full = PushPieces::single(0, 1, 0, 0, one_piece(0, 0)).encoded_to_vec();
    assert_eq!(
        PushPieces::decode(&full).unwrap_err(),
        DecodeError::BadCoeffCount { raw: 0 }
    );
}

#[test]
fn push_pieces_decode_coeff_count_above_max_is_err() {
    let mut bytes = one_piece(0, 1);
    bytes[13] = MAX_PIECE_COEFFS as u8 + 1;
    let full = PushPieces::single(0, 1, 0, 0, bytes).encoded_to_vec();
    assert_eq!(
        PushPieces::decode(&full).unwrap_err(),
        DecodeError::BadCoeffCount {
            raw: MAX_PIECE_COEFFS as u8 + 1
        }
    );
}

#[test]
fn push_pieces_response_frame_level_round_trips() {
    // result(i32) | arrival_clock(u64) | axis_count(u8) | per-axis(axis_idx u8 + front_start_time u64).
    let msg = PushPiecesResponse {
        result: -2,
        arrival_clock: 0x0102_0304_0506_0708_u64,
        axes: vec![
            AxisDiag {
                axis_idx: 0,
                front_start_time: 0xDEAD_BEEF_CAFE_1234,
            },
            AxisDiag {
                axis_idx: 2,
                front_start_time: 0x1111_2222_3333_4444,
            },
        ],
    };
    let buf = msg.encoded_to_vec();
    assert_eq!(buf.len(), 4 + 8 + 1 + 2 * (1 + 8));
    assert_eq!(&buf[0..4], &0xFFFF_FFFE_u32.to_le_bytes());
    assert_eq!(buf[12], 2, "axis_count byte");
    assert_eq!(PushPiecesResponse::decode(&buf).expect("decode ok"), msg);
}

#[test]
fn push_pieces_response_single_helper_round_trips() {
    let msg = PushPiecesResponse::single(0, 7, 1, 9000);
    let decoded = PushPiecesResponse::decode(&msg.encoded_to_vec()).expect("decode ok");
    assert_eq!(decoded, msg);
    assert_eq!(
        decoded.axes[0],
        AxisDiag {
            axis_idx: 1,
            front_start_time: 9000
        }
    );
}

#[test]
fn push_pieces_kind_in_message_kind_table() {
    assert_eq!(MessageKind::from_u16(0x0060), Some(MessageKind::PushPieces));
    assert_eq!(
        MessageKind::from_u16(0x0061),
        Some(MessageKind::PushPiecesResponse)
    );
    assert_eq!(MessageKind::PushPieces.as_u16(), 0x0060);
    assert_eq!(MessageKind::PushPiecesResponse.as_u16(), 0x0061);
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
        slot: 2,
        following_error_counts: 8192,
        max_torque_tenth_pct: 500,
    };
    let bytes = msg.encoded_to_vec();
    let decoded = SetDriveLimits::decode(&bytes).unwrap();
    assert_eq!(decoded, msg);
}

#[test]
fn restore_drive_limits_round_trips_with_slot() {
    let msg = RestoreDriveLimits { slot: 2 };
    let bytes = msg.encoded_to_vec();
    assert_eq!(bytes.len(), 1);
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
