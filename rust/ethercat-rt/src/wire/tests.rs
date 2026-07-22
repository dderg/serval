use super::*;
use mcu_protocol::messages::{
    MotorStateResponse, ResonanceBuzz, RestoreDriveLimits, RestoreDriveLimitsResponse,
    ResumeStreamResponse, SdoRead, SdoReadResponse, SdoWrite, SdoWriteResponse, SeedServoHome,
    SeedServoHomeResponse, SetDriveLimits, SetDriveLimitsResponse, SetFfLead, SetFfLeadResponse,
    SlaveState, SlaveStatus, StartCapture, StartCaptureResponse, StopCaptureResponse, StopResponse,
};
use mcu_transport::demux::{Demuxer, Frame};
use mcu_transport::frame::decode_frame;

#[test]
fn decodes_identify_on_control_channel() {
    let payload = frame_payload(MessageKind::Identify, 1, &[3u8]);
    match decode_command(0, &payload).unwrap() {
        Command::Identify {
            correlation_id: 1,
            proto_version: 3,
        } => {}
        other => panic!("wrong variant: {other:?}"),
    }
}

#[test]
fn decodes_push_pieces_on_pieces_channel() {
    let msg = PushPieces::single(0, 0, 0, 1, vec![]);
    let payload = frame_payload(MessageKind::PushPieces, 7, &msg.encoded_to_vec());
    match decode_command(MCU_CHANNEL_PIECES, &payload).unwrap() {
        Command::PushPieces {
            correlation_id,
            msg: m,
        } => {
            assert_eq!(correlation_id, 7);
            assert_eq!(m.axes[0].axis_idx, 0);
            assert_eq!(m.axes[0].piece_count, 0);
            assert_eq!(m.axes[0].new_head, 1);
        }
        other => panic!("wrong variant: {other:?}"),
    }
}

#[test]
fn push_pieces_response_decodes_back() {
    let frame = push_pieces_response_frame(42, 0, 0, 0, 1_000_000_000);
    let (chan, payload) = decode_frame(&frame).unwrap();
    assert_eq!(chan, CHANNEL_CONTROL);
    let (hdr, body) = decode_message_header(payload).unwrap();
    assert_eq!(hdr.correlation_id, 42);
    assert_eq!(
        MessageKind::from_u16(hdr.kind_raw),
        Some(MessageKind::PushPiecesResponse)
    );
    let r = PushPiecesResponse::decode(body).unwrap();
    assert_eq!(r.result, 0);
    assert_eq!(
        r.axes.first().map(|a| a.front_start_time).unwrap_or(0),
        1_000_000_000
    );
}

#[test]
fn push_pieces_response_multi_echoes_every_axis() {
    let frame = push_pieces_response_frame_multi(7, 0, 5_000, &[(0, 111), (1, 222)]);
    let (_chan, payload) = decode_frame(&frame).unwrap();
    let (hdr, body) = decode_message_header(payload).unwrap();
    assert_eq!(hdr.correlation_id, 7);
    let r = PushPiecesResponse::decode(body).unwrap();
    assert_eq!(r.result, 0);
    assert_eq!(r.arrival_clock, 5_000);
    assert_eq!(r.axes.len(), 2);
    assert_eq!((r.axes[0].axis_idx, r.axes[0].front_start_time), (0, 111));
    assert_eq!((r.axes[1].axis_idx, r.axes[1].front_start_time), (1, 222));
}

#[test]
fn motor_state_response_multi_carries_one_sample_per_slot() {
    let frame = motor_state_response_frame_multi(9, &[(0, 1.0, 2.0), (1, -3.0, 4.0)]);
    let (_chan, payload) = decode_frame(&frame).unwrap();
    let (hdr, body) = decode_message_header(payload).unwrap();
    assert_eq!(hdr.correlation_id, 9);
    let r = MotorStateResponse::decode(body).unwrap();
    assert_eq!(r.motors.len(), 2);
    assert_eq!(r.motors[0].slot, 0);
    assert_eq!(r.motors[1].slot, 1);
    // q16 round-trip of the position fields.
    assert_eq!(r.motors[0].pos_q16, (1.0_f64 * 65536.0) as i32);
    assert_eq!(r.motors[1].pos_q16, (-3.0_f64 * 65536.0) as i32);
}

#[test]
fn claim_handshake_reply_frame_decodes() {
    let reply = ClaimHandshakeReply {
        slave_statuses: vec![SlaveStatus {
            slave_idx: 1,
            state: SlaveState::Ok,
            fault_code: 0,
        }],
    };
    let frame = claim_handshake_reply_frame(7, &reply);
    let (chan, payload) = decode_frame(&frame).unwrap();
    assert_eq!(chan, CHANNEL_CONTROL);
    let (hdr, body) = decode_message_header(payload).unwrap();
    assert_eq!(hdr.correlation_id, 7);
    assert_eq!(
        MessageKind::from_u16(hdr.kind_raw),
        Some(MessageKind::ClaimHandshakeReply)
    );
    let decoded = ClaimHandshakeReply::decode(body).unwrap();
    assert_eq!(decoded, reply);
}

#[test]
fn decode_command_yields_claim_handshake_variant() {
    let payload = frame_payload(MessageKind::ClaimHandshake, 99, &[]);
    match decode_command(0, &payload).unwrap() {
        Command::ClaimHandshake { correlation_id: 99 } => {}
        other => panic!("expected ClaimHandshake, got {other:?}"),
    }
}

#[test]
fn status_heartbeat_frame_on_events_channel() {
    let frame = status_heartbeat_frame(1, 0, &[42u32, 0u32], 0);
    let (chan, payload) = decode_frame(&frame).unwrap();
    assert_eq!(chan, CHANNEL_EVENTS);
    let (hdr, body) = decode_message_header(payload).unwrap();
    assert_eq!(
        MessageKind::from_u16(hdr.kind_raw),
        Some(MessageKind::StatusHeartbeat)
    );
    assert_eq!(hdr.correlation_id, 0);
    let hb = StatusHeartbeat::decode(body).unwrap();
    assert_eq!(hb.engine_state, 1);
    assert_eq!(hb.retired_counts, vec![42u32, 0u32]);
}

#[test]
fn decodes_set_torque_command() {
    let msg = SetTorque {
        value: 1,
        execute_at_ns: 123_456_789,
    };
    let payload = frame_payload(MessageKind::SetTorque, 7, &msg.encoded_to_vec());
    let cmd = decode_command(0, &payload[..]).expect("decode");
    match cmd {
        Command::SetTorque {
            correlation_id,
            msg: m,
        } => {
            assert_eq!(correlation_id, 7);
            assert_eq!(m.value, 1);
            assert_eq!(m.execute_at_ns, 123_456_789);
        }
        other => panic!("expected SetTorque, got {other:?}"),
    }
}

#[test]
fn decodes_resonance_buzz_command() {
    let msg = ResonanceBuzz {
        axis_mask: 0b001,
        sign_mask: 0b000,
        freq_start_millihz: 5_000,
        freq_end_millihz: 300_000,
        amplitude_nm: 4_200,
        duration_ms: 3_000,
        ramp_ms: 300,
    };
    let payload = frame_payload(MessageKind::ResonanceBuzz, 42, &msg.encoded_to_vec());
    match decode_command(0, &payload).expect("decode") {
        Command::ResonanceBuzz {
            correlation_id,
            msg: m,
        } => {
            assert_eq!(correlation_id, 42);
            assert_eq!(m, msg);
        }
        other => panic!("expected ResonanceBuzz, got {other:?}"),
    }
}

#[test]
fn resonance_buzz_response_frame_round_trips() {
    let frame = resonance_buzz_response_frame(42, 0);
    let mut demux = Demuxer::new();
    let (frames, errs) = demux.feed_slice(&frame);
    assert!(errs.is_empty());
    let Frame::Kalico { payload, .. } = &frames[0] else {
        panic!("expected kalico frame");
    };
    let (hdr, _body) = decode_message_header(payload).expect("header");
    assert_eq!(
        MessageKind::from_u16(hdr.kind_raw),
        Some(MessageKind::ResonanceBuzzResponse)
    );
}

#[test]
fn set_torque_response_frame_round_trips() {
    let frame = set_torque_response_frame(9, -312);
    let mut demux = Demuxer::new();
    let (frames, errs) = demux.feed_slice(&frame);
    assert!(errs.is_empty());
    let Frame::Kalico { payload, .. } = &frames[0] else {
        panic!("expected kalico frame");
    };
    let (hdr, body) = decode_message_header(payload).expect("header");
    assert_eq!(
        MessageKind::from_u16(hdr.kind_raw),
        Some(MessageKind::SetTorqueResponse)
    );
    assert_eq!(hdr.correlation_id, 9);
    let resp = SetTorqueResponse::decode(body).expect("body");
    assert_eq!(resp.result, -312);
}

#[test]
fn decode_start_capture_command() {
    let msg = StartCapture {
        path: "/tmp/t.scap".into(),
        started_utc: "2026-06-10T12:00:00Z".into(),
        drives: vec![mcu_protocol::messages::CaptureDrive {
            slot: 0,
            name: "x".into(),
        }],
    };
    let payload = frame_payload(MessageKind::StartCapture, 77, &msg.encoded_to_vec());
    match decode_command(0, &payload).unwrap() {
        Command::StartCapture {
            correlation_id,
            msg: m,
        } => {
            assert_eq!(correlation_id, 77);
            assert_eq!(m.path, "/tmp/t.scap");
            assert_eq!(m.started_utc, "2026-06-10T12:00:00Z");
            assert_eq!(m.drives.len(), 1);
            assert_eq!(m.drives[0].slot, 0);
            assert_eq!(m.drives[0].name, "x");
        }
        other => panic!("expected StartCapture, got {other:?}"),
    }
}

#[test]
fn decode_stop_capture_command() {
    let payload = frame_payload(MessageKind::StopCapture, 78, &[]);
    match decode_command(0, &payload).unwrap() {
        Command::StopCapture { correlation_id: 78 } => {}
        other => panic!("expected StopCapture, got {other:?}"),
    }
}

#[test]
fn start_capture_response_frame_round_trips() {
    let frame = start_capture_response_frame(11, 0);
    let mut demux = Demuxer::new();
    let (frames, errs) = demux.feed_slice(&frame);
    assert!(errs.is_empty());
    let Frame::Kalico { payload, .. } = &frames[0] else {
        panic!("expected kalico frame");
    };
    let (hdr, body) = decode_message_header(payload).expect("header");
    assert_eq!(
        MessageKind::from_u16(hdr.kind_raw),
        Some(MessageKind::StartCaptureResponse)
    );
    assert_eq!(hdr.correlation_id, 11);
    let resp = StartCaptureResponse::decode(body).expect("body");
    assert_eq!(resp.result, 0);
}

#[test]
fn stop_capture_response_frame_round_trips() {
    let frame = stop_capture_response_frame(9, -323, 1234, 567);
    let mut demux = Demuxer::new();
    let (frames, errs) = demux.feed_slice(&frame);
    assert!(errs.is_empty());
    let Frame::Kalico { payload, .. } = &frames[0] else {
        panic!("expected kalico frame");
    };
    let (hdr, body) = decode_message_header(payload).expect("header");
    assert_eq!(
        MessageKind::from_u16(hdr.kind_raw),
        Some(MessageKind::StopCaptureResponse)
    );
    assert_eq!(hdr.correlation_id, 9);
    let resp = StopCaptureResponse::decode(body).expect("body");
    assert_eq!(resp.result, -323);
    assert_eq!(resp.samples, 1234);
    assert_eq!(resp.overflow_cycle, 567);
}

#[test]
fn decodes_stop_command() {
    let payload = frame_payload(MessageKind::Stop, 11, &[]);
    match decode_command(0, &payload).unwrap() {
        Command::Stop { correlation_id: 11 } => {}
        other => panic!("expected Stop, got {other:?}"),
    }
}

#[test]
fn decodes_resume_stream_command() {
    let payload = frame_payload(MessageKind::ResumeStream, 12, &[]);
    match decode_command(0, &payload).unwrap() {
        Command::ResumeStream { correlation_id: 12 } => {}
        other => panic!("expected ResumeStream, got {other:?}"),
    }
}

#[test]
fn resume_stream_response_frame_round_trips() {
    let frame = resume_stream_response_frame(7, 0);
    let (chan, payload) = decode_frame(&frame).unwrap();
    assert_eq!(chan, CHANNEL_CONTROL);
    let (hdr, body) = decode_message_header(payload).unwrap();
    assert_eq!(hdr.correlation_id, 7);
    assert_eq!(
        MessageKind::from_u16(hdr.kind_raw),
        Some(MessageKind::ResumeStreamResponse)
    );
    assert_eq!(ResumeStreamResponse::decode(body).unwrap().result, 0);
}

#[test]
fn decodes_set_drive_limits_command() {
    let msg = SetDriveLimits {
        slot: 0,
        following_error_counts: 8192,
        max_torque_tenth_pct: 500,
    };
    let payload = frame_payload(MessageKind::SetDriveLimits, 3, &msg.encoded_to_vec());
    match decode_command(0, &payload).unwrap() {
        Command::SetDriveLimits {
            correlation_id: 3,
            msg: m,
        } => {
            assert_eq!(m.following_error_counts, 8192);
            assert_eq!(m.max_torque_tenth_pct, 500);
        }
        other => panic!("expected SetDriveLimits, got {other:?}"),
    }
}

#[test]
fn decodes_restore_drive_limits_command() {
    let payload = frame_payload(
        MessageKind::RestoreDriveLimits,
        4,
        &RestoreDriveLimits { slot: 0 }.encoded_to_vec(),
    );
    match decode_command(0, &payload).unwrap() {
        Command::RestoreDriveLimits {
            correlation_id: 4,
            slot: 0,
        } => {}
        other => panic!("expected RestoreDriveLimits, got {other:?}"),
    }
}

#[test]
fn drive_limits_response_frames_round_trip() {
    let frame = set_drive_limits_response_frame(6, -315);
    let (chan, payload) = decode_frame(&frame).unwrap();
    assert_eq!(chan, CHANNEL_CONTROL);
    let (hdr, body) = decode_message_header(payload).unwrap();
    assert_eq!(hdr.correlation_id, 6);
    assert_eq!(
        MessageKind::from_u16(hdr.kind_raw),
        Some(MessageKind::SetDriveLimitsResponse)
    );
    assert_eq!(SetDriveLimitsResponse::decode(body).unwrap().result, -315);

    let frame = restore_drive_limits_response_frame(7, 0);
    let (_, payload) = decode_frame(&frame).unwrap();
    let (hdr, body) = decode_message_header(payload).unwrap();
    assert_eq!(
        MessageKind::from_u16(hdr.kind_raw),
        Some(MessageKind::RestoreDriveLimitsResponse)
    );
    assert_eq!(RestoreDriveLimitsResponse::decode(body).unwrap().result, 0);
    assert_eq!(hdr.correlation_id, 7);
}

#[test]
fn decodes_seed_servo_home_command() {
    let msg = SeedServoHome {
        slot: 0,
        home_q16: -98_304,
    };
    let payload = frame_payload(MessageKind::SeedServoHome, 8, &msg.encoded_to_vec());
    match decode_command(0, &payload).unwrap() {
        Command::SeedServoHome {
            correlation_id: 8,
            slot: 0,
            home_q16,
        } => assert_eq!(home_q16, -98_304),
        other => panic!("expected SeedServoHome, got {other:?}"),
    }
}

#[test]
fn seed_servo_home_response_frame_round_trips() {
    let frame = seed_servo_home_response_frame(13, -801);
    let (chan, payload) = decode_frame(&frame).unwrap();
    assert_eq!(chan, CHANNEL_CONTROL);
    let (hdr, body) = decode_message_header(payload).unwrap();
    assert_eq!(hdr.correlation_id, 13);
    assert_eq!(
        MessageKind::from_u16(hdr.kind_raw),
        Some(MessageKind::SeedServoHomeResponse)
    );
    assert_eq!(SeedServoHomeResponse::decode(body).unwrap().result, -801);
}

#[test]
fn status_heartbeat_frame_carries_fault_code() {
    let frame = status_heartbeat_frame(1, 0x8611, &[5u32], 0);
    let (_, payload) = decode_frame(&frame).unwrap();
    let (_, body) = decode_message_header(payload).unwrap();
    let hb = StatusHeartbeat::decode(body).unwrap();
    assert_eq!(hb.fault_code, 0x8611);
    assert_eq!(hb.engine_state, 1);
}

#[test]
fn stop_response_frame_round_trips() {
    let frame = stop_response_frame(5, -311, 123_456_789);
    let (chan, payload) = decode_frame(&frame).unwrap();
    assert_eq!(chan, CHANNEL_CONTROL);
    let (hdr, body) = decode_message_header(payload).unwrap();
    assert_eq!(hdr.correlation_id, 5);
    assert_eq!(
        MessageKind::from_u16(hdr.kind_raw),
        Some(MessageKind::StopResponse)
    );
    let r = StopResponse::decode(body).unwrap();
    assert_eq!(r.result, -311);
    assert_eq!(r.discard_clock, 123_456_789);
}

#[test]
fn decodes_sdo_read_command() {
    let msg = SdoRead {
        slot: 0,
        index: 0x2002,
        subindex: 1,
    };
    let payload = frame_payload(MessageKind::SdoRead, 9, &msg.encoded_to_vec());
    match decode_command(0, &payload).unwrap() {
        Command::SdoRead {
            correlation_id: 9,
            msg: m,
        } => assert_eq!(m, msg),
        other => panic!("wrong variant: {other:?}"),
    }
}

#[test]
fn decodes_sdo_write_command() {
    let msg = SdoWrite {
        slot: 0,
        index: 0x2003,
        subindex: 0,
        size: 0,
        value: -42,
    };
    let payload = frame_payload(MessageKind::SdoWrite, 10, &msg.encoded_to_vec());
    match decode_command(0, &payload).unwrap() {
        Command::SdoWrite {
            correlation_id: 10,
            msg: m,
        } => assert_eq!(m, msg),
        other => panic!("wrong variant: {other:?}"),
    }
}

#[test]
fn decodes_query_motor_state_command() {
    let payload = frame_payload(MessageKind::QueryMotorState, 55, &[]);
    match decode_command(0, &payload).unwrap() {
        Command::QueryMotorState { correlation_id: 55 } => {}
        other => panic!("expected QueryMotorState, got {other:?}"),
    }
}

#[test]
fn motor_state_response_frame_round_trips() {
    let pos_mm: f64 = 12.5;
    let vel_mm_s: f64 = -400.0;
    let frame = motor_state_response_frame_multi(33, &[(0, pos_mm, vel_mm_s)]);
    let (chan, payload) = decode_frame(&frame).unwrap();
    assert_eq!(chan, CHANNEL_CONTROL);
    let (hdr, body) = decode_message_header(payload).unwrap();
    assert_eq!(hdr.correlation_id, 33);
    assert_eq!(
        MessageKind::from_u16(hdr.kind_raw),
        Some(MessageKind::MotorStateResponse)
    );
    let resp = MotorStateResponse::decode(body).unwrap();
    assert_eq!(resp.motors.len(), 1);
    let sample = resp.motors[0];
    assert_eq!(sample.slot, 0);
    assert_eq!(sample.pos_q16, (pos_mm * 65536.0).round() as i32);
    assert_eq!(sample.vel_q16, (vel_mm_s * 65536.0).round() as i32);
}

#[test]
fn motor_state_empty_frame_round_trips() {
    let frame = motor_state_empty_frame(34);
    let (chan, payload) = decode_frame(&frame).unwrap();
    assert_eq!(chan, CHANNEL_CONTROL);
    let (hdr, body) = decode_message_header(payload).unwrap();
    assert_eq!(hdr.correlation_id, 34);
    assert_eq!(
        MessageKind::from_u16(hdr.kind_raw),
        Some(MessageKind::MotorStateResponse)
    );
    let resp = MotorStateResponse::decode(body).unwrap();
    assert!(resp.motors.is_empty());
}

#[test]
fn sdo_response_frames_decode_back() {
    let frame = sdo_read_response_frame(
        11,
        &SdoReadResponse {
            result: 0,
            size: 2,
            data: [0x64, 0, 0, 0],
        },
    );
    let (chan, payload) = decode_frame(&frame).unwrap();
    assert_eq!(chan, CHANNEL_CONTROL);
    let (hdr, body) = decode_message_header(payload).unwrap();
    assert_eq!(hdr.correlation_id, 11);
    assert_eq!(
        MessageKind::from_u16(hdr.kind_raw),
        Some(MessageKind::SdoReadResponse)
    );
    let r = SdoReadResponse::decode(body).unwrap();
    assert_eq!((r.result, r.size, r.data), (0, 2, [0x64, 0, 0, 0]));

    let frame = sdo_write_response_frame(
        12,
        &SdoWriteResponse {
            result: -802,
            readback_size: 2,
            readback_data: [0xF4, 0x01, 0, 0],
        },
    );
    let (chan, payload) = decode_frame(&frame).unwrap();
    assert_eq!(chan, CHANNEL_CONTROL);
    let (hdr, body) = decode_message_header(payload).unwrap();
    assert_eq!(hdr.correlation_id, 12);
    assert_eq!(
        MessageKind::from_u16(hdr.kind_raw),
        Some(MessageKind::SdoWriteResponse)
    );
    let r = SdoWriteResponse::decode(body).unwrap();
    assert_eq!(
        (r.result, r.readback_size, r.readback_data),
        (-802, 2, [0xF4, 0x01, 0, 0])
    );
}

#[test]
fn decodes_set_dynamics_model_command() {
    let msg = SetDynamicsModel {
        slots_count: 2,
        modes_count: 2,
        frame: vec![0.5, 0.5, 0.5, -0.5],
        mass: vec![0.030, 0.030],
        viscous: vec![0.004, 0.004],
        coulomb: vec![1.0, 1.0],
        compliance: vec![0.0, 0.0],
        pairs: vec![mcu_protocol::messages::DynamicsPair {
            first: 0,
            second: 1,
            direction_split: 0.1,
        }],
    };
    let payload = frame_payload(MessageKind::SetDynamicsModel, 33, &msg.encoded_to_vec());
    match decode_command(0, &payload).expect("decode") {
        Command::SetDynamicsModel {
            correlation_id,
            msg: m,
        } => {
            assert_eq!(correlation_id, 33);
            assert_eq!(m, msg);
        }
        other => panic!("expected SetDynamicsModel, got {other:?}"),
    }
}

#[test]
fn set_dynamics_model_response_frame_round_trips() {
    let frame = set_dynamics_model_response_frame(34, -862);
    let (chan, payload) = decode_frame(&frame).unwrap();
    assert_eq!(chan, CHANNEL_CONTROL);
    let (hdr, body) = decode_message_header(payload).unwrap();
    assert_eq!(hdr.correlation_id, 34);
    assert_eq!(
        MessageKind::from_u16(hdr.kind_raw),
        Some(MessageKind::SetDynamicsModelResponse)
    );
    let r = SetDynamicsModelResponse::decode(body).unwrap();
    assert_eq!(r.result, -862);
}

#[test]
fn set_strain_comp_decodes_into_a_prepared_map() {
    let msg = SetStrainComp {
        slot_a: 0,
        slot_b: 1,
        lane_a: 0,
        lane_b: 1,
        kinematics: 0,
        nx: 2,
        ny: 2,
        x0: 0.0,
        y0: 0.0,
        dx: 1.0,
        dy: 1.0,
        values_um: vec![0, 100, -100, 50],
    };
    let payload = frame_payload(MessageKind::SetStrainComp, 9, &msg.encoded_to_vec());
    match decode_command(0, &payload).unwrap() {
        Command::SetStrainComp {
            correlation_id: 9,
            prepared,
        } => {
            assert_eq!(prepared.grid_rc, 0);
            assert_eq!(prepared.wire_values, 4);
            assert_eq!(prepared.values_mm, vec![0.0, 0.1, -0.1, 0.05]);
        }
        other => panic!("expected SetStrainComp, got {other:?}"),
    }
}

#[test]
fn set_strain_comp_decode_rejects_an_oversized_offset() {
    let msg = SetStrainComp {
        slot_a: 0,
        slot_b: 1,
        lane_a: 0,
        lane_b: 1,
        kinematics: 0,
        nx: 1,
        ny: 2,
        x0: 0.0,
        y0: 0.0,
        dx: 1.0,
        dy: 1.0,
        values_um: vec![0, 501],
    };
    let payload = frame_payload(MessageKind::SetStrainComp, 10, &msg.encoded_to_vec());
    match decode_command(0, &payload).unwrap() {
        Command::SetStrainComp { prepared, .. } => {
            assert_eq!(prepared.grid_rc, crate::strain_comp::ERR_COMP_BAD_GRID);
            assert!(prepared.values_mm.is_empty());
        }
        other => panic!("expected SetStrainComp, got {other:?}"),
    }
}

#[test]
fn decodes_set_ff_lead_command() {
    let msg = SetFfLead {
        slot: 1,
        lead_ns: 500_000,
    };
    let payload = frame_payload(MessageKind::SetFfLead, 40, &msg.encoded_to_vec());
    match decode_command(0, &payload).unwrap() {
        Command::SetFfLead {
            correlation_id: 40,
            msg: m,
        } => {
            assert_eq!(m.slot, 1);
            assert_eq!(m.lead_ns, 500_000);
        }
        other => panic!("expected SetFfLead, got {other:?}"),
    }
}

#[test]
fn set_ff_lead_response_frame_round_trips() {
    let frame = set_ff_lead_response_frame(41, -309);
    let (chan, payload) = decode_frame(&frame).unwrap();
    assert_eq!(chan, CHANNEL_CONTROL);
    let (hdr, body) = decode_message_header(payload).unwrap();
    assert_eq!(hdr.correlation_id, 41);
    assert_eq!(
        MessageKind::from_u16(hdr.kind_raw),
        Some(MessageKind::SetFfLeadResponse)
    );
    assert_eq!(SetFfLeadResponse::decode(body).unwrap().result, -309);
}
