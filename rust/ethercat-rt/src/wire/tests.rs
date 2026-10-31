use super::*;
use mcu_protocol::messages::{DriveLimitEntry, DynamicsPair, SlaveState, SlaveStatus};
use mcu_transport::frame::{decode_frame, CHANNEL_CONTROL};

/// (channel, kind, correlation_id, body) of an encoded frame.
fn parts(frame: &[u8]) -> (u8, MessageKind, u32, Vec<u8>) {
    let (chan, payload) = decode_frame(frame).expect("frame decodes");
    let (hdr, body) = decode_message_header(payload).expect("header decodes");
    (
        chan,
        MessageKind::from_u16(hdr.kind_raw).expect("known kind"),
        hdr.correlation_id,
        body.to_vec(),
    )
}

fn decoded(kind: MessageKind, cid: u32, body: &[u8]) -> Command {
    decode_command(&frame_payload(kind, cid, body)).expect("command decodes")
}

#[test]
fn bodyless_commands_decode_to_their_variant() {
    let cases: [(MessageKind, fn(&Command) -> Option<u32>); 7] = [
        (MessageKind::QueryRuntimeCaps, |c| match c {
            Command::QueryRuntimeCaps { correlation_id } => Some(*correlation_id),
            _ => None,
        }),
        (MessageKind::QuerySampleGrid, |c| match c {
            Command::QuerySampleGrid { correlation_id } => Some(*correlation_id),
            _ => None,
        }),
        (MessageKind::QueryMotorState, |c| match c {
            Command::QueryMotorState { correlation_id } => Some(*correlation_id),
            _ => None,
        }),
        (MessageKind::ClaimHandshake, |c| match c {
            Command::ClaimHandshake { correlation_id } => Some(*correlation_id),
            _ => None,
        }),
        (MessageKind::StopCapture, |c| match c {
            Command::StopCapture { correlation_id } => Some(*correlation_id),
            _ => None,
        }),
        (MessageKind::Stop, |c| match c {
            Command::Stop { correlation_id } => Some(*correlation_id),
            _ => None,
        }),
        (MessageKind::ResumeStream, |c| match c {
            Command::ResumeStream { correlation_id } => Some(*correlation_id),
            _ => None,
        }),
    ];
    for (kind, extract) in cases {
        let cmd = decoded(kind, 77, &[]);
        assert_eq!(
            extract(&cmd),
            Some(77),
            "{kind:?} decoded to the wrong variant: {cmd:?}"
        );
    }
}

#[test]
fn typed_commands_carry_their_decoded_body() {
    let set_torque = SetTorque {
        value: 1,
        execute_at_ns: 123_456_789,
    };
    let buzz = ResonanceBuzz {
        axis_mask: 0b001,
        sign_mask: 0b000,
        freq_start_millihz: 5_000,
        freq_end_millihz: 300_000,
        amplitude_nm: 4_200,
        duration_ms: 3_000,
        ramp_ms: 300,
    };
    let capture = StartCapture {
        path: "/tmp/t.scap".into(),
        started_utc: "2026-06-10T12:00:00Z".into(),
        drives: vec![mcu_protocol::messages::CaptureDrive {
            slot: 0,
            name: "x".into(),
        }],
    };
    let limits = SetDriveLimits {
        drives: vec![
            DriveLimitEntry {
                slot: 0,
                following_error_counts: 8192,
                max_torque_tenth_pct: 500,
            },
            DriveLimitEntry {
                slot: 1,
                following_error_counts: 4096,
                max_torque_tenth_pct: 300,
            },
        ],
    };
    let sdo_read = SdoRead {
        slot: 0,
        index: 0x2002,
        subindex: 1,
    };
    let sdo_write = SdoWrite {
        slot: 0,
        index: 0x2003,
        subindex: 0,
        size: 0,
        value: -42,
    };
    let ff_lead = SetFfLead {
        slot: 1,
        lead_ns: 500_000,
    };
    let dynamics = SetDynamicsModel {
        slots_count: 2,
        modes_count: 2,
        frame: vec![0.5, 0.5, 0.5, -0.5],
        mass: vec![0.030, 0.030],
        viscous: vec![0.004, 0.004],
        coulomb: vec![1.0, 1.0],
        compliance: vec![0.0, 0.0],
        pin_mass: vec![0.0, 0.0],
        pin_zeta: vec![0.0, 0.0],
        pin_lead_us: 0.0,
        pairs: vec![DynamicsPair {
            first: 0,
            second: 1,
            direction_split: 0.1,
        }],
    };

    let mut checks: Vec<(MessageKind, Vec<u8>, Box<dyn Fn(Command) -> bool>)> = Vec::new();
    checks.push((
        MessageKind::Identify,
        vec![3u8],
        Box::new(|c| {
            matches!(
                c,
                Command::Identify {
                    correlation_id: 5,
                    proto_version: 3
                }
            )
        }),
    ));
    checks.push((
        MessageKind::SetTorque,
        set_torque.encoded_to_vec(),
        Box::new(move |c| matches!(c, Command::SetTorque { msg, .. } if msg == set_torque)),
    ));
    checks.push((
        MessageKind::ResonanceBuzz,
        buzz.encoded_to_vec(),
        Box::new(move |c| matches!(c, Command::ResonanceBuzz { msg, .. } if msg == buzz)),
    ));
    checks.push((
        MessageKind::StartCapture,
        capture.encoded_to_vec(),
        Box::new(move |c| {
            matches!(c, Command::StartCapture { msg, .. }
                if msg.path == capture.path
                    && msg.started_utc == capture.started_utc
                    && msg.drives == capture.drives)
        }),
    ));
    checks.push((
        MessageKind::SetDriveLimits,
        limits.encoded_to_vec(),
        Box::new(move |c| matches!(c, Command::SetDriveLimits { msg, .. } if msg == limits)),
    ));
    checks.push((
        MessageKind::RestoreDriveLimits,
        RestoreDriveLimits { slot_mask: 0b11 }.encoded_to_vec(),
        Box::new(|c| {
            matches!(
                c,
                Command::RestoreDriveLimits {
                    slot_mask: 0b11,
                    ..
                }
            )
        }),
    ));
    checks.push((
        MessageKind::SeedServoHome,
        SeedServoHome {
            slot: 0,
            home_q16: -98_304,
        }
        .encoded_to_vec(),
        Box::new(|c| {
            matches!(
                c,
                Command::SeedServoHome {
                    slot: 0,
                    home_q16: -98_304,
                    ..
                }
            )
        }),
    ));
    checks.push((
        MessageKind::SdoRead,
        sdo_read.encoded_to_vec(),
        Box::new(move |c| matches!(c, Command::SdoRead { msg, .. } if msg == sdo_read)),
    ));
    checks.push((
        MessageKind::SdoWrite,
        sdo_write.encoded_to_vec(),
        Box::new(move |c| matches!(c, Command::SdoWrite { msg, .. } if msg == sdo_write)),
    ));
    checks.push((
        MessageKind::SetFfLead,
        ff_lead.encoded_to_vec(),
        Box::new(move |c| matches!(c, Command::SetFfLead { msg, .. } if msg == ff_lead)),
    ));
    checks.push((
        MessageKind::SetDynamicsModel,
        dynamics.encoded_to_vec(),
        Box::new(move |c| matches!(c, Command::SetDynamicsModel { msg, .. } if msg == dynamics)),
    ));

    for (kind, body, check) in checks {
        let cmd = decoded(kind, 5, &body);
        let shown = format!("{cmd:?}");
        assert!(check(cmd), "{kind:?} decoded wrong: {shown}");
    }
}

#[test]
fn result_frames_round_trip_on_the_control_channel() {
    let kinds = [
        MessageKind::ResumeStreamResponse,
        MessageKind::SetTorqueResponse,
        MessageKind::StartCaptureResponse,
        MessageKind::SetDriveLimitsResponse,
        MessageKind::RestoreDriveLimitsResponse,
        MessageKind::SeedServoHomeResponse,
        MessageKind::ArmSensorlessEndstopResponse,
        MessageKind::ResonanceBuzzResponse,
        MessageKind::SetDiffDamperResponse,
        MessageKind::SetStrainCompResponse,
        MessageKind::SetDynamicsModelResponse,
        MessageKind::SetDiffTrimResponse,
        MessageKind::SetFfLeadResponse,
    ];
    for kind in kinds {
        let frame = result_frame(kind, 9, -312);
        let (chan, decoded_kind, cid, body) = parts(&frame);
        assert_eq!(chan, CHANNEL_CONTROL);
        assert_eq!(decoded_kind, kind);
        assert_eq!(cid, 9);
        assert_eq!(
            i32::from_le_bytes(body[..4].try_into().expect("4-byte result")),
            -312,
            "{kind:?} body is not the plain i32 result"
        );
    }
}

#[test]
fn stop_response_carries_the_discard_clock() {
    let (chan, kind, cid, body) = parts(&stop_response_frame(5, -311, 123_456_789));
    assert_eq!(chan, CHANNEL_CONTROL);
    assert_eq!(kind, MessageKind::StopResponse);
    assert_eq!(cid, 5);
    let r = StopResponse::decode(&body).unwrap();
    assert_eq!((r.result, r.discard_clock), (-311, 123_456_789));
}

#[test]
fn stop_capture_response_carries_samples_and_overflow() {
    let (_, kind, cid, body) = parts(&stop_capture_response_frame(9, -323, 1234, 567));
    assert_eq!(kind, MessageKind::StopCaptureResponse);
    assert_eq!(cid, 9);
    let r = StopCaptureResponse::decode(&body).unwrap();
    assert_eq!((r.result, r.samples, r.overflow_cycle), (-323, 1234, 567));
}

#[test]
fn sdo_response_frames_decode_back() {
    let (_, kind, cid, body) = parts(&sdo_read_response_frame(
        11,
        &SdoReadResponse {
            result: 0,
            size: 2,
            data: [0x64, 0, 0, 0],
        },
    ));
    assert_eq!(kind, MessageKind::SdoReadResponse);
    assert_eq!(cid, 11);
    let r = SdoReadResponse::decode(&body).unwrap();
    assert_eq!((r.result, r.size, r.data), (0, 2, [0x64, 0, 0, 0]));

    let (_, kind, cid, body) = parts(&sdo_write_response_frame(
        12,
        &SdoWriteResponse {
            result: -802,
            readback_size: 2,
            readback_data: [0xF4, 0x01, 0, 0],
        },
    ));
    assert_eq!(kind, MessageKind::SdoWriteResponse);
    assert_eq!(cid, 12);
    let r = SdoWriteResponse::decode(&body).unwrap();
    assert_eq!(
        (r.result, r.readback_size, r.readback_data),
        (-802, 2, [0xF4, 0x01, 0, 0])
    );
}

#[test]
fn motor_state_response_carries_one_q16_sample_per_slot() {
    let (chan, kind, cid, body) = parts(&motor_state_response_frame_multi(
        9,
        &[(0, 12.5, -400.0), (1, -3.0, 4.0)],
    ));
    assert_eq!(chan, CHANNEL_CONTROL);
    assert_eq!(kind, MessageKind::MotorStateResponse);
    assert_eq!(cid, 9);
    let r = MotorStateResponse::decode(&body).unwrap();
    assert_eq!(r.motors.len(), 2);
    assert_eq!(r.motors[0].slot, 0);
    assert_eq!(r.motors[0].pos_q16, (12.5_f64 * 65536.0) as i32);
    assert_eq!(r.motors[0].vel_q16, (-400.0_f64 * 65536.0) as i32);
    assert_eq!(r.motors[1].slot, 1);
    assert_eq!(r.motors[1].pos_q16, (-3.0_f64 * 65536.0) as i32);

    let (_, _, _, body) = parts(&motor_state_response_frame_multi(34, &[]));
    assert!(MotorStateResponse::decode(&body).unwrap().motors.is_empty());
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
    let (chan, kind, cid, body) = parts(&claim_handshake_reply_frame(7, &reply));
    assert_eq!(chan, CHANNEL_CONTROL);
    assert_eq!(kind, MessageKind::ClaimHandshakeReply);
    assert_eq!(cid, 7);
    assert_eq!(ClaimHandshakeReply::decode(&body).unwrap(), reply);
}

#[test]
fn status_heartbeat_rides_the_events_channel_with_progress_and_fault() {
    let (chan, kind, cid, body) = parts(&status_heartbeat_frame(
        1,
        0x8611,
        &[42u32, 0u32],
        &[900u64, 0u64],
        0,
    ));
    assert_eq!(chan, CHANNEL_EVENTS);
    assert_eq!(kind, MessageKind::StatusHeartbeat);
    assert_eq!(cid, 0);
    let hb = StatusHeartbeat::decode(&body).unwrap();
    assert_eq!(hb.engine_state, 1);
    assert_eq!(hb.fault_code, 0x8611);
    assert_eq!(hb.retired_counts, vec![42u32, 0u32]);
    assert_eq!(hb.playback_clocks, vec![900u64, 0u64]);
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
    match decoded(MessageKind::SetStrainComp, 9, &msg.encoded_to_vec()) {
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
    match decoded(MessageKind::SetStrainComp, 10, &msg.encoded_to_vec()) {
        Command::SetStrainComp { prepared, .. } => {
            assert_eq!(prepared.grid_rc, crate::strain_comp::ERR_COMP_BAD_GRID);
            assert!(prepared.values_mm.is_empty());
        }
        other => panic!("expected SetStrainComp, got {other:?}"),
    }
}
