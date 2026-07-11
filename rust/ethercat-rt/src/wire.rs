use mcu_protocol::bootstrap::{IdentifyResponse, IDENTIFY_RESPONSE_BODY_LEN};
use mcu_protocol::codec::{Decode, Encode};
use mcu_protocol::messages::{
    ArmSensorlessEndstop, ArmSensorlessEndstopResponse, AxisDiag, ClaimHandshakeReply, EndstopTrip,
    MessageKind, MotorSample, MotorStateResponse, PushPieces, PushPiecesResponse, ResonanceBuzz,
    ResonanceBuzzResponse, RestoreDriveLimits, RestoreDriveLimitsResponse, ResumeStreamResponse,
    RuntimeCapsResponse, SdoRead, SdoReadResponse, SdoWrite, SdoWriteResponse, SeedServoHome,
    SeedServoHomeResponse, SetDiffDamper, SetDiffDamperResponse, SetDriveLimits,
    SetDriveLimitsResponse, SetTorque, SetTorqueResponse, StartCapture, StartCaptureResponse,
    StatusHeartbeat, StopCaptureResponse, StopResponse, SyncPair, SyncPairResponse,
};
use mcu_protocol::MCU_CHANNEL_PIECES;
use mcu_transport::frame::{encode_frame, CHANNEL_CONTROL, CHANNEL_EVENTS};
use mcu_transport::wire_helpers::{
    decode_message_header, encode_message_header, MESSAGE_VERSION_DEFAULT,
};

#[derive(Debug)]
pub enum Command {
    Identify {
        correlation_id: u32,
        proto_version: u8,
    },
    PushPieces {
        correlation_id: u32,
        msg: PushPieces,
    },
    QueryRuntimeCaps {
        correlation_id: u32,
    },
    QueryMotorState {
        correlation_id: u32,
    },
    ClaimHandshake {
        correlation_id: u32,
    },
    SetTorque {
        correlation_id: u32,
        msg: SetTorque,
    },
    StartCapture {
        correlation_id: u32,
        msg: StartCapture,
    },
    StopCapture {
        correlation_id: u32,
    },
    Stop {
        correlation_id: u32,
    },
    ResumeStream {
        correlation_id: u32,
    },
    SetDriveLimits {
        correlation_id: u32,
        msg: SetDriveLimits,
    },
    RestoreDriveLimits {
        correlation_id: u32,
        slot: u8,
    },
    SeedServoHome {
        correlation_id: u32,
        slot: u8,
        home_q16: i32,
    },
    ArmSensorlessEndstop {
        correlation_id: u32,
        msg: ArmSensorlessEndstop,
    },
    ResonanceBuzz {
        correlation_id: u32,
        msg: ResonanceBuzz,
    },
    SdoRead {
        correlation_id: u32,
        msg: SdoRead,
    },
    SdoWrite {
        correlation_id: u32,
        msg: SdoWrite,
    },
    SyncPair {
        correlation_id: u32,
        msg: SyncPair,
    },
    SetDiffDamper {
        correlation_id: u32,
        msg: SetDiffDamper,
    },
    Unknown {
        correlation_id: u32,
        kind_raw: u16,
    },
}

#[derive(Debug, PartialEq, Eq)]
pub enum DecodeCmdError {
    BadHeader,
    BadBody,
}

pub fn decode_command(channel: u8, payload: &[u8]) -> Result<Command, DecodeCmdError> {
    let (hdr, body) = decode_message_header(payload).ok_or(DecodeCmdError::BadHeader)?;
    let cid = hdr.correlation_id;
    if channel == MCU_CHANNEL_PIECES
        || MessageKind::from_u16(hdr.kind_raw) == Some(MessageKind::PushPieces)
    {
        let msg = PushPieces::decode(body).map_err(|_| DecodeCmdError::BadBody)?;
        return Ok(Command::PushPieces {
            correlation_id: cid,
            msg,
        });
    }
    match MessageKind::from_u16(hdr.kind_raw) {
        Some(MessageKind::Identify) => {
            let proto_version = body.first().copied().unwrap_or(0);
            Ok(Command::Identify {
                correlation_id: cid,
                proto_version,
            })
        }
        Some(MessageKind::QueryRuntimeCaps) => Ok(Command::QueryRuntimeCaps {
            correlation_id: cid,
        }),
        Some(MessageKind::QueryMotorState) => Ok(Command::QueryMotorState {
            correlation_id: cid,
        }),
        Some(MessageKind::ClaimHandshake) => Ok(Command::ClaimHandshake {
            correlation_id: cid,
        }),
        Some(MessageKind::SetTorque) => {
            let msg = SetTorque::decode(body).map_err(|_| DecodeCmdError::BadBody)?;
            Ok(Command::SetTorque {
                correlation_id: cid,
                msg,
            })
        }
        Some(MessageKind::StartCapture) => {
            let msg = StartCapture::decode(body).map_err(|_| DecodeCmdError::BadBody)?;
            Ok(Command::StartCapture {
                correlation_id: cid,
                msg,
            })
        }
        Some(MessageKind::StopCapture) => Ok(Command::StopCapture {
            correlation_id: cid,
        }),
        Some(MessageKind::Stop) => Ok(Command::Stop {
            correlation_id: cid,
        }),
        Some(MessageKind::ResumeStream) => Ok(Command::ResumeStream {
            correlation_id: cid,
        }),
        Some(MessageKind::SetDriveLimits) => {
            let msg = SetDriveLimits::decode(body).map_err(|_| DecodeCmdError::BadBody)?;
            Ok(Command::SetDriveLimits {
                correlation_id: cid,
                msg,
            })
        }
        Some(MessageKind::RestoreDriveLimits) => {
            let msg = RestoreDriveLimits::decode(body).map_err(|_| DecodeCmdError::BadBody)?;
            Ok(Command::RestoreDriveLimits {
                correlation_id: cid,
                slot: msg.slot,
            })
        }
        Some(MessageKind::SeedServoHome) => {
            let msg = SeedServoHome::decode(body).map_err(|_| DecodeCmdError::BadBody)?;
            Ok(Command::SeedServoHome {
                correlation_id: cid,
                slot: msg.slot,
                home_q16: msg.home_q16,
            })
        }
        Some(MessageKind::ArmSensorlessEndstop) => {
            let msg = ArmSensorlessEndstop::decode(body).map_err(|_| DecodeCmdError::BadBody)?;
            Ok(Command::ArmSensorlessEndstop {
                correlation_id: cid,
                msg,
            })
        }
        Some(MessageKind::ResonanceBuzz) => {
            let msg = ResonanceBuzz::decode(body).map_err(|_| DecodeCmdError::BadBody)?;
            Ok(Command::ResonanceBuzz {
                correlation_id: cid,
                msg,
            })
        }
        Some(MessageKind::SyncPair) => {
            let msg = SyncPair::decode(body).map_err(|_| DecodeCmdError::BadBody)?;
            Ok(Command::SyncPair {
                correlation_id: cid,
                msg,
            })
        }
        Some(MessageKind::SdoRead) => {
            let msg = SdoRead::decode(body).map_err(|_| DecodeCmdError::BadBody)?;
            Ok(Command::SdoRead {
                correlation_id: cid,
                msg,
            })
        }
        Some(MessageKind::SdoWrite) => {
            let msg = SdoWrite::decode(body).map_err(|_| DecodeCmdError::BadBody)?;
            Ok(Command::SdoWrite {
                correlation_id: cid,
                msg,
            })
        }
        Some(MessageKind::SetDiffDamper) => {
            let msg = SetDiffDamper::decode(body).map_err(|_| DecodeCmdError::BadBody)?;
            Ok(Command::SetDiffDamper {
                correlation_id: cid,
                msg,
            })
        }
        _ => Ok(Command::Unknown {
            correlation_id: cid,
            kind_raw: hdr.kind_raw,
        }),
    }
}

pub fn frame_payload(kind: MessageKind, correlation_id: u32, body: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(7 + body.len());
    out.extend_from_slice(&encode_message_header(
        kind,
        MESSAGE_VERSION_DEFAULT,
        correlation_id,
    ));
    out.extend_from_slice(body);
    out
}

pub fn control_frame(kind: MessageKind, correlation_id: u32, body: &[u8]) -> Vec<u8> {
    encode_frame(CHANNEL_CONTROL, &frame_payload(kind, correlation_id, body))
}

pub fn stop_response_frame(cid: u32, result: i32, discard_clock: u64) -> Vec<u8> {
    let body = StopResponse {
        result,
        discard_clock,
    }
    .encoded_to_vec();
    control_frame(MessageKind::StopResponse, cid, &body)
}

pub fn resume_stream_response_frame(cid: u32, result: i32) -> Vec<u8> {
    let body = ResumeStreamResponse { result }.encoded_to_vec();
    control_frame(MessageKind::ResumeStreamResponse, cid, &body)
}

pub fn set_torque_response_frame(cid: u32, result: i32) -> Vec<u8> {
    let body = SetTorqueResponse { result }.encoded_to_vec();
    control_frame(MessageKind::SetTorqueResponse, cid, &body)
}

pub fn start_capture_response_frame(cid: u32, result: i32) -> Vec<u8> {
    let body = StartCaptureResponse { result }.encoded_to_vec();
    control_frame(MessageKind::StartCaptureResponse, cid, &body)
}

pub fn stop_capture_response_frame(
    cid: u32,
    result: i32,
    samples: u64,
    overflow_cycle: u64,
) -> Vec<u8> {
    let body = StopCaptureResponse {
        result,
        samples,
        overflow_cycle,
    }
    .encoded_to_vec();
    control_frame(MessageKind::StopCaptureResponse, cid, &body)
}

pub fn sdo_read_response_frame(cid: u32, resp: &SdoReadResponse) -> Vec<u8> {
    control_frame(MessageKind::SdoReadResponse, cid, &resp.encoded_to_vec())
}

pub fn sdo_write_response_frame(cid: u32, resp: &SdoWriteResponse) -> Vec<u8> {
    control_frame(MessageKind::SdoWriteResponse, cid, &resp.encoded_to_vec())
}

pub fn push_pieces_response_frame(
    cid: u32,
    result: i32,
    arrival_clock: u64,
    axis_idx: u8,
    front_start_time: u64,
) -> Vec<u8> {
    let body = PushPiecesResponse::single(result, arrival_clock, axis_idx, front_start_time)
        .encoded_to_vec();
    control_frame(MessageKind::PushPiecesResponse, cid, &body)
}

/// One `AxisDiag` per axis the endpoint pushed, in `(axis_idx, front_start_time)`
/// order. The bridge matches each axis' transit diag by `axis_idx`, so a
/// multi-axis PushPieces needs every axis echoed back in one response.
pub fn push_pieces_response_frame_multi(
    cid: u32,
    result: i32,
    arrival_clock: u64,
    axes: &[(u8, u64)],
) -> Vec<u8> {
    let body = PushPiecesResponse {
        result,
        arrival_clock,
        axes: axes
            .iter()
            .map(|&(axis_idx, front_start_time)| AxisDiag {
                axis_idx,
                front_start_time,
            })
            .collect(),
    }
    .encoded_to_vec();
    control_frame(MessageKind::PushPiecesResponse, cid, &body)
}

pub fn set_drive_limits_response_frame(cid: u32, result: i32) -> Vec<u8> {
    let body = SetDriveLimitsResponse { result }.encoded_to_vec();
    control_frame(MessageKind::SetDriveLimitsResponse, cid, &body)
}

pub fn restore_drive_limits_response_frame(cid: u32, result: i32) -> Vec<u8> {
    let body = RestoreDriveLimitsResponse { result }.encoded_to_vec();
    control_frame(MessageKind::RestoreDriveLimitsResponse, cid, &body)
}

pub fn seed_servo_home_response_frame(cid: u32, result: i32) -> Vec<u8> {
    let body = SeedServoHomeResponse { result }.encoded_to_vec();
    control_frame(MessageKind::SeedServoHomeResponse, cid, &body)
}

pub fn arm_sensorless_endstop_response_frame(cid: u32, result: i32) -> Vec<u8> {
    let body = ArmSensorlessEndstopResponse { result }.encoded_to_vec();
    control_frame(MessageKind::ArmSensorlessEndstopResponse, cid, &body)
}

pub fn sync_pair_response_frame(cid: u32, resp: &SyncPairResponse) -> Vec<u8> {
    let body = resp.encoded_to_vec();
    control_frame(MessageKind::SyncPairResponse, cid, &body)
}

pub fn endstop_trip_frame(endstop_id: u8, trip_clock: u64) -> Vec<u8> {
    let body = EndstopTrip {
        endstop_id,
        trip_clock,
    }
    .encoded_to_vec();
    let mut payload =
        encode_message_header(MessageKind::EndstopTrip, MESSAGE_VERSION_DEFAULT, 0).to_vec();
    payload.extend_from_slice(&body);
    encode_frame(CHANNEL_EVENTS, &payload)
}

pub fn resonance_buzz_response_frame(cid: u32, result: i32) -> Vec<u8> {
    let body = ResonanceBuzzResponse { result }.encoded_to_vec();
    control_frame(MessageKind::ResonanceBuzzResponse, cid, &body)
}

pub fn set_diff_damper_response_frame(cid: u32, result: i32) -> Vec<u8> {
    let body = SetDiffDamperResponse { result }.encoded_to_vec();
    control_frame(MessageKind::SetDiffDamperResponse, cid, &body)
}

pub fn status_heartbeat_frame(
    engine_state: u8,
    fault_code: u16,
    retired_counts: &[u32],
    ff_saturation_count: u32,
) -> Vec<u8> {
    let hb = StatusHeartbeat {
        engine_state,
        fault_code,
        retired_counts: retired_counts.to_vec(),
        ff_saturation_count,
    };
    let body = hb.encoded_to_vec();
    let payload = {
        let mut p = encode_message_header(MessageKind::StatusHeartbeat, MESSAGE_VERSION_DEFAULT, 0)
            .to_vec();
        p.extend_from_slice(&body);
        p
    };
    encode_frame(CHANNEL_EVENTS, &payload)
}

pub fn runtime_caps_response_frame(cid: u32, total_piece_memory: u32) -> Vec<u8> {
    let body = RuntimeCapsResponse { total_piece_memory }.encoded_to_vec();
    control_frame(MessageKind::RuntimeCapsResponse, cid, &body)
}

#[allow(clippy::cast_possible_truncation)]
fn mm_to_q16(mm: f64) -> i32 {
    (mm * 65536.0).round() as i32
}

/// One `MotorSample` per slot, in `(slot, pos_mm, vel_mm_s)` order, so the host
/// receives the full chain's state in a single query.
pub fn motor_state_response_frame_multi(
    correlation_id: u32,
    samples: &[(u8, f64, f64)],
) -> Vec<u8> {
    let resp = MotorStateResponse {
        motors: samples
            .iter()
            .map(|&(slot, pos_mm, vel_mm_s)| MotorSample {
                slot,
                pos_q16: mm_to_q16(pos_mm),
                vel_q16: mm_to_q16(vel_mm_s),
            })
            .collect(),
    };
    control_frame(
        MessageKind::MotorStateResponse,
        correlation_id,
        &resp.encoded_to_vec(),
    )
}

pub fn motor_state_empty_frame(correlation_id: u32) -> Vec<u8> {
    let resp = MotorStateResponse { motors: vec![] };
    control_frame(
        MessageKind::MotorStateResponse,
        correlation_id,
        &resp.encoded_to_vec(),
    )
}

pub fn claim_handshake_reply_frame(cid: u32, reply: &ClaimHandshakeReply) -> Vec<u8> {
    control_frame(
        MessageKind::ClaimHandshakeReply,
        cid,
        &reply.encoded_to_vec(),
    )
}

pub fn identify_response_frame(cid: u32, proto_version: u8) -> Vec<u8> {
    let resp = IdentifyResponse {
        proto_version,
        firmware_ver: 1,
        build_hash: [0u8; 20],
        schema_hash: [0u8; 32],
        reset_epoch: 0,
        capabilities: 0,
        mcu_serial: *b"ETHERCAT-RT\0",
    };
    let body = resp.encode_body_to_array();
    debug_assert_eq!(body.len(), IDENTIFY_RESPONSE_BODY_LEN);
    control_frame(MessageKind::IdentifyResponse, cid, &body)
}

#[cfg(test)]
mod tests;
