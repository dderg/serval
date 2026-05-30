//! Wire helpers: piece-bytes -> WirePiece, control-message decode, responses.

use kalico_native_transport::frame::{encode_frame, CHANNEL_CONTROL};
use kalico_native_transport::wire_helpers::{
    decode_message_header, encode_message_header, MESSAGE_VERSION_DEFAULT,
};
use kalico_protocol::bootstrap::{IdentifyResponse, IDENTIFY_RESPONSE_BODY_LEN};
use kalico_protocol::codec::{Decode, Encode};
use kalico_protocol::messages::{
    LoadCurveCubic, LoadCurveResponse, MessageKind, PushSegment, PushSegmentResponse,
    ResetCurvePoolResponse,
};
use runtime::cubic_curve::WirePiece;

#[derive(Debug, PartialEq, Eq)]
pub enum PiecesError {
    BadLength,
}

/// Split a `LoadCurveCubic.pieces_bytes` blob into `WirePiece`s.
pub fn wire_pieces_from_bytes(piece_count: u8, bytes: &[u8]) -> Result<Vec<WirePiece>, PiecesError> {
    let n = piece_count as usize;
    if bytes.len() != n * 20 {
        return Err(PiecesError::BadLength);
    }
    let mut out = Vec::with_capacity(n);
    for chunk in bytes.chunks_exact(20) {
        let rd = |i: usize| u32::from_le_bytes([chunk[i], chunk[i + 1], chunk[i + 2], chunk[i + 3]]);
        out.push(WirePiece {
            bp0_bits: rd(0),
            bp1_bits: rd(4),
            bp2_bits: rd(8),
            bp3_bits: rd(12),
            duration_bits: rd(16),
        });
    }
    Ok(out)
}

/// A decoded control-channel command plus the correlation id to answer with.
#[derive(Debug)]
pub enum Command {
    Identify { correlation_id: u32, proto_version: u8 },
    LoadCurve { correlation_id: u32, msg: LoadCurveCubic },
    PushSegment { correlation_id: u32, msg: PushSegment },
    ResetPool { correlation_id: u32 },
    Unknown { correlation_id: u32, kind_raw: u16 },
}

#[derive(Debug, PartialEq, Eq)]
pub enum DecodeCmdError {
    BadHeader,
    BadBody,
}

/// `payload` is a `Frame::Kalico` payload: 7-byte message header + body.
pub fn decode_command(payload: &[u8]) -> Result<Command, DecodeCmdError> {
    let (hdr, body) = decode_message_header(payload).ok_or(DecodeCmdError::BadHeader)?;
    let cid = hdr.correlation_id;
    match MessageKind::from_u16(hdr.kind_raw) {
        Some(MessageKind::Identify) => {
            let proto_version = body.first().copied().unwrap_or(0);
            Ok(Command::Identify { correlation_id: cid, proto_version })
        }
        Some(MessageKind::LoadCurveCubic) => {
            let msg = LoadCurveCubic::decode(body).map_err(|_| DecodeCmdError::BadBody)?;
            Ok(Command::LoadCurve { correlation_id: cid, msg })
        }
        Some(MessageKind::PushSegment) => {
            let msg = PushSegment::decode(body).map_err(|_| DecodeCmdError::BadBody)?;
            Ok(Command::PushSegment { correlation_id: cid, msg })
        }
        Some(MessageKind::ResetCurvePool) => Ok(Command::ResetPool { correlation_id: cid }),
        _ => Ok(Command::Unknown { correlation_id: cid, kind_raw: hdr.kind_raw }),
    }
}

/// Build a control-channel command payload (header + body) for a `Decode`/`Encode`
/// message. Test/client helper.
pub fn frame_payload(kind: MessageKind, correlation_id: u32, body: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(7 + body.len());
    out.extend_from_slice(&encode_message_header(kind, MESSAGE_VERSION_DEFAULT, correlation_id));
    out.extend_from_slice(body);
    out
}

/// Wrap a header+body payload into a full Layer-1 frame on the control channel.
pub fn control_frame(kind: MessageKind, correlation_id: u32, body: &[u8]) -> Vec<u8> {
    encode_frame(CHANNEL_CONTROL, &frame_payload(kind, correlation_id, body))
}

pub fn load_curve_response_frame(cid: u32, result: i32, handle_packed: u32) -> Vec<u8> {
    let body = LoadCurveResponse { result, curve_handle_packed: handle_packed }.encoded_to_vec();
    control_frame(MessageKind::LoadCurveResponse, cid, &body)
}

pub fn push_segment_response_frame(cid: u32, result: i32, accepted_id: u32) -> Vec<u8> {
    let body =
        PushSegmentResponse { result, accepted_segment_id: accepted_id, credit_epoch: 0 }.encoded_to_vec();
    control_frame(MessageKind::PushSegmentResponse, cid, &body)
}

pub fn reset_pool_response_frame(cid: u32, result: i32) -> Vec<u8> {
    let body = ResetCurvePoolResponse { result }.encoded_to_vec();
    control_frame(MessageKind::ResetCurvePoolResponse, cid, &body)
}

/// Canned identify response advertising one motion channel, no special caps.
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
mod tests {
    use super::*;

    #[test]
    fn decodes_one_piece() {
        let bytes = {
            let mut v = Vec::new();
            for x in [0.0f32, 0.0, 10.0, 10.0] {
                v.extend_from_slice(&x.to_bits().to_le_bytes());
            }
            v.extend_from_slice(&0.5f32.to_bits().to_le_bytes());
            v
        };
        let pieces = wire_pieces_from_bytes(1, &bytes).unwrap();
        assert_eq!(pieces.len(), 1);
        assert_eq!(f32::from_bits(pieces[0].bp2_bits), 10.0);
        assert_eq!(f32::from_bits(pieces[0].duration_bits), 0.5);
    }

    #[test]
    fn rejects_wrong_length() {
        assert!(matches!(wire_pieces_from_bytes(2, &[0u8; 20]), Err(PiecesError::BadLength)));
    }
}

#[cfg(test)]
mod decode_cmd_tests {
    use super::*;

    #[test]
    fn round_trips_push_segment() {
        let seg = PushSegment {
            id: 7,
            handle_x: 0x0001_0000,
            handle_y: 0,
            handle_z: 0,
            handle_e: 0,
            t_start: 1_000,
            t_end: 2_000,
            kinematics: 0,
            e_mode: 0,
            extrusion_ratio: 0.0,
        };
        let payload = frame_payload(MessageKind::PushSegment, 42, &seg.encoded_to_vec());
        match decode_command(&payload).unwrap() {
            Command::PushSegment { correlation_id, msg } => {
                assert_eq!(correlation_id, 42);
                assert_eq!(msg.id, 7);
                assert_eq!(msg.handle_x, 0x0001_0000);
            }
            other => panic!("wrong variant: {other:?}"),
        }
    }

    #[test]
    fn round_trips_load_curve() {
        let msg = LoadCurveCubic { slot_idx: 3, axis_idx: 0, piece_count: 0, pieces_bytes: vec![] };
        let payload = frame_payload(MessageKind::LoadCurveCubic, 9, &msg.encoded_to_vec());
        match decode_command(&payload).unwrap() {
            Command::LoadCurve { correlation_id, msg } => {
                assert_eq!(correlation_id, 9);
                assert_eq!(msg.slot_idx, 3);
            }
            other => panic!("wrong variant: {other:?}"),
        }
    }

    #[test]
    fn decodes_identify() {
        let payload = frame_payload(MessageKind::Identify, 1, &[3u8]);
        match decode_command(&payload).unwrap() {
            Command::Identify { correlation_id: 1, proto_version: 3 } => {}
            other => panic!("wrong variant: {other:?}"),
        }
    }
}

#[cfg(test)]
mod response_tests {
    use super::*;
    use kalico_native_transport::frame::decode_frame;

    #[test]
    fn push_response_decodes_back() {
        let frame = push_segment_response_frame(42, 0, 7);
        let (chan, payload) = decode_frame(&frame).unwrap();
        assert_eq!(chan, CHANNEL_CONTROL);
        let (hdr, body) = decode_message_header(payload).unwrap();
        assert_eq!(hdr.correlation_id, 42);
        assert_eq!(MessageKind::from_u16(hdr.kind_raw), Some(MessageKind::PushSegmentResponse));
        let r = PushSegmentResponse::decode(body).unwrap();
        assert_eq!(r.accepted_segment_id, 7);
        assert_eq!(r.result, 0);
    }

    #[test]
    fn load_response_carries_handle() {
        let frame = load_curve_response_frame(5, 0, 0x0002_0003);
        let (_chan, payload) = decode_frame(&frame).unwrap();
        let (_hdr, body) = decode_message_header(payload).unwrap();
        let r = LoadCurveResponse::decode(body).unwrap();
        assert_eq!(r.curve_handle_packed, 0x0002_0003);
    }
}
