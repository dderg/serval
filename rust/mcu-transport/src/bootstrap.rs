use crate::wire_helpers::{MESSAGE_VERSION_DEFAULT, encode_message_header};
use mcu_protocol::bootstrap::{IDENTIFY_BODY_LEN, IDENTIFY_RESPONSE_BODY_LEN};
use mcu_protocol::{MessageKind, PER_MESSAGE_HEADER_LEN};

pub use mcu_protocol::bootstrap::IdentifyResponse;

pub const BOOTSTRAP_IDENTIFY_BODY_LEN: usize = IDENTIFY_BODY_LEN;
pub const BOOTSTRAP_IDENTIFY_RESPONSE_BODY_LEN: usize = IDENTIFY_RESPONSE_BODY_LEN;

pub const BOOTSTRAP_IDENTIFY_LEN: usize = PER_MESSAGE_HEADER_LEN + BOOTSTRAP_IDENTIFY_BODY_LEN;
pub const BOOTSTRAP_IDENTIFY_RESPONSE_LEN: usize =
    PER_MESSAGE_HEADER_LEN + BOOTSTRAP_IDENTIFY_RESPONSE_BODY_LEN;

pub fn encode_identify(correlation_id: u32, proto_version: u8) -> Vec<u8> {
    let mut out = Vec::with_capacity(BOOTSTRAP_IDENTIFY_LEN);
    out.extend_from_slice(&encode_message_header(
        MessageKind::Identify,
        MESSAGE_VERSION_DEFAULT,
        correlation_id,
    ));
    out.push(proto_version);
    out
}

pub fn decode_identify_response(payload: &[u8]) -> Option<(u32, IdentifyResponse)> {
    if payload.len() != BOOTSTRAP_IDENTIFY_RESPONSE_LEN {
        return None;
    }
    let (header, body) = crate::wire_helpers::decode_message_header(payload)?;
    if header.kind_raw != MessageKind::IdentifyResponse as u16 {
        return None;
    }
    let resp = IdentifyResponse::decode_body(body).ok()?;
    Some((header.correlation_id, resp))
}

pub fn encode_identify_response(correlation_id: u32, resp: &IdentifyResponse) -> Vec<u8> {
    let mut out = Vec::with_capacity(BOOTSTRAP_IDENTIFY_RESPONSE_LEN);
    out.extend_from_slice(&encode_message_header(
        MessageKind::IdentifyResponse,
        MESSAGE_VERSION_DEFAULT,
        correlation_id,
    ));
    resp.encode_body(&mut out);
    debug_assert_eq!(out.len(), BOOTSTRAP_IDENTIFY_RESPONSE_LEN);
    out
}

#[cfg(test)]
mod tests;
