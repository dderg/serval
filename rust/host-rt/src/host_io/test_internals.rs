use super::*;

#[test]
fn extract_packet_picks_up_minimal_nak_frame() {
    let crc = wire::crc16_ccitt(&[0x05, 0x10]);
    let frame = vec![
        0x05,
        0x10,
        (crc >> 8) as u8,
        (crc & 0xFF) as u8,
        wire::MESSAGE_SYNC,
    ];
    let mut buf = frame.clone();
    let extracted = wire::extract_packet(&mut buf).expect("must extract NAK");
    assert_eq!(extracted, frame);
    assert!(buf.is_empty());
}

#[test]
fn extract_packet_resyncs_past_garbage_byte_smaller_than_message_min() {
    let mut buf: Vec<u8> = vec![0x02];
    let result = wire::extract_packet(&mut buf);
    assert!(
        result.is_none(),
        "still no complete frame, but buf must have been drained"
    );
    assert!(
        buf.is_empty(),
        "garbage leading byte should have been dropped, got {buf:?}"
    );
}

#[test]
fn extract_packet_resyncs_past_oversized_msglen_byte() {
    let mut buf: Vec<u8> = vec![0xFF];
    let result = wire::extract_packet(&mut buf);
    assert!(result.is_none());
    assert!(
        buf.is_empty(),
        "oversized msglen byte should have been dropped, got {buf:?}"
    );
}

#[test]
fn decode_vlq_caps_continuation_at_5_bytes() {
    let malformed = vec![0xFFu8; 8];
    let result = parser::decode_vlq(&malformed);
    assert!(
        matches!(result, Err(parser::ParseError::BadVlq)),
        "malformed VLQ must return BadVlq, not roll past 5 bytes"
    );
}
