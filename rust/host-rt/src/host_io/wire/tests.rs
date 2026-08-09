use super::*;

#[test]
fn decode_absolute_walks_within_one_epoch() {
    assert_eq!(decode_absolute(0, 0), 0);
    assert_eq!(decode_absolute(0, 1), 1);
    assert_eq!(decode_absolute(0, 5), 5);
}

#[test]
fn decode_absolute_handles_wrap() {
    assert_eq!(decode_absolute(15, 0), 16);
    assert_eq!(decode_absolute(15, 5), 21);
    assert_eq!(decode_absolute(31, 0), 32);
}

#[test]
fn crc16_matches_klipper_test_vector() {
    // Python: msgproto.crc16_ccitt(bytearray([5, 0x10])) returns 0x9E81.
    assert_eq!(crc16_ccitt(&[0x05, 0x10]), 0x9E81);
}

#[test]
fn build_frame_roundtrips() {
    let frame = build_frame(&[0x01, 0x02], 0);
    assert_eq!(frame[0], 5 + 2);
    assert_eq!(frame[1] & MESSAGE_SEQ_MASK, 0);
    assert_eq!(frame[1] & !MESSAGE_SEQ_MASK, MESSAGE_DEST);
    assert_eq!(*frame.last().unwrap(), MESSAGE_SYNC);
}

#[test]
fn retransmit_buffer_starts_with_sync() {
    let f1 = build_frame(&[1], 0);
    let f2 = build_frame(&[2], 1);
    let buf = build_retransmit_buffer([f1.as_slice(), f2.as_slice()]);
    assert_eq!(buf[0], MESSAGE_SYNC);
    assert_eq!(buf.len(), 1 + f1.len() + f2.len());
}

#[test]
fn pack_blocks_fills_a_block_before_opening_the_next() {
    let payloads = vec![vec![0xAA; 30], vec![0xBB; 29], vec![0xCC; 1]];
    let blocks = pack_blocks(&payloads).unwrap();
    assert_eq!(blocks.len(), 2);
    assert_eq!(blocks[0].len(), BLOCK_PAYLOAD_MAX);
    assert_eq!(&blocks[0][..30], &payloads[0][..]);
    assert_eq!(&blocks[0][30..], &payloads[1][..]);
    assert_eq!(blocks[1], payloads[2]);
}

#[test]
fn packed_blocks_preserve_command_order_and_bytes() {
    let payloads: Vec<Vec<u8>> = (0u8..40).map(|i| vec![i, i, i, i, i]).collect();
    let blocks = pack_blocks(&payloads).unwrap();
    let flat: Vec<u8> = blocks.concat();
    assert_eq!(flat, payloads.concat());
    for block in &blocks {
        assert!(block.len() <= BLOCK_PAYLOAD_MAX);
        assert!(MESSAGE_MIN + block.len() <= MESSAGE_MAX);
    }
}

#[test]
fn pack_blocks_never_splits_a_command_across_blocks() {
    let payloads = vec![vec![0x11; 40], vec![0x22; 40]];
    let blocks = pack_blocks(&payloads).unwrap();
    assert_eq!(blocks, payloads);
}

#[test]
fn a_command_too_wide_for_a_block_fails_loud() {
    let err = pack_blocks(&[vec![0u8; BLOCK_PAYLOAD_MAX + 1]]).unwrap_err();
    assert!(err.contains("block payload"), "{err}");
}
