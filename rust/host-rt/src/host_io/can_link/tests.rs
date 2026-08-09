use super::*;

const UUID: u64 = 0x1122_3344_5566;

#[test]
fn query_payload_requests_the_extended_form() {
    assert_eq!(query_extended_payload(), [0x00, 0x01]);
}

#[test]
fn set_nodeid_payload_matches_wire_layout() {
    let frame = set_nodeid_payload(UUID, 0x40);
    assert_eq!(
        frame,
        [0x01, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x40],
        "cmd, 6-byte big-endian uuid, nodeid"
    );
}

#[test]
fn need_nodeid_response_round_trips_uuid_as_unassigned() {
    let data = [0x20, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x01];
    let node = parse_query_response(&data).expect("need-nodeid response");
    assert_eq!(node.uuid, UUID);
    assert_eq!(node.assignment, NodeAssignment::Unassigned);
}

#[test]
fn kalico_application_marker_is_accepted_as_unassigned() {
    let data = [0x20, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x07];
    let node = parse_query_response(&data).expect("kalico need-nodeid response");
    assert_eq!(node.assignment, NodeAssignment::Unassigned);
}

#[test]
fn have_nodeid_response_reports_the_existing_assignment() {
    let data = [0x21, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x40];
    let node = parse_query_response(&data).expect("have-nodeid response");
    assert_eq!(node.uuid, UUID);
    assert_eq!(node.assignment, NodeAssignment::AlreadyAssigned(0x40));
}

#[test]
fn foreign_application_marker_and_short_frames_are_rejected() {
    assert!(parse_query_response(&[0x20, 1, 2, 3, 4, 5, 6, 0x00]).is_none());
    assert!(parse_query_response(&[0x20, 1, 2, 3, 4, 5, 6]).is_none());
    assert!(parse_query_response(&[0x99, 1, 2, 3, 4, 5, 6, 1]).is_none());
}

#[test]
fn query_response_uuid_mismatch_is_visible() {
    let data = [0x20, 0x11, 0x22, 0x33, 0x44, 0x55, 0x67, 0x01];
    let node = parse_query_response(&data).expect("need-nodeid response");
    assert_ne!(node.uuid, UUID);
}

#[test]
fn data_ids_derive_from_nodeid() {
    assert_eq!(tx_id(0x40), 0x180);
    assert_eq!(rx_id(0x40), 0x181);
    assert_eq!(tx_id(0x41), 0x182);
    assert_eq!(rx_id(0x41), 0x183);
    assert_eq!(rx_id(NODEID_LAST), tx_id(NODEID_LAST) + 1);
    assert!(rx_id(NODEID_LAST) <= 0x7ff, "data ids must stay 11-bit");
}

#[test]
fn uuid_bytes_take_the_low_six_bytes_big_endian() {
    assert_eq!(uuid_bytes(UUID), [0x11, 0x22, 0x33, 0x44, 0x55, 0x66]);
}

#[test]
fn fd_chunk_selection_only_uses_exact_fit_sizes() {
    assert_eq!(fd_chunk_len(0), 0);
    assert_eq!(fd_chunk_len(1), 1);
    assert_eq!(fd_chunk_len(8), 8);
    assert_eq!(fd_chunk_len(9), 8, "no legal FD size below 12");
    assert_eq!(fd_chunk_len(11), 8);
    assert_eq!(fd_chunk_len(12), 12);
    assert_eq!(fd_chunk_len(13), 12);
    assert_eq!(fd_chunk_len(15), 12);
    assert_eq!(fd_chunk_len(16), 16);
    assert_eq!(fd_chunk_len(19), 16);
    assert_eq!(fd_chunk_len(20), 20);
    assert_eq!(fd_chunk_len(23), 20);
    assert_eq!(fd_chunk_len(24), 24);
    assert_eq!(fd_chunk_len(31), 24);
    assert_eq!(fd_chunk_len(32), 32);
    assert_eq!(fd_chunk_len(47), 32);
    assert_eq!(fd_chunk_len(48), 48);
    assert_eq!(fd_chunk_len(63), 48);
    assert_eq!(fd_chunk_len(64), 64);
    assert_eq!(fd_chunk_len(65), 64);
    assert_eq!(fd_chunk_len(100), 64);
}

#[test]
fn fd_chunking_never_pads_the_byte_stream() {
    for total in 0usize..=200 {
        let mut remaining = total;
        while remaining > 0 {
            let take = fd_chunk_len(remaining);
            assert!(take > 0 && take <= remaining, "chunk {take} of {remaining}");
            assert!(
                take <= CAN_MAX_DLEN || CANFD_PAYLOAD_SIZES.contains(&take),
                "chunk {take} is not a legal FD payload size"
            );
            remaining -= take;
        }
    }
}

#[test]
fn classic_chunking_caps_at_eight() {
    assert_eq!(chunk_len(FrameFormat::Classic, 3), 3);
    assert_eq!(chunk_len(FrameFormat::Classic, 64), CAN_MAX_DLEN);
    assert_eq!(chunk_len(FrameFormat::Fd, 64), 64);
}

#[test]
fn short_payloads_encode_as_classic_frames() {
    for len in 0usize..=CAN_MAX_DLEN {
        let payload: Vec<u8> = (0..len as u8).collect();
        let frame = encode_frame(0x180, &payload).expect("classic frame");
        assert_eq!(frame.format(), FrameFormat::Classic);
        let wire = frame.as_bytes();
        assert_eq!(wire.len(), CAN_FRAME_SIZE);
        assert_eq!(wire[..4], 0x180u32.to_ne_bytes());
        assert_eq!(usize::from(wire[4]), len);
        assert_eq!(&wire[5..8], &[0, 0, 0], "classic pad bytes stay zero");
        assert_eq!(&wire[8..8 + len], &payload[..]);
    }
}

#[test]
fn long_payloads_encode_as_fd_frames_with_brs_set() {
    let payload: Vec<u8> = (0..64u8).collect();
    let frame = encode_frame(0x181, &payload).expect("fd frame");
    assert_eq!(frame.format(), FrameFormat::Fd);
    let wire = frame.as_bytes();
    assert_eq!(wire.len(), CANFD_FRAME_SIZE);
    assert_eq!(wire[..4], 0x181u32.to_ne_bytes());
    assert_eq!(wire[4], 64);
    assert_eq!(wire[5], CANFD_BRS_FLAG, "BRS must be set on every FD frame");
    assert_eq!(&wire[6..8], &[0, 0], "reserved bytes stay zero");
    assert_eq!(&wire[8..72], &payload[..]);
}

#[test]
fn nine_byte_payload_is_the_first_fd_frame() {
    let frame = encode_frame(0x180, &[7u8; 9]).expect("fd frame");
    assert_eq!(frame.format(), FrameFormat::Fd);
    assert_eq!(
        frame.as_bytes()[4],
        9,
        "no padding to a legal DLC on the wire"
    );
}

#[test]
fn payload_beyond_the_fd_limit_is_rejected() {
    let err = encode_frame(0x180, &[0u8; 65]).expect_err("overflow must fail loudly");
    assert!(err.to_string().contains("65"), "{err}");
}

#[test]
fn rx_dispatches_on_the_exact_datagram_length() {
    let classic = encode_frame(0x3f1, &[1, 2, 3]).expect("classic");
    let (id, payload) = decode_frame(classic.as_bytes()).expect("classic decode");
    assert_eq!(id, 0x3f1);
    assert_eq!(payload, &[1, 2, 3]);

    let long: Vec<u8> = (0..48u8).collect();
    let fd = encode_frame(0x181, &long).expect("fd");
    let (id, payload) = decode_frame(fd.as_bytes()).expect("fd decode");
    assert_eq!(id, 0x181);
    assert_eq!(payload, &long[..]);
}

#[test]
fn bogus_datagram_lengths_are_rejected() {
    for len in [0usize, 8, 15, 17, 32, 71, 73, 88] {
        let buf = vec![0u8; len];
        assert!(
            decode_frame(&buf).is_err(),
            "datagram of {len} bytes must not decode"
        );
    }
}

#[test]
fn classic_datagram_cannot_declare_an_fd_length() {
    let mut buf = vec![0u8; CAN_FRAME_SIZE];
    buf[4] = 12;
    assert!(decode_frame(&buf).is_err());
}

#[test]
fn frame_format_follows_the_interface_mtu() {
    assert_eq!(from_mtu(16).expect("classic mtu"), FrameFormat::Classic);
    assert_eq!(from_mtu(72).expect("fd mtu"), FrameFormat::Fd);
    assert!(from_mtu(1500).is_err());
}
