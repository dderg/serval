use super::*;

const UUID: u64 = 0x1122_3344_5566;

#[test]
fn payload_chunks_splits_on_eight_with_short_tail() {
    let bytes: Vec<u8> = (0u8..20).collect();
    let chunks: Vec<&[u8]> = payload_chunks(&bytes).collect();
    assert_eq!(chunks.len(), 3);
    assert_eq!(chunks[0], &bytes[0..8]);
    assert_eq!(chunks[1], &bytes[8..16]);
    assert_eq!(chunks[2], &bytes[16..20]);
    assert_eq!(chunks.iter().map(|c| c.len()).sum::<usize>(), bytes.len());
}

#[test]
fn payload_chunks_exact_multiple_has_no_empty_tail() {
    let bytes = [0u8; 16];
    let chunks: Vec<&[u8]> = payload_chunks(&bytes).collect();
    assert_eq!(chunks.len(), 2);
    assert!(chunks.iter().all(|c| c.len() == CAN_MAX_DLEN));
}

#[test]
fn payload_chunks_of_empty_slice_yields_nothing() {
    assert_eq!(payload_chunks(&[]).count(), 0);
}

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
