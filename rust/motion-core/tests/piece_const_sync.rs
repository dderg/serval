//! The piece wire geometry is defined independently in `runtime` (ring/ISR
//! side) and `mcu-protocol` (codec side, which cannot depend on runtime).
//! This test is the seam that keeps them equal.

#[test]
fn piece_wire_constants_match_across_crates() {
    assert_eq!(
        mcu_protocol::messages::PIECE_WIRE_HEADER_LEN,
        runtime::piece_ring::PIECE_WIRE_HEADER_LEN
    );
    assert_eq!(
        mcu_protocol::messages::PIECE_WIRE_MAX_LEN,
        runtime::piece_ring::PIECE_ENTRY_BYTES
    );
    assert_eq!(
        mcu_protocol::messages::MAX_PIECE_COEFFS,
        runtime::piece_ring::MAX_PIECE_COEFFS
    );
    assert_eq!(
        core::mem::size_of::<runtime::piece_ring::PieceEntry>(),
        runtime::piece_ring::PIECE_ENTRY_BYTES
    );
}

#[test]
fn wire_entry_is_prefix_of_ring_slot() {
    let mut entry = runtime::piece_ring::PieceEntry::zeroed();
    entry.start_time = 0x1122_3344_5566_7788;
    entry.duration = 0.025;
    entry.motor_mask = 2;
    entry.coeff_count = 5;
    for (k, c) in entry.coeffs.iter_mut().enumerate() {
        *c = k as f32 + 0.5;
    }
    let slot = entry.to_le_bytes();
    let mut wire = Vec::new();
    entry.to_wire_bytes(&mut wire);
    assert_eq!(wire.len(), entry.wire_len());
    assert_eq!(&slot[..wire.len()], wire.as_slice());
}
