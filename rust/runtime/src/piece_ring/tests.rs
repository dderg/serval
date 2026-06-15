use super::*;

#[test]
fn piece_entry_to_le_bytes_matches_field_layout() {
    let p = PieceEntry {
        start_time: 0x0102_0304_0506_0708,
        coeffs: [1.0, 2.0, 3.0, 4.0],
        duration: 0.5,
        motor_mask: 0,
        _reserved: [0; 3],
    };
    let b = p.to_le_bytes();
    assert_eq!(b.len(), 32);
    assert_eq!(&b[0..8], &0x0102_0304_0506_0708u64.to_le_bytes());
    assert_eq!(&b[8..12], &1.0f32.to_le_bytes());
    assert_eq!(&b[12..16], &2.0f32.to_le_bytes());
    assert_eq!(&b[16..20], &3.0f32.to_le_bytes());
    assert_eq!(&b[20..24], &4.0f32.to_le_bytes());
    assert_eq!(&b[24..28], &0.5f32.to_le_bytes());
    assert_eq!(&b[28..32], &0u32.to_le_bytes());
}

#[test]
fn motor_mask_round_trips_at_byte_28() {
    let p = PieceEntry {
        start_time: 7,
        coeffs: [1.0, 2.0, 3.0, 4.0],
        duration: 0.5,
        motor_mask: 0b0000_0100,
        _reserved: [0; 3],
    };
    let b = p.to_le_bytes();
    assert_eq!(b[28], 0b0000_0100);
    assert_eq!(&b[29..32], &[0u8; 3]);
    let r = PieceEntry::from_le_bytes(&b);
    assert_eq!(r.motor_mask, 0b0000_0100);
    assert_eq!(r.start_time, 7);
}

#[test]
fn stepper_sel_from_mask_cases() {
    assert_eq!(stepper_sel_from_mask(0), Ok(0));
    assert_eq!(stepper_sel_from_mask(0b0000_0001), Ok(1));
    assert_eq!(stepper_sel_from_mask(0b0000_1000), Ok(4));
    assert_eq!(stepper_sel_from_mask(0b1000_0000), Ok(8));
    assert!(stepper_sel_from_mask(0b0000_0011).is_err());
}
