#[test]
fn multi_bit_mask_is_rejected_by_stepper_sel() {
    assert!(runtime::piece_ring::stepper_sel_from_mask(0b0000_0011).is_err());
    assert!(runtime::piece_ring::stepper_sel_from_mask(0b0000_0010).is_ok());
    assert!(runtime::piece_ring::stepper_sel_from_mask(0).is_ok());
}
