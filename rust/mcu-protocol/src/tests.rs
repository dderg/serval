use super::result_codes;

#[test]
fn result_codes_are_stable() {
    assert_eq!(result_codes::OK, 0);
    assert_eq!(result_codes::RING_FULL, -309);
    assert_eq!(result_codes::STREAM_HALTED, -142);
    assert_eq!(result_codes::EC_PIECES_WHILE_HALTED, -315);
}
