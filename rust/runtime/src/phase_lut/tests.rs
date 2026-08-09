#![allow(clippy::integer_division)]
#![allow(clippy::indexing_slicing)]

use super::{COIL_AMPLITUDE, PHASE_LUT, PHASE_LUT_SIZE};

#[test]
fn phase_lut_has_expected_size() {
    assert_eq!(PHASE_LUT_SIZE, 1024);
    assert_eq!(PHASE_LUT.len(), PHASE_LUT_SIZE);
}

#[test]
fn anchors_match_expectation() {
    assert_eq!(PHASE_LUT[0], (COIL_AMPLITUDE, 0));
    assert_eq!(PHASE_LUT[PHASE_LUT_SIZE / 4], (0, COIL_AMPLITUDE));
    assert_eq!(PHASE_LUT[PHASE_LUT_SIZE / 2], (-COIL_AMPLITUDE, 0));
    assert_eq!(PHASE_LUT[3 * PHASE_LUT_SIZE / 4], (0, -COIL_AMPLITUDE));
}

#[test]
fn all_entries_within_amplitude() {
    for (i, (a, b)) in PHASE_LUT.iter().enumerate() {
        assert!(
            a.abs() <= COIL_AMPLITUDE,
            "PHASE_LUT[{i}].0 = {a} out of range"
        );
        assert!(
            b.abs() <= COIL_AMPLITUDE,
            "PHASE_LUT[{i}].1 = {b} out of range"
        );
    }
}
