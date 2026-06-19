#![allow(clippy::integer_division)]
#![allow(clippy::indexing_slicing)]

use super::{COIL_AMPLITUDE, PHASE_LUT, PHASE_LUT_SIZE};

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

#[test]
fn coil_for_phase_matches_dispatch_table() {
    use super::coil_for_phase;
    for i in 0..PHASE_LUT_SIZE {
        assert_eq!(
            coil_for_phase(i as u16),
            PHASE_LUT[i],
            "coil_for_phase({i}) diverged from PHASE_LUT[{i}]"
        );
    }
}

#[test]
fn coil_for_phase_wraps_past_one_cycle() {
    use super::coil_for_phase;
    assert_eq!(coil_for_phase(PHASE_LUT_SIZE as u16), PHASE_LUT[0]);
    assert_eq!(coil_for_phase(PHASE_LUT_SIZE as u16 + 7), PHASE_LUT[7]);
}

#[test]
fn legacy_lut_entries_anchors() {
    use super::{CURRENT_AMPLITUDE, LUT_ENTRIES, MOTOR_PERIOD};
    // LUT_ENTRIES[i] = (sin, cos)
    assert_eq!(LUT_ENTRIES[0], (0, CURRENT_AMPLITUDE));
    assert_eq!(LUT_ENTRIES[MOTOR_PERIOD / 4], (CURRENT_AMPLITUDE, 0));
}
