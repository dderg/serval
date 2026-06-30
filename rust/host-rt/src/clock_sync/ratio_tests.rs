use super::*;

const REF_FREQ: f64 = 180_000_000.0;

#[test]
fn drift_ppm_between_zero_reference_is_zero() {
    assert_eq!(drift_ppm_between(REF_FREQ, 0.0), 0.0);
}

#[test]
fn drift_ppm_between_matches_definition() {
    let live = REF_FREQ * (1.0 + 50e-6);
    let ppm = drift_ppm_between(live, REF_FREQ);
    assert!((ppm - 50.0).abs() < 1e-6, "expected ~50 ppm, got {ppm}");

    let slow = REF_FREQ * (1.0 - 50e-6);
    assert!((drift_ppm_between(slow, REF_FREQ) + 50.0).abs() < 1e-6);
}

#[test]
fn within_authority_at_and_beyond_the_bound() {
    let just_inside = REF_FREQ * (1.0 + (MAX_DRIFT_PPM_DEFAULT - 1.0) * 1e-6);
    let just_outside = REF_FREQ * (1.0 + (MAX_DRIFT_PPM_DEFAULT + 1.0) * 1e-6);
    assert!(drift_within_authority(just_inside, REF_FREQ));
    assert!(!drift_within_authority(just_outside, REF_FREQ));

    let negative_outside = REF_FREQ * (1.0 - (MAX_DRIFT_PPM_DEFAULT + 1.0) * 1e-6);
    assert!(!drift_within_authority(negative_outside, REF_FREQ));

    let exactly_at = REF_FREQ * (1.0 + MAX_DRIFT_PPM_DEFAULT * 1e-6);
    assert!(
        drift_within_authority(exactly_at, REF_FREQ),
        "the bound is inclusive"
    );
}

#[test]
fn fault_threshold_is_three_consecutive_samples() {
    assert_eq!(MAX_CONSECUTIVE_OUT_OF_BOUND, 3);
}
