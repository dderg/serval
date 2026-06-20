#![allow(
    clippy::unwrap_used,
    clippy::indexing_slicing,
    clippy::cast_possible_truncation
)]

use super::*;

const TWO_PI_F64: f64 = 2.0 * core::f64::consts::PI;

#[test]
fn envelope_is_zero_at_both_ends_and_unity_on_flat_top() {
    let total = 1.0;
    let ramp = 0.1;
    assert_eq!(envelope(0.0, total, ramp), 0.0);
    assert_eq!(envelope(total, total, ramp), 0.0);
    assert!((envelope(0.5, total, ramp) - 1.0).abs() < 1e-12);
}

#[test]
fn envelope_never_exceeds_unity_or_drops_below_zero() {
    let total = 0.2;
    let ramp = 0.05;
    let mut t = 0.0;
    while t <= total {
        let e = envelope(t, total, ramp);
        assert!((0.0..=1.0).contains(&e), "env({t}) = {e} out of range");
        t += total / 200.0;
    }
}

#[test]
fn arm_rejects_out_of_range_arguments() {
    let buzz = Buzz::new();
    assert_eq!(buzz.arm(4, 0b1, 0, 0, 100_000, 10_000, 1000, 100), -1);
    assert_eq!(buzz.arm(4, 0b1, 0, 100_000, 0, 10_000, 1000, 100), -1);
    assert_eq!(
        buzz.arm(4, 0b1, 0, 9_000_000, 9_000_000, 10_000, 1000, 100),
        -1
    );
    assert_eq!(
        buzz.arm(4, 0b1, 0, 100_000, 100_000, 9_000_000, 1000, 100),
        -1
    );
    // axis bit beyond num_axes (axis 4 set, only 0..3 valid)
    assert_eq!(
        buzz.arm(4, 0b1_0000, 0, 100_000, 100_000, 10_000, 1000, 100),
        -1
    );
}

#[test]
fn disarm_form_yields_no_excitations() {
    let mut buzz = Buzz::new();
    // amplitude 0 == disarm; accepted (returns 0), resolves to an empty set.
    assert_eq!(buzz.arm(4, 0b11, 0, 100_000, 100_000, 0, 100, 10), 0);
    assert!(buzz.has_pending());
    assert!(buzz.take_excitations().is_empty());
    assert!(!buzz.has_pending());
}

#[test]
fn tone_resolves_to_one_excitation_per_set_axis() {
    let mut buzz = Buzz::new();
    assert_eq!(buzz.arm(4, 0b101, 0, 100_000, 100_000, 50_000, 200, 20), 0);
    let ex = buzz.take_excitations();
    assert_eq!(ex.len(), 2);
    assert_eq!(ex[0].axis_idx, 0);
    assert_eq!(ex[1].axis_idx, 2);
    // Tone: zero chirp rate, omega = 2*pi*100 Hz.
    assert_eq!(ex[0].mu, 0.0);
    assert!((ex[0].omega - TWO_PI_F64 * 100.0).abs() < 1e-6);
    // 50_000 nm displacement = 0.05 mm.
    assert!((ex[0].amplitude_mm - 0.05).abs() < 1e-12);
    assert!((ex[0].total_seconds - 0.2).abs() < 1e-12);
}

#[test]
fn corexy_pair_is_anti_phase_via_sign_mask() {
    // axis 0 (+) and axis 1 (-): cartesian-Y excitation on CoreXY.
    let mut buzz = Buzz::new();
    assert_eq!(
        buzz.arm(4, 0b11, 0b10, 100_000, 100_000, 50_000, 200, 20),
        0
    );
    let ex = buzz.take_excitations();
    assert_eq!(ex.len(), 2);
    assert_eq!(ex[0].sign, 1.0);
    assert_eq!(ex[1].sign, -1.0);
    // Same carrier (phase-coherent), opposite sign.
    assert_eq!(ex[0].omega, ex[1].omega);
    assert_eq!(ex[0].amplitude_mm, ex[1].amplitude_mm);
}

#[test]
fn chirp_sets_nonzero_slope_matching_band_edges() {
    let mut buzz = Buzz::new();
    // 20 -> 120 Hz over 200 ms.
    assert_eq!(buzz.arm(4, 0b01, 0, 20_000, 120_000, 100_000, 200, 20), 0);
    let ex = buzz.take_excitations();
    assert_eq!(ex.len(), 1);
    let e = ex[0];
    assert!((e.omega - TWO_PI_F64 * 20.0).abs() < 1e-6);
    // omega_inst(total) must land on 2*pi*120.
    let omega_end = e.omega + e.mu * e.total_seconds;
    assert!((omega_end - TWO_PI_F64 * 120.0).abs() < 1e-6);
}

#[test]
fn ramp_is_clamped_to_half_total() {
    let mut buzz = Buzz::new();
    // 50 ms ramp on a 60 ms tone would overlap; clamp to 30 ms.
    assert_eq!(buzz.arm(4, 0b01, 0, 100_000, 100_000, 50_000, 60, 50), 0);
    let ex = buzz.take_excitations();
    assert_eq!(ex.len(), 1);
    assert!((ex[0].ramp_seconds - 0.03).abs() < 1e-12);
}

#[test]
fn into_params_carries_engine_owned_fields() {
    let mut buzz = Buzz::new();
    assert_eq!(buzz.arm(4, 0b01, 0, 100_000, 100_000, 50_000, 200, 20), 0);
    let ex = buzz.take_excitations();
    let p = ex[0].into_params(1.25, 0.000_781_25, 520_000_000.0, 7_777);
    assert_eq!(p.base_mm, 1.25);
    assert_eq!(p.microstep_distance, 0.000_781_25);
    assert_eq!(p.cycles_per_second, 520_000_000.0);
    assert_eq!(p.anchor_cycle, 7_777);
    assert_eq!(p.omega, ex[0].omega as f32);
    assert_eq!(p.sign, ex[0].sign as f32);
}
