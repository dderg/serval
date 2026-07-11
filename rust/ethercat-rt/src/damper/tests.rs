use super::*;

const CYCLE_NS: i64 = 250_000;

fn armed_bank(gain_milli: u32, clamp_tenths: u16, lpf_millihz: u32) -> DiffDamperBank {
    let mut bank = DiffDamperBank::new(CYCLE_NS);
    assert_eq!(bank.set(4, 0, 1, gain_milli, clamp_tenths, lpf_millihz), 0);
    bank
}

fn settle(bank: &mut DiffDamperBank, vel: &[f64], cycles: usize) -> Vec<f32> {
    let mut out = vec![0f32; vel.len()];
    for _ in 0..cycles {
        out.iter_mut().for_each(|t| *t = 0.0);
        bank.accumulate(vel, &mut out);
    }
    out
}

#[test]
fn set_rejects_bad_slots() {
    let mut bank = DiffDamperBank::new(CYCLE_NS);
    assert_eq!(bank.set(4, 2, 2, 1000, 50, 300_000), ERR_DAMPER_BAD_SLOT);
    assert_eq!(bank.set(4, 0, 4, 1000, 50, 300_000), ERR_DAMPER_BAD_SLOT);
    assert!(!bank.active());
}

#[test]
fn set_rejects_bad_clamp_and_lpf() {
    let mut bank = DiffDamperBank::new(CYCLE_NS);
    assert_eq!(bank.set(4, 0, 1, 1000, 0, 300_000), ERR_DAMPER_BAD_CLAMP);
    assert_eq!(
        bank.set(4, 0, 1, 1000, MAX_DAMPER_CLAMP_TENTHS + 1, 300_000),
        ERR_DAMPER_BAD_CLAMP
    );
    assert_eq!(bank.set(4, 0, 1, 1000, 50, 500), ERR_DAMPER_BAD_LPF);
    assert_eq!(
        bank.set(4, 0, 1, 1000, 50, MAX_DAMPER_LPF_MILLIHZ + 1),
        ERR_DAMPER_BAD_LPF
    );
    assert!(!bank.active());
}

#[test]
fn set_rejects_slot_shared_with_another_pair() {
    let mut bank = armed_bank(1000, 50, 300_000);
    assert_eq!(bank.set(4, 1, 2, 1000, 50, 300_000), ERR_DAMPER_SLOT_IN_USE);
    assert_eq!(bank.set(4, 2, 3, 1000, 50, 300_000), 0);
}

#[test]
fn set_replaces_the_same_pair_in_either_slot_order() {
    let mut bank = armed_bank(1000, 50, 300_000);
    assert_eq!(bank.set(4, 1, 0, 2000, 50, 300_000), 0);
    let out = settle(&mut bank, &[1.0, -1.0, 0.0, 0.0], 4000);
    assert!(
        (f64::from(out[1]) - 4.0).abs() < 0.05,
        "replacement gain applies to slot order b,a: {out:?}"
    );
}

#[test]
fn zero_gain_disarms_the_pair() {
    let mut bank = armed_bank(1000, 50, 300_000);
    assert_eq!(bank.set(4, 1, 0, 0, 0, 0), 0);
    assert!(!bank.active());
}

#[test]
fn torque_opposes_differential_velocity_and_is_antisymmetric() {
    let mut bank = armed_bank(2000, 500, 300_000);
    let out = settle(&mut bank, &[3.0, -2.0, 0.0, 0.0], 4000);
    let expected = -2.0 * 5.0;
    assert!(
        (f64::from(out[0]) - expected).abs() < 0.1,
        "slot 0 opposes its lead: {out:?}"
    );
    assert_eq!(out[1], -out[0]);
    assert_eq!(out[2], 0.0);
    assert_eq!(out[3], 0.0);
}

#[test]
fn common_mode_velocity_produces_no_torque() {
    let mut bank = armed_bank(5000, 500, 300_000);
    let out = settle(&mut bank, &[120.0, 120.0, 7.0, -7.0], 100);
    assert_eq!(out, vec![0.0; 4]);
}

#[test]
fn torque_is_clamped_per_pair() {
    let mut bank = armed_bank(50_000, 30, 300_000);
    let out = settle(&mut bank, &[10.0, -10.0, 0.0, 0.0], 4000);
    assert_eq!(out[0], -30.0);
    assert_eq!(out[1], 30.0);
}

#[test]
fn low_pass_needs_time_to_ramp() {
    let mut bank = armed_bank(1000, 500, 10_000);
    let mut out = vec![0f32; 2];
    bank.accumulate(&[10.0, -10.0], &mut out);
    let first = f64::from(out[0]).abs();
    let settled = f64::from(settle(&mut bank, &[10.0, -10.0], 4000)[0]).abs();
    assert!(first < 0.1 * settled, "first={first} settled={settled}");
    assert!((settled - 20.0).abs() < 0.2);
}

#[test]
fn reset_filters_clears_accumulated_state() {
    let mut bank = armed_bank(1000, 500, 300_000);
    settle(&mut bank, &[10.0, -10.0], 4000);
    bank.reset_filters();
    let mut out = vec![0f32; 2];
    bank.accumulate(&[0.0, 0.0], &mut out);
    assert_eq!(out, vec![0.0, 0.0]);
}
