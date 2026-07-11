use super::*;

const CYCLE_NS: i64 = 250_000;
const CYCLE_S: f64 = 250e-6;

fn armed_bank(gain_milli: u32, clamp_tenths: u16, lpf_millihz: u32) -> DiffDamperBank {
    armed_bank_with_lead(gain_milli, clamp_tenths, lpf_millihz, 0)
}

fn armed_bank_with_lead(
    gain_milli: u32,
    clamp_tenths: u16,
    lpf_millihz: u32,
    lead_us: u16,
) -> DiffDamperBank {
    let mut bank = DiffDamperBank::new(CYCLE_NS);
    assert_eq!(
        bank.set(4, 0, 1, gain_milli, clamp_tenths, lpf_millihz, lead_us),
        0
    );
    bank
}

/// Walks every slot at a constant host-frame velocity (mm/s) and returns the
/// torques of the final cycle.
fn settle(bank: &mut DiffDamperBank, vel_mm_s: &[f64], cycles: usize) -> Vec<f32> {
    let mut pos = vec![0f64; vel_mm_s.len()];
    let mut out = vec![0f32; vel_mm_s.len()];
    for _ in 0..cycles {
        for (p, v) in pos.iter_mut().zip(vel_mm_s) {
            *p += v * CYCLE_S;
        }
        out.iter_mut().for_each(|t| *t = 0.0);
        bank.accumulate(&pos, &mut out);
    }
    out
}

#[test]
fn set_rejects_bad_slots() {
    let mut bank = DiffDamperBank::new(CYCLE_NS);
    assert_eq!(bank.set(4, 2, 2, 1000, 50, 300_000, 0), ERR_DAMPER_BAD_SLOT);
    assert_eq!(bank.set(4, 0, 4, 1000, 50, 300_000, 0), ERR_DAMPER_BAD_SLOT);
    assert!(!bank.active());
}

#[test]
fn set_rejects_bad_clamp_lpf_and_lead() {
    let mut bank = DiffDamperBank::new(CYCLE_NS);
    assert_eq!(bank.set(4, 0, 1, 1000, 0, 300_000, 0), ERR_DAMPER_BAD_CLAMP);
    assert_eq!(
        bank.set(4, 0, 1, 1000, MAX_DAMPER_CLAMP_TENTHS + 1, 300_000, 0),
        ERR_DAMPER_BAD_CLAMP
    );
    assert_eq!(bank.set(4, 0, 1, 1000, 50, 500, 0), ERR_DAMPER_BAD_LPF);
    assert_eq!(
        bank.set(4, 0, 1, 1000, 50, MAX_DAMPER_LPF_MILLIHZ + 1, 0),
        ERR_DAMPER_BAD_LPF
    );
    assert_eq!(
        bank.set(4, 0, 1, 1000, 50, 300_000, MAX_DAMPER_LEAD_US + 1),
        ERR_DAMPER_BAD_LEAD
    );
    assert!(!bank.active());
}

#[test]
fn set_rejects_slot_shared_with_another_pair() {
    let mut bank = armed_bank(1000, 50, 300_000);
    assert_eq!(
        bank.set(4, 1, 2, 1000, 50, 300_000, 0),
        ERR_DAMPER_SLOT_IN_USE
    );
    assert_eq!(bank.set(4, 2, 3, 1000, 50, 300_000, 0), 0);
}

#[test]
fn set_replaces_the_same_pair_in_either_slot_order() {
    let mut bank = armed_bank(1000, 50, 300_000);
    assert_eq!(bank.set(4, 1, 0, 2000, 50, 300_000, 0), 0);
    let out = settle(&mut bank, &[1.0, -1.0, 0.0, 0.0], 4000);
    assert!(
        (f64::from(out[1]) - 4.0).abs() < 0.05,
        "replacement gain applies to slot order b,a: {out:?}"
    );
}

#[test]
fn zero_gain_disarms_the_pair() {
    let mut bank = armed_bank(1000, 50, 300_000);
    assert_eq!(bank.set(4, 1, 0, 0, 0, 0, 0), 0);
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
    let first = f64::from(settle(&mut bank, &[10.0, -10.0], 2)[0]).abs();
    let settled = f64::from(settle(&mut bank, &[10.0, -10.0], 4000)[0]).abs();
    assert!(first < 0.1 * settled, "first={first} settled={settled}");
    assert!((settled - 20.0).abs() < 0.2);
}

#[test]
fn first_cycle_after_arm_only_seeds_the_differentiator() {
    let mut bank = armed_bank(1000, 500, 300_000);
    let mut out = vec![0f32; 2];
    // A large standing position offset must not read as a velocity spike.
    bank.accumulate(&[40.0, -40.0], &mut out);
    assert_eq!(out, vec![0.0, 0.0]);
}

#[test]
fn lead_at_constant_velocity_changes_nothing() {
    let mut plain = armed_bank(2000, 500, 300_000);
    let mut led = armed_bank_with_lead(2000, 500, 300_000, 1000);
    let expected = f64::from(settle(&mut plain, &[3.0, -2.0], 4000)[0]);
    let got = f64::from(settle(&mut led, &[3.0, -2.0], 4000)[0]);
    assert!(
        (got - expected).abs() < 0.1,
        "got={got} expected={expected}"
    );
}

#[test]
fn lead_advances_the_torque_on_a_sinusoidal_mode() {
    // 170 Hz differential oscillation: the led torque must peak EARLIER in
    // the cycle than the plain one by roughly atan(2*pi*f*T_lead).
    let freq_hz = 170.0;
    let lead_us = 1000u16;
    let mut plain = armed_bank(1000, 900, 1_200_000);
    let mut led = armed_bank_with_lead(1000, 900, 1_200_000, lead_us);
    let peak_cycle = |bank: &mut DiffDamperBank| {
        let cycles = (4.0 / (freq_hz * CYCLE_S)) as usize;
        let mut out = vec![0f32; 2];
        let mut best = (0usize, f32::MIN);
        for c in 0..cycles {
            let t = c as f64 * CYCLE_S;
            let x = 0.01 * libm::sin(2.0 * std::f64::consts::PI * freq_hz * t);
            out.iter_mut().for_each(|v| *v = 0.0);
            bank.accumulate(&[x, -x], &mut out);
            let last_period = t > 3.0 / freq_hz;
            if last_period && out[0] > best.1 {
                best = (c, out[0]);
            }
        }
        best.0 as f64 * CYCLE_S
    };
    let advance_s = peak_cycle(&mut plain) - peak_cycle(&mut led);
    let expected_s = libm::atan(2.0 * std::f64::consts::PI * freq_hz * f64::from(lead_us) * 1e-6)
        / (2.0 * std::f64::consts::PI * freq_hz);
    assert!(
        (advance_s - expected_s).abs() < 2.5 * CYCLE_S,
        "advance={advance_s} expected={expected_s}"
    );
}

#[test]
fn reset_filters_clears_accumulated_state() {
    let mut bank = armed_bank(1000, 500, 300_000);
    settle(&mut bank, &[10.0, -10.0], 4000);
    bank.reset_filters();
    let mut out = vec![0f32; 2];
    bank.accumulate(&[100.0, -100.0], &mut out);
    assert_eq!(out, vec![0.0, 0.0]);
}
