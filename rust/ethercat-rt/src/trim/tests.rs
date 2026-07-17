use super::*;

const CYCLE_NS: i64 = 250_000;
const CYCLE_S: f64 = 250e-6;
/// Pair stiffness seen by the trim loop: 0.1% rated torque per mm of
/// antisymmetric offset. From the bench: ~15.7% fight per 0.059 mm strain.
const STIFFNESS_TENTHS_PER_MM: f64 = 2660.0;

fn armed_bank(gain_micro: u32, clamp_um: u16, lpf_millihz: u32) -> DiffTrimBank {
    let mut bank = DiffTrimBank::new(CYCLE_NS);
    assert_eq!(bank.set(4, 0, 1, gain_micro, clamp_um, lpf_millihz, 0), 0);
    bank
}

/// Simulates the pair as a spring on the position difference: with the two
/// rotors held at their targets, trapped strain `strain_mm` produces
/// diff_torque = K*(strain + 2*offset) — the trim's antisymmetric offset
/// moves both rotors, so nulling the fight needs offset = -strain/2.
/// Returns (final_diff_tenths, final_offset_mm_slot0).
fn run_spring(
    bank: &mut DiffTrimBank,
    strain_mm: f64,
    quiescent: bool,
    cycles: usize,
) -> (f64, f64) {
    let mut offset = vec![0f64; 4];
    let mut diff_tenths = 0.0;
    for _ in 0..cycles {
        diff_tenths = (strain_mm + 2.0 * offset[0]) * STIFFNESS_TENTHS_PER_MM;
        let torques = [diff_tenths, -diff_tenths, 0.0, 0.0];
        offset.iter_mut().for_each(|o| *o = 0.0);
        bank.update(&torques, &[quiescent; 4], &mut offset);
    }
    (diff_tenths, offset[0])
}

#[test]
fn set_rejects_bad_slots_gain_clamp_lpf_and_settle() {
    let mut bank = DiffTrimBank::new(CYCLE_NS);
    assert_eq!(bank.set(4, 2, 2, 50_000, 150, 25_000, 0), ERR_TRIM_BAD_SLOT);
    assert_eq!(bank.set(4, 0, 4, 50_000, 150, 25_000, 0), ERR_TRIM_BAD_SLOT);
    assert_eq!(
        bank.set(4, 0, 1, MAX_TRIM_GAIN_MICRO + 1, 150, 25_000, 0),
        ERR_TRIM_BAD_GAIN
    );
    assert_eq!(
        bank.set(4, 0, 1, 50_000, MAX_TRIM_CLAMP_UM + 1, 25_000, 0),
        ERR_TRIM_BAD_CLAMP
    );
    assert_eq!(bank.set(4, 0, 1, 50_000, 150, 50, 0), ERR_TRIM_BAD_LPF);
    assert_eq!(
        bank.set(4, 0, 1, 50_000, 150, MAX_TRIM_LPF_MILLIHZ + 1, 0),
        ERR_TRIM_BAD_LPF
    );
    assert_eq!(
        bank.set(4, 0, 1, 50_000, 150, 25_000, MAX_TRIM_SETTLE_MS + 1),
        ERR_TRIM_BAD_SETTLE
    );
    assert!(!bank.active());
}

#[test]
fn set_rejects_slot_shared_with_another_pair() {
    let mut bank = armed_bank(50_000, 150, 25_000);
    assert_eq!(
        bank.set(4, 1, 2, 50_000, 150, 25_000, 0),
        ERR_TRIM_SLOT_IN_USE
    );
    assert_eq!(bank.set(4, 2, 3, 50_000, 150, 25_000, 0), 0);
}

#[test]
fn zero_clamp_removes_the_pair_in_either_slot_order() {
    let mut bank = armed_bank(50_000, 150, 25_000);
    assert_eq!(bank.set(4, 1, 0, 50_000, 0, 0, 0), 0);
    assert!(!bank.active());
}

#[test]
fn zero_gain_freezes_the_offset_but_keeps_it_applied() {
    let mut bank = armed_bank(50_000, 150, 25_000);
    let (_diff, settled) = run_spring(&mut bank, 0.06, true, 40_000);
    assert!(settled.abs() > 0.02);
    assert_eq!(bank.set(4, 1, 0, 0, 150, 25_000, 0), 0);
    let mut offset = vec![0f64; 4];
    for _ in 0..4_000 {
        offset.iter_mut().for_each(|o| *o = 0.0);
        bank.update(&[400.0, -400.0, 0.0, 0.0], &[true; 4], &mut offset);
    }
    assert!(bank.active());
    assert_eq!(
        offset[0], settled,
        "gain 0 must hold the learned offset, not drop it"
    );
}

#[test]
fn reconfiguring_a_pair_keeps_offset_and_filter_state() {
    let mut bank = armed_bank(50_000, 150, 25_000);
    let (_diff, settled) = run_spring(&mut bank, 0.06, true, 40_000);
    let filtered = bank.snapshot()[0].3;
    assert_eq!(bank.set(4, 1, 0, 100_000, 300, 10_000, 100), 0);
    let (_a, _b, offset_mm, filtered_after, _integrating) = bank.snapshot()[0];
    assert_eq!(offset_mm, settled, "retune must not discard the offset");
    assert_eq!(filtered_after, filtered, "retune must not reset the filter");
}

#[test]
fn tightening_the_clamp_reclamps_the_held_offset() {
    let mut bank = armed_bank(50_000, 150, 25_000);
    let (_diff, settled) = run_spring(&mut bank, 0.06, true, 40_000);
    assert!(settled < -0.02);
    assert_eq!(bank.set(4, 0, 1, 50_000, 10, 25_000, 0), 0);
    assert_eq!(bank.snapshot()[0].2, -0.01);
}

#[test]
fn integrator_nulls_a_standing_fight() {
    let mut bank = armed_bank(50_000, 150, 25_000);
    // 0.06 mm of trapped strain = ~16% fight, the bench X-belt case.
    let (diff, offset) = run_spring(&mut bank, 0.06, true, 40_000);
    assert!(
        diff.abs() < 1.0,
        "fight must be nulled below 0.1%: {diff} tenths left"
    );
    assert!(
        (offset + 0.03).abs() < 0.001,
        "each rotor must absorb half the strain: {offset}"
    );
}

#[test]
fn outputs_are_antisymmetric_and_other_slots_untouched() {
    let mut bank = armed_bank(50_000, 150, 25_000);
    let mut offset = vec![0f64; 4];
    for _ in 0..4_000 {
        offset.iter_mut().for_each(|o| *o = 0.0);
        bank.update(&[80.0, -80.0, 55.0, 55.0], &[true; 4], &mut offset);
    }
    assert!(offset[0] < 0.0);
    assert_eq!(offset[0], -offset[1]);
    assert_eq!(offset[2], 0.0);
    assert_eq!(offset[3], 0.0);
}

#[test]
fn common_mode_torque_produces_no_offset() {
    let mut bank = armed_bank(2_000_000, 500, 25_000);
    let mut offset = vec![0f64; 4];
    for _ in 0..4_000 {
        offset.iter_mut().for_each(|o| *o = 0.0);
        bank.update(&[250.0, 250.0, 0.0, 0.0], &[true; 4], &mut offset);
    }
    assert_eq!(offset[0], 0.0);
    assert_eq!(offset[1], 0.0);
}

#[test]
fn frozen_pair_holds_offset_and_filter_while_not_quiescent() {
    let mut bank = armed_bank(50_000, 150, 25_000);
    let (_diff, settled) = run_spring(&mut bank, 0.06, true, 40_000);
    let filtered_before = bank.snapshot()[0].3;
    let mut offset = vec![0f64; 4];
    for _ in 0..4_000 {
        offset.iter_mut().for_each(|o| *o = 0.0);
        bank.update(&[300.0, -300.0, 0.0, 0.0], &[false; 4], &mut offset);
    }
    assert_eq!(
        offset[0], settled,
        "non-quiescent cycles must not move the offset"
    );
    assert_eq!(
        bank.snapshot()[0].3,
        filtered_before,
        "in-motion torque must not pollute the filter"
    );
}

#[test]
fn settle_window_blinds_the_trim_after_motion() {
    let settle_ms = 100;
    let settle_cycles = (f64::from(settle_ms) * 1e-3 / CYCLE_S).ceil() as usize;
    let mut bank = DiffTrimBank::new(CYCLE_NS);
    assert_eq!(bank.set(4, 0, 1, 50_000, 150, 25_000, settle_ms), 0);
    let mut offset = vec![0f64; 4];
    bank.update(&[400.0, -400.0, 0.0, 0.0], &[false; 4], &mut offset);
    for _ in 0..settle_cycles {
        offset.iter_mut().for_each(|o| *o = 0.0);
        bank.update(&[400.0, -400.0, 0.0, 0.0], &[true; 4], &mut offset);
        assert_eq!(offset[0], 0.0, "must stay blind through the settle window");
        assert_eq!(bank.snapshot()[0].3, 0.0);
    }
    for _ in 0..1_000 {
        offset.iter_mut().for_each(|o| *o = 0.0);
        bank.update(&[400.0, -400.0, 0.0, 0.0], &[true; 4], &mut offset);
    }
    assert!(offset[0] < 0.0, "must integrate once the window has passed");
}

#[test]
fn any_motion_cycle_restarts_the_settle_window() {
    let settle_ms = 100;
    let settle_cycles = (f64::from(settle_ms) * 1e-3 / CYCLE_S).ceil() as usize;
    let mut bank = DiffTrimBank::new(CYCLE_NS);
    assert_eq!(bank.set(4, 0, 1, 50_000, 150, 25_000, settle_ms), 0);
    let mut offset = vec![0f64; 4];
    for _ in 0..settle_cycles {
        offset.iter_mut().for_each(|o| *o = 0.0);
        bank.update(&[400.0, -400.0, 0.0, 0.0], &[true; 4], &mut offset);
    }
    bank.update(&[400.0, -400.0, 0.0, 0.0], &[false; 4], &mut offset);
    for _ in 0..settle_cycles {
        offset.iter_mut().for_each(|o| *o = 0.0);
        bank.update(&[400.0, -400.0, 0.0, 0.0], &[true; 4], &mut offset);
        assert_eq!(offset[0], 0.0, "one motion cycle must restart the window");
    }
}

#[test]
fn quiescence_of_one_slot_is_not_enough() {
    let mut bank = armed_bank(50_000, 150, 25_000);
    let mut offset = vec![0f64; 4];
    for _ in 0..4_000 {
        offset.iter_mut().for_each(|o| *o = 0.0);
        bank.update(
            &[400.0, -400.0, 0.0, 0.0],
            &[true, false, true, true],
            &mut offset,
        );
    }
    assert_eq!(offset[0], 0.0);
}

#[test]
fn offset_clamps_and_warns_exactly_once_per_arm() {
    let mut bank = armed_bank(2_000_000, 50, 25_000);
    let mut offset = vec![0f64; 4];
    for _ in 0..8_000 {
        offset.iter_mut().for_each(|o| *o = 0.0);
        bank.update(&[400.0, -400.0, 0.0, 0.0], &[true; 4], &mut offset);
    }
    assert_eq!(offset[0], -0.05);
    assert_eq!(bank.drain_clamp_warning(), Some((0, 1)));
    assert_eq!(bank.drain_clamp_warning(), None);
    offset.iter_mut().for_each(|o| *o = 0.0);
    bank.update(&[400.0, -400.0, 0.0, 0.0], &[true; 4], &mut offset);
    assert_eq!(bank.drain_clamp_warning(), None, "warn once per arm");
}

#[test]
fn slew_is_capped_against_torque_transients() {
    let mut bank = armed_bank(2_000_000, 500, 100_000);
    let mut offset = vec![0f64; 4];
    let mut prev = 0.0;
    for _ in 0..2_000 {
        offset.iter_mut().for_each(|o| *o = 0.0);
        bank.update(&[3_000.0, -3_000.0, 0.0, 0.0], &[true; 4], &mut offset);
        let step = (offset[0] - prev).abs();
        assert!(
            step <= 2.0 * CYCLE_S * 1.0001,
            "offset stepped {step} mm in one cycle"
        );
        prev = offset[0];
    }
}

#[test]
fn reset_clears_offset_filter_settle_and_warning_state() {
    let mut bank = DiffTrimBank::new(CYCLE_NS);
    assert_eq!(bank.set(4, 0, 1, 2_000_000, 50, 25_000, 100), 0);
    let mut offset = vec![0f64; 4];
    for _ in 0..8_000 {
        offset.iter_mut().for_each(|o| *o = 0.0);
        bank.update(&[400.0, -400.0, 0.0, 0.0], &[true; 4], &mut offset);
    }
    assert!(offset[0] < 0.0, "must have integrated before the reset");
    bank.reset();
    assert_eq!(bank.drain_clamp_warning(), None);
    offset.iter_mut().for_each(|o| *o = 0.0);
    bank.update(&[400.0, -400.0, 0.0, 0.0], &[true; 4], &mut offset);
    assert_eq!(offset[0], 0.0, "reset must restart the settle window too");
    assert_eq!(offset[1], 0.0);
}
