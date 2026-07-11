use super::*;

#[test]
fn poll_below_threshold_does_not_trip() {
    let mut arm = SensorlessArm::new(4, 500, None);
    assert_eq!(arm.poll(100), None);
    assert_eq!(arm.poll(-499), None);
}

#[test]
fn poll_fires_once_on_threshold_cross_after_seeing_below() {
    let mut arm = SensorlessArm::new(7, 500, None);
    assert_eq!(arm.poll(499), None);
    assert_eq!(arm.poll(500), Some(7));
}

#[test]
fn poll_latches_and_fires_at_most_once() {
    let mut arm = SensorlessArm::new(3, 200, None);
    assert_eq!(arm.poll(100), None);
    assert_eq!(arm.poll(250), Some(3));
    assert_eq!(arm.poll(250), None);
    assert_eq!(arm.poll(5000), None);
}

#[test]
fn poll_trips_on_negative_torque_magnitude() {
    let mut arm = SensorlessArm::new(5, 300, None);
    assert_eq!(arm.poll(0), None);
    assert_eq!(arm.poll(-301), Some(5));
}

#[test]
fn poll_never_trips_if_torque_starts_at_or_above_threshold() {
    let mut arm = SensorlessArm::new(9, 400, None);
    assert_eq!(arm.poll(400), None);
    assert_eq!(arm.poll(6000), None);
    assert_eq!(arm.poll(450), None);
    assert_eq!(arm.poll(399), None);
    assert_eq!(arm.poll(450), Some(9));
}

fn drain(bank: &mut SensorlessBank, torque: &[i16]) -> Vec<(usize, u8, i16)> {
    let mut fired = Vec::new();
    bank.poll(
        |slot| torque[slot],
        |slot, endstop_id, t| fired.push((slot, endstop_id, t)),
    );
    fired
}

#[test]
fn bank_arms_slots_independently_without_clobber() {
    let mut bank = SensorlessBank::new(2);
    bank.arm(0, 4, 500, None);
    bank.arm(1, 7, 500, None);

    assert!(drain(&mut bank, &[100, 100]).is_empty());
    assert_eq!(drain(&mut bank, &[600, 100]), vec![(0, 4, 600)]);
    assert_eq!(drain(&mut bank, &[100, 600]), vec![(1, 7, 600)]);
}

#[test]
fn bank_disarm_targets_one_slot_only() {
    let mut bank = SensorlessBank::new(2);
    bank.arm(0, 4, 500, None);
    bank.arm(1, 7, 500, None);
    bank.disarm(0);

    assert!(drain(&mut bank, &[100, 100]).is_empty());
    assert_eq!(drain(&mut bank, &[600, 600]), vec![(1, 7, 600)]);
}

#[test]
fn bank_reports_every_slot_that_crosses_in_one_cycle() {
    let mut bank = SensorlessBank::new(2);
    bank.arm(0, 4, 500, None);
    bank.arm(1, 7, 500, None);

    assert!(drain(&mut bank, &[100, 100]).is_empty());
    assert_eq!(
        drain(&mut bank, &[600, 600]),
        vec![(0, 4, 600), (1, 7, 600)]
    );
}

fn paired_bank(threshold: u16) -> SensorlessBank {
    let mut bank = SensorlessBank::new(2);
    bank.arm(0, 4, threshold, Some(1));
    assert!(drain(&mut bank, &[0, 0]).is_empty());
    bank
}

#[test]
fn paired_arm_ignores_pure_belt_pair_fight() {
    let mut bank = paired_bank(300);
    // A standing fight (or the differential damper's injection) is equal and
    // opposite in the mechanical frame — far past threshold per drive, zero
    // common mode.
    assert!(drain(&mut bank, &[2000, -2000]).is_empty());
    assert!(drain(&mut bank, &[-1500, 1500]).is_empty());
}

#[test]
fn paired_arm_trips_on_common_mode_crash_torque() {
    let mut bank = paired_bank(300);
    assert_eq!(drain(&mut bank, &[350, 350]), vec![(0, 4, 350)]);
}

#[test]
fn paired_arm_trips_on_crash_riding_on_top_of_fight() {
    let mut bank = paired_bank(300);
    // Fight of ±250 alone stays below threshold on the common mode...
    assert!(drain(&mut bank, &[250, -250]).is_empty());
    // ...and a crash adding the same 320 to both trips at exactly 320.
    assert_eq!(drain(&mut bank, &[570, 70]), vec![(0, 4, 320)]);
}

#[test]
fn paired_arm_trips_on_negative_direction_crash() {
    let mut bank = paired_bank(300);
    assert_eq!(drain(&mut bank, &[-400, -300]), vec![(0, 4, -350)]);
}

#[test]
fn unpaired_slot_still_trips_on_its_own_reading() {
    let mut bank = SensorlessBank::new(2);
    bank.arm(0, 4, 300, None);
    assert!(drain(&mut bank, &[0, 0]).is_empty());
    assert_eq!(drain(&mut bank, &[2000, -2000]), vec![(0, 4, 2000)]);
}
