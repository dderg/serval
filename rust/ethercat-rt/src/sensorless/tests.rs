use super::*;

#[test]
fn poll_below_threshold_does_not_trip() {
    let mut arm = SensorlessArm::new(4, 500);
    assert_eq!(arm.poll(100), None);
    assert_eq!(arm.poll(-499), None);
}

#[test]
fn poll_fires_once_on_threshold_cross_after_seeing_below() {
    let mut arm = SensorlessArm::new(7, 500);
    assert_eq!(arm.poll(499), None);
    assert_eq!(arm.poll(500), Some(7));
}

#[test]
fn poll_latches_and_fires_at_most_once() {
    let mut arm = SensorlessArm::new(3, 200);
    assert_eq!(arm.poll(100), None);
    assert_eq!(arm.poll(250), Some(3));
    assert_eq!(arm.poll(250), None);
    assert_eq!(arm.poll(5000), None);
}

#[test]
fn poll_trips_on_negative_torque_magnitude() {
    let mut arm = SensorlessArm::new(5, 300);
    assert_eq!(arm.poll(0), None);
    assert_eq!(arm.poll(-301), Some(5));
}

#[test]
fn poll_never_trips_if_torque_starts_at_or_above_threshold() {
    let mut arm = SensorlessArm::new(9, 400);
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
    bank.arm(0, 4, 500);
    bank.arm(1, 7, 500);

    assert!(drain(&mut bank, &[100, 100]).is_empty());
    assert_eq!(drain(&mut bank, &[600, 100]), vec![(0, 4, 600)]);
    assert_eq!(drain(&mut bank, &[100, 600]), vec![(1, 7, 600)]);
}

#[test]
fn bank_disarm_targets_one_slot_only() {
    let mut bank = SensorlessBank::new(2);
    bank.arm(0, 4, 500);
    bank.arm(1, 7, 500);
    bank.disarm(0);

    assert!(drain(&mut bank, &[100, 100]).is_empty());
    assert_eq!(drain(&mut bank, &[600, 600]), vec![(1, 7, 600)]);
}

#[test]
fn bank_reports_every_slot_that_crosses_in_one_cycle() {
    let mut bank = SensorlessBank::new(2);
    bank.arm(0, 4, 500);
    bank.arm(1, 7, 500);

    assert!(drain(&mut bank, &[100, 100]).is_empty());
    assert_eq!(
        drain(&mut bank, &[600, 600]),
        vec![(0, 4, 600), (1, 7, 600)]
    );
}
