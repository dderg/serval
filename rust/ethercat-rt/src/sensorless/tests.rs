use super::*;

#[test]
fn poll_below_threshold_does_not_trip() {
    let mut arm = SensorlessArm::new(0, 4, 500);
    assert_eq!(arm.poll(100), None);
    assert_eq!(arm.poll(-499), None);
}

#[test]
fn poll_fires_once_on_threshold_cross_after_seeing_below() {
    let mut arm = SensorlessArm::new(0, 7, 500);
    assert_eq!(arm.poll(499), None);
    assert_eq!(arm.poll(500), Some(7));
}

#[test]
fn poll_latches_and_fires_at_most_once() {
    let mut arm = SensorlessArm::new(0, 3, 200);
    assert_eq!(arm.poll(100), None);
    assert_eq!(arm.poll(250), Some(3));
    assert_eq!(arm.poll(250), None);
    assert_eq!(arm.poll(5000), None);
}

#[test]
fn poll_trips_on_negative_torque_magnitude() {
    let mut arm = SensorlessArm::new(0, 5, 300);
    assert_eq!(arm.poll(0), None);
    assert_eq!(arm.poll(-301), Some(5));
}

#[test]
fn poll_never_trips_if_torque_starts_at_or_above_threshold() {
    let mut arm = SensorlessArm::new(0, 9, 400);
    assert_eq!(arm.poll(400), None);
    assert_eq!(arm.poll(6000), None);
    assert_eq!(arm.poll(450), None);
    assert_eq!(arm.poll(399), None);
    assert_eq!(arm.poll(450), Some(9));
}
