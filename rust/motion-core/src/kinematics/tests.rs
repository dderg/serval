use super::*;

#[test]
fn from_tag_zero_is_corexy_one_is_cartesian() {
    assert_eq!(
        KinematicsModule::from_tag(0).unwrap().kind(),
        KinematicsKind::CoreXy
    );
    assert_eq!(
        KinematicsModule::from_tag(1).unwrap().kind(),
        KinematicsKind::Cartesian
    );
}

#[test]
fn from_tag_unknown_is_loud() {
    assert!(KinematicsModule::from_tag(7).is_err());
}

#[test]
fn corexy_forward_matches_legacy_values() {
    let m = KinematicsModule::from_tag(0).unwrap();
    assert_eq!(m.forward([150.0, 150.0, 50.0]), [300.0, 0.0, 50.0]);
    assert_eq!(m.forward([10.0, 4.0, 0.0]), [14.0, 6.0, 0.0]);
}

#[test]
fn corexy_roundtrip_is_identity() {
    let m = KinematicsModule::from_tag(0).unwrap();
    let axes = [12.5, -3.25, 7.0];
    let back = m.inverse(m.forward(axes));
    for (a, b) in axes.iter().zip(back.iter()) {
        assert!((a - b).abs() < 1e-12);
    }
}

#[test]
fn cartesian_is_identity_lanes() {
    let m = KinematicsModule::from_tag(1).unwrap();
    assert!(m.lane_is_identity(0) && m.lane_is_identity(1) && m.lane_is_identity(2));
    assert_eq!(m.forward([1.0, 2.0, 3.0]), [1.0, 2.0, 3.0]);
}

#[test]
fn corexy_lane_weights_are_sum_and_difference() {
    let m = KinematicsModule::from_tag(0).unwrap();
    assert_eq!(m.lane_weights(0), [1.0, 1.0, 0.0]);
    assert_eq!(m.lane_weights(1), [1.0, -1.0, 0.0]);
    assert_eq!(m.lane_weights(2), [0.0, 0.0, 1.0]);
    assert!(!m.lane_is_identity(0));
    assert!(m.lane_is_identity(2));
}
