use super::*;
use crate::mcu_config::{AXIS_X, AXIS_Y, AXIS_Z, LaneKind};

fn cfg() -> McuAxisConfig {
    McuAxisConfig {
        mcu_id: 7,
        axes: vec![AXIS_X, AXIS_Y, AXIS_Z],
        lane_kinds: vec![LaneKind::Pulse, LaneKind::Phase, LaneKind::PhaseWithPulse],
        ..Default::default()
    }
}

fn key(axis: u8) -> AxisKey {
    AxisKey { mcu_id: 7, axis }
}

#[test]
fn a_lane_starts_on_the_transport_its_kind_names() {
    let t = AxisTransports::from_configs(&[cfg()]);
    assert!(t.is_pulse(key(AXIS_X as u8)));
    assert!(t.is_phase(key(AXIS_Y as u8)));
    assert!(
        t.is_phase(key(AXIS_Z as u8)),
        "a dual lane prints in phase mode; the pulse binding is the exception"
    );
}

#[test]
fn only_a_dual_lane_can_be_switched_to_pulse() {
    let t = AxisTransports::from_configs(&[cfg()]);
    assert_eq!(
        t.adopt(key(AXIS_Z as u8), TRANSPORT_PULSE),
        Ok(TRANSPORT_PHASE)
    );
    assert!(t.is_pulse(key(AXIS_Z as u8)));

    let err = t
        .adopt(key(AXIS_Y as u8), TRANSPORT_PULSE)
        .expect_err("a phase-only lane has no classic step/dir binding to switch to");
    assert!(err.contains("phase only"), "{err}");
}

#[test]
fn a_pulse_only_lane_cannot_be_switched_to_phase() {
    let t = AxisTransports::from_configs(&[cfg()]);
    let err = t
        .adopt(key(AXIS_X as u8), TRANSPORT_PHASE)
        .expect_err("a step/dir lane has no phase binding");
    assert!(err.contains("pulse only"), "{err}");
}

#[test]
fn an_unknown_lane_is_never_routable() {
    let t = AxisTransports::from_configs(&[cfg()]);
    let stranger = AxisKey { mcu_id: 9, axis: 0 };
    assert!(!t.supports(stranger, TRANSPORT_PULSE));
    assert!(!t.supports(stranger, TRANSPORT_PHASE));
    assert!(t.adopt(stranger, TRANSPORT_PULSE).is_err());
}

#[test]
fn a_dual_lane_round_trips_between_its_two_bindings() {
    let t = AxisTransports::from_configs(&[cfg()]);
    let z = key(AXIS_Z as u8);
    assert_eq!(t.adopt(z, TRANSPORT_PULSE), Ok(TRANSPORT_PHASE));
    assert_eq!(t.adopt(z, TRANSPORT_PHASE), Ok(TRANSPORT_PULSE));
    assert!(t.is_phase(z));
}
