use super::ring_depth_for_axis_inner;
use crate::mcu_config::{AXIS_X, AXIS_Y, AXIS_Z, McuAxisConfig, McuCaps};

fn configs() -> Vec<McuAxisConfig> {
    vec![
        McuAxisConfig {
            max_motor_velocity: Vec::new(),
            mcu_id: 1,
            axes: vec![AXIS_X, AXIS_Y],
            kinematics: 0,
            caps: McuCaps {
                total_piece_memory: 62 * 1024,
            },
        },
        McuAxisConfig {
            max_motor_velocity: Vec::new(),
            mcu_id: 2,
            axes: vec![AXIS_Z],
            kinematics: 1,
            caps: McuCaps {
                total_piece_memory: 62 * 1024,
            },
        },
    ]
}

#[test]
fn success_two_axis_mcu() {
    let expected = (1322 / 2) as u16;
    assert_eq!(
        ring_depth_for_axis_inner(&configs(), 1, AXIS_X as u8).unwrap(),
        expected
    );
    assert_eq!(
        ring_depth_for_axis_inner(&configs(), 1, AXIS_Y as u8).unwrap(),
        expected
    );
}

#[test]
fn success_single_axis_mcu() {
    let expected = 1322u16;
    assert_eq!(
        ring_depth_for_axis_inner(&configs(), 2, AXIS_Z as u8).unwrap(),
        expected
    );
}

#[test]
fn unknown_mcu_handle_errors() {
    let e = ring_depth_for_axis_inner(&configs(), 99, AXIS_X as u8).unwrap_err();
    assert!(e.contains("unknown mcu_handle 99"), "got: {e}");
}

#[test]
fn axis_not_on_mcu_errors() {
    let e = ring_depth_for_axis_inner(&configs(), 1, AXIS_Z as u8).unwrap_err();
    assert!(e.contains("not configured"), "got: {e}");
}

#[test]
fn ring_depth_over_u16_is_hard_error_not_clamp() {
    let configs = vec![McuAxisConfig {
        max_motor_velocity: Vec::new(),
        mcu_id: 0,
        axes: vec![AXIS_X],
        kinematics: 0,
        caps: McuCaps {
            total_piece_memory: 70_000 * 48,
        },
    }];
    let res = ring_depth_for_axis_inner(&configs, 0, AXIS_X as u8);
    assert!(
        res.is_err(),
        "depth > u16::MAX must be a hard error, not a clamp"
    );
    let e = res.unwrap_err();
    assert!(
        e.contains("exceeds u16::MAX"),
        "error message should mention u16::MAX, got: {e}"
    );
}
