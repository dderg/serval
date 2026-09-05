use super::drip_cohort_participants;
use crate::mcu_config::{AXIS_X, AXIS_Y, AXIS_Z, McuAxisConfig};
use crate::types::AxisKey;

const FOLLOWER_E: usize = 3;

fn cfg(mcu_id: u32, axes: Vec<usize>) -> McuAxisConfig {
    McuAxisConfig {
        max_motor_velocity: Vec::new(),
        mcu_id,
        axes,
        kinematics: 1,
        ethercat: false,
        ..Default::default()
    }
}

#[test]
fn includes_every_configured_axis_so_lane_3_enqueues_stay_in_cohort() {
    let configs = vec![
        cfg(0, vec![AXIS_Y, AXIS_Z, FOLLOWER_E]),
        cfg(1, vec![AXIS_X]),
    ];
    let participants = drip_cohort_participants(&configs);
    assert_eq!(
        participants,
        vec![
            AxisKey {
                mcu_id: 0,
                axis: AXIS_Y as u8
            },
            AxisKey {
                mcu_id: 0,
                axis: AXIS_Z as u8
            },
            AxisKey {
                mcu_id: 0,
                axis: FOLLOWER_E as u8
            },
            AxisKey {
                mcu_id: 1,
                axis: AXIS_X as u8
            },
        ]
    );
}
