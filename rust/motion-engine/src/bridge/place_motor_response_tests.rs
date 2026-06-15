use super::place_motor_response;
use mcu_protocol::messages::{MotorSample, MotorStateResponse};
use runtime::stepping_state::MAX_AXES;

fn sample(slot: u8, pos_q16: i32, vel_q16: i32) -> MotorSample {
    MotorSample {
        slot,
        pos_q16,
        vel_q16,
    }
}

#[test]
fn serial_uses_response_slot_index() {
    let resp = MotorStateResponse {
        motors: vec![sample(2, 3 * 65536, 5 * 65536)],
    };
    let mut motors = [None; MAX_AXES];
    let mut vmotors = [None; MAX_AXES];

    place_motor_response(&resp, &[7], false, &mut motors, &mut vmotors);

    assert_eq!(motors[2], Some(3.0));
    assert_eq!(vmotors[2], Some(5.0));
    assert_eq!(motors[7], None, "serial ignores cfg.axes");
}

#[test]
fn ethercat_maps_reply_onto_cfg_axes_ignoring_local_slot() {
    let resp = MotorStateResponse {
        motors: vec![sample(0, 4 * 65536, 6 * 65536)],
    };
    let mut motors = [None; MAX_AXES];
    let mut vmotors = [None; MAX_AXES];

    place_motor_response(&resp, &[1], true, &mut motors, &mut vmotors);

    assert_eq!(motors[1], Some(4.0), "first motor -> first cfg axis");
    assert_eq!(vmotors[1], Some(6.0));
    assert_eq!(motors[0], None, "endpoint slot:0 placeholder ignored");
}

#[test]
fn ethercat_zips_multiple_motors_to_axes_in_order() {
    let resp = MotorStateResponse {
        motors: vec![sample(0, 65536, 0), sample(0, 2 * 65536, 0)],
    };
    let mut motors = [None; MAX_AXES];
    let mut vmotors = [None; MAX_AXES];

    place_motor_response(&resp, &[3, 5], true, &mut motors, &mut vmotors);

    assert_eq!(motors[3], Some(1.0));
    assert_eq!(motors[5], Some(2.0));
}

#[test]
fn out_of_range_slots_are_dropped_not_panic() {
    let resp = MotorStateResponse {
        motors: vec![sample(250, 65536, 0)],
    };
    let mut motors = [None; MAX_AXES];
    let mut vmotors = [None; MAX_AXES];

    place_motor_response(&resp, &[MAX_AXES], true, &mut motors, &mut vmotors);
    place_motor_response(&resp, &[0], false, &mut motors, &mut vmotors);

    assert!(motors.iter().all(Option::is_none));
}
