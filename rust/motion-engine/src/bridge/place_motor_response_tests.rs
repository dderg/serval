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
    assert_eq!(motors[7], None, "serial ignores the slot map");
}

#[test]
fn ethercat_maps_slot_through_slot_axis_map() {
    let resp = MotorStateResponse {
        motors: vec![sample(0, 4 * 65536, 6 * 65536)],
    };
    let mut motors = [None; MAX_AXES];
    let mut vmotors = [None; MAX_AXES];

    place_motor_response(&resp, &[1], true, &mut motors, &mut vmotors);

    assert_eq!(motors[1], Some(4.0), "slot 0 -> its mapped axis");
    assert_eq!(vmotors[1], Some(6.0));
    assert_eq!(motors[0], None, "no slot maps to axis 0");
}

#[test]
fn ethercat_awd_corexy_reports_pair_mean_per_axis() {
    // AWD corexy: two drives per belt, slot map [0, 0, 1, 1]. The pair's
    // differential is internal belt strain; the common mode (mean) is where
    // the carriage actually is, and it is what the parked resync adopts.
    let resp = MotorStateResponse {
        motors: vec![
            sample(3, 246 * 65536, 0),
            sample(0, 286 * 65536, 0),
            sample(1, 287 * 65536, 0),
            sample(2, 245 * 65536, 0),
        ],
    };
    let mut motors = [None; MAX_AXES];
    let mut vmotors = [None; MAX_AXES];

    place_motor_response(&resp, &[0, 0, 1, 1], true, &mut motors, &mut vmotors);

    assert_eq!(
        motors[0],
        Some(286.5),
        "axis 0 reports the belt-a pair mean"
    );
    assert_eq!(
        motors[1],
        Some(245.5),
        "axis 1 reports the belt-b pair mean"
    );
}

#[test]
fn ethercat_maps_multiple_motors_to_axes_by_slot() {
    let resp = MotorStateResponse {
        motors: vec![sample(0, 65536, 0), sample(1, 2 * 65536, 0)],
    };
    let mut motors = [None; MAX_AXES];
    let mut vmotors = [None; MAX_AXES];

    place_motor_response(&resp, &[3, 5], true, &mut motors, &mut vmotors);

    assert_eq!(motors[3], Some(1.0));
    assert_eq!(motors[5], Some(2.0));
}

#[test]
fn ethercat_maps_by_slot_field_not_arrival_order() {
    let resp = MotorStateResponse {
        motors: vec![sample(1, 2 * 65536, 0), sample(0, 65536, 0)],
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
