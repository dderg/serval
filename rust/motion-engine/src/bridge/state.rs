use std::sync::Arc;
use std::sync::mpsc::Receiver;

use host_rt::host_io::McuHostIo;
use host_rt::mcu_serial_conn::McuSerialConn;

use crate::kinematics::SPATIAL_AXES;

pub(crate) struct HomingRun {
    pub(crate) cohort: u64,
    pub(crate) endstop_id: u8,
    pub(crate) endstop_mcu: u32,
    pub(crate) axis: u8,
    pub(crate) axis_key: crate::types::AxisKey,
    pub(crate) all_axis_keys: Vec<crate::types::AxisKey>,
    pub(crate) window_start_host: f64,
    pub(crate) notify: crossbeam_channel::Sender<Result<([f64; 3], [f64; 3], u64), String>>,
}

pub(crate) fn trip_position_to_motor_frame(
    axis: u8,
    motor_pos: f64,
    _configs: &[crate::mcu_config::McuAxisConfig],
    _axis_mcu: u32,
) -> [f64; SPATIAL_AXES] {
    assert!(
        (axis as usize) < SPATIAL_AXES,
        "follower axis {axis} in homing trip is a bug — a follower axis must never reach homing recovery"
    );
    let mut frame = [0.0f64; SPATIAL_AXES];
    frame[axis as usize] = motor_pos;
    frame
}

pub(crate) struct McuConnection {
    pub(crate) label: String,
    pub(crate) host_io: Option<Arc<McuHostIo>>,
    pub(crate) runtime_rx_priority:
        Option<Receiver<host_rt::host_io::runtime_events::RuntimeEvent>>,
    pub(crate) runtime_rx_bulk: Option<Receiver<host_rt::host_io::runtime_events::RuntimeEvent>>,
    pub(crate) runtime_caps: Option<mcu_protocol::messages::RuntimeCapsResponse>,
    pub(crate) identify_caps: u64,
    pub(crate) mcu_transport_supported: bool,
    pub(crate) ethercat_socket: Option<String>,
    pub(crate) endpoint_process: Option<std::process::Child>,
    pub(crate) endpoint_conn: Option<Arc<McuSerialConn>>,
    pub(crate) ethercat_slot_axes: Vec<usize>,
}

pub(crate) type EthercatDrive = (
    i32,
    usize,
    f64,
    f64,
    Option<u32>,
    Option<u16>,
    bool,
    f64,
    bool,
    Option<String>,
);

#[derive(Debug, Clone)]

pub(crate) struct FlushWait {
    pub(crate) rx: crossbeam_channel::Receiver<Option<std::time::Instant>>,
    pub(crate) deadline: Option<std::time::Instant>,
}
