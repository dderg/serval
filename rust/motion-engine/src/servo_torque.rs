use std::time::Duration;

use host_rt::mcu_serial_conn::McuSerialConn;
use mcu_protocol::codec::Encode as _;
use mcu_protocol::messages::{
    ArmSensorlessEndstop, ArmSensorlessEndstopResponse, MessageKind, ResonanceBuzz,
    ResonanceBuzzResponse, RestoreDriveLimits, RestoreDriveLimitsResponse, SeedServoHome,
    SeedServoHomeResponse, SetDriveLimits, SetDriveLimitsResponse, SetTorque, SetTorqueResponse,
    StopResponse,
};

use crate::servo_call::mcu_typed_call;

const WORST_CASE_LADDER_ENABLE: Duration = Duration::from_secs(3);
const SET_TORQUE_TIMEOUT_MARGIN: Duration = Duration::from_secs(5);
const SET_TORQUE_TIMEOUT: Duration =
    WORST_CASE_LADDER_ENABLE.saturating_add(SET_TORQUE_TIMEOUT_MARGIN);

pub fn send_set_torque(
    conn: &McuSerialConn,
    value: bool,
    execute_at_ns: u64,
) -> Result<i32, String> {
    let body = SetTorque {
        value: u8::from(value),
        execute_at_ns,
    }
    .encoded_to_vec();
    let r: SetTorqueResponse = mcu_typed_call(
        conn,
        "SetTorque",
        MessageKind::SetTorque,
        MessageKind::SetTorqueResponse,
        body,
        SET_TORQUE_TIMEOUT,
    )?;
    Ok(r.result)
}

const DRIVE_LIMITS_TIMEOUT: Duration = Duration::from_secs(10);

pub fn send_drive_limits(
    conn: &McuSerialConn,
    slot: u8,
    following_error_counts: u32,
    max_torque_tenth_pct: u16,
) -> Result<i32, String> {
    let body = SetDriveLimits {
        slot,
        following_error_counts,
        max_torque_tenth_pct,
    }
    .encoded_to_vec();
    let r: SetDriveLimitsResponse = mcu_typed_call(
        conn,
        "SetDriveLimits",
        MessageKind::SetDriveLimits,
        MessageKind::SetDriveLimitsResponse,
        body,
        DRIVE_LIMITS_TIMEOUT,
    )?;
    Ok(r.result)
}

pub fn send_arm_sensorless_endstop(
    conn: &McuSerialConn,
    slot: u8,
    endstop_id: u8,
    torque_trip_tenth_pct: u16,
    enable: bool,
) -> Result<i32, String> {
    let body = ArmSensorlessEndstop {
        slot,
        endstop_id,
        torque_trip_tenth_pct,
        enable: u8::from(enable),
    }
    .encoded_to_vec();
    let r: ArmSensorlessEndstopResponse = mcu_typed_call(
        conn,
        "ArmSensorlessEndstop",
        MessageKind::ArmSensorlessEndstop,
        MessageKind::ArmSensorlessEndstopResponse,
        body,
        DRIVE_LIMITS_TIMEOUT,
    )?;
    Ok(r.result)
}

pub fn send_restore_drive_limits(conn: &McuSerialConn, slot: u8) -> Result<i32, String> {
    let body = RestoreDriveLimits { slot }.encoded_to_vec();
    let r: RestoreDriveLimitsResponse = mcu_typed_call(
        conn,
        "RestoreDriveLimits",
        MessageKind::RestoreDriveLimits,
        MessageKind::RestoreDriveLimitsResponse,
        body,
        DRIVE_LIMITS_TIMEOUT,
    )?;
    Ok(r.result)
}

pub fn send_seed_servo_home(
    conn: &McuSerialConn,
    slot: u8,
    home_q16: i32,
    timeout: Duration,
) -> Result<i32, String> {
    let body = SeedServoHome { slot, home_q16 }.encoded_to_vec();
    let r: SeedServoHomeResponse = mcu_typed_call(
        conn,
        "SeedServoHome",
        MessageKind::SeedServoHome,
        MessageKind::SeedServoHomeResponse,
        body,
        timeout,
    )?;
    Ok(r.result)
}

const STOP_TIMEOUT: Duration = Duration::from_secs(3);

pub fn send_stop(conn: &McuSerialConn) -> Result<i32, String> {
    let r: StopResponse = mcu_typed_call(
        conn,
        "Stop",
        MessageKind::Stop,
        MessageKind::StopResponse,
        Vec::new(),
        STOP_TIMEOUT,
    )?;
    Ok(r.result)
}

const RESONANCE_BUZZ_TIMEOUT: Duration = Duration::from_secs(5);

#[allow(clippy::too_many_arguments)]
pub fn send_resonance_buzz(
    conn: &McuSerialConn,
    axis_mask: u8,
    sign_mask: u8,
    freq_start_millihz: u32,
    freq_end_millihz: u32,
    amplitude_nm: u32,
    duration_ms: u32,
    ramp_ms: u32,
) -> Result<i32, String> {
    let body = ResonanceBuzz {
        axis_mask,
        sign_mask,
        freq_start_millihz,
        freq_end_millihz,
        amplitude_nm,
        duration_ms,
        ramp_ms,
    }
    .encoded_to_vec();
    let r: ResonanceBuzzResponse = mcu_typed_call(
        conn,
        "ResonanceBuzz",
        MessageKind::ResonanceBuzz,
        MessageKind::ResonanceBuzzResponse,
        body,
        RESONANCE_BUZZ_TIMEOUT,
    )?;
    Ok(r.result)
}

#[cfg(test)]
mod tests;
