use std::time::Duration;

use host_rt::mcu_call::McuCall as _;
use host_rt::mcu_serial_conn::McuSerialConn;
use mcu_protocol::codec::{Decode as _, Encode as _};
use mcu_protocol::messages::{
    ArmSensorlessEndstop, ArmSensorlessEndstopResponse, MessageKind, ResonanceBuzz,
    ResonanceBuzzResponse, RestoreDriveLimits, RestoreDriveLimitsResponse, SeedServoHome,
    SeedServoHomeResponse, SetDriveLimits, SetDriveLimitsResponse, SetTorque, SetTorqueResponse,
};

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
    let (kind, resp) = conn
        .mcu_call(MessageKind::SetTorque, body, SET_TORQUE_TIMEOUT)
        .map_err(|e| format!("SetTorque transport: {e:?}"))?;
    if kind != MessageKind::SetTorqueResponse {
        return Err(format!(
            "SetTorque: unexpected response kind 0x{:04x}",
            kind.as_u16()
        ));
    }
    let r =
        SetTorqueResponse::decode(&resp).map_err(|e| format!("SetTorqueResponse decode: {e:?}"))?;
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
    let (kind, resp) = conn
        .mcu_call(MessageKind::SetDriveLimits, body, DRIVE_LIMITS_TIMEOUT)
        .map_err(|e| format!("SetDriveLimits transport: {e:?}"))?;
    if kind != MessageKind::SetDriveLimitsResponse {
        return Err(format!(
            "SetDriveLimits: unexpected response kind 0x{:04x}",
            kind.as_u16()
        ));
    }
    let r = SetDriveLimitsResponse::decode(&resp)
        .map_err(|e| format!("SetDriveLimitsResponse decode: {e:?}"))?;
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
    let (kind, resp) = conn
        .mcu_call(
            MessageKind::ArmSensorlessEndstop,
            body,
            DRIVE_LIMITS_TIMEOUT,
        )
        .map_err(|e| format!("ArmSensorlessEndstop transport: {e:?}"))?;
    if kind != MessageKind::ArmSensorlessEndstopResponse {
        return Err(format!(
            "ArmSensorlessEndstop: unexpected response kind 0x{:04x}",
            kind.as_u16()
        ));
    }
    let r = ArmSensorlessEndstopResponse::decode(&resp)
        .map_err(|e| format!("ArmSensorlessEndstopResponse decode: {e:?}"))?;
    Ok(r.result)
}

pub fn send_restore_drive_limits(conn: &McuSerialConn, slot: u8) -> Result<i32, String> {
    let body = RestoreDriveLimits { slot }.encoded_to_vec();
    let (kind, resp) = conn
        .mcu_call(MessageKind::RestoreDriveLimits, body, DRIVE_LIMITS_TIMEOUT)
        .map_err(|e| format!("RestoreDriveLimits transport: {e:?}"))?;
    if kind != MessageKind::RestoreDriveLimitsResponse {
        return Err(format!(
            "RestoreDriveLimits: unexpected response kind 0x{:04x}",
            kind.as_u16()
        ));
    }
    let r = RestoreDriveLimitsResponse::decode(&resp)
        .map_err(|e| format!("RestoreDriveLimitsResponse decode: {e:?}"))?;
    Ok(r.result)
}

pub fn send_seed_servo_home(
    conn: &McuSerialConn,
    slot: u8,
    home_q16: i32,
    timeout: Duration,
) -> Result<i32, String> {
    let body = SeedServoHome { slot, home_q16 }.encoded_to_vec();
    let (kind, resp) = conn
        .mcu_call(MessageKind::SeedServoHome, body, timeout)
        .map_err(|e| format!("SeedServoHome transport: {e:?}"))?;
    if kind != MessageKind::SeedServoHomeResponse {
        return Err(format!(
            "SeedServoHome: unexpected response kind 0x{:04x}",
            kind.as_u16()
        ));
    }
    let r = SeedServoHomeResponse::decode(&resp)
        .map_err(|e| format!("SeedServoHomeResponse decode: {e:?}"))?;
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
    let (kind, resp) = conn
        .mcu_call(MessageKind::ResonanceBuzz, body, RESONANCE_BUZZ_TIMEOUT)
        .map_err(|e| format!("ResonanceBuzz transport: {e:?}"))?;
    if kind != MessageKind::ResonanceBuzzResponse {
        return Err(format!(
            "ResonanceBuzz: unexpected response kind 0x{:04x}",
            kind.as_u16()
        ));
    }
    let r = ResonanceBuzzResponse::decode(&resp)
        .map_err(|e| format!("ResonanceBuzzResponse decode: {e:?}"))?;
    Ok(r.result)
}

#[cfg(test)]
mod tests;
