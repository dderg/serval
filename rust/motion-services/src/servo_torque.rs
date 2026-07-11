use std::time::Duration;

use host_rt::mcu_serial_conn::McuSerialConn;
use mcu_protocol::codec::Encode as _;
use mcu_protocol::messages::{
    ArmSensorlessEndstop, ArmSensorlessEndstopResponse, MessageKind, ResonanceBuzz,
    ResonanceBuzzResponse, RestoreDriveLimits, RestoreDriveLimitsResponse, SeedServoHome,
    SeedServoHomeResponse, SetDiffDamper, SetDiffDamperResponse, SetDiffTrim, SetDiffTrimResponse,
    SetDriveLimits, SetDriveLimitsResponse, SetTorque, SetTorqueResponse, StopResponse, SyncPair,
    SyncPairResponse,
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

pub fn send_resonance_buzz(conn: &McuSerialConn, buzz: ResonanceBuzz) -> Result<i32, String> {
    let body = buzz.encoded_to_vec();
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

const SET_DIFF_DAMPER_TIMEOUT: Duration = Duration::from_secs(5);

pub fn send_set_diff_damper(conn: &McuSerialConn, damper: SetDiffDamper) -> Result<i32, String> {
    let body = damper.encoded_to_vec();
    let r: SetDiffDamperResponse = mcu_typed_call(
        conn,
        "SetDiffDamper",
        MessageKind::SetDiffDamper,
        MessageKind::SetDiffDamperResponse,
        body,
        SET_DIFF_DAMPER_TIMEOUT,
    )?;
    Ok(r.result)
}

const SET_DIFF_TRIM_TIMEOUT: Duration = Duration::from_secs(5);

pub fn send_set_diff_trim(conn: &McuSerialConn, trim: SetDiffTrim) -> Result<i32, String> {
    let body = trim.encoded_to_vec();
    let r: SetDiffTrimResponse = mcu_typed_call(
        conn,
        "SetDiffTrim",
        MessageKind::SetDiffTrim,
        MessageKind::SetDiffTrimResponse,
        body,
        SET_DIFF_TRIM_TIMEOUT,
    )?;
    Ok(r.result)
}

/// A pair sync runs baseline/coast/dither/final phases with per-phase settle
/// windows; a couple of seconds is normal, so give the round-trip real room.
const SYNC_PAIR_TIMEOUT: Duration = Duration::from_secs(30);

pub fn send_sync_pair(conn: &McuSerialConn, msg: SyncPair) -> Result<SyncPairResponse, String> {
    let body = msg.encoded_to_vec();
    mcu_typed_call(
        conn,
        "SyncPair",
        MessageKind::SyncPair,
        MessageKind::SyncPairResponse,
        body,
        SYNC_PAIR_TIMEOUT,
    )
}

#[cfg(test)]
mod tests;
