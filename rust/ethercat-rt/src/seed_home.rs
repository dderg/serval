//! Off-loop CoE work for the CiA-402 method-35 ("current position is home")
//! drive-frame handshake.
//!
//! Mode-of-operation (6060h), homing method (6098h) and home offset (607Ch)
//! are not in the cyclic PDO, so they go over SDO. Blocking SDO must never run
//! on the DC thread (it would pause process data past the SYNC0 watchdog), so
//! these functions run on the mailbox companion thread. The controlword pulse
//! and statusword poll — which DO ride the PDO — are driven separately by the
//! self-contained C `ec_rt_run_homing` between the two functions here.
//!
//! Sequence: `seed_home_setup` (method=35, offset, switch to Homing, confirm
//! 6061h=6) → C `ec_rt_run_homing` on the DC loop → `seed_home_restore` (switch
//! back to CSP, confirm 6061h=8).

use crate::sdo::SdoBus;

const OD_MODE_OF_OPERATION: u16 = 0x6060;
const OD_MODE_DISPLAY: u16 = 0x6061;
const OD_HOMING_METHOD: u16 = 0x6098;
const OD_HOME_OFFSET: u16 = 0x607C;

const MODE_HOMING: i8 = 6;
const MODE_CSP: i8 = 8;
const HOMING_METHOD_CURRENT_POSITION: i8 = 35;

/// 6061h read attempts before declaring the mode switch un-acknowledged. Each
/// attempt is one SDO round trip (sub-ms to low-ms), so this bounds the poll
/// well under a second without a clock dependency.
const MODE_POLL_ATTEMPTS: u32 = 200;

pub const ERR_SEED_HOME_METHOD_WRITE: i32 = -820;
pub const ERR_SEED_HOME_OFFSET_WRITE: i32 = -821;
pub const ERR_SEED_HOME_MODE_WRITE: i32 = -822;
pub const ERR_SEED_HOME_MODE_NOT_ATTAINED: i32 = -823;
pub const ERR_SEED_HOME_BUSY: i32 = -824;
pub const ERR_SEED_HOME_NOT_ENABLED: i32 = -825;
pub const ERR_SEED_HOME_STREAMING: i32 = -826;
pub const ERR_SEED_HOME_RESTORE: i32 = -827;

fn write_i8(bus: &mut dyn SdoBus, slot: u8, index: u16, value: i8, err: i32) -> Result<(), i32> {
    bus.write(slot, index, 0x00, &[value as u8])
        .map_err(|_| err)
}

fn write_i32(bus: &mut dyn SdoBus, slot: u8, index: u16, value: i32, err: i32) -> Result<(), i32> {
    bus.write(slot, index, 0x00, &value.to_le_bytes())
        .map_err(|_| err)
}

fn poll_mode_display(bus: &mut dyn SdoBus, slot: u8, wanted: i8) -> Result<(), i32> {
    for _ in 0..MODE_POLL_ATTEMPTS {
        if let Ok((size, data)) = bus.read(slot, OD_MODE_DISPLAY, 0x00) {
            if size >= 1 && data[0] as i8 == wanted {
                return Ok(());
            }
        }
    }
    Err(ERR_SEED_HOME_MODE_NOT_ATTAINED)
}

fn seed_home_setup_inner(bus: &mut dyn SdoBus, slot: u8, offset_counts: i32) -> Result<(), i32> {
    write_i8(
        bus,
        slot,
        OD_HOMING_METHOD,
        HOMING_METHOD_CURRENT_POSITION,
        ERR_SEED_HOME_METHOD_WRITE,
    )?;
    write_i32(
        bus,
        slot,
        OD_HOME_OFFSET,
        offset_counts,
        ERR_SEED_HOME_OFFSET_WRITE,
    )?;
    write_i8(
        bus,
        slot,
        OD_MODE_OF_OPERATION,
        MODE_HOMING,
        ERR_SEED_HOME_MODE_WRITE,
    )?;
    poll_mode_display(bus, slot, MODE_HOMING)
}

/// Stage method=35 and the home offset, switch the drive to Homing mode, and
/// confirm 6061h reads Homing. Returns 0 on success or a negative ERR_*.
pub fn seed_home_setup(bus: &mut dyn SdoBus, slot: u8, offset_counts: i32) -> i32 {
    match seed_home_setup_inner(bus, slot, offset_counts) {
        Ok(()) => 0,
        Err(code) => code,
    }
}

fn seed_home_restore_inner(bus: &mut dyn SdoBus, slot: u8) -> Result<(), i32> {
    write_i8(
        bus,
        slot,
        OD_MODE_OF_OPERATION,
        MODE_CSP,
        ERR_SEED_HOME_MODE_WRITE,
    )?;
    poll_mode_display(bus, slot, MODE_CSP)
}

/// Switch the drive back to CSP mode and confirm 6061h reads CSP. Returns 0 on
/// success or a negative ERR_*.
pub fn seed_home_restore(bus: &mut dyn SdoBus, slot: u8) -> i32 {
    match seed_home_restore_inner(bus, slot) {
        Ok(()) => 0,
        Err(code) => code,
    }
}

#[cfg(test)]
mod tests;
