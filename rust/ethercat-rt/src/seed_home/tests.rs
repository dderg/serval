use super::*;

/// Fake A6-EC for the SDO half of the handshake: accepts the method/offset/mode
/// writes and mirrors 6060h into 6061h (mode display) the way the drive does
/// once it adopts the requested mode. `stuck_mode` models a drive that never
/// adopts the requested mode (display frozen), driving the not-attained path.
struct FakeDrive {
    method: i8,
    offset: i32,
    mode: i8,
    display: i8,
    stuck: bool,
}

impl FakeDrive {
    fn new(stuck: bool) -> Self {
        Self {
            method: 0,
            offset: 0,
            mode: MODE_CSP,
            display: MODE_CSP,
            stuck,
        }
    }

    fn stuck_in_homing() -> Self {
        Self {
            method: HOMING_METHOD_CURRENT_POSITION,
            offset: 0,
            mode: MODE_HOMING,
            display: MODE_HOMING,
            stuck: true,
        }
    }
}

impl SdoBus for FakeDrive {
    fn read(&mut self, _slot: u8, index: u16, _sub: u8) -> Result<(u8, [u8; 4]), i32> {
        match index {
            OD_MODE_DISPLAY => Ok((1, [self.display as u8, 0, 0, 0])),
            OD_MODE_OF_OPERATION => Ok((1, [self.mode as u8, 0, 0, 0])),
            OD_HOMING_METHOD => Ok((1, [self.method as u8, 0, 0, 0])),
            OD_HOME_OFFSET => Ok((4, self.offset.to_le_bytes())),
            _ => Err(0x0602_0000),
        }
    }

    fn write(&mut self, _slot: u8, index: u16, _sub: u8, bytes: &[u8]) -> Result<(), i32> {
        match index {
            OD_HOMING_METHOD => self.method = bytes[0] as i8,
            OD_HOME_OFFSET => {
                self.offset = i32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
            }
            OD_MODE_OF_OPERATION => {
                self.mode = bytes[0] as i8;
                if !self.stuck {
                    self.display = self.mode;
                }
            }
            _ => return Err(0x0602_0000),
        }
        Ok(())
    }
}

#[test]
fn setup_writes_method_offset_mode_and_confirms_display() {
    let mut bus = FakeDrive::new(false);
    let rc = seed_home_setup(&mut bus, 0, -98_304);
    assert_eq!(rc, 0);
    assert_eq!(bus.method, HOMING_METHOD_CURRENT_POSITION);
    assert_eq!(bus.offset, -98_304);
    assert_eq!(bus.mode, MODE_HOMING);
}

#[test]
fn setup_fails_when_mode_display_never_reaches_homing() {
    let mut bus = FakeDrive::new(true);
    let rc = seed_home_setup(&mut bus, 0, 0);
    assert_eq!(rc, ERR_SEED_HOME_MODE_NOT_ATTAINED);
}

#[test]
fn restore_switches_back_to_csp() {
    let mut bus = FakeDrive::new(false);
    assert_eq!(seed_home_setup(&mut bus, 0, 0), 0);
    assert_eq!(bus.mode, MODE_HOMING);
    let rc = seed_home_restore(&mut bus, 0);
    assert_eq!(rc, 0);
    assert_eq!(bus.mode, MODE_CSP);
}

#[test]
fn restore_fails_when_mode_display_stuck() {
    let mut bus = FakeDrive::stuck_in_homing();
    let rc = seed_home_restore(&mut bus, 0);
    assert_eq!(rc, ERR_SEED_HOME_MODE_NOT_ATTAINED);
}
