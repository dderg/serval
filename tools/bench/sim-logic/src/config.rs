use crate::thermal::{HeaterParams, BED_DEFAULT, EXTRUDER_DEFAULT, HEATERS};
use crate::walls::{Wall, WallError, Walls};
use crate::CHANNELS;

pub const TAG_ORIGIN: u8 = 1;
pub const TAG_WALL: u8 = 2;
pub const TAG_HEATER: u8 = 3;
pub const TAG_EXPECT_MCO: u8 = 4;
pub const MAX_PAYLOAD: usize = 512;
const WALL_FLAG_ACTIVE_HIGH: u8 = 1 << 0;
const WALL_FLAG_AT_OR_BELOW: u8 = 1 << 1;
const HEATER_FIELDS: usize = 8;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SimConfig {
    pub id: u16,
    pub origin: [i32; CHANNELS],
    pub walls: Walls,
    pub heaters: [HeaterParams; HEATERS],
    pub expect_mco: bool,
}

impl Default for SimConfig {
    fn default() -> Self {
        Self {
            id: 0,
            origin: [0; CHANNELS],
            walls: Walls::default(),
            heaters: [EXTRUDER_DEFAULT, BED_DEFAULT],
            expect_mco: false,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConfigError {
    Truncated,
    TooLong,
    UnknownTag(u8),
    BadLength { tag: u8, len: u8 },
    Wall(WallError),
    HeaterIndex(u8),
    HeaterParams(u8),
    BadFlag(u8),
}

struct Cursor<'a> {
    bytes: &'a [u8],
    at: usize,
}

impl<'a> Cursor<'a> {
    fn take(&mut self, n: usize) -> Result<&'a [u8], ConfigError> {
        let end = self.at.checked_add(n).ok_or(ConfigError::Truncated)?;
        let slice = self.bytes.get(self.at..end).ok_or(ConfigError::Truncated)?;
        self.at = end;
        Ok(slice)
    }

    fn u8(&mut self) -> Result<u8, ConfigError> {
        Ok(self.take(1)?[0])
    }

    fn array<const N: usize>(&mut self) -> Result<[u8; N], ConfigError> {
        Ok(self.take(N)?.try_into().expect("take returned N bytes"))
    }

    fn i32(&mut self) -> Result<i32, ConfigError> {
        Ok(i32::from_le_bytes(self.array()?))
    }

    fn f32(&mut self) -> Result<f32, ConfigError> {
        Ok(f32::from_le_bytes(self.array()?))
    }

    fn done(&self) -> bool {
        self.at == self.bytes.len()
    }
}

fn expect_len(tag: u8, len: u8, want: usize) -> Result<(), ConfigError> {
    if usize::from(len) == want {
        Ok(())
    } else {
        Err(ConfigError::BadLength { tag, len })
    }
}

pub fn parse(payload: &[u8]) -> Result<SimConfig, ConfigError> {
    if payload.len() > MAX_PAYLOAD {
        return Err(ConfigError::TooLong);
    }
    let mut cursor = Cursor {
        bytes: payload,
        at: 0,
    };
    let mut config = SimConfig {
        id: u16::from_le_bytes(cursor.array()?),
        ..SimConfig::default()
    };
    while !cursor.done() {
        let tag = cursor.u8()?;
        let len = cursor.u8()?;
        let mut body = Cursor {
            bytes: cursor.take(usize::from(len))?,
            at: 0,
        };
        match tag {
            TAG_ORIGIN => {
                expect_len(tag, len, 4 * CHANNELS)?;
                for slot in &mut config.origin {
                    *slot = body.i32()?;
                }
            }
            TAG_WALL => {
                expect_len(tag, len, 2 + CHANNELS + 4)?;
                let pin = body.u8()?;
                let flags = body.u8()?;
                if flags & !(WALL_FLAG_ACTIVE_HIGH | WALL_FLAG_AT_OR_BELOW) != 0 {
                    return Err(ConfigError::BadFlag(flags));
                }
                let coeffs = body.array::<CHANNELS>()?.map(|b| b as i8);
                config
                    .walls
                    .push(Wall {
                        pin,
                        active_high: flags & WALL_FLAG_ACTIVE_HIGH != 0,
                        at_or_below: flags & WALL_FLAG_AT_OR_BELOW != 0,
                        coeffs,
                        threshold: body.i32()?,
                    })
                    .map_err(ConfigError::Wall)?;
            }
            TAG_HEATER => {
                expect_len(tag, len, 1 + 4 * HEATER_FIELDS)?;
                let index = body.u8()?;
                if usize::from(index) >= HEATERS {
                    return Err(ConfigError::HeaterIndex(index));
                }
                let params = HeaterParams {
                    power_w: body.f32()?,
                    capacity_j_per_k: body.f32()?,
                    loss_w_per_k: body.f32()?,
                    fan_loss_w_per_k: body.f32()?,
                    ambient_c: body.f32()?,
                    r25_ohm: body.f32()?,
                    beta: body.f32()?,
                    pullup_ohm: body.f32()?,
                };
                if !params.valid() {
                    return Err(ConfigError::HeaterParams(index));
                }
                config.heaters[usize::from(index)] = params;
            }
            TAG_EXPECT_MCO => {
                expect_len(tag, len, 1)?;
                config.expect_mco = match body.u8()? {
                    0 => false,
                    1 => true,
                    other => return Err(ConfigError::BadFlag(other)),
                };
            }
            other => return Err(ConfigError::UnknownTag(other)),
        }
    }
    Ok(config)
}
