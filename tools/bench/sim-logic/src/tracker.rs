use crate::config::SimConfig;
use crate::extend::Extend16;
use crate::walls::Walls;
use crate::CHANNELS;

pub const MCO_COUNTS_PER_MS: u64 = 6_250;
pub const MCO_TOLERANCE: u64 = 50;

#[derive(Clone, Copy, Debug)]
pub struct Tracker {
    positions: [i32; CHANNELS],
    walls: Walls,
    expect_mco: bool,
    mco: Extend16,
    ticks_per_aux: u32,
    ticks: u32,
    mco_at_aux: u64,
    levels: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AuxStamp {
    pub mco: u64,
    pub mco_ok: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TickOutput {
    pub levels: u8,
    pub aux: Option<AuxStamp>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct McoLost {
    pub delta: u64,
}

impl Tracker {
    pub fn start(config: &SimConfig, ticks_per_aux: u32, mco_raw: u16) -> Self {
        Self {
            positions: config.origin,
            walls: config.walls,
            expect_mco: config.expect_mco,
            mco: Extend16::new(mco_raw),
            ticks_per_aux,
            ticks: 0,
            mco_at_aux: 0,
            levels: config.walls.levels(&config.origin),
        }
    }

    pub fn positions(&self) -> [i32; CHANNELS] {
        self.positions
    }

    pub fn levels(&self) -> u8 {
        self.levels
    }

    pub fn tick(&mut self, deltas: &[i32; CHANNELS], mco_raw: u16) -> Result<TickOutput, McoLost> {
        for (position, delta) in self.positions.iter_mut().zip(deltas) {
            *position += delta;
        }
        self.levels = self.walls.levels(&self.positions);
        let mco = self.mco.update(mco_raw);
        self.ticks += 1;
        if self.ticks < self.ticks_per_aux {
            return Ok(TickOutput {
                levels: self.levels,
                aux: None,
            });
        }
        self.ticks = 0;
        let delta = mco - self.mco_at_aux;
        self.mco_at_aux = mco;
        let mco_ok = delta.abs_diff(MCO_COUNTS_PER_MS) <= MCO_TOLERANCE;
        if self.expect_mco && !mco_ok {
            return Err(McoLost { delta });
        }
        Ok(TickOutput {
            levels: self.levels,
            aux: Some(AuxStamp { mco, mco_ok }),
        })
    }
}
