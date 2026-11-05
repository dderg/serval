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

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Step {
    pub time: u64,
    pub sign: i32,
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

    pub fn tick(
        &mut self,
        deltas: &[i32; CHANNELS],
        excursions: &[i32; CHANNELS],
        mco_raw: u16,
    ) -> Result<TickOutput, McoLost> {
        let start = self.positions;
        let mut during = 0u8;
        for channel in 0..CHANNELS {
            if excursions[channel] != deltas[channel] {
                let mut turning_point = start;
                turning_point[channel] += excursions[channel];
                during |= self
                    .walls
                    .triggered_by_channel_alone(&turning_point, channel);
            }
            self.positions[channel] += deltas[channel];
        }
        self.finish_tick(during, mco_raw)
    }

    pub fn tick_steps(
        &mut self,
        steps: [&[Step]; CHANNELS],
        mco_raw: u16,
    ) -> Result<TickOutput, McoLost> {
        let mut next = [0usize; CHANNELS];
        let mut during = 0u8;
        loop {
            let earliest = (0..CHANNELS)
                .filter_map(|channel| {
                    steps[channel]
                        .get(next[channel])
                        .map(|step| (step.time, channel))
                })
                .min();
            let Some((_, channel)) = earliest else {
                break;
            };
            self.positions[channel] += steps[channel][next[channel]].sign;
            next[channel] += 1;
            during |= self.walls.triggered_involving(&self.positions, channel);
        }
        self.finish_tick(during, mco_raw)
    }

    fn finish_tick(&mut self, triggered_during: u8, mco_raw: u16) -> Result<TickOutput, McoLost> {
        self.levels = self
            .walls
            .levels_for(self.walls.triggered(&self.positions) | triggered_during);
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
