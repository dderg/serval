use crate::regs::{self, InputPin, Timer};

pub const CHANNEL_COUNT: usize = 4;
pub const TICK_HZ: u32 = 40_000;
const SNAPSHOT_TIMER_CLOCK_HZ: u32 = 84_000_000;
const COUNTER_TOP: u32 = 0xFFFF;

const CCMR1_CC1S_TI1: u32 = 0b01;
const CCMR1_IC1F_N4: u32 = 0b0010 << 4;
const CCMR1_CC2S_TI2: u32 = 0b01 << 8;
const CCMR1_IC2F_N4: u32 = 0b0010 << 12;
const CCER_CC2E: u32 = 1 << 4;
const CCER_CC2_BOTH_EDGES: u32 = (1 << 5) | (1 << 7);
const SMCR_SMS_EXTERNAL_CLOCK: u32 = 0b111;
const SMCR_TS_TI1F_ED: u32 = 0b100 << 4;
const SMCR_TS_TI1FP1: u32 = 0b101 << 4;

pub struct Channel {
    timer: Timer,
    dir: InputPin,
}

pub const CHANNELS: [Channel; CHANNEL_COUNT] = [
    Channel {
        timer: regs::TIM1,
        dir: InputPin {
            port: regs::GPIOE,
            pin: 11,
        },
    },
    Channel {
        timer: regs::TIM3,
        dir: InputPin {
            port: regs::GPIOA,
            pin: 7,
        },
    },
    Channel {
        timer: regs::TIM4,
        dir: InputPin {
            port: regs::GPIOD,
            pin: 13,
        },
    },
    Channel {
        timer: regs::TIM8,
        dir: InputPin {
            port: regs::GPIOC,
            pin: 7,
        },
    },
];

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum Fault {
    DirOvercapture = 1,
    DeltaOverflow = 2,
    DirDesync = 3,
    RingOverflow = 4,
    StepOvercapture = 5,
    TickLate = 6,
    DmaStall = 7,
    DirIsrLate = 8,
    DmaLap = 9,
    TimebaseDesync = 10,
    OutputOverflow = 11,
    DmaError = 12,
    DirQueueFull = 13,
    NonMonotonic = 14,
    McoLost = 15,
    AuxOverflow = 16,
    DirAmbiguous = 17,
    WallStepsOverflow = 18,
}

#[derive(Clone, Copy)]
pub enum EdgeMode {
    Rising,
    Both,
}

struct Lane {
    previous_count: u16,
    dir_high: bool,
    desync_pending: bool,
}

pub struct Sampler {
    lanes: [Lane; CHANNEL_COUNT],
}

pub fn enable_clocks() {
    regs::enable_apb1(regs::APB1_TIM3 | regs::APB1_TIM4 | regs::APB1_TIM6);
    regs::enable_apb2(regs::APB2_TIM1 | regs::APB2_TIM8);
}

fn arm_counter(timer: Timer, mode: EdgeMode) {
    let trigger = match mode {
        EdgeMode::Rising => SMCR_TS_TI1FP1,
        EdgeMode::Both => SMCR_TS_TI1F_ED,
    };
    timer.set_cr1(0);
    timer.set_smcr(0);
    timer.set_ccer(0);
    timer.set_ccmr1(CCMR1_CC1S_TI1 | CCMR1_IC1F_N4 | CCMR1_CC2S_TI2 | CCMR1_IC2F_N4);
    timer.set_ccer(CCER_CC2E | CCER_CC2_BOTH_EDGES);
    timer.set_smcr(trigger | SMCR_SMS_EXTERNAL_CLOCK);
    timer.set_psc(0);
    timer.set_arr(COUNTER_TOP);
    timer.set_egr(regs::EGR_UG);
    timer.set_cnt(0);
    timer.set_sr(0);
    timer.set_cr1(regs::CR1_CEN);
}

pub fn start_snapshot_timer(hz: u32) {
    let tim = regs::TIM6;
    tim.set_cr1(0);
    tim.set_psc(0);
    tim.set_arr(SNAPSHOT_TIMER_CLOCK_HZ / hz - 1);
    tim.set_egr(regs::EGR_UG);
    tim.set_sr(0);
    tim.set_dier(regs::DIER_UIE);
    tim.set_cr1(regs::CR1_CEN);
}

pub fn stop_snapshot_timer() {
    let tim = regs::TIM6;
    tim.set_cr1(0);
    tim.set_dier(0);
    tim.set_sr(0);
}

pub fn ack_snapshot_tick() {
    regs::TIM6.set_sr(!regs::SR_UIF);
}

impl Sampler {
    pub fn arm(both_edges_mask: u8) -> Self {
        let lanes = core::array::from_fn(|i| {
            let mode = if both_edges_mask & (1 << i) != 0 {
                EdgeMode::Both
            } else {
                EdgeMode::Rising
            };
            arm_counter(CHANNELS[i].timer, mode);
            Lane {
                previous_count: 0,
                dir_high: CHANNELS[i].dir.is_high(),
                desync_pending: false,
            }
        });
        Self { lanes }
    }

    pub fn sample(&mut self) -> Result<Sample, (Fault, u8)> {
        let mut sample = Sample {
            packed: 0,
            deltas: [0; CHANNEL_COUNT],
            excursions: [0; CHANNEL_COUNT],
        };
        for (i, (lane, channel)) in self.lanes.iter_mut().zip(CHANNELS.iter()).enumerate() {
            let movement = lane.advance(channel).map_err(|f| (f, i as u8))?;
            let byte = i8::try_from(movement.delta).map_err(|_| (Fault::DeltaOverflow, i as u8))?;
            sample.packed |= (byte as u8 as u32) << (8 * i);
            sample.deltas[i] = movement.delta;
            sample.excursions[i] = movement.excursion;
        }
        Ok(sample)
    }
}

fn signed(steps: u16, dir_high: bool) -> i32 {
    if dir_high {
        steps as i32
    } else {
        -(steps as i32)
    }
}

pub struct Sample {
    pub packed: u32,
    pub deltas: [i32; CHANNEL_COUNT],
    pub excursions: [i32; CHANNEL_COUNT],
}

struct Movement {
    delta: i32,
    excursion: i32,
}

impl Lane {
    fn advance(&mut self, channel: &Channel) -> Result<Movement, Fault> {
        let timer = channel.timer;
        let (status, count, pin_high) = loop {
            let before = timer.sr();
            let count = timer.cnt();
            let pin_high = channel.dir.is_high();
            let after = timer.sr();
            if (before ^ after) & regs::SR_CC2IF == 0 {
                break (after, count, pin_high);
            }
        };
        if status & regs::SR_CC2OF != 0 {
            return Err(Fault::DirOvercapture);
        }

        let movement = if status & regs::SR_CC2IF != 0 {
            let reversal = timer.ccr2();
            if timer.sr() & regs::SR_CC2OF != 0 {
                return Err(Fault::DirOvercapture);
            }
            let excursion = signed(reversal.wrapping_sub(self.previous_count), self.dir_high);
            let after_reversal = count.wrapping_sub(reversal);
            self.dir_high = !self.dir_high;
            Movement {
                delta: excursion + signed(after_reversal, self.dir_high),
                excursion,
            }
        } else {
            let delta = signed(count.wrapping_sub(self.previous_count), self.dir_high);
            Movement {
                delta,
                excursion: delta,
            }
        };
        self.previous_count = count;

        if pin_high == self.dir_high {
            self.desync_pending = false;
        } else if self.desync_pending {
            return Err(Fault::DirDesync);
        } else {
            self.desync_pending = true;
        }
        Ok(movement)
    }
}
