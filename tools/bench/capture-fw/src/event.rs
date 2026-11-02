use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};

use cortex_m::peripheral::{DWT, NVIC};
use stm32f4xx_hal::pac::Interrupt;

use crate::capture::{Fault, CHANNEL_COUNT};
use crate::regs::{self, DmaStream, InputPin, Timer};
use crate::ring::{ByteRing, DirQueue, DIR_QUEUE_LEN};
use crate::sim;
use bench_sim_logic::dir_merge::DirMerge;
use bench_sim_logic::tracker::Step;

pub const SNAPSHOT_HZ: u32 = 10_000;
const TIMER_HZ: u32 = 84_000_000;
const CORE_HZ: u32 = 168_000_000;
const CORE_CYCLES_PER_TIMER_TICK: u64 = (CORE_HZ / TIMER_HZ) as u64;
const TICK_LATE_CYCLES: u32 = CORE_HZ / 1_000_000 * 600;
const TIMEBASE_SLACK_CYCLES: i64 = 128;
const LANE_SKEW_SLACK_TICKS: i32 = 32;
const DMA_RING_LEN: usize = 1024;
const DMA_HALF: usize = DMA_RING_LEN / 2;
const DMA_FLAG_RACE_ENTRIES: usize = 4;
const ANCHOR_EVERY_BLOCKS: u32 = 1000;
const ANCHOR_FLAG: u8 = 0x80;
const MAX_VARINT_LEN: usize = 10;
const ANCHOR_LEN: usize = 8 + 2 + CHANNEL_COUNT;

const CCMR1_CC1S_TI1: u32 = 0b01;
const CCMR1_CC2S_TI2: u32 = 0b01 << 8;
const IC_FILTER_CK_INT_N4: u32 = 0b0010;
const IC_FILTER_CK_INT_N8: u32 = 0b0011;
const CCER_CC1E: u32 = 1 << 0;
const CCER_CC1_BOTH_EDGES: u32 = (1 << 1) | (1 << 3);
const CCER_CC2E: u32 = 1 << 4;
const CCER_CC2_BOTH_EDGES: u32 = (1 << 5) | (1 << 7);
const DIER_CC2IE: u32 = 1 << 2;
const DIER_CC1DE: u32 = 1 << 9;
const SMCR_SMS_TRIGGER: u32 = 0b110;
const SMCR_TS_ITR0: u32 = 0b000 << 4;
const CR2_MMS_ENABLE: u32 = 0b001 << 4;
const COUNTER_TOP: u32 = 0xFFFF;

struct EventChannel {
    timer: Timer,
    prescaler: u32,
    input_filter: u32,
    dma: DmaStream,
    dma_channel: u32,
    dir: InputPin,
    dir_irq: Interrupt,
}

const CHANNELS: [EventChannel; CHANNEL_COUNT] = [
    EventChannel {
        timer: regs::TIM1,
        prescaler: 1,
        input_filter: IC_FILTER_CK_INT_N8,
        dma: DmaStream {
            controller: regs::DMA2,
            index: 1,
        },
        dma_channel: 6,
        dir: InputPin {
            port: regs::GPIOE,
            pin: 11,
        },
        dir_irq: Interrupt::TIM1_CC,
    },
    EventChannel {
        timer: regs::TIM3,
        prescaler: 0,
        input_filter: IC_FILTER_CK_INT_N4,
        dma: DmaStream {
            controller: regs::DMA1,
            index: 4,
        },
        dma_channel: 5,
        dir: InputPin {
            port: regs::GPIOA,
            pin: 7,
        },
        dir_irq: Interrupt::TIM3,
    },
    EventChannel {
        timer: regs::TIM4,
        prescaler: 0,
        input_filter: IC_FILTER_CK_INT_N4,
        dma: DmaStream {
            controller: regs::DMA1,
            index: 0,
        },
        dma_channel: 2,
        dir: InputPin {
            port: regs::GPIOD,
            pin: 13,
        },
        dir_irq: Interrupt::TIM4,
    },
    EventChannel {
        timer: regs::TIM8,
        prescaler: 1,
        input_filter: IC_FILTER_CK_INT_N8,
        dma: DmaStream {
            controller: regs::DMA2,
            index: 2,
        },
        dma_channel: 7,
        dir: InputPin {
            port: regs::GPIOC,
            pin: 7,
        },
        dir_irq: Interrupt::TIM8_CC,
    },
];

const MASTER: usize = 0;

struct DmaRings(UnsafeCell<[[u16; DMA_RING_LEN]; CHANNEL_COUNT]>);

unsafe impl Sync for DmaRings {}

static DMA_RINGS: DmaRings = DmaRings(UnsafeCell::new([[0; DMA_RING_LEN]; CHANNEL_COUNT]));

pub static OUTPUT: ByteRing = ByteRing::new();
static DIR_QUEUES: [DirQueue; CHANNEL_COUNT] = [const { DirQueue::new() }; CHANNEL_COUNT];
static DIR_SERVICED: [AtomicU32; CHANNEL_COUNT] = [const { AtomicU32::new(0) }; CHANNEL_COUNT];
static DIR_LEVELS: [AtomicU8; CHANNEL_COUNT] = [const { AtomicU8::new(0) }; CHANNEL_COUNT];

struct EngineCell(UnsafeCell<Option<Engine>>);

unsafe impl Sync for EngineCell {}

static ENGINE: EngineCell = EngineCell(UnsafeCell::new(None));

#[derive(Clone, Copy)]
struct Lane {
    dma_position: usize,
    counter: u16,
    now: u64,
    skew: i32,
    skew_wander: u8,
    last_step: u64,
    last_interval: u64,
    last_dir: u64,
    step_level: bool,
    step_pending: bool,
    dir_pending: bool,
    dir_serviced: u32,
}

pub const WALL_STEPS_PER_SNAPSHOT: usize = 32;

struct StepLog {
    steps: [[Step; WALL_STEPS_PER_SNAPSHOT]; CHANNEL_COUNT],
    counts: [usize; CHANNEL_COUNT],
}

impl StepLog {
    fn new() -> Self {
        Self {
            steps: [[Step::default(); WALL_STEPS_PER_SNAPSHOT]; CHANNEL_COUNT],
            counts: [0; CHANNEL_COUNT],
        }
    }

    fn push(&mut self, channel: usize, step: Step) -> Result<(), Fault> {
        let slot = self.steps[channel]
            .get_mut(self.counts[channel])
            .ok_or(Fault::WallStepsOverflow)?;
        *slot = step;
        self.counts[channel] += 1;
        Ok(())
    }

    fn lists(&self) -> [&[Step]; CHANNEL_COUNT] {
        core::array::from_fn(|i| &self.steps[i][..self.counts[i]])
    }
}

struct Engine {
    lanes: [Lane; CHANNEL_COUNT],
    cycles: u32,
    block: u32,
    drift_wander: u16,
    steps: StepLog,
}

struct Snapshot {
    positions: [usize; CHANNEL_COUNT],
    flags: [u32; CHANNEL_COUNT],
    dir_heads: [usize; CHANNEL_COUNT],
    dir_serviced: [u32; CHANNEL_COUNT],
    status: [u32; CHANNEL_COUNT],
    cycles: u32,
    counters: [u16; CHANNEL_COUNT],
    mco: u16,
}

pub struct SnapshotOutput<'a> {
    pub steps: [&'a [Step]; CHANNEL_COUNT],
    pub mco: u16,
    pub time: u64,
}

type LaneFault = (Fault, u8);

fn read_clocks() -> (u32, [u16; CHANNEL_COUNT], u16) {
    cortex_m::interrupt::free(|_| {
        let cycles = DWT::cycle_count();
        let counters = core::array::from_fn(|i| CHANNELS[i].timer.cnt());
        (cycles, counters, sim::mco_count())
    })
}

fn dma_position(channel: &EventChannel) -> usize {
    (DMA_RING_LEN - channel.dma.ndtr() as usize) % DMA_RING_LEN
}

fn take_snapshot() -> Snapshot {
    let positions = core::array::from_fn(|i| dma_position(&CHANNELS[i]));
    let flags = core::array::from_fn(|i| CHANNELS[i].dma.flags());
    let dir_heads = core::array::from_fn(|i| DIR_QUEUES[i].produced());
    let dir_serviced = core::array::from_fn(|i| DIR_SERVICED[i].load(Ordering::Acquire));
    let status = core::array::from_fn(|i| CHANNELS[i].timer.sr());
    let (cycles, counters, mco) = read_clocks();
    Snapshot {
        positions,
        flags,
        dir_heads,
        dir_serviced,
        status,
        cycles,
        counters,
        mco,
    }
}

pub fn enable_clocks() {
    regs::enable_ahb1(regs::AHB1_DMA1 | regs::AHB1_DMA2);
}

fn arm_timer(channel: &EventChannel, both_edges: bool) {
    let timer = channel.timer;
    timer.set_cr1(0);
    timer.set_cr2(0);
    timer.set_smcr(0);
    timer.set_dier(0);
    timer.set_ccer(0);
    timer.set_ccmr1(
        CCMR1_CC1S_TI1 | channel.input_filter << 4 | CCMR1_CC2S_TI2 | channel.input_filter << 12,
    );
    let step_edges = if both_edges { CCER_CC1_BOTH_EDGES } else { 0 };
    timer.set_ccer(CCER_CC1E | step_edges | CCER_CC2E | CCER_CC2_BOTH_EDGES);
    timer.set_psc(channel.prescaler);
    timer.set_arr(COUNTER_TOP);
    timer.set_egr(regs::EGR_UG);
    timer.set_cnt(0);
    timer.set_sr(0);
}

fn arm_dma(lane: usize) {
    let channel = &CHANNELS[lane];
    let stream = channel.dma;
    stream.set_cr(0);
    while stream.enabled() {}
    stream.clear_flags(regs::DMA_FLAGS_ALL);
    stream.set_par(channel.timer.ccr1_address());
    let ring = unsafe { (*DMA_RINGS.0.get())[lane].as_ptr() };
    stream.set_m0ar(ring as u32);
    stream.set_ndtr(DMA_RING_LEN as u32);
    stream.set_fcr(0);
    stream.set_cr(
        channel.dma_channel << regs::DMA_CR_CHSEL_SHIFT
            | regs::DMA_CR_PL_VERY_HIGH
            | regs::DMA_CR_MSIZE_HALF
            | regs::DMA_CR_PSIZE_HALF
            | regs::DMA_CR_MINC
            | regs::DMA_CR_CIRC
            | regs::DMA_CR_EN,
    );
}

pub fn start(both_edges_mask: u8) {
    halt();
    OUTPUT.reset();
    for lane in 0..CHANNEL_COUNT {
        DIR_QUEUES[lane].reset();
        DIR_SERVICED[lane].store(0, Ordering::Release);
        arm_timer(&CHANNELS[lane], both_edges_mask & (1 << lane) != 0);
        arm_dma(lane);
    }
    let mut initial_dir = 0u8;
    for (lane, channel) in CHANNELS.iter().enumerate() {
        let high = channel.dir.is_high();
        DIR_LEVELS[lane].store(high as u8, Ordering::Release);
        initial_dir |= (high as u8) << lane;
        channel.timer.set_dier(DIER_CC1DE | DIER_CC2IE);
        if lane != MASTER {
            channel.timer.set_smcr(SMCR_SMS_TRIGGER | SMCR_TS_ITR0);
        }
    }
    CHANNELS[MASTER].timer.set_cr2(CR2_MMS_ENABLE);
    let mut writer = OUTPUT.writer();
    writer.put(initial_dir);
    writer.put(both_edges_mask);
    writer.commit();
    for channel in &CHANNELS {
        unsafe { NVIC::unmask(channel.dir_irq) };
    }
    let (cycles, counters, _) = cortex_m::interrupt::free(|_| {
        CHANNELS[MASTER].timer.set_cr1(regs::CR1_CEN);
        read_clocks()
    });
    let lanes = core::array::from_fn(|i| Lane {
        dma_position: 0,
        counter: counters[i],
        now: counters[i] as u64,
        skew: counters[i].wrapping_sub(counters[MASTER]) as i16 as i32,
        skew_wander: 0,
        last_step: 0,
        last_interval: 0,
        last_dir: 0,
        step_level: initial_dir & (1 << i) != 0,
        step_pending: false,
        dir_pending: false,
        dir_serviced: 0,
    });
    unsafe {
        *ENGINE.0.get() = Some(Engine {
            lanes,
            cycles,
            block: 0,
            drift_wander: 0,
            steps: StepLog::new(),
        })
    };
}

pub fn halt_interrupts() {
    for channel in &CHANNELS {
        NVIC::mask(channel.dir_irq);
    }
}

pub fn halt() {
    halt_interrupts();
    for channel in &CHANNELS {
        channel.timer.set_dier(0);
        channel.timer.set_cr1(0);
        channel.timer.set_smcr(0);
        channel.timer.set_cr2(0);
        channel.dma.set_cr(0);
        while channel.dma.enabled() {}
        NVIC::unpend(channel.dir_irq);
    }
    unsafe { *ENGINE.0.get() = None };
}

pub fn dir_edge(lane: usize) -> Result<(), Fault> {
    let channel = &CHANNELS[lane];
    let status = channel.timer.sr();
    if status & regs::SR_CC2OF != 0 {
        return Err(Fault::DirOvercapture);
    }
    if status & regs::SR_CC2IF == 0 {
        return Ok(());
    }
    let captured = channel.timer.ccr2();
    let counter = channel.timer.cnt();
    let level = channel.dir.is_high();
    if level == (DIR_LEVELS[lane].load(Ordering::Relaxed) != 0) {
        return Err(Fault::DirDesync);
    }
    DIR_LEVELS[lane].store(level as u8, Ordering::Relaxed);
    DIR_QUEUES[lane]
        .push(captured as u64 | (counter as u64) << 16 | (level as u64) << 32)
        .map_err(|_| Fault::DirQueueFull)?;
    DIR_SERVICED[lane].fetch_add(1, Ordering::Release);
    Ok(())
}

fn crosses(start: usize, advance: usize, boundary: usize) -> bool {
    let distance = (boundary + DMA_RING_LEN - start) % DMA_RING_LEN;
    let distance = if distance == 0 {
        DMA_RING_LEN
    } else {
        distance
    };
    distance <= advance
}

fn verify_boundary(
    channel: &EventChannel,
    flags: u32,
    flag: u32,
    boundary: usize,
    start: usize,
    advance: usize,
    position: usize,
) -> Result<(), Fault> {
    let crossed = crosses(start, advance, boundary);
    let raised = flags & flag != 0;
    match (crossed, raised) {
        (true, true) => {
            channel.dma.clear_flags(flag);
            Ok(())
        }
        (true, false) => Err(Fault::DmaLap),
        (false, true) => {
            let ahead = (boundary + DMA_RING_LEN - position) % DMA_RING_LEN;
            if ahead > 0 && ahead <= DMA_FLAG_RACE_ENTRIES {
                Ok(())
            } else {
                Err(Fault::DmaLap)
            }
        }
        (false, false) => Ok(()),
    }
}

fn check_lane(
    lane: &mut Lane,
    channel: &EventChannel,
    snapshot: &Snapshot,
    index: usize,
) -> Result<usize, Fault> {
    let status = snapshot.status[index];
    let flags = snapshot.flags[index];
    if status & regs::SR_CC1OF != 0 {
        return Err(Fault::StepOvercapture);
    }
    if status & regs::SR_CC2OF != 0 {
        return Err(Fault::DirOvercapture);
    }
    if flags & (regs::DMA_FLAG_TE | regs::DMA_FLAG_DME) != 0 {
        return Err(Fault::DmaError);
    }
    let position = snapshot.positions[index];
    let advance = (position + DMA_RING_LEN - lane.dma_position) % DMA_RING_LEN;
    if advance >= DMA_HALF {
        return Err(Fault::DmaLap);
    }
    let start = lane.dma_position;
    verify_boundary(
        channel,
        flags,
        regs::DMA_FLAG_HT,
        DMA_HALF,
        start,
        advance,
        position,
    )?;
    verify_boundary(
        channel,
        flags,
        regs::DMA_FLAG_TC,
        0,
        start,
        advance,
        position,
    )?;
    let step_pending = status & regs::SR_CC1IF != 0;
    if step_pending && lane.step_pending && advance == 0 {
        return Err(Fault::DmaStall);
    }
    lane.step_pending = step_pending;
    let dir_pending = status & regs::SR_CC2IF != 0;
    let serviced = snapshot.dir_serviced[index];
    if dir_pending && lane.dir_pending && serviced == lane.dir_serviced {
        return Err(Fault::DirIsrLate);
    }
    lane.dir_pending = dir_pending;
    lane.dir_serviced = serviced;
    Ok(advance)
}

fn advance_clocks(engine: &mut Engine, snapshot: &Snapshot) -> Result<(), LaneFault> {
    let elapsed = snapshot.cycles.wrapping_sub(engine.cycles);
    if elapsed > TICK_LATE_CYCLES {
        return Err((Fault::TickLate, MASTER as u8));
    }
    engine.cycles = snapshot.cycles;
    let master_ticks = snapshot.counters[MASTER].wrapping_sub(engine.lanes[MASTER].counter) as u64;
    let drift = (master_ticks * CORE_CYCLES_PER_TIMER_TICK) as i64 - elapsed as i64;
    if drift.abs() > TIMEBASE_SLACK_CYCLES {
        return Err((Fault::TimebaseDesync, MASTER as u8));
    }
    engine.drift_wander = engine.drift_wander.max(drift.unsigned_abs() as u16);
    for (index, lane) in engine.lanes.iter_mut().enumerate() {
        let counter = snapshot.counters[index];
        let skew = counter.wrapping_sub(snapshot.counters[MASTER]) as i16 as i32;
        let wander = (skew - lane.skew).unsigned_abs();
        if wander > LANE_SKEW_SLACK_TICKS as u32 {
            return Err((Fault::TimebaseDesync, index as u8));
        }
        lane.skew_wander = lane.skew_wander.max(wander as u8);
        lane.now += counter.wrapping_sub(lane.counter) as u64;
        lane.counter = counter;
    }
    Ok(())
}

fn extend(now: u64, counter: u16, captured: u16) -> Result<u64, Fault> {
    now.checked_sub(counter.wrapping_sub(captured) as u64)
        .ok_or(Fault::NonMonotonic)
}

pub fn tick() -> Option<Result<SnapshotOutput<'static>, LaneFault>> {
    let engine = unsafe { (*ENGINE.0.get()).as_mut()? };
    Some(process(engine))
}

fn process(engine: &mut Engine) -> Result<SnapshotOutput<'_>, LaneFault> {
    let snapshot = take_snapshot();
    advance_clocks(engine, &snapshot)?;
    let mut advances = [0usize; CHANNEL_COUNT];
    let mut dir_counts = [0usize; CHANNEL_COUNT];
    let mut worst = 1 + ANCHOR_LEN;
    for index in 0..CHANNEL_COUNT {
        let lane = &mut engine.lanes[index];
        advances[index] =
            check_lane(lane, &CHANNELS[index], &snapshot, index).map_err(|f| (f, index as u8))?;
        dir_counts[index] = snapshot.dir_heads[index] - DIR_QUEUES[index].consumed();
        worst += 2 * MAX_VARINT_LEN
            + advances[index] * MAX_VARINT_LEN
            + dir_counts[index] * (MAX_VARINT_LEN + 1);
    }
    if OUTPUT.free() < worst {
        return Err((Fault::OutputOverflow, 0));
    }
    let anchor = engine.block.is_multiple_of(ANCHOR_EVERY_BLOCKS);
    let mut mask = if anchor { ANCHOR_FLAG } else { 0 };
    for index in 0..CHANNEL_COUNT {
        if advances[index] > 0 || dir_counts[index] > 0 {
            mask |= 1 << index;
        }
    }
    let mut writer = OUTPUT.writer();
    writer.put(mask);
    if anchor {
        writer.put_slice(&engine.lanes[MASTER].now.to_le_bytes());
        writer.put_slice(&engine.drift_wander.to_le_bytes());
        engine.drift_wander = 0;
        for lane in engine.lanes.iter_mut() {
            writer.put(lane.skew_wander);
            lane.skew_wander = 0;
        }
    }
    let rings = unsafe { &*DMA_RINGS.0.get() };
    engine.steps.counts = [0; CHANNEL_COUNT];
    for index in 0..CHANNEL_COUNT {
        if mask & (1 << index) == 0 {
            continue;
        }
        let lane = &mut engine.lanes[index];
        let counter = snapshot.counters[index];
        let fault = |f: Fault| (f, index as u8);
        let queue = &DIR_QUEUES[index];
        let mut edges = [(0u64, false); DIR_QUEUE_LEN];
        for (edge, slot) in edges
            .iter_mut()
            .zip(queue.consumed()..snapshot.dir_heads[index])
        {
            let entry = queue.entry(slot);
            let captured = entry as u16;
            let isr_counter = (entry >> 16) as u16;
            let isr_time = extend(lane.now, counter, isr_counter).map_err(fault)?;
            let time = extend(isr_time, isr_counter, captured).map_err(fault)?;
            *edge = (time, (entry >> 32) & 1 != 0);
        }
        let edges = &edges[..dir_counts[index]];
        let mut signs = DirMerge::new(edges, lane.step_level);
        writer.put_varint(advances[index] as u64);
        writer.put_varint(dir_counts[index] as u64);
        for k in 0..advances[index] {
            let captured = rings[index][(lane.dma_position + k) % DMA_RING_LEN];
            let time = extend(lane.now, counter, captured).map_err(fault)?;
            if time <= lane.last_step {
                return Err(fault(Fault::NonMonotonic));
            }
            let interval = time - lane.last_step;
            writer.put_zigzag(interval as i64 - lane.last_interval as i64);
            lane.last_step = time;
            lane.last_interval = interval;
            let sign = signs
                .sign_at(time)
                .map_err(|_| fault(Fault::DirAmbiguous))?;
            engine
                .steps
                .push(index, Step { time, sign })
                .map_err(fault)?;
        }
        lane.step_level = signs.finish();
        lane.dma_position = snapshot.positions[index];
        for &(time, level) in edges {
            if time <= lane.last_dir {
                return Err(fault(Fault::NonMonotonic));
            }
            writer.put_varint(time - lane.last_dir);
            writer.put(level as u8);
            lane.last_dir = time;
        }
        queue.release_through(snapshot.dir_heads[index]);
    }
    writer.commit();
    engine.block = engine.block.wrapping_add(1);
    Ok(SnapshotOutput {
        steps: engine.steps.lists(),
        mco: snapshot.mco,
        time: engine.lanes[MASTER].now,
    })
}
