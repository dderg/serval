use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicI16, AtomicU16, AtomicU32, AtomicU8, Ordering};

use bench_sim_logic::aux::{AuxRecord, FLAG_DAC_CLAMPED, FLAG_MCO_OK, INPUTS};
use bench_sim_logic::config::SimConfig;
use bench_sim_logic::thermal::{Heater, HEATERS};
use bench_sim_logic::tmc::{reply_wait_ticks, Reply, TmcBus, REPLY_LEN};
use bench_sim_logic::tracker::{McoLost, TickOutput, Tracker};
use cortex_m::peripheral::NVIC;
use stm32f4xx_hal::pac::Interrupt;

use crate::capture::{Fault, Sample};
use crate::event::SnapshotOutput;
use crate::regs::{self, InputPin, Usart};
use crate::ring::AuxRing;

const SAMPLE_HZ: u32 = 10_000;
const APB1_TIMER_HZ: u32 = 84_000_000;
const APB1_HZ: u32 = 42_000_000;
const TMC_BAUD: u32 = 40_000;
const SAMPLES_PER_MODEL_STEP: u32 = 10;
const MODEL_DT_S: f32 = SAMPLES_PER_MODEL_STEP as f32 / SAMPLE_HZ as f32;
const SAMPLE_PERIOD_NS: u32 = 1_000_000_000 / SAMPLE_HZ;
const TMC_BIT_NS: u32 = 1_000_000_000 / TMC_BAUD;
const ENDSTOP_PORT: usize = regs::GPIOD;
const ENDSTOP_SHIFT: u32 = 8;
const ENDSTOP_MASK: u32 = 0xF;
const BSRR_RESET_SHIFT: u32 = 16;
const MCO_TIMER: regs::Timer = regs::TIM9;
const CCMR1_CC1S_TI1: u32 = 0b01;
const CCMR1_IC1F_CK_INT_N2: u32 = 0b0001 << 4;
const SMCR_SMS_EXTERNAL_CLOCK: u32 = 0b111;
const SMCR_TS_TI1FP1: u32 = 0b101 << 4;
const COUNTER_TOP: u32 = 0xFFFF;
const FAN_INPUT: usize = 2;

const INPUT_PINS: [InputPin; INPUTS] = [
    InputPin {
        port: regs::GPIOD,
        pin: 14,
    },
    InputPin {
        port: regs::GPIOD,
        pin: 15,
    },
    InputPin {
        port: regs::GPIOE,
        pin: 8,
    },
];

pub static AUX: AuxRing = AuxRing::new();

static SAMPLES: AtomicU32 = AtomicU32::new(0);
static HIGH: [AtomicU32; INPUTS] = [const { AtomicU32::new(0) }; INPUTS];
static TEMP_CENTI: [AtomicI16; HEATERS] = [const { AtomicI16::new(0) }; HEATERS];
static DAC_CODE: [AtomicU16; HEATERS] = [const { AtomicU16::new(0) }; HEATERS];
static DAC_FLAGS: AtomicU8 = AtomicU8::new(0);
static DEDGE: AtomicU8 = AtomicU8::new(0);
static TMC_WRITES: AtomicU8 = AtomicU8::new(0);
static TMC_READS: AtomicU8 = AtomicU8::new(0);
static TMC_REJECTS: AtomicU8 = AtomicU8::new(0);

struct Transmit {
    frame: [u8; REPLY_LEN],
    sent: usize,
}

struct Peripherals {
    heaters: [Heater; HEATERS],
    window_high: [u32; INPUTS],
    window_samples: u32,
    tmc: TmcBus,
    pending: Option<(Reply, u32)>,
    transmit: Option<Transmit>,
}

struct Capture {
    tracker: Tracker,
    aux: bool,
    config_id: u16,
}

struct Shared<T>(UnsafeCell<T>);

unsafe impl<T> Sync for Shared<T> {}

static PERIPHERALS: Shared<Option<Peripherals>> = Shared(UnsafeCell::new(None));
static CONFIG: Shared<Option<SimConfig>> = Shared(UnsafeCell::new(None));
static CAPTURE: Shared<Option<Capture>> = Shared(UnsafeCell::new(None));

fn peripherals() -> &'static mut Peripherals {
    unsafe {
        (*PERIPHERALS.0.get())
            .as_mut()
            .expect("sim::init runs before any sim interrupt is unmasked")
    }
}

fn config() -> &'static SimConfig {
    unsafe {
        (*CONFIG.0.get())
            .as_ref()
            .expect("sim::init runs before the config is read")
    }
}

pub fn enable_clocks() {
    regs::enable_apb1(regs::APB1_TIM7 | regs::APB1_USART2 | regs::APB1_DAC);
    regs::enable_apb2(regs::APB2_TIM9);
}

pub fn init() {
    let boot = SimConfig::default();
    unsafe {
        *CONFIG.0.get() = Some(boot);
        *PERIPHERALS.0.get() = Some(Peripherals {
            heaters: boot.heaters.map(Heater::at_ambient),
            window_high: [0; INPUTS],
            window_samples: 0,
            tmc: TmcBus::default(),
            pending: None,
            transmit: None,
        });
    }
    drive_endstops(boot.walls.idle_levels());
    regs::dac_set_cr(regs::DAC_CR_EN1 | regs::DAC_CR_EN2);
    publish_thermal(peripherals());
    arm_mco_counter();
    arm_tmc_uart();
    let tim = regs::TIM7;
    tim.set_cr1(0);
    tim.set_psc(0);
    tim.set_arr(APB1_TIMER_HZ / SAMPLE_HZ - 1);
    tim.set_egr(regs::EGR_UG);
    tim.set_sr(0);
    tim.set_dier(regs::DIER_UIE);
    tim.set_cr1(regs::CR1_CEN);
    unsafe {
        NVIC::unmask(Interrupt::TIM7);
        NVIC::unmask(Interrupt::USART2);
    }
}

fn arm_mco_counter() {
    let tim = MCO_TIMER;
    tim.set_cr1(0);
    tim.set_ccmr1(CCMR1_CC1S_TI1 | CCMR1_IC1F_CK_INT_N2);
    tim.set_ccer(0);
    tim.set_smcr(SMCR_TS_TI1FP1 | SMCR_SMS_EXTERNAL_CLOCK);
    tim.set_psc(0);
    tim.set_arr(COUNTER_TOP);
    tim.set_egr(regs::EGR_UG);
    tim.set_cr1(regs::CR1_CEN);
}

pub fn mco_count() -> u16 {
    MCO_TIMER.cnt()
}

fn arm_tmc_uart() {
    let uart = regs::USART2;
    uart.set_cr1(0);
    uart.set_brr(APB1_HZ / TMC_BAUD);
    uart.set_cr3(regs::USART_CR3_HDSEL);
    uart.set_cr1(receiving());
}

fn receiving() -> u32 {
    regs::USART_CR1_UE
        | regs::USART_CR1_TE
        | regs::USART_CR1_RE
        | regs::USART_CR1_RXNEIE
        | regs::USART_CR1_IDLEIE
}

fn drive_endstops(levels: u8) {
    let set = u32::from(levels) & ENDSTOP_MASK;
    let clear = !u32::from(levels) & ENDSTOP_MASK;
    regs::write_bsrr(
        ENDSTOP_PORT,
        (set << ENDSTOP_SHIFT) | (clear << (ENDSTOP_SHIFT + BSRR_RESET_SHIFT)),
    );
}

pub fn apply_config(config: SimConfig) {
    cortex_m::interrupt::free(|_| {
        let periph = peripherals();
        for (heater, params) in periph.heaters.iter_mut().zip(config.heaters) {
            heater.params = params;
        }
        unsafe { *CONFIG.0.get() = Some(config) };
    });
    drive_endstops(config.walls.idle_levels());
}

pub fn start_capture(aux: bool, ticks_per_aux: u32) {
    let config = config();
    let capture = Capture {
        tracker: Tracker::start(config, ticks_per_aux, mco_count()),
        aux,
        config_id: config.id,
    };
    drive_endstops(capture.tracker.levels());
    AUX.reset();
    cortex_m::interrupt::free(|_| unsafe { *CAPTURE.0.get() = Some(capture) });
}

pub fn stop_capture() {
    cortex_m::interrupt::free(|_| unsafe { *CAPTURE.0.get() = None });
    AUX.reset();
    drive_endstops(config().walls.idle_levels());
}

pub fn on_count_tick(sample: &Sample, mco_raw: u16, time: u64) -> Result<(), Fault> {
    with_capture(time, |tracker| {
        tracker.tick(&sample.deltas, &sample.excursions, mco_raw)
    })
}

pub fn on_event_tick(snapshot: &SnapshotOutput, time: u64) -> Result<(), Fault> {
    with_capture(time, |tracker| {
        tracker.tick_steps(snapshot.steps, snapshot.mco)
    })
}

fn with_capture(
    time: u64,
    tick: impl FnOnce(&mut Tracker) -> Result<TickOutput, McoLost>,
) -> Result<(), Fault> {
    let Some(capture) = (unsafe { (*CAPTURE.0.get()).as_mut() }) else {
        return Ok(());
    };
    let output = tick(&mut capture.tracker).map_err(|_| Fault::McoLost)?;
    drive_endstops(output.levels);
    if !capture.aux {
        return Ok(());
    }
    let TickOutput {
        levels,
        aux: Some(stamp),
    } = output
    else {
        return Ok(());
    };
    let record = AuxRecord {
        time,
        mco: stamp.mco as u32,
        samples: SAMPLES.load(Ordering::Relaxed),
        high: core::array::from_fn(|i| HIGH[i].load(Ordering::Relaxed)),
        temp_centi: core::array::from_fn(|i| TEMP_CENTI[i].load(Ordering::Relaxed)),
        dac: core::array::from_fn(|i| DAC_CODE[i].load(Ordering::Relaxed)),
        config_id: capture.config_id,
        endstops: levels,
        flags: DAC_FLAGS.load(Ordering::Relaxed) | if stamp.mco_ok { FLAG_MCO_OK } else { 0 },
        dedge: DEDGE.load(Ordering::Relaxed),
        tmc_writes: TMC_WRITES.load(Ordering::Relaxed),
        tmc_reads: TMC_READS.load(Ordering::Relaxed),
        tmc_rejects: TMC_REJECTS.load(Ordering::Relaxed),
    };
    AUX.push(record.encode()).map_err(|_| Fault::AuxOverflow)
}

fn publish_thermal(periph: &Peripherals) {
    let mut flags = 0;
    for (index, heater) in periph.heaters.iter().enumerate() {
        let dac = heater.dac();
        regs::dac_write(index, dac.code);
        DAC_CODE[index].store(dac.code, Ordering::Relaxed);
        TEMP_CENTI[index].store(heater.temp_centi(), Ordering::Relaxed);
        if dac.clamped {
            flags |= FLAG_DAC_CLAMPED[index];
        }
    }
    DAC_FLAGS.store(flags, Ordering::Relaxed);
}

fn publish_tmc(tmc: &TmcBus) {
    DEDGE.store(tmc.dedge_mask(), Ordering::Relaxed);
    TMC_WRITES.store(tmc.writes, Ordering::Relaxed);
    TMC_READS.store(tmc.reads, Ordering::Relaxed);
    TMC_REJECTS.store(tmc.rejects, Ordering::Relaxed);
}

pub fn on_sample() {
    regs::TIM7.set_sr(!regs::SR_UIF);
    let periph = peripherals();
    for (index, pin) in INPUT_PINS.iter().enumerate() {
        if pin.is_high() {
            periph.window_high[index] += 1;
            HIGH[index].fetch_add(1, Ordering::Relaxed);
        }
    }
    SAMPLES.fetch_add(1, Ordering::Relaxed);
    periph.window_samples += 1;
    if periph.window_samples == SAMPLES_PER_MODEL_STEP {
        let duty = periph
            .window_high
            .map(|high| high as f32 / SAMPLES_PER_MODEL_STEP as f32);
        for (index, heater) in periph.heaters.iter_mut().enumerate() {
            heater.step(MODEL_DT_S, duty[index], duty[FAN_INPUT]);
        }
        periph.window_high = [0; INPUTS];
        periph.window_samples = 0;
        publish_thermal(periph);
    }
    if let Some((reply, remaining)) = periph.pending.as_mut() {
        *remaining -= 1;
        if *remaining == 0 {
            let frame = reply.frame;
            periph.pending = None;
            begin_transmit(periph, frame);
        }
    }
}

fn begin_transmit(periph: &mut Peripherals, frame: [u8; REPLY_LEN]) {
    let uart = regs::USART2;
    uart.set_cr1(regs::USART_CR1_UE | regs::USART_CR1_TE);
    uart.set_dr(frame[0]);
    periph.transmit = Some(Transmit { frame, sent: 1 });
    uart.set_cr1(regs::USART_CR1_UE | regs::USART_CR1_TE | regs::USART_CR1_TXEIE);
}

pub fn on_uart() {
    let uart: Usart = regs::USART2;
    let periph = peripherals();
    let status = uart.sr();
    if let Some(transmit) = periph.transmit.as_mut() {
        if status & regs::USART_SR_TXE != 0 && uart.cr1() & regs::USART_CR1_TXEIE != 0 {
            if transmit.sent < REPLY_LEN {
                uart.set_dr(transmit.frame[transmit.sent]);
                transmit.sent += 1;
            } else {
                uart.set_cr1(regs::USART_CR1_UE | regs::USART_CR1_TE | regs::USART_CR1_TCIE);
            }
            return;
        }
        if status & regs::USART_SR_TC != 0 {
            periph.transmit = None;
            uart.sr();
            uart.dr();
            uart.set_cr1(receiving());
        }
        return;
    }
    if status & regs::USART_SR_ORE != 0 {
        periph.tmc.on_overrun();
    } else if status & regs::USART_SR_IDLE != 0 {
        periph.tmc.on_idle();
    }
    let received = uart.dr();
    if status & regs::USART_SR_RXNE != 0 {
        if let Some(reply) = periph.tmc.on_byte(received) {
            if periph.pending.is_some() {
                periph.tmc.on_reply_collision();
            } else {
                let wait = reply_wait_ticks(reply.delay_bits, TMC_BIT_NS, SAMPLE_PERIOD_NS);
                periph.pending = Some((reply, wait));
            }
        }
    }
    publish_tmc(&periph.tmc);
}
