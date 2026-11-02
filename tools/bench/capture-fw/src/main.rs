#![no_std]
#![no_main]

mod capture;
mod event;
mod regs;
mod ring;
mod sim;

use core::cell::RefCell;
use core::sync::atomic::{AtomicU16, Ordering};

use cortex_m::interrupt::Mutex;
use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;
use stm32f4xx_hal::otg_fs::{UsbBus, USB};
use stm32f4xx_hal::pac::{self, interrupt, Interrupt};
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::rcc::Config;
use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::SerialPort;

use bench_sim_logic::aux::AUX_RECORD_LEN;
use bench_sim_logic::config::MAX_PAYLOAD;
use capture::{Fault, Sampler, CHANNEL_COUNT};
use ring::TickRing;

const COUNT_MAGIC: [u8; 4] = *b"SVC1";
const EVENT_MAGIC: [u8; 4] = *b"SVE1";
const THROUGHPUT_MAGIC: [u8; 4] = *b"SVT1";
const AUX_MAGIC: [u8; 4] = *b"SVA1";
const FRAME_HEADER_LEN: usize = 16;
const TICKS_PER_FRAME: usize = 128;
const TICK_BYTES: usize = CHANNEL_COUNT;
const FRAME_MAX_LEN: usize = FRAME_HEADER_LEN + TICKS_PER_FRAME * TICK_BYTES;
const FRAME_PAYLOAD_LEN: usize = FRAME_MAX_LEN - FRAME_HEADER_LEN;
const AUX_PER_FRAME: usize = FRAME_PAYLOAD_LEN / AUX_RECORD_LEN;
const COUNT_TICKS_PER_AUX: u32 = capture::TICK_HZ / 1_000;
const EVENT_SNAPSHOTS_PER_AUX: u32 = event::SNAPSHOT_HZ / 1_000;
const STATUS_DATA: u8 = 0;
const CMD_START: u8 = b'S';
const CMD_EVENTS: u8 = b'E';
const CMD_START_AUX: u8 = b's';
const CMD_EVENTS_AUX: u8 = b'e';
const CMD_CONFIG: u8 = b'C';
const CONFIG_HEADER_LEN: usize = 3;
const CMD_STOP: u8 = b'X';
const CMD_THROUGHPUT: u8 = b'T';
const NO_FAULT: u16 = 0;
const USB_VID_ST: u16 = 0x0483;
const USB_PID_VCP: u16 = 0x5740;
const DIR_EDGE_PRIORITY: u8 = 0x00;
const SNAPSHOT_PRIORITY: u8 = 0x40;
const PERIPHERAL_PRIORITY: u8 = 0x80;

static RING: TickRing = TickRing::new();
static SAMPLER: Mutex<RefCell<Option<Sampler>>> = Mutex::new(RefCell::new(None));
static FAULT: AtomicU16 = AtomicU16::new(NO_FAULT);
static mut EP_MEMORY: [u32; 1024] = [0; 1024];
static mut USB_BUS: Option<UsbBusAllocator<UsbBus<USB>>> = None;

fn latch_fault(fault: Fault, channel: u8) {
    capture::stop_snapshot_timer();
    event::halt_interrupts();
    FAULT.store(
        u16::from_le_bytes([fault as u8, channel]),
        Ordering::Release,
    );
}

#[interrupt]
fn TIM6_DAC() {
    capture::ack_snapshot_tick();
    match event::tick() {
        Some(Ok(snapshot)) => {
            if let Err(fault) = sim::on_event_tick(&snapshot, snapshot.time) {
                latch_fault(fault, 0);
            }
            return;
        }
        Some(Err((fault, channel))) => {
            latch_fault(fault, channel);
            return;
        }
        None => {}
    }
    let mco = sim::mco_count();
    cortex_m::interrupt::free(|cs| {
        let mut slot = SAMPLER.borrow(cs).borrow_mut();
        let Some(sampler) = slot.as_mut() else {
            return;
        };
        match sampler.sample() {
            Ok(sample) => {
                let time = RING.produced() as u64;
                if RING.push(sample.packed).is_err() {
                    latch_fault(Fault::RingOverflow, 0);
                    return;
                }
                if let Err(fault) = sim::on_count_tick(&sample, mco, time) {
                    latch_fault(fault, 0);
                }
            }
            Err((fault, channel)) => latch_fault(fault, channel),
        }
    });
}

#[interrupt]
fn TIM7() {
    sim::on_sample();
}

#[interrupt]
fn USART2() {
    sim::on_uart();
}

fn dir_edge(lane: usize) {
    if let Err(fault) = event::dir_edge(lane) {
        latch_fault(fault, lane as u8);
    }
}

#[interrupt]
fn TIM1_CC() {
    dir_edge(0);
}

#[interrupt]
fn TIM3() {
    dir_edge(1);
}

#[interrupt]
fn TIM4() {
    dir_edge(2);
}

#[interrupt]
fn TIM8_CC() {
    dir_edge(3);
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Mode {
    Counts,
    Events,
    Throughput,
}

impl Mode {
    fn magic(self) -> [u8; 4] {
        match self {
            Self::Counts => COUNT_MAGIC,
            Self::Events => EVENT_MAGIC,
            Self::Throughput => THROUGHPUT_MAGIC,
        }
    }
}

struct Stream {
    frame: [u8; FRAME_MAX_LEN],
    frame_len: usize,
    sent: usize,
    seq: u32,
    fault_reported: bool,
    mode: Mode,
    aux: bool,
    command: [u8; CONFIG_HEADER_LEN + MAX_PAYLOAD],
    command_len: usize,
}

impl Stream {
    fn new() -> Self {
        Self {
            frame: [0; FRAME_MAX_LEN],
            frame_len: 0,
            sent: 0,
            seq: 0,
            fault_reported: true,
            mode: Mode::Counts,
            aux: false,
            command: [0; CONFIG_HEADER_LEN + MAX_PAYLOAD],
            command_len: 0,
        }
    }

    fn idle(&self) -> bool {
        self.sent == self.frame_len
    }

    fn write_header(&mut self, position: u32, length: u16, status: u8, detail: u8) {
        self.write_header_as(self.mode.magic(), position, length, status, detail);
    }

    fn write_header_as(
        &mut self,
        magic: [u8; 4],
        position: u32,
        length: u16,
        status: u8,
        detail: u8,
    ) {
        self.frame[0..4].copy_from_slice(&magic);
        self.frame[4..8].copy_from_slice(&self.seq.to_le_bytes());
        self.frame[8..12].copy_from_slice(&position.to_le_bytes());
        self.frame[12..14].copy_from_slice(&length.to_le_bytes());
        self.frame[14] = status;
        self.frame[15] = detail;
        self.seq = self.seq.wrapping_add(1);
        self.sent = 0;
    }

    fn stage_ticks(&mut self, ticks: usize) {
        let first_tick = RING.consumed() as u32;
        self.write_header(first_tick, ticks as u16, STATUS_DATA, 0);
        let payload_end = FRAME_HEADER_LEN + ticks * TICK_BYTES;
        RING.pop_into(&mut self.frame[FRAME_HEADER_LEN..payload_end]);
        self.frame_len = payload_end;
    }

    fn stage_events(&mut self, bytes: usize) {
        let first_byte = event::OUTPUT.consumed() as u32;
        self.write_header(first_byte, bytes as u16, STATUS_DATA, 0);
        let payload_end = FRAME_HEADER_LEN + bytes;
        event::OUTPUT.pop_into(&mut self.frame[FRAME_HEADER_LEN..payload_end]);
        self.frame_len = payload_end;
    }

    fn stage_aux(&mut self, records: usize) {
        let first = sim::AUX.consumed() as u32;
        self.write_header_as(AUX_MAGIC, first, records as u16, STATUS_DATA, 0);
        let payload_end = FRAME_HEADER_LEN + records * AUX_RECORD_LEN;
        sim::AUX.pop_into(&mut self.frame[FRAME_HEADER_LEN..payload_end]);
        self.frame_len = payload_end;
    }

    fn stage_fault(&mut self, fault: u16, position: usize) {
        let [code, channel] = fault.to_le_bytes();
        self.write_header(position as u32, 0, code, channel);
        self.frame_len = FRAME_HEADER_LEN;
        self.fault_reported = true;
    }

    fn stage_throughput(&mut self) {
        let pattern = self.seq as u8;
        self.write_header(0, FRAME_PAYLOAD_LEN as u16, STATUS_DATA, 0);
        self.frame[FRAME_HEADER_LEN..].fill(pattern);
        self.frame_len = FRAME_MAX_LEN;
    }

    fn pending_fault(&self) -> Option<u16> {
        let fault = FAULT.load(Ordering::Acquire);
        (fault != NO_FAULT && !self.fault_reported).then_some(fault)
    }

    fn stage_next(&mut self) {
        let (available, full, position) = match self.mode {
            Mode::Throughput => {
                self.stage_throughput();
                return;
            }
            Mode::Counts => (RING.available(), TICKS_PER_FRAME, RING.consumed()),
            Mode::Events => (
                event::OUTPUT.available(),
                FRAME_PAYLOAD_LEN,
                event::OUTPUT.consumed(),
            ),
        };
        if available >= full {
            self.stage_data(full);
            return;
        }
        let aux = if self.aux { sim::AUX.available() } else { 0 };
        if aux >= AUX_PER_FRAME {
            self.stage_aux(AUX_PER_FRAME);
            return;
        }
        let Some(fault) = self.pending_fault() else {
            return;
        };
        if available > 0 {
            self.stage_data(available);
        } else if aux > 0 {
            self.stage_aux(aux);
        } else {
            self.stage_fault(fault, position);
        }
    }

    fn stage_data(&mut self, amount: usize) {
        match self.mode {
            Mode::Counts => self.stage_ticks(amount),
            Mode::Events => self.stage_events(amount),
            Mode::Throughput => unreachable!("throughput frames are staged directly"),
        }
    }

    fn transmit(&mut self, serial: &mut SerialPort<UsbBus<USB>>) {
        if self.idle() {
            self.stage_next();
        }
        if self.idle() {
            return;
        }
        if let Ok(written) = serial.write(&self.frame[self.sent..self.frame_len]) {
            self.sent += written;
        }
    }

    fn start_counts(&mut self, both_edges_mask: u8, aux: bool) {
        self.stop();
        self.mode = Mode::Counts;
        self.aux = aux;
        self.fault_reported = false;
        let sampler = Sampler::arm(both_edges_mask);
        cortex_m::interrupt::free(|cs| SAMPLER.borrow(cs).replace(Some(sampler)));
        sim::start_capture(aux, COUNT_TICKS_PER_AUX);
        capture::start_snapshot_timer(capture::TICK_HZ);
    }

    fn start_events(&mut self, both_edges_mask: u8, aux: bool) {
        self.stop();
        self.mode = Mode::Events;
        self.aux = aux;
        self.fault_reported = false;
        sim::start_capture(aux, EVENT_SNAPSHOTS_PER_AUX);
        event::start(both_edges_mask);
        capture::start_snapshot_timer(event::SNAPSHOT_HZ);
    }

    fn configure(&mut self, payload: &[u8]) {
        self.stop();
        if let Ok(config) = bench_sim_logic::config::parse(payload) {
            sim::apply_config(config);
        }
    }

    fn start_throughput(&mut self) {
        self.stop();
        self.mode = Mode::Throughput;
    }

    fn stop(&mut self) {
        capture::stop_snapshot_timer();
        event::halt();
        cortex_m::interrupt::free(|cs| SAMPLER.borrow(cs).replace(None));
        RING.reset();
        event::OUTPUT.reset();
        sim::stop_capture();
        FAULT.store(NO_FAULT, Ordering::Release);
        self.mode = Mode::Counts;
        self.aux = false;
        self.seq = 0;
        self.frame_len = 0;
        self.sent = 0;
        self.fault_reported = true;
    }

    fn receive(&mut self, serial: &mut SerialPort<UsbBus<USB>>) {
        let mut buf = [0u8; 64];
        let Ok(count) = serial.read(&mut buf) else {
            return;
        };
        for &byte in &buf[..count] {
            self.command[self.command_len] = byte;
            self.command_len += 1;
            if self.command_len < self.command_len_needed() {
                continue;
            }
            let len = self.command_len;
            self.command_len = 0;
            match self.command[..len] {
                [CMD_CONFIG, _, _, ..] => {
                    let mut payload = [0u8; MAX_PAYLOAD];
                    let size = len - CONFIG_HEADER_LEN;
                    payload[..size].copy_from_slice(&self.command[CONFIG_HEADER_LEN..len]);
                    self.configure(&payload[..size]);
                }
                [CMD_START, mask] => self.start_counts(mask, false),
                [CMD_START_AUX, mask] => self.start_counts(mask, true),
                [CMD_EVENTS, mask] => self.start_events(mask, false),
                [CMD_EVENTS_AUX, mask] => self.start_events(mask, true),
                [CMD_THROUGHPUT, _] => self.start_throughput(),
                [CMD_STOP, _] => self.stop(),
                _ => panic!("unknown command"),
            }
        }
    }

    fn command_len_needed(&self) -> usize {
        if self.command[0] != CMD_CONFIG {
            return 2;
        }
        if self.command_len < CONFIG_HEADER_LEN {
            return CONFIG_HEADER_LEN;
        }
        let payload = usize::from(u16::from_le_bytes([self.command[1], self.command[2]]));
        assert!(payload <= MAX_PAYLOAD, "config payload too long");
        CONFIG_HEADER_LEN + payload
    }
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.freeze(
        Config::hse(8.MHz())
            .sysclk(168.MHz())
            .pclk1(42.MHz())
            .pclk2(84.MHz())
            .require_pll48clk(),
    );
    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);
    let gpiod = dp.GPIOD.split(&mut rcc);
    let gpioe = dp.GPIOE.split(&mut rcc);

    let _m0_step = gpioe.pe9.into_alternate::<1>();
    let _m0_dir = gpioe.pe11.into_alternate::<1>();
    let _m1_step = gpioa.pa6.into_alternate::<2>();
    let _m1_dir = gpioa.pa7.into_alternate::<2>();
    let _m2_step = gpiod.pd12.into_alternate::<2>();
    let _m2_dir = gpiod.pd13.into_alternate::<2>();
    let _m3_step = gpioc.pc6.into_alternate::<3>();
    let _m3_dir = gpioc.pc7.into_alternate::<3>();

    let _x_endstop = gpiod.pd8.into_push_pull_output();
    let _y_endstop = gpiod.pd9.into_push_pull_output();
    let _z_endstop = gpiod.pd10.into_push_pull_output();
    let _probe = gpiod.pd11.into_push_pull_output();
    let _extruder_heater = gpiod.pd14.into_pull_down_input();
    let _bed_heater = gpiod.pd15.into_pull_down_input();
    let _part_fan = gpioe.pe8.into_pull_down_input();
    let _mco = gpioe.pe5.into_alternate::<3>();
    let _thermistor_extruder = gpioa.pa4.into_analog();
    let _thermistor_bed = gpioa.pa5.into_analog();
    let _tmc_uart = gpioa
        .pa2
        .into_alternate_open_drain::<7>()
        .internal_pull_up(true);

    capture::enable_clocks();
    event::enable_clocks();
    sim::enable_clocks();
    unsafe {
        cp.NVIC.set_priority(Interrupt::TIM6_DAC, SNAPSHOT_PRIORITY);
        for irq in [
            Interrupt::TIM1_CC,
            Interrupt::TIM3,
            Interrupt::TIM4,
            Interrupt::TIM8_CC,
        ] {
            cp.NVIC.set_priority(irq, DIR_EDGE_PRIORITY);
        }
        cp.NVIC.set_priority(Interrupt::TIM7, PERIPHERAL_PRIORITY);
        cp.NVIC.set_priority(Interrupt::USART2, PERIPHERAL_PRIORITY);
        NVIC::unmask(Interrupt::TIM6_DAC);
    }
    sim::init();

    let usb = USB::new(
        (dp.OTG_FS_GLOBAL, dp.OTG_FS_DEVICE, dp.OTG_FS_PWRCLK),
        (gpioa.pa11, gpioa.pa12),
        &rcc.clocks,
    );
    let usb_bus = unsafe {
        let bus = &mut *core::ptr::addr_of_mut!(USB_BUS);
        bus.insert(UsbBus::new(usb, &mut *core::ptr::addr_of_mut!(EP_MEMORY)))
    };
    let mut serial = SerialPort::new(usb_bus);
    let mut usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(USB_VID_ST, USB_PID_VCP))
        .device_class(usbd_serial::USB_CLASS_CDC)
        .strings(&[StringDescriptors::default()
            .manufacturer("serval-bench")
            .product("step-capture")
            .serial_number("f407-capture")])
        .unwrap()
        .build();

    let mut stream = Stream::new();
    loop {
        usb_dev.poll(&mut [&mut serial]);
        stream.receive(&mut serial);
        stream.transmit(&mut serial);
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    cortex_m::interrupt::disable();
    loop {
        cortex_m::asm::nop();
    }
}
