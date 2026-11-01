#![no_std]
#![no_main]

mod capture;
mod regs;
mod ring;

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

use capture::{Fault, Sampler, CHANNEL_COUNT};
use ring::TickRing;

const FRAME_MAGIC: [u8; 4] = *b"SVC1";
const THROUGHPUT_MAGIC: [u8; 4] = *b"SVT1";
const FRAME_HEADER_LEN: usize = 16;
const TICKS_PER_FRAME: usize = 128;
const TICK_BYTES: usize = CHANNEL_COUNT;
const FRAME_MAX_LEN: usize = FRAME_HEADER_LEN + TICKS_PER_FRAME * TICK_BYTES;
const STATUS_DATA: u8 = 0;
const CMD_START: u8 = b'S';
const CMD_STOP: u8 = b'X';
const CMD_THROUGHPUT: u8 = b'T';
const NO_FAULT: u16 = 0;
const USB_VID_ST: u16 = 0x0483;
const USB_PID_VCP: u16 = 0x5740;

static RING: TickRing = TickRing::new();
static SAMPLER: Mutex<RefCell<Option<Sampler>>> = Mutex::new(RefCell::new(None));
static FAULT: AtomicU16 = AtomicU16::new(NO_FAULT);
static mut EP_MEMORY: [u32; 1024] = [0; 1024];
static mut USB_BUS: Option<UsbBusAllocator<UsbBus<USB>>> = None;

fn latch_fault(fault: Fault, channel: u8) {
    capture::stop_snapshot_timer();
    FAULT.store(
        u16::from_le_bytes([fault as u8, channel]),
        Ordering::Release,
    );
}

#[interrupt]
fn TIM6_DAC() {
    capture::ack_snapshot_tick();
    cortex_m::interrupt::free(|cs| {
        let mut slot = SAMPLER.borrow(cs).borrow_mut();
        let Some(sampler) = slot.as_mut() else {
            return;
        };
        match sampler.sample() {
            Ok(tick) => {
                if RING.push(tick).is_err() {
                    latch_fault(Fault::RingOverflow, 0);
                }
            }
            Err((fault, channel)) => latch_fault(fault, channel),
        }
    });
}

struct Stream {
    frame: [u8; FRAME_MAX_LEN],
    frame_len: usize,
    sent: usize,
    seq: u32,
    fault_reported: bool,
    throughput: bool,
    command: [u8; 2],
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
            throughput: false,
            command: [0; 2],
            command_len: 0,
        }
    }

    fn idle(&self) -> bool {
        self.sent == self.frame_len
    }

    fn write_header(&mut self, first_tick: u32, ticks: u16, status: u8, detail: u8) {
        self.write_header_with(FRAME_MAGIC, first_tick, ticks, status, detail);
    }

    fn write_header_with(
        &mut self,
        magic: [u8; 4],
        first_tick: u32,
        ticks: u16,
        status: u8,
        detail: u8,
    ) {
        self.frame[0..4].copy_from_slice(&magic);
        self.frame[4..8].copy_from_slice(&self.seq.to_le_bytes());
        self.frame[8..12].copy_from_slice(&first_tick.to_le_bytes());
        self.frame[12..14].copy_from_slice(&ticks.to_le_bytes());
        self.frame[14] = status;
        self.frame[15] = detail;
        self.seq = self.seq.wrapping_add(1);
        self.sent = 0;
    }

    fn stage_data(&mut self, ticks: usize) {
        let first_tick = RING.consumed() as u32;
        self.write_header(first_tick, ticks as u16, STATUS_DATA, 0);
        let payload_end = FRAME_HEADER_LEN + ticks * TICK_BYTES;
        RING.pop_into(&mut self.frame[FRAME_HEADER_LEN..payload_end]);
        self.frame_len = payload_end;
    }

    fn stage_fault(&mut self, fault: u16) {
        let [code, channel] = fault.to_le_bytes();
        self.write_header(RING.consumed() as u32, 0, code, channel);
        self.frame_len = FRAME_HEADER_LEN;
        self.fault_reported = true;
    }

    fn stage_throughput(&mut self) {
        let pattern = self.seq as u8;
        let payload_len = (FRAME_MAX_LEN - FRAME_HEADER_LEN) as u16;
        self.write_header_with(THROUGHPUT_MAGIC, 0, payload_len, STATUS_DATA, 0);
        self.frame[FRAME_HEADER_LEN..].fill(pattern);
        self.frame_len = FRAME_MAX_LEN;
    }

    fn stage_next(&mut self) {
        if self.throughput {
            self.stage_throughput();
            return;
        }
        let available = RING.available();
        if available >= TICKS_PER_FRAME {
            self.stage_data(TICKS_PER_FRAME);
            return;
        }
        let fault = FAULT.load(Ordering::Acquire);
        if fault == NO_FAULT || self.fault_reported {
            return;
        }
        if available > 0 {
            self.stage_data(available);
            return;
        }
        self.stage_fault(fault);
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

    fn start(&mut self, both_edges_mask: u8) {
        self.stop();
        self.seq = 0;
        self.fault_reported = false;
        let sampler = Sampler::arm(both_edges_mask);
        cortex_m::interrupt::free(|cs| SAMPLER.borrow(cs).replace(Some(sampler)));
        capture::start_snapshot_timer();
    }

    fn start_throughput(&mut self) {
        self.stop();
        self.seq = 0;
        self.throughput = true;
    }

    fn stop(&mut self) {
        self.throughput = false;
        capture::stop_snapshot_timer();
        cortex_m::interrupt::free(|cs| SAMPLER.borrow(cs).replace(None));
        RING.reset();
        FAULT.store(NO_FAULT, Ordering::Release);
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
            if self.command_len < self.command.len() {
                continue;
            }
            self.command_len = 0;
            match self.command {
                [CMD_START, mask] => self.start(mask),
                [CMD_THROUGHPUT, _] => self.start_throughput(),
                [CMD_STOP, _] => self.stop(),
                _ => panic!("unknown command"),
            }
        }
    }
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.freeze(
        Config::hse(8.MHz())
            .sysclk(168.MHz())
            .pclk1(42.MHz())
            .pclk2(84.MHz())
            .require_pll48clk(),
    );

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

    capture::enable_clocks();
    unsafe { NVIC::unmask(Interrupt::TIM6_DAC) };

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
