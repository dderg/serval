use core::ptr::{read_volatile, write_volatile};

const RCC_APB1ENR: usize = 0x4002_3840;
const RCC_APB2ENR: usize = 0x4002_3844;

pub const APB1_TIM3: u32 = 1 << 1;
pub const APB1_TIM4: u32 = 1 << 2;
pub const APB1_TIM6: u32 = 1 << 4;
pub const APB2_TIM1: u32 = 1 << 0;
pub const APB2_TIM8: u32 = 1 << 1;

pub fn enable_apb1(mask: u32) {
    set_bits(RCC_APB1ENR, mask);
}

pub fn enable_apb2(mask: u32) {
    set_bits(RCC_APB2ENR, mask);
}

fn set_bits(addr: usize, mask: u32) {
    unsafe { write_volatile(addr as *mut u32, read_volatile(addr as *const u32) | mask) }
}

#[derive(Clone, Copy)]
pub struct Timer(pub usize);

pub const TIM1: Timer = Timer(0x4001_0000);
pub const TIM8: Timer = Timer(0x4001_0400);
pub const TIM3: Timer = Timer(0x4000_0400);
pub const TIM4: Timer = Timer(0x4000_0800);
pub const TIM6: Timer = Timer(0x4000_1000);

pub const CR1_CEN: u32 = 1 << 0;
pub const DIER_UIE: u32 = 1 << 0;
pub const SR_UIF: u32 = 1 << 0;
pub const SR_CC2IF: u32 = 1 << 2;
pub const SR_CC2OF: u32 = 1 << 10;
pub const EGR_UG: u32 = 1 << 0;

impl Timer {
    fn reg(self, offset: usize) -> *mut u32 {
        (self.0 + offset) as *mut u32
    }

    fn read(self, offset: usize) -> u32 {
        unsafe { read_volatile(self.reg(offset)) }
    }

    fn write(self, offset: usize, value: u32) {
        unsafe { write_volatile(self.reg(offset), value) }
    }

    pub fn set_cr1(self, v: u32) {
        self.write(0x00, v)
    }
    pub fn set_smcr(self, v: u32) {
        self.write(0x08, v)
    }
    pub fn set_dier(self, v: u32) {
        self.write(0x0C, v)
    }
    pub fn sr(self) -> u32 {
        self.read(0x10)
    }
    pub fn set_sr(self, v: u32) {
        self.write(0x10, v)
    }
    pub fn set_egr(self, v: u32) {
        self.write(0x14, v)
    }
    pub fn set_ccmr1(self, v: u32) {
        self.write(0x18, v)
    }
    pub fn set_ccer(self, v: u32) {
        self.write(0x20, v)
    }
    pub fn cnt(self) -> u16 {
        self.read(0x24) as u16
    }
    pub fn set_cnt(self, v: u32) {
        self.write(0x24, v)
    }
    pub fn set_psc(self, v: u32) {
        self.write(0x28, v)
    }
    pub fn set_arr(self, v: u32) {
        self.write(0x2C, v)
    }
    pub fn ccr2(self) -> u16 {
        self.read(0x38) as u16
    }
}

#[derive(Clone, Copy)]
pub struct InputPin {
    pub port: usize,
    pub pin: u32,
}

impl InputPin {
    pub fn is_high(self) -> bool {
        let idr = unsafe { read_volatile((self.port + 0x10) as *const u32) };
        idr & (1 << self.pin) != 0
    }
}

pub const GPIOA: usize = 0x4002_0000;
pub const GPIOC: usize = 0x4002_0800;
pub const GPIOD: usize = 0x4002_0C00;
pub const GPIOE: usize = 0x4002_1000;
