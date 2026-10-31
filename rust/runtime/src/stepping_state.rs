use core::sync::atomic::{AtomicI32, AtomicU8};
use heapless::Vec;
use runtime_contract::axes::{MAX_STEPPERS_PER_AXIS, StepMode};

#[derive(Debug)]
pub struct StepperRef {
    pub stepper_oid: u8,
    pub position_count: AtomicI32,
    /// OID of `command_config_spi` for this stepper's TMC driver.
    /// `None` means Pulse-only (no SPI traffic for this stepper).
    pub tmc_cs_oid: Option<u8>,
    pub phase_offset_microsteps: AtomicI32,
    pub phase_offset_target: AtomicI32,
    pub last_phase_target: AtomicI32,
}

impl StepperRef {
    pub fn new(stepper_oid: u8, tmc_cs_oid: Option<u8>) -> Self {
        Self {
            stepper_oid,
            position_count: AtomicI32::new(0),
            tmc_cs_oid,
            phase_offset_microsteps: AtomicI32::new(0),
            phase_offset_target: AtomicI32::new(0),
            last_phase_target: AtomicI32::new(0),
        }
    }
}

/// `mode` is atomic so the host can flip between Pulse and Phase without
/// a stop-the-world handshake.
#[derive(Debug)]
pub struct AxisState {
    pub mode: AtomicU8,
    pub steppers: Vec<StepperRef, MAX_STEPPERS_PER_AXIS>,
    pub microstep_distance: f32,
    pub last_step_count: i32,
    pub p_prev: f32,
    pub v_prev: f32,
}

impl AxisState {
    pub const fn new_unconfigured() -> Self {
        Self {
            mode: AtomicU8::new(StepMode::Pulse as u8),
            steppers: Vec::new(),
            microstep_distance: 0.0,
            last_step_count: 0,
            p_prev: 0.0,
            v_prev: 0.0,
        }
    }

    pub fn reset_isr_cache(&mut self) {
        self.last_step_count = 0;
        self.p_prev = 0.0;
        self.v_prev = 0.0;
    }
}
