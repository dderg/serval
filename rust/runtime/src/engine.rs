use core::sync::atomic::{AtomicI32, AtomicU8, Ordering};

use crate::clock::TickCounter;
use crate::error::{RUNTIME_ERR_INVALID_ARG, RUNTIME_ERR_RING_FULL, RUNTIME_OK};
use crate::fault_sink::FaultSink;
use crate::piece_ring::PieceEntry;
use crate::state::SharedState;
use crate::step::StepMotorState;
use crate::stepping_state::{AxisState, MAX_AXES};

pub use crate::stepping_state::N_AXES;

mod config;
mod manual;
mod query;
#[cfg(feature = "sample-stepping")]
mod sample;
mod tick;

pub(crate) struct SharedFaultSink<'a> {
    pub shared: &'a SharedState,
}

impl FaultSink for SharedFaultSink<'_> {
    #[inline]
    fn piece_start_in_past(&self, axis_idx: usize, deficit_us: u32) {
        crate::fault_helpers::raise_piece_start_in_past(self.shared, axis_idx, deficit_us);
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum RuntimeStatus {
    Idle = 0,
    Running = 1,
    Drained = 2,
    Fault = 3,
}

impl RuntimeStatus {
    #[inline]
    pub fn from_u8(v: u8) -> Self {
        match v {
            0 => Self::Idle,
            1 => Self::Running,
            2 => Self::Drained,
            _ => Self::Fault,
        }
    }
}

#[allow(missing_debug_implementations)]
pub struct Engine {
    pub(crate) status: AtomicU8,
    pub(crate) last_error: AtomicI32,
    pub(crate) tick_counter: TickCounter,
    pub sample_period_cycles: u32,
    pub cycles_per_second: f32,
    pub stepping_axes: [Option<AxisState>; MAX_AXES],
    pub num_axes: u8,
    ring_alloc_cursor: usize,
    pub(crate) step_state: [StepMotorState; MAX_AXES],
    pub(crate) last_motors: [f32; MAX_AXES],
    pub tick_caches: crate::stepping_state::TickCaches,
    pieces_gated: bool,
    pub(crate) buzz: crate::buzz::Buzz,
    #[cfg(any(test, feature = "host"))]
    test_queue_ptrs: [*mut crate::step_queue::StepQueue; MAX_AXES],
    #[cfg(feature = "sample-stepping")]
    pub(crate) sample_lanes: [crate::sample_exec::SampleLane; MAX_AXES],
}

impl Engine {
    pub fn new(clock_freq: u32, sample_rate_hz: u32) -> Self {
        let (_, sample_period_cycles) = Self::compute_sample_period(clock_freq, sample_rate_hz);
        Self {
            status: AtomicU8::new(RuntimeStatus::Idle as u8),
            last_error: AtomicI32::new(0),
            tick_counter: TickCounter::new(),
            sample_period_cycles,
            cycles_per_second: clock_freq as f32,
            stepping_axes: [const { None }; MAX_AXES],
            num_axes: 0,
            ring_alloc_cursor: 0,
            step_state: [StepMotorState::default(); MAX_AXES],
            last_motors: [0.0; MAX_AXES],
            tick_caches: crate::stepping_state::TickCaches::new(),
            pieces_gated: false,
            buzz: crate::buzz::Buzz::new(),
            #[cfg(any(test, feature = "host"))]
            test_queue_ptrs: [core::ptr::null_mut(); MAX_AXES],
            #[cfg(feature = "sample-stepping")]
            sample_lanes: [const { crate::sample_exec::SampleLane::new() }; MAX_AXES],
        }
    }

    #[inline]
    fn compute_sample_period(clock_freq: u32, sample_rate_hz: u32) -> (f32, u32) {
        if sample_rate_hz == 0 {
            return (0.0, 0);
        }
        let sec = 1.0_f32 / (sample_rate_hz as f32);
        #[allow(clippy::integer_division)]
        let cycles = (clock_freq + sample_rate_hz / 2) / sample_rate_hz;
        (sec, cycles)
    }

    /// # Safety
    /// `ptr` must be valid for writes of `size_of::<Engine>()` bytes and must
    /// not be aliased for the duration of this call.
    #[allow(unsafe_code)]
    pub unsafe fn init_in_place(ptr: *mut Self, clock_freq: u32, sample_rate_hz: u32) {
        use core::ptr::addr_of_mut;
        let (_, sample_period_cycles) = Self::compute_sample_period(clock_freq, sample_rate_hz);
        unsafe {
            addr_of_mut!((*ptr).status).write(AtomicU8::new(RuntimeStatus::Idle as u8));
            addr_of_mut!((*ptr).last_error).write(AtomicI32::new(0));
            addr_of_mut!((*ptr).tick_counter).write(TickCounter::new());
            addr_of_mut!((*ptr).sample_period_cycles).write(sample_period_cycles);
            addr_of_mut!((*ptr).cycles_per_second).write(clock_freq as f32);
            addr_of_mut!((*ptr).stepping_axes).write([const { None }; MAX_AXES]);
            addr_of_mut!((*ptr).num_axes).write(0);
            addr_of_mut!((*ptr).ring_alloc_cursor).write(0);
            addr_of_mut!((*ptr).step_state).write([StepMotorState::default(); MAX_AXES]);
            addr_of_mut!((*ptr).last_motors).write([0.0; MAX_AXES]);
            addr_of_mut!((*ptr).tick_caches).write(crate::stepping_state::TickCaches::new());
            addr_of_mut!((*ptr).pieces_gated).write(false);
            addr_of_mut!((*ptr).buzz).write(crate::buzz::Buzz::new());
            #[cfg(any(test, feature = "host"))]
            addr_of_mut!((*ptr).test_queue_ptrs).write([core::ptr::null_mut(); MAX_AXES]);
            #[cfg(feature = "sample-stepping")]
            addr_of_mut!((*ptr).sample_lanes)
                .write([const { crate::sample_exec::SampleLane::new() }; MAX_AXES]);
        }
    }

    /// # Safety
    /// See [`init_in_place`].
    #[allow(unsafe_code)]
    pub unsafe fn init_in_place_production(ptr: *mut Self, clock_freq: u32, sample_rate_hz: u32) {
        unsafe { Self::init_in_place(ptr, clock_freq, sample_rate_hz) }
    }
}

impl Engine {
    /// Preserves `sample_period_cycles`, `cycles_per_second`, and the running
    /// `tick_counter` — resetting those would desync the ISR time base.
    pub fn reset(&mut self) {
        self.ring_alloc_cursor = 0;
        self.stepping_axes = [const { None }; MAX_AXES];
        self.num_axes = 0;
        self.step_state = [StepMotorState::default(); MAX_AXES];
        self.last_motors = [0.0; MAX_AXES];
        self.tick_caches = crate::stepping_state::TickCaches::new();
        self.status
            .store(RuntimeStatus::Idle as u8, Ordering::Release);
        self.last_error.store(0, Ordering::Release);
        self.pieces_gated = false;
        #[cfg(feature = "sample-stepping")]
        {
            self.sample_lanes = [const { crate::sample_exec::SampleLane::new() }; MAX_AXES];
        }
    }

    pub fn discard_pending(&mut self) {
        for axis_opt in self.stepping_axes.iter_mut() {
            let Some(axis) = axis_opt.as_mut() else {
                continue;
            };
            while axis.ring.front_slot().is_some() {
                axis.ring.advance_counter();
            }
            axis.armed = None;
        }
    }

    pub fn gate_pieces(&mut self) {
        self.discard_pending();
        self.pieces_gated = true;
    }

    pub fn ungate_pieces(&mut self) -> i32 {
        if !self.pieces_gated {
            return crate::error::RUNTIME_ERR_STREAM_STATE_VIOLATION;
        }
        self.discard_pending();
        self.pieces_gated = false;
        crate::error::RUNTIME_OK
    }

    pub fn pieces_gated(&self) -> bool {
        self.pieces_gated
    }

    pub fn push_pieces(
        &mut self,
        axis_idx: u8,
        pieces: &[PieceEntry],
        storage: &mut [PieceEntry],
    ) -> i32 {
        if crate::buzz_stream::axis_active(axis_idx as usize) {
            return RUNTIME_ERR_INVALID_ARG;
        }
        #[cfg(feature = "sample-stepping")]
        if self
            .sample_lanes
            .get(axis_idx as usize)
            .is_some_and(crate::sample_exec::SampleLane::is_anchored)
        {
            return crate::error::RUNTIME_ERR_MOTION_IN_PROGRESS;
        }
        let Some(axis) = self
            .stepping_axes
            .get_mut(axis_idx as usize)
            .and_then(|s| s.as_mut())
        else {
            return RUNTIME_ERR_INVALID_ARG;
        };
        for &piece in pieces {
            if axis.ring.push(storage, piece).is_err() {
                return RUNTIME_ERR_RING_FULL;
            }
        }
        RUNTIME_OK
    }

    pub fn runtime_force_idle(&mut self, shared: &SharedState) {
        for ss in &mut self.step_state {
            ss.reset_accumulator();
        }
        for axis_opt in &mut self.stepping_axes {
            if let Some(axis) = axis_opt.as_mut() {
                axis.reset_isr_cache();
                axis.ring.drain();
            }
        }
        self.last_motors = [0.0; MAX_AXES];
        if self.status() != RuntimeStatus::Fault {
            self.status
                .store(RuntimeStatus::Idle as u8, Ordering::Release);
        }
        shared.acked_force_idle.store(true, Ordering::Release);
    }

    #[cfg(any(test, feature = "host"))]
    pub fn test_install_step_queues(
        &mut self,
        queues: [*mut crate::step_queue::StepQueue; MAX_AXES],
    ) {
        self.test_queue_ptrs = queues;
    }
}

#[cfg(test)]
impl Default for Engine {
    fn default() -> Self {
        Self::new(520_000_000, crate::clock::TEST_ONLY_TICK_RATE_HZ)
    }
}

#[cfg(test)]
mod tests;
