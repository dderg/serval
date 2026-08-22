#![allow(unsafe_code)]

use core::cell::UnsafeCell;
use portable_atomic::{AtomicBool, AtomicI32, AtomicU8, AtomicU16, AtomicU32, AtomicU64};

use crate::clock::WidenState;
use crate::engine::Engine;

pub use crate::sizing::RT_STORAGE_SIZE;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum StepMode {
    Modulated = 0,
    StepTime = 1,
}

impl StepMode {
    pub fn from_u8(raw: u8) -> Option<StepMode> {
        match raw {
            0 => Some(StepMode::Modulated),
            1 => Some(StepMode::StepTime),
            _ => None,
        }
    }
}

pub const MAX_STEPPER_OIDS: usize = 16;

#[derive(Debug, Clone, Copy)]
pub struct TickState {
    pub dt: f32,
    pub positions: [f32; 4], // [x, y, z, e] logical
    pub motors: [f32; 4],    // [a, b, z, e] or [x, y, z, e] post-kinematic
}

pub type EngineImpl = Engine;

// `force_idle` handshake calls Klipper's `irq_save`/`irq_restore` to gate
// the queue-drain step. We import via thin C wrappers `runtime_irq_save` /
// `runtime_irq_restore` defined in `src/runtime_tick.c`, not the bare
// functions directly: Klipper's MCU build uses `-flto=auto -fwhole-program`,
// which lets the LTO inliner DCE the standalone `irq_save` / `irq_restore`
// symbols — leaving the kalico staticlib's references unresolvable at the
// final link. The wrappers are marked `used + externally_visible` on the C
// side so LTO keeps them.
unsafe extern "C" {
    pub fn runtime_irq_save() -> u32;
    pub fn runtime_irq_restore(flags: u32);
}

#[derive(Debug)]
pub struct FgState {
    pub current_stream_id: Option<u32>,
    pub armed_t_start_t0: Option<u64>,
    pub flush_start_tick: Option<u64>,
}

/// ISR-half state. Written exclusively by the TIM5 ISR
/// (`runtime_tick_sample`). Mutual exclusion is enforced by
/// Cortex-M NVIC priority arbitration.
#[allow(missing_debug_implementations)]
#[repr(C)]
pub struct IsrState {
    pub engine: EngineImpl,
    /// ISR is the sole writer of `widen_state`; foreground must not call
    /// `widen()` directly.
    pub widen_state: WidenState,
    pub last_tick_now: Option<u64>,
}

#[derive(Debug)]
#[repr(C)]
pub struct SharedState {
    pub last_error: AtomicI32,
    pub runtime_status: AtomicU8,
    pub stream_open: AtomicBool,
    /// Foreground sets `force_idle` first, ack-waits, then clears
    /// `stream_open` (flush handshake decision A).
    pub force_idle: AtomicBool,
    pub acked_force_idle: AtomicBool,
    /// §11.4 widened-clock seqlock — foreground reads, ISR writes.
    pub widened_now_lo: AtomicU32,
    pub widened_now_hi: AtomicU32,
    pub widened_now_seq: AtomicU32,
    /// §9.2: last latched fault's encoded `fault_detail` payload.
    /// Set in lockstep with `last_error`. `0` when no fault has latched.
    pub fault_detail: AtomicU32,
    /// Most-recently-dispatched scheduler timer func at `-311` fault time.
    /// `0` until `-311`.
    pub tick_blocker_func: AtomicU32,
    /// Stacked exception-frame PC at TIM5 handler entry, captured on the
    /// `-311` path. Primary `addr2line` target. `0` until `-311`.
    pub tick_blocker_pc: AtomicU32,
    /// Stacked xPSR exception number at `-311` time (`xPSR & 0x1FF`).
    /// `0` = foreground was interrupted; nonzero = that IRQ overran into TIM5.
    pub tick_blocker_exc: AtomicU32,
    pub stepper_counts: [AtomicI32; MAX_STEPPER_OIDS],
    pub step_modes: [AtomicU8; MAX_STEPPER_OIDS],
    /// `phase_slot_idx[motor_idx]` is the kinematic slot whose commanded
    /// `motors[slot_idx]` position drives motor `motor_idx`'s XDIRECT output.
    /// Unused entries hold `0xFF`.
    pub phase_slot_idx: [AtomicU8; MAX_STEPPER_OIDS],
    /// ISR loops `0..phase_motor_count` rather than scanning the full array.
    /// `0` disables phase stepping on this MCU.
    pub phase_motor_count: AtomicU8,

    /// Per-stage running MAX cycle counts for `runtime::tick::isr_sample_tick`.
    /// Read via status_drain rotation as tags 0xE6/0xE7/0xE8.
    /// If any stage approaches the TIM5 period (~13000 cycles at 40 kHz on H7),
    /// that stage is starving the foreground.
    pub isr_widen_cycles_max: AtomicU32,
    pub isr_arm_cycles_max: AtomicU32,
    pub isr_eval_cycles_max: AtomicU32,

    pub queue_overflow_count: [AtomicU32; 4],

    pub sample_period_cycles: AtomicU32,

    pub max_phase_offset_ramp_per_sample: AtomicU16,

    /// trsync trip handshake. The trip runs at a lower NVIC priority than
    /// TIM5, so it may not touch `IsrState`: it publishes the halt clock here
    /// and the next tick performs the halt. [`NO_HALT_REQUEST`] means idle, and
    /// the first requester wins so a trip that signals every stepper still
    /// halts every lane at one clock.
    pub sample_halt_clock: AtomicU64,
}

pub const NO_HALT_REQUEST: u64 = u64::MAX;

impl SharedState {
    pub const fn new() -> Self {
        Self {
            last_error: AtomicI32::new(0),
            runtime_status: AtomicU8::new(crate::engine::RuntimeStatus::Idle as u8),
            stream_open: AtomicBool::new(false),
            force_idle: AtomicBool::new(false),
            acked_force_idle: AtomicBool::new(false),
            widened_now_lo: AtomicU32::new(0),
            widened_now_hi: AtomicU32::new(0),
            widened_now_seq: AtomicU32::new(0),
            fault_detail: AtomicU32::new(0),
            tick_blocker_func: AtomicU32::new(0),
            tick_blocker_pc: AtomicU32::new(0),
            tick_blocker_exc: AtomicU32::new(0),
            stepper_counts: [
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
            ],
            step_modes: [
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
            ],
            phase_slot_idx: [
                AtomicU8::new(0xFF),
                AtomicU8::new(0xFF),
                AtomicU8::new(0xFF),
                AtomicU8::new(0xFF),
                AtomicU8::new(0xFF),
                AtomicU8::new(0xFF),
                AtomicU8::new(0xFF),
                AtomicU8::new(0xFF),
                AtomicU8::new(0xFF),
                AtomicU8::new(0xFF),
                AtomicU8::new(0xFF),
                AtomicU8::new(0xFF),
                AtomicU8::new(0xFF),
                AtomicU8::new(0xFF),
                AtomicU8::new(0xFF),
                AtomicU8::new(0xFF),
            ],
            phase_motor_count: AtomicU8::new(0),
            isr_widen_cycles_max: AtomicU32::new(0),
            isr_arm_cycles_max: AtomicU32::new(0),
            isr_eval_cycles_max: AtomicU32::new(0),

            queue_overflow_count: [const { AtomicU32::new(0) }; 4],

            sample_period_cycles: AtomicU32::new(0),

            max_phase_offset_ramp_per_sample: AtomicU16::new(0),

            sample_halt_clock: AtomicU64::new(NO_HALT_REQUEST),
        }
    }
}

impl Default for SharedState {
    fn default() -> Self {
        Self::new()
    }
}

/// Half-split runtime context. `fg` and `isr` are disjoint memory regions
/// behind `UnsafeCell` so the FFI shim can project to either half via
/// `core::ptr::addr_of!` + `UnsafeCell::raw_get` without materializing
/// `&mut RuntimeContext`. At most ONE `&mut FgState` OR `&mut IsrState`
/// (disjoint memory) exists at a time. `shared` uses no `UnsafeCell` — all
/// writes go through atomics.
#[allow(missing_debug_implementations)]
pub struct RuntimeContext {
    pub fg: UnsafeCell<FgState>,
    pub isr: UnsafeCell<IsrState>,
    pub shared: SharedState,
}

// SAFETY: `UnsafeCell` is `!Sync` by default — we provide `Sync` for
// `RuntimeContext` because the FFI shim only ever forms shared `&` and
// projects to disjoint `&mut FgState` / `&mut IsrState` via raw pointers,
// never `&mut RuntimeContext`.
unsafe impl Sync for RuntimeContext {}

unsafe extern "C" {
    static runtime_clock_freq: u32;
    /// TIM5 ISR fire rate (Hz). Defined in `src/runtime_tick.c` as
    /// `CONFIG_MOTION_SAMPLE_RATE_HZ` (defaults: 40000 on H7,
    /// 20000 on F4, 10000 on Linux sim).
    static runtime_sample_rate_hz: u32;
}

impl RuntimeContext {
    /// # Safety
    ///
    /// Caller must guarantee single-threaded init before any FFI call hits the
    /// runtime. Writes through raw-pointer projections; never materializes
    /// `&mut RuntimeContext`; sound to call against `MaybeUninit::as_mut_ptr()`.
    pub unsafe fn init(rt_ptr: *mut RuntimeContext) {
        // SAFETY: caller guarantees `rt_ptr` is valid for writes and
        // exclusively-owned for the duration of init. We only form raw
        // pointers to fields and never a `&mut RuntimeContext`.
        unsafe {
            let shared_ptr = core::ptr::addr_of_mut!((*rt_ptr).shared);
            shared_ptr.write(SharedState::new());

            let freq = core::ptr::read_volatile(core::ptr::addr_of!(runtime_clock_freq));
            let sample_rate_hz =
                core::ptr::read_volatile(core::ptr::addr_of!(runtime_sample_rate_hz));

            // Integer division is intentional: round-to-nearest cycles.
            #[allow(clippy::integer_division)]
            let sample_period_cycles_init: u32 = if sample_rate_hz == 0 {
                0
            } else {
                (freq + sample_rate_hz / 2) / sample_rate_hz
            };
            let shared_ref: *const SharedState = core::ptr::addr_of!((*rt_ptr).shared);
            (*shared_ref).sample_period_cycles.store(
                sample_period_cycles_init,
                core::sync::atomic::Ordering::Release,
            );

            let fg_ptr = core::ptr::addr_of_mut!((*rt_ptr).fg);
            fg_ptr.write(UnsafeCell::new(FgState {
                current_stream_id: None,
                armed_t_start_t0: None,
                flush_start_tick: None,
            }));

            // Initialize IsrState in-place to avoid materializing the
            // multi-KB `EngineImpl` on the 8 KB MCU stack.
            let isr_ptr = core::ptr::addr_of_mut!((*rt_ptr).isr);
            let inner_ptr: *mut IsrState = UnsafeCell::raw_get(isr_ptr);
            EngineImpl::init_in_place_production(
                core::ptr::addr_of_mut!((*inner_ptr).engine),
                freq,
                sample_rate_hz,
            );
            core::ptr::addr_of_mut!((*inner_ptr).widen_state).write(WidenState::default());
            core::ptr::addr_of_mut!((*inner_ptr).last_tick_now).write(None);
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SetStepModeError {
    CapabilityMissing,
    OutOfRange,
}

/// `Release` ordering on the store pairs with `Acquire` loads in the
/// engine ISR and any foreground reader of `step_modes`.
pub fn set_step_mode(
    shared: &SharedState,
    stepper_idx: u8,
    mode: StepMode,
    mcu_supports_phase: bool,
) -> Result<(), SetStepModeError> {
    if (stepper_idx as usize) >= MAX_STEPPER_OIDS {
        return Err(SetStepModeError::OutOfRange);
    }
    if mode == StepMode::Modulated && !mcu_supports_phase {
        return Err(SetStepModeError::CapabilityMissing);
    }
    // SAFETY: bounds checked above: stepper_idx < MAX_STEPPER_OIDS == step_modes.len()
    #[allow(clippy::indexing_slicing)]
    shared.step_modes[stepper_idx as usize]
        .store(mode as u8, core::sync::atomic::Ordering::Release);
    Ok(())
}

/// Install the `motor_idx → kinematic slot` mapping `write_phase_coils` uses to
/// address XDIRECT SPI writes, and mark the slot Modulated. Called from the
/// foreground command path (C `runtime_register_phase_motor`), never the ISR.
/// `Release` stores pair with the ISR's `Acquire` loads.
pub fn bind_phase_motor(
    shared: &SharedState,
    motor_idx: u8,
    slot_idx: u8,
) -> Result<(), SetStepModeError> {
    if (motor_idx as usize) >= MAX_STEPPER_OIDS
        || (slot_idx as usize) >= crate::stepping_state::MAX_AXES
    {
        return Err(SetStepModeError::OutOfRange);
    }
    // SAFETY of indexing: both bounds checked above.
    #[allow(clippy::indexing_slicing)]
    shared.phase_slot_idx[motor_idx as usize]
        .store(slot_idx, core::sync::atomic::Ordering::Release);
    let count = shared
        .phase_motor_count
        .load(core::sync::atomic::Ordering::Acquire);
    if motor_idx + 1 > count {
        shared
            .phase_motor_count
            .store(motor_idx + 1, core::sync::atomic::Ordering::Release);
    }
    #[allow(clippy::indexing_slicing)]
    shared.step_modes[slot_idx as usize].store(
        StepMode::Modulated as u8,
        core::sync::atomic::Ordering::Release,
    );
    Ok(())
}

#[cfg(test)]
mod size_task18;

#[cfg(test)]
mod tests;
