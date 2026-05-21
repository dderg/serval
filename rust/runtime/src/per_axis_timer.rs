//! Per-axis Klipper `SysTick` consumer. Mainline pattern: fire one entry
//! per dispatch when its `cycle_abs` has arrived; never early.
//!
//! Body called from C-side `struct timer.func` via `extern "C"`.
//!
//! Spec: docs/superpowers/specs/2026-05-19-stepping-redesign-design.md
//! (Task 10 of 2026-05-19-stepping-redesign-implementation.md).
//!
//! Lifecycle:
//! 1. C-side `init_per_axis_step_timers` installs four `struct timer`s, one
//!    per axis (X=0, Y=1, Z=2, E=3), each bound to a thin C trampoline that
//!    reads `timer_read_time()` and calls this module with `(axis_idx, now)`.
//! 2. Each dispatch peeks the head of `step_queues[axis_idx]` and, if its
//!    `cycle_abs` is at-or-before `now`, pops + emits one step pulse via
//!    `runtime_emit_step_pulses(axis_idx, dir)`.
//! 3. The returned `u32` is the next waketime: the next entry's `cycle_abs`
//!    (floored by `dispatcher_floor_cycles` to prevent runaway re-entry), or
//!    `now + sample_period_cycles` if the queue is empty.
//!
//! # LTO-safety design
//!
//! The Klipper Linux build uses `-fwhole-program -flto` which internalises
//! C functions that the compiler does not see being called.  Rust's
//! `extern "C"` references in the staticlib are not visible to the C LTO
//! pass, so symbols without `__attribute__((used, externally_visible))` may
//! disappear at link time.
//!
//! To avoid this:
//! - `timer_read_time()` is called by the **C trampoline** (not Rust); the
//!   trampoline passes the result as the `now: u32` parameter.
//! - `timer_is_before()` is replaced by inline Rust arithmetic (same
//!   semantics: `(a.wrapping_sub(b) as i32) < 0`).
//! - `runtime_emit_step_pulses()` carries `__attribute__((used,
//!   externally_visible))` in `src/stepper.c` and survives LTO.
//! - `kalico_runtime_get_*` are defined in the Rust staticlib itself
//!   (`runtime_ffi.rs` with `#[no_mangle]`) and are resolved intra-archive.
//!
//! Coexists alongside the legacy `step_time_event` / `runtime_producer_event`
//! path until Task 16 removes the older code.

#![allow(unsafe_code)]

use crate::step_queue::{peek as queue_peek, pop as queue_pop};

// `runtime_emit_step_pulses` carries `__attribute__((used, externally_visible))`
// in `src/stepper.c` and is therefore safe to call from Rust even under
// `-fwhole-program -flto`.  Pure host builds and tests must not pull the
// undefined symbol into the cdylib — provide an inert stub instead.
#[cfg(not(any(test, all(feature = "host", not(feature = "linux-mcu")))))]
unsafe extern "C" {
    fn runtime_emit_step_pulses(axis_idx: u8, n_steps: i32);
}

// `kalico_runtime_get_dispatcher_floor_cycles` and
// `kalico_runtime_get_sample_period_cycles` are defined in the Rust staticlib
// (`kalico-c-api/src/runtime_ffi.rs`) with `#[unsafe(no_mangle)] pub
// extern "C"`.  The linker resolves them intra-archive without LTO risk.
// Host stubs return 0 (disabled path / uninitialized runtime).
#[cfg(not(any(test, all(feature = "host", not(feature = "linux-mcu")))))]
unsafe extern "C" {
    fn kalico_runtime_get_dispatcher_floor_cycles() -> u32;
    fn kalico_runtime_get_sample_period_cycles() -> u32;
}

#[cfg(any(test, all(feature = "host", not(feature = "linux-mcu"))))]
unsafe fn runtime_emit_step_pulses(_axis_idx: u8, _n_steps: i32) {}
#[cfg(any(test, all(feature = "host", not(feature = "linux-mcu"))))]
unsafe fn kalico_runtime_get_dispatcher_floor_cycles() -> u32 { 0 }
#[cfg(any(test, all(feature = "host", not(feature = "linux-mcu"))))]
unsafe fn kalico_runtime_get_sample_period_cycles() -> u32 { 0 }

/// Inline replacement for Klipper's `timer_is_before(a, b)`: returns `true`
/// iff `a` is strictly before `b` in wrap-around clock space.
///
/// Semantics match `src/linux/timer.c`:
/// ```c
/// uint8_t timer_is_before(uint32_t time1, uint32_t time2) {
///     return (int32_t)(time1 - time2) < 0;
/// }
/// ```
#[inline(always)]
fn clock_is_before(a: u32, b: u32) -> bool {
    (a.wrapping_sub(b) as i32) < 0
}

/// Rust body for the per-axis `struct timer.func` callback. Called from C
/// trampolines (one per axis 0..=3) in `src/runtime_tick.c`. The trampoline
/// reads `timer_read_time()` and passes the result as `now`.
///
/// Returns the next waketime (u32 cycle absolute) that the C wrapper writes
/// back to `t->waketime`.
///
/// Mainline pattern: one entry per dispatch, never fire early.
#[unsafe(no_mangle)]
pub extern "C" fn kalico_per_axis_step_event(axis_idx: u8, now: u32) -> u32 {
    let queue_ptr = resolve_queue_ptr(axis_idx as usize);

    // Pop one entry if its `cycle_abs` has arrived. Guard against a null
    // queue pointer (host builds and pre-Task-11 boot states) to keep this
    // entry point sound even before `init_per_axis_step_timers` would have
    // populated the C-side `step_queues` array.
    if !queue_ptr.is_null() {
        // SAFETY: `queue_ptr` is non-null, points at a live `StepQueue` for
        // the duration of the program (storage is the C-declared
        // `step_queues[N_AXIS_STEP_QUEUES]` placed in `.axi_bss`), and this
        // timer is the sole consumer for axis `axis_idx`.
        if let Some(entry) = unsafe { queue_peek(queue_ptr) } {
            // `now` is at-or-after `entry.cycle_abs` when `now` is NOT
            // strictly before `cycle_abs` (wrap-safe comparison).
            let arrived = !clock_is_before(now, entry.cycle_abs);
            if arrived {
                // SAFETY: same as peek above — sole consumer discipline.
                let _ = unsafe { queue_pop(queue_ptr) };
                // SAFETY: `runtime_emit_step_pulses` carries
                // `__attribute__((used, externally_visible))` (src/stepper.c)
                // and is a NOP for `axis_idx >= RUNTIME_MOTOR_COUNT`.
                unsafe { runtime_emit_step_pulses(axis_idx, i32::from(entry.dir)) };
            }
        }
    }

    // Next waketime: prefer the next pending entry's `cycle_abs`, floored
    // by `dispatcher_floor_cycles` to prevent runaway re-entry; if the
    // queue is empty, sleep until the next sample boundary.
    // SAFETY: both accessors are read-only AtomicU32 loads on `SharedState`
    // (via `runtime_handle_or_null`); they return 0 if the runtime hasn't
    // initialised yet. `0` for either tunable degrades safely: a 0 floor
    // means "no extra padding," and a 0 sample period means "wake `now`,"
    // which the next dispatch will immediately reschedule.
    let floor_cycles = unsafe { kalico_runtime_get_dispatcher_floor_cycles() };
    let sample_period = unsafe { kalico_runtime_get_sample_period_cycles() };
    let floor_time = now.wrapping_add(floor_cycles);
    let next_sample = now.wrapping_add(sample_period);

    if queue_ptr.is_null() {
        return next_sample;
    }

    // SAFETY: `queue_ptr` non-null + sole-consumer as above.
    match unsafe { queue_peek(queue_ptr) } {
        Some(next) => {
            // max(next.cycle_abs, floor_time), wrap-aware. If
            // `next.cycle_abs` is strictly before `floor_time`, push the
            // wake out to the floor to avoid spinning; else schedule for
            // the entry's exact arrival.
            if clock_is_before(next.cycle_abs, floor_time) {
                floor_time
            } else {
                next.cycle_abs
            }
        }
        None => next_sample,
    }
}

/// MCU build (bare-metal or Linux sim): resolve the queue pointer from the
/// C-declared `step_queues[N_AXIS_STEP_QUEUES]` array. Bounds-checked by
/// the caller (axis_idx ∈ 0..=3 is implicit from the four C trampolines).
///
/// Active for bare-metal MCU builds AND for the `linux-mcu` sim build (which
/// sets `feature = "host"` for std but links against real C objects).
#[cfg(not(any(test, all(feature = "host", not(feature = "linux-mcu")))))]
fn resolve_queue_ptr(axis_idx: usize) -> *mut crate::step_queue::StepQueue {
    use crate::step_queue::{step_queues, StepQueue};
    // SAFETY: `step_queues` is the C-declared array, `.add(axis_idx)` is
    // in-bounds for axis_idx ∈ 0..N_AXIS_STEP_QUEUES (caller invariant).
    unsafe { step_queues.get().cast::<StepQueue>().add(axis_idx) }
}

/// Pure host / test build: there is no C-declared `step_queues` to project
/// from — return null and let `kalico_per_axis_step_event` fall through its
/// null-check guards. Host-side smoke tests for the timer body would need
/// to mock through a host-only hook; deferred to Task 18 / bench bring-up.
#[cfg(any(test, all(feature = "host", not(feature = "linux-mcu"))))]
fn resolve_queue_ptr(_axis_idx: usize) -> *mut crate::step_queue::StepQueue {
    core::ptr::null_mut()
}
