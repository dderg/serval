// NON-RACING, not lock-free: single-core safety requires the producer (TIM5
// ISR) and consumer (step-output timer ISR) to share one NVIC priority so they
// never interleave. If that ever splits, the volatile-u16 + fence discipline is
// insufficient (torn slot/counter) — upgrade to a true-atomic SPSC. Invariant
// + priority map: `src/generic/motion_nvic_prio.h`.

#![allow(unsafe_code)]

use core::ptr;
use core::sync::atomic::{Ordering, fence};

/// Power-of-two ring depth shared with the C side; see `src/step_queue.h`.
/// Derived in build.rs from the configured sample rate: two worst-case
/// `MAX_STEPS_PER_SAMPLE` producer bursts, rounded up to a power of two.
pub const STEP_QUEUE_DEPTH: usize = crate::sizing::STEP_QUEUE_DEPTH;
pub const STEP_QUEUE_DEPTH_MASK: u16 = (STEP_QUEUE_DEPTH as u16) - 1;
pub const N_AXIS_STEP_QUEUES: usize = 4;

/// One pending step pulse: an absolute MCU cycle time, a direction, and a
/// stepper selector (`0` = every stepper of the motor, `n` = only stepper
/// `n-1` — used by correction streams to move one motor of a multi-stepper
/// axis).
///
/// Layout must match the C struct exactly — `#[repr(C)]` + the same field
/// order + the explicit 2-byte tail pad gives an 8-byte entry on every
/// target we care about (ABI-stable across H7 / F4 / host).
/// One scheduled action at `cycle_abs`. The trailing `payload` is interpreted by
/// the consumer per the axis `StepMode`, which is authoritative at both produce
/// and consume: in Pulse mode it packs `(dir, stepper_sel)` for a STEP/DIR pulse;
/// in Phase mode it is the signed `offset_steps` for an XDIRECT coil write. The
/// two never alias on one axis — a Phase-mode axis routes normal motion straight
/// to XDIRECT from TIM5 and only ever queues buzz updates here.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct StepEntry {
    pub cycle_abs: u32,
    payload: i32,
}

pub const STEPPER_SEL_ALL: u8 = 0;

impl StepEntry {
    #[must_use]
    #[allow(clippy::cast_possible_wrap)]
    pub fn pulse(cycle_abs: u32, dir: i8, stepper_sel: u8) -> Self {
        let payload = (u32::from(dir as u8) | (u32::from(stepper_sel) << 8)) as i32;
        Self { cycle_abs, payload }
    }

    #[must_use]
    pub fn xdirect(cycle_abs: u32, offset_steps: i32) -> Self {
        Self {
            cycle_abs,
            payload: offset_steps,
        }
    }

    #[must_use]
    #[allow(clippy::cast_possible_truncation)]
    pub fn dir(self) -> i8 {
        (self.payload & 0xFF) as i8
    }

    #[must_use]
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    pub fn stepper_sel(self) -> u8 {
        ((self.payload >> 8) & 0xFF) as u8
    }

    #[must_use]
    pub fn offset_steps(self) -> i32 {
        self.payload
    }
}

#[repr(C)]
#[derive(Debug)]
pub struct StepQueue {
    pub tail: u16,
    pub head: u16,
    _pad: [u8; 4],
    pub buf: [StepEntry; STEP_QUEUE_DEPTH],
}

impl StepQueue {
    #[cfg(any(test, feature = "host"))]
    #[must_use]
    pub fn new() -> Self {
        StepQueue {
            tail: 0,
            head: 0,
            _pad: [0; 4],
            buf: [StepEntry {
                cycle_abs: 0,
                payload: 0,
            }; STEP_QUEUE_DEPTH],
        }
    }

    /// Empty the queue by resetting both SPSC counters to 0.
    ///
    /// The caller must hold exclusive access (an IRQ guard): both producer
    /// (writes `tail`) and consumer (writes `head`) must be quiescent.
    #[inline]
    pub fn clear(&mut self) {
        self.tail = 0;
        self.head = 0;
    }
}

#[cfg(any(test, feature = "host"))]
impl Default for StepQueue {
    fn default() -> Self {
        Self::new()
    }
}

const _: () = {
    assert!(core::mem::size_of::<StepEntry>() == 8);
    assert!(core::mem::size_of::<StepQueue>() == 8 + 8 * STEP_QUEUE_DEPTH);
    assert!(STEP_QUEUE_DEPTH.is_power_of_two());
    assert!(STEP_QUEUE_DEPTH >= 2 * crate::sub_sample_timing::MAX_STEPS_PER_SAMPLE);
};

#[cfg(not(any(test, feature = "host")))]
unsafe extern "C" {
    pub static step_queues: core::cell::UnsafeCell<[StepQueue; N_AXIS_STEP_QUEUES]>;
}

#[cfg(not(any(test, feature = "host")))]
#[must_use]
pub fn queue_for_axis(i: usize) -> *mut StepQueue {
    if i >= N_AXIS_STEP_QUEUES {
        return ptr::null_mut();
    }
    // SAFETY: `i < N_AXIS_STEP_QUEUES` is checked above, and `step_queues` is
    // the C-declared array of exactly `N_AXIS_STEP_QUEUES` queues, so `add(i)`
    // stays in-bounds and yields a pointer to a live, aligned `StepQueue`.
    unsafe { step_queues.get().cast::<StepQueue>().add(i) }
}

/// Clear all per-axis step queues. MCU-only.
///
/// The caller (`runtime_reset`) holds the IRQ guard, so no producer
/// ISR or consumer timer runs concurrently with these writes.
#[cfg(not(any(test, feature = "host")))]
pub fn reset_all_queues() {
    for i in 0..N_AXIS_STEP_QUEUES {
        let q = queue_for_axis(i);
        // SAFETY: `i < N_AXIS_STEP_QUEUES` so `q` is non-null and points at a
        // live `StepQueue`; the IRQ guard guarantees exclusive access.
        unsafe {
            (*q).clear();
        }
    }
}

#[derive(Debug, PartialEq, Eq)]
pub struct StepQueueFull;

/// Push one entry into the ring.
///
/// # Safety
///
/// - `q` must be a non-null, properly aligned pointer to a live `StepQueue`
///   whose storage outlives this call.
/// - The caller must be the *sole* producer for `q`; calling `push` from
///   two threads / cores / ISRs against the same queue is UB. The
///   consumer (`pop` / `peek`) is allowed to run concurrently on the
///   opposite core.
pub unsafe fn push(q: *mut StepQueue, entry: StepEntry) -> Result<(), StepQueueFull> {
    let tail = unsafe { ptr::read_volatile(&(*q).tail) };
    let head = unsafe { ptr::read_volatile(&(*q).head) };
    if tail.wrapping_sub(head) >= STEP_QUEUE_DEPTH as u16 {
        return Err(StepQueueFull);
    }
    let slot = (tail & STEP_QUEUE_DEPTH_MASK) as usize;
    unsafe {
        #[allow(clippy::indexing_slicing)]
        ptr::write_volatile(&mut (*q).buf[slot], entry);
    }
    fence(Ordering::Release);
    unsafe { ptr::write_volatile(&mut (*q).tail, tail.wrapping_add(1)) };
    Ok(())
}

/// Pop the oldest entry from the ring, or return `None` if empty.
///
/// # Safety
///
/// - `q` must be a non-null, properly aligned pointer to a live `StepQueue`
///   whose storage outlives this call.
/// - The caller must be the *sole* consumer for `q`; calling `pop` from
///   two threads / cores / ISRs against the same queue is UB. The
///   producer (`push`) is allowed to run concurrently on the opposite core.
pub unsafe fn pop(q: *mut StepQueue) -> Option<StepEntry> {
    let tail = unsafe { ptr::read_volatile(&(*q).tail) };
    let head = unsafe { ptr::read_volatile(&(*q).head) };
    if tail == head {
        return None;
    }
    fence(Ordering::Acquire);
    let slot = (head & STEP_QUEUE_DEPTH_MASK) as usize;
    let entry = unsafe {
        #[allow(clippy::indexing_slicing)]
        ptr::read_volatile(&(*q).buf[slot])
    };
    fence(Ordering::Release);
    unsafe { ptr::write_volatile(&mut (*q).head, head.wrapping_add(1)) };
    Some(entry)
}

/// Look at the oldest entry without consuming it.
///
/// # Safety
///
/// Same constraints as [`pop`] — `q` must be live and the caller must be
/// the sole consumer.
pub unsafe fn peek(q: *mut StepQueue) -> Option<StepEntry> {
    let tail = unsafe { ptr::read_volatile(&(*q).tail) };
    let head = unsafe { ptr::read_volatile(&(*q).head) };
    if tail == head {
        return None;
    }
    fence(Ordering::Acquire);
    let slot = (head & STEP_QUEUE_DEPTH_MASK) as usize;
    Some(unsafe {
        #[allow(clippy::indexing_slicing)]
        ptr::read_volatile(&(*q).buf[slot])
    })
}

/// Current populated length. Racy by design — both endpoints may read
/// this for monitoring without coordination.
///
/// # Safety
///
/// - `q` must be a non-null, properly aligned pointer to a live `StepQueue`
///   whose storage outlives this call.
/// - Safe to call from any context; does not advance counters and cannot
///   violate SPSC discipline.
pub unsafe fn len(q: *mut StepQueue) -> u16 {
    let tail = unsafe { ptr::read_volatile(&(*q).tail) };
    let head = unsafe { ptr::read_volatile(&(*q).head) };
    tail.wrapping_sub(head)
}

#[cfg(test)]
mod tests;
