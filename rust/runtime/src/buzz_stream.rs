#![allow(unsafe_code)]

use core::sync::atomic::Ordering;

use portable_atomic::AtomicI32;

use crate::buzz_gen::{ToneCursor, ToneError, ToneParams, next_crossing};
use crate::error::FaultCode;
use crate::step_queue::{
    N_AXIS_STEP_QUEUES, STEPPER_SEL_ALL, StepEntry, StepQueue, len as queue_len,
    peek as queue_peek, push as queue_push,
};

static REFILL_FAULT: AtomicI32 = AtomicI32::new(0);

pub fn latch_refill_fault(err: RefillError) {
    let code = match err {
        RefillError::QueueFull => FaultCode::StepQueueOverflow,
        RefillError::Solver(e) => e.fault_code().unwrap_or(FaultCode::InternalInvariant),
    };
    REFILL_FAULT.store(code.as_i32(), Ordering::Release);
}

#[must_use]
pub fn take_refill_fault() -> i32 {
    REFILL_FAULT.swap(0, Ordering::AcqRel)
}

const HIGH_WATER: u16 = 24;

const REFILL_BATCH: u16 = 16;

#[derive(Clone, Copy, Debug)]
struct AxisStream {
    params: ToneParams,
    cursor: ToneCursor,
    anchored: bool,
    closed: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RefillError {
    Solver(ToneError),
    QueueFull,
}

#[cfg(not(any(test, feature = "host")))]
mod store {
    use super::AxisStream;
    use crate::step_queue::N_AXIS_STEP_QUEUES;
    use core::cell::UnsafeCell;

    pub(super) struct Streams(UnsafeCell<[Option<AxisStream>; N_AXIS_STEP_QUEUES]>);
    unsafe impl Sync for Streams {}

    pub(super) static STREAMS: Streams =
        Streams(UnsafeCell::new([const { None }; N_AXIS_STEP_QUEUES]));

    pub(super) fn with_slot<R>(axis_idx: usize, f: impl FnOnce(&mut Option<AxisStream>) -> R) -> R {
        let slots = unsafe { &mut *STREAMS.0.get() };
        #[allow(clippy::indexing_slicing)]
        f(&mut slots[axis_idx])
    }
}

#[cfg(any(test, feature = "host"))]
mod store {
    use super::AxisStream;
    use crate::step_queue::N_AXIS_STEP_QUEUES;
    use core::cell::RefCell;

    std::thread_local! {
        static STREAMS: RefCell<[Option<AxisStream>; N_AXIS_STEP_QUEUES]> =
            const { RefCell::new([None; N_AXIS_STEP_QUEUES]) };
    }

    pub(super) fn with_slot<R>(axis_idx: usize, f: impl FnOnce(&mut Option<AxisStream>) -> R) -> R {
        STREAMS.with(|s| {
            let mut slots = s.borrow_mut();
            #[allow(clippy::indexing_slicing)]
            f(&mut slots[axis_idx])
        })
    }

    pub fn reset_for_test() {
        for i in 0..N_AXIS_STEP_QUEUES {
            super::clear_axis(i);
        }
    }
}

#[cfg(any(test, feature = "host"))]
pub use store::reset_for_test;

#[cfg(any(test, feature = "host"))]
#[must_use]
pub fn test_high_water() -> u16 {
    HIGH_WATER
}

pub fn arm_axis(axis_idx: usize, params: ToneParams) {
    if axis_idx >= N_AXIS_STEP_QUEUES {
        return;
    }
    store::with_slot(axis_idx, |slot| {
        *slot = Some(AxisStream {
            params,
            cursor: ToneCursor::start(),
            anchored: false,
            closed: false,
        });
    });
}

pub fn clear_axis(axis_idx: usize) {
    if axis_idx >= N_AXIS_STEP_QUEUES {
        return;
    }
    store::with_slot(axis_idx, |slot| *slot = None);
}

#[must_use]
pub fn axis_active(axis_idx: usize) -> bool {
    if axis_idx >= N_AXIS_STEP_QUEUES {
        return false;
    }
    store::with_slot(axis_idx, |slot| slot.as_ref().is_some_and(|s| !s.closed))
}

pub unsafe fn refill_step_axis(
    axis_idx: usize,
    queue_ptr: *mut StepQueue,
    now: u32,
) -> Result<bool, RefillError> {
    if axis_idx >= N_AXIS_STEP_QUEUES || queue_ptr.is_null() {
        return Ok(false);
    }
    store::with_slot(axis_idx, |slot| {
        let Some(stream) = slot.as_mut() else {
            return Ok(false);
        };
        if stream.closed {
            return Ok(false);
        }
        if !stream.anchored {
            stream.params.anchor_cycle = now;
            stream.anchored = true;
        }
        let mut pushed: u16 = 0;
        loop {
            let occupancy = unsafe { queue_len(queue_ptr) };
            if occupancy >= HIGH_WATER || pushed >= REFILL_BATCH {
                return Ok(pushed > 0);
            }
            match next_crossing(&stream.params, stream.cursor) {
                Ok(crossing) => {
                    let entry = StepEntry {
                        cycle_abs: crossing.cycle_abs,
                        dir: crossing.dir,
                        stepper_sel: STEPPER_SEL_ALL,
                        _pad: [0; 2],
                    };
                    if unsafe { queue_push(queue_ptr, entry) }.is_err() {
                        return Err(RefillError::QueueFull);
                    }
                    stream.cursor = ToneCursor {
                        level: crossing.level,
                        t_cursor: crossing.t,
                    };
                    pushed += 1;
                }
                Err(ToneError::Done) => {
                    stream.closed = true;
                    return Ok(pushed > 0);
                }
                Err(other) => return Err(RefillError::Solver(other)),
            }
        }
    })
}

pub unsafe fn refill_foreground_all(
    now: u32,
    resolve: impl Fn(usize) -> *mut StepQueue,
    kick: impl Fn(usize, u32),
) {
    for axis_idx in 0..N_AXIS_STEP_QUEUES {
        if !axis_active(axis_idx) {
            continue;
        }
        let q = resolve(axis_idx);
        if q.is_null() {
            continue;
        }
        let was_empty = unsafe { queue_len(q) } == 0;
        match unsafe { refill_step_axis(axis_idx, q, now) } {
            Ok(pushed) => {
                if was_empty && pushed {
                    if let Some(front) = unsafe { queue_peek(q) } {
                        kick(axis_idx, front.cycle_abs);
                    }
                }
            }
            Err(err) => latch_refill_fault(err),
        }
    }
}

#[cfg(not(any(test, feature = "host")))]
#[unsafe(no_mangle)]
pub extern "C" fn runtime_buzz_refill_foreground() {
    let now = unsafe { timer_read_time() };
    unsafe {
        refill_foreground_all(
            now,
            crate::step_queue::queue_for_axis,
            crate::dispatch_stepper::kick_per_axis_timer_foreground,
        );
    }
}

#[cfg(not(any(test, feature = "host")))]
unsafe extern "C" {
    fn timer_read_time() -> u32;
}

#[cfg(test)]
mod tests;
