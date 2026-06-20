// Exact-crossing buzz streaming: the producer half of the resonance excitation.
//
// `arm` (foreground, via the engine) latches one immutable `ToneParams` per
// excited axis into the per-axis stream slot, anchored on the current MCU cycle,
// then kicks the step-output timer. `refill_step_axis` runs inside the
// step-output consumer ISR (see `per_axis_timer::step_output_event`): it drives
// `buzz_gen::next_crossing` and pushes `StepEntry`s into the axis ring up to a
// high-water mark, always keeping the next crossing queued so the timer stays
// armed across the carrier's turnaround gaps. All ring pushes therefore happen
// in the consumer context — producer and consumer share one NVIC priority, so
// the SPSC ring never sees a racing producer.
//
// The stream is decoupled from the motion ISR tick rate: a crossing's
// `cycle_abs` depends only on the curve, the anchor, and `cycles_per_second`.

#![allow(unsafe_code)]

use core::sync::atomic::{AtomicI32, Ordering};

use crate::buzz_gen::{ToneCursor, ToneError, ToneParams, next_crossing};
use crate::error::FaultCode;
use crate::step_queue::{
    N_AXIS_STEP_QUEUES, STEPPER_SEL_ALL, StepEntry, StepQueue, len as queue_len, push as queue_push,
};

/// Latched refill fault code (0 == none). The consumer ISR has no `SharedState`
/// handle, so a refill fault is recorded here for the foreground to surface via
/// `take_refill_fault`. Both refill errors are "cannot happen" by construction
/// (sole producer below `HIGH_WATER`; solver guards monotonicity) — latching
/// rather than dropping edges keeps the failure loud.
static REFILL_FAULT: AtomicI32 = AtomicI32::new(0);

pub fn latch_refill_fault(err: RefillError) {
    let code = match err {
        RefillError::QueueFull => FaultCode::StepQueueOverflow,
        RefillError::Solver(e) => e.fault_code().unwrap_or(FaultCode::InternalInvariant),
    };
    REFILL_FAULT.store(code.as_i32(), Ordering::Release);
}

/// Take and clear any latched refill fault (0 == none).
#[must_use]
pub fn take_refill_fault() -> i32 {
    REFILL_FAULT.swap(0, Ordering::AcqRel)
}

/// Keep the ring comfortably below `STEP_QUEUE_DEPTH` (32). Refill tops up to
/// this on each consumer pass; the gap to depth absorbs the in-flight pops.
const HIGH_WATER: u16 = 8;

/// Bound on pushes per `refill_step_axis` call so a single consumer pass cannot
/// monopolise the ISR generating a long run of crossings. The solver runs in
/// the step-output ISR, which competes with the 10 kHz motion timer (~100 us
/// period), so this is kept small: a few f32 crossings stay well inside one
/// motion tick. The queue refills incrementally across consumer passes.
const REFILL_BATCH: u16 = 4;

/// One axis's resumable excitation: the latched curve and the cursor threaded
/// through `next_crossing`. `anchored` flips on the first refill, where the
/// anchor cycle is bound to the consumer's `now` so the first crossing is never
/// already in the past; thereafter `cycle_abs` depends only on curve + anchor +
/// `cycles_per_second`. `closed` flips once the final net-zero edge has been
/// emitted, after which the slot stays inert until re-armed.
#[derive(Clone, Copy, Debug)]
struct AxisStream {
    params: ToneParams,
    cursor: ToneCursor,
    anchored: bool,
    closed: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RefillError {
    /// The solver faulted (non-monotonic time or a diverged refinement). The
    /// caller raises `InternalInvariant` — never silently recovered.
    Solver(ToneError),
    /// A ring push failed. The stream is the sole producer and tops up only to
    /// `HIGH_WATER < STEP_QUEUE_DEPTH`, so this is impossible; surfaced so the
    /// consumer can raise `StepQueueOverflow` rather than drop edges.
    QueueFull,
}

#[cfg(not(any(test, feature = "host")))]
mod store {
    use super::AxisStream;
    use crate::step_queue::N_AXIS_STEP_QUEUES;
    use core::cell::UnsafeCell;

    // Single-core MCU: `arm` (foreground) and refill (step-output ISR) never run
    // concurrently against the same slot — arm only writes a slot before kicking
    // the timer, and the kicked ISR is the only reader/mutator thereafter. No
    // shared-memory placement here, so this stays Rust-owned (boundary rule:
    // Rust owns the motion engine's private state).
    //
    // One slot per serviceable StepQueue: a buzz stream is the sole producer for
    // exactly one StepQueue, and the consumer ISR only refills axes 0..N_AXIS,
    // so a stream on an axis without a queue could never be drained.
    pub(super) struct Streams(UnsafeCell<[Option<AxisStream>; N_AXIS_STEP_QUEUES]>);
    // SAFETY: access is serialised by the single-core same-priority discipline
    // described above; never touched from two contexts at once.
    unsafe impl Sync for Streams {}

    pub(super) static STREAMS: Streams =
        Streams(UnsafeCell::new([const { None }; N_AXIS_STEP_QUEUES]));

    pub(super) fn with_slot<R>(axis_idx: usize, f: impl FnOnce(&mut Option<AxisStream>) -> R) -> R {
        // SAFETY: `axis_idx < N_AXIS_STEP_QUEUES` is enforced by callers.
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

/// Latch one axis's excitation curve and arm its stream. Called from the
/// foreground arm path with the curve already expressed in absolute seconds.
/// `params.anchor_cycle` is provisional: the first refill rebinds it to the
/// consumer's `now`. Replaces any prior stream on the axis.
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

/// Drop any stream on the axis. Used to disarm and by the test reset.
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

/// Push pending crossings for one axis up to `HIGH_WATER`, bounded by
/// `REFILL_BATCH`. Runs in the consumer ISR (sole producer there). Keeps the
/// next crossing queued whenever the stream is live so the timer re-arms across
/// turnaround gaps; emits the closing net-zero edge then marks the stream done.
///
/// # Safety
///
/// `queue_ptr` must be a non-null, live `StepQueue` for `axis_idx`, and the
/// caller must be the sole producer for it (the step-output ISR).
pub unsafe fn refill_step_axis(
    axis_idx: usize,
    queue_ptr: *mut StepQueue,
    now: u32,
) -> Result<(), RefillError> {
    if axis_idx >= N_AXIS_STEP_QUEUES || queue_ptr.is_null() {
        return Ok(());
    }
    store::with_slot(axis_idx, |slot| {
        let Some(stream) = slot.as_mut() else {
            return Ok(());
        };
        if stream.closed {
            return Ok(());
        }
        if !stream.anchored {
            stream.params.anchor_cycle = now;
            stream.anchored = true;
        }
        let mut pushed: u16 = 0;
        loop {
            // SAFETY: caller guarantees `queue_ptr` is live; `len` is racy-safe.
            let occupancy = unsafe { queue_len(queue_ptr) };
            if occupancy >= HIGH_WATER || pushed >= REFILL_BATCH {
                return Ok(());
            }
            match next_crossing(&stream.params, stream.cursor) {
                Ok(crossing) => {
                    let entry = StepEntry {
                        cycle_abs: crossing.cycle_abs,
                        dir: crossing.dir,
                        stepper_sel: STEPPER_SEL_ALL,
                        _pad: [0; 2],
                    };
                    // SAFETY: sole producer (consumer ISR); `queue_ptr` live.
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
                    return Ok(());
                }
                Err(other) => return Err(RefillError::Solver(other)),
            }
        }
    })
}

#[cfg(test)]
mod tests;
