use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use crate::transport::TransportError;

/// Admission control for bulk fire-and-forget traffic.
///
/// `queued` is the reactor's own gauge: blocks it has taken off the submission
/// channel and parked because the sequence window is full. `reserved` is
/// capacity claimed by senders whose batch has been handed to the channel but
/// not yet processed by the reactor. The gate reads their sum, so concurrent
/// senders cannot each observe a stale `queued` and collectively overshoot
/// [`FIRE_AND_FORGET_HIGH_WATER`].
///
/// The reactor publishes the new queue depth before releasing a batch's
/// reservation, so the sum never dips below the real occupancy: a rejected
/// sender is never rejected for capacity that has already been freed twice
/// over, and an admitted sender never slips through a transient trough.
#[derive(Debug, Default)]
pub struct FireAndForgetDepth {
    queued: AtomicUsize,
    reserved: AtomicUsize,
    closed: AtomicBool,
}

pub const FIRE_AND_FORGET_HIGH_WATER: usize = 256;

impl FireAndForgetDepth {
    pub(crate) fn publish(&self, queued_blocks: usize) {
        if self.closed.load(Ordering::Acquire) {
            return;
        }
        self.queued.store(queued_blocks, Ordering::Release);
    }

    /// Claim `blocks` of admission capacity for a burst about to be handed to
    /// the reactor. `blocks` is the burst's upper bound on queued blocks —
    /// packing can only ever produce fewer.
    pub(crate) fn reserve(&self, blocks: usize) -> Result<(), TransportError> {
        if self.closed.load(Ordering::Acquire) {
            return Err(TransportError::Closed);
        }
        let mut reserved = self.reserved.load(Ordering::Acquire);
        loop {
            if self.queued.load(Ordering::Acquire) + reserved >= FIRE_AND_FORGET_HIGH_WATER {
                return Err(TransportError::Backpressure);
            }
            match self.reserved.compare_exchange_weak(
                reserved,
                reserved + blocks,
                Ordering::AcqRel,
                Ordering::Acquire,
            ) {
                Ok(_) => break,
                Err(observed) => reserved = observed,
            }
        }
        if self.closed.load(Ordering::Acquire) {
            self.release(blocks);
            return Err(TransportError::Closed);
        }
        Ok(())
    }

    /// Give back a reservation once its blocks are queued, written, or dropped.
    pub(crate) fn release(&self, blocks: usize) {
        if self.closed.load(Ordering::Acquire) {
            return;
        }
        if self
            .reserved
            .fetch_update(Ordering::AcqRel, Ordering::Acquire, |reserved| {
                reserved.checked_sub(blocks)
            })
            .is_err()
        {
            tracing::error!(
                subsystem = "mcu-comms",
                event = "fire_and_forget_reservation_underflow",
                blocks,
                reserved = self.reserved.load(Ordering::Acquire),
                "released more fire-and-forget capacity than was reserved"
            );
        }
    }

    /// Zero both gauges and refuse every later reservation with
    /// [`TransportError::Closed`].
    pub(crate) fn close(&self) {
        self.closed.store(true, Ordering::Release);
        self.reserved.store(0, Ordering::Release);
        self.queued.store(0, Ordering::Release);
    }

    pub fn queued_blocks(&self) -> usize {
        self.queued.load(Ordering::Acquire) + self.reserved.load(Ordering::Acquire)
    }

    pub fn at_high_water(&self) -> bool {
        self.queued_blocks() >= FIRE_AND_FORGET_HIGH_WATER
    }
}
