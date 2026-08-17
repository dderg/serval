use std::sync::atomic::{AtomicUsize, Ordering};

/// Blocks the reactor has taken off the submission channel and parked because
/// the sequence window is full, published so a bulk sender can stop feeding a
/// stalled link. The reactor itself never refuses a payload it has accepted —
/// a queue_step block that reached the reactor is always written in stream
/// order — so this gauge is the only place bulk traffic is throttled.
///
/// [`FIRE_AND_FORGET_HIGH_WATER`] is a watermark, not a cap: a sender that
/// passed the gate may still hand over a burst larger than the remaining
/// room, and the reactor queues all of it.
#[derive(Debug, Default)]
pub struct FireAndForgetDepth(AtomicUsize);

pub const FIRE_AND_FORGET_HIGH_WATER: usize = 256;

impl FireAndForgetDepth {
    pub(crate) fn publish(&self, queued_blocks: usize) {
        self.0.store(queued_blocks, Ordering::Release);
    }

    pub fn queued_blocks(&self) -> usize {
        self.0.load(Ordering::Acquire)
    }

    pub fn at_high_water(&self) -> bool {
        self.queued_blocks() >= FIRE_AND_FORGET_HIGH_WATER
    }
}
