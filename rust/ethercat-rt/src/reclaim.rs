//! Off-thread disposal of host-allocated command payloads. Freeing memory
//! that another thread allocated takes that thread's malloc-arena lock, so a
//! `drop` on the DC thread can block on a futex held by a preempted
//! normal-priority thread — the same priority inversion the rtrb record
//! rings exist to prevent. The DC thread hands consumed payloads to this
//! janitor through a wait-free ring; only if the ring is ever full does the
//! payload fall back to an inline drop.

use std::thread::JoinHandle;
use std::time::Duration;

use mcu_protocol::messages::PushSampleRuns;

use crate::thread_prio::demote_to_normal_scheduling;

/// Sample-run frames arrive at a few hundred hertz during a print and the
/// janitor drains every poll, so depth beyond a couple of poll intervals is
/// dead weight.
const RECLAIM_RING_CAPACITY: usize = 256;
const RECLAIM_POLL: Duration = Duration::from_millis(10);

pub struct Reclaim {
    tx: Option<rtrb::Producer<PushSampleRuns>>,
    service: Option<JoinHandle<()>>,
}

impl Reclaim {
    /// Spawn during bringup: under mlockall(MCL_FUTURE) a thread spawn
    /// prefaults and locks its stack, which is milliseconds — banned on the
    /// DC thread.
    #[must_use]
    pub fn spawn() -> Self {
        let (tx, mut rx) = rtrb::RingBuffer::new(RECLAIM_RING_CAPACITY);
        let service = std::thread::Builder::new()
            .name("ec-rt-reclaim".into())
            .spawn(move || {
                demote_to_normal_scheduling();
                loop {
                    while rx.pop().is_ok() {}
                    if rx.is_abandoned() && rx.is_empty() {
                        return;
                    }
                    std::thread::sleep(RECLAIM_POLL);
                }
            })
            .expect("spawn ec-rt-reclaim thread");
        Self {
            tx: Some(tx),
            service: Some(service),
        }
    }

    /// DC-thread side: wait-free. A full ring drops inline — rare, and the
    /// caller's `free_ns` span exposes it.
    pub fn dispose(&mut self, msg: PushSampleRuns) {
        if let Some(tx) = &mut self.tx {
            let _ = tx.push(msg);
        }
    }
}

impl Drop for Reclaim {
    fn drop(&mut self) {
        self.tx = None;
        if let Some(service) = self.service.take() {
            let _ = service.join();
        }
    }
}

#[cfg(test)]
mod tests;
