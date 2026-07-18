//! Readiness signal from the engine to the host reactor: a nonblocking
//! socketpair whose read end the host parks on instead of polling the
//! submit/fence APIs. Space wakeups are edge-triggered — a submit that found
//! the input channel full arms the signal, and the ingress fires it when it
//! next frees a slot. Fence wakeups fire unconditionally on every
//! resolution. Spurious bytes are harmless: the host re-polls whatever it
//! was waiting on and parks again.

use std::io::Write;
use std::os::fd::AsRawFd;
use std::os::unix::net::UnixStream;
use std::sync::atomic::{AtomicBool, Ordering};

pub struct FeedWakeup {
    armed: AtomicBool,
    tx: UnixStream,
    rx: UnixStream,
}

impl std::fmt::Debug for FeedWakeup {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("FeedWakeup")
            .field("armed", &self.armed.load(Ordering::Relaxed))
            .field("read_fd", &self.rx.as_raw_fd())
            .finish()
    }
}

impl Default for FeedWakeup {
    fn default() -> Self {
        let (tx, rx) = UnixStream::pair().expect("feed wakeup socketpair");
        tx.set_nonblocking(true)
            .expect("feed wakeup tx nonblocking");
        rx.set_nonblocking(true)
            .expect("feed wakeup rx nonblocking");
        Self {
            armed: AtomicBool::new(false),
            tx,
            rx,
        }
    }
}

impl FeedWakeup {
    /// A submit found the input channel full and its caller is about to park.
    pub fn arm(&self) {
        self.armed.store(true, Ordering::Release);
    }

    /// The ingress freed an input-channel slot: wake the parked submitter,
    /// if one armed. Stays silent otherwise so an idle stream writes nothing.
    pub fn notify_space_freed(&self) {
        if self.armed.swap(false, Ordering::AcqRel) {
            self.ping();
        }
    }

    /// A fence resolved: wake the host so it re-polls its pending fences.
    pub fn notify_fence_resolved(&self) {
        self.ping();
    }

    fn ping(&self) {
        if let Err(e) = (&self.tx).write(&[1]) {
            // WouldBlock means bytes are already pending — readiness is
            // level-triggered on the host side, nothing is lost.
            if e.kind() != std::io::ErrorKind::WouldBlock {
                tracing::warn!(
                    subsystem = "motion",
                    event = "feed_wakeup_ping_failed",
                    error = %e,
                    "feed wakeup write failed; host falls back to its timeout poll"
                );
            }
        }
    }

    /// Fd the host parks on. Owned by this struct — the host must not close it.
    pub fn read_fd(&self) -> i32 {
        self.rx.as_raw_fd()
    }
}

#[cfg(test)]
mod tests;
