use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};

/// Link vitals the reactor publishes every tick so failures elsewhere in the
/// stack (a stepcompress deficit, a barrier stall) can say whether the wire
/// itself was alive when they fired — a silent link otherwise masquerades as
/// a pipeline throughput deficit.
pub struct LinkHealth {
    reactor_now_ms: AtomicU64,
    last_inbound_ms: AtomicU64,
    unacked: AtomicU32,
    front_retry: AtomicU32,
}

impl std::fmt::Debug for LinkHealth {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.describe())
    }
}

impl Default for LinkHealth {
    fn default() -> Self {
        Self {
            reactor_now_ms: AtomicU64::new(0),
            last_inbound_ms: AtomicU64::new(0),
            unacked: AtomicU32::new(0),
            front_retry: AtomicU32::new(0),
        }
    }
}

impl LinkHealth {
    pub(crate) fn publish(
        &self,
        reactor_now_ms: u64,
        last_inbound_ms: u64,
        unacked: u32,
        front_retry: u32,
    ) {
        self.reactor_now_ms.store(reactor_now_ms, Ordering::Relaxed);
        self.last_inbound_ms
            .store(last_inbound_ms, Ordering::Relaxed);
        self.unacked.store(unacked, Ordering::Relaxed);
        self.front_retry.store(front_retry, Ordering::Relaxed);
    }

    pub fn silence_ms(&self) -> u64 {
        self.reactor_now_ms
            .load(Ordering::Relaxed)
            .saturating_sub(self.last_inbound_ms.load(Ordering::Relaxed))
    }

    pub fn unacked(&self) -> u32 {
        self.unacked.load(Ordering::Relaxed)
    }

    pub fn describe(&self) -> String {
        format!(
            "link: last inbound {} ms ago, {} unacked frames, front frame retransmitted {}x",
            self.silence_ms(),
            self.unacked(),
            self.front_retry.load(Ordering::Relaxed)
        )
    }
}
