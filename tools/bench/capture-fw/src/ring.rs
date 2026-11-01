use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicUsize, Ordering};

pub const RING_TICKS: usize = 8192;

pub struct TickRing {
    slots: UnsafeCell<[u32; RING_TICKS]>,
    produced: AtomicUsize,
    consumed: AtomicUsize,
}

unsafe impl Sync for TickRing {}

pub struct RingFull;

impl TickRing {
    pub const fn new() -> Self {
        Self {
            slots: UnsafeCell::new([0; RING_TICKS]),
            produced: AtomicUsize::new(0),
            consumed: AtomicUsize::new(0),
        }
    }

    pub fn push(&self, tick: u32) -> Result<(), RingFull> {
        let produced = self.produced.load(Ordering::Relaxed);
        if produced - self.consumed.load(Ordering::Acquire) == RING_TICKS {
            return Err(RingFull);
        }
        unsafe { (*self.slots.get())[produced % RING_TICKS] = tick };
        self.produced.store(produced + 1, Ordering::Release);
        Ok(())
    }

    pub fn available(&self) -> usize {
        self.produced.load(Ordering::Acquire) - self.consumed.load(Ordering::Relaxed)
    }

    pub fn pop_into(&self, out: &mut [u8]) {
        let consumed = self.consumed.load(Ordering::Relaxed);
        let (chunks, _) = out.as_chunks_mut::<4>();
        for (i, chunk) in chunks.iter_mut().enumerate() {
            let tick = unsafe { (*self.slots.get())[(consumed + i) % RING_TICKS] };
            *chunk = tick.to_le_bytes();
        }
        self.consumed
            .store(consumed + out.len() / 4, Ordering::Release);
    }

    pub fn consumed(&self) -> usize {
        self.consumed.load(Ordering::Relaxed)
    }

    pub fn reset(&self) {
        self.produced.store(0, Ordering::Release);
        self.consumed.store(0, Ordering::Release);
    }
}
