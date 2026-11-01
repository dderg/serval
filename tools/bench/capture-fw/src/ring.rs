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
        let (chunks, remainder) = out.as_chunks_mut::<4>();
        assert!(remainder.is_empty());
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

pub const BYTE_RING_LEN: usize = 48 * 1024;

pub struct ByteRing {
    bytes: UnsafeCell<[u8; BYTE_RING_LEN]>,
    produced: AtomicUsize,
    consumed: AtomicUsize,
}

unsafe impl Sync for ByteRing {}

pub struct ByteWriter<'a> {
    ring: &'a ByteRing,
    position: usize,
}

impl ByteRing {
    pub const fn new() -> Self {
        Self {
            bytes: UnsafeCell::new([0; BYTE_RING_LEN]),
            produced: AtomicUsize::new(0),
            consumed: AtomicUsize::new(0),
        }
    }

    pub fn free(&self) -> usize {
        BYTE_RING_LEN
            - (self.produced.load(Ordering::Relaxed) - self.consumed.load(Ordering::Acquire))
    }

    pub fn writer(&self) -> ByteWriter<'_> {
        ByteWriter {
            ring: self,
            position: self.produced.load(Ordering::Relaxed),
        }
    }

    pub fn available(&self) -> usize {
        self.produced.load(Ordering::Acquire) - self.consumed.load(Ordering::Relaxed)
    }

    pub fn pop_into(&self, out: &mut [u8]) {
        let consumed = self.consumed.load(Ordering::Relaxed);
        for (i, byte) in out.iter_mut().enumerate() {
            *byte = unsafe { (*self.bytes.get())[(consumed + i) % BYTE_RING_LEN] };
        }
        self.consumed.store(consumed + out.len(), Ordering::Release);
    }

    pub fn consumed(&self) -> usize {
        self.consumed.load(Ordering::Relaxed)
    }

    pub fn reset(&self) {
        self.produced.store(0, Ordering::Release);
        self.consumed.store(0, Ordering::Release);
    }
}

impl ByteWriter<'_> {
    pub fn put(&mut self, byte: u8) {
        unsafe { (*self.ring.bytes.get())[self.position % BYTE_RING_LEN] = byte };
        self.position += 1;
    }

    pub fn put_slice(&mut self, bytes: &[u8]) {
        for &byte in bytes {
            self.put(byte);
        }
    }

    pub fn put_varint(&mut self, mut value: u64) {
        while value >= 0x80 {
            self.put(value as u8 | 0x80);
            value >>= 7;
        }
        self.put(value as u8);
    }

    pub fn put_zigzag(&mut self, value: i64) {
        self.put_varint(((value << 1) ^ (value >> 63)) as u64);
    }

    pub fn commit(self) {
        self.ring.produced.store(self.position, Ordering::Release);
    }
}

pub const DIR_QUEUE_LEN: usize = 64;

pub struct DirQueue {
    entries: UnsafeCell<[u64; DIR_QUEUE_LEN]>,
    produced: AtomicUsize,
    consumed: AtomicUsize,
}

unsafe impl Sync for DirQueue {}

impl DirQueue {
    pub const fn new() -> Self {
        Self {
            entries: UnsafeCell::new([0; DIR_QUEUE_LEN]),
            produced: AtomicUsize::new(0),
            consumed: AtomicUsize::new(0),
        }
    }

    pub fn push(&self, entry: u64) -> Result<(), RingFull> {
        let produced = self.produced.load(Ordering::Relaxed);
        if produced - self.consumed.load(Ordering::Acquire) == DIR_QUEUE_LEN {
            return Err(RingFull);
        }
        unsafe { (*self.entries.get())[produced % DIR_QUEUE_LEN] = entry };
        self.produced.store(produced + 1, Ordering::Release);
        Ok(())
    }

    pub fn produced(&self) -> usize {
        self.produced.load(Ordering::Acquire)
    }

    pub fn consumed(&self) -> usize {
        self.consumed.load(Ordering::Relaxed)
    }

    pub fn entry(&self, index: usize) -> u64 {
        unsafe { (*self.entries.get())[index % DIR_QUEUE_LEN] }
    }

    pub fn release_through(&self, index: usize) {
        self.consumed.store(index, Ordering::Release);
    }

    pub fn reset(&self) {
        self.produced.store(0, Ordering::Release);
        self.consumed.store(0, Ordering::Release);
    }
}
