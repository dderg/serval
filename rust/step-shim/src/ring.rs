use std::collections::VecDeque;

use runtime::piece_ring::PieceEntry;

use crate::ShimError;

#[derive(Debug)]
pub struct PieceRing {
    entries: VecDeque<PieceEntry>,
    capacity: u32,
    pushed: u32,
    retired: u32,
}

impl PieceRing {
    pub fn new(capacity: u32) -> Self {
        Self {
            entries: VecDeque::with_capacity(capacity as usize),
            capacity,
            pushed: 0,
            retired: 0,
        }
    }

    pub fn push(&mut self, motor: usize, entry: PieceEntry) -> Result<(), ShimError> {
        if self.entries.len() as u32 >= self.capacity {
            return Err(ShimError::RingFull { motor });
        }
        self.entries.push_back(entry);
        self.pushed += 1;
        Ok(())
    }

    pub fn front(&self) -> Option<&PieceEntry> {
        self.entries.front()
    }

    /// Start of the piece after the front one. It is the authoritative end of
    /// the front piece whenever it lands before the duration-derived end: the
    /// two disagree by the host's clock-domain skew, and the MCU treats
    /// `start_time` as the truth.
    pub fn next_start(&self) -> Option<u64> {
        self.entries.get(1).map(|e| e.start_time)
    }

    pub fn retire_front(&mut self) {
        if self.entries.pop_front().is_some() {
            self.retired += 1;
        }
    }

    pub fn retire_all(&mut self) {
        self.entries.clear();
        self.retired = self.pushed;
    }

    pub fn pushed(&self) -> u32 {
        self.pushed
    }

    pub fn retired(&self) -> u32 {
        self.retired
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn capacity(&self) -> u32 {
        self.capacity
    }
}
