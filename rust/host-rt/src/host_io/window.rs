use std::collections::VecDeque;
use std::sync::mpsc::SyncSender;
use std::time::Instant;

use crate::transport::{MessageParams, TransportError};

pub const MAX_PENDING_BLOCKS: usize = 12;

#[derive(Debug)]
pub struct UnackedEntry {
    pub seq: u64,
    pub frame_bytes: Vec<u8>,
    pub sent_at: Instant,
    pub retry_count: u32,
}

pub type UnackedWindow = VecDeque<UnackedEntry>;

pub fn is_full(window: &UnackedWindow) -> bool {
    window.len() >= MAX_PENDING_BLOCKS
}

pub fn pop_acked(window: &mut UnackedWindow, rseq: u64) -> Vec<UnackedEntry> {
    let mut popped = Vec::new();
    while window.front().is_some_and(|f| f.seq < rseq) {
        popped.push(window.pop_front().expect("front checked above"));
    }
    popped
}

#[derive(Debug)]
pub struct AwaitEntry {
    pub call_id: u64,
    pub seq: u64,
    pub expected_response_name: String,
    pub completion: SyncSender<Result<MessageParams, TransportError>>,
    pub submitted_at: Instant,
    pub deadline: Instant,
    pub abandoned: bool,
    /// CLOCK_MONOTONIC_RAW seconds captured just before the frame was written to wire.
    /// Zero when not measured.
    pub sent_time_raw: f64,
}

#[derive(Debug, Default)]
pub struct AwaitingResponse {
    entries: VecDeque<AwaitEntry>,
}

const AWAITING_DEFENSIVE_CEILING: usize = 1024;

impl AwaitingResponse {
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn push(&mut self, entry: AwaitEntry) -> Result<(), TransportError> {
        if self.entries.len() >= AWAITING_DEFENSIVE_CEILING {
            return Err(TransportError::Parse(
                "AwaitingResponse defensive ceiling exceeded".into(),
            ));
        }
        self.entries.push_back(entry);
        Ok(())
    }

    pub fn find_match(&self, name: &str) -> Option<usize> {
        self.entries
            .iter()
            .position(|e| !e.abandoned && e.expected_response_name == name)
    }

    pub fn remove(&mut self, idx: usize) -> AwaitEntry {
        self.entries.remove(idx).expect("idx valid")
    }

    pub fn mark_abandoned(&mut self, call_id: u64) {
        if let Some(e) = self.entries.iter_mut().find(|e| e.call_id == call_id) {
            e.abandoned = true;
        }
    }

    pub fn evict_expired(&mut self, now: Instant) -> Vec<AwaitEntry> {
        let mut evicted = Vec::new();
        let mut idx = 0;
        while idx < self.entries.len() {
            if now >= self.entries[idx].deadline {
                evicted.push(self.entries.remove(idx).unwrap());
            } else {
                idx += 1;
            }
        }
        evicted
    }

    pub fn drain_all(&mut self) -> Vec<AwaitEntry> {
        std::mem::take(&mut self.entries).into_iter().collect()
    }

    pub fn iter(&self) -> impl Iterator<Item = &AwaitEntry> {
        self.entries.iter()
    }
}

#[cfg(test)]
mod tests;

#[cfg(test)]
mod awaiting_tests;
