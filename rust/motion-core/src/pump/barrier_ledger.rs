// Barrier receipts, shared by every mcu transport sink.
//
// A barrier is a numbered receipt the mcu returns once it has consumed
// everything queued ahead of it. Sequence numbers wrap, so ordering is
// modular: `barrier_seq_after` reads a difference as signed, and
// `barrier_seq_covers` treats an ack as covering every earlier receipt because
// the mcu acks in queue order.
//
// The seed is randomised per process so a host restart cannot have its fresh
// receipts covered by the acks the mcu still holds from the previous run.

use std::collections::{HashMap, VecDeque};

#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
pub struct BarrierId {
    pub oid: u32,
    pub seq: u32,
}

pub fn barrier_seq_seed() -> u32 {
    let elapsed = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .expect("system clock is before the Unix epoch");
    (elapsed.as_nanos() as u32) | 1
}

pub fn barrier_seq_after(candidate: u32, reference: u32) -> bool {
    let distance = candidate.wrapping_sub(reference);
    distance != 0 && distance < (1 << 31)
}

pub fn barrier_seq_before(candidate: u32, reference: u32) -> bool {
    barrier_seq_after(reference, candidate)
}

fn barrier_seq_covers(high_water: u32, seq: u32) -> bool {
    high_water == seq || barrier_seq_after(high_water, seq)
}

#[derive(Debug)]
struct SentBarrier {
    id: BarrierId,
    sent_clock: u64,
}

#[derive(Debug)]
struct ReceiptSequence {
    next: u32,
    acked: Option<u32>,
}

/// Issue, track and retire barrier receipts for one mcu's lanes.
#[derive(Debug)]
pub struct BarrierLedger {
    seed: u32,
    sequences: HashMap<u32, ReceiptSequence>,
    sent: VecDeque<SentBarrier>,
}

impl Default for BarrierLedger {
    fn default() -> Self {
        Self::new()
    }
}

impl BarrierLedger {
    pub fn new() -> Self {
        Self::with_seed(barrier_seq_seed())
    }

    pub fn with_seed(seed: u32) -> Self {
        Self {
            seed,
            sequences: HashMap::new(),
            sent: VecDeque::new(),
        }
    }

    pub fn with_capacity(lanes: usize) -> Self {
        let mut ledger = Self::new();
        ledger.sequences.reserve(lanes);
        ledger
    }

    pub fn issue(&mut self, oid: u32) -> BarrierId {
        let slot = self.sequences.entry(oid).or_insert(ReceiptSequence {
            next: self.seed,
            acked: None,
        });
        let seq = slot.next;
        slot.next = seq.wrapping_add(1);
        BarrierId { oid, seq }
    }

    pub fn is_acked(&self, id: BarrierId) -> bool {
        self.sequences
            .get(&id.oid)
            .and_then(|lane| lane.acked)
            .is_some_and(|high_water| barrier_seq_covers(high_water, id.seq))
    }

    /// Adopt an ack from the mcu. A receipt the host never issued, or one that
    /// walks the high-water mark backwards, means the two sides disagree about
    /// the stream — the caller escalates.
    pub fn record_ack(&mut self, oid: u32, seq: u32) -> Result<(), AckFault> {
        let lane = self.sequences.get_mut(&oid).ok_or(AckFault::Unknown)?;
        let issued = lane.next;
        if !barrier_seq_before(seq, issued) {
            return Err(AckFault::Unissued { issued });
        }
        match lane.acked {
            Some(high_water) if !barrier_seq_after(seq, high_water) => {
                return Err(AckFault::Regressed { high_water });
            }
            _ => {}
        }
        lane.acked = Some(seq);
        self.sent
            .retain(|entry| !barrier_seq_covers(seq, entry.id.seq) || entry.id.oid != oid);
        Ok(())
    }

    pub fn record_ordered_ack(&mut self, oid: u32, seq: u32) -> Result<bool, AckFault> {
        let lane = self.sequences.get_mut(&oid).ok_or(AckFault::Unknown)?;
        let issued = lane.next;
        let expected = lane.acked.map_or(self.seed, |seq| seq.wrapping_add(1));
        if barrier_seq_before(seq, expected) {
            return Ok(false);
        }
        if !barrier_seq_before(seq, issued) {
            return Err(AckFault::Unissued { issued });
        }
        if seq != expected {
            return Err(AckFault::OutOfOrder { expected });
        }
        lane.acked = Some(seq);
        self.prune_acked();
        Ok(true)
    }

    pub fn cancel(&mut self, id: BarrierId) {
        let lane = self
            .sequences
            .get_mut(&id.oid)
            .expect("only issued barriers can be cancelled");
        let acked = lane.acked.get_or_insert(id.seq);
        if barrier_seq_after(id.seq, *acked) {
            *acked = id.seq;
        }
    }

    pub fn forget_sent(&mut self, oid: u32) {
        self.sent.retain(|entry| entry.id.oid != oid);
    }

    pub fn clear_sent(&mut self) {
        self.sent.clear();
    }

    pub fn has_sent(&self) -> bool {
        !self.sent.is_empty()
    }

    pub fn sent_clock_of(&self, id: BarrierId) -> Option<u64> {
        self.sent
            .iter()
            .find(|entry| entry.id == id)
            .map(|entry| entry.sent_clock)
    }

    pub fn note_sent(&mut self, id: BarrierId, sent_clock: u64) {
        self.sent.push_back(SentBarrier { id, sent_clock });
    }

    pub fn prune_acked(&mut self) {
        let mut sent = std::mem::take(&mut self.sent);
        sent.retain(|entry| !self.is_acked(entry.id));
        self.sent = sent;
    }

    /// Receipts the mcu has owed for longer than `deadline_ticks`, measured on
    /// the mcu clock: a barrier that never comes back parks its lane forever,
    /// so the caller escalates instead of waiting.
    pub fn overdue(&self, now: u64, deadline_ticks: u64) -> Vec<(BarrierId, u64)> {
        self.sent
            .iter()
            .filter(|entry| entry.sent_clock.saturating_add(deadline_ticks) < now)
            .map(|entry| (entry.id, entry.sent_clock))
            .collect()
    }

    pub fn ledger_line(&self) -> String {
        let mut acked: Vec<(u32, u32)> = self
            .sequences
            .iter()
            .filter_map(|(&oid, lane)| lane.acked.map(|seq| (oid, seq)))
            .collect();
        acked.sort_unstable();
        let acked: Vec<String> = acked
            .into_iter()
            .map(|(oid, seq)| format!("oid {oid} acked {seq}"))
            .collect();
        if acked.is_empty() {
            return "no barrier acks recorded".to_string();
        }
        acked.join(", ")
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AckFault {
    Unknown,
    Unissued { issued: u32 },
    Regressed { high_water: u32 },
    OutOfOrder { expected: u32 },
}

#[cfg(test)]
#[path = "barrier_ledger_tests.rs"]
mod barrier_ledger_tests;
