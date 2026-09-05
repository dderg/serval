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

pub fn barrier_seq_covers(high_water: u32, seq: u32) -> bool {
    high_water == seq || barrier_seq_after(high_water, seq)
}

#[derive(Debug)]
struct SentBarrier {
    id: BarrierId,
    sent_clock: u64,
}

/// Issue, track and retire barrier receipts for one mcu's lanes.
#[derive(Debug)]
pub struct BarrierLedger {
    seed: u32,
    next_seq: HashMap<u32, u32>,
    acked_seq: HashMap<u32, u32>,
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
            next_seq: HashMap::new(),
            acked_seq: HashMap::new(),
            sent: VecDeque::new(),
        }
    }

    pub fn issue(&mut self, oid: u32) -> BarrierId {
        let seed = self.seed;
        let slot = self.next_seq.entry(oid).or_insert(seed);
        let seq = *slot;
        *slot = seq.wrapping_add(1);
        BarrierId { oid, seq }
    }

    pub fn is_acked(&self, id: BarrierId) -> bool {
        self.acked_seq
            .get(&id.oid)
            .is_some_and(|&high_water| barrier_seq_covers(high_water, id.seq))
    }

    /// Adopt an ack from the mcu. A receipt the host never issued, or one that
    /// walks the high-water mark backwards, means the two sides disagree about
    /// the stream — the caller escalates.
    pub fn record_ack(&mut self, oid: u32, seq: u32) -> Result<(), AckFault> {
        let issued = self.next_seq.get(&oid).copied().ok_or(AckFault::Unknown)?;
        if !barrier_seq_before(seq, issued) {
            return Err(AckFault::Unissued { issued });
        }
        match self.acked_seq.get(&oid).copied() {
            Some(high_water) if !barrier_seq_after(seq, high_water) => {
                return Err(AckFault::Regressed { high_water });
            }
            _ => {}
        }
        self.acked_seq.insert(oid, seq);
        self.sent
            .retain(|entry| !barrier_seq_covers(seq, entry.id.seq) || entry.id.oid != oid);
        Ok(())
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
        let mut acked: Vec<(u32, u32)> = self.acked_seq.iter().map(|(&k, &v)| (k, v)).collect();
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
}

#[cfg(test)]
#[path = "barrier_ledger_tests.rs"]
mod barrier_ledger_tests;
