use super::{AxisKey, MAX_LEAD_SECS};
use runtime::piece_ring::PieceEntry;
use std::collections::{BTreeMap, BTreeSet, VecDeque};

#[derive(Debug)]
pub struct AxisQueue {
    pub pieces: VecDeque<(PieceEntry, f64)>,
    pub pushed: u32,
    pub retired: u32,
    pub ring_depth: u32,
    pub physical_write_cursor: u32,
    pub lead_secs: f64,
    /// Staged pieces that carry motion (`!is_hold_piece`), maintained
    /// incrementally so the per-loop ledger publish never scans the queue.
    pub staged_motion: u32,
    /// Consecutive hold pieces at the pushed (wire) tail; any non-hold send
    /// resets it. Feeds the drain ledger's motion-only drained condition.
    pub wire_hold_tail: u32,
}

/// A constant-position piece: one coefficient, so zero velocity everywhere.
/// These are dwell / idle-blanket coverage, not motion.
pub fn is_hold_piece(p: &PieceEntry) -> bool {
    p.coeff_count == 1
}

impl AxisQueue {
    pub fn new(ring_depth: u32) -> Self {
        Self {
            pieces: VecDeque::new(),
            pushed: 0,
            retired: 0,
            ring_depth,
            physical_write_cursor: 0,
            lead_secs: MAX_LEAD_SECS,
            staged_motion: 0,
            wire_hold_tail: 0,
        }
    }
    pub fn room(&self) -> u32 {
        let in_flight = self.pushed.wrapping_sub(self.retired);
        if in_flight > self.ring_depth {
            self.ring_depth
        } else {
            self.ring_depth - in_flight
        }
    }
    pub fn advance_write_cursor(&mut self, n: u32) {
        if self.ring_depth == 0 {
            return;
        }
        self.physical_write_cursor = (self.physical_write_cursor + n) % self.ring_depth;
    }
}

// Merged holds keep f32 `duration` rounding of `end_time` far inside the
// walker's 200 µs start-in-past budget (ulp(30 s) ≈ 3.8 µs).
pub const MAX_MERGED_HOLD_SECS: f64 = 30.0;

// Consecutive segments project to abutting ticks; anything wider than this is
// a genuine gap (dwell, stream restart) and must stay a separate piece.
const HOLD_MERGE_SEAM_SLOP_SECS: f64 = 2e-6;

fn try_extend_hold(last: &mut PieceEntry, next: &PieceEntry, freq: f64) -> bool {
    let same_hold = last.coeff_count == 1
        && next.coeff_count == 1
        && last.motor_mask == next.motor_mask
        && last.coeffs[0].to_bits() == next.coeffs[0].to_bits();
    if !same_hold {
        return false;
    }
    #[allow(clippy::cast_possible_truncation)]
    let last_end = last.end_time(freq as f32);
    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    let slop = (freq * HOLD_MERGE_SEAM_SLOP_SECS) as u64;
    if last_end.abs_diff(next.start_time) > slop {
        return false;
    }
    #[allow(clippy::cast_precision_loss)]
    let merged_secs =
        next.start_time.saturating_sub(last.start_time) as f64 / freq + f64::from(next.duration);
    if merged_secs > MAX_MERGED_HOLD_SECS {
        return false;
    }
    #[allow(clippy::cast_possible_truncation)]
    {
        last.duration = merged_secs as f32;
    }
    true
}

/// Append `pieces`, coalescing runs of bit-identical constant (hold) pieces
/// with the queue tail — a stationary axis otherwise ships one 20-byte wire
/// entry per planner segment. Tick-anchored duration recomputation keeps the
/// merged end time drift-free. `allow_tail_merge=false` fences the first
/// incoming piece from a pre-existing tail (fresh stream re-anchor).
pub fn append_pieces_merging_holds(
    queue: &mut VecDeque<(PieceEntry, f64)>,
    pieces: Vec<(PieceEntry, f64)>,
    freq: f64,
    allow_tail_merge: bool,
) {
    let mut merge_with_tail = allow_tail_merge;
    for (piece, host) in pieces {
        let merged = merge_with_tail
            && queue
                .back_mut()
                .is_some_and(|(last, _)| try_extend_hold(last, &piece, freq));
        if !merged {
            queue.push_back((piece, host));
        }
        merge_with_tail = true;
    }
}

#[derive(Debug)]
pub struct FramePlan {
    pub key: AxisKey,
    pub pieces: Vec<PieceEntry>,
    pub start_slot: u16,
}

/// One axis' pieces within a single-MCU bundle, carrying the ring bookkeeping
/// the transport needs. `schedule()` only ever groups axes of one MCU into a
/// `Send`, so a slice of these is exactly the work for one MCU transaction.
pub struct AxisFrame {
    pub axis: u8,
    pub pieces: Vec<PieceEntry>,
    pub start_slot: u16,
    pub new_head: u32,
    pub room: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DeferredReason {
    RingFull,
    CapExhausted,
    Horizon,
}

#[derive(Debug)]
pub enum Schedule {
    Send(Vec<FramePlan>),
    SendDeferred(Vec<FramePlan>, AxisKey, DeferredReason),
    StallFull(AxisKey),
    StallAhead(AxisKey),
    Idle,
}

#[must_use]
pub fn schedule(
    queues: &BTreeMap<AxisKey, AxisQueue>,
    max_per_frame: usize,
    bundle_wire_budget: usize,
    horizon_of: impl Fn(&AxisKey, &AxisQueue) -> Option<u64>,
    releasable_cap_of: impl Fn(&AxisKey) -> usize,
) -> Schedule {
    let mut stall_ahead_candidate: Option<AxisKey> = None;
    let mut bypassed_ahead_candidate: Option<AxisKey> = None;
    let mut cap_skipped: BTreeSet<AxisKey> = BTreeSet::new();
    let mut cap_exhausted_candidate: Option<AxisKey> = None;
    let mut stall_full_candidate: Option<AxisKey> = None;

    let head_key = loop {
        let candidate = queues
            .iter()
            .filter(|(k, q)| !q.pieces.is_empty() && !cap_skipped.contains(*k))
            .min_by(|(ka, qa), (kb, qb)| {
                let host_a = qa.pieces.front().unwrap().1;
                let host_b = qb.pieces.front().unwrap().1;
                host_a.total_cmp(&host_b).then(ka.cmp(kb))
            });
        let (&k, q) = match candidate {
            None => {
                if let Some(k) = stall_full_candidate {
                    return Schedule::StallFull(k);
                }
                if let Some(k) = stall_ahead_candidate {
                    return Schedule::StallAhead(k);
                }
                if let Some(k) = cap_exhausted_candidate {
                    return Schedule::StallAhead(k);
                }
                return Schedule::Idle;
            }
            Some(c) => c,
        };

        if q.room() == 0 {
            if stall_full_candidate.is_none() {
                stall_full_candidate = Some(k);
            }
            cap_skipped.insert(k);
            continue;
        }

        if releasable_cap_of(&k) == 0 {
            if cap_exhausted_candidate.is_none() {
                cap_exhausted_candidate = Some(k);
            }
            cap_skipped.insert(k);
            continue;
        }

        let head_start_ticks = q.pieces.front().unwrap().0.start_time;
        if let Some(horizon) = horizon_of(&k, q) {
            if head_start_ticks > horizon {
                if stall_ahead_candidate.is_none() {
                    stall_ahead_candidate = Some(k);
                }
                if bypassed_ahead_candidate.is_none() {
                    bypassed_ahead_candidate = Some(k);
                }
                cap_skipped.insert(k);
                continue;
            }
        }

        break k;
    };

    let mut taken: BTreeMap<AxisKey, usize> = BTreeMap::new();
    let mut maxed: BTreeSet<AxisKey> = cap_skipped;
    let mut bundle_bytes = 0usize;
    loop {
        let next = queues
            .iter()
            .filter_map(|(k, q)| {
                if k.mcu_id != head_key.mcu_id || maxed.contains(k) {
                    return None;
                }
                let already = taken.get(k).copied().unwrap_or(0);
                q.pieces
                    .get(already)
                    .map(|&(ref p, host)| (*k, p.start_time, host, p.wire_len()))
            })
            .min_by(|(ka, _, ha, _), (kb, _, hb, _)| ha.total_cmp(hb).then(ka.cmp(kb)));
        let (k, start_ticks, _host, wire_len) = match next {
            Some(n) => n,
            None => break,
        };
        if !taken.is_empty() && bundle_bytes + wire_len > bundle_wire_budget {
            break;
        }
        let already = taken.get(&k).copied().unwrap_or(0);
        let q = &queues[&k];
        let room = q.room() as usize;
        let cap = releasable_cap_of(&k);
        if already >= room || already >= max_per_frame || already >= cap {
            maxed.insert(k);
            continue;
        }
        if let Some(horizon) = horizon_of(&k, q) {
            if start_ticks > horizon {
                if stall_ahead_candidate.is_none() {
                    stall_ahead_candidate = Some(k);
                }
                maxed.insert(k);
                continue;
            }
        }
        bundle_bytes += wire_len;
        *taken.entry(k).or_insert(0) += 1;
    }

    if taken.is_empty() {
        if let Some(k) = stall_ahead_candidate {
            return Schedule::StallAhead(k);
        }
        return Schedule::StallFull(head_key);
    }

    let frames: Vec<FramePlan> = taken
        .into_iter()
        .filter(|(_, n)| *n > 0)
        .map(|(k, n)| FramePlan {
            key: k,
            pieces: queues[&k].pieces.iter().take(n).map(|(p, _)| *p).collect(),
            start_slot: 0,
        })
        .collect();
    debug_assert!(!frames.is_empty());
    if let Some(key) = bypassed_ahead_candidate {
        Schedule::SendDeferred(frames, key, DeferredReason::Horizon)
    } else if let Some(key) = stall_full_candidate {
        Schedule::SendDeferred(frames, key, DeferredReason::RingFull)
    } else if let Some(key) = cap_exhausted_candidate {
        Schedule::SendDeferred(frames, key, DeferredReason::CapExhausted)
    } else {
        Schedule::Send(frames)
    }
}
