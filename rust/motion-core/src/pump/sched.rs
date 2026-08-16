use super::{AxisKey, MAX_LEAD_SECS};
use runtime::piece_ring::PieceEntry;
use std::collections::{BTreeMap, BTreeSet, VecDeque};

#[derive(Debug)]
pub struct AxisQueue {
    pub pieces: VecDeque<(PieceEntry, f64)>,
    pub pushed: u32,
    pub consumed: u32,
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
            consumed: 0,
            retired: 0,
            ring_depth,
            physical_write_cursor: 0,
            lead_secs: MAX_LEAD_SECS,
            staged_motion: 0,
            wire_hold_tail: 0,
        }
    }
    pub fn room(&self) -> u32 {
        let in_flight = self.pushed.wrapping_sub(self.consumed);
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
    /// Undo `advance_write_cursor(n)` for a bundle the MCU refused without
    /// advancing its head (endpoint halt): the next write must land on the
    /// slot the MCU still expects or the contiguity guard rejects it.
    pub fn rewind_write_cursor(&mut self, n: u32) {
        if self.ring_depth == 0 {
            return;
        }
        self.physical_write_cursor =
            (self.physical_write_cursor + self.ring_depth - n % self.ring_depth) % self.ring_depth;
    }
}

// Merged holds keep f32 `duration` rounding of `end_time` far inside the
// walker's 200 µs start-in-past budget (ulp(30 s) ≈ 3.8 µs).
pub const MAX_MERGED_HOLD_SECS: f64 = 30.0;

// Consecutive segments project to abutting ticks; anything wider than this is
// a genuine gap (dwell, stream restart) and must stay a separate piece.
const HOLD_MERGE_SEAM_SLOP_SECS: f64 = 2e-6;

// What the mcu piece walker tolerates when a piece starts before the clock it
// is handed to. A transport that ships pieces to the walker untouched has no
// host-side seam projector, so this is the only bound on a merged duration.
const WIRE_WALKER_START_SLOP_SECS: f64 = 200e-6;

/// How the consumer of a piece seam turns `duration` back into clock ticks.
/// A merge rewrites the tail's `duration` from a tick span, and both the
/// span-to-seconds and the seconds-to-ticks halves of that round trip must
/// use `freq` or the merged piece reprojects somewhere the following piece
/// does not start. `skew_budget_cycles` bounds what is left after the round
/// trip: `duration` is an f32, so past ~2^24 ticks the reprojection is
/// quantized coarser than a seam check can accept and the merge is refused.
#[derive(Debug, Clone, Copy)]
pub struct SeamBasis {
    pub freq: f64,
    pub skew_budget_cycles: u64,
}

impl SeamBasis {
    /// The basis for a transport that hands pieces straight to the mcu walker.
    #[must_use]
    pub fn wire_walker(freq: f64) -> Self {
        #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
        Self {
            freq,
            skew_budget_cycles: (freq * WIRE_WALKER_START_SLOP_SECS) as u64,
        }
    }
}

fn try_extend_hold(last: &mut PieceEntry, next: &PieceEntry, basis: SeamBasis) -> bool {
    let same_hold = last.coeff_count == 1
        && next.coeff_count == 1
        && last.motor_mask == next.motor_mask
        && last.coeffs[0].to_bits() == next.coeffs[0].to_bits();
    if !same_hold {
        return false;
    }
    #[allow(clippy::cast_possible_truncation)]
    let freq32 = basis.freq as f32;
    let last_end = last.end_time(freq32);
    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    let slop = (basis.freq * HOLD_MERGE_SEAM_SLOP_SECS) as u64;
    if last_end.abs_diff(next.start_time) > slop {
        return false;
    }
    #[allow(clippy::cast_precision_loss)]
    let merged_secs = next.start_time.saturating_sub(last.start_time) as f64 / basis.freq
        + f64::from(next.duration);
    if merged_secs > MAX_MERGED_HOLD_SECS {
        return false;
    }
    #[allow(clippy::cast_possible_truncation)]
    let merged_duration = merged_secs as f32;
    let merged_end = PieceEntry {
        duration: merged_duration,
        ..*last
    }
    .end_time(freq32);
    if merged_end.abs_diff(next.end_time(freq32)) > basis.skew_budget_cycles {
        return false;
    }
    last.duration = merged_duration;
    true
}

/// Append `pieces`, coalescing runs of bit-identical constant (hold) pieces
/// with the queue tail — a stationary axis otherwise ships one 20-byte wire
/// entry per planner segment. `allow_tail_merge=false` fences the first
/// incoming piece from a pre-existing tail (fresh stream re-anchor).
pub fn append_pieces_merging_holds(
    queue: &mut VecDeque<(PieceEntry, f64)>,
    pieces: Vec<(PieceEntry, f64)>,
    basis: SeamBasis,
    allow_tail_merge: bool,
) {
    let mut merge_with_tail = allow_tail_merge;
    for (piece, host) in pieces {
        let merged = merge_with_tail
            && queue
                .back_mut()
                .is_some_and(|(last, _)| try_extend_hold(last, &piece, basis));
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
    pub guard_recorded_ns: u64,
    pub guard_mcu_clock: u64,
}

#[derive(Debug)]
pub enum Schedule {
    Send(Vec<FramePlan>),
    StallFull(AxisKey),
    StallAhead(AxisKey),
    Idle,
}

#[must_use]
pub fn schedule(
    queues: &BTreeMap<AxisKey, AxisQueue>,
    limits_of: impl Fn(u32) -> super::BundleLimits,
    horizon_of: impl Fn(&AxisKey, &AxisQueue) -> Option<u64>,
    releasable_cap_of: impl Fn(&AxisKey) -> usize,
) -> Schedule {
    let mut stall_ahead_candidate: Option<AxisKey> = None;
    let mut cap_skipped: BTreeSet<AxisKey> = BTreeSet::new();
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
            if stall_ahead_candidate.is_none() {
                stall_ahead_candidate = Some(k);
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
                cap_skipped.insert(k);
                continue;
            }
        }

        break k;
    };

    let super::BundleLimits {
        wire_budget: bundle_wire_budget,
        pieces_per_axis,
    } = limits_of(head_key.mcu_id);
    let max_per_frame = pieces_per_axis.min(u8::MAX as usize);
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
    Schedule::Send(frames)
}
