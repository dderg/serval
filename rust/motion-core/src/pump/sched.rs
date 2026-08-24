use super::messages::RetiredBy;
use super::{AxisKey, MAX_LEAD_SECS};
use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::sync::Arc;
use trajectory::{
    ClockedMotorSpan, ContinuousAxis, MAX_SPAN_SECS, MotorGroup, MotorSpan, MotorTerm,
};

/// One reporting transport's odometers for one axis.
#[derive(Debug, Default, Clone, Copy)]
struct WireCredit {
    consumed: u32,
    retired: u32,
}

#[derive(Debug)]
pub struct AxisQueue {
    pub spans: VecDeque<ClockedMotorSpan>,
    pub pushed: u32,
    /// `consumed` and `retired` are the axis totals: the sum of `credits`,
    /// recomputed on every report. `pushed` counts the views the pump handed
    /// to whichever transport owned the axis at the time, so only the sum of
    /// every transport's credit is comparable with it.
    pub consumed: u32,
    pub retired: u32,
    credits: [WireCredit; RetiredBy::COUNT],
    pub ring_depth: u32,
    pub lead_secs: f64,
    /// Staged views that carry motion (`!is_hold_span`), maintained
    /// incrementally so the per-loop ledger publish never scans the queue.
    pub staged_motion: u32,
    /// Consecutive hold views at the pushed (wire) tail; any non-hold send
    /// resets it. Feeds the drain ledger's motion-only drained condition.
    pub wire_hold_tail: u32,
    pub wire_end_clock: Option<u64>,
    /// Projected MCU-clock end of the last enqueued view and whether that
    /// view parked the lane at rest. A later enqueue whose first view
    /// starts past this by more than the rejoin floor is a lane-local hole
    /// (single-lane nudge traffic advanced the stream while this lane sat
    /// out); the pump sanctions it as a forward seam gap iff the lane was
    /// at rest.
    pub seam_end_clock: Option<u64>,
    pub seam_end_at_rest: bool,
}

/// A view whose every contributing term holds one position: dwell / idle-blanket
/// coverage, not motion.
#[must_use]
pub fn is_hold_span(span: &ClockedMotorSpan) -> bool {
    span.signal.is_explicit_hold
}

/// A lane-local forward hole wider than this is a genuine sat-out gap
/// (single-lane nudge traffic), not seam skew: legitimate seam reprojection
/// error is span-scaled and stays in the microseconds.
pub const LANE_REJOIN_GAP_FLOOR_SECS: f64 = 1e-3;

/// A lane whose last view ends slower than this parked at rest.
pub const LANE_REJOIN_REST_VEL_MM_S: f64 = 1e-3;

#[must_use]
pub fn span_ends_at_rest(span: &ClockedMotorSpan) -> bool {
    is_hold_span(span)
        || span
            .signal
            .eval_pva(span.stream_t_end)
            .is_ok_and(|pva| pva.velocity.abs() <= LANE_REJOIN_REST_VEL_MM_S)
}

impl AxisQueue {
    pub fn new(ring_depth: u32) -> Self {
        Self {
            spans: VecDeque::new(),
            pushed: 0,
            consumed: 0,
            retired: 0,
            ring_depth,
            lead_secs: MAX_LEAD_SECS,
            staged_motion: 0,
            wire_hold_tail: 0,
            wire_end_clock: None,
            seam_end_clock: None,
            seam_end_at_rest: false,
            credits: [WireCredit::default(); RetiredBy::COUNT],
        }
    }

    /// Record one transport's absolute odometers for this axis and refresh the
    /// axis totals.
    pub fn credit(&mut self, by: RetiredBy, consumed: u32, retired: u32) {
        self.credits[by as usize] = WireCredit { consumed, retired };
        self.consumed = self
            .credits
            .iter()
            .fold(0, |sum, c| sum.wrapping_add(c.consumed));
        self.retired = self
            .credits
            .iter()
            .fold(0, |sum, c| sum.wrapping_add(c.retired));
    }
    pub fn room(&self) -> u32 {
        let in_flight = self.pushed.wrapping_sub(self.consumed);
        if in_flight > self.ring_depth {
            self.ring_depth
        } else {
            self.ring_depth - in_flight
        }
    }
}

fn hold_position(span: &ClockedMotorSpan) -> Option<f64> {
    span.signal
        .eval_pva(span.stream_t_start)
        .ok()
        .map(|pva| pva.position)
}

fn merged_hold_span(last: &ClockedMotorSpan, next: &ClockedMotorSpan) -> Option<ClockedMotorSpan> {
    if !last.signal.is_explicit_hold || !next.signal.is_explicit_hold {
        return None;
    }
    if last.signal.motor_mask != next.signal.motor_mask
        || last.clock_freq_hz != next.clock_freq_hz
        || last.end_clock != next.start_clock
    {
        return None;
    }
    let position = hold_position(last)?;
    if position.to_bits() != hold_position(next)?.to_bits() {
        return None;
    }
    let merged_secs =
        (last.stream_t_end - last.stream_t_start) + (next.stream_t_end - next.stream_t_start);
    if merged_secs > MAX_SPAN_SECS {
        return None;
    }
    let t_start = last.stream_t_start;
    let t_end = t_start + merged_secs;
    let groups: Arc<[MotorGroup]> = Arc::from(vec![MotorGroup::Independent(MotorTerm {
        source_axis: last.signal.first_source_axis(),
        axis: ContinuousAxis::Hold {
            position,
            t_start,
            t_end,
        },
        scale: 1.0,
    })]);
    let signal = MotorSpan::try_new(
        groups,
        t_start,
        t_end,
        last.signal.motor_mask,
        last.signal.source_line,
        true,
    )
    .ok()?;
    let merged = ClockedMotorSpan::try_new(
        Arc::new(signal),
        t_start,
        t_end,
        last.start_host,
        next.end_host,
        last.start_clock_exact,
        last.clock_freq_hz,
    )
    .ok()?;
    (merged.start_clock == last.start_clock && merged.end_clock == next.end_clock).then_some(merged)
}

/// Append `spans`, coalescing runs of abutting bit-identical hold views with
/// the queue tail — a stationary axis otherwise ships one wire entry per
/// planner segment. `allow_tail_merge=false` fences the first incoming view
/// from a pre-existing tail (fresh stream re-anchor).
pub fn append_spans_merging_holds(
    queue: &mut VecDeque<ClockedMotorSpan>,
    spans: Vec<ClockedMotorSpan>,
    allow_tail_merge: bool,
) {
    let mut merge_with_tail = allow_tail_merge;
    for span in spans {
        let merged = if merge_with_tail {
            queue.back().and_then(|last| merged_hold_span(last, &span))
        } else {
            None
        };
        match merged {
            Some(merged) => *queue.back_mut().expect("a merge requires a tail") = merged,
            None => queue.push_back(span),
        }
        merge_with_tail = true;
    }
}

#[derive(Debug)]
pub struct FramePlan {
    pub key: AxisKey,
    pub spans: Vec<ClockedMotorSpan>,
}

/// One axis' views within a single-MCU bundle, carrying the wire bookkeeping
/// the transport needs. `schedule()` only ever groups axes of one MCU into a
/// `Send`, so a slice of these is exactly the work for one MCU transaction.
pub struct AxisFrame {
    pub axis: u8,
    pub spans: Vec<ClockedMotorSpan>,
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
            .filter(|(k, q)| !q.spans.is_empty() && !cap_skipped.contains(*k))
            .min_by(|(ka, qa), (kb, qb)| {
                let host_a = qa.spans.front().unwrap().start_host;
                let host_b = qb.spans.front().unwrap().start_host;
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

        let head_start_clock = q.spans.front().unwrap().start_clock;
        if let Some(horizon) = horizon_of(&k, q) {
            if head_start_clock > horizon {
                if stall_ahead_candidate.is_none() {
                    stall_ahead_candidate = Some(k);
                }
                cap_skipped.insert(k);
                continue;
            }
        }

        break k;
    };

    let super::BundleLimits { spans_per_axis } = limits_of(head_key.mcu_id);
    let max_per_frame = spans_per_axis.min(u8::MAX as usize);
    let mut taken: BTreeMap<AxisKey, usize> = BTreeMap::new();
    let mut maxed: BTreeSet<AxisKey> = cap_skipped;
    loop {
        let next = queues
            .iter()
            .filter_map(|(k, q)| {
                if k.mcu_id != head_key.mcu_id || maxed.contains(k) {
                    return None;
                }
                let already = taken.get(k).copied().unwrap_or(0);
                q.spans
                    .get(already)
                    .map(|span| (*k, span.start_clock, span.start_host))
            })
            .min_by(|(ka, _, ha), (kb, _, hb)| ha.total_cmp(hb).then(ka.cmp(kb)));
        let (k, start_clock, _host) = match next {
            Some(n) => n,
            None => break,
        };
        let already = taken.get(&k).copied().unwrap_or(0);
        let q = &queues[&k];
        let room = q.room() as usize;
        let cap = releasable_cap_of(&k);
        if already >= room || already >= max_per_frame || already >= cap {
            maxed.insert(k);
            continue;
        }
        if let Some(horizon) = horizon_of(&k, q) {
            if start_clock > horizon {
                if stall_ahead_candidate.is_none() {
                    stall_ahead_candidate = Some(k);
                }
                maxed.insert(k);
                continue;
            }
        }
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
            spans: queues[&k].spans.iter().take(n).cloned().collect(),
        })
        .collect();
    debug_assert!(!frames.is_empty());
    Schedule::Send(frames)
}
