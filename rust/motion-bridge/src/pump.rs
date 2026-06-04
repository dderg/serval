//! Host-side piece pump: merges per-(mcu,axis) piece queues by absolute
//! start_time and streams PushPieces frames in strict time order with
//! per-ring flow control. See
//! `docs/superpowers/specs/2026-05-28-push-pieces-wiring-design.md`.

use runtime::piece_ring::PieceEntry;
use std::collections::{BTreeMap, BTreeSet, HashMap, VecDeque};
use std::sync::Weak;
use std::sync::mpsc::{Receiver, RecvError, RecvTimeoutError};
use std::time::Duration;

/// Destination ring identity.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug, PartialOrd, Ord)]
pub struct AxisKey {
    pub mcu_id: u32,
    pub axis: u8,
}

/// One axis's outbound queue plus flow-control accounting. `pushed` and
/// `retired` are wrapping u32 mirrors of the MCU's monotonic ring counter
/// (spec §3.3) — never reset on a time re-anchor, only on an MCU ring reset.
///
/// `physical_write_cursor` tracks the MCU ring slot that will receive the
/// next piece. It is advanced incrementally modulo `ring_depth` on each
/// successful send — it is NEVER derived as `pushed % ring_depth`, which
/// would produce wrong results across u32 wrap of `pushed`.
#[derive(Debug)]
pub struct AxisQueue {
    pub pieces: VecDeque<PieceEntry>,
    pub pushed: u32,
    pub retired: u32,
    pub ring_depth: u32,
    /// Physical ring write cursor: the slot index in `[0, ring_depth)` where
    /// the next batch of pieces will be written on the MCU side. Advanced
    /// incrementally (mod ring_depth) on each ACKed send — never reset on
    /// time re-anchor, only on an MCU ring reset.
    pub physical_write_cursor: u32,
    /// Projected MCU-tick end of the last piece from the previous Enqueue for
    /// this axis. Used by the Face-A overlap diagnostic to detect non-monotonic
    /// junctions between consecutive Enqueue batches.
    pub last_enqueued_end: Option<u64>,
}

impl AxisQueue {
    pub fn new(ring_depth: u32) -> Self {
        Self {
            pieces: VecDeque::new(),
            pushed: 0,
            retired: 0,
            ring_depth,
            physical_write_cursor: 0,
            last_enqueued_end: None,
        }
    }
    /// Free ring slots = depth − in-flight, where in-flight = pushed − retired
    /// (wrapping). Saturates at 0.
    pub fn room(&self) -> u32 {
        let in_flight = self.pushed.wrapping_sub(self.retired);
        self.ring_depth.saturating_sub(in_flight)
    }
    /// Advance the physical write cursor by `n` slots, wrapping at `ring_depth`.
    /// No-op when `ring_depth == 0` (degenerate / uninitialised ring).
    pub fn advance_write_cursor(&mut self, n: u32) {
        if self.ring_depth == 0 {
            return;
        }
        // cursor < ring_depth ≤ 65535, n ≤ 255 → sum ≤ 65789 < u32::MAX; no overflow.
        self.physical_write_cursor = (self.physical_write_cursor + n) % self.ring_depth;
    }
}

/// A planned outbound frame: one axis's contiguous run of pieces.
///
/// `start_slot` is filled in by the send loop (just before sending) from
/// `AxisQueue::physical_write_cursor` — the scheduler does not have mutable
/// queue access at plan time, so this field is set to 0 at construction and
/// overwritten at send time.
#[derive(Debug)]
pub struct FramePlan {
    pub key: AxisKey,
    pub pieces: Vec<PieceEntry>,
    /// Physical ring slot where this frame's first piece will land on the MCU.
    /// Set to 0 at schedule time; overwritten with `q.physical_write_cursor`
    /// just before the send call in `run_pump`.
    pub start_slot: u16,
}

/// Outcome of one scheduling decision.
#[derive(Debug)]
pub enum Schedule {
    /// Send these frames (all on one MCU, a contiguous prefix of global order).
    Send(Vec<FramePlan>),
    /// Global head's ring is full — wait for a heartbeat. Do not send anything.
    StallFull(AxisKey),
    /// Global head has ring room but its start_time exceeds the MCU's current
    /// commit-lead horizon. Wait for the MCU clock to advance. The contained
    /// key is the earliest-start non-empty axis that has room but is
    /// time-gated.
    StallAhead(AxisKey),
    /// No pieces queued anywhere.
    Idle,
}

/// Decide the next action over the queue map. Does **not** mutate the queues;
/// the caller applies the returned plan (pops pieces, bumps `pushed`).
/// `max_per_frame` caps a single PushPieces frame's piece_count (u8 wire field).
///
/// `horizon_of(mcu_id)` returns the MCU-tick deadline beyond which a piece's
/// `start_time` must not be committed this pass. `None` means "not synced
/// yet — apply no time gate for this MCU" (count-only, same as pre-sync
/// behaviour).
///
/// Scheduling is performed **independently per MCU**. Raw `start_time` ticks
/// are only compared within a single MCU's clock domain — cross-MCU tick
/// comparison is meaningless (H7 runs at 520 MHz with its own epoch, F446 at
/// 180 MHz with a different epoch). A single global head that happens to be a
/// numerically-small F446 tick would otherwise starve the H7's ready work via
/// a premature `StallAhead`/`StallFull` early-return; this is the bench-proven
/// root cause of the Face-B -308 shutdown.
///
/// Merge rule: if ANY MCU produced frames → `Send(all frames)`. Else if any
/// MCU is ahead-stalled → `StallAhead` (first by mcu_id). Else if any MCU is
/// full-stalled → `StallFull` (first by mcu_id). Else `Idle`.
///
/// When one MCU is ahead-stalled and another is full-stalled with no frames,
/// `StallAhead` is returned (not `StallFull`) — the 50 ms poll covers both
/// conditions, whereas blocking on recv() would re-introduce starvation for
/// the ahead-gated MCU.
#[must_use]
pub fn schedule(
    queues: &BTreeMap<AxisKey, AxisQueue>,
    max_per_frame: usize,
    horizon_of: impl Fn(u32) -> Option<u64>,
) -> Schedule {
    // Collect the distinct MCU ids that have at least one non-empty queue.
    let active_mcus: Vec<u32> = {
        let mut ids: Vec<u32> = queues
            .keys()
            .filter(|k| !queues[k].pieces.is_empty())
            .map(|k| k.mcu_id)
            .collect();
        ids.dedup(); // BTreeMap is sorted by (mcu_id, axis) so duplicates are adjacent
        ids
    };

    if active_mcus.is_empty() {
        return Schedule::Idle;
    }

    let mut all_frames: Vec<FramePlan> = Vec::new();
    let mut first_stall_ahead: Option<AxisKey> = None;
    let mut first_stall_full: Option<AxisKey> = None;

    for mcu_id in active_mcus {
        // Pick this MCU's head: the non-empty queue with the smallest
        // start_time (raw ticks are valid within one clock domain); break
        // ties by AxisKey ordering for determinism.
        let mcu_head = queues
            .iter()
            .filter(|(k, q)| k.mcu_id == mcu_id && !q.pieces.is_empty())
            .min_by(|(ka, qa), (kb, qb)| {
                qa.pieces
                    .front()
                    .unwrap()
                    .start_time
                    .cmp(&qb.pieces.front().unwrap().start_time)
                    .then(ka.cmp(kb))
            });
        let (&head_key, head_q) = match mcu_head {
            None => continue,
            Some(h) => h,
        };
        let head_start = head_q.pieces.front().unwrap().start_time;

        // Ring-full check for this MCU's head.
        if head_q.room() == 0 {
            if first_stall_full.is_none() {
                first_stall_full = Some(head_key);
            }
            continue;
        }

        // Time-gate check for this MCU's head.
        if let Some(horizon) = horizon_of(mcu_id) {
            if head_start > horizon {
                if first_stall_ahead.is_none() {
                    first_stall_ahead = Some(head_key);
                }
                continue;
            }
        }

        // Greedily take the contiguous prefix of this MCU's time order that
        // has room and passes the time gate. `maxed` tracks axes saturated
        // this pass to avoid re-selecting them (which would infinite-loop).
        let mut taken: BTreeMap<AxisKey, usize> = BTreeMap::new();
        let mut maxed: BTreeSet<AxisKey> = BTreeSet::new();
        let mut mcu_stall_ahead: Option<AxisKey> = None;
        loop {
            let next = queues
                .iter()
                .filter_map(|(k, q)| {
                    if k.mcu_id != mcu_id || maxed.contains(k) {
                        return None;
                    }
                    let already = taken.get(k).copied().unwrap_or(0);
                    q.pieces.get(already).map(|p| (*k, p.start_time))
                })
                .min_by(|(ka, sa), (kb, sb)| sa.cmp(sb).then(ka.cmp(kb)));
            let (k, start) = match next {
                Some(n) => n,
                None => break,
            };
            let already = taken.get(&k).copied().unwrap_or(0);
            let q = &queues[&k];
            let room = q.room() as usize;
            if already >= room || already >= max_per_frame {
                maxed.insert(k);
                continue;
            }
            if let Some(horizon) = horizon_of(mcu_id) {
                if start > horizon {
                    if mcu_stall_ahead.is_none() {
                        mcu_stall_ahead = Some(k);
                    }
                    maxed.insert(k);
                    continue;
                }
            }
            *taken.entry(k).or_insert(0) += 1;
        }

        if taken.is_empty() {
            // Nothing sendable this pass for this MCU.
            if let Some(k) = mcu_stall_ahead {
                if first_stall_ahead.is_none() {
                    first_stall_ahead = Some(k);
                }
            } else {
                // Head had room but the batch loop found nothing — head_q.room()
                // was > 0 above, so this path means every axis became maxed by
                // count; fall back to StallFull for this MCU.
                if first_stall_full.is_none() {
                    first_stall_full = Some(head_key);
                }
            }
            continue;
        }

        for (k, n) in taken {
            if n > 0 {
                all_frames.push(FramePlan {
                    key: k,
                    pieces: queues[&k].pieces.iter().take(n).copied().collect(),
                    start_slot: 0,
                });
            }
        }
    }

    // Merge: frames from any MCU → Send. Prefer StallAhead over StallFull when
    // no frames were produced (50 ms poll covers both; blocking recv would
    // re-introduce starvation for the ahead-gated MCU).
    if !all_frames.is_empty() {
        return Schedule::Send(all_frames);
    }
    if let Some(k) = first_stall_ahead {
        return Schedule::StallAhead(k);
    }
    if let Some(k) = first_stall_full {
        return Schedule::StallFull(k);
    }
    Schedule::Idle
}

#[cfg(test)]
mod sched_tests {
    use super::*;

    fn q_with(ring_depth: u32, starts: &[u64]) -> AxisQueue {
        let mut q = AxisQueue::new(ring_depth);
        for &s in starts {
            q.pieces.push_back(PieceEntry {
                start_time: s,
                coeffs: [0.0; 4],
                duration: 0.001,
                _reserved: 0,
            });
        }
        q
    }

    #[test]
    fn idle_when_empty() {
        let queues: BTreeMap<AxisKey, AxisQueue> = BTreeMap::new();
        assert!(matches!(schedule(&queues, 255, |_| None), Schedule::Idle));
    }

    #[test]
    fn stalls_when_global_head_ring_full() {
        // Single-MCU fixture: the only non-empty queue is full → StallFull.
        let mut queues = BTreeMap::new();
        let mut a = q_with(2, &[10]);
        a.pushed = 2; // full
        queues.insert(AxisKey { mcu_id: 1, axis: 0 }, a);
        assert!(matches!(
            schedule(&queues, 255, |_| None),
            Schedule::StallFull(AxisKey { mcu_id: 1, axis: 0 })
        ));
    }

    #[test]
    fn batches_per_mcu_independently() {
        // Two MCUs whose time ranges overlap in global order. Under per-MCU
        // scheduling each MCU is batched independently: mcu1 gets all of its
        // ready pieces regardless of mcu2's time position, and vice versa.
        //
        // mcu1: A/x has pieces at t=0 and t=3; A/y has a piece at t=1.
        // mcu2: B/x has a piece at t=2.
        //
        // Per-MCU result: mcu1 sends A/x×2 + A/y×1; mcu2 sends B/x×1.
        // (The old global-order loop stopped mcu1's batch at B/x@2, so A/x
        // only got [0]. That was the cross-MCU coupling bug; the new behavior
        // is correct — B/x@2 is on a different clock domain and irrelevant.)
        let mut queues = BTreeMap::new();
        queues.insert(AxisKey { mcu_id: 1, axis: 0 }, q_with(8, &[0, 3]));
        queues.insert(AxisKey { mcu_id: 1, axis: 1 }, q_with(8, &[1]));
        queues.insert(AxisKey { mcu_id: 2, axis: 0 }, q_with(8, &[2]));
        match schedule(&queues, 255, |_| None) {
            Schedule::Send(frames) => {
                let ax: Vec<_> = frames.iter().map(|f| (f.key, f.pieces.len())).collect();
                // mcu1/axis0 gets both pieces (t=0 and t=3).
                assert!(ax.contains(&(AxisKey { mcu_id: 1, axis: 0 }, 2)));
                // mcu1/axis1 gets its one piece.
                assert!(ax.contains(&(AxisKey { mcu_id: 1, axis: 1 }, 1)));
                // mcu2/axis0 also gets its piece — independent MCU.
                assert!(ax.contains(&(AxisKey { mcu_id: 2, axis: 0 }, 1)));
            }
            other => panic!("expected Send, got {other:?}"),
        }
    }

    #[test]
    fn frame_cap_splits() {
        let mut queues = BTreeMap::new();
        queues.insert(AxisKey { mcu_id: 1, axis: 0 }, q_with(8, &[0, 1, 2, 3]));
        let s = schedule(&queues, 2, |_| None);
        match s {
            Schedule::Send(frames) => {
                assert_eq!(frames.len(), 1);
                assert_eq!(frames[0].pieces.len(), 2); // capped at 2 this pass
            }
            other => panic!("expected Send, got {other:?}"),
        }
    }

    #[test]
    fn full_axis_does_not_block_same_mcu_sibling() {
        // Scenario: MCU 1, two axes. Y is the global head (Y@0 < X@1).
        // X has depth 1 and pushed==1 (room==0). Y has depth 8 and room.
        // The full X should be excluded from the batch, not cause an early
        // loop exit — Y's pieces must still be planned this pass.
        let mut q: BTreeMap<AxisKey, AxisQueue> = BTreeMap::new();
        let yq = q_with(8, &[0, 2]); // Y: global head, has room
        let mut xq = q_with(1, &[1]); // X: depth 1, one piece at t=1
        xq.pushed = 1; // room == 0 (full)
        q.insert(AxisKey { mcu_id: 1, axis: 1 }, yq);
        q.insert(AxisKey { mcu_id: 1, axis: 0 }, xq);
        // Y@0 is the global head and has room → top-level StallFull does NOT fire.
        match schedule(&q, 255, |_| None) {
            Schedule::Send(frames) => {
                // Y must be batched despite full sibling X.
                let yf = frames
                    .iter()
                    .find(|f| f.key == AxisKey { mcu_id: 1, axis: 1 });
                assert!(yf.is_some(), "Y should be batched despite full sibling X");
                // X is full → contributes nothing to this batch.
                assert!(
                    !frames
                        .iter()
                        .any(|f| f.key == AxisKey { mcu_id: 1, axis: 0 }),
                    "full X must not appear in the batch"
                );
            }
            other => panic!("expected Send, got {other:?}"),
        }
    }

    #[test]
    fn time_gate_blocks_piece_beyond_horizon() {
        // Two axes on MCU 1. Head (axis 0) has start_time=100 within horizon=150.
        // Axis 1 has start_time=200 beyond the horizon → only axis 0 is planned.
        let mut queues = BTreeMap::new();
        queues.insert(AxisKey { mcu_id: 1, axis: 0 }, q_with(8, &[100]));
        queues.insert(AxisKey { mcu_id: 1, axis: 1 }, q_with(8, &[200]));
        // horizon = 150: axis 0 passes (100 <= 150), axis 1 blocked (200 > 150).
        match schedule(&queues, 255, |_| Some(150)) {
            Schedule::Send(frames) => {
                assert_eq!(frames.len(), 1, "only axis 0 should be batched");
                assert_eq!(frames[0].key, AxisKey { mcu_id: 1, axis: 0 });
                assert_eq!(frames[0].pieces.len(), 1);
            }
            other => panic!("expected Send, got {other:?}"),
        }
    }

    #[test]
    fn all_beyond_horizon_returns_stall_ahead() {
        // Single-MCU fixture: sole piece beyond horizon, ring has room → StallAhead.
        let mut queues = BTreeMap::new();
        queues.insert(AxisKey { mcu_id: 1, axis: 0 }, q_with(8, &[1000]));
        assert!(
            matches!(
                schedule(&queues, 255, |_| Some(500)),
                Schedule::StallAhead(AxisKey { mcu_id: 1, axis: 0 })
            ),
            "expected StallAhead when sole piece is beyond horizon"
        );
    }

    #[test]
    fn no_horizon_none_uses_count_only_gate() {
        // When horizon_of returns None, no time gate applies even if the piece
        // start_time is arbitrarily large. Verifies startup-before-clock-sync path.
        let mut queues = BTreeMap::new();
        queues.insert(AxisKey { mcu_id: 1, axis: 0 }, q_with(8, &[u64::MAX]));
        match schedule(&queues, 255, |_| None) {
            Schedule::Send(frames) => {
                assert_eq!(frames.len(), 1);
                assert_eq!(frames[0].pieces.len(), 1);
            }
            other => panic!("expected Send (no time gate), got {other:?}"),
        }
    }

    #[test]
    fn face_b_regression_ahead_stalled_mcu_does_not_starve_ready_mcu() {
        // Regression: Face-B bench starvation. mcu1 (F446) has one piece at a
        // far-future tick (numerically small in the 180 MHz domain but beyond
        // its horizon); mcu0 (H7) has pieces within its horizon and ring room.
        //
        // Old global-head logic: F446 tick 50 < H7 tick 10_000_000 → F446 became
        // the global head → StallAhead early-return → H7's ready backlog starved
        // for 2.15 s → pieces 649 ms stale → -308 PieceStartInPast.
        //
        // Per-MCU scheduling: each MCU is evaluated independently; mcu0's frames
        // must be in the Send result even though mcu1 is ahead-stalled.
        let mut queues = BTreeMap::new();
        // mcu0 (H7): ticks in the 520 MHz domain; horizon 10_000_100; pieces ready.
        queues.insert(AxisKey { mcu_id: 0, axis: 0 }, q_with(8, &[10_000_000]));
        queues.insert(AxisKey { mcu_id: 0, axis: 1 }, q_with(8, &[10_000_010]));
        // mcu1 (F446): tick 50 is numerically tiny but its horizon is 40; ahead-stalled.
        queues.insert(AxisKey { mcu_id: 1, axis: 0 }, q_with(8, &[50]));
        let horizon_of = |mcu_id: u32| -> Option<u64> {
            match mcu_id {
                0 => Some(10_000_100), // mcu0 horizon: both pieces within range
                1 => Some(40),         // mcu1 horizon: piece@50 is beyond it
                _ => None,
            }
        };
        match schedule(&queues, 255, horizon_of) {
            Schedule::Send(frames) => {
                // mcu0 must have been scheduled.
                assert!(
                    frames.iter().any(|f| f.key.mcu_id == 0),
                    "mcu0 frames must be present; got {frames:?}"
                );
                // mcu1 must NOT appear — it is ahead-stalled.
                assert!(
                    !frames.iter().any(|f| f.key.mcu_id == 1),
                    "mcu1 must not appear while ahead-stalled; got {frames:?}"
                );
            }
            other => panic!("expected Send, got {other:?}"),
        }
    }

    #[test]
    fn face_b_regression_full_mcu_does_not_starve_ready_mcu() {
        // Regression: StallFull cross-MCU coupling. mcu1's head queue ring is full
        // (room==0); mcu0 has pieces within its horizon and ring room.
        //
        // Old global-head logic: if mcu1 happened to be the global head (e.g.
        // numerically-smaller tick), StallFull early-return would starve mcu0's
        // ready work until a heartbeat freed mcu1's ring — causing the same class
        // of -308 on mcu0 once its pieces went stale.
        //
        // Per-MCU scheduling: mcu1 is full-stalled for this round; mcu0's frames
        // must still be in the Send result.
        let mut queues = BTreeMap::new();
        // mcu0: ready — pieces within horizon, ring has room.
        queues.insert(AxisKey { mcu_id: 0, axis: 0 }, q_with(8, &[100]));
        // mcu1: ring full (pushed == ring_depth).
        let mut full_q = q_with(4, &[200]);
        full_q.pushed = 4;
        queues.insert(AxisKey { mcu_id: 1, axis: 0 }, full_q);
        let horizon_of = |_mcu_id: u32| -> Option<u64> { Some(1_000) };
        match schedule(&queues, 255, horizon_of) {
            Schedule::Send(frames) => {
                assert!(
                    frames.iter().any(|f| f.key.mcu_id == 0),
                    "mcu0 frames must be present; got {frames:?}"
                );
                assert!(
                    !frames.iter().any(|f| f.key.mcu_id == 1),
                    "full mcu1 must not appear in the batch; got {frames:?}"
                );
            }
            other => panic!("expected Send, got {other:?}"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::mpsc;
    use std::sync::{Arc, Mutex};

    #[test]
    fn room_full_then_drains() {
        let mut q = AxisQueue::new(4);
        assert_eq!(q.room(), 4);
        q.pushed = 4;
        assert_eq!(q.room(), 0); // full
        q.retired = 1;
        assert_eq!(q.room(), 1); // one freed
    }

    #[test]
    fn room_correct_across_u32_wrap() {
        let mut q = AxisQueue::new(8);
        q.pushed = 2; // wrapped past u32::MAX
        q.retired = u32::MAX; // retired is "behind" pushed by 3
        // in_flight = 2 - (u32::MAX) wrapping = 3
        assert_eq!(q.room(), 5);
    }

    #[test]
    fn physical_write_cursor_advances_and_wraps_at_n() {
        let mut q = AxisQueue::new(4); // ring_depth 4
        assert_eq!(q.physical_write_cursor, 0);
        q.advance_write_cursor(3);
        assert_eq!(q.physical_write_cursor, 3);
        // 3 + 3 = 6 → 6 % 4 = 2
        q.advance_write_cursor(3);
        assert_eq!(q.physical_write_cursor, 2);
    }

    // ── Mock sink for run_pump tests ──────────────────────────────────────────

    /// Records every send_frame call's (start_slot, new_head) for assertions.
    #[derive(Clone)]
    struct RecordingSink {
        calls: Arc<Mutex<Vec<(u16, u32)>>>,
    }

    impl RecordingSink {
        fn new() -> Self {
            Self {
                calls: Arc::new(Mutex::new(Vec::new())),
            }
        }
        fn recorded(&self) -> Vec<(u16, u32)> {
            self.calls.lock().unwrap().clone()
        }
    }

    impl PieceSink for RecordingSink {
        fn send_frame(
            &self,
            _key: AxisKey,
            _pieces: &[PieceEntry],
            start_slot: u16,
            new_head: u32,
        ) -> Result<i32, String> {
            self.calls.lock().unwrap().push((start_slot, new_head));
            Ok(kalico_protocol::result_codes::OK)
        }
    }

    fn make_piece(t: u64) -> PieceEntry {
        PieceEntry {
            start_time: t,
            coeffs: [0.0; 4],
            duration: 0.001,
            _reserved: 0,
        }
    }

    #[test]
    fn run_pump_sets_start_slot_from_cursor_and_advances_it() {
        // ring_depth = 8 (> 2*N), so there is always room for both batches
        // without needing a heartbeat to free slots. N=3 pieces per batch:
        //
        //   First send:  start_slot = 0,        new_head = N=3.
        //                cursor advances to N % 8 = 3.
        //   Second send: start_slot = 3,        new_head = 2*N=6.
        //                cursor advances to (3+3) % 8 = 6.
        //
        // The pump runs in a separate thread (matching pump_loop.rs style)
        // because the burst-drain logic would otherwise see the Shutdown
        // message and exit before sending if everything is pre-queued.
        const RING_DEPTH: u32 = 8; // > 2*N, guarantees room for both batches
        const N: u32 = 3;

        let sink = RecordingSink::new();
        let (tx, rx) = mpsc::channel::<PumpMsg>();
        let sink_clone = sink.clone();
        let handle = std::thread::spawn(move || {
            run_pump(rx, sink_clone, |_key| RING_DEPTH, |_mcu| None);
        });

        // Enqueue first batch of N pieces and spin-wait until the pump drains it.
        tx.send(PumpMsg::Enqueue(EnqueueMsg {
            key: AxisKey { mcu_id: 1, axis: 0 },
            pieces: (0..N).map(|i| make_piece(i as u64)).collect(),
            fresh_stream: false,
        }))
        .unwrap();
        {
            let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
            while sink.recorded().is_empty() {
                assert!(
                    std::time::Instant::now() < deadline,
                    "pump did not drain first batch within deadline"
                );
                std::thread::yield_now();
            }
        }

        // Enqueue second batch of N pieces and spin-wait until the pump drains it.
        tx.send(PumpMsg::Enqueue(EnqueueMsg {
            key: AxisKey { mcu_id: 1, axis: 0 },
            pieces: (N..N * 2).map(|i| make_piece(i as u64)).collect(),
            fresh_stream: false,
        }))
        .unwrap();
        {
            let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
            while sink.recorded().len() < 2 {
                assert!(
                    std::time::Instant::now() < deadline,
                    "pump did not drain second batch within deadline"
                );
                std::thread::yield_now();
            }
        }

        tx.send(PumpMsg::Shutdown).unwrap();
        handle.join().unwrap();

        let recorded = sink.recorded();
        assert_eq!(
            recorded.len(),
            2,
            "expected exactly 2 sends, got {}",
            recorded.len()
        );

        let (s0, h0) = recorded[0];
        let (s1, h1) = recorded[1];

        // First send: cursor was 0, new_head = N.
        assert_eq!(s0, 0, "first start_slot should be 0");
        assert_eq!(h0, N, "first new_head should be N={N}");

        // Second send: cursor advanced to (0 + N) % RING_DEPTH after first send.
        let expected_s1 = (N % RING_DEPTH) as u16;
        assert_eq!(s1, expected_s1, "second start_slot should be {expected_s1}");
        assert_eq!(h1, N * 2, "second new_head should be {}", N * 2);
        // Sanity: cursor after second send would be (N + N) % RING_DEPTH.
        // We don't assert it here (no access to the queue after run_pump exits),
        // but the new_head values prove both batches were fully sent.
    }

    // ── Monotonic junction clamp tests ───────────────────────────────────────

    /// A sink that records the `start_time` of every piece sent across all
    /// frames, in delivery order.
    #[derive(Clone)]
    struct StartTimeSink {
        starts: Arc<Mutex<Vec<u64>>>,
    }

    impl StartTimeSink {
        fn new() -> Self {
            Self {
                starts: Arc::new(Mutex::new(Vec::new())),
            }
        }
        fn recorded(&self) -> Vec<u64> {
            self.starts.lock().unwrap().clone()
        }
    }

    impl PieceSink for StartTimeSink {
        fn send_frame(
            &self,
            _key: AxisKey,
            pieces: &[PieceEntry],
            _start_slot: u16,
            _new_head: u32,
        ) -> Result<i32, String> {
            let mut g = self.starts.lock().unwrap();
            for p in pieces {
                g.push(p.start_time);
            }
            Ok(kalico_protocol::result_codes::OK)
        }
    }

    fn make_piece_dur(t: u64, duration: f32) -> PieceEntry {
        PieceEntry {
            start_time: t,
            coeffs: [0.0; 4],
            duration,
            _reserved: 0,
        }
    }

    /// Helper: send a single Enqueue, wait until `expected_count` pieces have
    /// been recorded by the sink, then send Shutdown and join.
    fn pump_one_batch(
        sink: StartTimeSink,
        key: AxisKey,
        pieces: Vec<PieceEntry>,
        fresh_stream: bool,
        ring_depth: u32,
    ) -> Vec<u64> {
        let (tx, rx) = std::sync::mpsc::channel::<PumpMsg>();
        let sink_clone = sink.clone();
        let expected = pieces.len();
        let handle = std::thread::spawn(move || {
            run_pump(rx, sink_clone, |_| ring_depth, |_| None);
        });
        tx.send(PumpMsg::Enqueue(EnqueueMsg {
            key,
            pieces,
            fresh_stream,
        }))
        .unwrap();
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
        loop {
            if sink.recorded().len() >= expected {
                break;
            }
            assert!(
                std::time::Instant::now() < deadline,
                "pump did not drain batch within deadline"
            );
            std::thread::yield_now();
        }
        tx.send(PumpMsg::Shutdown).unwrap();
        handle.join().unwrap();
        sink.recorded()
    }

    /// Pump two batches sequentially through the same pump thread.
    /// Returns the recorded start_times of all sent pieces.
    fn pump_two_batches(
        batch_a: (Vec<PieceEntry>, bool),
        batch_b: (Vec<PieceEntry>, bool),
        ring_depth: u32,
    ) -> Vec<u64> {
        let key = AxisKey { mcu_id: 1, axis: 0 };
        let sink = StartTimeSink::new();
        let (tx, rx) = std::sync::mpsc::channel::<PumpMsg>();
        let sink_clone = sink.clone();
        let total = batch_a.0.len() + batch_b.0.len();
        let handle = std::thread::spawn(move || {
            run_pump(rx, sink_clone, |_| ring_depth, |_| None);
        });

        tx.send(PumpMsg::Enqueue(EnqueueMsg {
            key,
            pieces: batch_a.0,
            fresh_stream: batch_a.1,
        }))
        .unwrap();
        // Wait for batch A to drain so last_enqueued_end is committed.
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
        loop {
            if sink.recorded().len() >= total / 2 {
                break;
            }
            assert!(
                std::time::Instant::now() < deadline,
                "batch A did not drain"
            );
            std::thread::yield_now();
        }

        tx.send(PumpMsg::Enqueue(EnqueueMsg {
            key,
            pieces: batch_b.0,
            fresh_stream: batch_b.1,
        }))
        .unwrap();
        let deadline2 = std::time::Instant::now() + std::time::Duration::from_secs(2);
        loop {
            if sink.recorded().len() >= total {
                break;
            }
            assert!(
                std::time::Instant::now() < deadline2,
                "batch B did not drain"
            );
            std::thread::yield_now();
        }

        tx.send(PumpMsg::Shutdown).unwrap();
        handle.join().unwrap();
        sink.recorded()
    }

    /// Use the F446 approx_freq (180 MHz).  clamp_max = 0.05 × 180_000_000
    /// = 9_000_000 ticks ≈ 50 ms.
    ///
    /// Batch A: 2 pieces with duration 0.001 s each.
    ///   piece 0: start=0,       end ≈ 0 + 0.001×180e6 = 180_000
    ///   piece 1: start=180_000, end ≈ 360_000
    ///
    /// Batch B with overlap X=500 ticks (< clamp_max):
    ///   piece 0 raw: start = 360_000 − 500 = 359_500
    ///   piece 1 raw: start = 359_500 + 180_000 = 539_500
    ///
    /// After clamp (+500 ticks):
    ///   piece 0: start = 360_000
    ///   piece 1: start = 540_000
    ///
    /// Assert: all B pieces shifted by X; full queue timeline is monotonic.
    #[test]
    fn clamp_shifts_overlapping_batch() {
        // F446 freq: 180_000_000 Hz; duration 0.001 s → 180_000 ticks per piece.
        const DUR: f32 = 0.001;
        const FREQ: u64 = 180_000_000;
        let ticks_per_piece = (DUR * FREQ as f32) as u64; // 180_000

        let batch_a = vec![
            make_piece_dur(0, DUR),
            make_piece_dur(ticks_per_piece, DUR),
        ];
        let end_a = ticks_per_piece * 2; // 360_000

        let overlap: u64 = 500;
        let batch_b_raw_start = end_a - overlap; // 359_500
        let batch_b = vec![
            make_piece_dur(batch_b_raw_start, DUR),
            make_piece_dur(batch_b_raw_start + ticks_per_piece, DUR),
        ];

        let starts = pump_two_batches(
            (batch_a, false),
            (batch_b, false),
            32,
        );

        assert_eq!(starts.len(), 4, "expected 4 pieces total");

        // Batch A unchanged.
        assert_eq!(starts[0], 0);
        assert_eq!(starts[1], ticks_per_piece);

        // Batch B shifted by `overlap`.
        assert_eq!(
            starts[2], end_a,
            "batch B piece 0 should be clamped to end_a={end_a}, got {}",
            starts[2]
        );
        assert_eq!(
            starts[3],
            end_a + ticks_per_piece,
            "batch B piece 1 should be end_a + ticks_per_piece"
        );

        // Full timeline monotonic: each start_time >= previous.
        for w in starts.windows(2) {
            assert!(
                w[1] >= w[0],
                "timeline non-monotonic: {} then {}",
                w[0],
                w[1]
            );
        }
    }

    /// A fresh_stream=true batch that overlaps with the previous batch's end
    /// must NOT be clamped — it legitimately re-anchors the timeline.
    #[test]
    fn fresh_stream_not_clamped() {
        const DUR: f32 = 0.001;
        const FREQ: u64 = 180_000_000;
        let ticks_per_piece = (DUR * FREQ as f32) as u64; // 180_000

        let batch_a = vec![make_piece_dur(0, DUR), make_piece_dur(ticks_per_piece, DUR)];
        let end_a = ticks_per_piece * 2;

        let overlap: u64 = 500;
        let raw_start = end_a - overlap; // overlaps batch A's end

        // Batch B: fresh_stream=true — not a wobble, a real re-anchor.
        let batch_b = vec![make_piece_dur(raw_start, DUR)];

        let starts = pump_two_batches(
            (batch_a, false),
            (batch_b, true), // fresh_stream!
            32,
        );

        assert_eq!(starts.len(), 3);
        // Batch B piece must appear at the raw (un-clamped) start_time.
        assert_eq!(
            starts[2], raw_start,
            "fresh_stream batch must not be clamped; expected raw_start={raw_start}, got {}",
            starts[2]
        );
    }

    /// An overlap that exceeds clamp_max_ticks (> 50 ms × freq) must NOT be
    /// shifted — a jump that large indicates a missed re-anchor, not a
    /// projection wobble.  The pieces are left at their raw start_times.
    #[test]
    fn overlap_beyond_bound_not_clamped() {
        const DUR: f32 = 0.001;
        const FREQ: u64 = 180_000_000;
        let ticks_per_piece = (DUR * FREQ as f32) as u64; // 180_000
        // clamp_max = 0.05 × 180_000_000 = 9_000_000 ticks.
        let clamp_max: u64 = (0.05 * FREQ as f64) as u64; // 9_000_000

        // Start batch A far enough forward that end_a > huge_overlap, so that
        // raw_start is positive (saturating_sub would silently zero it otherwise).
        const START_OFFSET: u64 = 100_000_000; // >> clamp_max
        let batch_a = vec![
            make_piece_dur(START_OFFSET, DUR),
            make_piece_dur(START_OFFSET + ticks_per_piece, DUR),
        ];
        let end_a = START_OFFSET + ticks_per_piece * 2;

        // Overlap is clamp_max + 1 — one tick beyond the bound.
        let huge_overlap = clamp_max + 1;
        let raw_start = end_a - huge_overlap; // positive: end_a >> huge_overlap

        let batch_b = vec![make_piece_dur(raw_start, DUR)];

        let starts = pump_two_batches(
            (batch_a, false),
            (batch_b, false),
            32,
        );

        assert_eq!(starts.len(), 3);
        // Batch B piece must appear at the raw (un-clamped) start_time.
        assert_eq!(
            starts[2], raw_start,
            "overlap beyond bound must not be clamped; expected raw_start={raw_start}, got {}",
            starts[2]
        );
    }
}

// ── Inbound message types ────────────────────────────────────────────────────

/// Pieces handed to the pump for one (mcu, axis), in time order.
pub struct EnqueueMsg {
    pub key: AxisKey,
    pub pieces: Vec<PieceEntry>,
    /// Set when this batch begins a fresh stream (timeline re-anchor): the
    /// pump leaves flow-control counters alone (spec §3.3) — this flag exists
    /// only so future logic can react; for now it is informational.
    pub fresh_stream: bool,
}

/// Per-MCU heartbeat: retired counts indexed by axis.
pub struct HeartbeatMsg {
    pub mcu_id: u32,
    pub retired_counts: Vec<u32>,
}

/// Inbound to the pump loop.
pub enum PumpMsg {
    Enqueue(EnqueueMsg),
    Heartbeat(HeartbeatMsg),
    Shutdown,
}

// ── PieceSink trait ──────────────────────────────────────────────────────────

/// Sends one axis's frame to the wire. Returns the MCU's result code
/// (`result_codes::OK` on success).
pub trait PieceSink: Send {
    /// Send `pieces` for `key`, starting at physical ring slot `start_slot`.
    /// `new_head` is `pushed.wrapping_add(pieces.len())` — the post-send
    /// monotonic counter the MCU will see as the new head.
    fn send_frame(
        &self,
        key: AxisKey,
        pieces: &[PieceEntry],
        start_slot: u16,
        new_head: u32,
    ) -> Result<i32, String>;
}

// ── run_pump ─────────────────────────────────────────────────────────────────

/// Commit-lead horizon: the pump will not commit a piece whose `start_time` is
/// more than this many seconds ahead of the MCU's projected clock-now.
///
/// NOT PROVEN / NOT FINAL: related to engine's MAX_START_IN_PAST_SECS
/// (rust/runtime/src/engine.rs) through clock drift, but the correct value and
/// ratio are not established. Do not couple them numerically.
const MAX_LEAD_SECS: f64 = 1.0;

/// Run the pump until `Shutdown`. `ring_depth_of` supplies each ring's depth
/// the first time its key is seen. `sink` performs the actual wire send.
/// `mcu_clock_of(mcu_id)` returns `Some((ack_now_ticks, freq_hz))` when the
/// MCU's clock-sync is established, or `None` before sync — in which case no
/// time gate is applied for that MCU (count-only, safe for startup).
pub fn run_pump<S, F, C>(rx: Receiver<PumpMsg>, sink: S, ring_depth_of: F, mcu_clock_of: C)
where
    S: PieceSink,
    F: Fn(AxisKey) -> u32,
    C: Fn(u32) -> Option<(u64, f64)>,
{
    let mut queues: BTreeMap<AxisKey, AxisQueue> = BTreeMap::new();
    // Cap pieces per PushPieces frame to bound the USB OTG ISR burst that fences
    // the motion tick. Bench (2026-06-01, diff-minimization iter 1): at the 255
    // wire-max the F446 -308 trips ~5 s into a jog; at 32 it holds ~13-17 s — so
    // the cap is load-bearing, an active mitigation of the F446 USB fence behind
    // the residual -308 (NOT a -311 leftover; the -311 fix was the clock-domain
    // bug). A smaller per-MCU cap for the F446 is a candidate for the -308 fix.
    const MAX_PER_FRAME: usize = 32;

    let apply = |msg: PumpMsg, queues: &mut BTreeMap<AxisKey, AxisQueue>| -> bool {
        match msg {
            PumpMsg::Shutdown => return false,
            PumpMsg::Enqueue(EnqueueMsg {
                key,
                pieces,
                fresh_stream,
            }) => {
                let q = queues
                    .entry(key)
                    .or_insert_with(|| AxisQueue::new(ring_depth_of(key)));

                // Face-A overlap diagnostic: check for non-monotonic MCU-tick
                // junctions between consecutive Enqueue batches on the same axis.
                if !pieces.is_empty() {
                    // Approximate MCU clock frequency for tick <-> µs conversion.
                    // Mirrors the approx_freq_hz pattern in WireSink::send_frame.
                    let approx_freq_hz: f64 = if key.mcu_id == 0 {
                        520_000_000.0 // H723
                    } else {
                        180_000_000.0 // F446 and other serial MCUs
                    };
                    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                    let approx_freq_f32 = approx_freq_hz as f32;

                    // Within-Enqueue monotonicity check (intra-batch).
                    for i in 0..pieces.len().saturating_sub(1) {
                        let this_end = pieces[i].end_time(approx_freq_f32);
                        let next_start = pieces[i + 1].start_time;
                        if next_start < this_end {
                            let overlap_ticks = this_end - next_start;
                            let overlap_us = (overlap_ticks as f64 / approx_freq_hz) * 1e6;
                            log::warn!(
                                "[faceb-overlap-intra] key={:?} idx={} overlap_us={:.1}",
                                key,
                                i,
                                overlap_us,
                            );
                        }
                    }

                    let first_start = pieces[0].start_time;

                    // A fresh_stream legitimately re-anchors the timeline;
                    // reset the tracking state before the junction check so we
                    // don't spuriously flag it as an overlap, but still log it
                    // with the fresh_stream=true marker so the distribution is
                    // visible.
                    if fresh_stream {
                        q.last_enqueued_end = None;
                    }

                    // Maximum overlap we will absorb by shifting the batch.
                    // Anything beyond this threshold means a re-anchor was missed
                    // (not a projection wobble), so we leave it unclamped and let
                    // the MCU fault loudly.
                    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
                    let clamp_max_ticks: u64 = (0.05 * approx_freq_hz) as u64;

                    // Track how many ticks we shifted (0 = no clamp applied).
                    let mut clamped_shift: u64 = 0;

                    match q.last_enqueued_end {
                        Some(prev_end) if first_start < prev_end => {
                            let overlap_ticks = prev_end - first_start;
                            let overlap_us = (overlap_ticks as f64 / approx_freq_hz) * 1e6;
                            // Log pre-clamp values so the raw projection wobble
                            // distribution is always visible.
                            log::warn!(
                                "[faceb-overlap] key={:?} OVERLAP first_start={} prev_end={} \
                                 overlap_ticks={} overlap_us={:.1} fresh_stream={}",
                                key,
                                first_start,
                                prev_end,
                                overlap_ticks,
                                overlap_us,
                                fresh_stream,
                            );
                            if overlap_ticks <= clamp_max_ticks {
                                // Physical meaning: shifting the entire batch
                                // forward by `overlap` ticks converts the
                                // sub-ms projection wobble into a sub-ms dwell
                                // at the junction — position-continuous,
                                // kinematically negligible.  The real fix is
                                // tick-exact incremental projection per chain;
                                // this clamp exists to verify the mechanism on
                                // the bench.
                                clamped_shift = overlap_ticks;
                                log::warn!(
                                    "[faceb-clamp] key={:?} shifted batch +{}ticks (+{:.1}us) \
                                     to restore monotonicity",
                                    key,
                                    overlap_ticks,
                                    overlap_us,
                                );
                            } else {
                                log::error!(
                                    "[faceb-clamp] key={:?} overlap {:.1}us EXCEEDS clamp bound \
                                     — leaving unclamped (will fault)",
                                    key,
                                    overlap_us,
                                );
                            }
                        }
                        Some(prev_end) => {
                            let gap_ticks = first_start - prev_end;
                            let gap_us = (gap_ticks as f64 / approx_freq_hz) * 1e6;
                            log::warn!(
                                "[faceb-junction] key={:?} gap_ticks={} gap_us={:.1} \
                                 fresh_stream={}",
                                key,
                                gap_ticks,
                                gap_us,
                                fresh_stream,
                            );
                        }
                        None => {}
                    }

                    // Apply the shift to every piece in the incoming batch before
                    // extending the queue.  When clamped_shift == 0 this is a
                    // no-op that the compiler elides.
                    let pieces: Vec<_> = if clamped_shift > 0 {
                        pieces
                            .into_iter()
                            .map(|mut p| {
                                p.start_time += clamped_shift;
                                p
                            })
                            .collect()
                    } else {
                        pieces
                    };

                    // last_end must reflect the post-clamp timeline so the next
                    // junction check sees the shifted boundary.
                    let last_end = pieces[pieces.len() - 1].end_time(approx_freq_f32);
                    q.last_enqueued_end = Some(last_end);

                    q.pieces.extend(pieces);
                }
            }
            PumpMsg::Heartbeat(HeartbeatMsg {
                mcu_id,
                retired_counts,
            }) => {
                // INVARIANT: retired_counts[i] is axis index i, the SAME axis
                // numbering used by PushPieces.axis_idx and the enqueue adapter's
                // AxisKey.axis. Verified end-to-end on the MCU: the heartbeat FFI
                // writes retired_counts()[i] = stepping_axes[i].ring.retired_count()
                // in index order (runtime_ffi.rs), and push_pieces(axis_idx) targets
                // that same stepping_axes[axis_idx]. Do not reorder either side.
                log::warn!(
                    "[faceb] heartbeat mcu={} retired_counts={:?}",
                    mcu_id,
                    retired_counts,
                );
                for (axis, &c) in retired_counts.iter().enumerate() {
                    let key = AxisKey {
                        mcu_id,
                        axis: axis as u8,
                    };
                    if let Some(q) = queues.get_mut(&key) {
                        q.retired = c;
                    }
                }
            }
        }
        true
    };

    // Build a horizon closure from the mcu_clock_of callback. Called once per
    // schedule pass so the horizon tracks the advancing MCU clock.
    let horizon_of = |mcu_id: u32| -> Option<u64> {
        let (ack_now, freq) = mcu_clock_of(mcu_id)?;
        #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
        Some(ack_now + (MAX_LEAD_SECS * freq) as u64)
    };

    // Whether the most recent schedule decision was StallAhead. When true the
    // outer wait uses recv_timeout(50 ms) so the horizon can advance even with
    // no inbound messages; when false (idle or count-stalled) blocking recv()
    // is fine — heartbeats/enqueues wake it.
    let mut holding_ahead = false;

    loop {
        // If we are holding pieces that are time-gated, poll every 50 ms so
        // the horizon sweeps forward (ack_now advances with the MCU clock).
        // Otherwise block indefinitely — a heartbeat or enqueue will wake us.
        let first = if holding_ahead {
            match rx.recv_timeout(Duration::from_millis(50)) {
                Ok(m) => {
                    let tag = match &m {
                        PumpMsg::Enqueue(e) => {
                            log::warn!(
                                "[faceb] wake=msg variant=Enqueue key={:?} n_pieces={}",
                                e.key,
                                e.pieces.len(),
                            );
                            "Enqueue"
                        }
                        PumpMsg::Heartbeat(h) => {
                            log::warn!(
                                "[faceb] wake=msg variant=Heartbeat mcu_id={}",
                                h.mcu_id,
                            );
                            "Heartbeat"
                        }
                        PumpMsg::Shutdown => {
                            log::warn!("[faceb] wake=msg variant=Shutdown");
                            "Shutdown"
                        }
                    };
                    let _ = tag;
                    Some(m)
                }
                Err(RecvTimeoutError::Timeout) => {
                    // MCU clock has advanced; re-evaluate the send loop without
                    // consuming a message. `holding_ahead` stays true until the
                    // schedule loop clears it.
                    log::warn!("[faceb] wake=timeout-poll (holding_ahead 50ms tick)");
                    None
                }
                Err(RecvTimeoutError::Disconnected) => return,
            }
        } else {
            match rx.recv() {
                Ok(m) => {
                    match &m {
                        PumpMsg::Enqueue(e) => {
                            log::warn!(
                                "[faceb] wake=msg variant=Enqueue key={:?} n_pieces={}",
                                e.key,
                                e.pieces.len(),
                            );
                        }
                        PumpMsg::Heartbeat(h) => {
                            log::warn!(
                                "[faceb] wake=msg variant=Heartbeat mcu_id={}",
                                h.mcu_id,
                            );
                        }
                        PumpMsg::Shutdown => {
                            log::warn!("[faceb] wake=msg variant=Shutdown");
                        }
                    }
                    Some(m)
                }
                Err(RecvError) => return,
            }
        };

        if let Some(msg) = first {
            if !apply(msg, &mut queues) {
                return;
            }
            // Drain anything else already queued (coalesce bursts before sending).
            while let Ok(m) = rx.try_recv() {
                if !apply(m, &mut queues) {
                    return;
                }
            }
        }

        // Send as far as the schedule allows. A send failure breaks all the
        // way back to the outer wait (labeled break) instead of re-running
        // schedule on the still-queued pieces — otherwise a persistent failure
        // spins a tight busy-loop. The next inbound message (heartbeat/enqueue)
        // retries.
        holding_ahead = false;
        'send: loop {
            match schedule(&queues, MAX_PER_FRAME, &horizon_of) {
                Schedule::Idle => {
                    log::warn!("[faceb] sched=idle");
                    break 'send;
                }
                Schedule::StallFull(stall_key) => {
                    let q = queues.get(&stall_key);
                    let (pieces_len, pushed, retired, room, ring_depth, front_start) =
                        q.map(|q| {
                            (
                                q.pieces.len(),
                                q.pushed,
                                q.retired,
                                q.room(),
                                q.ring_depth,
                                q.pieces.front().map(|p| p.start_time),
                            )
                        })
                        .unwrap_or((0, 0, 0, 0, 0, None));
                    let horizon = horizon_of(stall_key.mcu_id);
                    log::warn!(
                        "[faceb] sched=StallFull key={:?} pieces={} pushed={} retired={} \
                         room={} ring_depth={} front_start={:?} horizon={:?}",
                        stall_key,
                        pieces_len,
                        pushed,
                        retired,
                        room,
                        ring_depth,
                        front_start,
                        horizon,
                    );
                    break 'send;
                }
                Schedule::StallAhead(stall_key) => {
                    let q = queues.get(&stall_key);
                    let (pieces_len, pushed, retired, room, ring_depth, front_start) =
                        q.map(|q| {
                            (
                                q.pieces.len(),
                                q.pushed,
                                q.retired,
                                q.room(),
                                q.ring_depth,
                                q.pieces.front().map(|p| p.start_time),
                            )
                        })
                        .unwrap_or((0, 0, 0, 0, 0, None));
                    let horizon = horizon_of(stall_key.mcu_id);
                    log::warn!(
                        "[faceb] sched=StallAhead key={:?} pieces={} pushed={} retired={} \
                         room={} ring_depth={} front_start={:?} horizon={:?}",
                        stall_key,
                        pieces_len,
                        pushed,
                        retired,
                        room,
                        ring_depth,
                        front_start,
                        horizon,
                    );
                    // Ring has room but the head piece is beyond the current
                    // commit-lead horizon. Arm the 50 ms poll so we
                    // re-evaluate as the MCU clock advances.
                    holding_ahead = true;
                    break 'send;
                }
                Schedule::Send(frames) => {
                    if frames.is_empty() {
                        break 'send;
                    }
                    let total_pieces: usize = frames.iter().map(|f| f.pieces.len()).sum();
                    let first_start = frames
                        .first()
                        .and_then(|f| f.pieces.first())
                        .map(|p| p.start_time)
                        .unwrap_or(0);
                    log::warn!(
                        "[faceb] sched=Send n_frames={} total_pieces={} first_start={}",
                        frames.len(),
                        total_pieces,
                        first_start,
                    );
                    for mut f in frames {
                        let n = f.pieces.len() as u32;
                        // Look up cursor + compute new_head BEFORE borrowing the
                        // queue mutably after the send. Borrow ends at end of
                        // this block so the mut borrow below doesn't conflict.
                        let new_head = {
                            let q = queues.get(&f.key).expect("planned key exists");
                            debug_assert!(
                                q.ring_depth <= u32::from(u16::MAX),
                                "ring_depth {} exceeds u16::MAX; start_slot cast is lossy",
                                q.ring_depth
                            );
                            f.start_slot = q.physical_write_cursor as u16;
                            q.pushed.wrapping_add(n)
                        };
                        let frame_first_start = f.pieces.first().map(|p| p.start_time).unwrap_or(0);
                        let t_before_send = std::time::Instant::now();
                        match sink.send_frame(f.key, &f.pieces, f.start_slot, new_head) {
                            Ok(_) => {
                                let rtt_us = t_before_send.elapsed().as_micros();
                                log::warn!(
                                    "[faceb] send_frame key={:?} n_pieces={} first_start={} rtt_us={}",
                                    f.key,
                                    f.pieces.len(),
                                    frame_first_start,
                                    rtt_us,
                                );
                                let q = queues.get_mut(&f.key).expect("planned key exists");
                                for _ in 0..f.pieces.len() {
                                    q.pieces.pop_front();
                                }
                                q.pushed = q.pushed.wrapping_add(n);
                                q.advance_write_cursor(n);
                            }
                            Err(ref e) => {
                                log::error!("pump send_frame failed for {:?}: {e}", f.key);
                                // Leave the pieces queued; retry on next message.
                                break 'send;
                            }
                        }
                    }
                }
            }
        }
    }
}

// ── WireSink ─────────────────────────────────────────────────────────────────

/// Per-MCU transport variant held by `WireSink`.
///
/// Serial MCUs communicate via `KalicoHostIo` (the reactor-backed serial
/// transport). EtherCAT MCUs communicate via `UnixNativeConn` (a same-host
/// Unix-socket client). Both implement the `PushPieces` / `PushPiecesResponse`
/// exchange; they differ only in the underlying byte pipe.
///
/// `Weak<KalicoHostIo>` is intentional: `detach_serial` drops the strong
/// `Arc` to tear down the reactor; the pump must not pin the IO alive.
/// `Arc<UnixNativeConn>` for EtherCAT: there is no separate "detach" path for
/// EtherCAT conns; the pump holds the only strong ref and the conn drops with
/// it.
pub enum McuTransport {
    Serial(Weak<kalico_host_rt::host_io::KalicoHostIo>),
    EtherCat(std::sync::Arc<kalico_host_rt::unix_native_conn::UnixNativeConn>),
}

impl std::fmt::Debug for McuTransport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Serial(_) => write!(f, "McuTransport::Serial"),
            Self::EtherCat(_) => write!(f, "McuTransport::EtherCat"),
        }
    }
}

/// Production sink: one `kalico_call(PushPieces)` per frame.
///
/// Routes each frame to the appropriate transport by `mcu_id`:
/// - Serial MCUs → `Weak<KalicoHostIo>::kalico_call_on_channel`
/// - EtherCAT MCUs → `Arc<UnixNativeConn>::kalico_call_on_channel`
///
/// A missing transport entry for an `mcu_id` is a hard error — it indicates
/// a logic bug in `init_planner` that left an axis without a transport.
pub struct WireSink {
    pub transports: HashMap<u32, McuTransport>,
    pub timeout: Duration,
}

impl WireSink {
    /// Build the encoded PushPieces body and call it on the appropriate
    /// transport for `key.mcu_id`. Returns the raw `result` field from
    /// `PushPiecesResponse` on success.
    fn call_push_pieces(
        &self,
        key: AxisKey,
        pieces: &[PieceEntry],
        start_slot: u16,
        new_head: u32,
    ) -> Result<kalico_protocol::messages::PushPiecesResponse, String> {
        let mut pieces_bytes = Vec::with_capacity(std::mem::size_of_val(pieces));
        for p in pieces {
            pieces_bytes.extend_from_slice(&p.to_le_bytes());
        }

        let msg = kalico_protocol::messages::PushPieces {
            axis_idx: key.axis,
            piece_count: pieces.len() as u8,
            start_slot,
            new_head,
            pieces_bytes,
        };
        let mut body = Vec::with_capacity(8 + std::mem::size_of_val(pieces));
        kalico_protocol::codec::Encode::encode(&msg, &mut body);

        let transport = self.transports.get(&key.mcu_id).ok_or_else(|| {
            format!(
                "WireSink: no transport for mcu_id {} (axis {}); \
                 this is a logic bug in init_planner — the axis was enqueued \
                 without registering its transport",
                key.mcu_id, key.axis
            )
        })?;

        let resp_body = match transport {
            McuTransport::Serial(weak) => {
                let io = weak
                    .upgrade()
                    .ok_or_else(|| format!("KalicoHostIo for mcu {} detached", key.mcu_id))?;
                let (_kind, b) = io
                    .kalico_call_on_channel(
                        kalico_protocol::KALICO_CHANNEL_PIECES,
                        kalico_protocol::MessageKind::PushPieces,
                        body,
                        self.timeout,
                    )
                    .map_err(|e| format!("serial PushPieces mcu {}: {e:?}", key.mcu_id))?;
                b
            }
            McuTransport::EtherCat(conn) => {
                let (_kind, b) = conn
                    .kalico_call_on_channel(
                        kalico_protocol::KALICO_CHANNEL_PIECES,
                        kalico_protocol::MessageKind::PushPieces,
                        body,
                        self.timeout,
                    )
                    .map_err(|e| format!("ethercat PushPieces mcu {}: {e:?}", key.mcu_id))?;
                b
            }
        };

        use kalico_protocol::codec::Decode as _;
        kalico_protocol::messages::PushPiecesResponse::decode(&resp_body)
            .map_err(|e| format!("decode PushPiecesResponse mcu {}: {e:?}", key.mcu_id))
    }
}

impl PieceSink for WireSink {
    fn send_frame(
        &self,
        key: AxisKey,
        pieces: &[PieceEntry],
        start_slot: u16,
        new_head: u32,
    ) -> Result<i32, String> {
        // schedule() caps frames at MAX_PER_FRAME (currently 32); this guards
        // against callers bypassing schedule() and hitting a silent truncation.
        debug_assert!(
            pieces.len() <= 255,
            "PushPieces frame exceeds u8 piece_count; schedule() must cap at MAX_PER_FRAME"
        );

        let host_front_start_time: u64 = pieces.first().map(|p| p.start_time).unwrap_or(0);

        let r = self.call_push_pieces(key, pieces, start_slot, new_head)?;

        // Emit transit/arrival-lead diagnostic for every successful PushPieces.
        // arrival_lead = mcu_front_start_time - arrival_clock (both MCU ticks).
        // Negative → piece already in MCU past at commit time → -308 risk.
        // host_front_start_time==0 means the router had no clock sync when the
        // piece was dispatched (project() returned 0); log it as a zero so the
        // gap is visible rather than silently skipped.
        {
            // The arithmetic uses i64 to correctly represent negative arrival-lead.
            let arrival_lead_ticks = r.front_start_time as i64 - r.arrival_clock as i64;
            // Approximate µs: EtherCAT clocks are CLOCK_MONOTONIC nanoseconds
            // (>> 1e12 on a running system); serial MCU clocks are < 1e12. For
            // serial, use per-MCU frequency: 520 MHz for H723 (mcu_id 0), 180 MHz
            // for F446 (mcu_id 1+). Both raw-ticks and µs are logged so the exact
            // value can be derived offline with the per-MCU freq from klippy log.
            let approx_freq_hz: f64 = if r.arrival_clock > 1_000_000_000_000 {
                1_000_000_000.0 // EtherCAT: CLOCK_MONOTONIC ns domain
            } else if key.mcu_id == 0 {
                520_000_000.0 // H723 (serial MCU 0)
            } else {
                180_000_000.0 // F446 and other serial MCUs
            };
            let arrival_lead_us = (arrival_lead_ticks as f64 / approx_freq_hz) * 1e6;
            // Wall-clock seconds for offline correlation with klippy clock-sync.
            let host_send_secs = {
                use std::time::{SystemTime, UNIX_EPOCH};
                SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .map(|d| d.as_secs_f64())
                    .unwrap_or(0.0)
            };
            // Warn when arrival_lead is negative (piece arrived in MCU's past)
            // or when host_front_start_time is zero (clock-sync gap on dispatch).
            // Log at info for positive lead so routine-operation frames don't
            // flood the journal; warn on the cases that indicate a -308 risk.
            let zero_st = host_front_start_time == 0;
            let past_arrival = arrival_lead_ticks < 0;
            if zero_st || past_arrival {
                log::warn!(
                    "[transit-diag] mcu={} axis={} \
                     host_front_start_time={} mcu_front_start_time={} \
                     arrival_clock={} \
                     arrival_lead_ticks={} arrival_lead_us={:.1} \
                     host_send_unix_secs={:.6} \
                     ALERT: {}",
                    key.mcu_id,
                    key.axis,
                    host_front_start_time,
                    r.front_start_time,
                    r.arrival_clock,
                    arrival_lead_ticks,
                    arrival_lead_us,
                    host_send_secs,
                    if zero_st && past_arrival {
                        "host_start_time=0 (clock-sync gap) AND piece in MCU past"
                    } else if zero_st {
                        "host_start_time=0 (router clock_freq=0 at dispatch — clock-sync gap)"
                    } else {
                        "piece arrived in MCU past (arrival_lead<0) — PieceStartInPast risk"
                    },
                );
            } else {
                log::info!(
                    "[transit-diag] mcu={} axis={} \
                     host_front_start_time={} mcu_front_start_time={} \
                     arrival_clock={} \
                     arrival_lead_ticks={} arrival_lead_us={:.1} \
                     host_send_unix_secs={:.6}",
                    key.mcu_id,
                    key.axis,
                    host_front_start_time,
                    r.front_start_time,
                    r.arrival_clock,
                    arrival_lead_ticks,
                    arrival_lead_us,
                    host_send_secs,
                );
            }
        }

        if r.result != kalico_protocol::result_codes::OK {
            return Err(format!(
                "MCU rejected PushPieces (mcu {} axis {}): {}",
                key.mcu_id, key.axis, r.result
            ));
        }
        Ok(r.result)
    }
}
