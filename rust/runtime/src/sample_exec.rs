// Sample-stream playback for phase-stepped lanes.
//
// The tick ISR interpolates linearly between the two samples bracketing `now`,
// rounds to the nearest LUT phase quantum, and hands the result to the existing
// `write_phase_coils` quantizer and SPI arbitration. No Chebyshev evaluation
// and no piece ring live on this path.
//
// C/Rust boundary: every byte of run storage lives inside `SampleLane`, which
// the engine embeds in `IsrState` inside the C-declared `rt_storage[]` buffer.
// C owns the linker-section placement (docs/rewrite/mcu-c-rust-boundary.md
// rule B2), so no `#[link_section]` appears here.

use crate::sample_run::{
    LaneCursor, SAMPLE_RUN_COUNT_MAX, SampleRunBuf, SampleRunError, SampleRunHeader, decode_deltas,
};
use crate::sizing::{SAMPLE_OVERLAY_RUNS_PER_LANE, SAMPLE_RUNS_PER_LANE};
use crate::state::SharedState;

const _: () = assert!(
    SAMPLE_RUNS_PER_LANE > 0 && SAMPLE_OVERLAY_RUNS_PER_LANE > 0,
    "sample-stepping is enabled but build.rs sized a lane ring to zero"
);

pub type SampleSlot = SampleRunBuf<SAMPLE_RUN_COUNT_MAX>;

/// A run whose start clock is already this many ticks behind the playback
/// clock has missed its window. Matches the tolerance `get_piece_for_time`
/// applies to a late piece.
pub const LATE_TOLERANCE_TICKS: u64 = 2;

/// Widen a 32-bit wire clock against the playback clock by picking the
/// candidate nearest `now`, so a command straddling a counter wrap lands on the
/// right side of it.
pub fn widen_wire_clock(now: u64, clock: u32) -> u64 {
    let distance = |candidate: u64| {
        if candidate >= now {
            candidate - now
        } else {
            now - candidate
        }
    };
    let mut best = (now & !0xFFFF_FFFFu64) | u64::from(clock);
    let mut best_distance = distance(best);
    for candidate in [best.wrapping_add(1u64 << 32), best.wrapping_sub(1u64 << 32)] {
        let candidate_distance = distance(candidate);
        if candidate_distance < best_distance {
            best = candidate;
            best_distance = candidate_distance;
        }
    }
    best
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SampleLaneFault {
    RingFull,
    Late { deficit_ticks: u64 },
    Run(SampleRunError),
}

impl SampleLaneFault {
    pub fn latch(self, shared: &SharedState, lane_idx: usize) {
        match self {
            Self::RingFull => crate::fault_helpers::raise_sample_ring_full(shared, lane_idx),
            Self::Late { deficit_ticks } => crate::fault_helpers::raise_sample_run_late(
                shared,
                lane_idx,
                u32::try_from(deficit_ticks).unwrap_or(u32::MAX),
            ),
            Self::Run(err) => {
                crate::fault_helpers::raise_sample_run_rejected(shared, lane_idx, err.fault_code());
            }
        }
    }

    pub const fn as_str(self) -> &'static str {
        match self {
            Self::RingFull => "sample lane run ring is full",
            Self::Late { .. } => "sample run start clock is already in the past",
            Self::Run(err) => err.as_str(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LaneOutput {
    /// The lane carries no anchor; the caller leaves this axis alone.
    Unanchored,
    /// Position in LUT phase quanta the coils must express this tick.
    Position(i32),
}

#[derive(Debug, Clone, Copy)]
struct SlotRing<const DEPTH: usize> {
    slots: [SampleSlot; DEPTH],
    tail: usize,
    len: usize,
}

impl<const DEPTH: usize> SlotRing<DEPTH> {
    const fn new() -> Self {
        Self {
            slots: [SampleSlot::new(0, 1); DEPTH],
            tail: 0,
            len: 0,
        }
    }

    fn clear(&mut self) {
        self.tail = 0;
        self.len = 0;
    }

    const fn len(&self) -> usize {
        self.len
    }

    const fn is_full(&self) -> bool {
        self.len >= DEPTH
    }

    fn get(&self, offset: usize) -> Option<&SampleSlot> {
        if offset >= self.len {
            return None;
        }
        self.slots.get((self.tail + offset) % DEPTH)
    }

    fn push_slot(&mut self) -> Option<&mut SampleSlot> {
        if self.len >= DEPTH {
            return None;
        }
        let index = (self.tail + self.len) % DEPTH;
        self.len += 1;
        self.slots.get_mut(index)
    }

    fn pop(&mut self) {
        if self.len == 0 {
            return;
        }
        self.tail = (self.tail + 1) % DEPTH;
        self.len -= 1;
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Halt {
    clock: u64,
    position: i32,
}

/// One phase lane's sample stream: the run ring, the additive overlay ring,
/// and the abutment cursors that police both.
#[derive(Debug)]
pub struct SampleLane {
    main: SlotRing<SAMPLE_RUNS_PER_LANE>,
    overlay: SlotRing<SAMPLE_OVERLAY_RUNS_PER_LANE>,
    cursor: LaneCursor,
    overlay_cursor: LaneCursor,
    /// Last sample value retired off the main ring; the idle hold value.
    last_sample: i32,
    last_overlay_sample: i32,
    /// Last position handed to the coils, and the clock it was evaluated at.
    played: i32,
    played_clock: u64,
    halt: Option<Halt>,
}

impl SampleLane {
    pub const fn new() -> Self {
        Self {
            main: SlotRing::new(),
            overlay: SlotRing::new(),
            cursor: LaneCursor::new(),
            overlay_cursor: LaneCursor::new(),
            last_sample: 0,
            last_overlay_sample: 0,
            played: 0,
            played_clock: 0,
            halt: None,
        }
    }

    pub const fn is_anchored(&self) -> bool {
        self.cursor.is_anchored() || self.halt.is_some()
    }

    pub const fn is_halted(&self) -> bool {
        self.halt.is_some()
    }

    pub fn depth(&self) -> usize {
        self.main.len()
    }

    /// Cross a sanctioned discontinuity: drop everything queued and restart the
    /// lane at `position` on `clock`. Also the command that clears a halt.
    pub fn anchor(&mut self, now: u64, clock: u64, position: i32) -> Result<(), SampleLaneFault> {
        late_deficit(now, clock)?;
        self.main.clear();
        self.cursor.anchor(clock, position);
        self.last_sample = position;
        self.played = position;
        self.played_clock = clock;
        self.halt = None;
        Ok(())
    }

    pub fn push_run(
        &mut self,
        now: u64,
        interval_ticks: u32,
        count: u8,
        data: &[u8],
    ) -> Result<(), SampleLaneFault> {
        let start_clock =
            self.cursor
                .next_clock()
                .ok_or(SampleLaneFault::Run(SampleRunError::NotAnchored {
                    start_clock: 0,
                }))?;
        late_deficit(now, start_clock)?;
        push_into(
            &mut self.main,
            &mut self.cursor,
            SampleRunHeader::new(start_clock, interval_ticks, u16::from(count)),
            data,
        )
    }

    /// An overlay run carries its own clock and anchors itself: a nudge that
    /// abuts the previous overlay run continues it, anything else restarts the
    /// overlay lane from a zero offset — the relativization `OverlayFrame`
    /// performs when it arms.
    pub fn push_overlay(
        &mut self,
        now: u64,
        clock: u64,
        interval_ticks: u32,
        count: u8,
        data: &[u8],
    ) -> Result<(), SampleLaneFault> {
        late_deficit(now, clock)?;
        if self.overlay_cursor.next_clock() != Some(clock) {
            self.overlay.clear();
            self.overlay_cursor.anchor(clock, 0);
            self.last_overlay_sample = 0;
        }
        push_into(
            &mut self.overlay,
            &mut self.overlay_cursor,
            SampleRunHeader::new(clock, interval_ticks, u16::from(count)),
            data,
        )
    }

    pub fn tick(&mut self, now: u64, shared: &SharedState, lane_idx: usize) -> LaneOutput {
        if let Some(halt) = self.halt {
            return LaneOutput::Position(halt.position);
        }
        if !self.cursor.is_anchored() {
            return LaneOutput::Unanchored;
        }
        let base = self.advance_main(now, shared, lane_idx);
        let nudge = self.advance_overlay(now);
        let position = base.saturating_add(nudge);
        self.played = position;
        self.played_clock = now;
        LaneOutput::Position(position)
    }

    /// Trip halt: freeze playback at the position `now` interpolates to and
    /// drop every queued run, mirroring `stepper_classic_halt`. The lane holds
    /// that position until the host re-anchors.
    pub fn halt(&mut self, now: u64, shared: &SharedState, lane_idx: usize) {
        if self.halt.is_some() {
            return;
        }
        let position = match self.tick(now, shared, lane_idx) {
            LaneOutput::Position(p) => p,
            LaneOutput::Unanchored => self.played,
        };
        self.main.clear();
        self.overlay.clear();
        self.cursor.unanchor();
        self.overlay_cursor.unanchor();
        self.halt = Some(Halt {
            clock: now,
            position,
        });
    }

    /// Executed position for host reconcile: the halt point when halted,
    /// otherwise the last position the ISR drove. Mirrors
    /// `stepper_get_position`, which likewise reports what actually executed
    /// rather than what was queued.
    pub fn executed(&self) -> (u64, i32) {
        match self.halt {
            Some(halt) => (halt.clock, halt.position),
            None => (self.played_clock, self.played),
        }
    }

    fn advance_main(&mut self, now: u64, shared: &SharedState, lane_idx: usize) -> i32 {
        while let Some(front) = self.main.get(0) {
            if now < front.header().end_clock() {
                break;
            }
            let tail_delta = tail_delta(front, self.last_sample);
            self.last_sample = front.last_position().unwrap_or(self.last_sample);
            self.main.pop();
            if self.main.len() == 0 && tail_delta != 0 {
                crate::fault_helpers::raise_sample_ring_underrun(
                    shared,
                    lane_idx,
                    tail_delta.unsigned_abs(),
                );
            }
        }
        let successor = self
            .main
            .get(1)
            .and_then(|next| next.samples().first().copied());
        match self.main.get(0) {
            Some(front) => sample_at(front, now, successor).unwrap_or(self.last_sample),
            None => self.last_sample,
        }
    }

    fn advance_overlay(&mut self, now: u64) -> i32 {
        while let Some(front) = self.overlay.get(0) {
            if now < front.header().end_clock() {
                break;
            }
            self.last_overlay_sample = front.last_position().unwrap_or(self.last_overlay_sample);
            self.overlay.pop();
        }
        let successor = self
            .overlay
            .get(1)
            .and_then(|next| next.samples().first().copied());
        match self.overlay.get(0) {
            Some(front) => sample_at(front, now, successor).unwrap_or(self.last_overlay_sample),
            None => self.last_overlay_sample,
        }
    }
}

impl Default for SampleLane {
    fn default() -> Self {
        Self::new()
    }
}

fn push_into<const DEPTH: usize>(
    ring: &mut SlotRing<DEPTH>,
    cursor: &mut LaneCursor,
    header: SampleRunHeader,
    data: &[u8],
) -> Result<(), SampleLaneFault> {
    if ring.is_full() {
        return Err(SampleLaneFault::RingFull);
    }
    let mut probe = *cursor;
    probe.accept(&header).map_err(SampleLaneFault::Run)?;

    let count = usize::from(header.count);
    let mut decoded = [0i32; SAMPLE_RUN_COUNT_MAX];
    let samples =
        decoded
            .get_mut(..count)
            .ok_or(SampleLaneFault::Run(SampleRunError::CountExceeded {
                count,
                cap: SAMPLE_RUN_COUNT_MAX,
            }))?;
    decode_deltas(cursor.position(), data, count, samples).map_err(SampleLaneFault::Run)?;
    let last = *samples
        .last()
        .ok_or(SampleLaneFault::Run(SampleRunError::ZeroCount {
            start_clock: header.start_clock,
        }))?;

    let slot = ring.push_slot().ok_or(SampleLaneFault::RingFull)?;
    slot.reset(header.start_clock, header.interval_ticks);
    for &position in samples.iter() {
        slot.push(position).map_err(SampleLaneFault::Run)?;
    }
    *cursor = probe;
    cursor.commit(last);
    Ok(())
}

fn late_deficit(now: u64, start_clock: u64) -> Result<(), SampleLaneFault> {
    let deficit = now.saturating_sub(start_clock);
    if deficit > LATE_TOLERANCE_TICKS {
        return Err(SampleLaneFault::Late {
            deficit_ticks: deficit,
        });
    }
    Ok(())
}

/// Velocity at the moment a run leaves the ring, in quanta per interval. A
/// stream that ends on purpose ends at rest, so a nonzero value here means the
/// ring drained mid-motion.
fn tail_delta(run: &SampleSlot, previous_sample: i32) -> i32 {
    let samples = run.samples();
    let Some(&last) = samples.last() else {
        return 0;
    };
    let before = match samples.len() {
        0 | 1 => previous_sample,
        n => samples.get(n - 2).copied().unwrap_or(previous_sample),
    };
    last.wrapping_sub(before)
}

/// Linear interpolation inside `run` at `now`. `successor` is the sample that
/// follows the run's last one — the next abutting run's first sample. Without
/// it the final interval of everything queued is a zero-order hold.
fn sample_at(run: &SampleSlot, now: u64, successor: Option<i32>) -> Option<i32> {
    let header = run.header();
    if now < header.start_clock {
        return None;
    }
    let interval = u64::from(header.interval_ticks);
    if interval == 0 {
        return None;
    }
    let elapsed = now - header.start_clock;
    #[allow(clippy::integer_division)]
    let index = (elapsed / interval) as usize;
    let frac = elapsed - (index as u64) * interval;
    let samples = run.samples();
    let s0 = samples.get(index).copied()?;
    let s1 = match samples.get(index + 1).copied() {
        Some(next) => next,
        None => successor.unwrap_or(s0),
    };
    Some(lerp_round(s0, s1, frac, interval))
}

/// Round-to-nearest linear blend in the lane's own fixed point. The result is
/// bracketed by `s0` and `s1`, so the narrowing cast cannot lose a bit.
#[allow(clippy::cast_possible_truncation, clippy::integer_division)]
fn lerp_round(s0: i32, s1: i32, frac: u64, interval: u64) -> i32 {
    if frac == 0 || s0 == s1 {
        return s0;
    }
    let delta = i64::from(s1) - i64::from(s0);
    let numerator = delta * (frac as i64);
    let denominator = interval as i64;
    let half = denominator / 2;
    let step = if numerator >= 0 {
        (numerator + half) / denominator
    } else {
        -((-numerator + half) / denominator)
    };
    let blended = i64::from(s0) + step;
    blended.clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32
}

#[cfg(test)]
#[path = "sample_exec_tests.rs"]
mod sample_exec_tests;

#[cfg(test)]
#[path = "sample_equivalence_tests.rs"]
mod sample_equivalence_tests;
