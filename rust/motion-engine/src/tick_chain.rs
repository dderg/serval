//! Per-MCU piece-start-tick chaining across dispatched segments.
//!
//! Re-projecting each segment's host time to MCU ticks independently injects the
//! clock estimate's absolute-offset jitter — `lever_arm (~28 s) × freq_noise
//! (~ppm)` ≈ tens of µs — at every seam. Our cubic-piece scheme requires the next
//! piece to start exactly where the previous one ends in the MCU clock domain, so
//! that jitter becomes a faulting one-sample step burst (mainline tolerates it
//! because it schedules discrete steps, not continuous pieces).
//!
//! Chaining derives each piece tick as `segment_start_tick + offset_secs ·
//! live_freq` — only the slope (live freq) is used, never the jittery absolute
//! offset — so the seam error collapses to `piece_dur · freq_tracking_error`
//! (sub-ns). Each MCU chains with its own live freq, so multi-MCU sync is
//! preserved (it is *frozen*-freq chaining that desyncs). A bounded per-seam slew
//! toward the live absolute projection caps the residual offset drift without a
//! faulting jump; `fresh`/underrun re-anchors hard.

use std::collections::HashMap;

pub struct TickChain {
    next_start: HashMap<u32, u64>,
    max_slew_secs: f64,
}

impl TickChain {
    pub fn new(max_slew_secs: f64) -> Self {
        Self {
            next_start: HashMap::new(),
            max_slew_secs,
        }
    }

    /// Hard re-anchor (first segment / underrun): the next segment starts at the
    /// live absolute projection, accepting the expected discontinuity.
    pub fn anchor(&mut self, mcu: u32, absolute_tick: u64) {
        self.next_start.insert(mcu, absolute_tick);
    }

    /// Bounded slew of the chained anchor toward the live absolute projection,
    /// capping accumulated offset drift. The step is clamped to `max_slew_secs`
    /// worth of ticks so the induced seam shift stays far below the motion fault
    /// threshold. No-op for an MCU that has not been anchored yet.
    pub fn slew(&mut self, mcu: u32, absolute_tick: u64, freq: f64) {
        if let Some(cur) = self.next_start.get_mut(&mcu) {
            let budget = (self.max_slew_secs * freq).max(0.0) as i64;
            let diff = absolute_tick as i64 - *cur as i64;
            let step = diff.clamp(-budget, budget);
            *cur = (*cur as i64 + step).max(0) as u64;
        }
    }

    /// Chained start tick of a piece at `offset_secs` into the current segment.
    /// `offset_secs` is measured from the segment's `t_start`; offset 0 reproduces
    /// the chained anchor exactly (seam continuity). Returns `None` if the MCU has
    /// not been anchored.
    pub fn piece_tick(&self, mcu: u32, offset_secs: f64, freq: f64) -> Option<u64> {
        let base = *self.next_start.get(&mcu)?;
        Some(base + (offset_secs * freq).max(0.0) as u64)
    }

    /// Advance the chain past a segment of `seg_dur_secs` so the next segment's
    /// anchor continues exactly where this one ended.
    pub fn advance(&mut self, mcu: u32, seg_dur_secs: f64, freq: f64) {
        if let Some(cur) = self.next_start.get_mut(&mcu) {
            *cur += (seg_dur_secs * freq).max(0.0) as u64;
        }
    }

    pub fn is_anchored(&self, mcu: u32) -> bool {
        self.next_start.contains_key(&mcu)
    }

    pub fn anchor_tick(&self, mcu: u32) -> Option<u64> {
        self.next_start.get(&mcu).copied()
    }
}

#[cfg(test)]
mod tests;
