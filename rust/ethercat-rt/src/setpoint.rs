//! Per-DC-cycle setpoint ring: the sample-stream executor's storage.
//!
//! One entry per DC cycle, addressed by absolute grid index. The host fills
//! whole runs at its own pace; the cyclic task pops exactly one entry per
//! exchange. Runs abut by construction — a hole, an overlap, a late run or a
//! drained ring under motion is a latched fault, never a pad or a clamp.

use runtime::error::{
    RUNTIME_ERR_INTERNAL_INVARIANT, RUNTIME_ERR_SAMPLE_RATE_MISCONFIGURED,
    RUNTIME_ERR_SAMPLE_RING_FULL, RUNTIME_ERR_SAMPLE_RING_UNDERRUN, RUNTIME_ERR_SAMPLE_RUN_LATE,
    RUNTIME_ERR_SAMPLE_RUN_REJECTED,
};
use runtime::sample_run::SampleRunError;

/// The executor code the endpoint reports in `SampleGridResponse.executor`:
/// the setpoint ring is the only executor there is.
pub const EXECUTOR_SETPOINT_RING: u8 = 1;

/// `ResonanceBuzz` reached a setpoint-ring endpoint: the buzz is generated on
/// the host and streamed through the ring like any other motion.
pub const ERR_BUZZ_IN_RING_MODE: i32 = -838;

/// Ring depth in DC cycles. At the 250 µs default cycle this is 256 ms of
/// motion, past the pump's 100 ms drip lead and its 250 ms post-re-anchor
/// lead, so lead — not depth — stays the binding constraint.
///
/// Depth also has to fit the endpoint's 100 µs command-dispatch budget
/// (`DISPATCH_BUDGET_NS`): a fill is a bounded copy of `count` 16-byte
/// entries per lane, so one full-depth 8-lane frame moves 128 KiB and cannot
/// fit. The host therefore sizes each frame by its lead, and
/// [`MAX_FILL_CYCLES`] caps one lane's share of a single frame so no frame
/// can overrun the budget regardless of what the host asks for.
pub const RING_DEPTH_CYCLES: usize = 1024;

/// Per-lane cap on one `PushSampleRuns` block: 8 lanes × 256 entries × 16 B
/// = 32 KiB of copy, ~10 µs at 3 GB/s, a tenth of the dispatch budget.
pub const MAX_FILL_CYCLES: usize = 256;

/// One DC cycle's commanded state for one drive.
///
/// `pos_counts` is in the lane's anchored count frame (see
/// [`SetpointRing::origin_mm`]); the cyclic task adds the drive-frame origin
/// it latched at the first played entry. `vel_ff` is drive counts per second
/// and `torque_ff` tenths of a percent of rated torque — both already the
/// drive's own command quanta. `acc_mm_s2` is the commanded host-frame accel
/// the pin-rotor oscillator uses as its forcing term; the pin is encoder-fed
/// and therefore stays in the cyclic task, so its regressor has to ride here.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct SetpointEntry {
    pub pos_counts: i32,
    pub vel_ff: i32,
    pub torque_ff: i16,
    pub acc_mm_s2: f32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RingFault {
    /// A run arrived for cycles the ring has already played.
    RunLate {
        deficit_us: u32,
    },
    /// The ring drained while the last played entry still had velocity.
    Underrun {
        tail_vel_counts_s: u32,
    },
    RingFull {
        free_cycles: u32,
        asked: u32,
    },
    Rejected(SampleRunError),
    /// The run's interval is not the DC cycle. Resampling is the host's job;
    /// the cyclic task never interpolates.
    IntervalMismatch {
        expected_ticks: u32,
        got_ticks: u32,
    },
    /// The lane's `pos_counts == 0` reference moved without a re-anchor.
    OriginShift {
        expected_nm: i64,
        got_nm: i64,
    },
    /// The playback grid index went backwards.
    GridRegression {
        play_index: u64,
        grid_index: u64,
    },
    /// A run block asked for more than [`MAX_FILL_CYCLES`] in one frame.
    FillTooLarge {
        asked: u32,
    },
}

impl RingFault {
    #[must_use]
    pub fn code(self) -> i32 {
        match self {
            RingFault::RunLate { .. } => RUNTIME_ERR_SAMPLE_RUN_LATE,
            RingFault::Underrun { .. } => RUNTIME_ERR_SAMPLE_RING_UNDERRUN,
            RingFault::RingFull { .. } | RingFault::FillTooLarge { .. } => {
                RUNTIME_ERR_SAMPLE_RING_FULL
            }
            RingFault::Rejected(_) | RingFault::OriginShift { .. } => {
                RUNTIME_ERR_SAMPLE_RUN_REJECTED
            }
            RingFault::IntervalMismatch { .. } => RUNTIME_ERR_SAMPLE_RATE_MISCONFIGURED,
            RingFault::GridRegression { .. } => RUNTIME_ERR_INTERNAL_INVARIANT,
        }
    }

    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            RingFault::RunLate { .. } => "sample_run_late",
            RingFault::Underrun { .. } => "sample_ring_underrun",
            RingFault::RingFull { .. } => "sample_ring_full",
            RingFault::FillTooLarge { .. } => "sample_fill_too_large",
            RingFault::Rejected(_) => "sample_run_rejected",
            RingFault::IntervalMismatch { .. } => "sample_interval_mismatch",
            RingFault::OriginShift { .. } => "sample_origin_shift",
            RingFault::GridRegression { .. } => "sample_grid_regression",
        }
    }

    /// Detail word carried in the fault register's high half.
    #[must_use]
    pub fn detail(self) -> u32 {
        match self {
            RingFault::RunLate { deficit_us } => deficit_us,
            RingFault::Underrun { tail_vel_counts_s } => tail_vel_counts_s,
            RingFault::RingFull { asked, .. } | RingFault::FillTooLarge { asked } => asked,
            RingFault::Rejected(e) => u32::from(e.fault_code()),
            RingFault::IntervalMismatch { got_ticks, .. } => got_ticks,
            RingFault::OriginShift { .. } | RingFault::GridRegression { .. } => 0,
        }
    }

    /// Endpoint fault-register layout: `detail << 16 | code`, matching the
    /// host's `StatusHeartbeat` decoder.
    #[must_use]
    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    pub fn reg_value(self) -> u32 {
        let code_u16 = (self.code() as i16) as u16;
        let detail_hi16 = self.detail().min(u32::from(u16::MAX)) as u16;
        (u32::from(detail_hi16) << 16) | u32::from(code_u16)
    }
}

/// Result of one cycle's `play`: the entry to command, or why there is none.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Played {
    Entry(SetpointEntry),
    /// The ring is empty at this index — the slot holds its last target.
    Drained,
}

/// One run's header as the endpoint receives it. `final_run` says the run ends
/// the lane's commanded motion — a gap, a stream end, or the end of a buzz —
/// so the ring draining right after it is the expected hold and not a
/// starvation. Any other drain is an underrun.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RunHeader {
    pub start_index: u64,
    pub interval_ticks: u32,
    pub origin_mm: f64,
    pub anchor: bool,
    pub final_run: bool,
}

pub struct SetpointRing {
    entries: Box<[SetpointEntry]>,
    /// Grid index of the first queued entry, and how many follow it.
    queue_base: u64,
    len: usize,
    /// Grid index the next cycle will ask for. It advances every cycle whether
    /// or not an entry was there, so lateness is measured against real time
    /// and never against how far ahead the host has filled.
    play_index: u64,
    anchored: bool,
    origin_mm: Option<f64>,
    interval_ticks: u32,
    tail: Option<SetpointEntry>,
    /// Grid index of the last entry of the most recent run that declared
    /// itself final, and the index of the last entry actually played: the
    /// ring drained legitimately exactly when they agree.
    final_index: Option<u64>,
    last_played: Option<u64>,
    fault: Option<u32>,
    played: u32,
    skipped: u32,
    slot: usize,
}

impl core::fmt::Debug for SetpointRing {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("SetpointRing")
            .field("slot", &self.slot)
            .field("queue_base", &self.queue_base)
            .field("len", &self.len)
            .field("play_index", &self.play_index)
            .field("anchored", &self.anchored)
            .field("played", &self.played)
            .finish()
    }
}

impl SetpointRing {
    #[must_use]
    pub fn new(slot: usize, interval_ticks: u32) -> Self {
        Self {
            entries: vec![SetpointEntry::default(); RING_DEPTH_CYCLES].into_boxed_slice(),
            queue_base: 0,
            len: 0,
            play_index: 0,
            anchored: false,
            origin_mm: None,
            interval_ticks,
            tail: None,
            final_index: None,
            last_played: None,
            fault: None,
            played: 0,
            skipped: 0,
            slot,
        }
    }

    #[must_use]
    pub fn free(&self) -> usize {
        RING_DEPTH_CYCLES - self.len
    }

    #[must_use]
    pub fn len(&self) -> usize {
        self.len
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Host-frame mm that `pos_counts == 0` stands for, for the strain-comp
    /// map's absolute-position lookup.
    #[must_use]
    pub fn origin_mm(&self) -> Option<f64> {
        self.origin_mm
    }

    #[must_use]
    pub fn played_count(&self) -> u32 {
        self.played
    }

    /// The DC grid index this ring last played; every entry at or before it has
    /// left the ring. Zero before the first played cycle.
    #[must_use]
    pub fn playback_clock(&self) -> u64 {
        self.last_played.unwrap_or(0)
    }

    #[must_use]
    pub fn skipped_count(&self) -> u32 {
        self.skipped
    }

    #[must_use]
    pub fn next_index(&self) -> u64 {
        self.queue_base + self.len as u64
    }

    /// Queue one run. `header.anchor` restarts the lane (stream start,
    /// Stop/ResumeStream, homing trip, set_position, a resumed gap);
    /// otherwise the run must abut what is already queued.
    pub fn fill(&mut self, header: &RunHeader, entries: &[SetpointEntry]) -> Result<(), RingFault> {
        let RunHeader {
            start_index,
            interval_ticks,
            origin_mm,
            anchor,
            final_run,
        } = *header;
        if interval_ticks != self.interval_ticks {
            return Err(self.latch(RingFault::IntervalMismatch {
                expected_ticks: self.interval_ticks,
                got_ticks: interval_ticks,
            }));
        }
        if entries.is_empty() {
            return Err(self.latch(RingFault::Rejected(SampleRunError::ZeroCount {
                start_clock: start_index,
            })));
        }
        if entries.len() > MAX_FILL_CYCLES {
            return Err(self.latch(RingFault::FillTooLarge {
                asked: entries.len() as u32,
            }));
        }
        if start_index < self.play_index {
            let deficit_cycles = self.play_index - start_index;
            let deficit_us = deficit_cycles
                .saturating_mul(u64::from(self.interval_ticks) / 1000)
                .min(u64::from(u32::MAX)) as u32;
            return Err(self.latch(RingFault::RunLate { deficit_us }));
        }
        if anchor {
            self.queue_base = start_index;
            self.len = 0;
            self.tail = None;
            self.final_index = None;
            self.anchored = true;
            self.origin_mm = Some(origin_mm);
        } else {
            if !self.anchored {
                return Err(self.latch(RingFault::Rejected(SampleRunError::NotAnchored {
                    start_clock: start_index,
                })));
            }
            if self.origin_mm != Some(origin_mm) {
                let expected = self.origin_mm.unwrap_or(0.0);
                return Err(self.latch(RingFault::OriginShift {
                    expected_nm: (expected * 1e6).round() as i64,
                    got_nm: (origin_mm * 1e6).round() as i64,
                }));
            }
            if start_index != self.next_index() {
                return Err(
                    self.latch(RingFault::Rejected(SampleRunError::Discontinuity {
                        expected_clock: self.next_index(),
                        start_clock: start_index,
                    })),
                );
            }
        }
        if entries.len() > self.free() {
            return Err(self.latch(RingFault::RingFull {
                free_cycles: self.free() as u32,
                asked: entries.len() as u32,
            }));
        }
        let base = self.next_index();
        for (i, entry) in entries.iter().enumerate() {
            let cell = ((base + i as u64) % RING_DEPTH_CYCLES as u64) as usize;
            self.entries[cell] = *entry;
        }
        self.len += entries.len();
        if final_run {
            self.final_index = Some(base + entries.len() as u64 - 1);
        }
        Ok(())
    }

    /// Pop the entry for `grid_index`. Cycles the loop skipped (a policed
    /// overrun keeps the DC grid phase but eats whole cycles) are discarded
    /// here so playback stays on the wall clock instead of replaying stale
    /// setpoints.
    pub fn play(&mut self, grid_index: u64) -> Played {
        if grid_index < self.play_index {
            self.latch(RingFault::GridRegression {
                play_index: self.play_index,
                grid_index,
            });
            return Played::Drained;
        }
        self.play_index = grid_index + 1;
        while self.len > 0 && self.queue_base < grid_index {
            self.queue_base += 1;
            self.len -= 1;
            self.skipped = self.skipped.saturating_add(1);
        }
        if self.len == 0 || self.queue_base != grid_index {
            if let Some(tail) = self.tail.take() {
                if self.final_index != self.last_played {
                    self.latch(RingFault::Underrun {
                        tail_vel_counts_s: tail.vel_ff.unsigned_abs(),
                    });
                }
            }
            return Played::Drained;
        }
        let cell = (grid_index % RING_DEPTH_CYCLES as u64) as usize;
        let entry = self.entries[cell];
        self.queue_base += 1;
        self.len -= 1;
        self.played = self.played.saturating_add(1);
        self.tail = Some(entry);
        self.last_played = Some(grid_index);
        Played::Entry(entry)
    }

    pub fn take_fault(&mut self) -> Option<u32> {
        self.fault.take()
    }

    /// Drop every queued entry and the anchor: the next run must re-anchor.
    /// Invoked on Stop, homing trip and drive fault.
    pub fn reset(&mut self) {
        self.len = 0;
        self.tail = None;
        self.final_index = None;
        self.last_played = None;
        self.anchored = false;
        self.origin_mm = None;
        self.fault = None;
    }

    fn latch(&mut self, fault: RingFault) -> RingFault {
        self.fault.get_or_insert(fault.reg_value());
        fault
    }
}

/// Endpoint fault code for a grid whose phase drifted off the DC period.
pub const GRID_PHASE_FAULT_CODE: i32 = RUNTIME_ERR_INTERNAL_INVARIANT;

/// The DC cycle grid: the sample stream's clock. `g_ts` in the C backend
/// advances by whole cycle periods forever (an overrun re-anchor jumps a
/// whole multiple), so grid indices are exact integers off a base latched
/// once — no rounding, no drift, and a phase residual is a hard invariant
/// break rather than a tolerance.
#[derive(Debug, Clone, Copy)]
pub struct SampleGrid {
    base_mono_ns: u64,
    interval_ns: u64,
    anchored: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GridPhaseError {
    pub mono_ns: u64,
    pub base_mono_ns: u64,
    pub residual_ns: u64,
}

impl SampleGrid {
    #[must_use]
    pub fn new(interval_ns: u64) -> Self {
        Self {
            base_mono_ns: 0,
            interval_ns,
            anchored: false,
        }
    }

    #[must_use]
    pub fn interval_ns(&self) -> u64 {
        self.interval_ns
    }

    /// Grid index of the DC apply point `mono_ns` (CLOCK_MONOTONIC, the
    /// domain `g_ts` lives in). The first call latches the base.
    pub fn index_of(&mut self, mono_ns: u64) -> Result<u64, GridPhaseError> {
        if !self.anchored {
            self.base_mono_ns = mono_ns;
            self.anchored = true;
        }
        let elapsed = mono_ns.saturating_sub(self.base_mono_ns);
        let residual_ns = elapsed % self.interval_ns;
        if residual_ns != 0 {
            return Err(GridPhaseError {
                mono_ns,
                base_mono_ns: self.base_mono_ns,
                residual_ns,
            });
        }
        Ok(elapsed / self.interval_ns)
    }
}

#[cfg(test)]
mod tests;
