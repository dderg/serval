//! Host-side filler for the setpoint ring.
//!
//! Runs on the pump's thread, never on the DC thread. It evaluates the
//! trajectory's clocked motor spans on the endpoint's DC grid and computes
//! everything the cyclic task would otherwise compute per cycle: the anchored
//! count target, the velocity feedforward, and the coupled dynamics torque
//! feedforward. What stays in the cyclic task is what needs this cycle's
//! encoder image — the damper, the trim, the strain comp and the pin.
//!
//! The grid is the endpoint's: [`ChainFiller::observe_grid`] takes the
//! `(grid_index, grid_clock)` pair every `PushSampleRunsResponse` echoes, so
//! index `n`'s sample clock is `grid_clock + (n - grid_index) * interval`.
//!
//! A lane holds at most two views: the one the fill is converting and its
//! successor, which the feedforward lead reaches across. A view leaves the
//! active slot when the fill has converted past its end — that is
//! consumption. It leaves the filler entirely, dropping the host's `Arc` on
//! the signal, only once the endpoint proves it played past that end — that
//! is retirement. A cut abandons whatever is unresolved and credits neither.

use std::collections::VecDeque;

use mcu_protocol::messages::{LaneRun, SetpointSample, LANE_RUN_FLAG_REANCHOR, LANE_RUN_FLAG_TAIL};
use trajectory::ClockedMotorSpan;

use crate::buzz::{BuzzOsc, MAX_BUZZ_SLOTS};
use crate::dynamics::DynamicsModel;
use crate::scale::mm_to_counts;
use crate::setpoint::MAX_FILL_CYCLES;

/// The DC grid is stamped in nanoseconds, so a span's clock map must tick at
/// 1 GHz for its clocks to be this grid's clocks.
pub const CLOCK_FREQ_HZ: f64 = 1_000_000_000.0;

/// Views one lane may hold at once: the active one plus the successor the
/// feedforward lead reaches into.
pub const LANE_SPAN_SLOTS: usize = 2;

/// The pump anchors every route of one sweep on the same instant, so a buzz
/// can only be armed once the endpoint has echoed a grid the anchor can be
/// placed on.
pub const ERR_BUZZ_UNGRIDDED_START: i32 = -839;

/// The anchor the pump handed down already played: the grid the endpoint last
/// echoed is past it, so no cycle of this sweep could carry its first sample.
pub const ERR_BUZZ_START_IN_PAST: i32 = -840;

/// A feedforward reconfiguration reached the filler while it still owed the
/// endpoint samples, or while samples it already emitted had not played.
pub const ERR_RECONFIG_STREAMING: i32 = -841;

/// The reconfigured slot is not a lane of this chain.
pub const ERR_RECONFIG_UNKNOWN_SLOT: i32 = -842;

/// The offered dynamics model covers a different number of slots than the
/// chain has lanes.
pub const ERR_RECONFIG_BAD_DIM: i32 = -843;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LaneSpec {
    pub axis: u8,
    pub cmd_counts_per_mm: f64,
    pub ff_lead_ns: u64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FillError {
    NonFiniteTorque {
        slot: usize,
        acc: f32,
        vel: f32,
    },
    GridUnobserved,
    GridRegression {
        observed: u64,
        previous: u64,
    },
    SpanSlotsFull {
        axis: u8,
    },
    SpanClockMismatch {
        axis: u8,
        clock_freq_hz: f64,
    },
    SpanOutOfOrder {
        axis: u8,
        start_clock: u64,
        previous_end: u64,
    },
    SpanEval {
        axis: u8,
        clock: u64,
    },
}

impl FillError {
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            FillError::NonFiniteTorque { .. } => "sample_fill_non_finite_torque",
            FillError::GridUnobserved => "sample_fill_grid_unobserved",
            FillError::GridRegression { .. } => "sample_fill_grid_regression",
            FillError::SpanSlotsFull { .. } => "sample_fill_span_slots_full",
            FillError::SpanClockMismatch { .. } => "sample_fill_span_clock_mismatch",
            FillError::SpanOutOfOrder { .. } => "sample_fill_span_out_of_order",
            FillError::SpanEval { .. } => "sample_fill_span_eval",
        }
    }
}

struct Lane {
    spec: LaneSpec,
    active: Option<ClockedMotorSpan>,
    successor: Option<ClockedMotorSpan>,
    /// Converted views the endpoint has not yet proven it played.
    released: VecDeque<ClockedMotorSpan>,
    consumed: usize,
    /// Host mm that `pos_counts == 0` stands for in the current epoch. An
    /// epoch starts at every re-anchor and never shifts inside one.
    origin_mm: Option<f64>,
    /// Grid index the next sample must carry to abut the last one emitted.
    next_index: Option<u64>,
}

impl Lane {
    fn new(spec: LaneSpec) -> Self {
        Self {
            spec,
            active: None,
            successor: None,
            released: VecDeque::new(),
            consumed: 0,
            origin_mm: None,
            next_index: None,
        }
    }

    fn tail_end_clock(&self) -> Option<u64> {
        self.successor
            .as_ref()
            .or(self.active.as_ref())
            .map(|span| span.end_clock)
    }

    fn free_slots(&self) -> usize {
        LANE_SPAN_SLOTS - usize::from(self.active.is_some()) - usize::from(self.successor.is_some())
    }

    fn admit(&mut self, span: &ClockedMotorSpan) -> Result<(), FillError> {
        let axis = self.spec.axis;
        if span.clock_freq_hz != CLOCK_FREQ_HZ {
            return Err(FillError::SpanClockMismatch {
                axis,
                clock_freq_hz: span.clock_freq_hz,
            });
        }
        if let Some(previous_end) = self.tail_end_clock() {
            if span.start_clock < previous_end {
                return Err(FillError::SpanOutOfOrder {
                    axis,
                    start_clock: span.start_clock,
                    previous_end,
                });
            }
        }
        let slot = if self.active.is_none() {
            &mut self.active
        } else if self.successor.is_none() {
            &mut self.successor
        } else {
            return Err(FillError::SpanSlotsFull { axis });
        };
        *slot = Some(span.clone());
        Ok(())
    }

    /// Move the cursor past every view the fill has converted through. The
    /// released view stays owned until the endpoint proves the playback.
    fn consume_through(&mut self, clock: u64) {
        while self
            .active
            .as_ref()
            .is_some_and(|span| span.end_clock <= clock)
        {
            let span = self.active.take().expect("checked above");
            self.released.push_back(span);
            self.consumed += 1;
            self.active = self.successor.take();
        }
    }

    fn covering(&self, clock: u64) -> Option<&ClockedMotorSpan> {
        [self.active.as_ref(), self.successor.as_ref()]
            .into_iter()
            .flatten()
            .find(|span| clock >= span.start_clock && clock < span.end_clock)
    }

    #[allow(clippy::cast_possible_truncation)]
    fn eval(&self, clock: u64) -> Result<Option<(f64, f32, f32)>, FillError> {
        match self.covering(clock) {
            None => Ok(None),
            Some(span) => {
                let pva = span.eval_at_clock(clock).map_err(|_| FillError::SpanEval {
                    axis: self.spec.axis,
                    clock,
                })?;
                Ok(Some((
                    pva.position,
                    pva.velocity as f32,
                    pva.acceleration as f32,
                )))
            }
        }
    }

    /// Feedforward lookahead: commanded `(vel, acc)` a lead ahead of the
    /// position cursor, reaching across the successor and never consuming a
    /// view. A lead landing in a gap or past the stream end is a stationary
    /// target.
    fn lead_vel_acc(&self, clock: u64) -> Result<(f32, f32), FillError> {
        Ok(self
            .eval(clock)?
            .map_or((0.0, 0.0), |(_, vel, acc)| (vel, acc)))
    }

    fn has_pending(&self) -> bool {
        self.active.is_some()
    }

    fn retire_through(&mut self, played_clock: u64) -> usize {
        let mut retired = 0;
        while self
            .released
            .front()
            .is_some_and(|span| span.end_clock <= played_clock)
        {
            self.released.pop_front();
            retired += 1;
        }
        retired
    }

    /// Drop everything the lane holds without crediting retirement: the
    /// endpoint discarded this motion, so nothing here was ever played.
    fn abandon(&mut self) -> usize {
        let abandoned = usize::from(self.active.is_some())
            + usize::from(self.successor.is_some())
            + self.released.len();
        self.active = None;
        self.successor = None;
        self.released.clear();
        self.origin_mm = None;
        self.next_index = None;
        abandoned
    }
}

/// One lane's state for a single sampled instant.
#[derive(Debug, Clone, Copy, Default)]
struct LaneSample {
    pos_mm: Option<f64>,
    vel_host: f32,
    acc_host: f32,
}

/// One drive chain's filler: every lane the endpoint drives, plus the coupled
/// dynamics model, because a slot's torque feedforward needs every slot's
/// commanded kinematics at the same instant.
pub struct ChainFiller {
    lanes: Vec<Lane>,
    dynamics: Option<DynamicsModel>,
    drive_dirs: Vec<f32>,
    interval_ns: u64,
    /// Cycles ahead of the observed grid index a freshly anchored epoch
    /// starts. The pump's lead expressed in DC cycles.
    lead_cycles: u64,
    grid: Option<(u64, u64)>,
    buzz: BuzzOsc,
    buzz_next_index: Option<u64>,
    samples: Vec<LaneSample>,
    acc_drive: Vec<f32>,
    vel_drive: Vec<f32>,
    buzz_slots: Vec<bool>,
    runs: Vec<Option<LaneRun>>,
    closed: Vec<bool>,
}

impl std::fmt::Debug for ChainFiller {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ChainFiller")
            .field("lanes", &self.lanes.len())
            .field("interval_ns", &self.interval_ns)
            .field("lead_cycles", &self.lead_cycles)
            .field("grid", &self.grid)
            .finish()
    }
}

impl ChainFiller {
    #[must_use]
    #[allow(clippy::cast_possible_truncation)]
    pub fn new(
        specs: &[LaneSpec],
        dynamics: Option<DynamicsModel>,
        interval_ns: u64,
        lead_cycles: u64,
    ) -> Self {
        assert!(
            u8::try_from(specs.len()).is_ok(),
            "an endpoint's drive slots must fit the wire's u8 slot index"
        );
        let n = specs.len();
        Self {
            lanes: specs.iter().copied().map(Lane::new).collect(),
            drive_dirs: specs
                .iter()
                .map(|s| s.cmd_counts_per_mm.signum() as f32)
                .collect(),
            dynamics,
            interval_ns,
            lead_cycles,
            grid: None,
            buzz: BuzzOsc::new(),
            buzz_next_index: None,
            buzz_slots: vec![false; n],
            samples: vec![LaneSample::default(); n],
            acc_drive: vec![0.0; n],
            vel_drive: vec![0.0; n],
            runs: vec![None; n],
            closed: vec![false; n],
        }
    }

    /// Adopt the endpoint's latest `(grid_index, grid_clock)` pair. The DC
    /// grid only ever advances, so an index below the last observed one means
    /// the endpoint re-anchored its grid under a stream the host still
    /// believes in — the samples already staged against the old pair would
    /// land at the wrong cycles, so it terminates the fill.
    pub fn observe_grid(&mut self, grid_index: u64, grid_clock: u64) -> Result<(), FillError> {
        if let Some((previous, _)) = self.grid {
            if grid_index < previous {
                return Err(FillError::GridRegression {
                    observed: grid_index,
                    previous,
                });
            }
        }
        self.grid = Some((grid_index, grid_clock));
        Ok(())
    }

    #[must_use]
    pub fn lane_count(&self) -> usize {
        self.lanes.len()
    }

    #[must_use]
    pub fn drives_axis(&self, axis: u8) -> bool {
        self.lanes.iter().any(|lane| lane.spec.axis == axis)
    }

    /// Drop one axis' unresolved views and its anchor epoch. The endpoint has
    /// discarded that lane's motion (`Stop`, homing trip, abort), so nothing
    /// staged against the old epoch may still reach the ring and the run that
    /// resumes the lane must carry the re-anchor flag. Returns the abandoned
    /// view count; none of them are retired.
    pub fn cut_axis(&mut self, axis: u8) -> usize {
        self.lanes
            .iter_mut()
            .filter(|l| l.spec.axis == axis)
            .map(Lane::abandon)
            .sum()
    }

    /// Stage clocked views on a lane. The host keeps the `Arc` on each
    /// signal until retirement, so the wire only ever carries the fixed
    /// `LaneRun` values these views evaluate to.
    pub fn push_spans(&mut self, axis: u8, spans: &[ClockedMotorSpan]) -> Result<(), FillError> {
        for span in spans {
            for lane in self.lanes.iter_mut().filter(|l| l.spec.axis == axis) {
                lane.admit(span)?;
            }
        }
        Ok(())
    }

    /// Views this axis can still take. The scheduler's `spans_per_axis`
    /// budget for an EtherCAT lane is exactly this.
    #[must_use]
    pub fn free_span_slots(&self, axis: u8) -> usize {
        self.lanes
            .iter()
            .filter(|l| l.spec.axis == axis)
            .map(Lane::free_slots)
            .min()
            .unwrap_or(0)
    }

    /// Views fully converted into `LaneRun` samples and released from the
    /// active cursor since the last call.
    pub fn take_consumed(&mut self, axis: u8) -> usize {
        self.lanes
            .iter_mut()
            .filter(|l| l.spec.axis == axis)
            .map(|lane| std::mem::take(&mut lane.consumed))
            .sum()
    }

    /// Drop every released view the endpoint has proven it played past,
    /// reclaiming the host's `Arc` on each signal.
    pub fn retire_through(&mut self, axis: u8, played_clock: u64) -> usize {
        self.lanes
            .iter_mut()
            .filter(|l| l.spec.axis == axis)
            .map(|lane| lane.retire_through(played_clock))
            .sum()
    }

    /// Arm a host-generated buzz on the instant the pump anchored the whole
    /// sweep on. The buzz is a sample source like any other: it fills the
    /// same ring through the same runs, so the endpoint has no buzz
    /// evaluation of its own. `start_clock_ns` is that anchor on this node's
    /// DC clock; the sweep opens on the first grid cycle at or after it, so
    /// every transport of one arming starts at the same instant snapped to
    /// its own device grid. Rejected while any lane still has trajectory
    /// queued: the sweep replaces the whole window, suppressing every
    /// undriven lane for its full duration, so an unrelated lane's queued
    /// motion would be swallowed instead of played.
    #[allow(clippy::too_many_arguments, clippy::cast_possible_truncation)]
    pub fn arm_buzz(
        &mut self,
        slot_mask: u8,
        sign_mask: u8,
        freq_start_millihz: u32,
        freq_end_millihz: u32,
        amplitude_nm: u32,
        duration_ms: u32,
        ramp_ms: u32,
        start_clock_ns: u64,
    ) -> i32 {
        if self.buzz.active() {
            return crate::buzz::ERR_BUZZ_BUSY;
        }
        if self.lanes.iter().any(Lane::has_pending) {
            return crate::buzz::ERR_BUZZ_STREAMING;
        }
        let driven: Vec<bool> = (0..self.lanes.len())
            .map(|slot| slot < MAX_BUZZ_SLOTS && slot_mask & (1 << slot) != 0)
            .collect();
        let Some((_, grid_clock)) = self.grid else {
            return ERR_BUZZ_UNGRIDDED_START;
        };
        if start_clock_ns < grid_clock {
            return ERR_BUZZ_START_IN_PAST;
        }
        let Some(start_index) = self.index_at_or_after(start_clock_ns) else {
            return ERR_BUZZ_UNGRIDDED_START;
        };
        let rc = self.buzz.arm(
            self.lanes.len() as u8,
            slot_mask,
            sign_mask,
            freq_start_millihz,
            freq_end_millihz,
            amplitude_nm,
            duration_ms,
            ramp_ms,
            [0; MAX_BUZZ_SLOTS],
        );
        if rc != 0 {
            return rc;
        }
        self.open_buzz_epoch(driven, start_index);
        0
    }

    /// The buzz is its own anchor epoch: it starts from whatever the lane is
    /// holding, so the trajectory epoch must not continue into it.
    fn open_buzz_epoch(&mut self, driven: Vec<bool>, start_index: u64) {
        for (slot, lane) in self.lanes.iter_mut().enumerate() {
            if driven.get(slot) == Some(&true) {
                lane.next_index = None;
                lane.origin_mm = None;
            }
        }
        self.buzz_slots = driven;
        self.buzz_next_index = Some(start_index);
    }

    #[must_use]
    pub fn buzz_active(&self) -> bool {
        self.buzz.active()
    }

    /// True while the filler still owes the ring samples — the pump has to
    /// keep draining, since a buzz outlives one frame's worth of cycles.
    #[must_use]
    pub fn wants_drain(&self) -> bool {
        self.buzz.active() || self.lanes.iter().any(Lane::has_pending)
    }

    /// Nothing the endpoint can still play is outstanding: no view is staged,
    /// no buzz is armed, and the grid the endpoint last reported has passed
    /// every sample the lanes emitted. Only here does a feedforward change
    /// land whole — the samples already on the wire carry the model they were
    /// computed with and the endpoint plays them unchanged.
    #[must_use]
    pub fn quiescent(&self) -> bool {
        let Some((grid_index, _)) = self.grid else {
            return false;
        };
        !self.wants_drain()
            && self
                .lanes
                .iter()
                .all(|lane| lane.next_index.is_none_or(|next| next <= grid_index))
    }

    /// Retarget one lane's feedforward lead. The lead shifts every sample's
    /// velocity and torque feedforward, so applying it to the tail of a
    /// stream would step both mid-motion.
    pub fn set_ff_lead(&mut self, slot: usize, lead_ns: u64) -> i32 {
        if slot >= self.lanes.len() {
            return ERR_RECONFIG_UNKNOWN_SLOT;
        }
        if !self.quiescent() {
            return ERR_RECONFIG_STREAMING;
        }
        self.lanes[slot].spec.ff_lead_ns = lead_ns;
        0
    }

    /// Swap the coupled dynamics model every lane's torque feedforward is
    /// computed from. One model covers the whole chain, so a swap that
    /// reached only part of a stream would leave the coupling terms of one
    /// motion computed from two different models.
    pub fn install_dynamics(&mut self, model: DynamicsModel) -> i32 {
        if model.n_slots != self.lanes.len() {
            return ERR_RECONFIG_BAD_DIM;
        }
        if !self.quiescent() {
            return ERR_RECONFIG_STREAMING;
        }
        self.dynamics = Some(model);
        0
    }

    /// Abandon every view, the buzz and every anchor: the next run on each
    /// lane must re-anchor. The Stop / homing-trip / drive-fault path.
    /// Nothing abandoned here is credited as retired.
    pub fn reset(&mut self) -> usize {
        let abandoned = self.lanes.iter_mut().map(Lane::abandon).sum();
        self.buzz.clear();
        self.buzz_next_index = None;
        abandoned
    }

    fn clock_of(&self, index: u64) -> Option<u64> {
        let (grid_index, grid_clock) = self.grid?;
        let delta = i128::from(index) - i128::from(grid_index);
        let clock = i128::from(grid_clock) + delta * i128::from(self.interval_ns);
        u64::try_from(clock).ok()
    }

    fn index_at_or_after(&self, clock: u64) -> Option<u64> {
        let (grid_index, grid_clock) = self.grid?;
        let ahead = clock.saturating_sub(grid_clock);
        let whole = ahead / self.interval_ns;
        Some(grid_index + whole + u64::from(ahead % self.interval_ns != 0))
    }

    /// Sample every lane over the next window and return one contiguous run
    /// per lane. A lane whose coverage ends inside the window closes there;
    /// the run that resumes it carries the re-anchor flag.
    pub fn drain(&mut self) -> Result<Vec<LaneRun>, FillError> {
        let Some(start) = self.window_start() else {
            return Ok(Vec::new());
        };
        for slot in 0..self.lanes.len() {
            self.runs[slot] = None;
            self.closed[slot] = false;
        }
        let buzzing = self.buzz.active();
        let mut window_exhausted = false;
        for step in 0..MAX_FILL_CYCLES as u64 {
            let index = start + step;
            let Some(clock) = self.clock_of(index) else {
                return Err(FillError::GridUnobserved);
            };
            if !self.sample_chain(clock)? {
                window_exhausted = true;
                break;
            }
            self.fill_drive_frame();
            self.append_samples(index)?;
        }
        for slot in 0..self.lanes.len() {
            if !(window_exhausted || self.closed[slot]) {
                continue;
            }
            if let Some(run) = &mut self.runs[slot] {
                run.flags |= LANE_RUN_FLAG_TAIL;
            }
        }
        if buzzing && !self.buzz.active() {
            self.close_buzz_epoch();
        }
        Ok(self.runs.iter_mut().filter_map(Option::take).collect())
    }

    /// A finished buzz ends its epoch: the trajectory that follows anchors
    /// afresh instead of continuing the buzz's zero-origin count frame.
    fn close_buzz_epoch(&mut self) {
        for (slot, lane) in self.lanes.iter_mut().enumerate() {
            if self.buzz_slots.get(slot) == Some(&true) {
                lane.next_index = None;
                lane.origin_mm = None;
            }
        }
        self.buzz_next_index = None;
    }

    fn window_start(&self) -> Option<u64> {
        if self.buzz.active() {
            return self.buzz_window_start();
        }
        let mut earliest: Option<u64> = None;
        for lane in &self.lanes {
            let candidate = match lane.next_index {
                Some(index) => Some(index),
                None => lane
                    .active
                    .as_ref()
                    .and_then(|span| self.index_at_or_after(span.start_clock)),
            };
            if let Some(index) = candidate {
                earliest = Some(earliest.map_or(index, |e: u64| e.min(index)));
            }
        }
        earliest
    }

    /// A buzz owns the whole window: [`ChainFiller::sample_buzz`] suppresses
    /// every undriven lane anyway, so letting one of them open the window
    /// earlier would only hand the oscillator a first cycle before the
    /// instant the sweep was armed on.
    fn buzz_window_start(&self) -> Option<u64> {
        self.lanes
            .iter()
            .enumerate()
            .filter(|(slot, _)| self.buzz_slots.get(*slot) == Some(&true))
            .filter_map(|(_, lane)| lane.next_index.or(self.buzz_next_index))
            .min()
    }

    /// Host-frame position and lead-shifted velocity/accel of every lane at
    /// `clock`. Returns false once no lane can still extend an open run — a
    /// lane closed by a coverage gap keeps its remaining views for the next
    /// drain, which re-anchors them.
    fn sample_chain(&mut self, clock: u64) -> Result<bool, FillError> {
        if self.buzz.active() {
            return Ok(self.sample_buzz(clock));
        }
        let mut any = false;
        for slot in 0..self.lanes.len() {
            if self.closed[slot] {
                self.samples[slot] = LaneSample::default();
                continue;
            }
            let lane = &mut self.lanes[slot];
            lane.consume_through(clock);
            let lane = &self.lanes[slot];
            let Some((pos_mm, vel_mm_s, acc_mm_s2)) = lane.eval(clock)? else {
                let pending = lane.tail_end_clock().is_some_and(|end| clock < end);
                self.samples[slot] = LaneSample::default();
                any |= pending;
                continue;
            };
            let lead = lane.spec.ff_lead_ns;
            let (ff_vel, ff_acc) = if lead > 0 {
                lane.lead_vel_acc(clock + lead)?
            } else {
                (vel_mm_s, acc_mm_s2)
            };
            self.samples[slot] = LaneSample {
                pos_mm: Some(pos_mm),
                vel_host: ff_vel,
                acc_host: ff_acc,
            };
            any = true;
        }
        Ok(any)
    }

    fn sample_buzz(&mut self, clock: u64) -> bool {
        let tone = self.buzz.eval(clock);
        let mut any = false;
        for slot in 0..self.lanes.len() {
            let driven = self.buzz.drives_slot(slot);
            self.samples[slot] = match tone.filter(|_| driven) {
                Some((rel_mm, vel_mm_s, acc_mm_s2)) => {
                    let sign = self.buzz.slot_sign(slot);
                    any = true;
                    LaneSample {
                        pos_mm: Some(f64::from(sign * rel_mm)),
                        vel_host: sign * vel_mm_s,
                        acc_host: sign * acc_mm_s2,
                    }
                }
                None => LaneSample::default(),
            };
        }
        any
    }

    /// The dynamics profile is fitted in the drive frame (the capture flips
    /// each drive's commanded kinematics by its direction sign), so the model
    /// must be evaluated on drive-frame vectors — flipping only the output
    /// torque by the slot's own sign would negate the off-diagonal coupling
    /// terms whenever the drives' inverts differ.
    fn fill_drive_frame(&mut self) {
        for slot in 0..self.lanes.len() {
            let dir = self.drive_dirs[slot];
            self.acc_drive[slot] = dir * self.samples[slot].acc_host;
            self.vel_drive[slot] = dir * self.samples[slot].vel_host;
        }
    }

    /// Coulomb drops for the whole chain during a buzz: it is a mode-space
    /// quantity, and a buzz on one slot flips the sign of a mode velocity the
    /// others share.
    #[allow(clippy::cast_possible_truncation)]
    fn torque_tenths(&self, slot: usize) -> Result<i16, FillError> {
        let Some(model) = self.dynamics.as_ref() else {
            return Ok(0);
        };
        let raw = if self.buzz.active() {
            model.torque_ff_without_coulomb(slot, &self.acc_drive, &self.vel_drive)
        } else {
            model.torque_ff(slot, &self.acc_drive, &self.vel_drive)
        };
        if !raw.is_finite() {
            return Err(FillError::NonFiniteTorque {
                slot,
                acc: self.acc_drive[slot],
                vel: self.vel_drive[slot],
            });
        }
        Ok(raw.round().clamp(f32::from(i16::MIN), f32::from(i16::MAX)) as i16)
    }

    #[allow(clippy::cast_possible_truncation)]
    fn append_samples(&mut self, index: u64) -> Result<(), FillError> {
        for slot in 0..self.lanes.len() {
            let Some(pos_mm) = self.samples[slot].pos_mm else {
                self.closed[slot] = self.runs[slot].is_some();
                continue;
            };
            if self.closed[slot] {
                continue;
            }
            let torque_ff = self.torque_tenths(slot)?;
            let starting = self.runs[slot].is_none();
            let anchor = starting && self.lanes[slot].next_index != Some(index);
            if anchor {
                // A buzz's samples are already relative to the position the
                // lane is holding, so its epoch origin is zero — the endpoint
                // adds the last commanded target as the count base.
                let epoch_origin = if self.buzz.active() { 0.0 } else { pos_mm };
                self.lanes[slot].origin_mm = Some(epoch_origin);
            }
            let lane = &mut self.lanes[slot];
            let cpm = lane.spec.cmd_counts_per_mm;
            let origin_mm = *lane.origin_mm.get_or_insert(pos_mm);
            let sample = SetpointSample {
                pos_counts: mm_to_counts(pos_mm - origin_mm, cpm),
                vel_ff: (f64::from(self.samples[slot].vel_host) * cpm).round() as i32,
                torque_ff,
                acc_mm_s2: self.samples[slot].acc_host,
            };
            match &mut self.runs[slot] {
                Some(run) => run.samples.push(sample),
                None => {
                    self.runs[slot] = Some(LaneRun {
                        axis_idx: lane.spec.axis,
                        slot_idx: slot as u8,
                        flags: if anchor { LANE_RUN_FLAG_REANCHOR } else { 0 },
                        origin_mm_q16: (origin_mm * 65536.0).round() as i32,
                        start_index: index,
                        interval_ticks: self.interval_ns as u32,
                        samples: vec![sample],
                    });
                }
            }
            self.lanes[slot].next_index = Some(index + 1);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests;
