//! Host-side filler for the setpoint ring.
//!
//! Runs on the pump's thread, never on the DC thread. It samples the piece
//! trajectory on the endpoint's DC grid and computes everything the cyclic
//! task would otherwise compute per cycle: the anchored count target, the
//! velocity feedforward, and the coupled dynamics torque feedforward. What
//! stays in the cyclic task is what needs this cycle's encoder image — the
//! damper, the trim, the strain comp and the pin.
//!
//! The grid is the endpoint's: [`ChainFiller::observe_grid`] takes the
//! `(grid_index, grid_clock)` pair every `PushSampleRunsResponse` echoes, so
//! index `n`'s sample clock is `grid_clock + (n - grid_index) * interval`.

use std::collections::VecDeque;

use mcu_protocol::messages::{LaneRun, SetpointSample, LANE_RUN_FLAG_REANCHOR, LANE_RUN_FLAG_TAIL};
use runtime::motion_core::{arm_piece, ArmedPiece};
use runtime::piece_ring::PieceEntry;

use crate::buzz::{BuzzOsc, MAX_BUZZ_SLOTS};
use crate::dynamics::DynamicsModel;
use crate::scale::mm_to_counts;
use crate::setpoint::MAX_FILL_CYCLES;

/// Piece timestamps are nanoseconds, so the piece clock ticks at 1 GHz.
pub const CLOCK_FREQ_HZ: f32 = 1_000_000_000.0;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LaneSpec {
    pub axis: u8,
    pub cmd_counts_per_mm: f64,
    pub ff_lead_ns: u64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FillError {
    NonFiniteTorque { slot: usize, acc: f32, vel: f32 },
    GridUnobserved,
    GridRegression { observed: u64, previous: u64 },
}

impl FillError {
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            FillError::NonFiniteTorque { .. } => "sample_fill_non_finite_torque",
            FillError::GridUnobserved => "sample_fill_grid_unobserved",
            FillError::GridRegression { .. } => "sample_fill_grid_regression",
        }
    }
}

struct Lane {
    spec: LaneSpec,
    pieces: VecDeque<PieceEntry>,
    /// Host mm that `pos_counts == 0` stands for in the current epoch. An
    /// epoch starts at every re-anchor and never shifts inside one.
    origin_mm: Option<f64>,
    /// Grid index the next sample must carry to abut the last one emitted.
    next_index: Option<u64>,
    lookahead: Option<ArmedPiece>,
}

impl Lane {
    fn new(spec: LaneSpec) -> Self {
        Self {
            spec,
            pieces: VecDeque::new(),
            origin_mm: None,
            next_index: None,
            lookahead: None,
        }
    }

    fn retire_before(&mut self, clock: u64) {
        while let Some(front) = self.pieces.front() {
            if front.end_time(CLOCK_FREQ_HZ) > clock {
                break;
            }
            self.pieces.pop_front();
        }
    }

    fn piece_covering(&self, clock: u64) -> Option<ArmedPiece> {
        for entry in &self.pieces {
            if clock < entry.start_time {
                return None;
            }
            if clock < entry.end_time(CLOCK_FREQ_HZ) {
                return Some(arm_piece(entry, CLOCK_FREQ_HZ));
            }
        }
        None
    }

    /// Feedforward lookahead: commanded `(vel, acc)` a lead ahead of the
    /// position cursor, cached across samples and never retiring a piece. A
    /// lead landing in a gap or past the stream end is a stationary target.
    fn lead_vel_acc(&mut self, clock: u64) -> (f32, f32) {
        let covers = |p: &ArmedPiece| clock >= p.piece_start_cycles && clock < p.piece_end_cycles;
        if !self.lookahead.as_ref().is_some_and(covers) {
            self.lookahead = self.piece_covering(clock);
        }
        match &self.lookahead {
            Some(p) => (p.eval_pos_vel(clock).1, p.eval_accel(clock)),
            None => (0.0, 0.0),
        }
    }

    fn stream_end(&self) -> Option<u64> {
        self.pieces.back().map(|e| e.end_time(CLOCK_FREQ_HZ))
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
    pub fn drives_axis(&self, axis: u8) -> bool {
        self.lanes.iter().any(|lane| lane.spec.axis == axis)
    }

    /// Drop one axis' staged pieces and its anchor epoch. The endpoint has
    /// discarded that lane's motion (`Stop`, homing trip, abort), so nothing
    /// staged against the old epoch may still reach the ring and the run that
    /// resumes the lane must carry the re-anchor flag.
    pub fn cut_axis(&mut self, axis: u8) {
        for lane in self.lanes.iter_mut().filter(|l| l.spec.axis == axis) {
            lane.pieces.clear();
            lane.origin_mm = None;
            lane.next_index = None;
            lane.lookahead = None;
        }
    }

    pub fn push_pieces(&mut self, axis: u8, pieces: &[PieceEntry]) {
        for lane in self.lanes.iter_mut().filter(|l| l.spec.axis == axis) {
            lane.pieces.extend(pieces.iter().copied());
        }
    }

    /// Arm a host-generated buzz. The buzz is a sample source like any other:
    /// it fills the same ring through the same runs, so the endpoint has no
    /// buzz evaluation of its own. Rejected while a driven lane still has
    /// trajectory queued, exactly as the endpoint's own oscillator was.
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
    ) -> i32 {
        if self.buzz.active() {
            return crate::buzz::ERR_BUZZ_BUSY;
        }
        let driven: Vec<bool> = (0..self.lanes.len())
            .map(|slot| slot < MAX_BUZZ_SLOTS && slot_mask & (1 << slot) != 0)
            .collect();
        if driven
            .iter()
            .zip(&self.lanes)
            .any(|(driven, lane)| *driven && !lane.pieces.is_empty())
        {
            return crate::buzz::ERR_BUZZ_STREAMING;
        }
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
        // The buzz is its own anchor epoch: it starts from whatever the lane
        // is holding, so the trajectory epoch must not continue into it.
        for (slot, lane) in self.lanes.iter_mut().enumerate() {
            if driven.get(slot) == Some(&true) {
                lane.next_index = None;
                lane.origin_mm = None;
                lane.lookahead = None;
            }
        }
        self.buzz_slots = driven;
        self.buzz_next_index = self.grid.map(|(index, _)| index + self.lead_cycles);
        0
    }

    #[must_use]
    pub fn buzz_active(&self) -> bool {
        self.buzz.active()
    }

    /// True while the filler still owes the ring samples — the pump has to
    /// keep draining, since a buzz outlives one frame's worth of cycles.
    #[must_use]
    pub fn wants_drain(&self) -> bool {
        self.buzz.active() || self.lanes.iter().any(|l| !l.pieces.is_empty())
    }

    /// Drop every queued piece, the buzz and every anchor: the next run on
    /// each lane must re-anchor. The Stop / homing-trip / drive-fault path.
    pub fn reset(&mut self) {
        for lane in &mut self.lanes {
            lane.pieces.clear();
            lane.origin_mm = None;
            lane.next_index = None;
            lane.lookahead = None;
        }
        self.buzz.clear();
        self.buzz_next_index = None;
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
            if !self.sample_chain(clock) {
                window_exhausted = true;
                break;
            }
            self.fill_drive_frame();
            self.append_samples(index)?;
        }
        for slot in 0..self.lanes.len() {
            let tail = window_exhausted || self.closed[slot];
            if let Some(run) = &mut self.runs[slot] {
                run.sample_count = u16::try_from(run.samples.len()).unwrap_or(u16::MAX);
                if tail {
                    run.flags |= LANE_RUN_FLAG_TAIL;
                }
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
                lane.lookahead = None;
            }
        }
        self.buzz_next_index = None;
    }

    fn window_start(&self) -> Option<u64> {
        let mut earliest: Option<u64> = None;
        for (slot, lane) in self.lanes.iter().enumerate() {
            let buzz_driven = self.buzz.active() && self.buzz_slots.get(slot) == Some(&true);
            let candidate = match (lane.next_index, buzz_driven) {
                (Some(index), _) => Some(index),
                (None, true) => self.buzz_next_index,
                (None, false) => lane
                    .pieces
                    .front()
                    .and_then(|p| self.index_at_or_after(p.start_time)),
            };
            if let Some(index) = candidate {
                earliest = Some(earliest.map_or(index, |e: u64| e.min(index)));
            }
        }
        earliest
    }

    /// Host-frame position and lead-shifted velocity/accel of every lane at
    /// `clock`. Returns false once no lane can still extend an open run — a
    /// lane closed by a coverage gap keeps its remaining pieces for the next
    /// drain, which re-anchors them.
    fn sample_chain(&mut self, clock: u64) -> bool {
        if self.buzz.active() {
            return self.sample_buzz(clock);
        }
        let mut any = false;
        for slot in 0..self.lanes.len() {
            if self.closed[slot] {
                self.samples[slot] = LaneSample::default();
                continue;
            }
            let lane = &mut self.lanes[slot];
            lane.retire_before(clock);
            let Some(armed) = lane.piece_covering(clock) else {
                let pending = lane.stream_end().is_some_and(|end| clock < end);
                self.samples[slot] = LaneSample::default();
                any |= pending;
                continue;
            };
            let (pos_mm, vel_mm_s) = armed.eval_pos_vel(clock);
            let acc_mm_s2 = armed.eval_accel(clock);
            let lead = lane.spec.ff_lead_ns;
            let (ff_vel, ff_acc) = if lead > 0 {
                lane.lead_vel_acc(clock + lead)
            } else {
                (vel_mm_s, acc_mm_s2)
            };
            self.samples[slot] = LaneSample {
                pos_mm: Some(f64::from(pos_mm)),
                vel_host: ff_vel,
                acc_host: ff_acc,
            };
            any = true;
        }
        any
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
                        flags: if anchor { LANE_RUN_FLAG_REANCHOR } else { 0 },
                        origin_mm_q16: (origin_mm * 65536.0).round() as i32,
                        start_index: index,
                        interval_ticks: self.interval_ns as u32,
                        sample_count: 0,
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
