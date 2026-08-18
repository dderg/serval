// Engine-side wiring for the sample-stream executor: oid → lane resolution,
// the foreground command entry points, and the tick-time dispatch for a
// sample-driven phase lane.

use crate::clock::read_widened_now;
use crate::sample_exec::{LaneOutput, SampleLane};
use crate::state::SharedState;

use super::Engine;

impl Engine {
    /// A sample command names a stepper oid; the lane is the kinematic slot
    /// that oid is bound to by `runtime_configure_axis`.
    pub fn sample_lane_for_oid(&self, oid: u8) -> Option<usize> {
        self.stepping_axes.iter().position(|slot| {
            slot.as_ref()
                .is_some_and(|axis| axis.steppers.iter().any(|s| s.stepper_oid == oid))
        })
    }

    fn lane_mut(&mut self, oid: u8, shared: &SharedState) -> Option<(usize, &mut SampleLane)> {
        let Some(lane_idx) = self.sample_lane_for_oid(oid) else {
            crate::fault_helpers::raise_sample_lane_unknown(shared, oid);
            return None;
        };
        let lane = self.sample_lanes.get_mut(lane_idx)?;
        Some((lane_idx, lane))
    }

    pub fn sample_anchor(&mut self, shared: &SharedState, oid: u8, clock: u64, position: i32) {
        let now = read_widened_now(shared);
        let Some((lane_idx, lane)) = self.lane_mut(oid, shared) else {
            return;
        };
        if let Err(fault) = lane.anchor(now, clock, position) {
            fault.latch(shared, lane_idx);
        }
    }

    pub fn sample_push_run(
        &mut self,
        shared: &SharedState,
        oid: u8,
        interval_ticks: u32,
        count: u8,
        data: &[u8],
    ) {
        let now = read_widened_now(shared);
        let Some((lane_idx, lane)) = self.lane_mut(oid, shared) else {
            return;
        };
        if let Err(fault) = lane.push_run(now, interval_ticks, count, data) {
            fault.latch(shared, lane_idx);
        }
    }

    pub fn sample_push_overlay(
        &mut self,
        shared: &SharedState,
        oid: u8,
        clock: u64,
        interval_ticks: u32,
        count: u8,
        data: &[u8],
    ) {
        let now = read_widened_now(shared);
        let Some((lane_idx, lane)) = self.lane_mut(oid, shared) else {
            return;
        };
        if let Err(fault) = lane.push_overlay(now, clock, interval_ticks, count, data) {
            fault.latch(shared, lane_idx);
        }
    }

    /// Executed position at the halt clock (or at the last tick when running),
    /// for the host's `sample_get_position` reconcile.
    pub fn sample_executed(&self, oid: u8) -> Option<(u64, i32)> {
        let lane_idx = self.sample_lane_for_oid(oid)?;
        self.sample_lanes.get(lane_idx).map(SampleLane::executed)
    }

    pub fn sample_push_barrier(&mut self, shared: &SharedState, oid: u8, seq: u32) {
        let now = read_widened_now(shared);
        let Some((lane_idx, lane)) = self.lane_mut(oid, shared) else {
            return;
        };
        if let Err(fault) = lane.push_barrier(now, seq) {
            fault.latch(shared, lane_idx);
        }
    }

    /// Pop one passed fence across every lane, tagged with the stepper oid the
    /// host addressed it by. Foreground-only; the caller loops until `None` and
    /// sends one `sample_barrier_ack` per result.
    pub fn sample_take_barrier_ack(&mut self) -> Option<(u8, u32)> {
        for lane_idx in 0..self.sample_lanes.len() {
            let Some(oid) = self.sample_lane_oid(lane_idx) else {
                continue;
            };
            let Some(lane) = self.sample_lanes.get_mut(lane_idx) else {
                continue;
            };
            if let Some(seq) = lane.take_passed_barrier() {
                return Some((oid, seq));
            }
        }
        None
    }

    fn sample_lane_oid(&self, lane_idx: usize) -> Option<u8> {
        self.stepping_axes
            .get(lane_idx)?
            .as_ref()?
            .steppers
            .first()
            .map(|stepper| stepper.stepper_oid)
    }

    /// Publish a trip halt from a context that may not touch `IsrState`. The
    /// next tick performs the halt at exactly this clock. The first requester
    /// wins: a trip signals every stepper, and every lane must freeze on one
    /// clock.
    pub fn sample_request_halt(shared: &SharedState, halt_clock: u64) {
        let _ = shared.sample_halt_clock.compare_exchange(
            crate::state::NO_HALT_REQUEST,
            halt_clock,
            core::sync::atomic::Ordering::AcqRel,
            core::sync::atomic::Ordering::Acquire,
        );
    }

    /// Freeze every sample lane at the position the requested halt clock
    /// interpolates to and discard the queued runs.
    pub(crate) fn sample_take_halt_request(&mut self, shared: &SharedState) {
        let halt_clock = shared.sample_halt_clock.swap(
            crate::state::NO_HALT_REQUEST,
            core::sync::atomic::Ordering::AcqRel,
        );
        if halt_clock == crate::state::NO_HALT_REQUEST {
            return;
        }
        for (lane_idx, lane) in self.sample_lanes.iter_mut().enumerate() {
            lane.halt(halt_clock, shared, lane_idx);
        }
    }

    pub fn sample_lane_anchored(&self, lane_idx: usize) -> bool {
        self.sample_lanes
            .get(lane_idx)
            .is_some_and(SampleLane::is_anchored)
    }

    /// Drive one sample lane for this tick. Returns whether the lane owns the
    /// axis this tick.
    ///
    /// A halted lane repeats its freeze position every tick and executes no
    /// host sample. A trip stop halts every lane, including one whose axis the
    /// host now drives through the classic step queue, so that hold yields the
    /// axis rather than tripping the mis-routing guard: only a lane still
    /// playing host samples into a pulse-mode axis is genuinely mis-routed.
    pub(crate) fn sample_dispatch(
        &mut self,
        lane_idx: usize,
        now: u64,
        shared: &SharedState,
    ) -> bool {
        let Some(lane) = self.sample_lanes.get_mut(lane_idx) else {
            return false;
        };
        let LaneOutput::Position(position) = lane.tick(now, shared, lane_idx) else {
            return false;
        };
        let only_holding_a_halt = lane.is_halted();
        if crate::buzz_stream::is_xdirect(lane_idx) {
            return true;
        }
        let Some(axis) = self
            .stepping_axes
            .get_mut(lane_idx)
            .and_then(|slot| slot.as_mut())
        else {
            return true;
        };
        if axis.mode.load(core::sync::atomic::Ordering::Acquire)
            != crate::stepping_state::StepMode::Phase as u8
        {
            if only_holding_a_halt {
                return false;
            }
            crate::fault_helpers::raise_phase_mode_not_available(shared, lane_idx);
            return true;
        }
        axis.last_step_count = position;
        #[cfg(feature = "motion-module-stepper")]
        {
            let max_ramp = i32::from(
                shared
                    .max_phase_offset_ramp_per_sample
                    .load(core::sync::atomic::Ordering::Acquire),
            );
            for stepper in &axis.steppers {
                crate::dispatch_stepper::ramp_phase_offset(stepper, max_ramp);
            }
            crate::dispatch_stepper::write_phase_coils(lane_idx, axis, shared, 0);
        }
        true
    }
}

#[cfg(test)]
#[path = "sample_tests.rs"]
mod sample_tests;
