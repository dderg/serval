use crate::lock_ext::LockExt;
use host_rt::passthrough_queue::McuHandle;
use std::sync::Arc;
use std::sync::Mutex;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

use trajectory::{
    ContinuousAxis, ContinuousSegment, MotorGroup, MotorSpan, MotorTerm, NudgeProfile,
};

use super::CommittedFrontier;
use super::dispatch::{DispatchError, SegmentSink};

pub(crate) struct Projection {
    pub(crate) transports: Arc<crate::axis_transport::AxisTransports>,
    pub(crate) router: Arc<Mutex<host_rt::passthrough_queue::PassthroughRouter>>,
    pub(crate) anchor: Arc<Mutex<crate::anchor::Anchor>>,
    pub(crate) mcu_configs: Vec<crate::mcu_config::McuAxisConfig>,
    pub(crate) counter: Arc<AtomicU64>,
    pub(crate) frontier: Arc<CommittedFrontier>,
    pub(crate) frozen_projection: std::collections::HashMap<u32, FrozenProjection>,
}

#[derive(Clone, Copy)]
pub(crate) struct FrozenProjection {
    host_ref: f64,
    mcu_ref: f64,
    freq: f64,
}

impl FrozenProjection {
    fn project_exact(self, host_secs: f64) -> f64 {
        self.mcu_ref + (host_secs - self.host_ref) * self.freq
    }
}

/// Anchoring context shared by segments and nudges: the timeline origin, the
/// clock projection, and the drip-mode lead limit.
struct AnchorPoint {
    t0: f64,
    epoch: crate::anchor::StreamEpoch,
    host_now: f64,
    lead_secs: f64,
    rest_from_host: Option<f64>,
}

impl Projection {
    fn host_now(&self) -> f64 {
        self.router.lock_ok().host_now_secs()
    }

    /// Whether this mcu's lanes are reached through a host-side committed
    /// stream — every serial transport is, pulse and phase alike, and each
    /// demands the epoch slope its frames were already projected on. Only the
    /// EtherCAT ring re-anchors against a grid the endpoint reports, so it
    /// keeps the live projection.
    fn freezes_projection(&self, mcu_id: u32) -> bool {
        self.mcu_configs
            .iter()
            .any(|c| c.mcu_id == mcu_id && !c.ethercat)
    }

    fn live_clock_record(&self, mcu_id: u32) -> host_rt::passthrough_queue::ClockRecordSnapshot {
        self.router
            .lock_ok()
            .clock_record(McuHandle::from_raw(mcu_id))
            .unwrap_or_else(|| {
                panic!(
                    "mcu {mcu_id} projected a span with no valid clocksync record — \
                     a projection off an invalidated record would send step clocks from the \
                     previous boot epoch"
                )
            })
    }

    /// The unrounded host→mcu map every clocked span anchors on. A serial
    /// lane reads the epoch's frozen slope; only the EtherCAT ring, which
    /// re-anchors against the grid its endpoint reports, reads the live
    /// clocksync record.
    fn live_projection_exact(&self, mcu_id: u32, host_secs: f64) -> f64 {
        let record = self.live_clock_record(mcu_id);
        record.last_clock as f64 + ((host_secs - record.clock_offset) * record.clock_freq).max(0.0)
    }

    fn reanchor_projection(&mut self, mcu_id: u32, host_now: f64) -> Result<(), DispatchError> {
        let handle = McuHandle::from_raw(mcu_id);
        let record = self
            .router
            .lock_ok()
            .clock_record(handle)
            .filter(|r| r.converged)
            .ok_or(DispatchError::ClockRecordUnusable {
                mcu_id,
                mcu_handle: handle,
            })?;
        if record.age_secs > host_rt::passthrough_queue::MAX_CLOCK_RECORD_AGE_SECS {
            tracing::error!(
                subsystem = "motion",
                event = "clock_record_stale",
                mcu = mcu_id,
                host_now,
                record_age_secs = record.age_secs,
                max_age_secs = host_rt::passthrough_queue::MAX_CLOCK_RECORD_AGE_SECS,
                centroid_lag_secs = record.centroid_lag_secs,
                clock_offset = record.clock_offset,
                last_clock = record.last_clock,
                "[reanchor] refusing to anchor on a clock record the router has not \
                 updated for {:.3}s",
                record.age_secs
            );
            return Err(DispatchError::ClockRecordStale {
                mcu_id,
                mcu_handle: handle,
                age_secs: record.age_secs,
                max_age_secs: host_rt::passthrough_queue::MAX_CLOCK_RECORD_AGE_SECS,
            });
        }
        if record.age_secs > host_rt::passthrough_queue::DEGRADED_CLOCK_RECORD_AGE_SECS {
            tracing::warn!(
                subsystem = "motion",
                event = "clock_record_degraded",
                mcu = mcu_id,
                host_now,
                record_age_secs = record.age_secs,
                degraded_age_secs = host_rt::passthrough_queue::DEGRADED_CLOCK_RECORD_AGE_SECS,
                centroid_lag_secs = record.centroid_lag_secs,
                "[reanchor] anchoring on a clock record clocksync last refreshed \
                 {:.3}s ago — samples are being missed",
                record.age_secs
            );
        }
        let freq = record.clock_freq;
        // The anchor point always comes from the live clocksync record, never
        // from the previous frozen projection: the frozen slope drifts from
        // the live estimate by `freq_error * elapsed` over a long epoch (an
        // idle resume after a ten-minute park carries tens of ms with a
        // 100 ppm crystal), and a chained mcu_ref carries that drift forward
        // forever. Re-anchoring from the live record bounds the first-volley
        // clock's error to the clocksync's own — the guards then hold it to
        // the floor margin — and the reanchor cut re-bases the MCU step
        // clock anyway, so nothing downstream depends on continuity.
        let frozen = &mut self.frozen_projection;
        let mcu_ref = self
            .router
            .lock_ok()
            .host_time_to_mcu_clock(handle, host_now)
            .map_err(|_| DispatchError::ClockRecordUnusable {
                mcu_id,
                mcu_handle: handle,
            })? as f64;
        tracing::info!(
            subsystem = "motion",
            event = "reanchor_record",
            mcu = mcu_id,
            host_now,
            clock_freq = record.clock_freq,
            clock_offset = record.clock_offset,
            last_clock = record.last_clock,
            converged = record.converged,
            projected_now = record.projected_now,
            mcu_ref,
            anchor_lead_secs = (mcu_ref - record.projected_now as f64) / freq,
            record_age_secs = record.age_secs,
            centroid_lag_secs = record.centroid_lag_secs,
            "[reanchor] anchored the host→mcu map on the live clocksync record"
        );
        if let Some(prev) = frozen.get(&mcu_id).copied() {
            let drift_ticks = prev.project_exact(host_now) - mcu_ref;
            if drift_ticks.abs() > crate::anchor::LOW_MARGIN_WARN_SECS * freq {
                let drift_us = drift_ticks / freq * 1e6;
                let span_s = host_now - prev.host_ref;
                tracing::warn!(
                    subsystem = "motion",
                    event = "reanchor_projection_drift",
                    mcu = mcu_id,
                    drift_us,
                    prev_host_ref = prev.host_ref,
                    span_s,
                    prev_freq = prev.freq,
                    live_freq = freq,
                    host_now,
                    mcu_ref,
                    "[reanchor] the previous epoch's frozen projection drifted \
                     {drift_us:.0} us from the live clock over {span_s:.1} s — \
                     its step clocks were that far off the mcu"
                );
            }
        }
        frozen.insert(
            mcu_id,
            FrozenProjection {
                host_ref: host_now,
                mcu_ref,
                freq,
            },
        );
        Ok(())
    }

    fn project_exact(&self, mcu_id: u32, host_secs: f64) -> f64 {
        if !self.freezes_projection(mcu_id) {
            return self.live_projection_exact(mcu_id, host_secs);
        }
        self.frozen_epoch(mcu_id).project_exact(host_secs)
    }

    fn project(&self, mcu_id: u32, host_secs: f64) -> u64 {
        self.project_exact(mcu_id, host_secs).round().max(0.0) as u64
    }

    /// The cycle rate every clocked span on this mcu is cut against: the
    /// epoch slope the frozen map was anchored on, so a chain of abutting
    /// spans lands on the same lattice the seams were projected on.
    fn clock_freq_hz(&self, mcu_id: u32) -> f64 {
        if !self.freezes_projection(mcu_id) {
            return self.live_clock_record(mcu_id).clock_freq;
        }
        self.frozen_epoch(mcu_id).freq
    }

    fn frozen_epoch(&self, mcu_id: u32) -> FrozenProjection {
        self.frozen_projection
            .get(&mcu_id)
            .copied()
            .unwrap_or_else(|| {
                panic!(
                    "stepcompress mcu {mcu_id} projected a span before its segment anchored \
                     the host→mcu map — reanchor_projection must run before every dispatch"
                )
            })
    }

    /// Every fresh epoch resumes all lanes from rest, so correcting the
    /// host→mcu map there shifts the resumed stream inside the rest instead
    /// of inside motion. Between rests the frozen slope's error grows
    /// unchecked, so every serial mcu re-syncs at every one — a mcu that
    /// only holds in the epoch's first segment still moves later in it. A
    /// correction pulling the resume back past the start of the rest would
    /// schedule it before the previous stream ended; it waits for a rest
    /// long enough to take it.
    fn resync_at_rest(
        &mut self,
        seam_host: f64,
        rest_from_host: Option<f64>,
    ) -> Result<(), DispatchError> {
        let mcu_ids: std::collections::BTreeSet<u32> = self
            .mcu_configs
            .iter()
            .filter(|cfg| self.freezes_projection(cfg.mcu_id))
            .map(|cfg| cfg.mcu_id)
            .collect();
        for mcu_id in mcu_ids {
            let Some(prev) = self.frozen_projection.get(&mcu_id).copied() else {
                self.reanchor_projection(mcu_id, seam_host)?;
                continue;
            };
            let live = self
                .router
                .lock_ok()
                .host_time_to_mcu_clock(McuHandle::from_raw(mcu_id), seam_host)
                .map_err(|_| DispatchError::ClockRecordUnusable {
                    mcu_id,
                    mcu_handle: McuHandle::from_raw(mcu_id),
                })? as f64;
            let correction_s = (live - prev.project_exact(seam_host)) / prev.freq;
            let rest_s = rest_from_host.map_or(f64::INFINITY, |from| seam_host - from);
            let fits_in_rest = -correction_s < rest_s;
            tracing::info!(
                subsystem = "motion",
                event = "clock_resync",
                mcu = mcu_id,
                correction_us = correction_s * 1e6,
                rest_s,
                applied = fits_in_rest,
                seam_host,
                "[reanchor] re-syncing the host→mcu map at a rest"
            );
            if fits_in_rest {
                self.reanchor_projection(mcu_id, seam_host)?;
            }
        }
        Ok(())
    }

    fn cut_sibling_lanes_after_rebase<S: crate::pump::SpanSink>(
        &self,
        pump: &mut crate::pump::Pump<S>,
        mcu_id: u32,
        nudged_axis: u8,
        seam_host: f64,
    ) {
        let at_start_clock = self.project(mcu_id, seam_host);
        let epoch_freq = self.frozen_projection.get(&mcu_id).map(|f| f.freq);
        for cfg in self.mcu_configs.iter().filter(|c| c.mcu_id == mcu_id) {
            for &axis_idx in &cfg.axes {
                let axis = axis_idx as u8;
                if axis != nudged_axis {
                    pump.cut_lane_at(
                        crate::types::AxisKey { mcu_id, axis },
                        at_start_clock,
                        epoch_freq,
                    );
                }
            }
        }
    }

    fn anchor(&self, t_start: f64, t_end: f64, drip_active: bool) -> AnchorPoint {
        let host_now = self.host_now();
        let mut anchor = self.anchor.lock_ok();
        let rest_from_host = anchor.committed_end_host();
        let (t0, epoch) = anchor.anchor_segment(t_start, t_end, host_now);
        AnchorPoint {
            t0,
            epoch,
            host_now,
            lead_secs: if drip_active {
                crate::pump::DRIP_WINDOW_SECS
            } else {
                crate::pump::MAX_LEAD_SECS
            },
            rest_from_host,
        }
    }

    fn anchor_segment(
        &mut self,
        t_start: f64,
        t_end: f64,
        drip_active: bool,
    ) -> Result<AnchorPoint, DispatchError> {
        let at = self.anchor(t_start, t_end, drip_active);
        if at.epoch.is_fresh() || self.frozen_projection.is_empty() {
            self.resync_at_rest(at.t0 + t_start, at.rest_from_host)?;
        }
        Ok(at)
    }

    fn log_seg0_lead(&self, mcu_ids: impl Iterator<Item = u32>, seg_start_host: f64, t0: f64) {
        let r = self.router.lock_ok();
        for mcu_id in mcu_ids {
            let h = McuHandle::from_raw(mcu_id);
            r.log_seg0_lead(h, seg_start_host, t0);
        }
    }
}

impl Projection {
    fn dispatch<S: crate::pump::SpanSink>(
        &mut self,
        seg: &ContinuousSegment,
        pump: &mut crate::pump::Pump<S>,
    ) -> Result<(), DispatchError> {
        if let Some(reason) = &pump.fatal_reason {
            return Err(DispatchError::ExecutionHalted(reason.clone()));
        }
        tracing::debug!(
            subsystem = "engine",
            event = "dispatch_entered",
            seg_t_start = seg.t_start,
            seg_t_end = seg.t_end,
            "[engine-trace] dispatch entered"
        );

        let at = self.anchor_segment(seg.t_start, seg.t_end, pump.drip_active())?;

        let runway_s = at.t0 + seg.t_end - at.host_now;
        if runway_s > 0.0 {
            self.frontier
                .advance_to(Instant::now() + Duration::from_secs_f64(runway_s));
        }

        if at.epoch.is_fresh() {
            self.log_seg0_lead(
                self.mcu_configs.iter().map(|cfg| cfg.mcu_id),
                at.t0 + seg.t_start,
                at.t0,
            );
        }

        let fresh = at.epoch.is_fresh();
        let frozen_for_ctx = &self.frozen_projection;
        let epoch_freq_of = move |mcu_id: u32| -> Option<f64> {
            if !fresh {
                return None;
            }
            frozen_for_ctx.get(&mcu_id).map(|f| f.freq)
        };
        let msgs = crate::enqueue::enqueue_segment(
            seg,
            &self.mcu_configs,
            &crate::enqueue::EnqueueCtx {
                t0: at.t0,
                epoch: at.epoch,
                host_now: at.host_now,
                lead_secs: at.lead_secs,
                epoch_freq: &epoch_freq_of,
                project_exact: |mcu_id, host_secs| self.project_exact(mcu_id, host_secs),
                clock_freq_hz: &|mcu_id| self.clock_freq_hz(mcu_id),
                lane_is_phase: &|key| self.transports.is_phase(key),
            },
        )?;

        if at.epoch.retimed() {
            if let Some(history) = &pump.history {
                history.lock_ok().drop_pieces_on_reanchor();
            }
        }
        for m in msgs {
            pump.enqueue(m);
        }

        tracing::trace!(
            subsystem = "motion",
            event = "pipe_pump_in",
            line = seg.source_line,
            t_us = motion_pipeline::timing::mono_us(),
            "[pipe] handed to pump"
        );

        self.counter.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }

    /// The single-axis, planner-bypassing sibling of `dispatch`.
    fn dispatch_nudge<S: crate::pump::SpanSink>(
        &mut self,
        mcu_id: u32,
        axis: u8,
        motor_mask: u8,
        profile: &NudgeProfile,
        pump: &mut crate::pump::Pump<S>,
    ) -> Result<(), DispatchError> {
        if let Some(reason) = &pump.fatal_reason {
            return Err(DispatchError::ExecutionHalted(reason.clone()));
        }
        let axis_idx = axis as usize;
        let lane_present = self
            .mcu_configs
            .iter()
            .filter(|c| c.mcu_id == mcu_id)
            .any(|c| c.axes.contains(&axis_idx));
        if !lane_present {
            return Err(DispatchError::NudgeTargetMissing { mcu_id, axis });
        }

        let at = self.anchor(profile.t_start(), profile.t_end(), pump.drip_active());
        self.anchor.lock_ok().mark_parked();

        let seam_host = at.t0 + profile.t_start();
        let fresh_projection = self.freezes_projection(mcu_id)
            && (at.epoch.retimed() || !self.frozen_projection.contains_key(&mcu_id));
        if fresh_projection {
            self.reanchor_projection(mcu_id, seam_host)?;
            self.cut_sibling_lanes_after_rebase(pump, mcu_id, axis, seam_host);
        }

        if at.epoch.is_fresh() {
            self.log_seg0_lead(std::iter::once(mcu_id), seam_host, at.t0);
        }

        let epoch_freq = (fresh_projection
            || (at.epoch.is_fresh() && self.freezes_projection(mcu_id)))
        .then(|| self.frozen_epoch(mcu_id).freq);
        let signal = MotorSpan::try_new(
            Arc::from(vec![MotorGroup::Independent(MotorTerm {
                source_axis: axis_idx,
                axis: ContinuousAxis::Nudge(profile.clone()),
                scale: 1.0,
            })]),
            profile.t_start(),
            profile.t_end(),
            motor_mask,
            u32::MAX,
            false,
        )?;
        let spans = crate::enqueue::clock_span(
            Arc::new(signal),
            mcu_id,
            axis_idx,
            &crate::enqueue::EnqueueCtx {
                t0: at.t0,
                epoch: at.epoch,
                host_now: at.host_now,
                lead_secs: at.lead_secs,
                epoch_freq: &|_| epoch_freq,
                project_exact: |proj_mcu_id, host_secs| self.project_exact(proj_mcu_id, host_secs),
                clock_freq_hz: &|proj_mcu_id| self.clock_freq_hz(proj_mcu_id),
                lane_is_phase: &|key| self.transports.is_phase(key),
            },
        )?;

        if !spans.is_empty() {
            let key = crate::types::AxisKey { mcu_id, axis };
            pump.enqueue(crate::pump::LaneProjection {
                epoch_freq,
                key,
                spans,
                epoch: at.epoch,
                lead_secs: at.lead_secs,
                source_line: u32::MAX,
            });
        }

        self.counter.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }

    /// The drain declares the park; the anchor needs it because the committed
    /// track's end derivative cannot report one — a trailing derivative-gain
    /// stage (pressure advance) leaves the parked extruder's commanded
    /// velocity at `k·ë`, nonzero at every stop the profile reaches with
    /// acceleration still applied.
    fn mark_parked(&mut self) {
        self.anchor.lock_ok().mark_parked();
    }
}

pub(crate) struct PumpSink<'a, S> {
    pub(crate) projection: &'a mut Projection,
    pub(crate) pump: &'a mut crate::pump::Pump<S>,
}

impl<S: crate::pump::SpanSink> SegmentSink for PumpSink<'_, S> {
    fn dispatch(&mut self, seg: &ContinuousSegment) -> Result<(), DispatchError> {
        self.projection.dispatch(seg, self.pump)
    }

    fn dispatch_nudge(
        &mut self,
        mcu_id: u32,
        axis: u8,
        motor_mask: u8,
        profile: &NudgeProfile,
    ) -> Result<(), DispatchError> {
        self.projection
            .dispatch_nudge(mcu_id, axis, motor_mask, profile, self.pump)
    }

    fn mark_parked(&mut self) {
        self.projection.mark_parked();
    }
}

#[cfg(test)]
#[path = "projection_tests.rs"]
mod projection_tests;

#[cfg(test)]
#[path = "clock_record_gate_tests.rs"]
mod clock_record_gate_tests;
