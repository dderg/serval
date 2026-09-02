//! Production [`SegmentSink`]: anchors committed motion to the MCU clock,
//! splits it into per-axis clocked spans, and hands each span to the pump.

use crate::lock_ext::LockExt;
use std::sync::Arc;
use std::sync::Mutex;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

use crossbeam_channel::Sender;
use trajectory::{
    ContinuousAxis, ContinuousSegment, MotorGroup, MotorSpan, MotorTerm, NudgeProfile,
};

use super::CommittedFrontier;
use super::dispatch::{DispatchError, SegmentSink};

/// State a committed `ContinuousSegment` needs to reach the pump: per-MCU
/// clock anchoring/projection, the axis-lane split, and the motion-history
/// store whose retained spans a re-anchor invalidates.
pub(crate) struct PumpSink {
    pub(crate) transports: Arc<crate::axis_transport::AxisTransports>,
    pub(crate) router: Arc<Mutex<host_rt::passthrough_queue::PassthroughRouter>>,
    pub(crate) anchor: Arc<Mutex<crate::anchor::Anchor>>,
    pub(crate) mcu_configs: Vec<crate::mcu_config::McuAxisConfig>,
    pub(crate) pump_tx: Sender<crate::pump::EnqueueMsg>,
    pub(crate) pump_control: Option<Sender<crate::pump::PumpMsg>>,
    pub(crate) counter: Arc<AtomicU64>,
    pub(crate) active_drip_cohort: Arc<Mutex<Option<u64>>>,
    pub(crate) motion_history: Arc<Mutex<crate::motion_history::HistoryStore>>,
    pub(crate) frontier: Arc<CommittedFrontier>,
    pub(crate) frozen_projection: Mutex<std::collections::HashMap<u32, FrozenProjection>>,
    /// Set by the pump's fatal-transport action before the pump thread exits,
    /// so a closed enqueue channel is reported as the endpoint fatal it is
    /// rather than as a vanished thread.
    pub(crate) transport_fatal: Arc<Mutex<Option<String>>>,
}

impl PumpSink {
    fn pump_gone(&self) -> DispatchError {
        match self.transport_fatal.lock_ok().clone() {
            Some(reason) => DispatchError::TransportFatal(reason),
            None => DispatchError::PumpGone,
        }
    }
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
}

impl PumpSink {
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

    /// Whether any lane this mcu serves carries real motion in the segment —
    /// the same hold test the wire path applies, decided on the lane span so
    /// a re-anchor can tell a moving lane (whose clocks must track the live
    /// record) from an idle one (whose step-clock stream must not jump).
    fn mcu_has_motion(
        &self,
        cfg: &crate::mcu_config::McuAxisConfig,
        seg: &ContinuousSegment,
    ) -> bool {
        let module = crate::kinematics::KinematicsModule::from_tag(cfg.kinematics)
            .expect("mcu_configs were validated at build");
        cfg.axes.iter().any(|&axis_idx| {
            if axis_idx >= seg.axes.len() {
                return false;
            }
            !crate::enqueue::lane_span(&module, seg, axis_idx)
                .is_ok_and(|span| span.is_explicit_hold)
        })
    }

    fn live_clock_record(&self, mcu_id: u32) -> host_rt::passthrough_queue::ClockRecordSnapshot {
        self.router
            .lock_ok()
            .clock_record(crate::types::mcu_handle_from_raw(mcu_id))
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

    fn reanchor_projection(&self, mcu_id: u32, host_now: f64) -> Result<(), DispatchError> {
        let handle = crate::types::mcu_handle_from_raw(mcu_id);
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
        let mut frozen = self.frozen_projection.lock_ok();
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
            .lock_ok()
            .get(&mcu_id)
            .copied()
            .unwrap_or_else(|| {
                panic!(
                    "stepcompress mcu {mcu_id} projected a span before its segment anchored \
                     the host→mcu map — reanchor_projection must run before every dispatch"
                )
            })
    }

    /// Whether a retimed dispatch must re-base this mcu's frozen host→mcu
    /// projection on the live clocksync record. Moving lanes always re-base:
    /// their span clocks must track the clocksync, not a frozen slope that
    /// drifted since the last anchor. Hold-only (idle) lanes keep their
    /// frozen domain while it still tracks the live clock — re-basing them
    /// would jump their step-clock stream by the projection's drift for no
    /// motion. But that drift grows without bound while the lane sits parked
    /// (crystal ppm × hours), and once it exceeds the margin floor the
    /// frozen slope would project even hold spans into the mcu's past —
    /// re-base then; the reanchor cut re-bases the seams and a hold carries
    /// no steps, so no motion results.
    pub(crate) fn needs_rebase(
        &self,
        cfg: &crate::mcu_config::McuAxisConfig,
        seg: &ContinuousSegment,
        retimed: bool,
        prev: Option<FrozenProjection>,
        seam_host: f64,
    ) -> bool {
        let Some(prev) = prev else {
            return true;
        };
        if !retimed {
            return false;
        }
        if self.mcu_has_motion(cfg, seg) {
            return true;
        }
        let handle = crate::types::mcu_handle_from_raw(cfg.mcu_id);
        let live = self
            .router
            .lock_ok()
            .host_time_to_mcu_clock(handle, seam_host);
        match live {
            Ok(live) => {
                let drift = prev.project_exact(seam_host) - live as f64;
                let freq = prev.freq.max(1.0);
                let drifted = drift.abs() > crate::anchor::LOW_MARGIN_WARN_SECS * freq;
                if drifted {
                    tracing::warn!(
                        subsystem = "motion",
                        event = "hold_lane_projection_rebase",
                        mcu = cfg.mcu_id,
                        drift_us = drift / freq * 1e6,
                        "[reanchor] idle mcu's frozen projection drifted past the \
                         margin floor — re-basing its hold lanes on the live clock"
                    );
                }
                drifted
            }
            Err(_) => true,
        }
    }

    /// A nudge-path projection rebase moved this MCU's host→mcu map for
    /// every lane, but only the nudged lane carries spans (and so a cut)
    /// through the pump. Cut every sibling lane at the same clock via the
    /// pump's control channel — otherwise their shim seams keep the previous
    /// epoch's slope and the next spans (projected on the new map) miss the
    /// seam by `freq_delta × span` plus the rebase's offset jump.
    fn cut_sibling_lanes_after_rebase(
        &self,
        mcu_id: u32,
        nudged_axis: u8,
        seam_host: f64,
    ) -> Result<(), DispatchError> {
        let Some(control) = &self.pump_control else {
            return Ok(());
        };
        let at_start_clock = self.project(mcu_id, seam_host);
        let epoch_freq = self
            .frozen_projection
            .lock_ok()
            .get(&mcu_id)
            .map(|f| f.freq);
        for cfg in self.mcu_configs.iter().filter(|c| c.mcu_id == mcu_id) {
            for &axis_idx in &cfg.axes {
                let axis = axis_idx as u8;
                if axis == nudged_axis {
                    continue;
                }
                control
                    .send(crate::pump::PumpMsg::MarkReanchor {
                        key: crate::types::AxisKey { mcu_id, axis },
                        at_start_clock,
                        epoch_freq,
                    })
                    .map_err(|_| self.pump_gone())?;
            }
        }
        Ok(())
    }

    fn anchor(&self, t_start: f64, t_end: f64) -> AnchorPoint {
        let host_now = self.host_now();
        let (t0, epoch) = self
            .anchor
            .lock_ok()
            .anchor_segment(t_start, t_end, host_now);
        let drip_active = self.active_drip_cohort.lock_ok().is_some();
        AnchorPoint {
            t0,
            epoch,
            host_now,
            lead_secs: if drip_active {
                crate::pump::DRIP_WINDOW_SECS
            } else {
                crate::pump::MAX_LEAD_SECS
            },
        }
    }

    fn log_seg0_lead(&self, mcu_ids: impl Iterator<Item = u32>, seg_start_host: f64, t0: f64) {
        let r = self.router.lock_ok();
        for mcu_id in mcu_ids {
            let h = crate::types::mcu_handle_from_raw(mcu_id);
            r.log_seg0_lead(h, seg_start_host, t0);
        }
    }
}

impl SegmentSink for PumpSink {
    fn dispatch(&mut self, seg: &ContinuousSegment) -> Result<(), DispatchError> {
        tracing::debug!(
            subsystem = "engine",
            event = "dispatch_entered",
            seg_t_start = seg.t_start,
            seg_t_end = seg.t_end,
            "[engine-trace] dispatch entered"
        );

        let at = self.anchor(seg.t_start, seg.t_end);

        let runway_s = at.t0 + seg.t_end - at.host_now;
        if runway_s > 0.0 {
            self.frontier
                .advance_to(Instant::now() + Duration::from_secs_f64(runway_s));
        }

        let seam_host = at.t0 + seg.t_start;
        if at.epoch.retimed() || self.frozen_projection.lock_ok().is_empty() {
            let reanchor = {
                let frozen = self.frozen_projection.lock_ok();
                self.mcu_configs
                    .iter()
                    .filter(|cfg| self.freezes_projection(cfg.mcu_id))
                    .filter(|cfg| {
                        self.needs_rebase(
                            cfg,
                            seg,
                            at.epoch.retimed(),
                            frozen.get(&cfg.mcu_id).copied(),
                            seam_host,
                        )
                    })
                    .map(|cfg| cfg.mcu_id)
                    .collect::<Vec<_>>()
            };
            for mcu_id in reanchor {
                self.reanchor_projection(mcu_id, seam_host)?;
            }
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
            frozen_for_ctx.lock_ok().get(&mcu_id).map(|f| f.freq)
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
            self.motion_history.lock_ok().drop_pieces_on_reanchor();
        }
        for m in msgs {
            self.pump_tx.send(m).map_err(|_| self.pump_gone())?;
        }

        tracing::trace!(
            subsystem = "motion",
            event = "pipe_pump_in",
            line = seg.source_line,
            t_us = crate::timing::mono_us(),
            "[pipe] handed to pump"
        );

        self.counter.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }

    /// The single-axis, planner-bypassing sibling of `dispatch`.
    fn dispatch_nudge(
        &mut self,
        mcu_id: u32,
        axis: u8,
        motor_mask: u8,
        profile: &NudgeProfile,
    ) -> Result<(), DispatchError> {
        let axis_idx = axis as usize;
        let lane_present = self
            .mcu_configs
            .iter()
            .filter(|c| c.mcu_id == mcu_id)
            .any(|c| c.axes.contains(&axis_idx));
        if !lane_present {
            return Err(DispatchError::NudgeTargetMissing { mcu_id, axis });
        }

        let at = self.anchor(profile.t_start(), profile.t_end());
        self.anchor.lock_ok().mark_parked();

        let seam_host = at.t0 + profile.t_start();
        let fresh_projection = self.freezes_projection(mcu_id)
            && (at.epoch.retimed() || !self.frozen_projection.lock_ok().contains_key(&mcu_id));
        if fresh_projection {
            self.reanchor_projection(mcu_id, seam_host)?;
            self.cut_sibling_lanes_after_rebase(mcu_id, axis, seam_host)?;
        }

        if at.epoch.is_fresh() {
            self.log_seg0_lead(std::iter::once(mcu_id), seam_host, at.t0);
        }

        let epoch_freq = fresh_projection.then(|| self.frozen_epoch(mcu_id).freq);
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
            self.pump_tx
                .send(crate::pump::EnqueueMsg {
                    epoch_freq,
                    key,
                    spans,
                    epoch: at.epoch,
                    lead_secs: at.lead_secs,
                    source_line: u32::MAX,
                    batch_end: true,
                })
                .map_err(|_| self.pump_gone())?;
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

#[cfg(test)]
#[path = "projection_tests.rs"]
mod projection_tests;

#[cfg(test)]
#[path = "clock_record_gate_tests.rs"]
mod clock_record_gate_tests;
