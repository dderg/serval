//! Production [`SegmentSink`]: anchors committed motion to the MCU clock,
//! splits it into per-axis pieces, and hands each piece to the pump.

use crate::lock_ext::LockExt;
use std::sync::Arc;
use std::sync::Mutex;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

use crossbeam_channel::Sender;
use motion_pipeline::NudgePiece;
use trajectory::ShapedSegment;

use super::CommittedFrontier;
use super::dispatch::{DispatchError, SegmentSink};

/// State a committed `ShapedSegment` needs to reach the pump: per-MCU clock
/// anchoring/projection, the axis-lane split, and the motion-history store
/// whose retained pieces a re-anchor invalidates.
pub(crate) struct PumpSink {
    pub(crate) router: Arc<Mutex<host_rt::passthrough_queue::PassthroughRouter>>,
    pub(crate) anchor: Arc<Mutex<crate::anchor::Anchor>>,
    pub(crate) mcu_configs: Vec<crate::mcu_config::McuAxisConfig>,
    pub(crate) pump_tx: Sender<crate::pump::EnqueueMsg>,
    pub(crate) counter: Arc<AtomicU64>,
    pub(crate) active_drip_cohort: Arc<Mutex<Option<u64>>>,
    pub(crate) motion_history: Arc<Mutex<crate::motion_history::HistoryStore>>,
    pub(crate) frontier: Arc<CommittedFrontier>,
    pub(crate) frozen_projection: Mutex<std::collections::HashMap<u32, FrozenProjection>>,
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

    fn project(self, host_secs: f64) -> u64 {
        self.project_exact(host_secs).round().max(0.0) as u64
    }
}

/// Anchoring context shared by segments and nudges: the timeline origin, the
/// clock projection, and the drip-mode piece/lead limits.
struct AnchorPoint {
    t0: f64,
    epoch: crate::anchor::StreamEpoch,
    host_now: f64,
    max_piece_secs: Option<f64>,
    lead_secs: f64,
}

impl PumpSink {
    fn host_now(&self) -> f64 {
        self.router.lock_ok().host_now_secs()
    }

    fn is_stepcompress(&self, mcu_id: u32) -> bool {
        self.mcu_configs.iter().any(|c| {
            c.mcu_id == mcu_id && c.stepping_mode == crate::mcu_config::SteppingMode::Stepcompress
        })
    }

    /// Whether any lane this mcu serves carries real motion in the segment —
    /// the same hold test `is_pure_hold` applies to wire pieces, decided on
    /// the lane curve so a re-anchor can tell a moving lane (whose clocks
    /// must track the live record) from an idle one (whose step-clock stream
    /// must not jump).
    fn mcu_has_motion(&self, cfg: &crate::mcu_config::McuAxisConfig, seg: &ShapedSegment) -> bool {
        let module = crate::kinematics::KinematicsModule::from_tag(cfg.kinematics)
            .expect("mcu_configs were validated at build");
        cfg.axes.iter().any(|&axis_idx| {
            if axis_idx >= seg.axes.len() {
                return false;
            }
            let curve = crate::enqueue::lane_curve(&module, &seg.axes, axis_idx);
            !crate::enqueue::lane_curve_is_hold(&curve)
        })
    }

    fn live_projection(&self, mcu_id: u32, host_secs: f64) -> u64 {
        self.router
            .lock_ok()
            .host_time_to_mcu_clock(crate::types::mcu_handle_from_raw(mcu_id), host_secs)
            .unwrap_or_else(|e| {
                panic!(
                    "mcu {mcu_id} projected a piece with no valid clocksync record ({e}) — \
                     a projection off an invalidated record would send step clocks from the \
                     previous boot epoch"
                )
            })
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

    fn project(&self, mcu_id: u32, host_secs: f64) -> u64 {
        if !self.is_stepcompress(mcu_id) {
            return self.live_projection(mcu_id, host_secs);
        }
        self.frozen_projection
            .lock_ok()
            .get(&mcu_id)
            .copied()
            .unwrap_or_else(|| {
                panic!(
                    "stepcompress mcu {mcu_id} projected a piece before its segment anchored \
                     the host→mcu map — reanchor_projection must run before every dispatch"
                )
            })
            .project(host_secs)
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
            max_piece_secs: drip_active.then_some(0.025_f64),
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
    fn dispatch(&mut self, seg: &ShapedSegment) -> Result<(), DispatchError> {
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
                    .filter(|cfg| self.is_stepcompress(cfg.mcu_id))
                    .filter(|cfg| {
                        if !frozen.contains_key(&cfg.mcu_id) {
                            true
                        } else {
                            // A retimed epoch re-bases the lanes that actually
                            // move on the live clock — the piece clocks of a
                            // moving lane must track the clocksync, not a
                            // frozen slope that drifted since the last anchor.
                            // Hold-only (idle) lanes keep their frozen domain:
                            // re-basing them would jump their step-clock
                            // stream by the projection's drift, moving the
                            // lane (or tripping the count reconcile) for no
                            // motion.
                            at.epoch.retimed() && self.mcu_has_motion(cfg, seg)
                        }
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
                project: |mcu_id, host_secs| self.project(mcu_id, host_secs),
                max_piece_secs: at.max_piece_secs,
            },
        );

        if at.epoch.retimed() {
            self.motion_history.lock_ok().drop_pieces_on_reanchor();
        }
        for m in msgs {
            self.pump_tx.send(m).map_err(|_| DispatchError::PumpGone)?;
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
    fn dispatch_nudge(&mut self, mcu_id: u32, np: &NudgePiece) -> Result<(), DispatchError> {
        let axis = np.axis;
        let lane_present = self
            .mcu_configs
            .iter()
            .filter(|c| c.mcu_id == mcu_id)
            .any(|c| c.axes.contains(&(axis as usize)));
        if !lane_present {
            return Err(DispatchError::NudgeTargetMissing { mcu_id, axis });
        }

        let at = self.anchor(np.piece.u_start, np.piece.u_end);
        self.anchor.lock_ok().mark_parked();

        let fresh_projection = self.is_stepcompress(mcu_id)
            && (at.epoch.retimed() || !self.frozen_projection.lock_ok().contains_key(&mcu_id));
        if fresh_projection {
            self.reanchor_projection(mcu_id, at.t0 + np.piece.u_start)?;
        }

        if at.epoch.is_fresh() {
            self.log_seg0_lead(std::iter::once(mcu_id), at.t0 + np.piece.u_start, at.t0);
        }

        let project = |proj_mcu_id: u32, host_secs: f64| self.project(proj_mcu_id, host_secs);
        let pieces = crate::enqueue::flatten_bezier_pieces(
            std::slice::from_ref(&np.piece),
            &crate::enqueue::FlattenCtx {
                t0: at.t0,
                mcu_id,
                axis_idx: axis as usize,
                host_now: at.host_now,
                project: &project,
                max_piece_secs: at.max_piece_secs,
                motor_mask: np.motor_mask,
            },
        );

        if !pieces.is_empty() {
            let key = crate::types::AxisKey { mcu_id, axis };
            self.pump_tx
                .send(crate::pump::EnqueueMsg {
                    epoch_freq: if fresh_projection {
                        self.frozen_projection
                            .lock_ok()
                            .get(&mcu_id)
                            .map(|f| f.freq)
                    } else {
                        None
                    },
                    key,
                    pieces,
                    epoch: at.epoch,
                    lead_secs: at.lead_secs,
                    source_line: u32::MAX,
                    batch_end: true,
                })
                .map_err(|_| DispatchError::PumpGone)?;
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
