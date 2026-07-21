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

    fn project(&self, mcu_id: u32, host_secs: f64) -> u64 {
        self.router
            .lock_ok()
            .host_time_to_mcu_clock(crate::types::mcu_handle_from_raw(mcu_id), host_secs)
            .unwrap_or(0)
    }

    fn anchor(&self, t_start: f64, t_end: f64, ends_at_rest: bool) -> AnchorPoint {
        let host_now = self.host_now();
        let (t0, epoch) =
            self.anchor
                .lock_ok()
                .anchor_segment(t_start, t_end, host_now, ends_at_rest);
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

        let at = self.anchor(seg.t_start, seg.t_end, segment_ends_at_rest(seg));

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

        let msgs = crate::enqueue::enqueue_segment(
            seg,
            &self.mcu_configs,
            &crate::enqueue::EnqueueCtx {
                t0: at.t0,
                epoch: at.epoch,
                host_now: at.host_now,
                lead_secs: at.lead_secs,
                project: |mcu_id, host_secs| self.project(mcu_id, host_secs),
                max_piece_secs: at.max_piece_secs,
            },
        );

        if at.epoch.is_fresh() {
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
        if !self.mcu_configs.iter().any(|c| c.mcu_id == mcu_id) {
            return Err(DispatchError::NudgeTargetMissing { mcu_id, axis });
        }

        let at = self.anchor(np.piece.u_start, np.piece.u_end, true);

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
                    key,
                    pieces,
                    epoch: at.epoch,
                    lead_secs: at.lead_secs,
                    source_line: u32::MAX,
                })
                .map_err(|_| DispatchError::PumpGone)?;
        }

        self.counter.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }
}

/// Whether every axis of the segment has zero velocity at its end — the
/// machine is parked there, so a later resume may re-anchor instead of
/// faulting. Hold segments are exactly constant; brake-to-rest tails end at
/// an exact zero, so the epsilon only absorbs floating-point noise.
const REST_VELOCITY_EPS: f64 = 1e-6;

fn segment_ends_at_rest(seg: &ShapedSegment) -> bool {
    seg.axes.iter().all(|axis| {
        let Some(&u_end) = axis.knots().last() else {
            return true;
        };
        nurbs::eval::eval_derivative(axis.control_points(), axis.knots(), axis.degree(), u_end)
            .abs()
            < REST_VELOCITY_EPS
    })
}
