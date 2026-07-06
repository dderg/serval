use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, Sender};
use trajectory::ShapedSegment;

use motion_pipeline::{BarrierAck, Control, ShapedItem};

use super::{CommittedFrontier, fatal};

#[derive(Debug, thiserror::Error)]
pub enum DispatchError {
    #[error(
        "motion-engine: curve for mcu {mcu_id} exceeds caps \
         (pieces {pieces} > {max_pieces}); \
         logical-move splitting not yet implemented (Task 13 follow-up)."
    )]
    CapsExceeded {
        mcu_id: u32,
        pieces: usize,
        max_pieces: usize,
    },
    #[error("compute_ack_clock: {0}")]
    ComputeAckClock(String),
    #[error(
        "compute_ack_clock returned 0 after 5s — \
         clock-sync didn't establish for mcu {mcu_id} (mcu_h={mcu_handle:?})"
    )]
    ClockSyncTimeout {
        mcu_id: u32,
        mcu_handle: host_rt::passthrough_queue::McuHandle,
    },
    #[error("MCU {0}: connection dropped during dispatch")]
    ConnectionDropped(u32),
    #[error("piece pump thread is gone; cannot dispatch")]
    PumpGone,
    #[error("nudge target mcu_id={mcu_id} axis={axis} not present in mcu_configs")]
    NudgeTargetMissing { mcu_id: u32, axis: u8 },
}

pub(crate) type DispatchFn = Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync>;
pub(crate) type NudgeDispatchFn =
    Arc<dyn Fn(u32, &crate::nudge::NudgePiece) -> Result<(), DispatchError> + Send + Sync>;

/// State a committed `ShapedSegment` needs to reach the pump: per-MCU clock
/// anchoring/projection, the axis-lane split, and the motion-history store
/// whose retained pieces a re-anchor invalidates.
pub(crate) struct SegmentDispatchCtx {
    pub(crate) router: Arc<Mutex<host_rt::passthrough_queue::PassthroughRouter>>,
    pub(crate) anchor: Arc<Mutex<crate::anchor::Anchor>>,
    pub(crate) mcu_configs: Vec<crate::mcu_config::McuAxisConfig>,
    pub(crate) pump_tx: Sender<crate::pump::EnqueueMsg>,
    pub(crate) counter: Arc<AtomicU64>,
    pub(crate) active_drip_cohort: Arc<Mutex<Option<u64>>>,
    pub(crate) motion_history: Arc<Mutex<crate::motion_history::HistoryStore>>,
    pub(crate) frontier: Arc<CommittedFrontier>,
}

/// Anchor a committed segment to the MCU clock, split it into per-axis
/// pieces, and hand each piece to the pump.
pub(crate) fn dispatch_segment(
    ctx: &SegmentDispatchCtx,
    seg: &ShapedSegment,
) -> Result<(), DispatchError> {
    tracing::debug!(
        subsystem = "engine",
        event = "dispatch_entered",
        seg_t_start = seg.t_start,
        seg_t_end = seg.t_end,
        "[engine-trace] dispatch entered"
    );

    let host_now = {
        let r = ctx.router.lock().unwrap_or_else(|p| p.into_inner());
        r.host_now_secs()
    };

    let (t0, fresh) = ctx
        .anchor
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .anchor_segment(seg.t_start, seg.t_end, host_now);

    let runway_s = t0 + seg.t_end - host_now;
    if runway_s > 0.0 {
        ctx.frontier
            .advance_to(Instant::now() + Duration::from_secs_f64(runway_s));
    }

    if fresh {
        let r = ctx.router.lock().unwrap_or_else(|p| p.into_inner());
        for cfg in ctx.mcu_configs.iter() {
            let h = crate::types::mcu_handle_from_raw(cfg.mcu_id);
            r.log_seg0_lead(h, t0 + seg.t_start, t0);
        }
    }

    let project = |mcu_id: u32, host_secs: f64| -> u64 {
        let r = ctx.router.lock().unwrap_or_else(|p| p.into_inner());
        r.host_time_to_mcu_clock(crate::types::mcu_handle_from_raw(mcu_id), host_secs)
            .unwrap_or(0)
    };

    let active_cohort: Option<u64> = *ctx
        .active_drip_cohort
        .lock()
        .unwrap_or_else(|p| p.into_inner());

    let max_piece_secs = if active_cohort.is_some() {
        Some(0.025_f64)
    } else {
        None::<f64>
    };
    let lead_secs = if active_cohort.is_some() {
        crate::pump::DRIP_WINDOW_SECS
    } else {
        crate::pump::MAX_LEAD_SECS
    };

    let msgs = crate::enqueue::enqueue_segment(
        seg,
        &ctx.mcu_configs,
        t0,
        fresh,
        host_now,
        lead_secs,
        project,
        max_piece_secs,
    );

    if fresh {
        ctx.motion_history
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .drop_pieces_on_reanchor();
    }
    for m in msgs {
        ctx.pump_tx.send(m).map_err(|_| DispatchError::PumpGone)?;
    }

    tracing::info!(
        subsystem = "motion",
        event = "pipe_pump_in",
        line = seg.source_line,
        t_us = crate::timing::mono_us(),
        "[pipe] handed to pump"
    );

    ctx.counter.fetch_add(1, Ordering::Relaxed);
    Ok(())
}

/// Anchor a nudge piece to the MCU clock and hand it to the pump — the
/// single-axis, pipeline-bypassing sibling of `dispatch_segment`.
pub(crate) fn dispatch_nudge(
    ctx: &SegmentDispatchCtx,
    mcu_id: u32,
    np: &crate::nudge::NudgePiece,
) -> Result<(), DispatchError> {
    let axis = np.axis;
    if !ctx.mcu_configs.iter().any(|c| c.mcu_id == mcu_id) {
        return Err(DispatchError::NudgeTargetMissing { mcu_id, axis });
    }

    let host_now = {
        let r = ctx.router.lock().unwrap_or_else(|p| p.into_inner());
        r.host_now_secs()
    };

    let active_cohort: Option<u64> = *ctx
        .active_drip_cohort
        .lock()
        .unwrap_or_else(|p| p.into_inner());

    let lead_secs = if active_cohort.is_some() {
        crate::pump::DRIP_WINDOW_SECS
    } else {
        crate::pump::MAX_LEAD_SECS
    };

    let (t0, fresh) = ctx
        .anchor
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .anchor_segment(np.piece.u_start, np.piece.u_end, host_now);

    if fresh {
        let r = ctx.router.lock().unwrap_or_else(|p| p.into_inner());
        let h = crate::types::mcu_handle_from_raw(mcu_id);
        r.log_seg0_lead(h, t0 + np.piece.u_start, t0);
    }

    let project = |proj_mcu_id: u32, host_secs: f64| -> u64 {
        let r = ctx.router.lock().unwrap_or_else(|p| p.into_inner());
        r.host_time_to_mcu_clock(crate::types::mcu_handle_from_raw(proj_mcu_id), host_secs)
            .unwrap_or(0)
    };

    let max_piece_secs = if active_cohort.is_some() {
        Some(0.025_f64)
    } else {
        None::<f64>
    };

    let pieces = crate::enqueue::flatten_bezier_pieces(
        std::slice::from_ref(&np.piece),
        t0,
        mcu_id,
        axis as usize,
        host_now,
        &project,
        max_piece_secs,
        np.motor_mask,
    );

    if !pieces.is_empty() {
        let key = crate::types::AxisKey { mcu_id, axis };
        ctx.pump_tx
            .send(crate::pump::EnqueueMsg {
                key,
                pieces,
                fresh_stream: fresh,
                lead_secs,
                source_line: u32::MAX,
            })
            .map_err(|_| DispatchError::PumpGone)?;
    }

    ctx.counter.fetch_add(1, Ordering::Relaxed);
    Ok(())
}

/// State the pipeline-output consumer thread shares with the worker.
#[derive(Clone)]
pub(crate) struct ConsumerShared {
    pub(crate) dispatch: DispatchFn,
    pub(crate) sync_instant: Arc<Mutex<Option<Instant>>>,
    pub(crate) last_move_time_bits: Arc<AtomicU64>,
    pub(crate) commit_fire_count: Arc<AtomicU32>,
    pub(crate) fences: Arc<crate::fence::FenceRegistry>,
}

impl ConsumerShared {
    pub(crate) fn dispatch_segment(&self, seg: &ShapedSegment) -> Result<(), DispatchError> {
        let n_ax = seg.axes.len();
        tracing::info!(
            subsystem = "motion",
            event = "pipe_dispatch",
            line = seg.source_line,
            t_us = crate::timing::mono_us(),
            seg_t_start = seg.t_start,
            seg_t_end = seg.t_end,
            x_end = if n_ax > 0 {
                nurbs::eval::eval(&seg.axes[0], seg.t_end)
            } else {
                0.0
            },
            y_end = if n_ax > 1 {
                nurbs::eval::eval(&seg.axes[1], seg.t_end)
            } else {
                0.0
            },
            z_end = if n_ax > 2 {
                nurbs::eval::eval(&seg.axes[2], seg.t_end)
            } else {
                0.0
            },
            "[pipe] dispatch"
        );
        (self.dispatch)(seg)?;
        let mut sync = self.sync_instant.lock().unwrap_or_else(|p| p.into_inner());
        if sync.is_none() {
            *sync = Some(Instant::now());
        }
        drop(sync);
        self.last_move_time_bits
            .store(seg.t_end.to_bits(), Ordering::Release);
        self.commit_fire_count.fetch_add(1, Ordering::AcqRel);
        self.fences.on_dispatch(seg.source_line, seg.t_end);
        Ok(())
    }
}

/// Final pipeline consumer: dispatches shaped segments to the pump and
/// services the control tokens that reach the end of the stream. `Barrier`
/// is acknowledged here — everything ahead of it has been dispatched or
/// discarded. The `discard` gate (set out-of-band by reset paths) drops
/// segments until the in-band `Reset` token catches up and lifts it, which
/// makes "clear everything queued, dispatch nothing" exact rather than racy.
pub(crate) struct Consumer {
    pub(crate) shared: ConsumerShared,
    pub(crate) discard: Arc<AtomicBool>,
    /// While set (homing paths), a dispatch error is captured and reported at
    /// the next `Barrier` instead of aborting the process; segments behind a
    /// captured error are dropped.
    pub(crate) capture_errors: Arc<AtomicBool>,
    pub(crate) frontier: Arc<CommittedFrontier>,
    pub(crate) dispatched_through: Option<f64>,
    pub(crate) pending_error: Option<String>,
}

impl Consumer {
    pub(crate) fn run(mut self, output: &Receiver<ShapedItem>) {
        while let Ok(item) = output.recv() {
            match item {
                ShapedItem::Seg(seg) => {
                    if self.discard.load(Ordering::Acquire) || self.pending_error.is_some() {
                        continue;
                    }
                    match self.shared.dispatch_segment(&seg) {
                        Ok(()) => self.dispatched_through = Some(seg.t_end),
                        Err(e) if self.capture_errors.load(Ordering::Acquire) => {
                            self.pending_error = Some(format!("dispatch failed: {e}"));
                        }
                        Err(e) => fatal(&format!("dispatch failed: {e}")),
                    }
                }
                ShapedItem::Control(Control::Barrier(tx)) => {
                    let ack = BarrierAck {
                        dispatched_through: self.dispatched_through,
                        sync_instant: *self
                            .shared
                            .sync_instant
                            .lock()
                            .unwrap_or_else(|p| p.into_inner()),
                        result: self.pending_error.take().map_or(Ok(()), Err),
                    };
                    let _ = tx.send(ack);
                }
                ShapedItem::Control(Control::Reset { .. }) => {
                    self.discard.store(false, Ordering::Release);
                    self.frontier.clear();
                    self.dispatched_through = None;
                    self.shared.fences.on_reset();
                    self.shared
                        .last_move_time_bits
                        .store(0.0_f64.to_bits(), Ordering::Release);
                    *self
                        .shared
                        .sync_instant
                        .lock()
                        .unwrap_or_else(|p| p.into_inner()) = None;
                }
                ShapedItem::Control(Control::Dwell { secs }) => {
                    if let Some(t) = &mut self.dispatched_through {
                        *t += secs;
                    }
                }
                ShapedItem::Control(Control::SetAxisChains(_)) => {}
            }
        }
    }
}
