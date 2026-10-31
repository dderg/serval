use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};

use trajectory::{ContinuousSegment, NudgeProfile};

use motion_pipeline::{BarrierAck, Control, DispatchCommand, TrajectoryItem};

use super::{CommittedFrontier, fatal};

#[derive(Debug, thiserror::Error)]
pub enum DispatchError {
    #[error(
        "mcu {mcu_id} (mcu_h={mcu_handle:?}) has no converged clocksync record — \
         refusing to anchor a step stream on it. The record is invalidated by every \
         (re)connect and re-armed only by a converged clocksync estimate; anchoring \
         without one sends step clocks from the previous boot epoch."
    )]
    ClockRecordUnusable {
        mcu_id: u32,
        mcu_handle: host_rt::passthrough_queue::McuHandle,
    },
    #[error(
        "mcu {mcu_id} (mcu_h={mcu_handle:?}) clocksync record is {age_secs:.3}s old \
         (limit {max_age_secs:.3}s) — clocksync has stopped feeding the router, so \
         anchoring would project the step stream off a dead estimate. Note that a \
         healthy record's regression centroid legitimately trails now by up to 30 \
         get_clock periods; this age counts only the missed router updates."
    )]
    ClockRecordStale {
        mcu_id: u32,
        mcu_handle: host_rt::passthrough_queue::McuHandle,
        age_secs: f64,
        max_age_secs: f64,
    },
    #[error("execution halted on a fatal endpoint condition: {0}")]
    ExecutionHalted(String),
    #[error("nudge target mcu_id={mcu_id} axis={axis} not present in mcu_configs")]
    NudgeTargetMissing { mcu_id: u32, axis: u8 },
    #[error("enqueue: {0}")]
    Enqueue(#[from] trajectory::ContinuousError),
}

/// Where committed motion goes when it reaches the end of the pipeline.
/// Production uses [`super::pump_sink::PumpSink`] (clock anchoring + per-axis
/// span enqueue into the pump); tests substitute a capture.
pub(crate) trait SegmentSink {
    fn dispatch(&mut self, seg: &ContinuousSegment) -> Result<(), DispatchError>;
    fn dispatch_nudge(
        &mut self,
        mcu_id: u32,
        axis: u8,
        motor_mask: u8,
        profile: &NudgeProfile,
    ) -> Result<(), DispatchError>;
    /// The committed trajectory is planned to rest through its end; a resume
    /// across the idle gap that follows may re-anchor the timeline forward.
    fn mark_parked(&mut self) {}
}

#[derive(Default)]
pub(crate) struct WorkerLinks {
    /// Raised out-of-band by reset paths so segments already past the shaper
    /// are dropped immediately; the in-band `Reset` token lowers it when it
    /// catches up.
    pub(crate) discard: AtomicBool,
    pub(crate) finite_homing_admission: AtomicBool,
    pub(crate) shutting_down: AtomicBool,
    pub(crate) last_move_time_bits: AtomicU64,
    pub(crate) fences: crate::fence::FenceRegistry,
    pub(crate) wakeup: crate::feed_wakeup::FeedWakeup,
}

impl WorkerLinks {
    pub(crate) fn bypasses_capacity(&self, item: &TrajectoryItem, cohort_active: bool) -> bool {
        self.discard.load(Ordering::Acquire)
            || self.shutting_down.load(Ordering::Acquire)
            || (cohort_active
                && self.finite_homing_admission.load(Ordering::Acquire)
                && matches!(item, TrajectoryItem::Seg(_)))
            || !matches!(
                item,
                TrajectoryItem::Seg(_)
                    | TrajectoryItem::Control(Control::Dispatch(DispatchCommand::Nudge { .. }))
            )
    }
}

/// Final pipeline stage: dispatches shaped segments into the sink and
/// services the control tokens that reach the end of the stream. `Barrier`
/// is acknowledged here — everything ahead of it has been dispatched or
/// discarded. Segments behind a captured error are dropped until the error
/// is reported at the next `Barrier`.
pub(crate) struct Dispatcher {
    links: Arc<WorkerLinks>,
    frontier: Arc<CommittedFrontier>,
    dispatched_through: Option<f64>,
    pending_error: Option<String>,
    terminal_halt: Option<String>,
}

impl Dispatcher {
    pub(crate) fn new(links: Arc<WorkerLinks>, frontier: Arc<CommittedFrontier>) -> Self {
        Self {
            links,
            frontier,
            dispatched_through: None,
            pending_error: None,
            terminal_halt: None,
        }
    }

    pub(crate) fn halt(&mut self, reason: &str) {
        if self.terminal_halt.is_none() {
            self.terminal_halt = Some(reason.to_string());
            self.dispatched_through = None;
            self.frontier.clear();
            self.links.fences.on_reset();
            self.links.wakeup.notify_fence_resolved();
        }
    }

    pub(crate) fn feed(&mut self, item: TrajectoryItem, sink: &mut impl SegmentSink) {
        match item {
            TrajectoryItem::Seg(seg) => self.handle_segment(&seg, sink),
            TrajectoryItem::Parked => sink.mark_parked(),
            TrajectoryItem::Control(ctrl) => self.handle_control(ctrl, sink),
        }
    }

    fn handle_segment(&mut self, seg: &ContinuousSegment, sink: &mut impl SegmentSink) {
        if self.links.discard.load(Ordering::Acquire)
            || self.links.shutting_down.load(Ordering::Acquire)
            || self.pending_error.is_some()
            || self.terminal_halt.is_some()
        {
            return;
        }
        log_dispatch(seg);
        match sink.dispatch(seg) {
            Ok(()) => {
                self.dispatched_through = Some(seg.t_end);
                self.publish_progress(seg.t_end);
                if self.links.fences.on_dispatch(seg.source_line, seg.t_end) {
                    self.links.wakeup.notify_fence_resolved();
                }
            }
            Err(e) if self.links.shutting_down.load(Ordering::Acquire) => {
                tracing::debug!(
                    subsystem = "motion",
                    event = "dispatch_interrupted_by_shutdown",
                    error = %e,
                    "dispatch stopped after shutdown closed the pump"
                );
            }
            Err(e) if self.links.finite_homing_admission.load(Ordering::Acquire) => {
                self.pending_error = Some(format!("dispatch failed: {e}"));
            }
            Err(DispatchError::ExecutionHalted(reason)) => {
                self.halt(&reason);
            }
            Err(e) => fatal(&format!("dispatch failed: {e}")),
        }
    }

    fn publish_progress(&mut self, t_end: f64) {
        self.links
            .last_move_time_bits
            .store(t_end.to_bits(), Ordering::Release);
    }

    fn handle_control(&mut self, ctrl: Control, sink: &mut impl SegmentSink) {
        match ctrl {
            Control::Dispatch(DispatchCommand::Barrier(tx)) => {
                let ack = BarrierAck {
                    dispatched_through: self.dispatched_through,
                    result: self
                        .terminal_halt
                        .clone()
                        .or_else(|| self.pending_error.take())
                        .map_or(Ok(()), Err),
                };
                let _ = tx.send(ack);
            }
            Control::Reset { .. } => {
                self.links.discard.store(false, Ordering::Release);
                self.frontier.clear();
                self.dispatched_through = None;
                self.links.fences.on_reset();
                self.links
                    .last_move_time_bits
                    .store(0.0_f64.to_bits(), Ordering::Release);
            }
            Control::Dwell { secs } => {
                if let Some(t) = &mut self.dispatched_through {
                    *t += secs;
                    self.links
                        .last_move_time_bits
                        .store(t.to_bits(), Ordering::Release);
                }
            }
            Control::Dispatch(DispatchCommand::Nudge {
                mcu_id,
                axis,
                motor_mask,
                profile,
            }) => self.handle_nudge(mcu_id, axis, motor_mask, &profile, sink),
            Control::SetAxisChains(_) | Control::SetMesh { .. } => {}
        }
    }

    /// Nudge errors are never fatal: the sender always follows a nudge with a
    /// `Barrier`, so the error reaches the caller through the ack.
    fn handle_nudge(
        &mut self,
        mcu_id: u32,
        axis: u8,
        motor_mask: u8,
        profile: &NudgeProfile,
        sink: &mut impl SegmentSink,
    ) {
        if self.pending_error.is_some()
            || self.terminal_halt.is_some()
            || self.links.discard.load(Ordering::Acquire)
            || self.links.shutting_down.load(Ordering::Acquire)
        {
            return;
        }
        if let Err(e) = sink.dispatch_nudge(mcu_id, axis, motor_mask, profile) {
            self.pending_error = Some(format!("nudge dispatch: {e}"));
            return;
        }
        self.links
            .last_move_time_bits
            .store(profile.t_end().to_bits(), Ordering::Release);
    }
}

fn log_dispatch(seg: &ContinuousSegment) {
    let n_ax = seg.axes.len();
    let end_of = |i: usize| {
        if n_ax > i {
            seg.eval_axis(i, seg.t_end)
                .map_or(f64::NAN, |pva| pva.position)
        } else {
            0.0
        }
    };
    tracing::trace!(
        subsystem = "motion",
        event = "pipe_dispatch",
        line = seg.source_line,
        t_us = motion_pipeline::timing::mono_us(),
        seg_t_start = seg.t_start,
        seg_t_end = seg.t_end,
        x_end = end_of(0),
        y_end = end_of(1),
        z_end = end_of(2),
        "[pipe] dispatch"
    );
}
