//! Terminal pipeline stage: consumes shaped segments and control tokens from
//! the shaper and drives a [`SegmentSink`]. Follows the same stage pattern as
//! the pure stages in `motion_pipeline` — a struct owning its state with a
//! `run(input)` loop — except its output is the sink rather than a channel,
//! because dispatch is where the stream leaves the pure-stage world.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::time::Instant;

use crossbeam_channel::Receiver;
use trajectory::ShapedSegment;

use motion_pipeline::{BarrierAck, Control, NudgePiece, ShapedItem};

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

/// Where committed motion goes when it reaches the end of the pipeline.
/// Production uses [`super::pump_sink::PumpSink`] (clock anchoring + per-axis
/// piece enqueue into the pump); tests substitute a capture.
pub trait SegmentSink: Send + 'static {
    fn dispatch(&mut self, seg: &ShapedSegment) -> Result<(), DispatchError>;
    fn dispatch_nudge(&mut self, mcu_id: u32, piece: &NudgePiece) -> Result<(), DispatchError>;
}

/// State the ingress, dispatcher, and worker handle share. Everything here is
/// either a gate the ingress raises out-of-band (`discard`, `capture_errors`)
/// or progress telemetry the dispatcher publishes.
#[derive(Default)]
pub(crate) struct WorkerLinks {
    /// Raised out-of-band by reset paths so segments already past the shaper
    /// are dropped immediately; the in-band `Reset` token lowers it when it
    /// catches up.
    pub(crate) discard: AtomicBool,
    /// While set (homing paths), a dispatch error is captured and reported at
    /// the next `Barrier` instead of aborting the process.
    pub(crate) capture_errors: AtomicBool,
    pub(crate) shutting_down: AtomicBool,
    pub(crate) last_move_time_bits: AtomicU64,
    pub(crate) commit_fire_count: AtomicU32,
    pub(crate) fences: crate::fence::FenceRegistry,
    pub(crate) wakeup: crate::feed_wakeup::FeedWakeup,
}

/// Final pipeline stage: dispatches shaped segments into the sink and
/// services the control tokens that reach the end of the stream. `Barrier`
/// is acknowledged here — everything ahead of it has been dispatched or
/// discarded. Segments behind a captured error are dropped until the error
/// is reported at the next `Barrier`.
pub(crate) struct Dispatcher<S> {
    sink: S,
    links: Arc<WorkerLinks>,
    frontier: Arc<CommittedFrontier>,
    /// Host instant of the first dispatch since the last reset, for
    /// projecting stream time onto the wall clock.
    sync_instant: Option<Instant>,
    dispatched_through: Option<f64>,
    pending_error: Option<String>,
}

impl<S: SegmentSink> Dispatcher<S> {
    pub(crate) fn new(sink: S, links: Arc<WorkerLinks>, frontier: Arc<CommittedFrontier>) -> Self {
        Self {
            sink,
            links,
            frontier,
            sync_instant: None,
            dispatched_through: None,
            pending_error: None,
        }
    }

    pub(crate) fn run(mut self, input: &Receiver<ShapedItem>) {
        while let Ok(item) = input.recv() {
            match item {
                ShapedItem::Seg(seg) => self.handle_segment(&seg),
                ShapedItem::Control(ctrl) => self.handle_control(ctrl),
            }
        }
    }

    fn handle_segment(&mut self, seg: &ShapedSegment) {
        if self.links.discard.load(Ordering::Acquire)
            || self.links.shutting_down.load(Ordering::Acquire)
            || self.pending_error.is_some()
        {
            return;
        }
        log_dispatch(seg);
        match self.sink.dispatch(seg) {
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
            Err(e) if self.links.capture_errors.load(Ordering::Acquire) => {
                self.pending_error = Some(format!("dispatch failed: {e}"));
            }
            Err(e) => fatal(&format!("dispatch failed: {e}")),
        }
    }

    fn publish_progress(&mut self, t_end: f64) {
        if self.sync_instant.is_none() {
            self.sync_instant = Some(Instant::now());
        }
        self.links
            .last_move_time_bits
            .store(t_end.to_bits(), Ordering::Release);
        self.links.commit_fire_count.fetch_add(1, Ordering::AcqRel);
    }

    fn handle_control(&mut self, ctrl: Control) {
        match ctrl {
            Control::Barrier(tx) => {
                let ack = BarrierAck {
                    dispatched_through: self.dispatched_through,
                    sync_instant: self.sync_instant,
                    result: self.pending_error.take().map_or(Ok(()), Err),
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
                self.sync_instant = None;
            }
            Control::Dwell { secs } => {
                if let Some(t) = &mut self.dispatched_through {
                    *t += secs;
                    self.links
                        .last_move_time_bits
                        .store(t.to_bits(), Ordering::Release);
                }
            }
            Control::Nudge { mcu_id, pieces } => self.handle_nudge(mcu_id, &pieces),
            Control::SetAxisChains(_) | Control::SetMesh { .. } => {}
        }
    }

    /// Nudge errors are never fatal: the sender always follows a nudge with a
    /// `Barrier`, so the error reaches the caller through the ack.
    fn handle_nudge(&mut self, mcu_id: u32, pieces: &[NudgePiece]) {
        if self.pending_error.is_some() {
            return;
        }
        for p in pieces {
            if let Err(e) = self.sink.dispatch_nudge(mcu_id, p) {
                self.pending_error = Some(format!("nudge dispatch: {e}"));
                return;
            }
        }
        if let Some(t_end) = pieces.last().map(|p| p.piece.u_end) {
            self.links
                .last_move_time_bits
                .store(t_end.to_bits(), Ordering::Release);
        }
    }
}

fn log_dispatch(seg: &ShapedSegment) {
    let n_ax = seg.axes.len();
    let end_of = |i: usize| {
        if n_ax > i {
            nurbs::eval::eval(&seg.axes[i], seg.t_end)
        } else {
            0.0
        }
    };
    tracing::trace!(
        subsystem = "motion",
        event = "pipe_dispatch",
        line = seg.source_line,
        t_us = crate::timing::mono_us(),
        seg_t_start = seg.t_start,
        seg_t_end = seg.t_end,
        x_end = end_of(0),
        y_end = end_of(1),
        z_end = end_of(2),
        "[pipe] dispatch"
    );
}
