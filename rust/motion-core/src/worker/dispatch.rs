use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use trajectory::{ContinuousSegment, NudgeProfile};

use motion_pipeline::{BarrierAck, Control, DispatchCommand, TrajectoryItem};

use super::{CommittedFrontier, fatal};
use crate::lock_ext::LockExt;

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
    #[error("{0}")]
    DispatchRecord(String),
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
    fn follow_stream_to(&mut self, stream_t: f64);
    /// How long from now the stream would start if `seg` set it off from
    /// rest; `None` while `seg` continues it in motion. A sink that anchors
    /// nothing sets nothing off.
    fn set_off_in(&self, _seg: &ContinuousSegment) -> Option<Duration> {
        None
    }
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
    /// The runway the pacer keeps for braking to rest: the intake caps the
    /// motion queued ahead on it, and a stream sets off only with that much
    /// committed.
    reserve_bits: AtomicU64,
    set_off_hold: Mutex<Option<SetOffHold>>,
    #[cfg(test)]
    pub(crate) pacer_drains: PacerDrains,
    pub(crate) fences: crate::fence::FenceRegistry,
    pub(crate) start_claims: super::StartClaims,
    pub(crate) wakeup: crate::feed_wakeup::FeedWakeup,
}

/// A stream at rest the dispatcher holds back from setting off.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct SetOffHold {
    pub(crate) since: Instant,
    /// When the held motion would start playing, had it set off on its
    /// first segment: until then it stands ahead of a playhead not yet
    /// running.
    pub(crate) starts: Instant,
    /// Stream time held, from the first segment's start.
    pub(crate) held_s: f64,
    /// Stream time the held segments run through.
    pub(crate) through_s: f64,
}

impl SetOffHold {
    /// The motion standing ahead of the playhead at `now` once the held
    /// motion counts: it starts at `starts`, behind any committed `runway_s`
    /// still playing (a stream parked on a stop it has not reached yet sets
    /// off again from its end), and runs `held_s`.
    pub(crate) fn ahead_secs(&self, runway_s: f64, now: Instant) -> f64 {
        let starts_in_s = self.starts.saturating_duration_since(now).as_secs_f64();
        runway_s.max(starts_in_s) + self.held_s
    }
}

/// Brakes to rest the pacer ordered, by the path that ordered them; a
/// flush's drain is neither.
#[cfg(test)]
#[derive(Default)]
pub(crate) struct PacerDrains {
    /// The runway fell while moves kept arriving.
    pub(crate) busy: AtomicU64,
    /// The inbox was found empty with the runway at the reserve.
    pub(crate) idle: AtomicU64,
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

    pub(crate) fn reserve_secs(&self) -> f64 {
        f64::from_bits(self.reserve_bits.load(Ordering::Acquire))
    }

    pub(crate) fn publish_reserve(&self, reserve_s: f64) {
        self.reserve_bits
            .store(reserve_s.to_bits(), Ordering::Release);
    }

    pub(crate) fn set_off_hold(&self) -> Option<SetOffHold> {
        *self.set_off_hold.lock_ok()
    }

    /// The motion standing ahead of the playhead at `now`: the committed
    /// `runway_s` and a held set-off. The intake bounds it and the pacer
    /// waits on it alike, or the intake would refuse moves to a front the
    /// pacer brakes for want of them.
    pub(crate) fn ahead_secs(&self, runway_s: f64, now: Instant) -> f64 {
        self.set_off_hold()
            .map_or(runway_s, |hold| hold.ahead_secs(runway_s, now))
    }
}

/// Final pipeline stage: dispatches shaped segments into the sink and
/// services the control tokens that reach the end of the stream. `Barrier`
/// is acknowledged here — everything ahead of it has been dispatched or
/// discarded. Segments behind a captured error are dropped until the error
/// is reported at the next `Barrier`.
///
/// A stream at rest sets off only once the pacer's reserve is committed past
/// its start, it ends at rest, or a control token follows it: the planning
/// front releases the first segments of a stream while it still holds the
/// motion behind them (#983: the lowerer's 8.4 ms rest hold, then 0.3-0.8 s
/// of planning on the Pi), and a stream started on less could not be braked
/// before its playhead overran. A homing or probing move sets off as it
/// comes: it is drained the moment it is sent, so nothing waits behind it,
/// and its trigger timing stays as it was.
pub(crate) struct Dispatcher {
    links: Arc<WorkerLinks>,
    frontier: Arc<CommittedFrontier>,
    dispatched_through: Option<f64>,
    stream_time: f64,
    pending_error: Option<String>,
    terminal_halt: Option<String>,
    setting_off: Vec<ContinuousSegment>,
}

impl Dispatcher {
    pub(crate) fn new(links: Arc<WorkerLinks>, frontier: Arc<CommittedFrontier>) -> Self {
        Self {
            links,
            frontier,
            dispatched_through: None,
            stream_time: 0.0,
            pending_error: None,
            terminal_halt: None,
            setting_off: Vec::new(),
        }
    }

    pub(crate) fn halt(&mut self, reason: &str) {
        if self.terminal_halt.is_none() {
            self.terminal_halt = Some(reason.to_string());
            self.dispatched_through = None;
            self.drop_hold();
            self.frontier.clear();
            self.links.fences.on_reset();
            self.links.wakeup.notify_fence_resolved();
        }
    }

    pub(crate) fn feed(&mut self, item: TrajectoryItem, sink: &mut impl SegmentSink) {
        match item {
            TrajectoryItem::Seg(seg) => self.handle_segment(seg, sink),
            TrajectoryItem::Parked => {
                self.set_off(sink);
                sink.mark_parked();
            }
            TrajectoryItem::Control(ctrl) => {
                self.set_off(sink);
                self.handle_control(ctrl, sink);
            }
        }
    }

    fn drops_segments(&self) -> bool {
        self.links.discard.load(Ordering::Acquire)
            || self.links.shutting_down.load(Ordering::Acquire)
            || self.pending_error.is_some()
            || self.terminal_halt.is_some()
    }

    fn handle_segment(&mut self, seg: ContinuousSegment, sink: &mut impl SegmentSink) {
        if self.drops_segments() {
            self.drop_hold();
            return;
        }
        let homing = self.links.finite_homing_admission.load(Ordering::Acquire);
        let starts_in = if self.setting_off.is_empty() && !homing {
            sink.set_off_in(&seg)
        } else {
            None
        };
        if self.setting_off.is_empty() && starts_in.is_none() {
            self.dispatch(&seg, sink);
            return;
        }
        self.hold(seg, starts_in);
        if self.held_off_secs() >= self.links.reserve_secs() {
            self.set_off(sink);
        }
    }

    /// Holds `seg` back for the set-off, which starts `starts_in` from now
    /// when `seg` is its first segment.
    fn hold(&mut self, seg: ContinuousSegment, starts_in: Option<Duration>) {
        let now = Instant::now();
        let (since, starts) = match (self.links.set_off_hold(), starts_in) {
            (Some(hold), _) => (hold.since, hold.starts),
            (None, Some(starts_in)) => (now, now + starts_in),
            (None, None) => unreachable!("a set-off holds its first segment with its start"),
        };
        let through_s = seg.t_end;
        self.setting_off.push(seg);
        *self.links.set_off_hold.lock_ok() = Some(SetOffHold {
            since,
            starts,
            held_s: self.held_off_secs(),
            through_s,
        });
    }

    fn drop_hold(&mut self) {
        self.setting_off.clear();
        *self.links.set_off_hold.lock_ok() = None;
    }

    /// Stream time held back for the set-off, from its first segment on.
    fn held_off_secs(&self) -> f64 {
        match (self.setting_off.first(), self.setting_off.last()) {
            (Some(first), Some(last)) => last.t_end - first.t_start,
            _ => 0.0,
        }
    }

    /// Dispatches the segments held back for the set-off, in order.
    fn set_off(&mut self, sink: &mut impl SegmentSink) {
        let held = std::mem::take(&mut self.setting_off);
        self.drop_hold();
        for seg in held {
            if self.drops_segments() {
                return;
            }
            self.dispatch(&seg, sink);
        }
    }

    fn dispatch(&mut self, seg: &ContinuousSegment, sink: &mut impl SegmentSink) {
        log_dispatch(seg);
        match sink.dispatch(seg) {
            Ok(()) => {
                self.dispatched_through = Some(seg.t_end);
                self.stream_time = seg.t_end;
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
                self.stream_time = 0.0;
                sink.follow_stream_to(self.stream_time);
            }
            Control::Dwell { secs } => {
                self.stream_time += secs;
                sink.follow_stream_to(self.stream_time);
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

#[cfg(test)]
mod tests;
