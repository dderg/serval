//! Owns ingress continuity, synchronous numerical planning, and measured runway pacing.

use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use crossbeam_channel::Receiver;

use motion_pipeline::{
    BarrierAck, CONTIGUITY_EPS_MM, Control, DispatchCommand, StreamConfig, StreamError,
    StreamInput, advance_odometer,
};

use super::dispatch::WorkerLinks;
use super::{CommittedFrontier, HomeDripParams, NudgeParams, StreamMsg, fatal};

const DRAIN_RESERVE_FLOOR_S: f64 = 0.5;
const DRAIN_RESERVE_SAFETY: f64 = 2.0;

// TODO: expose as a config knob if 250 ms turns out wrong for slower feeds.
const STARTUP_PRIME_S: f64 = 0.250;

pub(super) struct Ingress {
    pub(super) config: StreamConfig,
    /// Expected toolhead position after every move ingested so far; the
    /// ingress contiguity check anchors here.
    pub(super) odometer: Vec<f64>,
    /// Stream time the dispatched timeline has reached, mirrored from barrier
    /// acks; nudge profiles are planned from it.
    pub(super) t_next: f64,
    pub(super) pipeline: motion_pipeline::Pipeline,
    pub(super) output: crossbeam_channel::Sender<motion_pipeline::TrajectoryItem>,
    pub(super) links: Arc<WorkerLinks>,
    pub(super) frontier: Arc<CommittedFrontier>,
    pub(super) undrained_since: Option<Instant>,
    pub(super) worst_drain_s: f64,
    /// Source line of the last move forwarded into the pipeline; a fence
    /// arriving now sequences after it.
    pub(super) last_line: u32,
    pub(super) pump: crossbeam_channel::Sender<crate::pump::PumpMsg>,
}

impl Ingress {
    fn reserve_secs(&self) -> f64 {
        (self.worst_drain_s * DRAIN_RESERVE_SAFETY).max(DRAIN_RESERVE_FLOOR_S)
    }

    pub(super) fn run(mut self, rx: Receiver<StreamMsg>) {
        loop {
            if self.links.shutting_down.load(Ordering::Acquire) {
                return;
            }
            let received = if self.undrained_since.is_some() {
                match rx.try_recv() {
                    Ok(msg) => Some(msg),
                    Err(crossbeam_channel::TryRecvError::Empty) => {
                        let output = &self.output;
                        if !self.pipeline.idle(&mut |item| output.send(item).is_ok()) {
                            if self.links.shutting_down.load(Ordering::Acquire) {
                                return;
                            }
                            fatal("execution owner closed during planning idle");
                        }
                        match self.drain_or_runway() {
                            None => continue,
                            Some(wait) => match rx.recv_timeout(wait) {
                                Ok(msg) => Some(msg),
                                Err(crossbeam_channel::RecvTimeoutError::Timeout) => continue,
                                Err(crossbeam_channel::RecvTimeoutError::Disconnected) => None,
                            },
                        }
                    }
                    Err(crossbeam_channel::TryRecvError::Disconnected) => None,
                }
            } else {
                rx.recv().ok()
            };
            let Some(msg) = received else {
                self.drain_and_fence();
                return;
            };
            self.links.wakeup.notify_space_freed();
            match msg {
                StreamMsg::Move(m) => self.handle_move(m),
                other => {
                    if self.handle_control(other) {
                        return;
                    }
                }
            }
        }
    }

    fn send(&mut self, item: StreamInput) {
        if self.links.shutting_down.load(Ordering::Acquire) {
            return;
        }
        let output = &self.output;
        if !self
            .pipeline
            .feed(item, &mut |item| output.send(item).is_ok())
            && !self.links.shutting_down.load(Ordering::Acquire)
        {
            fatal("execution owner closed during planning");
        }
    }

    /// Fence: everything sent before this has been dispatched (or discarded)
    /// once it returns. Advances the ingress's timeline mirror.
    fn barrier(&mut self) -> BarrierAck {
        if self.links.shutting_down.load(Ordering::Acquire) {
            return BarrierAck {
                dispatched_through: None,
                result: Err("planning cancelled for shutdown".to_string()),
            };
        }
        let (tx, rx) = crossbeam_channel::bounded(1);
        self.send(StreamInput::Control(Control::Dispatch(
            DispatchCommand::Barrier(tx),
        )));
        let ack = match rx.recv() {
            Ok(ack) => ack,
            Err(_) if self.links.shutting_down.load(Ordering::Acquire) => BarrierAck {
                dispatched_through: None,
                result: Err("planning cancelled for shutdown".to_string()),
            },
            Err(_) => fatal("execution owner dropped an intake barrier"),
        };
        if let Some(t) = ack.dispatched_through {
            self.t_next = t;
        }
        if self.links.fences.resolve_armed(ack.dispatched_through) {
            self.links.wakeup.notify_fence_resolved();
        }
        ack
    }

    /// Drain the lookahead and fence: the pipeline is empty and the full
    /// braked-to-rest trajectory is dispatched when this returns, so no
    /// intake remains uncommitted. Every drain runs through here, so every
    /// drain measures the traversal the pacer's reserve has to cover.
    fn drain_and_fence(&mut self) -> BarrierAck {
        let sent = Instant::now();
        self.send(StreamInput::Drain);
        self.undrained_since = None;
        let ack = self.barrier();
        let latency_s = sent.elapsed().as_secs_f64();
        if latency_s > self.worst_drain_s {
            self.worst_drain_s = latency_s;
            tracing::info!(
                subsystem = "motion",
                event = "pacer_reserve_raised",
                latency_s,
                reserve_s = self.reserve_secs(),
                "[pacer] slowest brake-to-rest yet — widening the runway the \
                 pacer keeps for the next one"
            );
        }
        ack
    }

    fn handle_move(&mut self, m: geometry::Move) {
        tracing::trace!(
            subsystem = "motion",
            event = "pipe_ingress",
            line = m.source.start_line,
            t_us = motion_pipeline::timing::mono_us(),
            "[pipe] ingress"
        );
        if let Some(seg) = &m.segment.spatial {
            use geometry::path::lowering::PositionProfile;
            let got = seg.point_at(0.0);
            let expected = [self.odometer[0], self.odometer[1], self.odometer[2]];
            let gap_mm = geometry::vec3::dist(expected, got);
            if gap_mm > CONTIGUITY_EPS_MM {
                fatal(
                    &StreamError::Discontinuity {
                        line_no: m.source.start_line,
                        expected,
                        got,
                        gap_mm,
                    }
                    .to_string(),
                );
            }
        }
        advance_odometer(&mut self.odometer, &m);
        self.last_line = m.source.start_line;
        self.send(m.into());
        self.undrained_since.get_or_insert_with(Instant::now);
    }

    /// The pacer's one decision. Called when the inbox is silent while the
    /// pipeline holds undrained moves: with runway beyond the reserve there is
    /// provably time to wait for more input, so report how long; at the
    /// reserve, send `Drain` so the fit stage and planner materialize the
    /// brake-to-rest and the drained trajectory beats the playhead to the
    /// pump.
    fn drain_or_runway(&mut self) -> Option<Duration> {
        let wait_s = self.frontier.runway_secs() - self.reserve_secs();
        if wait_s > 0.0 {
            return Some(Duration::from_secs_f64(wait_s));
        }
        if let Some(since) = self.undrained_since {
            let remaining =
                Duration::from_secs_f64(STARTUP_PRIME_S).saturating_sub(since.elapsed());
            if !remaining.is_zero() {
                return Some(remaining);
            }
        }
        tracing::debug!(
            subsystem = "motion",
            event = "pipe_drain",
            t_us = motion_pipeline::timing::mono_us(),
            "[pipe] runway exhausted — draining pipeline to rest"
        );
        self.drain_and_fence();
        None
    }

    /// Handle one non-move control message. Returns `true` when the loop
    /// should exit (shutdown).
    fn handle_control(&mut self, msg: StreamMsg) -> bool {
        match msg {
            StreamMsg::Move(_) => unreachable!("moves handled by the ingress path"),
            StreamMsg::Flush { notify } => {
                let ack = self.drain_and_fence();
                let _ = notify.send(ack.result);
            }
            StreamMsg::Fence { id, force } => {
                if self.undrained_since.is_none() {
                    let ack = self.barrier();
                    self.links.fences.resolve(id, ack.dispatched_through);
                    self.links.wakeup.notify_fence_resolved();
                } else if force {
                    let ack = self.drain_and_fence();
                    self.links.fences.resolve(id, ack.dispatched_through);
                    self.links.wakeup.notify_fence_resolved();
                } else {
                    self.links.fences.arm(id, self.last_line);
                }
            }
            StreamMsg::Dwell { duration_s, notify } => {
                self.drain_and_fence();
                if duration_s > 0.0 {
                    self.send(StreamInput::Control(Control::Dwell { secs: duration_s }));
                    let before = self.t_next;
                    if self.barrier().dispatched_through.is_none() {
                        self.t_next = before + duration_s;
                    }
                }
                let _ = notify.send(());
            }
            StreamMsg::Reset { pos } => {
                self.reset_to(pos);
            }
            StreamMsg::SetAxisChains(chains) => {
                self.drain_and_fence();
                self.send(StreamInput::Control(Control::SetAxisChains(chains)));
                self.barrier();
            }
            StreamMsg::SetMesh {
                mesh,
                gcode_z_rebase,
                notify,
            } => {
                self.drain_and_fence();
                self.odometer[2] = gcode_z_rebase;
                self.send(StreamInput::Control(Control::SetMesh {
                    mesh,
                    gcode_z_rebase,
                }));
                self.barrier();
                let _ = notify.send(());
            }
            StreamMsg::HomeDrip { params, notify } => {
                let result = self.run_home_drip(&params);
                let _ = notify.send(result);
            }
            StreamMsg::Nudge { params, notify } => {
                let result = self.run_nudge(&params);
                let _ = notify.send(result);
            }
            StreamMsg::Buzz { params, notify } => {
                let result = self.run_buzz(params);
                let _ = notify.send(result);
            }
            StreamMsg::Shutdown => {
                self.drain_and_fence();
                return true;
            }
        }
        false
    }

    /// Drop everything queued without dispatching it and restart the timeline
    /// at rest at `pos`. The discard gate goes up out-of-band (segments
    /// already past the shaper are dropped immediately) and the in-band
    /// `Reset` lifts it when it catches up, so nothing sent before this call
    /// reaches the pump and everything sent after does.
    fn reset_to(&mut self, pos: Vec<f64>) {
        self.links.discard.store(true, Ordering::Release);
        self.frontier.clear();
        self.send(StreamInput::Control(Control::Reset { pos: pos.clone() }));
        self.undrained_since = None;
        self.barrier();
        self.odometer = pos;
        self.t_next = 0.0;
    }

    /// Run a homing drip through the pipeline with dispatch errors captured,
    /// so a failure surfaces to the homing caller instead of aborting.
    fn run_home_drip(&mut self, p: &HomeDripParams) -> Result<(), String> {
        self.reset_to(p.home_pos.to_vec());
        let travel = p.direction * p.max_travel_mm;
        let (dx, dy, dz) = match p.axis {
            0 => (travel, 0.0, 0.0),
            1 => (0.0, travel, 0.0),
            2 => (0.0, 0.0, travel),
            other => return Err(format!("HomeDrip: unsupported axis {other} (only 0/1/2)")),
        };
        let m = crate::classify::build_move(
            p.start,
            [dx, dy, dz],
            0,
            0.0,
            self.config.limits,
            p.speed_mm_s,
            0,
        )
        .map_err(|e| format!("HomeDrip build_move: {e:?}"))?;
        advance_odometer(&mut self.odometer, &m);

        self.links
            .finite_homing_admission
            .store(true, Ordering::Release);
        self.send(m.into());
        self.undrained_since.get_or_insert_with(Instant::now);
        let ack = self.drain_and_fence();
        self.links
            .finite_homing_admission
            .store(false, Ordering::Release);
        ack.result
    }

    /// Plan a nudge profile from the current stream time and send it down the
    /// (drained) pipeline as a control token; the dispatcher executes it and
    /// the closing barrier carries back any dispatch error. The `Dwell`
    /// advances the stream clock over the nudge's duration.
    fn run_nudge(&mut self, p: &NudgeParams) -> Result<(), String> {
        self.drain_and_fence().result?;
        let profile =
            crate::nudge::plan_nudge_profile(p.axis, p.delta_mm, p.speed, p.accel, self.t_next)?;
        let total_dur = profile.duration();
        self.send(StreamInput::Control(Control::Dispatch(
            DispatchCommand::Nudge {
                mcu_id: p.mcu_id,
                axis: p.axis,
                motor_mask: p.motor_mask,
                profile,
            },
        )));
        if total_dur > 0.0 {
            self.send(StreamInput::Control(Control::Dwell { secs: total_dur }));
        }
        self.t_next += total_dur;
        self.barrier().result.map_err(|e| format!("nudge: {e}"))
    }

    /// Arm every route of one buzz request. The pipeline is drained and the
    /// pump fenced exactly once before the request goes out, so no route can
    /// be holding queued trajectory when the pump validates it; the pump then
    /// answers with a single token spanning the whole route set, or an error
    /// with nothing armed.
    fn run_buzz(
        &mut self,
        params: crate::pump::BuzzParams,
    ) -> Result<crate::pump::BuzzToken, String> {
        self.drain_and_fence().result?;
        let pump = &self.pump;
        let (reply, verdict) = std::sync::mpsc::sync_channel(1);
        if pump
            .send(crate::pump::PumpMsg::Buzz { params, reply })
            .is_err()
        {
            fatal("execution owner closed during buzz arming");
        }
        match verdict.recv_timeout(Duration::from_secs(5)) {
            Ok(result) => result,
            Err(_) => fatal("execution owner did not answer the buzz request within 5s"),
        }
    }
}

#[cfg(test)]
mod tests;
