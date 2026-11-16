//! Owns ingress continuity, fitting, velocity planning and lowering (the
//! shaper runs behind them on its own thread), and measured runway pacing.

use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use crossbeam_channel::Receiver;

use motion_pipeline::{
    BarrierAck, CONTIGUITY_EPS_MM, Control, DispatchCommand, StreamConfig, StreamError,
    StreamInput, advance_odometer,
};

use super::dispatch::WorkerLinks;
use super::intake_gate::{self, IntakeGate};
use super::{ExecutionTimeline, HomeDripParams, NudgeParams, StreamMsg, fatal};
use crate::dispatch_record::{DispatchRecorder, DrainCause};
use crate::lock_ext::LockExt;

mod pacer;

use pacer::IdleVerdict;
pub(super) use pacer::Pacer;

/// Proof the ingress just found its inbox empty: the idle path's runway
/// rule only holds then, and a stream whose moves keep arriving is paced by
/// [`BusyRunwayWatch`] instead.
pub(super) struct InboxFoundEmpty(());

/// Why the pacer braked a stream to rest.
#[derive(Debug, Clone, Copy)]
enum BrakeCause {
    /// The runway fell while moves kept arriving.
    BusyRunwayFalling,
    /// The inbox was found empty with the runway at the reserve.
    RunwayExhausted,
}

/// What a brake to rest is measured to cost, the reserve's two terms.
#[derive(Debug, Clone, Copy)]
enum ReserveCost {
    /// Runway lost while planning, between two pacer decisions.
    PlanningStep,
    /// Runway the brake itself consumed.
    Drain,
}

/// What handed planned motion on: one planning step, or a drain, which
/// hands on everything the planning front held.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Handoff {
    Step,
    Drain,
}

pub(super) struct Ingress {
    pub(super) config: StreamConfig,
    /// Expected toolhead position after every move ingested so far; the
    /// ingress contiguity check anchors here.
    pub(super) odometer: Vec<f64>,
    /// Stream time the dispatched timeline has reached, mirrored from barrier
    /// acks; nudge profiles are planned from it.
    pub(super) t_next: f64,
    pub(super) pipeline: motion_pipeline::StagedPipeline,
    pub(super) links: Arc<WorkerLinks>,
    pub(super) timeline: ExecutionTimeline,
    pub(super) pacer: Pacer,
    pub(super) throttle: Option<super::producer_throttle::ProducerThrottle>,
    /// Source line of the last move forwarded into the pipeline; a fence
    /// arriving now sequences after it.
    pub(super) last_line: u32,
    /// `StreamInput::Move`s fed into the pipeline since it started.
    pub(super) moves_sent: u64,
    pub(super) dispatch_record: Option<Arc<DispatchRecorder>>,
    pub(super) pump: crossbeam_channel::Sender<crate::pump::PumpMsg>,
    pub(super) intake: Arc<IntakeGate>,
}

impl Ingress {
    /// A brake the pacer orders because the producer fell behind. While
    /// committed motion still runs ahead of the playhead, the motion that
    /// resumes after the brake must start the whole reserve ahead of it: the
    /// producer is still behind, and a resume nearer than one brake to rest
    /// could not be braked again before the playhead overran its committed
    /// end. That includes a brake that found the stream parked on a stop it
    /// has not reached yet: resumed at the default lead, it braked again
    /// within a few planning steps, a stop per move. A machine already at
    /// rest (a jog the prime timed out on) resumes at the default lead.
    fn pacer_brake(&mut self, cause: BrakeCause) {
        let runway_s = self.timeline.frontier.runway_secs();
        let cap_s = self.intake.cap_secs();
        let peak_ahead_s = self.intake.take_peak_ahead();
        let planned_from_s = self.pipeline.planned_through_s();
        self.drain_and_fence_as(DrainCause::PacerBrake);
        tracing::info!(
            subsystem = "motion",
            event = "pipe_drain",
            ?cause,
            runway_s,
            reserve_s = self.pacer.reserve_secs(),
            cap_s,
            peak_ahead_s,
            drained_s = self.pipeline.planned_through_s() - planned_from_s,
            t_us = motion_pipeline::timing::mono_us(),
            "[pipe] pacer braked the stream to rest"
        );
        if runway_s <= 0.0 || self.links.shutting_down.load(Ordering::Acquire) {
            return;
        }
        let reserve_s = self.pacer.reserve_secs();
        if let Err(error) = self
            .timeline
            .anchor
            .lock_ok()
            .require_resume_lead(reserve_s)
        {
            fatal(&format!(
                "pacer brake did not leave the stream at rest: {error}"
            ));
        }
    }

    pub(super) fn run(mut self, rx: Receiver<StreamMsg>) {
        loop {
            if self.links.shutting_down.load(Ordering::Acquire) {
                return;
            }
            let received = if self.pacer.undrained() {
                match rx.try_recv() {
                    Ok(msg) => Some(msg),
                    Err(crossbeam_channel::TryRecvError::Empty) => {
                        let runway_before_s = self.watch_runway();
                        let planned_from_s = self.pipeline.planned_through_s();
                        let planned = self.pipeline.idle();
                        self.note_planned(planned_from_s, Handoff::Step);
                        self.note_planning_step(runway_before_s);
                        if !planned {
                            if self.links.shutting_down.load(Ordering::Acquire) {
                                return;
                            }
                            fatal("execution owner closed during planning idle");
                        }
                        match self.drain_or_runway(InboxFoundEmpty(())) {
                            None => continue,
                            Some(wait) => match self.recv_reopening_intake(&rx, Some(wait)) {
                                Ok(msg) => Some(msg),
                                Err(crossbeam_channel::RecvTimeoutError::Timeout) => continue,
                                Err(crossbeam_channel::RecvTimeoutError::Disconnected) => None,
                            },
                        }
                    }
                    Err(crossbeam_channel::TryRecvError::Disconnected) => None,
                }
            } else {
                self.recv_reopening_intake(&rx, None).ok()
            };
            let Some(msg) = received else {
                self.drain_and_fence();
                return;
            };
            if let StreamMsg::Move(m) = &msg {
                self.intake.leave(intake_gate::shortest_ns(m));
            }
            if self.intake.admits() {
                self.links.wakeup.notify_space_freed();
            }
            match msg {
                StreamMsg::Move(m) => {
                    let runway_before_s = self.watch_runway();
                    self.handle_move(m);
                    self.note_planning_step(runway_before_s);
                    self.brake_if_busy_runway_falls();
                }
                other => {
                    if self.handle_control(other) {
                        return;
                    }
                }
            }
        }
    }

    /// Waits up to `limit` (without one on `None`) for the next message,
    /// waking a submitter the intake refused the moment the playhead has
    /// made room for its move.
    fn recv_reopening_intake(
        &self,
        rx: &Receiver<StreamMsg>,
        limit: Option<Duration>,
    ) -> Result<StreamMsg, crossbeam_channel::RecvTimeoutError> {
        let deadline = limit.map(|limit| Instant::now() + limit);
        loop {
            let reopens_in = self.intake.reopens_in();
            if reopens_in.is_none() {
                self.links.wakeup.notify_space_freed();
            }
            let until_deadline =
                deadline.map(|deadline| deadline.saturating_duration_since(Instant::now()));
            let wait = match (until_deadline, reopens_in) {
                (Some(until_deadline), Some(reopens_in)) => Some(until_deadline.min(reopens_in)),
                (until_deadline, reopens_in) => until_deadline.or(reopens_in),
            };
            let Some(wait) = wait else {
                return rx
                    .recv()
                    .map_err(|_| crossbeam_channel::RecvTimeoutError::Disconnected);
            };
            match rx.recv_timeout(wait) {
                Err(crossbeam_channel::RecvTimeoutError::Timeout)
                    if deadline.is_none_or(|deadline| Instant::now() < deadline) => {}
                received => return received,
            }
        }
    }

    /// Starts a measured interval: marks the frontier and returns the runway
    /// now.
    fn watch_runway(&self) -> f64 {
        self.timeline.frontier.mark_lowest_lead();
        self.timeline.frontier.runway_secs()
    }

    fn note_planning_step(&mut self, runway_before_s: f64) {
        let Some(lowest_s) = self.timeline.frontier.lowest_lead_secs() else {
            return;
        };
        if self.pacer.note_step(runway_before_s, lowest_s) {
            self.report_reserve_raised(ReserveCost::PlanningStep, runway_before_s - lowest_s);
        }
    }

    /// The pacer measured a costlier brake: the intake's cap follows the
    /// wider reserve. One that would need more than the reserve cap holds
    /// warns: a brake costing more than the cap underruns.
    fn report_reserve_raised(&self, cost: ReserveCost, consumed_s: f64) {
        let reserve_s = self.pacer.reserve_secs();
        self.intake.set_reserve(reserve_s);
        tracing::info!(
            subsystem = "motion",
            event = "pacer_reserve_raised",
            ?cost,
            consumed_s,
            reserve_s,
            "[pacer] costliest brake to rest yet: widening the runway the pacer \
             keeps for the next one"
        );
        if self.pacer.reserve_capped() {
            tracing::warn!(
                subsystem = "motion",
                event = "pacer_reserve_capped",
                ?cost,
                consumed_s,
                reserve_s,
                "[pacer] this host's brakes to rest need more runway than the \
                 reserve cap holds: a brake costing more than the cap will underrun"
            );
        }
    }

    fn send(&mut self, item: StreamInput) {
        if self.links.shutting_down.load(Ordering::Acquire) {
            return;
        }
        let planned_from_s = self.pipeline.planned_through_s();
        let is_move = matches!(item, StreamInput::Move(_));
        let handoff = if matches!(item, StreamInput::Drain) {
            Handoff::Drain
        } else {
            Handoff::Step
        };
        if !self.pipeline.feed(item) && !self.links.shutting_down.load(Ordering::Acquire) {
            fatal("execution owner closed during planning");
        }
        if is_move {
            self.moves_sent += 1;
        }
        self.note_planned(planned_from_s, handoff);
    }

    /// Planning ran from `planned_from_s`: the intake counts what it handed
    /// on, and a throttled producer waits out its share of it.
    fn note_planned(&mut self, planned_from_s: f64, handoff: Handoff) {
        let planned_to_s = self.pipeline.planned_through_s();
        self.intake.set_planned_through(planned_to_s);
        if handoff == Handoff::Step {
            self.intake.note_ahead();
        }
        if let Some(throttle) = &mut self.throttle {
            std::thread::sleep(throttle.hold(planned_from_s, planned_to_s));
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
    /// drain measures the runway a brake to rest consumes.
    fn drain_and_fence(&mut self) -> BarrierAck {
        self.drain_and_fence_as(DrainCause::Requested)
    }

    fn drain_and_fence_as(&mut self, cause: DrainCause) -> BarrierAck {
        let runway_before_s = self.watch_runway();
        self.timeline.anchor.lock_ok().release_prime();
        self.send(StreamInput::Drain);
        self.pacer.note_reset();
        let ack = self.barrier();
        if let Some(record) = &self.dispatch_record {
            record.record_drain(cause, self.moves_sent);
            if let Err(error) = record.check() {
                fatal(&error);
            }
        }
        let lowest_s = self.timeline.frontier.lowest_lead_secs();
        if self.pacer.note_drain(runway_before_s, lowest_s) {
            self.report_reserve_raised(
                ReserveCost::Drain,
                runway_before_s - lowest_s.unwrap_or(runway_before_s),
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
        if !self.pacer.undrained() {
            self.prime_start();
        }
        self.send(m.into());
        self.pacer.note_move(Instant::now());
    }

    /// The first move of a stream setting off from rest: it starts playing
    /// no sooner than [`Pacer::primed_start_secs`] from now.
    fn prime_start(&self) {
        let host_now = self.timeline.router.lock_ok().host_now_secs();
        self.timeline.anchor.lock_ok().prime_start(
            host_now,
            self.pacer.primed_start_secs(),
            self.pacer.reserve_secs(),
        );
    }

    /// The pacer's decision while moves keep arriving. A saturated inbox
    /// never lets the idle path run, so a planner committing slower than
    /// the playhead consumes would overrun the committed end; see
    /// [`BusyRunwayWatch`] for when that brakes.
    fn brake_if_busy_runway_falls(&mut self) {
        let runway_s = self.timeline.frontier.runway_secs();
        if !self.pacer.busy_brake_due(runway_s) {
            return;
        }
        #[cfg(test)]
        self.links.pacer_drains.busy.fetch_add(1, Ordering::SeqCst);
        self.pacer_brake(BrakeCause::BusyRunwayFalling);
    }

    /// The pacer's decision when the inbox is silent while the pipeline
    /// holds undrained moves ([`Pacer::idle_verdict`]): report how long it
    /// may wait for more input, or send `Drain` so the fit stage and planner
    /// materialize the brake-to-rest and the drained trajectory beats the
    /// playhead to the pump.
    fn drain_or_runway(&mut self, _: InboxFoundEmpty) -> Option<Duration> {
        let runway_s = self.timeline.frontier.runway_secs();
        let at_rest = self.timeline.anchor.lock_ok().is_parked();
        if let IdleVerdict::Wait(wait) = self.pacer.idle_verdict(runway_s, at_rest, Instant::now())
        {
            return Some(wait);
        }
        #[cfg(test)]
        self.links.pacer_drains.idle.fetch_add(1, Ordering::SeqCst);
        self.pacer_brake(BrakeCause::RunwayExhausted);
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
                if !self.pacer.undrained() {
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
            StreamMsg::ClaimEarliestStart { id } => {
                let claim = self.timeline.claim_earliest_start();
                self.links.start_claims.resolve(id, claim);
                self.links.wakeup.notify_fence_resolved();
            }
            StreamMsg::HoldStartUntil { host_s, notify } => {
                if self.timeline.hold_start_until(host_s).is_err() {
                    self.drain_and_fence();
                    if let Err(e) = self.timeline.hold_start_until(host_s) {
                        fatal(&format!("a drained stream refused a start hold: {e}"));
                    }
                }
                let _ = notify.send(());
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
        self.timeline.frontier.clear();
        self.send(StreamInput::Control(Control::Reset { pos: pos.clone() }));
        self.pacer.note_reset();
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
        self.pacer.note_move(Instant::now());
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
