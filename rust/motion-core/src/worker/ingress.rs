//! The pipe's front door. One thread owns three jobs, in order of importance:
//!
//! 1. **Ingress guard** — every move entering the pipeline is checked for
//!    position contiguity against the odometer.
//! 2. **Pacing** — the single place that decides what a silent inbox means.
//!    The host feeds by pushing as much as fits, so a silent inbox means the
//!    gcode stream is genuinely dry; when the committed runway counts down to
//!    the reserve, `Drain` is sent so the stages materialize the
//!    brake-to-rest before the playhead overruns.
//! 3. **Control adaptation** — request/reply messages from the bridge
//!    (`Flush`, `Dwell`, `Reset`, homing drips, nudges) become ordered
//!    control tokens riding the stream, with barrier rendezvous for the
//!    replies.
//!
//! The stages downstream (fit stage → planner → lowerer → shaper) never consult
//! a clock; time lives here and in the dispatcher.

use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::Duration;

use crossbeam_channel::Receiver;

use motion_pipeline::{
    BarrierAck, CONTIGUITY_EPS_MM, Control, StreamConfig, StreamError, StreamInput,
    advance_odometer, dist3,
};

use super::dispatch::WorkerLinks;
use super::{CommittedFrontier, HomeDripParams, NudgeParams, StreamMsg, fatal};

/// Runway kept in reserve when the pacer waits out a silent input instead of
/// sending `Drain`. A drain fired at the reserve must still travel
/// fit → plan → lower → shape → dispatch and reach the pump before the
/// playhead overruns the committed frontier; those stages take milliseconds,
/// so this covers them with a wide margin while staying under the anchor's
/// 250 ms initial lead. Overrunning anyway is not fatal — the anchor
/// re-anchors forward with a logged stutter.
const RUNWAY_RESERVE_S: f64 = 0.1;

const LEAD: f64 = crate::anchor::DEFAULT_LEAD_SECS;

#[cfg(test)]
pub(crate) fn lead_secs() -> f64 {
    LEAD
}

pub(super) struct Ingress {
    pub(super) config: StreamConfig,
    /// Expected toolhead position after every move ingested so far; the
    /// ingress contiguity check anchors here.
    pub(super) odometer: Vec<f64>,
    /// Stream time the dispatched timeline has reached, mirrored from barrier
    /// acks; nudge profiles are planned from it.
    pub(super) t_next: f64,
    pub(super) input: crossbeam_channel::Sender<StreamInput>,
    pub(super) links: Arc<WorkerLinks>,
    pub(super) frontier: Arc<CommittedFrontier>,
    /// The pipeline holds moves ingested since the last `Drain`; the pacer
    /// only schedules a drain while this is set.
    pub(super) undrained: bool,
    /// Source line of the last move forwarded into the pipeline; a fence
    /// arriving now sequences after it.
    pub(super) last_line: u32,
    /// `Flush` completion runs a pump barrier through this before notifying,
    /// so a completed flush guarantees the pump has ingested everything
    /// dispatched and published a current drain ledger. `None` only in the
    /// pump-less test seam.
    pub(super) pump_control: Option<crossbeam_channel::Sender<crate::pump::PumpMsg>>,
}

impl Ingress {
    pub(super) fn run(mut self, rx: Receiver<StreamMsg>) {
        loop {
            let received = if self.undrained {
                match rx.try_recv() {
                    Ok(msg) => Some(msg),
                    Err(crossbeam_channel::TryRecvError::Empty) => match self.drain_or_runway() {
                        None => continue,
                        Some(wait) => match rx.recv_timeout(wait) {
                            Ok(msg) => Some(msg),
                            Err(crossbeam_channel::RecvTimeoutError::Timeout) => continue,
                            Err(crossbeam_channel::RecvTimeoutError::Disconnected) => None,
                        },
                    },
                    Err(crossbeam_channel::TryRecvError::Disconnected) => None,
                }
            } else {
                rx.recv().ok()
            };
            let Some(msg) = received else {
                self.drain_and_fence();
                return;
            };
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
        if self.input.send(item).is_err() {
            fatal("pipeline input closed — a stage died");
        }
    }

    /// Fence: everything sent before this has been dispatched (or discarded)
    /// once it returns. Advances the ingress's timeline mirror.
    fn barrier(&mut self) -> BarrierAck {
        let (tx, rx) = crossbeam_channel::bounded(1);
        self.send(StreamInput::Control(Control::Barrier(tx)));
        let ack = rx
            .recv()
            .unwrap_or_else(|_| fatal("pipeline dropped a barrier — a stage died"));
        if let Some(t) = ack.dispatched_through {
            self.t_next = t;
        }
        self.links.fences.resolve_armed(ack.dispatched_through);
        ack
    }

    /// Drain the lookahead and fence: the pipeline is empty and the full
    /// braked-to-rest trajectory is dispatched when this returns, so no
    /// intake remains uncommitted.
    fn drain_and_fence(&mut self) -> BarrierAck {
        self.send(StreamInput::Drain);
        self.undrained = false;
        self.barrier()
    }

    fn handle_move(&mut self, m: geometry::Move) {
        tracing::trace!(
            subsystem = "motion",
            event = "pipe_ingress",
            line = m.source.start_line,
            t_us = crate::timing::mono_us(),
            "[pipe] ingress"
        );
        if let Some(seg) = &m.segment.spatial {
            use geometry::path::lowering::PositionProfile;
            let got = seg.point_at(0.0);
            let expected = [self.odometer[0], self.odometer[1], self.odometer[2]];
            let gap_mm = dist3(expected, got);
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
        self.undrained = true;
    }

    /// The pacer's one decision. Called when the inbox is silent while the
    /// pipeline holds undrained moves: with runway beyond the reserve there is
    /// provably time to wait for more input, so report how long; at the
    /// reserve, send `Drain` so the fit stage and planner materialize the
    /// brake-to-rest and the drained trajectory beats the playhead to the
    /// pump.
    fn drain_or_runway(&mut self) -> Option<Duration> {
        let wait_s = self.frontier.runway_secs() - RUNWAY_RESERVE_S;
        if wait_s > 0.0 {
            return Some(Duration::from_secs_f64(wait_s));
        }
        tracing::debug!(
            subsystem = "motion",
            event = "pipe_drain",
            t_us = crate::timing::mono_us(),
            "[pipe] runway exhausted — draining pipeline to rest"
        );
        self.send(StreamInput::Drain);
        self.undrained = false;
        if self.links.fences.has_armed() {
            self.barrier();
        }
        None
    }

    /// Handle one non-move control message. Returns `true` when the loop
    /// should exit (shutdown).
    fn handle_control(&mut self, msg: StreamMsg) -> bool {
        match msg {
            StreamMsg::Move(_) => unreachable!("moves handled by the ingress path"),
            StreamMsg::Flush { notify } => {
                let ack = self.drain_and_fence();
                self.pump_barrier();
                let finish = ack.sync_instant.map(|t| {
                    t + Duration::try_from_secs_f64((self.t_next + LEAD).max(0.0))
                        .unwrap_or(Duration::ZERO)
                });
                let _ = notify.send(finish);
            }
            StreamMsg::Fence { id, force } => {
                if !self.undrained {
                    let ack = self.barrier();
                    self.links.fences.resolve(id, ack.dispatched_through);
                } else if force {
                    let ack = self.drain_and_fence();
                    self.links.fences.resolve(id, ack.dispatched_through);
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
            StreamMsg::StreamOpen { home_pos } => {
                self.reset_to(home_pos);
            }
            StreamMsg::Reset { recovered_pos } => {
                self.reset_to(recovered_pos);
            }
            StreamMsg::SetAxisChains(chains) => {
                self.drain_and_fence();
                self.send(StreamInput::Control(Control::SetAxisChains(chains)));
                self.barrier();
            }
            StreamMsg::HomeDrip(p) => {
                let result = self.run_home_drip(&p);
                let _ = p.notify.send(result);
            }
            StreamMsg::Nudge(p) => {
                let result = self.run_nudge(&p);
                let _ = p.notify.send(result);
            }
            StreamMsg::Shutdown => {
                self.drain_and_fence();
                return true;
            }
        }
        false
    }

    fn pump_barrier(&self) {
        let Some(tx) = &self.pump_control else {
            return;
        };
        let (ack_tx, ack_rx) = std::sync::mpsc::sync_channel(1);
        if tx.send(crate::pump::PumpMsg::Barrier(ack_tx)).is_err() {
            fatal("pump control channel closed — the pump thread died");
        }
        if ack_rx.recv_timeout(Duration::from_secs(5)).is_err() {
            fatal("pump did not acknowledge the flush barrier within 5s");
        }
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
        self.undrained = false;
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

        self.links.capture_errors.store(true, Ordering::Release);
        self.send(m.into());
        self.undrained = true;
        let ack = self.drain_and_fence();
        self.links.capture_errors.store(false, Ordering::Release);
        ack.result
    }

    /// Plan a nudge profile from the current stream time and send it down the
    /// (drained) pipeline as a control token; the dispatcher executes it and
    /// the closing barrier carries back any dispatch error. The `Dwell`
    /// advances the stream clock over the nudge's duration.
    fn run_nudge(&mut self, p: &NudgeParams) -> Result<(), String> {
        self.drain_and_fence();
        let pieces = crate::nudge::plan_nudge_profile(
            p.axis,
            p.delta_mm,
            p.speed,
            p.accel,
            p.motor_mask,
            self.t_next,
        )?;
        let total_dur: f64 = pieces.iter().map(|s| s.piece.u_end - s.piece.u_start).sum();
        self.send(StreamInput::Control(Control::Nudge {
            mcu_id: p.mcu_id,
            pieces,
        }));
        if total_dur > 0.0 {
            self.send(StreamInput::Control(Control::Dwell { secs: total_dur }));
        }
        self.t_next += total_dur;
        self.barrier().result.map_err(|e| format!("nudge: {e}"))
    }
}
