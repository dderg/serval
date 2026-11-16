//! Bounds the motion queued ahead of the playhead by time. PAUSE and
//! CANCEL_PRINT stop feeding moves and then wait for everything already
//! queued to play out, so whatever the intake admits is their latency.
//! Message counts bound nothing in time: on the Pi bench the pump's span
//! backlog alone held 14-28 s of motion.
//!
//! The intake admits a move while the committed runway and the moves still
//! in the inbox fall short of the pacer's reserve plus
//! [`STALL_HEADROOM_S`]. Motion the pipeline holds is not counted, planned
//! or not: its stages hold it until later moves arrive (the shaper holds
//! its last segment until the next one covers its support, the planning
//! front holds moves by count), so refusing those moves while counting it
//! starves them. On the bench speed ladder the shaper held 3.3 s of planned
//! strokes for the next move, more than the headroom, and the runway ran
//! down to the reserve behind a shut intake: three pacer brakes mid-print.
//!
//! So once klippy stops feeding (PAUSE, CANCEL_PRINT, the end of a file),
//! motion ends within the most that stood ahead, which is the committed
//! runway, the planned motion the pump has not taken and the inbox, plus
//! what the planning front still holds, which the pacer's closing brake
//! plans to rest. That brake logs both (`pipe_drain`: `peak_ahead_s`,
//! `drained_s`).

use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Duration;

use super::CommittedFrontier;
use super::dispatch::WorkerLinks;

/// Runway kept beyond the pacer's reserve: how long the producer may stall
/// before the pacer brakes. The pump pushes this far ahead of the MCU
/// playhead because it is the host stall it must ride out
/// ([`crate::pump::MAX_LEAD_SECS`]); the planning side gets the same.
pub(crate) const STALL_HEADROOM_S: f64 = crate::pump::MAX_LEAD_SECS;

const NS_PER_S: f64 = 1e9;

pub(crate) struct IntakeGate {
    frontier: Arc<CommittedFrontier>,
    links: Arc<WorkerLinks>,
    planned_through_bits: AtomicU64,
    inbox_ns: AtomicU64,
    reserve_bits: AtomicU64,
    peak_ahead_bits: AtomicU64,
}

/// The least time `m` can take, the unit the inbox is counted in.
pub(crate) fn shortest_ns(m: &geometry::Move) -> u64 {
    (m.shortest_secs() * NS_PER_S) as u64
}

impl IntakeGate {
    pub(crate) fn new(
        frontier: Arc<CommittedFrontier>,
        links: Arc<WorkerLinks>,
        reserve_s: f64,
    ) -> Self {
        Self {
            frontier,
            links,
            planned_through_bits: AtomicU64::new(0.0_f64.to_bits()),
            inbox_ns: AtomicU64::new(0),
            reserve_bits: AtomicU64::new(reserve_s.to_bits()),
            peak_ahead_bits: AtomicU64::new(0.0_f64.to_bits()),
        }
    }

    /// The stream time the planning side has handed on through.
    pub(crate) fn set_planned_through(&self, stream_s: f64) {
        self.planned_through_bits
            .store(stream_s.to_bits(), Ordering::Release);
    }

    pub(crate) fn set_reserve(&self, reserve_s: f64) {
        self.reserve_bits
            .store(reserve_s.to_bits(), Ordering::Release);
    }

    /// The most runway the intake lets queue ahead of the playhead.
    pub(crate) fn cap_secs(&self) -> f64 {
        f64::from_bits(self.reserve_bits.load(Ordering::Acquire)) + STALL_HEADROOM_S
    }

    pub(crate) fn enter(&self, ns: u64) {
        self.inbox_ns.fetch_add(ns, Ordering::AcqRel);
    }

    pub(crate) fn leave(&self, ns: u64) {
        let before = self.inbox_ns.fetch_sub(ns, Ordering::AcqRel);
        assert!(
            before >= ns,
            "intake gate: {ns} ns left an inbox that held only {before} ns"
        );
    }

    pub(crate) fn admits(&self) -> bool {
        self.reopens_in().is_none()
    }

    /// How long until the playhead frees room for another move; `None`
    /// while there is room. A pipeline no MCU executes has no playhead to
    /// wait for. A reopening less than the clock's nanosecond away has
    /// happened: timed, it returns at once, and in the simulator, whose
    /// clock stands still while a host thread runs, the ingress waited on
    /// it forever.
    pub(crate) fn reopens_in(&self) -> Option<Duration> {
        if self.frontier.unexecuted {
            return None;
        }
        let excess_s = self.queued_secs() - self.cap_secs();
        let reopens_in = Duration::from_secs_f64(excess_s.max(0.0));
        (!reopens_in.is_zero()).then_some(reopens_in)
    }

    /// Raises the peak [`Self::take_peak_ahead`] reports to the motion
    /// standing ahead now. Called after every admission and planning step,
    /// the only events that add to it.
    pub(crate) fn note_ahead(&self) {
        if self.frontier.unexecuted {
            return;
        }
        let ahead_s = self.ahead_secs();
        let _ = self
            .peak_ahead_bits
            .fetch_update(Ordering::AcqRel, Ordering::Acquire, |bits| {
                (ahead_s > f64::from_bits(bits)).then_some(ahead_s.to_bits())
            });
    }

    /// The most motion that stood ahead since the last call. Once klippy
    /// stops feeding, motion ends within it plus what the planning front
    /// still holds.
    pub(crate) fn take_peak_ahead(&self) -> f64 {
        f64::from_bits(
            self.peak_ahead_bits
                .swap(0.0_f64.to_bits(), Ordering::AcqRel),
        )
    }

    /// Motion handed over that the planning front does not hold: what the
    /// intake counts and the planned motion the pump has not taken yet.
    fn ahead_secs(&self) -> f64 {
        let planned_ahead_s = self.planned_ahead_secs();
        self.queued_secs() + planned_ahead_s
    }

    /// The committed runway and the moves still in the inbox: motion that
    /// plays out or reaches the pipeline without waiting on later moves.
    fn queued_secs(&self) -> f64 {
        let inbox_s = self.inbox_ns.load(Ordering::Acquire) as f64 / NS_PER_S;
        self.frontier.runway_secs() + inbox_s
    }

    /// Planned motion the pump has not taken yet: it plays once the
    /// committed runway has. Read before the runway: the dispatcher extends
    /// the frontier first, so a segment it takes in between is counted
    /// twice, never missed.
    fn planned_ahead_secs(&self) -> f64 {
        let dispatched_s = f64::from_bits(self.links.last_move_time_bits.load(Ordering::Acquire));
        let planned_s = f64::from_bits(self.planned_through_bits.load(Ordering::Acquire));
        (planned_s - dispatched_s).max(0.0)
    }
}

#[cfg(test)]
mod tests;
