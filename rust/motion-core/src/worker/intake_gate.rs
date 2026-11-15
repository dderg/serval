//! Bounds the motion queued ahead of the playhead by time. PAUSE and
//! CANCEL_PRINT stop feeding moves and then wait for everything already
//! queued to play out, so whatever the intake admits is their latency.
//! Message counts bound nothing in time: on the Pi bench the pump's span
//! backlog alone held 14-28 s of motion.
//!
//! The intake admits a move while the committed runway plus the moves still
//! in the inbox fall short of the pacer's reserve plus
//! [`STALL_HEADROOM_S`]. Moves the pipeline holds uncommitted are not
//! counted: it needs them to commit anything, and the planner commits by
//! stream time ([`motion_pipeline`]'s `REPLAN_BATCH_S`), so they stay short.

use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Duration;

use super::CommittedFrontier;

/// Runway kept beyond the pacer's reserve: how long the producer may stall
/// before the pacer brakes. The pump pushes this far ahead of the MCU
/// playhead because it is the host stall it must ride out
/// ([`crate::pump::MAX_LEAD_SECS`]); the planning side gets the same.
pub(crate) const STALL_HEADROOM_S: f64 = crate::pump::MAX_LEAD_SECS;

const NS_PER_S: f64 = 1e9;

pub(crate) struct IntakeGate {
    frontier: Arc<CommittedFrontier>,
    inbox_ns: AtomicU64,
    reserve_bits: AtomicU64,
}

/// The least time `m` can take, the unit the inbox is counted in.
pub(crate) fn shortest_ns(m: &geometry::Move) -> u64 {
    (m.shortest_secs() * NS_PER_S) as u64
}

impl IntakeGate {
    pub(crate) fn new(frontier: Arc<CommittedFrontier>, reserve_s: f64) -> Self {
        Self {
            frontier,
            inbox_ns: AtomicU64::new(0),
            reserve_bits: AtomicU64::new(reserve_s.to_bits()),
        }
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
    /// wait for.
    pub(crate) fn reopens_in(&self) -> Option<Duration> {
        if self.frontier.unexecuted {
            return None;
        }
        let inbox_s = self.inbox_ns.load(Ordering::Acquire) as f64 / NS_PER_S;
        let excess_s = self.frontier.runway_secs() + inbox_s - self.cap_secs();
        (excess_s >= 0.0).then(|| Duration::from_secs_f64(excess_s))
    }
}

#[cfg(test)]
mod tests;
