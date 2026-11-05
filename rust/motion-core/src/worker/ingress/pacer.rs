//! The pacer's decisions, apart from the clocks and channels they act on:
//! when a stream whose producer may fall behind its playhead must brake to
//! rest, and how much runway it keeps for doing so.

use std::time::{Duration, Instant};

use crate::anchor::LOW_MARGIN_WARN_SECS;

const DRAIN_RESERVE_FLOOR_S: f64 = 0.5;
pub(super) const DRAIN_RESERVE_SAFETY: f64 = 2.0;

// TODO: expose as a config knob if 250 ms turns out wrong for slower feeds.
const STARTUP_PRIME_S: f64 = 0.250;

/// Watches the committed runway while moves keep arriving. Under the
/// reserve the runway is watched instead of drained on: a healthy stream
/// holds or regains it as commits land, whereas one whose planning falls
/// behind the playhead loses it. Losing half the reserve from the best
/// runway seen under it leaves the other half — one brake to rest — to
/// brake in. A stream that held the reserve when last observed had the
/// whole reserve under it, however far below the next observation finds
/// it. A stream that never built runway has nothing in flight to brake.
#[derive(Debug, Default)]
pub(in crate::worker) struct BusyRunwayWatch {
    peak_s: Option<f64>,
    held_reserve: bool,
}

impl BusyRunwayWatch {
    /// Whether the stream must brake to rest now.
    pub(in crate::worker) fn observe(&mut self, runway_s: f64, reserve_s: f64) -> bool {
        if runway_s >= reserve_s {
            self.peak_s = None;
            self.held_reserve = true;
            return false;
        }
        let best_s = if self.held_reserve {
            reserve_s
        } else {
            runway_s
        };
        self.held_reserve = false;
        let peak_s = self.peak_s.map_or(best_s, |peak_s| peak_s.max(best_s));
        self.peak_s = Some(peak_s);
        peak_s - runway_s >= 0.5 * reserve_s
    }

    pub(in crate::worker) fn reset(&mut self) {
        self.peak_s = None;
        self.held_reserve = false;
    }
}

/// What the idle path does with an inbox it found empty.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(in crate::worker) enum IdleVerdict {
    /// There is provably time to wait this long for more input.
    Wait(Duration),
    /// Brake to rest now.
    Brake,
}

#[derive(Debug, Default)]
pub(in crate::worker) struct Pacer {
    undrained_since: Option<Instant>,
    worst_drain_s: f64,
    worst_blind_drop_s: f64,
    busy: BusyRunwayWatch,
}

impl Pacer {
    /// The runway the pacer keeps for braking to rest: a multiple of one
    /// brake to rest.
    pub(in crate::worker) fn reserve_secs(&self) -> f64 {
        (self.one_brake_secs() * DRAIN_RESERVE_SAFETY).max(DRAIN_RESERVE_FLOOR_S)
    }

    /// The runway one brake to rest consumes, measured from the last
    /// moment the pacer could decide on it: the most runway it lost while
    /// planning between two decisions, the slowest drain, and the transport
    /// floor the brake's trajectory must still arrive ahead of the playhead
    /// by.
    fn one_brake_secs(&self) -> f64 {
        self.worst_blind_drop_s + self.worst_drain_s + LOW_MARGIN_WARN_SECS
    }

    /// The runway fell from `before_s` to `after_s` while the ingress
    /// planned, unable to pace. Runway the dispatcher kept extending while
    /// planning waited on it costs nothing.
    pub(in crate::worker) fn note_step(&mut self, before_s: f64, after_s: f64) {
        self.worst_blind_drop_s = self.worst_blind_drop_s.max(before_s - after_s);
    }

    pub(in crate::worker) fn undrained(&self) -> bool {
        self.undrained_since.is_some()
    }

    pub(in crate::worker) fn note_move(&mut self, now: Instant) {
        self.undrained_since.get_or_insert(now);
    }

    pub(in crate::worker) fn note_reset(&mut self) {
        self.undrained_since = None;
    }

    /// A brake to rest finished after `latency_s`; true when it is the
    /// slowest yet and widened the reserve.
    pub(in crate::worker) fn note_drain(&mut self, latency_s: f64) -> bool {
        self.undrained_since = None;
        self.busy.reset();
        if latency_s <= self.worst_drain_s {
            return false;
        }
        self.worst_drain_s = latency_s;
        true
    }

    /// Whether a stream whose moves keep arriving must brake now.
    pub(in crate::worker) fn busy_brake_due(&mut self, runway_s: f64) -> bool {
        let reserve_s = self.reserve_secs();
        self.busy.observe(runway_s, reserve_s)
    }

    /// The idle path's decision. With runway beyond the reserve there is
    /// provably time to wait for more input. Below it a stream just starting
    /// may wait out its prime: freely while everything committed still ends
    /// at rest, and once motion is in flight never past the runway one
    /// slowest brake to rest needs.
    pub(in crate::worker) fn idle_verdict(
        &self,
        runway_s: f64,
        at_rest: bool,
        now: Instant,
    ) -> IdleVerdict {
        let wait_s = runway_s - self.reserve_secs();
        if wait_s > 0.0 {
            return IdleVerdict::Wait(Duration::from_secs_f64(wait_s));
        }
        let Some(since) = self.undrained_since else {
            return IdleVerdict::Brake;
        };
        let prime_left = Duration::from_secs_f64(STARTUP_PRIME_S)
            .saturating_sub(now.saturating_duration_since(since));
        if prime_left.is_zero() {
            return IdleVerdict::Brake;
        }
        if at_rest {
            return IdleVerdict::Wait(prime_left);
        }
        let spare_runway_s = runway_s - self.one_brake_secs();
        if spare_runway_s <= 0.0 {
            return IdleVerdict::Brake;
        }
        IdleVerdict::Wait(prime_left.min(Duration::from_secs_f64(spare_runway_s)))
    }
}

#[cfg(test)]
mod tests;
