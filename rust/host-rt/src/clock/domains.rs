//! Time-domain newtypes for the host↔MCU seam.
//!
//! Every value that crosses a clock-domain boundary does so through
//! [`crate::passthrough_queue::PassthroughRouter`], which converts between
//! these types from a single clock read per query. Holding a raw `f64` is the
//! bug this module exists to prevent: two of these domains added together
//! compile fine and produce a plausible-looking timestamp that is wrong by an
//! epoch.

use std::time::Instant;

use super::instant_to_f64;

/// Seconds on the host monotonic clock, in the process-lifetime anchor frame
/// of [`instant_to_f64`]. Comparable and subtractable with any other
/// `HostSecs` from the same process; not wall time, not print time.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct HostSecs(f64);

impl HostSecs {
    #[must_use]
    pub fn from_instant(instant: Instant) -> Self {
        Self(instant_to_f64(instant))
    }

    /// Wrap a value already known to be in the `instant_to_f64` frame
    /// (e.g. an anchored stream projection `t0 + t`).
    #[must_use]
    pub fn from_anchor_frame(secs: f64) -> Self {
        Self(secs)
    }

    #[must_use]
    pub fn get(self) -> f64 {
        self.0
    }
}

/// Seconds on the printer's shared scheduling timeline: the reference
/// (primary) MCU's clock divided by its nominal frequency — klippy's
/// `print_time`. Secondary MCUs reach this timeline through their own
/// calibrated conversions; only the reference MCU's record defines it.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct PrintTime(f64);

impl PrintTime {
    #[must_use]
    pub fn new(secs: f64) -> Self {
        Self(secs)
    }

    #[must_use]
    pub fn get(self) -> f64 {
        self.0
    }

    #[must_use]
    pub fn max(self, other: Self) -> Self {
        Self(self.0.max(other.0))
    }
}
