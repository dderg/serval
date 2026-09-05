//! Drained-ness of the motion pipeline, observed rather than re-counted.
//!
//! The pump is the single owner of every per-axis counter that matters here
//! (`pending` staged pieces, `pushed` wire pieces, MCU-confirmed `retired`)
//! and publishes a snapshot of them after every loop iteration. Readers only
//! ever compare counters that were written together, in the same unit, by the
//! same thread — there is no parallel ledger to drift out of sync.
//!
//! Ordering: a snapshot can lag what the dispatcher has handed the pump. A
//! reader that needs "everything submitted so far is reflected" must sequence
//! behind a `PumpMsg::Barrier`, whose ack the pump sends only after
//! publishing — the pipeline `Flush` does exactly that.

use crate::lock_ext::LockExt;
use std::collections::BTreeMap;
use std::sync::{Condvar, Mutex};
use std::time::{Duration, Instant};

type AxisKey = (u32, u8);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AxisDrainState {
    pub pending: u32,
    pub pushed: u32,
    pub retired: u32,
    /// Pieces still staged in the pump must reach their endpoint before a
    /// reseed can reset endpoint state. This includes holds because a fresh
    /// hold can carry the seam mark that sanctions its new clock epoch.
    pub staged_motion: u32,
    /// Consecutive hold pieces at the pushed (wire) tail. A hold retires
    /// only when its end time passes, up to the full scheduling lead after
    /// the last real move, so pushed trailing holds need not gate a drain.
    pub hold_tail: u32,
}

impl AxisDrainState {
    fn drained(&self) -> bool {
        self.pending == 0
            && self.staged_motion == 0
            && self.pushed.wrapping_sub(self.retired) <= self.hold_tail
    }
}

pub struct DrainLedger {
    axes: Mutex<BTreeMap<AxisKey, AxisDrainState>>,
    cv: Condvar,
}

impl DrainLedger {
    pub fn new() -> Self {
        Self {
            axes: Mutex::new(BTreeMap::new()),
            cv: Condvar::new(),
        }
    }

    /// Pump-only: replace the snapshot and wake waiters.
    pub fn publish(&self, snapshot: BTreeMap<AxisKey, AxisDrainState>) {
        let mut axes = self.axes.lock_ok();
        *axes = snapshot;
        drop(axes);
        self.cv.notify_all();
    }

    pub fn drained(&self) -> bool {
        let axes = self.axes.lock_ok();
        axes.values().all(AxisDrainState::drained)
    }

    pub fn lagging_axes(&self) -> Vec<(u32, u8, AxisDrainState)> {
        let axes = self.axes.lock_ok();
        axes.iter()
            .filter(|(_, s)| !s.drained())
            .map(|(&(mcu, axis), &s)| (mcu, axis, s))
            .collect()
    }

    pub fn wait_drained(&self, timeout: Duration) -> Result<(), String> {
        let deadline = Instant::now() + timeout;
        let mut axes = self.axes.lock_ok();
        while !axes.values().all(AxisDrainState::drained) {
            let now = Instant::now();
            if now >= deadline {
                let lagging: Vec<String> = axes
                    .iter()
                    .filter(|(_, s)| !s.drained())
                    .map(|(&(mcu, axis), s)| {
                        format!(
                            "mcu{mcu} axis{axis}: pending {} pushed {} retired {}",
                            s.pending, s.pushed, s.retired
                        )
                    })
                    .collect();
                return Err(format!(
                    "motion drain timed out after {:?}; not finished: [{}]",
                    timeout,
                    lagging.join(", ")
                ));
            }
            let (guard, _) = self
                .cv
                .wait_timeout(axes, deadline - now)
                .unwrap_or_else(|p| p.into_inner());
            axes = guard;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests;
