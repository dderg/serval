use super::{AxisKey, AxisQueue};
use std::collections::{BTreeMap, BTreeSet};
use std::time::{Duration, Instant};

pub const DRIP_WINDOW_SECS: f64 = crate::anchor::DEFAULT_LEAD_SECS;

pub struct DripArm {
    pub cohort: u64,
    pub participants: Vec<AxisKey>,
    pub timeout: Duration,
}

pub(super) struct DripCohort {
    pub id: u64,
    pub participants: BTreeSet<AxisKey>,
    pub timeout: Duration,
    pub baseline: BTreeMap<AxisKey, u32>,
    pub last_retired: BTreeMap<AxisKey, u32>,
    pub step_deadline: Instant,
    pub deadline_floor: u32,
}

impl DripCohort {
    pub(super) fn executed(&self, k: &AxisKey, queues: &BTreeMap<AxisKey, AxisQueue>) -> u32 {
        let retired = queues.get(k).map_or(0, |q| q.retired);
        let baseline = self.baseline.get(k).copied().unwrap_or(0);
        retired.wrapping_sub(baseline)
    }

    /// Progress floor across the lanes that actually have work: a parked
    /// ethercat lane receives no pieces during another axis's homing drip
    /// (pure-hold lanes are skipped at enqueue), so a participant with
    /// nothing queued and nothing in flight cannot execute anything and must
    /// not pin the floor at zero — that would starve the stall deadline and
    /// abort every homing run longer than the timeout.
    pub(super) fn floor(&self, queues: &BTreeMap<AxisKey, AxisQueue>) -> u32 {
        self.participants
            .iter()
            .filter(|k| {
                queues
                    .get(k)
                    .is_some_and(|q| !q.pieces.is_empty() || q.pushed != q.retired)
            })
            .map(|k| self.executed(k, queues))
            .min()
            .unwrap_or(0)
    }
}
