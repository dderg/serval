use super::{AxisKey, AxisQueue};
use std::collections::{BTreeMap, BTreeSet};
use std::time::{Duration, Instant};

pub const DRIP_WINDOW_SECS: f64 = super::stepcompress_sink::SEND_LEAD_SECONDS + 1.0;
pub const DRIP_ANCHOR_LEAD_SECS: f64 = DRIP_WINDOW_SECS + 1.0;

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
    pub progress: u64,
}

impl DripCohort {
    pub(super) fn executed(&self, k: &AxisKey, queues: &BTreeMap<AxisKey, AxisQueue>) -> u32 {
        let retired = queues.get(k).map_or(0, |q| q.retired);
        let baseline = self.baseline.get(k).copied().unwrap_or(0);
        retired.wrapping_sub(baseline)
    }

    pub(super) fn progress(&self, queues: &BTreeMap<AxisKey, AxisQueue>) -> u64 {
        self.participants
            .iter()
            .map(|k| u64::from(self.executed(k, queues)))
            .sum()
    }
}
