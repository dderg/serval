use super::{AxisKey, AxisQueue};
use std::collections::BTreeMap;
use std::time::{Duration, Instant};

pub const DRIP_WINDOW_SECS: f64 = 0.100;

pub struct DripArm {
    pub cohort: u64,
    pub participants: Vec<AxisKey>,
    pub timeout: Duration,
}

pub(super) struct DripParticipant {
    pub baseline: u32,
    pub last_retired: u32,
}

pub(super) struct DripCohort {
    pub id: u64,
    pub participants: BTreeMap<AxisKey, DripParticipant>,
    pub timeout: Duration,
    pub step_deadline: Instant,
    pub execution_floor: u32,
}

impl DripCohort {
    pub(super) fn executed(&self, k: &AxisKey, queues: &BTreeMap<AxisKey, AxisQueue>) -> u32 {
        let retired = queues.get(k).map_or(0, |q| q.credit.snapshot().retired);
        let baseline = self.participants[k].baseline;
        retired.wrapping_sub(baseline)
    }

    pub(super) fn active_execution_floor(&self, queues: &BTreeMap<AxisKey, AxisQueue>) -> u32 {
        self.participants
            .keys()
            .filter(|k| {
                queues
                    .get(k)
                    .is_some_and(|q| !q.spans.is_empty() || q.credit.outstanding() != 0)
            })
            .map(|k| self.executed(k, queues))
            .min()
            .unwrap_or(0)
    }
}
