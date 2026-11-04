use std::time::{Duration, Instant};

use geometry::{StepCapacity, StepCapacityOverload};

pub const STEP_CAPACITY_LOG_INTERVAL: Duration = Duration::from_secs(30);

/// What one mcu's step capacity cut since the last report.
#[derive(Debug, Clone, PartialEq)]
pub struct StepCapacityReport {
    pub mcu: String,
    pub moves_capped: u64,
    pub worst_demand_over_capacity: f64,
}

#[derive(Debug, Clone, Copy, Default)]
struct Pending {
    moves_capped: u64,
    worst_demand_over_capacity: f64,
}

/// Counts the moves the step capacity cut, per mcu, and reports them at
/// once the first time and then at most once per interval.
#[derive(Debug, Default)]
pub struct StepCapacityLog {
    pending: Vec<Pending>,
    last_report: Option<Instant>,
}

impl StepCapacityLog {
    pub fn record(
        &mut self,
        capacity: &StepCapacity,
        overload: StepCapacityOverload,
        now: Instant,
    ) -> Vec<StepCapacityReport> {
        self.pending.resize(capacity.mcus.len(), Pending::default());
        let pending = &mut self.pending[overload.mcu_index];
        pending.moves_capped += 1;
        pending.worst_demand_over_capacity = pending
            .worst_demand_over_capacity
            .max(overload.demand_over_capacity);
        let due = self
            .last_report
            .is_none_or(|last| now.duration_since(last) >= STEP_CAPACITY_LOG_INTERVAL);
        if !due {
            return Vec::new();
        }
        self.last_report = Some(now);
        self.pending
            .iter_mut()
            .zip(&capacity.mcus)
            .filter(|(pending, _)| pending.moves_capped > 0)
            .map(|(pending, mcu)| {
                let report = StepCapacityReport {
                    mcu: mcu.mcu.clone(),
                    moves_capped: pending.moves_capped,
                    worst_demand_over_capacity: pending.worst_demand_over_capacity,
                };
                *pending = Pending::default();
                report
            })
            .collect()
    }
}

#[cfg(test)]
#[path = "step_capacity_log_tests.rs"]
mod step_capacity_log_tests;
