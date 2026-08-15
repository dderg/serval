use std::time::{Duration, Instant};

use crate::types::AxisKey;

pub(super) struct ConsumptionStallWatch {
    last_log: Option<Instant>,
    fatal_after: Duration,
    started: Option<(AxisKey, u32, Instant)>,
}

pub(super) struct StallObservation {
    pub(super) log_due: bool,
    pub(super) stalled_secs: Option<f64>,
}

impl ConsumptionStallWatch {
    pub(super) fn new(fatal_after: Duration) -> Self {
        Self {
            last_log: None,
            fatal_after,
            started: None,
        }
    }

    pub(super) fn observe(
        &mut self,
        key: AxisKey,
        consumed: u32,
        now: Instant,
    ) -> StallObservation {
        let log_due = self
            .last_log
            .is_none_or(|last| now.duration_since(last) >= Duration::from_secs(1));
        if log_due {
            self.last_log = Some(now);
        }
        let stalled_secs = match self.started {
            Some((prior_key, prior_consumed, started))
                if prior_key == key && prior_consumed == consumed =>
            {
                let elapsed = now.duration_since(started);
                (elapsed >= self.fatal_after).then_some(elapsed.as_secs_f64())
            }
            _ => {
                self.started = Some((key, consumed, now));
                None
            }
        };
        StallObservation {
            log_due,
            stalled_secs,
        }
    }

    pub(super) fn reset(&mut self) {
        self.started = None;
    }

    #[cfg(test)]
    pub(super) fn started(&self) -> Option<(AxisKey, u32, Instant)> {
        self.started
    }
}
