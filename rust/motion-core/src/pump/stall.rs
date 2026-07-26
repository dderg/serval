use std::time::{Duration, Instant};

use crate::types::AxisKey;

pub(super) struct RetirementStallWatch {
    last_log: Option<Instant>,
    fatal_after: Duration,
    started: Option<(AxisKey, u32, Instant)>,
}

pub(super) struct StallObservation {
    pub(super) log_due: bool,
    pub(super) stalled_secs: Option<f64>,
}

impl RetirementStallWatch {
    pub(super) fn new(fatal_after: Duration) -> Self {
        Self {
            last_log: None,
            fatal_after,
            started: None,
        }
    }

    pub(super) fn observe(&mut self, key: AxisKey, retired: u32, now: Instant) -> StallObservation {
        let log_due = self
            .last_log
            .is_none_or(|last| now.duration_since(last) >= Duration::from_secs(1));
        if log_due {
            self.last_log = Some(now);
        }
        let stalled_secs = match self.started {
            Some((prior_key, prior_retired, started))
                if prior_key == key && prior_retired == retired =>
            {
                let elapsed = now.duration_since(started);
                (elapsed >= self.fatal_after).then_some(elapsed.as_secs_f64())
            }
            _ => {
                self.started = Some((key, retired, now));
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

pub(super) struct AheadStallWatch {
    threshold: Duration,
    started: Option<(AxisKey, Instant)>,
    last_key: Option<AxisKey>,
    reported: bool,
}

pub(super) struct AheadStallEnd {
    pub(super) first_key: AxisKey,
    pub(super) last_key: AxisKey,
    pub(super) elapsed: Duration,
}

impl AheadStallWatch {
    pub(super) fn new(threshold: Duration) -> Self {
        Self {
            threshold,
            started: None,
            last_key: None,
            reported: false,
        }
    }

    pub(super) fn observe(&mut self, key: AxisKey, now: Instant) -> Option<Duration> {
        self.last_key = Some(key);
        let started = match self.started {
            Some((_, started)) => started,
            None => {
                self.started = Some((key, now));
                self.reported = false;
                return None;
            }
        };
        let elapsed = now.duration_since(started);
        if self.reported || elapsed < self.threshold {
            return None;
        }
        self.reported = true;
        Some(elapsed)
    }

    pub(super) fn reset(&mut self, now: Instant) -> Option<AheadStallEnd> {
        let started = self.started.take();
        let last_key = self.last_key.take();
        let reported = std::mem::take(&mut self.reported);
        match (started, last_key, reported) {
            (Some((first_key, started)), Some(last_key), true) => Some(AheadStallEnd {
                first_key,
                last_key,
                elapsed: now.duration_since(started),
            }),
            _ => None,
        }
    }
}
