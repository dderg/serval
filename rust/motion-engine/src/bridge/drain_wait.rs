use std::time::Instant;

/// A drain cannot finish before the motion the pipeline has already committed
/// has physically executed on the MCU: `outstanding_secs` (the committed
/// frontier minus now) is that horizon, and waiting it out is the drain's SLA,
/// not a lag — a print that ends with twenty seconds of buffered motion takes
/// twenty seconds to drain, by construction.
///
/// Past the horizon the books close on the pipeline's own schedule: the last
/// barrier is queued behind the steps it accounts for, so its ack trails the
/// MCU by one round trip. One full scheduling lead — the margin the host
/// already works ahead by — is the slack that covers that round trip plus the
/// pump loop's publish; anything still unretired beyond it is the drain
/// genuinely lagging.
const OVERDUE_GRACE_SECS: f64 = crate::anchor::DEFAULT_LEAD_SECS;
const REPORT_PERIOD_SECS: f64 = 5.0;

pub(crate) struct DrainOverdue {
    pub(crate) waited_s: f64,
    pub(crate) overdue_s: f64,
    pub(crate) horizon_s: f64,
}

pub(crate) struct DrainWaitDiag {
    started: Instant,
    horizon_at_start_s: f64,
    horizon_spent: Option<Instant>,
    last_report: Option<Instant>,
}

impl DrainWaitDiag {
    pub(crate) fn new(now: Instant, outstanding_secs: f64) -> Self {
        Self {
            started: now,
            horizon_at_start_s: outstanding_secs,
            horizon_spent: None,
            last_report: None,
        }
    }

    /// What the wait cost and what it was owed — the pair that says whether a
    /// long drain was the machine still moving or the pipeline dragging.
    pub(crate) fn elapsed(&self, now: Instant) -> (f64, f64) {
        (
            now.duration_since(self.started).as_secs_f64(),
            self.horizon_at_start_s,
        )
    }

    /// `outstanding_secs` is the committed motion still ahead of `now`. Fresh
    /// motion committed mid-wait rewinds the horizon: the drain owes the wait
    /// again.
    pub(crate) fn poll(&mut self, now: Instant, outstanding_secs: f64) -> Option<DrainOverdue> {
        if outstanding_secs > 0.0 {
            self.horizon_spent = None;
            return None;
        }
        let spent = *self.horizon_spent.get_or_insert(now);
        let overdue_s = now.duration_since(spent).as_secs_f64();
        if overdue_s < OVERDUE_GRACE_SECS {
            return None;
        }
        if self
            .last_report
            .is_some_and(|t| now.duration_since(t).as_secs_f64() < REPORT_PERIOD_SECS)
        {
            return None;
        }
        self.last_report = Some(now);
        Some(DrainOverdue {
            waited_s: now.duration_since(self.started).as_secs_f64(),
            overdue_s,
            horizon_s: self.horizon_at_start_s,
        })
    }
}

#[cfg(test)]
#[path = "drain_wait_tests.rs"]
mod drain_wait_tests;
