//! A host slower than the one running the pipeline: the simulator's stand-in
//! for a Pi, whose clock stands still while host threads compute. Set
//! [`HOST_COMPUTE_SCALE_ENV`] to `<scale>` and every staged pipeline thread
//! waits out `scale` times the CPU time it spent before each item it hands
//! on, so the item leaves when a host that much slower would have produced
//! it. `SERVAL_PRODUCER_THROTTLE` instead holds planning to a realtime factor
//! after its output has left: it starves a stream deterministically, but
//! cannot delay the motion behind a stream's first segment.

use std::time::Duration;

pub(crate) const HOST_COMPUTE_SCALE_ENV: &str = "SERVAL_HOST_COMPUTE_SCALE";
const SCHEDSTAT: &str = "/proc/thread-self/schedstat";

pub(crate) fn scale_from_env() -> Option<f64> {
    let spec = std::env::var(HOST_COMPUTE_SCALE_ENV).ok()?;
    let scale =
        parse_scale(&spec).unwrap_or_else(|error| panic!("{HOST_COMPUTE_SCALE_ENV}: {error}"));
    tracing::warn!(
        subsystem = "motion",
        event = "host_compute_charged",
        scale,
        "[pipeline] every pipeline thread waits out scaled CPU time on purpose \
         ({HOST_COMPUTE_SCALE_ENV})"
    );
    Some(scale)
}

fn parse_scale(spec: &str) -> Result<f64, String> {
    let scale: f64 = spec
        .trim()
        .parse()
        .map_err(|e| format!("scale {spec:?}: {e}"))?;
    if !(scale.is_finite() && scale > 0.0) {
        return Err(format!("scale must be finite and positive, got {scale}"));
    }
    Ok(scale)
}

/// The CPU time the calling thread has run for, from the scheduler's account.
fn thread_cpu_time() -> Duration {
    let stat = std::fs::read_to_string(SCHEDSTAT)
        .unwrap_or_else(|error| panic!("{HOST_COMPUTE_SCALE_ENV} reads {SCHEDSTAT}: {error}"));
    let nanos = stat
        .split_whitespace()
        .next()
        .and_then(|field| field.parse().ok())
        .unwrap_or_else(|| panic!("{SCHEDSTAT} holds no CPU time: {stat:?}"));
    Duration::from_nanos(nanos)
}

/// One thread's charge: the CPU time it had spent when it last paid. Opened
/// on the thread it charges, as the CPU clock is that thread's own.
pub(super) struct ChargeAccount {
    scale: f64,
    paid_through: Option<Duration>,
}

impl ChargeAccount {
    pub(super) fn closed(scale: f64) -> Self {
        Self {
            scale,
            paid_through: None,
        }
    }

    pub(super) fn opened(scale: f64) -> Self {
        let mut account = Self::closed(scale);
        account.open();
        account
    }

    pub(super) fn open(&mut self) {
        self.open_at(thread_cpu_time());
    }

    fn open_at(&mut self, spent: Duration) {
        self.paid_through.get_or_insert(spent);
    }

    /// The wait owed for the CPU time spent through `spent`, now paid.
    fn owe(&mut self, spent: Duration) -> Duration {
        let paid_through = self
            .paid_through
            .replace(spent)
            .expect("a compute charge is paid on the thread that opened it");
        spent.saturating_sub(paid_through).mul_f64(self.scale)
    }

    pub(super) fn pay(&mut self) {
        std::thread::sleep(self.owe(thread_cpu_time()));
        self.paid_through = Some(thread_cpu_time());
    }
}

pub(super) fn pay(account: Option<&mut ChargeAccount>) {
    if let Some(account) = account {
        account.pay();
    }
}

#[cfg(test)]
mod tests;
