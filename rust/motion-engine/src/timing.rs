use std::sync::OnceLock;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Instant;

fn epoch() -> Instant {
    static EPOCH: OnceLock<Instant> = OnceLock::new();
    *EPOCH.get_or_init(Instant::now)
}

/// Microseconds since the first call anywhere in this process. Shared across the
/// planner and pump threads so per-stage `pipe_*` timestamps are directly
/// comparable: subtract two `t_us` values to get the inter-stage latency.
#[must_use]
pub fn mono_us() -> u64 {
    u64::try_from(epoch().elapsed().as_micros()).unwrap_or(u64::MAX)
}

/// Monotonic id stamped on each commit batch so a gcode line can be traced from
/// fit through plan, lower, dispatch, and the pump hand-off as one unit.
#[must_use]
pub fn next_batch_seq() -> u64 {
    static SEQ: AtomicU64 = AtomicU64::new(0);
    SEQ.fetch_add(1, Ordering::Relaxed)
}
