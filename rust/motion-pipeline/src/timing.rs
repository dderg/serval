use std::sync::OnceLock;
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

#[must_use]
pub fn monotonic_ns() -> u64 {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    #[allow(unsafe_code)]
    unsafe {
        libc::clock_gettime(libc::CLOCK_MONOTONIC_RAW, &mut ts);
    }
    (ts.tv_sec as u64) * 1_000_000_000 + (ts.tv_nsec as u64)
}

#[cfg(test)]
mod tests;
