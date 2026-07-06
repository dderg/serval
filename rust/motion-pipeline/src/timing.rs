#[cfg(not(target_arch = "wasm32"))]
fn epoch() -> std::time::Instant {
    static EPOCH: std::sync::OnceLock<std::time::Instant> = std::sync::OnceLock::new();
    *EPOCH.get_or_init(std::time::Instant::now)
}

/// Microseconds since the first call anywhere in this process. Shared across the
/// planner and pump threads so per-stage `pipe_*` timestamps are directly
/// comparable: subtract two `t_us` values to get the inter-stage latency.
#[cfg(not(target_arch = "wasm32"))]
#[must_use]
pub fn mono_us() -> u64 {
    u64::try_from(epoch().elapsed().as_micros()).unwrap_or(u64::MAX)
}

#[cfg(not(target_arch = "wasm32"))]
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

/// wasm32 has no monotonic clock in `std`; the playground runs the pipeline
/// synchronously in one call, so inter-stage latency timestamps carry no
/// information there.
#[cfg(target_arch = "wasm32")]
#[must_use]
pub fn mono_us() -> u64 {
    0
}

#[cfg(target_arch = "wasm32")]
#[must_use]
pub fn monotonic_ns() -> u64 {
    0
}

/// Per-stage latency clock for the `pipe_*` tracing fields. A wrapper instead
/// of a bare `Instant` so the stages stay buildable on wasm32, where
/// `Instant::now()` aborts at runtime.
#[derive(Debug, Clone, Copy)]
pub struct Stopwatch {
    #[cfg(not(target_arch = "wasm32"))]
    start: std::time::Instant,
}

#[must_use]
pub fn stopwatch() -> Stopwatch {
    Stopwatch {
        #[cfg(not(target_arch = "wasm32"))]
        start: std::time::Instant::now(),
    }
}

impl Stopwatch {
    #[cfg(not(target_arch = "wasm32"))]
    #[must_use]
    pub fn elapsed_us(&self) -> u128 {
        self.start.elapsed().as_micros()
    }

    #[cfg(target_arch = "wasm32")]
    #[must_use]
    pub fn elapsed_us(&self) -> u128 {
        0
    }
}

#[cfg(test)]
mod tests;
