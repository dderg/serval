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

/// A stage phase that runs longer than this is attributed by one
/// `shaper_phase_slow` record. The shaper's whole budget is the anchor lead,
/// so a single phase burning 20ms is already a scheduling event.
pub const SLOW_PHASE_US: u128 = 20_000;

/// Workload size behind one `shaper_phase_slow` record. Every field is a
/// count the emitting phase already has in hand, so a fast phase pays a
/// stopwatch read and scalar bookkeeping and nothing else.
///
/// Event schema (`subsystem = "motion"`, `event = "shaper_phase_slow"`):
/// - `phase` — which phase burned the time: `materialize_source`,
///   `leader_fit`, `follower_projection`, `motor_side`.
/// - `elapsed_us` — wall time inside that phase.
/// - `segments` — segments the phase walked.
/// - `window` — frontier window length the phase read; 0 when not windowed.
/// - `commit` — segments this pass commits downstream.
/// - `frontier` — shaping-frontier length this pass fitted through.
/// - `axes` — axis columns or tracks the phase rebuilt.
/// - `pieces` — piecewise-relative pieces the phase re-fitted.
/// - `force` — drain/flush pass, where the window is clamped instead of
///   covered by lookahead.
/// - `detail` — phase-specific breakdown, empty when the phase has none.
#[derive(Debug, Default, Clone, Copy)]
pub struct PhaseWorkload {
    pub segments: usize,
    pub window: usize,
    pub commit: usize,
    pub frontier: usize,
    pub axes: usize,
    pub pieces: usize,
    pub force: bool,
}

#[must_use]
pub fn is_slow_phase(elapsed_us: u128) -> bool {
    elapsed_us >= SLOW_PHASE_US
}

/// Emits the record described on [`PhaseWorkload`]. Callers guard with
/// [`is_slow_phase`] so neither the formatting nor a `detail` string is paid
/// for on the fast path.
pub fn log_slow_phase(phase: &'static str, elapsed_us: u128, work: PhaseWorkload, detail: &str) {
    tracing::warn!(
        subsystem = "motion",
        event = "shaper_phase_slow",
        phase,
        elapsed_us = elapsed_us as u64,
        segments = work.segments,
        window = work.window,
        commit = work.commit,
        frontier = work.frontier,
        axes = work.axes,
        pieces = work.pieces,
        force = work.force,
        detail,
        "shaper phase exceeded the slow-phase threshold"
    );
}
