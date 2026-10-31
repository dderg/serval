/// Microseconds since the first call anywhere in this process. Shared across
/// the planner and the pump threads in `motion-core` so per-stage `pipe_*`
/// timestamps are directly comparable: subtract two `t_us` values to get the
/// inter-stage latency. This is the lowest crate both sides depend on, so the
/// epoch has to live here for the two clocks to be the same clock.
#[cfg(not(target_arch = "wasm32"))]
#[must_use]
pub fn mono_us() -> u64 {
    static EPOCH: std::sync::LazyLock<std::time::Instant> =
        std::sync::LazyLock::new(std::time::Instant::now);
    u64::try_from(EPOCH.elapsed().as_micros()).unwrap_or(u64::MAX)
}

/// wasm32 has no monotonic clock in `std`; the playground runs the pipeline
/// synchronously in one call, so inter-stage latency timestamps carry no
/// information there.
#[cfg(target_arch = "wasm32")]
#[must_use]
pub fn mono_us() -> u64 {
    0
}

/// Per-stage latency clock for the `pipe_*` tracing fields. A wrapper instead
/// of a bare `Instant` so the stages stay buildable on wasm32, where
/// `Instant::now()` aborts at runtime.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Stopwatch {
    #[cfg(not(target_arch = "wasm32"))]
    start: std::time::Instant,
}

#[must_use]
pub(crate) fn stopwatch() -> Stopwatch {
    Stopwatch {
        #[cfg(not(target_arch = "wasm32"))]
        start: std::time::Instant::now(),
    }
}

impl Stopwatch {
    #[cfg(not(target_arch = "wasm32"))]
    #[must_use]
    pub(crate) fn elapsed_us(&self) -> u128 {
        self.start.elapsed().as_micros()
    }

    #[cfg(target_arch = "wasm32")]
    #[must_use]
    pub(crate) fn elapsed_us(&self) -> u128 {
        0
    }
}

/// A stage phase that runs longer than this is attributed by one
/// `shaper_phase_slow` record. The shaper's whole budget is the anchor lead,
/// so a single phase burning 20ms is already a scheduling event.
const SLOW_PHASE_US: u128 = 20_000;

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
pub(crate) struct PhaseWorkload {
    pub segments: usize,
    pub window: usize,
    pub commit: usize,
    pub frontier: usize,
    pub axes: usize,
    pub pieces: usize,
    pub force: bool,
}

#[must_use]
pub(crate) fn is_slow_phase(elapsed_us: u128) -> bool {
    elapsed_us >= SLOW_PHASE_US
}

/// Emits the record described on [`PhaseWorkload`]. Callers guard with
/// [`is_slow_phase`] so neither the formatting nor a `detail` string is paid
/// for on the fast path.
pub(crate) fn log_slow_phase(
    phase: &'static str,
    elapsed_us: u128,
    work: PhaseWorkload,
    detail: &str,
) {
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
