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

/// Pipeline stages whose cumulative wall time the process keeps, so a
/// replay can report where its planning budget went.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Stage {
    VelocityPlan,
    PlanningBlockedOnShaper,
    ShaperBusy,
    MaterializeSource,
    LeaderFit,
    FollowerProjection,
    FollowerIngest,
    FollowerSourceFit,
    FollowerBreakpoints,
    FollowerKernelFit,
    MotorSide,
}

impl Stage {
    pub const ALL: [Self; 11] = [
        Self::VelocityPlan,
        Self::PlanningBlockedOnShaper,
        Self::ShaperBusy,
        Self::MaterializeSource,
        Self::LeaderFit,
        Self::FollowerProjection,
        Self::FollowerIngest,
        Self::FollowerSourceFit,
        Self::FollowerBreakpoints,
        Self::FollowerKernelFit,
        Self::MotorSide,
    ];

    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Self::VelocityPlan => "velocity_plan",
            Self::PlanningBlockedOnShaper => "planning_blocked_on_shaper",
            Self::ShaperBusy => "shaper_busy",
            Self::MaterializeSource => "materialize_source",
            Self::LeaderFit => "leader_fit",
            Self::FollowerProjection => "follower_projection",
            Self::FollowerIngest => "follower_ingest",
            Self::FollowerSourceFit => "follower_source_fit",
            Self::FollowerBreakpoints => "follower_breakpoints",
            Self::FollowerKernelFit => "follower_kernel_fit",
            Self::MotorSide => "motor_side",
        }
    }
}

/// Cumulative time and entry count of one [`Stage`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StageTotal {
    pub stage: Stage,
    pub total_us: u64,
    pub calls: u64,
}

struct StageCounter {
    total_us: std::sync::atomic::AtomicU64,
    calls: std::sync::atomic::AtomicU64,
}

static STAGE_COUNTERS: [StageCounter; Stage::ALL.len()] = [const {
    StageCounter {
        total_us: std::sync::atomic::AtomicU64::new(0),
        calls: std::sync::atomic::AtomicU64::new(0),
    }
}; Stage::ALL.len()];

pub(crate) fn record_stage(stage: Stage, elapsed_us: u128) {
    let counter = &STAGE_COUNTERS[stage as usize];
    counter.total_us.fetch_add(
        u64::try_from(elapsed_us).unwrap_or(u64::MAX),
        std::sync::atomic::Ordering::Relaxed,
    );
    counter
        .calls
        .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
}

/// Every stage's totals since the process started.
#[must_use]
pub fn stage_totals() -> [StageTotal; Stage::ALL.len()] {
    Stage::ALL.map(|stage| {
        let counter = &STAGE_COUNTERS[stage as usize];
        StageTotal {
            stage,
            total_us: counter.total_us.load(std::sync::atomic::Ordering::Relaxed),
            calls: counter.calls.load(std::sync::atomic::Ordering::Relaxed),
        }
    })
}
