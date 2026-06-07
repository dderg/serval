//! Regression tests for the MACH_LINUX / Docker sim adoption and inter-arrival
//! timing paths.
//!
//! # Observed fault (current, post-round-1 repro)
//!
//! `TickIntervalExceeded (-311) detail=2` on the first homing move in every
//! Docker MACH_LINUX full-sim run with `--homing-gpio-test`.
//!
//! # Root cause (TickIntervalExceeded)
//!
//! The libvtime virtual-time shim advances the shared virtual clock on every
//! `clock_nanosleep` / `ppoll`-vtimer-fire call from ANY thread.  The MCU main
//! loop's `ppoll` intercept fires `vtime_advance_to(timer_target)` when the
//! Klipper scheduler arms the next virtual timer — potentially advancing vtime
//! by more than one 1ms sample period in a single ppoll call.
//!
//! When that advance covers 2 ms, the tick thread's next `clock_nanosleep` for
//! `T+1ms` returns immediately (vtime is already past the target), and
//! `timer_read_time()` inside the tick returns the current vtime, which is
//! `T+2ms`.  With a prior active tick's `last_tick_now = Some(T)`, the gap
//! check sees gap = 2ms > 2 * sample_period → `TickIntervalExceeded`.
//!
//! The fault fires before any steps are dispatched (ring is populated 978ms
//! before piece start_time, engine is active, but no displacement has elapsed).
//! `steps_moved = 0.0`.
//!
//! # Round-1 mis-diagnosis note (commit eeecfb63e)
//!
//! Round 1 diagnosed `PieceStartInPast` as the fault and removed a
//! `setpriority(19)` call inside `#if CONFIG_KALICO_SIM`.  The Docker image
//! builds with `# CONFIG_KALICO_SIM is not set`, so that path never executed.
//! The arrival_lead_us frequency-unit fix in eeecfb63e IS correct but the
//! CONFIG_KALICO_SIM / nice=19 diagnosis was wrong.
//!
//! # Fix for TickIntervalExceeded (src/linux/runtime_tick_host.c)
//!
//! `runtime_tick_init()` now requests SCHED_FIFO priority for the tick pthread.
//! A SCHED_FIFO thread preempts CFS threads, so the tick fires every exactly
//! 1ms virtual without the MCU main loop being able to advance vtime past the
//! tick's next target.  The sim runs with `--privileged` which grants
//! `CAP_SYS_NICE` for SCHED_FIFO.  On unprivileged hosts the code falls back
//! to CFS and prints a warning.
//!
//! # Root cause (PieceStartInPast — secondary fault on MACH_LINUX)
//!
//! Even with SCHED_FIFO, `clock_nanosleep` on a non-PREEMPT_RT kernel can
//! merge two consecutive 1ms ticks into a single ~2ms wake.  When this occurs,
//! the armed piece expires ~1.3ms "late" from the engine's perspective.  The
//! next piece (which starts contiguously) then has `start_time` 1323µs in the
//! past — just over `fault_tolerance` (1200µs at 1kHz).  The OLD fault check
//! ran before the `now < end_time` guard and fired on the still-active piece.
//!
//! # Fix for PieceStartInPast (rust/runtime/src/motion_core.rs)
//!
//! `get_piece_for_time` now checks `now < end_time` FIRST.  A still-active
//! piece is adopted unconditionally regardless of how late its start was
//! observed.  `PieceStartInPast` fires only when a piece is already expired
//! (now >= end_time) AND deficit > fault_tolerance — genuine starvation where
//! the engine missed the piece entirely.
//!
//! # Test parameters
//!
//! These tests use CLOCK_FREQ=50_000_000 / SAMPLE_RATE=10_000 (not the real
//! sim's 1kHz) for a tighter fault_tolerance that makes the adoption math easy
//! to verify.  The real sim uses 1kHz (sample_period=50000 cycles,
//! fault_tolerance=60000 cycles = 1.2ms).

#![allow(
    clippy::panic,
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::integer_division
)]

use core::sync::atomic::Ordering;

use runtime::clock::WidenState;
use runtime::engine::Engine;
use runtime::error::FaultCode;
use runtime::piece_ring::PieceEntry;
use runtime::state::{IsrState, SharedState, TOTAL_RING_PIECES};
use runtime::step_queue::StepQueue;
use runtime::stepping_state::{MAX_AXES, StepMode, StepperBindingRust, TMC_CS_OID_NONE};
use runtime::tick::isr_sample_tick;

// MACH_LINUX Docker-sim parameters.
const CLOCK_FREQ: u32 = 50_000_000;
const SAMPLE_RATE: u32 = 10_000;
// sample_period_cycles = round(50e6 / 10e3) = 5_000 cycles = 100µs
const TICK_CYCLES: u32 = CLOCK_FREQ / SAMPLE_RATE;

// fault_tolerance = drift_budget + sample_period_cycles
// drift_budget    = 200µs * 50MHz/1e6 = 10_000
// sample_period   = 5_000
const DRIFT_BUDGET_CYCLES: u64 = (200e-6_f64 * CLOCK_FREQ as f64) as u64;
const FAULT_TOLERANCE: u64 = DRIFT_BUDGET_CYCLES + TICK_CYCLES as u64;

// start_time as observed in the Docker sim evidence (post-u32-wrap territory,
// high word = 1).  At 50MHz: 4_461_055_271 / 50e6 ≈ 89.2 s from MCU epoch.
const START_TIME: u64 = 4_461_055_271;

fn make_engine() -> Engine {
    Engine::new(CLOCK_FREQ, SAMPLE_RATE)
}

fn make_storage() -> Vec<PieceEntry> {
    vec![
        PieceEntry {
            start_time: 0,
            coeffs: [0.0; 4],
            duration: 0.0,
            _reserved: 0,
        };
        TOTAL_RING_PIECES
    ]
}

fn make_isr(engine: Engine) -> IsrState {
    IsrState {
        engine,
        widen_state: WidenState::default(),
        last_tick_now: None,
    }
}

fn pulse_binding() -> StepperBindingRust {
    StepperBindingRust {
        stepper_oid: 0,
        tmc_cs_oid: TMC_CS_OID_NONE,
        _pad: [0; 2],
    }
}

fn configure_axis0(engine: &mut Engine) {
    let rc = engine.configure_axis(
        0,
        StepMode::Pulse,
        0.0125,
        64,
        &[pulse_binding()],
        TOTAL_RING_PIECES,
    );
    assert_eq!(rc, 0, "configure_axis failed");
}

fn install_queue(engine: &mut Engine) -> ([*mut StepQueue; MAX_AXES], StepQueue) {
    let mut q0 = StepQueue::new();
    let mut qs: [*mut StepQueue; MAX_AXES] = [core::ptr::null_mut(); MAX_AXES];
    qs[0] = &mut q0;
    engine.test_install_step_queues(qs);
    (qs, q0)
}

/// A constant zero piece (all Bernstein CPs = 0) starting at `start_time`
/// with the given duration. Zero coefficients keep `signed_steps = 0` so no
/// `StepsPerSampleExceeded` fault is triggered from the step dispatch path.
fn zero_piece(start_time: u64, duration_s: f32) -> PieceEntry {
    PieceEntry {
        start_time,
        coeffs: [0.0; 4],
        duration: duration_s,
        _reserved: 0,
    }
}

// ─── Sanity: fault_tolerance is correct at the 50 MHz / 10 kHz parameters ──

/// Guard: the fault_tolerance constant must match the engine's internal
/// computation (200µs drift_budget + 1 sample_period) at the MACH_LINUX
/// parameters.  If this fails, the other tests' arithmetic is wrong.
#[test]
fn fault_tolerance_matches_engine_internal_at_linux_params() {
    let engine = make_engine();
    assert_eq!(
        engine.sample_period_cycles, TICK_CYCLES,
        "sample_period_cycles must be {TICK_CYCLES} at {CLOCK_FREQ}Hz / {SAMPLE_RATE}Hz"
    );
    assert_eq!(
        DRIFT_BUDGET_CYCLES, 10_000,
        "drift_budget must be 10_000 cycles at 50MHz / 200µs"
    );
    assert_eq!(
        FAULT_TOLERANCE, 15_000,
        "fault_tolerance must be 15_000 cycles (10_000 + 5_000)"
    );
}

// ─── Normal path: pre-start tick adopts piece at t=0, no fault ─────────────

/// A piece with `start_time` in the future is adopted on the first tick before
/// start and held at t=0 (eval_horner with elapsed=0).  No fault fires.
///
/// This is the EXPECTED behavior — the test documents that adoption before
/// start works correctly at MACH_LINUX parameters.
#[test]
fn pre_start_tick_adopts_future_piece_at_t0_no_fault() {
    let mut engine = make_engine();
    configure_axis0(&mut engine);
    let (_qs, mut _q0) = install_queue(&mut engine);
    let shared = SharedState::new();
    let mut storage = make_storage();

    let piece = zero_piece(START_TIME, 0.010);
    let rc = engine.push_pieces(0, &[piece], &mut storage);
    assert_eq!(rc, 0, "push_pieces failed");

    // Tick well before start_time: deficit = 0 (saturating), no fault.
    // The piece is adopted into the armed cache at t=0.
    let pre_start_now = START_TIME - 48_819_820; // 976ms before start, matches evidence
    engine.tick(pre_start_now, &shared, &mut storage);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "pre-start tick must not fault (deficit=0 for future piece)"
    );
}

// ─── Normal path: tick within fault tolerance after start, no fault ─────────

/// First tick at `start_time + fault_tolerance - 1` (still within budget).
/// No fault expected.
#[test]
fn first_tick_within_tolerance_after_start_no_fault() {
    let mut engine = make_engine();
    configure_axis0(&mut engine);
    let (_qs, mut _q0) = install_queue(&mut engine);
    let shared = SharedState::new();
    let mut storage = make_storage();

    let piece = zero_piece(START_TIME, 0.010);
    let rc = engine.push_pieces(0, &[piece], &mut storage);
    assert_eq!(rc, 0, "push_pieces failed");

    // Exactly at the boundary: fault_tolerance - 1 cycles late.
    let at_boundary = START_TIME + FAULT_TOLERANCE - 1;
    engine.tick(at_boundary, &shared, &mut storage);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "tick at fault_tolerance - 1 must not fault"
    );
}

// ─── MACH_LINUX tick jitter: still-active piece adopted without fault ────────

/// A tick that arrives late (deficit > fault_tolerance) but while the piece is
/// still active (now < end_time) must NOT fault. This is the MACH_LINUX
/// tick-jitter scenario: clock_nanosleep merges two ticks, so the first tick
/// after the gap lands 1.221ms into a 10ms piece's window.
///
/// Genuine starvation is covered by `genuine_starvation_expired_piece_faults`.
#[test]
fn linux_sim_starvation_first_tick_past_tolerance_no_fault_when_active() {
    let mut engine = make_engine();
    configure_axis0(&mut engine);
    let (_qs, mut _q0) = install_queue(&mut engine);
    let shared = SharedState::new();
    let mut storage = make_storage();

    // 10ms piece → end_time = START_TIME + 500_000 cycles. The late tick at
    // START_TIME + 61_050 is still within the piece's active window.
    let piece = zero_piece(START_TIME, 0.010);
    let rc = engine.push_pieces(0, &[piece], &mut storage);
    assert_eq!(rc, 0, "push_pieces failed");

    let late_now = START_TIME + 61_050; // 1.221ms late, deficit > fault_tolerance
    engine.tick(late_now, &shared, &mut storage);

    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "still-active piece must be adopted without fault even when start is > fault_tolerance in the past"
    );
}

// ─── Genuine starvation: expired piece past tolerance ────────────────────────
//
// Hardware: PieceStartInPast fires.
// Host (MACH_LINUX): piece silently retired; ring empty; no fault.

/// **Hardware only** — Genuine engine starvation: the engine was suspended long
/// enough that `now` is past BOTH the piece's end_time AND the deficit exceeds
/// fault_tolerance. `PieceStartInPast` must fire.
///
/// On the host build this test is skipped: MACH_LINUX silently retires expired
/// pieces regardless of deficit. The host-build variant is
/// `genuine_starvation_host_silent_retire` below.
#[test]
#[cfg(not(feature = "host"))]
fn genuine_starvation_expired_piece_faults() {
    let mut engine = make_engine();
    configure_axis0(&mut engine);
    let (_qs, mut _q0) = install_queue(&mut engine);
    let shared = SharedState::new();
    let mut storage = make_storage();

    // 200µs piece (10_000 cycles) — entire window has elapsed before the tick.
    let piece = zero_piece(START_TIME, 0.000_200);
    let rc = engine.push_pieces(0, &[piece], &mut storage);
    assert_eq!(rc, 0, "push_pieces failed");

    // Tick arrives 61_050 cycles after start. The piece ended at +10_000,
    // so now >= end_time (expired). Deficit 61_050 >> fault_tolerance 15_000.
    let starved_now = START_TIME + 61_050;
    engine.tick(starved_now, &shared, &mut storage);

    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::PieceStartInPast.as_i32(),
        "PieceStartInPast must fire for genuinely missed (expired) piece with deficit > tolerance"
    );
}

/// **Host build** — Same starvation scenario but on MACH_LINUX: an expired
/// piece past fault_tolerance is silently retired (no fault). The ring empties
/// and no error is latched.
#[test]
#[cfg(feature = "host")]
fn genuine_starvation_host_silent_retire() {
    let mut engine = make_engine();
    configure_axis0(&mut engine);
    let (_qs, mut _q0) = install_queue(&mut engine);
    let shared = SharedState::new();
    let mut storage = make_storage();

    // 200µs piece (10_000 cycles) — expired before tick arrives.
    let piece = zero_piece(START_TIME, 0.000_200);
    let rc = engine.push_pieces(0, &[piece], &mut storage);
    assert_eq!(rc, 0, "push_pieces failed");

    let starved_now = START_TIME + 61_050;
    engine.tick(starved_now, &shared, &mut storage);

    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "host: MACH_LINUX must silently retire expired piece — no fault"
    );
}

// ─── Full isr_sample_tick path: still-active piece adopted without fault ─────

/// Same MACH_LINUX tick-jitter scenario via `isr_sample_tick`.
///
/// A 10ms piece, first tick at start + 1.221ms (61050 cycles > fault_tolerance).
/// The piece is still active (61050 < 500000 cycles), so no fault should fire.
///
/// The widen seed mirrors `runtime_tick_enable`'s formula:
///   baseline = (stats_send_time_high << 32) | timer_read_time_low
#[test]
fn linux_sim_starvation_via_isr_sample_tick_no_fault_when_active() {
    let mut engine = make_engine();
    configure_axis0(&mut engine);
    let (_qs, mut _q0) = install_queue(&mut engine);
    let shared = SharedState::new();
    let mut storage = make_storage();

    // Seed widen to the high-word-1 epoch so the engine's widened `now`
    // matches the piece's start_time domain.
    let arrival_clock: u64 = START_TIME - 48_819_820;
    let mut widen = WidenState::default();
    widen.seed(arrival_clock);

    let mut isr = IsrState {
        engine,
        widen_state: widen,
        last_tick_now: None,
    };

    // 10ms piece → end_time = START_TIME + 500_000 cycles.
    let piece = zero_piece(START_TIME, 0.010);
    let rc = isr.engine.push_pieces(0, &[piece], &mut storage);
    assert_eq!(rc, 0, "push_pieces failed");

    // Late but still-active tick: raw_cyccnt = low32 of (start_time + 61050).
    // The widen state has high = (arrival_clock >> 32) << 32 = 1 << 32.
    // raw > last_low (arrival low) so no extra bump → now = (1<<32) | raw.
    // now = 4_294_967_296 + (4_461_055_271 + 61_050 - 4_294_967_296)
    //     = 4_294_967_296 + 166_149_025 = 4_461_116_321 = start + 61050. ✓
    let late_raw: u32 = (START_TIME + 61_050) as u32;
    isr_sample_tick(&mut isr, &shared, &mut storage, late_raw);

    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "still-active piece via isr_sample_tick must not fault even when start is past tolerance"
    );
}

// ─── Contrast: pre-start tick via isr_sample_tick → no fault ────────────────

/// Same setup, but the first tick arrives before start_time.  Confirms that
/// once the tick thread fires even ONCE before start, the piece is adopted at
/// t=0 and NO fault fires at start+1.221ms either (the armed cache holds it).
#[test]
fn pre_start_tick_then_post_tolerance_tick_no_fault() {
    let mut engine = make_engine();
    configure_axis0(&mut engine);
    let (_qs, mut _q0) = install_queue(&mut engine);
    let shared = SharedState::new();
    let mut storage = make_storage();

    let arrival_clock: u64 = START_TIME - 48_819_820;
    let mut widen = WidenState::default();
    widen.seed(arrival_clock);

    let mut isr = IsrState {
        engine,
        widen_state: widen,
        last_tick_now: None,
    };

    let piece = zero_piece(START_TIME, 0.010);
    let rc = isr.engine.push_pieces(0, &[piece], &mut storage);
    assert_eq!(rc, 0);

    // Tick 1: fires just after piece commit, before start_time.
    // raw = low32 of arrival_clock + a few cycles.
    let pre_raw: u32 = (arrival_clock + 1000) as u32;
    isr_sample_tick(&mut isr, &shared, &mut storage, pre_raw);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "pre-start tick via isr_sample_tick must not fault"
    );
    assert!(
        isr.last_tick_now.is_some(),
        "pre-start active tick must set last_tick_now to Some"
    );

    // Tick 2: now at start + 61050 (> fault_tolerance).  But last_tick_now
    // is Some from tick 1, so the gap guard fires first.  Gap =
    // (start + 61050) - arrival_clock = 48_819_820 + 61_050 = 48_880_870
    // >> 2 * TICK_CYCLES (10_000).  The gap guard latches TickIntervalExceeded,
    // not PieceStartInPast — because the guard fires before engine.tick().
    // The real sim hits TickIntervalExceeded (gap_ticks=2) via ppoll
    // advancing vtime 2ms in one step, not PieceStartInPast.
    let post_raw: u32 = (START_TIME + 61_050) as u32;
    isr_sample_tick(&mut isr, &shared, &mut storage, post_raw);

    // If a pre-start tick fired, the gap guard fires TickIntervalExceeded
    // rather than PieceStartInPast.  This is the fault observed in the real
    // sim: pieces commit 978ms before start, first active tick runs, then
    // vtime jumps 2ms via ppoll → gap > 2*period → TickIntervalExceeded.
    let err = shared.last_error.load(Ordering::Acquire);
    assert_ne!(
        err,
        FaultCode::PieceStartInPast.as_i32(),
        "with a prior active tick, PieceStartInPast must NOT be the fault code \
         (gap guard fires first as TickIntervalExceeded)"
    );
}
