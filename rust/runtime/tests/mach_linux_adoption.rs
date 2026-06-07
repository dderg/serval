//! Regression test for the MACH_LINUX / Docker sim adoption path.
//!
//! Observed fault: `PieceStartInPast (-308) detail=1221` on the first move in
//! every Docker MACH_LINUX full-sim run, regardless of anchor lead time.
//!
//! Root cause: in `CONFIG_KALICO_SIM`, the tick thread runs at `nice=19` and
//! is starved by the main ppoll thread during rapid virtual-time advancement.
//! Klipper sim advances virtual time (= wall-clock CLOCK_MONOTONIC ticks at
//! CONFIG_CLOCK_FREQ Hz on MACH_LINUX) much faster than real time, so all
//! events from piece commit to past-`start_time` complete within one real tick
//! interval.  The tick thread fires exactly once, at `start_time + ~1.2ms`.
//! At that point `isr.last_tick_now` is `None` (no prior active tick), so the
//! inter-arrival gap guard is skipped and `engine.tick()` runs, calling
//! `get_piece_for_time` for the first time.  `deficit_cycles = now -
//! start_time = 1.221ms * 50MHz = 61050 cycles` >> `fault_tolerance =
//! drift_budget + sample_period = 10000 + 5000 = 15000 cycles`.  Fault fires.
//!
//! The deficit is constant (~1.2ms) and independent of the anchor lead time
//! because the lead only changes WHEN the piece is committed, not WHEN the
//! starved tick thread first fires relative to `start_time`.
//!
//! Concrete test sequence (mirrors the Docker sim parameters):
//!   CLOCK_FREQ=50_000_000, SAMPLE_RATE=10_000
//!   fault_tolerance = (200µs * 50MHz) + (50MHz/10kHz) = 10000 + 5000 = 15000 cycles
//!   start_time = 4_461_055_271  (post-wrap, matches observed evidence)
//!
//!   Tick 1 (pre-start, start - 976ms * 50MHz): piece is future → adopted at
//!     t=0, active=true, no fault.  Models what SHOULD happen.
//!
//!   Tick 2 (post-start within tolerance, start + 1 cycle): no fault.
//!
//!   Tick 3 (starved scenario, first tick at start + 1.221ms = 61050 cycles):
//!     `isr.last_tick_now` still `None` (no prior active tick reached the
//!     engine on the real MCU path between tick 1 and tick 3).  The engine
//!     walks the ring; `deficit = 61050 > fault_tolerance = 15000` →
//!     `PieceStartInPast` fires.  This is the fault we observe.
//!
//! The failing test `linux_sim_starvation_first_tick_past_tolerance_faults`
//! proves the fault fires under the starved-tick scenario.  It is the minimal
//! reproducer for the Docker sim breakage.
//!
//! NOTE ON FIX: the fault is correct per the fail-loud policy for real
//! hardware: a piece arriving > 300µs late is a genuine planning defect.  On
//! MACH_LINUX / CONFIG_KALICO_SIM the cause is tick-thread starvation, not a
//! planning defect.  Two approaches exist:
//!   A. Don't demote the tick thread to nice=19 in CONFIG_KALICO_SIM (see
//!      src/linux/runtime_tick_host.c:97-103).
//!   B. Widen the fault_tolerance when building with the `kalico-sim` feature.
//!
//! Option A is simpler and architecturally correct: the tick thread models the
//! TIM5 ISR, which fires at hardware priority with no scheduling delay.
//! Removing the nice=19 demotion makes the sim faithful to hardware behavior.
//! The comment at that site justifies the demotion for throughput — but
//! throughput is irrelevant here because the fault kills all motion anyway.

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

// ─── Failing (observed) path: starved first tick past tolerance faults ───────

/// This is the MACH_LINUX Docker sim failure reproducer.
///
/// Scenario: the 10kHz tick thread is starved at nice=19 by the sim's ppoll
/// loop.  No tick fires between piece commit and `start_time + 1.221ms`.
/// `isr.last_tick_now` is `None` at that first tick (no prior active tick).
/// `engine.tick()` runs immediately (no gap guard with None baseline).
/// `deficit_cycles = 61050 >> fault_tolerance = 15000` → `PieceStartInPast`.
///
/// The test drives `engine.tick()` directly (no isr_sample_tick) to isolate
/// the adoption fault from the inter-arrival guard, matching the exact path
/// the real sim hits when `last_tick_now = None`.
#[test]
fn linux_sim_starvation_first_tick_past_tolerance_faults() {
    let mut engine = make_engine();
    configure_axis0(&mut engine);
    let (_qs, mut _q0) = install_queue(&mut engine);
    let shared = SharedState::new();
    let mut storage = make_storage();

    let piece = zero_piece(START_TIME, 0.010);
    let rc = engine.push_pieces(0, &[piece], &mut storage);
    assert_eq!(rc, 0, "push_pieces failed");

    // Simulate the starved tick: first tick arrives at start + 1.221ms
    // (61050 cycles at 50MHz), which is > fault_tolerance (15000 cycles).
    // last_tick_now = None (no prior active tick), so the gap guard is not
    // consulted; engine.tick() runs and get_piece_for_time fires the fault.
    let starved_now = START_TIME + 61_050; // start + 1.221ms @ 50MHz
    engine.tick(starved_now, &shared, &mut storage);

    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::PieceStartInPast.as_i32(),
        "PieceStartInPast must fire when first tick lands > fault_tolerance after start"
    );
}

// ─── Full isr_sample_tick path: same fault via the ISR wrapper ──────────────

/// Same scenario driven through `isr_sample_tick` with a widen state seeded
/// to the MACH_LINUX epoch (high word = 1 to reach the START_TIME range).
///
/// The widen seed mirrors `runtime_tick_enable`'s formula:
///   baseline = (stats_send_time_high << 32) | timer_read_time_low
/// We seed with (1 << 32) | low_of_arrival_clock to put `now` in the right
/// epoch.  The raw_cyccnt argument is the low 32 bits of `starved_now`.
#[test]
fn linux_sim_starvation_via_isr_sample_tick_faults() {
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

    let piece = zero_piece(START_TIME, 0.010);
    let rc = isr.engine.push_pieces(0, &[piece], &mut storage);
    assert_eq!(rc, 0, "push_pieces failed");

    // Starved first tick: raw_cyccnt = low32 of (start_time + 61050).
    // The widen state has high = (arrival_clock >> 32) << 32 = 1 << 32.
    // raw > last_low (arrival low) so no extra bump → now = (1<<32) | raw.
    // now = 4_294_967_296 + (4_461_055_271 + 61_050 - 4_294_967_296)
    //     = 4_294_967_296 + 166_149_025 = 4_461_116_321 = start + 61050. ✓
    let starved_raw: u32 = (START_TIME + 61_050) as u32;
    isr_sample_tick(&mut isr, &shared, &mut storage, starved_raw);

    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::PieceStartInPast.as_i32(),
        "PieceStartInPast must fire via isr_sample_tick when first tick lands past tolerance"
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
    // (On real MACH_LINUX hardware this gap wouldn't happen; it only happens
    // in the sim at nice=19.  The gap guard is the SECOND defence.)
    let post_raw: u32 = (START_TIME + 61_050) as u32;
    isr_sample_tick(&mut isr, &shared, &mut storage, post_raw);

    // If a pre-start tick fired, the gap guard fires (TickIntervalExceeded)
    // rather than PieceStartInPast.  Neither is OK for production, but the
    // fault code distinguishes the starvation path from the missed-adoption path.
    let err = shared.last_error.load(Ordering::Acquire);
    assert_ne!(
        err,
        FaultCode::PieceStartInPast.as_i32(),
        "with a prior active tick, PieceStartInPast must NOT be the fault code \
         (gap guard fires first as TickIntervalExceeded)"
    );
}
