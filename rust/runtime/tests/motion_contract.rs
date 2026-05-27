//! MCU motion contract tests.
//!
//! These tests specify what the MCU motion engine SHOULD do, expressed as
//! the contract defined in the clean-motion-contract design:
//!
//! - MCU is a dumb executor. Host pre-computes all timing in MCU clock domain.
//! - t_start is always explicit, never 0. Every segment carries a wall-clock
//!   start time in MCU cycles.
//! - TIM5 runs continuously. No cold-start, no timer enable/disable.
//! - t_start < now at arm time is a fault (host timing error), not a silent rebase.
//! - Queue empty after last segment = MCU holds position (no fault).
//! - Cancel = stop immediately, report state.

#![allow(unsafe_code, clippy::unwrap_used)]

// Tests must run serially: the c_segment_queue is a process-wide singleton.
// Run with: cargo test -p runtime --test motion_contract -- --test-threads=1
//
// Each test acquires SERIAL_LOCK to enforce this even without the flag.
use std::sync::Mutex;
static SERIAL_LOCK: Mutex<()> = Mutex::new(());

mod bench;

use core::sync::atomic::Ordering;
use bench::McuTestBench;
use runtime::curve_pool::CurveHandle;

// ---------------------------------------------------------------------------
// Test 1: Happy path — segment in the future is parked, armed, evaluated, retired
// ---------------------------------------------------------------------------

#[test]
fn segment_with_future_t_start_is_evaluated_and_retired() {
    let _lock = SERIAL_LOCK.lock().unwrap();
    let mut bench = McuTestBench::new_h7();

    let x_curve = bench.load_linear_curve(0.0, 10.0, 0.1);
    let seg_id = bench.push_segment_xy(1, x_curve, CurveHandle::UNUSED_SENTINEL);

    bench.tick_for_ms(120.0);

    assert_eq!(
        bench.retired_through(),
        seg_id,
        "segment must retire after its duration elapses"
    );
    assert!(
        bench.x_step_count().abs() >= 700,
        "10mm at 0.0125mm/microstep should produce ~800 steps, got {}",
        bench.x_step_count()
    );
}

// ---------------------------------------------------------------------------
// Test 2: Segment with t_start slightly in the past (chaining jitter) arms OK
// ---------------------------------------------------------------------------

#[test]
fn segment_with_t_start_slightly_past_arms_normally() {
    let _lock = SERIAL_LOCK.lock().unwrap();
    let mut bench = McuTestBench::new_h7();

    // Advance clock a bit so now > 0.
    bench.tick_for_ms(5.0);

    // Push a segment whose t_start is 2 sample periods in the past.
    // This simulates normal chaining jitter — the ISR should arm it.
    let x_curve = bench.load_linear_curve(0.0, 10.0, 0.1);
    let now = bench.now_cycles();
    let two_ticks = 2 * u64::from(bench.cycles_per_sample());
    let t_start = now.saturating_sub(two_ticks);
    let duration = bench.ms_to_cycles(100.0);
    bench.push_segment_raw(1, x_curve, CurveHandle::UNUSED_SENTINEL, t_start, duration);

    bench.tick_for_ms(120.0);

    assert!(
        !bench.has_fault(),
        "segment with t_start a few ticks in the past must arm, not fault"
    );
    assert_eq!(bench.retired_through(), 1, "segment must retire");
}

// ---------------------------------------------------------------------------
// Test 3: Queue empty after last segment → hold position, no fault
// ---------------------------------------------------------------------------

#[test]
fn empty_queue_after_segment_holds_position_no_fault() {
    let _lock = SERIAL_LOCK.lock().unwrap();
    let mut bench = McuTestBench::new_h7();

    let x_curve = bench.load_linear_curve(0.0, 10.0, 0.1);
    bench.push_segment_xy(1, x_curve, CurveHandle::UNUSED_SENTINEL);

    // Tick through the segment plus extra time with empty queue.
    bench.tick_for_ms(120.0);
    assert_eq!(bench.retired_through(), 1);

    let position_after_retire = bench.x_step_count();

    // Continue ticking for 200ms with no segments queued.
    bench.tick_for_ms(200.0);

    assert!(
        !bench.has_fault(),
        "empty queue must not fault — MCU holds position"
    );
    assert_eq!(
        bench.x_step_count(),
        position_after_retire,
        "position must not change while queue is empty"
    );
}

// ---------------------------------------------------------------------------
// Test 4: Two chained segments execute in sequence
// ---------------------------------------------------------------------------

#[test]
fn two_chained_segments_execute_in_order() {
    let _lock = SERIAL_LOCK.lock().unwrap();
    let mut bench = McuTestBench::new_h7();

    let duration_cycles = bench.ms_to_cycles(100.0);
    let lead = bench.ms_to_cycles(1.0);
    let t_start_1 = bench.now_cycles() + lead;
    let t_start_2 = t_start_1 + duration_cycles;

    let curve1 = bench.load_linear_curve(0.0, 10.0, 0.1);
    let curve2 = bench.load_linear_curve(10.0, 20.0, 0.1);

    bench.push_segment_raw(1, curve1, CurveHandle::UNUSED_SENTINEL, t_start_1, duration_cycles);
    bench.push_segment_raw(2, curve2, CurveHandle::UNUSED_SENTINEL, t_start_2, duration_cycles);

    // Tick through both segments plus margin.
    bench.tick_for_ms(250.0);

    assert_eq!(
        bench.retired_through(),
        2,
        "both segments must retire"
    );
    assert!(
        bench.x_step_count().abs() >= 1400,
        "20mm total at 0.0125mm/microstep should produce ~1600 steps, got {}",
        bench.x_step_count()
    );
}
