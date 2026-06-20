// Gating oracle for the exact-crossing buzz stream. The producer (refill) now
// runs in the FOREGROUND and the step-output ISR (`step_output_event`) is a pure
// consumer, so every test drives the two separately: `refill_foreground` tops up
// the rings, `step_output_event` only pops + fires.
#![allow(clippy::unwrap_used, clippy::indexing_slicing)]

use crate::buzz_gen::{ToneCursor, ToneParams, next_crossing};
use crate::buzz_stream::{arm_axis, refill_foreground_all, reset_for_test};
use crate::per_axis_timer::{step_output_event, test_hooks};
use std::vec::Vec;

/// Foreground refill against the same thread-local queues `step_output_event`
/// consumes, with a no-op kick (the host build arms no real timer). This is the
/// sole producer in the new model.
fn refill_foreground(now: u32) {
    // SAFETY: `test_hooks::queue_for_axis` yields null or a live thread-local
    // `StepQueue`; the test thread is the sole producer; the kick is a no-op.
    unsafe {
        refill_foreground_all(now, test_hooks::queue_for_axis, |_axis, _cycle| {});
    }
}

const CPS: f64 = 520_000_000.0;
const AXIS: usize = 2;

fn tone_params(anchor: u32) -> ToneParams {
    ToneParams {
        omega: 2.0 * core::f32::consts::PI * 100.0,
        mu: 0.0,
        amplitude_mm: 0.1,
        sign: 1.0,
        base_mm: 0.0,
        microstep_distance: 0.01,
        anchor_cycle: anchor,
        cycles_per_second: CPS,
        total_seconds: 0.02,
        ramp_seconds: 0.002,
    }
}

fn chirp_params(anchor: u32) -> ToneParams {
    let f0 = 40.0f32;
    let f1 = 160.0f32;
    let total = 0.05f32;
    ToneParams {
        omega: 2.0 * core::f32::consts::PI * f0,
        mu: 2.0 * core::f32::consts::PI * (f1 - f0) / total,
        amplitude_mm: 0.1,
        sign: 1.0,
        base_mm: 0.0,
        microstep_distance: 0.01,
        anchor_cycle: anchor,
        cycles_per_second: CPS,
        total_seconds: total,
        ramp_seconds: 0.005,
    }
}

/// The rate-independent ground truth: every (cycle_abs, dir) the solver yields,
/// collected by threading one cursor through `next_crossing`.
fn solver_edges(p: &ToneParams) -> Vec<(u32, i8)> {
    let mut cursor = ToneCursor::start();
    let mut out = Vec::new();
    while let Ok(c) = next_crossing(p, cursor) {
        out.push((c.cycle_abs, c.dir));
        cursor = ToneCursor {
            level: c.level,
            t_cursor: c.t,
        };
    }
    out
}

/// Drive the consumer ISR over `[start_now, end_now)` advancing `now` by
/// `now_step` each call. Returns the emitted (dir) edges in order.
fn drain_via_consumer(start_now: u32, end_now: u32, now_step: u32) -> Vec<i8> {
    test_hooks::set_owned_mask(1u8 << AXIS);
    // Never count buzz edges as "late" — lateness is a metric, not behaviour.
    test_hooks::set_late_threshold(u32::MAX);
    let mut now = start_now;
    while (end_now.wrapping_sub(now) as i32) > 0 {
        test_hooks::set_now(now);
        refill_foreground(now);
        let _ = step_output_event();
        now = now.wrapping_add(now_step);
    }
    test_hooks::set_now(end_now);
    refill_foreground(end_now);
    let _ = step_output_event();
    test_hooks::take_emits()
        .into_iter()
        .map(|(_axis, dir, _sel)| dir as i8)
        .collect()
}

fn setup() {
    test_hooks::reset();
    reset_for_test();
}

#[test]
fn stream_is_rate_invariant_10khz_vs_40khz() {
    let anchor = 1_000_000u32;
    let p = tone_params(anchor);
    let edges = solver_edges(&p);
    assert!(!edges.is_empty(), "tone produced no crossings");
    let last_cycle = edges.iter().map(|(c, _)| *c).max().unwrap();
    let end = last_cycle.wrapping_add(10_000);

    // 40 kHz engine ISR cadence: 520 MHz / 40 kHz = 13_000 cycles per call.
    setup();
    arm_axis(AXIS, p);
    let dirs_40k = drain_via_consumer(anchor, end, 13_000);

    // 10 kHz cadence: 52_000 cycles per call. Same curve, same anchor.
    setup();
    arm_axis(AXIS, p);
    let dirs_10k = drain_via_consumer(anchor, end, 52_000);

    let oracle_dirs: Vec<i8> = edges.iter().map(|(_, d)| *d).collect();
    assert_eq!(dirs_40k, oracle_dirs, "40 kHz stream != solver oracle");
    assert_eq!(dirs_10k, oracle_dirs, "10 kHz stream != solver oracle");
    assert_eq!(dirs_40k, dirs_10k, "10 vs 40 kHz streams differ");
}

#[test]
fn tone_is_net_zero() {
    let anchor = 2_000_000u32;
    let p = tone_params(anchor);
    let edges = solver_edges(&p);
    let net: i32 = edges.iter().map(|(_, d)| i32::from(*d)).sum();
    assert_eq!(net, 0, "tone did not return to base (net {net})");
    // And via the consumer.
    let last = edges.iter().map(|(c, _)| *c).max().unwrap();
    setup();
    arm_axis(AXIS, p);
    let dirs = drain_via_consumer(anchor, last.wrapping_add(10_000), 13_000);
    let net_emitted: i32 = dirs.iter().map(|d| i32::from(*d)).sum();
    assert_eq!(net_emitted, 0, "consumer-emitted tone not net-zero");
}

#[test]
fn chirp_is_net_zero() {
    let anchor = 3_000_000u32;
    let p = chirp_params(anchor);
    let edges = solver_edges(&p);
    assert!(!edges.is_empty(), "chirp produced no crossings");
    let net: i32 = edges.iter().map(|(_, d)| i32::from(*d)).sum();
    assert_eq!(net, 0, "chirp did not return to base (net {net})");
}

#[test]
fn cycle_abs_is_strictly_monotonic() {
    for (label, p) in [
        ("tone", tone_params(4_000_000)),
        ("chirp", chirp_params(5_000_000)),
    ] {
        let edges = solver_edges(&p);
        assert!(!edges.is_empty(), "{label} produced no crossings");
        for w in edges.windows(2) {
            assert!(
                w[1].0 > w[0].0,
                "{label} cycle_abs not strictly increasing: {} then {}",
                w[0].0,
                w[1].0
            );
        }
    }
}

#[test]
fn oscillates_then_returns_via_consumer_no_fault() {
    let anchor = 6_000_000u32;
    let p = tone_params(anchor);
    let edges = solver_edges(&p);
    let last = edges.iter().map(|(c, _)| *c).max().unwrap();

    setup();
    arm_axis(AXIS, p);

    // Track the running microstep position and its peak excursion as the
    // consumer emits each edge.
    test_hooks::set_owned_mask(1u8 << AXIS);
    test_hooks::set_late_threshold(u32::MAX);
    let mut now = anchor;
    let mut pos = 0i32;
    let mut peak = 0i32;
    let end = last.wrapping_add(20_000);
    while (end.wrapping_sub(now) as i32) > 0 {
        test_hooks::set_now(now);
        refill_foreground(now);
        let _ = step_output_event();
        for (_axis, dir, _sel) in test_hooks::take_emits() {
            pos += dir;
            peak = peak.max(pos.abs());
        }
        now = now.wrapping_add(13_000);
    }

    assert!(
        peak >= 8,
        "buzz barely moved (peak {peak} microsteps); expected ~10"
    );
    assert_eq!(pos, 0, "position did not return to base");
    // No refill fault was latched.
    assert_eq!(crate::buzz_stream::take_refill_fault(), 0);
    // Stream is closed.
    assert!(!crate::buzz_stream::axis_active(AXIS));
}

#[test]
fn bench_scale_consumer_driven_by_next_wake_does_not_spin() {
    reset_for_test();
    let anchor = 6_000_000u32;
    let p = ToneParams {
        omega: 2.0 * core::f32::consts::PI * 54.3,
        mu: 0.0,
        amplitude_mm: 0.035,
        sign: 1.0,
        base_mm: 0.137,
        microstep_distance: 0.00625,
        anchor_cycle: anchor,
        cycles_per_second: 520_000_000.0,
        total_seconds: 4.0,
        ramp_seconds: 0.055,
    };
    arm_axis(AXIS, p);
    test_hooks::set_owned_mask(1u8 << AXIS);
    test_hooks::set_late_threshold(u32::MAX);

    let mut now = anchor;
    let (mut events, mut pos): (u64, i32) = (0, 0);
    loop {
        test_hooks::set_now(now);
        refill_foreground(now);
        let next = step_output_event();
        for (_a, dir, _s) in test_hooks::take_emits() {
            pos += dir;
        }
        events += 1;
        assert!(events < 5_000_000, "runaway: {events} events");
        if next == crate::per_axis_timer::STEP_OUTPUT_DISABLE {
            assert!(
                !crate::buzz_stream::axis_active(AXIS),
                "disabled but still active"
            );
            break;
        }
        // The hardware arms the compare timer to `next`; it MUST be strictly in
        // the future or the timer re-fires immediately and the ISR spins.
        assert!(
            (next.wrapping_sub(now) as i32) > 0,
            "SPIN at event {events}: next_wake {next} <= now {now}"
        );
        now = next;
    }
    assert_eq!(pos, 0, "net-zero");
    assert_eq!(crate::buzz_stream::take_refill_fault(), 0);
    std::eprintln!("bench-scale: {events} events, net pos {pos}");
}

/// The step-output ISR is a PURE CONSUMER: it never produces. After ONE
/// foreground refill primes the ring, draining via `step_output_event` alone
/// (no further foreground refill) empties the ring and stalls the buzz — the ISR
/// neither tops the queue up nor advances the solver, even though the stream is
/// still nominally active.
#[test]
fn step_output_event_is_pure_consumer_no_refill() {
    let anchor = 7_000_000u32;
    let p = tone_params(anchor);
    let edges = solver_edges(&p);
    let last = edges.iter().map(|(c, _)| *c).max().unwrap();
    assert!(edges.len() > crate::buzz_stream::test_high_water() as usize);

    setup();
    arm_axis(AXIS, p);
    test_hooks::set_owned_mask(1u8 << AXIS);
    test_hooks::set_late_threshold(u32::MAX);

    // Foreground refill primes the ring up to HIGH_WATER (a single call is
    // REFILL_BATCH-bounded, so loop a few passes as the real foreground does).
    test_hooks::set_now(anchor);
    for _ in 0..4 {
        refill_foreground(anchor);
    }
    let primed = unsafe { crate::step_queue::len(test_hooks::queue_for_axis(AXIS)) };
    assert_eq!(primed, crate::buzz_stream::test_high_water());

    // Drive the consumer ISR to the end WITHOUT any further refill. It can only
    // emit what was primed; once those pop it has nothing left to fire.
    let mut now = anchor;
    let mut emitted = 0usize;
    let end = last.wrapping_add(20_000);
    while (end.wrapping_sub(now) as i32) > 0 {
        test_hooks::set_now(now);
        let _ = step_output_event();
        emitted += test_hooks::take_emits().len();
        now = now.wrapping_add(13_000);
    }

    // At most the primed depth was ever emitted (the ISR produced nothing), the
    // ring is now empty, and the solver never closed the stream — it stalled.
    assert!(
        emitted <= crate::buzz_stream::test_high_water() as usize,
        "ISR produced edges: emitted {emitted} > primed {}",
        crate::buzz_stream::test_high_water()
    );
    assert!(emitted < edges.len(), "ISR drained the whole tone unaided");
    let remaining = unsafe { crate::step_queue::len(test_hooks::queue_for_axis(AXIS)) };
    assert_eq!(remaining, 0, "ring not drained");
    assert!(
        crate::buzz_stream::axis_active(AXIS),
        "stream closed without foreground refill — ISR must have produced"
    );
    assert_eq!(crate::buzz_stream::take_refill_fault(), 0);
}
