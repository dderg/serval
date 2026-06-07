#![allow(clippy::unnecessary_mut_passed)]
//!
//! ## Branch map
//!
//! 1. **Current piece still live** (`now < piece_end`) — eval_horner returns
//!    a non-None (pos, vel). Covered by `walker_branch1_current_piece_eval`.
//!
//! 2. **Ring empty** — returns `None` without faulting. Covered by
//!    `walker_branch2_empty_ring_returns_none`.
//!
//! 3. **Expired piece with deficit > fault_tolerance** — `PieceStartInPast`
//!    fires. `TestFaultSink` count increments, returns `None`. Covered by
//!    `walker_branch3_past_piece_faults` and `walker_fault_boundary_*`.
//!
//! 4. (Walk/load) — walked-past pieces retired without monomialisation; only
//!    the landed piece is armed.
//!
//! ## Fault boundary invariant (DO NOT MODIFY)
//!
//! The tolerance formula is:
//!   `drift_budget = (200e-6 * cycles_per_second) as u64`
//!   `fault_tolerance = drift_budget + sample_period_cycles`
//!
//! At 520 MHz / 40 kHz (TICK_CYCLES = 13_000):
//!   `drift_budget = (200e-6 * 520_000_000) as u64 = 104_000`
//!   `fault_tolerance = 104_000 + 13_000 = 117_000`
//!
//! The fault condition is: piece is EXPIRED (now >= end_time) AND
//! `now.saturating_sub(start) > fault_tolerance` (strictly greater-than).
//! - Piece still active (`now < end_time`) → ALWAYS adopted, no fault.
//! - Piece expired AND deficit == fault_tolerance → retire, no fault.
//! - Piece expired AND deficit == fault_tolerance + 1 → PieceStartInPast fault.
//!
//! These exact values are load-bearing: they were derived and validated
//! on hardware. Any refactor that changes the formula or inequality sense must
//! be explicitly confirmed with the user.
//!
//! Tests for the fault boundary use a SHORT piece (duration < fault_tolerance
//! cycles) so the piece is expired at `now = start + FAULT_TOLERANCE + 1`.
//! At 520MHz, FAULT_TOLERANCE = 117_000 cycles = 225µs; use duration = 100µs
//! (52_000 cycles). At `now = start + 117_001`, piece is expired (52_000 < 117_001)
//! and deficit (117_001) > fault_tolerance (117_000) → fault fires.

use std::cell::Cell;

use runtime::fault_sink::FaultSink;
use runtime::monomial::bernstein_to_monomial_with_duration;
use runtime::motion_core::get_position_and_velocity;
use runtime::piece_ring::{PieceEntry, RingDescriptor};

const CLOCK_FREQ: f32 = 520_000_000.0;
const TICK_CYCLES: u32 = 520_000_000_u32 / 40_000_u32;
const TICK_U64: u64 = TICK_CYCLES as u64;

const DRIFT_BUDGET: u64 = (200e-6_f32 * CLOCK_FREQ) as u64;
const FAULT_TOLERANCE: u64 = DRIFT_BUDGET + TICK_CYCLES as u64;

struct TestFaultSink {
    count: Cell<usize>,
}

impl TestFaultSink {
    fn new() -> Self {
        Self {
            count: Cell::new(0),
        }
    }
    fn fault_count(&self) -> usize {
        self.count.get()
    }
}

impl FaultSink for TestFaultSink {
    fn piece_start_in_past(&self, _axis_idx: usize, _deficit_us: u32) {
        self.count.set(self.count.get() + 1);
    }
}

fn make_entry(start: u64, coeffs: [f32; 4], duration: f32) -> PieceEntry {
    PieceEntry {
        start_time: start,
        coeffs,
        duration,
        _reserved: 0,
    }
}

fn empty_ring() -> RingDescriptor {
    RingDescriptor::new_unconfigured()
}

fn ring_with_one(entry: PieceEntry) -> (RingDescriptor, Vec<PieceEntry>) {
    let mut storage = vec![entry; 4];
    let mut ring = RingDescriptor::new(0, 4);
    ring.push(&mut storage, entry).expect("push must succeed");
    (ring, storage)
}

#[test]
fn walker_branch1_current_piece_eval() {
    let start = TICK_U64 * 100;
    let duration_s = 0.1_f32;
    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    let dur_cycles: u64 = (duration_s * CLOCK_FREQ) as u64;

    let entry = make_entry(start, [0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0], duration_s);
    let (mut ring, storage) = ring_with_one(entry);

    let fault = TestFaultSink::new();
    let mut armed = None;

    let res = get_position_and_velocity(
        &mut armed,
        &mut ring,
        &storage,
        start,
        TICK_CYCLES,
        CLOCK_FREQ,
        0,
        &fault,
    );
    assert!(res.is_some(), "first call at start must return Some");
    let (p0, _) = res.unwrap();
    assert!(
        p0.abs() < 1e-4,
        "P(0) must be 0.0 mm; got {p0}. c0=0 for this Bernstein piece."
    );
    assert_eq!(fault.fault_count(), 0, "no fault on valid arm");

    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    let t025_cycles: u64 = (0.025_f32 * CLOCK_FREQ) as u64;
    let now2 = start + t025_cycles;
    assert!(
        now2 < start + dur_cycles,
        "precondition: still inside piece window"
    );

    let res2 = get_position_and_velocity(
        &mut armed,
        &mut ring,
        &storage,
        now2,
        TICK_CYCLES,
        CLOCK_FREQ,
        0,
        &fault,
    );
    assert!(
        res2.is_some(),
        "branch 1: piece still live must return Some"
    );
    let (p2, v2) = res2.unwrap();

    let m = bernstein_to_monomial_with_duration([0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0], duration_s);
    let t = 0.025_f32;
    let p_analytic = m.coeffs[0] + t * (m.coeffs[1] + t * (m.coeffs[2] + t * m.coeffs[3]));
    let v_analytic = m.vel_coeffs[0] + t * (m.vel_coeffs[1] + t * m.vel_coeffs[2]);

    assert!(
        (p2 - p_analytic).abs() < 1e-4,
        "branch 1 position={p2}, analytic={p_analytic}. Difference must be < 1e-4 mm."
    );
    assert!(
        (v2 - v_analytic).abs() < 1e-2,
        "branch 1 velocity={v2}, analytic={v_analytic}. Difference must be < 0.01 mm/s."
    );
    assert_eq!(fault.fault_count(), 0, "no fault on live piece eval");

    let _ = storage;
}

#[test]
fn walker_branch2_empty_ring_returns_none() {
    let mut ring = empty_ring();
    let mut storage: Vec<PieceEntry> = Vec::new();
    let fault = TestFaultSink::new();
    let mut armed = None;

    let res = get_position_and_velocity(
        &mut armed,
        &mut ring,
        &mut storage,
        TICK_U64 * 10,
        TICK_CYCLES,
        CLOCK_FREQ,
        0,
        &fault,
    );
    assert!(res.is_none(), "empty ring must return None");
    assert_eq!(fault.fault_count(), 0, "empty ring must not fault");
}

#[test]
fn walker_branch2_configured_empty_ring_returns_none() {
    let mut storage = vec![
        PieceEntry {
            start_time: 0,
            coeffs: [0.0; 4],
            duration: 0.0,
            _reserved: 0
        };
        8
    ];
    let mut ring = RingDescriptor::new(0, 8);
    let fault = TestFaultSink::new();
    let mut armed = None;

    let res = get_position_and_velocity(
        &mut armed,
        &mut ring,
        &mut storage,
        TICK_U64 * 10,
        TICK_CYCLES,
        CLOCK_FREQ,
        0,
        &fault,
    );
    assert!(res.is_none(), "configured-but-empty ring must return None");
    assert_eq!(fault.fault_count(), 0);
}

// ── Branch 3: expired piece past fault_tolerance ─────────────────────────────
//
// Hardware: hard-faults PieceStartInPast, returns None, does not retire.
// Host (MACH_LINUX): silently retires, ring drains, returns None — no fault.

/// **Hardware only** — Push a short piece (100µs = 52_000 cycles at 520MHz)
/// with start_time = 1_000 cycles. Call walker at `now = 1_000 + FAULT_TOLERANCE + 1`.
///
/// At this `now`:
///   - Piece is EXPIRED: now (118_001) > end_time (start + 52_000 = 53_000).
///   - Deficit: 117_001 > fault_tolerance (117_000) → PieceStartInPast fires.
///
/// Expects:
///   - `None` returned
///   - `TestFaultSink::fault_count()` == 1
///   - `ring.retired_count()` unchanged (0) — the fault path does NOT retire
///
/// The last point is deliberately tested: the walker returns `None` without
/// calling `advance_counter`, so `retired` stays at 0. This matches the spec
/// (fault = hard stop, not a soft retire).
///
/// On the host build this test is skipped: expired pieces are silently retired
/// (not faulted) on MACH_LINUX to tolerate CFS scheduler jitter.
#[test]
#[cfg(not(feature = "host"))]
fn walker_branch3_past_piece_faults() {
    let start = 1_000_u64;
    // 100µs piece → 52_000 cycles at 520MHz. Expired before fault threshold.
    let entry = make_entry(start, [0.0; 4], 0.000_100);
    let (mut ring, mut storage) = ring_with_one(entry);

    let fault = TestFaultSink::new();
    let mut armed = None;

    let now = start + FAULT_TOLERANCE + 1;
    let res = get_position_and_velocity(
        &mut armed,
        &mut ring,
        &mut storage,
        now,
        TICK_CYCLES,
        CLOCK_FREQ,
        0,
        &fault,
    );

    assert!(res.is_none(), "branch 3: past-piece must return None");
    assert_eq!(
        fault.fault_count(),
        1,
        "branch 3: fault_count must be 1 after PieceStartInPast"
    );
    assert_eq!(
        ring.retired_count(),
        0,
        "branch 3: retired must NOT be incremented on a fault (hard-stop semantics)"
    );
}

/// **Host build** — Same scenario as `walker_branch3_past_piece_faults` but on
/// MACH_LINUX: an expired piece past fault_tolerance is silently retired (no
/// fault). The ring empties and the walker returns `None`.
#[test]
#[cfg(feature = "host")]
fn walker_branch3_host_expired_piece_silently_retired() {
    let start = 1_000_u64;
    // 100µs piece → 52_000 cycles at 520MHz. Expired at `now = start + 117_001`.
    let entry = make_entry(start, [0.0; 4], 0.000_100);
    let (mut ring, mut storage) = ring_with_one(entry);

    let fault = TestFaultSink::new();
    let mut armed = None;

    let now = start + FAULT_TOLERANCE + 1;
    let res = get_position_and_velocity(
        &mut armed,
        &mut ring,
        &mut storage,
        now,
        TICK_CYCLES,
        CLOCK_FREQ,
        0,
        &fault,
    );

    assert!(
        res.is_none(),
        "host: expired piece past tolerance must return None (ring empty after silent retire)"
    );
    assert_eq!(
        fault.fault_count(),
        0,
        "host: no fault must fire for expired piece — MACH_LINUX silently retires"
    );
    // The piece was retired: ring.retired_count() == 1.
    assert_eq!(
        ring.retired_count(),
        1,
        "host: retired_count must be 1 after silent retire of the expired piece"
    );
}

// ── Fault boundary invariant ──────────────────────────────────────────────────

/// `now - start == FAULT_TOLERANCE` is NOT a fault (strictly greater-than).
///
/// Tolerance at 520 MHz / 40 kHz:
///   drift_budget = 104_000 cycles (200 µs × 520 MHz)
///   fault_tolerance = 104_000 + 13_000 = 117_000 cycles
///
/// This is a load-bearing invariant: changing `>` to `>=` in the walker would
/// break late-arm near the boundary (a valid ISR behaviour when the ISR runs
/// slightly after the piece nominally starts). The boundary was derived and
/// validated on hardware.
#[test]
fn walker_fault_boundary_exact_is_not_a_fault() {
    let start = 1_000_u64;

    let entry = make_entry(start, [0.0; 4], 0.1);
    let (mut ring, mut storage) = ring_with_one(entry);
    let fault = TestFaultSink::new();
    let mut armed = None;

    let now = start + FAULT_TOLERANCE;
    let res = get_position_and_velocity(
        &mut armed,
        &mut ring,
        &mut storage,
        now,
        TICK_CYCLES,
        CLOCK_FREQ,
        0,
        &fault,
    );
    assert!(
        res.is_some(),
        "now - start == FAULT_TOLERANCE ({FAULT_TOLERANCE}) must NOT fault. \
         The condition is strictly-greater-than, not >=. got None (fault_count={})",
        fault.fault_count()
    );
    assert_eq!(
        fault.fault_count(),
        0,
        "no fault at exactly FAULT_TOLERANCE={FAULT_TOLERANCE} lateness (boundary is strictly greater-than)"
    );
}

/// **Hardware only** — `now - start == FAULT_TOLERANCE + 1` IS a fault when the
/// piece is expired.
///
/// This pins the upper side of the boundary: one cycle past the tolerance
/// must trigger the fault when now >= end_time.
///
/// Uses a short piece (100µs = 52_000 cycles at 520MHz) so end_time =
/// start + 52_000. At `now = start + FAULT_TOLERANCE + 1 = start + 117_001`:
///   - Piece is expired (117_001 > 52_000).
///   - Deficit 117_001 > fault_tolerance 117_000 → PieceStartInPast fires.
///
/// On the host build this test is skipped: expired pieces are silently retired
/// (not faulted) on MACH_LINUX regardless of deficit.
///
/// Tolerance at 520 MHz / 40 kHz: FAULT_TOLERANCE = 117_000 cycles.
#[test]
#[cfg(not(feature = "host"))]
fn walker_fault_boundary_plus_one_is_a_fault() {
    let start = 1_000_u64;

    // 100µs piece → 52_000 cycles at 520MHz. Expired well before fault threshold.
    let entry = make_entry(start, [0.0; 4], 0.000_100);
    let (mut ring, mut storage) = ring_with_one(entry);
    let fault = TestFaultSink::new();
    let mut armed = None;

    let now = start + FAULT_TOLERANCE + 1;
    let res = get_position_and_velocity(
        &mut armed,
        &mut ring,
        &mut storage,
        now,
        TICK_CYCLES,
        CLOCK_FREQ,
        0,
        &fault,
    );
    assert!(
        res.is_none(),
        "now - start == FAULT_TOLERANCE + 1 = {} must fault and return None; got Some",
        FAULT_TOLERANCE + 1
    );
    assert_eq!(
        fault.fault_count(),
        1,
        "fault_count must be 1 at FAULT_TOLERANCE+1 = {} lateness",
        FAULT_TOLERANCE + 1
    );
}

use proptest::prelude::*;

proptest! {
    #[test]
    fn proptest_contiguous_pieces_no_spurious_fault(
        n_pieces in 2usize..=8usize,
        duration_ms in 1u32..=50u32,
        target_mm in 0.5f32..=5.0f32,
    ) {
        let duration_s = duration_ms as f32 * 0.001_f32;
        #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
        let dur_cycles: u64 = (duration_s * CLOCK_FREQ) as u64;

        let mut storage_vec: Vec<PieceEntry> = Vec::with_capacity(n_pieces + 2);
        for _ in 0..n_pieces + 2 {
            storage_vec.push(PieceEntry {
                start_time: 0,
                coeffs: [0.0; 4],
                duration: 0.0,
                _reserved: 0,
            });
        }

        let mut ring = RingDescriptor::new(0, n_pieces);
        let base_start = TICK_U64 * 1_000;
        let mut prev_pos = 0.0_f32;

        for i in 0..n_pieces {
            #[allow(clippy::cast_possible_truncation)]
            let piece_start = base_start + i as u64 * dur_cycles;
            let offset = prev_pos;
            let entry = PieceEntry {
                start_time: piece_start,
                coeffs: [
                    offset,
                    offset + target_mm / 3.0,
                    offset + 2.0 * target_mm / 3.0,
                    offset + target_mm,
                ],
                duration: duration_s,
                _reserved: 0,
            };
            ring.push(&mut storage_vec, entry)
                .expect("ring must not be full while filling");
            prev_pos += target_mm;
        }

        let fault = TestFaultSink::new();
        let mut armed: Option<runtime::motion_core::ArmedPiece> = None;
        let mut last_p = -f32::INFINITY;

        let total_cycles = n_pieces as u64 * dur_cycles;
        let end = base_start + total_cycles + TICK_U64;

        let mut now = base_start;
        while now <= end {
            let res = get_position_and_velocity(
                &mut armed,
                &mut ring,
                &mut storage_vec,
                now,
                TICK_CYCLES,
                CLOCK_FREQ,
                0,
                &fault,
            );
            if let Some((p, _)) = res {
                prop_assert!(
                    p >= last_p - 1e-3,
                    "position decreased: p={p} < last_p={last_p} at now={now}"
                );
                last_p = p;
            }
            let fc = fault.fault_count();
            prop_assert!(
                fc == 0,
                "spurious fault (count={fc}) at now={now} during contiguous piece sequence"
            );
            now += TICK_U64;
        }
    }
}
