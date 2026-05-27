//! Contract tests for [`runtime::timeline::Timeline`].
//!
//! Each test specifies a scenario via a flat list of [`TimedPiece`]s, drives
//! [`Timeline::get_piece`] at a particular clock value, and asserts on the
//! returned piece reference and `t_local` value.
//!
//! # H7 clock constants
//!
//! All timing arithmetic uses the STM32H723 default:
//! `CLOCK_HZ = 520_000_000` cycles / second.

use runtime::monomial::BezierPieceMonomial;
use runtime::timeline::{TimedPiece, Timeline};

/// H7 MCU clock frequency in Hz.
const CLOCK_HZ: u32 = 520_000_000;

/// Reciprocal of CLOCK_HZ for multiply-based t_local conversion.
const INV_CLOCK_HZ: f32 = 1.0 / 520_000_000.0;

/// Convert a duration in milliseconds to CPU cycles (u64).
fn ms_to_cycles(ms: f32) -> u64 {
    (ms / 1_000.0 * CLOCK_HZ as f32) as u64
}

/// Build a constant piece (zero velocity) for use where the coefficients are
/// not under test.
fn dummy_piece(duration_sec: f32) -> BezierPieceMonomial {
    BezierPieceMonomial {
        coeffs: [0.0, 0.0, 0.0, 0.0],
        vel_coeffs: [0.0, 0.0, 0.0],
        duration: duration_sec,
    }
}

/// Build a linear piece: P(t) = 10·t/duration (position 0..10 mm).
fn linear_piece(duration_sec: f32) -> BezierPieceMonomial {
    // Monomial: c0=0, c1=10/duration, c2=0, c3=0
    let slope = 10.0 / duration_sec;
    BezierPieceMonomial {
        coeffs: [0.0, slope, 0.0, 0.0],
        vel_coeffs: [slope, 0.0, 0.0],
        duration: duration_sec,
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn single_piece_timeline(piece: BezierPieceMonomial, start_ms: f32, end_ms: f32) -> Timeline {
    let mut tl = Timeline::new(INV_CLOCK_HZ);
    let tp = TimedPiece {
        piece,
        start_cycles: ms_to_cycles(start_ms),
        end_cycles: ms_to_cycles(end_ms),
    };
    tl.push_piece(0, tp).expect("push must succeed on empty timeline");
    tl
}

// ---------------------------------------------------------------------------
// Test: returns correct piece and t_local within a single piece
// ---------------------------------------------------------------------------

/// One piece: linear 0→10 mm, 100 ms duration.
/// Query at now = 50 ms in cycles.
/// Expected: returns the piece, t_local ≈ 0.05 s.
#[test]
fn returns_piece_and_t_local_within_piece() {
    let piece = linear_piece(0.1);
    let mut tl = single_piece_timeline(piece, 0.0, 100.0);

    let now = ms_to_cycles(50.0);
    let result = tl.get_piece(0, now);

    assert!(result.is_some(), "should return Some for now inside the piece");
    let (_, t_local) = result.unwrap();
    let expected = 0.05_f32;
    assert!(
        (t_local - expected).abs() < 1e-4,
        "t_local = {t_local}, expected ≈ {expected}"
    );
}

// ---------------------------------------------------------------------------
// Test: advances to the next piece when time passes the first
// ---------------------------------------------------------------------------

/// Two pieces: piece_a covers [0, 50 ms), piece_b covers [50, 100 ms).
/// Query at now = 75 ms.
/// Expected: returns piece_b, t_local ≈ 0.025 s.
#[test]
fn advances_to_next_piece_when_time_passes() {
    let piece_a = linear_piece(0.05);
    let piece_b = linear_piece(0.05);

    let mut tl = Timeline::new(INV_CLOCK_HZ);
    tl.push_piece(
        0,
        TimedPiece {
            piece: piece_a,
            start_cycles: ms_to_cycles(0.0),
            end_cycles: ms_to_cycles(50.0),
        },
    )
    .expect("push a");
    tl.push_piece(
        0,
        TimedPiece {
            piece: piece_b,
            start_cycles: ms_to_cycles(50.0),
            end_cycles: ms_to_cycles(100.0),
        },
    )
    .expect("push b");

    let now = ms_to_cycles(75.0);
    let result = tl.get_piece(0, now);

    assert!(result.is_some(), "should return Some for now in piece_b");
    let (returned_piece, t_local) = result.unwrap();

    // piece_b has slope = 10 / 0.05 = 200.0; piece_a has slope = 10 / 0.05 = 200.0 too.
    // Distinguish by checking t_local ≈ 0.025 s (not 0.075 s which would be from start_a).
    let expected_t = 0.025_f32;
    assert!(
        (t_local - expected_t).abs() < 1e-4,
        "t_local = {t_local}, expected ≈ {expected_t} (piece_b local time)"
    );

    // The returned piece's slope should match piece_b.
    let slope_b = 10.0_f32 / 0.05;
    assert!(
        (returned_piece.coeffs[1] - slope_b).abs() < 1.0,
        "returned piece c1 = {}, expected slope_b = {slope_b}",
        returned_piece.coeffs[1]
    );
}

// ---------------------------------------------------------------------------
// Test: returns None for empty timeline
// ---------------------------------------------------------------------------

#[test]
fn returns_none_when_empty() {
    let mut tl = Timeline::new(INV_CLOCK_HZ);
    assert!(tl.get_piece(0, ms_to_cycles(50.0)).is_none());
    assert!(tl.get_piece(1, ms_to_cycles(0.0)).is_none());
}

// ---------------------------------------------------------------------------
// Test: returns None after the last piece has expired
// ---------------------------------------------------------------------------

#[test]
fn returns_none_after_last_piece() {
    let piece = dummy_piece(0.1);
    let mut tl = single_piece_timeline(piece, 0.0, 100.0);

    // 100 ms is the end; 150 ms is past it.
    let result = tl.get_piece(0, ms_to_cycles(150.0));
    assert!(result.is_none(), "should return None past the last piece");
}

// ---------------------------------------------------------------------------
// Test: skips a piece whose duration is shorter than one tick period
// ---------------------------------------------------------------------------

/// Piece 1: 10 µs (shorter than one 40 kHz tick ≈ 25 µs at 520 MHz).
/// Piece 2: covers [10 µs, 100 ms).
/// Query at 50 ms → should return piece 2, not piece 1.
#[test]
fn skips_piece_shorter_than_tick_period() {
    let piece1 = dummy_piece(10e-6_f32);
    let piece2 = linear_piece(0.1_f32);

    // Piece 1: [0, 10 µs)
    let start1 = 0_u64;
    let end1 = (10e-6_f32 * CLOCK_HZ as f32) as u64;
    // Piece 2: [10 µs, 10 µs + 100 ms)
    let start2 = end1;
    let end2 = start2 + ms_to_cycles(100.0);

    let mut tl = Timeline::new(INV_CLOCK_HZ);
    tl.push_piece(0, TimedPiece { piece: piece1, start_cycles: start1, end_cycles: end1 })
        .expect("push piece1");
    tl.push_piece(0, TimedPiece { piece: piece2, start_cycles: start2, end_cycles: end2 })
        .expect("push piece2");

    let now = ms_to_cycles(50.0);
    let result = tl.get_piece(0, now);

    assert!(result.is_some(), "should return Some for now in piece2");
    let (returned_piece, t_local) = result.unwrap();

    // t_local for piece2 at 50 ms: now - start2 in seconds.
    let start2_sec = end1 as f32 * INV_CLOCK_HZ;
    let expected_t = 0.05_f32 - start2_sec; // ≈ 0.05 s (10 µs is negligible)
    assert!(
        (t_local - expected_t).abs() < 1e-3,
        "t_local = {t_local}, expected ≈ {expected_t}"
    );

    // The returned piece's duration should match piece2 (0.1 s), not piece1 (10 µs).
    assert!(
        (returned_piece.duration - 0.1_f32).abs() < 1e-6,
        "returned duration {}, expected 0.1", returned_piece.duration
    );
}

// ---------------------------------------------------------------------------
// Test: consecutive calls to the same piece use the cache (no advancement)
// ---------------------------------------------------------------------------

/// One piece: 0..100 ms.
/// Call at 25 ms, then at 26 ms.
/// Both should return the same piece; t_local must differ accordingly.
#[test]
fn consecutive_calls_same_piece_no_advance() {
    let piece = linear_piece(0.1);
    let mut tl = single_piece_timeline(piece, 0.0, 100.0);

    let now1 = ms_to_cycles(25.0);
    let now2 = ms_to_cycles(26.0);

    let (_, t1) = tl
        .get_piece(0, now1)
        .expect("call 1 should return Some");
    let (_, t2) = tl
        .get_piece(0, now2)
        .expect("call 2 should return Some");

    let expected_t1 = 0.025_f32;
    let expected_t2 = 0.026_f32;

    assert!(
        (t1 - expected_t1).abs() < 1e-4,
        "first call t_local = {t1}, expected ≈ {expected_t1}"
    );
    assert!(
        (t2 - expected_t2).abs() < 1e-4,
        "second call t_local = {t2}, expected ≈ {expected_t2}"
    );

    // Verify t2 > t1 and the difference is ≈ 1 ms.
    let dt = t2 - t1;
    let expected_dt = 0.001_f32;
    assert!(
        (dt - expected_dt).abs() < 1e-4,
        "Δt_local = {dt}, expected ≈ {expected_dt}"
    );
}

// ---------------------------------------------------------------------------
// Test: advances across segment boundaries (two independent segments)
// ---------------------------------------------------------------------------

/// Segment 1: one piece [0, 100 ms).
/// Segment 2: one piece [100 ms, 200 ms).
/// Query at 150 ms → returns segment 2's piece, t_local ≈ 0.05 s.
///
/// This uses axis 0 for both segments (they are queued consecutively) to
/// test that the timeline correctly crosses what would be a segment boundary.
#[test]
fn advances_across_segments() {
    let piece1 = linear_piece(0.1);
    let piece2 = linear_piece(0.1);

    let mut tl = Timeline::new(INV_CLOCK_HZ);
    tl.push_piece(
        0,
        TimedPiece {
            piece: piece1,
            start_cycles: ms_to_cycles(0.0),
            end_cycles: ms_to_cycles(100.0),
        },
    )
    .expect("push seg1 piece");
    tl.push_piece(
        0,
        TimedPiece {
            piece: piece2,
            start_cycles: ms_to_cycles(100.0),
            end_cycles: ms_to_cycles(200.0),
        },
    )
    .expect("push seg2 piece");

    let now = ms_to_cycles(150.0);
    let result = tl.get_piece(0, now);

    assert!(result.is_some(), "should return Some at 150 ms");
    let (_, t_local) = result.unwrap();

    let expected_t = 0.05_f32; // 150 ms - 100 ms = 50 ms = 0.05 s
    assert!(
        (t_local - expected_t).abs() < 1e-4,
        "t_local = {t_local}, expected ≈ {expected_t}"
    );
}

// ---------------------------------------------------------------------------
// Test: advancing one axis does not affect another
// ---------------------------------------------------------------------------

#[test]
fn multi_axis_independence() {
    let mut tl = Timeline::new(INV_CLOCK_HZ);

    // Axis 0: two pieces, 0..50ms and 50..100ms
    tl.push_piece(0, TimedPiece {
        piece: linear_piece(0.05),
        start_cycles: ms_to_cycles(0.0),
        end_cycles: ms_to_cycles(50.0),
    }).unwrap();
    tl.push_piece(0, TimedPiece {
        piece: linear_piece(0.05),
        start_cycles: ms_to_cycles(50.0),
        end_cycles: ms_to_cycles(100.0),
    }).unwrap();

    // Axis 1: one piece, 0..100ms
    tl.push_piece(1, TimedPiece {
        piece: dummy_piece(0.1),
        start_cycles: ms_to_cycles(0.0),
        end_cycles: ms_to_cycles(100.0),
    }).unwrap();

    // Query axis 0 at 75ms — should advance past piece 0 into piece 1
    let (_, t0) = tl.get_piece(0, ms_to_cycles(75.0)).unwrap();
    assert!((t0 - 0.025).abs() < 1e-4, "axis 0 t_local should be 25ms into piece 1");

    // Query axis 1 at 30ms — should still be in its only piece, unaffected
    let (_, t1) = tl.get_piece(1, ms_to_cycles(30.0)).unwrap();
    assert!((t1 - 0.030).abs() < 1e-4, "axis 1 t_local should be 30ms, not affected by axis 0 advance");
}

// ---------------------------------------------------------------------------
// Test: cursor recovers after all pieces exhausted and new piece pushed
// ---------------------------------------------------------------------------

#[test]
fn cursor_recovers_after_exhaustion_then_push() {
    let mut tl = Timeline::new(INV_CLOCK_HZ);

    tl.push_piece(0, TimedPiece {
        piece: dummy_piece(0.1),
        start_cycles: ms_to_cycles(0.0),
        end_cycles: ms_to_cycles(100.0),
    }).unwrap();

    // Exhaust: query past the end
    assert!(tl.get_piece(0, ms_to_cycles(150.0)).is_none());

    // Push a new piece starting at 200ms
    tl.push_piece(0, TimedPiece {
        piece: linear_piece(0.1),
        start_cycles: ms_to_cycles(200.0),
        end_cycles: ms_to_cycles(300.0),
    }).unwrap();

    // Query at 250ms — should find the new piece
    let result = tl.get_piece(0, ms_to_cycles(250.0));
    assert!(result.is_some(), "should find newly pushed piece after exhaustion");
    let (_, t_local) = result.unwrap();
    assert!(
        (t_local - 0.05).abs() < 1e-4,
        "t_local should be 50ms into the new piece, got {t_local}"
    );
}

// ---------------------------------------------------------------------------
// Test: push_piece returns Err when queue is full
// ---------------------------------------------------------------------------

#[test]
fn push_piece_returns_err_when_full() {
    let mut tl = Timeline::new(INV_CLOCK_HZ);

    // Fill all 16 slots
    for i in 0..16u64 {
        let start = i * ms_to_cycles(10.0);
        let end = (i + 1) * ms_to_cycles(10.0);
        assert!(
            tl.push_piece(0, TimedPiece {
                piece: dummy_piece(0.01),
                start_cycles: start,
                end_cycles: end,
            }).is_ok(),
            "push {i} should succeed"
        );
    }

    // 17th push should fail
    assert!(
        tl.push_piece(0, TimedPiece {
            piece: dummy_piece(0.01),
            start_cycles: ms_to_cycles(160.0),
            end_cycles: ms_to_cycles(170.0),
        }).is_err(),
        "17th push must return Err — queue capacity is 16"
    );
}

// ---------------------------------------------------------------------------
// Test: monotonic sweep through multiple pieces (ISR simulation)
// ---------------------------------------------------------------------------

#[test]
fn monotonic_sweep_no_gaps() {
    let mut tl = Timeline::new(INV_CLOCK_HZ);

    // 4 pieces, each 25ms, covering 0..100ms total
    for i in 0..4u64 {
        let start = i * ms_to_cycles(25.0);
        let end = (i + 1) * ms_to_cycles(25.0);
        tl.push_piece(0, TimedPiece {
            piece: linear_piece(0.025),
            start_cycles: start,
            end_cycles: end,
        }).unwrap();
    }

    // Simulate ISR: tick every 25µs (one sample period at 40kHz) from 0 to 99ms
    let tick_cycles = (CLOCK_HZ / 40_000) as u64; // 13000 cycles per tick
    let end_cycles = ms_to_cycles(99.0);
    let mut now: u64 = 0;
    let mut ticks_with_piece = 0u32;
    let mut ticks_none = 0u32;

    while now < end_cycles {
        match tl.get_piece(0, now) {
            Some((_, t_local)) => {
                // t_local must be non-negative and within the piece's duration
                assert!(
                    t_local >= 0.0 && t_local <= 0.026, // 25ms + small tolerance
                    "t_local out of range: {t_local} at now={now}"
                );
                ticks_with_piece += 1;
            }
            None => {
                ticks_none += 1;
            }
        }
        now += tick_cycles;
    }

    assert!(
        ticks_with_piece > 3900,
        "most ticks should return a piece, got {ticks_with_piece} with piece, {ticks_none} None"
    );
    assert_eq!(
        ticks_none, 0,
        "no gaps should exist within the 0..99ms window"
    );
}
