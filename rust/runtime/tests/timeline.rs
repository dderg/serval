//! Contract tests for [`runtime::timeline::Timeline`].
//!
//! Tests create `LoadedCubicCurve` structs on the stack and load them into
//! the Timeline via `test_load_axis_raw`. No CurvePool needed — the Timeline
//! just holds pointers into the test-owned curve data.

#![allow(unsafe_code, clippy::unwrap_used)]

use runtime::cubic_curve::LoadedCubicCurve;
use runtime::monomial::BezierPieceMonomial;
use runtime::timeline::Timeline;

const CLOCK_HZ: f32 = 520_000_000.0;

fn ms_to_cycles(ms: f32) -> u64 {
    (ms / 1_000.0 * CLOCK_HZ) as u64
}

const ZERO_PIECE: BezierPieceMonomial = BezierPieceMonomial {
    coeffs: [0.0; 4],
    vel_coeffs: [0.0; 3],
    duration: 0.0,
};

fn make_curve(pieces: &[BezierPieceMonomial]) -> LoadedCubicCurve {
    let mut curve = LoadedCubicCurve::empty();
    curve.piece_count = pieces.len() as u16;
    for (i, p) in pieces.iter().enumerate() {
        curve.pieces[i] = *p;
    }
    curve
}

fn linear_piece(duration_sec: f32) -> BezierPieceMonomial {
    let slope = 10.0 / duration_sec;
    BezierPieceMonomial {
        coeffs: [0.0, slope, 0.0, 0.0],
        vel_coeffs: [slope, 0.0, 0.0],
        duration: duration_sec,
    }
}

fn constant_piece(position: f32, duration_sec: f32) -> BezierPieceMonomial {
    BezierPieceMonomial {
        coeffs: [position, 0.0, 0.0, 0.0],
        vel_coeffs: [0.0, 0.0, 0.0],
        duration: duration_sec,
    }
}

/// Helper: load a curve onto axis 0 with a given start time.
fn load_axis0(tl: &mut Timeline, curve: &LoadedCubicCurve, start_ms: f32) {
    unsafe {
        tl.test_load_axis_raw(
            0,
            curve as *const _,
            curve.piece_count,
            ms_to_cycles(start_ms),
            CLOCK_HZ,
        );
    }
}

// ---------------------------------------------------------------------------
// Basic: returns piece and correct t_local
// ---------------------------------------------------------------------------

#[test]
fn returns_piece_and_t_local_within_piece() {
    let curve = make_curve(&[linear_piece(0.1)]); // 100ms
    let mut tl = Timeline::new(CLOCK_HZ);
    load_axis0(&mut tl, &curve, 0.0);

    let (_, t_local) = tl.get_piece(0, ms_to_cycles(50.0)).unwrap();
    assert!(
        (t_local - 0.05).abs() < 1e-4,
        "t_local = {t_local}, expected ≈ 0.05"
    );
}

// ---------------------------------------------------------------------------
// Advances to next piece when time passes
// ---------------------------------------------------------------------------

#[test]
fn advances_to_next_piece_when_time_passes() {
    let curve = make_curve(&[linear_piece(0.05), constant_piece(5.0, 0.05)]);
    let mut tl = Timeline::new(CLOCK_HZ);
    load_axis0(&mut tl, &curve, 0.0);

    let (piece, t_local) = tl.get_piece(0, ms_to_cycles(75.0)).unwrap();
    // Should be in piece 2 (constant at 5.0), t_local ≈ 25ms
    assert!(
        (t_local - 0.025).abs() < 1e-4,
        "t_local = {t_local}, expected ≈ 0.025"
    );
    assert!(
        (piece.coeffs[0] - 5.0).abs() < 0.01,
        "should be the constant piece (c0=5.0), got c0={}",
        piece.coeffs[0]
    );
}

// ---------------------------------------------------------------------------
// Returns None when no curve loaded
// ---------------------------------------------------------------------------

#[test]
fn returns_none_when_empty() {
    let mut tl = Timeline::new(CLOCK_HZ);
    assert!(tl.get_piece(0, ms_to_cycles(50.0)).is_none());
    assert!(tl.get_piece(1, ms_to_cycles(0.0)).is_none());
}

// ---------------------------------------------------------------------------
// Returns None after last piece
// ---------------------------------------------------------------------------

#[test]
fn returns_none_after_last_piece() {
    let curve = make_curve(&[linear_piece(0.1)]);
    let mut tl = Timeline::new(CLOCK_HZ);
    load_axis0(&mut tl, &curve, 0.0);

    assert!(tl.get_piece(0, ms_to_cycles(150.0)).is_none());
    assert!(!tl.axis_active(0), "axis should be idle after exhaustion");
}

// ---------------------------------------------------------------------------
// Skips sub-tick piece
// ---------------------------------------------------------------------------

#[test]
fn skips_piece_shorter_than_tick_period() {
    let curve = make_curve(&[
        constant_piece(0.0, 10e-6), // 10µs — shorter than one 40kHz tick
        linear_piece(0.1),           // 100ms
    ]);
    let mut tl = Timeline::new(CLOCK_HZ);
    load_axis0(&mut tl, &curve, 0.0);

    let (piece, _) = tl.get_piece(0, ms_to_cycles(50.0)).unwrap();
    assert!(
        (piece.duration - 0.1).abs() < 1e-6,
        "should skip the 10µs piece and return the 100ms one"
    );
}

// ---------------------------------------------------------------------------
// Consecutive calls same piece (cache hit)
// ---------------------------------------------------------------------------

#[test]
fn consecutive_calls_same_piece_no_advance() {
    let curve = make_curve(&[linear_piece(0.1)]);
    let mut tl = Timeline::new(CLOCK_HZ);
    load_axis0(&mut tl, &curve, 0.0);

    let (_, t1) = tl.get_piece(0, ms_to_cycles(25.0)).unwrap();
    let (_, t2) = tl.get_piece(0, ms_to_cycles(26.0)).unwrap();

    assert!((t1 - 0.025).abs() < 1e-4);
    assert!((t2 - 0.026).abs() < 1e-4);
    assert!((t2 - t1 - 0.001).abs() < 1e-4, "delta should be ~1ms");
}

// ---------------------------------------------------------------------------
// Multi-axis independence
// ---------------------------------------------------------------------------

#[test]
fn multi_axis_independence() {
    let curve0 = make_curve(&[linear_piece(0.05), linear_piece(0.05)]);
    let curve1 = make_curve(&[constant_piece(0.0, 0.1)]);
    let mut tl = Timeline::new(CLOCK_HZ);

    load_axis0(&mut tl, &curve0, 0.0);
    unsafe {
        tl.test_load_axis_raw(1, &curve1 as *const _, curve1.piece_count, 0, CLOCK_HZ);
    }

    // Advance axis 0 past its first piece
    let (_, t0) = tl.get_piece(0, ms_to_cycles(75.0)).unwrap();
    assert!((t0 - 0.025).abs() < 1e-4, "axis 0 in piece 2");

    // Axis 1 should be unaffected
    let (_, t1) = tl.get_piece(1, ms_to_cycles(30.0)).unwrap();
    assert!((t1 - 0.030).abs() < 1e-4, "axis 1 unaffected");
}

// ---------------------------------------------------------------------------
// Cursor recovery after exhaustion (reload)
// ---------------------------------------------------------------------------

#[test]
fn cursor_recovers_after_reload() {
    let curve1 = make_curve(&[linear_piece(0.1)]);
    let mut tl = Timeline::new(CLOCK_HZ);
    load_axis0(&mut tl, &curve1, 0.0);

    // Exhaust
    assert!(tl.get_piece(0, ms_to_cycles(150.0)).is_none());
    assert!(!tl.axis_active(0));

    // Reload with a new curve starting at 200ms
    let curve2 = make_curve(&[linear_piece(0.1)]);
    unsafe {
        tl.test_load_axis_raw(0, &curve2, curve2.piece_count, ms_to_cycles(200.0), CLOCK_HZ);
    }

    let (_, t_local) = tl.get_piece(0, ms_to_cycles(250.0)).unwrap();
    assert!(
        (t_local - 0.05).abs() < 1e-4,
        "reloaded curve should work, got t_local={t_local}"
    );
}

// ---------------------------------------------------------------------------
// Reset clears all axes
// ---------------------------------------------------------------------------

#[test]
fn reset_clears_all_axes() {
    let curve = make_curve(&[linear_piece(0.1)]);
    let mut tl = Timeline::new(CLOCK_HZ);
    load_axis0(&mut tl, &curve, 0.0);

    assert!(tl.axis_active(0));
    tl.reset();
    assert!(!tl.axis_active(0));
    assert!(tl.all_idle());
    assert!(tl.get_piece(0, ms_to_cycles(50.0)).is_none());
}

// ---------------------------------------------------------------------------
// Monotonic sweep (ISR simulation)
// ---------------------------------------------------------------------------

#[test]
fn monotonic_sweep_no_gaps() {
    let curve = make_curve(&[
        linear_piece(0.025),
        linear_piece(0.025),
        linear_piece(0.025),
        linear_piece(0.025),
    ]); // 4 pieces × 25ms = 100ms
    let mut tl = Timeline::new(CLOCK_HZ);
    load_axis0(&mut tl, &curve, 0.0);

    let tick_cycles = (CLOCK_HZ / 40_000.0) as u64; // 13000 cycles per tick
    let end = ms_to_cycles(99.0);
    let mut now: u64 = 0;
    let mut ticks_with_piece = 0u32;

    while now < end {
        match tl.get_piece(0, now) {
            Some((_, t_local)) => {
                assert!(
                    t_local >= 0.0 && t_local <= 0.026,
                    "t_local out of range: {t_local} at now={now}"
                );
                ticks_with_piece += 1;
            }
            None => {
                panic!("got None at now={now} cycles ({}ms) — gap in timeline", now as f32 / CLOCK_HZ * 1000.0);
            }
        }
        now += tick_cycles;
    }

    assert!(
        ticks_with_piece > 3900,
        "expected ~3960 ticks with pieces, got {ticks_with_piece}"
    );
}
