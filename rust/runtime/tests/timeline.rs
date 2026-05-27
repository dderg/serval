//! Contract tests for the Timeline.

#![allow(clippy::unwrap_used)]

use runtime::monomial::BezierPieceMonomial;
use runtime::timeline::{GetPieceResult, Timeline};

const CLOCK_HZ: f32 = 520_000_000.0;

fn ms_to_cycles(ms: f32) -> u64 {
    (ms / 1_000.0 * CLOCK_HZ) as u64
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

/// Helper: extract (&piece, t_local) from a Hit result, panic otherwise.
fn unwrap_hit<'a>(result: GetPieceResult<'a>) -> (&'a BezierPieceMonomial, f32) {
    match result {
        GetPieceResult::Hit(p, t) => (p, t),
        GetPieceResult::NeedsAdvance => panic!("expected Hit, got NeedsAdvance"),
        GetPieceResult::Idle => panic!("expected Hit, got Idle"),
    }
}

/// Helper: get_piece with auto-advance for tests. Simulates what the ISR
/// loop does: try get_piece, if NeedsAdvance call test_advance_piece.
fn get_or_advance<'a>(
    tl: &'a mut Timeline,
    axis: usize,
    now: u64,
    pieces: &[BezierPieceMonomial],
) -> Option<(&'a BezierPieceMonomial, f32)> {
    // Two-step borrow: try get_piece first, then advance if needed.
    // Can't hold the reference across the advance call, so we check
    // the discriminant first.
    let needs_advance = matches!(tl.get_piece(axis, now), GetPieceResult::NeedsAdvance);
    if needs_advance {
        match tl.test_advance_piece(axis, now, pieces) {
            GetPieceResult::Hit(p, t) => Some((p, t)),
            _ => None,
        }
    } else {
        match tl.get_piece(axis, now) {
            GetPieceResult::Hit(p, t) => Some((p, t)),
            _ => None,
        }
    }
}

// ---------------------------------------------------------------------------

#[test]
fn returns_piece_and_t_local_within_piece() {
    let pieces = [linear_piece(0.1)];
    let mut tl = Timeline::new(CLOCK_HZ);
    tl.test_load_pieces(0, &pieces, 0);

    let (_, t_local) = unwrap_hit(tl.get_piece(0, ms_to_cycles(50.0)));
    assert!(
        (t_local - 0.05).abs() < 1e-4,
        "t_local = {t_local}, expected ≈ 0.05"
    );
}

#[test]
fn advances_to_next_piece_when_time_passes() {
    let pieces = [linear_piece(0.05), constant_piece(5.0, 0.05)];
    let mut tl = Timeline::new(CLOCK_HZ);
    tl.test_load_pieces(0, &pieces, 0);

    let (piece, t_local) = get_or_advance(&mut tl, 0, ms_to_cycles(75.0), &pieces).unwrap();
    assert!(
        (t_local - 0.025).abs() < 1e-4,
        "t_local = {t_local}, expected ≈ 0.025"
    );
    assert!(
        (piece.coeffs[0] - 5.0).abs() < 0.01,
        "should be the constant piece"
    );
}

#[test]
fn returns_idle_when_empty() {
    let mut tl = Timeline::new(CLOCK_HZ);
    assert!(matches!(tl.get_piece(0, ms_to_cycles(50.0)), GetPieceResult::Idle));
}

#[test]
fn returns_idle_after_last_piece() {
    let pieces = [linear_piece(0.1)];
    let mut tl = Timeline::new(CLOCK_HZ);
    tl.test_load_pieces(0, &pieces, 0);

    // Advance past the end
    let result = tl.get_piece(0, ms_to_cycles(50.0)); // within piece
    assert!(matches!(result, GetPieceResult::Hit(..)));

    // Now past the end — get_piece returns NeedsAdvance, advance returns Idle
    match tl.get_piece(0, ms_to_cycles(150.0)) {
        GetPieceResult::NeedsAdvance => {
            let result = tl.test_advance_piece(0, ms_to_cycles(150.0), &pieces);
            assert!(matches!(result, GetPieceResult::Idle));
        }
        other => panic!("expected NeedsAdvance, got {other:?}"),
    }
    assert!(!tl.axis_active(0));
}

#[test]
fn skips_piece_shorter_than_tick_period() {
    let pieces = [
        constant_piece(0.0, 10e-6), // 10µs — shorter than one 40kHz tick
        linear_piece(0.1),
    ];
    let mut tl = Timeline::new(CLOCK_HZ);
    tl.test_load_pieces(0, &pieces, 0);

    let (piece, _) = get_or_advance(&mut tl, 0, ms_to_cycles(50.0), &pieces).unwrap();
    assert!(
        (piece.duration - 0.1).abs() < 1e-6,
        "should skip the 10µs piece and return the 100ms one"
    );
}

#[test]
fn consecutive_calls_same_piece_no_advance() {
    let pieces = [linear_piece(0.1)];
    let mut tl = Timeline::new(CLOCK_HZ);
    tl.test_load_pieces(0, &pieces, 0);

    let (_, t1) = unwrap_hit(tl.get_piece(0, ms_to_cycles(25.0)));
    let (_, t2) = unwrap_hit(tl.get_piece(0, ms_to_cycles(26.0)));

    assert!((t1 - 0.025).abs() < 1e-4);
    assert!((t2 - 0.026).abs() < 1e-4);
    assert!((t2 - t1 - 0.001).abs() < 1e-4, "delta should be ~1ms");
}

#[test]
fn multi_axis_independence() {
    let pieces0 = [linear_piece(0.05), linear_piece(0.05)];
    let pieces1 = [constant_piece(0.0, 0.1)];
    let mut tl = Timeline::new(CLOCK_HZ);
    tl.test_load_pieces(0, &pieces0, 0);
    tl.test_load_pieces(1, &pieces1, 0);

    let (_, t0) = get_or_advance(&mut tl, 0, ms_to_cycles(75.0), &pieces0).unwrap();
    assert!((t0 - 0.025).abs() < 1e-4, "axis 0 in piece 2");

    let (_, t1) = unwrap_hit(tl.get_piece(1, ms_to_cycles(30.0)));
    assert!((t1 - 0.030).abs() < 1e-4, "axis 1 unaffected");
}

#[test]
fn cursor_recovers_after_reload() {
    let pieces1 = [linear_piece(0.1)];
    let mut tl = Timeline::new(CLOCK_HZ);
    tl.test_load_pieces(0, &pieces1, 0);

    // Exhaust via advance
    let _ = get_or_advance(&mut tl, 0, ms_to_cycles(150.0), &pieces1);
    assert!(!tl.axis_active(0));

    // Reload
    let pieces2 = [linear_piece(0.1)];
    tl.test_load_pieces(0, &pieces2, ms_to_cycles(200.0));

    let (_, t_local) = unwrap_hit(tl.get_piece(0, ms_to_cycles(250.0)));
    assert!(
        (t_local - 0.05).abs() < 1e-4,
        "reloaded curve should work"
    );
}

#[test]
fn reset_clears_all_axes() {
    let pieces = [linear_piece(0.1)];
    let mut tl = Timeline::new(CLOCK_HZ);
    tl.test_load_pieces(0, &pieces, 0);

    assert!(tl.axis_active(0));
    tl.reset();
    assert!(!tl.axis_active(0));
    assert!(tl.all_idle());
    assert!(matches!(tl.get_piece(0, ms_to_cycles(50.0)), GetPieceResult::Idle));
}

#[test]
fn monotonic_sweep_no_gaps() {
    let pieces = [
        linear_piece(0.025),
        linear_piece(0.025),
        linear_piece(0.025),
        linear_piece(0.025),
    ];
    let mut tl = Timeline::new(CLOCK_HZ);
    tl.test_load_pieces(0, &pieces, 0);

    let tick_cycles = (CLOCK_HZ / 40_000.0) as u64;
    let end = ms_to_cycles(99.0);
    let mut now: u64 = 0;
    let mut ticks_with_piece = 0u32;

    while now < end {
        let result = tl.get_piece(0, now);
        match result {
            GetPieceResult::Hit(_, t_local) => {
                assert!(
                    t_local >= 0.0 && t_local <= 0.026,
                    "t_local out of range: {t_local} at now={now}"
                );
                ticks_with_piece += 1;
            }
            GetPieceResult::NeedsAdvance => {
                match tl.test_advance_piece(0, now, &pieces) {
                    GetPieceResult::Hit(_, t_local) => {
                        assert!(t_local >= 0.0 && t_local <= 0.026);
                        ticks_with_piece += 1;
                    }
                    _ => panic!("advance returned non-Hit at now={now}"),
                }
            }
            GetPieceResult::Idle => {
                panic!("got Idle at now={now} — gap in timeline");
            }
        }
        now += tick_cycles;
    }

    assert!(
        ticks_with_piece > 3900,
        "expected ~3960 ticks with pieces, got {ticks_with_piece}"
    );
}
