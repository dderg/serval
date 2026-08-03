use super::{DRAIN_RESERVE_FLOOR_S, DRAIN_RESERVE_SAFETY, DrainReserve};

/// Slowest brake-to-rest traversal the bench has produced: the pacer fired
/// `Drain` with 100 ms of committed runway left and the tail reached the
/// anchor 43.4 ms after the playhead had already passed the committed end
/// (host-rust.jsonl 2026-08-02T13:19:10.724Z, `seg_t_start=2926.848305`, at
/// the last segment of cube_motion_only.gcode; the same print shape did it
/// again at T08:34:37.213Z, 12.5 ms over). A reserve that does not cover
/// this is a print abort, not a stutter.
const BENCH_WORST_DRAIN_S: f64 = 0.1434;

#[test]
fn the_floor_covers_the_worst_drain_the_bench_has_produced() {
    assert!(
        DrainReserve::new().secs() > BENCH_WORST_DRAIN_S,
        "a session's first drain has no measurement behind it, so the floor \
         alone has to beat the pipeline: {} <= {BENCH_WORST_DRAIN_S}",
        DrainReserve::new().secs()
    );
}

#[test]
fn a_fresh_reserve_is_the_floor() {
    assert!((DrainReserve::new().secs() - DRAIN_RESERVE_FLOOR_S).abs() < 1e-12);
}

#[test]
fn drains_inside_the_floor_leave_the_reserve_alone() {
    let mut r = DrainReserve::new();
    let inside = DRAIN_RESERVE_FLOOR_S / DRAIN_RESERVE_SAFETY * 0.5;
    assert!(
        r.observe(inside),
        "the first sample is always the worst yet"
    );
    assert!((r.secs() - DRAIN_RESERVE_FLOOR_S).abs() < 1e-12);
}

#[test]
fn a_slow_drain_widens_the_reserve_to_cover_it_again() {
    let mut r = DrainReserve::new();
    let slow = DRAIN_RESERVE_FLOOR_S * 1.5;
    assert!(r.observe(slow));
    assert!(
        r.secs() >= slow,
        "a reserve that does not cover the traversal it just measured would \
         let the same drain overrun again: {} < {slow}",
        r.secs()
    );
    assert!((r.secs() - slow * DRAIN_RESERVE_SAFETY).abs() < 1e-12);
}

#[test]
fn a_faster_drain_never_shrinks_the_reserve() {
    let mut r = DrainReserve::new();
    let slow = DRAIN_RESERVE_FLOOR_S * 1.5;
    r.observe(slow);
    let widened = r.secs();
    assert!(
        !r.observe(slow / 10.0),
        "a faster drain is not new evidence"
    );
    assert!((r.secs() - widened).abs() < 1e-12);
}
