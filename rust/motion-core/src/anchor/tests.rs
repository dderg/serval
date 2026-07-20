use super::*;

#[test]
fn first_segment_lands_lead_ahead() {
    let mut a = Anchor::new();
    let (t0, epoch) = a.anchor_segment(0.0, 1.0, 100.0);
    assert_eq!(epoch, StreamEpoch::Reposition);
    assert!((t0 + 0.0 - (100.0 + DEFAULT_LEAD_SECS)).abs() < 1e-9);
}

#[test]
fn contiguous_segment_keeps_t0() {
    let mut a = Anchor::new();
    let (t0_a, _) = a.anchor_segment(0.0, 1.0, 100.0);
    let (t0_b, epoch) = a.anchor_segment(1.0, 2.0, 100.9);
    assert_eq!(epoch, StreamEpoch::Continuation);
    assert_eq!(t0_a, t0_b);
}

#[test]
fn underrun_reanchors_forward_instead_of_aborting() {
    let mut a = Anchor::new();
    let (t0_first, _) = a.anchor_segment(0.0, 1.0, 100.0);
    // Playhead (104.0) has overrun the scheduled start (t0_first + 1.0): a
    // genuine underrun. The anchor must re-anchor forward, not fail.
    let (t0_new, epoch) = a.anchor_segment(1.0, 2.0, 104.0);
    assert_eq!(epoch, StreamEpoch::Reanchor, "underrun must re-anchor");
    assert_ne!(t0_first, t0_new);
    // The recovered segment lands a lead ahead of the current playhead.
    assert!(
        (t0_new + 1.0 - (104.0 + DEFAULT_LEAD_SECS)).abs() < 1e-9,
        "t0_new={t0_new}"
    );
}
#[test]
fn thin_margin_continuation_reanchors_instead_of_racing_transport() {
    let mut a = Anchor::new();
    let (t0_first, _) = a.anchor_segment(0.0, 1.0, 100.0);
    // Post-homing seam on the bench: the segment start is still (barely)
    // ahead of the playhead, but closer than transport latency can cover —
    // continuing here latched a -308 PieceStartInPast at the drive.
    let host_now = t0_first + 1.0 - 0.5 * LOW_MARGIN_WARN_SECS;
    let (t0_new, epoch) = a.anchor_segment(1.0, 2.0, host_now);
    assert_eq!(epoch, StreamEpoch::Reanchor, "thin margin must re-anchor");
    assert!(
        (t0_new + 1.0 - (host_now + DEFAULT_LEAD_SECS)).abs() < 1e-9,
        "t0_new={t0_new}"
    );
}

#[test]
fn margin_above_the_floor_stays_a_continuation() {
    let mut a = Anchor::new();
    let (t0_first, _) = a.anchor_segment(0.0, 1.0, 100.0);
    let host_now = t0_first + 1.0 - 2.0 * LOW_MARGIN_WARN_SECS;
    let (t0_next, epoch) = a.anchor_segment(1.0, 2.0, host_now);
    assert_eq!(epoch, StreamEpoch::Continuation);
    assert_eq!(t0_first, t0_next);
}

#[test]
fn backward_jump_reanchors() {
    let mut a = Anchor::new();
    let (t0_a, _) = a.anchor_segment(0.0, 5.0, 100.0);
    let (t0_b, epoch) = a.anchor_segment(0.0, 1.0, 130.0);
    assert_eq!(epoch, StreamEpoch::Reposition);
    assert_ne!(t0_a, t0_b);
    assert!((t0_b - (130.0 + DEFAULT_LEAD_SECS)).abs() < 1e-9);
}

#[test]
fn backward_jump_takes_priority_over_underrun() {
    // A backward jump (idle restart) that is also "late" re-anchors as a clean
    // restart, not an underrun stutter.
    let mut a = Anchor::new();
    let _ = a.anchor_segment(0.0, 5.0, 100.0);
    let (t0_new, epoch) = a.anchor_segment(0.0, 1.0, 130.0);
    assert_eq!(
        epoch,
        StreamEpoch::Reposition,
        "backward jump must re-anchor as a clean restart, not an underrun stutter"
    );
    assert!(
        (t0_new - (130.0 + DEFAULT_LEAD_SECS)).abs() < 1e-9,
        "t0_new={t0_new}"
    );
}

#[test]
fn default_lead_is_quarter_second_and_shared_with_planner() {
    assert_eq!(super::DEFAULT_LEAD_SECS, 0.25);
    assert_eq!(crate::worker::lead_secs(), super::DEFAULT_LEAD_SECS);
}

// `queued_motion_secs` (bridge.rs) reads `t0 + last_move_time - host_now`: the
// committed frontier in stream time, grounded onto the host clock by `t0`. The
// host backpressure gate is meant to ride this signal. It was abandoned (commit
// ab538756d) on the claim that it "collapsed to 0 after the first re-anchor"
// because an ungrounded form (stream time minus absolute clock) was used. These
// tests drive the grounding through an idle-gap re-anchor at a large stream-time
// baseline and confirm the grounded frontier reports REAL queued seconds — no
// collapse, no stream-time magnitude leaking through.
fn grounded_queued_secs(t0: f64, frontier_stream_t: f64, host_now: f64) -> f64 {
    t0 + frontier_stream_t - host_now
}

#[test]
fn grounded_frontier_reports_real_queued_seconds_after_reanchor() {
    let mut a = Anchor::new();

    // Deep into a long print: stream time sits near 500 s.
    let _ = a.anchor_segment(500.0, 501.0, 100.0);

    // Idle gap: the playhead (host clock) overran the committed end, then the
    // queue refills. The first post-gap commit underruns and re-anchors.
    let host_now = 130.0;
    let (t0, epoch) = a.anchor_segment(501.0, 502.0, host_now);
    assert!(epoch.is_fresh(), "idle-gap underrun must re-anchor");

    // Queue keeps filling contiguously; the frontier advances, t0 holds.
    let (t0_cont, epoch2) = a.anchor_segment(502.0, 505.0, host_now + 0.5);
    assert_eq!(epoch2, StreamEpoch::Continuation);
    assert_eq!(t0, t0_cont);

    let frontier_stream_t = 505.0;
    let read_host_now = host_now + 0.5;
    let queued = grounded_queued_secs(t0_cont, frontier_stream_t, read_host_now);

    // Committed span [501,505] = 4 s mapped to the host clock, minus how far the
    // playhead advanced into it, plus the lead the frontier floats ahead.
    let expected = DEFAULT_LEAD_SECS + (frontier_stream_t - 501.0) - (read_host_now - host_now);
    assert!(
        (queued - expected).abs() < 1e-9,
        "queued={queued} expected={expected}"
    );
    assert!(
        queued > 0.0,
        "signal collapsed across re-anchor: queued={queued}"
    );
}

#[test]
fn grounding_cancels_the_stream_time_baseline() {
    // Two prints with identical queued depth but wildly different stream-time
    // baselines must yield the same grounded signal — proving t0 cancels the
    // baseline. The ungrounded form (frontier - host_now) would differ by ~500.
    let queued_at = |baseline: f64| {
        let mut a = Anchor::new();
        let _ = a.anchor_segment(baseline, baseline + 1.0, 100.0);
        let host_now = 130.0;
        let (t0, epoch) = a.anchor_segment(baseline + 1.0, baseline + 3.0, host_now);
        assert!(epoch.is_fresh());
        grounded_queued_secs(t0, baseline + 3.0, host_now)
    };
    let near_zero = queued_at(1.0);
    let deep = queued_at(500.0);
    assert!(
        (near_zero - deep).abs() < 1e-9,
        "grounded signal must not depend on stream baseline: {near_zero} vs {deep}"
    );
    assert!((near_zero - (DEFAULT_LEAD_SECS + 2.0)).abs() < 1e-9);
}
