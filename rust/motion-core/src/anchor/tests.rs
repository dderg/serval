use super::*;

// The classifier is the pure verdict behind `anchor_segment`; these assert
// the fatal branches (which `anchor_segment` turns into a process abort, so
// they cannot be exercised through the public method) plus the recoverable
// ones, all without aborting the test process.

fn primed(last_ends_at_rest: bool) -> Anchor {
    // t0 anchored at 0 with the previous segment ending at stream-t 1.0.
    let mut a = Anchor::new();
    a.anchor_segment(0.0, 1.0, 100.0, last_ends_at_rest);
    a
}

#[test]
fn classify_mid_motion_underrun_is_fatal() {
    let a = primed(false);
    // t0 = 100 + 0.25 - 0 = 100.25; next seg starts at stream-t 1.0 -> abs
    // 101.25. Playhead at 101.5 has overrun it by 0.25s, mid-motion.
    let class = a.classify(1.0, 101.5);
    assert!(
        matches!(class, AnchorClass::UnderrunFatal { gap_s, .. } if (gap_s - 0.25).abs() < 1e-9),
        "mid-motion underrun must be fatal, got {class:?}",
    );
}

#[test]
fn classify_mid_motion_low_margin_is_fatal() {
    let a = primed(false);
    // Abs start 101.25; playhead 101.24 leaves a +0.01s margin, under the
    // 0.02s floor, mid-motion.
    let class = a.classify(1.0, 101.24);
    assert!(
        matches!(class, AnchorClass::LowMarginFatal { margin_s, .. }
            if margin_s > 0.0 && margin_s < LOW_MARGIN_WARN_SECS),
        "mid-motion sub-floor margin must be fatal, got {class:?}",
    );
}

#[test]
fn classify_same_starvation_from_rest_is_an_idle_resume() {
    // Identical geometry to the underrun case, but the previous segment
    // ended at rest: the very same overrun is a recoverable idle resume.
    let a = primed(true);
    let class = a.classify(1.0, 101.5);
    assert!(
        matches!(class, AnchorClass::IdleResume { .. }),
        "an overrun from rest must re-anchor, not fault, got {class:?}",
    );
}

#[test]
fn classify_healthy_margin_is_a_continuation() {
    let a = primed(false);
    // Playhead well behind the start (0.25s margin): a healthy continuation.
    assert_eq!(a.classify(1.0, 101.0), AnchorClass::Continuation);
}

#[test]
fn classify_backward_jump_repositions_even_mid_motion() {
    let a = primed(false);
    // A start before the last segment's end is an idle restart regardless of
    // motion state — a clean reposition, never a fault.
    assert_eq!(a.classify(0.0, 130.0), AnchorClass::Reposition);
}
#[test]
fn first_segment_lands_lead_ahead() {
    let mut a = Anchor::new();
    let (t0, epoch) = a.anchor_segment(0.0, 1.0, 100.0, false);
    assert_eq!(epoch, StreamEpoch::Reposition);
    assert!((t0 + 0.0 - (100.0 + DEFAULT_LEAD_SECS)).abs() < 1e-9);
}

#[test]
fn contiguous_segment_keeps_t0() {
    let mut a = Anchor::new();
    let (t0_a, _) = a.anchor_segment(0.0, 1.0, 100.0, false);
    let (t0_b, epoch) = a.anchor_segment(1.0, 2.0, 100.9, false);
    assert_eq!(epoch, StreamEpoch::Continuation);
    assert_eq!(t0_a, t0_b);
}

#[test]
fn underrun_after_rest_reanchors_as_idle_resume() {
    let mut a = Anchor::new();
    // The first segment brakes to rest: the machine is parked at its end.
    let (t0_first, _) = a.anchor_segment(0.0, 1.0, 100.0, true);
    // Playhead (104.0) has overrun the scheduled start (t0_first + 1.0): an
    // idle gap at rest, not a mid-motion underrun — it must re-anchor.
    let (t0_new, epoch) = a.anchor_segment(1.0, 2.0, 104.0, false);
    assert_eq!(epoch, StreamEpoch::Reanchor, "idle resume must re-anchor");
    assert_ne!(t0_first, t0_new);
    // The resumed segment lands a lead ahead of the current playhead.
    assert!(
        (t0_new + 1.0 - (104.0 + DEFAULT_LEAD_SECS)).abs() < 1e-9,
        "t0_new={t0_new}"
    );
}

#[test]
fn thin_margin_after_rest_reanchors_instead_of_racing_transport() {
    let mut a = Anchor::new();
    let (t0_first, _) = a.anchor_segment(0.0, 1.0, 100.0, true);
    // Post-homing seam on the bench: the segment start is still (barely)
    // ahead of the playhead, but closer than transport latency can cover —
    // continuing here latched a -308 PieceStartInPast at the drive. The
    // machine sits at rest after homing, so this re-anchors.
    let host_now = t0_first + 1.0 - 0.5 * LOW_MARGIN_WARN_SECS;
    let (t0_new, epoch) = a.anchor_segment(1.0, 2.0, host_now, false);
    assert_eq!(
        epoch,
        StreamEpoch::Reanchor,
        "resume from rest must re-anchor"
    );
    assert!(
        (t0_new + 1.0 - (host_now + DEFAULT_LEAD_SECS)).abs() < 1e-9,
        "t0_new={t0_new}"
    );
}

#[test]
fn margin_above_the_floor_stays_a_continuation() {
    let mut a = Anchor::new();
    let (t0_first, _) = a.anchor_segment(0.0, 1.0, 100.0, false);
    let host_now = t0_first + 1.0 - 2.0 * LOW_MARGIN_WARN_SECS;
    let (t0_next, epoch) = a.anchor_segment(1.0, 2.0, host_now, false);
    assert_eq!(epoch, StreamEpoch::Continuation);
    assert_eq!(t0_first, t0_next);
}

#[test]
fn backward_jump_reanchors() {
    let mut a = Anchor::new();
    let (t0_a, _) = a.anchor_segment(0.0, 5.0, 100.0, false);
    let (t0_b, epoch) = a.anchor_segment(0.0, 1.0, 130.0, false);
    assert_eq!(epoch, StreamEpoch::Reposition);
    assert_ne!(t0_a, t0_b);
    assert!((t0_b - (130.0 + DEFAULT_LEAD_SECS)).abs() < 1e-9);
}

#[test]
fn backward_jump_takes_priority_over_underrun() {
    // A backward jump (idle restart) that is also "late" re-anchors as a clean
    // restart, not an underrun — even when the last segment ended mid-motion,
    // because the reposition legitimately redefines the timeline.
    let mut a = Anchor::new();
    let _ = a.anchor_segment(0.0, 5.0, 100.0, false);
    let (t0_new, epoch) = a.anchor_segment(0.0, 1.0, 130.0, false);
    assert_eq!(
        epoch,
        StreamEpoch::Reposition,
        "backward jump must re-anchor as a clean restart, not an underrun"
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

    // Deep into a long print: stream time sits near 500 s, braking to rest.
    let _ = a.anchor_segment(500.0, 501.0, 100.0, true);

    // Idle gap at rest: the playhead (host clock) overran the committed end,
    // then the queue refills. The first post-gap commit re-anchors.
    let host_now = 130.0;
    let (t0, epoch) = a.anchor_segment(501.0, 502.0, host_now, false);
    assert!(epoch.is_fresh(), "idle-gap resume must re-anchor");

    // Queue keeps filling contiguously; the frontier advances, t0 holds.
    let (t0_cont, epoch2) = a.anchor_segment(502.0, 505.0, host_now + 0.5, false);
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
        let _ = a.anchor_segment(baseline, baseline + 1.0, 100.0, true);
        let host_now = 130.0;
        let (t0, epoch) = a.anchor_segment(baseline + 1.0, baseline + 3.0, host_now, false);
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
