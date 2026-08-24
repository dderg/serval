use super::*;

// The classifier is the pure verdict behind `anchor_segment`; these assert
// the fatal branches (which `anchor_segment` turns into a process abort, so
// they cannot be exercised through the public method) plus the recoverable
// ones, all without aborting the test process.

fn primed(parked: bool) -> Anchor {
    // t0 anchored at 0 with the previous segment ending at stream-t 1.0.
    let mut a = Anchor::new();
    a.anchor_segment(0.0, 1.0, 100.0);
    if parked {
        a.mark_parked();
    }
    a
}

#[test]
fn classify_mid_motion_underrun_is_fatal() {
    let a = primed(false);
    let class = a.classify(1.0, 101.0 + DEFAULT_LEAD_SECS + 0.25);
    assert!(
        matches!(class, AnchorClass::UnderrunFatal { gap_s, .. } if (gap_s - 0.25).abs() < 1e-9),
        "mid-motion underrun must be fatal, got {class:?}",
    );
}

#[test]
fn classify_mid_motion_low_margin_is_fatal() {
    let a = primed(false);
    let class = a.classify(1.0, 101.0 + DEFAULT_LEAD_SECS - 0.01);
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
    let class = a.classify(1.0, 101.0 + DEFAULT_LEAD_SECS + 0.25);
    assert!(
        matches!(class, AnchorClass::IdleResume { .. }),
        "an overrun from rest must re-anchor, not fault, got {class:?}",
    );
}

#[test]
fn classify_healthy_margin_is_a_continuation() {
    let a = primed(false);
    assert!(matches!(
        a.classify(1.0, 101.0 + DEFAULT_LEAD_SECS - 0.25),
        AnchorClass::Continuation { .. }
    ));
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
fn underrun_after_rest_reanchors_as_idle_resume() {
    let mut a = Anchor::new();
    // The first segment brakes to rest: the machine is parked at its end.
    let (t0_first, _) = a.anchor_segment(0.0, 1.0, 100.0);
    a.mark_parked();
    // Playhead (104.0) has overrun the scheduled start (t0_first + 1.0): an
    // idle gap at rest, not a mid-motion underrun — it must re-anchor.
    let (t0_new, epoch) = a.anchor_segment(1.0, 2.0, 104.0);
    assert_eq!(epoch, StreamEpoch::Reanchor, "idle resume must re-anchor");
    assert_ne!(t0_first, t0_new);
    // The resumed segment lands the earned lead ahead of the current playhead.
    assert!(
        (t0_new + 1.0 - (104.0 + DEFAULT_LEAD_SECS)).abs() < 1e-9,
        "t0_new={t0_new}"
    );
}

#[test]
fn thin_margin_after_rest_reanchors_instead_of_racing_transport() {
    let mut a = Anchor::new();
    let (t0_first, _) = a.anchor_segment(0.0, 1.0, 100.0);
    a.mark_parked();
    // Post-homing seam on the bench: the segment start is still (barely)
    // ahead of the playhead, but closer than transport latency can cover —
    // continuing here latched a -308 PieceStartInPast at the drive. The
    // machine sits at rest after homing, so this re-anchors.
    let host_now = t0_first + 1.0 - 0.5 * LOW_MARGIN_WARN_SECS;
    let (t0_new, epoch) = a.anchor_segment(1.0, 2.0, host_now);
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
fn dwell_hole_at_rest_with_healthy_margin_rejoins_on_the_standing_anchor() {
    // A dwell drains to rest and advances stream time with no pieces; the
    // next segment arrives while the margin is still comfortable (buffered
    // print, no M400). Timing must stay on the standing anchor, but the
    // epoch must be fresh so per-lane transports cut their seams.
    let mut a = primed(true);
    let t0_before = a.t0().unwrap();
    // 0.2s hole (two DISABLE_STALL_TIME dwells); playhead far behind.
    let (t0, epoch) = a.anchor_segment(1.2, 2.2, 100.0);
    assert_eq!(epoch, StreamEpoch::Rejoin, "dwell hole must cut seams");
    assert!(epoch.is_fresh());
    assert!(!epoch.retimed(), "a rejoin must not re-derive t0");
    assert_eq!(t0, t0_before, "the standing anchor keeps the timing");
}

#[test]
fn dwell_hole_with_decayed_margin_is_still_an_idle_resume() {
    // Once the playhead has actually consumed the dwell, the resume is a
    // plain idle resume: re-anchor forward on the earned lead.
    let mut a = primed(true);
    let (t0_new, epoch) = a.anchor_segment(1.2, 2.2, 104.0);
    assert_eq!(epoch, StreamEpoch::Reanchor);
    assert!(
        (t0_new + 1.2 - (104.0 + DEFAULT_LEAD_SECS)).abs() < 1e-9,
        "t0_new={t0_new}"
    );
}

#[test]
fn classify_hole_mid_motion_is_fatal() {
    // A forward stream-time hole while the previous segment ended in motion
    // means trajectory content is missing — never paper over it.
    let a = primed(false);
    let class = a.classify(1.2, 100.0);
    assert!(
        matches!(class, AnchorClass::HoleMidMotionFatal { hole_s }
            if (hole_s - 0.2).abs() < 1e-9),
        "mid-motion hole must be fatal, got {class:?}",
    );
}

#[test]
fn sub_eps_stream_gap_stays_a_continuation() {
    let mut a = primed(false);
    let (_, epoch) = a.anchor_segment(1.0 + 0.5 * CONTIGUITY_EPS, 2.0, 100.9);
    assert_eq!(epoch, StreamEpoch::Continuation);
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
    // restart, not an underrun — even when the last segment ended mid-motion,
    // because the reposition legitimately redefines the timeline.
    let mut a = Anchor::new();
    let _ = a.anchor_segment(0.0, 5.0, 100.0);
    let (t0_new, epoch) = a.anchor_segment(0.0, 1.0, 130.0);
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
fn default_lead_covers_continuous_post_processing_and_matches_planner() {
    assert_eq!(super::DEFAULT_LEAD_SECS, 0.525);
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
    let _ = a.anchor_segment(500.0, 501.0, 100.0);
    a.mark_parked();

    // Idle gap at rest: the playhead (host clock) overran the committed end,
    // then the queue refills. The first post-gap commit re-anchors.
    let host_now = 130.0;
    let (t0, epoch) = a.anchor_segment(501.0, 502.0, host_now);
    assert!(epoch.is_fresh(), "idle-gap resume must re-anchor");

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
        a.mark_parked();
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

#[test]
fn every_fresh_anchor_uses_the_default_lead() {
    let mut a = Anchor::new();
    for i in 0..8 {
        let stream_start = f64::from(i);
        let host_now = 100.0 * f64::from(i + 1);
        let (t0, _) = a.anchor_segment(stream_start, stream_start + 1.0, host_now);
        assert!(
            (t0 + stream_start - (host_now + DEFAULT_LEAD_SECS)).abs() < 1e-9,
            "resume {i} did not use the fixed default lead"
        );
        a.mark_parked();
    }
}
