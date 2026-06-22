use super::*;

#[test]
fn first_segment_lands_lead_ahead() {
    let mut a = Anchor::new();
    let (t0, fresh) = a.anchor_segment(0.0, 1.0, 100.0);
    assert!(fresh);
    assert!((t0 + 0.0 - (100.0 + DEFAULT_LEAD_SECS)).abs() < 1e-9);
}

#[test]
fn contiguous_segment_keeps_t0() {
    let mut a = Anchor::new();
    let (t0_a, _) = a.anchor_segment(0.0, 1.0, 100.0);
    let (t0_b, fresh) = a.anchor_segment(1.0, 2.0, 100.9);
    assert!(!fresh);
    assert_eq!(t0_a, t0_b);
}

#[test]
fn underrun_reanchors_forward_instead_of_aborting() {
    let mut a = Anchor::new();
    let (t0_first, _) = a.anchor_segment(0.0, 1.0, 100.0);
    // Playhead (104.0) has overrun the scheduled start (t0_first + 1.0): a
    // genuine underrun. The anchor must re-anchor forward, not fail.
    let (t0_new, fresh) = a.anchor_segment(1.0, 2.0, 104.0);
    assert!(fresh, "underrun must re-anchor");
    assert_ne!(t0_first, t0_new);
    // The recovered segment lands a lead ahead of the current playhead.
    assert!(
        (t0_new + 1.0 - (104.0 + DEFAULT_LEAD_SECS)).abs() < 1e-9,
        "t0_new={t0_new}"
    );
}

#[test]
fn backward_jump_reanchors() {
    let mut a = Anchor::new();
    let (t0_a, _) = a.anchor_segment(0.0, 5.0, 100.0);
    let (t0_b, fresh) = a.anchor_segment(0.0, 1.0, 130.0);
    assert!(fresh);
    assert_ne!(t0_a, t0_b);
    assert!((t0_b - (130.0 + DEFAULT_LEAD_SECS)).abs() < 1e-9);
}

#[test]
fn backward_jump_takes_priority_over_underrun() {
    // A backward jump (idle restart) that is also "late" re-anchors as a clean
    // restart, not an underrun stutter.
    let mut a = Anchor::new();
    let _ = a.anchor_segment(0.0, 5.0, 100.0);
    let (t0_new, fresh) = a.anchor_segment(0.0, 1.0, 130.0);
    assert!(fresh, "backward jump must re-anchor");
    assert!(
        (t0_new - (130.0 + DEFAULT_LEAD_SECS)).abs() < 1e-9,
        "t0_new={t0_new}"
    );
}

#[test]
fn default_lead_is_quarter_second_and_shared_with_planner() {
    assert_eq!(super::DEFAULT_LEAD_SECS, 0.25);
    assert_eq!(crate::planner::lead_secs(), super::DEFAULT_LEAD_SECS);
}
