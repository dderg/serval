use super::*;

#[test]
fn first_segment_lands_lead_ahead() {
    let mut a = Anchor::new();
    let (t0, fresh) = a.anchor_segment(0.0, 1.0, 100.0).unwrap();
    assert!(fresh);
    assert!((t0 + 0.0 - (100.0 + DEFAULT_LEAD_SECS)).abs() < 1e-9);
}

#[test]
fn contiguous_segment_keeps_t0() {
    let mut a = Anchor::new();
    let (t0_a, _) = a.anchor_segment(0.0, 1.0, 100.0).unwrap();
    let (t0_b, fresh) = a.anchor_segment(1.0, 2.0, 100.9).unwrap();
    assert!(!fresh);
    assert_eq!(t0_a, t0_b);
}

#[test]
fn late_segment_returns_err_with_correct_gap() {
    let mut a = Anchor::new();
    let _ = a.anchor_segment(0.0, 1.0, 100.0).unwrap();
    let result = a.anchor_segment(1.0, 2.0, 104.0);
    let err = result.expect_err("starvation must return Err");
    assert!(err.gap_s > 0.0, "gap_s must be positive, got {}", err.gap_s);
    let expected_gap = 104.0 - (100.25 + 1.0);
    assert!(
        (err.gap_s - expected_gap).abs() < 1e-9,
        "gap_s={} expected={expected_gap}",
        err.gap_s
    );
    assert_eq!(err.seg_t_start, 1.0);
}

#[test]
fn larger_resume_cushion_avoids_segmentlate_that_smaller_one_triggers() {
    let t0_offset = 100.0;
    let first_t_end = 1.0;
    let host_now_at_resume = t0_offset + 1.867;

    let small_cushion = 0.25;
    let large_cushion = 0.95;

    let mut a_small = Anchor::new();
    let (t0_small, _) = a_small.anchor_segment(0.0, first_t_end, t0_offset).unwrap();
    assert!((t0_small - (t0_offset + DEFAULT_LEAD_SECS)).abs() < 1e-9);
    let small_resume_t_start = first_t_end + small_cushion;
    let small_result = a_small.anchor_segment(
        small_resume_t_start,
        small_resume_t_start + 1.0,
        host_now_at_resume,
    );
    small_result.expect_err("a 250ms cushion must still SegmentLate after an 867ms solve");

    let mut a_large = Anchor::new();
    let _ = a_large.anchor_segment(0.0, first_t_end, t0_offset).unwrap();
    let large_resume_t_start = first_t_end + large_cushion;
    let large_result = a_large.anchor_segment(
        large_resume_t_start,
        large_resume_t_start + 1.0,
        host_now_at_resume,
    );
    large_result.expect("a 950ms cushion must keep seg0 in the future for the same 867ms solve");
}

#[test]
fn backward_jump_reanchors() {
    let mut a = Anchor::new();
    let (t0_a, _) = a.anchor_segment(0.0, 5.0, 100.0).unwrap();
    let (t0_b, fresh) = a.anchor_segment(0.0, 1.0, 130.0).unwrap();
    assert!(fresh);
    assert_ne!(t0_a, t0_b);
    assert!((t0_b - (130.0 + DEFAULT_LEAD_SECS)).abs() < 1e-9);
}

#[test]
fn backward_jump_while_late_reanchors_silently() {
    let mut a = Anchor::new();
    let _ = a.anchor_segment(0.0, 5.0, 100.0).unwrap();
    let result = a.anchor_segment(0.0, 1.0, 130.0);
    let (t0_new, fresh) = result.expect("backward jump must re-anchor, not error");
    assert!(fresh, "backward jump must be fresh");
    assert!(
        (t0_new - (130.0 + DEFAULT_LEAD_SECS)).abs() < 1e-9,
        "t0_new={t0_new}"
    );
}
