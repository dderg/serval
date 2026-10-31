use super::skew_monitor::{
    SKEW_FATAL_CONSECUTIVE, SKEW_FATAL_SECS, SKEW_WARN_HIGH_SECS, SkewMonitor, SkewVerdict,
};

#[test]
fn healthy_echo_band_is_in_bounds() {
    let mut m = SkewMonitor::default();
    for skew in [0.0, 0.0005, 0.019, -0.0009] {
        assert_eq!(m.observe(skew), SkewVerdict::InBounds, "skew {skew}");
    }
}

#[test]
fn out_of_band_but_sub_fatal_skew_warns_without_escalating() {
    let mut m = SkewMonitor::default();
    for _ in 0..100 {
        assert_eq!(m.observe(0.050), SkewVerdict::Warn);
        assert_eq!(m.observe(-0.030), SkewVerdict::Warn);
    }
}

#[test]
fn sustained_divergence_goes_fatal_on_the_third_echo() {
    let mut m = SkewMonitor::default();
    assert_eq!(m.observe(0.520), SkewVerdict::Warn);
    assert_eq!(m.observe(0.520), SkewVerdict::Warn);
    assert_eq!(m.observe(0.520), SkewVerdict::Fatal);
}

#[test]
fn lagging_projection_is_as_fatal_as_a_leading_one() {
    let mut m = SkewMonitor::default();
    for _ in 1..SKEW_FATAL_CONSECUTIVE {
        assert_eq!(m.observe(-2.0 * SKEW_FATAL_SECS), SkewVerdict::Warn);
    }
    assert_eq!(m.observe(-2.0 * SKEW_FATAL_SECS), SkewVerdict::Fatal);
}

#[test]
fn one_good_echo_resets_the_fatal_streak() {
    let mut m = SkewMonitor::default();
    assert_eq!(m.observe(0.200), SkewVerdict::Warn);
    assert_eq!(m.observe(0.200), SkewVerdict::Warn);
    assert_eq!(m.observe(SKEW_WARN_HIGH_SECS), SkewVerdict::InBounds);
    assert_eq!(m.observe(0.200), SkewVerdict::Warn);
    assert_eq!(m.observe(0.200), SkewVerdict::Warn);
    assert_eq!(m.observe(0.200), SkewVerdict::Fatal);
}
