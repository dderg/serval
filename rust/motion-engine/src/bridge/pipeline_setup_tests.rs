use super::pipeline_setup::{build_stream_config, require_unlimited_config_jerk, retired_by_axis};
use super::{PyMotionEngine, planner_api::require_supported_jerk_override};
use crate::config::PlannerConfig;
use crate::lock_ext::LockExt;

#[test]
fn single_slave_places_retired_at_its_axis() {
    assert_eq!(retired_by_axis(&[2], &[7]), vec![0, 0, 7]);
}

#[test]
fn distinct_axes_map_one_to_one() {
    assert_eq!(retired_by_axis(&[0, 1], &[3, 9]), vec![3, 9]);
}

#[test]
fn awd_axis_reports_the_laggard_slot() {
    assert_eq!(retired_by_axis(&[0, 0, 1, 1], &[5, 3, 8, 8]), vec![3, 8]);
}

#[test]
fn missing_slot_counter_is_skipped() {
    assert_eq!(retired_by_axis(&[0, 1], &[4]), vec![4, 0]);
}

#[test]
fn stream_config_accepts_unlimited_jerk() {
    let mut cfg = PlannerConfig::default();
    cfg.cartesian.max_jerk = f64::INFINITY;

    assert!(build_stream_config(&cfg).is_ok());
}

#[test]
fn stream_config_rejects_finite_jerk() {
    let mut cfg = PlannerConfig::default();
    cfg.cartesian.max_jerk = 100_000.0;

    assert_eq!(
        require_unlimited_config_jerk(cfg.cartesian.max_jerk),
        Err(
            "finite [printer] max_jerk is not supported by the continuous trajectory pipeline; set max_jerk: 0"
        )
    );
    assert!(build_stream_config(&cfg).is_err());
}

#[test]
fn jerk_override_accepts_none_and_positive_infinity() {
    let engine = PyMotionEngine::new();

    engine.set_jerk_override(Some(f64::INFINITY)).unwrap();
    assert_eq!(
        engine.planner_config.lock_ok().runtime_caps.jerk_override,
        Some(f64::INFINITY)
    );

    engine.set_jerk_override(None).unwrap();
    assert_eq!(
        engine.planner_config.lock_ok().runtime_caps.jerk_override,
        None
    );
}

#[test]
fn jerk_override_rejects_every_finite_value() {
    let engine = PyMotionEngine::new();

    for jerk in [0.0, 1.0, -1.0] {
        assert_eq!(
            require_supported_jerk_override(Some(jerk)),
            Err("finite jerk overrides are not supported by the continuous trajectory pipeline")
        );
        assert!(engine.set_jerk_override(Some(jerk)).is_err());
        assert_eq!(
            engine.planner_config.lock_ok().runtime_caps.jerk_override,
            None
        );
    }
}

#[test]
fn jerk_override_rejects_other_non_finite_values() {
    let engine = PyMotionEngine::new();

    for jerk in [f64::NEG_INFINITY, f64::NAN] {
        assert_eq!(
            require_supported_jerk_override(Some(jerk)),
            Err("jerk override must be positive infinity or None")
        );
        assert!(engine.set_jerk_override(Some(jerk)).is_err());
        assert_eq!(
            engine.planner_config.lock_ok().runtime_caps.jerk_override,
            None
        );
    }
}
