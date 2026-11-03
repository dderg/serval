use super::*;

#[test]
fn stream_config_accepts_unlimited_jerk() {
    let mut cfg = PlannerConfig::default();
    cfg.cartesian.max_jerk = f64::INFINITY;

    assert!(stream_config(&cfg).is_ok());
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
    assert!(stream_config(&cfg).is_err());
}
