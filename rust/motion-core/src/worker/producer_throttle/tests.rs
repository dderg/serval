use super::*;

#[test]
fn a_throttle_reads_its_realtime_factor_and_onset() {
    assert_eq!(
        ProducerThrottle::parse("0.6@3.5"),
        Ok(ProducerThrottle {
            realtime_factor: 0.6,
            from_stream_s: 3.5,
        })
    );
}

#[test]
fn a_malformed_throttle_is_refused() {
    for spec in ["0.6", "0@1", "-1@1", "inf@1", "0.5@-1", "x@1", "0.5@nan"] {
        assert!(
            ProducerThrottle::parse(spec).is_err(),
            "{spec} must be refused"
        );
    }
}

#[test]
fn planning_runs_free_before_the_onset_and_at_the_factor_after_it() {
    let throttle = ProducerThrottle::parse("0.5@1.0").unwrap();
    assert_eq!(throttle.hold(0.0, 0.8), Duration::ZERO);
    assert_eq!(throttle.hold(0.8, 1.2), Duration::from_secs_f64(0.4));
    assert_eq!(throttle.hold(1.2, 1.2), Duration::ZERO);
    assert_eq!(throttle.hold(2.0, 2.5), Duration::from_secs_f64(1.0));
}
