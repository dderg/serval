use super::*;

#[test]
fn a_throttle_reads_its_realtime_factor_and_onset() {
    assert_eq!(
        ProducerThrottle::parse("0.6@3.5"),
        Ok(ProducerThrottle {
            realtime_factor: 0.6,
            from_stream_s: 3.5,
            owed_s: 0.0,
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
    let mut throttle = ProducerThrottle::parse("0.5@1.0").unwrap();
    assert_eq!(throttle.hold(0.0, 0.8), Duration::ZERO);
    assert!((throttle.hold(0.8, 1.004).as_secs_f64() - 0.008).abs() < 1e-9);
    assert_eq!(throttle.hold(1.004, 1.004), Duration::ZERO);
}

#[test]
fn a_release_of_seconds_is_paid_a_step_at_a_time() {
    let mut throttle = ProducerThrottle::parse("0.5@0.0").unwrap();
    let mut held = vec![throttle.hold(0.0, 2.4)];
    while held.last() != Some(&Duration::ZERO) {
        held.push(throttle.hold(2.4, 2.4));
    }
    assert!(held.iter().all(|&step| step <= MAX_HOLD_PER_STEP));
    let total: f64 = held.iter().map(Duration::as_secs_f64).sum();
    assert!(
        (total - 4.8).abs() < 1e-9,
        "the release still costs {total}s"
    );
}
