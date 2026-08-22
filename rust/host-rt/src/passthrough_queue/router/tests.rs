use super::*;
use crate::clock::MockClock;
use std::time::Duration;

fn make_router() -> (PassthroughRouter, Arc<MockClock>) {
    let clock = MockClock::new();
    let router = PassthroughRouter::with_clock(Arc::clone(&clock) as Arc<dyn Clock + Send + Sync>);
    (router, clock)
}

#[test]
fn two_mcus_claim_release_independently() {
    let (mut router, _) = make_router();
    let a = router.claim_mcu("mcu_a");
    let b = router.claim_mcu("mcu_b");
    assert_ne!(a, b);
    router.set_clock_est(b, 48_000_000.0, 0.0, 1000).unwrap();

    router.release_mcu(a);
    assert!(router.compute_ack_clock(b).is_ok());
    assert!(matches!(
        router.compute_ack_clock(a),
        Err(RouterError::UnknownMcu(_))
    ));
}

#[test]
fn set_clock_est_stores_values() {
    let (mut router, _) = make_router();
    let mcu = router.claim_mcu("mcu");

    assert!(matches!(
        router.compute_ack_clock(mcu),
        Err(RouterError::NoClockEstimate(_))
    ));

    router.set_clock_est(mcu, 48_000_000.0, 0.0, 1000).unwrap();

    let ack = router.compute_ack_clock(mcu).unwrap();
    assert!(ack >= 1000, "ack_clock should be at least last_clock");
}

#[test]
fn compute_ack_clock_projects_from_host_time() {
    let (mut router, clock) = make_router();
    let mcu = router.claim_mcu("mcu");

    let base_host = instant_to_f64(clock.now());
    router
        .set_clock_est(mcu, 1_000_000.0, base_host, 0)
        .unwrap();

    let ack0 = router.compute_ack_clock(mcu).unwrap();
    assert_eq!(ack0, 0);

    clock.advance(Duration::from_secs(1));
    let ack1 = router.compute_ack_clock(mcu).unwrap();
    let diff = (ack1 as i64 - 1_000_000_i64).unsigned_abs();
    assert!(diff <= 1, "expected ~1_000_000, got {ack1}");
}

#[test]
fn ack_clock_and_freq_projects_across_endpoint_command_delay() {
    let (mut router, clock) = make_router();
    let mcu = router.claim_mcu("ethercat");
    let base_host = instant_to_f64(clock.now());
    router
        .set_clock_est(mcu, 1_000_000.0, base_host, 7_000_000)
        .unwrap();

    clock.advance(Duration::from_millis(31));

    let (projected, freq) = router.ack_clock_and_freq(mcu).unwrap();
    assert_eq!(freq, 1_000_000.0);
    assert_eq!(projected, 7_031_000);
}

#[test]
fn set_clock_est_rebased_advances_with_mock_clock() {
    let (mut router, clock) = make_router();
    let mcu = router.claim_mcu("mcu");

    // `offset_raw` must be in the same RAW-domain epoch as `bridge_now_raw`
    // inside `set_clock_est_rebased`.  Capture it here so the conversion
    // `raw_at_anchor = bridge_now_raw - bridge_now_instant` yields a small,
    // positive number and `clock_offset` lands near the mock clock's current
    // Instant value.
    let offset_raw = crate::clock::monotonic_raw_secs();

    router
        .set_clock_est_rebased(mcu, 1_000_000.0, offset_raw, 10_000_000, true, 0.0)
        .unwrap();

    let ack0 = router.compute_ack_clock(mcu).unwrap();

    clock.advance(Duration::from_secs(1));
    let ack1 = router.compute_ack_clock(mcu).unwrap();
    let diff = (ack1 as i64 - ack0 as i64 - 1_000_000_i64).unsigned_abs();
    assert!(
        diff <= 1,
        "ack_clock must advance by ~1_000_000 ticks per second; \
         ack0={ack0} ack1={ack1} diff={diff}"
    );
}

#[test]
fn set_clock_est_rebased_epsilon_independent() {
    let freq = 1_000_000.0_f64;
    let offset_raw = 990.0_f64;
    let last_clock = 10_000_000_u64;

    let (mut router_a, _clock_a) = make_router();
    let mcu_a = router_a.claim_mcu("mcu_a");

    let (mut router_b, _clock_b) = make_router();
    let mcu_b = router_b.claim_mcu("mcu_b");

    router_a
        .set_clock_est_rebased(mcu_a, freq, offset_raw, last_clock, true, 1000.0)
        .unwrap();

    router_b
        .set_clock_est_rebased(mcu_b, freq, offset_raw, last_clock, true, 1000.0 - 0.050)
        .unwrap();

    let ack_a = router_a.compute_ack_clock(mcu_a).unwrap();
    let ack_b = router_b.compute_ack_clock(mcu_b).unwrap();

    let diff = (ack_a as i64 - ack_b as i64).unsigned_abs();
    assert!(
        diff <= 2,
        "compute_ack_clock must not vary with host_now_raw (ε-independence); \
         host_now_raw differed by 50 ms but ack_clock differed by {diff} ticks \
         (ack_a={ack_a} ack_b={ack_b})"
    );
}

#[test]
fn compute_ack_clock_unknown_mcu_errors() {
    let (router, _) = make_router();
    let bogus = McuHandle(999);
    assert!(router.compute_ack_clock(bogus).is_err());
}

#[test]
fn wall_time_at_mcu_known_record_returns_wall_time() {
    let (mut router, clock) = make_router();
    let mcu = router.claim_mcu("mcu");

    let anchor_host = instant_to_f64(clock.now());
    router
        .set_clock_est(mcu, 100_000_000.0, anchor_host, 100_000_000)
        .unwrap();

    let (dt, estimated) = router
        .wall_time_at_mcu(mcu, 100_000_000)
        .expect("must return Some when clock record is set");

    let now_unix = time::OffsetDateTime::now_utc();
    let diff = (dt - now_unix).abs();
    assert!(
        diff <= time::Duration::seconds(1),
        "wall time {dt} must be within 1 s of system clock {now_unix}"
    );
    assert!(
        !estimated,
        "estimated must be false when delta is exactly 0 ticks"
    );
}

#[test]
fn wall_time_at_mcu_far_from_anchor_returns_estimated_true() {
    let (mut router, clock) = make_router();
    let mcu = router.claim_mcu("mcu");

    let anchor_host = crate::clock::instant_to_f64(clock.now());
    router
        .set_clock_est(mcu, 100_000_000.0, anchor_host, 100_000_000)
        .unwrap();

    let (_, estimated) = router
        .wall_time_at_mcu(mcu, 300_000_000)
        .expect("must return Some when clock record is set");

    assert!(
        estimated,
        "estimated must be true when tick is 2 MCU-seconds from anchor"
    );
}

#[test]
fn clock_to_host_secs_round_trips() {
    let (mut router, clock) = make_router();
    let mcu = router.claim_mcu("mcu");

    let base_host = instant_to_f64(clock.now());
    router
        .set_clock_est(mcu, 1_000_000.0, base_host, 10_000_000)
        .unwrap();

    let trip_host = base_host + 1.5;
    let mcu_clock = router.host_time_to_mcu_clock(mcu, trip_host).unwrap();
    assert_eq!(
        mcu_clock, 11_500_000,
        "forward projection must yield 11_500_000"
    );

    let recovered = router.clock_to_host_secs(mcu, mcu_clock).unwrap();
    let diff = (recovered - trip_host).abs();
    assert!(
        diff < 1e-9,
        "round-trip error too large: recovered={recovered:.12} expected={trip_host:.12} diff={diff:e}"
    );
}

#[test]
fn clock_to_host_secs_no_record_returns_none() {
    let (mut router, _) = make_router();
    let mcu = router.claim_mcu("mcu");
    assert!(
        router.clock_to_host_secs(mcu, 1_000_000).is_none(),
        "must return None when clock_freq == 0"
    );
}

#[test]
fn print_time_to_host_secs_matches_clock_conversion() {
    let (mut router, clock) = make_router();
    let mcu = router.claim_mcu("mcu");

    let base_host = instant_to_f64(clock.now());
    router
        .set_clock_est(mcu, 1_000_000.0, base_host, 10_000_000)
        .unwrap();
    router.set_nominal_freq(mcu, 1_000_000.0).unwrap();

    let via_print_time = router.print_time_to_host_secs(mcu, 11.5).unwrap();
    let via_clock = router.clock_to_host_secs(mcu, 11_500_000).unwrap();
    let diff = (via_print_time - via_clock).abs();
    assert!(
        diff < 1e-9,
        "print_time conversion must equal clock conversion: \
         via_print_time={via_print_time:.12} via_clock={via_clock:.12}"
    );
}

#[test]
fn print_time_to_host_secs_unsynced_returns_none() {
    let (mut router, _) = make_router();
    let mcu = router.claim_mcu("mcu");
    assert!(
        router.print_time_to_host_secs(mcu, 1.0).is_none(),
        "must return None when clock_freq == 0"
    );
}

#[test]
fn clock_to_host_secs_unknown_mcu_returns_none() {
    let (router, _) = make_router();
    assert!(
        router
            .clock_to_host_secs(McuHandle::from_raw(999), 0)
            .is_none()
    );
}

#[test]
fn wall_time_at_mcu_no_record_returns_none() {
    let (mut router, _) = make_router();
    let mcu = router.claim_mcu("mcu");

    assert!(
        router.wall_time_at_mcu(mcu, 1_000_000_000).is_none(),
        "must return None when no clock record has been set"
    );
}

#[test]
fn correction_anchor_round_trips_print_time_to_mcu_clock() {
    let (mut router, _) = make_router();
    let mcu = router.claim_mcu("mcu");
    let nominal = 48_000_000.0;
    let regression = 48_000_048.0; // 1 ppm off from nominal — must not be conflated.
    let offset = 5.0;
    let last_clock = 1_000_000u64;
    router
        .set_clock_est(mcu, regression, offset, last_clock)
        .unwrap();
    router.set_nominal_freq(mcu, nominal).unwrap();
    let glmt_print_time = 7.5;
    let host = router
        .print_time_to_host_secs(mcu, glmt_print_time)
        .unwrap();
    let clock = router.host_time_to_mcu_clock(mcu, host).unwrap();
    // Klipper defines print_time_to_clock as pt * nominal CLOCK_FREQ
    // (klippy/clocksync.py:181); the regression frequency used along the way
    // must cancel out, not leak into the target tick count.
    let expected = (glmt_print_time * nominal) as u64;
    let diff = (clock as i64 - expected as i64).unsigned_abs();
    assert!(
        diff <= 1,
        "clock target must track the nominal frequency, not the drifted \
         regression: clock={clock} expected={expected} diff={diff}"
    );
}

#[test]
fn print_time_at_host_matches_hand_computed_case_after_offset() {
    let (mut router, _) = make_router();
    let mcu = router.claim_mcu("mcu");
    // Regression (clock_freq) and nominal (datasheet) frequencies deliberately
    // differ so conflating them would be caught.
    router
        .set_clock_est(mcu, 1_000_000.0, 100.0, 5_000_000)
        .unwrap();
    router.set_nominal_freq(mcu, 2_000_000.0).unwrap();

    // clock = last_clock + (host - offset) * clock_freq
    //       = 5_000_000 + 1.0 * 1_000_000 = 6_000_000
    // print_time = clock / nominal_freq = 6_000_000 / 2_000_000 = 3.0
    let pt = router
        .print_time_at_host(mcu, HostSecs::from_anchor_frame(101.0))
        .unwrap();
    assert!(
        (pt.get() - 3.0).abs() < 1e-9,
        "expected print_time 3.0, got {}",
        pt.get()
    );
}

#[test]
fn print_time_at_host_second_hand_computed_case() {
    let (mut router, _) = make_router();
    let mcu = router.claim_mcu("mcu");
    // Swap which of the two frequencies is larger versus the first case, so
    // a test that only worked by accident (e.g. using max/min) would fail.
    router
        .set_clock_est(mcu, 2_000_000.0, 50.0, 3_000_000)
        .unwrap();
    router.set_nominal_freq(mcu, 1_000_000.0).unwrap();

    // clock = 3_000_000 + (52.0 - 50.0) * 2_000_000 = 7_000_000
    // print_time = 7_000_000 / 1_000_000 = 7.0
    let pt = router
        .print_time_at_host(mcu, HostSecs::from_anchor_frame(52.0))
        .unwrap();
    assert!(
        (pt.get() - 7.0).abs() < 1e-9,
        "expected print_time 7.0, got {}",
        pt.get()
    );
}

#[test]
fn print_time_at_host_before_offset_goes_negative_without_clamping() {
    let (mut router, _) = make_router();
    let mcu = router.claim_mcu("mcu");
    router
        .set_clock_est(mcu, 1_000_000.0, 100.0, 4_000_000)
        .unwrap();
    router.set_nominal_freq(mcu, 500_000.0).unwrap();

    // host precedes offset by 10s:
    // clock = 4_000_000 + (-10.0) * 1_000_000 = -6_000_000
    // print_time = -6_000_000 / 500_000 = -12.0 — the projection must NOT
    // clamp to last_clock/nominal_freq (8.0).
    let pt = router
        .print_time_at_host(mcu, HostSecs::from_anchor_frame(90.0))
        .unwrap();
    assert!(
        (pt.get() - (-12.0)).abs() < 1e-9,
        "expected print_time -12.0 (host before offset must not clamp), got {}",
        pt.get()
    );
    assert!(
        pt.get() < 4_000_000.0 / 500_000.0,
        "print_time must go below last_clock/nominal_freq when host precedes offset, got {}",
        pt.get()
    );
}

#[test]
fn print_time_at_host_clock_est_set_but_nominal_missing_returns_none() {
    let (mut router, _) = make_router();
    let mcu = router.claim_mcu("mcu");
    router.set_clock_est(mcu, 1_000_000.0, 0.0, 0).unwrap();
    assert!(
        router
            .print_time_at_host(mcu, HostSecs::from_anchor_frame(1.0))
            .is_none(),
        "clock est set but nominal_freq never set must return None"
    );
}

#[test]
fn print_time_at_host_nominal_set_but_clock_est_missing_returns_none() {
    let (mut router, _) = make_router();
    let mcu = router.claim_mcu("mcu");
    router.set_nominal_freq(mcu, 1_000_000.0).unwrap();
    assert!(
        router
            .print_time_at_host(mcu, HostSecs::from_anchor_frame(1.0))
            .is_none(),
        "nominal_freq set but clock est never set (clock_freq == 0) must return None"
    );
}

#[test]
fn print_time_now_equals_print_time_at_host_of_mock_now() {
    let (mut router, clock) = make_router();
    let mcu = router.claim_mcu("mcu");
    let base_host = instant_to_f64(clock.now());
    router
        .set_clock_est(mcu, 1_000_000.0, base_host, 10_000_000)
        .unwrap();
    router.set_nominal_freq(mcu, 1_000_000.0).unwrap();

    let now_hs = HostSecs::from_instant(clock.now());
    let via_now = router.print_time_now(mcu).unwrap();
    let via_at_host = router.print_time_at_host(mcu, now_hs).unwrap();
    assert!(
        (via_now.get() - via_at_host.get()).abs() < 1e-9,
        "print_time_now must equal print_time_at_host(now): via_now={} via_at_host={}",
        via_now.get(),
        via_at_host.get()
    );
}

#[test]
fn print_time_now_advances_by_exactly_the_clock_advance() {
    let (mut router, clock) = make_router();
    let mcu = router.claim_mcu("mcu");
    let base_host = instant_to_f64(clock.now());
    // No drift here (clock_freq == nominal_freq): advancing the host clock by
    // D must advance print_time by exactly D.
    router
        .set_clock_est(mcu, 1_000_000.0, base_host, 10_000_000)
        .unwrap();
    router.set_nominal_freq(mcu, 1_000_000.0).unwrap();

    let before = router.print_time_now(mcu).unwrap();
    clock.advance(Duration::from_millis(1500));
    let after = router.print_time_now(mcu).unwrap();

    let delta = after.get() - before.get();
    assert!(
        (delta - 1.5).abs() < 1e-9,
        "print_time_now must advance by exactly the mock clock advance: delta={delta}"
    );
}

#[test]
fn print_time_at_host_zero_freq_returns_none() {
    let (mut router, _) = make_router();
    let mcu = router.claim_mcu("mcu");
    assert!(
        router
            .print_time_at_host(mcu, HostSecs::from_anchor_frame(1.0))
            .is_none(),
        "freq == 0 (no estimate yet) must return None"
    );
}

#[test]
fn print_time_now_zero_freq_returns_none() {
    let (mut router, _) = make_router();
    let mcu = router.claim_mcu("mcu");
    assert!(
        router.print_time_now(mcu).is_none(),
        "freq == 0 (no estimate yet) must return None"
    );
}

#[test]
fn print_time_at_host_unknown_mcu_returns_none() {
    let (router, _) = make_router();
    assert!(
        router
            .print_time_at_host(McuHandle::from_raw(999), HostSecs::from_anchor_frame(1.0))
            .is_none()
    );
}

#[test]
fn print_time_now_unknown_mcu_returns_none() {
    let (router, _) = make_router();
    assert!(router.print_time_now(McuHandle::from_raw(999)).is_none());
}

#[test]
fn print_time_at_host_consistency_with_print_time_now() {
    let (mut router, clock) = make_router();
    let mcu = router.claim_mcu("mcu");
    let base_host = instant_to_f64(clock.now());
    // No drift here: the two frequencies match, so the offsets cancel exactly
    // and print_time advances 1:1 with host time.
    router
        .set_clock_est(mcu, 3_000_000.0, base_host, 7_000_000)
        .unwrap();
    router.set_nominal_freq(mcu, 3_000_000.0).unwrap();

    let now = router.print_time_now(mcu).unwrap();
    let now_secs = instant_to_f64(clock.now());

    for h_delta in [-5.0_f64, -0.001, 0.0, 0.001, 5.0, 123.456] {
        let h = HostSecs::from_anchor_frame(now_secs + h_delta);
        let pt = router.print_time_at_host(mcu, h).unwrap();
        let diff = pt.get() - now.get();
        assert!(
            (diff - h_delta).abs() < 1e-9,
            "print_time_at_host(h) - print_time_now() must equal h - now within 1e-9: \
             h_delta={h_delta} diff={diff}"
        );
    }
}

#[test]
fn print_time_round_trips_through_host_secs_with_regression_drift() {
    let (mut router, _) = make_router();
    let mcu = router.claim_mcu("mcu");
    let nominal = 400_000_000.0;
    let regression = 400_000_400.0; // 1 ppm high — must not be conflated with nominal.
    router.set_clock_est(mcu, regression, 10.0, 0).unwrap();
    router.set_nominal_freq(mcu, nominal).unwrap();

    for pt in [0.0_f64, 1.0, 10.5, 123.456] {
        let host = router.print_time_to_host_secs(mcu, pt).unwrap();
        let back = router
            .print_time_at_host(mcu, HostSecs::from_anchor_frame(host))
            .unwrap();
        assert!(
            (back.get() - pt).abs() < 1e-6,
            "round trip must recover print_time through the drifted regression \
             frequency, not the nominal one: pt={pt} back={}",
            back.get()
        );
    }
}
