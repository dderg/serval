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

    router.release_mcu(a);
    assert!(router.compute_ack_clock(b).is_ok());
    assert!(router.compute_ack_clock(a).is_err());
}

#[test]
fn set_clock_est_stores_values() {
    let (mut router, _) = make_router();
    let mcu = router.claim_mcu("mcu");

    assert_eq!(router.compute_ack_clock(mcu).unwrap(), 0);

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
        .set_clock_est_rebased(mcu, 1_000_000.0, offset_raw, 10_000_000, 0.0)
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
        .set_clock_est_rebased(mcu_a, freq, offset_raw, last_clock, 1000.0)
        .unwrap();

    router_b
        .set_clock_est_rebased(mcu_b, freq, offset_raw, last_clock, 1000.0 - 0.050)
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
    let freq = 48_000_000.0;
    let offset = 5.0;
    let last_clock = 1_000_000u64;
    router.set_clock_est(mcu, freq, offset, last_clock).unwrap();
    let glmt_print_time = 7.5;
    let host = router
        .print_time_to_host_secs(mcu, glmt_print_time)
        .unwrap();
    let clock = router.host_time_to_mcu_clock(mcu, host).unwrap();
    assert_eq!(clock, (glmt_print_time * freq) as u64);
}
