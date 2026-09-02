use std::sync::Arc;
use std::time::Duration;

use host_rt::clock::{Clock, MockClock, instant_to_f64, monotonic_raw_secs};
use host_rt::passthrough_queue::{McuHandle, PassthroughRouter, RouterError};
use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;

/// Every projection the router publishes is `last_clock + (host -
/// clock_offset) * clock_freq`, so a recovered value is only ever exact to the
/// tick the truncation drops, plus f64 rounding on operands of this magnitude.
const TRUNCATION_TICKS: f64 = 1.0;
/// `Duration` — and so `MockClock::advance` — resolves to the nanosecond, and
/// the record's stamps make one round trip through it each.
const MOCK_ADVANCE_QUANTUM_SECS: f64 = 2e-9;

fn make_router() -> (PassthroughRouter, Arc<MockClock>) {
    let clock = MockClock::new();
    let router = PassthroughRouter::with_clock(Arc::clone(&clock) as Arc<dyn Clock + Send + Sync>);
    (router, clock)
}

fn rounding_ticks(freq: f64, magnitudes: [f64; 2]) -> f64 {
    16.0 * f64::EPSILON * (magnitudes[0].abs() + magnitudes[1].abs()) * freq
}

fn arb_freq() -> impl Strategy<Value = f64> {
    prop_oneof![
        prop::sample::select(vec![1e6, 16e6, 72e6, 180e6, 400e6, 520e6]),
        1e5f64..6e8,
    ]
}

/// Uptimes from a fresh boot to eleven days at 520 MHz.
fn arb_last_clock() -> impl Strategy<Value = u64> {
    prop_oneof![Just(0u64), 0u64..1_000_000, 0u64..500_000_000_000_000]
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 512,
        failure_persistence: Some(Box::new(FileFailurePersistence::Direct(
            "proptest-regressions/router_clock_fuzz.txt",
        ))),
        ..ProptestConfig::default()
    })]

    /// Tick to host instant and back is the identity up to the tick the
    /// projection truncates.
    #[test]
    fn a_tick_survives_the_round_trip_through_host_time(
        freq in arb_freq(),
        last_clock in arb_last_clock(),
        centroid_lag in 0.0f64..30.0,
        elapsed in 0.0f64..30.0,
        tick_lead_secs in 0.0f64..30.0,
    ) {
        let (mut router, clock) = make_router();
        let mcu = router.claim_mcu("mcu");
        let anchor = instant_to_f64(clock.now());
        router
            .set_clock_est(mcu, freq, anchor - centroid_lag, last_clock)
            .unwrap();
        clock.advance(Duration::from_secs_f64(elapsed));

        let host_now = router.host_now_secs();
        let projected = router.compute_ack_clock(mcu).unwrap();
        prop_assert!(
            projected >= last_clock,
            "the ack clock may never fall behind the last acked tick {last_clock}"
        );

        let recovered_host = router.clock_to_host_secs(mcu, projected).unwrap();
        let slack = TRUNCATION_TICKS + rounding_ticks(freq, [host_now, recovered_host]);
        prop_assert!(
            (recovered_host - host_now).abs() * freq <= slack,
            "projecting {host_now} to tick {projected} and back gave {recovered_host}"
        );

        let target = last_clock + (tick_lead_secs * freq) as u64;
        let target_host = router.clock_to_host_secs(mcu, target).unwrap();
        let round_tripped = router.host_time_to_mcu_clock(mcu, target_host).unwrap();
        prop_assert!(
            (round_tripped as f64 - target as f64).abs()
                <= TRUNCATION_TICKS + rounding_ticks(freq, [target_host, anchor]),
            "tick {target} became host {target_host} and came back as {round_tripped}"
        );
    }

    /// `compute_ack_clock`, `ack_clock_and_freq` and `clock_record` each carry
    /// their own copy of the projection: one host instant must give one tick.
    #[test]
    fn every_projection_of_one_instant_agrees_exactly(
        freq in arb_freq(),
        last_clock in arb_last_clock(),
        centroid_lag in 0.0f64..30.0,
        elapsed in 0.0f64..30.0,
    ) {
        let (mut router, clock) = make_router();
        let mcu = router.claim_mcu("mcu");
        let anchor = instant_to_f64(clock.now());
        router
            .set_clock_est(mcu, freq, anchor - centroid_lag, last_clock)
            .unwrap();
        router.set_nominal_freq(mcu, freq).unwrap();
        clock.advance(Duration::from_secs_f64(elapsed));

        let host_now = router.host_now_secs();
        let ack = router.compute_ack_clock(mcu).unwrap();
        let (ack_paired, paired_freq) = router.ack_clock_and_freq(mcu).unwrap();
        let record = router.clock_record(mcu).expect("a record was just set");
        let by_host_time = router.host_time_to_mcu_clock(mcu, host_now).unwrap();

        prop_assert_eq!(ack, ack_paired, "ack_clock_and_freq must project like compute_ack_clock");
        prop_assert_eq!(paired_freq, freq);
        prop_assert_eq!(ack, record.projected_now, "clock_record must project like compute_ack_clock");
        prop_assert_eq!(ack, by_host_time, "host_time_to_mcu_clock at now must project like compute_ack_clock");
        prop_assert_eq!(record.last_clock, last_clock);
        prop_assert_eq!(record.clock_freq, freq);
        let stamp_slack = MOCK_ADVANCE_QUANTUM_SECS
            + 16.0 * f64::EPSILON * (anchor.abs() + centroid_lag + elapsed);
        prop_assert!(
            (record.age_secs - elapsed).abs() <= stamp_slack,
            "a record set before {elapsed} s of mock time must read {elapsed} s old, not {}",
            record.age_secs
        );
        prop_assert!(
            (record.centroid_lag_secs - (centroid_lag + elapsed)).abs() <= stamp_slack,
            "centroid lag {} does not match {} + {elapsed}",
            record.centroid_lag_secs,
            centroid_lag
        );

        let print_time = router
            .print_time_at_host(mcu, host_rt::clock::HostSecs::from_anchor_frame(host_now))
            .expect("both frequencies are set");
        let back_to_host = router.print_time_to_host_secs(mcu, print_time.get()).unwrap();
        prop_assert!(
            (back_to_host - host_now).abs() * freq
                <= rounding_ticks(freq, [host_now, print_time.get()]),
            "print_time round trip moved {host_now} to {back_to_host}"
        );
    }

    /// `wall_time_at_mcu` shifts the wall clock by exactly the host-instant
    /// distance the record puts between now and the tick.
    #[test]
    fn wall_time_tracks_the_host_instant_of_the_tick(
        freq in arb_freq(),
        last_clock in arb_last_clock(),
        centroid_lag in 0.0f64..30.0,
        tick_offset_secs in prop_oneof![
            Just(-1.0),
            Just(1.0),
            -3.0f64..3.0,
            -1e5f64..1e5,
        ],
    ) {
        let (mut router, clock) = make_router();
        let mcu = router.claim_mcu("mcu");
        let anchor = instant_to_f64(clock.now());
        router
            .set_clock_est(mcu, freq, anchor - centroid_lag, last_clock)
            .unwrap();

        let ticks = (last_clock as f64 + tick_offset_secs * freq).max(0.0) as u64;
        let tick_host = router.clock_to_host_secs(mcu, ticks).unwrap();
        let shift_secs = tick_host - router.host_now_secs();

        let before = std::time::SystemTime::now();
        let (wall, estimated) = router
            .wall_time_at_mcu(mcu, ticks)
            .expect("a record was just set");
        let after = std::time::SystemTime::now();

        let delta_ticks = (ticks as f64) - (last_clock as f64);
        prop_assert_eq!(
            estimated,
            delta_ticks.abs() / freq > 1.0,
            "the estimated flag must mark ticks more than a frequency-second from the anchor"
        );

        let shift_nanos = (shift_secs * 1e9) as i128;
        let slack_nanos = 4 + (shift_nanos.abs() as f64 * f64::EPSILON) as i128;
        let low = time::OffsetDateTime::from(before).unix_timestamp_nanos() + shift_nanos
            - slack_nanos;
        let high = time::OffsetDateTime::from(after).unix_timestamp_nanos() + shift_nanos
            + slack_nanos;
        let got = wall.unix_timestamp_nanos();
        prop_assert!(
            got >= low && got <= high,
            "wall time {got} for a {shift_secs} s shift is outside [{low}, {high}]"
        );
    }

    /// `set_clock_est_from_sample` anchors on a send `Instant` where
    /// `set_clock_est` anchors on the same instant in seconds: one record, two
    /// spellings, so they must project the same tick.
    #[test]
    fn the_sample_setter_anchors_where_the_offset_setter_does(
        freq in arb_freq(),
        mcu_at_send in arb_last_clock(),
        elapsed in 0.0f64..30.0,
    ) {
        let clock = MockClock::new();
        let shared = Arc::clone(&clock) as Arc<dyn Clock + Send + Sync>;
        let mut from_sample = PassthroughRouter::with_clock(Arc::clone(&shared));
        let sample_mcu = from_sample.claim_mcu("from_sample");
        let mut from_offset = PassthroughRouter::with_clock(shared);
        let offset_mcu = from_offset.claim_mcu("from_offset");

        let send_instant = clock.now();
        from_sample
            .set_clock_est_from_sample(sample_mcu, freq, send_instant, mcu_at_send)
            .unwrap();
        from_offset
            .set_clock_est(offset_mcu, freq, instant_to_f64(send_instant), mcu_at_send)
            .unwrap();
        clock.advance(Duration::from_secs_f64(elapsed));

        prop_assert_eq!(
            from_sample.compute_ack_clock(sample_mcu).unwrap(),
            from_offset.compute_ack_clock(offset_mcu).unwrap(),
            "the two spellings of one anchor must project the same tick"
        );
        prop_assert_eq!(
            from_sample.clock_to_host_secs(sample_mcu, mcu_at_send),
            from_offset.clock_to_host_secs(offset_mcu, mcu_at_send)
        );
        prop_assert!(
            from_sample
                .clock_est_converged(sample_mcu)
                .eq(&from_offset.clock_est_converged(offset_mcu)),
            "both setters publish an already-converged record"
        );
    }

    /// The rebased setter differs from the plain one only by the
    /// CLOCK_MONOTONIC_RAW to `Instant` frame change, and `host_now_raw` — the
    /// publisher's capture stamp — must not reach the projection anchor at all.
    #[test]
    fn rebasing_only_changes_the_time_frame_not_the_projection(
        freq in arb_freq(),
        last_clock in arb_last_clock(),
        centroid_lag in 0.0f64..30.0,
        publish_lag in 0.0f64..0.5,
    ) {
        let (mut plain, plain_clock) = make_router();
        let plain_mcu = plain.claim_mcu("plain");
        let (mut rebased, _rebased_clock) = make_router();
        let rebased_mcu = rebased.claim_mcu("rebased");
        let (mut stale_stamp, _stale_clock) = make_router();
        let stale_mcu = stale_stamp.claim_mcu("stale");

        let offset_instant = instant_to_f64(plain_clock.now()) - centroid_lag;
        let raw_before = monotonic_raw_secs();
        let raw_at_anchor = raw_before - instant_to_f64(plain_clock.now());
        let offset_raw = offset_instant + raw_at_anchor;

        plain.set_clock_est(plain_mcu, freq, offset_instant, last_clock).unwrap();
        rebased
            .set_clock_est_rebased(rebased_mcu, freq, offset_raw, last_clock, true, raw_before)
            .unwrap();
        stale_stamp
            .set_clock_est_rebased(
                stale_mcu,
                freq,
                offset_raw,
                last_clock,
                true,
                raw_before - publish_lag,
            )
            .unwrap();
        let raw_after = monotonic_raw_secs();

        let plain_ack = plain.compute_ack_clock(plain_mcu).unwrap();
        let rebased_ack = rebased.compute_ack_clock(rebased_mcu).unwrap();
        let stale_ack = stale_stamp.compute_ack_clock(stale_mcu).unwrap();

        let read_skew_ticks = (raw_after - raw_before) * freq + TRUNCATION_TICKS;
        prop_assert!(
            (rebased_ack as f64 - plain_ack as f64).abs() <= read_skew_ticks,
            "rebased projection {rebased_ack} left the plain projection {plain_ack} by more \
             than the {read_skew_ticks} ticks that elapsed between the RAW clock reads"
        );
        prop_assert!(
            (stale_ack as f64 - rebased_ack as f64).abs() <= read_skew_ticks,
            "a capture stamp {publish_lag} s older moved the projection from {rebased_ack} \
             to {stale_ack}: host_now_raw leaked into the anchor"
        );
        prop_assert!(
            stale_stamp
                .clock_record(stale_mcu)
                .expect("the record was accepted")
                .age_secs
                >= publish_lag - (raw_after - raw_before),
            "an estimate that spent {publish_lag} s in transit must read at least that old"
        );
    }

    /// A capture stamp ahead of the router's own RAW read means the publisher is
    /// not in the RAW domain: the record's age is unmeasurable, so it is refused
    /// rather than guessed.
    #[test]
    fn a_capture_stamp_from_the_future_is_refused(
        freq in arb_freq(),
        last_clock in arb_last_clock(),
        skew_secs in 1e-3f64..10.0,
    ) {
        let (mut router, clock) = make_router();
        let mcu = router.claim_mcu("mcu");
        let now_raw = monotonic_raw_secs();
        let offset_raw = now_raw - instant_to_f64(clock.now());

        let refused = router.set_clock_est_rebased(
            mcu,
            freq,
            offset_raw,
            last_clock,
            true,
            now_raw + skew_secs,
        );

        prop_assert!(
            matches!(refused, Err(RouterError::ClockEstStampAhead { .. })),
            "a stamp {skew_secs} s ahead must be refused, got {refused:?}"
        );
        prop_assert!(
            router.clock_record(mcu).is_none(),
            "a refused estimate must leave no record behind"
        );
        prop_assert!(matches!(
            router.compute_ack_clock(mcu),
            Err(RouterError::NoClockEstimate(_))
        ));
    }

    /// A released or never-claimed handle has no record to project from, and
    /// says so instead of answering with someone else's numbers.
    #[test]
    fn a_handle_without_a_record_refuses_to_project(
        freq in arb_freq(),
        last_clock in arb_last_clock(),
        stranger in any::<u32>(),
    ) {
        let (mut router, clock) = make_router();
        let mcu = router.claim_mcu("mcu");
        router
            .set_clock_est(mcu, freq, instant_to_f64(clock.now()), last_clock)
            .unwrap();
        prop_assume!(McuHandle::from_raw(stranger) != mcu);

        prop_assert!(matches!(
            router.compute_ack_clock(McuHandle::from_raw(stranger)),
            Err(RouterError::UnknownMcu(_))
        ));
        prop_assert!(router.ack_clock_and_freq(McuHandle::from_raw(stranger)).is_none());
        prop_assert!(router.wall_time_at_mcu(McuHandle::from_raw(stranger), last_clock).is_none());
        prop_assert!(router.clock_record(McuHandle::from_raw(stranger)).is_none());

        router.invalidate_clock_est(mcu).unwrap();
        prop_assert!(matches!(
            router.compute_ack_clock(mcu),
            Err(RouterError::NoClockEstimate(_))
        ));
        prop_assert!(router.wall_time_at_mcu(mcu, last_clock).is_none());
        prop_assert!(router.clock_to_host_secs(mcu, last_clock).is_none());
    }
}
