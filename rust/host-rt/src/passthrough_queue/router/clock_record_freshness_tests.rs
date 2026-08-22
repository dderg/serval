//! Freshness of the clock record, which is a different quantity from the
//! regression's centroid lag. `clock_offset` is the decay-weighted sample
//! centroid and legitimately trails now by up to `1/decay` get_clock periods on
//! a perfectly live record; only `age_secs` counts the samples the router
//! actually missed. Reading the centroid lag as staleness (or the other way
//! round) is exactly the misdiagnosis these tests pin down.

use super::*;
use crate::clock::MockClock;
use crate::clock_regression::NON_RESONANT_GET_CLOCK_PERIOD_SECS;
use std::time::Duration;

const FREQ: f64 = 168_000_000.0;
const DECAY_WINDOW_SAMPLES: u32 = 30;

fn router() -> (PassthroughRouter, Arc<MockClock>) {
    let clock = MockClock::new();
    let router = PassthroughRouter::with_clock(Arc::clone(&clock) as Arc<dyn Clock + Send + Sync>);
    (router, clock)
}

fn publish_with_centroid_lag(
    router: &mut PassthroughRouter,
    mcu: McuHandle,
    last_clock: u64,
    centroid_lag_secs: f64,
) {
    let offset_raw = crate::clock::monotonic_raw_secs() - centroid_lag_secs;
    router
        .set_clock_est_rebased(mcu, FREQ, offset_raw, last_clock, true, 0.0)
        .unwrap();
}

#[test]
fn a_record_updated_this_instant_has_no_age() {
    let (mut router, _clock) = router();
    let mcu = router.claim_mcu("mcu");

    publish_with_centroid_lag(&mut router, mcu, 1_000, 0.0);

    let record = router.clock_record(mcu).expect("record present");
    assert!(
        record.age_secs.abs() < 0.001,
        "a just-published record must have zero age, got {}",
        record.age_secs
    );
}

/// The whole point of a separate `updated_at`: a live regression's centroid sits
/// half a decay window in the past, so the record that anchored the failing G28
/// (centroid 13.6 s back, updated microseconds ago) must read as fresh.
#[test]
fn a_deep_centroid_lag_on_a_live_record_is_not_age() {
    let (mut router, _clock) = router();
    let mcu = router.claim_mcu("mcu");
    let centroid_lag = f64::from(DECAY_WINDOW_SAMPLES) * NON_RESONANT_GET_CLOCK_PERIOD_SECS;

    publish_with_centroid_lag(&mut router, mcu, 1_000, centroid_lag);

    let record = router.clock_record(mcu).expect("record present");
    assert!(
        (record.centroid_lag_secs - centroid_lag).abs() < 0.01,
        "centroid lag must be reported as measured, got {}",
        record.centroid_lag_secs
    );
    assert!(
        record.age_secs < MAX_CLOCK_RECORD_AGE_SECS,
        "a record published now must be fresh however far back its centroid \
         sits, got age {} against limit {}",
        record.age_secs,
        MAX_CLOCK_RECORD_AGE_SECS
    );
}

#[test]
fn age_grows_with_the_host_clock_and_resets_on_every_accepted_estimate() {
    let (mut router, clock) = router();
    let mcu = router.claim_mcu("mcu");
    publish_with_centroid_lag(&mut router, mcu, 1_000, 0.0);

    clock.advance(Duration::from_secs_f64(2.5));
    let stale = router.clock_record(mcu).expect("record present");
    assert!(
        (stale.age_secs - 2.5).abs() < 0.01,
        "age must track the host clock, got {}",
        stale.age_secs
    );

    publish_with_centroid_lag(&mut router, mcu, 2_000, 0.0);
    let refreshed = router.clock_record(mcu).expect("record present");
    assert!(
        refreshed.age_secs.abs() < 0.001,
        "a new estimate must reset the age, got {}",
        refreshed.age_secs
    );
}

/// Every sample of a whole connection's worth of estimates has to land: a
/// callback that fires once and dies is the failure mode this guards.
#[test]
fn a_connections_worth_of_estimates_each_refresh_the_record() {
    let (mut router, clock) = router();
    let mcu = router.claim_mcu("mcu");
    let period = NON_RESONANT_GET_CLOCK_PERIOD_SECS;

    for sample in 1..=60u64 {
        clock.advance(Duration::from_secs_f64(period));
        publish_with_centroid_lag(&mut router, mcu, sample * 1_000, period * 15.0);
        let record = router.clock_record(mcu).expect("record present");
        assert!(
            record.age_secs < MAX_CLOCK_RECORD_AGE_SECS,
            "sample {sample} left the record {}s old",
            record.age_secs
        );
        assert_eq!(record.last_clock, sample * 1_000);
    }
}

#[test]
fn invalidation_removes_the_record_rather_than_ageing_it() {
    let (mut router, clock) = router();
    let mcu = router.claim_mcu("mcu");
    publish_with_centroid_lag(&mut router, mcu, 1_000, 0.0);
    clock.advance(Duration::from_secs_f64(10.0));

    router.invalidate_clock_est(mcu).unwrap();

    assert!(router.clock_record(mcu).is_none());
}
