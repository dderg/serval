//! The clock record's boot-epoch lifecycle: a reflash/reconnect must drop the
//! previous epoch's numbers, and every projection must fail loudly until a
//! fresh converged estimate arrives.

use super::*;
use crate::clock::MockClock;

const FREQ: f64 = 168_000_000.0;

fn router() -> (PassthroughRouter, Arc<MockClock>) {
    let clock = MockClock::new();
    let router = PassthroughRouter::with_clock(Arc::clone(&clock) as Arc<dyn Clock + Send + Sync>);
    (router, clock)
}

/// A rebased estimate needs its offset in the same CLOCK_MONOTONIC_RAW epoch
/// the router reads inside `set_clock_est_rebased`.
fn publish(router: &mut PassthroughRouter, mcu: McuHandle, last_clock: u64, converged: bool) {
    let offset_raw = crate::clock::monotonic_raw_secs();
    router
        .set_clock_est_rebased(mcu, FREQ, offset_raw, last_clock, converged, 0.0)
        .unwrap();
}

/// A reflash restarts the MCU's tick counter at zero while the host still
/// holds the record from the previous boot epoch. That record projects clocks
/// ahead of the true MCU clock by the previous boot's uptime — 14.4 s here,
/// past the 2^31-tick ambiguity horizon of the MCU's uint32 clock domain.
#[test]
fn reconnect_invalidates_the_previous_boot_epochs_record() {
    let (mut router, clock) = router();
    let mcu = router.claim_mcu("mcu");
    let host_at_connect = instant_to_f64(clock.now());
    let previous_boot_uptime_ticks = (14.4 * FREQ) as u64;
    router
        .set_clock_est(mcu, FREQ, host_at_connect, previous_boot_uptime_ticks)
        .unwrap();

    let stale = router
        .host_time_to_mcu_clock(mcu, host_at_connect)
        .expect("the stale record projects happily");
    assert_eq!(
        stale, previous_boot_uptime_ticks,
        "the previous epoch's record projects the previous boot's uptime"
    );

    router.invalidate_clock_est(mcu).unwrap();

    assert!(matches!(
        router.host_time_to_mcu_clock(mcu, host_at_connect),
        Err(RouterError::NoClockEstimate(_))
    ));
    assert!(matches!(
        router.compute_ack_clock(mcu),
        Err(RouterError::NoClockEstimate(_))
    ));
    assert!(router.ack_clock_and_freq(mcu).is_none());
    assert!(router.clock_record(mcu).is_none());
    assert!(!router.clock_est_converged(mcu));
    assert!(router.clock_to_host_secs(mcu, 1_000).is_none());
    assert!(router.wall_time_at_mcu(mcu, 1_000).is_none());
}

/// The handle survives the reconnect (klippy re-identifies the same MCU
/// object), so invalidation must be keyed on the record, not the handle.
#[test]
fn a_fresh_estimate_after_invalidation_projects_the_new_boot_epoch() {
    let (mut router, clock) = router();
    let mcu = router.claim_mcu("mcu");
    let host_at_connect = instant_to_f64(clock.now());
    router
        .set_clock_est(mcu, FREQ, host_at_connect, (14.4 * FREQ) as u64)
        .unwrap();
    router.invalidate_clock_est(mcu).unwrap();

    let new_epoch_clock = (0.2 * FREQ) as u64;
    publish(&mut router, mcu, new_epoch_clock, true);

    assert!(router.clock_est_converged(mcu));
    let record = router.clock_record(mcu).expect("record re-armed");
    assert_eq!(record.last_clock, new_epoch_clock);
    assert_eq!(record.clock_freq, FREQ);
    let projected = router.compute_ack_clock(mcu).unwrap();
    assert!(
        projected.abs_diff(new_epoch_clock) < (FREQ * 0.050) as u64,
        "the fresh record must project the new boot epoch's clock, got {projected} \
         against {new_epoch_clock}"
    );
    assert_eq!(record.projected_now, projected);
}

/// The estimate published before the regression latches convergence seeds the
/// record (log timestamping needs it) but must not enable anchoring.
#[test]
fn an_unconverged_estimate_leaves_the_record_unconverged() {
    let (mut router, _clock) = router();
    let mcu = router.claim_mcu("mcu");
    publish(&mut router, mcu, (0.2 * FREQ) as u64, false);

    assert!(!router.clock_est_converged(mcu));
    let record = router.clock_record(mcu).expect("record present");
    assert!(!record.converged);
    assert!(
        router.compute_ack_clock(mcu).is_ok(),
        "an unconverged record still projects — only anchoring is gated"
    );

    publish(&mut router, mcu, (1.2 * FREQ) as u64, true);
    assert!(router.clock_est_converged(mcu));
}

/// A second reconnect after a healthy epoch must invalidate again — the record
/// is never sticky.
#[test]
fn invalidation_is_repeatable_across_epochs() {
    let (mut router, _clock) = router();
    let mcu = router.claim_mcu("mcu");
    publish(&mut router, mcu, (1.0 * FREQ) as u64, true);
    assert!(router.clock_est_converged(mcu));

    router.invalidate_clock_est(mcu).unwrap();
    assert!(!router.clock_est_converged(mcu));

    publish(&mut router, mcu, (0.1 * FREQ) as u64, true);
    router.invalidate_clock_est(mcu).unwrap();
    assert!(matches!(
        router.compute_ack_clock(mcu),
        Err(RouterError::NoClockEstimate(_))
    ));
}

#[test]
fn invalidating_an_unclaimed_mcu_errors() {
    let (mut router, _clock) = router();
    assert!(matches!(
        router.invalidate_clock_est(McuHandle::from_raw(42)),
        Err(RouterError::UnknownMcu(_))
    ));
}
