use super::{ClockEstimate, ClockSyncEstimator, DecayRegression};

const MCU_FREQ: f64 = 400e6;
// Mirror of clocksync.py module constants.
const DECAY: f64 = 1.0 / 30.0;
const RTT_AGE: f64 = 0.000_010 / (60.0 * 60.0);
const SYNC_STABLE_FREQ_PPM: f64 = 5e-6;
const SYNC_STABLE_SAMPLES: u32 = 3;

fn make_estimator() -> ClockSyncEstimator {
    let mut est =
        ClockSyncEstimator::new(DECAY, RTT_AGE, SYNC_STABLE_FREQ_PPM, SYNC_STABLE_SAMPLES);
    est.set_last_clock(0);
    est.set_clock_avg(0.0);
    est.set_time_avg(0.0);
    est.set_prediction_variance((0.001 * MCU_FREQ).powi(2));
    est
}

#[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
fn raw_low(clock: f64) -> u32 {
    (clock as u64 & 0xFFFF_FFFF) as u32
}

/// One `clock = freq * sent_time` sample, tracking the published frequency the
/// way `ClockSync` does (`prev_freq` seeded from `mcu_freq`).
fn feed(
    est: &mut ClockSyncEstimator,
    prev_freq: &mut f64,
    sent_time: f64,
    clock: f64,
) -> Option<ClockEstimate> {
    let out = est.handle_clock(
        raw_low(clock),
        sent_time,
        sent_time + 0.0001,
        MCU_FREQ,
        *prev_freq,
    );
    if let Some(e) = out {
        *prev_freq = e.freq;
    }
    out
}

fn feed_exact(
    est: &mut ClockSyncEstimator,
    prev_freq: &mut f64,
    sent_time: f64,
) -> Option<ClockEstimate> {
    feed(est, prev_freq, sent_time, MCU_FREQ * sent_time)
}

#[test]
fn decay_regression_recovers_line_slope() {
    let mut reg = DecayRegression::new(1.0 / 20.0);
    reg.reset(0.0, 7.0);
    for i in 1..500 {
        let x = f64::from(i);
        reg.update(x, 3.0 * x + 7.0);
    }
    let slope = reg.xy_covariance() / reg.x_variance();
    assert!((slope - 3.0).abs() < 1e-6, "slope={slope}");
}

#[test]
fn estimator_converges_to_freq_and_offset() {
    let mut est = make_estimator();
    let mut prev_freq = MCU_FREQ;
    let mut last = None;
    for i in 0..40 {
        last = feed_exact(&mut est, &mut prev_freq, 1.0 + f64::from(i));
    }
    let est_out = last.expect("estimate published");
    assert!(
        (est_out.freq - MCU_FREQ).abs() < 1.0,
        "freq={} expected~{MCU_FREQ}",
        est_out.freq
    );
    assert!(est_out.offset > 0.0, "offset={}", est_out.offset);
    assert!(est.synced(), "should latch synced on a stable stream");
}

#[test]
fn estimator_latches_after_stable_samples() {
    let mut est = make_estimator();
    let mut prev_freq = MCU_FREQ;
    for i in 0..SYNC_STABLE_SAMPLES {
        assert!(!est.synced());
        feed_exact(&mut est, &mut prev_freq, 1.0 + f64::from(i));
    }
    assert!(est.synced());
}

#[test]
fn unstable_freq_resets_stability_count() {
    let mut est = make_estimator();
    let mut prev_freq = MCU_FREQ;
    feed_exact(&mut est, &mut prev_freq, 1.0);
    feed_exact(&mut est, &mut prev_freq, 2.0);
    let drift = 200e-6 * MCU_FREQ;
    for t in [3.0_f64, 4.0] {
        feed(&mut est, &mut prev_freq, t, MCU_FREQ * t + drift * t);
    }
    assert!(!est.synced());
}
