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

#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss
)]
fn seeded(last_clock: u64, clock_avg: f64, time_avg: f64) -> ClockSyncEstimator {
    let mut est =
        ClockSyncEstimator::new(DECAY, RTT_AGE, SYNC_STABLE_FREQ_PPM, SYNC_STABLE_SAMPLES);
    est.set_last_clock(last_clock);
    est.set_clock_avg(clock_avg);
    est.set_time_avg(time_avg);
    est.set_prediction_variance((0.001 * MCU_FREQ).powi(2));
    est
}

/// `ClockSync.connect` seeds `last_clock`/`clock_avg` from `get_uptime` and then
/// drives an 8-sample priming loop; convergence must latch inside it exactly as
/// the pre-refactor Python did, even with the large 64-bit clock seed.
#[test]
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn connect_priming_with_seeded_clock_latches() {
    let t0 = 1000.0_f64;
    let c0 = (MCU_FREQ * t0) as u64;
    let mut est = seeded(c0, c0 as f64, t0);
    let mut prev_freq = MCU_FREQ;
    let mut t = t0;
    for _ in 0..8 {
        t += 0.05;
        let clock = c0 + (MCU_FREQ * (t - t0)) as u64;
        if let Some(e) = est.handle_clock(
            (clock & 0xFFFF_FFFF) as u32,
            t,
            t + 0.0002,
            MCU_FREQ,
            prev_freq,
        ) {
            prev_freq = e.freq;
        }
    }
    assert!(
        est.synced(),
        "connect() priming loop must latch, count={}",
        est.sync_stable_count()
    );
}

/// A sample stream that carries the low 32 bits across a `2^32` boundary must
/// reconstruct the full 64-bit clock and keep advancing the latch.
#[test]
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn wraparound_reconstruction_advances_latch() {
    let c0 = (1u64 << 32) - (MCU_FREQ * 0.4) as u64;
    let mut est = seeded(c0, c0 as f64, 0.0);
    let mut prev_freq = MCU_FREQ;
    let mut t = 0.0_f64;
    let mut full = c0;
    for _ in 0..8 {
        t += 0.1;
        full = c0 + (MCU_FREQ * t) as u64;
        if let Some(e) = est.handle_clock(
            (full & 0xFFFF_FFFF) as u32,
            t,
            t + 0.0002,
            MCU_FREQ,
            prev_freq,
        ) {
            prev_freq = e.freq;
        }
    }
    assert_eq!(
        est.last_clock(),
        full,
        "32-bit low word must reconstruct the 64-bit clock across the wrap"
    );
    assert!(est.synced());
}

/// A periodic `clock` sample that arrived without engine wire stamps reaches the
/// estimator with `sent_time == 0` (serialhdl drops it that way). It must update
/// `last_clock`, return `None`, and leave the stability count untouched so the
/// latch resumes across the gap — matching `if not sent_time: return`.
#[test]
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn zero_sent_time_sample_is_dropped_without_touching_latch() {
    let mut est = make_estimator();
    let mut prev_freq = MCU_FREQ;
    feed_exact(&mut est, &mut prev_freq, 1.0);
    let count = est.sync_stable_count();
    let dropped = est.handle_clock(raw_low(MCU_FREQ * 1.05), 0.0, 0.0, MCU_FREQ, prev_freq);
    assert!(dropped.is_none(), "unstamped clock sample must be dropped");
    assert_eq!(
        est.last_clock(),
        (MCU_FREQ * 1.05) as u64,
        "last_clock still advances on a dropped sample"
    );
    assert_eq!(
        est.sync_stable_count(),
        count,
        "a dropped sample must not disturb the stability count"
    );
    feed_exact(&mut est, &mut prev_freq, 2.0);
    feed_exact(&mut est, &mut prev_freq, 3.0);
    assert!(est.synced(), "latch resumes across the dropped sample");
}
