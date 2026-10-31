use host_rt::clock_regression::{
    CLOCK_REGRESSION_DECAY, ClockEstimate, ClockSyncEstimator, DecayRegression,
};
use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;

/// klippy `clocksync.RTT_AGE`.
const RTT_AGE: f64 = 0.000_010 / (60.0 * 60.0);
/// `danger_options.clock_sync_stable_ppm` default, scaled to a fraction.
const SYNC_STABLE_FREQ_PPM: f64 = 5e-6;
/// klippy `clocksync.SYNC_STABLE_SAMPLES`.
const SYNC_STABLE_SAMPLES: u32 = 3;

const MCU_FREQS: [f64; 5] = [1e6, 16e6, 72e6, 180e6, 520e6];

/// Below this retained weight `(1-decay)^n`, the `x - x_avg` residual the
/// recurrence squares is smaller than the rounding of `x_avg` itself and the
/// closed form stops being checkable in f64. Bounding the sample count by it
/// keeps `VARIANCE_REL_TOL` a statement about the algebra, not about f64.
const RETAINED_WEIGHT_FLOOR: f64 = 1e-5;
const AVERAGE_TOL_PER_SCALE: f64 = 1e-13;
const VARIANCE_REL_TOL: f64 = 1e-9;
const SLOPE_REL_TOL: f64 = 1e-6;

const SAMPLES: usize = 160;
const WARMUP_SAMPLES: usize = 60;
const FREQ_TOL_PPM: f64 = 200e-6;
const PREDICTION_TOL_SECS: f64 = 1e-3;
/// One in this many `clock` responses reaches the estimator with no wire
/// stamps, the way serialhdl delivers a sample it could not time.
const UNSTAMPED_ONE_IN: u64 = 32;
/// One in this many samples arrives after a deferred `get_clock` timer: a long
/// enough stall loses whole `2^32` wraps, which only the expected-clock
/// correction can put back.
const STALL_ONE_IN: u64 = 16;

struct SplitMix64(u64);

impl SplitMix64 {
    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
}

/// A drift-free MCU counter: `true_freq` ticks per host second, reading
/// `clock_at_epoch` at host instant `epoch_host`.
struct SyntheticMcu {
    true_freq: f64,
    clock_at_epoch: u64,
    epoch_host: f64,
}

impl SyntheticMcu {
    fn latch(&self, host_secs: f64) -> u64 {
        self.clock_at_epoch + (self.true_freq * (host_secs - self.epoch_host)) as u64
    }
}

/// Seeded the way `ClockSync.connect` seeds it off the `get_uptime` response.
fn primed_estimator(mcu: &SyntheticMcu, nominal_freq: f64) -> ClockSyncEstimator {
    let mut est = ClockSyncEstimator::new(
        CLOCK_REGRESSION_DECAY,
        RTT_AGE,
        SYNC_STABLE_FREQ_PPM,
        SYNC_STABLE_SAMPLES,
    );
    est.set_last_clock(mcu.clock_at_epoch);
    est.set_clock_avg(mcu.clock_at_epoch as f64);
    est.set_time_avg(mcu.epoch_host);
    est.set_prediction_variance((0.001 * nominal_freq).powi(2));
    est
}

/// The largest `n <= requested` whose retained weight stays above the f64 floor,
/// with that weight `(1-decay)^n`.
fn bounded_updates(decay: f64, requested: usize) -> (usize, f64) {
    let aging = 1.0 - decay;
    let mut weight = 1.0;
    let mut updates = 0;
    while updates < requested && weight * aging >= RETAINED_WEIGHT_FLOOR {
        weight *= aging;
        updates += 1;
    }
    (updates, weight)
}

/// `_synced` / `_sync_stable_count` replayed from the published frequencies
/// alone: the latch is a run-length counter over consecutive samples whose
/// published frequency moved less than `ppm * mcu_freq`.
fn replay_latch(published: &[(f64, f64)], ppm: f64, mcu_freq: f64) -> (u32, bool) {
    let mut count = 0u32;
    let mut synced = false;
    for (new_freq, prev_freq) in published {
        if synced {
            break;
        }
        if (new_freq - prev_freq).abs() <= ppm * mcu_freq {
            count += 1;
            if count >= SYNC_STABLE_SAMPLES {
                synced = true;
            }
        } else {
            count = 0;
        }
    }
    (count, synced)
}

#[derive(Debug, Clone, Copy)]
struct Sample {
    sent: f64,
    received: f64,
    truth: u64,
}

#[derive(Debug, Clone, Copy)]
struct Outcome {
    published: Option<ClockEstimate>,
    last_clock: u64,
}

/// Feed a whole sample stream the way `ClockSync` does, threading the published
/// frequency back in as `clock_est[2]`.
fn run_stream(est: &mut ClockSyncEstimator, stream: &[Sample], nominal_freq: f64) -> Vec<Outcome> {
    let mut prev_freq = nominal_freq;
    stream
        .iter()
        .map(|sample| {
            let published = est.handle_clock(
                sample.truth as u32,
                sample.sent,
                sample.received,
                nominal_freq,
                prev_freq,
            );
            if let Some(estimate) = published {
                prev_freq = estimate.freq;
            }
            Outcome {
                published,
                last_clock: est.last_clock(),
            }
        })
        .collect()
}

fn arb_scale() -> impl Strategy<Value = f64> {
    prop_oneof![
        Just(1.0),
        Just(1e3),
        Just(1e6),
        Just(1e9),
        Just(1e12),
        1.0f64..1e12,
    ]
}

/// A displacement that keeps `|origin| / |step|` inside three decades, so the
/// closed-form check measures the recurrence and not f64 cancellation.
fn arb_step() -> impl Strategy<Value = f64> {
    prop_oneof![-1.0f64..=-1e-3, 1e-3f64..=1.0]
}

fn arb_mcu_freq() -> impl Strategy<Value = f64> {
    prop::sample::select(MCU_FREQS.to_vec())
}

/// The MCU counter at the first sample: either just short of a `2^32` boundary
/// so the wrap lands inside the run at any frequency, or an arbitrary uptime.
fn arb_epoch_clock() -> impl Strategy<Value = u64> {
    prop_oneof![
        (1u64..=4, 0u64..2000).prop_map(|(epoch, back)| (epoch << 32) - back),
        0u64..1_000_000_000_000_000,
    ]
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 512,
        failure_persistence: Some(Box::new(FileFailurePersistence::Direct(
            "proptest-regressions/clock_regression_fuzz.txt",
        ))),
        ..ProptestConfig::default()
    })]

    /// The decay recurrence on a repeated sample has a closed form: the residual
    /// `x - x_avg` ages by `1-decay` per update, so after `n` updates
    /// `x_avg = x - w*(x - x0)` and `x_variance = (x - x0)^2 * w * (1 - w)` with
    /// `w = (1-decay)^n`.
    #[test]
    fn a_repeated_sample_follows_the_closed_form_decay(
        decay in 1.0f64/1024.0..=0.5,
        requested in 1usize..=200,
        scale in arb_scale(),
        x_origin in -1.0f64..=1.0,
        x_step in arb_step(),
        y_origin in -1.0f64..=1.0,
        y_step in arb_step(),
    ) {
        let (x0, y0) = (scale * x_origin, scale * y_origin);
        let (dx, dy) = (scale * x_step, scale * y_step);
        let (x, y) = (x0 + dx, y0 + dy);
        let (updates, weight) = bounded_updates(decay, requested);

        let mut reg = DecayRegression::new(decay);
        reg.reset(x0, y0);
        for _ in 0..updates {
            reg.update(x, y);
        }

        let avg_tol = AVERAGE_TOL_PER_SCALE * scale;
        prop_assert!(
            (reg.x_avg() - (x - weight * dx)).abs() <= avg_tol,
            "x_avg {} != {} after {updates} updates at decay {decay}",
            reg.x_avg(),
            x - weight * dx
        );
        prop_assert!(
            (reg.y_avg() - (y - weight * dy)).abs() <= avg_tol,
            "y_avg {} != {} after {updates} updates at decay {decay}",
            reg.y_avg(),
            y - weight * dy
        );

        let decayed = weight * (1.0 - weight);
        let want_variance = dx * dx * decayed;
        let want_covariance = dx * dy * decayed;
        prop_assert!(
            (reg.x_variance() - want_variance).abs() <= VARIANCE_REL_TOL * want_variance.abs(),
            "x_variance {} != {want_variance} after {updates} updates at decay {decay}",
            reg.x_variance()
        );
        prop_assert!(
            (reg.xy_covariance() - want_covariance).abs()
                <= VARIANCE_REL_TOL * want_covariance.abs(),
            "xy_covariance {} != {want_covariance} after {updates} updates at decay {decay}",
            reg.xy_covariance()
        );

        prop_assert!(
            (reg.x_avg() - x).abs() <= dx.abs() * weight + avg_tol,
            "the averages must converge to the repeated sample as the weight decays"
        );
        prop_assert!(
            reg.x_variance() <= dx * dx * weight + avg_tol,
            "the variance must decay to zero with the retained weight"
        );
    }

    /// Samples on an exact line stay on it: the decay averages are a weighted
    /// mean, so `y_avg = m*x_avg + b` holds at every step and the regressed
    /// slope `xy_covariance / x_variance` is `m` itself.
    #[test]
    fn an_exact_line_regresses_to_its_own_slope(
        slope in arb_mcu_freq(),
        uptime_secs in 0.0f64..=1e5,
        x_start in 0.0f64..=1e5,
        gaps in prop::collection::vec(0.05f64..=10.0, 120..=200),
        decay in prop_oneof![Just(CLOCK_REGRESSION_DECAY), 1.0f64/1024.0..=0.25],
    ) {
        let line = |x: f64| slope * (x + uptime_secs);
        let mut reg = DecayRegression::new(decay);
        reg.reset(x_start, line(x_start));

        let mut x = x_start;
        for gap in &gaps {
            x += gap;
            reg.update(x, line(x));
        }

        prop_assert!(reg.x_variance() > 0.0, "an advancing x must build variance");

        let regressed = reg.xy_covariance() / reg.x_variance();
        prop_assert!(
            (regressed - slope).abs() <= SLOPE_REL_TOL * slope,
            "regressed slope {regressed} is not {slope} (decay {decay})"
        );

        let intercept_tol = 256.0 * f64::EPSILON / decay * line(x).abs();
        prop_assert!(
            (reg.y_avg() - line(reg.x_avg())).abs() <= intercept_tol,
            "y_avg {} left the line at x_avg {} (want {}, tol {intercept_tol})",
            reg.y_avg(),
            reg.x_avg(),
            line(reg.x_avg())
        );
    }

    /// A drift-free synthetic MCU behind a link with a fixed one-way latency and
    /// bounded jitter: the estimator must reconstruct the 64-bit counter exactly
    /// across every `2^32` wrap — including the wraps a stalled reactor loses
    /// whole — publish a frequency inside 200 ppm of the truth after warm-up,
    /// and predict the counter at a future host instant to within a
    /// millisecond. `min_half_rtt` cancels the fixed latency, so the prediction
    /// error is bounded by the jitter alone, not by the latency.
    #[test]
    fn a_jittery_link_converges_to_the_true_clock(
        nominal_freq in arb_mcu_freq(),
        freq_error_ppm in -100.0f64..=100.0,
        clock_at_epoch in arb_epoch_clock(),
        epoch_host in 1.0f64..=1e5,
        period in 0.3f64..=2.0,
        base_half_rtt in prop_oneof![Just(0.0), 1e-6f64..=9e-4],
        jitter_secs in prop_oneof![Just(0.0), 1e-7f64..=1e-4],
        stall_secs in prop_oneof![Just(0.0), 0.0f64..=12.0],
        lever_secs in 0.0f64..=0.5,
        noise_seed in any::<u64>(),
    ) {
        let mcu = SyntheticMcu {
            true_freq: nominal_freq * (1.0 + freq_error_ppm * 1e-6),
            clock_at_epoch,
            epoch_host,
        };
        let mut est = primed_estimator(&mcu, nominal_freq);
        let mut rng = SplitMix64(noise_seed);
        let mut prev_freq = nominal_freq;

        let mut published: Vec<(f64, f64)> = Vec::new();
        let mut last_estimate: Option<ClockEstimate> = None;
        let mut half_rtts: Vec<f64> = Vec::new();
        let mut sent = epoch_host;
        let mut last_truth = clock_at_epoch;
        let mut lost_wrap_samples = 0usize;
        let mut stalls = 0usize;

        for index in 1..=SAMPLES {
            let stalled = rng.next_u64() % STALL_ONE_IN == 0;
            sent += period + if stalled { stall_secs } else { 0.0 };
            stalls += usize::from(stalled);
            let out_delay = base_half_rtt + jitter_secs * rng.unit();
            let in_delay = base_half_rtt + jitter_secs * rng.unit();
            let received = sent + out_delay + in_delay;
            let truth = mcu.latch(sent + out_delay);
            lost_wrap_samples += usize::from(truth - last_truth >= 1u64 << 32);

            if rng.next_u64() % UNSTAMPED_ONE_IN == 0 {
                let before = (est.time_avg(), est.clock_avg(), est.sync_stable_count());
                let out = est.handle_clock(truth as u32, 0.0, received, nominal_freq, prev_freq);
                prop_assert!(out.is_none(), "an unstamped sample cannot publish");
                prop_assert_eq!(
                    est.last_clock(),
                    truth,
                    "unstamped sample {} reconstructed {}",
                    index,
                    est.last_clock()
                );
                prop_assert_eq!(
                    (est.time_avg(), est.clock_avg(), est.sync_stable_count()),
                    before,
                    "an unstamped sample must leave the regression and the latch alone"
                );
                last_truth = truth;
                continue;
            }

            let expected_clock = (sent - est.time_avg()) * prev_freq + est.clock_avg();
            let clock_diff2 = (truth as f64 - expected_clock).powi(2);
            let outlier = clock_diff2 > 25.0 * est.prediction_variance()
                && clock_diff2 > (0.000_500 * nominal_freq).powi(2);
            let expect_dropped = outlier
                && (truth as f64) > expected_clock
                && sent < est.last_prediction_time() + 10.0;
            let centroid_before = (est.time_avg(), est.clock_avg());
            let synced_before = est.synced();

            let out = est.handle_clock(truth as u32, sent, received, nominal_freq, prev_freq);

            prop_assert_eq!(
                est.last_clock(),
                truth,
                "sample {} reconstructed {} for a true clock of {}",
                index,
                est.last_clock(),
                truth
            );
            prop_assert_eq!(
                out.is_none(),
                expect_dropped,
                "sample {} drop decision disagrees with the published state",
                index
            );
            prop_assert!(
                est.synced() || !synced_before,
                "convergence must latch, never unlatch"
            );
            half_rtts.push(0.5 * (received - sent));

            if let Some(estimate) = out {
                prop_assert!(
                    est.time_avg() > centroid_before.0 && est.clock_avg() > centroid_before.1,
                    "an accepted sample must advance both centroids"
                );
                published.push((estimate.freq, prev_freq));
                prev_freq = estimate.freq;
                last_estimate = Some(estimate);
                if index > WARMUP_SAMPLES {
                    prop_assert!(
                        (estimate.freq - mcu.true_freq).abs() <= FREQ_TOL_PPM * mcu.true_freq,
                        "sample {} published {} against a true {}",
                        index,
                        estimate.freq,
                        mcu.true_freq
                    );
                }
            }
            last_truth = truth;
        }

        if nominal_freq == 520e6 {
            let wraps = (last_truth >> 32) - (clock_at_epoch >> 32);
            prop_assert!(
                wraps >= 2,
                "the run must cross at least two 32-bit wraps at 520 MHz, saw {}",
                wraps
            );
            if stalls > 0 && stall_secs * mcu.true_freq >= (1u64 << 32) as f64 {
                prop_assert!(
                    lost_wrap_samples > 0,
                    "a stall longer than a wrap period must lose a whole wrap"
                );
            }
        }

        let min_observed = half_rtts.iter().copied().fold(f64::INFINITY, f64::min);
        prop_assert!(
            half_rtts.contains(&est.min_half_rtt()),
            "min_half_rtt {} was never observed on the link",
            est.min_half_rtt()
        );
        prop_assert!(
            est.min_half_rtt() <= min_observed + (sent - epoch_host) * RTT_AGE,
            "min_half_rtt {} drifted above the observed minimum {}",
            est.min_half_rtt(),
            min_observed
        );

        prop_assert_eq!(
            (est.sync_stable_count(), est.synced()),
            replay_latch(&published, SYNC_STABLE_FREQ_PPM, nominal_freq),
            "the latch state must follow from the published frequencies"
        );

        let estimate = last_estimate.expect("a stamped sample must publish");
        let at_host = sent + lever_secs;
        let predicted = estimate.clock + (at_host - estimate.offset) * estimate.freq;
        let truth = mcu.latch(at_host) as f64;
        let error = (predicted - truth).abs();
        prop_assert!(
            error <= PREDICTION_TOL_SECS * mcu.true_freq,
            "predicting {} s ahead missed by {} ticks ({} s)",
            lever_secs,
            error,
            error / mcu.true_freq
        );
        let link_asymmetry = mcu.true_freq * jitter_secs;
        let slope_lever = (estimate.freq - mcu.true_freq).abs() * (at_host - estimate.offset).abs();
        let rounding = 4.0 + 32.0 * f64::EPSILON * (predicted.abs() + truth);
        prop_assert!(
            error <= link_asymmetry + slope_lever + rounding,
            "prediction error {} exceeds the link jitter {} plus the slope lever {}",
            error,
            link_asymmetry,
            slope_lever
        );
    }

    /// A delayed MCU latch reports a counter far ahead of the projection, and
    /// klippy rejects it. Rejection must be total: the run has to come out
    /// bit-identical to one where that sample never arrived at all — while
    /// `last_clock` still takes it, because that anchor is what every
    /// `clock32_to_clock64` resolves a 32-bit stamp against.
    #[test]
    fn a_rejected_latch_leaves_the_estimator_as_if_it_never_arrived(
        nominal_freq in arb_mcu_freq(),
        freq_error_ppm in -100.0f64..=100.0,
        clock_at_epoch in arb_epoch_clock(),
        epoch_host in 1.0f64..=1e5,
        period in 0.3f64..=2.0,
        base_half_rtt in prop_oneof![Just(0.0), 1e-6f64..=9e-4],
        jitter_secs in prop_oneof![Just(0.0), 1e-7f64..=1e-4],
        latch_delay_secs in 8e-3f64..=60e-3,
        delayed_index in 12usize..=SAMPLES,
        noise_seed in any::<u64>(),
    ) {
        let mcu = SyntheticMcu {
            true_freq: nominal_freq * (1.0 + freq_error_ppm * 1e-6),
            clock_at_epoch,
            epoch_host,
        };
        let mut rng = SplitMix64(noise_seed);
        let stream: Vec<Sample> = (1..=SAMPLES)
            .map(|index| {
                let sent = epoch_host + index as f64 * period;
                let mut out_delay = base_half_rtt + jitter_secs * rng.unit();
                let in_delay = base_half_rtt + jitter_secs * rng.unit();
                if index == delayed_index {
                    out_delay += latch_delay_secs;
                }
                Sample {
                    sent,
                    received: sent + out_delay + in_delay,
                    truth: mcu.latch(sent + out_delay),
                }
            })
            .collect();
        let without_delayed: Vec<Sample> = stream
            .iter()
            .enumerate()
            .filter(|(index, _)| *index + 1 != delayed_index)
            .map(|(_, sample)| *sample)
            .collect();

        let mut delayed_est = primed_estimator(&mcu, nominal_freq);
        let delayed_run = run_stream(&mut delayed_est, &stream, nominal_freq);
        let mut clean_est = primed_estimator(&mcu, nominal_freq);
        let clean_run = run_stream(&mut clean_est, &without_delayed, nominal_freq);

        for (index, (sample, outcome)) in stream.iter().zip(&delayed_run).enumerate() {
            prop_assert_eq!(
                outcome.last_clock,
                sample.truth,
                "sample {} reconstructed {}",
                index + 1,
                outcome.last_clock
            );
            prop_assert_eq!(
                outcome.published.is_none(),
                index + 1 == delayed_index,
                "sample {} rejection: only the {}-tick delayed latch may be rejected",
                index + 1,
                latch_delay_secs * mcu.true_freq
            );
        }

        let delayed_published: Vec<ClockEstimate> =
            delayed_run.iter().filter_map(|o| o.published).collect();
        let clean_published: Vec<ClockEstimate> =
            clean_run.iter().filter_map(|o| o.published).collect();
        prop_assert_eq!(
            delayed_published,
            clean_published,
            "a rejected sample must not perturb a single published estimate"
        );
        prop_assert_eq!(
            (
                delayed_est.time_avg(),
                delayed_est.clock_avg(),
                delayed_est.prediction_variance(),
                delayed_est.last_prediction_time(),
                delayed_est.min_half_rtt(),
                delayed_est.sync_stable_count(),
                delayed_est.synced(),
            ),
            (
                clean_est.time_avg(),
                clean_est.clock_avg(),
                clean_est.prediction_variance(),
                clean_est.last_prediction_time(),
                clean_est.min_half_rtt(),
                clean_est.sync_stable_count(),
                clean_est.synced(),
            ),
            "a rejected sample must leave no trace in the estimator state"
        );
    }
}
