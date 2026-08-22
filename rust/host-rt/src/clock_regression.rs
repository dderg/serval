//! Exponentially-decayed linear clock regression shared by the host clock
//! synchronization paths (klippy `ClockSync` and `bulk_sensor`).
//!
//! [`DecayRegression`] is the reusable decay-weighted least-squares core: feed
//! it `(x, y)` samples and read back the running averages, variance of `x`, and
//! covariance of `(x, y)`. [`ClockSyncEstimator`] builds the full MCU clock sync
//! estimate on top of it (wraparound correction, minimum-RTT tracking, outlier
//! rejection, convergence latching), matching `ClockSync._handle_clock`.

/// Seconds between klippy `ClockSync` `get_clock` queries: deliberately not a
/// round number so the samples do not resonate with other periodic reactor
/// events.
pub const NON_RESONANT_GET_CLOCK_PERIOD_SECS: f64 = 0.9839;

/// Per-sample decay weight of the clock regression: the estimate is an
/// exponential window `1/DECAY` samples wide.
pub const CLOCK_REGRESSION_DECAY: f64 = 1.0 / 30.0;

/// Span of samples the published estimate is built from. A record older than
/// this contains no live sample at all: the clocksync that produced it has
/// stopped feeding the router.
pub const REGRESSION_WINDOW_SECS: f64 = NON_RESONANT_GET_CLOCK_PERIOD_SECS / CLOCK_REGRESSION_DECAY;

const TWO_POW_32: f64 = 4_294_967_296.0;

/// Decay-weighted least-squares accumulator over `(x, y)` sample pairs.
///
/// Each update ages the running averages and (co)variances by `decay`, matching
/// the identical recurrence formerly duplicated in `clocksync.py` and
/// `bulk_sensor.py`.
#[derive(Debug, Clone)]
pub struct DecayRegression {
    decay: f64,
    x_avg: f64,
    x_variance: f64,
    y_avg: f64,
    xy_covariance: f64,
}

impl DecayRegression {
    #[must_use]
    pub fn new(decay: f64) -> Self {
        Self {
            decay,
            x_avg: 0.0,
            x_variance: 0.0,
            y_avg: 0.0,
            xy_covariance: 0.0,
        }
    }

    /// Reseed the averages to `(x0, y0)` and zero the (co)variances.
    pub fn reset(&mut self, x0: f64, y0: f64) {
        self.x_avg = x0;
        self.y_avg = y0;
        self.x_variance = 0.0;
        self.xy_covariance = 0.0;
    }

    pub fn update(&mut self, x: f64, y: f64) {
        let decay = self.decay;
        let diff_x = x - self.x_avg;
        self.x_avg += decay * diff_x;
        self.x_variance = (1.0 - decay) * (self.x_variance + diff_x * diff_x * decay);
        let diff_y = y - self.y_avg;
        self.y_avg += decay * diff_y;
        self.xy_covariance = (1.0 - decay) * (self.xy_covariance + diff_x * diff_y * decay);
    }

    #[must_use]
    pub fn decay(&self) -> f64 {
        self.decay
    }
    #[must_use]
    pub fn x_avg(&self) -> f64 {
        self.x_avg
    }
    #[must_use]
    pub fn y_avg(&self) -> f64 {
        self.y_avg
    }
    #[must_use]
    pub fn x_variance(&self) -> f64 {
        self.x_variance
    }
    #[must_use]
    pub fn xy_covariance(&self) -> f64 {
        self.xy_covariance
    }

    pub fn set_x_avg(&mut self, v: f64) {
        self.x_avg = v;
    }
    pub fn set_y_avg(&mut self, v: f64) {
        self.y_avg = v;
    }
    pub fn set_x_variance(&mut self, v: f64) {
        self.x_variance = v;
    }
    pub fn set_xy_covariance(&mut self, v: f64) {
        self.xy_covariance = v;
    }
}

/// The published clock estimate produced by [`ClockSyncEstimator::handle_clock`].
///
/// `offset` is `time_avg + min_half_rtt` (the sample instant of the regression),
/// `clock` is the regressed MCU clock at that instant, `freq` is the measured
/// ticks-per-host-second slope.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClockEstimate {
    pub freq: f64,
    pub offset: f64,
    pub clock: f64,
}

/// Full MCU clock synchronization estimator: decay regression of MCU clock
/// against host `sent_time`, with 32-bit wraparound reconstruction, minimum
/// round-trip-time tracking, outlier rejection, and convergence latching.
#[derive(Debug, Clone)]
pub struct ClockSyncEstimator {
    core: DecayRegression,
    rtt_age: f64,
    sync_stable_freq_ppm: f64,
    sync_stable_samples: u32,
    prediction_variance: f64,
    last_prediction_time: f64,
    min_half_rtt: f64,
    min_rtt_time: f64,
    last_clock: u64,
    sync_stable_count: u32,
    synced: bool,
}

impl ClockSyncEstimator {
    #[must_use]
    pub fn new(
        decay: f64,
        rtt_age: f64,
        sync_stable_freq_ppm: f64,
        sync_stable_samples: u32,
    ) -> Self {
        Self {
            core: DecayRegression::new(decay),
            rtt_age,
            sync_stable_freq_ppm,
            sync_stable_samples,
            prediction_variance: 0.0,
            last_prediction_time: 0.0,
            min_half_rtt: 999_999_999.9,
            min_rtt_time: 0.0,
            last_clock: 0,
            sync_stable_count: 0,
            synced: false,
        }
    }

    /// Process one `clock` response. `raw_clock_low` is the low 32 bits of the
    /// MCU counter, `sent_time`/`receive_time` are the host send/receive stamps,
    /// `mcu_freq` the datasheet frequency, `prev_freq` the currently published
    /// estimate frequency (`clock_est[2]`).
    ///
    /// Always updates `last_clock`. Returns `Some(estimate)` when a new estimate
    /// is published, `None` for the early-return paths (no sample time yet, or
    /// an outlier that is ignored) — matching `ClockSync._handle_clock`.
    pub fn handle_clock(
        &mut self,
        raw_clock_low: u32,
        sent_time: f64,
        receive_time: f64,
        mcu_freq: f64,
        prev_freq: f64,
    ) -> Option<ClockEstimate> {
        let last_clock = self.last_clock;
        let clock_delta = u64::from(raw_clock_low.wrapping_sub(last_clock as u32));
        let mut clock = last_clock + clock_delta;
        if sent_time != 0.0 {
            let exp_clock = (sent_time - self.core.x_avg()) * prev_freq + self.core.y_avg();
            #[allow(clippy::cast_precision_loss)]
            let wraps_lost = ((exp_clock - clock as f64) / TWO_POW_32).round();
            if wraps_lost > 0.0 {
                #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
                let wraps = wraps_lost as u64;
                clock += wraps * (1u64 << 32);
            }
        }
        self.last_clock = clock;
        if sent_time == 0.0 {
            return None;
        }
        #[allow(clippy::cast_precision_loss)]
        let clock_f = clock as f64;
        let half_rtt = 0.5 * (receive_time - sent_time);
        let aged_rtt = (sent_time - self.min_rtt_time) * self.rtt_age;
        if half_rtt < self.min_half_rtt + aged_rtt {
            self.min_half_rtt = half_rtt;
            self.min_rtt_time = sent_time;
        }
        let exp_clock = (sent_time - self.core.x_avg()) * prev_freq + self.core.y_avg();
        let clock_diff2 = (clock_f - exp_clock).powi(2);
        if clock_diff2 > 25.0 * self.prediction_variance
            && clock_diff2 > (0.000_500 * mcu_freq).powi(2)
        {
            if clock_f > exp_clock && sent_time < self.last_prediction_time + 10.0 {
                return None;
            }
            self.prediction_variance = (0.001 * mcu_freq).powi(2);
        } else {
            self.last_prediction_time = sent_time;
            let decay = self.core.decay();
            self.prediction_variance =
                (1.0 - decay) * (self.prediction_variance + clock_diff2 * decay);
        }
        self.core.update(sent_time, clock_f);
        let new_freq = self.core.xy_covariance() / self.core.x_variance();
        if !self.synced {
            if (new_freq - prev_freq).abs() <= self.sync_stable_freq_ppm * mcu_freq {
                self.sync_stable_count += 1;
                if self.sync_stable_count >= self.sync_stable_samples {
                    self.synced = true;
                }
            } else {
                self.sync_stable_count = 0;
            }
        }
        Some(ClockEstimate {
            freq: new_freq,
            offset: self.core.x_avg() + self.min_half_rtt,
            clock: self.core.y_avg(),
        })
    }

    #[must_use]
    pub fn time_avg(&self) -> f64 {
        self.core.x_avg()
    }
    #[must_use]
    pub fn clock_avg(&self) -> f64 {
        self.core.y_avg()
    }
    #[must_use]
    pub fn time_variance(&self) -> f64 {
        self.core.x_variance()
    }
    #[must_use]
    pub fn clock_covariance(&self) -> f64 {
        self.core.xy_covariance()
    }
    #[must_use]
    pub fn prediction_variance(&self) -> f64 {
        self.prediction_variance
    }
    #[must_use]
    pub fn last_prediction_time(&self) -> f64 {
        self.last_prediction_time
    }
    #[must_use]
    pub fn min_half_rtt(&self) -> f64 {
        self.min_half_rtt
    }
    #[must_use]
    pub fn min_rtt_time(&self) -> f64 {
        self.min_rtt_time
    }
    #[must_use]
    pub fn last_clock(&self) -> u64 {
        self.last_clock
    }
    #[must_use]
    pub fn sync_stable_count(&self) -> u32 {
        self.sync_stable_count
    }
    #[must_use]
    pub fn synced(&self) -> bool {
        self.synced
    }

    pub fn set_time_avg(&mut self, v: f64) {
        self.core.set_x_avg(v);
    }
    pub fn set_clock_avg(&mut self, v: f64) {
        self.core.set_y_avg(v);
    }
    pub fn set_time_variance(&mut self, v: f64) {
        self.core.set_x_variance(v);
    }
    pub fn set_clock_covariance(&mut self, v: f64) {
        self.core.set_xy_covariance(v);
    }
    pub fn set_prediction_variance(&mut self, v: f64) {
        self.prediction_variance = v;
    }
    pub fn set_last_prediction_time(&mut self, v: f64) {
        self.last_prediction_time = v;
    }
    pub fn set_min_half_rtt(&mut self, v: f64) {
        self.min_half_rtt = v;
    }
    pub fn set_min_rtt_time(&mut self, v: f64) {
        self.min_rtt_time = v;
    }
    pub fn set_last_clock(&mut self, v: u64) {
        self.last_clock = v;
    }
    pub fn set_sync_stable_count(&mut self, v: u32) {
        self.sync_stable_count = v;
    }
    pub fn set_synced(&mut self, v: bool) {
        self.synced = v;
    }
}

#[cfg(test)]
mod tests;
