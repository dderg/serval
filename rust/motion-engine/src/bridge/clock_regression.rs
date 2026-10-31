//! PyO3 surface for the shared clock regression primitives in `host_rt`.
//!
//! `ClockSyncEstimator` backs klippy's `ClockSync`; `DecayRegression` backs
//! `bulk_sensor.ClockSyncRegression`. Both hand raw samples in and read the
//! running estimate out, so the decay-weighted least-squares math lives only
//! here.

use host_rt::clock_regression::{
    CLOCK_REGRESSION_DECAY, ClockSyncEstimator as CoreEstimator, DecayRegression as CoreRegression,
    NON_RESONANT_GET_CLOCK_PERIOD_SECS,
};
use pyo3::prelude::*;

#[pyclass(name = "DecayRegression")]
#[allow(missing_debug_implementations)]
pub struct PyDecayRegression {
    inner: CoreRegression,
}

#[pymethods]
impl PyDecayRegression {
    #[new]
    fn new(decay: f64) -> Self {
        Self {
            inner: CoreRegression::new(decay),
        }
    }

    fn reset(&mut self, x0: f64, y0: f64) {
        self.inner.reset(x0, y0);
    }

    fn update(&mut self, x: f64, y: f64) {
        self.inner.update(x, y);
    }

    #[getter]
    fn x_avg(&self) -> f64 {
        self.inner.x_avg()
    }
    #[getter]
    fn y_avg(&self) -> f64 {
        self.inner.y_avg()
    }
    #[getter]
    fn x_variance(&self) -> f64 {
        self.inner.x_variance()
    }
    #[getter]
    fn xy_covariance(&self) -> f64 {
        self.inner.xy_covariance()
    }
}

#[pyclass(name = "ClockSyncEstimator")]
#[allow(missing_debug_implementations)]
pub struct PyClockSyncEstimator {
    inner: CoreEstimator,
}

#[pymethods]
impl PyClockSyncEstimator {
    #[new]
    fn new(decay: f64, rtt_age: f64, sync_stable_freq_ppm: f64, sync_stable_samples: u32) -> Self {
        Self {
            inner: CoreEstimator::new(decay, rtt_age, sync_stable_freq_ppm, sync_stable_samples),
        }
    }

    #[classattr]
    const DECAY: f64 = CLOCK_REGRESSION_DECAY;

    #[getter(get_clock_period_secs)]
    fn get_clock_period_secs(&self) -> f64 {
        NON_RESONANT_GET_CLOCK_PERIOD_SECS
    }

    /// Process one clock response. Returns `(freq, offset, clock_avg)` when a
    /// new estimate is published, or `None` for the early-return paths.
    fn handle_clock(
        &mut self,
        raw_clock_low: u32,
        sent_time: f64,
        receive_time: f64,
        mcu_freq: f64,
        prev_freq: f64,
    ) -> Option<(f64, f64, f64)> {
        self.inner
            .handle_clock(raw_clock_low, sent_time, receive_time, mcu_freq, prev_freq)
            .map(|e| (e.freq, e.offset, e.clock))
    }

    #[getter]
    fn time_avg(&self) -> f64 {
        self.inner.time_avg()
    }
    #[setter]
    fn set_time_avg(&mut self, v: f64) {
        self.inner.set_time_avg(v);
    }
    #[getter]
    fn clock_avg(&self) -> f64 {
        self.inner.clock_avg()
    }
    #[setter]
    fn set_clock_avg(&mut self, v: f64) {
        self.inner.set_clock_avg(v);
    }
    #[getter]
    fn time_variance(&self) -> f64 {
        self.inner.time_variance()
    }
    #[setter]
    fn set_time_variance(&mut self, v: f64) {
        self.inner.set_time_variance(v);
    }
    #[getter]
    fn clock_covariance(&self) -> f64 {
        self.inner.clock_covariance()
    }
    #[setter]
    fn set_clock_covariance(&mut self, v: f64) {
        self.inner.set_clock_covariance(v);
    }
    #[getter]
    fn prediction_variance(&self) -> f64 {
        self.inner.prediction_variance()
    }
    #[setter]
    fn set_prediction_variance(&mut self, v: f64) {
        self.inner.set_prediction_variance(v);
    }
    #[getter]
    fn last_prediction_time(&self) -> f64 {
        self.inner.last_prediction_time()
    }
    #[setter]
    fn set_last_prediction_time(&mut self, v: f64) {
        self.inner.set_last_prediction_time(v);
    }
    #[getter]
    fn min_half_rtt(&self) -> f64 {
        self.inner.min_half_rtt()
    }
    #[setter]
    fn set_min_half_rtt(&mut self, v: f64) {
        self.inner.set_min_half_rtt(v);
    }
    #[getter]
    fn min_rtt_time(&self) -> f64 {
        self.inner.min_rtt_time()
    }
    #[setter]
    fn set_min_rtt_time(&mut self, v: f64) {
        self.inner.set_min_rtt_time(v);
    }
    #[getter]
    fn last_clock(&self) -> u64 {
        self.inner.last_clock()
    }
    #[setter]
    fn set_last_clock(&mut self, v: u64) {
        self.inner.set_last_clock(v);
    }
    #[getter]
    fn sync_stable_count(&self) -> u32 {
        self.inner.sync_stable_count()
    }
    #[setter]
    fn set_sync_stable_count(&mut self, v: u32) {
        self.inner.set_sync_stable_count(v);
    }
    #[getter]
    fn synced(&self) -> bool {
        self.inner.synced()
    }
    #[setter]
    fn set_synced(&mut self, v: bool) {
        self.inner.set_synced(v);
    }
}
