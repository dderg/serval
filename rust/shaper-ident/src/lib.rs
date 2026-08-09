//! `klippy._shaper_ident`: the numeric core of input-shaper resonance
//! identification behind `klippy/extras/shaper_calibrate.py`. The Welch PSD,
//! shaper response estimation, and the `fit_shaper` search live in Rust; the
//! Python module keeps G-code responses, CSV I/O, and CalibrationData
//! bookkeeping.

pub mod core;
use pyo3::prelude::*;

use core::{FitResult, FreqResponse, ShaperFreqs};

/// `(freq_bins, psd_sum, psd_x, psd_y, psd_z)`, or `None` when the capture is
/// too short for the analysis window (matching `calc_freq_response`).
type FreqResponsePy = (Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>);

/// Compute the per-axis and summed PSD from a raw accelerometer capture.
/// `samples` rows are `[t, x, y, z]`.
#[pyfunction]
fn calc_freq_response(samples: Vec<[f64; 4]>) -> Option<FreqResponsePy> {
    core::calc_freq_response(&samples).map(|r| {
        let FreqResponse {
            freq_bins,
            psd_sum,
            psd_x,
            psd_y,
            psd_z,
        } = r;
        (freq_bins, psd_sum, psd_x, psd_y, psd_z)
    })
}

/// `(name, freq, vals, vibrs, smoothing, score, max_accel)`.
type FitResultPy = (String, f64, Vec<f64>, f64, f64, f64, f64);

/// Fit a single shaper family against a PSD, returning the selected result.
///
/// `shaper_freqs_range` is `(start, end, step)` (each optional); when
/// `shaper_freqs_list` is provided it takes precedence and is swept verbatim.
#[pyfunction]
#[allow(clippy::too_many_arguments)]
#[pyo3(signature = (
    name,
    freq_bins,
    psd_sum,
    shaper_freqs_range,
    shaper_freqs_list,
    damping_ratio,
    scv,
    max_smoothing,
    test_damping_ratios,
    max_freq,
))]
fn fit_shaper(
    name: &str,
    freq_bins: Vec<f64>,
    psd_sum: Vec<f64>,
    shaper_freqs_range: Option<(Option<f64>, Option<f64>, Option<f64>)>,
    shaper_freqs_list: Option<Vec<f64>>,
    damping_ratio: Option<f64>,
    scv: f64,
    max_smoothing: Option<f64>,
    test_damping_ratios: Option<Vec<f64>>,
    max_freq: Option<f64>,
) -> PyResult<Option<FitResultPy>> {
    let freqs = match shaper_freqs_list {
        Some(list) => ShaperFreqs::List(list),
        None => {
            let (a, b, c) = shaper_freqs_range.unwrap_or((None, None, None));
            ShaperFreqs::Range(a, b, c)
        }
    };
    let result = if let Some(cfg) = core::find_shaper_cfg(name) {
        core::fit_shaper(
            cfg,
            &freq_bins,
            &psd_sum,
            &freqs,
            damping_ratio,
            scv,
            max_smoothing,
            test_damping_ratios,
            max_freq,
        )
    } else if let Some(cfg) = core::find_smoother_cfg(name) {
        core::fit_smoother(
            cfg,
            &freq_bins,
            &psd_sum,
            &freqs,
            scv,
            max_smoothing,
            test_damping_ratios,
            max_freq,
        )
    } else {
        return Err(pyo3::exceptions::PyValueError::new_err(format!(
            "unknown shaper '{name}'"
        )));
    };
    Ok(result.map(|r| {
        let FitResult {
            name,
            freq,
            vals,
            vibrs,
            smoothing,
            score,
            max_accel,
        } = r;
        (name, freq, vals, vibrs, smoothing, score, max_accel)
    }))
}

#[pymodule]
fn _shaper_ident(_py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(calc_freq_response, m)?)?;
    m.add_function(wrap_pyfunction!(fit_shaper, m)?)?;
    Ok(())
}
