//! `klippy._shaper_ident`: the numeric core of input-shaper resonance
//! identification behind `klippy/extras/shaper_calibrate.py`. The Welch PSD,
//! shaper response estimation, and the `fit_shaper` search live in Rust; the
//! Python module keeps G-code responses, CSV I/O, and CalibrationData
//! bookkeeping.

pub mod core;
use pyo3::exceptions::PyTypeError;
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyTuple};

use core::{FitParams, FitResult, FreqResponse, ShaperFreqs};

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

const FIT_SHAPER_PARAMS: [&str; 10] = [
    "name",
    "freq_bins",
    "psd_sum",
    "shaper_freqs_range",
    "shaper_freqs_list",
    "damping_ratio",
    "scv",
    "max_smoothing",
    "test_damping_ratios",
    "max_freq",
];

/// One `fit_shaper` call, decoded from the Python argument list.
struct FitRequest {
    name: String,
    freq_bins: Vec<f64>,
    psd_sum: Vec<f64>,
    freqs: ShaperFreqs,
    params: FitParams,
}

type ArgSlots<'py> = [Option<Bound<'py, PyAny>>; FIT_SHAPER_PARAMS.len()];

fn slot<'a, 'py, T>(slots: &'a ArgSlots<'py>, idx: usize) -> PyResult<T>
where
    T: FromPyObject<'a, 'py>,
    PyErr: From<T::Error>,
{
    let bound = slots[idx].as_ref().ok_or_else(|| {
        PyTypeError::new_err(format!(
            "fit_shaper() missing required argument '{}'",
            FIT_SHAPER_PARAMS[idx]
        ))
    })?;
    bound.extract().map_err(PyErr::from)
}

impl FitRequest {
    fn extract(args: &Bound<'_, PyTuple>, kwargs: Option<&Bound<'_, PyDict>>) -> PyResult<Self> {
        if args.len() > FIT_SHAPER_PARAMS.len() {
            return Err(PyTypeError::new_err(format!(
                "fit_shaper() takes {} arguments but {} were given",
                FIT_SHAPER_PARAMS.len(),
                args.len()
            )));
        }
        let mut slots: ArgSlots<'_> = Default::default();
        for (i, arg) in args.iter().enumerate() {
            slots[i] = Some(arg);
        }
        if let Some(kwargs) = kwargs {
            for (key, value) in kwargs.iter() {
                let key: String = key.extract()?;
                let idx = FIT_SHAPER_PARAMS
                    .iter()
                    .position(|n| *n == key)
                    .ok_or_else(|| {
                        PyTypeError::new_err(format!(
                            "fit_shaper() got an unexpected keyword argument '{key}'"
                        ))
                    })?;
                if slots[idx].is_some() {
                    return Err(PyTypeError::new_err(format!(
                        "fit_shaper() got multiple values for argument '{key}'"
                    )));
                }
                slots[idx] = Some(value);
            }
        }
        let shaper_freqs_range: Option<(Option<f64>, Option<f64>, Option<f64>)> = slot(&slots, 3)?;
        let shaper_freqs_list: Option<Vec<f64>> = slot(&slots, 4)?;
        let freqs = match shaper_freqs_list {
            Some(list) => ShaperFreqs::List(list),
            None => {
                let (a, b, c) = shaper_freqs_range.unwrap_or((None, None, None));
                ShaperFreqs::Range(a, b, c)
            }
        };
        Ok(Self {
            name: slot(&slots, 0)?,
            freq_bins: slot(&slots, 1)?,
            psd_sum: slot(&slots, 2)?,
            freqs,
            params: FitParams {
                damping_ratio: slot(&slots, 5)?,
                scv: slot(&slots, 6)?,
                max_smoothing: slot(&slots, 7)?,
                test_damping_ratios: slot(&slots, 8)?,
                max_freq: slot(&slots, 9)?,
            },
        })
    }
}

/// Fit a single shaper family against a PSD, returning the selected result.
///
/// Python signature: `fit_shaper(name, freq_bins, psd_sum, shaper_freqs_range,
/// shaper_freqs_list, damping_ratio, scv, max_smoothing, test_damping_ratios,
/// max_freq)`. `shaper_freqs_list` takes precedence over the range when given.
#[pyfunction]
#[pyo3(signature = (*args, **kwargs))]
fn fit_shaper(
    args: &Bound<'_, PyTuple>,
    kwargs: Option<&Bound<'_, PyDict>>,
) -> PyResult<Option<FitResultPy>> {
    let req = FitRequest::extract(args, kwargs)?;
    let result = if let Some(cfg) = core::find_shaper_cfg(&req.name) {
        core::fit_shaper(cfg, &req.freq_bins, &req.psd_sum, &req.freqs, &req.params)
    } else if let Some(cfg) = core::find_smoother_cfg(&req.name) {
        core::fit_smoother(cfg, &req.freq_bins, &req.psd_sum, &req.freqs, &req.params)
    } else {
        return Err(pyo3::exceptions::PyValueError::new_err(format!(
            "unknown shaper '{}'",
            req.name
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
