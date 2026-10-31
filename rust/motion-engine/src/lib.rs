mod bridge;
#[cfg(feature = "snapshot")]
pub mod viz;

use pyo3::prelude::*;

use bridge::{PyClockSyncEstimator, PyDecayRegression, PyMotionEngine};

#[pymodule]
fn _motion_engine(_py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyMotionEngine>()?;
    m.add_class::<PyClockSyncEstimator>()?;
    m.add_class::<PyDecayRegression>()?;
    #[cfg(feature = "snapshot")]
    m.add_function(wrap_pyfunction!(viz::pipeline_snapshot, m)?)?;
    #[cfg(feature = "snapshot")]
    m.add_function(wrap_pyfunction!(viz::parse_gcode, m)?)?;
    #[cfg(feature = "snapshot")]
    m.add_function(wrap_pyfunction!(viz::pipeline_snapshot_axis_samples, m)?)?;
    Ok(())
}
