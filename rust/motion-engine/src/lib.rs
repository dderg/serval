mod bridge;
#[cfg(feature = "snapshot")]
mod parity;
mod step_benchmarks;
#[cfg(feature = "snapshot")]
pub mod viz;

use pyo3::prelude::*;

use bridge::{PyClockSyncEstimator, PyDecayRegression, PyMotionEngine};

#[pyfunction]
fn build_id() -> &'static str {
    env!("SERVAL_ENGINE_BUILD_ID")
}

#[pymodule]
fn _motion_engine(_py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(build_id, m)?)?;
    m.add_function(wrap_pyfunction!(step_benchmarks::step_capacity_budget, m)?)?;
    m.add_class::<PyMotionEngine>()?;
    m.add_class::<PyClockSyncEstimator>()?;
    m.add_class::<PyDecayRegression>()?;
    #[cfg(feature = "snapshot")]
    m.add_function(wrap_pyfunction!(viz::pipeline_snapshot, m)?)?;
    #[cfg(feature = "snapshot")]
    m.add_function(wrap_pyfunction!(viz::parse_gcode, m)?)?;
    #[cfg(feature = "snapshot")]
    m.add_function(wrap_pyfunction!(viz::pipeline_snapshot_axis_samples, m)?)?;
    #[cfg(feature = "snapshot")]
    m.add_function(wrap_pyfunction!(viz::snapshot_moves, m)?)?;
    #[cfg(feature = "snapshot")]
    m.add_class::<viz::PlannerLimits>()?;
    #[cfg(feature = "snapshot")]
    m.add_class::<parity::ParityRecord>()?;
    #[cfg(feature = "snapshot")]
    m.add_function(wrap_pyfunction!(parity::snapshot_parity_record, m)?)?;
    Ok(())
}
