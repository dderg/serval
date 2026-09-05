mod bridge;
#[cfg(feature = "snapshot")]
pub mod viz;

#[doc(hidden)]
pub use motion_core::{
    anchor, axis_transport, classify, config, drain, enqueue, fence, homing, kinematics, lock_ext,
    mcu_config, motion_history, nudge, pump, timing, types, worker,
};

#[doc(hidden)]
pub use motion_services::{
    bg_call, logging, mcu_log, position_query, remote_trigger, servo_capture, servo_sdo,
    servo_torque,
};

#[cfg(feature = "test-support")]
#[doc(hidden)]
pub use motion_core::seam_test_harness;

use pyo3::prelude::*;

use bridge::{PyClockSyncEstimator, PyDecayRegression, PyMotionEngine};

#[pymodule]
fn _motion_engine(_py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyMotionEngine>()?;
    m.add_class::<PyClockSyncEstimator>()?;
    m.add_class::<PyDecayRegression>()?;
    #[cfg(feature = "snapshot")]
    m.add_function(wrap_pyfunction!(viz::pipeline_snapshot, m)?)?;
    Ok(())
}
