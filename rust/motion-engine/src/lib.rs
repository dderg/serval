mod bridge;
#[cfg(feature = "snapshot")]
pub mod viz;

#[doc(hidden)]
pub use motion_core::{
    anchor, bg_call, classify, config, drain, enqueue, fence, homing, kinematics, lock_ext,
    logging, mcu_config, mcu_log, motion_history, nudge, position_query, pump, remote_trigger,
    servo_capture, servo_sdo, servo_torque, timing, types, worker,
};

#[cfg(feature = "test-support")]
#[doc(hidden)]
pub use motion_core::seam_test_harness;

use pyo3::prelude::*;

use bridge::PyMotionEngine;

#[pymodule]
fn _motion_engine(_py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyMotionEngine>()?;
    #[cfg(feature = "snapshot")]
    m.add_function(wrap_pyfunction!(viz::pipeline_snapshot, m)?)?;
    Ok(())
}
