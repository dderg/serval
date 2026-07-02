#[doc(hidden)]
pub mod anchor;
mod bridge;
#[doc(hidden)]
pub mod classify;
#[doc(hidden)]
pub mod config;
pub mod drain;
#[doc(hidden)]
pub mod enqueue;
#[doc(hidden)]
pub mod homing;
#[doc(hidden)]
pub mod kinematics;
pub mod logging;
#[doc(hidden)]
pub mod mcu_config;
pub mod mcu_log;
pub mod motion_history;
#[doc(hidden)]
pub mod nudge;
#[doc(hidden)]
pub mod position_query;
#[doc(hidden)]
pub mod pump;
pub mod remote_trigger;
mod router_transport;
mod servo_call;
#[doc(hidden)]
pub mod servo_capture;
#[doc(hidden)]
pub mod servo_sdo;
#[doc(hidden)]
pub mod servo_torque;
mod types;
pub mod viz;
pub mod worker;

pub use motion_pipeline::timing;

pub mod seam_test_harness;

pub mod test_support;

use pyo3::prelude::*;

use bridge::PyMotionEngine;

#[pymodule]
fn _motion_engine(_py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyMotionEngine>()?;
    m.add_function(wrap_pyfunction!(viz::pipeline_snapshot, m)?)?;
    Ok(())
}
