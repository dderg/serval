#[doc(hidden)]
pub mod anchor;
mod bridge;
#[doc(hidden)]
pub mod classify;
#[doc(hidden)]
pub mod config;
#[doc(hidden)]
pub mod dispatch;
pub mod drain;
#[doc(hidden)]
pub mod enqueue;
#[doc(hidden)]
pub mod homing;
pub mod jerk_probe;
#[doc(hidden)]
pub mod kinematics;
pub mod logging;
pub mod lowering;
pub mod mcu_log;
pub mod motion_history;
#[doc(hidden)]
pub mod motion_node;
#[doc(hidden)]
pub mod nudge;
#[doc(hidden)]
pub mod position_query;
#[doc(hidden)]
pub mod pump;
pub mod remote_trigger;
mod router_transport;
#[doc(hidden)]
pub mod servo_capture;
#[doc(hidden)]
pub mod servo_sdo;
#[doc(hidden)]
pub mod servo_torque;
pub mod stream;
pub mod stream_planner;
pub mod timing;
mod types;
pub mod viz;

pub mod test_support;

use pyo3::prelude::*;

use bridge::PyMotionEngine;

#[pymodule]
fn _motion_engine(_py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyMotionEngine>()?;
    m.add_function(wrap_pyfunction!(viz::pipeline_snapshot, m)?)?;
    Ok(())
}
