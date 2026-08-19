use super::{PyMotionEngine, PyResult, PyRuntimeError, Python, pymethods};
use crate::axis_transport::{TRANSPORT_PHASE, TRANSPORT_PULSE, transport_name};
use crate::lock_ext::LockExt;
use crate::types::AxisKey;
use std::sync::Arc;

/// One transport's side of a handover: read the position it actually executed,
/// and hand a position back to the transport taking over.
enum Side {
    Pulse(Arc<std::sync::Mutex<crate::pump::StepcompressEndpoint>>),
    Phase(Arc<std::sync::Mutex<crate::pump::SampleEndpoint>>),
}

impl Side {
    fn transport(&self) -> u8 {
        match self {
            Self::Pulse(_) => TRANSPORT_PULSE,
            Self::Phase(_) => TRANSPORT_PHASE,
        }
    }

    fn quiescent(&self) -> Result<bool, String> {
        match self {
            Self::Pulse(e) => Ok(e.lock_ok().transport_quiescent()),
            Self::Phase(e) => e.lock_ok().transport_quiescent().map_err(|e| e.to_string()),
        }
    }

    fn executed_position(&self, axis: u8) -> Result<i64, String> {
        match self {
            Self::Pulse(e) => e.lock_ok().executed_position(axis),
            Self::Phase(e) => e.lock_ok().executed_position(axis),
        }
        .map_err(|e| e.to_string())
    }

    fn adopt_position(&self, axis: u8, position: i64) -> Result<(), String> {
        match self {
            Self::Pulse(e) => e.lock_ok().reset_axis_position(axis, position),
            Self::Phase(e) => e.lock_ok().reset_axis_position(axis, position),
        }
        .map_err(|e| e.to_string())
    }
}

#[pymethods]
impl PyMotionEngine {
    /// Move one lane between its two mcu bindings. klippy calls this from the
    /// TMC phase-mode helper, on both sides of a StallGuard homing move:
    /// exiting phase mode routes the lane through the classic step queue,
    /// re-entering routes it back through the sample executor.
    ///
    /// The switch is a transport cut, so it is ordered, not merely announced:
    /// the pump is barriered and the pipeline drained, the outgoing transport
    /// must then be quiescent (nothing staged, nothing unretired), its executed
    /// position is read back off the mcu and cross-checked against the host's
    /// own counter, and only that reconciled position seeds the incoming
    /// transport. Anything out of order fails loudly rather than streaming into
    /// a lane the mcu is not running.
    #[pyo3(signature = (mcu_handle, axis_idx, mode))]
    fn switch_axis_transport(
        &self,
        py: Python<'_>,
        mcu_handle: u32,
        axis_idx: u8,
        mode: u8,
    ) -> PyResult<()> {
        if mode != TRANSPORT_PULSE && mode != TRANSPORT_PHASE {
            return Err(PyRuntimeError::new_err(format!(
                "switch_axis_transport: unknown transport {mode}; known: \
                 {TRANSPORT_PULSE}=pulse, {TRANSPORT_PHASE}=phase"
            )));
        }
        let key = AxisKey {
            mcu_id: mcu_handle,
            axis: axis_idx,
        };
        let transports = Arc::clone(&self.axis_transports.lock_ok());
        if !transports.supports(key, mode) {
            return Err(PyRuntimeError::new_err(format!(
                "switch_axis_transport: mcu {mcu_handle} axis {axis_idx} has no {} binding",
                transport_name(mode)
            )));
        }
        let from = transports.mode(key);
        if from == mode {
            tracing::info!(
                subsystem = "phase-stepping",
                event = "transport_switch_noop",
                mcu = mcu_handle,
                axis = axis_idx,
                mode = transport_name(mode),
                "transport switch requested but the lane is already there"
            );
            return Ok(());
        }
        let outgoing = self.transport_side(key, from)?;
        let incoming = self.transport_side(key, mode)?;

        self.quiesce_pump_and_drain(py)?;

        let position = py
            .detach(|| -> Result<i64, String> {
                // The outgoing side may still be playing its buffered lead
                // (the mcu retires sample runs asynchronously after the
                // pump drains); wait it out instead of failing on a race,
                // but never unboundedly.
                let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
                while !outgoing.quiescent()? {
                    if std::time::Instant::now() >= deadline {
                        return Err(format!(
                            "switch_axis_transport: mcu {mcu_handle} axis {axis_idx} still has \
                             motion in flight on its {} transport after a 5s drain wait",
                            transport_name(outgoing.transport())
                        ));
                    }
                    std::thread::sleep(std::time::Duration::from_millis(2));
                }
                let position = outgoing.executed_position(axis_idx)?;
                transports.adopt(key, mode)?;
                incoming.adopt_position(axis_idx, position)?;
                Ok(position)
            })
            .map_err(PyRuntimeError::new_err)?;

        tracing::info!(
            subsystem = "phase-stepping",
            event = "transport_switch",
            mcu = mcu_handle,
            axis = axis_idx,
            from = transport_name(from),
            to = transport_name(mode),
            position_lane_units = position,
            "lane handed over between its phase and pulse bindings"
        );
        Ok(())
    }
}

impl PyMotionEngine {
    fn transport_side(&self, key: AxisKey, mode: u8) -> PyResult<Side> {
        let missing = |what: &str| {
            PyRuntimeError::new_err(format!(
                "switch_axis_transport: mcu {} axis {} claims a {what} binding but no {what} \
                 endpoint is registered for that mcu",
                key.mcu_id, key.axis
            ))
        };
        match mode {
            TRANSPORT_PULSE => self
                .stepcompress_endpoints
                .lock_ok()
                .get(&key.mcu_id)
                .map(|e| Side::Pulse(Arc::clone(e)))
                .ok_or_else(|| missing("pulse")),
            _ => self
                .sample_endpoints
                .lock_ok()
                .get(&key.mcu_id)
                .map(|e| Side::Phase(Arc::clone(e)))
                .ok_or_else(|| missing("phase")),
        }
    }
}
