use super::{PyMotionEngine, PyResult, PyRuntimeError, Python, pymethods};
use motion_core::axis_transport::{TRANSPORT_PHASE, TRANSPORT_PULSE, transport_name};
use motion_core::lock_ext::LockExt;
use motion_core::types::AxisKey;
use std::sync::Arc;

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

        self.quiesce_pump_and_drain(py)?;

        let position = py
            .detach(|| -> Result<i64, String> {
                let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
                loop {
                    let (position_tx, position_rx) = std::sync::mpsc::sync_channel(1);
                    self.endpoint_command(motion_core::pump::EndpointCommand::Handover {
                        key,
                        from,
                        to: mode,
                        position: position_tx,
                    })?;
                    if let Some(position) = position_rx
                        .recv()
                        .map_err(|e| format!("switch_axis_transport: result channel closed: {e}"))?
                    {
                        return Ok(position);
                    }
                    if std::time::Instant::now() >= deadline {
                        return Err(format!(
                            "switch_axis_transport: mcu {mcu_handle} axis {axis_idx} still has \
                             motion in flight on its {} transport after a 5s drain wait",
                            transport_name(from)
                        ));
                    }
                    std::thread::sleep(std::time::Duration::from_millis(2));
                }
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
    pub(super) fn endpoint_command(
        &self,
        command: motion_core::pump::EndpointCommand,
    ) -> Result<(), String> {
        let tx = self
            .pump
            .tx
            .lock_ok()
            .clone()
            .ok_or_else(|| "endpoint command: execution owner is not running".to_string())?;
        endpoint_command(&tx, command)
    }
}

pub(super) fn endpoint_command(
    tx: &crossbeam_channel::Sender<motion_core::pump::PumpMsg>,
    command: motion_core::pump::EndpointCommand,
) -> Result<(), String> {
    let (reply, response) = std::sync::mpsc::sync_channel(1);
    tx.send(motion_core::pump::PumpMsg::Endpoint { command, reply })
        .map_err(|_| "endpoint command: execution owner channel closed".to_string())?;
    response
        .recv()
        .map_err(|e| format!("endpoint command: execution owner reply failed: {e}"))??;
    Ok(())
}
