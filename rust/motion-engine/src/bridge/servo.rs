use super::{
    PyMotionEngine, PyResult, PyRuntimeError, Python, mcu_handle_from_raw, pymethods, slot_for_axis,
};
use pyo3::prelude::*;

#[pymethods]
impl PyMotionEngine {
    fn set_torque(&self, mcu_handle: u32, value: bool, print_time: f64) -> PyResult<()> {
        let reference_mcu = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            *mcus
                .iter()
                .find(|(_, mc)| mc.label == "mcu")
                .map(|(raw, _)| raw)
                .ok_or_else(|| {
                    PyRuntimeError::new_err(
                        "set_torque: no MCU labeled 'mcu' claimed — \
                         cannot resolve the print_time reference clock",
                    )
                })?
        };
        let execute_at_ns = {
            let router = self.router.lock().unwrap_or_else(|p| p.into_inner());
            let host_secs = router
                .print_time_to_host_secs(mcu_handle_from_raw(reference_mcu), print_time)
                .ok_or_else(|| {
                    PyRuntimeError::new_err(format!(
                        "set_torque: reference mcu {reference_mcu} clock not synced — \
                         cannot convert print_time {print_time}"
                    ))
                })?;
            router
                .host_time_to_mcu_clock(mcu_handle_from_raw(mcu_handle), host_secs)
                .map_err(|e| {
                    PyRuntimeError::new_err(format!(
                        "set_torque: no clock mapping for mcu {mcu_handle}: {e:?}"
                    ))
                })?
        };
        if execute_at_ns == 0 {
            return Err(PyRuntimeError::new_err(format!(
                "set_torque: EtherCAT clock for mcu {mcu_handle} not seeded \
                 (init_planner not run?)"
            )));
        }
        let conn = self.ethercat_conn(mcu_handle, "set_torque")?;
        tracing::info!(
            subsystem = "engine",
            event = "servo_torque_command",
            mcu_handle,
            value,
            print_time,
            execute_at_ns,
            "servo torque command"
        );
        let result = crate::servo_torque::send_set_torque(&conn, value, execute_at_ns)
            .map_err(PyRuntimeError::new_err)?;
        if result != 0 {
            tracing::error!(
                subsystem = "engine",
                event = "servo_torque_rejected",
                mcu_handle,
                value,
                result,
                "servo torque command rejected"
            );
            return Err(PyRuntimeError::new_err(format!(
                "servo torque {} failed: endpoint result {result}",
                if value { "enable" } else { "disable" }
            )));
        }
        Ok(())
    }
    fn start_servo_capture(
        &self,
        mcu_handle: u32,
        path: String,
        started_utc: String,
        drives: Vec<(u8, String)>,
    ) -> PyResult<()> {
        let conn = self.ethercat_conn(mcu_handle, "start_servo_capture")?;
        tracing::info!(
            subsystem = "engine",
            event = "servo_capture_start",
            mcu_handle,
            path,
            "servo capture start"
        );
        let result = crate::servo_capture::send_start_capture(&conn, &path, &started_utc, &drives)
            .map_err(PyRuntimeError::new_err)?;
        if result != 0 {
            return Err(PyRuntimeError::new_err(format!(
                "servo capture start failed: endpoint result {result}"
            )));
        }
        Ok(())
    }
    fn stop_servo_capture(&self, mcu_handle: u32) -> PyResult<(i32, u64, Option<u64>)> {
        let conn = self.ethercat_conn(mcu_handle, "stop_servo_capture")?;
        tracing::info!(
            subsystem = "engine",
            event = "servo_capture_stop",
            mcu_handle,
            "servo capture stop"
        );
        let resp =
            crate::servo_capture::send_stop_capture(&conn).map_err(PyRuntimeError::new_err)?;
        tracing::info!(
            subsystem = "engine",
            event = "servo_capture_stopped",
            mcu_handle,
            result = resp.result,
            samples = resp.samples,
            "servo capture stopped"
        );
        let overflow = (resp.overflow_cycle
            != mcu_protocol::messages::StopCaptureResponse::NO_OVERFLOW)
            .then_some(resp.overflow_cycle);
        Ok((resp.result, resp.samples, overflow))
    }
    fn set_drive_limits(
        &self,
        mcu_handle: u32,
        slot: u8,
        following_error_counts: u32,
        max_torque_tenth_pct: u16,
    ) -> PyResult<()> {
        let conn = self.ethercat_conn(mcu_handle, "set_drive_limits")?;
        tracing::info!(
            subsystem = "engine",
            event = "servo_drive_limits",
            mcu_handle,
            following_error_counts,
            max_torque_tenth_pct,
            "servo drive limits set"
        );
        let result = crate::servo_torque::send_drive_limits(
            &conn,
            slot,
            following_error_counts,
            max_torque_tenth_pct,
        )
        .map_err(PyRuntimeError::new_err)?;
        if result != 0 {
            return Err(PyRuntimeError::new_err(format!(
                "set_drive_limits: SDO write failed: endpoint result {result}"
            )));
        }
        Ok(())
    }
    fn restore_drive_limits(&self, mcu_handle: u32, slot: u8) -> PyResult<()> {
        let conn = self.ethercat_conn(mcu_handle, "restore_drive_limits")?;
        tracing::info!(
            subsystem = "engine",
            event = "servo_drive_limits",
            mcu_handle,
            "servo drive limits restored"
        );
        let result = crate::servo_torque::send_restore_drive_limits(&conn, slot)
            .map_err(PyRuntimeError::new_err)?;
        if result != 0 {
            return Err(PyRuntimeError::new_err(format!(
                "restore_drive_limits: SDO write failed: endpoint result {result}"
            )));
        }
        Ok(())
    }
    fn stop_node(&self, mcu_handle: u32) -> PyResult<()> {
        let conn = self.ethercat_conn(mcu_handle, "stop_node")?;
        tracing::warn!(
            subsystem = "engine",
            event = "servo_emergency_stop",
            mcu_handle,
            "servo motion discarded on shutdown"
        );
        let result = crate::servo_torque::send_stop(&conn).map_err(PyRuntimeError::new_err)?;
        if result != 0 {
            return Err(PyRuntimeError::new_err(format!(
                "stop_node: endpoint rejected Stop: result {result}"
            )));
        }
        Ok(())
    }
    fn arm_sensorless_endstop(
        &self,
        mcu_handle: u32,
        slot: u8,
        endstop_id: u8,
        torque_trip_tenth_pct: u16,
        enable: bool,
    ) -> PyResult<()> {
        let conn = self.ethercat_conn(mcu_handle, "arm_sensorless_endstop")?;
        tracing::info!(
            subsystem = "engine",
            event = "sensorless_endstop_arm",
            mcu_handle,
            endstop_id,
            torque_trip_tenth_pct,
            enable,
            "servo sensorless endstop arm/disarm"
        );
        let result = crate::servo_torque::send_arm_sensorless_endstop(
            &conn,
            slot,
            endstop_id,
            torque_trip_tenth_pct,
            enable,
        )
        .map_err(PyRuntimeError::new_err)?;
        if result != 0 {
            return Err(PyRuntimeError::new_err(format!(
                "arm_sensorless_endstop: endpoint rejected arm (result {result})"
            )));
        }
        Ok(())
    }
    #[pyo3(signature = (mcu_handle, axis, pos_mm, timeout_s = 2.0))]
    fn finalize_homed_axis(
        &self,
        py: Python<'_>,
        mcu_handle: u32,
        axis: usize,
        pos_mm: f64,
        timeout_s: f64,
    ) -> PyResult<()> {
        let (conn, slot) = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            let mc = mcus.get(&mcu_handle).ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "finalize_homed_axis: unknown mcu_handle {mcu_handle}"
                ))
            })?;
            let conn = match mc.endpoint_conn.clone() {
                Some(conn) => conn,
                None => return Ok(()),
            };
            let slot = slot_for_axis(&mc.ethercat_slot_axes, axis).ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "finalize_homed_axis: axis {axis} not driven by mcu {mcu_handle} \
                     (slot map {:?})",
                    mc.ethercat_slot_axes
                ))
            })?;
            (conn, slot)
        };
        let home_q16 = crate::mcu_config::encode_q16(pos_mm);
        tracing::info!(
            subsystem = "engine",
            event = "servo_finalize_home",
            mcu_handle,
            pos_mm,
            home_q16,
            "servo home finalize"
        );
        let timeout = std::time::Duration::from_secs_f64(timeout_s);
        let result = py
            .detach(|| crate::servo_torque::send_seed_servo_home(&conn, slot, home_q16, timeout))
            .map_err(PyRuntimeError::new_err)?;
        if result != 0 {
            return Err(PyRuntimeError::new_err(format!(
                "finalize_homed_axis: method-35 home-set failed: endpoint result {result}"
            )));
        }
        Ok(())
    }
    fn take_drive_fault(&self, mcu_handle: u32) -> PyResult<Option<u16>> {
        Ok(self
            .latched_drive_fault
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .remove(&mcu_handle))
    }
    fn take_endpoint_death(&self, mcu_handle: u32) -> PyResult<Option<String>> {
        Ok(self
            .latched_endpoint_death
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .remove(&mcu_handle))
    }
    fn sdo_read(&self, mcu_handle: u32, slot: u8, index: u16, subindex: u8) -> PyResult<(u8, u32)> {
        let conn = self.ethercat_conn(mcu_handle, "sdo_read")?;
        tracing::info!(
            subsystem = "engine",
            event = "servo_sdo_read",
            mcu_handle,
            slot,
            index,
            subindex,
            "servo SDO read"
        );
        let r = crate::servo_sdo::send_sdo_read(&conn, slot, index, subindex)
            .map_err(PyRuntimeError::new_err)?;
        if r.result != 0 {
            tracing::error!(
                subsystem = "engine",
                event = "servo_sdo_read_failed",
                mcu_handle,
                index,
                subindex,
                result = r.result,
                "servo SDO read failed"
            );
            return Err(PyRuntimeError::new_err(format!(
                "SDO read 0x{index:04x}.{subindex}: {}",
                crate::servo_sdo::failure_text(r.result)
            )));
        }
        Ok((r.size, u32::from_le_bytes(r.data)))
    }
    fn sdo_write(
        &self,
        mcu_handle: u32,
        slot: u8,
        index: u16,
        subindex: u8,
        size: u8,
        value: i64,
    ) -> PyResult<(u8, u32)> {
        let conn = self.ethercat_conn(mcu_handle, "sdo_write")?;
        tracing::info!(
            subsystem = "engine",
            event = "servo_sdo_write",
            mcu_handle,
            slot,
            index,
            subindex,
            size,
            value,
            "servo SDO write"
        );
        let r = crate::servo_sdo::send_sdo_write(&conn, slot, index, subindex, size, value)
            .map_err(PyRuntimeError::new_err)?;
        if r.result != 0 {
            tracing::error!(
                subsystem = "engine",
                event = "servo_sdo_write_failed",
                mcu_handle,
                index,
                subindex,
                size,
                value,
                result = r.result,
                "servo SDO write failed"
            );
            let readback = u32::from_le_bytes(r.readback_data);
            return Err(PyRuntimeError::new_err(format!(
                "SDO write 0x{index:04x}.{subindex} = {value} (size {size}): {} \
                 (drive reports raw 0x{readback:x})",
                crate::servo_sdo::failure_text(r.result)
            )));
        }
        Ok((r.readback_size, u32::from_le_bytes(r.readback_data)))
    }
    #[allow(clippy::too_many_arguments)]
    fn resonance_buzz(
        &self,
        py: Python<'_>,
        mcu_handle: u32,
        axis_mask: u8,
        sign_mask: u8,
        freq_start_millihz: u32,
        freq_end_millihz: u32,
        amplitude_nm: u32,
        duration_ms: u32,
        ramp_ms: u32,
    ) -> PyResult<()> {
        let conn = self.ethercat_conn(mcu_handle, "resonance_buzz")?;
        tracing::info!(
            subsystem = "engine",
            event = "servo_resonance_buzz",
            mcu_handle,
            axis_mask,
            sign_mask,
            freq_start_millihz,
            freq_end_millihz,
            amplitude_nm,
            duration_ms,
            ramp_ms,
            "servo resonance buzz"
        );
        let result = py
            .detach(|| {
                crate::servo_torque::send_resonance_buzz(
                    &conn,
                    axis_mask,
                    sign_mask,
                    freq_start_millihz,
                    freq_end_millihz,
                    amplitude_nm,
                    duration_ms,
                    ramp_ms,
                )
            })
            .map_err(PyRuntimeError::new_err)?;
        if result != 0 {
            return Err(PyRuntimeError::new_err(format!(
                "resonance_buzz: endpoint rejected (result {result})"
            )));
        }
        Ok(())
    }
}
