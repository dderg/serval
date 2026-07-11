use super::{
    PyMotionEngine, PyResult, PyRuntimeError, Python, mcu_handle_from_raw, pymethods,
    slots_for_axis,
};
use crate::lock_ext::LockExt;

#[pymethods]
impl PyMotionEngine {
    /// `Ok(false)` while the endpoint call is still in flight; `Ok(true)`
    /// once done. Poll from a reactor-yielding loop — see `bg_call`.
    fn endpoint_call_done(&self, call_id: u64) -> PyResult<bool> {
        self.endpoint_calls
            .done(call_id)
            .map_err(PyRuntimeError::new_err)
    }
    fn set_torque_start(&self, mcu_handle: u32, value: bool, print_time: f64) -> PyResult<u64> {
        let reference_mcu = {
            let mcus = self.mcus.lock_ok();
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
            let router = self.router.lock_ok();
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
        Ok(self.endpoint_calls.start("set_torque", move || {
            let result = crate::servo_torque::send_set_torque(&conn, value, execute_at_ns)?;
            if result != 0 {
                tracing::error!(
                    subsystem = "engine",
                    event = "servo_torque_rejected",
                    mcu_handle,
                    value,
                    result,
                    "servo torque command rejected"
                );
                return Err(format!(
                    "servo torque {} failed: endpoint result {result}",
                    if value { "enable" } else { "disable" }
                ));
            }
            Ok(())
        }))
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
    fn set_drive_limits_start(
        &self,
        mcu_handle: u32,
        slot: u8,
        following_error_counts: u32,
        max_torque_tenth_pct: u16,
    ) -> PyResult<u64> {
        let conn = self.ethercat_conn(mcu_handle, "set_drive_limits")?;
        tracing::info!(
            subsystem = "engine",
            event = "servo_drive_limits",
            mcu_handle,
            following_error_counts,
            max_torque_tenth_pct,
            "servo drive limits set"
        );
        Ok(self.endpoint_calls.start("set_drive_limits", move || {
            let result = crate::servo_torque::send_drive_limits(
                &conn,
                slot,
                following_error_counts,
                max_torque_tenth_pct,
            )?;
            require_endpoint_ok(result, "set_drive_limits: SDO write failed")
        }))
    }
    fn restore_drive_limits_start(&self, mcu_handle: u32, slot: u8) -> PyResult<u64> {
        let conn = self.ethercat_conn(mcu_handle, "restore_drive_limits")?;
        tracing::info!(
            subsystem = "engine",
            event = "servo_drive_limits",
            mcu_handle,
            "servo drive limits restored"
        );
        Ok(self.endpoint_calls.start("restore_drive_limits", move || {
            let result = crate::servo_torque::send_restore_drive_limits(&conn, slot)?;
            require_endpoint_ok(result, "restore_drive_limits: SDO write failed")
        }))
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
        let result = crate::servo_torque::send_set_torque(&conn, false, 0)
            .map_err(PyRuntimeError::new_err)?;
        if result != 0 {
            return Err(PyRuntimeError::new_err(format!(
                "stop_node: endpoint rejected torque disable: result {result}"
            )));
        }
        Ok(())
    }
    fn arm_sensorless_endstop_start(
        &self,
        mcu_handle: u32,
        slot: u8,
        endstop_id: u8,
        torque_trip_tenth_pct: u16,
        enable: bool,
    ) -> PyResult<u64> {
        let conn = self.ethercat_conn(mcu_handle, "arm_sensorless_endstop")?;
        if enable {
            *self.homing.pending_trip.lock_ok() = None;
            let host_now = self.router.lock_ok().host_now_secs();
            *self.homing.last_arm.lock_ok() = Some((mcu_handle, endstop_id, host_now));
        }
        tracing::info!(
            subsystem = "engine",
            event = "sensorless_endstop_arm",
            mcu_handle,
            endstop_id,
            torque_trip_tenth_pct,
            enable,
            "servo sensorless endstop arm/disarm"
        );
        Ok(self
            .endpoint_calls
            .start("arm_sensorless_endstop", move || {
                let result = crate::servo_torque::send_arm_sensorless_endstop(
                    &conn,
                    slot,
                    endstop_id,
                    torque_trip_tenth_pct,
                    enable,
                )?;
                if result != 0 {
                    return Err(format!(
                        "arm_sensorless_endstop: endpoint rejected arm (result {result})"
                    ));
                }
                Ok(())
            }))
    }
    #[pyo3(signature = (mcu_handle, axis, pos_mm, timeout_s = 5.0))]
    fn finalize_homed_axis_start(
        &self,
        mcu_handle: u32,
        axis: usize,
        pos_mm: [f64; 3],
        timeout_s: f64,
    ) -> PyResult<u64> {
        let cfg = {
            let configs = self.mcu_axis_configs.lock_ok();
            configs
                .iter()
                .find(|c| c.mcu_id == mcu_handle)
                .cloned()
                .ok_or_else(|| {
                    PyRuntimeError::new_err(format!(
                        "finalize_homed_axis: no axis config for mcu_handle {mcu_handle}"
                    ))
                })?
        };
        let motor = crate::mcu_config::motor_frame(&cfg, pos_mm);
        let seed_lanes: &[usize] =
            if cfg.kinematics == crate::mcu_config::KINEMATICS_COREXY && axis <= 1 {
                &[0, 1]
            } else {
                &[axis]
            };
        let (conn, seeds) = {
            let mcus = self.mcus.lock_ok();
            let mc = mcus.get(&mcu_handle).ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "finalize_homed_axis: unknown mcu_handle {mcu_handle}"
                ))
            })?;
            let conn = match mc.endpoint_conn.clone() {
                Some(conn) => conn,
                None => {
                    return Ok(self.endpoint_calls.start("finalize_noop", || Ok(())));
                }
            };
            let seeds = seed_lanes
                .iter()
                .map(|&lane| {
                    let slots = slots_for_axis(&mc.ethercat_slot_axes, lane);
                    if slots.is_empty() {
                        return Err(PyRuntimeError::new_err(format!(
                            "finalize_homed_axis: axis {lane} not driven by mcu \
                             {mcu_handle} (slot map {:?})",
                            mc.ethercat_slot_axes
                        )));
                    }
                    let home_q16 = crate::mcu_config::encode_q16(motor[lane]);
                    Ok(slots.into_iter().map(move |slot| (slot, home_q16)))
                })
                .collect::<PyResult<Vec<_>>>()?
                .into_iter()
                .flatten()
                .collect::<Vec<(u8, i32)>>();
            (conn, seeds)
        };
        tracing::info!(
            subsystem = "engine",
            event = "servo_finalize_home",
            mcu_handle,
            axis,
            x = pos_mm[0],
            y = pos_mm[1],
            z = pos_mm[2],
            seeds = format!("{seeds:?}"),
            "servo home finalize (motor-frame seeds per slot)"
        );
        let timeout = std::time::Duration::from_secs_f64(timeout_s);
        Ok(self.endpoint_calls.start("finalize_homed_axis", move || {
            for (slot, home_q16) in seeds {
                let result =
                    crate::servo_torque::send_seed_servo_home(&conn, slot, home_q16, timeout)?;
                require_endpoint_ok(
                    result,
                    &format!("finalize_homed_axis: method-35 home-set failed for slot {slot}"),
                )?;
            }
            Ok(())
        }))
    }
    #[allow(clippy::too_many_arguments)]
    fn sync_servo_pair_start(
        &self,
        mcu_handle: u32,
        axis: u8,
        torque_ok_tenth_pct: u16,
        settle_timeout_ms: u16,
        dither_amplitude_nm: u32,
        dither_freq_millihz: u32,
        dither_duration_ms: u16,
    ) -> PyResult<u64> {
        let conn = self.ethercat_conn(mcu_handle, "sync_servo_pair")?;
        let reports = std::sync::Arc::clone(&self.sync_reports);
        tracing::info!(
            subsystem = "engine",
            event = "servo_pair_sync_start",
            mcu_handle,
            axis,
            torque_ok_tenth_pct,
            settle_timeout_ms,
            dither_amplitude_nm,
            dither_freq_millihz,
            dither_duration_ms,
            "servo belt pair sync"
        );
        Ok(self.endpoint_calls.start("sync_servo_pair", move || {
            let resp = crate::servo_torque::send_sync_pair(
                &conn,
                mcu_protocol::messages::SyncPair {
                    axis,
                    torque_ok_tenth_pct,
                    settle_timeout_ms,
                    dither_amplitude_nm,
                    dither_freq_millihz,
                    dither_duration_ms,
                },
            )?;
            let result = resp.result;
            reports.lock_ok().insert(mcu_handle, resp);
            if result != 0 {
                return Err(format!("sync_servo_pair: endpoint result {result}"));
            }
            Ok(())
        }))
    }
    /// The measurements of the most recent pair sync on this node, as
    /// (result, primary_slot, secondary_slot, torque_baseline_primary,
    /// torque_baseline_secondary, torque_released, torque_dithered,
    /// torque_final_primary, torque_final_secondary, released_delta_counts).
    /// Available for failed runs too — the phase torques are the diagnosis.
    #[allow(clippy::type_complexity)]
    fn take_sync_report(
        &self,
        mcu_handle: u32,
    ) -> PyResult<Option<(i32, u8, u8, i32, i32, i32, i32, i32, i32, i32)>> {
        Ok(self.sync_reports.lock_ok().remove(&mcu_handle).map(|r| {
            (
                r.result,
                r.primary_slot,
                r.secondary_slot,
                r.torque_baseline_primary,
                r.torque_baseline_secondary,
                r.torque_released,
                r.torque_dithered,
                r.torque_final_primary,
                r.torque_final_secondary,
                r.released_delta_counts,
            )
        }))
    }
    fn take_drive_fault(&self, mcu_handle: u32) -> PyResult<Option<u16>> {
        Ok(self.latched.drive.lock_ok().remove(&mcu_handle))
    }
    fn take_endpoint_death(&self, mcu_handle: u32) -> PyResult<Option<String>> {
        Ok(self.latched.endpoint_death.lock_ok().remove(&mcu_handle))
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
                    mcu_protocol::messages::ResonanceBuzz {
                        axis_mask,
                        sign_mask,
                        freq_start_millihz,
                        freq_end_millihz,
                        amplitude_nm,
                        duration_ms,
                        ramp_ms,
                    },
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
    fn set_diff_damper(
        &self,
        py: Python<'_>,
        mcu_handle: u32,
        slot_a: u8,
        slot_b: u8,
        gain_milli: u32,
        clamp_tenths: u16,
        lpf_millihz: u32,
    ) -> PyResult<()> {
        let conn = self.ethercat_conn(mcu_handle, "set_diff_damper")?;
        tracing::info!(
            subsystem = "engine",
            event = "servo_set_diff_damper",
            mcu_handle,
            slot_a,
            slot_b,
            gain_milli,
            clamp_tenths,
            lpf_millihz,
            "servo differential damper"
        );
        let result = py
            .detach(|| {
                crate::servo_torque::send_set_diff_damper(
                    &conn,
                    mcu_protocol::messages::SetDiffDamper {
                        slot_a,
                        slot_b,
                        gain_milli,
                        clamp_tenths,
                        lpf_millihz,
                    },
                )
            })
            .map_err(PyRuntimeError::new_err)?;
        if result != 0 {
            return Err(PyRuntimeError::new_err(format!(
                "set_diff_damper: endpoint rejected (result {result})"
            )));
        }
        Ok(())
    }
}

fn require_endpoint_ok(result: i32, context: &str) -> Result<(), String> {
    if result != 0 {
        return Err(format!("{context}: endpoint result {result}"));
    }
    Ok(())
}
