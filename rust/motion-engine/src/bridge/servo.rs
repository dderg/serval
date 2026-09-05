use super::{
    PyMotionEngine, PyResult, PyRuntimeError, Python, mcu_handle_from_raw, pymethods,
    slots_for_axis,
};
use crate::lock_ext::LockExt;
use crate::pump::{BuzzLane, BuzzParams, BuzzRoute, BuzzWave};
use crate::types::AxisKey;
use pyo3::types::PyAnyMethods;
use pyo3::{Bound, PyAny};
use std::sync::Arc;
use std::time::{Duration, Instant};

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
        require_py_endpoint_ok(result, |result| {
            format!("servo capture start failed: endpoint result {result}")
        })
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
        drives: Vec<(u8, u32, u16)>,
    ) -> PyResult<u64> {
        let conn = self.ethercat_conn(mcu_handle, "set_drive_limits")?;
        tracing::info!(
            subsystem = "engine",
            event = "servo_drive_limits",
            mcu_handle,
            drives = format!("{drives:?}").as_str(),
            "servo drive limits set"
        );
        let entries: Vec<mcu_protocol::messages::DriveLimitEntry> = drives
            .into_iter()
            .map(|(slot, following_error_counts, max_torque_tenth_pct)| {
                mcu_protocol::messages::DriveLimitEntry {
                    slot,
                    following_error_counts,
                    max_torque_tenth_pct,
                }
            })
            .collect();
        Ok(self.endpoint_calls.start("set_drive_limits", move || {
            let result = crate::servo_torque::send_drive_limits(&conn, entries)?;
            require_endpoint_ok(result, "set_drive_limits: SDO write failed")
        }))
    }
    fn restore_drive_limits_start(&self, mcu_handle: u32, slots: Vec<u8>) -> PyResult<u64> {
        let conn = self.ethercat_conn(mcu_handle, "restore_drive_limits")?;
        tracing::info!(
            subsystem = "engine",
            event = "servo_drive_limits",
            mcu_handle,
            slots = format!("{slots:?}").as_str(),
            "servo drive limits restored"
        );
        let slot_mask = slots.iter().fold(0u32, |m, &s| m | (1 << s));
        Ok(self.endpoint_calls.start("restore_drive_limits", move || {
            let result = crate::servo_torque::send_restore_drive_limits(&conn, slot_mask)?;
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
        require_py_endpoint_ok(result, |result| {
            format!("stop_node: endpoint rejected Stop: result {result}")
        })?;
        let result = crate::servo_torque::send_set_torque(&conn, false, 0)
            .map_err(PyRuntimeError::new_err)?;
        require_py_endpoint_ok(result, |result| {
            format!("stop_node: endpoint rejected torque disable: result {result}")
        })
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
            let host_now = self.router.lock_ok().host_now_secs();
            self.homing.note_arm(mcu_handle, endstop_id, host_now);
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
    /// One request covers every route the caller named, whatever transport
    /// each rides. Nothing arms until the pipeline is drained, the pump is
    /// fenced, and every route has validated, so a mixed pulse/phase/EtherCAT
    /// sweep either starts together or not at all.
    ///
    /// `wave` is `(freq_start_millihz, freq_end_millihz, amplitude_nm,
    /// duration_ms, ramp_ms)`. Each route is a tagged tuple, either
    /// `("ethercat", mcu_handle, slot_mask, slot_sign_mask)` or
    /// `("stepper", axis_mask, sign_mask)`; the stepper route is split into
    /// pulse and phase endpoints here, by the axis transport bindings.
    fn resonance_buzz(
        &self,
        py: Python<'_>,
        routes: Vec<Bound<'_, PyAny>>,
        wave: (u32, u32, u32, u32, u32),
    ) -> PyResult<()> {
        let specs = parse_buzz_routes(&routes)?;
        let params = BuzzParams {
            routes: self.build_buzz_routes(&specs)?,
            wave: BuzzWave {
                freq_start_millihz: wave.0,
                freq_end_millihz: wave.1,
                amplitude_nm: wave.2,
                duration_ms: wave.3,
                ramp_ms: wave.4,
            },
        };
        let rx = {
            let guard = self.planner.lock_ok();
            let planner = guard.as_ref().ok_or_else(|| {
                PyRuntimeError::new_err(
                    "resonance_buzz: planner not initialized — call init_planner first",
                )
            })?;
            planner
                .submit_buzz(params)
                .map_err(|e| PyRuntimeError::new_err(format!("resonance_buzz: {e}")))?
        };
        let token = py
            .detach(|| rx.recv())
            .map_err(|_| PyRuntimeError::new_err("resonance_buzz: notify dropped"))?
            .map_err(|e| PyRuntimeError::new_err(format!("resonance_buzz: {e}")))?;
        *self.buzz_token.lock_ok() = Some(token);
        Ok(())
    }

    /// Polls the one token the last [`Self::resonance_buzz`] handed back, so
    /// completion spans every route that request armed.
    fn resonance_buzz_done(&self) -> PyResult<bool> {
        self.buzz_token
            .lock_ok()
            .as_ref()
            .ok_or_else(|| {
                PyRuntimeError::new_err(
                    "resonance_buzz_done: no buzz has been armed on this engine",
                )
            })?
            .complete()
            .map_err(|e| PyRuntimeError::new_err(format!("resonance_buzz_done: {e}")))
    }
    #[allow(clippy::too_many_arguments)]
    fn set_diff_damper(
        &self,
        py: Python<'_>,
        mcu_handle: u32,
        slot_a: u8,
        slot_b: u8,
        gain_milli: u32,
        clamp_tenths: u16,
        lpf_millihz: u32,
        lead_us: u16,
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
            lead_us,
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
                        lead_us,
                    },
                )
            })
            .map_err(PyRuntimeError::new_err)?;
        require_py_endpoint_ok(result, |result| {
            format!("set_diff_damper: endpoint rejected (result {result})")
        })
    }
    fn set_ff_lead(&self, py: Python<'_>, mcu_handle: u32, slot: u8, lead_ns: u64) -> PyResult<()> {
        let conn = self.ethercat_conn(mcu_handle, "set_ff_lead")?;
        let ring = self.ring_filler(mcu_handle, "set_ff_lead")?;
        tracing::info!(
            subsystem = "engine",
            event = "servo_set_ff_lead",
            mcu_handle,
            slot,
            lead_ns,
            "servo feedforward lead"
        );
        py.detach(|| {
            reconfigure_feedforward(&conn, &ring, "set_ff_lead", |filler| {
                require_endpoint_ok(
                    crate::servo_torque::send_set_ff_lead(
                        &conn,
                        mcu_protocol::messages::SetFfLead { slot, lead_ns },
                    )?,
                    "set_ff_lead",
                )?;
                require_filler_ok(filler.set_ff_lead(slot as usize, lead_ns), "set_ff_lead")
            })
        })
        .map_err(PyRuntimeError::new_err)
    }
    #[allow(clippy::too_many_arguments)]
    fn set_strain_comp(
        &self,
        mcu_handle: u32,
        slot_a: u8,
        slot_b: u8,
        lane_a: u8,
        lane_b: u8,
        kinematics: u8,
        nx: u16,
        ny: u16,
        x0: f32,
        y0: f32,
        dx: f32,
        dy: f32,
        values_um: Vec<i32>,
    ) -> PyResult<()> {
        let conn = self.ethercat_conn(mcu_handle, "set_strain_comp")?;
        tracing::info!(
            subsystem = "engine",
            event = "servo_strain_comp",
            mcu_handle,
            slot_a,
            slot_b,
            nx,
            ny,
            values = values_um.len(),
            "servo strain compensation map upload"
        );
        let result = crate::servo_torque::send_set_strain_comp(
            &conn,
            mcu_protocol::messages::SetStrainComp {
                slot_a,
                slot_b,
                lane_a,
                lane_b,
                kinematics,
                nx,
                ny,
                x0,
                y0,
                dx,
                dy,
                values_um,
            },
        )
        .map_err(PyRuntimeError::new_err)?;
        require_py_endpoint_ok(result, |result| {
            format!("set_strain_comp: endpoint result {result}")
        })
    }
    fn set_diff_trim(
        &self,
        py: Python<'_>,
        mcu_handle: u32,
        slot_a: u8,
        slot_b: u8,
        gain_micro: u32,
        clamp_um: u16,
        lpf_millihz: u32,
        settle_ms: u32,
    ) -> PyResult<()> {
        let conn = self.ethercat_conn(mcu_handle, "set_diff_trim")?;
        tracing::info!(
            subsystem = "engine",
            event = "servo_set_diff_trim",
            mcu_handle,
            slot_a,
            slot_b,
            gain_micro,
            clamp_um,
            lpf_millihz,
            settle_ms,
            "servo differential trim"
        );
        let result = py
            .detach(|| {
                crate::servo_torque::send_set_diff_trim(
                    &conn,
                    mcu_protocol::messages::SetDiffTrim {
                        slot_a,
                        slot_b,
                        gain_micro,
                        clamp_um,
                        lpf_millihz,
                        settle_ms,
                    },
                )
            })
            .map_err(PyRuntimeError::new_err)?;
        require_py_endpoint_ok(result, |result| {
            format!("set_diff_trim: endpoint rejected (result {result})")
        })
    }
    #[allow(clippy::too_many_arguments)]
    fn set_dynamics_model(
        &self,
        py: Python<'_>,
        mcu_handle: u32,
        frame: Vec<f32>,
        mass: Vec<f32>,
        viscous: Vec<f32>,
        coulomb: Vec<f32>,
        compliance: Vec<f32>,
        pin_mass: Vec<f32>,
        pin_zeta: Vec<f32>,
        pin_lead_us: f32,
        pairs: Vec<u32>,
        direction_split: Vec<f32>,
    ) -> PyResult<()> {
        let modes = mass.len();
        if modes == 0 {
            return Err(PyRuntimeError::new_err(
                "set_dynamics_model: at least one mode required".to_string(),
            ));
        }
        if viscous.len() != modes
            || coulomb.len() != modes
            || compliance.len() != modes
            || pin_mass.len() != modes
            || pin_zeta.len() != modes
        {
            return Err(PyRuntimeError::new_err(format!(
                "set_dynamics_model: per-mode length mismatch (mass {modes}, \
                 viscous {}, coulomb {}, compliance {}, pin_mass {}, pin_zeta {})",
                viscous.len(),
                coulomb.len(),
                compliance.len(),
                pin_mass.len(),
                pin_zeta.len()
            )));
        }
        if frame.len() % modes != 0 {
            return Err(PyRuntimeError::new_err(format!(
                "set_dynamics_model: frame length {} not a multiple of {modes} modes",
                frame.len()
            )));
        }
        let slots = frame.len() / modes;
        let slots_count = u8::try_from(slots).map_err(|_| {
            PyRuntimeError::new_err(format!("set_dynamics_model: {slots} slots exceed u8"))
        })?;
        let modes_count = u8::try_from(modes).map_err(|_| {
            PyRuntimeError::new_err(format!("set_dynamics_model: {modes} modes exceed u8"))
        })?;
        if modes > slots {
            return Err(PyRuntimeError::new_err(format!(
                "set_dynamics_model: {modes} modes exceed {slots} slots"
            )));
        }
        let wire_pairs = validate_dynamics_pairs(&frame, modes, slots, &pairs, &direction_split)
            .map_err(PyRuntimeError::new_err)?;
        let pair_specs: Vec<ethercat_rt::dynamics::PairSpec> = wire_pairs
            .iter()
            .map(|pair| ethercat_rt::dynamics::PairSpec {
                first: pair.first as usize,
                second: pair.second as usize,
                direction_split: pair.direction_split,
            })
            .collect();
        let host_model = ethercat_rt::dynamics::DynamicsModel::from_parts(
            slots,
            modes,
            &frame,
            &mass,
            &viscous,
            &coulomb,
            &compliance,
            &pin_mass,
            &pin_zeta,
            f64::from(pin_lead_us),
            &pair_specs,
        )
        .map_err(|e| {
            PyRuntimeError::new_err(format!("set_dynamics_model: model rejected: {e:?}"))
        })?;
        let conn = self.ethercat_conn(mcu_handle, "set_dynamics_model")?;
        let ring = self.ring_filler(mcu_handle, "set_dynamics_model")?;
        tracing::info!(
            subsystem = "engine",
            event = "servo_set_dynamics_model",
            mcu_handle,
            slots_count,
            modes_count,
            pairs = wire_pairs.len(),
            "servo dynamics feedforward model upload"
        );
        let msg = mcu_protocol::messages::SetDynamicsModel {
            slots_count,
            modes_count,
            frame,
            mass,
            viscous,
            coulomb,
            compliance,
            pin_mass,
            pin_zeta,
            pin_lead_us,
            pairs: wire_pairs,
        };
        py.detach(|| {
            reconfigure_feedforward(&conn, &ring, "set_dynamics_model", |filler| {
                if host_model.n_slots != filler.lane_count() {
                    return Err(format!(
                        "set_dynamics_model: the model covers {} slots but the endpoint's \
                         filler drives {} lanes",
                        host_model.n_slots,
                        filler.lane_count()
                    ));
                }
                require_endpoint_ok(
                    crate::servo_torque::send_set_dynamics_model(&conn, msg)?,
                    "set_dynamics_model",
                )?;
                require_filler_ok(filler.install_dynamics(host_model), "set_dynamics_model")
            })
        })
        .map_err(PyRuntimeError::new_err)
    }
}

/// One route as Python named it, before any endpoint is resolved.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum BuzzRouteSpec {
    Ethercat {
        mcu_handle: u32,
        slot_mask: u8,
        sign_mask: u8,
    },
    Stepper {
        axis_mask: u8,
        sign_mask: u8,
    },
}

fn parse_buzz_routes(routes: &[Bound<'_, PyAny>]) -> PyResult<Vec<BuzzRouteSpec>> {
    if routes.is_empty() {
        return Err(PyRuntimeError::new_err(
            "resonance_buzz: no routes given — nothing to buzz",
        ));
    }
    let mut specs = Vec::with_capacity(routes.len());
    for route in routes {
        let arity = route.len()?;
        let kind: String = route.get_item(0)?.extract()?;
        specs.push(match (kind.as_str(), arity) {
            ("ethercat", 4) => BuzzRouteSpec::Ethercat {
                mcu_handle: route.get_item(1)?.extract()?,
                slot_mask: route.get_item(2)?.extract()?,
                sign_mask: route.get_item(3)?.extract()?,
            },
            ("stepper", 3) => BuzzRouteSpec::Stepper {
                axis_mask: route.get_item(1)?.extract()?,
                sign_mask: route.get_item(2)?.extract()?,
            },
            ("ethercat" | "stepper", n) => {
                return Err(PyRuntimeError::new_err(format!(
                    "resonance_buzz: route kind '{kind}' takes \
                     {} elements, got {n}",
                    if kind == "ethercat" { 4 } else { 3 }
                )));
            }
            (other, _) => {
                return Err(PyRuntimeError::new_err(format!(
                    "resonance_buzz: unknown route kind '{other}' \
                     (expected 'ethercat' or 'stepper')"
                )));
            }
        });
    }
    Ok(specs)
}

/// The subset of `axis_mask` an endpoint actually owns for this buzz.
pub(super) fn buzz_axis_bits(axis_mask: u8, keep: impl Fn(u8) -> bool) -> u8 {
    (0u8..8)
        .filter(|&axis| axis_mask & (1 << axis) != 0 && keep(axis))
        .fold(0u8, |bits, axis| bits | (1 << axis))
}

/// A phase route names its lanes outright: the sign mask is a per-axis
/// direction flip, not a mask of anything the endpoint has to decode.
pub(super) fn buzz_lanes(axis_bits: u8, sign_mask: u8) -> Vec<BuzzLane> {
    (0u8..8)
        .filter(|&axis| axis_bits & (1 << axis) != 0)
        .map(|axis| BuzzLane {
            axis,
            sign: if sign_mask & (1 << axis) != 0 {
                -1.0
            } else {
                1.0
            },
        })
        .collect()
}

impl PyMotionEngine {
    /// The host-side setpoint filler of a claimed EtherCAT node. Only an
    /// EtherCAT connection has one, so a handle without a filler cannot
    /// execute setpoints at all.
    fn ring_filler(&self, mcu_handle: u32, what: &str) -> PyResult<crate::pump::RingFiller> {
        self.mcus
            .lock_ok()
            .get(&mcu_handle)
            .and_then(|mcu| mcu.ring_filler.clone())
            .ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "{what}: mcu_handle {mcu_handle} has no EtherCAT setpoint filler"
                ))
            })
    }

    /// Resolve every spec into a live endpoint handle. Every lookup happens
    /// here, before the request leaves the Python thread, so a missing
    /// endpoint or an empty mask is a loud failure with nothing armed.
    fn build_buzz_routes(&self, specs: &[BuzzRouteSpec]) -> PyResult<Arc<[BuzzRoute]>> {
        let transports = Arc::clone(&self.axis_transports.lock_ok());
        let mut routes: Vec<BuzzRoute> = Vec::new();
        for spec in specs {
            match *spec {
                BuzzRouteSpec::Ethercat {
                    mcu_handle,
                    slot_mask,
                    sign_mask,
                } => {
                    if slot_mask == 0 {
                        return Err(PyRuntimeError::new_err(
                            "resonance_buzz: ethercat route has an empty slot mask",
                        ));
                    }
                    let filler = self.ring_filler(mcu_handle, "resonance_buzz")?;
                    routes.push(BuzzRoute::Ethercat {
                        mcu_id: mcu_handle,
                        filler,
                        slot_mask,
                        sign_mask,
                    });
                }
                BuzzRouteSpec::Stepper {
                    axis_mask,
                    sign_mask,
                } => {
                    if axis_mask == 0 {
                        return Err(PyRuntimeError::new_err(
                            "resonance_buzz: stepper route has an empty axis mask",
                        ));
                    }
                    let selected = routes.len();
                    let mut pulse: Vec<_> = self
                        .stepcompress_endpoints
                        .lock_ok()
                        .iter()
                        .map(|(&mcu_id, endpoint)| (mcu_id, Arc::clone(endpoint)))
                        .collect();
                    pulse.sort_by_key(|(mcu_id, _)| *mcu_id);
                    for (mcu_id, endpoint) in pulse {
                        let bits = {
                            let ep = endpoint.lock_ok();
                            buzz_axis_bits(axis_mask, |axis| {
                                ep.drives_axis(axis)
                                    && !transports.is_phase(AxisKey { mcu_id, axis })
                            })
                        };
                        if bits != 0 {
                            routes.push(BuzzRoute::Pulse {
                                mcu_id,
                                endpoint,
                                axis_mask: bits,
                                sign_mask,
                            });
                        }
                    }
                    let mut phase: Vec<_> = self
                        .sample_endpoints
                        .lock_ok()
                        .iter()
                        .map(|(&mcu_id, endpoint)| (mcu_id, Arc::clone(endpoint)))
                        .collect();
                    phase.sort_by_key(|(mcu_id, _)| *mcu_id);
                    for (mcu_id, endpoint) in phase {
                        let bits = {
                            let ep = endpoint.lock_ok();
                            buzz_axis_bits(axis_mask, |axis| {
                                ep.drives_axis(axis)
                                    && transports.is_phase(AxisKey { mcu_id, axis })
                            })
                        };
                        if bits != 0 {
                            routes.push(BuzzRoute::Phase {
                                mcu_id,
                                endpoint,
                                lanes: buzz_lanes(bits, sign_mask),
                            });
                        }
                    }
                    if routes.len() == selected {
                        return Err(PyRuntimeError::new_err(format!(
                            "resonance_buzz: axis mask 0x{axis_mask:02x} selects no \
                             pulse or phase endpoint"
                        )));
                    }
                }
            }
        }
        Ok(routes.into())
    }
}

pub(super) fn validate_dynamics_pairs(
    frame: &[f32],
    modes: usize,
    slots: usize,
    pairs: &[u32],
    direction_split: &[f32],
) -> Result<Vec<mcu_protocol::messages::DynamicsPair>, String> {
    if pairs.len() % 2 != 0 {
        return Err(format!(
            "set_dynamics_model: pairs must be flat [first, second, ...], got {} entries",
            pairs.len()
        ));
    }
    let pair_count = pairs.len() / 2;
    if direction_split.len() != pair_count {
        return Err(format!(
            "set_dynamics_model: direction_split must have one coefficient per pair \
             ({pair_count} expected, got {})",
            direction_split.len()
        ));
    }
    if pair_count > u8::MAX as usize {
        return Err(format!("set_dynamics_model: {pair_count} pairs exceed u8"));
    }
    let wire_pairs = pairs
        .chunks_exact(2)
        .zip(direction_split.iter().copied())
        .map(|(pair, direction_split)| {
            let first = u8::try_from(pair[0]).map_err(|_| {
                format!("set_dynamics_model: pair slot {} exceeds u8", pair[0])
            })?;
            let second = u8::try_from(pair[1]).map_err(|_| {
                format!("set_dynamics_model: pair slot {} exceeds u8", pair[1])
            })?;
            if first == second || first as usize >= slots || second as usize >= slots {
                return Err(format!(
                    "set_dynamics_model: invalid pair slots ({first}, {second}) for {slots} slots"
                ));
            }
            if (0..modes).all(|mode| frame[mode * slots + first as usize] == 0.0) {
                return Err(format!(
                    "set_dynamics_model: pair slots ({first}, {second}) first frame column must be nonzero"
                ));
            }
            let columns_match = |lambda: f32| {
                (0..modes).all(|mode| {
                    frame[mode * slots + second as usize]
                        == lambda * frame[mode * slots + first as usize]
                })
            };
            if !columns_match(1.0) && !columns_match(-1.0) {
                return Err(format!(
                    "set_dynamics_model: pair slots ({first}, {second}) must have exact equal or opposite frame columns"
                ));
            }
            if !direction_split.is_finite() || direction_split.abs() >= 0.5 {
                return Err(format!(
                    "set_dynamics_model: direction_split must be finite with abs < 0.5, got {direction_split}"
                ));
            }
            Ok(mcu_protocol::messages::DynamicsPair {
                first,
                second,
                direction_split,
            })
        })
        .collect::<Result<Vec<_>, String>>()?;
    let mut used = vec![false; slots];
    for pair in &wire_pairs {
        for slot in [pair.first as usize, pair.second as usize] {
            if std::mem::replace(&mut used[slot], true) {
                return Err(format!(
                    "set_dynamics_model: slot {slot} appears in more than one pair"
                ));
            }
        }
    }
    Ok(wire_pairs)
}

fn require_py_endpoint_ok(result: i32, error: impl FnOnce(i32) -> String) -> PyResult<()> {
    (result == 0)
        .then_some(())
        .ok_or_else(|| PyRuntimeError::new_err(error(result)))
}

fn require_endpoint_ok(result: i32, context: &str) -> Result<(), String> {
    if result != 0 {
        return Err(format!("{context}: endpoint result {result}"));
    }
    Ok(())
}

fn require_filler_ok(result: i32, context: &str) -> Result<(), String> {
    if result != 0 {
        return Err(format!(
            "{context}: host filler refused it (result {result})"
        ));
    }
    Ok(())
}

/// Reading the endpoint's grid is one control call, so it gets the same
/// budget as the reconfiguration it precedes.
const RECONFIG_GRID_TIMEOUT: Duration = Duration::from_secs(5);

/// A feedforward change has to land on one side of every sample: the filler
/// computes each sample's velocity and torque feedforward, the endpoint only
/// clamps it and adds the pin. The grid is re-read first — the pair the filler
/// holds was reported at fill time, so nothing else tells it whether the
/// samples it already emitted have played — and the endpoint call plus the
/// filler update run under the filler lock, so no drain can slip a sample of
/// the old configuration in between. Motion still outstanding is refused, not
/// split.
fn reconfigure_feedforward<T>(
    conn: &host_rt::mcu_serial_conn::McuSerialConn,
    ring: &crate::pump::RingFiller,
    what: &str,
    apply: impl FnOnce(&mut ethercat_rt::setpoint_fill::ChainFiller) -> Result<T, String>,
) -> Result<T, String> {
    let mut filler = ring.lock_ok();
    let grid =
        super::ethercat_endpoint::verify_sample_grid(conn, Instant::now() + RECONFIG_GRID_TIMEOUT)
            .map_err(|e| format!("{what}: the endpoint's sample grid is unreadable: {e:?}"))?;
    filler
        .observe_grid(grid.grid_index, grid.grid_clock)
        .map_err(|e| format!("{what}: the endpoint's sample grid was refused: {e:?}"))?;
    if !filler.quiescent() {
        return Err(format!(
            "{what}: the endpoint still has setpoints outstanding — changing the feedforward \
             mid-stream would step the velocity and torque feedforward; wait for the motion to \
             finish"
        ));
    }
    apply(&mut filler)
}
