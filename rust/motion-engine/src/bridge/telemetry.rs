use super::{
    DRAIN_TIMEOUT, FlushWait, HashMap, Ordering, PyMotionEngine, PyResult, PyRuntimeError, Python,
    collect_motor_positions_inner, planner_err, pymethods,
};

#[pymethods]
impl PyMotionEngine {
    #[pyo3(signature = (mcu_handle, bus_id, rate, timeout_s = 5.0))]
    fn register_phase_bus(
        &self,
        py: Python<'_>,
        mcu_handle: u32,
        bus_id: u8,
        rate: u32,
        timeout_s: f64,
    ) -> PyResult<()> {
        let io = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            let conn = mcus.get(&mcu_handle).ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "register_phase_bus: unknown mcu_handle {mcu_handle}"
                ))
            })?;
            if !conn.mcu_transport_supported {
                return Ok(());
            }
            conn.host_io
                .as_ref()
                .ok_or_else(|| {
                    PyRuntimeError::new_err(
                        "register_phase_bus: attach_serial has not been called for this MCU",
                    )
                })?
                .clone()
        };
        let timeout = std::time::Duration::from_secs_f64(timeout_s);
        let msg = format!("runtime_register_phase_bus bus_id={bus_id} rate={rate}");
        let params = py.detach(|| -> PyResult<_> {
            use host_rt::transport::Transport;
            io.call(&msg, "kalico_register_phase_bus_response", timeout)
                .map_err(|e| {
                    PyRuntimeError::new_err(format!("register_phase_bus: transport error: {e:?}"))
                })
        })?;
        let result = params.try_get_i32("result").ok_or_else(|| {
            PyRuntimeError::new_err(
                "register_phase_bus: response missing or non-integer result field",
            )
        })?;
        if result != 0 {
            return Err(PyRuntimeError::new_err(format!(
                "register_phase_bus: MCU returned error {result} (bus_id={bus_id})"
            )));
        }
        Ok(())
    }
    #[pyo3(signature = (mcu_handle, motor_idx, bus_id, cs_pin_id, slot_idx, timeout_s = 5.0))]
    fn register_phase_motor(
        &self,
        py: Python<'_>,
        mcu_handle: u32,
        motor_idx: u8,
        bus_id: u8,
        cs_pin_id: u8,
        slot_idx: u8,
        timeout_s: f64,
    ) -> PyResult<()> {
        let io = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            let conn = mcus.get(&mcu_handle).ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "register_phase_motor: unknown mcu_handle {mcu_handle}"
                ))
            })?;
            if !conn.mcu_transport_supported {
                return Ok(());
            }
            conn.host_io
                .as_ref()
                .ok_or_else(|| {
                    PyRuntimeError::new_err(
                        "register_phase_motor: attach_serial has not been called for this MCU",
                    )
                })?
                .clone()
        };
        let timeout = std::time::Duration::from_secs_f64(timeout_s);
        let msg = format!(
            "runtime_register_phase_motor motor_idx={motor_idx} \
             bus_id={bus_id} cs_pin_id={cs_pin_id} slot_idx={slot_idx}"
        );
        let params = py.detach(|| -> PyResult<_> {
            use host_rt::transport::Transport;
            io.call(&msg, "kalico_register_phase_motor_response", timeout)
                .map_err(|e| {
                    PyRuntimeError::new_err(format!("register_phase_motor: transport error: {e:?}"))
                })
        })?;
        let result = params.try_get_i32("result").ok_or_else(|| {
            PyRuntimeError::new_err(
                "register_phase_motor: response missing or non-integer result field",
            )
        })?;
        if result != 0 {
            return Err(PyRuntimeError::new_err(format!(
                "register_phase_motor: MCU returned error {result} \
                 (motor_idx={motor_idx} bus_id={bus_id} cs_pin_id={cs_pin_id})"
            )));
        }
        Ok(())
    }
    fn wait_moves(&self, py: Python<'_>) -> PyResult<()> {
        let guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
        let planner = guard.as_ref().ok_or_else(|| {
            PyRuntimeError::new_err("planner not initialized — call init_planner first")
        })?;
        py.detach(|| planner.flush()).map_err(planner_err)
    }
    fn drain_motion(&self, py: Python<'_>) -> PyResult<()> {
        let guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
        let planner = guard.as_ref().ok_or_else(|| {
            PyRuntimeError::new_err("planner not initialized — call init_planner first")
        })?;
        py.detach(|| planner.flush()).map_err(planner_err)?;
        let drain = self.drain.clone();
        py.detach(|| drain.wait_drained(DRAIN_TIMEOUT))
            .map_err(PyRuntimeError::new_err)
    }
    fn wait_moves_start(&self) -> PyResult<u64> {
        let rx = {
            let guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
            let planner = guard.as_ref().ok_or_else(|| {
                PyRuntimeError::new_err("planner not initialized — call init_planner first")
            })?;
            planner.flush_start().map_err(planner_err)?
        };
        let mut pending = self
            .pending_flushes
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        let id = self.next_flush_id.fetch_add(1, Ordering::Relaxed);
        pending.insert(id, FlushWait { rx, deadline: None });
        Ok(id)
    }
    fn wait_moves_poll(&self, flush_id: u64) -> PyResult<bool> {
        let mut pending = self
            .pending_flushes
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        let Some(wait) = pending.get_mut(&flush_id) else {
            return Err(PyRuntimeError::new_err(format!(
                "wait_moves_poll: unknown flush id {flush_id}"
            )));
        };
        if wait.deadline.is_none() {
            match wait.rx.try_recv() {
                Ok(finish) => {
                    wait.deadline = Some(finish.unwrap_or_else(std::time::Instant::now));
                }
                Err(crossbeam_channel::TryRecvError::Empty) => return Ok(false),
                Err(crossbeam_channel::TryRecvError::Disconnected) => {
                    pending.remove(&flush_id);
                    return Err(PyRuntimeError::new_err(
                        "wait_moves_poll: planner channel closed",
                    ));
                }
            }
        }
        let done = wait
            .deadline
            .map(|d| std::time::Instant::now() >= d)
            .unwrap_or(false);
        if done {
            pending.remove(&flush_id);
        }
        Ok(done)
    }
    fn motion_drain_poll(&self, py: Python<'_>) -> PyResult<bool> {
        let guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
        let planner = guard.as_ref().ok_or_else(|| {
            PyRuntimeError::new_err("planner not initialized — call init_planner first")
        })?;
        py.detach(|| planner.flush()).map_err(planner_err)?;
        Ok(self.drain.is_drained_now())
    }
    fn motion_drain_finalize(&self) {}
    fn pending_channel_moves(&self) -> u64 {
        self.planner
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .as_ref()
            .map_or(0, |p| p.pending_channel_moves() as u64)
    }
    fn input_channel_capacity(&self) -> u64 {
        crate::worker::INPUT_CHANNEL_CAP as u64
    }
    fn fence_start(&self, force: bool) -> PyResult<u64> {
        let guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
        let planner = guard.as_ref().ok_or_else(|| {
            PyRuntimeError::new_err("planner not initialized — call init_planner first")
        })?;
        planner.fence_start(force).map_err(planner_err)
    }
    /// `None` while pending; once resolved, the seconds from now until the
    /// fenced motion ends (0.0 when the stream was reset or already idle).
    /// Consumes the resolution.
    fn fence_poll(&self, id: u64) -> Option<f64> {
        let taken = {
            let guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
            guard.as_ref()?.fence_take(id)
        }?;
        let Some(t_end) = taken else {
            return Some(0.0);
        };
        let anchored = self
            .dispatch_anchor
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .t0();
        let Some(t0) = anchored else {
            return Some(0.0);
        };
        let host_now = self
            .router
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .host_now_secs();
        Some((t0 + t_end - host_now).max(0.0))
    }
    fn get_last_move_time(&self) -> f64 {
        match self
            .planner
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .as_ref()
        {
            Some(p) => p.last_move_time(),
            None => 0.0,
        }
    }
    pub(crate) fn queued_motion_secs(&self) -> f64 {
        self.committed_lead_secs()
    }
    fn dispatched_lead_secs(&self) -> f64 {
        self.committed_lead_secs()
    }
    pub(crate) fn committed_lead_secs(&self) -> f64 {
        let last_move_time = {
            let planner = self.planner.lock().unwrap_or_else(|p| p.into_inner());
            let Some(p) = planner.as_ref() else {
                return 0.0;
            };
            p.last_move_time()
        };
        let anchored = self
            .dispatch_anchor
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .t0();
        let Some(t0) = anchored else {
            return 0.0;
        };
        let host_now = self
            .router
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .host_now_secs();
        (t0 + last_move_time - host_now).max(0.0)
    }
    fn pump_backlog(&self) -> u64 {
        self.pump_backlog.load(Ordering::Acquire)
    }
    fn motion_lead_secs(&self) -> f64 {
        crate::anchor::DEFAULT_LEAD_SECS
    }
    fn dispatched_segment_count(&self) -> u64 {
        self.dispatched_segments.load(Ordering::Relaxed)
    }
    fn fallback_clock_conversions(&self) -> u64 {
        self.fallback_clock_conversions.load(Ordering::Relaxed)
    }
    #[pyo3(signature = (timeout_s=0.25))]
    fn query_motor_positions(
        &self,
        py: Python<'_>,
        timeout_s: f64,
    ) -> PyResult<HashMap<String, (f64, f64)>> {
        let timeout = std::time::Duration::from_secs_f64(timeout_s.max(0.0));
        py.detach(|| collect_motor_positions_inner(&self.mcu_axis_configs, &self.mcus, timeout))
            .map_err(PyRuntimeError::new_err)
    }
    fn live_motor_positions(&self) -> std::collections::HashMap<String, (f64, f64)> {
        self.live_position_cache
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .0
            .clone()
    }
}
