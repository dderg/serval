use super::{
    DRAIN_TIMEOUT, FlushWait, HashMap, Ordering, PyMotionEngine, PyResult, PyRuntimeError, Python,
    collect_motor_positions_inner, planner_err, pymethods,
};
use crate::lock_ext::LockExt;

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
        let msg = format!("runtime_register_phase_bus bus_id={bus_id} rate={rate}");
        self.phase_register_call(
            py,
            "register_phase_bus",
            mcu_handle,
            &msg,
            "kalico_register_phase_bus_response",
            timeout_s,
            &format!("(bus_id={bus_id})"),
        )
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
        let msg = format!(
            "runtime_register_phase_motor motor_idx={motor_idx} \
             bus_id={bus_id} cs_pin_id={cs_pin_id} slot_idx={slot_idx}"
        );
        self.phase_register_call(
            py,
            "register_phase_motor",
            mcu_handle,
            &msg,
            "kalico_register_phase_motor_response",
            timeout_s,
            &format!("(motor_idx={motor_idx} bus_id={bus_id} cs_pin_id={cs_pin_id})"),
        )
    }
    fn wait_moves(&self, py: Python<'_>) -> PyResult<()> {
        let guard = self.planner.lock_ok();
        let planner = guard.as_ref().ok_or_else(|| {
            PyRuntimeError::new_err("planner not initialized — call init_planner first")
        })?;
        py.detach(|| planner.flush()).map_err(planner_err)
    }
    fn drain_motion(&self, py: Python<'_>) -> PyResult<()> {
        let guard = self.planner.lock_ok();
        let planner = guard.as_ref().ok_or_else(|| {
            PyRuntimeError::new_err("planner not initialized — call init_planner first")
        })?;
        py.detach(|| planner.flush()).map_err(planner_err)?;
        let drain = self.drain.clone();
        py.detach(|| drain.wait_drained(DRAIN_TIMEOUT))
            .map_err(PyRuntimeError::new_err)
    }
    fn wait_moves_start(&self) -> PyResult<u64> {
        let rx = self.flush_try_start_inner()?;
        let mut pending = self.flush.pending.lock_ok();
        let id = self.flush.next_id.fetch_add(1, Ordering::Relaxed);
        pending.insert(id, FlushWait { rx, deadline: None });
        Ok(id)
    }
    fn wait_moves_poll(&self, flush_id: u64) -> PyResult<bool> {
        let started_rx = {
            let pending = self.flush.pending.lock_ok();
            let Some(wait) = pending.get(&flush_id) else {
                return Err(PyRuntimeError::new_err(format!(
                    "wait_moves_poll: unknown flush id {flush_id}"
                )));
            };
            wait.rx.is_some()
        };
        let late_rx = if started_rx {
            None
        } else {
            match self.flush_try_start_inner()? {
                Some(rx) => Some(rx),
                None => return Ok(false),
            }
        };
        let mut pending = self.flush.pending.lock_ok();
        let Some(wait) = pending.get_mut(&flush_id) else {
            return Err(PyRuntimeError::new_err(format!(
                "wait_moves_poll: unknown flush id {flush_id}"
            )));
        };
        if let Some(rx) = late_rx {
            wait.rx = Some(rx);
        }
        if wait.deadline.is_none() {
            let rx = wait
                .rx
                .as_ref()
                .expect("flush receiver present past the try-start gate");
            match rx.try_recv() {
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
    fn motion_drain_poll(&self) -> PyResult<bool> {
        self.report_lagging_drain_wait();
        let mut pending = self.flush.pending_drain.lock_ok();
        if pending.is_none() {
            match self.flush_try_start_inner()? {
                Some(rx) => *pending = Some(rx),
                None => return Ok(false),
            }
        }
        let rx = pending
            .as_ref()
            .expect("drain flush receiver just installed");
        match rx.try_recv() {
            Ok(_committed_through) => {
                *pending = None;
                let drained = self.drain.drained();
                if drained {
                    *self.flush.drain_wait_diag.lock_ok() = None;
                }
                Ok(drained)
            }
            Err(crossbeam_channel::TryRecvError::Empty) => Ok(false),
            Err(crossbeam_channel::TryRecvError::Disconnected) => {
                *pending = None;
                Err(PyRuntimeError::new_err(
                    "motion_drain_poll: planner channel closed",
                ))
            }
        }
    }
    fn motion_drain_finalize(&self) {
        *self.flush.pending_drain.lock_ok() = None;
        *self.flush.drain_wait_diag.lock_ok() = None;
    }
    fn pending_channel_moves(&self) -> u64 {
        self.planner
            .lock_ok()
            .as_ref()
            .map_or(0, |p| p.pending_channel_moves() as u64)
    }
    fn input_channel_capacity(&self) -> u64 {
        crate::worker::INPUT_CHANNEL_CAP as u64
    }
    /// `None` when the move channel is full — the caller yields and retries;
    /// blocking here would stall the whole klippy reactor thread.
    fn fence_start(&self, force: bool) -> PyResult<Option<u64>> {
        let guard = self.planner.lock_ok();
        let planner = guard.as_ref().ok_or_else(|| {
            PyRuntimeError::new_err("planner not initialized — call init_planner first")
        })?;
        match planner.fence_start(force) {
            Ok(id) => Ok(Some(id)),
            Err(crate::worker::StreamWorkerError::ChannelFull) => Ok(None),
            Err(e) => Err(planner_err(e)),
        }
    }
    /// `None` while pending; once resolved, the seconds from now until the
    /// fenced motion ends (0.0 when the stream was reset or already idle).
    /// Consumes the resolution.
    fn fence_poll(&self, id: u64) -> Option<f64> {
        let taken = {
            let guard = self.planner.lock_ok();
            guard.as_ref()?.fence_take(id)
        }?;
        let Some(t_end) = taken else {
            return Some(0.0);
        };
        let anchored = self.dispatch_anchor.lock_ok().t0();
        let Some(t0) = anchored else {
            return Some(0.0);
        };
        let host_now = self.router.lock_ok().host_now_secs();
        Some((t0 + t_end - host_now).max(0.0))
    }
    fn get_last_move_time(&self) -> f64 {
        match self.planner.lock_ok().as_ref() {
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
            let planner = self.planner.lock_ok();
            let Some(p) = planner.as_ref() else {
                return 0.0;
            };
            p.last_move_time()
        };
        let anchored = self.dispatch_anchor.lock_ok().t0();
        let Some(t0) = anchored else {
            return 0.0;
        };
        let host_now = self.router.lock_ok().host_now_secs();
        (t0 + last_move_time - host_now).max(0.0)
    }
    fn pump_backlog(&self) -> u64 {
        self.pump.backlog.load(Ordering::Acquire)
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
        self.position_poll.cache.lock_ok().0.clone()
    }
}

impl PyMotionEngine {
    #[allow(clippy::too_many_arguments)]
    fn phase_register_call(
        &self,
        py: Python<'_>,
        op: &str,
        mcu_handle: u32,
        request: &str,
        response: &str,
        timeout_s: f64,
        err_ctx: &str,
    ) -> PyResult<()> {
        let io = {
            let mcus = self.mcus.lock_ok();
            let conn = mcus.get(&mcu_handle).ok_or_else(|| {
                PyRuntimeError::new_err(format!("{op}: unknown mcu_handle {mcu_handle}"))
            })?;
            if !conn.mcu_transport_supported {
                return Ok(());
            }
            conn.host_io
                .as_ref()
                .ok_or_else(|| {
                    PyRuntimeError::new_err(format!(
                        "{op}: attach_serial has not been called for this MCU"
                    ))
                })?
                .clone()
        };
        let timeout = std::time::Duration::from_secs_f64(timeout_s);
        let params = py.detach(|| -> PyResult<_> {
            use host_rt::transport::Transport;
            io.call(request, response, timeout)
                .map_err(|e| PyRuntimeError::new_err(format!("{op}: transport error: {e:?}")))
        })?;
        let result = params.try_get_i32("result").ok_or_else(|| {
            PyRuntimeError::new_err(format!(
                "{op}: response missing or non-integer result field"
            ))
        })?;
        if result != 0 {
            return Err(PyRuntimeError::new_err(format!(
                "{op}: MCU returned error {result} {err_ctx}"
            )));
        }
        Ok(())
    }

    fn report_lagging_drain_wait(&self) {
        const DRAIN_WAIT_REPORT_AFTER: std::time::Duration = std::time::Duration::from_secs(5);
        let now = std::time::Instant::now();
        let mut diag = self.flush.drain_wait_diag.lock_ok();
        let (started, last_report) = diag.get_or_insert((now, None));
        if now.duration_since(*started) < DRAIN_WAIT_REPORT_AFTER {
            return;
        }
        if last_report.is_some_and(|t| now.duration_since(t) < DRAIN_WAIT_REPORT_AFTER) {
            return;
        }
        *last_report = Some(now);
        let waited_s = now.duration_since(*started).as_secs_f64();
        drop(diag);
        for (mcu, axis, state) in self.drain.lagging_axes() {
            tracing::warn!(
                subsystem = "motion",
                event = "drain_wait_lagging",
                mcu,
                axis,
                pending = state.pending,
                pushed = state.pushed,
                retired = state.retired,
                waited_s,
                "drain wait not completing — this axis still has staged or \
                 unretired wire pieces"
            );
        }
    }

    fn flush_try_start_inner(
        &self,
    ) -> PyResult<Option<crossbeam_channel::Receiver<Option<std::time::Instant>>>> {
        let guard = self.planner.lock_ok();
        let planner = guard.as_ref().ok_or_else(|| {
            PyRuntimeError::new_err("planner not initialized — call init_planner first")
        })?;
        match planner.flush_try_start() {
            Ok(rx) => Ok(Some(rx)),
            Err(crate::worker::StreamWorkerError::ChannelFull) => Ok(None),
            Err(e) => Err(planner_err(e)),
        }
    }
}
