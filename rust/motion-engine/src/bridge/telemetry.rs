use super::{
    DRAIN_TIMEOUT, FlushWait, HashMap, Ordering, PyMotionEngine, PyResult, PyRuntimeError, Python,
    collect_motor_positions_inner, planner_err, pymethods,
};
use crate::lock_ext::LockExt;
use crate::types::mcu_handle_from_raw;
use host_rt::clock::{HostSecs, PrintTime};

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
                    self.report_drain_wait_done();
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
    /// Fd the host reactor registers for engine readiness: it becomes
    /// readable when input-channel space frees after a refused submit, and
    /// on every fence resolution. Owned by the engine — never close it.
    fn feed_wakeup_fd(&self) -> PyResult<i32> {
        let guard = self.planner.lock_ok();
        let planner = guard.as_ref().ok_or_else(|| {
            PyRuntimeError::new_err("planner not initialized — call init_planner first")
        })?;
        Ok(planner.feed_wakeup_read_fd())
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
    /// `None` while the fence is pending; once resolved, the absolute
    /// print_time at which everything submitted before the fence ends —
    /// the fenced trajectory's end projected through the same clock record
    /// the pump schedules pieces with. An idle or reset stream resolves to
    /// now. Consumes the resolution. Not floored: callers that schedule MCU
    /// commands against this apply their own scheduling floor
    /// (`Motion._schedule_floor`).
    ///
    /// Absolute results are the authority's contract: a "seconds from now"
    /// lead decays between the instant it is computed and the instant the
    /// caller adds its own "now" to it, and the two nows never match.
    fn fence_print_time_poll(&self, id: u64, mcu_handle: u32) -> PyResult<Option<f64>> {
        let taken = {
            let guard = self.planner.lock_ok();
            let planner = guard.as_ref().ok_or_else(|| {
                PyRuntimeError::new_err("planner not initialized — call init_planner first")
            })?;
            planner.fence_take(id)
        };
        let Some(fenced_end) = taken else {
            return Ok(None);
        };
        let t0 = self.dispatch_anchor.lock_ok().t0();
        Ok(Some(self.timeline_end_print_time(
            mcu_handle,
            fenced_end,
            t0,
            "fence_print_time_poll",
        )?))
    }

    /// Absolute print_time at which everything committed to the MCUs ends —
    /// segments, dwells, and nudges alike. This is the engine's own frontier;
    /// it replaces the host-side shadow clock that used to approximate it.
    /// The value sits in the past while the printer is idle — deliberately:
    /// idle detection measures `est_now − frontier`, so flooring it to now
    /// would keep the printer "busy" forever. Schedulers floor it themselves.
    /// Cheap: two atomics and one clock read.
    fn frontier_print_time(&self, mcu_handle: u32) -> PyResult<f64> {
        let last_move_time = {
            let guard = self.planner.lock_ok();
            let planner = guard.as_ref().ok_or_else(|| {
                PyRuntimeError::new_err("planner not initialized — call init_planner first")
            })?;
            planner.last_move_time()
        };
        let t0 = self.dispatch_anchor.lock_ok().t0();
        self.timeline_end_print_time(mcu_handle, Some(last_move_time), t0, "frontier_print_time")
    }

    /// Estimated print_time at this instant, from the router's clock record
    /// for `mcu_handle` — the same record the pump projects pieces with.
    /// `None` until a clock estimate is established. Meaningful only for the
    /// MCU whose clock defines the print_time timeline (the primary).
    fn print_time_now(&self, mcu_handle: u32) -> Option<f64> {
        self.router
            .lock_ok()
            .print_time_now(mcu_handle_from_raw(mcu_handle))
            .map(PrintTime::get)
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
        let machine = py
            .detach(|| collect_motor_positions_inner(&self.mcu_axis_configs, &self.mcus, timeout))
            .map_err(PyRuntimeError::new_err)?;
        self.motor_map_to_gcode(machine)
    }
    fn live_motor_positions(&self) -> PyResult<std::collections::HashMap<String, (f64, f64)>> {
        let machine = self.position_poll.cache.lock_ok().0.clone();
        self.motor_map_to_gcode(machine)
    }
}

impl PyMotionEngine {
    /// Project a stream-time end onto the print_time timeline: `t0 +
    /// stream_end` in the anchor's host frame, converted through the router's
    /// record for `mcu_handle` — all against one clock read, so no caller
    /// ever composes an absolute time from two different nows. The result is
    /// NOT floored to now; an ended timeline reports the past honestly.
    ///
    /// A `stream_end` of exactly 0.0 is the dispatcher's "nothing dispatched
    /// on this timeline" state (a real segment always ends past 0.0), and a
    /// reset leaves the anchor's `t0` pointing at the abandoned timeline —
    /// both mean the committed timeline holds no motion, so its end is now.
    fn timeline_end_print_time(
        &self,
        mcu_handle: u32,
        stream_end: Option<f64>,
        t0: Option<f64>,
        what: &str,
    ) -> PyResult<f64> {
        let mcu = mcu_handle_from_raw(mcu_handle);
        let router = self.router.lock_ok();
        let now = router.print_time_now(mcu).ok_or_else(|| {
            PyRuntimeError::new_err(format!(
                "{what}: no clock estimate established for mcu_handle {mcu_handle}"
            ))
        })?;
        let pt = match (stream_end, t0) {
            (Some(end), Some(t0)) if end > 0.0 => router
                .print_time_at_host(mcu, HostSecs::from_anchor_frame(t0 + end))
                .expect("print_time_now succeeded against the same record"),
            _ => now,
        };
        Ok(pt.get())
    }

    /// Live/queried motor states are machine space; every Python consumer
    /// compares them against gcode-space toolhead positions, so the Z lane
    /// crosses through the active mesh here (position via the exact inverse,
    /// velocity via the chain rule with zero-accel XY). Identity without a
    /// mesh.
    fn motor_map_to_gcode(
        &self,
        mut map: std::collections::HashMap<String, (f64, f64)>,
    ) -> PyResult<std::collections::HashMap<String, (f64, f64)>> {
        let mesh = self.bed_mesh.lock_ok();
        let Some(t) = mesh.as_deref() else {
            return Ok(map);
        };
        let Some(&(zp, zv)) = map.get("z") else {
            return Ok(map);
        };
        let (Some(&(xp, xv)), Some(&(yp, yv))) = (map.get("x"), map.get("y")) else {
            return Err(PyRuntimeError::new_err(
                "motor positions: a bed mesh is active but the response has a Z lane \
                 without X/Y — cannot unwarp machine Z without the XY the mesh was \
                 sampled at",
            ));
        };
        let (gz, gzv, _) = t.unwarp_z_state([xp, yp], [xv, yv], [0.0, 0.0], (zp, zv, 0.0));
        map.insert("z".to_string(), (gz, gzv));
        Ok(map)
    }

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
        let now = std::time::Instant::now();
        let outstanding_secs = self.committed_lead_secs();
        let report = {
            let mut diag = self.flush.drain_wait_diag.lock_ok();
            diag.get_or_insert_with(|| super::drain_wait::DrainWaitDiag::new(now, outstanding_secs))
                .poll(now, outstanding_secs)
        };
        let Some(report) = report else {
            return;
        };
        for (mcu, axis, state) in self.drain.lagging_axes() {
            tracing::warn!(
                subsystem = "motion",
                event = "drain_wait_lagging",
                mcu,
                axis,
                pending = state.pending,
                pushed = state.pushed,
                retired = state.retired,
                staged_motion = state.staged_motion,
                hold_tail = state.hold_tail,
                waited_s = report.waited_s,
                overdue_s = report.overdue_s,
                horizon_s = report.horizon_s,
                "drain wait is past the committed-motion horizon — this axis still \
                 has staged or unretired motion pieces (trailing hold coverage excluded)"
            );
        }
    }

    /// A drain that outlasts a wire round trip is worth a line: paired with
    /// the horizon it waited on, it says whether the machine was still moving
    /// or the pipeline was dragging, without needing the warning to fire.
    fn report_drain_wait_done(&self) {
        const WORTH_REPORTING: std::time::Duration = std::time::Duration::from_secs(1);
        let Some(diag) = self.flush.drain_wait_diag.lock_ok().take() else {
            return;
        };
        let (waited_s, horizon_s) = diag.elapsed(std::time::Instant::now());
        if waited_s < WORTH_REPORTING.as_secs_f64() {
            return;
        }
        tracing::info!(
            subsystem = "motion",
            event = "drain_wait_done",
            waited_s,
            horizon_s,
            "[drain] wait finished after the committed motion it was owed"
        );
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
