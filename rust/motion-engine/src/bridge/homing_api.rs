use super::{
    Arc, DRAIN_TIMEOUT, Duration, HomingRun, Ordering, PyMotionEngine, PyResult, PyRuntimeError,
    Python, TripDeps, dispatch_endstop_trip, drip_cohort_participants, planner_err, pymethods,
};

struct ResolvedHomingTarget {
    all_axis_keys: Vec<crate::types::AxisKey>,
    axis_key: crate::types::AxisKey,
}

struct AbortContext {
    all_axis_keys: Vec<crate::types::AxisKey>,
    cohort: u64,
    axis_key: crate::types::AxisKey,
}

fn next_homing_cohort() -> u64 {
    use std::sync::atomic::AtomicU64;
    static SEQ: AtomicU64 = AtomicU64::new(1);
    SEQ.fetch_add(1, Ordering::Relaxed)
}

#[pymethods]
impl PyMotionEngine {
    #[pyo3(signature = (axis, direction, speed_mm_s, max_travel_mm, endstop_id, endstop_mcu))]
    #[allow(clippy::too_many_arguments)]
    fn home_axis_start(
        &self,
        py: Python<'_>,
        axis: u8,
        direction: f64,
        speed_mm_s: f64,
        max_travel_mm: f64,
        endstop_id: u8,
        endstop_mcu: u32,
    ) -> PyResult<()> {
        let guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
        let planner = guard
            .as_ref()
            .ok_or_else(|| PyRuntimeError::new_err("home_axis: planner not initialized"))?;

        let ResolvedHomingTarget {
            all_axis_keys,
            axis_key,
        } = self.resolve_homing_target(axis)?;
        let cohort = next_homing_cohort();
        let start_pos = *self.commanded_pos.lock().unwrap_or_else(|p| p.into_inner());

        self.latched_drive_fault
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .remove(&axis_key.mcu_id);

        self.quiesce_pump_and_drain(py)?;

        let window_start_host = {
            let router = self.router.lock().unwrap_or_else(|p| p.into_inner());
            router.host_now_secs()
        };

        *self
            .active_drip_cohort
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = Some(cohort);

        self.homing_pump_tx()?
            .send(crate::pump::PumpMsg::DripArm(crate::pump::DripArm {
                cohort,
                participants: all_axis_keys.clone(),
                timeout: Duration::from_secs(5),
            }))
            .map_err(|_| PyRuntimeError::new_err("home_axis: pump channel closed"))?;

        let (result_tx, result_rx) =
            crossbeam_channel::bounded::<Result<([f64; 3], [f64; 3], u64), String>>(1);

        *self.homing_run.lock().unwrap_or_else(|p| p.into_inner()) = Some(HomingRun {
            cohort,
            endstop_id,
            endstop_mcu,
            axis_key,
            all_axis_keys: all_axis_keys.clone(),
            window_start_host,
            notify: result_tx,
        });

        let (planner_done_tx, planner_done_rx) =
            crossbeam_channel::bounded::<Result<(), String>>(1);
        planner
            .home_drip(crate::worker::HomeDripParams {
                home_pos: [start_pos[0], start_pos[1], start_pos[2], 0.0],
                start: start_pos,
                axis,
                direction,
                speed_mm_s,
                max_travel_mm,
                cohort,
                participants: all_axis_keys,
                notify: planner_done_tx,
            })
            .map_err(|e| {
                self.finish_homing();
                planner_err(e)
            })?;
        self.await_homing_dispatch(py, &planner_done_rx)?;

        *self.homing_result.lock().unwrap_or_else(|p| p.into_inner()) = Some(result_rx);

        self.consume_buffered_early_trip(endstop_mcu, endstop_id);
        Ok(())
    }
    fn motion_drained(&self) -> bool {
        self.drain.drained()
    }
    fn home_axis_poll(&self) -> PyResult<Option<([f64; 3], [f64; 3], u64)>> {
        let rx = {
            let guard = self.homing_result.lock().unwrap_or_else(|p| p.into_inner());
            match guard.as_ref() {
                Some(rx) => rx.clone(),
                None => {
                    return Err(PyRuntimeError::new_err(
                        "home_axis_poll: no homing in progress",
                    ));
                }
            }
        };
        match rx.try_recv() {
            Err(crossbeam_channel::TryRecvError::Empty) => Ok(None),
            Err(crossbeam_channel::TryRecvError::Disconnected) => {
                self.finish_homing();
                Err(PyRuntimeError::new_err(
                    "home_axis_poll: homing result channel closed",
                ))
            }
            Ok(result) => {
                self.finish_homing();
                let (trip_pos, final_pos, trip_clock) = result.map_err(PyRuntimeError::new_err)?;
                *self.commanded_pos.lock().unwrap_or_else(|p| p.into_inner()) = final_pos;
                self.reanchor_after_trip(final_pos)?;
                Ok(Some((trip_pos, final_pos, trip_clock)))
            }
        }
    }
    fn arm_remote_trigger(&self, mcu_handle: u32, trsync_oid: u32, endstop_id: u8) -> PyResult<()> {
        {
            let armed = self
                .remote_triggers
                .lock()
                .unwrap_or_else(|p| p.into_inner());
            if armed.contains_key(&endstop_id) {
                return Err(PyRuntimeError::new_err(format!(
                    "arm_remote_trigger: endstop_id {endstop_id} is already armed"
                )));
            }
        }
        let host_io = self
            .mcus
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .get(&mcu_handle)
            .and_then(|c| c.host_io.as_ref().map(Arc::clone))
            .ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "arm_remote_trigger: mcu {mcu_handle} has no serial transport"
                ))
            })?;
        let deps = self.trip_deps();
        *self.pending_trip.lock().unwrap_or_else(|p| p.into_inner()) = None;
        let router = Arc::clone(&self.router);
        let fired = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let id = host_io
            .register_frame_interceptor(
                "trsync_state",
                Some(trsync_oid),
                Box::new(move |params| {
                    let decision = crate::remote_trigger::relay_decision(
                        params.try_get_u32("can_trigger"),
                        fired.load(Ordering::SeqCst),
                    );
                    if decision != crate::remote_trigger::RelayAction::Fire {
                        return;
                    }
                    fired.store(true, Ordering::SeqCst);
                    let clock32 = params.try_get_u32("clock").unwrap_or(0);
                    let reference = router
                        .lock()
                        .unwrap_or_else(|p| p.into_inner())
                        .compute_ack_clock(host_rt::passthrough_queue::McuHandle::from_raw(
                            mcu_handle,
                        ))
                        .unwrap_or(0);
                    let clock64 = crate::remote_trigger::relay_trip_clock(clock32, reference);
                    tracing::info!(
                        subsystem = "trip-relay",
                        event = "remote_trigger_fired",
                        mcu = mcu_handle,
                        endstop_id,
                        trsync_oid,
                        clock32,
                        clock64,
                        reason = params.try_get_u32("trigger_reason"),
                        "remote trsync terminal report — dispatching endstop trip"
                    );
                    dispatch_endstop_trip(&deps, mcu_handle, endstop_id, clock64);
                }),
            )
            .map_err(|e| {
                PyRuntimeError::new_err(format!(
                    "arm_remote_trigger: interceptor registration failed: {e:?}"
                ))
            })?;
        self.remote_triggers
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .insert(endstop_id, (mcu_handle, id));
        Ok(())
    }
    fn disarm_remote_trigger(&self, endstop_id: u8) -> PyResult<()> {
        let entry = self
            .remote_triggers
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .remove(&endstop_id);
        let Some((mcu_handle, id)) = entry else {
            return Err(PyRuntimeError::new_err(format!(
                "disarm_remote_trigger: endstop_id {endstop_id} is not armed"
            )));
        };
        let host_io = self
            .mcus
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .get(&mcu_handle)
            .and_then(|c| c.host_io.as_ref().map(Arc::clone));
        match host_io {
            Some(io) => io.unregister_frame_interceptor(id).map_err(|e| {
                PyRuntimeError::new_err(format!("disarm_remote_trigger: unregister failed: {e:?}"))
            }),
            None => Ok(()),
        }
    }
    fn home_abort(&self, py: Python<'_>) {
        let Some(ctx) = self.abort_context() else {
            self.finish_homing();
            return;
        };

        if !self.flush_aborted_cohort(py, ctx.all_axis_keys, ctx.cohort) {
            self.finish_homing();
            return;
        }

        self.finish_homing();

        let cartesian = match self.reconcile_aborted_position(ctx.axis_key) {
            Ok(p) => p,
            Err(()) => return,
        };

        let drain = self.drain.clone();
        let drain_result = py.detach(|| drain.wait_drained(DRAIN_TIMEOUT));
        if let Err(e) = drain_result {
            tracing::error!(
                event = "home_abort_drain_timeout",
                error = %e,
                "home_abort: drain timed out after aborted homing move — \
                 commanded_pos is STALE; a firmware restart is required: {e}"
            );
            return;
        }

        if !self.reopen_stream_at(cartesian) {
            return;
        }

        *self.commanded_pos.lock().unwrap_or_else(|p| p.into_inner()) = cartesian;
        *self.last_g5_pq.lock().unwrap_or_else(|p| p.into_inner()) = None;
    }
    #[pyo3(signature = (source_mcu, clock, host_now))]
    fn motion_state_at_clock(
        &self,
        source_mcu: u32,
        clock: u64,
        host_now: f64,
    ) -> PyResult<std::collections::HashMap<String, (f64, f64, f64)>> {
        const AXIS_NAMES: [&str; 4] = ["x", "y", "z", "e"];
        let configs: Vec<crate::mcu_config::McuAxisConfig> = self
            .mcu_axis_configs
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone();
        if configs.is_empty() {
            return Err(PyRuntimeError::new_err(
                "motion_state_at: no axes configured on the engine",
            ));
        }
        let query_host = {
            let router = self.router.lock().unwrap_or_else(|p| p.into_inner());
            crate::motion_history::clock_to_host(
                &router,
                crate::types::mcu_handle_from_raw(source_mcu),
                clock,
            )
            .map_err(PyRuntimeError::new_err)?
        };
        let resolved: Vec<crate::types::AxisKey> = configs
            .iter()
            .flat_map(|cfg| {
                cfg.axes.iter().map(|&axis| crate::types::AxisKey {
                    mcu_id: cfg.mcu_id,
                    axis: axis as u8,
                })
            })
            .collect();
        let store = self
            .motion_history
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        let mut out = std::collections::HashMap::new();
        for key in resolved {
            let st = match store.state_at_host(key, query_host, Some(host_now)) {
                Ok(st) => st,
                Err(crate::motion_history::HistoryError::NoHistoryForAxis(_)) => continue,
                Err(e) => return Err(PyRuntimeError::new_err(e.to_string())),
            };
            let name = AXIS_NAMES.get(key.axis as usize).ok_or_else(|| {
                PyRuntimeError::new_err(format!("motion_state_at: unnamed axis {}", key.axis))
            })?;
            out.insert(
                (*name).to_string(),
                (st.position, st.velocity, st.acceleration),
            );
        }
        Ok(out)
    }
}

impl PyMotionEngine {
    fn resolve_homing_target(&self, axis: u8) -> PyResult<ResolvedHomingTarget> {
        if axis > 2 {
            return Err(PyRuntimeError::new_err(format!(
                "home_axis: axis {axis} out of range (0=X, 1=Y, 2=Z)"
            )));
        }
        let configs = self
            .mcu_axis_configs
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        let all_axis_keys = drip_cohort_participants(&configs);
        let mcu_id = configs
            .iter()
            .find(|cfg| cfg.axes.iter().any(|&a| a == axis as usize))
            .map(|cfg| cfg.mcu_id)
            .ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "home_axis: axis {axis} not found in mcu_axis_configs \
                     (init_planner not called?)"
                ))
            })?;
        Ok(ResolvedHomingTarget {
            all_axis_keys,
            axis_key: crate::types::AxisKey { mcu_id, axis },
        })
    }

    fn homing_pump_tx(&self) -> PyResult<crossbeam_channel::Sender<crate::pump::PumpMsg>> {
        self.pump_tx
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone()
            .ok_or_else(|| PyRuntimeError::new_err("home_axis: pump not started"))
    }

    fn quiesce_pump_and_drain(&self, py: Python<'_>) -> PyResult<()> {
        let pump_tx = self.homing_pump_tx()?;
        let drain = self.drain.clone();
        py.detach(|| {
            let (ack_tx, ack_rx) = std::sync::mpsc::sync_channel(1);
            pump_tx
                .send(crate::pump::PumpMsg::Barrier(ack_tx))
                .map_err(|_| "home_axis: pump control channel closed".to_string())?;
            ack_rx
                .recv_timeout(Duration::from_secs(5))
                .map_err(|_| "home_axis: pump barrier not acknowledged".to_string())?;
            drain.wait_drained(DRAIN_TIMEOUT)
        })
        .map_err(PyRuntimeError::new_err)
    }

    fn await_homing_dispatch(
        &self,
        py: Python<'_>,
        planner_done_rx: &crossbeam_channel::Receiver<Result<(), String>>,
    ) -> PyResult<()> {
        let dispatch = py.detach(|| {
            planner_done_rx
                .recv_timeout(Duration::from_secs(5))
                .map_err(|_| "home_axis: planner timed out dispatching homing move".to_owned())
                .and_then(|r| r)
        });
        if let Err(e) = dispatch {
            self.finish_homing();
            return Err(PyRuntimeError::new_err(e));
        }
        Ok(())
    }

    fn consume_buffered_early_trip(&self, endstop_mcu: u32, endstop_id: u8) {
        let pending = self
            .pending_trip
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .take();
        if let Some((p_mcu, p_endstop, p_clock)) = pending {
            if p_mcu == endstop_mcu && p_endstop == endstop_id {
                tracing::warn!(
                    subsystem = "trip-relay",
                    event = "early_trip_consumed",
                    mcu = p_mcu,
                    endstop_id = p_endstop,
                    trip_clock = p_clock,
                    "dispatching buffered early trip"
                );
                dispatch_endstop_trip(&self.trip_deps(), p_mcu, p_endstop, p_clock);
            }
        }
    }

    fn abort_context(&self) -> Option<AbortContext> {
        let guard = self.homing_run.lock().unwrap_or_else(|p| p.into_inner());
        guard.as_ref().map(|r| AbortContext {
            all_axis_keys: r.all_axis_keys.clone(),
            cohort: r.cohort,
            axis_key: r.axis_key,
        })
    }

    fn flush_aborted_cohort(
        &self,
        py: Python<'_>,
        all_axis_keys: Vec<crate::types::AxisKey>,
        cohort: u64,
    ) -> bool {
        let Some(tx) = self
            .pump_tx
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone()
        else {
            return true;
        };
        let _ = tx.send(crate::pump::PumpMsg::Flush(all_axis_keys));
        let _ = tx.send(crate::pump::PumpMsg::DripDisarm(cohort));
        let (ack_tx, ack_rx) = std::sync::mpsc::sync_channel(1);
        let _ = tx.send(crate::pump::PumpMsg::Barrier(ack_tx));
        let barrier = py.detach(move || ack_rx.recv_timeout(std::time::Duration::from_secs(1)));
        if barrier.is_err() {
            tracing::error!(
                event = "home_abort_flush_barrier_timeout",
                "home_abort: pump did not acknowledge the flush barrier — \
                 commanded_pos is STALE; a firmware restart is required"
            );
            return false;
        }
        true
    }

    fn reconcile_aborted_position(&self, axis_key: crate::types::AxisKey) -> Result<[f64; 3], ()> {
        let final_cartesian = {
            let configs = self
                .mcu_axis_configs
                .lock()
                .unwrap_or_else(|p| p.into_inner());
            configs
                .iter()
                .find(|c| c.mcu_id == axis_key.mcu_id)
                .ok_or_else(|| format!("no axis config for mcu {}", axis_key.mcu_id))
                .and_then(|cfg| crate::homing::final_cartesian_position(cfg, &self.motion_history))
        };
        final_cartesian.map_err(|e| {
            tracing::error!(
                event = "home_abort_position_reconcile_failed",
                axis_key = ?axis_key,
                "home_abort: cannot reconcile position after aborted homing move \
                 (trajectory store empty or missing for axis {:?}): {e} — \
                 commanded_pos is STALE; a firmware restart is required to \
                 recover a consistent position",
                axis_key
            );
        })
    }

    fn reopen_stream_at(&self, cartesian: [f64; 3]) -> bool {
        let planner_guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
        let Some(planner) = planner_guard.as_ref() else {
            return true;
        };
        let open_result = planner.stream_open(vec![cartesian[0], cartesian[1], cartesian[2], 0.0]);
        if let Err(e) = open_result {
            tracing::error!(
                event = "home_abort_stream_open_failed",
                error = ?e,
                "home_abort: runtime_stream_open failed after drain — \
                 commanded_pos is STALE; a firmware restart is required: {e:?}"
            );
            return false;
        }
        true
    }

    fn finish_homing(&self) {
        *self
            .active_drip_cohort
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = None;
        *self.homing_run.lock().unwrap_or_else(|p| p.into_inner()) = None;
        *self.homing_result.lock().unwrap_or_else(|p| p.into_inner()) = None;
        *self.pending_trip.lock().unwrap_or_else(|p| p.into_inner()) = None;
    }
    pub(super) fn trip_deps(&self) -> TripDeps {
        TripDeps {
            homing_run: Arc::clone(&self.homing_run),
            pending_trip: Arc::clone(&self.pending_trip),
            active_drip_cohort: Arc::clone(&self.active_drip_cohort),
            pump_tx: Arc::clone(&self.pump_tx),
            mcus: Arc::clone(&self.mcus),
            router: Arc::clone(&self.router),
            motion_history: Arc::clone(&self.motion_history),
            mcu_axis_configs: Arc::clone(&self.mcu_axis_configs),
        }
    }
    fn reanchor_after_trip(&self, stop_pos: [f64; 3]) -> PyResult<()> {
        let guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
        match guard.as_ref() {
            Some(planner) => planner
                .reset(vec![stop_pos[0], stop_pos[1], stop_pos[2], 0.0])
                .map_err(planner_err),
            None => Ok(()),
        }
    }
}
