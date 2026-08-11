use super::{
    Arc, DRAIN_TIMEOUT, Duration, HomingRun, Ordering, PyMotionEngine, PyResult, PyRuntimeError,
    Python, RemoteFreeze, TripDeps, TripMember, dispatch_endstop_trip, drip_cohort_participants,
    planner_err, pymethods,
};
use crate::lock_ext::LockExt;

struct ResolvedHomingTarget {
    all_axis_keys: Vec<crate::types::AxisKey>,
    axis_key: crate::types::AxisKey,
}

struct AbortContext {
    all_axis_keys: Vec<crate::types::AxisKey>,
    cohort: u64,
    axis_key: crate::types::AxisKey,
    pending_suppresses: Arc<(std::sync::Mutex<usize>, std::sync::Condvar)>,
}

fn next_homing_cohort() -> u64 {
    use std::sync::atomic::AtomicU64;
    static SEQ: AtomicU64 = AtomicU64::new(1);
    SEQ.fetch_add(1, Ordering::Relaxed)
}

#[pymethods]
impl PyMotionEngine {
    #[pyo3(signature = (axis, direction, speed_mm_s, max_travel_mm, endstops))]
    #[allow(clippy::too_many_arguments)]
    fn home_axis_start(
        &self,
        py: Python<'_>,
        axis: u8,
        direction: f64,
        speed_mm_s: f64,
        max_travel_mm: f64,
        endstops: Vec<(u8, u32, Option<(u32, u8, u8)>)>,
    ) -> PyResult<()> {
        if endstops.is_empty() {
            return Err(PyRuntimeError::new_err(
                "home_axis: endstops list must not be empty",
            ));
        }
        let remaining_trips: Vec<TripMember> = endstops
            .iter()
            .map(|&(endstop_id, endstop_mcu, freeze)| TripMember {
                endstop_mcu,
                endstop_id,
                remote_freeze: freeze.map(|(motor_mcu, motor_idx, stepper_idx)| RemoteFreeze {
                    motor_mcu,
                    motor_idx,
                    stepper_idx,
                }),
            })
            .collect();

        let guard = self.planner.lock_ok();
        let planner = guard
            .as_ref()
            .ok_or_else(|| PyRuntimeError::new_err("home_axis: planner not initialized"))?;

        let ResolvedHomingTarget {
            all_axis_keys,
            axis_key,
        } = self.resolve_homing_target(axis)?;
        let cohort = next_homing_cohort();
        let start_pos = *self.commanded_pos.lock_ok();

        self.latched.drive.lock_ok().remove(&axis_key.mcu_id);

        self.quiesce_pump_and_drain(py)?;

        // The counters are machine space, so the gcode rest point crosses
        // the warp here; the follower lanes take the same origin home_drip
        // restarts the stream odometer's follower coordinate at.
        let machine_start = self.machine_from_gcode(start_pos);
        self.send_serial_position_seeds(machine_start)?;

        let window_start_host = self
            .homing
            .take_arm_window_start(
                &remaining_trips
                    .iter()
                    .map(|t| (t.endstop_mcu, t.endstop_id))
                    .collect::<Vec<_>>(),
            )
            .unwrap_or_else(|| self.router.lock_ok().host_now_secs());

        *self.homing.active_drip_cohort.lock_ok() = Some(cohort);

        self.homing_pump_tx()?
            .send(crate::pump::PumpMsg::DripArm(crate::pump::DripArm {
                cohort,
                participants: all_axis_keys.clone(),
                timeout: Duration::from_secs(5),
            }))
            .map_err(|_| PyRuntimeError::new_err("home_axis: pump channel closed"))?;

        let (result_tx, result_rx) = crossbeam_channel::bounded::<
            Result<(geometry::MachinePos, geometry::MachinePos, u64), String>,
        >(1);

        *self.homing.run.lock_ok() = Some(HomingRun {
            cohort,
            remaining_trips,
            axis_key,
            all_axis_keys: all_axis_keys.clone(),
            window_start_host,
            start_pos: machine_start,
            notify: result_tx,
            pending_suppresses: Arc::new((std::sync::Mutex::new(0), std::sync::Condvar::new())),
        });

        let planner_done_rx = planner
            .home_drip(crate::worker::HomeDripParams {
                home_pos: crate::mcu_config::reanchor_home_pos(start_pos),
                start: start_pos.0,
                axis,
                direction,
                speed_mm_s,
                max_travel_mm,
                cohort,
                participants: all_axis_keys,
            })
            .map_err(|e| {
                self.finish_homing();
                planner_err(e)
            })?;
        self.await_homing_dispatch(py, &planner_done_rx)?;

        *self.homing.result.lock_ok() = Some(result_rx);

        self.consume_buffered_early_trips();
        Ok(())
    }
    fn motion_drained(&self) -> bool {
        self.drain.drained()
    }
    fn note_endstop_arm(&self, endstop_mcu: u32, endstop_id: u8) {
        let host_now = self.router.lock_ok().host_now_secs();
        self.homing.note_arm(endstop_mcu, endstop_id, host_now);
    }
    fn home_axis_poll(&self) -> PyResult<Option<([f64; 3], [f64; 3], u64)>> {
        let rx = {
            let guard = self.homing.result.lock_ok();
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
                let trip_pos = self.gcode_from_machine(trip_pos);
                let final_pos = self.gcode_from_machine(final_pos);
                *self.commanded_pos.lock_ok() = final_pos;
                self.reanchor_after_trip(final_pos)?;
                Ok(Some((trip_pos.0, final_pos.0, trip_clock)))
            }
        }
    }
    fn arm_remote_trigger(&self, mcu_handle: u32, trsync_oid: u32, endstop_id: u8) -> PyResult<()> {
        {
            let armed = self.remote_triggers.lock_ok();
            if armed.contains_key(&endstop_id) {
                return Err(PyRuntimeError::new_err(format!(
                    "arm_remote_trigger: endstop_id {endstop_id} is already armed"
                )));
            }
        }
        let host_io = self
            .mcus
            .lock_ok()
            .get(&mcu_handle)
            .and_then(|c| c.host_io.as_ref().map(Arc::clone))
            .ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "arm_remote_trigger: mcu {mcu_handle} has no serial transport"
                ))
            })?;
        let deps = self.trip_deps();
        {
            let host_now = self.router.lock_ok().host_now_secs();
            self.homing.note_arm(mcu_handle, endstop_id, host_now);
        }
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
                        .lock_ok()
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
            .lock_ok()
            .insert(endstop_id, (mcu_handle, id));
        Ok(())
    }
    fn disarm_remote_trigger(&self, endstop_id: u8) -> PyResult<()> {
        let entry = self.remote_triggers.lock_ok().remove(&endstop_id);
        let Some((mcu_handle, id)) = entry else {
            return Err(PyRuntimeError::new_err(format!(
                "disarm_remote_trigger: endstop_id {endstop_id} is not armed"
            )));
        };
        let host_io = self
            .mcus
            .lock_ok()
            .get(&mcu_handle)
            .and_then(|c| c.host_io.as_ref().map(Arc::clone));
        match host_io {
            Some(io) => io.unregister_frame_interceptor(id).map_err(|e| {
                PyRuntimeError::new_err(format!("disarm_remote_trigger: unregister failed: {e:?}"))
            }),
            None => Ok(()),
        }
    }
    /// Aborts the active homing run. Returns the reconciled gcode position
    /// the toolhead actually stopped at, or `None` when the position could
    /// not be reconciled — commanded_pos is then stale and the caller must
    /// treat the position as unknown.
    fn home_abort(&self, py: Python<'_>) -> Option<[f64; 3]> {
        let Some(ctx) = self.abort_context() else {
            self.finish_homing();
            return None;
        };

        if !self.flush_aborted_cohort(py, ctx.all_axis_keys, ctx.cohort) {
            self.finish_homing();
            return None;
        }
        if super::endstop::wait_for_pending_suppresses(&ctx.pending_suppresses).is_err() {
            self.finish_homing();
            return None;
        }

        self.finish_homing();

        if !self.clear_all_suppress_masks() {
            return None;
        }

        let machine = self.reconcile_aborted_position(ctx.axis_key).ok()?;

        let drain = self.drain.clone();
        let drain_result = py.detach(|| drain.wait_drained(DRAIN_TIMEOUT));
        if let Err(e) = drain_result {
            tracing::error!(
                event = "home_abort_drain_timeout",
                error = %e,
                "home_abort: drain timed out after aborted homing move — \
                 commanded_pos is STALE; a firmware restart is required: {e}"
            );
            return None;
        }

        let gcode = self.gcode_from_machine(machine);
        if !self.reopen_stream_at(gcode) {
            return None;
        }

        *self.commanded_pos.lock_ok() = gcode;
        *self.last_g5_pq.lock_ok() = None;
        Some([gcode.x(), gcode.y(), gcode.z()])
    }
    #[pyo3(signature = (source_mcu, clock, host_now))]
    fn motion_state_at_clock(
        &self,
        source_mcu: u32,
        clock: u64,
        host_now: f64,
    ) -> PyResult<std::collections::HashMap<String, (f64, f64, f64)>> {
        let configs: Vec<crate::mcu_config::McuAxisConfig> =
            self.mcu_axis_configs.lock_ok().clone();
        if configs.is_empty() {
            return Err(PyRuntimeError::new_err(
                "motion_state_at: no axes configured on the engine",
            ));
        }
        let query_host = {
            let router = self.router.lock_ok();
            crate::motion_history::clock_to_host(
                &router,
                crate::types::mcu_handle_from_raw(source_mcu),
                clock,
            )
            .map_err(PyRuntimeError::new_err)?
        };
        // The history ring is recorded in motor frame (the lowerer's output —
        // e.g. CoreXY A/B, not cartesian X/Y), same as the live QueryMotorState
        // path in runtime_caps.rs. Invert through the same kinematics tag so
        // this answers cartesian, matching what `assemble_cartesian` does there.
        let kin_tag = configs
            .iter()
            .find(|c| c.axes.contains(&0usize))
            .map(|c| c.kinematics)
            .unwrap_or(runtime::segment::KinematicTag::Cartesian as u8);
        let kin = crate::kinematics::KinematicsModule::from_tag(kin_tag)
            .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;
        let resolved: Vec<crate::types::AxisKey> = configs
            .iter()
            .flat_map(|cfg| {
                cfg.axes.iter().map(|&axis| crate::types::AxisKey {
                    mcu_id: cfg.mcu_id,
                    axis: axis as u8,
                })
            })
            .collect();
        let store = self.motion_history.lock_ok();
        let mut motor_state: [Option<crate::motion_history::AxisState>; 4] = [None; 4];
        for key in resolved {
            let axis = key.axis as usize;
            if axis >= motor_state.len() {
                return Err(PyRuntimeError::new_err(format!(
                    "motion_state_at: unnamed axis {}",
                    key.axis
                )));
            }
            match store.state_at_host(key, query_host, Some(host_now)) {
                Ok(st) => motor_state[axis] = Some(st),
                Err(crate::motion_history::HistoryError::NoHistoryForAxis(_)) => {}
                Err(e @ crate::motion_history::HistoryError::BeforeRetainedWindow { .. }) => {
                    let Some(initial) = store.initial_hold_state(key) else {
                        return Err(PyRuntimeError::new_err(e.to_string()));
                    };
                    motor_state[axis] = Some(initial);
                }
                Err(e) => return Err(PyRuntimeError::new_err(e.to_string())),
            }
        }
        drop(store);
        let machine = crate::motion_history::assemble_cartesian_state(motor_state, &kin);
        self.cartesian_state_to_gcode(machine)
    }
}

impl PyMotionEngine {
    fn resolve_homing_target(&self, axis: u8) -> PyResult<ResolvedHomingTarget> {
        if axis > 2 {
            return Err(PyRuntimeError::new_err(format!(
                "home_axis: axis {axis} out of range (0=X, 1=Y, 2=Z)"
            )));
        }
        let configs = self.mcu_axis_configs.lock_ok();
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
        self.pump
            .tx
            .lock_ok()
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

    fn consume_buffered_early_trips(&self) {
        let pending: Vec<(u32, u8, u64)> =
            std::mem::take(&mut *self.homing.pending_trips.lock_ok());
        if pending.is_empty() {
            return;
        }
        let deps = self.trip_deps();
        for (p_mcu, p_endstop, p_clock) in pending {
            tracing::warn!(
                subsystem = "trip-relay",
                event = "early_trip_consumed",
                mcu = p_mcu,
                endstop_id = p_endstop,
                trip_clock = p_clock,
                "dispatching buffered early trip"
            );
            dispatch_endstop_trip(&deps, p_mcu, p_endstop, p_clock);
        }
    }

    fn abort_context(&self) -> Option<AbortContext> {
        let guard = self.homing.run.lock_ok();
        guard.as_ref().map(|r| AbortContext {
            all_axis_keys: r.all_axis_keys.clone(),
            cohort: r.cohort,
            axis_key: r.axis_key,
            pending_suppresses: Arc::clone(&r.pending_suppresses),
        })
    }

    fn flush_aborted_cohort(
        &self,
        py: Python<'_>,
        all_axis_keys: Vec<crate::types::AxisKey>,
        cohort: u64,
    ) -> bool {
        let Some(tx) = self.pump.tx.lock_ok().clone() else {
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

    fn reconcile_aborted_position(
        &self,
        axis_key: crate::types::AxisKey,
    ) -> Result<geometry::MachinePos, ()> {
        let final_cartesian = {
            let configs = self.mcu_axis_configs.lock_ok();
            crate::homing::final_cartesian_position(&configs, &self.motion_history)
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

    fn reopen_stream_at(&self, gcode: geometry::GcodePos) -> bool {
        let planner_guard = self.planner.lock_ok();
        let Some(planner) = planner_guard.as_ref() else {
            return true;
        };
        let open_result = planner.stream_open(crate::mcu_config::reanchor_stream_pos(gcode));
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

    /// A partial trip may have engaged suppress masks on motor MCUs before
    /// the run aborted; ResumeStream never reaches them on this path, so the
    /// masks must be cleared explicitly or a stepper stays silently frozen.
    fn clear_all_suppress_masks(&self) -> bool {
        use mcu_protocol::codec::{Decode as _, Encode as _};
        let transports: Vec<(u32, Arc<dyn host_rt::mcu_call::McuCall>)> = {
            let mcus = self.mcus.lock_ok();
            mcus.iter()
                .filter(|(_, conn)| {
                    conn.endpoint_conn.is_some()
                        || conn
                            .runtime_caps
                            .as_ref()
                            .is_some_and(|caps| caps.total_piece_memory > 0)
                })
                .filter_map(|(&id, conn)| {
                    if let Some(io) = conn.host_io.as_ref() {
                        Some((id, Arc::clone(io) as Arc<dyn host_rt::mcu_call::McuCall>))
                    } else {
                        conn.endpoint_conn
                            .as_ref()
                            .map(|ec| (id, Arc::clone(ec) as Arc<dyn host_rt::mcu_call::McuCall>))
                    }
                })
                .collect()
        };
        let mut ok = true;
        for (mcu_id, transport) in transports {
            let mut body = Vec::with_capacity(3);
            mcu_protocol::messages::StepperSuppress {
                motor: 0xFF,
                stepper: 0xFF,
                engage: 0,
            }
            .encode(&mut body);
            let outcome = transport
                .mcu_call(
                    mcu_protocol::MessageKind::StepperSuppress,
                    body,
                    Duration::from_secs(3),
                )
                .map_err(|e| format!("{e:?}"))
                .and_then(|(_kind, resp_body)| {
                    mcu_protocol::messages::StepperSuppressResponse::decode(&resp_body)
                        .map_err(|e| format!("{e:?}"))
                });
            match outcome {
                Ok(resp) if resp.effective_clock != 0 => {}
                Ok(_) => {
                    tracing::error!(
                        event = "suppress_clear_rejected",
                        mcu = mcu_id,
                        "home_abort: suppress mask clear rejected — a stepper may \
                         remain frozen; a firmware restart is required"
                    );
                    ok = false;
                }
                Err(e) => {
                    tracing::error!(
                        event = "suppress_clear_failed",
                        mcu = mcu_id,
                        error = %e,
                        "home_abort: suppress mask clear failed — a stepper may \
                         remain frozen; a firmware restart is required"
                    );
                    ok = false;
                }
            }
        }
        ok
    }

    fn finish_homing(&self) {
        self.homing.finish();
    }
    /// Motion history answers in machine space (the lowerer's output frame);
    /// this is the bridge crossing for full kinematic states returned to
    /// Python: the Z lane is unwarped through the active mesh with the exact
    /// inverse of the lowerer's chain rule. XY (and follower axes) are
    /// warp-invariant. Identity when no mesh is active.
    fn cartesian_state_to_gcode(
        &self,
        mut state: std::collections::HashMap<String, (f64, f64, f64)>,
    ) -> PyResult<std::collections::HashMap<String, (f64, f64, f64)>> {
        let mesh = self.bed_mesh.lock_ok();
        let Some(t) = mesh.as_deref() else {
            return Ok(state);
        };
        let Some(&z_machine) = state.get("z") else {
            return Ok(state);
        };
        let (Some(&x), Some(&y)) = (state.get("x"), state.get("y")) else {
            return Err(PyRuntimeError::new_err(
                "motion_state_at: a bed mesh is active but the queried instant has a Z \
                 state without X/Y — cannot unwarp machine Z to gcode Z without the XY \
                 the mesh was sampled at",
            ));
        };
        let gcode_z = t.unwarp_z_state([x.0, y.0], [x.1, y.1], [x.2, y.2], z_machine);
        state.insert("z".to_string(), gcode_z);
        Ok(state)
    }
    pub(super) fn trip_deps(&self) -> TripDeps {
        TripDeps {
            homing: Arc::clone(&self.homing),
            pump_tx: Arc::clone(&self.pump.tx),
            mcus: Arc::clone(&self.mcus),
            router: Arc::clone(&self.router),
            motion_history: Arc::clone(&self.motion_history),
            mcu_axis_configs: Arc::clone(&self.mcu_axis_configs),
            stepcompress_endpoints: Arc::clone(&self.stepcompress_endpoints),
        }
    }
    fn reanchor_after_trip(&self, stop_pos: geometry::GcodePos) -> PyResult<()> {
        let guard = self.planner.lock_ok();
        match guard.as_ref() {
            Some(planner) => planner
                .reset(vec![stop_pos.x(), stop_pos.y(), stop_pos.z(), 0.0])
                .map_err(planner_err),
            None => Ok(()),
        }
    }
}
