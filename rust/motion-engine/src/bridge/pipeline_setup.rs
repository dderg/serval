use super::{
    Arc, Duration, ETHERCAT_CLOCK_FREQ_HZ, HashMap, HashSet, HomingRun, Instant, McuAxisConfig,
    McuCaps, McuConnection, McuHostIo, McuSerialConn, Mutex, PyMotionEngine, PyResult,
    PyRuntimeError, PyValueError, STREAM_INTEGRATION_TOL, STREAM_MAX_BUFFER_MOVES,
    abort_after_tracing_appender_drains, arm_endpoint_death_watchdog, axis_ring_depth,
    build_mcu_configs, collect_motor_positions_inner, config, dispatch_endstop_trip,
    mcu_handle_from_raw, query_ethercat_runtime_caps, report_ethercat_endpoint_death,
    resolve_motion_caps,
};

fn escalate_endpoint_death(latch: &Arc<Mutex<HashMap<u32, String>>>, mcu_id: u32, reason: &str) {
    if report_ethercat_endpoint_death(latch, mcu_id, reason) {
        arm_endpoint_death_watchdog(Arc::clone(latch), mcu_id);
    }
}

fn log_abandoned_pieces(key: crate::types::AxisKey, dropped: u32) {
    tracing::debug!(
        subsystem = "motion",
        event = "pump_abandon_unpushed",
        mcu = key.mcu_id,
        axis = key.axis,
        dropped,
        "pump flush dropped pieces that never reached the wire"
    );
}

fn abort_on_drip_stall(msg: String) {
    tracing::error!(
        subsystem = "motion",
        event = "drip_cohort_stalled",
        msg,
        "EXIT_ON_FAULT — drip cohort stalled; \
         aborting klippy so systemd restarts it"
    );
    abort_after_tracing_appender_drains();
}

fn build_stream_config(cfg: &config::PlannerConfig) -> PyResult<motion_pipeline::StreamConfig> {
    let cart = cfg.cartesian;
    Ok(motion_pipeline::StreamConfig {
        chain: cfg.chain,
        integration_tol: STREAM_INTEGRATION_TOL,
        max_extrude_only_velocity_mm_s: cfg.max_extrude_only_velocity.unwrap_or(f64::INFINITY),
        max_extrude_only_accel_mm_s2: cfg.max_extrude_only_accel.unwrap_or(f64::INFINITY),
        fit_tol_mm: cfg.fit_tolerance_mm,
        fit_tol_accel_mm_s2: cfg.fit_tolerance_accel_mm_s2,
        max_buffer_moves: STREAM_MAX_BUFFER_MOVES,
        limits: geometry::VelocityLimits::try_new(
            cart.max_velocity,
            cart.max_accel,
            cart.square_corner_velocity,
            cart.max_jerk,
        )
        .map_err(PyRuntimeError::new_err)?,
    })
}

impl PyMotionEngine {
    pub(super) fn resolve_mcu_topology(
        &self,
        mcus: &[(u32, Vec<u8>, u8)],
    ) -> PyResult<(HashMap<u32, Arc<McuSerialConn>>, Vec<McuAxisConfig>)> {
        let ec_conns: HashMap<u32, Arc<McuSerialConn>> = {
            let ethercat_handles: Vec<(u32, Arc<McuSerialConn>, String)> = {
                let mcus_lock = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
                mcus.iter()
                    .filter_map(|(handle, _, _)| {
                        let c = mcus_lock.get(handle)?;
                        let socket = c.ethercat_socket.as_ref()?;
                        let conn = c.endpoint_conn.as_ref()?.clone();
                        Some((*handle, conn, socket.clone()))
                    })
                    .collect()
            };

            let mut out = HashMap::new();
            for (mcu_id, conn, socket) in ethercat_handles {
                let caps = query_ethercat_runtime_caps(&conn, std::time::Duration::from_secs(5))
                    .map_err(|e| {
                        PyRuntimeError::new_err(format!(
                            "init_planner: QueryRuntimeCaps failed for ethercat mcu \
                                 {mcu_id} ({socket}): {e} — endpoint must respond with \
                                 RuntimeCapsResponse; is ethercat-rt running?"
                        ))
                    })?;
                tracing::debug!(
                    subsystem = "engine",
                    event = "init_planner_ethercat_caps",
                    mcu_id,
                    total_piece_memory = caps.total_piece_memory,
                    "[caps-trace] init_planner: ethercat mcu caps"
                );
                {
                    let mut mcus_lock = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
                    if let Some(c) = mcus_lock.get_mut(&mcu_id) {
                        c.runtime_caps = Some(caps);
                    }
                }
                out.insert(mcu_id, conn);
            }
            out
        };

        let caps_by_handle: std::collections::HashMap<u32, McuCaps> = {
            let mcus_lock = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            mcus.iter()
                .map(|(handle, _, _)| {
                    let conn = mcus_lock.get(handle).ok_or_else(|| {
                        PyRuntimeError::new_err(format!(
                            "init_planner: unknown mcu_handle {handle}"
                        ))
                    })?;
                    let caps = resolve_motion_caps(conn.runtime_caps, &conn.label, *handle)
                        .map_err(PyRuntimeError::new_err)?;
                    Ok((*handle, caps))
                })
                .collect::<PyResult<_>>()?
        };
        let mcu_configs = build_mcu_configs(mcus, &caps_by_handle)
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        *self
            .mcu_axis_configs
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = mcu_configs.clone();

        Ok((ec_conns, mcu_configs))
    }

    pub(super) fn build_transport_maps(
        &self,
        mcu_configs: &[McuAxisConfig],
    ) -> PyResult<(
        HashSet<u32>,
        HashMap<u32, Arc<McuHostIo>>,
        HashMap<crate::types::AxisKey, u32>,
    )> {
        let ethercat_mcu_ids: HashSet<u32> = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            mcu_configs
                .iter()
                .filter(|c| {
                    mcus.get(&c.mcu_id)
                        .map_or(false, |conn| conn.ethercat_socket.is_some())
                })
                .map(|c| c.mcu_id)
                .collect()
        };

        let host_ios: HashMap<u32, Arc<McuHostIo>> = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            let mut out = HashMap::new();
            for cfg_mcu in mcu_configs {
                if ethercat_mcu_ids.contains(&cfg_mcu.mcu_id) {
                    continue;
                }
                let conn = mcus.get(&cfg_mcu.mcu_id).ok_or_else(|| {
                    PyRuntimeError::new_err(format!(
                        "init_planner: unknown mcu_handle {}",
                        cfg_mcu.mcu_id
                    ))
                })?;
                let io = conn.host_io.as_ref().ok_or_else(|| {
                    PyRuntimeError::new_err(format!(
                        "init_planner: attach_serial has not been called for MCU {}",
                        cfg_mcu.mcu_id
                    ))
                })?;
                out.insert(cfg_mcu.mcu_id, Arc::clone(io));
            }
            out
        };

        let ring_depth_table: HashMap<crate::types::AxisKey, u32> = {
            let mut t = HashMap::new();
            for cfg_mcu in mcu_configs {
                let total = cfg_mcu.caps.total_pieces() as u32;
                let n = cfg_mcu.axes.len() as u32;
                let depth = axis_ring_depth(total, n);
                for &axis in &cfg_mcu.axes {
                    t.insert(
                        crate::types::AxisKey {
                            mcu_id: cfg_mcu.mcu_id,
                            axis: axis as u8,
                        },
                        depth,
                    );
                }
            }
            t
        };

        Ok((ethercat_mcu_ids, host_ios, ring_depth_table))
    }

    pub(super) fn seed_ethercat_clock_estimates(&self, ethercat_mcu_ids: &HashSet<u32>) {
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        let now_ns = crate::timing::monotonic_ns();
        for &mcu_id in ethercat_mcu_ids {
            let mcu_h = mcu_handle_from_raw(mcu_id);
            let _ = router.set_clock_est_from_sample(
                mcu_h,
                f64::from(ETHERCAT_CLOCK_FREQ_HZ),
                Instant::now(),
                now_ns,
            );
        }
    }

    fn build_pump_resources(
        &self,
        host_ios: &HashMap<u32, Arc<McuHostIo>>,
        ec_conns: &HashMap<u32, Arc<McuSerialConn>>,
        ring_depth_table: HashMap<crate::types::AxisKey, u32>,
    ) -> crate::worker::PumpResources {
        let wire_transports: HashMap<u32, crate::pump::McuTransport> = {
            let mut t = HashMap::new();
            for (&id, io) in host_ios {
                t.insert(id, crate::pump::McuTransport::Serial(Arc::downgrade(io)));
            }
            for (&id, conn) in ec_conns {
                t.insert(
                    id,
                    crate::pump::McuTransport::EtherCat(Arc::downgrade(conn)),
                );
            }
            t
        };

        let ring_depth_table_for_pump = ring_depth_table;
        let router_for_pump = Arc::clone(&self.router);
        let drain_for_pump = self.drain.clone();
        let router_for_freq = Arc::clone(&self.router);
        let endpoint_death_for_pump = Arc::clone(&self.latched_endpoint_death);
        crate::worker::PumpResources {
            sink: crate::pump::WireSink {
                transports: wire_transports,
                timeout: Duration::from_secs(5),
                freq_of: Arc::new(move |mcu_id: u32| {
                    let r = router_for_freq.lock().unwrap_or_else(|p| p.into_inner());
                    r.ack_clock_and_freq(mcu_handle_from_raw(mcu_id))
                        .map(|(_, f)| f)
                }),
            },
            callbacks: crate::pump::PumpCallbacks {
                ring_depth_of: Box::new(move |k| {
                    ring_depth_table_for_pump
                        .get(&k)
                        .copied()
                        .unwrap_or_else(|| {
                            tracing::error!(
                                subsystem = "engine",
                                event = "pump_missing_ring_depth",
                                axis_key = ?k,
                                "pump: no ring_depth for axis — absent from init_planner config; using sentinel depth 1 (expect PieceStartInPast fault)"
                            );
                            1
                        })
                }),
                mcu_clock_of: Box::new(move |mcu_id: u32| {
                    let r = router_for_pump.lock().unwrap_or_else(|p| p.into_inner());
                    r.ack_clock_and_freq(mcu_handle_from_raw(mcu_id))
                }),
                on_fatal_transport: Box::new(move |key: crate::types::AxisKey| {
                    escalate_endpoint_death(
                        &endpoint_death_for_pump,
                        key.mcu_id,
                        "pump transport went fatal (broken pipe / endpoint gone) \
                         — see the send_frame_fatal log for the exact transport error",
                    );
                }),
                on_abandon: Box::new(log_abandoned_pieces),
                on_drip_stall: Box::new(abort_on_drip_stall),
            },
            history: crate::pump::HistoryRecorder {
                store: Arc::clone(&self.motion_history),
                nominal_freqs: Arc::clone(&self.nominal_clock_freqs),
            },
            drain: drain_for_pump,
            backlog: Arc::clone(&self.pump_backlog),
        }
    }

    pub(super) fn spawn_pipeline(
        &self,
        cfg: &config::PlannerConfig,
        mcu_configs: &[McuAxisConfig],
        host_ios: &HashMap<u32, Arc<McuHostIo>>,
        ec_conns: &HashMap<u32, Arc<McuSerialConn>>,
        ring_depth_table: HashMap<crate::types::AxisKey, u32>,
    ) -> PyResult<crossbeam_channel::Sender<crate::pump::PumpMsg>> {
        let counter = Arc::clone(&self.dispatched_segments);
        let router_arc = Arc::clone(&self.router);

        let pump_resources = self.build_pump_resources(host_ios, ec_conns, ring_depth_table);

        let anchor_mutex = Arc::clone(&self.dispatch_anchor);
        *anchor_mutex.lock().unwrap_or_else(|p| p.into_inner()) = crate::anchor::Anchor::new();
        let dispatch_resources = crate::worker::DispatchResources {
            router: Arc::clone(&router_arc),
            anchor: anchor_mutex,
            mcu_configs: mcu_configs.to_vec(),
            counter: Arc::clone(&counter),
            active_drip_cohort: Arc::clone(&self.active_drip_cohort),
            motion_history: Arc::clone(&self.motion_history),
        };

        let stream_cfg = build_stream_config(cfg)?;
        let axis_chains = cfg
            .post_processors
            .compile(&cfg.axis_registry)
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        let home = vec![0.0; cfg.axis_registry.n_axes()];

        let mut planner_guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
        if planner_guard.is_some() {
            return Err(PyRuntimeError::new_err(
                "planner already initialized (raced)",
            ));
        }
        let pipeline = crate::worker::setup_pipeline(
            stream_cfg,
            axis_chains,
            home,
            dispatch_resources,
            pump_resources,
        );
        let pump_control = pipeline.pump_control.clone();
        *self.pump_tx.lock().unwrap_or_else(|p| p.into_inner()) = Some(pipeline.pump_control);
        *self.pump_thread.lock().unwrap_or_else(|p| p.into_inner()) = Some(pipeline.pump_thread);
        *planner_guard = Some(pipeline.worker);
        drop(planner_guard);

        Ok(pump_control)
    }

    pub(super) fn spawn_live_position_poll_thread(&self) {
        let configs = Arc::clone(&self.mcu_axis_configs);
        let mcus = Arc::clone(&self.mcus);
        let cache = Arc::clone(&self.live_position_cache);
        let stop = Arc::clone(&self.position_poll_stop);
        let handle = std::thread::Builder::new()
            .name("live-position-poll".into())
            .spawn(move || {
                use std::sync::atomic::Ordering;
                let period = std::time::Duration::from_millis(200);
                let timeout = std::time::Duration::from_millis(250);
                const WARN_INTERVAL: std::time::Duration = std::time::Duration::from_secs(30);
                let mut consecutive_failures: u64 = 0;
                let mut suppressed_since_warn: u64 = 0;
                let mut last_warn: Option<std::time::Instant> = None;
                while !stop.load(Ordering::Relaxed) {
                    std::thread::sleep(period);
                    if stop.load(Ordering::Relaxed) {
                        break;
                    }
                    match collect_motor_positions_inner(&configs, &mcus, timeout) {
                        Ok(map) => {
                            let mut c = cache.lock().unwrap_or_else(|p| p.into_inner());
                            *c = (map, std::time::Instant::now());
                            if consecutive_failures > 0 {
                                tracing::info!(
                                    event = "live_position_poll_recovered",
                                    after_failures = consecutive_failures,
                                    "live-position poll recovered"
                                );
                            }
                            consecutive_failures = 0;
                            suppressed_since_warn = 0;
                            last_warn = None;
                        }
                        Err(e) => {
                            if !e.contains("no axes configured") {
                                consecutive_failures += 1;
                                let now = std::time::Instant::now();
                                let should_warn = match last_warn {
                                    None => true,
                                    Some(t) => now.duration_since(t) >= WARN_INTERVAL,
                                };
                                if should_warn {
                                    if consecutive_failures == 1 {
                                        tracing::warn!(
                                            event = "live_position_poll_failed",
                                            error = %e,
                                            "live-position poll failed; serving stale cache"
                                        );
                                    } else {
                                        tracing::warn!(
                                            event = "live_position_poll_failed",
                                            error = %e,
                                            consecutive_failures,
                                            suppressed = suppressed_since_warn,
                                            "live-position poll still failing ({consecutive_failures} consecutive, \
                                             {suppressed_since_warn} suppressed in last 30s): {e}"
                                        );
                                    }
                                    last_warn = Some(now);
                                    suppressed_since_warn = 0;
                                } else {
                                    suppressed_since_warn += 1;
                                }
                            }
                        }
                    }
                }
            })
            .expect("spawn live-position-poll thread");
        *self
            .position_poll_thread
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = Some(handle);
    }

    pub(super) fn wire_mcu_supervision(
        &self,
        mcu_configs: &[McuAxisConfig],
        ethercat_mcu_ids: &HashSet<u32>,
        ec_conns: &HashMap<u32, Arc<McuSerialConn>>,
        host_ios: &HashMap<u32, Arc<McuHostIo>>,
        pump_control: crossbeam_channel::Sender<crate::pump::PumpMsg>,
    ) {
        for cfg_mcu in mcu_configs {
            self.wire_mcu_supervision_for(
                cfg_mcu,
                ethercat_mcu_ids,
                ec_conns,
                host_ios,
                &pump_control,
            );
        }
    }

    fn wire_mcu_supervision_for(
        &self,
        cfg_mcu: &McuAxisConfig,
        ethercat_mcu_ids: &HashSet<u32>,
        ec_conns: &HashMap<u32, Arc<McuSerialConn>>,
        host_ios: &HashMap<u32, Arc<McuHostIo>>,
        pump_control: &crossbeam_channel::Sender<crate::pump::PumpMsg>,
    ) {
        let mcu_id = cfg_mcu.mcu_id;

        if ethercat_mcu_ids.contains(&mcu_id) {
            let conn = ec_conns
                .get(&mcu_id)
                .expect("ec_conns built from ethercat_mcu_ids")
                .clone();

            let supervisor = EthercatHeartbeatSupervisor {
                mcu_id,
                mcu_label: self.mcu_label(mcu_id),
                homing_run: Arc::clone(&self.homing_run),
                active_drip_cohort: Arc::clone(&self.active_drip_cohort),
                latched_drive_fault: Arc::clone(&self.latched_drive_fault),
                pump_tx: pump_control.clone(),
            };
            conn.attach_heartbeat_callback(Arc::new(
                move |hb: &mcu_protocol::messages::StatusHeartbeat| supervisor.on_heartbeat(hb),
            ));

            let trip_deps = self.trip_deps();
            conn.attach_endstop_trip_callback(Arc::new(move |endstop_id: u8, trip_clock: u64| {
                dispatch_endstop_trip(&trip_deps, mcu_id, endstop_id, trip_clock);
            }));

            spawn_endpoint_liveness_poll(
                mcu_id,
                &conn,
                Arc::clone(&self.mcus),
                Arc::clone(&self.latched_endpoint_death),
            );
        } else {
            let io = host_ios
                .get(&mcu_id)
                .expect("host_io map built from mcu_configs")
                .clone();
            let pump_tx = pump_control.clone();
            io.attach_heartbeat_callback(Arc::new(move |retired: &[u32]| {
                forward_retired_heartbeat(&pump_tx, mcu_id, retired.to_vec());
            }));
        }
    }

    fn mcu_label(&self, mcu_id: u32) -> String {
        self.mcus
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .get(&mcu_id)
            .map(|c| c.label.clone())
            .unwrap_or_else(|| format!("mcu-{mcu_id}"))
    }
}

fn forward_retired_heartbeat(
    pump_tx: &crossbeam_channel::Sender<crate::pump::PumpMsg>,
    mcu_id: u32,
    retired_counts: Vec<u32>,
) {
    let _ = pump_tx.send(crate::pump::PumpMsg::Heartbeat(crate::pump::HeartbeatMsg {
        mcu_id,
        retired_counts,
    }));
}

struct EthercatHeartbeatSupervisor {
    mcu_id: u32,
    mcu_label: String,
    homing_run: Arc<Mutex<Option<HomingRun>>>,
    active_drip_cohort: Arc<Mutex<Option<u64>>>,
    latched_drive_fault: Arc<Mutex<HashMap<u32, u16>>>,
    pump_tx: crossbeam_channel::Sender<crate::pump::PumpMsg>,
}

impl EthercatHeartbeatSupervisor {
    fn on_heartbeat(&self, hb: &mcu_protocol::messages::StatusHeartbeat) {
        if hb.fault_code != 0 {
            self.on_drive_fault(hb.fault_code);
            return;
        }
        forward_retired_heartbeat(&self.pump_tx, self.mcu_id, hb.retired_counts.clone());
    }

    fn on_drive_fault(&self, fault_code: u16) {
        match self.take_homing_run_owning_fault() {
            Some(run) => self.fail_homing_run(run, fault_code),
            None => self.latch_fault_for_klippy(fault_code),
        }
    }

    fn take_homing_run_owning_fault(&self) -> Option<HomingRun> {
        let mut guard = self.homing_run.lock().unwrap_or_else(|p| p.into_inner());
        match guard.as_ref().map(|r| r.axis_key.mcu_id) {
            Some(axis_mcu)
                if crate::homing::route_drive_fault(self.mcu_id, Some(axis_mcu))
                    == crate::homing::DriveFaultRoute::HomingError =>
            {
                guard.take()
            }
            _ => None,
        }
    }

    fn fail_homing_run(&self, run: HomingRun, fault_code: u16) {
        self.latched_drive_fault
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .insert(self.mcu_id, fault_code);
        *self
            .active_drip_cohort
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = None;
        let _ = self
            .pump_tx
            .send(crate::pump::PumpMsg::Flush(run.all_axis_keys.clone()));
        let _ = self
            .pump_tx
            .send(crate::pump::PumpMsg::DripDisarm(run.cohort));
        let _ = run.notify.send(Err(format!(
            "drive fault 0x{fault_code:04x} during homing — \
             following-error/torque limit exceeded (endstop failure?)"
        )));
    }

    fn latch_fault_for_klippy(&self, fault_code: u16) {
        let prev = self
            .latched_drive_fault
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .insert(self.mcu_id, fault_code);
        if prev != Some(fault_code) {
            tracing::error!(
                event = "ethercat_drive_fault_latched",
                mcu_id = self.mcu_id,
                mcu_label = %self.mcu_label,
                fault_code,
                "ethercat drive fault — latched for klippy to report"
            );
        }
    }
}

fn endpoint_fault_reason(peer_eof: bool, conn: &mut McuConnection) -> Option<String> {
    if peer_eof {
        return Some("conn EOF".to_string());
    }
    let child = conn.endpoint_process.as_mut()?;
    match child.try_wait() {
        Ok(Some(status)) => Some(format!("child exited: {status}")),
        Ok(None) => None,
        Err(e) => Some(format!("try_wait error: {e}")),
    }
}

fn spawn_endpoint_liveness_poll(
    mcu_id: u32,
    conn: &Arc<McuSerialConn>,
    mcus: Arc<Mutex<HashMap<u32, McuConnection>>>,
    endpoint_death_latch: Arc<Mutex<HashMap<u32, String>>>,
) {
    let conn_for_poll = Arc::downgrade(conn);
    let _ = std::thread::Builder::new()
        .name(format!("ec-heartbeat-poll-{mcu_id}"))
        .spawn(move || {
            loop {
                let Some(conn) = conn_for_poll.upgrade() else {
                    return;
                };
                let peer_eof = conn.peer_closed();
                drop(conn);

                let fault_reason = {
                    let mut mcus = mcus.lock().unwrap_or_else(|p| p.into_inner());
                    let Some(c) = mcus.get_mut(&mcu_id) else {
                        return;
                    };
                    endpoint_fault_reason(peer_eof, c)
                };

                if let Some(reason) = fault_reason {
                    escalate_endpoint_death(&endpoint_death_latch, mcu_id, &reason);
                    return;
                }

                std::thread::sleep(Duration::from_millis(1));
            }
        })
        .expect("spawn ec-heartbeat-poll thread");
}
