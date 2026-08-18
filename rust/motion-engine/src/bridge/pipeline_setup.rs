use super::{
    Arc, Duration, ETHERCAT_CLOCK_FREQ_HZ, HashMap, HashSet, HomingRun, HomingState, Instant,
    McuAxisConfig, McuConnection, McuHostIo, McuSerialConn, McuTopologyInput, Mutex,
    PyMotionEngine, PyResult, PyRuntimeError, PyValueError, STREAM_INTEGRATION_TOL,
    STREAM_MAX_BUFFER_MOVES, abort_after_tracing_appender_drains, arm_endpoint_death_watchdog,
    build_mcu_configs, collect_motor_positions_inner, config, dispatch_endstop_trip,
    mcu_handle_from_raw, query_ethercat_runtime_caps, report_ethercat_endpoint_death,
};
use crate::lock_ext::LockExt;
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
        corner: cfg.corner,
        integration_tol: STREAM_INTEGRATION_TOL,
        max_extrude_only_velocity_mm_s: cfg.max_extrude_only_velocity.unwrap_or(f64::INFINITY),
        max_extrude_only_accel_mm_s2: cfg.max_extrude_only_accel.unwrap_or(f64::INFINITY),
        fit_tol_mm: cfg.fit_tolerance_mm,
        fit_tol_accel_mm_s2: cfg.fit_tolerance_accel_mm_s2,
        max_buffer_moves: STREAM_MAX_BUFFER_MOVES,
        limits: geometry::VelocityLimits::try_new(
            cart.max_velocity,
            cart.max_accel,
            cart.corner_deviation,
            cart.max_jerk,
        )
        .map_err(PyRuntimeError::new_err)?,
    })
}

impl PyMotionEngine {
    pub(super) fn resolve_mcu_topology(
        &self,
        mcus: &[McuTopologyInput],
    ) -> PyResult<(HashMap<u32, Arc<McuSerialConn>>, Vec<McuAxisConfig>)> {
        let ec_conns: HashMap<u32, Arc<McuSerialConn>> = {
            let ethercat_handles: Vec<(u32, Arc<McuSerialConn>, String)> = {
                let mcus_lock = self.mcus.lock_ok();
                mcus.iter()
                    .filter_map(|topology| {
                        let c = mcus_lock.get(&topology.mcu_id)?;
                        let socket = c.ethercat_socket.as_ref()?;
                        let conn = c.endpoint_conn.as_ref()?.clone();
                        Some((topology.mcu_id, conn, socket.clone()))
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
                {
                    let mut mcus_lock = self.mcus.lock_ok();
                    if let Some(c) = mcus_lock.get_mut(&mcu_id) {
                        c.runtime_caps = Some(caps);
                    }
                }
                out.insert(mcu_id, conn);
            }
            out
        };

        let ethercat_mcu_ids: HashSet<u32> = ec_conns.keys().copied().collect();
        let mcu_configs = build_mcu_configs(mcus, &ethercat_mcu_ids)
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        *self.mcu_axis_configs.lock_ok() = mcu_configs.clone();
        *self.axis_transports.lock_ok() = Arc::new(
            crate::axis_transport::AxisTransports::from_configs(&mcu_configs),
        );

        Ok((ec_conns, mcu_configs))
    }

    pub(super) fn build_transport_maps(
        &self,
        mcu_configs: &[McuAxisConfig],
    ) -> PyResult<(HashSet<u32>, HashMap<u32, Arc<McuHostIo>>)> {
        let ethercat_mcu_ids: HashSet<u32> = mcu_configs
            .iter()
            .filter(|c| c.ethercat)
            .map(|c| c.mcu_id)
            .collect();

        let host_ios: HashMap<u32, Arc<McuHostIo>> = {
            let mcus = self.mcus.lock_ok();
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

        Ok((ethercat_mcu_ids, host_ios))
    }

    pub(super) fn seed_ethercat_clock_estimates(&self, ethercat_mcu_ids: &HashSet<u32>) {
        let mut router = self.router.lock_ok();
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

    /// Build one endpoint per transport per mcu and index them by mcu for the
    /// pump. A board with both lane kinds gets both endpoints, and so does a
    /// single dual-transport lane: a phase motor homed on StallGuard streams
    /// through the sample executor while printing and through the classic step
    /// queue while the trip is armed. Which one owns a lane at any instant is
    /// [`AxisTransports`], not membership. Each endpoint also declares the
    /// depth its own buffer gives the lanes it owns — the pacing signal the
    /// pump's `room()` uses — so a dual lane carries one depth per transport.
    fn build_pump_resources(
        &self,
        mcu_configs: &[McuAxisConfig],
        host_ios: &HashMap<u32, Arc<McuHostIo>>,
        ec_conns: &HashMap<u32, Arc<McuSerialConn>>,
        pump_control: &crossbeam_channel::Sender<crate::pump::PumpMsg>,
    ) -> PyResult<crate::worker::PumpResources> {
        let router_for_clock = Arc::clone(&self.router);
        let clock_of: crate::pump::ClockSource = Arc::new(move |mcu_id: u32| {
            let r = router_for_clock.lock_ok();
            r.ack_clock_and_freq(mcu_handle_from_raw(mcu_id))
        });

        let transports = Arc::clone(&self.axis_transports.lock_ok());
        let mut stepcompress = HashMap::new();
        let mut samples = HashMap::new();
        let mut ethercat = HashMap::new();
        let mut ring_depth_table: HashMap<crate::types::AxisKey, [u32; 2]> = HashMap::new();
        let mut paced_step = Vec::new();
        let mut paced_sample = Vec::new();

        for cfg in mcu_configs.iter().filter(|cfg| cfg.ethercat) {
            let conn = ec_conns.get(&cfg.mcu_id).ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "init_planner: ethercat mcu {} has no endpoint connection",
                    cfg.mcu_id
                ))
            })?;
            let (ring, depth) = {
                let mcus = self.mcus.lock_ok();
                let mcu = mcus.get(&cfg.mcu_id).ok_or_else(|| {
                    PyRuntimeError::new_err(format!(
                        "init_planner: unknown mcu_handle {}",
                        cfg.mcu_id
                    ))
                })?;
                let grid = mcu.sample_grid.ok_or_else(|| {
                    PyRuntimeError::new_err(format!(
                        "init_planner: ethercat mcu {} retained no setpoint grid from its \
                         claim — the pump cannot pace a ring whose depth it does not know",
                        cfg.mcu_id
                    ))
                })?;
                let ring = mcu.ring_filler.clone().ok_or_else(|| {
                    PyRuntimeError::new_err(format!(
                        "init_planner: ethercat mcu {} built no setpoint filler at claim time",
                        cfg.mcu_id
                    ))
                })?;
                (ring, grid.ring_depth_cycles)
            };
            for &axis in &cfg.axes {
                ring_depth_table.insert(
                    crate::types::AxisKey {
                        mcu_id: cfg.mcu_id,
                        axis: axis as u8,
                    },
                    [depth; 2],
                );
            }
            ethercat.insert(
                cfg.mcu_id,
                crate::pump::EtherCatRing {
                    conn: Arc::downgrade(conn),
                    ring,
                },
            );
        }

        for cfg in mcu_configs.iter().filter(|cfg| !cfg.ethercat) {
            let io = host_ios.get(&cfg.mcu_id).ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "init_planner: serial mcu {} has no host transport",
                    cfg.mcu_id
                ))
            })?;
            let measured_freq = clock_of(cfg.mcu_id).map(|(_, f)| f).ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "init_planner: mcu {} has no clock estimate; a lane's spans must use the \
                     same slope the host projects its starts with",
                    cfg.mcu_id
                ))
            })?;

            if cfg.has_pulse_lanes() {
                let endpoint = crate::pump::build_endpoint(
                    cfg,
                    Arc::downgrade(io),
                    pump_control.clone(),
                    measured_freq,
                    Arc::clone(&clock_of),
                )
                .map_err(PyRuntimeError::new_err)?;
                self.register_barrier_ack_interceptor(
                    io,
                    cfg.mcu_id,
                    "stepcompress_barrier_ack",
                    "barrier_seq",
                    pump_control,
                )?;
                let depth = endpoint.ring_depth();
                for axis in cfg.pulse_capable_axes() {
                    ring_depth_table
                        .entry(crate::types::AxisKey {
                            mcu_id: cfg.mcu_id,
                            axis: axis as u8,
                        })
                        .or_default()[crate::axis_transport::TRANSPORT_PULSE as usize] = depth;
                }
                let shared = Arc::new(Mutex::new(endpoint));
                self.stepcompress_endpoints
                    .lock_ok()
                    .insert(cfg.mcu_id, Arc::clone(&shared));
                paced_step.push(Arc::clone(&shared));
                stepcompress.insert(cfg.mcu_id, shared);
            }

            let phase_lanes = cfg.phase_capable_axes();
            if !phase_lanes.is_empty() {
                let endpoint = crate::pump::build_sample_endpoint(
                    cfg,
                    Arc::downgrade(io),
                    pump_control.clone(),
                    measured_freq,
                    Arc::clone(&clock_of),
                )
                .map_err(PyRuntimeError::new_err)?;
                self.register_barrier_ack_interceptor(
                    io,
                    cfg.mcu_id,
                    runtime::sample_wire::SAMPLE_BARRIER_ACK_NAME,
                    "seq",
                    pump_control,
                )?;
                for axis in phase_lanes {
                    ring_depth_table
                        .entry(crate::types::AxisKey {
                            mcu_id: cfg.mcu_id,
                            axis: axis as u8,
                        })
                        .or_default()[crate::axis_transport::TRANSPORT_PHASE as usize] =
                        crate::pump::SAMPLE_LANE_PIECE_WINDOW;
                }
                let mcu_retired = endpoint.mcu_retired();
                io.attach_heartbeat_callback(Arc::new(move |counts: &[u32], clocks: &[u64]| {
                    mcu_retired.record(counts, clocks)
                }));
                let shared = Arc::new(Mutex::new(endpoint));
                self.sample_endpoints
                    .lock_ok()
                    .insert(cfg.mcu_id, Arc::clone(&shared));
                paced_sample.push(Arc::clone(&shared));
                samples.insert(cfg.mcu_id, shared);
            }
        }

        *self.pump.pacer.lock_ok() = if paced_step.is_empty() {
            None
        } else {
            Some(crate::pump::StepcompressPacer::spawn(paced_step))
        };
        *self.pump.sample_pacer.lock_ok() = if paced_sample.is_empty() {
            None
        } else {
            Some(crate::pump::SamplePacer::spawn(paced_sample))
        };

        let ring_depth_table_for_pump = ring_depth_table;
        let router_for_pump = Arc::clone(&self.router);
        let drain_for_pump = self.drain.clone();
        let endpoint_death_for_pump = Arc::clone(&self.latched.endpoint_death);
        let transports_for_depth = Arc::clone(&transports);
        Ok(crate::worker::PumpResources {
            sink: crate::pump::WireSink {
                stepcompress,
                samples,
                ethercat,
                transports,
                timeout: Duration::from_secs(5),
            },
            callbacks: crate::pump::PumpCallbacks {
                ring_depth_of: Box::new(move |k| {
                    let slots = ring_depth_table_for_pump
                        .get(&k)
                        .unwrap_or_else(|| panic!("pump axis {k:?} has no validated ring depth"));
                    let mode = transports_for_depth.mode(k);
                    let depth = slots[mode as usize];
                    assert!(
                        depth != 0,
                        "pump axis {k:?} has no ring depth for the {} transport it is routed \
                         through",
                        crate::axis_transport::transport_name(mode)
                    );
                    depth
                }),
                mcu_clock_of: Box::new(move |mcu_id: u32| {
                    let r = router_for_pump.lock_ok();
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
            backlog: Arc::clone(&self.pump.backlog),
        })
    }

    /// Route an endpoint's barrier receipt back to the pump. Both serial
    /// transports issue barriers and both name the stepper oid they belong to,
    /// so the pump's single ack message carries whichever frame arrived and
    /// `WireSink` resolves the owning endpoint by that oid.
    fn register_barrier_ack_interceptor(
        &self,
        io: &Arc<McuHostIo>,
        mcu_id: u32,
        frame: &'static str,
        seq_field: &'static str,
        pump_control: &crossbeam_channel::Sender<crate::pump::PumpMsg>,
    ) -> PyResult<()> {
        let ack_tx = pump_control.clone();
        io.register_frame_interceptor(
            frame,
            None,
            Box::new(move |params| {
                let oid = params
                    .try_get_u32("oid")
                    .unwrap_or_else(|| panic!("mcu {mcu_id}: {frame} carried no oid parameter"));
                let seq = params.try_get_u32(seq_field).unwrap_or_else(|| {
                    panic!("mcu {mcu_id}: {frame} carried no {seq_field} parameter")
                });
                let _ = ack_tx.send(crate::pump::PumpMsg::StepcompressBarrierAck {
                    mcu_id,
                    oid: oid as u8,
                    seq,
                });
            }),
        )
        .map(|_| ())
        .map_err(|e| {
            PyRuntimeError::new_err(format!("mcu {mcu_id}: cannot intercept {frame}: {e:?}"))
        })
    }

    pub(super) fn spawn_pipeline(
        &self,
        cfg: &config::PlannerConfig,
        mcu_configs: &[McuAxisConfig],
        host_ios: &HashMap<u32, Arc<McuHostIo>>,
        ec_conns: &HashMap<u32, Arc<McuSerialConn>>,
    ) -> PyResult<crossbeam_channel::Sender<crate::pump::PumpMsg>> {
        let counter = Arc::clone(&self.dispatched_segments);
        let router_arc = Arc::clone(&self.router);

        let (pump_tx, pump_rx) = crossbeam_channel::unbounded::<crate::pump::PumpMsg>();
        let pump_resources =
            self.build_pump_resources(mcu_configs, host_ios, ec_conns, &pump_tx)?;

        let anchor_mutex = Arc::clone(&self.dispatch_anchor);
        *anchor_mutex.lock_ok() = crate::anchor::Anchor::new();
        let dispatch_resources = crate::worker::DispatchResources {
            router: Arc::clone(&router_arc),
            anchor: anchor_mutex,
            mcu_configs: mcu_configs.to_vec(),
            counter: Arc::clone(&counter),
            active_drip_cohort: Arc::clone(&self.homing.active_drip_cohort),
            motion_history: Arc::clone(&self.motion_history),
        };

        let stream_cfg = build_stream_config(cfg)?;
        let axis_chains = cfg
            .post_processors
            .compile(&cfg.axis_registry)
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        let home = vec![0.0; cfg.axis_registry.n_axes()];

        let mut planner_guard = self.planner.lock_ok();
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
            (pump_tx, pump_rx),
        );
        let pump_control = pipeline.pump_control.clone();
        *self.pump.tx.lock_ok() = Some(pipeline.pump_control);
        *self.pump.thread.lock_ok() = Some(pipeline.pump_thread);
        *planner_guard = Some(pipeline.worker);
        drop(planner_guard);

        Ok(pump_control)
    }

    pub(super) fn spawn_live_position_poll_thread(&self) {
        let configs = Arc::clone(&self.mcu_axis_configs);
        let mcus = Arc::clone(&self.mcus);
        let cache = Arc::clone(&self.position_poll.cache);
        let stop = Arc::clone(&self.position_poll.stop);
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
                            let mut c = cache.lock_ok();
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
        *self.position_poll.thread.lock_ok() = Some(handle);
    }

    pub(super) fn wire_mcu_supervision(
        &self,
        mcu_configs: &[McuAxisConfig],
        ethercat_mcu_ids: &HashSet<u32>,
        ec_conns: &HashMap<u32, Arc<McuSerialConn>>,
        pump_control: crossbeam_channel::Sender<crate::pump::PumpMsg>,
    ) {
        for cfg_mcu in mcu_configs {
            self.wire_mcu_supervision_for(cfg_mcu, ethercat_mcu_ids, ec_conns, &pump_control);
        }
    }

    fn wire_mcu_supervision_for(
        &self,
        cfg_mcu: &McuAxisConfig,
        ethercat_mcu_ids: &HashSet<u32>,
        ec_conns: &HashMap<u32, Arc<McuSerialConn>>,
        pump_control: &crossbeam_channel::Sender<crate::pump::PumpMsg>,
    ) {
        let mcu_id = cfg_mcu.mcu_id;

        if ethercat_mcu_ids.contains(&mcu_id) {
            let conn = ec_conns
                .get(&mcu_id)
                .expect("ec_conns built from ethercat_mcu_ids")
                .clone();

            let slot_axes = self
                .mcus
                .lock_ok()
                .get(&mcu_id)
                .map(|c| c.ethercat_slot_axes.clone())
                .unwrap_or_default();
            let supervisor = EthercatHeartbeatSupervisor {
                mcu_id,
                mcu_label: self.mcu_label(mcu_id),
                homing: Arc::clone(&self.homing),
                latched_drive_fault: Arc::clone(&self.latched.drive),
                pump_tx: pump_control.clone(),
                slot_axes,
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
                Arc::clone(&self.latched.endpoint_death),
            );
        } else {
            tracing::info!(
                subsystem = "motion",
                event = "serial_heartbeat_suppressed",
                mcu_id,
                "serial mcu retires host-computed frames — the step shim and the sample \
                 endpoint are the sole sources of pump credit"
            );
        }
    }

    fn mcu_label(&self, mcu_id: u32) -> String {
        self.mcus
            .lock_ok()
            .get(&mcu_id)
            .map(|c| c.label.clone())
            .unwrap_or_else(|| format!("mcu-{mcu_id}"))
    }
}

/// Re-index an EtherCAT endpoint's per-SLOT retired counters into the pump's
/// per-AXIS view. With AWD several slots retire the same axis's pieces; the
/// minimum is the axis's true progress — capacity accounting must wait for the
/// laggard ring.
pub(super) fn retired_by_axis(slot_axes: &[usize], retired_slots: &[u32]) -> Vec<u32> {
    let max_axis = slot_axes.iter().copied().max().unwrap_or(0);
    let mut out = vec![0u32; max_axis + 1];
    let mut seen = vec![false; max_axis + 1];
    for (slot, &axis) in slot_axes.iter().enumerate() {
        let Some(&retired) = retired_slots.get(slot) else {
            continue;
        };
        if !seen[axis] || retired < out[axis] {
            out[axis] = retired;
            seen[axis] = true;
        }
    }
    out
}

fn forward_retired_heartbeat(
    pump_tx: &crossbeam_channel::Sender<crate::pump::PumpMsg>,
    mcu_id: u32,
    retired_counts: Vec<u32>,
) {
    let _ = pump_tx.send(crate::pump::PumpMsg::Heartbeat(crate::pump::HeartbeatMsg {
        mcu_id,
        #[allow(clippy::cast_possible_truncation)]
        axes: (0..retired_counts.len() as u8).collect(),
        consumed_counts: None,
        retired_counts,
        retired_by: crate::pump::RetiredBy::EtherCat,
    }));
}

pub(super) struct EthercatHeartbeatSupervisor {
    pub(super) mcu_id: u32,
    pub(super) mcu_label: String,
    pub(super) homing: Arc<HomingState>,
    pub(super) latched_drive_fault: Arc<Mutex<HashMap<u32, u16>>>,
    pub(super) pump_tx: crossbeam_channel::Sender<crate::pump::PumpMsg>,
    pub(super) slot_axes: Vec<usize>,
}

impl EthercatHeartbeatSupervisor {
    pub(super) fn on_heartbeat(&self, hb: &mcu_protocol::messages::StatusHeartbeat) {
        if hb.fault_code != 0 {
            self.on_drive_fault(hb.fault_code);
            return;
        }
        forward_retired_heartbeat(
            &self.pump_tx,
            self.mcu_id,
            retired_by_axis(&self.slot_axes, &hb.retired_counts),
        );
    }

    fn on_drive_fault(&self, fault_code: u16) {
        match self.take_homing_run_owning_fault() {
            Some(run) => self.fail_homing_run(run, fault_code),
            None => self.latch_fault_for_klippy(fault_code),
        }
    }

    fn take_homing_run_owning_fault(&self) -> Option<HomingRun> {
        let mut guard = self.homing.run.lock_ok();
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
            .lock_ok()
            .insert(self.mcu_id, fault_code);
        crate::pump::emit_fault_snapshot("homing_drive_fault", i32::from(fault_code));
        *self.homing.active_drip_cohort.lock_ok() = None;
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
            .lock_ok()
            .insert(self.mcu_id, fault_code);
        if prev != Some(fault_code) {
            crate::pump::emit_fault_snapshot("drive_fault_heartbeat", i32::from(fault_code));
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
                    let mut mcus = mcus.lock_ok();
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
