use super::{
    Arc, DRAIN_TIMEOUT, Duration, ETHERCAT_CLOCK_FREQ_HZ, FieldValue, HashMap, HashSet, Instant,
    McuAxisConfig, McuCaps, McuHostIo, McuSerialConn, Ordering, PyMotionEngine, PyResult,
    PyRuntimeError, PyValueError, Python, SPATIAL_AXES, STREAM_INTEGRATION_TOL,
    STREAM_MAX_BUFFER_MOVES, abort_after_tracing_appender_drains, arm_endpoint_death_watchdog,
    axis_ring_depth, build_mcu_configs, classify, collect_motor_positions_inner, config,
    dispatch_endstop_trip, mcu_handle_from_raw, planner_err, pymethods,
    query_ethercat_runtime_caps, report_ethercat_endpoint_death, require_positive,
    resolve_motion_caps,
};

#[allow(clippy::too_many_arguments)]
fn build_planner_config(
    axes: Vec<(String, Vec<String>, Vec<String>, Vec<String>)>,
    limits: Vec<(String, Vec<String>, Option<f64>, Option<f64>, Option<f64>)>,
    post_processors: Vec<(String, String, Vec<(String, f64)>)>,
    kinematics_axes: &[String],
    cartesian_limits: (f64, f64, f64, f64, f64, f64),
    arc_fit: Option<u32>,
    max_extrude_only_velocity: Option<f64>,
    max_extrude_only_accel: Option<f64>,
) -> PyResult<config::PlannerConfig> {
    let axis_registry = config::AxisRegistry::try_new(
        axes.into_iter()
            .map(
                |(name, follows, motors, post_processors)| config::AxisDecl {
                    name,
                    follows,
                    motors,
                    post_processors,
                },
            )
            .collect(),
    )
    .map_err(|e| PyValueError::new_err(e.to_string()))?;

    axis_registry
        .validate_motor_mapping(kinematics_axes)
        .map_err(|e| PyValueError::new_err(e.to_string()))?;

    let pp_decls: Vec<config::PostProcessorDecl> = post_processors
        .into_iter()
        .map(|(name, ty, params)| config::PostProcessorDecl { name, ty, params })
        .collect();
    let post_processor_set = config::PostProcessorSet::try_new(&axis_registry, &pp_decls)
        .map_err(|e| PyValueError::new_err(e.to_string()))?;

    let limit_sections: Vec<config::LimitSection> = limits
        .into_iter()
        .map(|(name, axes, max_velocity, max_accel, max_jerk)| {
            let axes = axes
                .iter()
                .map(|a| axis_registry.axis_index(a))
                .collect::<Result<Vec<usize>, _>>()
                .map_err(|e| PyValueError::new_err(e.to_string()))?;
            Ok(config::LimitSection {
                name,
                axes,
                max_velocity,
                max_accel,
                max_jerk,
            })
        })
        .collect::<PyResult<_>>()?;

    let (max_velocity, max_accel, max_jerk, max_z_velocity, max_z_accel, square_corner_velocity) =
        cartesian_limits;
    let cartesian = config::CartesianLimits {
        max_velocity,
        max_accel,
        max_jerk,
        max_z_velocity,
        max_z_accel,
        square_corner_velocity,
    };
    cartesian.validate().map_err(PyValueError::new_err)?;

    for (label, value) in [
        ("max_extrude_only_velocity", max_extrude_only_velocity),
        ("max_extrude_only_accel", max_extrude_only_accel),
    ] {
        if let Some(v) = value {
            if !(v.is_finite() && v > 0.0) {
                return Err(PyValueError::new_err(format!(
                    "[extruder] {label} must be finite and positive, got {v}"
                )));
            }
        }
    }

    let mut cfg = config::PlannerConfig::default();
    cfg.axis_registry = axis_registry;
    cfg.limit_sections = limit_sections;
    cfg.cartesian = cartesian;
    cfg.post_processors = post_processor_set;
    cfg.max_extrude_only_velocity = max_extrude_only_velocity;
    cfg.max_extrude_only_accel = max_extrude_only_accel;
    cfg.chain = match arc_fit {
        Some(min_run_facets) => {
            if min_run_facets < 3 {
                return Err(PyValueError::new_err(
                    "[arc_fit] min_run_facets must be at least 3",
                ));
            }
            geometry::ChainFitConfig::with_arc_fit(min_run_facets)
        }
        None => geometry::ChainFitConfig::default(),
    };

    Ok(cfg)
}

#[pymethods]
impl PyMotionEngine {
    #[pyo3(signature = (mcu_id, axis_idx, motor_mask, delta_mm, speed, accel))]
    fn submit_nudge(
        &self,
        _py: Python<'_>,
        mcu_id: u32,
        axis_idx: u8,
        motor_mask: u8,
        delta_mm: f64,
        speed: f64,
        accel: f64,
    ) -> PyResult<f64> {
        if runtime::piece_ring::stepper_sel_from_mask(motor_mask).is_err() {
            return Err(PyRuntimeError::new_err(format!(
                "submit_nudge: multi-bit motor_mask {motor_mask:#010b} not supported"
            )));
        }
        let (tx, rx) = crossbeam_channel::bounded(1);
        {
            let guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
            let planner = guard.as_ref().ok_or_else(|| {
                PyRuntimeError::new_err("planner not initialized — call init_planner first")
            })?;
            planner
                .submit_nudge(crate::worker::NudgeParams {
                    mcu_id,
                    axis: axis_idx,
                    motor_mask,
                    delta_mm,
                    speed,
                    accel,
                    notify: tx,
                })
                .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;
        }
        rx.recv()
            .map_err(|_| PyRuntimeError::new_err("nudge notify dropped"))?
            .map_err(PyRuntimeError::new_err)?;
        let (accel_t, cruise_t, _v) = crate::nudge::calc_move_time(delta_mm, speed, accel);
        Ok(accel_t + cruise_t + accel_t)
    }
    #[pyo3(signature = (
        axes,
        limits,
        post_processors,
        mcus,
        kinematics_axes,
        cartesian_limits,
        arc_fit = None,
        max_extrude_only_velocity = None,
        max_extrude_only_accel = None,
    ))]
    #[allow(clippy::too_many_arguments, clippy::type_complexity)]
    fn init_planner(
        &self,
        axes: Vec<(String, Vec<String>, Vec<String>, Vec<String>)>,
        limits: Vec<(String, Vec<String>, Option<f64>, Option<f64>, Option<f64>)>,
        post_processors: Vec<(String, String, Vec<(String, f64)>)>,
        mcus: Vec<(u32, Vec<u8>, u8)>,
        kinematics_axes: Vec<String>,
        cartesian_limits: (f64, f64, f64, f64, f64, f64),
        arc_fit: Option<u32>,
        max_extrude_only_velocity: Option<f64>,
        max_extrude_only_accel: Option<f64>,
    ) -> PyResult<()> {
        if self
            .planner
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .is_some()
        {
            return Err(PyRuntimeError::new_err("planner already initialized"));
        }

        let cfg = build_planner_config(
            axes,
            limits,
            post_processors,
            &kinematics_axes,
            cartesian_limits,
            arc_fit,
            max_extrude_only_velocity,
            max_extrude_only_accel,
        )?;
        *self
            .planner_config
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = cfg.clone();

        let (ec_conns, mcu_configs) = self.resolve_mcu_topology(&mcus)?;

        let (ethercat_mcu_ids, host_ios, ring_depth_table) =
            self.build_transport_maps(&mcu_configs)?;
        self.seed_ethercat_clock_estimates(&ethercat_mcu_ids);

        let pump_control =
            self.spawn_pipeline(&cfg, &mcu_configs, &host_ios, &ec_conns, ring_depth_table)?;

        self.spawn_live_position_poll_thread();

        self.wire_mcu_supervision(
            &mcu_configs,
            &ethercat_mcu_ids,
            &ec_conns,
            &host_ios,
            pump_control,
        );

        Ok(())
    }
    #[pyo3(signature = (dx, dy, dz, de, feedrate))]
    fn submit_move(
        &self,
        py: Python<'_>,
        dx: f64,
        dy: f64,
        dz: f64,
        de: f64,
        feedrate: f64,
    ) -> PyResult<()> {
        tracing::info!(
            subsystem = "motion",
            event = "submit_move_enter",
            dx,
            dy,
            dz,
            de,
            feedrate,
            "engine.submit_move enter"
        );
        py.detach(|| -> PyResult<()> {
            let followers = self.e_followers(de)?;
            let (extruder_axis, e_delta) = match followers.as_slice() {
                [] => (0usize, 0.0),
                [(axis, delta)] => (*axis, *delta),
                _ => {
                    return Err(PyRuntimeError::new_err(
                        "submit_move: multiple follower axes not yet supported by the new pipeline",
                    ));
                }
            };
            let pos = *self.commanded_pos.lock().unwrap_or_else(|p| p.into_inner());
            let (max_v, max_a, scv, jerk) = {
                let cfg = self
                    .planner_config
                    .lock()
                    .unwrap_or_else(|p| p.into_inner());
                let (mut v, mut a) = cfg.cartesian.for_move(dx, dy, dz);
                if let Some(rv) = cfg.runtime_caps.velocity {
                    v = v.min(rv);
                }
                if let Some(ra) = cfg.runtime_caps.accel {
                    a = a.min(ra);
                }
                // No runtime jerk cap exists yet (RuntimeCaps has no `jerk` field);
                // this is the static [printer] max_jerk. A future
                // `cfg.runtime_caps.jerk` would `.min()` in here exactly like
                // velocity/accel above.
                let j = cfg.cartesian.max_jerk;
                (v, a, cfg.square_corner_velocity(), j)
            };
            let limits = geometry::VelocityLimits::try_new(max_v, max_a, scv, jerk)
                .map_err(PyRuntimeError::new_err)?;
            let line_no = self.move_seq.fetch_add(1, Ordering::Relaxed) as u32;
            let m = classify::build_move(
                pos,
                dx,
                dy,
                dz,
                extruder_axis,
                e_delta,
                limits,
                feedrate,
                line_no,
            )
            .map_err(|e| PyRuntimeError::new_err(format!("{e:?}")))?;

            {
                let guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
                let planner = guard.as_ref().ok_or_else(|| {
                    PyRuntimeError::new_err("planner not initialized — call init_planner first")
                })?;
                planner.submit_move(m).map_err(planner_err)?;
                tracing::info!(
                    subsystem = "motion",
                    event = "intake_submit",
                    line_no,
                    channel_pending = planner.pending_channel_moves(),
                    uncommitted_secs = planner.uncommitted_intake_secs(),
                    "[intake] move pushed to channel; channel_pending grows when the planner thread can't pull (backpressure blind spot)"
                );
            }

            let mut pos = self.commanded_pos.lock().unwrap_or_else(|p| p.into_inner());
            pos[0] += dx;
            pos[1] += dy;
            pos[2] += dz;
            *self.last_g5_pq.lock().unwrap_or_else(|p| p.into_inner()) = None;
            Ok(())
        })
    }
    #[pyo3(signature = (i, j, p, q, dx, dy, dz, de, feedrate))]
    fn submit_bezier(
        &self,
        py: Python<'_>,
        i: Option<f64>,
        j: Option<f64>,
        p: f64,
        q: f64,
        dx: f64,
        dy: f64,
        dz: f64,
        de: f64,
        feedrate: f64,
    ) -> PyResult<()> {
        tracing::debug!(
            subsystem = "motion",
            event = "submit_bezier_enter",
            i = ?i,
            j = ?j,
            p,
            q,
            dx,
            dy,
            dz,
            de,
            feedrate,
            "engine.submit_bezier enter"
        );
        py.detach(|| -> PyResult<()> {
            Err(PyRuntimeError::new_err(
                "submit_bezier (G5 cubic) is not yet supported by the new geometry pipeline \
                 — V1 streams G0/G1 line moves (and reconstructs arcs from facets); curve \
                 faceting is a follow-up. Slice without G5.",
            ))
        })
    }
    #[pyo3(signature = (i, j, dx, dy, dz, de, feedrate))]
    fn submit_quadratic(
        &self,
        py: Python<'_>,
        i: f64,
        j: f64,
        dx: f64,
        dy: f64,
        dz: f64,
        de: f64,
        feedrate: f64,
    ) -> PyResult<()> {
        tracing::debug!(
            subsystem = "motion",
            event = "submit_quadratic_enter",
            i,
            j,
            dx,
            dy,
            dz,
            de,
            feedrate,
            "engine.submit_quadratic enter"
        );
        py.detach(|| -> PyResult<()> {
            Err(PyRuntimeError::new_err(
                "submit_quadratic (G2/G3 arc as quadratic) is not yet supported by the new \
                 geometry pipeline — V1 streams G0/G1 line moves; curve faceting is a \
                 follow-up. Decompose arcs into line segments upstream.",
            ))
        })
    }
    fn submit_dwell(&self, duration_s: f64) -> PyResult<()> {
        let guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
        let planner = guard.as_ref().ok_or_else(|| {
            PyRuntimeError::new_err("planner not initialized — call init_planner first")
        })?;
        planner.dwell(duration_s).map_err(planner_err)?;
        *self.last_g5_pq.lock().unwrap_or_else(|p| p.into_inner()) = None;
        Ok(())
    }
    #[pyo3(signature = (x, y, z, host_now))]
    fn set_position(&self, py: Python<'_>, x: f64, y: f64, z: f64, host_now: f64) -> PyResult<()> {
        {
            let mut pos = self.commanded_pos.lock().unwrap_or_else(|p| p.into_inner());
            *pos = [x, y, z];
        }
        *self.last_g5_pq.lock().unwrap_or_else(|p| p.into_inner()) = None;
        let planner_guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
        if let Some(planner) = planner_guard.as_ref() {
            py.detach(|| planner.flush()).map_err(planner_err)?;
            {
                let drain = self.drain.clone();
                py.detach(|| drain.wait_drained(DRAIN_TIMEOUT))
                    .map_err(PyRuntimeError::new_err)?;
            }

            planner
                .stream_open(vec![x, y, z, 0.0])
                .map_err(planner_err)?;

            self.drain.reset();

            let sends = {
                let configs = self
                    .mcu_axis_configs
                    .lock()
                    .unwrap_or_else(|p| p.into_inner());
                let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
                let ethercat_mcu_ids: HashSet<u32> = configs
                    .iter()
                    .filter(|c| {
                        mcus.get(&c.mcu_id)
                            .map_or(false, |conn| conn.ethercat_socket.is_some())
                    })
                    .map(|c| c.mcu_id)
                    .collect();
                crate::mcu_config::build_serial_seed_sends(&configs, &ethercat_mcu_ids, x, y, z)
            };
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            for s in sends {
                let conn = mcus.get(&s.mcu_id).unwrap_or_else(|| {
                    panic!(
                        "set_position seed: planner up but mcu_id {} absent \
                         (broken invariant)",
                        s.mcu_id
                    )
                });
                let io = conn.host_io.as_ref().unwrap_or_else(|| {
                    panic!(
                        "set_position seed: serial mcu_id {} has no host_io \
                         (broken invariant — attach_serial not called?)",
                        s.mcu_id
                    )
                });
                io.send_typed(
                    "runtime_seed_position",
                    &[
                        ("x_q16", FieldValue::I32(s.x_q16)),
                        ("y_q16", FieldValue::I32(s.y_q16)),
                        ("z_q16", FieldValue::I32(s.z_q16)),
                    ],
                )
                .map_err(|e| {
                    PyRuntimeError::new_err(format!(
                        "set_position seed send to mcu_id {} failed: {e:?}",
                        s.mcu_id
                    ))
                })?;
            }
        }

        {
            let configs: Vec<crate::mcu_config::McuAxisConfig> = self
                .mcu_axis_configs
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .clone();
            let positions = [x, y, z];
            let rebases: Vec<(crate::types::AxisKey, f64)> = configs
                .iter()
                .flat_map(|cfg| {
                    cfg.axes
                        .iter()
                        .filter(|&&a| a < SPATIAL_AXES)
                        .map(move |&axis| {
                            (
                                crate::types::AxisKey {
                                    mcu_id: cfg.mcu_id,
                                    axis: axis as u8,
                                },
                                positions[axis],
                            )
                        })
                        .collect::<Vec<_>>()
                })
                .collect();
            let follower_keys: Vec<crate::types::AxisKey> = configs
                .iter()
                .flat_map(|cfg| {
                    cfg.axes
                        .iter()
                        .filter(|&&a| a >= 3)
                        .map(move |&axis| crate::types::AxisKey {
                            mcu_id: cfg.mcu_id,
                            axis: axis as u8,
                        })
                        .collect::<Vec<_>>()
                })
                .collect();
            {
                let mut store = self
                    .motion_history
                    .lock()
                    .unwrap_or_else(|p| p.into_inner());
                for (key, pos) in rebases {
                    store.rebase_axis(key, host_now, pos);
                }
                for key in follower_keys {
                    let held_position = store.final_position(key).unwrap_or(0.0);
                    store.rebase_axis(key, host_now, held_position);
                }
            }
        }

        Ok(())
    }
    fn effective_limits(&self) -> (f64, f64, f64) {
        self.planner_config
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .effective_limits()
    }
    #[pyo3(signature = (velocity))]
    fn set_velocity_cap(&self, velocity: Option<f64>) -> PyResult<()> {
        require_positive(velocity, "velocity")?;
        self.planner_config
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .runtime_caps
            .velocity = velocity;
        Ok(())
    }
    #[pyo3(signature = (accel))]
    fn set_accel_cap(&self, accel: Option<f64>) -> PyResult<()> {
        require_positive(accel, "accel")?;
        self.planner_config
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .runtime_caps
            .accel = accel;
        Ok(())
    }
    #[pyo3(signature = (square_corner_velocity))]
    fn set_square_corner_velocity(&self, square_corner_velocity: Option<f64>) -> PyResult<()> {
        if let Some(scv) = square_corner_velocity {
            if !(scv.is_finite() && scv >= 0.0) {
                return Err(PyValueError::new_err(
                    "square_corner_velocity must be finite and non-negative",
                ));
            }
        }
        self.planner_config
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .runtime_square_corner_velocity = square_corner_velocity;
        Ok(())
    }
    fn update_post_processor(&self, name: &str, key: &str, value: f64) -> PyResult<()> {
        let axis_chains = {
            let mut cfg = self
                .planner_config
                .lock()
                .unwrap_or_else(|p| p.into_inner());
            cfg.post_processors
                .set_param(name, key, value)
                .map_err(|e| PyValueError::new_err(e.to_string()))?;
            cfg.post_processors
                .compile(&cfg.axis_registry)
                .map_err(|e| PyValueError::new_err(e.to_string()))?
        };
        if let Some(handle) = self
            .planner
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .as_ref()
        {
            handle
                .update_axis_chains(axis_chains)
                .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;
        }
        Ok(())
    }
}

impl PyMotionEngine {
    fn resolve_mcu_topology(
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

    fn build_transport_maps(
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

    fn seed_ethercat_clock_estimates(&self, ethercat_mcu_ids: &HashSet<u32>) {
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

    fn spawn_pipeline(
        &self,
        cfg: &config::PlannerConfig,
        mcu_configs: &[McuAxisConfig],
        host_ios: &HashMap<u32, Arc<McuHostIo>>,
        ec_conns: &HashMap<u32, Arc<McuSerialConn>>,
        ring_depth_table: HashMap<crate::types::AxisKey, u32>,
    ) -> PyResult<crossbeam_channel::Sender<crate::pump::PumpMsg>> {
        let counter = Arc::clone(&self.dispatched_segments);
        let router_arc = Arc::clone(&self.router);

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
        let pump_resources = crate::worker::PumpResources {
            sink: crate::pump::WireSink {
                transports: wire_transports,
                timeout: Duration::from_secs(5),
                freq_of: Arc::new(move |mcu_id: u32| {
                    let r = router_for_freq.lock().unwrap_or_else(|p| p.into_inner());
                    r.ack_clock_and_freq(mcu_handle_from_raw(mcu_id))
                        .map(|(_, f)| f)
                }),
            },
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
                if report_ethercat_endpoint_death(
                    &endpoint_death_for_pump,
                    key.mcu_id,
                    "pump transport went fatal (broken pipe / endpoint gone) \
                     — see the send_frame_fatal log for the exact transport error",
                ) {
                    arm_endpoint_death_watchdog(Arc::clone(&endpoint_death_for_pump), key.mcu_id);
                }
            }),
            on_abandon: Box::new(move |key: crate::types::AxisKey, n: u32| {
                drain_for_pump.unsend(key.mcu_id, key.axis, n);
            }),
            on_drip_stall: Box::new(|msg: String| {
                tracing::error!(
                    msg,
                    "EXIT_ON_FAULT — drip cohort stalled; \
                     aborting klippy so systemd restarts it"
                );
                abort_after_tracing_appender_drains();
            }),
            backlog: Arc::clone(&self.pump_backlog),
        };

        let anchor_mutex = Arc::clone(&self.dispatch_anchor);
        *anchor_mutex.lock().unwrap_or_else(|p| p.into_inner()) = crate::anchor::Anchor::new();
        let dispatch_resources = crate::worker::DispatchResources {
            router: Arc::clone(&router_arc),
            anchor: anchor_mutex,
            mcu_configs: mcu_configs.to_vec(),
            drain: self.drain.clone(),
            counter: Arc::clone(&counter),
            active_drip_cohort: Arc::clone(&self.active_drip_cohort),
            motion_history: Arc::clone(&self.motion_history),
            nominal_freqs: Arc::clone(&self.nominal_clock_freqs),
        };

        let stream_cfg = {
            let cart = cfg.cartesian;
            motion_pipeline::StreamConfig {
                chain: cfg.chain,
                integration_tol: STREAM_INTEGRATION_TOL,
                max_extrude_only_velocity_mm_s: cfg
                    .max_extrude_only_velocity
                    .unwrap_or(f64::INFINITY),
                max_extrude_only_accel_mm_s2: cfg.max_extrude_only_accel.unwrap_or(f64::INFINITY),
                fit_tol_mm: cfg.fit_tolerance_mm,
                max_buffer_moves: STREAM_MAX_BUFFER_MOVES,
                limits: geometry::VelocityLimits::try_new(
                    cart.max_velocity,
                    cart.max_accel,
                    cart.square_corner_velocity,
                    cart.max_jerk,
                )
                .map_err(PyRuntimeError::new_err)?,
            }
        };
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

    fn spawn_live_position_poll_thread(&self) {
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
                while !stop.load(Ordering::Relaxed) {
                    std::thread::sleep(period);
                    if stop.load(Ordering::Relaxed) {
                        break;
                    }
                    match collect_motor_positions_inner(&configs, &mcus, timeout) {
                        Ok(map) => {
                            let mut c = cache.lock().unwrap_or_else(|p| p.into_inner());
                            *c = (map, std::time::Instant::now());
                        }
                        Err(e) => {
                            if !e.contains("no axes configured") {
                                tracing::warn!(
                                    error = %e,
                                    "live-position poll failed; serving stale cache"
                                );
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

    fn wire_mcu_supervision(
        &self,
        mcu_configs: &[McuAxisConfig],
        ethercat_mcu_ids: &HashSet<u32>,
        ec_conns: &HashMap<u32, Arc<McuSerialConn>>,
        host_ios: &HashMap<u32, Arc<McuHostIo>>,
        pump_control: crossbeam_channel::Sender<crate::pump::PumpMsg>,
    ) {
        for cfg_mcu in mcu_configs {
            let mcu_id = cfg_mcu.mcu_id;
            let pump_tx_hb = pump_control.clone();
            let drain_hb = self.drain.clone();

            if ethercat_mcu_ids.contains(&mcu_id) {
                let conn = ec_conns
                    .get(&mcu_id)
                    .expect("ec_conns built from ethercat_mcu_ids")
                    .clone();

                let mcu_label = {
                    let mcus_lock = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
                    mcus_lock
                        .get(&mcu_id)
                        .map(|c| c.label.clone())
                        .unwrap_or_else(|| format!("mcu-{mcu_id}"))
                };

                let homing_run_hb = Arc::clone(&self.homing_run);
                let active_cohort_hb = Arc::clone(&self.active_drip_cohort);
                let pump_tx_fault = pump_control.clone();
                let latched_fault_hb = Arc::clone(&self.latched_drive_fault);
                let mcu_label_hb = mcu_label.clone();
                let slot_axes_hb = cfg_mcu.axes.clone();
                conn.attach_heartbeat_callback(Arc::new(
                    move |hb: &mcu_protocol::messages::StatusHeartbeat| {
                        if hb.fault_code != 0 {
                            let run_opt = {
                                let mut guard =
                                    homing_run_hb.lock().unwrap_or_else(|p| p.into_inner());
                                match guard.as_ref().map(|r| r.axis_key.mcu_id) {
                                    Some(axis_mcu)
                                        if crate::homing::route_drive_fault(
                                            mcu_id,
                                            Some(axis_mcu),
                                        ) == crate::homing::DriveFaultRoute::HomingError =>
                                    {
                                        guard.take()
                                    }
                                    _ => None,
                                }
                            };
                            match run_opt {
                                Some(run) => {
                                    latched_fault_hb
                                        .lock()
                                        .unwrap_or_else(|p| p.into_inner())
                                        .insert(mcu_id, hb.fault_code);
                                    *active_cohort_hb.lock().unwrap_or_else(|p| p.into_inner()) =
                                        None;
                                    let _ = pump_tx_fault.send(crate::pump::PumpMsg::Flush(
                                        run.all_axis_keys.clone(),
                                    ));
                                    let _ = pump_tx_fault
                                        .send(crate::pump::PumpMsg::DripDisarm(run.cohort));
                                    let _ = run.notify.send(Err(format!(
                                        "drive fault 0x{:04x} during homing — \
                                     following-error/torque limit exceeded (endstop failure?)",
                                        hb.fault_code
                                    )));
                                }
                                None => {
                                    let prev = latched_fault_hb
                                        .lock()
                                        .unwrap_or_else(|p| p.into_inner())
                                        .insert(mcu_id, hb.fault_code);
                                    if prev != Some(hb.fault_code) {
                                        tracing::error!(
                                            mcu_id,
                                            mcu_label = %mcu_label_hb,
                                            fault_code = hb.fault_code,
                                            "ethercat drive fault — latched for klippy to report"
                                        );
                                    }
                                }
                            }
                            return;
                        }
                        let _ = pump_tx_hb.send(crate::pump::PumpMsg::Heartbeat(
                            crate::pump::HeartbeatMsg {
                                mcu_id,
                                retired_counts: hb.retired_counts.clone(),
                            },
                        ));
                        for (slot, &r) in hb.retired_counts.iter().enumerate() {
                            if let Some(&axis) = slot_axes_hb.get(slot) {
                                drain_hb.set_retired(mcu_id, axis as u8, r);
                            }
                        }
                    },
                ));

                let trip_deps = self.trip_deps();
                conn.attach_endstop_trip_callback(Arc::new(
                    move |endstop_id: u8, trip_clock: u64| {
                        dispatch_endstop_trip(&trip_deps, mcu_id, endstop_id, trip_clock);
                    },
                ));

                let conn_for_poll = Arc::downgrade(&conn);
                let mcus_for_supervision = Arc::clone(&self.mcus);
                let endpoint_death_for_supervision = Arc::clone(&self.latched_endpoint_death);
                let on_endpoint_death: Box<dyn Fn(&str) + Send + 'static> =
                    Box::new(move |reason: &str| {
                        if report_ethercat_endpoint_death(
                            &endpoint_death_for_supervision,
                            mcu_id,
                            reason,
                        ) {
                            arm_endpoint_death_watchdog(
                                Arc::clone(&endpoint_death_for_supervision),
                                mcu_id,
                            );
                        }
                    });

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
                                let mut mcus = mcus_for_supervision
                                    .lock()
                                    .unwrap_or_else(|p| p.into_inner());
                                let Some(c) = mcus.get_mut(&mcu_id) else {
                                    return;
                                };
                                if peer_eof {
                                    Some("conn EOF".to_string())
                                } else if let Some(ref mut child) = c.endpoint_process {
                                    match child.try_wait() {
                                        Ok(Some(status)) => Some(format!("child exited: {status}")),
                                        Ok(None) => None,
                                        Err(e) => Some(format!("try_wait error: {e}")),
                                    }
                                } else {
                                    None
                                }
                            };

                            if let Some(reason) = fault_reason {
                                on_endpoint_death(&reason);
                                return;
                            }

                            std::thread::sleep(Duration::from_millis(1));
                        }
                    })
                    .expect("spawn ec-heartbeat-poll thread");
            } else {
                let io = host_ios
                    .get(&mcu_id)
                    .expect("host_io map built from mcu_configs")
                    .clone();
                io.attach_heartbeat_callback(Arc::new(move |retired: &[u32]| {
                    let _ = pump_tx_hb.send(crate::pump::PumpMsg::Heartbeat(
                        crate::pump::HeartbeatMsg {
                            mcu_id,
                            retired_counts: retired.to_vec(),
                        },
                    ));
                    for (axis, &r) in retired.iter().enumerate() {
                        drain_hb.set_retired(mcu_id, axis as u8, r);
                    }
                }));
            }
        }
    }

    fn e_followers(&self, de: f64) -> PyResult<Vec<(usize, f64)>> {
        if de.abs() > 0.0 {
            let cfg = self
                .planner_config
                .lock()
                .unwrap_or_else(|p| p.into_inner());
            let axis_index = cfg.axis_registry.axis_index("e").map_err(|_| {
                PyRuntimeError::new_err(
                    "E word on a move but no [axis e] is declared — declare the \
                     follower axis or stop sending E",
                )
            })?;
            Ok(vec![(axis_index, de)])
        } else {
            Ok(vec![])
        }
    }
}
