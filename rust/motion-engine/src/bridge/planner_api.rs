use super::{
    DRAIN_TIMEOUT, FieldValue, HashSet, Ordering, PyMotionEngine, PyResult, PyRuntimeError,
    PyValueError, Python, SPATIAL_AXES, classify, config, planner_err, pymethods, require_positive,
};

fn unsupported_curve(py: Python<'_>, message: &'static str) -> PyResult<()> {
    py.detach(|| Err(PyRuntimeError::new_err(message)))
}

fn read_planner_config_sections(
    axes: Vec<(String, Vec<String>, Vec<String>, Vec<String>)>,
    limits: Vec<(String, Vec<String>, Option<f64>, Option<f64>, Option<f64>)>,
    post_processors: Vec<(String, String, Vec<(String, f64)>)>,
    kinematics_axes: &[String],
    cartesian_limits: (f64, f64, f64, f64, f64, f64),
) -> PyResult<(
    config::AxisRegistry,
    config::PostProcessorSet,
    Vec<config::LimitSection>,
    config::CartesianLimits,
)> {
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

    Ok((axis_registry, post_processor_set, limit_sections, cartesian))
}

fn validate_extrude_and_fit_params(
    max_extrude_only_velocity: Option<f64>,
    max_extrude_only_accel: Option<f64>,
    fit_tolerance_mm: Option<f64>,
    fit_tolerance_accel_mm_s2: Option<f64>,
) -> PyResult<()> {
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

    if let Some(v) = fit_tolerance_mm {
        if !(v.is_finite() && v > 0.0) {
            return Err(PyValueError::new_err(format!(
                "[printer] max_path_deviation must be finite and positive, got {v}"
            )));
        }
    }

    if let Some(v) = fit_tolerance_accel_mm_s2 {
        if !(v > 0.0) {
            return Err(PyValueError::new_err(format!(
                "[printer] max_accel_deviation must be positive, got {v}"
            )));
        }
    }

    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn apply_planner_config(
    axis_registry: config::AxisRegistry,
    limit_sections: Vec<config::LimitSection>,
    cartesian: config::CartesianLimits,
    post_processor_set: config::PostProcessorSet,
    max_extrude_only_velocity: Option<f64>,
    max_extrude_only_accel: Option<f64>,
    fit_tolerance_mm: Option<f64>,
    fit_tolerance_accel_mm_s2: Option<f64>,
    arc_fit: Option<u32>,
) -> PyResult<config::PlannerConfig> {
    let mut cfg = config::PlannerConfig::default();
    cfg.axis_registry = axis_registry;
    cfg.limit_sections = limit_sections;
    cfg.cartesian = cartesian;
    cfg.post_processors = post_processor_set;
    cfg.max_extrude_only_velocity = max_extrude_only_velocity;
    cfg.max_extrude_only_accel = max_extrude_only_accel;
    if let Some(v) = fit_tolerance_mm {
        cfg.fit_tolerance_mm = v;
    }
    if let Some(v) = fit_tolerance_accel_mm_s2 {
        cfg.fit_tolerance_accel_mm_s2 = v;
    }
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
    fit_tolerance_mm: Option<f64>,
    fit_tolerance_accel_mm_s2: Option<f64>,
) -> PyResult<config::PlannerConfig> {
    let (axis_registry, post_processor_set, limit_sections, cartesian) =
        read_planner_config_sections(
            axes,
            limits,
            post_processors,
            kinematics_axes,
            cartesian_limits,
        )?;

    validate_extrude_and_fit_params(
        max_extrude_only_velocity,
        max_extrude_only_accel,
        fit_tolerance_mm,
        fit_tolerance_accel_mm_s2,
    )?;

    apply_planner_config(
        axis_registry,
        limit_sections,
        cartesian,
        post_processor_set,
        max_extrude_only_velocity,
        max_extrude_only_accel,
        fit_tolerance_mm,
        fit_tolerance_accel_mm_s2,
        arc_fit,
    )
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
        fit_tolerance_mm = None,
        fit_tolerance_accel_mm_s2 = None,
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
        fit_tolerance_mm: Option<f64>,
        fit_tolerance_accel_mm_s2: Option<f64>,
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
            fit_tolerance_mm,
            fit_tolerance_accel_mm_s2,
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
    /// Push one move into the pipe. Returns `false` when the pipe is full —
    /// queued motion has reached the configured depth, or the entry channel
    /// itself is full — in which case nothing was consumed and the caller
    /// retries after yielding. This return value is the host's entire feed
    /// throttle: the reader pushes as much as fits, the pipe defines "fits".
    #[pyo3(signature = (dx, dy, dz, de, feedrate))]
    fn submit_move(
        &self,
        py: Python<'_>,
        dx: f64,
        dy: f64,
        dz: f64,
        de: f64,
        feedrate: f64,
    ) -> PyResult<bool> {
        py.detach(|| -> PyResult<bool> {
            tracing::trace!(
                subsystem = "motion",
                event = "submit_move_enter",
                dx,
                dy,
                dz,
                de,
                feedrate,
                "engine.submit_move enter"
            );
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
                match planner.submit_move(m) {
                    Ok(()) => {}
                    Err(crate::worker::StreamWorkerError::ChannelFull) => return Ok(false),
                    Err(e) => return Err(planner_err(e)),
                }
                tracing::trace!(
                    subsystem = "motion",
                    event = "intake_submit",
                    line_no,
                    channel_pending = planner.pending_channel_moves(),
                    "[intake] move accepted into the pipe"
                );
            }

            let mut pos = self.commanded_pos.lock().unwrap_or_else(|p| p.into_inner());
            pos[0] += dx;
            pos[1] += dy;
            pos[2] += dz;
            *self.last_g5_pq.lock().unwrap_or_else(|p| p.into_inner()) = None;
            Ok(true)
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
        unsupported_curve(
            py,
            "submit_bezier (G5 cubic) is not yet supported by the new geometry pipeline \
             — V1 streams G0/G1 line moves (and reconstructs arcs from facets); curve \
             faceting is a follow-up. Slice without G5.",
        )
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
        unsupported_curve(
            py,
            "submit_quadratic (G2/G3 arc as quadratic) is not yet supported by the new \
             geometry pipeline — V1 streams G0/G1 line moves; curve faceting is a \
             follow-up. Decompose arcs into line segments upstream.",
        )
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

        self.reseed_mcus_after_position_set(py, x, y, z)?;
        self.rebase_motion_history_after_position_set(host_now, x, y, z);

        Ok(())
    }

    fn reseed_mcus_after_position_set(
        &self,
        py: Python<'_>,
        x: f64,
        y: f64,
        z: f64,
    ) -> PyResult<()> {
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
        Ok(())
    }

    fn rebase_motion_history_after_position_set(&self, host_now: f64, x: f64, y: f64, z: f64) {
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
