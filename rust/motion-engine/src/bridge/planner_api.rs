use super::{
    Arc, DRAIN_TIMEOUT, FieldValue, HashSet, Ordering, PyMotionEngine, PyResult, PyRuntimeError,
    PyValueError, Python, SPATIAL_AXES, classify, config, planner_err, pymethods, require_positive,
};
use crate::lock_ext::LockExt;

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

struct PlannerTuning {
    arc_fit: Option<u32>,
    max_extrude_only_velocity: Option<f64>,
    max_extrude_only_accel: Option<f64>,
    fit_tolerance_mm: Option<f64>,
    fit_tolerance_accel_mm_s2: Option<f64>,
}

fn apply_planner_config(
    axis_registry: config::AxisRegistry,
    limit_sections: Vec<config::LimitSection>,
    cartesian: config::CartesianLimits,
    post_processor_set: config::PostProcessorSet,
    tuning: &PlannerTuning,
) -> PyResult<config::PlannerConfig> {
    let mut cfg = config::PlannerConfig::default();
    cfg.axis_registry = axis_registry;
    cfg.limit_sections = limit_sections;
    cfg.cartesian = cartesian;
    cfg.post_processors = post_processor_set;
    cfg.max_extrude_only_velocity = tuning.max_extrude_only_velocity;
    cfg.max_extrude_only_accel = tuning.max_extrude_only_accel;
    if let Some(v) = tuning.fit_tolerance_mm {
        cfg.fit_tolerance_mm = v;
    }
    if let Some(v) = tuning.fit_tolerance_accel_mm_s2 {
        cfg.fit_tolerance_accel_mm_s2 = v;
    }
    cfg.chain = match tuning.arc_fit {
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

fn build_planner_config(
    axes: Vec<(String, Vec<String>, Vec<String>, Vec<String>)>,
    limits: Vec<(String, Vec<String>, Option<f64>, Option<f64>, Option<f64>)>,
    post_processors: Vec<(String, String, Vec<(String, f64)>)>,
    kinematics_axes: &[String],
    cartesian_limits: (f64, f64, f64, f64, f64, f64),
    tuning: &PlannerTuning,
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
        tuning.max_extrude_only_velocity,
        tuning.max_extrude_only_accel,
        tuning.fit_tolerance_mm,
        tuning.fit_tolerance_accel_mm_s2,
    )?;

    apply_planner_config(
        axis_registry,
        limit_sections,
        cartesian,
        post_processor_set,
        tuning,
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
            let guard = self.planner.lock_ok();
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
        if self.planner.lock_ok().is_some() {
            return Err(PyRuntimeError::new_err("planner already initialized"));
        }

        let cfg = build_planner_config(
            axes,
            limits,
            post_processors,
            &kinematics_axes,
            cartesian_limits,
            &PlannerTuning {
                arc_fit,
                max_extrude_only_velocity,
                max_extrude_only_accel,
                fit_tolerance_mm,
                fit_tolerance_accel_mm_s2,
            },
        )?;
        *self.planner_config.lock_ok() = cfg.clone();

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
            let pos = *self.commanded_pos.lock_ok();
            let (max_v, max_a, scv, jerk) = {
                let cfg = self.planner_config.lock_ok();
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
                [dx, dy, dz],
                extruder_axis,
                e_delta,
                limits,
                feedrate,
                line_no,
            )
            .map_err(|e| PyRuntimeError::new_err(format!("{e:?}")))?;

            {
                let guard = self.planner.lock_ok();
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

            let mut pos = self.commanded_pos.lock_ok();
            pos[0] += dx;
            pos[1] += dy;
            pos[2] += dz;
            *self.last_g5_pq.lock_ok() = None;
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
        let guard = self.planner.lock_ok();
        let planner = guard.as_ref().ok_or_else(|| {
            PyRuntimeError::new_err("planner not initialized — call init_planner first")
        })?;
        planner.dwell(duration_s).map_err(planner_err)?;
        *self.last_g5_pq.lock_ok() = None;
        Ok(())
    }
    #[pyo3(signature = (x, y, z, host_now))]
    fn set_position(&self, py: Python<'_>, x: f64, y: f64, z: f64, host_now: f64) -> PyResult<()> {
        {
            let mut pos = self.commanded_pos.lock_ok();
            *pos = [x, y, z];
        }
        *self.last_g5_pq.lock_ok() = None;

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
        let planner_guard = self.planner.lock_ok();
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
                let configs = self.mcu_axis_configs.lock_ok();
                let mcus = self.mcus.lock_ok();
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
            let mcus = self.mcus.lock_ok();
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
        let configs: Vec<crate::mcu_config::McuAxisConfig> =
            self.mcu_axis_configs.lock_ok().clone();
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
            let mut store = self.motion_history.lock_ok();
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
        self.planner_config.lock_ok().effective_limits()
    }
    #[pyo3(signature = (velocity))]
    fn set_velocity_cap(&self, velocity: Option<f64>) -> PyResult<()> {
        require_positive(velocity, "velocity")?;
        self.planner_config.lock_ok().runtime_caps.velocity = velocity;
        Ok(())
    }
    #[pyo3(signature = (accel))]
    fn set_accel_cap(&self, accel: Option<f64>) -> PyResult<()> {
        require_positive(accel, "accel")?;
        self.planner_config.lock_ok().runtime_caps.accel = accel;
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
        self.planner_config.lock_ok().runtime_square_corner_velocity = square_corner_velocity;
        Ok(())
    }
    fn update_post_processor(&self, name: &str, key: &str, value: f64) -> PyResult<()> {
        let axis_chains = {
            let mut cfg = self.planner_config.lock_ok();
            cfg.post_processors
                .set_param(name, key, value)
                .map_err(|e| PyValueError::new_err(e.to_string()))?;
            cfg.post_processors
                .compile(&cfg.axis_registry)
                .map_err(|e| PyValueError::new_err(e.to_string()))?
        };
        if let Some(handle) = self.planner.lock_ok().as_ref() {
            handle
                .update_axis_chains(axis_chains)
                .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;
        }
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    fn set_bed_mesh(
        &self,
        points: Vec<f64>,
        x_min: f64,
        y_min: f64,
        dx: f64,
        dy: f64,
        nx: usize,
        ny: usize,
        tension: f64,
        fade: Option<(f64, f64, f64)>,
        zero_ref_x: f64,
        zero_ref_y: f64,
    ) -> PyResult<f64> {
        let mut mesh = geometry::MeshGrid::new(x_min, y_min, dx, dy, nx, ny, points, tension)
            .map_err(|e| PyValueError::new_err(format!("set_bed_mesh: {e}")))?;
        if !mesh.contains(zero_ref_x, zero_ref_y) {
            return Err(PyValueError::new_err(format!(
                "set_bed_mesh: zero reference ({zero_ref_x}, {zero_ref_y}) is outside the \
                 mesh area {:?} x {:?} — a mesh nonzero at the Z datum would shift it",
                mesh.x_range(),
                mesh.y_range()
            )));
        }
        mesh.zero_at(zero_ref_x, zero_ref_y);
        let fade = match fade {
            Some((start, end, target)) => geometry::Fade::new(start, end, target)
                .map_err(|e| PyValueError::new_err(format!("set_bed_mesh: {e}")))?,
            None => geometry::Fade::disabled(),
        };
        let transform = geometry::SurfaceTransform::new(mesh, fade);
        let bounds = transform.bounds();
        if !fade.is_disabled() {
            let (start, end) = fade.band();
            let mesh_span = bounds.z_max.abs().max(bounds.z_min.abs());
            if end - start <= mesh_span {
                return Err(PyValueError::new_err(format!(
                    "set_bed_mesh: fade band {:.3}mm is not wider than the mesh deviation \
                     {mesh_span:.3}mm — the transform would not be invertible",
                    end - start
                )));
            }
        }
        let (limits, z_velocity_budget, z_accel_budget) = {
            let cfg = self.planner_config.lock_ok();
            let z_axis = cfg
                .axis_registry
                .axis_index("z")
                .map_err(|e| PyValueError::new_err(format!("set_bed_mesh: {e}")))?;
            let mut z_v = cfg.cartesian.max_z_velocity;
            let mut z_a = cfg.cartesian.max_z_accel;
            for section in cfg
                .limit_sections
                .iter()
                .filter(|s| s.axes.contains(&z_axis))
            {
                if let Some(v) = section.max_velocity {
                    z_v = z_v.min(v);
                }
                if let Some(a) = section.max_accel {
                    z_a = z_a.min(a);
                }
            }
            (cfg.cartesian, z_v, z_a)
        };
        let coupled_v = bounds.max_gradient * limits.max_velocity;
        let coupled_a = bounds.max_gradient * limits.max_accel
            + bounds.max_curvature * limits.max_velocity * limits.max_velocity;
        if coupled_v > z_velocity_budget || coupled_a > z_accel_budget {
            return Err(PyValueError::new_err(format!(
                "set_bed_mesh: bed deviation needs {coupled_v:.2}mm/s / {coupled_a:.1}mm/s² \
                 of Z at your XY limits; Z allows {z_velocity_budget:.2}mm/s / \
                 {z_accel_budget:.1}mm/s² — the bed is warped or the Z limits are too \
                 conservative (mesh range {:.3}..{:.3}mm, max slope {:.4})",
                bounds.z_min, bounds.z_max, bounds.max_gradient
            )));
        }
        tracing::info!(
            subsystem = "motion",
            event = "bed_mesh_activated",
            z_min = bounds.z_min,
            z_max = bounds.z_max,
            max_slope = bounds.max_gradient,
            envelope_v_mm_s = z_velocity_budget + coupled_v,
            envelope_a_mm_s2 = z_accel_budget + coupled_a,
            "bed mesh activated; transient Z exceedance envelope logged"
        );
        self.swap_bed_mesh(Some(Arc::new(transform)))
    }

    fn clear_bed_mesh(&self) -> PyResult<f64> {
        self.swap_bed_mesh(None)
    }

    /// Gcode-space Z the toolhead is commanded at, given a machine-space Z at
    /// (x, y) — the single machine→gcode crossing, used by the host after
    /// homing and probing. Identity when no mesh is active.
    fn bed_mesh_gcode_z(&self, x: f64, y: f64, z_machine: f64) -> f64 {
        match self.bed_mesh.lock_ok().as_ref() {
            Some(t) => t.gcode_z(x, y, z_machine),
            None => z_machine,
        }
    }
}

impl PyMotionEngine {
    /// Swap the active surface transform behind a pipeline drain, keeping the
    /// physical position invariant: the machine Z at the current rest point is
    /// re-expressed as a gcode Z through the *new* transform, and that rebase
    /// rides the token so every gcode-space odometer moves together. Returns
    /// the rebased gcode Z for the host's own position bookkeeping.
    fn swap_bed_mesh(&self, new: Option<Arc<geometry::SurfaceTransform>>) -> PyResult<f64> {
        let pos = *self.commanded_pos.lock_ok();
        let mut current = self.bed_mesh.lock_ok();
        let machine_z = pos[2]
            + current
                .as_ref()
                .map_or(0.0, |t| t.correction_at(pos[0], pos[1], pos[2]));
        let rebase = new
            .as_ref()
            .map_or(machine_z, |t| t.gcode_z(pos[0], pos[1], machine_z));
        if let Some(handle) = self.planner.lock_ok().as_ref() {
            handle
                .update_mesh(new.clone(), rebase)
                .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;
        }
        *current = new;
        drop(current);
        self.commanded_pos.lock_ok()[2] = rebase;
        Ok(rebase)
    }

    pub(crate) fn machine_to_gcode(&self, pos: [f64; 3]) -> [f64; 3] {
        match self.bed_mesh.lock_ok().as_ref() {
            Some(t) => [pos[0], pos[1], t.gcode_z(pos[0], pos[1], pos[2])],
            None => pos,
        }
    }

    fn e_followers(&self, de: f64) -> PyResult<Vec<(usize, f64)>> {
        if de.abs() > 0.0 {
            let cfg = self.planner_config.lock_ok();
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
