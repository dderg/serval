use super::{
    Arc, DRAIN_TIMEOUT, FieldValue, HashSet, McuTopologyInput, Ordering, PyMotionEngine, PyResult,
    PyRuntimeError, PyValueError, Python, classify, config, planner_err, pymethods,
    require_positive,
};
use crate::lock_ext::LockExt;
use config::from_doc::{planner_config_from_settings, read_motion_settings};
use pyo3::FromPyObject;

fn unsupported_curve(py: Python<'_>, message: &'static str) -> PyResult<()> {
    py.detach(|| Err(PyRuntimeError::new_err(message)))
}

#[derive(FromPyObject)]
struct McuTopology {
    mcu_id: u32,
    axes: Vec<u8>,
    kinematics: u8,
    max_motor_velocity: Vec<f64>,
    stepping_mode: u8,
    microstep_distance: Vec<f64>,
    invert_dir: Vec<bool>,
    stepper_oids: Vec<u32>,
    stepcompress_sample_rate: f64,
    move_queue_slots: u32,
}

impl McuTopology {
    fn into_core(self) -> McuTopologyInput {
        McuTopologyInput {
            mcu_id: self.mcu_id,
            axes: self.axes,
            kinematics: self.kinematics,
            max_motor_velocity: self.max_motor_velocity,
            stepping_mode: self.stepping_mode,
            microstep_distance: self.microstep_distance,
            invert_dir: self.invert_dir,
            stepper_oids: self.stepper_oids,
            stepcompress_sample_rate: self.stepcompress_sample_rate,
            move_queue_slots: self.move_queue_slots,
        }
    }
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
        let rx = {
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
                })
                .map_err(|e| PyRuntimeError::new_err(e.to_string()))?
        };
        rx.recv()
            .map_err(|_| PyRuntimeError::new_err("nudge notify dropped"))?
            .map_err(PyRuntimeError::new_err)?;
        let (accel_t, cruise_t, _v) = crate::nudge::calc_move_time(delta_mm, speed, accel);
        Ok(accel_t + cruise_t + accel_t)
    }
    /// `config_text` is the serialized config document; the motion-owned
    /// sections are re-read here with the same reader
    /// (`_config_doc.read_motion_settings`) klippy used at config time, so
    /// the planner cannot drift from what the host validated and reported.
    fn init_planner(&self, config_text: &str, mcus: Vec<McuTopology>) -> PyResult<()> {
        if self.planner.lock_ok().is_some() {
            return Err(PyRuntimeError::new_err("planner already initialized"));
        }

        let doc = config_doc::Document::parse(config_text, "<config>")
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        let (settings, _consumed) = read_motion_settings(&doc).map_err(PyValueError::new_err)?;
        let cfg = planner_config_from_settings(&settings).map_err(PyValueError::new_err)?;
        *self.planner_config.lock_ok() = cfg.clone();

        let mcus: Vec<McuTopologyInput> = mcus.into_iter().map(McuTopology::into_core).collect();
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
            let pos = self.commanded_pos.lock_ok().0;
            let (max_v, max_a, corner_deviation, jerk) = {
                let cfg = self.planner_config.lock_ok();
                let (mut v, mut a) = cfg.cartesian.for_move(dx, dy, dz);
                if let Some(rv) = cfg.runtime_caps.velocity {
                    v = v.min(rv);
                }
                if let Some(ra) = cfg.runtime_caps.accel {
                    a = a.min(ra);
                }
                let j = cfg
                    .runtime_caps
                    .jerk_override
                    .unwrap_or(cfg.cartesian.max_jerk);
                (v, a, cfg.corner_deviation(), j)
            };
            let limits = geometry::VelocityLimits::try_new(max_v, max_a, corner_deviation, jerk)
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
            pos.0[0] += dx;
            pos.0[1] += dy;
            pos.0[2] += dz;
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
        let gcode = geometry::GcodePos([x, y, z]);
        {
            let mut pos = self.commanded_pos.lock_ok();
            *pos = gcode;
        }
        *self.last_g5_pq.lock_ok() = None;

        // A set_position renames the physical rest point in gcode space; the
        // physical point itself is machine space, so everything that tracks
        // physical state (step counters, motion history) is seeded through
        // the forward warp while the stream odometer takes the gcode value.
        let machine = self.machine_from_gcode(gcode);
        self.reseed_mcus_after_position_set(py, gcode, machine)?;
        self.rebase_motion_history_after_position_set(host_now, machine);

        Ok(())
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
    /// Replace the static `[printer] max_jerk` for subsequent moves —
    /// unlike the velocity/accel caps this may RAISE the limit (infinity
    /// disables jerk limiting entirely). Calibration transients
    /// (SERVO_MEASURE_RINGDOWN) use it so the stop excites the raw plant.
    /// `None` restores the configured limit.
    #[pyo3(signature = (jerk))]
    fn set_jerk_override(&self, jerk: Option<f64>) -> PyResult<()> {
        if let Some(j) = jerk {
            if !(j > 0.0) {
                return Err(PyValueError::new_err(
                    "jerk override must be positive (infinity disables jerk limiting)",
                ));
            }
        }
        self.planner_config.lock_ok().runtime_caps.jerk_override = jerk;
        Ok(())
    }
    /// Swap the live pipeline chains between the configured post-processors
    /// and identity (no shaping). Fails without touching the flag when the
    /// restored chains no longer compile — the caller must hear that
    /// shaping did NOT come back.
    fn set_post_processor_bypass(&self, enabled: bool) -> PyResult<()> {
        let axis_chains = {
            let mut cfg = self.planner_config.lock_ok();
            let previous = cfg.post_processor_bypass;
            cfg.post_processor_bypass = enabled;
            let axis_chains = match cfg.compile_active_chains() {
                Ok(chains) => chains,
                Err(e) => {
                    cfg.post_processor_bypass = previous;
                    return Err(PyValueError::new_err(e.to_string()));
                }
            };
            axis_chains
        };
        if let Some(handle) = self.planner.lock_ok().as_ref() {
            handle
                .update_axis_chains(axis_chains)
                .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;
        }
        Ok(())
    }
    #[pyo3(signature = (corner_deviation))]
    fn set_corner_deviation(&self, corner_deviation: Option<f64>) -> PyResult<()> {
        if let Some(deviation) = corner_deviation {
            if !(deviation.is_finite() && deviation >= 0.0) {
                return Err(PyValueError::new_err(
                    "corner_deviation must be finite and non-negative",
                ));
            }
        }
        self.planner_config.lock_ok().runtime_corner_deviation = corner_deviation;
        Ok(())
    }
    fn update_post_processor(&self, name: &str, key: &str, value: f64) -> PyResult<()> {
        let axis_chains = {
            let mut cfg = self.planner_config.lock_ok();
            cfg.post_processors
                .set_param(name, key, value)
                .map_err(|e| PyValueError::new_err(e.to_string()))?;
            cfg.compile_active_chains()
                .map_err(|e| PyValueError::new_err(e.to_string()))?
        };
        if let Some(handle) = self.planner.lock_ok().as_ref() {
            handle
                .update_axis_chains(axis_chains)
                .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;
        }
        Ok(())
    }

    fn post_processor_param(&self, name: &str, key: &str) -> Option<f64> {
        self.planner_config
            .lock_ok()
            .post_processors
            .param(name, key)
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
        z_velocity_limit: Option<f64>,
        z_accel_limit: Option<f64>,
    ) -> PyResult<(f64, (f64, f64, f64, f64, f64))> {
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
            (
                cfg.cartesian,
                z_velocity_limit.unwrap_or(cfg.cartesian.max_z_velocity),
                z_accel_limit.unwrap_or(cfg.cartesian.max_z_accel),
            )
        };
        let coupled_v = bounds.max_gradient * limits.max_velocity;
        let coupled_a = bounds.max_gradient * limits.max_accel
            + bounds.max_curvature * limits.max_velocity * limits.max_velocity;
        if coupled_v > z_velocity_budget || coupled_a > z_accel_budget {
            tracing::warn!(
                subsystem = "motion",
                event = "bed_mesh_z_budget_exceeded",
                coupled_v_mm_s = coupled_v,
                coupled_a_mm_s2 = coupled_a,
                z_velocity_budget,
                z_accel_budget,
                max_slope = bounds.max_gradient,
                "mesh-following Z demand exceeds the Z budget; mesh activated anyway"
            );
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
        let rebase = self.swap_bed_mesh(Some(Arc::new(transform)))?;
        Ok((
            rebase,
            (
                coupled_v,
                coupled_a,
                z_velocity_budget,
                z_accel_budget,
                bounds.max_gradient,
            ),
        ))
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
    /// Re-anchor every serial MCU's step counters at a machine-space rest
    /// position. The MCU-side `runtime_seed_position` also zeroes all
    /// non-spatial motor positions, so this is the counterpart of any
    /// host-side reset that returns the extruder odometer to 0. Requires a
    /// quiesced, drained pipeline. Counters count physical steps: callers
    /// holding a gcode position convert with [`Self::machine_from_gcode`].
    pub(crate) fn send_serial_position_seeds(&self, pos: geometry::MachinePos) -> PyResult<()> {
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
            crate::mcu_config::build_serial_seed_sends(&configs, &ethercat_mcu_ids, pos)
        };
        let mcus = self.mcus.lock_ok();
        for s in sends {
            let conn = mcus.get(&s.mcu_id).unwrap_or_else(|| {
                panic!(
                    "position seed: planner up but mcu_id {} absent \
                     (broken invariant)",
                    s.mcu_id
                )
            });
            let io = conn.host_io.as_ref().unwrap_or_else(|| {
                panic!(
                    "position seed: serial mcu_id {} has no host_io \
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
                    "position seed send to mcu_id {} failed: {e:?}",
                    s.mcu_id
                ))
            })?;
        }
        self.seed_stepcompress_shims(pos)
    }

    /// Classic stepping keeps the step counter on the host: re-anchor each
    /// stepcompress motor's shim counter so the next drain re-emits
    /// `reset_step_clock` from the new position.
    fn seed_stepcompress_shims(&self, pos: geometry::MachinePos) -> PyResult<()> {
        let configs = self.mcu_axis_configs.lock_ok().clone();
        let endpoints = self.stepcompress_endpoints.lock_ok().clone();
        for cfg in configs
            .iter()
            .filter(|c| c.stepping_mode == crate::mcu_config::SteppingMode::Stepcompress)
        {
            let motor = crate::mcu_config::motor_frame(cfg, pos.0);
            let mut counts = Vec::with_capacity(cfg.axes.len());
            for &axis in &cfg.axes {
                let key = crate::types::AxisKey {
                    mcu_id: cfg.mcu_id,
                    axis: axis as u8,
                };
                let lane = crate::homing::stepcompress_lane(cfg, key)
                    .map_err(PyRuntimeError::new_err)?
                    .ok_or_else(|| {
                        PyRuntimeError::new_err(format!(
                            "position seed: stepcompress mcu {} axis {axis} has no shim lane",
                            cfg.mcu_id
                        ))
                    })?;
                // A follower lane (extruder) has no spatial coordinate: hold
                // whatever motion history already answers for it, the value
                // `rebase_motion_history_after_position_set` writes back, so
                // shim, mcu counter and history keep one shared origin.
                let mm = match motor.get(axis) {
                    Some(&spatial) => spatial,
                    None => self
                        .motion_history
                        .lock_ok()
                        .final_position(key)
                        .unwrap_or(0.0),
                };
                counts.push(lane.mm_to_steps(mm));
            }
            let endpoint = endpoints.get(&cfg.mcu_id).ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "position seed: no shim endpoint registered for stepcompress mcu {}",
                    cfg.mcu_id
                ))
            })?;
            endpoint.lock_ok().reset_position(&counts).map_err(|e| {
                PyRuntimeError::new_err(format!(
                    "position seed: shim reseed failed for mcu {}: {e:?}",
                    cfg.mcu_id
                ))
            })?;
        }
        Ok(())
    }

    /// Swap the active surface transform behind a pipeline drain, keeping the
    /// physical position invariant: the machine Z at the current rest point is
    /// re-expressed as a gcode Z through the *new* transform, and that rebase
    /// rides the token so every gcode-space odometer moves together. Returns
    /// the rebased gcode Z for the host's own position bookkeeping.
    fn swap_bed_mesh(&self, new: Option<Arc<geometry::SurfaceTransform>>) -> PyResult<f64> {
        let pos = *self.commanded_pos.lock_ok();
        let mut current = self.bed_mesh.lock_ok();
        let machine = pos.to_machine(current.as_deref());
        let rebase = machine.to_gcode(new.as_deref()).z();
        // `update_mesh` blocks until the lowerer has adopted the new
        // transform behind the pipeline drain; `current` stays locked across
        // the wait so no bridge crossing can invert through a mesh the
        // pipeline is not warping with yet.
        if let Some(handle) = self.planner.lock_ok().as_ref() {
            handle
                .update_mesh(new.clone(), rebase)
                .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;
        }
        *current = new;
        drop(current);
        self.commanded_pos.lock_ok().0[2] = rebase;
        Ok(rebase)
    }

    fn reseed_mcus_after_position_set(
        &self,
        py: Python<'_>,
        gcode: geometry::GcodePos,
        machine: geometry::MachinePos,
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
                .stream_open(vec![gcode.x(), gcode.y(), gcode.z(), 0.0])
                .map_err(planner_err)?;

            self.send_serial_position_seeds(machine)?;
        }
        Ok(())
    }

    fn rebase_motion_history_after_position_set(
        &self,
        host_now: f64,
        machine: geometry::MachinePos,
    ) {
        let configs: Vec<crate::mcu_config::McuAxisConfig> =
            self.mcu_axis_configs.lock_ok().clone();
        let rebases = crate::mcu_config::spatial_rebase_targets(&configs, machine);
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
    /// The machine→gcode crossing: the only place a measured (machine-space)
    /// position becomes a gcode-space one.
    pub(crate) fn gcode_from_machine(&self, pos: geometry::MachinePos) -> geometry::GcodePos {
        pos.to_gcode(self.bed_mesh.lock_ok().as_deref())
    }

    /// The gcode→machine crossing: the only place a commanded (gcode-space)
    /// position becomes the physical one that step counters and motion
    /// history are seeded with.
    pub(crate) fn machine_from_gcode(&self, pos: geometry::GcodePos) -> geometry::MachinePos {
        pos.to_machine(self.bed_mesh.lock_ok().as_deref())
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
