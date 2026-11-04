use super::{
    Arc, ArgValue, DRAIN_TIMEOUT, HashSet, McuTopologyInput, Ordering, PyMotionEngine, PyResult,
    PyRuntimeError, PyValueError, Python, planner_err, pymethods,
};
use motion_core::lock_ext::LockExt;
use pyo3::FromPyObject;

fn unsupported_curve(py: Python<'_>, message: &'static str) -> PyResult<()> {
    py.detach(|| Err(PyRuntimeError::new_err(message)))
}

pub(super) fn require_single_motor_mask(motor_mask: u8) -> Result<(), String> {
    if motor_mask.count_ones() > 1 {
        return Err(format!(
            "submit_nudge: multi-bit motor_mask {motor_mask:#010b} not supported"
        ));
    }
    Ok(())
}

impl PyMotionEngine {
    fn adopt_planner_config(&self, config_text: &str) -> PyResult<planner_config::PlannerConfig> {
        if self.planner.lock_ok().is_some() {
            return Err(PyRuntimeError::new_err("planner already initialized"));
        }
        let cfg = planner_config::from_doc::planner_config_from_text(config_text)
            .map_err(PyValueError::new_err)?;
        *self.extruder_caps.lock_ok() =
            motion_pipeline::live_config::extruder_caps(&cfg).map_err(PyValueError::new_err)?;
        *self.planner_config.lock_ok() = cfg.clone();
        Ok(cfg)
    }

    #[cfg(feature = "snapshot")]
    /// A fence of the detached planner, resolved to the stream time its
    /// fenced trajectory ends at (0 for an idle stream): with no MCU clock to
    /// project it through, the detached planner's print time is its stream
    /// time.
    pub(crate) fn detached_fence_end(&self, id: u64) -> PyResult<Option<f64>> {
        if self.detached_record.lock_ok().is_none() {
            return Err(PyRuntimeError::new_err(
                "no detached planner: call init_planner_detached",
            ));
        }
        let guard = self.planner.lock_ok();
        let planner = guard.as_ref().ok_or_else(|| {
            PyRuntimeError::new_err("planner not initialized — call init_planner_detached first")
        })?;
        Ok(planner.fence_take(id).map(|end| end.unwrap_or(0.0)))
    }

    #[cfg(feature = "snapshot")]
    /// What the detached planner was fed and shaped since it started; call
    /// after a drain so the shaper has emitted everything.
    pub(crate) fn take_detached_record(
        &self,
    ) -> PyResult<(
        Vec<motion_pipeline::InputTrace>,
        Vec<trajectory::ContinuousSegment>,
    )> {
        let guard = self.detached_record.lock_ok();
        let record = guard.as_ref().ok_or_else(|| {
            PyRuntimeError::new_err("no detached planner: call init_planner_detached")
        })?;
        Ok((
            record.inputs.try_iter().collect(),
            record.segments.try_iter().collect(),
        ))
    }
}
pub(super) fn motion_history_host_now() -> f64 {
    host_rt::clock::instant_to_f64(std::time::Instant::now())
}

#[derive(FromPyObject)]
struct McuTopology {
    mcu_id: u32,
    axes: Vec<u8>,
    kinematics: u8,
    max_motor_velocity: Vec<f64>,
    lane_kinds: Vec<u8>,
    motor_counts: Vec<u8>,
    microstep_distance: Vec<f64>,
    invert_dir: Vec<bool>,
    stepper_oids: Vec<u32>,
    move_queue_slots: u32,
    step_pulse_seconds: Vec<f64>,
    high_precision_step_compress: Vec<bool>,
    stepcompress_max_error_secs: f64,
    stepcompress_drain_budget_secs: f64,
    phase_sample_rate: f64,
    phase_ring_depth: u32,
    mcu_name: String,
    step_capacity_ticks_per_s: Option<f64>,
    ticks_per_step: Vec<u32>,
}

impl McuTopology {
    fn into_core(self) -> McuTopologyInput {
        McuTopologyInput {
            mcu_id: self.mcu_id,
            axes: self.axes,
            lane_kinds: self.lane_kinds,
            high_precision_step_compress: self.high_precision_step_compress,
            hw: motion_core::mcu_config::McuHardware {
                kinematics: self.kinematics,
                max_motor_velocity: self.max_motor_velocity,
                motor_counts: self.motor_counts,
                microstep_distance: self.microstep_distance,
                invert_dir: self.invert_dir,
                stepper_oids: self.stepper_oids,
                move_queue_slots: self.move_queue_slots,
                step_pulse_seconds: self.step_pulse_seconds,
                stepcompress_max_error_secs: self.stepcompress_max_error_secs,
                stepcompress_drain_budget_secs: self.stepcompress_drain_budget_secs,
                phase_sample_rate: self.phase_sample_rate,
                phase_ring_depth: self.phase_ring_depth,
                step_capacity: self.step_capacity_ticks_per_s.map(|ticks_per_s| {
                    motion_core::mcu_config::McuStepCapacity {
                        mcu_name: self.mcu_name,
                        ticks_per_s,
                        ticks_per_step: self.ticks_per_step,
                    }
                }),
            },
        }
    }
}

/// One bed mesh as klippy hands it over: the probed grid, the datum the mesh
/// is zeroed at, the fade band, and the Z envelope the mesh-following demand
/// is judged against.
#[derive(FromPyObject)]
#[pyo3(from_item_all)]
struct BedMeshRequest {
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
}

/// The one motor a nudge acts on.
#[derive(Debug, Clone, Copy, FromPyObject)]
#[pyo3(from_item_all)]
struct NudgeTarget {
    mcu_id: u32,
    axis_idx: u8,
    motor_mask: u8,
}

/// How far and how fast a nudge runs.
#[derive(Debug, Clone, Copy, FromPyObject)]
#[pyo3(from_item_all)]
struct NudgeMotion {
    delta_mm: f64,
    speed: f64,
    accel: f64,
}

/// A G5 cubic's control handles: the incoming pair (absent on a chained
/// segment, which reflects the previous one) and the outgoing pair.
#[derive(Debug, Clone, Copy, FromPyObject)]
#[pyo3(from_item_all)]
struct CurveHandles {
    i: Option<f64>,
    j: Option<f64>,
    p: f64,
    q: f64,
}

/// One curve segment's endpoint displacement, spatial plus extrusion.
#[derive(Debug, Clone, Copy, FromPyObject)]
#[pyo3(from_item_all)]
struct CurveDisplacement {
    dx: f64,
    dy: f64,
    dz: f64,
    de: f64,
}

#[pymethods]
impl PyMotionEngine {
    #[pyo3(signature = (target, motion))]
    fn submit_nudge(
        &self,
        py: Python<'_>,
        target: NudgeTarget,
        motion: NudgeMotion,
    ) -> PyResult<f64> {
        require_single_motor_mask(target.motor_mask).map_err(PyRuntimeError::new_err)?;
        let profile =
            trajectory::NudgeProfile::try_new(motion.delta_mm, motion.speed, motion.accel, 0.0)
                .map_err(|e| PyRuntimeError::new_err(format!("submit_nudge: {e}")))?;
        let rx = {
            let guard = self.planner.lock_ok();
            let planner = guard.as_ref().ok_or_else(|| {
                PyRuntimeError::new_err("planner not initialized — call init_planner first")
            })?;
            planner
                .submit_nudge(motion_core::worker::NudgeParams {
                    mcu_id: target.mcu_id,
                    axis: target.axis_idx,
                    motor_mask: target.motor_mask,
                    delta_mm: motion.delta_mm,
                    speed: motion.speed,
                    accel: motion.accel,
                })
                .map_err(|e| PyRuntimeError::new_err(e.to_string()))?
        };
        py.detach(|| rx.recv())
            .map_err(|_| PyRuntimeError::new_err("nudge notify dropped"))?
            .map_err(PyRuntimeError::new_err)?;
        Ok(profile.duration())
    }
    /// `config_text` is the serialized config document; the motion-owned
    /// sections are re-read here with the same reader
    /// (`_config_doc.read_motion_settings`) klippy used at config time, so
    /// the planner cannot drift from what the host validated and reported.
    fn init_planner(&self, config_text: &str, mcus: Vec<McuTopology>) -> PyResult<()> {
        let cfg = self.adopt_planner_config(config_text)?;

        let mcus: Vec<McuTopologyInput> = mcus.into_iter().map(McuTopology::into_core).collect();
        let (ec_conns, mcu_configs) = self.resolve_mcu_topology(&mcus)?;
        let machine = self.machine_from_gcode(*self.commanded_pos.lock_ok());
        self.rebase_motion_history_after_position_set(machine);

        let (ethercat_mcu_ids, host_ios) = self.build_transport_maps(&mcu_configs)?;
        self.seed_ethercat_clock_estimates(&ethercat_mcu_ids)?;

        host_rt::memory_lock::HOST_MEMORY_LOCK
            .start_pipeline_threads(|| {
                let pump_control =
                    self.spawn_pipeline(&cfg, &mcu_configs, &host_ios, &ec_conns, None)?;

                self.spawn_live_position_poll_thread();

                self.wire_mcu_supervision(&mcu_configs, &ethercat_mcu_ids, &ec_conns, pump_control);

                Ok(())
            })
            .map_err(|denied| PyRuntimeError::new_err(denied.to_string()))?
    }
    #[cfg(feature = "snapshot")]
    /// The live planner with no MCU behind it: the same config reading, move
    /// construction, ingress and pipeline as `init_planner`, but nothing
    /// executes the trajectory, so the pacer never brakes for a playhead and
    /// no memory needs locking. It records every pipeline input and shaped
    /// segment for `take_detached_record`: the live half of the snapshot
    /// parity gate.
    fn init_planner_detached(&self, config_text: &str) -> PyResult<()> {
        let cfg = self.adopt_planner_config(config_text)?;
        let (inputs_tx, inputs) = crossbeam_channel::unbounded();
        let (segments_tx, segments) = crossbeam_channel::unbounded();
        *self.detached_record.lock_ok() = Some(super::DetachedRecordRx { inputs, segments });
        self.spawn_pipeline(
            &cfg,
            &[],
            &std::collections::HashMap::new(),
            &std::collections::HashMap::new(),
            Some(motion_core::worker::DetachedTaps {
                inputs: inputs_tx,
                segments: segments_tx,
            }),
        )?;
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
            let line = motion_pipeline::live_config::GcodeLine {
                start: self.commanded_pos.lock_ok().0,
                delta: [dx, dy, dz],
                de,
                feedrate_mm_s: feedrate,
                seq: self.move_seq.load(Ordering::Relaxed) as u32,
            };
            let built = {
                let cfg = self.planner_config.lock_ok();
                motion_pipeline::live_config::GcodeMoveBase::of(&cfg, &self.extruder_caps.lock_ok())
                    .and_then(|base| base.line_move(&cfg.runtime, line))
            }
            .map_err(PyRuntimeError::new_err)?;

            if let Some(m) = built {
                let guard = self.planner.lock_ok();
                let planner = guard.as_ref().ok_or_else(|| {
                    PyRuntimeError::new_err("planner not initialized — call init_planner first")
                })?;
                match planner.submit_move(m) {
                    Ok(()) => {}
                    Err(motion_core::worker::StreamWorkerError::ChannelFull) => return Ok(false),
                    Err(e) => return Err(planner_err(e)),
                }
                tracing::trace!(
                    subsystem = "motion",
                    event = "intake_submit",
                    line_no = line.seq,
                    channel_pending = planner.pending_channel_moves(),
                    "[intake] move accepted into the pipe"
                );
            }

            self.move_seq.fetch_add(1, Ordering::Relaxed);
            self.commanded_pos.lock_ok().0 = line.end();
            *self.last_g5_pq.lock_ok() = None;
            Ok(true)
        })
    }

    #[pyo3(signature = (handles, displacement, feedrate))]
    fn submit_bezier(
        &self,
        py: Python<'_>,
        handles: CurveHandles,
        displacement: CurveDisplacement,
        feedrate: f64,
    ) -> PyResult<()> {
        tracing::debug!(
            subsystem = "motion",
            event = "submit_bezier_enter",
            i = ?handles.i,
            j = ?handles.j,
            p = handles.p,
            q = handles.q,
            dx = displacement.dx,
            dy = displacement.dy,
            dz = displacement.dz,
            de = displacement.de,
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
    #[pyo3(signature = (i, j, displacement, feedrate))]
    fn submit_quadratic(
        &self,
        py: Python<'_>,
        i: f64,
        j: f64,
        displacement: CurveDisplacement,
        feedrate: f64,
    ) -> PyResult<()> {
        tracing::debug!(
            subsystem = "motion",
            event = "submit_quadratic_enter",
            i,
            j,
            dx = displacement.dx,
            dy = displacement.dy,
            dz = displacement.dz,
            de = displacement.de,
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
    fn dwell_until(&self, print_time: f64, mcu_handle: u32) -> PyResult<()> {
        let host_s = self
            .router
            .lock_ok()
            .print_time_to_host_secs(
                host_rt::passthrough_queue::McuHandle::from_raw(mcu_handle),
                print_time,
            )
            .ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "dwell_until: no clock estimate established for mcu_handle {mcu_handle}"
                ))
            })?;
        let guard = self.planner.lock_ok();
        let planner = guard.as_ref().ok_or_else(|| {
            PyRuntimeError::new_err("planner not initialized — call init_planner first")
        })?;
        planner.hold_start_until(host_s).map_err(planner_err)?;
        *self.last_g5_pq.lock_ok() = None;
        Ok(())
    }
    #[pyo3(signature = (x, y, z))]
    fn set_position(&self, py: Python<'_>, x: f64, y: f64, z: f64) -> PyResult<()> {
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
        self.rebase_motion_history_after_position_set(machine);

        Ok(())
    }

    fn effective_limits(&self) -> (f64, f64, f64) {
        self.planner_config.lock_ok().effective_limits()
    }
    #[pyo3(signature = (velocity))]
    fn set_velocity_cap(&self, velocity: Option<f64>) -> PyResult<()> {
        self.planner_config
            .lock_ok()
            .runtime
            .set_velocity_cap(velocity)
            .map_err(|e| PyValueError::new_err(e.to_string()))
    }
    #[pyo3(signature = (accel))]
    fn set_accel_cap(&self, accel: Option<f64>) -> PyResult<()> {
        self.planner_config
            .lock_ok()
            .runtime
            .set_accel_cap(accel)
            .map_err(|e| PyValueError::new_err(e.to_string()))
    }
    pub(super) fn set_post_processor_bypass(&self, enabled: bool) -> PyResult<()> {
        self.publish_post_processors(|_, bypass| {
            *bypass = enabled;
            Ok(())
        })
    }
    #[pyo3(signature = (corner_deviation))]
    fn set_corner_deviation(&self, corner_deviation: Option<f64>) -> PyResult<()> {
        self.planner_config
            .lock_ok()
            .runtime
            .set_corner_deviation(corner_deviation)
            .map_err(|e| PyValueError::new_err(e.to_string()))
    }
    pub(super) fn update_post_processor(&self, name: &str, key: &str, value: f64) -> PyResult<()> {
        self.publish_post_processors(|processors, _| processors.set_param(name, key, value))
    }

    fn post_processor_param(&self, name: &str, key: &str) -> Option<f64> {
        self.planner_config
            .lock_ok()
            .post_processors
            .param(name, key)
    }

    fn set_bed_mesh(&self, request: BedMeshRequest) -> PyResult<(f64, (f64, f64, f64, f64, f64))> {
        let BedMeshRequest {
            points,
            x_min,
            y_min,
            dx,
            dy,
            nx,
            ny,
            tension,
            fade,
            zero_ref_x,
            zero_ref_y,
            z_velocity_limit,
            z_accel_limit,
        } = request;
        let mut mesh = geometry::MeshGrid::new(
            geometry::MeshGridSpec {
                x_min,
                y_min,
                dx,
                dy,
                nx,
                ny,
                tension,
            },
            points,
        )
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
    fn publish_post_processors(
        &self,
        update: impl FnOnce(
            &mut planner_config::PostProcessorSet,
            &mut bool,
        ) -> Result<(), planner_config::PostProcessorConfigError>,
    ) -> PyResult<()> {
        let mut cfg = self.planner_config.lock_ok();
        let mut processors = cfg.post_processors.clone();
        let mut bypass = cfg.post_processor_bypass;
        update(&mut processors, &mut bypass).map_err(|e| PyValueError::new_err(e.to_string()))?;
        let axis_chains = processors
            .compile_active_chains(&cfg.axis_registry, bypass)
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        let extruder_caps = planner_config::extruder_caps(&cfg.axis_registry, &axis_chains)
            .map_err(PyValueError::new_err)?;
        if let Some(handle) = self.planner.lock_ok().as_ref() {
            handle
                .update_axis_chains(axis_chains)
                .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;
        }
        cfg.post_processors = processors;
        cfg.post_processor_bypass = bypass;
        *self.extruder_caps.lock_ok() = extruder_caps;
        Ok(())
    }

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
            motion_core::mcu_config::build_serial_seed_sends(&configs, &ethercat_mcu_ids, pos)
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
            io.send_args(
                "runtime_seed_position",
                &[
                    ("x_q16", ArgValue::Int(i64::from(s.x_q16))),
                    ("y_q16", ArgValue::Int(i64::from(s.y_q16))),
                    ("z_q16", ArgValue::Int(i64::from(s.z_q16))),
                ],
            )
            .map_err(|e| {
                PyRuntimeError::new_err(format!(
                    "position seed send to mcu_id {} failed: {e:?}",
                    s.mcu_id
                ))
            })?;
        }
        drop(mcus);
        let configs = self.mcu_axis_configs.lock_ok().clone();
        self.endpoint_command(motion_core::pump::EndpointCommand::SeedPosition {
            configs,
            position: pos,
        })
        .map_err(PyRuntimeError::new_err)
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
        let flushed = py.detach(|| {
            let planner_guard = self.planner.lock_ok();
            match planner_guard.as_ref() {
                Some(planner) => planner.flush().map(|()| true),
                None => Ok(false),
            }
        });
        if !flushed.map_err(planner_err)? {
            return Ok(());
        }
        {
            let drain = self.drain.clone();
            py.detach(|| drain.wait_drained(DRAIN_TIMEOUT))
                .map_err(PyRuntimeError::new_err)?;
        }
        let planner_guard = self.planner.lock_ok();
        let planner = planner_guard
            .as_ref()
            .ok_or_else(|| PyRuntimeError::new_err("planner disappeared during position reseed"))?;
        planner
            .reset(motion_core::mcu_config::reanchor_stream_pos(gcode))
            .map_err(planner_err)?;
        self.send_serial_position_seeds(machine)?;
        Ok(())
    }

    fn rebase_motion_history_after_position_set(&self, machine: geometry::MachinePos) {
        let host_now = motion_history_host_now();
        let configs: Vec<motion_core::mcu_config::McuAxisConfig> =
            self.mcu_axis_configs.lock_ok().clone();
        let mut store = self.motion_history.lock_ok();
        for (key, pos) in motion_core::mcu_config::reanchor_axis_targets(&configs, machine) {
            store.rebase_axis(key, host_now, pos);
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
}
