use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use snapshot_core::waypoints::Waypoint;
use snapshot_core::{SampleSide, Snapshot, SnapshotParams};

use planner_config::RuntimeLimits;
use planner_config::from_doc::{MotionSettings, read_motion_settings};

/// The runtime limit overrides in force for a waypoint's move, as Python
/// sees them: `None` is "no override, the config base applies".
#[derive(IntoPyObject, FromPyObject)]
#[pyo3(from_item_all)]
pub(crate) struct PyRuntimeLimits {
    velocity: Option<f64>,
    accel: Option<f64>,
    corner_deviation: Option<f64>,
}

/// A waypoint as a Python dict: `position` is `[x, y, z, e]`;
/// `dwell_before_s` is the stop at rest the move to it starts from, `None`
/// when it blends from the previous move.
#[derive(IntoPyObject, FromPyObject)]
#[pyo3(from_item_all)]
pub(crate) struct PyWaypoint {
    position: [f64; 4],
    feedrate: f64,
    runtime: PyRuntimeLimits,
    dwell_before_s: Option<f64>,
}

impl From<Waypoint> for PyWaypoint {
    fn from(w: Waypoint) -> Self {
        Self {
            position: w.position,
            feedrate: w.feedrate,
            runtime: PyRuntimeLimits {
                velocity: w.runtime.velocity,
                accel: w.runtime.accel,
                corner_deviation: w.runtime.corner_deviation,
            },
            dwell_before_s: w.dwell_before_s,
        }
    }
}

impl From<PyWaypoint> for Waypoint {
    fn from(w: PyWaypoint) -> Self {
        Self {
            position: w.position,
            feedrate: w.feedrate,
            runtime: RuntimeLimits {
                velocity: w.runtime.velocity,
                accel: w.runtime.accel,
                corner_deviation: w.runtime.corner_deviation,
            },
            dwell_before_s: w.dwell_before_s,
        }
    }
}

/// One move exactly as the snapshot pipeline consumes it: the live
/// `submit_move` endpoints and feedrate plus the limits it is planned under.
#[derive(IntoPyObject)]
pub(crate) struct PyMoveSpec {
    start: [f64; 4],
    end: [f64; 4],
    feedrate: f64,
    velocity: f64,
    accel: f64,
    corner_deviation: f64,
    dwell_before_s: Option<f64>,
}

fn motion_settings(config_text: &str) -> PyResult<MotionSettings> {
    let doc = config_doc::Document::parse(config_text, "<config>")
        .map_err(|e| PyValueError::new_err(e.to_string()))?;
    let (settings, _consumed) = read_motion_settings(&doc).map_err(PyValueError::new_err)?;
    Ok(settings)
}

fn snapshot_params(settings: MotionSettings) -> SnapshotParams {
    SnapshotParams {
        max_velocity: settings.cartesian.max_velocity,
        max_accel: settings.cartesian.max_accel,
        corner_deviation: settings.cartesian.corner_deviation,
        max_jerk: settings.cartesian.max_jerk,
        max_z_velocity: SnapshotParams::binding_z_cap(
            settings.cartesian.max_z_velocity,
            settings.cartesian.max_velocity,
        ),
        max_z_accel: SnapshotParams::binding_z_cap(
            settings.cartesian.max_z_accel,
            settings.cartesian.max_accel,
        ),
        max_extrude_only_velocity: settings.max_extrude_only_velocity,
        max_extrude_only_accel: settings.max_extrude_only_accel,
        max_path_deviation: Some(settings.fit_tolerance_mm),
        max_accel_deviation: Some(settings.fit_tolerance_accel_mm_s2),
        axis_decls: settings.axes,
        post_processor_decls: settings.post_processors,
    }
}

fn waypoints_from_py(waypoints: Vec<PyWaypoint>) -> Vec<Waypoint> {
    waypoints.into_iter().map(Waypoint::from).collect()
}

fn snapshot_from_config(waypoints: &[Waypoint], config_text: &str) -> PyResult<Snapshot> {
    snapshot_core::pipeline_snapshot(waypoints, snapshot_params(motion_settings(config_text)?))
        .map_err(|e| PyValueError::new_err(e.to_string()))
}

/// Snapshot the pipeline for `waypoints` under the motion config parsed
/// from `config_text` — the same section reader (defaults, bounds) the live
/// printer uses — and return the snapshot as a JSON string for `json.loads`.
#[pyfunction]
pub(crate) fn pipeline_snapshot(waypoints: Vec<PyWaypoint>, config_text: &str) -> PyResult<String> {
    let snap = snapshot_from_config(&waypoints_from_py(waypoints), config_text)?;
    serde_json::to_string(&snap).map_err(|e| PyValueError::new_err(e.to_string()))
}

/// G-code text → waypoints under the motion config parsed from `config_text`.
#[pyfunction]
pub(crate) fn parse_gcode(text: &str, config_text: &str) -> PyResult<Vec<PyWaypoint>> {
    let settings = motion_settings(config_text)?;
    snapshot_core::waypoints::parse_gcode(text, &settings.cartesian)
        .map(|waypoints| waypoints.into_iter().map(PyWaypoint::from).collect())
        .map_err(|e| PyValueError::new_err(e.to_string()))
}

/// The moves `waypoints` feed the snapshot pipeline under `config_text`,
/// with the effective limits each is planned under.
#[pyfunction]
pub(crate) fn snapshot_moves(
    waypoints: Vec<PyWaypoint>,
    config_text: &str,
) -> PyResult<Vec<PyMoveSpec>> {
    let params = snapshot_params(motion_settings(config_text)?);
    let limits = geometry::VelocityLimits::try_new(
        params.max_velocity,
        params.max_accel,
        params.corner_deviation,
        params.max_jerk,
    )
    .and_then(|path| {
        path.with_z_caps(
            params.max_z_velocity.unwrap_or(f64::INFINITY),
            params.max_z_accel.unwrap_or(f64::INFINITY),
        )
    })
    .map_err(|e| PyValueError::new_err(e.to_string()))?;
    let specs = snapshot_core::move_specs(&waypoints_from_py(waypoints), limits)
        .map_err(|e| PyValueError::new_err(e.to_string()))?;
    Ok(specs
        .into_iter()
        .map(|spec| PyMoveSpec {
            start: spec.start,
            end: spec.end,
            feedrate: spec.feedrate,
            velocity: spec.limits.max_velocity_mm_s,
            accel: spec.limits.accel_mm_s2,
            corner_deviation: spec.limits.corner_deviation_mm,
            dwell_before_s: spec.dwell_before_s,
        })
        .collect())
}

/// The live engine's runtime limit state over a config's base limits:
/// the same [`RuntimeLimits`] setters and clamp `MotionEngine` exposes as
/// `set_velocity_cap` / `set_accel_cap` / `set_corner_deviation` /
/// `effective_limits`, without a planner behind it — the motion engine a
/// G-code replay through klippy's own handlers talks to.
#[pyclass]
pub(crate) struct PlannerLimits {
    base: planner_config::CartesianLimits,
    runtime: RuntimeLimits,
}

#[pymethods]
impl PlannerLimits {
    #[new]
    fn new(config_text: &str) -> PyResult<Self> {
        Ok(Self {
            base: motion_settings(config_text)?.cartesian,
            runtime: RuntimeLimits::default(),
        })
    }

    #[pyo3(signature = (velocity))]
    fn set_velocity_cap(&mut self, velocity: Option<f64>) -> PyResult<()> {
        self.runtime
            .set_velocity_cap(velocity)
            .map_err(|e| PyValueError::new_err(e.to_string()))
    }

    #[pyo3(signature = (accel))]
    fn set_accel_cap(&mut self, accel: Option<f64>) -> PyResult<()> {
        self.runtime
            .set_accel_cap(accel)
            .map_err(|e| PyValueError::new_err(e.to_string()))
    }

    #[pyo3(signature = (corner_deviation))]
    fn set_corner_deviation(&mut self, corner_deviation: Option<f64>) -> PyResult<()> {
        self.runtime
            .set_corner_deviation(corner_deviation)
            .map_err(|e| PyValueError::new_err(e.to_string()))
    }

    fn effective_limits(&self) -> (f64, f64, f64) {
        self.runtime.effective(&self.base)
    }
}

/// Plan `waypoints` under `config_text` and evaluate the exact carriers of
/// each axis in `axes` at `samples` times spread over the trajectory. Returns
/// the snapshot JSON alongside per-axis `(position, velocity, acceleration)`
/// rows so a plotting consumer never re-derives the carriers itself.
#[pyfunction]
pub(crate) fn pipeline_snapshot_axis_samples(
    waypoints: Vec<PyWaypoint>,
    config_text: &str,
    axes: Vec<usize>,
    samples: usize,
) -> PyResult<(String, Vec<Vec<(f64, f64, f64)>>)> {
    if samples < 2 {
        return Err(PyValueError::new_err("samples must be at least 2"));
    }
    let snap = snapshot_from_config(&waypoints_from_py(waypoints), config_text)?;
    let t_end = snap.trajectory.t_end();
    let mut out = Vec::with_capacity(axes.len());
    for axis in axes {
        let mut rows = Vec::with_capacity(samples);
        for i in 0..samples {
            let t = t_end * i as f64 / (samples - 1) as f64;
            let pvaj = snap
                .trajectory
                .eval_axis(axis, t, SampleSide::Right)
                .map_err(|e| PyValueError::new_err(e.to_string()))?;
            rows.push((pvaj.position, pvaj.velocity, pvaj.acceleration));
        }
        out.push(rows);
    }
    let json = serde_json::to_string(&snap).map_err(|e| PyValueError::new_err(e.to_string()))?;
    Ok((json, out))
}
