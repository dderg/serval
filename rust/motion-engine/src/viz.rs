use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use snapshot_core::waypoints::Waypoint;
use snapshot_core::{SampleSide, Snapshot};

use planner_config::RuntimeLimits;
use planner_config::from_doc::{MotionSettings, read_motion_settings};

use crate::step_benchmarks::{PyStepBenchmark, optional_step_capacity_input};

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

pub(crate) fn waypoints_from_py(waypoints: Vec<PyWaypoint>) -> Vec<Waypoint> {
    waypoints.into_iter().map(Waypoint::from).collect()
}

fn planner(config_text: &str) -> PyResult<snapshot_core::PlannerConfig> {
    planner_config::from_doc::planner_config_from_text(config_text).map_err(PyValueError::new_err)
}

fn snapshot_from_config(
    waypoints: &[Waypoint],
    config_text: &str,
    step_benchmarks: Option<&[PyStepBenchmark]>,
) -> PyResult<Snapshot> {
    let step_capacity = optional_step_capacity_input(config_text, step_benchmarks)?;
    snapshot_core::pipeline_snapshot_for_config(
        waypoints,
        &planner(config_text)?,
        step_capacity.as_ref(),
    )
    .map_err(|e| PyValueError::new_err(e.to_string()))
}

/// Snapshot the pipeline for `waypoints` under the motion config parsed
/// from `config_text` — the same section reader (defaults, bounds) the live
/// printer uses — and return the snapshot as a JSON string for `json.loads`.
/// `step_benchmarks` caps every move as the live printer whose mcus
/// reported them does; `None` plans uncapped.
#[pyfunction]
#[pyo3(signature = (waypoints, config_text, step_benchmarks = None))]
pub(crate) fn pipeline_snapshot(
    waypoints: Vec<PyWaypoint>,
    config_text: &str,
    step_benchmarks: Option<Vec<PyStepBenchmark>>,
) -> PyResult<String> {
    let snap = snapshot_from_config(
        &waypoints_from_py(waypoints),
        config_text,
        step_benchmarks.as_deref(),
    )?;
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
    let planner = planner(config_text)?;
    let base = motion_pipeline::live_config::GcodeMoveBase::of(
        &planner,
        &motion_pipeline::live_config::extruder_caps(&planner).map_err(PyValueError::new_err)?,
    )
    .map_err(PyValueError::new_err)?;
    let specs = snapshot_core::move_specs(&waypoints_from_py(waypoints), base)
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
/// `step_benchmarks` caps it as in [`pipeline_snapshot`].
#[pyfunction]
#[pyo3(signature = (waypoints, config_text, axes, samples, step_benchmarks = None))]
pub(crate) fn pipeline_snapshot_axis_samples(
    waypoints: Vec<PyWaypoint>,
    config_text: &str,
    axes: Vec<usize>,
    samples: usize,
    step_benchmarks: Option<Vec<PyStepBenchmark>>,
) -> PyResult<(String, Vec<Vec<(f64, f64, f64)>>)> {
    if samples < 2 {
        return Err(PyValueError::new_err("samples must be at least 2"));
    }
    let snap = snapshot_from_config(
        &waypoints_from_py(waypoints),
        config_text,
        step_benchmarks.as_deref(),
    )?;
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
