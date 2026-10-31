use pyo3::prelude::*;

use snapshot_core::{SampleSide, Snapshot, SnapshotParams};

use planner_config::from_doc::read_motion_settings;

fn snapshot_from_config(
    waypoints: &[snapshot_core::waypoints::Waypoint],
    config_text: &str,
) -> PyResult<Snapshot> {
    let doc = config_doc::Document::parse(config_text, "<config>")
        .map_err(|e| pyo3::exceptions::PyValueError::new_err(e.to_string()))?;
    let (settings, _consumed) =
        read_motion_settings(&doc).map_err(pyo3::exceptions::PyValueError::new_err)?;
    snapshot_core::pipeline_snapshot(
        waypoints,
        SnapshotParams {
            max_velocity: settings.cartesian.max_velocity,
            max_accel: settings.cartesian.max_accel,
            corner_deviation: settings.cartesian.corner_deviation,
            max_jerk: settings.cartesian.max_jerk,
            max_extrude_only_velocity: settings.max_extrude_only_velocity,
            max_extrude_only_accel: settings.max_extrude_only_accel,
            max_path_deviation: Some(settings.fit_tolerance_mm),
            max_accel_deviation: Some(settings.fit_tolerance_accel_mm_s2),
            axis_decls: settings.axes,
            post_processor_decls: settings.post_processors,
        },
    )
    .map_err(|e| pyo3::exceptions::PyValueError::new_err(e.to_string()))
}

/// Snapshot the pipeline for `waypoints` under the motion config parsed
/// from `config_text` — the same section reader (defaults, bounds) the live
/// printer uses — and return the snapshot as a JSON string for `json.loads`.
#[pyfunction]
pub(crate) fn pipeline_snapshot(
    waypoints: Vec<snapshot_core::waypoints::Waypoint>,
    config_text: &str,
) -> PyResult<String> {
    let snap = snapshot_from_config(&waypoints, config_text)?;
    serde_json::to_string(&snap).map_err(|e| pyo3::exceptions::PyValueError::new_err(e.to_string()))
}

/// G-code text → absolute `(x, y, z, e, feedrate, accel)` waypoints.
#[pyfunction]
pub(crate) fn parse_gcode(
    text: &str,
    max_velocity: f64,
    max_accel: f64,
) -> PyResult<Vec<snapshot_core::waypoints::Waypoint>> {
    snapshot_core::waypoints::parse_gcode(text, max_velocity, max_accel)
        .map_err(|e| pyo3::exceptions::PyValueError::new_err(e.to_string()))
}

/// Plan `waypoints` under `config_text` and evaluate the exact carriers of
/// each axis in `axes` at `samples` times spread over the trajectory. Returns
/// the snapshot JSON alongside per-axis `(position, velocity, acceleration)`
/// rows so a plotting consumer never re-derives the carriers itself.
#[pyfunction]
pub(crate) fn pipeline_snapshot_axis_samples(
    waypoints: Vec<snapshot_core::waypoints::Waypoint>,
    config_text: &str,
    axes: Vec<usize>,
    samples: usize,
) -> PyResult<(String, Vec<Vec<(f64, f64, f64)>>)> {
    if samples < 2 {
        return Err(pyo3::exceptions::PyValueError::new_err(
            "samples must be at least 2",
        ));
    }
    let snap = snapshot_from_config(&waypoints, config_text)?;
    let t_end = snap.trajectory.t_end();
    let mut out = Vec::with_capacity(axes.len());
    for axis in axes {
        let mut rows = Vec::with_capacity(samples);
        for i in 0..samples {
            let t = t_end * i as f64 / (samples - 1) as f64;
            let pvaj = snap
                .trajectory
                .eval_axis(axis, t, SampleSide::Right)
                .map_err(|e| pyo3::exceptions::PyValueError::new_err(e.to_string()))?;
            rows.push((pvaj.position, pvaj.velocity, pvaj.acceleration));
        }
        out.push(rows);
    }
    let json = serde_json::to_string(&snap)
        .map_err(|e| pyo3::exceptions::PyValueError::new_err(e.to_string()))?;
    Ok((json, out))
}
