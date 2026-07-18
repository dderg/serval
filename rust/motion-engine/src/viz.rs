use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList};

use snapshot_core::SnapshotParams;

use crate::config::from_doc::read_motion_settings;

/// Snapshot the pipeline for `waypoints` under the motion config parsed
/// from `config_text` — the same section reader (defaults, bounds,
/// scv→corner_deviation conversion) the live printer uses.
#[pyfunction]
pub(crate) fn pipeline_snapshot(
    py: Python<'_>,
    waypoints: Vec<(f64, f64, f64, f64, f64, f64)>,
    config_text: &str,
) -> PyResult<Py<PyDict>> {
    let doc = config_doc::Document::parse(config_text, "<config>")
        .map_err(|e| pyo3::exceptions::PyValueError::new_err(e.to_string()))?;
    let (settings, _consumed) =
        read_motion_settings(&doc).map_err(pyo3::exceptions::PyValueError::new_err)?;
    let snap = snapshot_core::pipeline_snapshot(
        &waypoints,
        SnapshotParams {
            max_velocity: settings.cartesian.max_velocity,
            max_accel: settings.cartesian.max_accel,
            square_corner_velocity: 0.0,
            corner_deviation: Some(settings.cartesian.corner_deviation),
            max_jerk: settings.cartesian.max_jerk,
            max_extrude_only_velocity: settings.max_extrude_only_velocity,
            max_extrude_only_accel: settings.max_extrude_only_accel,
            max_path_deviation: Some(settings.fit_tolerance_mm),
            max_accel_deviation: Some(settings.fit_tolerance_accel_mm_s2),
            axis_decls: settings.axes,
            post_processor_decls: settings.post_processors,
        },
    )
    .map_err(|e| pyo3::exceptions::PyValueError::new_err(e.to_string()))?;

    let dict = PyDict::new(py);
    dict.set_item("raw_x", snap.raw_x)?;
    dict.set_item("raw_y", snap.raw_y)?;

    dict.set_item("traj_x_pieces", snap.traj_x_pieces)?;
    dict.set_item("traj_y_pieces", snap.traj_y_pieces)?;
    dict.set_item("traj_z_pieces", snap.traj_z_pieces)?;
    dict.set_item("traj_e_pieces", snap.traj_e_pieces)?;
    dict.set_item("traj_t_end", snap.traj_t_end)?;
    dict.set_item("traversal_time_s", snap.traversal_time_s)?;

    dict.set_item("seam_max_dp", snap.seam_max_dp.to_vec())?;
    dict.set_item("seam_max_dv", snap.seam_max_dv.to_vec())?;
    dict.set_item("seam_max_da", snap.seam_max_da.to_vec())?;
    let worst = PyList::empty(py);
    for s in &snap.worst_seams {
        let d = PyDict::new(py);
        d.set_item("t", s.t)?;
        d.set_item("axis", s.axis)?;
        d.set_item("dp", s.dp)?;
        d.set_item("dv", s.dv)?;
        d.set_item("da", s.da)?;
        worst.append(d)?;
    }
    dict.set_item("worst_seams", worst)?;
    Ok(dict.into())
}
