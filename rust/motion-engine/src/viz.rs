use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList};

use snapshot_core::{FittedSegment, SnapshotParams};

use crate::bridge::{AxisSection, PostProcessor};

#[pyfunction]
#[allow(clippy::too_many_arguments)]
#[pyo3(signature = (waypoints, max_velocity, max_accel, square_corner_velocity, max_jerk, max_extrude_only_velocity = None, max_extrude_only_accel = None, max_path_deviation = None, max_accel_deviation = None, axes = Vec::new(), post_processors = Vec::new()))]
pub(crate) fn pipeline_snapshot(
    py: Python<'_>,
    waypoints: Vec<(f64, f64, f64, f64, f64)>,
    max_velocity: f64,
    max_accel: f64,
    square_corner_velocity: f64,
    max_jerk: f64,
    max_extrude_only_velocity: Option<f64>,
    max_extrude_only_accel: Option<f64>,
    max_path_deviation: Option<f64>,
    max_accel_deviation: Option<f64>,
    axes: Vec<AxisSection>,
    post_processors: Vec<PostProcessor>,
) -> PyResult<Py<PyDict>> {
    let snap = snapshot_core::pipeline_snapshot(
        &waypoints,
        SnapshotParams {
            max_velocity,
            max_accel,
            square_corner_velocity,
            max_jerk,
            max_extrude_only_velocity,
            max_extrude_only_accel,
            max_path_deviation,
            max_accel_deviation,
            axis_decls: axes.into_iter().map(Into::into).collect(),
            post_processor_decls: post_processors.into_iter().map(Into::into).collect(),
        },
    )
    .map_err(|e| pyo3::exceptions::PyValueError::new_err(e.to_string()))?;

    let dict = PyDict::new(py);
    dict.set_item("raw_x", snap.raw_x)?;
    dict.set_item("raw_y", snap.raw_y)?;

    let seg_list = PyList::empty(py);
    for seg in &snap.fitted_segments {
        seg_list.append(segment_to_pydict(py, seg)?)?;
    }
    dict.set_item("fitted_segments", seg_list)?;

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

fn segment_to_pydict<'py>(py: Python<'py>, seg: &FittedSegment) -> PyResult<Bound<'py, PyDict>> {
    let d = PyDict::new(py);
    match seg {
        FittedSegment::Line { x0, y0, x1, y1 } => {
            d.set_item("type", "line")?;
            d.set_item("x0", x0)?;
            d.set_item("y0", y0)?;
            d.set_item("x1", x1)?;
            d.set_item("y1", y1)?;
        }
        FittedSegment::Arc { x, y } => {
            d.set_item("type", "arc")?;
            d.set_item("x", x.clone())?;
            d.set_item("y", y.clone())?;
        }
        FittedSegment::Clothoid { x, y } => {
            d.set_item("type", "clothoid")?;
            d.set_item("x", x.clone())?;
            d.set_item("y", y.clone())?;
        }
    }
    Ok(d)
}
