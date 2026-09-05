use pyo3::IntoPyObjectExt;
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList};
use serde_json::Value;

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

    let value = serde_json::to_value(&snap)
        .map_err(|e| pyo3::exceptions::PyValueError::new_err(e.to_string()))?;
    let Value::Object(fields) = value else {
        return Err(pyo3::exceptions::PyValueError::new_err(
            "snapshot must serialize as a JSON object",
        ));
    };
    let dict = PyDict::new(py);
    for (key, field) in &fields {
        dict.set_item(key, json_to_py(py, field)?)?;
    }
    Ok(dict.into())
}

/// The snapshot schema lives in `pipeline-snapshot`; the binding mirrors
/// whatever it serializes, so a schema change never needs a field list here.
fn json_to_py(py: Python<'_>, value: &Value) -> PyResult<Py<PyAny>> {
    Ok(match value {
        Value::Null => py.None(),
        Value::Bool(b) => b.into_py_any(py)?,
        Value::Number(n) => match (n.as_i64(), n.as_u64()) {
            (Some(i), _) => i.into_py_any(py)?,
            (_, Some(u)) => u.into_py_any(py)?,
            _ => n
                .as_f64()
                .ok_or_else(|| {
                    pyo3::exceptions::PyValueError::new_err(format!("unrepresentable number {n}"))
                })?
                .into_py_any(py)?,
        },
        Value::String(s) => s.into_py_any(py)?,
        Value::Array(items) => {
            let list = PyList::empty(py);
            for item in items {
                list.append(json_to_py(py, item)?)?;
            }
            list.into_py_any(py)?
        }
        Value::Object(map) => {
            let dict = PyDict::new(py);
            for (key, field) in map {
                dict.set_item(key, json_to_py(py, field)?)?;
            }
            dict.into_py_any(py)?
        }
    })
}
