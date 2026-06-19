use geometry::path::CurvatureProfile;
use geometry::path::lowering::PositionProfile;
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList};

#[pyfunction]
#[pyo3(signature = (waypoints, max_velocity, max_accel, square_corner_velocity))]
pub fn pipeline_snapshot(
    py: Python<'_>,
    waypoints: Vec<(f64, f64, f64, f64)>,
    max_velocity: f64,
    max_accel: f64,
    square_corner_velocity: f64,
) -> PyResult<Py<PyDict>> {
    if waypoints.len() < 2 {
        return Err(pyo3::exceptions::PyValueError::new_err(
            "need at least 2 waypoints",
        ));
    }

    let limits = geometry::VelocityLimits::try_new(max_velocity, max_accel, square_corner_velocity)
        .map_err(pyo3::exceptions::PyValueError::new_err)?;

    let moves = build_moves(&waypoints, limits)?;
    let raw_points = extract_raw_path(&moves);

    let outcome = geometry::fit_chain(&moves, geometry::ChainFitConfig::default())
        .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(format!("{e:?}")))?;
    let fitted_segments = sample_fitted_segments(&outcome);

    let profile = geometry::plan_velocity(&outcome, geometry::VelocityConfig::default())
        .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(format!("{e:?}")))?;
    let velocity_samples = extract_velocity_profile(&profile);

    let dict = PyDict::new(py);
    dict.set_item("raw_x", raw_points.iter().map(|p| p.0).collect::<Vec<_>>())?;
    dict.set_item("raw_y", raw_points.iter().map(|p| p.1).collect::<Vec<_>>())?;

    let seg_list = PyList::empty(py);
    for seg in &fitted_segments {
        let d = PyDict::new(py);
        d.set_item("type", seg.kind)?;
        d.set_item("x", &seg.xs)?;
        d.set_item("y", &seg.ys)?;
        d.set_item("s", &seg.s_values)?;
        seg_list.append(d)?;
    }
    dict.set_item("fitted_segments", seg_list)?;

    dict.set_item(
        "vel_s",
        velocity_samples.iter().map(|p| p.0).collect::<Vec<_>>(),
    )?;
    dict.set_item(
        "vel_v",
        velocity_samples.iter().map(|p| p.1).collect::<Vec<_>>(),
    )?;
    dict.set_item("blended_corners", outcome.report.blended)?;
    dict.set_item("unblended_corners", outcome.report.unblended.len())?;
    dict.set_item("chain_fits", outcome.report.chains)?;
    dict.set_item("traversal_time_s", profile.report.traversal_time_s)?;
    Ok(dict.into())
}

fn build_moves(
    waypoints: &[(f64, f64, f64, f64)],
    limits: geometry::VelocityLimits,
) -> PyResult<Vec<geometry::Move>> {
    let mut moves = Vec::with_capacity(waypoints.len() - 1);
    for (i, pair) in waypoints.windows(2).enumerate() {
        let (x0, y0, z0, _) = pair[0];
        let (x1, y1, z1, feedrate) = pair[1];
        let start = [x0, y0, z0];
        let end = [x1, y1, z1];
        let ctx = geometry::MoveContext {
            extruder_axis: 0,
            feedrate_mm_s: feedrate,
            limits,
            source: geometry::SourceRange {
                start_line: i as u32,
                end_line: i as u32,
            },
        };
        match geometry::line_move(start, end, 0.0, ctx) {
            Ok(m) => moves.push(m),
            Err(geometry::FrontendError::ZeroMotion { .. }) => {}
            Err(e) => {
                return Err(pyo3::exceptions::PyValueError::new_err(format!(
                    "move {i}: {e:?}"
                )));
            }
        }
    }
    if moves.is_empty() {
        return Err(pyo3::exceptions::PyValueError::new_err(
            "no spatial moves after filtering zero-displacement pairs",
        ));
    }
    Ok(moves)
}

fn extract_raw_path(moves: &[geometry::Move]) -> Vec<(f64, f64)> {
    let mut points = Vec::with_capacity(moves.len() + 1);
    for (i, m) in moves.iter().enumerate() {
        if let Some(spatial) = &m.segment.spatial {
            let start = spatial.point_at(0.0);
            if i == 0 {
                points.push((start[0], start[1]));
            }
            let end = spatial.point_at(spatial.s_len());
            points.push((end[0], end[1]));
        }
    }
    points
}

struct TypedSegment {
    kind: &'static str,
    xs: Vec<f64>,
    ys: Vec<f64>,
    s_values: Vec<f64>,
}

const SAMPLES_PER_MM: f64 = 2.0;

fn sample_fitted_segments(outcome: &geometry::FitOutcome) -> Vec<TypedSegment> {
    let mut segments = Vec::new();
    let mut s_offset = 0.0;
    for m in &outcome.moves {
        if let Some(spatial) = &m.segment.spatial {
            let kind = match spatial {
                geometry::path::Segment::Line(_) => "line",
                geometry::path::Segment::Arc(_) => "arc",
                geometry::path::Segment::Clothoid(_) => "clothoid",
            };
            let len = spatial.s_len();
            let n = ((len * SAMPLES_PER_MM).ceil() as usize).max(2);
            let mut xs = Vec::with_capacity(n);
            let mut ys = Vec::with_capacity(n);
            let mut s_values = Vec::with_capacity(n);
            for k in 0..n {
                let s = len * (k as f64) / ((n - 1) as f64);
                let pt = spatial.point_at(s);
                xs.push(pt[0]);
                ys.push(pt[1]);
                s_values.push(s_offset + s);
            }
            s_offset += len;
            segments.push(TypedSegment {
                kind,
                xs,
                ys,
                s_values,
            });
        }
    }
    segments
}

fn extract_velocity_profile(profile: &geometry::VelocityProfile) -> Vec<(f64, f64)> {
    let mut samples = Vec::new();
    let mut s_offset = 0.0;
    for mv in &profile.moves {
        for sample in &mv.samples {
            samples.push((s_offset + sample.s, sample.v));
        }
        s_offset += mv.length;
    }
    samples
}

#[cfg(test)]
mod tests;
