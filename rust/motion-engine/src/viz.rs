use geometry::path::CurvatureProfile;
use geometry::path::lowering::PositionProfile;
use pyo3::prelude::*;
use pyo3::types::PyDict;

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
    let fitted_points = sample_fitted_path(&outcome);

    let profile = geometry::plan_velocity(&outcome, geometry::VelocityConfig::default())
        .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(format!("{e:?}")))?;
    let velocity_samples = extract_velocity_profile(&profile);

    let dict = PyDict::new(py);
    dict.set_item("raw_x", raw_points.iter().map(|p| p.0).collect::<Vec<_>>())?;
    dict.set_item("raw_y", raw_points.iter().map(|p| p.1).collect::<Vec<_>>())?;
    dict.set_item(
        "fitted_x",
        fitted_points.iter().map(|p| p.0).collect::<Vec<_>>(),
    )?;
    dict.set_item(
        "fitted_y",
        fitted_points.iter().map(|p| p.1).collect::<Vec<_>>(),
    )?;
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

const SAMPLES_PER_MM: f64 = 2.0;

fn sample_fitted_path(outcome: &geometry::FitOutcome) -> Vec<(f64, f64)> {
    let mut points = Vec::new();
    for m in &outcome.moves {
        if let Some(spatial) = &m.segment.spatial {
            let len = spatial.s_len();
            let n = ((len * SAMPLES_PER_MM).ceil() as usize).max(2);
            for k in 0..n {
                let s = len * (k as f64) / ((n - 1) as f64);
                let pt = spatial.point_at(s);
                points.push((pt[0], pt[1]));
            }
        }
    }
    points
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
