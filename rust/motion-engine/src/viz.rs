use geometry::path::CurvatureProfile;
use geometry::path::lowering::PositionProfile;
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList};

#[pyfunction]
#[pyo3(signature = (waypoints, max_velocity, max_accel, square_corner_velocity, max_jerk, arc_fit = None))]
pub fn pipeline_snapshot(
    py: Python<'_>,
    waypoints: Vec<(f64, f64, f64, f64)>,
    max_velocity: f64,
    max_accel: f64,
    square_corner_velocity: f64,
    max_jerk: f64,
    arc_fit: Option<u32>,
) -> PyResult<Py<PyDict>> {
    if waypoints.len() < 2 {
        return Err(pyo3::exceptions::PyValueError::new_err(
            "need at least 2 waypoints",
        ));
    }

    let limits = geometry::VelocityLimits::try_new(max_velocity, max_accel, square_corner_velocity)
        .map_err(pyo3::exceptions::PyValueError::new_err)?;
    let chain_cfg = arc_fit_config(arc_fit)?;

    let moves = build_moves(&waypoints, limits)?;
    let raw_points = extract_raw_path(&moves);

    let outcome = geometry::fit_chain(&moves, chain_cfg)
        .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(format!("{e:?}")))?;

    let velocity_config = geometry::VelocityConfig {
        consistency_tol: VELOCITY_CONSISTENCY_TOL,
        max_jerk_mm_s3: max_jerk,
        integration_tol: VELOCITY_INTEGRATION_TOL,
        ..geometry::VelocityConfig::default()
    };
    let profile = geometry::plan_velocity(&outcome, velocity_config)
        .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(format!("{e:?}")))?;
    let kinematics = sample_kinematics(&outcome, &profile);

    let dict = PyDict::new(py);
    dict.set_item("raw_x", raw_points.iter().map(|p| p.0).collect::<Vec<_>>())?;
    dict.set_item("raw_y", raw_points.iter().map(|p| p.1).collect::<Vec<_>>())?;

    let seg_list = PyList::empty(py);
    for m in &outcome.moves {
        if let Some(spatial) = &m.segment.spatial {
            let d = segment_to_pydict(py, spatial)?;
            seg_list.append(d)?;
        }
    }
    dict.set_item("fitted_segments", seg_list)?;

    dict.set_item("kin_x", &kinematics.x)?;
    dict.set_item("kin_y", &kinematics.y)?;
    dict.set_item("kin_v", &kinematics.v)?;

    dict.set_item("blended_corners", outcome.report.blended)?;
    dict.set_item("unblended_corners", outcome.report.unblended.len())?;
    dict.set_item("chain_fits", outcome.report.chains)?;
    dict.set_item("traversal_time_s", profile.report.traversal_time_s)?;
    Ok(dict.into())
}

fn segment_to_pydict<'py>(
    py: Python<'py>,
    spatial: &geometry::path::Segment,
) -> PyResult<Bound<'py, PyDict>> {
    let d = PyDict::new(py);
    match spatial {
        geometry::path::Segment::Line(line) => {
            d.set_item("type", "line")?;
            d.set_item("x0", line.start[0])?;
            d.set_item("y0", line.start[1])?;
            d.set_item("x1", line.end[0])?;
            d.set_item("y1", line.end[1])?;
        }
        geometry::path::Segment::Arc(_) => {
            d.set_item("type", "arc")?;
            let len = spatial.s_len();
            let n = ((len * SAMPLES_PER_MM).ceil() as usize).max(20);
            let mut xs = Vec::with_capacity(n);
            let mut ys = Vec::with_capacity(n);
            for k in 0..n {
                let s = len * (k as f64) / ((n - 1) as f64);
                let pt = spatial.point_at(s);
                xs.push(pt[0]);
                ys.push(pt[1]);
            }
            d.set_item("x", xs)?;
            d.set_item("y", ys)?;
        }
        geometry::path::Segment::Clothoid(_) => {
            d.set_item("type", "clothoid")?;
            let len = spatial.s_len();
            let n = ((len * SAMPLES_PER_MM).ceil() as usize).max(20);
            let mut xs = Vec::with_capacity(n);
            let mut ys = Vec::with_capacity(n);
            for k in 0..n {
                let s = len * (k as f64) / ((n - 1) as f64);
                let pt = spatial.point_at(s);
                xs.push(pt[0]);
                ys.push(pt[1]);
            }
            d.set_item("x", xs)?;
            d.set_item("y", ys)?;
        }
    }
    Ok(d)
}

fn arc_fit_config(arc_fit: Option<u32>) -> PyResult<geometry::ChainFitConfig> {
    let Some(min_run_facets) = arc_fit else {
        return Ok(geometry::ChainFitConfig::default());
    };
    if min_run_facets < 3 {
        return Err(pyo3::exceptions::PyValueError::new_err(
            "[arc_fit] min_run_facets must be at least 3",
        ));
    }
    Ok(geometry::ChainFitConfig::with_arc_fit(min_run_facets))
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
const TRAJECTORY_SAMPLES_PER_MM: f64 = 8.0;
const VELOCITY_CONSISTENCY_TOL: f64 = 1e-6;
const VELOCITY_INTEGRATION_TOL: f64 = 1e-7;

// Only the raw trajectory: where the toolhead is and how fast it travels there.
// The visualizer differentiates position itself, so it stays an independent
// check on the planner rather than a mirror of the planner's own derivatives.
struct KinematicSamples {
    x: Vec<f64>,
    y: Vec<f64>,
    v: Vec<f64>,
}

fn sample_kinematics(
    outcome: &geometry::FitOutcome,
    profile: &geometry::VelocityProfile,
) -> KinematicSamples {
    let mut kin = KinematicSamples {
        x: Vec::new(),
        y: Vec::new(),
        v: Vec::new(),
    };
    let mut started = false;
    for (geo_move, vel_move) in outcome.moves.iter().zip(profile.moves.iter()) {
        if let Some(spatial) = &geo_move.segment.spatial {
            let len = spatial.s_len();
            let n = ((len * TRAJECTORY_SAMPLES_PER_MM).ceil() as usize).max(1);
            // Each segment starts where the previous ended; emitting both would
            // leave a near-zero-length step that explodes the time derivative.
            let first_k = usize::from(started);
            for k in first_k..=n {
                let s = len * (k as f64) / (n as f64);
                let pt = spatial.point_at(s);
                kin.x.push(pt[0]);
                kin.y.push(pt[1]);
                kin.v.push(speed_at(&vel_move.samples, s));
            }
            started = true;
        }
    }

    kin
}

fn speed_at(samples: &[geometry::VelSample], s: f64) -> f64 {
    // The velocity profile is sampled densely along arc length; read the speed
    // at an arbitrary s by linear interpolation between the bracketing knots.
    if samples.is_empty() {
        return 0.0;
    }
    let i = samples.partition_point(|sm| sm.s < s);
    if i == 0 {
        return samples[0].v;
    }
    if i >= samples.len() {
        return samples[samples.len() - 1].v;
    }
    let lo = &samples[i - 1];
    let hi = &samples[i];
    let span = hi.s - lo.s;
    if span <= 0.0 {
        return hi.v;
    }
    lo.v + (hi.v - lo.v) * (s - lo.s) / span
}

#[cfg(test)]
mod tests;
