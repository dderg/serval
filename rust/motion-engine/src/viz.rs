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
    let traj = lower_trajectory(&outcome, &profile);

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

    let pieces = |ps: &[[f64; 6]]| ps.iter().map(|p| p.to_vec()).collect::<Vec<_>>();
    dict.set_item("traj_x_pieces", pieces(&traj.x))?;
    dict.set_item("traj_y_pieces", pieces(&traj.y))?;
    dict.set_item("traj_t_end", traj.t_end)?;

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
// Position tolerance for the cubic lowering — the same order the streamer ships.
const TRAJECTORY_FIT_TOL_MM: f64 = 0.005;
const VELOCITY_CONSISTENCY_TOL: f64 = 1e-6;
const VELOCITY_INTEGRATION_TOL: f64 = 1e-7;

// The trajectory the firmware actually executes: the host lowering's own per-axis
// cubic Bézier pieces of position-versus-time. Each piece is a cubic in local time
// `tau = t - t0`, `pos = c0 + c1*tau + c2*tau^2 + c3*tau^3`, stored as
// `[t0, t1, c0, c1, c2, c3]`. The visualizer differentiates these analytically
// (velocity quadratic, acceleration linear, jerk constant per piece), so every
// derivative is exact and continuous — no position sampling, and nothing copied
// from the planner's own acceleration, so it stays an independent check while
// storing only a handful of coefficients per piece.
struct TrajectoryPieces {
    x: Vec<[f64; 6]>,
    y: Vec<[f64; 6]>,
    t_end: f64,
}

fn lower_trajectory(
    outcome: &geometry::FitOutcome,
    profile: &geometry::VelocityProfile,
) -> TrajectoryPieces {
    fn collect(dst: &mut Vec<[f64; 6]>, pieces: &[nurbs::bezier::BezierPiece<f64>]) {
        for p in pieces {
            let c = |i: usize| p.coeffs.get(i).copied().unwrap_or(0.0);
            dst.push([p.u_start, p.u_end, c(0), c(1), c(2), c(3)]);
        }
    }

    let mut out = TrajectoryPieces {
        x: Vec::new(),
        y: Vec::new(),
        t_end: 0.0,
    };
    let mut t_start = 0.0;
    let mut pos = [0.0_f64; 3];
    let mut started = false;
    for (gm, vm) in outcome.moves.iter().zip(profile.moves.iter()) {
        let Some(spatial) = &gm.segment.spatial else {
            continue;
        };
        if !started {
            pos = spatial.point_at(0.0);
            started = true;
        }
        let Ok((axes, total_t)) =
            crate::lowering::lower_move_pieces(gm, vm, t_start, &pos, TRAJECTORY_FIT_TOL_MM, &[])
        else {
            continue;
        };
        collect(&mut out.x, &axes[0]);
        collect(&mut out.y, &axes[1]);
        t_start += total_t;
        out.t_end = t_start;
        pos = spatial.point_at(spatial.s_len());
    }
    out
}

#[cfg(test)]
mod tests;
