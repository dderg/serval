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
    arc_fit: Option<(f64, f64)>,
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

    dict.set_item("kin_s", &kinematics.s)?;
    dict.set_item("kin_v", &kinematics.v)?;
    dict.set_item("kin_heading_x", &kinematics.heading_x)?;
    dict.set_item("kin_heading_y", &kinematics.heading_y)?;
    dict.set_item("kin_kappa", &kinematics.kappa)?;
    dict.set_item("kin_dkappa_ds", &kinematics.dkappa_ds)?;
    dict.set_item("kin_a_t", &kinematics.a_t)?;
    dict.set_item("kin_j_t", &kinematics.j_t)?;
    dict.set_item("kin_j_n", &kinematics.j_n)?;
    dict.set_item("kin_j_n_geom", &kinematics.j_n_geom)?;
    dict.set_item("kin_j_n_couple", &kinematics.j_n_couple)?;

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
        geometry::path::Segment::Arc(arc) => {
            d.set_item("type", "arc")?;
            d.set_item("cx", arc.origin[0])?;
            d.set_item("cy", arc.origin[1])?;
            d.set_item("radius", arc.radius)?;
            let basis_angle_deg = arc.u[1].atan2(arc.u[0]).to_degrees();
            d.set_item("angle_deg", basis_angle_deg)?;
            d.set_item("theta1_deg", arc.start_angle.to_degrees())?;
            let theta2 = arc.start_angle + arc.sweep;
            d.set_item("theta2_deg", theta2.to_degrees())?;
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

fn arc_fit_config(arc_fit: Option<(f64, f64)>) -> PyResult<geometry::ChainFitConfig> {
    let Some((facet_length_mm, max_angle_deg)) = arc_fit else {
        return Ok(geometry::ChainFitConfig::default());
    };
    if !(facet_length_mm.is_finite() && facet_length_mm > 0.0) {
        return Err(pyo3::exceptions::PyValueError::new_err(
            "[arc_fit] facet_length_mm must be finite and positive",
        ));
    }
    if !(max_angle_deg.is_finite() && max_angle_deg > 0.0 && max_angle_deg < 180.0) {
        return Err(pyo3::exceptions::PyValueError::new_err(
            "[arc_fit] max_angle_deg must be finite and in (0, 180)",
        ));
    }
    Ok(geometry::ChainFitConfig::with_arc_fit(
        facet_length_mm,
        max_angle_deg.to_radians(),
    ))
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
const VELOCITY_CONSISTENCY_TOL: f64 = 1e-6;
const VELOCITY_INTEGRATION_TOL: f64 = 1e-7;

struct KinematicSamples {
    s: Vec<f64>,
    v: Vec<f64>,
    heading_x: Vec<f64>,
    heading_y: Vec<f64>,
    kappa: Vec<f64>,
    dkappa_ds: Vec<f64>,
    a_t: Vec<f64>,
    j_t: Vec<f64>,
    j_n: Vec<f64>,
    j_n_geom: Vec<f64>,
    j_n_couple: Vec<f64>,
}

fn sample_kinematics(
    outcome: &geometry::FitOutcome,
    profile: &geometry::VelocityProfile,
) -> KinematicSamples {
    let mut kin = KinematicSamples {
        s: Vec::new(),
        v: Vec::new(),
        heading_x: Vec::new(),
        heading_y: Vec::new(),
        kappa: Vec::new(),
        dkappa_ds: Vec::new(),
        a_t: Vec::new(),
        j_t: Vec::new(),
        j_n: Vec::new(),
        j_n_geom: Vec::new(),
        j_n_couple: Vec::new(),
    };
    let mut s_offset = 0.0;
    for (geo_move, vel_move) in outcome.moves.iter().zip(profile.moves.iter()) {
        if let Some(spatial) = &geo_move.segment.spatial {
            for sample in &vel_move.samples {
                let s_local = sample.s.clamp(0.0, spatial.s_len());
                let heading = spatial.heading_at(s_local);
                kin.s.push(s_offset + sample.s);
                kin.v.push(sample.v);
                kin.heading_x.push(heading[0]);
                kin.heading_y.push(heading[1]);
                kin.kappa.push(spatial.kappa(s_local));
                kin.dkappa_ds.push(spatial.dkappa_ds(s_local));
            }
        }
        s_offset += vel_move.length;
    }

    kin.a_t = tangential_accel(&kin.s, &kin.v);
    kin.j_t = tangential_jerk(&kin.s, &kin.v, &kin.a_t);

    for i in 0..kin.s.len() {
        let probe = crate::jerk_probe::jerk_at(
            kin.kappa[i],
            kin.dkappa_ds[i],
            kin.v[i],
            kin.a_t[i],
            kin.j_t[i],
        );
        kin.j_n.push(probe.j_n);
        kin.j_n_geom.push(probe.j_n_geom);
        kin.j_n_couple.push(probe.j_n_couple);
    }

    kin
}

fn tangential_accel(s: &[f64], v: &[f64]) -> Vec<f64> {
    let n = s.len();
    let mut a = vec![0.0; n];
    if n < 2 {
        return a;
    }
    let pair_dt = |i: usize, k: usize| -> f64 {
        let ds = (s[k] - s[i]).abs();
        let v_sum = v[i] + v[k];
        if v_sum > 0.0 { 2.0 * ds / v_sum } else { 0.0 }
    };
    for i in 0..n {
        let (lo, hi, span) = if i == 0 {
            (0, 1, pair_dt(0, 1))
        } else if i == n - 1 {
            (n - 2, n - 1, pair_dt(n - 2, n - 1))
        } else {
            (i - 1, i + 1, pair_dt(i - 1, i) + pair_dt(i, i + 1))
        };
        a[i] = if span > 0.0 {
            (v[hi] - v[lo]) / span
        } else {
            0.0
        };
    }
    a
}

fn tangential_jerk(s: &[f64], v: &[f64], a: &[f64]) -> Vec<f64> {
    let n = s.len();
    let mut j = vec![0.0; n];
    if n < 2 {
        return j;
    }
    let pair_dt = |i: usize, k: usize| -> f64 {
        let ds = (s[k] - s[i]).abs();
        let v_sum = v[i] + v[k];
        if v_sum > 0.0 { 2.0 * ds / v_sum } else { 0.0 }
    };
    for i in 0..n {
        let (lo, hi, span) = if i == 0 {
            (0, 1, pair_dt(0, 1))
        } else if i == n - 1 {
            (n - 2, n - 1, pair_dt(n - 2, n - 1))
        } else {
            (i - 1, i + 1, pair_dt(i - 1, i) + pair_dt(i, i + 1))
        };
        j[i] = if span > 0.0 {
            (a[hi] - a[lo]) / span
        } else {
            0.0
        };
    }
    j
}

#[cfg(test)]
mod tests;
