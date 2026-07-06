use crossbeam_channel::unbounded;
use geometry::path::CurvatureProfile;
use geometry::path::lowering::PositionProfile;
use nurbs::bezier::extract_bezier_pieces;
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList};
use trajectory::{AxisChainSet, ShapedSegment};

use motion_pipeline::fitter::Fitter;
use motion_pipeline::planner::Planner;
use motion_pipeline::{StreamConfig, run_lowerer};

#[pyfunction]
#[pyo3(signature = (waypoints, max_velocity, max_accel, square_corner_velocity, max_jerk, arc_fit = None, max_extrude_only_velocity = None, max_extrude_only_accel = None, max_path_deviation = None, max_accel_deviation = None, pressure_advance = None, smooth_zv_hz = None))]
#[allow(clippy::too_many_arguments)]
pub fn pipeline_snapshot(
    py: Python<'_>,
    waypoints: Vec<(f64, f64, f64, f64, f64)>,
    max_velocity: f64,
    max_accel: f64,
    square_corner_velocity: f64,
    max_jerk: f64,
    arc_fit: Option<u32>,
    max_extrude_only_velocity: Option<f64>,
    max_extrude_only_accel: Option<f64>,
    max_path_deviation: Option<f64>,
    max_accel_deviation: Option<f64>,
    pressure_advance: Option<f64>,
    smooth_zv_hz: Option<f64>,
) -> PyResult<Py<PyDict>> {
    if let Some(v) = max_path_deviation {
        if !(v.is_finite() && v > 0.0) {
            return Err(pyo3::exceptions::PyValueError::new_err(format!(
                "max_path_deviation must be finite and positive, got {v}"
            )));
        }
    }
    if let Some(v) = max_accel_deviation {
        if !(v > 0.0) {
            return Err(pyo3::exceptions::PyValueError::new_err(format!(
                "max_accel_deviation must be positive, got {v}"
            )));
        }
    }
    if waypoints.len() < 2 {
        return Err(pyo3::exceptions::PyValueError::new_err(
            "need at least 2 waypoints",
        ));
    }

    let limits = geometry::VelocityLimits::try_new(
        max_velocity,
        max_accel,
        square_corner_velocity,
        max_jerk,
    )
    .map_err(pyo3::exceptions::PyValueError::new_err)?;
    let chain_cfg = arc_fit_config(arc_fit)?;

    let moves = build_moves(&waypoints, limits)?;
    let raw_points = extract_raw_path(&moves);

    let config = StreamConfig {
        chain: chain_cfg,
        integration_tol: VELOCITY_INTEGRATION_TOL,
        max_extrude_only_velocity_mm_s: max_extrude_only_velocity.unwrap_or(f64::INFINITY),
        max_extrude_only_accel_mm_s2: max_extrude_only_accel.unwrap_or(f64::INFINITY),
        fit_tol_mm: max_path_deviation.unwrap_or(TRAJECTORY_FIT_TOL_MM),
        fit_tol_accel_mm_s2: max_accel_deviation.unwrap_or(TRAJECTORY_FIT_TOL_ACCEL_MM_S2),
        max_buffer_moves: SNAPSHOT_MAX_BUFFER_MOVES,
        limits,
    };
    let axis_chains = build_axis_chains(pressure_advance, smooth_zv_hz)
        .map_err(|e| pyo3::exceptions::PyValueError::new_err(e.to_string()))?;
    let (fitted, shaped) = run_pipeline(&moves, config, axis_chains);

    let dict = PyDict::new(py);
    dict.set_item("raw_x", raw_points.iter().map(|p| p.0).collect::<Vec<_>>())?;
    dict.set_item("raw_y", raw_points.iter().map(|p| p.1).collect::<Vec<_>>())?;

    let seg_list = PyList::empty(py);
    for fm in &fitted {
        if let Some(spatial) = &fm.segment.spatial {
            let d = segment_to_pydict(py, spatial)?;
            seg_list.append(d)?;
        }
    }
    dict.set_item("fitted_segments", seg_list)?;

    let traj = collect_trajectory_pieces(&shaped);
    dict.set_item("traj_x_pieces", traj.x.clone())?;
    dict.set_item("traj_y_pieces", traj.y.clone())?;
    dict.set_item("traj_z_pieces", traj.z.clone())?;
    dict.set_item("traj_e_pieces", traj.e.clone())?;
    dict.set_item("traj_t_end", traj.t_end)?;
    dict.set_item("traversal_time_s", traj.t_end)?;

    let seams = seam_metrics(&traj);
    dict.set_item("seam_max_dp", seams.max_dp.to_vec())?;
    dict.set_item("seam_max_dv", seams.max_dv.to_vec())?;
    dict.set_item("seam_max_da", seams.max_da.to_vec())?;
    let worst = PyList::empty(py);
    for s in &seams.worst {
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

/// The snapshot's optional post-processor chains: smoothing on x/y, pressure
/// advance on the extruder lane — the same compiled stages the live config
/// builds, so snapshot piece counts and seams reflect the real wire.
fn build_axis_chains(
    pressure_advance: Option<f64>,
    smooth_zv_hz: Option<f64>,
) -> Result<AxisChainSet, trajectory::PostProcessorError> {
    use trajectory::post_processor::PostProcessorInstance;
    use trajectory::post_processors::{LinearPressureAdvance, SmoothZv};
    use trajectory::{CompiledChain, PostProcessorError};

    if pressure_advance.is_none() && smooth_zv_hz.is_none() {
        return Ok(AxisChainSet::default());
    }
    let spatial_chain = |hz: Option<f64>| -> Result<CompiledChain, PostProcessorError> {
        match hz {
            Some(hz) => CompiledChain::compile(&[PostProcessorInstance::new(
                "smooth_zv",
                &SmoothZv,
                vec![hz],
            )]),
            None => Ok(CompiledChain::default()),
        }
    };
    let e_chain = match pressure_advance {
        Some(k) => CompiledChain::compile(&[PostProcessorInstance::new(
            "pressure_advance",
            &LinearPressureAdvance,
            vec![k],
        )])?,
        None => CompiledChain::default(),
    };
    Ok(AxisChainSet {
        chains: vec![
            spatial_chain(smooth_zv_hz)?,
            spatial_chain(smooth_zv_hz)?,
            CompiledChain::default(),
            e_chain,
        ],
        followers: Vec::new(),
    })
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
    waypoints: &[(f64, f64, f64, f64, f64)],
    limits: geometry::VelocityLimits,
) -> PyResult<Vec<geometry::Move>> {
    let mut moves = Vec::with_capacity(waypoints.len() - 1);
    for (i, pair) in waypoints.windows(2).enumerate() {
        let (x0, y0, z0, e0, _) = pair[0];
        let (x1, y1, z1, e1, feedrate) = pair[1];
        let start = [x0, y0, z0];
        let end = [x1, y1, z1];
        let e_delta = e1 - e0;
        let ctx = geometry::MoveContext {
            extruder_axis: EXTRUDER_AXIS,
            feedrate_mm_s: feedrate,
            limits,
            source: geometry::SourceRange {
                start_line: i as u32,
                end_line: i as u32,
            },
        };
        match geometry::line_move(start, end, e_delta, ctx) {
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
/// The E lane rides as axis 3, past the three spatial axes — the same index the
/// production bridge and the seam harness assign the extruder.
const EXTRUDER_AXIS: usize = 3;
// Position tolerance for the cubic lowering — the same order the streamer ships.
const TRAJECTORY_FIT_TOL_MM: f64 = 0.005;
const TRAJECTORY_FIT_TOL_ACCEL_MM_S2: f64 = 50.0;
const VELOCITY_INTEGRATION_TOL: f64 = 1e-7;
/// Comfortably above any snapshot case's move count, so a case is always
/// planned as a single window instead of being split by the streaming
/// planner's look-ahead backstop.
const SNAPSHOT_MAX_BUFFER_MOVES: usize = 65_536;

/// Drives the real pipeline stages — the same `Fitter`/`Planner`/
/// `run_lowerer`/`Shaper` types `setup_stages` wires into OS threads for a
/// live print — synchronously over unbounded channels on the calling thread.
/// No stage is reimplemented: this is the production pipeline observed with
/// its intermediate fitted-stage output (pre-axis-split spatial geometry)
/// tapped alongside the final shaped output.
fn run_pipeline(
    moves: &[geometry::Move],
    config: StreamConfig,
    axis_chains: AxisChainSet,
) -> (Vec<geometry::Move>, Vec<ShapedSegment>) {
    let spatial_home = moves
        .iter()
        .find_map(|m| m.segment.spatial.as_ref())
        .map_or([0.0, 0.0, 0.0], |seg| seg.point_at(0.0));
    let home_pos = vec![spatial_home[0], spatial_home[1], spatial_home[2], 0.0];

    let (raw_tx, raw_rx) = unbounded();
    for m in moves.iter().cloned() {
        raw_tx
            .send(m.into())
            .expect("unbounded channel never blocks");
    }
    drop(raw_tx);

    let (fitted_tx, fitted_rx) = unbounded();
    Fitter::new(config.chain).run(raw_rx, fitted_tx);
    let fitted: Vec<geometry::Move> = fitted_rx
        .into_iter()
        .filter_map(|item| match item {
            motion_pipeline::StreamInput::Move(m) => Some(m),
            motion_pipeline::StreamInput::Drain | motion_pipeline::StreamInput::Control(_) => None,
        })
        .collect();

    let (planner_tx, planner_rx) = unbounded();
    for fm in fitted.iter().cloned() {
        planner_tx
            .send(fm.into())
            .expect("unbounded channel never blocks");
    }
    drop(planner_tx);

    let (planned_tx, planned_rx) = unbounded();
    Planner::new(config).run(planner_rx, planned_tx);

    let (lowered_tx, lowered_rx) = unbounded();
    run_lowerer(
        planned_rx,
        lowered_tx,
        motion_pipeline::FitTol {
            pos_mm: config.fit_tol_mm,
            accel_mm_s2: config.fit_tol_accel_mm_s2,
        },
        axis_chains.clone(),
        home_pos,
        0.0,
    );

    let (shaped_tx, shaped_rx) = unbounded();
    motion_pipeline::Shaper::new(axis_chains).run(lowered_rx, shaped_tx);
    let shaped: Vec<ShapedSegment> = shaped_rx
        .into_iter()
        .filter_map(|item| match item {
            motion_pipeline::ShapedItem::Seg(seg) => Some(seg),
            motion_pipeline::ShapedItem::Control(_) => None,
        })
        .collect();

    (fitted, shaped)
}

/// The trajectory the firmware actually executes: the host lowering's own
/// per-axis polynomial pieces of position-versus-time. Each row is
/// `[t0, t1, c0, c1, ..., cn]` — monomial coefficients in local time
/// `tau = t - t0`, trailing near-zero coefficients trimmed so each piece's
/// true degree is visible in the snapshot. The visualizer differentiates
/// these analytically, so every derivative is exact and continuous — no
/// position sampling, and nothing copied from the planner's own acceleration,
/// so it stays an independent check.
struct TrajectoryPieces {
    x: Vec<Vec<f64>>,
    y: Vec<Vec<f64>>,
    z: Vec<Vec<f64>>,
    e: Vec<Vec<f64>>,
    t_end: f64,
}

fn collect_trajectory_pieces(shaped: &[ShapedSegment]) -> TrajectoryPieces {
    fn collect(dst: &mut Vec<Vec<f64>>, axis: Option<&nurbs::ScalarNurbs<f64>>) {
        let Some(axis) = axis else { return };
        for p in extract_bezier_pieces(axis) {
            let scale = p.coeffs.iter().fold(0.0_f64, |m, c| m.max(c.abs()));
            let mut coeffs = p.coeffs.clone();
            while coeffs.len() > 1
                && coeffs
                    .last()
                    .is_some_and(|c| c.abs() <= 1e-12 * (scale + 1.0))
            {
                coeffs.pop();
            }
            let mut row = vec![p.u_start, p.u_end];
            row.extend_from_slice(&coeffs);
            dst.push(row);
        }
    }

    let mut out = TrajectoryPieces {
        x: Vec::new(),
        y: Vec::new(),
        z: Vec::new(),
        e: Vec::new(),
        t_end: 0.0,
    };
    for seg in shaped {
        collect(&mut out.x, seg.axes.first());
        collect(&mut out.y, seg.axes.get(1));
        collect(&mut out.z, seg.axes.get(2));
        collect(&mut out.e, seg.axes.get(3));
        out.t_end = seg.t_end;
    }
    out
}

/// One interior piece boundary of a per-axis cubic track, quantifying how far
/// the left piece's endpoint state disagrees with the right piece's start
/// state. Named after `SeamDescriptor` in the seam test harness, but computed
/// analytically from the visualizer's own lowered cubic coefficients: position
/// jump `dp`, velocity jump `dv`, acceleration jump `da`.
struct SeamMetric {
    t: f64,
    axis: usize,
    dp: f64,
    dv: f64,
    da: f64,
}

struct SeamMetrics {
    max_dp: [f64; 4],
    max_dv: [f64; 4],
    max_da: [f64; 4],
    worst: Vec<SeamMetric>,
}

const WORST_SEAM_COUNT: usize = 10;

fn piece_state_at(p: &[f64], tau: f64) -> (f64, f64, f64) {
    let c = &p[2..];
    let mut pos = 0.0;
    let mut vel = 0.0;
    let mut acc = 0.0;
    for (k, &ck) in c.iter().enumerate().rev() {
        pos = pos * tau + ck;
        if k >= 1 {
            vel = vel * tau + k as f64 * ck;
        }
        if k >= 2 {
            acc = acc * tau + (k * (k - 1)) as f64 * ck;
        }
    }
    (pos, vel, acc)
}

fn piece_end_state(p: &[f64]) -> (f64, f64, f64) {
    piece_state_at(p, p[1] - p[0])
}

fn piece_start_state(p: &[f64]) -> (f64, f64, f64) {
    piece_state_at(p, 0.0)
}

fn seam_metrics(traj: &TrajectoryPieces) -> SeamMetrics {
    let axes = [&traj.x, &traj.y, &traj.z, &traj.e];
    let mut out = SeamMetrics {
        max_dp: [0.0; 4],
        max_dv: [0.0; 4],
        max_da: [0.0; 4],
        worst: Vec::new(),
    };
    for (axis, pieces) in axes.iter().enumerate() {
        for w in pieces.windows(2) {
            let (lp, lv, la) = piece_end_state(&w[0]);
            let (rp, rv, ra) = piece_start_state(&w[1]);
            let seam = SeamMetric {
                t: w[1][0],
                axis,
                dp: (rp - lp).abs(),
                dv: (rv - lv).abs(),
                da: (ra - la).abs(),
            };
            out.max_dp[axis] = out.max_dp[axis].max(seam.dp);
            out.max_dv[axis] = out.max_dv[axis].max(seam.dv);
            out.max_da[axis] = out.max_da[axis].max(seam.da);
            out.worst.push(seam);
        }
    }
    out.worst.sort_by(|a, b| {
        b.da.total_cmp(&a.da)
            .then(b.dv.total_cmp(&a.dv))
            .then(a.axis.cmp(&b.axis))
            .then(a.t.total_cmp(&b.t))
    });
    out.worst.truncate(WORST_SEAM_COUNT);
    out
}

#[cfg(test)]
mod tests;
