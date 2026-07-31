use super::*;
use crate::StreamConfig;
use crate::fit_stage::FitStage;
use crate::planner::Planner;
use crate::types::PlannedMove;
use crossbeam_channel::unbounded;
use geometry::path::{Arc, PathSegment, Segment};
use geometry::segment::SourceRange;
use geometry::{CornerFitConfig, MoveContext, VelocityLimits, line_move};
use nurbs::bezier::extract_bezier_pieces;
use nurbs::eval::eval;

fn stream_config() -> StreamConfig {
    StreamConfig {
        corner: CornerFitConfig::default(),
        integration_tol: 1e-7,
        max_extrude_only_velocity_mm_s: f64::INFINITY,
        max_extrude_only_accel_mm_s2: f64::INFINITY,
        fit_tol_mm: FIT_TOL_MM,
        fit_tol_accel_mm_s2: 50.0,
        max_buffer_moves: 512,
        limits: VelocityLimits::try_new(300.0, 5000.0, 5.0, 100_000.0).unwrap(),
    }
}

/// Fits and plans `moves` through the real streaming `FitStage`/`Planner`
/// stages, synchronously over unbounded channels (no threads needed — see
/// `fit_stage/tests.rs`'s `run_fit_stage` for the same technique).
fn fit_and_plan(moves: &[geometry::Move]) -> Vec<PlannedMove> {
    let (raw_tx, raw_rx) = unbounded();
    for m in moves.iter().cloned() {
        raw_tx
            .send(m.into())
            .expect("unbounded channel never blocks");
    }
    drop(raw_tx);

    let (fitted_tx, fitted_rx) = unbounded();
    FitStage::new(CornerFitConfig::default()).run(raw_rx, fitted_tx);

    let (planned_tx, planned_rx) = unbounded();
    Planner::new(stream_config()).run(fitted_rx, planned_tx);
    planned_rx
        .into_iter()
        .filter_map(|item| match item {
            crate::PlannedItem::Move(m) => Some(m),
            crate::PlannedItem::Drain | crate::PlannedItem::Control(_) => None,
        })
        .collect()
}

#[test]
fn lowering_emits_coarse_pieces_above_the_sample_floor() {
    for (start, end) in [
        ([100.0, 100.0, 10.0], [100.0, 90.0, 10.0]), // the bench Y jog
        ([0.0, 0.0, 0.0], [200.0, 0.0, 0.0]),        // long fast line
        ([0.0, 0.0, 0.0], [0.5, 0.0, 0.0]),          // short line
    ] {
        let m = line_move(start, end, 0.0, ctx(1, 100.0)).unwrap();
        let planned = fit_and_plan(std::slice::from_ref(&m));
        let seg = lower_move(
            &planned[0].geometry,
            &planned[0].velocity,
            0.0,
            &[0.0; 4],
            FIT_TOL,
            &[],
            None,
        )
        .unwrap();
        for axis in 0..3 {
            let pieces = extract_bezier_pieces(&seg.axes[axis]);
            assert!(
                pieces.len() < 256,
                "axis {axis}: {} pieces for {start:?}->{end:?} — should be coarse",
                pieces.len()
            );
            for p in &pieces {
                let fit_was_subdivided = pieces.len() > 1;
                if fit_was_subdivided {
                    assert!(
                        p.u_end - p.u_start >= MIN_FIT_PIECE_S - 1e-9,
                        "axis {axis}: piece {:.6}us below the floor",
                        (p.u_end - p.u_start) * 1e6
                    );
                }
            }
        }
    }
}

const FIT_TOL_MM: f64 = 1e-3;
const FIT_TOL: FitTol = FitTol {
    pos_mm: FIT_TOL_MM,
    accel_mm_s2: 50.0,
};

fn tol_pos(pos_mm: f64) -> FitTol {
    FitTol {
        pos_mm,
        accel_mm_s2: 50.0,
    }
}

fn ctx(line_no: u32, feed: f64) -> MoveContext {
    MoveContext {
        extruder_axis: 3,
        feedrate_mm_s: feed,
        limits: VelocityLimits::try_new(300.0, 5000.0, 5.0, 100_000.0).unwrap(),
        source: SourceRange {
            start_line: line_no,
            end_line: line_no,
        },
    }
}

fn eval_piece(p: &BezierPiece, t: f64) -> f64 {
    let z = t - p.u_start;
    let c = |i: usize| p.coeffs.get(i).copied().unwrap_or(0.0);
    c(0) + c(1) * z + c(2) * z * z + c(3) * z * z * z
}

fn vel_piece(p: &BezierPiece, t: f64) -> f64 {
    let z = t - p.u_start;
    let c = |i: usize| p.coeffs.get(i).copied().unwrap_or(0.0);
    c(1) + 2.0 * c(2) * z + 3.0 * c(3) * z * z
}

fn peak_accel(axes: &[Vec<BezierPiece>]) -> f64 {
    let accel = |p: &BezierPiece, t: f64| {
        let z = t - p.u_start;
        2.0 * p.coeffs[2] + 6.0 * p.coeffs[3] * z
    };
    let mut peak = 0.0_f64;
    for (px, py) in axes[0].iter().zip(&axes[1]) {
        for k in 0..=64 {
            let t = px.u_start + (px.u_end - px.u_start) * f64::from(k) / 64.0;
            peak = peak.max(libm::hypot(accel(px, t), accel(py, t)));
        }
    }
    peak
}

fn straight_ctx(line_no: u32) -> MoveContext {
    MoveContext {
        extruder_axis: 3,
        feedrate_mm_s: 300.0,
        limits: VelocityLimits::try_new(300.0, 1000.0, 0.0, 100_000.0).unwrap(),
        source: SourceRange {
            start_line: line_no,
            end_line: line_no,
        },
    }
}

#[test]
fn straight_move_lowers_one_cubic_per_phase_without_accel_overshoot() {
    let m = line_move([0.0, 0.0, 0.0], [10.0, 0.0, 0.0], 0.0, straight_ctx(1)).unwrap();
    let planned = fit_and_plan(std::slice::from_ref(&m));
    let vm = &planned[0].velocity;
    assert!(!vm.phases.is_empty(), "straight move should carry phases");

    let (axes, total_t) =
        lower_move_pieces(&planned[0].geometry, vm, 0.0, &[0.0; 4], FIT_TOL, &[], None).unwrap();

    assert_eq!(axes[0].len(), vm.phases.len(), "one cubic per phase");
    // The grid fit overshot the 1000 cap to ~1170 here; the phase fit is exact.
    assert!(
        peak_accel(&axes) <= 1000.0 + 1e-6,
        "peak |a| = {} exceeds cap",
        peak_accel(&axes)
    );
    let phase_t: f64 = vm.phases.iter().map(|p| p.dt).sum();
    assert!(
        (total_t - phase_t).abs() < 1e-12,
        "total time is the phase sum"
    );
    assert!((eval_piece(&axes[0][0], 0.0)).abs() < 1e-9, "starts at 0");
    let last = axes[0].last().unwrap();
    assert!(
        (eval_piece(last, last.u_end) - 10.0).abs() < 1e-9,
        "ends at 10"
    );
}

#[test]
fn collinear_run_slices_phases_c1_continuous_at_the_seam() {
    let m0 = line_move([0.0, 0.0, 0.0], [5.0, 0.0, 0.0], 0.0, straight_ctx(1)).unwrap();
    let m1 = line_move([5.0, 0.0, 0.0], [10.0, 0.0, 0.0], 0.0, straight_ctx(2)).unwrap();
    let planned = fit_and_plan(&[m0, m1]);
    assert!(!planned[0].velocity.phases.is_empty() && !planned[1].velocity.phases.is_empty());

    let (a0, t0) = lower_move_pieces(
        &planned[0].geometry,
        &planned[0].velocity,
        0.0,
        &[0.0; 4],
        FIT_TOL,
        &[],
        None,
    )
    .unwrap();
    let (a1, t1) = lower_move_pieces(
        &planned[1].geometry,
        &planned[1].velocity,
        t0,
        &[5.0, 0.0, 0.0, 0.0],
        FIT_TOL,
        &[],
        None,
    )
    .unwrap();

    let end0 = a0[0].last().unwrap();
    let start1 = &a1[0][0];
    assert!(
        (eval_piece(end0, end0.u_end) - 5.0).abs() < 1e-7,
        "seam at x=5"
    );
    assert!(
        (eval_piece(end0, end0.u_end) - eval_piece(start1, start1.u_start)).abs() < 1e-7,
        "position continuous across the seam"
    );
    assert!(
        (vel_piece(end0, end0.u_end) - vel_piece(start1, start1.u_start)).abs() < 1e-6,
        "velocity continuous across the seam"
    );
    assert!(peak_accel(&a0) <= 1000.0 + 1e-6 && peak_accel(&a1) <= 1000.0 + 1e-6);

    // The two slices retime to the same total as a single 10 mm move.
    let single = line_move([0.0, 0.0, 0.0], [10.0, 0.0, 0.0], 0.0, straight_ctx(1)).unwrap();
    let single_planned = fit_and_plan(std::slice::from_ref(&single));
    let single_t: f64 = single_planned[0].velocity.phases.iter().map(|p| p.dt).sum();
    assert!(
        (t0 + t1 - single_t).abs() < 1e-9,
        "sliced time equals the whole move"
    );
}

#[test]
fn linear_pressure_advance_is_exact_cubic_transform() {
    // pos = 1 + 2t + 3t^2 + 4t^3,  vel = 2 + 6t + 12t^2
    let mut coeffs = [1.0, 2.0, 3.0, 4.0];
    let k = 0.05;
    apply_derivative_gains(
        &mut coeffs,
        &CompiledChain {
            stages: vec![ChainStage::DerivativeGains { k1: k, k2: 0.0 }],
        },
    );
    // smoothed = pos + k*vel -> c0+=k*2, c1+=k*6, c2+=k*12, c3 unchanged
    assert!((coeffs[0] - (1.0 + k * 2.0)).abs() < 1e-12);
    assert!((coeffs[1] - (2.0 + k * 6.0)).abs() < 1e-12);
    assert!((coeffs[2] - (3.0 + k * 12.0)).abs() < 1e-12);
    assert!((coeffs[3] - 4.0).abs() < 1e-12);
}

#[test]
fn derivative_gains_second_order_is_exact_cubic_transform() {
    // pos = 1 + 2t + 3t^2 + 4t^3,  vel = 2 + 6t + 12t^2,  accel = 6 + 24t
    let mut coeffs = [1.0, 2.0, 3.0, 4.0];
    let (k1, k2) = (0.05, 0.002);
    apply_derivative_gains(
        &mut coeffs,
        &CompiledChain {
            stages: vec![ChainStage::DerivativeGains { k1, k2 }],
        },
    );
    assert!((coeffs[0] - (1.0 + k1 * 2.0 + k2 * 6.0)).abs() < 1e-12);
    assert!((coeffs[1] - (2.0 + k1 * 6.0 + k2 * 24.0)).abs() < 1e-12);
    assert!((coeffs[2] - (3.0 + k1 * 12.0)).abs() < 1e-12);
    assert!((coeffs[3] - 4.0).abs() < 1e-12);
}

fn lower_single(m: geometry::Move, t_start: f64, start_pos: &[f64]) -> ShapedSegment {
    let planned = fit_and_plan(std::slice::from_ref(&m));
    lower_move(
        &planned[0].geometry,
        &planned[0].velocity,
        t_start,
        start_pos,
        FIT_TOL,
        &[],
        None,
    )
    .expect("lower")
}

#[test]
fn line_lowers_to_exact_endpoints() {
    let m = line_move([0.0, 0.0, 0.0], [100.0, 0.0, 0.0], 0.0, ctx(1, 50.0)).unwrap();
    let seg = lower_single(m, 2.5, &[0.0, 0.0, 0.0]);

    assert_eq!(seg.t_start, 2.5);
    assert!(seg.t_end > seg.t_start);
    assert!((eval(&seg.axes[0], seg.t_start) - 0.0).abs() < 1e-6);
    assert!((eval(&seg.axes[0], seg.t_end) - 100.0).abs() < 1e-6);
    let mid = 0.5 * (seg.t_start + seg.t_end);
    assert!(eval(&seg.axes[1], mid).abs() < 1e-9);
    assert!(eval(&seg.axes[2], mid).abs() < 1e-9);
}

#[test]
fn line_position_is_monotone_and_speed_capped() {
    let m = line_move([0.0, 0.0, 0.0], [100.0, 0.0, 0.0], 0.0, ctx(1, 50.0)).unwrap();
    let seg = lower_single(m, 0.0, &[0.0, 0.0, 0.0]);

    let n = 200;
    let dt = (seg.t_end - seg.t_start) / n as f64;
    let mut prev_x = eval(&seg.axes[0], seg.t_start);
    for k in 1..=n {
        let t = seg.t_start + k as f64 * dt;
        let x = eval(&seg.axes[0], t);
        assert!(x + 1e-9 >= prev_x, "x regressed at t={t}: {x} < {prev_x}");
        let speed = (x - prev_x).abs() / dt;
        // The planned profile is strictly capped at the feed; this finite
        // difference of the position fit carries the lowering's `FIT_TOL_MM`
        // slop (non-shape-preserving coarse Bezier), so allow 0.1% over the cap.
        assert!(
            speed <= 50.0 * (1.0 + 1e-3),
            "speed {speed} exceeds feed cap at t={t}"
        );
        prev_x = x;
    }
}

#[test]
fn follower_endpoint_matches_commanded_delta() {
    let m = line_move([0.0, 0.0, 0.0], [40.0, 0.0, 0.0], 8.0, ctx(2, 50.0)).unwrap();
    let seg = lower_single(m, 0.0, &[0.0, 0.0, 0.0, 0.0]);

    assert_eq!(seg.followers.len(), 1);
    assert_eq!(seg.followers[0].axis_index, 3);
    assert!((eval(&seg.axes[3], seg.t_start) - 0.0).abs() < 1e-6);
    assert!((eval(&seg.axes[3], seg.t_end) - 8.0).abs() < 1e-3);

    let n = 100;
    let dt = (seg.t_end - seg.t_start) / n as f64;
    let mut prev_e = eval(&seg.axes[3], seg.t_start);
    for k in 1..=n {
        let e = eval(&seg.axes[3], seg.t_start + k as f64 * dt);
        assert!(e + 1e-9 >= prev_e, "extruder regressed: {e} < {prev_e}");
        prev_e = e;
    }
}

#[test]
fn follower_base_offsets_from_start_position() {
    let m = line_move([0.0, 0.0, 0.0], [40.0, 0.0, 0.0], 8.0, ctx(2, 50.0)).unwrap();
    let seg = lower_single(m, 0.0, &[0.0, 0.0, 0.0, 17.0]);
    assert!((eval(&seg.axes[3], seg.t_start) - 17.0).abs() < 1e-6);
    assert!((eval(&seg.axes[3], seg.t_end) - 25.0).abs() < 1e-3);
}

#[test]
fn arc_lowers_within_tolerance_of_the_circle() {
    let center = [0.0, 20.0, 0.0];
    let radius = 20.0;
    let m = quarter_arc();
    let seg = lower_single(m, 0.0, &[0.0, 0.0, 0.0]);

    let n = 300;
    let dt = (seg.t_end - seg.t_start) / n as f64;
    for k in 0..=n {
        let t = seg.t_start + k as f64 * dt;
        let x = eval(&seg.axes[0], t);
        let y = eval(&seg.axes[1], t);
        let r = ((x - center[0]).powi(2) + (y - center[1]).powi(2)).sqrt();
        assert!(
            (r - radius).abs() < 1e-2,
            "off-circle at t={t}: r={r} (expected {radius})"
        );
    }
}

#[test]
fn virtual_extrude_holds_spatial_and_ramps_follower() {
    let m = line_move([5.0, 6.0, 7.0], [5.0, 6.0, 7.0], 3.0, ctx(4, 20.0)).unwrap();
    let seg = lower_single(m, 0.0, &[5.0, 6.0, 7.0, 1.0]);

    let mid = 0.5 * (seg.t_start + seg.t_end);
    assert!((eval(&seg.axes[0], mid) - 5.0).abs() < 1e-9);
    assert!((eval(&seg.axes[1], mid) - 6.0).abs() < 1e-9);
    assert!((eval(&seg.axes[2], mid) - 7.0).abs() < 1e-9);
    assert!((eval(&seg.axes[3], seg.t_start) - 1.0).abs() < 1e-6);
    assert!((eval(&seg.axes[3], seg.t_end) - 4.0).abs() < 1e-3);
}

#[test]
fn source_mismatch_is_rejected() {
    let m = line_move([0.0, 0.0, 0.0], [10.0, 0.0, 0.0], 0.0, ctx(1, 50.0)).unwrap();
    let planned = fit_and_plan(std::slice::from_ref(&m));
    let other = line_move([0.0, 0.0, 0.0], [10.0, 0.0, 0.0], 0.0, ctx(99, 50.0)).unwrap();
    let other_planned = fit_and_plan(std::slice::from_ref(&other));
    assert!(matches!(
        lower_move(
            &other_planned[0].geometry,
            &planned[0].velocity,
            0.0,
            &[0.0, 0.0, 0.0],
            FIT_TOL,
            &[],
            None
        ),
        Err(LoweringError::SourceMismatch)
    ));
}

fn linear_pa_chains(extruder_axis: usize, k: f64) -> Vec<CompiledChain> {
    let mut chains = vec![CompiledChain::default(); extruder_axis + 1];
    chains[extruder_axis] = CompiledChain {
        stages: (k != 0.0)
            .then_some(trajectory::ChainStage::DerivativeGains { k1: k, k2: 0.0 })
            .into_iter()
            .collect(),
    };
    chains
}

#[test]
fn pressure_advance_shifts_follower_and_leaves_xyz_byte_identical() {
    let m = line_move([0.0, 0.0, 0.0], [40.0, 0.0, 0.0], 4.0, ctx(1, 100.0)).unwrap();
    let planned = fit_and_plan(std::slice::from_ref(&m));
    let start = [0.0; 4];

    let base = lower_move(
        &planned[0].geometry,
        &planned[0].velocity,
        0.0,
        &start,
        FIT_TOL,
        &[],
        None,
    )
    .unwrap();

    let k = 0.05;
    let pa = lower_move(
        &planned[0].geometry,
        &planned[0].velocity,
        0.0,
        &start,
        FIT_TOL,
        &linear_pa_chains(3, k),
        None,
    )
    .unwrap();

    for axis in 0..3 {
        let bp = extract_bezier_pieces(&base.axes[axis]);
        let pp = extract_bezier_pieces(&pa.axes[axis]);
        assert_eq!(
            bp.len(),
            pp.len(),
            "axis {axis} piece count changed under PA"
        );
        for (b, p) in bp.iter().zip(&pp) {
            assert_eq!(b.u_start, p.u_start, "axis {axis} grid moved under PA");
            assert_eq!(b.u_end, p.u_end, "axis {axis} grid moved under PA");
            assert_eq!(
                b.coeffs, p.coeffs,
                "axis {axis} XYZ coeffs changed under PA"
            );
        }
    }

    let (t0, t1) = (base.t_start, base.t_end);
    let h = 1e-6 * (t1 - t0);
    for frac in [0.1_f64, 0.3, 0.5, 0.7, 0.9] {
        let t = frac.mul_add(t1 - t0, t0);
        let e_base = eval(&base.axes[3], t);
        let e_pa = eval(&pa.axes[3], t);
        let e_dot = (eval(&base.axes[3], t + h) - eval(&base.axes[3], t - h)) / (2.0 * h);
        let want = k * e_dot;
        assert!(
            ((e_pa - e_base) - want).abs() < 5e-3,
            "PA shift at t={t}: got {}, want k*e_dot={want}",
            e_pa - e_base
        );
    }
}

fn piece_accel_at(pieces: &[BezierPiece], t: f64) -> f64 {
    let p = pieces
        .iter()
        .find(|p| t >= p.u_start - 1e-12 && t <= p.u_end + 1e-12)
        .unwrap_or_else(|| pieces.last().unwrap());
    let z = t - p.u_start;
    let mut acc = 0.0;
    for (k, &ck) in p.coeffs.iter().enumerate().skip(2).rev() {
        acc = acc * z + (k * (k - 1)) as f64 * ck;
    }
    acc
}

fn quarter_arc() -> geometry::Move {
    quarter_arc_with_jerk(100_000.0)
}

fn quarter_arc_with_jerk(jerk: f64) -> geometry::Move {
    let mut arc_ctx = ctx(3, 50.0);
    arc_ctx.limits = VelocityLimits::try_new(300.0, 5000.0, 5.0, jerk).unwrap();
    let arc = Arc::try_new(
        [0.0, 20.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        20.0,
        -std::f64::consts::FRAC_PI_2,
        std::f64::consts::FRAC_PI_2,
    )
    .unwrap();
    geometry::Move {
        segment: PathSegment::try_new(Segment::Arc(arc), Vec::new()).unwrap(),
        feedrate_mm_s: arc_ctx.feedrate_mm_s,
        limits: arc_ctx.limits,
        source: arc_ctx.source,
    }
}

#[test]
fn scalar_profile_reproduces_the_plan_at_every_phase_knot() {
    let planned = fit_and_plan(std::slice::from_ref(&quarter_arc()));
    let vm = &planned[0].velocity;
    assert!(vm.phases.len() >= 2, "arc must carry a multi-phase plan");

    let (profile, total_t) = profile_from_phases(&vm.phases).unwrap();
    assert_eq!(profile.knot_t.len(), vm.phases.len() + 1);
    for (i, phase) in vm.phases.iter().enumerate() {
        let (s, v, a) = profile.state_at(profile.knot_t[i]);
        assert!(
            (s - phase.s0).abs() < 1e-9,
            "s at knot {i}: {s} vs {}",
            phase.s0
        );
        assert!(
            (v - phase.v0).abs() < 1e-9,
            "v at knot {i}: {v} vs {}",
            phase.v0
        );
        assert!(
            (a - phase.a0).abs() < 1e-9,
            "a at knot {i}: {a} vs {}",
            phase.a0
        );
    }
    let (es, ev, ea) = vm.phases.last().unwrap().end_state();
    let (s, v, a) = profile.state_at(total_t);
    assert!((s - es).abs() < 1e-9, "s at the end: {s} vs {es}");
    assert!((v - ev).abs() < 1e-9, "v at the end: {v} vs {ev}");
    assert!((a - ea).abs() < 1e-9, "a at the end: {a} vs {ea}");
}

/// The fitted acceleration (second derivative of the cubic pieces) must join
/// continuously across interior knots, and converge to the analytic profile
/// acceleration as the fit tolerance tightens.
#[test]
fn curved_fit_acceleration_is_continuous_and_converges() {
    let gm = quarter_arc();
    let planned = fit_and_plan(std::slice::from_ref(&gm));
    let vm = &planned[0].velocity;
    assert!(!vm.phases.is_empty());

    let (profile, total_t) = profile_from_phases(&vm.phases).unwrap();
    let start = [0.0_f64; 4];
    let sampler = Sampler {
        profile: &profile,
        spatial: gm.segment.spatial.as_ref(),
        start_pos: &start,
        followers: &gm.segment.followers,
        s_len: gm.segment.s_len(),
        axis_chains: &[],
        z_warp: ZWarp::None,
    };

    let max_analytic_err = |tol: f64| -> f64 {
        let (axes, _t) = lower_move_pieces(
            &planned[0].geometry,
            vm,
            0.0,
            &start,
            tol_pos(tol),
            &[],
            None,
        )
        .unwrap();
        let mut worst = 0.0_f64;
        for axis in 0..2 {
            let pieces = &axes[axis];
            // Seams are C² by construction; the only permitted step is the
            // Chebyshev truncation budget, once per adjoining piece.
            for w in pieces.windows(2) {
                let (left, right) = (&w[0], &w[1]);
                let jump = (piece_accel_at(std::slice::from_ref(left), left.u_end)
                    - piece_accel_at(std::slice::from_ref(right), right.u_start))
                .abs();
                assert!(
                    jump <= 2.0 * FIT_TRUNC_ACC_MM_S2 + 1e-6,
                    "axis {axis}: accel jump {jump} at {} exceeds the truncation budget",
                    right.u_start
                );
            }
            // Convergence to the analytic accel over the interior, away from the
            // rest cusps at either end (`v(s) ~ s^(2/3)` there gives an unbounded
            // accel slope no finite piece resolves).
            let n = 400;
            for k in 1..n {
                let frac = f64::from(k) / f64::from(n);
                if !(0.1..=0.9).contains(&frac) {
                    continue;
                }
                let t = total_t * frac;
                let fit = piece_accel_at(pieces, t);
                let truth = sampler.axis_accel(axis, t, false);
                worst = worst.max((fit - truth).abs());
            }
        }
        worst
    };

    // Interior accel error no longer scales with the position tolerance — the
    // ladder holds it under the accel budget at any tolerance, with a floor
    // set by the truncation budget. Assert the absolute contract instead.
    let loose = max_analytic_err(1e-3);
    let tight = max_analytic_err(1e-4);
    assert!(
        tight <= loose + FIT_TRUNC_ACC_MM_S2,
        "tightening the tolerance regressed accel error: {tight} vs {loose}"
    );
    for (name, err) in [("loose", loose), ("tight", tight)] {
        assert!(
            err < 0.1 * FIT_TOL.accel_mm_s2,
            "{name} interior accel error {err} not comfortably inside the budget"
        );
    }
}

/// A straight move is lowered through the closed-form phase path, which ignores
/// `fit_tol_mm` entirely — so two very different tolerances produce bit-identical
/// pieces. The grid fit would subdivide differently under each.
#[test]
fn straight_move_ignores_fit_tolerance_and_stays_bit_identical() {
    let m = line_move([0.0, 0.0, 0.0], [10.0, 0.0, 0.0], 0.0, straight_ctx(1)).unwrap();
    let planned = fit_and_plan(std::slice::from_ref(&m));
    assert!(!planned[0].velocity.phases.is_empty());

    let (coarse, _) = lower_move_pieces(
        &planned[0].geometry,
        &planned[0].velocity,
        0.0,
        &[0.0; 4],
        tol_pos(1e-2),
        &[],
        None,
    )
    .unwrap();
    let (fine, _) = lower_move_pieces(
        &planned[0].geometry,
        &planned[0].velocity,
        0.0,
        &[0.0; 4],
        tol_pos(1e-6),
        &[],
        None,
    )
    .unwrap();
    assert_eq!(coarse, fine, "straight path must not depend on fit_tol_mm");
    assert_eq!(coarse[0].len(), planned[0].velocity.phases.len());
}

#[test]
fn pressure_advance_k_zero_is_identical_to_no_post_processor() {
    let m = line_move([0.0, 0.0, 0.0], [40.0, 0.0, 0.0], 4.0, ctx(1, 100.0)).unwrap();
    let planned = fit_and_plan(std::slice::from_ref(&m));
    let start = [0.0; 4];

    let none = lower_move(
        &planned[0].geometry,
        &planned[0].velocity,
        0.0,
        &start,
        FIT_TOL,
        &[],
        None,
    )
    .unwrap();
    let zero = lower_move(
        &planned[0].geometry,
        &planned[0].velocity,
        0.0,
        &start,
        FIT_TOL,
        &linear_pa_chains(3, 0.0),
        None,
    )
    .unwrap();

    for axis in 0..4 {
        assert_eq!(
            extract_bezier_pieces(&none.axes[axis]),
            extract_bezier_pieces(&zero.axes[axis]),
            "axis {axis} differs between no-PP and k=0"
        );
    }
}

fn wire_degree(coeffs: &[f64], h: f64) -> usize {
    // The degree the wire recovers: Chebyshev truncation at enqueue's
    // noise-level budgets (1e-6 mm / 1e-3 mm/s / 0.1 mm/s²).
    let cheb = nurbs::chebyshev::monomial_tau_to_chebyshev(coeffs, h);
    nurbs::chebyshev::truncate_chebyshev_c2(&cheb, h, 1e-6, 1e-3, 0.1).len()
}

#[test]
fn ladder_collapses_synthetic_profiles_to_natural_degrees() {
    let h = 0.02;
    // Cruise: p = 5 + 120·τ → 2 Chebyshev coefficients.
    assert_eq!(wire_degree(&[5.0, 120.0, 0.0, 0.0, 0.0, 0.0], h), 2);
    // Trapezoid leg (constant accel): + 1500·τ² → 3.
    assert_eq!(wire_degree(&[5.0, 120.0, 1500.0, 0.0, 0.0, 0.0], h), 3);
    // Constant jerk: + 2e5·τ³ → 4.
    assert_eq!(wire_degree(&[5.0, 120.0, 1500.0, 2.0e5, 0.0, 0.0], h), 4);
}

#[test]
fn arc_member_fits_within_the_wire_degree_cap() {
    let gm = quarter_arc();
    let planned = fit_and_plan(std::slice::from_ref(&gm));
    let vm = &planned[0].velocity;
    let start = [0.0_f64; 4];
    let (axes, total_t) = lower_move_pieces(
        &planned[0].geometry,
        vm,
        0.0,
        &start,
        tol_pos(5e-3),
        &[],
        None,
    )
    .unwrap();
    for pieces in &axes[..2] {
        assert!(!pieces.is_empty());
        for p in pieces {
            assert!(
                p.coeffs.len() <= MAX_PIECE_COEFFS,
                "piece exceeds the wire cap: {} coeffs",
                p.coeffs.len()
            );
        }
    }
    // The higher-degree ladder resolves the arc without deep bisection: the
    // spans stay long relative to the move (cubic fitting needed an order of
    // magnitude more pieces at this tolerance).
    let count = axes[0].len();
    let mean_span = total_t / count as f64;
    assert!(
        mean_span > 1e-3,
        "{count} pieces over {total_t:.4}s — mean span {mean_span:.6}s suggests deep bisection"
    );
}

#[test]
fn straight_phase_pieces_carry_natural_length() {
    let m = line_move([0.0, 0.0, 0.0], [30.0, 0.0, 0.0], 0.0, straight_ctx(1)).unwrap();
    let planned = fit_and_plan(std::slice::from_ref(&m));
    let vm = &planned[0].velocity;
    assert!(!vm.phases.is_empty());
    let start = [0.0_f64; 4];
    let (axes, _t) = lower_move_pieces(
        &planned[0].geometry,
        vm,
        0.0,
        &start,
        tol_pos(1e-3),
        &[],
        None,
    )
    .unwrap();
    let has_jerk_phase = vm.phases.iter().any(|p| p.j != 0.0);
    let expect = if has_jerk_phase { 4 } else { 3 };
    for p in &axes[0] {
        assert_eq!(
            p.coeffs.len(),
            expect,
            "move-wide padding to the max phase degree"
        );
    }
}

fn wavy_transform(fade: geometry::Fade) -> geometry::SurfaceTransform {
    let (nx, ny) = (6, 6);
    let z = (0..ny)
        .flat_map(|j| {
            (0..nx)
                .map(move |i| 0.12 * libm::sin(0.8 * i as f64) + 0.09 * libm::cos(1.1 * j as f64))
        })
        .collect();
    let mesh = geometry::MeshGrid::new(0.0, 0.0, 50.0, 50.0, nx, ny, z, 0.2).unwrap();
    geometry::SurfaceTransform::new(mesh, fade)
}

fn flat_transform(height: f64) -> geometry::SurfaceTransform {
    let mesh = geometry::MeshGrid::new(0.0, 0.0, 250.0, 250.0, 2, 2, vec![height; 4], 0.2).unwrap();
    geometry::SurfaceTransform::new(mesh, geometry::Fade::new(1.0, 10.0, 0.05).unwrap())
}

fn eval_piece_any_degree(p: &BezierPiece, t: f64) -> f64 {
    let z = t - p.u_start;
    p.coeffs.iter().rev().fold(0.0, |acc, &c| acc * z + c)
}

#[test]
fn surface_warped_move_tracks_the_mesh_along_the_path() {
    let t = wavy_transform(geometry::Fade::new(1.0, 10.0, 0.0).unwrap());
    let m = line_move([10.0, 20.0, 0.2], [210.0, 120.0, 0.2], 0.0, ctx(1, 150.0)).unwrap();
    let planned = fit_and_plan(std::slice::from_ref(&m));
    assert!(!planned[0].velocity.phases.is_empty(), "straight move");

    let start = [10.0, 20.0, 0.2, 0.0];
    let warped = lower_move(
        &planned[0].geometry,
        &planned[0].velocity,
        0.0,
        &start,
        FIT_TOL,
        &[],
        Some(&t),
    )
    .unwrap();

    let x_pieces = extract_bezier_pieces(&warped.axes[0]);
    let y_pieces = extract_bezier_pieces(&warped.axes[1]);
    let z_pieces = extract_bezier_pieces(&warped.axes[2]);
    assert!(
        z_pieces.len() > planned[0].velocity.phases.len(),
        "a mesh-warped move must leave the closed-form phase path"
    );
    let eval_axis = |pieces: &[BezierPiece], time: f64| -> f64 {
        let p = pieces
            .iter()
            .find(|p| (p.u_start..=p.u_end).contains(&time))
            .expect("time inside the move");
        eval_piece_any_degree(p, time)
    };
    let t_end = warped.t_end;
    let mut worst = 0.0_f64;
    for k in 0..=200 {
        let time = t_end * k as f64 / 200.0;
        let x = eval_axis(&x_pieces, time);
        let y = eval_axis(&y_pieces, time);
        let z = eval_axis(&z_pieces, time);
        let expected = 0.2 + t.correction_at(x, y, 0.2);
        worst = worst.max((z - expected).abs());
    }
    assert!(
        worst <= 5.0 * FIT_TOL_MM,
        "warped Z drifted {worst} from the surface"
    );
}

#[test]
fn surface_warped_single_axis_move_tracks_the_mesh() {
    let z = vec![0.10, 0.00, -0.10, 0.05, 0.00, -0.05, -0.10, 0.00, 0.10];
    let mut mesh = geometry::MeshGrid::new(20.0, 20.0, 100.0, 100.0, 3, 3, z, 0.2).unwrap();
    mesh.zero_at(120.0, 120.0);
    let t = geometry::SurfaceTransform::new(mesh, geometry::Fade::new(1.0, 10.0, 0.0).unwrap());

    let m = line_move([220.0, 20.0, 0.5], [220.0, 220.0, 0.5], 0.0, ctx(1, 150.0)).unwrap();
    let planned = fit_and_plan(std::slice::from_ref(&m));
    let start = [220.0, 20.0, 0.5, 0.0];
    let warped = lower_move(
        &planned[0].geometry,
        &planned[0].velocity,
        0.0,
        &start,
        FIT_TOL,
        &[],
        Some(&t),
    )
    .unwrap();

    let y_pieces = extract_bezier_pieces(&warped.axes[1]);
    let z_pieces = extract_bezier_pieces(&warped.axes[2]);
    let eval_axis = |pieces: &[BezierPiece], time: f64| -> f64 {
        let p = pieces
            .iter()
            .find(|p| (p.u_start..=p.u_end).contains(&time))
            .expect("time inside the move");
        eval_piece_any_degree(p, time)
    };
    let mut worst = 0.0_f64;
    for k in 0..=200 {
        let time = warped.t_end * k as f64 / 200.0;
        let y = eval_axis(&y_pieces, time);
        let z = eval_axis(&z_pieces, time);
        let expected = 0.5 + t.correction_at(220.0, y, 0.5);
        worst = worst.max((z - expected).abs());
    }
    assert!(
        worst <= 5.0 * FIT_TOL_MM,
        "pure-Y warped Z drifted {worst} from the surface"
    );
}

#[test]
fn flat_surface_keeps_the_phase_path_with_a_constant_offset() {
    let t = flat_transform(0.15);
    let m = line_move([10.0, 20.0, 0.2], [210.0, 120.0, 0.2], 0.0, ctx(1, 150.0)).unwrap();
    let planned = fit_and_plan(std::slice::from_ref(&m));
    let start = [10.0, 20.0, 0.2, 0.0];

    let base = lower_move(
        &planned[0].geometry,
        &planned[0].velocity,
        0.0,
        &start,
        FIT_TOL,
        &[],
        None,
    )
    .unwrap();
    let warped = lower_move(
        &planned[0].geometry,
        &planned[0].velocity,
        0.0,
        &start,
        FIT_TOL,
        &[],
        Some(&t),
    )
    .unwrap();

    for axis in [0, 1, 3] {
        assert_eq!(
            extract_bezier_pieces(&base.axes[axis]),
            extract_bezier_pieces(&warped.axes[axis]),
            "non-Z axes unchanged"
        );
    }
    let bz = extract_bezier_pieces(&base.axes[2]);
    let wz = extract_bezier_pieces(&warped.axes[2]);
    assert_eq!(bz.len(), wz.len(), "phase path retained");
    for (b, w) in bz.iter().zip(&wz) {
        assert!((w.coeffs[0] - b.coeffs[0] - 0.15).abs() < 1e-12);
        assert_eq!(&w.coeffs[1..], &b.coeffs[1..]);
    }
}

#[test]
fn fully_faded_move_gets_exactly_the_fade_target_offset() {
    let t = wavy_transform(geometry::Fade::new(1.0, 10.0, 0.07).unwrap());
    let m = line_move([10.0, 20.0, 12.0], [210.0, 120.0, 12.0], 0.0, ctx(1, 150.0)).unwrap();
    let planned = fit_and_plan(std::slice::from_ref(&m));
    let start = [10.0, 20.0, 12.0, 0.0];

    let base = lower_move(
        &planned[0].geometry,
        &planned[0].velocity,
        0.0,
        &start,
        FIT_TOL,
        &[],
        None,
    )
    .unwrap();
    let warped = lower_move(
        &planned[0].geometry,
        &planned[0].velocity,
        0.0,
        &start,
        FIT_TOL,
        &[],
        Some(&t),
    )
    .unwrap();
    let bz = extract_bezier_pieces(&base.axes[2]);
    let wz = extract_bezier_pieces(&warped.axes[2]);
    assert_eq!(bz.len(), wz.len(), "fade-out keeps the phase path");
    for (b, w) in bz.iter().zip(&wz) {
        assert!((w.coeffs[0] - b.coeffs[0] - 0.07).abs() < 1e-12);
        assert_eq!(&w.coeffs[1..], &b.coeffs[1..]);
    }
}

/// The regression the unlimited-jerk regime used to hit: an arc's plan only
/// existed as 0.01 mm grid samples with the acceleration stepping at every
/// cell, and the quintic reconstruction amplified those steps into piece
/// cascades and accel overshoot. With the phase-built profile the lowered
/// acceleration honors the disk cap and the piece grid stays coarse.
#[test]
fn infinite_jerk_arc_lowers_from_phases_within_the_accel_cap() {
    let gm = quarter_arc_with_jerk(f64::INFINITY);
    let planned = fit_and_plan(std::slice::from_ref(&gm));
    let vm = &planned[0].velocity;
    assert!(
        !vm.phases.is_empty(),
        "unlimited-jerk arc must carry phases"
    );

    let (axes, total_t) =
        lower_move_pieces(&planned[0].geometry, vm, 0.0, &[0.0; 4], FIT_TOL, &[], None).unwrap();

    let phase_t: f64 = vm.phases.iter().map(|p| p.dt).sum();
    assert!(
        (total_t - phase_t).abs() < 1e-9,
        "total time {total_t} is the phase sum {phase_t}"
    );
    let mut peak = 0.0_f64;
    for k in 0..=4096 {
        let t = total_t * f64::from(k) / 4096.0;
        let ax = piece_accel_at(&axes[0], t);
        let ay = piece_accel_at(&axes[1], t);
        peak = peak.max(libm::hypot(ax, ay));
    }
    assert!(
        peak <= 5000.0 + 4.0 * FIT_TOL.accel_mm_s2,
        "peak |a| = {peak} exceeds the 5000 disk cap"
    );
    for axis in 0..2 {
        assert!(
            axes[axis].len() < 128,
            "axis {axis}: {} pieces — phase-built profile should stay coarse",
            axes[axis].len()
        );
    }
}

#[test]
fn phase_profile_is_exact_and_reads_both_sides_of_an_accel_step() {
    use super::profile::{KnotSide, profile_from_phases};
    use geometry::StraightPhase;
    let phases = vec![
        StraightPhase {
            t0: 0.0,
            dt: 0.1,
            s0: 0.0,
            v0: 10.0,
            a0: 100.0,
            j: 0.0,
        },
        StraightPhase {
            t0: 0.1,
            dt: 0.2,
            s0: 1.5,
            v0: 20.0,
            a0: 0.0,
            j: 0.0,
        },
    ];
    let (profile, total_t) = profile_from_phases(&phases).unwrap();
    assert!((total_t - 0.3).abs() < 1e-12);

    let (s, v, a) = profile.state_at(0.05);
    assert!((s - (10.0 * 0.05 + 50.0 * 0.05 * 0.05)).abs() < 1e-12);
    assert!((v - 15.0).abs() < 1e-12);
    assert!((a - 100.0).abs() < 1e-12);

    let begin = profile.state_at_side(0.1, KnotSide::Begin);
    let end = profile.state_at_side(0.1, KnotSide::End);
    assert!((begin.0 - end.0).abs() < 1e-12 && (begin.1 - end.1).abs() < 1e-12);
    assert!((begin.2 - 0.0).abs() < 1e-12, "begin side reads the cruise");
    assert!((end.2 - 100.0).abs() < 1e-12, "end side reads the ramp");
}
