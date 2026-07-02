#![allow(deprecated)]

use super::*;
use crate::stream::fitter::Fitter;
use crate::stream::planner::Planner;
use crate::stream::{PlannedMove, StreamConfig};
use crossbeam_channel::unbounded;
use geometry::segment::SourceRange;
use geometry::{ChainFitConfig, MoveContext, VelocityLimits, arc_move, line_move};
use nurbs::bezier::extract_bezier_pieces;
use nurbs::eval::eval;

fn stream_config() -> StreamConfig {
    StreamConfig {
        chain: ChainFitConfig::default(),
        integration_tol: 1e-7,
        max_extrude_only_velocity_mm_s: f64::INFINITY,
        max_extrude_only_accel_mm_s2: f64::INFINITY,
        fit_tol_mm: FIT_TOL_MM,
        max_buffer_moves: 512,
        limits: VelocityLimits::try_new(300.0, 5000.0, 5.0, 100_000.0).unwrap(),
    }
}

/// Fits and plans `moves` through the real streaming `Fitter`/`Planner`
/// stages, synchronously over unbounded channels (no threads needed — see
/// `stream/fitter_tests.rs`'s `run_fitter` for the same technique).
fn fit_and_plan(moves: &[geometry::Move]) -> Vec<PlannedMove> {
    let (raw_tx, raw_rx) = unbounded();
    for m in moves.iter().cloned() {
        raw_tx
            .send(m.into())
            .expect("unbounded channel never blocks");
    }
    drop(raw_tx);

    let (fitted_tx, fitted_rx) = unbounded();
    Fitter::new(ChainFitConfig::default()).run(raw_rx, fitted_tx);

    let (planned_tx, planned_rx) = unbounded();
    Planner::new(stream_config()).run(fitted_rx, planned_tx);
    planned_rx.into_iter().collect()
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
            FIT_TOL_MM,
            &[],
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

fn eval_piece(p: &BezierPiece<f64>, t: f64) -> f64 {
    let z = t - p.u_start;
    let c = |i: usize| p.coeffs.get(i).copied().unwrap_or(0.0);
    c(0) + c(1) * z + c(2) * z * z + c(3) * z * z * z
}

fn vel_piece(p: &BezierPiece<f64>, t: f64) -> f64 {
    let z = t - p.u_start;
    let c = |i: usize| p.coeffs.get(i).copied().unwrap_or(0.0);
    c(1) + 2.0 * c(2) * z + 3.0 * c(3) * z * z
}

fn peak_accel(axes: &[Vec<BezierPiece<f64>>]) -> f64 {
    let accel = |p: &BezierPiece<f64>, t: f64| {
        let z = t - p.u_start;
        2.0 * p.coeffs[2] + 6.0 * p.coeffs[3] * z
    };
    let mut peak = 0.0_f64;
    for (px, py) in axes[0].iter().zip(&axes[1]) {
        for k in 0..=64 {
            let t = px.u_start + (px.u_end - px.u_start) * f64::from(k) / 64.0;
            peak = peak.max(accel(px, t).hypot(accel(py, t)));
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
        lower_move_pieces(&planned[0].geometry, vm, 0.0, &[0.0; 4], FIT_TOL_MM, &[]).unwrap();

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
        FIT_TOL_MM,
        &[],
    )
    .unwrap();
    let (a1, t1) = lower_move_pieces(
        &planned[1].geometry,
        &planned[1].velocity,
        t0,
        &[5.0, 0.0, 0.0, 0.0],
        FIT_TOL_MM,
        &[],
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
    apply_axis_chain(
        &mut coeffs,
        &CompiledChain {
            stages: vec![ChainStage::LinearPressureAdvance { k }],
        },
    );
    // smoothed = pos + k*vel -> c0+=k*2, c1+=k*6, c2+=k*12, c3 unchanged
    assert!((coeffs[0] - (1.0 + k * 2.0)).abs() < 1e-12);
    assert!((coeffs[1] - (2.0 + k * 6.0)).abs() < 1e-12);
    assert!((coeffs[2] - (3.0 + k * 12.0)).abs() < 1e-12);
    assert!((coeffs[3] - 4.0).abs() < 1e-12);
}

fn lower_single(m: geometry::Move, t_start: f64, start_pos: &[f64]) -> ShapedSegment {
    let planned = fit_and_plan(std::slice::from_ref(&m));
    lower_move(
        &planned[0].geometry,
        &planned[0].velocity,
        t_start,
        start_pos,
        FIT_TOL_MM,
        &[],
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
    let m = arc_move(
        [0.0, 0.0, 0.0],
        [20.0, 20.0, 0.0],
        0.0,
        20.0,
        true,
        0.0,
        ctx(3, 50.0),
    )
    .unwrap();
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
            FIT_TOL_MM,
            &[]
        ),
        Err(LoweringError::SourceMismatch)
    ));
}

fn linear_pa_chains(extruder_axis: usize, k: f64) -> Vec<CompiledChain> {
    let mut chains = vec![CompiledChain::default(); extruder_axis + 1];
    chains[extruder_axis] = CompiledChain {
        stages: (k != 0.0)
            .then_some(trajectory::ChainStage::LinearPressureAdvance { k })
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
        FIT_TOL_MM,
        &[],
    )
    .unwrap();

    let k = 0.05;
    let pa = lower_move(
        &planned[0].geometry,
        &planned[0].velocity,
        0.0,
        &start,
        FIT_TOL_MM,
        &linear_pa_chains(3, k),
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
        FIT_TOL_MM,
        &[],
    )
    .unwrap();
    let zero = lower_move(
        &planned[0].geometry,
        &planned[0].velocity,
        0.0,
        &start,
        FIT_TOL_MM,
        &linear_pa_chains(3, 0.0),
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
