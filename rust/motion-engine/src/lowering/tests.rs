use super::*;
use geometry::segment::SourceRange;
use geometry::{
    ChainFitConfig, MoveContext, VelocityConfig, VelocityLimits, arc_move, fit_chain, line_move,
    plan_velocity,
};
use nurbs::bezier::extract_bezier_pieces;
use nurbs::eval::eval;

#[test]
fn lowering_emits_coarse_pieces_above_the_sample_floor() {
    for (start, end) in [
        ([100.0, 100.0, 10.0], [100.0, 90.0, 10.0]), // the bench Y jog
        ([0.0, 0.0, 0.0], [200.0, 0.0, 0.0]),        // long fast line
        ([0.0, 0.0, 0.0], [0.5, 0.0, 0.0]),          // short line
    ] {
        let m = line_move(start, end, 0.0, ctx(1, 100.0)).unwrap();
        let outcome = fit_chain(std::slice::from_ref(&m), ChainFitConfig::default()).unwrap();
        let profile = plan_velocity(&outcome, VelocityConfig::default()).unwrap();
        let seg = lower_move(
            &outcome.moves[0],
            &profile.moves[0],
            0.0,
            &[0.0; 4],
            FIT_TOL_MM,
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
        limits: VelocityLimits::try_new(300.0, 5000.0, 5.0).unwrap(),
        source: SourceRange {
            start_line: line_no,
            end_line: line_no,
        },
    }
}

fn lower_single(m: geometry::Move, t_start: f64, start_pos: &[f64]) -> ShapedSegment {
    let outcome = fit_chain(std::slice::from_ref(&m), ChainFitConfig::default()).expect("fit");
    let profile = plan_velocity(&outcome, VelocityConfig::default()).expect("plan");
    lower_move(
        &outcome.moves[0],
        &profile.moves[0],
        t_start,
        start_pos,
        FIT_TOL_MM,
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
        assert!(
            speed <= 50.0 + 1e-3,
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
    let outcome = fit_chain(std::slice::from_ref(&m), ChainFitConfig::default()).unwrap();
    let profile = plan_velocity(&outcome, VelocityConfig::default()).unwrap();
    let other = line_move([0.0, 0.0, 0.0], [10.0, 0.0, 0.0], 0.0, ctx(99, 50.0)).unwrap();
    let other_out = fit_chain(std::slice::from_ref(&other), ChainFitConfig::default()).unwrap();
    assert!(matches!(
        lower_move(
            &other_out.moves[0],
            &profile.moves[0],
            0.0,
            &[0.0, 0.0, 0.0],
            FIT_TOL_MM
        ),
        Err(LoweringError::SourceMismatch)
    ));
}
