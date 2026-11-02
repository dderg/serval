use std::sync::Arc;

use nurbs::bezier::{BezierPiece, bezier_pieces_to_nurbs};
use trajectory::{ContinuousAxis, ContinuousSegment};

use super::{FollowerSignal, FollowerState, integrate};
use crate::shaper::TrackSignal;

/// Regression for the bench crash "shaper: axis 3: shaped sample is
/// non-finite": deep into a long stream the odometer's ulp exceeds
/// SPAN_MIN_LEN_MM, so a kernel-creep span's length is float-absorbed
/// (`s0 + len == s0`). The zero-width span's `e_at(s1)` was 0/0 = NaN,
/// poisoning the cumulative `e0` of every span pushed after it.
#[test]
fn float_absorbed_span_is_dropped_instead_of_poisoning_the_table() {
    let odometer = 2.0e5;
    let creep = 2.0e-12;
    assert!(
        creep > super::SPAN_MIN_LEN_MM && odometer + creep == odometer,
        "test premise: len passes the min-length gate but cannot advance \
         the odometer"
    );

    let mut state = FollowerState {
        s_ingested_end: odometer,
        ..FollowerState::default()
    };
    state.push_span(10.0, 0.05, 0.05);
    state.push_span(creep, 0.0, 0.0);
    state.push_span(10.0, 0.05, 0.05);

    assert_eq!(
        state.spans.len(),
        2,
        "the absorbed span must not enter the table"
    );
    for span in &state.spans {
        assert!(
            span.s1 > span.s0 && span.e0.is_finite(),
            "span [{}, {}] e0={} must stay finite and non-degenerate",
            span.s0,
            span.s1,
            span.e0
        );
    }
    let e_total = state.spans_e(state.s_ingested_end);
    assert!(
        (e_total - 2.0 * 10.0 * 0.05).abs() < 1e-9,
        "cumulative extrusion must survive the dropped sliver: {e_total}"
    );
}

/// Below the odometer's resolution threshold the old absolute gate still
/// applies: near zero, a span barely above SPAN_MIN_LEN_MM must be kept.
#[test]
fn representable_tiny_span_near_origin_is_kept() {
    let mut state = FollowerState::default();
    state.push_span(2.0e-12, 0.5, 0.5);
    assert_eq!(state.spans.len(), 1);
    assert!(state.spans[0].e0.is_finite());
    assert!(state.spans[0].s1 > state.spans[0].s0);
}

#[test]
fn incremental_arc_cache_matches_direct_integration_in_any_query_order() {
    let t0 = 300.0;
    let t1 = 300.02;
    let curve = |coeffs| {
        bezier_pieces_to_nurbs(&[BezierPiece {
            u_start: t0,
            u_end: t1,
            coeffs,
        }])
    };
    let held = Arc::new(curve(vec![0.0]));
    let shaped = ContinuousSegment {
        axes: Arc::from([
            ContinuousAxis::Spline(Arc::new(curve(vec![1.0, 120.0, -2_000.0, 50_000.0]))),
            ContinuousAxis::Spline(Arc::new(curve(vec![2.0, -50.0, 1_000.0]))),
            ContinuousAxis::Spline(Arc::clone(&held)),
            ContinuousAxis::Spline(held),
        ]),
        followers: Arc::from([]),
        spatial_path: true,
        t_start: t0,
        t_end: t1,
        motor_mask: 0,
        source_line: 1,
        rest_at_end: false,
    };
    let raw = shaped.clone();
    let state = FollowerState::default();
    let sig = FollowerSignal::new(
        &shaped,
        &raw,
        3,
        &[0, 1],
        &state,
        odometer_from_start(&shaped),
        0.0,
    );
    for fraction in [0.85_f64, 0.15, 0.65, 0.35, 0.95, 0.05, 0.5, 0.15] {
        let t = fraction.mul_add(t1 - t0, t0);
        let expected = integrate(&|u| sig.shaped_speed(u), t0, t);
        let got = sig.s_at(t) - sig.s_start;
        assert!(
            (got - expected).abs() <= 1e-8,
            "arc length at {t}: cache={got}, direct={expected}"
        );
    }
}

fn cusp_segment(x_coeffs: Vec<f64>, y_coeffs: Vec<f64>, t0: f64, t1: f64) -> ContinuousSegment {
    let curve = |coeffs| {
        bezier_pieces_to_nurbs(&[BezierPiece {
            u_start: t0,
            u_end: t1,
            coeffs,
        }])
    };
    leader_segment(curve(x_coeffs), curve(y_coeffs), t0, t1)
}

fn leader_segment(
    x: nurbs::ScalarNurbs,
    y: nurbs::ScalarNurbs,
    t0: f64,
    t1: f64,
) -> ContinuousSegment {
    let held = Arc::new(bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: t0,
        u_end: t1,
        coeffs: vec![0.0],
    }]));
    ContinuousSegment {
        axes: Arc::from([
            ContinuousAxis::Spline(Arc::new(x)),
            ContinuousAxis::Spline(Arc::new(y)),
            ContinuousAxis::Spline(Arc::clone(&held)),
            ContinuousAxis::Spline(held),
        ]),
        followers: Arc::from([]),
        spatial_path: true,
        t_start: t0,
        t_end: t1,
        motor_mask: 0,
        source_line: 1,
        rest_at_end: false,
    }
}

#[test]
fn follower_advance_fits_the_transformed_acceleration() {
    let shaped = cusp_segment(vec![0.0, 10.0], vec![0.0, 0.0, 500.0], 0.0, 0.02);
    let mut state = FollowerState::default();
    state.push_span(1.0, 0.05, 0.05);
    let stages = [
        trajectory::ChainStage::DerivativeGains { k1: 0.04, k2: 0.0 },
        trajectory::ChainStage::NonlinearAdvance(trajectory::NonlinearAdvance {
            model: trajectory::AdvanceModel::Tanh,
            linear_advance: 0.02,
            nonlinear_offset: 0.03,
            linearization_velocity: 1.0,
        }),
    ];
    for stage in stages {
        let fitted = super::fit_source_projection(
            super::FollowerSource {
                shaped: &shaped,
                raw: &shaped,
                axis: 3,
                leaders: &[0, 1],
                state: &state,
                s_start: 0.0,
                s_length: super::leader_arc_length(&shaped, &[0, 1]),
            },
            crate::lowering::FitTol {
                pos_mm: 5e-5,
                accel_mm_s2: 0.5,
            },
            Some(&stage),
        )
        .unwrap();
        let acceleration = nurbs::eval::derivative(&nurbs::eval::derivative(&fitted.track));
        for index in 1..200 {
            let t = f64::from(index) * 0.0001;
            let speed = (100.0 + 1e6 * t * t).sqrt();
            let velocity = 0.05 * speed;
            let raw_acceleration = 0.05 * 1e6 * t / speed;
            let jerk = 0.05 * 1e8 / speed.powi(3);
            let expected = match stage {
                trajectory::ChainStage::DerivativeGains { k1, .. } => raw_acceleration + k1 * jerk,
                trajectory::ChainStage::NonlinearAdvance(advance) => {
                    raw_acceleration
                        + advance.slope(velocity) * jerk
                        + advance.curvature(velocity) * raw_acceleration * raw_acceleration
                }
                _ => unreachable!(),
            };
            let actual = nurbs::eval::eval(&acceleration, t);
            assert!(
                (actual - expected).abs() <= 0.5,
                "advanced acceleration at {t}: {actual} != {expected}"
            );
        }
    }
}

#[test]
fn leader_velocity_sign_change_is_seeded_as_a_construction_breakpoint() {
    let (t0, t1) = (300.0, 300.02);
    let shaped = cusp_segment(vec![1.0, -16.0, 1_000.0], vec![2.0, 1e-3], t0, t1);
    let raw = shaped.clone();
    let state = FollowerState::default();
    let sig = FollowerSignal::new(
        &shaped,
        &raw,
        3,
        &[0, 1],
        &state,
        odometer_from_start(&shaped),
        0.0,
    );
    let breaks = sig.construction_breakpoints(&raw.axes[3], 50.0).fit_seeds;
    let vx = |t: f64| super::axis_pva(&shaped.axes[0], t).1;
    let seeded = breaks
        .iter()
        .copied()
        .find(|&t| t > t0 && t < t1 && vx(super::next_lower_float(t)) < 0.0 && vx(t) >= 0.0);
    let root = seeded.unwrap_or_else(|| panic!("no velocity-zero seed in {breaks:?}"));
    assert!(
        (root - 300.008).abs() <= 1e-12,
        "seed {root} is not the vx zero at 300.008"
    );
    assert!(
        breaks.len() <= 8,
        "component-zero isolation retained sampled times: {breaks:?}"
    );
}

#[test]
fn endpoint_velocity_zero_dedups_against_the_support_grid() {
    let (t0, t1) = (300.0, 300.02);
    let shaped = cusp_segment(vec![1.0, 0.0, 1_000.0], vec![2.0, 1e-3], t0, t1);
    let raw = shaped.clone();
    let state = FollowerState::default();
    let sig = FollowerSignal::new(
        &shaped,
        &raw,
        3,
        &[0, 1],
        &state,
        odometer_from_start(&shaped),
        0.0,
    );
    let breaks = sig.construction_breakpoints(&raw.axes[3], 50.0).fit_seeds;
    assert_eq!(
        super::axis_pva(&shaped.axes[0], t0).1,
        0.0,
        "fixture must place the vx zero exactly on the support start"
    );
    let at_start = breaks.iter().filter(|&&t| (t - t0).abs() <= 1e-12).count();
    assert_eq!(at_start, 1, "endpoint root duplicated in {breaks:?}");
}

#[test]
fn a_plain_follower_source_fit_opens_and_closes_on_the_signal_velocity() {
    let shaped = cusp_segment(
        vec![0.0, 80.0, -300.0],
        vec![0.0, 5.0, 100.0, -900.0],
        0.0,
        0.1,
    );
    let mut state = FollowerState::default();
    state.push_span(50.0, 0.05, 0.05);
    let source = super::FollowerSource {
        shaped: &shaped,
        raw: &shaped,
        axis: 3,
        leaders: &[0, 1],
        state: &state,
        s_start: 0.0,
        s_length: super::leader_arc_length(&shaped, &[0, 1]),
    };
    let fit_tol = crate::lowering::FitTol {
        pos_mm: 1e-4,
        accel_mm_s2: 50.0,
    };
    let fitted = super::fit_source_projection(source, fit_tol, None).unwrap();
    let sig = FollowerSignal::new(
        &shaped,
        &shaped,
        3,
        &[0, 1],
        &state,
        odometer_from_start(&shaped),
        0.0,
    );
    let pieces = nurbs::bezier::extract_bezier_pieces(&fitted.track);
    assert!(pieces.len() > 1, "fixture must fit with an interior seam");
    for piece in &pieces {
        let velocity =
            nurbs::eval::derivative(&bezier_pieces_to_nurbs(std::slice::from_ref(piece)));
        let ends = [
            (
                piece.u_start,
                trajectory::next_toward(piece.u_start, piece.u_end),
            ),
            (
                piece.u_end,
                trajectory::next_toward(piece.u_end, piece.u_start),
            ),
        ];
        for (end, inside) in ends {
            let fitted_velocity = nurbs::eval::eval(&velocity, end);
            let signal_velocity = sig.eval_pva(inside).1;
            assert!(
                (fitted_velocity - signal_velocity).abs() <= 1e-9 * signal_velocity.abs().max(1.0),
                "piece [{}, {}] meets the signal velocity {signal_velocity} with \
                 {fitted_velocity} at {end}",
                piece.u_start,
                piece.u_end
            );
        }
    }
}

#[test]
fn follower_jerk_is_the_derivative_of_its_acceleration() {
    let shaped = cusp_segment(
        vec![0.0, 0.0, 5000.0, -20000.0],
        vec![0.0, 0.0, 0.0, 3000.0],
        0.0,
        0.05,
    );
    let mut state = FollowerState::default();
    state.push_span(10.0, 0.05, 0.09);
    let sig = FollowerSignal::new(
        &shaped,
        &shaped,
        3,
        &[0, 1],
        &state,
        odometer_from_start(&shaped),
        0.0,
    );
    let step = 1e-7;
    for index in 1..50 {
        let t = 0.001 * f64::from(index);
        let expected = (sig.eval_pva(t + step).2 - sig.eval_pva(t - step).2) / (2.0 * step);
        let jerk = sig.jerk(t);
        assert!(
            (jerk - expected).abs() <= 1e-4 * expected.abs().max(1e3),
            "follower jerk at t={t} is {jerk}, its acceleration's derivative {expected}"
        );
    }
}

fn window_step(t: f64, step_mm: f64) -> super::AdvanceStep {
    super::AdvanceStep {
        t,
        velocity_before: 0.0,
        velocity_after: 0.0,
        step_mm,
    }
}

fn piece_value(pieces: &[BezierPiece], t: f64) -> f64 {
    let piece = pieces
        .iter()
        .rev()
        .find(|piece| piece.u_start <= t)
        .expect("t inside the pieces");
    let tau = t - piece.u_start;
    piece.coeffs.iter().rev().fold(0.0, |acc, &c| acc * tau + c)
}

#[test]
fn a_window_step_inside_a_piece_splits_it_and_raises_what_follows() {
    let pieces = vec![
        BezierPiece {
            u_start: 0.0,
            u_end: 1.0,
            coeffs: vec![1.0, 2.0, -3.0, 0.5],
        },
        BezierPiece {
            u_start: 1.0,
            u_end: 2.0,
            coeffs: vec![0.5, 0.5, 1.5, -0.5],
        },
    ];
    let steps = [window_step(0.25, 0.1), window_step(1.0, -0.2)];
    let mut steps_at = Vec::new();
    let stepped = super::with_window_steps(pieces.clone(), &steps, 10.0, &mut steps_at);
    assert_eq!(
        stepped
            .iter()
            .map(|piece| (piece.u_start, piece.u_end))
            .collect::<Vec<_>>(),
        vec![(0.0, 0.25), (0.25, 1.0), (1.0, 2.0)],
        "the step inside the first piece must split it at its own time"
    );
    assert_eq!(steps_at, vec![(0.25, 0.1), (1.0, -0.2)]);
    for (t, raised) in [
        (0.1, 10.0),
        (0.25, 10.1),
        (0.7, 10.1),
        (1.0, 9.9),
        (1.6, 9.9),
    ] {
        let expected = piece_value(&pieces, t) + raised;
        let got = piece_value(&stepped, t);
        assert!(
            (got - expected).abs() < 1e-12,
            "at t={t} the stepped pieces read {got}, the source raised by {raised} {expected}"
        );
    }
}

/// The advanced source is fitted to the follower signal with the advance
/// applied and its in-window steps taken out, which leaves it
/// velocity-continuous. Its pieces must open and close on that signal's
/// own velocity, as the plain source's do, so the emission never welds a
/// fit residual into the stretch after a seam.
#[test]
fn an_advanced_follower_source_fit_opens_and_closes_on_the_signal_velocity() {
    let shaped = cusp_segment(
        vec![0.0, 80.0, -300.0],
        vec![0.0, 5.0, 100.0, -900.0],
        0.0,
        0.1,
    );
    let mut state = FollowerState::default();
    state.push_span(50.0, 0.05, 0.05);
    let source = super::FollowerSource {
        shaped: &shaped,
        raw: &shaped,
        axis: 3,
        leaders: &[0, 1],
        state: &state,
        s_start: 0.0,
        s_length: super::leader_arc_length(&shaped, &[0, 1]),
    };
    let fit_tol = crate::lowering::FitTol {
        pos_mm: 1e-4,
        accel_mm_s2: 50.0,
    };
    let stage = trajectory::ChainStage::NonlinearAdvance(trajectory::NonlinearAdvance {
        model: trajectory::AdvanceModel::Tanh,
        linear_advance: 0.02,
        nonlinear_offset: 0.03,
        linearization_velocity: 1.0,
    });
    let fitted = super::fit_source_projection(source, fit_tol, Some(&stage)).unwrap();
    let sig = FollowerSignal::new(
        &shaped,
        &shaped,
        3,
        &[0, 1],
        &state,
        odometer_from_start(&shaped),
        0.0,
    );
    let advanced = super::AdvancedFollowerSignal {
        source: &sig,
        stage: &stage,
        steps: &[],
    };
    let pieces = nurbs::bezier::extract_bezier_pieces(&fitted.track);
    assert!(pieces.len() > 1, "fixture must fit with an interior seam");
    for piece in &pieces {
        let velocity =
            nurbs::eval::derivative(&bezier_pieces_to_nurbs(std::slice::from_ref(piece)));
        for (end, inside) in [
            (
                piece.u_start,
                trajectory::next_toward(piece.u_start, piece.u_end),
            ),
            (
                piece.u_end,
                trajectory::next_toward(piece.u_end, piece.u_start),
            ),
        ] {
            let fitted_velocity = nurbs::eval::eval(&velocity, end);
            let signal_velocity = advanced.eval_pva(inside).1;
            assert!(
                (fitted_velocity - signal_velocity).abs() <= 1e-9 * signal_velocity.abs().max(1.0),
                "piece [{}, {}] meets the advanced signal velocity {signal_velocity} with \
                 {fitted_velocity} at {end}",
                piece.u_start,
                piece.u_end
            );
        }
    }
}

const DRAIN_WINDOW: (f64, f64) = (0.388_333_333_333_333_36, 0.400_833_333_333_333_4);
const DRAIN_REVERSAL: f64 = 0.393_333_333_333_333_4;

fn drain_reversal_leader() -> ContinuousSegment {
    let (t0, t1) = DRAIN_WINDOW;
    let (decelerating_from, ramping_from) = (0.390_833_333_333_333_4, 0.398_333_333_333_333_4);
    let x = nurbs::ScalarNurbs::try_new(
        3,
        vec![
            t0,
            t0,
            t0,
            t0,
            decelerating_from,
            decelerating_from,
            decelerating_from,
            ramping_from,
            ramping_from,
            ramping_from,
            t1,
            t1,
            t1,
            t1,
        ],
        vec![
            10.0,
            10.0,
            10.0,
            10.0,
            10.0,
            9.999_560_546_875_001,
            10.001_318_359_375,
            10.001_904_296_875_002,
            10.002_777_777_777_78,
            10.004_166_666_666_668,
        ],
    )
    .expect("drain leader is a valid cubic");
    let held = bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: t0,
        u_end: t1,
        coeffs: vec![0.0],
    }]);
    leader_segment(x, held, t0, t1)
}

fn extruding_state() -> FollowerState {
    let mut state = FollowerState::default();
    state.push_span(1.0, 10.0, 10.0);
    state
}

#[test]
fn a_leader_reversal_seed_sits_on_the_signal_acceleration_flip() {
    let shaped = drain_reversal_leader();
    let state = extruding_state();
    let sig = FollowerSignal::new(
        &shaped,
        &shaped,
        3,
        &[0, 1],
        &state,
        odometer_from_start(&shaped),
        0.0,
    );
    let seeds = sig
        .construction_breakpoints(&shaped.axes[3], 50.0)
        .fit_seeds;
    let seed = seeds
        .iter()
        .copied()
        .min_by(|left, right| {
            (left - DRAIN_REVERSAL)
                .abs()
                .total_cmp(&(right - DRAIN_REVERSAL).abs())
        })
        .expect("the window is seeded");
    let before = sig.eval_pva(super::next_lower_float(seed)).2;
    let after = sig
        .eval_pva(trajectory::next_toward(seed, DRAIN_WINDOW.1))
        .2;
    assert!(
        before < 0.0 && after > 0.0,
        "seed {seed} reads acceleration {before} before and {after} after it, so the \
         reversal is not on the seed"
    );
}

#[test]
fn a_plain_follower_source_fits_through_a_leader_reversal() {
    let shaped = drain_reversal_leader();
    let state = extruding_state();
    let source = super::FollowerSource {
        shaped: &shaped,
        raw: &shaped,
        axis: 3,
        leaders: &[0, 1],
        state: &state,
        s_start: 0.0,
        s_length: super::leader_arc_length(&shaped, &[0, 1]),
    };
    let fit_tol = crate::lowering::FitTol {
        pos_mm: 1e-3,
        accel_mm_s2: 50.0,
    };
    super::fit_source_projection(source, fit_tol, None)
        .unwrap_or_else(|error| panic!("reversal fit failed: {error:?}"));
}

#[test]
#[should_panic(expected = "beyond the")]
fn an_ingested_length_off_by_more_than_leaf_reordering_panics() {
    let shaped = cusp_segment(vec![0.0, 10.0], vec![0.0, 0.0, 500.0], 0.0, 0.02);
    let state = FollowerState::default();
    let mut odometer = odometer_from_start(&shaped);
    odometer.length += 1e-9;
    FollowerSignal::new(&shaped, &shaped, 3, &[0, 1], &state, odometer, 0.0);
}

#[test]
#[should_panic(expected = "stand still")]
fn still_leaders_under_an_ingested_length_panic() {
    let shaped = cusp_segment(vec![1.0], vec![2.0], 0.0, 0.02);
    let state = FollowerState::default();
    let odometer = super::Odometer {
        start: 0.0,
        length: 1e-12,
    };
    FollowerSignal::new(&shaped, &shaped, 3, &[0, 1], &state, odometer, 0.0);
}

#[test]
fn a_rescaled_odometer_carries_its_scale_into_every_derivative() {
    let shaped = cusp_segment(vec![0.0, 10.0], vec![0.0, 0.0, 500.0], 0.0, 0.02);
    let mut state = FollowerState::default();
    state.push_span(1.0, 0.05, 0.09);
    let mut sig = FollowerSignal::new(
        &shaped,
        &shaped,
        3,
        &[0, 1],
        &state,
        odometer_from_start(&shaped),
        0.0,
    );
    sig.odometer_scale = 0.5;
    let step = 1e-5;
    let central = |before: f64, after: f64| (after - before) / (2.0 * step);
    let assert_close = |label: &str, t: f64, exact: f64, difference: f64| {
        assert!(
            (exact - difference).abs() <= 1e-4 * exact.abs().max(1.0),
            "{label} at {t}: {exact} but the central difference is {difference}"
        );
    };
    for index in 1..20 {
        let t = f64::from(index) * 0.001;
        let (_, velocity, acceleration) = sig.eval_pva(t);
        let before = sig.eval_pva(t - step);
        let after = sig.eval_pva(t + step);
        assert_close("velocity", t, velocity, central(before.0, after.0));
        assert_close("acceleration", t, acceleration, central(before.1, after.1));
        assert_close("jerk", t, sig.jerk(t), central(before.2, after.2));
        assert_close("deriv", t, sig.deriv(t), velocity);
        assert_close("second_deriv", t, sig.second_deriv(t), acceleration);
    }
}

fn odometer_from_start(shaped: &ContinuousSegment) -> super::Odometer {
    super::Odometer {
        start: 0.0,
        length: super::leader_arc_length(shaped, &[0, 1]),
    }
}

const SEAM_PIECE_S: f64 = 0.01;

fn velocity_seam_segment(pieces: usize, seam_velocity_jump: f64) -> ContinuousSegment {
    let knot = |index: usize| 2.0 + index as f64 * SEAM_PIECE_S;
    let (t0, t1) = (knot(0), knot(pieces));
    let mut coeffs = vec![150.0, 100.0, 2_000.0, -30_000.0];
    let mut x_pieces = Vec::with_capacity(pieces);
    for index in 0..pieces {
        let (u_start, u_end) = (knot(index), knot(index + 1));
        let (position, velocity, acceleration) = super::polynomial_pva(&coeffs, u_end - u_start);
        let next = vec![
            position,
            velocity + seam_velocity_jump,
            0.5 * acceleration,
            -coeffs[3],
        ];
        x_pieces.push(BezierPiece {
            u_start,
            u_end,
            coeffs: std::mem::replace(&mut coeffs, next),
        });
    }
    let x = bezier_pieces_to_nurbs(&x_pieces);
    let piece = |coeffs| {
        bezier_pieces_to_nurbs(&[BezierPiece {
            u_start: t0,
            u_end: t1,
            coeffs,
        }])
    };
    let held = Arc::new(piece(vec![0.0]));
    ContinuousSegment {
        axes: Arc::from([
            ContinuousAxis::Spline(Arc::new(x)),
            ContinuousAxis::Spline(Arc::new(piece(vec![157.0, -40.0, 3_000.0]))),
            ContinuousAxis::Spline(Arc::clone(&held)),
            ContinuousAxis::Spline(held),
        ]),
        followers: Arc::from([]),
        spatial_path: true,
        t_start: t0,
        t_end: t1,
        motor_mask: 0,
        source_line: 1,
        rest_at_end: false,
    }
}

#[test]
fn follower_odometer_measures_the_ingested_arc_across_a_velocity_seam() {
    let shaped = velocity_seam_segment(8, 1e-6);
    let state = FollowerState::default();
    let sig = FollowerSignal::new(
        &shaped,
        &shaped,
        3,
        &[0, 1],
        &state,
        odometer_from_start(&shaped),
        0.0,
    );
    let ingested = super::leader_arc_length(&shaped, &[0, 1]);
    let measured = sig.s_end() - sig.s_start;
    assert!(
        (measured - ingested).abs() <= super::span_alignment_envelope(ingested),
        "follower odometer {measured} drifted from the ingested span length {ingested}"
    );
}

#[test]
fn a_leader_acceleration_seam_is_seeded_as_a_construction_breakpoint() {
    let (t0, seam, t1) = (300.0, 300.01, 300.02);
    let x = bezier_pieces_to_nurbs(&[
        BezierPiece {
            u_start: t0,
            u_end: seam,
            coeffs: vec![0.0, 100.0, 0.0],
        },
        BezierPiece {
            u_start: seam,
            u_end: t1,
            coeffs: vec![1.0, 100.0, 2_500.0],
        },
    ]);
    let mut shaped = cusp_segment(vec![0.0, 100.0], vec![0.0, 50.0], t0, t1);
    Arc::make_mut(&mut shaped.axes)[0] = ContinuousAxis::Spline(Arc::new(x));
    let raw = shaped.clone();
    let mut state = FollowerState::default();
    state.push_span(10.0, 0.05, 0.05);
    let sig = FollowerSignal::new(
        &shaped,
        &raw,
        3,
        &[0, 1],
        &state,
        odometer_from_start(&shaped),
        0.0,
    );
    let seeds = sig.construction_breakpoints(&raw.axes[3], 50.0).fit_seeds;
    assert!(
        seeds.iter().any(|&t| (t - seam).abs() <= 1e-12),
        "the leader's acceleration jumps from 0 to 5000 mm/s² at {seam}, so the follower's \
         does too, yet the fit seeds {seeds:?} straddle it"
    );
}

fn laid_lead(lead: f64, idle: f64, lead_ratio: f64) -> FollowerState {
    let mut state = FollowerState::default();
    state.push_lead(lead, idle, lead_ratio);
    state
}

fn assert_lead_lays_its_material_continuously(lead: f64, idle: f64, lead_ratio: f64) {
    let state = laid_lead(lead, idle, lead_ratio);
    let first = state.spans.first().expect("the lead lays spans");
    let last = state.spans.last().expect("the lead lays spans");
    let context = format!("lead {lead} mm, idle {idle} mm, ratio {lead_ratio}");
    assert_eq!(
        first.r0, lead_ratio,
        "{context}: the lead must open on its ratio"
    );
    assert_eq!(
        last.r1, lead_ratio,
        "{context}: the lead must close on its ratio"
    );
    for pair in state.spans.windows(2) {
        assert_eq!(
            pair[0].r1, pair[1].r0,
            "{context}: the ratio steps at s={}",
            pair[0].s1
        );
    }
    assert!(
        state
            .spans
            .iter()
            .all(|span| span.r0 * lead_ratio >= 0.0 && span.r1 * lead_ratio >= 0.0),
        "{context}: the valley changes sign: {:?}",
        state.spans
    );
    assert!(
        (last.s1 - (lead + idle)).abs() <= 1e-15,
        "{context}: the spans cover {} mm of the joined arc",
        last.s1
    );
    let material = last.e_at(last.s1);
    assert!(
        (material - lead_ratio * lead).abs() <= 1e-15,
        "{context}: the lead lays {material} mm, its command is {}",
        lead_ratio * lead
    );
}

#[test]
fn a_lead_longer_than_its_idle_arc_dips_and_keeps_its_material() {
    assert_lead_lays_its_material_continuously(3e-5, 1e-5, 0.05);
}

#[test]
fn a_lead_shorter_than_its_idle_arc_bottoms_out_at_zero_and_keeps_its_material() {
    assert_lead_lays_its_material_continuously(2e-5, 5e-5, 0.05);
    assert_lead_lays_its_material_continuously(2e-5, 5e-5, -0.05);
    let state = laid_lead(2e-5, 5e-5, 0.05);
    assert_eq!(
        state
            .spans
            .iter()
            .map(|span| (span.r0, span.r1))
            .collect::<Vec<_>>(),
        vec![(0.05, 0.0), (0.0, 0.0), (0.0, 0.05)],
        "a lead shorter than its idle arc ramps one lead wide each way around a floor at zero"
    );
}

#[test]
fn the_valley_is_continuous_where_lead_and_idle_arcs_meet() {
    let at = |idle: f64| {
        laid_lead(4e-5, idle, 0.05)
            .spans
            .iter()
            .map(|span| (span.s1 - span.s0, span.r0, span.r1))
            .filter(|&(len, _, _)| len > 1e-12)
            .collect::<Vec<_>>()
    };
    let (below, above) = (at(4e-5 * (1.0 - 1e-9)), at(4e-5 * (1.0 + 1e-9)));
    let deepest = |spans: &[(f64, f64, f64)]| {
        spans
            .iter()
            .map(|&(_, r0, r1)| r0.min(r1))
            .fold(f64::INFINITY, f64::min)
    };
    assert!(
        (deepest(&below) - deepest(&above)).abs() <= 1e-9,
        "the valley floor jumps from {} to {} as the idle arc passes the lead",
        deepest(&below),
        deepest(&above)
    );
}
