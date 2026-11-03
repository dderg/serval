use std::sync::Arc;

use nurbs::ScalarNurbs;
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

const REST_NOISE_MM: f64 = 1e-18;

/// A y leader resting at the origin over `[t0, t0 + rest]`, then
/// accelerating away. `noisy` adds position noise of about `REST_NOISE_MM`
/// whose velocity changes sign at a quarter, half and three quarters of the
/// rest: the rounding a fit of an exact convolution leaves on a leader at
/// rest, which only a position near zero can carry.
fn resting_y(t0: f64, rest: f64, t1: f64, noisy: bool) -> ScalarNurbs {
    let scale = if noisy {
        256.0 * REST_NOISE_MM / rest.powi(4)
    } else {
        0.0
    };
    let mut pieces = vec![BezierPiece {
        u_start: t0,
        u_end: t0 + rest,
        coeffs: vec![
            0.0,
            -3.0 / 32.0 * scale * rest.powi(3),
            11.0 / 32.0 * scale * rest.powi(2),
            -0.5 * scale * rest,
            0.25 * scale,
        ],
    }];
    if t0 + rest < t1 {
        pieces.push(BezierPiece {
            u_start: t0 + rest,
            u_end: t1,
            coeffs: vec![0.0, 0.0, 800.0, 0.0, 0.0],
        });
    }
    bezier_pieces_to_nurbs(&pieces)
}

/// Rounding noise of either sign on a resting leader is rest to the
/// follower: its partition is the one of the exactly resting leader, both
/// where the leader rests through the support and where it starts moving
/// inside it.
#[test]
fn rounding_noise_on_a_resting_leader_leaves_the_follower_partition_unchanged() {
    let (t0, t1) = (300.0, 300.02);
    let x = |coeffs: Vec<f64>| {
        bezier_pieces_to_nurbs(&[BezierPiece {
            u_start: t0,
            u_end: t1,
            coeffs,
        }])
    };
    for rest in [0.01, t1 - t0] {
        let seeds = |noisy: bool| {
            let shaped = leader_segment(
                x(vec![1.0, 20.0, 500.0]),
                resting_y(t0, rest, t1, noisy),
                t0,
                t1,
            );
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
            let noisy_velocity_changes_sign = (0..15).any(|probe| {
                let at = |probe: i32| t0 + rest * (f64::from(probe) + 0.5) / 16.0;
                let vy = |t: f64| super::axis_pva(&shaped.axes[1], t).1;
                vy(at(probe)) * vy(at(probe + 1)) < 0.0
            });
            assert_eq!(
                noisy_velocity_changes_sign, noisy,
                "fixture: only the noisy rest reverses"
            );
            sig.construction_breakpoints(&raw.axes[3], 50.0).fit_seeds
        };
        assert_eq!(
            seeds(true),
            seeds(false),
            "resting for {rest} s: noise of {REST_NOISE_MM} mm moved the follower's seeds"
        );
    }
}

#[test]
fn leader_speed_step_on_the_grid_is_seeded_as_a_construction_breakpoint() {
    let (t0, step_at, t1) = (0.0, 0.01, 0.02);
    let x = bezier_pieces_to_nurbs(&[
        BezierPiece {
            u_start: t0,
            u_end: step_at,
            coeffs: vec![0.0, 100.0],
        },
        BezierPiece {
            u_start: step_at,
            u_end: t1,
            coeffs: vec![1.0, 200.0],
        },
    ]);
    let y = bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: t0,
        u_end: t1,
        coeffs: vec![2.0],
    }]);
    let shaped = leader_segment(x, y, t0, t1);
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
    let velocity_step = sig.deriv(trajectory::next_toward(step_at, f64::INFINITY))
        - sig.deriv(trajectory::next_toward(step_at, f64::NEG_INFINITY));
    assert!(
        (velocity_step - 5.0).abs() < 1e-9,
        "fixture must step the follower velocity by 5 mm/s at the leader knot, not \
         {velocity_step}"
    );
    let breaks = sig.construction_breakpoints(&raw.axes[3], 50.0).fit_seeds;
    assert!(
        breaks.contains(&step_at),
        "the follower's velocity step at {step_at} is not a fit seed: {breaks:?}"
    );
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

fn ramp_piece(start: f64, end: f64, velocity: f64) -> BezierPiece {
    BezierPiece {
        u_start: start,
        u_end: end,
        coeffs: vec![0.0, velocity],
    }
}

fn weld_window(start: f64, end: f64) -> super::WeldWindow {
    super::WeldWindow {
        axis: 3,
        start,
        end,
        tol: crate::lowering::FitTol {
            pos_mm: 0.005,
            accel_mm_s2: 50.0,
        },
    }
}

#[test]
fn a_weld_that_would_bridge_a_velocity_step_is_refused() {
    let pieces = [ramp_piece(0.0, 0.12, 1.7)];
    let weld = super::SeamWeld::spanning(weld_window(0.0, 0.12), (1.0, -233.0), 0.0, 1.0, &pieces);
    assert!(
        matches!(weld, Err(crate::types::PostProcessError::SeamWeld { .. })),
        "a 234.7 mm/s velocity step was welded silently"
    );
}

#[test]
fn a_weld_that_would_shift_the_track_past_the_tolerance_is_refused() {
    let pieces = [ramp_piece(0.0, 0.01, 2.0)];
    let weld = super::SeamWeld::spanning(weld_window(0.0, 0.01), (1.01, 2.0), 0.0, 1.0, &pieces);
    assert!(matches!(
        weld,
        Err(crate::types::PostProcessError::SeamWeld { .. })
    ));
}

#[test]
fn a_fit_residual_is_welded_onto_the_emitted_state() {
    let mut pieces = [ramp_piece(0.0, 0.01, 2.0)];
    let emitted = (1.0 + 1e-9, 2.0 + 1e-6);
    let weld = super::SeamWeld::spanning(weld_window(0.0, 0.01), emitted, 0.0, 1.0, &pieces)
        .expect("a residual well inside the tolerance welds");
    weld.apply(&mut pieces[0]);
    assert!((1.0 + pieces[0].coeffs[0] - emitted.0).abs() <= 1e-15);
    assert!((pieces[0].coeffs[1] - emitted.1).abs() <= 1e-12);
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

#[test]
fn a_lead_with_no_ratio_lays_one_flat_span() {
    for (lead, idle) in [(5e-12, 3e-5), (3e-5, 1e-5)] {
        let state = laid_lead(lead, idle, 0.0);
        assert_eq!(
            state
                .spans
                .iter()
                .map(|span| (span.r0, span.r1))
                .collect::<Vec<_>>(),
            vec![(0.0, 0.0)],
            "lead {lead} mm, idle {idle} mm: a lead that lays nothing needs no valley"
        );
    }
}

const DENSE_SEGMENT: (f64, f64) = (11.042_946_570_120_918, 11.049_146_570_120_918);
const DENSE_ODOMETER_START: f64 = 1_825.258_274_639_182_7;
const DENSE_LEADERS: [usize; 3] = [0, 1, 2];
const DENSE_EXTRUSION_RATIO: f64 = 0.033_697_061_385_477_234;
const CLEAR_ZERO_RATIO_TAIL_MM: f64 = 2e-6;
const SUB_FLOOR_ZERO_RATIO_TAIL_MM: f64 = 5e-7;

fn dense_corner_with_an_idle_z() -> ContinuousSegment {
    let (t0, t1) = DENSE_SEGMENT;
    cusp_segment(
        vec![120.1, -56.1, 3_328.9],
        vec![150.9, -29.3, -5_671.8],
        t0,
        t1,
    )
}

fn dense_odometer(shaped: &ContinuousSegment) -> super::Odometer {
    super::Odometer {
        start: DENSE_ODOMETER_START,
        length: super::leader_arc_length(shaped, &DENSE_LEADERS),
    }
}

fn extrusion_stopping_before_the_end(
    shaped: &ContinuousSegment,
    zero_ratio_tail_mm: f64,
) -> FollowerState {
    let odometer = dense_odometer(shaped);
    let mut state = FollowerState::default();
    state.push_span(
        odometer.start + odometer.length - zero_ratio_tail_mm,
        DENSE_EXTRUSION_RATIO,
        DENSE_EXTRUSION_RATIO,
    );
    state.push_span(1.0, 0.0, 0.0);
    state
}

fn fit_dense_extrusion_stopping_before_the_end(
    zero_ratio_tail_mm: f64,
) -> Result<super::SourceProjection, crate::types::PostProcessError> {
    let shaped = dense_corner_with_an_idle_z();
    let state = extrusion_stopping_before_the_end(&shaped, zero_ratio_tail_mm);
    let odometer = dense_odometer(&shaped);
    let source = super::FollowerSource {
        shaped: &shaped,
        raw: &shaped,
        axis: 3,
        leaders: &DENSE_LEADERS,
        state: &state,
        s_start: odometer.start,
        s_length: odometer.length,
    };
    let fit_tol = crate::lowering::FitTol {
        pos_mm: 0.0025,
        accel_mm_s2: 50.0,
    };
    super::fit_source_projection(source, fit_tol, None)
}

#[test]
fn a_leader_axis_that_never_moves_seeds_no_velocity_zeros() {
    let shaped = dense_corner_with_an_idle_z();
    let state = extrusion_stopping_before_the_end(&shaped, CLEAR_ZERO_RATIO_TAIL_MM);
    let sig = FollowerSignal::new(
        &shaped,
        &shaped,
        3,
        &DENSE_LEADERS,
        &state,
        dense_odometer(&shaped),
        0.0,
    );
    let breaks = sig.construction_breakpoints(&shaped.axes[3], 50.0);
    assert_eq!(
        breaks.fit_seeds, breaks.semantic,
        "x and y never stop and z never moves, so no velocity component has a zero to seed"
    );
}

#[test]
fn a_ratio_step_just_before_the_segment_end_fits_on_its_own_seed() {
    fit_dense_extrusion_stopping_before_the_end(CLEAR_ZERO_RATIO_TAIL_MM)
        .expect("the ratio step is a floor clear of the segment end, so it keeps its own seed");
}

#[test]
fn a_ratio_step_within_a_floor_of_the_segment_end_moves_onto_it() {
    fit_dense_extrusion_stopping_before_the_end(SUB_FLOOR_ZERO_RATIO_TAIL_MM).expect(
        "moving the ratio step onto the segment end shifts the extruder by far less than the \
         position tolerance",
    );
}

const SEAM_SPLIT_S: f64 = 1e-8;

fn leader_with_an_acceleration_step(
    t0: f64,
    knot: f64,
    t1: f64,
    (velocity, acceleration, acceleration_step): (f64, f64, f64),
) -> nurbs::ScalarNurbs {
    let before = vec![0.0, velocity, 0.5 * acceleration];
    let (position, knot_velocity, _) = super::polynomial_pva(&before, knot - t0);
    bezier_pieces_to_nurbs(&[
        BezierPiece {
            u_start: t0,
            u_end: knot,
            coeffs: before,
        },
        BezierPiece {
            u_start: knot,
            u_end: t1,
            coeffs: vec![
                position,
                knot_velocity,
                0.5 * (acceleration + acceleration_step),
            ],
        },
    ])
}

#[test]
fn advance_steps_at_leader_knots_closer_than_the_floor_plan() {
    let (t0, t1) = (0.0, 0.02);
    let knot = 0.01;
    let shaped = leader_segment(
        leader_with_an_acceleration_step(t0, knot, t1, (100.0, 1_000.0, 2_000.0)),
        leader_with_an_acceleration_step(t0, knot + SEAM_SPLIT_S, t1, (50.0, -500.0, -1_500.0)),
        t0,
        t1,
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
        pos_mm: 0.0025,
        accel_mm_s2: 50.0,
    };
    let stage = trajectory::ChainStage::NonlinearAdvance(trajectory::NonlinearAdvance {
        model: trajectory::AdvanceModel::Tanh,
        linear_advance: 0.02,
        nonlinear_offset: 0.03,
        linearization_velocity: 1.0,
    });
    super::fit_source_projection(source, fit_tol, Some(&stage)).expect(
        "the advance turns both leader acceleration steps into velocity steps a sub-floor \
         interval apart, which one knot carries within the position tolerance",
    );
}

const ROUNDING_SEAM: f64 = 0.005;
const SHORT_LEADER_PIECE_S: f64 = 1e-6;
const SEAM_VELOCITY_ROUNDING_MM_S: f64 = 3e-8;

fn leader_with_a_short_piece_after_a_rounding_velocity_seam(t0: f64, t1: f64) -> ScalarNurbs {
    let knots = [t0, ROUNDING_SEAM, ROUNDING_SEAM + SHORT_LEADER_PIECE_S, t1];
    let accelerations = [0.0, 4_000.0, 0.0];
    let seam_velocity_mismatches = [0.0, SEAM_VELOCITY_ROUNDING_MM_S, 0.0];
    let (mut position, mut velocity) = (0.0, 40.0);
    let pieces: Vec<BezierPiece> = knots
        .windows(2)
        .zip(accelerations.into_iter().zip(seam_velocity_mismatches))
        .map(|(span, (acceleration, mismatch))| {
            let coeffs = vec![position, velocity + mismatch, 0.5 * acceleration];
            (position, velocity, _) = super::polynomial_pva(&coeffs, span[1] - span[0]);
            BezierPiece {
                u_start: span[0],
                u_end: span[1],
                coeffs,
            }
        })
        .collect();
    bezier_pieces_to_nurbs(&pieces)
}

#[test]
fn an_advanced_follower_source_holds_the_tolerance_past_a_leader_seam_velocity_rounding() {
    let (t0, t1) = (0.0, 0.01);
    let shaped = leader_segment(
        leader_with_a_short_piece_after_a_rounding_velocity_seam(t0, t1),
        bezier_pieces_to_nurbs(&[BezierPiece {
            u_start: t0,
            u_end: t1,
            coeffs: vec![0.0, 30.0],
        }]),
        t0,
        t1,
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
        pos_mm: 0.000_125,
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
        steps: &fitted.window_advance_steps,
    };
    for piece in nurbs::bezier::extract_bezier_pieces(&fitted.track) {
        let duration = piece.u_end - piece.u_start;
        for sample in 0..64 {
            let tau = duration * (f64::from(sample) + 0.5) / 64.0;
            let fitted_acceleration = super::polynomial_pva(&piece.coeffs, tau).2;
            let signal_acceleration = advanced.eval_pva(piece.u_start + tau).2;
            assert!(
                (fitted_acceleration - signal_acceleration).abs() <= fit_tol.accel_mm_s2,
                "piece [{}, {}] accelerates at {fitted_acceleration} mm/s² where the advanced \
                 signal does at {signal_acceleration}, {tau} s in",
                piece.u_start,
                piece.u_end
            );
        }
    }
}

fn leader_with_a_knot(knot: f64, end: f64) -> ContinuousSegment {
    let before = [0.0, 80.0, 3_000.0, -400_000.0];
    let at_knot = |order: usize| {
        before
            .iter()
            .enumerate()
            .skip(order)
            .map(|(power, &coefficient)| {
                let falling = ((power - order + 1)..=power).product::<usize>() as f64;
                falling * coefficient * knot.powi((power - order) as i32)
            })
            .sum::<f64>()
    };
    let after = vec![
        at_knot(0),
        at_knot(1),
        (at_knot(2) + 250.0) / 2.0,
        (at_knot(3) + 900_000.0) / 6.0,
    ];
    let x = bezier_pieces_to_nurbs(&[
        BezierPiece {
            u_start: 0.0,
            u_end: knot,
            coeffs: before.to_vec(),
        },
        BezierPiece {
            u_start: knot,
            u_end: end,
            coeffs: after,
        },
    ]);
    let y = bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: 0.0,
        u_end: end,
        coeffs: vec![0.0, 30.0, -1_500.0, 60_000.0],
    }]);
    leader_segment(x, y, 0.0, end)
}

#[test]
fn a_follower_reports_its_steps_across_a_leader_knot() {
    let knot = 0.008;
    let shaped = leader_with_a_knot(knot, 0.02);
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
    assert_eq!(sig.interior_knots(0.001, 0.019), [knot]);
    let [before, after] = sig.eval_pva_across(knot);
    for (side, across) in [(f64::NEG_INFINITY, before), (f64::INFINITY, after)] {
        let direct = sig.eval_pva(trajectory::next_toward(knot, side));
        for (across, direct) in [
            (across.0, direct.0),
            (across.1, direct.1),
            (across.2, direct.2),
        ] {
            assert!(
                (across - direct).abs() <= 1e-9 * direct.abs().max(1.0),
                "evaluating across the knot reads {across}, one ulp to its side reads {direct}"
            );
        }
    }
    let acceleration_step = (after.2 - before.2).abs();
    let jerk_step = (sig.jerk(trajectory::next_toward(knot, f64::INFINITY))
        - sig.jerk(trajectory::next_toward(knot, f64::NEG_INFINITY)))
    .abs();
    assert!(
        acceleration_step > 1.0 && jerk_step > 1_000.0,
        "test premise: the follower steps across the knot ({acceleration_step} mm/s², \
         {jerk_step} mm/s³)"
    );
    let bound = sig.knot_jump(knot);
    assert!(
        acceleration_step <= bound.acceleration && jerk_step <= bound.jerk,
        "the follower steps by {acceleration_step} mm/s² and {jerk_step} mm/s³ across the \
         knot, past its reported bound of {} mm/s² and {} mm/s³",
        bound.acceleration,
        bound.jerk
    );
}
