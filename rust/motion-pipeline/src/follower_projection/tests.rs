use std::sync::Arc;

use nurbs::bezier::{BezierPiece, bezier_pieces_to_nurbs};
use trajectory::{ContinuousAxis, ContinuousSegment};

use super::{FollowerSignal, FollowerState, integrate};

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
    let sig = FollowerSignal::new(&shaped, &raw, 3, &[0, 1], &state, 0.0, 0.0);
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
    let held = Arc::new(curve(vec![0.0]));
    ContinuousSegment {
        axes: Arc::from([
            ContinuousAxis::Spline(Arc::new(curve(x_coeffs))),
            ContinuousAxis::Spline(Arc::new(curve(y_coeffs))),
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
            &shaped,
            &shaped,
            3,
            &[0, 1],
            &state,
            0.0,
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
            let actual = nurbs::eval::eval(&acceleration.as_view(), t);
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
    let sig = FollowerSignal::new(&shaped, &raw, 3, &[0, 1], &state, 0.0, 0.0);
    let breaks = sig.construction_breakpoints(&raw.axes[3]).fit_seeds;
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
    let sig = FollowerSignal::new(&shaped, &raw, 3, &[0, 1], &state, 0.0, 0.0);
    let breaks = sig.construction_breakpoints(&raw.axes[3]).fit_seeds;
    assert_eq!(
        super::axis_pva(&shaped.axes[0], t0).1,
        0.0,
        "fixture must place the vx zero exactly on the support start"
    );
    let at_start = breaks.iter().filter(|&&t| (t - t0).abs() <= 1e-12).count();
    assert_eq!(at_start, 1, "endpoint root duplicated in {breaks:?}");
}
