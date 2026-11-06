use std::sync::Arc;

use nurbs::ScalarNurbs;
use nurbs::bezier::{BezierPiece, bezier_pieces_to_nurbs};
use trajectory::algos::{self, PostProcessorAlgo};
use trajectory::{CompiledChain, ContinuousAxis, ContinuousSegment, PostProcessorInstance};

use crate::lowering::AccelBudget;

use super::{FollowerSignal, FollowerState, LeaderReads, ShapedLeaders, integrate};
use crate::shaper::TrackSignal;

impl<'a> FollowerSignal<'a> {
    fn new(
        shaped: &'a ContinuousSegment,
        raw: &'a ContinuousSegment,
        axis: usize,
        leaders: &'a [usize],
        state: &'a FollowerState,
        odometer: super::Odometer,
        e_start: f64,
    ) -> Self {
        Self::riding(
            shaped,
            raw,
            axis,
            ShapedLeaders::of(shaped, leaders),
            state,
            odometer,
            e_start,
        )
    }
}

fn leader_reads(shaped: &ContinuousSegment, leaders: &[usize]) -> LeaderReads {
    let shaped_leaders = ShapedLeaders::of(shaped, leaders);
    let (_, odometer) = super::shaped_leader_arc_length(shaped, leaders, &shaped_leaders);
    shaped_leaders.into_reads(odometer)
}

fn jerk_at(sig: &FollowerSignal, t: f64) -> f64 {
    sig.ramped_read(t, &sig.path_read(t)).2
}

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

/// Leaders riding one ninth-degree piece whose speed bends hard and
/// often, as a planned corner's does: where a quadrature's error is
/// largest, and where Simpson's halves can agree by chance.
fn wiggling_leaders() -> ContinuousSegment {
    cusp_segment(
        vec![
            0.0,
            1.5,
            -7157.1,
            27_460_800.0,
            804_354_666_666.666_7,
            3_392_574_814_814_816.5,
            3.673_197_037_037_039e19,
            -2.310_463_209_876_544_6e23,
            -1.442_361_591_220_850_8e26,
            8.428_521_993_598_543e30,
        ],
        vec![
            0.0,
            0.5,
            12_896.4,
            -8_841_466.666_666_67,
            -74_469_333_333.333_37,
            5_479_490_370_370_372.0,
            -6.636_800_000_000_003e18,
            -8.843_851_851_851_857e22,
            -1.714_717_146_776_407e27,
            -6.875_010_516_689_539e29,
        ],
        0.0,
        0.15e-3,
    )
}

/// The distance the leaders cover is continuous in time between odometer
/// knots: each step of a fine scan covers what their speed says it does,
/// so no query lands on a different quadrature than its neighbour.
#[test]
fn the_odometer_is_continuous_between_its_knots() {
    let shaped = wiggling_leaders();
    let (t0, t1) = (shaped.t_start, shaped.t_end);
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
    let steps = 30_000;
    let dt = (t1 - t0) / f64::from(steps);
    let mut worst = (0.0_f64, t0);
    for index in 1..steps - 1 {
        let t = t0 + dt * f64::from(index);
        let covered = sig.s_at(t + dt) - sig.s_at(t);
        let trapezoid = 0.5 * dt * (sig.shaped_speed(t) + sig.shaped_speed(t + dt));
        let jump = (covered - trapezoid).abs();
        if jump > worst.0 {
            worst = (jump, t);
        }
    }
    assert!(
        worst.0 <= 1e-13,
        "the odometer steps by {} mm at t={}",
        worst.0,
        worst.1
    );
}

/// A leaf's odometer grows at the leaders' speed everywhere inside it:
/// a bump in the speed too narrow and too small for either quadrature to
/// see, lying between two Kronrod nodes, still splits the leaf, because the
/// follower's position must follow its velocity for its fit to close.
#[test]
fn an_odometer_leaf_grows_at_the_speed_between_its_nodes() {
    let duration = 1e-4;
    let half = 0.5 * duration;
    let bump_at = half + half * 0.5 * 0.207_784_955_007_898_5;
    let speed = |t: f64| {
        let offset = (t - bump_at) / 2e-7;
        1.0 + 1e-4 * libm::exp(-offset * offset)
    };
    let mut leaves = Vec::new();
    super::odometer_leaves(&speed, 0.0, duration, &mut |end, leaf| {
        leaves.push((end, leaf))
    });
    let mut start = 0.0;
    let half_stencil = 1e-10;
    for (end, leaf) in &leaves {
        let probes = 2000;
        for index in 1..probes {
            let t = start + (end - start) * f64::from(index) / f64::from(probes);
            let rate = (leaf.covered_until(t + half_stencil)
                - leaf.covered_until(t - half_stencil))
                / (2.0 * half_stencil);
            assert!(
                (rate - speed(t)).abs() <= 1e-5,
                "the leaf over [{start}, {end}] grows at {rate} mm/s at t={t}, the speed is {}",
                speed(t)
            );
        }
        start = *end;
    }
}

/// The distance the leaders cover is their speed's integral to well under
/// the fit's resolution everywhere along the segment.
#[test]
fn the_odometer_integrates_the_leaders_speed() {
    let shaped = wiggling_leaders();
    let (t0, t1) = (shaped.t_start, shaped.t_end);
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
    let steps = 20_000;
    let h = (t1 - t0) / f64::from(steps);
    let mut reference = 0.0;
    let mut worst = (0.0_f64, t0);
    for index in 0..steps {
        let a = t0 + h * f64::from(index);
        reference += h / 6.0
            * (sig.shaped_speed(a) + 4.0 * sig.shaped_speed(a + 0.5 * h) + sig.shaped_speed(a + h));
        let error = (sig.s_at(a + h) - sig.s_start - reference).abs();
        if error > worst.0 {
            worst = (error, a + h);
        }
    }
    assert!(
        worst.0 <= 1e-12,
        "the odometer is off its speed's integral by {} mm at t={}",
        worst.0,
        worst.1
    );
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
fn kernel_advance(stage: &trajectory::ChainStage) -> super::LeadingAdvance<'_> {
    super::LeadingAdvance {
        stage,
        smoothing: super::AdvanceSmoothing::RatioRampAndKernel,
        instantaneous_corner_velocity: trajectory::DEFAULT_INSTANTANEOUS_CORNER_VELOCITY_MM_S,
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
                reads: &leader_reads(&shaped, &[0, 1]),
                state: &state,
                s_start: 0.0,
                s_length: super::leader_arc_length(&shaped, &[0, 1]),
            },
            crate::lowering::FitTol::absolute(5e-5, 0.5),
            Some(kernel_advance(&stage)),
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

/// Leaders braking to rest at the end of a segment 732 mm into the print:
/// the path distance reaches the span table's end to within its ulp some
/// 15 ns before the stop, and the speed-zero sliver begins 20 ns before
/// it. Neither is a break of the follower's signal — the sliver's speed
/// rate is the leaders' own there — and a fit piece that short reads its
/// acceleration off positions rounded at the distance's ulp, so neither
/// may seed one.
#[test]
fn a_stop_at_the_end_of_a_segment_seeds_no_break_within_rounding_of_it() {
    let (t0, t1) = (13.2, 13.22);
    let shaped = cusp_segment(vec![100.0, 20.0, -500.0], vec![50.0], t0, t1);
    let raw = shaped.clone();
    let length = super::leader_arc_length(&shaped, &[0, 1]);
    let start = 732.4;
    let mut state = FollowerState {
        s_ingested_end: start,
        ..FollowerState::default()
    };
    state.push_span(length, 0.034, 0.034);
    let sig = FollowerSignal::new(
        &shaped,
        &raw,
        3,
        &[0, 1],
        &state,
        super::Odometer { start, length },
        0.0,
    );
    let seeds = sig.construction_breakpoints(&raw.axes[3], 50.0).fit_seeds;
    assert!(
        seeds.iter().all(|&t| t >= t1 || t1 - t > 1e-6),
        "a seed within a microsecond of the stop at {t1}: {seeds:?}"
    );
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

/// The times a resting leader component is probed at are arbitrary, so its
/// rest seeds no fit piece of the follower.
#[test]
fn a_resting_leader_seeds_nothing_inside_its_rest() {
    let (t0, rest, t1) = (300.0, 0.01, 300.02);
    let x = bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: t0,
        u_end: t1,
        coeffs: vec![1.0, 20.0, 500.0],
    }]);
    let shaped = leader_segment(x, resting_y(t0, rest, t1, false), t0, t1);
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
    let seeds = sig.construction_breakpoints(&raw.axes[3], 50.0).fit_seeds;
    assert!(
        seeds.iter().all(|&seed| seed <= t0 || seed >= t0 + rest),
        "the rest over [{t0}, {}] seeded {seeds:?}",
        t0 + rest
    );
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
        assert_close("jerk", t, jerk_at(&sig, t), central(before.2, after.2));
        assert_close("deriv", t, sig.deriv(t), velocity);
        assert_close("second_deriv", t, sig.second_deriv(t), acceleration);
    }
}

/// Deep into a print the span table stands thousands of millimetres down
/// the path and a hundred into the extrusion; a segment's samples still
/// differ by their own travel, to its rounding rather than the depth's, so
/// a nanosecond fit span reads endpoints its endpoint delta agrees with.
#[test]
fn samples_deep_into_a_print_differ_by_their_own_travel() {
    let shaped = cusp_segment(vec![0.0, 10.0], vec![0.0, 0.0, 500.0], 0.0, 0.02);
    let mut state = FollowerState::default();
    state.push_span(3000.0, 0.0333, 0.0333);
    state.push_span(10.0, 0.0333, 0.05);
    let odometer = super::Odometer {
        start: 3000.0,
        length: super::leader_arc_length(&shaped, &[0, 1]),
    };
    let sig = FollowerSignal::new(&shaped, &shaped, 3, &[0, 1], &state, odometer, 0.0);
    let mut worst = (0.0_f64, 0.0);
    for index in 1..2000 {
        let t0 = f64::from(index) * 1e-5;
        let t1 = t0 + 2e-9;
        let sampled = sig.eval_pva(t1).0 - sig.eval_pva(t0).0;
        let delta = sig.position_delta((t0, 0.0), (t1, 0.0));
        let mismatch = (sampled - delta).abs();
        if mismatch > worst.0 {
            worst = (mismatch, t0);
        }
    }
    assert!(
        worst.0 <= 1e-17,
        "samples 2 ns apart at t={} differ from their travel by {} mm",
        worst.1,
        worst.0
    );
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
        tol: crate::lowering::FitTol::absolute(0.005, 50.0),
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
    let pieces = vec![ramp_piece(0.0, 0.01, 2.0)];
    let emitted = (1.0 + 1e-9, 2.0 + 1e-6);
    let weld = super::SeamWeld::spanning(weld_window(0.0, 0.01), emitted, 0.0, 1.0, &pieces)
        .expect("a residual well inside the tolerance welds");
    let pieces = weld.apply(pieces);
    assert!((1.0 + pieces[0].coeffs[0] - emitted.0).abs() <= 1e-15);
    assert!((pieces[0].coeffs[1] - emitted.1).abs() <= 1e-12);
}

/// Corpus `voron_first_layers` on the fast bench config: a 33 ns segment
/// whose fitted start sits one ulp of its 274 mm position off the emitted
/// state. Closing that ulp over 33 ns reads as 307 mm/s² of acceleration —
/// rounding, not a mismatch the weld should refuse.
#[test]
fn a_rounding_residual_welds_over_a_sliver_segment() {
    let (start, end) = (106.952_914_632_029_94, 106.952_914_665_370_61);
    let pieces = vec![ramp_piece(start, end, -0.189_501_874_886_035_84)];
    let emitted = (274.421_090_221_286_93, -0.189_501_874_877_325_78);
    let weld = super::SeamWeld::spanning(
        weld_window(start, end),
        emitted,
        0.0,
        274.421_090_221_287,
        &pieces,
    )
    .expect("a one-ulp residual welds over any window");
    let pieces = weld.apply(pieces);
    assert_eq!(274.421_090_221_287 + pieces[0].coeffs[0], emitted.0);
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
        reads: &leader_reads(&shaped, &[0, 1]),
        state: &state,
        s_start: 0.0,
        s_length: super::leader_arc_length(&shaped, &[0, 1]),
    };
    let fit_tol = crate::lowering::FitTol::absolute(1e-4, 50.0);
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
        let jerk = jerk_at(&sig, t);
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
        reads: &leader_reads(&shaped, &[0, 1]),
        state: &state,
        s_start: 0.0,
        s_length: super::leader_arc_length(&shaped, &[0, 1]),
    };
    let fit_tol = crate::lowering::FitTol::absolute(1e-4, 50.0);
    let stage = trajectory::ChainStage::NonlinearAdvance(trajectory::NonlinearAdvance {
        model: trajectory::AdvanceModel::Tanh,
        linear_advance: 0.02,
        nonlinear_offset: 0.03,
        linearization_velocity: 1.0,
    });
    let fitted =
        super::fit_source_projection(source, fit_tol, Some(kernel_advance(&stage))).unwrap();
    let sig = FollowerSignal::new(
        &shaped,
        &shaped,
        3,
        &[0, 1],
        &state,
        odometer_from_start(&shaped),
        0.0,
    );
    let advanced = super::AdvancedFollowerSignal::new(
        &sig,
        kernel_advance(&stage),
        &[],
        crate::lowering::FitTol::absolute(1e-4, 50.0),
    );
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
        reads: &leader_reads(&shaped, &[0, 1]),
        state: &state,
        s_start: 0.0,
        s_length: super::leader_arc_length(&shaped, &[0, 1]),
    };
    let fit_tol = crate::lowering::FitTol::absolute(1e-3, 50.0);
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

fn laid_rest(arcs: (f64, f64, f64), rest_arc: f64, ratios: (f64, f64)) -> FollowerState {
    let mut state = FollowerState::default();
    state.push_rest_arcs(arcs, rest_arc, ratios);
    state
}

/// `voron_first_layers` on the kernel-free fast bench config at t≈4.749:
/// the shaped axes curve into the stop after an extrusion and out of it
/// into a travel, covering 4.5 µm more than the tail's and the lead's
/// straight distances from the rest point, 1.8 µm of it before they rest.
const CURVED_TAIL_MM: f64 = 0.111;
const CURVED_IDLE_MM: f64 = 4.5e-3;
const CURVED_LEAD_MM: f64 = 0.281;
const CURVED_REST_ARC_MM: f64 = CURVED_TAIL_MM + 1.8e-3;
const CURVED_TAIL_RATIO: f64 = 0.0369;

#[test]
fn the_ratio_steps_from_the_tail_to_the_lead_where_the_leaders_rest() {
    let state = laid_rest(
        (CURVED_TAIL_MM, CURVED_IDLE_MM, CURVED_LEAD_MM),
        CURVED_REST_ARC_MM,
        (CURVED_TAIL_RATIO, 0.0),
    );
    let seams: Vec<f64> = state
        .spans
        .windows(2)
        .filter(|pair| pair[0].r1 != pair[1].r0 || pair[0].ratio_slope() != pair[1].ratio_slope())
        .map(|pair| pair[0].s1)
        .collect();
    assert_eq!(
        seams.len(),
        1,
        "the ratio or its slope changes at {seams:?}: {:?}",
        state.spans
    );
    assert!(
        (seams[0] - CURVED_REST_ARC_MM).abs() <= 1e-15,
        "the ratio steps at s={} while the leaders still move toward the rest at s={}",
        seams[0],
        CURVED_REST_ARC_MM
    );
    assert_eq!(
        state.spans[0].r0, CURVED_TAIL_RATIO,
        "the tail must open on its ratio"
    );
    let tail_material = state.spans_delta_e(0.0, 0.0, CURVED_REST_ARC_MM);
    assert!(
        (tail_material - CURVED_TAIL_RATIO * CURVED_TAIL_MM).abs() <= 1e-15,
        "the tail lays {tail_material} mm, its command is {}",
        CURVED_TAIL_RATIO * CURVED_TAIL_MM
    );
}

#[test]
fn a_tail_shorter_than_its_idle_arc_falls_to_zero_and_keeps_its_material() {
    let (tail, tail_idle, ratio) = (2e-5, 5e-5, 0.05);
    let state = laid_rest((tail, tail_idle, 0.0), tail + tail_idle, (ratio, 0.0));
    assert_eq!(
        state
            .spans
            .iter()
            .map(|span| (span.r0, span.r1))
            .collect::<Vec<_>>(),
        vec![(ratio, 0.0), (0.0, 0.0)],
        "a tail shorter than its idle arc falls to zero over twice its length"
    );
    let end = state.spans.last().expect("the tail lays spans").s1;
    let material = state.spans_delta_e(0.0, 0.0, end);
    assert!(
        (material - ratio * tail).abs() <= 1e-18,
        "the tail lays {material} mm, its command is {}",
        ratio * tail
    );
}

#[test]
fn the_lead_keeps_what_the_tail_leaves_of_the_idle_arc() {
    let lead_ratio = 0.05;
    let state = laid_rest(
        (CURVED_TAIL_MM, CURVED_IDLE_MM, CURVED_LEAD_MM),
        CURVED_REST_ARC_MM,
        (CURVED_TAIL_RATIO, lead_ratio),
    );
    let end = state.spans.last().expect("the rest lays spans").s1;
    let lead_material = state.spans_delta_e(0.0, CURVED_REST_ARC_MM, end);
    assert!(
        (end - (CURVED_TAIL_MM + CURVED_IDLE_MM + CURVED_LEAD_MM)).abs() <= 1e-15,
        "the spans cover {end} mm of the segment's arc"
    );
    assert!(
        (lead_material - lead_ratio * CURVED_LEAD_MM).abs() <= 1e-15,
        "the lead lays {lead_material} mm, its command is {}",
        lead_ratio * CURVED_LEAD_MM
    );
}

#[test]
fn a_tail_too_short_for_a_span_leaves_the_idle_arc_to_the_lead() {
    let (idle, lead, lead_ratio) = (1e-5, 3e-5, 0.05);
    let rest = laid_rest((1e-13, idle, lead), 5e-6, (0.04, lead_ratio));
    let mut lead_alone = FollowerState::default();
    lead_alone.push_lead(lead, idle, lead_ratio);
    let ratios = |state: &FollowerState| {
        state
            .spans
            .iter()
            .map(|span| (span.s1 - span.s0, span.r0, span.r1))
            .collect::<Vec<_>>()
    };
    assert_eq!(ratios(&rest), ratios(&lead_alone));
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
        reads: &leader_reads(&shaped, &DENSE_LEADERS),
        state: &state,
        s_start: odometer.start,
        s_length: odometer.length,
    };
    let fit_tol = crate::lowering::FitTol::absolute(0.0025, 50.0);
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
        reads: &leader_reads(&shaped, &[0, 1]),
        state: &state,
        s_start: 0.0,
        s_length: super::leader_arc_length(&shaped, &[0, 1]),
    };
    let fit_tol = crate::lowering::FitTol::absolute(0.0025, 50.0);
    let stage = trajectory::ChainStage::NonlinearAdvance(trajectory::NonlinearAdvance {
        model: trajectory::AdvanceModel::Tanh,
        linear_advance: 0.02,
        nonlinear_offset: 0.03,
        linearization_velocity: 1.0,
    });
    super::fit_source_projection(source, fit_tol, Some(kernel_advance(&stage))).expect(
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
        reads: &leader_reads(&shaped, &[0, 1]),
        state: &state,
        s_start: 0.0,
        s_length: super::leader_arc_length(&shaped, &[0, 1]),
    };
    let fit_tol = crate::lowering::FitTol::absolute(0.000_125, 50.0);
    let stage = trajectory::ChainStage::NonlinearAdvance(trajectory::NonlinearAdvance {
        model: trajectory::AdvanceModel::Tanh,
        linear_advance: 0.02,
        nonlinear_offset: 0.03,
        linearization_velocity: 1.0,
    });
    let fitted =
        super::fit_source_projection(source, fit_tol, Some(kernel_advance(&stage))).unwrap();
    let sig = FollowerSignal::new(
        &shaped,
        &shaped,
        3,
        &[0, 1],
        &state,
        odometer_from_start(&shaped),
        0.0,
    );
    let advanced = super::AdvancedFollowerSignal::new(
        &sig,
        kernel_advance(&stage),
        &fitted.window_advance_steps,
        fit_tol,
    );
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

/// A long slow move that ends in a brake shorter than a millionth of the
/// move: the follower reads the leaders' acceleration only where they
/// brake, not over a stretch of the cruise sized by the whole move.
#[test]
fn a_long_cruise_ending_in_a_short_brake_keeps_its_cruise_unaccelerated() {
    let (cruise_end, brake_accel, speed, ratio) = (100.0, 37_419.9, 1.0, 0.04166);
    let end = cruise_end + speed / brake_accel;
    let x = bezier_pieces_to_nurbs(&[
        BezierPiece {
            u_start: 0.0,
            u_end: cruise_end,
            coeffs: vec![0.0, speed, 0.0],
        },
        BezierPiece {
            u_start: cruise_end,
            u_end: end,
            coeffs: vec![speed * cruise_end, speed, -0.5 * brake_accel],
        },
    ]);
    let y = bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: 0.0,
        u_end: end,
        coeffs: vec![0.0],
    }]);
    let shaped = leader_segment(x, y, 0.0, end);
    let mut state = FollowerState::default();
    state.push_span(2.0 * cruise_end, ratio, ratio);
    let sig = FollowerSignal::new(
        &shaped,
        &shaped,
        3,
        &[0, 1],
        &state,
        odometer_from_start(&shaped),
        0.0,
    );
    let brake = end - cruise_end;
    for t in [cruise_end - 4.0 * brake, cruise_end - 0.25 * brake] {
        let acceleration = sig.second_deriv(t);
        assert!(
            acceleration.abs() < 1e-6,
            "the follower accelerates at {acceleration} mm/s² while the leaders cruise at {t}"
        );
    }
    let braking = sig.second_deriv(cruise_end + 0.5 * brake);
    assert!(
        (braking + ratio * brake_accel).abs() < 1e-6 * brake_accel,
        "the follower brakes at {braking} mm/s² with the leaders"
    );
}

/// The kernel input re-projects its whole cached history every batch, each
/// time offset by a different carried position. A piece leaving rest with
/// a start velocity of rounding (here the one that broke the extruder seam
/// weld in the voron cube replay) must be corrected the same way whatever
/// that offset, or two batches shape the same history differently.
#[test]
fn a_piece_leaving_rest_is_made_monotone_whatever_its_offset() {
    let tol = crate::lowering::FitTol::absolute(3.4e-5, 50.0);
    let leaving_rest = |offset: f64| BezierPiece {
        u_start: 2.312095117306531,
        u_end: 2.320504875762939,
        coeffs: vec![
            offset,
            -5.128971379577607e-26,
            -7.911937594636813,
            1883.275503327643,
            0.0,
            0.0,
        ],
    };
    let corrected = [1.4377803477359528e-28, 0.0011216116926244801].map(|offset| {
        let mut pieces = [leaving_rest(offset)];
        super::project_monotone(3, "test input", &mut pieces, tol);
        let [piece] = pieces;
        let h = piece.u_end - piece.u_start;
        let lowest_velocity = (0..=64)
            .map(|step| super::polynomial_pva(&piece.coeffs, h * step as f64 / 64.0).1)
            .fold(f64::INFINITY, f64::min);
        assert!(
            lowest_velocity >= -tol.accel_mm_s2 * geometry::MIN_DEVICE_INTERVAL_S,
            "at offset {offset} the piece leaving rest still retracts at {lowest_velocity} mm/s"
        );
        piece.coeffs[1..].to_vec()
    });
    for (low, high) in corrected[0].iter().zip(&corrected[1]) {
        assert!(
            (low - high).abs() <= 1e-9 * low.abs().max(high.abs()).max(1.0),
            "the same piece is corrected to {:?} at one offset and {:?} at another",
            corrected[0],
            corrected[1]
        );
    }
}

/// The bowden corner of `follower_advance_chains` opens on a rest whose
/// advanced input builds its lead in one degree-11 piece; a retreat of
/// about 1.5e-6 mm where it leaves rest set off the projection, whose
/// Bernstein velocity clamped and carried forward lifted the piece
/// 1.35e-3 mm, eight times the tolerance, above the source it corrects.
#[test]
fn a_monotone_correction_stays_within_the_tolerance_of_its_source() {
    let tol = crate::lowering::FitTol::absolute(1.0 / 6000.0, 50.0);
    let source = BezierPiece {
        u_start: 2.805183445567574e-3,
        u_end: 1.2687228749551069e-2,
        coeffs: vec![
            5.438224830090298e-10,
            1.3572182146183744e-5,
            0.12360558825861026,
            -38607.17514969189,
            82453802.0107496,
            -67564349171.56878,
            32524234426637.008,
            -7139135022599966.0,
            8.164979534420874e17,
            -5.06708942693456e19,
            1.5818995095411124e21,
            -1.8103712258762171e22,
        ],
    };
    let mut pieces = [source.clone()];
    super::project_monotone(3, "test input", &mut pieces, tol);
    let h = source.u_end - source.u_start;
    let worst = (0..=512)
        .map(|step| {
            let tau = h * f64::from(step) / 512.0;
            (super::polynomial_pva(&pieces[0].coeffs, tau).0
                - super::polynomial_pva(&source.coeffs, tau).0)
                .abs()
        })
        .fold(0.0, f64::max);
    assert!(
        worst <= tol.pos_mm,
        "the monotone correction moves the piece {worst} mm from its source, past {} mm",
        tol.pos_mm
    );
}

fn ramped_advance(stage: &trajectory::ChainStage) -> super::LeadingAdvance<'_> {
    super::LeadingAdvance {
        stage,
        smoothing: super::AdvanceSmoothing::RatioRamp,
        instantaneous_corner_velocity: trajectory::DEFAULT_INSTANTANEOUS_CORNER_VELOCITY_MM_S,
    }
}

const RAMPED_ADVANCE_STAGE: trajectory::ChainStage =
    trajectory::ChainStage::DerivativeGains { k1: 0.02, k2: 0.0 };

#[test]
fn a_ratio_ramped_advance_refuses_a_ratio_step_taken_at_speed() {
    let shaped = cusp_segment(vec![0.0, 100.0], vec![0.0, 0.0], 0.0, 0.2);
    let mut state = FollowerState::default();
    state.push_span(10.0, 0.03, 0.03);
    state.push_span(10.0, 0.06, 0.06);
    let source = super::FollowerSource {
        shaped: &shaped,
        raw: &shaped,
        axis: 3,
        leaders: &[0, 1],
        reads: &leader_reads(&shaped, &[0, 1]),
        state: &state,
        s_start: 0.0,
        s_length: super::leader_arc_length(&shaped, &[0, 1]),
    };
    let fit_tol = crate::lowering::FitTol::absolute(1e-4, 50.0);
    let refused =
        super::fit_source_projection(source, fit_tol, Some(ramped_advance(&RAMPED_ADVANCE_STAGE)));
    assert!(
        matches!(
            refused,
            Err(crate::types::PostProcessError::RatioRampedPositionStep { step_mm, .. })
                if (step_mm - 0.02 * 0.03 * 100.0).abs() < 1e-9
        ),
        "an unramped 0.03 ratio step at 100 mm/s is a 0.06 mm position step through the \
         advance, got {:?}",
        refused.map(|_| ())
    );
}

const LEADER_STEP_SEAM: f64 = 300.01;

fn project_over_a_leader_acceleration_step(
    accel_step: f64,
) -> Result<super::SourceProjection, crate::types::PostProcessError> {
    let (t0, seam, t1) = (300.0, LEADER_STEP_SEAM, 300.02);
    let x = bezier_pieces_to_nurbs(&[
        BezierPiece {
            u_start: t0,
            u_end: seam,
            coeffs: vec![0.0, 100.0, 0.0],
        },
        BezierPiece {
            u_start: seam,
            u_end: t1,
            coeffs: vec![1.0, 100.0, 0.5 * accel_step],
        },
    ]);
    let mut shaped = cusp_segment(vec![0.0, 100.0], vec![0.0, 50.0], t0, t1);
    Arc::make_mut(&mut shaped.axes)[0] = ContinuousAxis::Spline(Arc::new(x));
    let mut state = FollowerState::default();
    state.push_span(10.0, 0.05, 0.05);
    let source = super::FollowerSource {
        shaped: &shaped,
        raw: &shaped,
        axis: 3,
        leaders: &[0, 1],
        reads: &leader_reads(&shaped, &[0, 1]),
        state: &state,
        s_start: 0.0,
        s_length: super::leader_arc_length(&shaped, &[0, 1]),
    };
    let fit_tol = crate::lowering::FitTol::absolute(1e-4, 50.0);
    super::fit_source_projection(source, fit_tol, Some(ramped_advance(&RAMPED_ADVANCE_STAGE)))
}

#[test]
fn a_ratio_ramped_advance_refuses_a_leader_acceleration_step_beyond_the_allowance() {
    let refused = project_over_a_leader_acceleration_step(5000.0);
    assert!(
        matches!(
            refused,
            Err(crate::types::PostProcessError::RatioRampedVelocityStep { t, .. })
                if (t - LEADER_STEP_SEAM).abs() < 1e-9
        ),
        "the leader's acceleration jumps by 5000 mm/s² at {LEADER_STEP_SEAM}, which the \
         advance turns into a 4.5 mm/s velocity step, got {:?}",
        refused.map(|_| ())
    );
}

#[test]
fn a_ratio_ramped_advance_takes_a_leader_acceleration_step_within_its_allowance() {
    let tangent_share = 100.0 / 12_500.0_f64.sqrt();
    let step = 0.02 * 0.05 * 500.0 * tangent_share;
    assert!(step > 0.4 && step < trajectory::DEFAULT_INSTANTANEOUS_CORNER_VELOCITY_MM_S);
    project_over_a_leader_acceleration_step(500.0)
        .expect("a 0.45 mm/s advance velocity step is within the extruder's allowance");
}

#[test]
fn a_follower_fit_scales_the_position_tolerance_and_keeps_its_acceleration_budget() {
    let budget = AccelBudget::PositionEquivalent {
        accel_limit_mm_s2: EXTRUDER_ACCEL_LIMIT,
    };
    let tolerance = super::follower_fit_tol(
        crate::lowering::FitTol {
            accel_budget: budget,
            ..crate::lowering::FitTol::absolute(0.005, 50.0)
        },
        0.02,
    );
    assert_eq!(tolerance.accel_budget, budget);
    assert!((tolerance.pos_mm - 1e-4).abs() < 1e-18);
    assert_eq!(tolerance.accel_mm_s2, 50.0);
}

fn e_chain(stages: &[(&str, &'static dyn PostProcessorAlgo, Vec<f64>)]) -> CompiledChain {
    CompiledChain::compile(
        &stages
            .iter()
            .map(|(name, algo, params)| PostProcessorInstance::new(name, *algo, params.clone()))
            .collect::<Vec<_>>(),
    )
    .expect("extruder chain compiles")
}

fn source_budget(chain: CompiledChain) -> AccelBudget {
    super::source_accel_budget(&chain.with_follower_limits(trajectory::FollowerLimits {
        max_velocity_mm_s: 50.0,
        max_accel_mm_s2: EXTRUDER_ACCEL_LIMIT,
    }))
}

const EXTRUDER_ACCEL_LIMIT: f64 = 20_000.0;
const LINEAR_ADVANCE: f64 = 0.02;
const SMOOTH_TIME: f64 = 0.01;
const TANH: [f64; 3] = [0.02, 0.05, 2.0];

#[test]
fn a_source_the_motor_reads_spends_its_position_budget_within_the_extruder_limit() {
    assert_eq!(
        source_budget(e_chain(&[(
            "pa",
            &algos::LinearPressureAdvance,
            vec![LINEAR_ADVANCE]
        )])),
        AccelBudget::PositionEquivalent {
            accel_limit_mm_s2: EXTRUDER_ACCEL_LIMIT
        }
    );
}

#[test]
fn a_source_a_kernel_smooths_spends_its_position_budget_past_the_extruder_limit() {
    for chain in [
        e_chain(&[
            ("pa", &algos::TanhPressureAdvance, TANH.to_vec()),
            ("st", &algos::SmoothTriangle, vec![SMOOTH_TIME]),
        ]),
        e_chain(&[
            ("pa", &algos::LinearPressureAdvance, vec![LINEAR_ADVANCE]),
            ("st", &algos::SmoothTriangle, vec![SMOOTH_TIME]),
        ]),
    ] {
        assert_eq!(
            source_budget(chain),
            AccelBudget::PositionEquivalent {
                accel_limit_mm_s2: f64::INFINITY
            }
        );
    }
}

#[test]
fn a_source_fitted_to_the_plain_projection_keeps_the_absolute_budget() {
    for chain in [
        CompiledChain::default(),
        e_chain(&[("st", &algos::SmoothTriangle, vec![SMOOTH_TIME])]),
        e_chain(&[
            ("st", &algos::SmoothTriangle, vec![SMOOTH_TIME]),
            ("pa", &algos::LinearPressureAdvance, vec![LINEAR_ADVANCE]),
        ]),
        e_chain(&[
            ("st", &algos::SmoothTriangle, vec![SMOOTH_TIME]),
            ("pa", &algos::TanhPressureAdvance, TANH.to_vec()),
        ]),
    ] {
        assert_eq!(source_budget(chain), AccelBudget::Absolute);
    }
}

/// Leaders that speed up and turn without stopping, and a follower state
/// whose odometer crosses a seam between two ratio ramps.
fn turning_leaders_on_two_ramps() -> (ContinuousSegment, FollowerState) {
    let shaped = cusp_segment(
        vec![0.0, 80.0, 300.0],
        vec![0.0, 5.0, 100.0, -900.0],
        0.0,
        0.1,
    );
    let mut state = FollowerState::default();
    let length = super::leader_arc_length(&shaped, &[0, 1]);
    state.push_span(0.4 * length, 0.05, 0.08);
    state.push_span(0.7 * length, 0.08, 0.03);
    (shaped, state)
}

/// The enclosure of `signal` over `[lo, hi]` exists and no sample of the
/// signal leaves any of its pieces' bounds, each piece's ends read from
/// inside it, as the fit piece ending or starting there holds them.
fn assert_enclosure_holds_every_sample(signal: &impl TrackSignal, (lo, hi): (f64, f64)) {
    let pieces = signal
        .enclosure_on(lo, hi, [1e-7, f64::INFINITY, 1e-2])
        .expect("a moving follower on its span table has an enclosure");
    assert!(
        pieces.len() > 1,
        "the fixture's odometer crosses a span seam"
    );
    for piece in pieces.iter() {
        for k in 0..=32 {
            let t = match k {
                0 => trajectory::next_toward(piece.lo, f64::INFINITY),
                32 => trajectory::next_toward(piece.hi, f64::NEG_INFINITY),
                _ => piece.lo + (piece.hi - piece.lo) * f64::from(k) / 32.0,
            };
            let read = signal.eval_pva(t);
            let enclosed = piece.eval_pva(t);
            for (order, (read, model)) in [
                (read.0, enclosed.0),
                (read.1, enclosed.1),
                (read.2, enclosed.2),
            ]
            .into_iter()
            .enumerate()
            {
                let bound = piece.pva[order].rounding + 1e-9 * (1.0 + read.abs());
                assert!(
                    (read - model).abs() <= bound,
                    "order {order} at t={t}: the signal reads {read}, its enclosure {model} ± {}",
                    piece.pva[order].rounding
                );
            }
        }
    }
}

#[test]
fn a_follower_enclosure_holds_every_sample_of_its_signal() {
    let (shaped, state) = turning_leaders_on_two_ramps();
    let sig = FollowerSignal::new(
        &shaped,
        &shaped,
        3,
        &[0, 1],
        &state,
        odometer_from_start(&shaped),
        0.0,
    );
    assert_enclosure_holds_every_sample(&sig, (sig.grid[0], sig.grid[sig.grid.len() - 1]));
}

/// The leaders set off from rest on a span boundary, the span behind them
/// extruding and the one ahead not, as a travel opens after an extrusion
/// that stopped: the signal reads the span ahead from the boundary on, and
/// so does its enclosure, which resolves the follower standing still
/// there rather than straddling the extrusion's acceleration.
#[test]
fn a_follower_enclosure_opening_on_a_span_boundary_reads_the_span_ahead() {
    let accel = 20_000.0;
    let shaped = cusp_segment(vec![0.0, 0.0, 0.5 * accel], vec![0.0], 0.0, 0.01);
    let behind = 2.0;
    let ahead = super::leader_arc_length(&shaped, &[0, 1]);
    let mut state = FollowerState::default();
    state.push_span(behind, 0.05, 0.05);
    state.push_span(ahead, 0.0, 0.0);
    let sig = FollowerSignal::new(
        &shaped,
        &shaped,
        3,
        &[0, 1],
        &state,
        super::Odometer {
            start: behind,
            length: ahead,
        },
        0.0,
    );
    let accuracy = [1e-7, f64::INFINITY, 1e-2];
    let pieces = sig
        .enclosure_on(0.0, 1e-4, accuracy)
        .expect("leaders moving off a span boundary have an enclosure");
    for piece in pieces.iter() {
        assert!(
            piece.pva[2].rounding <= accuracy[2],
            "the enclosure over [{}, {}] holds the acceleration only to ±{} mm/s²",
            piece.lo,
            piece.hi,
            piece.pva[2].rounding
        );
        let t = trajectory::next_toward(piece.lo, f64::INFINITY);
        let (read, model) = (sig.eval_pva(t).2, piece.eval_pva(t).2);
        assert!(
            (read - model).abs() <= piece.pva[2].rounding + 1e-9,
            "at t={t} the follower accelerates at {read} mm/s², its enclosure at {model}"
        );
    }
}

#[test]
fn an_advanced_follower_enclosure_holds_every_sample_of_its_signal() {
    let (shaped, state) = turning_leaders_on_two_ramps();
    let sig = FollowerSignal::new(
        &shaped,
        &shaped,
        3,
        &[0, 1],
        &state,
        odometer_from_start(&shaped),
        0.0,
    );
    for stage in [
        trajectory::ChainStage::DerivativeGains { k1: 0.04, k2: 0.0 },
        trajectory::ChainStage::NonlinearAdvance(trajectory::NonlinearAdvance {
            model: trajectory::AdvanceModel::Tanh,
            linear_advance: 0.02,
            nonlinear_offset: 0.03,
            linearization_velocity: 1.0,
        }),
    ] {
        let advanced = super::AdvancedFollowerSignal::new(
            &sig,
            kernel_advance(&stage),
            &[],
            crate::lowering::FitTol::absolute(1e-4, 50.0),
        );
        assert_enclosure_holds_every_sample(&advanced, (sig.grid[0], sig.grid[sig.grid.len() - 1]));
    }
}
fn near_rest_leader(velocity: [f64; 2], acceleration: [f64; 2], jerk: [f64; 2]) -> f64 {
    let states = [
        (0.0, velocity[0], acceleration[0], jerk[0]),
        (0.0, velocity[1], acceleration[1], jerk[1]),
    ];
    let speed = (velocity[0] * velocity[0] + velocity[1] * velocity[1]).sqrt();
    super::speed_second_of(&states, speed)
}

#[test]
fn a_straight_onset_reads_its_speed_curvature_where_the_speed_cubed_underflows() {
    let second = near_rest_leader([3e-121, 4e-121], [3e-102, 4e-102], [3e-90, 4e-90]);
    assert!(
        (second - 5e-90).abs() <= 1e-12 * 5e-90,
        "d²|v|/dt² along a line is the jerk's projection, got {second}"
    );
}

#[test]
fn a_turning_onset_reads_its_normal_acceleration_where_the_speed_cubed_underflows() {
    let second = near_rest_leader([1e-121, 0.0], [0.0, 1e-102], [0.0, 0.0]);
    assert!(
        (second - 1e-83).abs() <= 1e-12 * 1e-83,
        "|a⊥|²/|v| is 1e-83, got {second}"
    );
}

/// Leaders on one line through the origin, `y = x/2` but for fit noise of
/// `1e-9·t³` mm in `y`, whose velocity along the line is `x_velocity` —
/// monomial coefficients of `ẋ` in `t` over `[0, 0.02]`.
fn leaders_on_a_noisy_line(x_velocity: &[f64]) -> ContinuousSegment {
    let mut x = vec![0.0];
    x.extend(
        x_velocity
            .iter()
            .enumerate()
            .map(|(power, &c)| c / (power + 1) as f64),
    );
    let mut y: Vec<f64> = x.iter().map(|&c| 0.5 * c).collect();
    y.resize(y.len().max(4), 0.0);
    y[3] += 1e-9;
    cusp_segment(x, y, 0.0, 0.02)
}

fn resting_follower<'a>(
    shaped: &'a ContinuousSegment,
    state: &'a FollowerState,
) -> FollowerSignal<'a> {
    FollowerSignal::new(
        shaped,
        shaped,
        3,
        &[0, 1],
        state,
        odometer_from_start(shaped),
        0.0,
    )
    .with_rests(crate::lowering::FitTol::absolute(0.005, 50.0))
}

fn extruding_on(shaped: &ContinuousSegment) -> FollowerState {
    let mut state = FollowerState::default();
    state.push_span(super::leader_arc_length(shaped, &[0, 1]), 0.05, 0.05);
    state
}

/// The path speed's rate just before and just after `t`.
fn rates_across(sig: &FollowerSignal, t: f64) -> [f64; 2] {
    [f64::NEG_INFINITY, f64::INFINITY]
        .map(|side| sig.odometer_rate(trajectory::next_toward(t, side)).1)
}

#[test]
fn a_reversal_is_a_rest_whose_split_steps_the_rate_by_the_acceleration_twice() {
    let shaped = leaders_on_a_noisy_line(&[-0.5, 50.0]);
    let state = extruding_on(&shaped);
    let sig = resting_follower(&shaped, &state);
    let [rest] = sig.rests[..] else {
        panic!("one reversal, one rest: {:?}", sig.rests);
    };
    assert!(
        (rest.split - 0.01).abs() < 1e-9,
        "the leaders reverse at 0.01 s, the rest splits at {}",
        rest.split
    );
    let acceleration = 50.0 * 1.25_f64.sqrt() * sig.odometer_scale;
    let [before, after] = rates_across(&sig, rest.split);
    assert!(
        (before + acceleration).abs() < 1e-9 * acceleration
            && (after - acceleration).abs() < 1e-9 * acceleration,
        "the rate steps from {before} to {after}, not from −{acceleration} to {acceleration}"
    );
}

#[test]
fn a_stop_and_go_reads_its_rate_continuously_through_its_rest() {
    let shaped = leaders_on_a_noisy_line(&[5e-2, -10.0, 500.0]);
    let state = extruding_on(&shaped);
    let sig = resting_follower(&shaped, &state);
    let [rest] = sig.rests[..] else {
        panic!("one stop, one rest: {:?}", sig.rests);
    };
    let [before, after] = rates_across(&sig, rest.split);
    assert!(
        (after - before).abs() < 1e-6,
        "a stop-and-go steps its rate by {} at its split",
        after - before
    );
}

#[test]
fn an_advanced_follower_enclosure_holds_every_sample_through_a_reversal() {
    let shaped = leaders_on_a_noisy_line(&[-0.5, 50.0]);
    let mut state = FollowerState::default();
    let length = super::leader_arc_length(&shaped, &[0, 1]);
    state.push_span(0.4 * length, 0.05, 0.08);
    state.push_span(length, 0.08, 0.03);
    let sig = resting_follower(&shaped, &state);
    assert_eq!(sig.rests.len(), 1, "the reversal is a rest");
    let stage = trajectory::ChainStage::DerivativeGains { k1: 0.04, k2: 0.0 };
    let advanced = super::AdvancedFollowerSignal::new(
        &sig,
        kernel_advance(&stage),
        &[],
        crate::lowering::FitTol::absolute(1e-4, 50.0),
    );
    assert_enclosure_holds_every_sample(&advanced, (sig.grid[0], sig.grid[sig.grid.len() - 1]));
}

/// Planar moves through `points` lowered to the analytic segments the
/// shaper reads, with no chain shaping them.
fn unshaped_segments(points: &[[f64; 2]], feedrate: f64) -> Vec<ContinuousSegment> {
    let limits = geometry::VelocityLimits::try_new(
        300.0,
        25_000.0,
        geometry::CornerBudget::Shaped(0.05),
        f64::INFINITY,
    )
    .unwrap();
    let config = crate::types::StreamConfig {
        corner: geometry::CornerFitConfig::default(),
        integration_tol: 1e-7,
        max_extrude_only_velocity_mm_s: f64::INFINITY,
        max_extrude_only_accel_mm_s2: f64::INFINITY,
        fit_tol_mm: 0.005,
        fit_tol_accel_mm_s2: 50.0,
        max_buffer_moves: 256,
        limits,
        step_capacity: None,
    };
    let mut pipeline = crate::Pipeline::new(
        config,
        trajectory::AxisChainSet::default(),
        vec![points[0][0], points[0][1], 0.2, 0.0],
        0.0,
    );
    let crate::Pipeline { front, .. } = &mut pipeline;
    let mut segments = Vec::new();
    let mut keep = |item: crate::types::BaseItem| {
        if let crate::types::BaseItem::Seg(segment) = item {
            segments.push(segment);
        }
        true
    };
    for (line, pair) in points.windows(2).enumerate() {
        let movement = geometry::line_move(
            [pair[0][0], pair[0][1], 0.2],
            [pair[1][0], pair[1][1], 0.2],
            0.03 * libm::hypot(pair[1][0] - pair[0][0], pair[1][1] - pair[0][1]),
            geometry::MoveContext {
                extruder_axis: 3,
                feedrate_mm_s: feedrate,
                limits,
                source: geometry::segment::SourceRange {
                    start_line: line as u32,
                    end_line: line as u32,
                },
            },
        )
        .unwrap();
        assert!(front.feed(movement.into(), &mut keep));
    }
    assert!(front.finish(&mut keep));
    segments
}

/// An advance riding unshaped leaders that leave their curvature cap as a
/// square root of the time: its acceleration is exempt on the sliver where
/// the onset's jerk alone passes the fit's tolerance, under the unshaped
/// leader step's allowance, and its enclosure holds every sample of its
/// position and velocity there and of all three elsewhere.
#[test]
fn an_advance_on_an_unshaped_cap_departure_is_enclosed_under_its_onset_exemption() {
    let square = [
        [0.0, 0.0],
        [10.0, 0.0],
        [10.0, 10.0],
        [0.0, 10.0],
        [0.0, 0.0],
    ];
    let stage = trajectory::ChainStage::DerivativeGains { k1: 0.01, k2: 0.0 };
    let mut exempt = 0usize;
    for segment in unshaped_segments(&square, 200.0) {
        let state = extruding_on(&segment);
        let sig = resting_follower(&segment, &state);
        let advanced = super::AdvancedFollowerSignal::new(
            &sig,
            kernel_advance(&stage),
            &[],
            crate::lowering::FitTol::absolute(0.005, 50.0),
        );
        let support = (sig.grid[0], sig.grid[sig.grid.len() - 1]);
        let slivers = advanced
            .onset_slivers(support.0, support.1)
            .expect("analytic leaders have formulas");
        if slivers.is_empty() {
            continue;
        }
        let pieces = advanced
            .enclosure(support.0, support.1, [1e-7, f64::INFINITY, 1e-2])
            .expect("the onsets' exemption leaves the advance an enclosure");
        for piece in &pieces {
            let in_sliver = slivers
                .iter()
                .any(|sliver| sliver.from <= piece.lo && piece.hi <= sliver.to);
            assert_eq!(
                piece.unshaped_step.is_some(),
                in_sliver,
                "[{}, {}] is exempt exactly inside an onset sliver",
                piece.lo,
                piece.hi
            );
            exempt += usize::from(in_sliver);
            for k in 1..32 {
                let t = piece.lo + (piece.hi - piece.lo) * f64::from(k) / 32.0;
                let read = advanced.eval_pva(t);
                let enclosed = piece.eval_pva(t);
                for (order, (read, model)) in [
                    (read.0, enclosed.0),
                    (read.1, enclosed.1),
                    (read.2, enclosed.2),
                ]
                .into_iter()
                .enumerate()
                .filter(|&(order, _)| piece.bounds(order))
                {
                    let bound = piece.pva[order].rounding + 1e-9 * (1.0 + read.abs());
                    assert!(
                        (read - model).abs() <= bound,
                        "order {order} at t={t}: the signal reads {read}, its enclosure \
                         {model} ± {}",
                        piece.pva[order].rounding
                    );
                }
            }
        }
    }
    assert!(exempt > 0, "the square's corners leave no curvature cap");
}

/// An advance steep enough that its velocity changes across a square-root
/// onset by more than the unshaped leader step's allowance is refused
/// loudly, before any fit.
#[test]
fn an_onset_step_past_the_unshaped_leader_allowance_is_refused() {
    let square = [
        [0.0, 0.0],
        [10.0, 0.0],
        [10.0, 10.0],
        [0.0, 10.0],
        [0.0, 0.0],
    ];
    let tolerance = crate::lowering::FitTol::absolute(0.005, 50.0);
    let mut refused = 0usize;
    for segment in unshaped_segments(&square, 200.0) {
        let state = extruding_on(&segment);
        let sig = resting_follower(&segment, &state);
        let support = (sig.grid[0], sig.grid[sig.grid.len() - 1]);
        for (k1, refuses) in [(0.01, false), (0.04, true)] {
            let stage = trajectory::ChainStage::DerivativeGains { k1, k2: 0.0 };
            let advanced =
                super::AdvancedFollowerSignal::new(&sig, kernel_advance(&stage), &[], tolerance);
            let slivers = advanced
                .onset_slivers(support.0, support.1)
                .expect("analytic leaders have formulas");
            match advanced.refuse_onset_steps(3, support) {
                Err(crate::types::PostProcessError::UnshapedLeaderOnsetVelocityStep {
                    change,
                    allowance,
                    ..
                }) => {
                    assert!(
                        refuses && change > allowance,
                        "k1={k1} refused a {change} step"
                    );
                    refused += 1;
                }
                Ok(()) => assert!(
                    !refuses || slivers.is_empty(),
                    "k1={k1} admits its onsets {slivers:?}"
                ),
                Err(other) => panic!("unexpected refusal: {other}"),
            }
        }
    }
    assert!(refused > 0, "no onset was refused");
}

#[test]
fn only_a_kernel_free_advance_over_an_unshaped_leader_may_step_the_follower() {
    let compile = |instances: &[PostProcessorInstance]| CompiledChain::compile(instances).unwrap();
    let pa = || PostProcessorInstance::new("pa", &algos::LinearPressureAdvance, vec![0.04]);
    let shaped = compile(&[PostProcessorInstance::new(
        "is",
        &algos::SmoothBell,
        vec![0.01605],
    )]);
    let unshaped = CompiledChain::default();
    let allowance = |leaders: [&CompiledChain; 3], follower: CompiledChain| {
        let chains = trajectory::AxisChainSet {
            chains: vec![
                leaders[0].clone(),
                leaders[1].clone(),
                leaders[2].clone(),
                follower,
            ],
            followers: vec![(3, vec![0, 1, 2])],
        };
        super::follower_velocity_step_allowance(&chains, 3, 0.05, 50.0)
    };
    let corner_and_rounding = 2.5 + 2.0 * 50.0 * 0.05 * 0.04;
    let stepping = allowance(
        [&shaped, &shaped, &unshaped],
        compile(&[pa()]).with_instantaneous_corner_velocity(2.5),
    );
    assert!(
        (stepping - corner_and_rounding).abs() <= 1e-12,
        "a kernel-free advance over an unshaped leader may step by {stepping}, not {corner_and_rounding}"
    );
    assert_eq!(
        allowance([&shaped, &shaped, &shaped], compile(&[pa()])),
        0.0
    );
    assert_eq!(
        allowance(
            [&unshaped, &unshaped, &unshaped],
            compile(&[
                pa(),
                PostProcessorInstance::new("st", &algos::SmoothTriangle, vec![0.002]),
            ])
        ),
        0.0
    );
    assert_eq!(
        allowance([&unshaped, &unshaped, &unshaped], CompiledChain::default()),
        0.0
    );
}

#[test]
fn only_the_leaders_a_kernel_free_advance_reads_fit_acceleration_continuous_seams() {
    use crate::lowering::SeamContinuity::{Acceleration, Velocity};
    let compile = |instances: &[PostProcessorInstance]| CompiledChain::compile(instances).unwrap();
    let pa = || PostProcessorInstance::new("pa", &algos::LinearPressureAdvance, vec![0.04]);
    let shaped = || {
        compile(&[PostProcessorInstance::new(
            "is",
            &algos::SmoothBell,
            vec![0.01605],
        )])
    };
    let continuity = |follower: CompiledChain| {
        let chains = trajectory::AxisChainSet {
            chains: vec![shaped(), shaped(), shaped(), follower],
            followers: vec![(3, vec![0, 1])],
        };
        (0..3)
            .map(|leader| super::leader_seam_continuity(&chains, leader))
            .collect::<Vec<_>>()
    };
    assert_eq!(
        continuity(compile(&[pa()])),
        [Acceleration, Acceleration, Velocity]
    );
    assert_eq!(
        continuity(compile(&[
            pa(),
            PostProcessorInstance::new("st", &algos::SmoothTriangle, vec![0.002]),
        ])),
        [Velocity; 3]
    );
    assert_eq!(continuity(CompiledChain::default()), [Velocity; 3]);
}

#[test]
fn a_kernel_free_advance_holds_its_leaders_jerk_to_what_its_acceleration_tolerance_takes() {
    let compile = |instances: &[PostProcessorInstance]| CompiledChain::compile(instances).unwrap();
    let pa = || PostProcessorInstance::new("pa", &algos::LinearPressureAdvance, vec![0.04]);
    let shaped = || {
        compile(&[PostProcessorInstance::new(
            "is",
            &algos::SmoothBell,
            vec![0.01605],
        )])
    };
    let chains_with = |follower: CompiledChain| trajectory::AxisChainSet {
        chains: vec![shaped(), shaped(), shaped(), follower],
        followers: vec![(3, vec![0, 1])],
    };
    let segment = |t_start: f64, ratio: f64, spatial_path: bool| ContinuousSegment {
        axes: Arc::from([]),
        followers: Arc::from([geometry::FollowerDemand::constant(3, ratio)]),
        spatial_path,
        t_start,
        t_end: t_start + 0.1,
        motor_mask: 0,
        source_line: 0,
        rest_at_end: false,
    };
    let (printing, retract, beyond) = (
        segment(0.0, 0.04, true),
        segment(0.1, -1.0, false),
        segment(0.3, 0.08, true),
    );
    let segments = [&printing, &retract, &beyond];
    let reach = (0.0, 0.25);
    let read = super::LeaderJerkRead::of(&chains_with(compile(&[pa()])), 0);
    let budget = read.jerk_budget(50.0, &segments, reach);
    let expected = 50.0 / (0.04 * 2.0_f64.sqrt() * 0.04);
    assert!(
        (budget - expected).abs() <= 1e-9 * expected,
        "the printing ratio alone sets the budget: {budget} against {expected}"
    );
    assert_eq!(
        super::LeaderJerkRead::of(&chains_with(compile(&[pa()])), 2)
            .jerk_budget(50.0, &segments, reach),
        f64::INFINITY
    );
    let kerneled = compile(&[
        pa(),
        PostProcessorInstance::new("st", &algos::SmoothTriangle, vec![0.002]),
    ]);
    assert_eq!(
        super::LeaderJerkRead::of(&chains_with(kerneled), 0).jerk_budget(50.0, &segments, reach),
        f64::INFINITY
    );
}

/// A leader of the voron_dense_top_layers print on the p3 kernel profile
/// overshooting a stop by a third of a micrometre per second before it
/// moves on: the odometer leaf over the stop runs under that speed, so its
/// reading climbs past a span boundary, steps back below it and climbs
/// again. The table's formula changes at the boundary's crossing alone,
/// as the follower's enclosures model it.
#[test]
fn the_follower_reads_a_boundarys_formula_from_its_crossing_on() {
    let stop_end = 0.001_092_048_345_084_251;
    let stopping = [
        150.519_941_572_294_21,
        0.157_829_662_124_684_55,
        -85.829_121_122_020_41,
        -109_646.230_262_216_37,
        137_372_192.481_226_65,
        -41_303_178_465.302_92,
    ];
    let stopped_at = stopping.iter().rev().fold(0.0, |y, c| y * stop_end + c);
    let moving_end = stop_end + 0.01;
    let shaped = leader_segment(
        bezier_pieces_to_nurbs(&[BezierPiece {
            u_start: 0.0,
            u_end: moving_end,
            coeffs: vec![120.122],
        }]),
        bezier_pieces_to_nurbs(&[
            BezierPiece {
                u_start: 0.0,
                u_end: stop_end,
                coeffs: stopping.to_vec(),
            },
            BezierPiece {
                u_start: stop_end,
                u_end: moving_end,
                coeffs: vec![stopped_at, 0.0, 50.0, 0.0, 0.0, 0.0],
            },
        ]),
        0.0,
        moving_end,
    );
    let boundary = 5.842_770_8e-5;
    let (before, after) = (0.04, 0.0);
    let mut state = FollowerState::default();
    state.push_span(boundary, before, before);
    state.push_span(1.0, after, after);
    let sig = FollowerSignal::new(
        &shaped,
        &shaped,
        3,
        &[0, 1],
        &state,
        odometer_from_start(&shaped),
        0.0,
    );
    let stepping_back = (0.001_074, 0.001_085);
    assert!(
        sig.s_at(stepping_back.0) > boundary && sig.s_at(stepping_back.1) < boundary,
        "test premise: the odometer reads {} then {} across the boundary {boundary}",
        sig.s_at(stepping_back.0),
        sig.s_at(stepping_back.1)
    );
    let crossing = sig
        .ratio_crossings()
        .iter()
        .find(|&&(_, at)| at == boundary)
        .map(|&(time, _)| time)
        .expect("the odometer crosses the boundary");
    let probes = 20_000;
    for index in 0..=probes {
        let t = stop_end * f64::from(index) / f64::from(probes);
        let ratio = sig.path_read(t).laid.ratio;
        let expected = if t < crossing { before } else { after };
        assert_eq!(
            ratio,
            expected,
            "at t={t} the odometer reads {} and the table ratio {ratio}; the boundary {boundary} \
             is crossed at {crossing}",
            sig.s_at(t)
        );
    }
}

fn source_fit_on(
    state: &FollowerState,
    leading: Option<super::LeadingAdvance>,
) -> super::SourceProjection {
    let shaped = cusp_segment(
        vec![0.0, 80.0, -300.0],
        vec![0.0, 5.0, 100.0, -900.0],
        0.0,
        0.1,
    );
    let source = super::FollowerSource {
        shaped: &shaped,
        raw: &shaped,
        axis: 3,
        leaders: &[0, 1],
        reads: &leader_reads(&shaped, &[0, 1]),
        state,
        s_start: 0.0,
        s_length: super::leader_arc_length(&shaped, &[0, 1]),
    };
    super::fit_source_projection(
        source,
        crate::lowering::FitTol::absolute(1e-4, 50.0),
        leading,
    )
    .unwrap_or_else(|error| panic!("source fit failed: {error:?}"))
}

fn state_laying(spans: &[(f64, f64, f64)]) -> FollowerState {
    let mut state = FollowerState::default();
    for &(length, r0, r1) in spans {
        state.push_span(length, r0, r1);
    }
    state
}

fn assert_same_source(fitted: &super::SourceProjection, reference: &super::SourceProjection) {
    assert_eq!(fitted.track, reference.track);
    assert_eq!(fitted.reach_retracts, reference.reach_retracts);
    assert_eq!(fitted.tol_scale.to_bits(), reference.tol_scale.to_bits());
    assert_eq!(
        fitted.e_end_relative.to_bits(),
        reference.e_end_relative.to_bits()
    );
    assert_eq!(
        fitted.window_advance_steps.len(),
        reference.window_advance_steps.len()
    );
}

/// Whether the advance reads the laid table or its retract-free copy, and
/// whether the kernel after it may hold its input monotone, is decided by
/// the spans the fit reaches alone: a retract laid past them, or none,
/// fits the same bits.
#[test]
fn a_source_fit_reads_nothing_laid_past_its_reach() {
    let segment_span = (50.0, 0.05, 0.05);
    let alone = state_laying(&[segment_span]);
    let with_a_later_retract = state_laying(&[segment_span, (5.0, -0.2, -0.2), (5.0, 0.05, 0.05)]);
    let kernel_stage = trajectory::ChainStage::NonlinearAdvance(trajectory::NonlinearAdvance {
        model: trajectory::AdvanceModel::Tanh,
        linear_advance: 0.02,
        nonlinear_offset: 0.03,
        linearization_velocity: 1.0,
    });
    let leadings = [
        None,
        Some(kernel_advance(&kernel_stage)),
        Some(ramped_advance(&RAMPED_ADVANCE_STAGE)),
    ];
    for leading in leadings {
        let reference = source_fit_on(&alone, leading);
        assert!(!reference.reach_retracts);
        assert_same_source(&source_fit_on(&with_a_later_retract, leading), &reference);
    }
}

#[test]
fn a_retract_within_the_reach_is_read() {
    let state = state_laying(&[(50.0, -0.05, 0.05)]);
    assert!(source_fit_on(&state, None).reach_retracts);
}

/// A segment owning only zero-ratio spans holds still, whatever ratio the
/// spans laid past its owned end take.
#[test]
fn a_constant_source_stays_constant_whatever_is_laid_past_it() {
    let shaped = cusp_segment(vec![0.0, 80.0, -300.0], vec![0.0, 0.0], 0.0, 0.1);
    let s_length = super::leader_arc_length(&shaped, &[0, 1]);
    let alone = state_laying(&[(s_length, 0.0, 0.0)]);
    let followed = state_laying(&[(s_length, 0.0, 0.0), (5.0, 0.05, 0.05)]);
    let constant_of = |state: &FollowerState| {
        FollowerSignal::new(
            &shaped,
            &shaped,
            3,
            &[0, 1],
            state,
            odometer_from_start(&shaped),
            0.0,
        )
        .constant_value()
        .map(f64::to_bits)
    };
    assert!(constant_of(&alone).is_some());
    assert_eq!(constant_of(&followed), constant_of(&alone));
}
