use super::{
    AxisFit, FloorVelocityGate, KnotJumpBound, MIN_DEVICE_INTERVAL_S, TrackSignal,
    fit_axis_from_signal,
};
use crate::lowering::{FitTol, LADDER_PROBES_U, eval_mono_dd, quintic_in_u};
use crate::types::PostProcessError;

#[derive(Clone, Copy)]
struct Motion {
    velocity: f64,
    acceleration: f64,
}

struct AccelerationStep {
    at: f64,
    velocity: f64,
    before: f64,
    after: f64,
}

impl TrackSignal for AccelerationStep {
    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        let tau = t - self.at;
        let acceleration = if t < self.at { self.before } else { self.after };
        (
            self.velocity * tau + acceleration * tau * tau / 2.0,
            self.velocity + acceleration * tau,
            acceleration,
        )
    }
}

struct RampToRest {
    at: f64,
    ramp_s: f64,
    approach: Motion,
}

impl TrackSignal for RampToRest {
    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        let Motion {
            velocity,
            acceleration,
        } = self.approach;
        let tau = t - self.at;
        if tau < 0.0 {
            return (
                velocity * tau + acceleration * tau * tau / 2.0,
                velocity + acceleration * tau,
                acceleration,
            );
        }
        let ramp = tau.min(self.ramp_s);
        let deceleration = velocity / self.ramp_s;
        if tau < self.ramp_s {
            (
                velocity * ramp - deceleration * ramp * ramp / 2.0,
                velocity - deceleration * ramp,
                -deceleration,
            )
        } else {
            (velocity * self.ramp_s / 2.0, 0.0, 0.0)
        }
    }
}

fn narrowest_knot_span(track: &nurbs::ScalarNurbs) -> f64 {
    let mut knots = track.knots().to_vec();
    knots.dedup();
    knots
        .windows(2)
        .map(|span| span[1] - span[0])
        .fold(f64::INFINITY, f64::min)
}

#[test]
fn bisecting_toward_an_unseeded_acceleration_step_keeps_pieces_device_wide() {
    let step = AccelerationStep {
        at: 0.994_644_409_797_3,
        velocity: -215.828_87,
        before: 11_700.5,
        after: 11_809.0,
    };
    let track = fit_axis_from_signal(
        AxisFit {
            axis: 3,
            t_start: 0.982_839_222_588_5,
            t_end: 1.002_872_787_996_2,
            seed_breakpoints: &[0.994_616_767_877, 0.994_655_895_934_5],
            fit_tol: FitTol {
                pos_mm: 0.0025,
                accel_mm_s2: 25.0,
            },
            fit_context: "unseeded acceleration step",
            velocity_continuous: false,
        },
        &step,
    )
    .expect("an acceleration step refines down to the device interval and fits");
    let narrowest = narrowest_knot_span(&track);
    assert!(
        narrowest >= MIN_DEVICE_INTERVAL_S,
        "refinement emitted a {narrowest:e} s piece, below the {MIN_DEVICE_INTERVAL_S:e} s \
         device interval"
    );
}

#[test]
fn a_picometre_ramp_to_rest_at_the_resolution_floor_fails_loudly() {
    let ramp = RampToRest {
        at: 4.340_821_412_13,
        ramp_s: 4e-14,
        approach: Motion {
            velocity: 0.1275,
            acceleration: -155.84,
        },
    };
    let fit = fit_axis_from_signal(
        AxisFit {
            axis: 3,
            t_start: 4.33,
            t_end: 4.35,
            seed_breakpoints: &[ramp.at - 0.7e-5, ramp.at + 1.3e-5],
            fit_tol: FitTol {
                pos_mm: 0.0025,
                accel_mm_s2: 50.0,
            },
            fit_context: "picometre ramp to rest",
            velocity_continuous: false,
        },
        &ramp,
    );
    match fit {
        Err(PostProcessError::VelocityStepAtFitFloor {
            axis,
            fit_context,
            t_start,
            t_end,
            velocity_change,
            ..
        }) => {
            assert_eq!((axis, fit_context), (3, "picometre ramp to rest"));
            assert!(
                t_start <= ramp.at && ramp.at + ramp.ramp_s <= t_end,
                "the reported span [{t_start}, {t_end}] misses the ramp at {}",
                ramp.at
            );
            assert!(
                (velocity_change + 0.1275).abs() < 1e-3,
                "the reported velocity change {velocity_change} is not the ramp's"
            );
        }
        Err(other) => panic!("a ramp to rest failed with an unrelated error: {other}"),
        Ok(track) => panic!(
            "a ramp to rest narrower than the resolution floor was accepted as {} knots",
            track.knots().len()
        ),
    }
}

struct Coasting;

impl TrackSignal for Coasting {
    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        (10.0 * t, 10.0, 0.0)
    }
}

#[test]
fn the_floor_admits_a_velocity_change_an_acceleration_within_tolerance_carries() {
    let fit_tol = FitTol {
        pos_mm: 0.0025,
        accel_mm_s2: 50.0,
    };
    let (t0, t1) = (1.0, 1.0 + 4e-9);
    let within = 0.5 * fit_tol.accel_mm_s2 * (t1 - t0);
    let gate = FloorVelocityGate::over(&Coasting, t0, t1, fit_tol);
    assert!(
        gate.admits(within) && gate.admits(-within),
        "a velocity change of {within} is what half the acceleration tolerance carries across \
         the span"
    );
    assert!(
        !gate.admits(4.0 * within),
        "twice the acceleration tolerance across the span is a velocity step"
    );
}

struct CubicWithProbeBlindBump {
    t0: f64,
    h: f64,
    cubic: [f64; 4],
    bump: Vec<f64>,
}

fn times_root(polynomial: &[f64], root: f64) -> Vec<f64> {
    let mut shifted = vec![0.0; polynomial.len() + 1];
    for (power, &coefficient) in polynomial.iter().enumerate() {
        shifted[power + 1] += coefficient;
        shifted[power] -= root * coefficient;
    }
    shifted
}

fn integrated_from_minus_one(polynomial: &[f64]) -> Vec<f64> {
    let mut integral = vec![0.0];
    integral.extend(
        polynomial
            .iter()
            .enumerate()
            .map(|(power, &coefficient)| coefficient / (power + 1) as f64),
    );
    integral[0] = -eval_polynomial(&integral, -1.0);
    integral
}

fn eval_polynomial(polynomial: &[f64], x: f64) -> f64 {
    polynomial
        .iter()
        .rev()
        .fold(0.0, |value, &coefficient| value * x + coefficient)
}

fn moment_over_unit_span(polynomial: &[f64], power: usize) -> f64 {
    let mut weighted = vec![0.0; power];
    weighted.extend_from_slice(polynomial);
    eval_polynomial(&integrated_from_minus_one(&weighted), 1.0)
}

fn peak_on_unit_span(polynomial: &[f64]) -> f64 {
    (0..=1000)
        .map(|k| eval_polynomial(polynomial, 2.0 * f64::from(k) / 1000.0 - 1.0).abs())
        .fold(0.0, f64::max)
}

impl CubicWithProbeBlindBump {
    fn position_blind(t0: f64, h: f64, cubic: [f64; 4], bump_scale_mm: f64) -> Self {
        let ends_flat = [-1.0, -1.0, -1.0, 1.0, 1.0, 1.0];
        let interior_probes = LADDER_PROBES_U.iter().filter(|u| u.abs() < 1.0);
        let bump = ends_flat
            .iter()
            .chain(interior_probes)
            .fold(vec![bump_scale_mm], |bump, &root| times_root(&bump, root));
        Self { t0, h, cubic, bump }
    }

    fn acceleration_blind(t0: f64, h: f64, cubic: [f64; 4], peak_mm_s2: f64) -> Self {
        let vanishing_at_probes = LADDER_PROBES_U
            .iter()
            .fold(vec![1.0], |shape, &root| times_root(&shape, root));
        let ends_at_rest = -moment_over_unit_span(&vanishing_at_probes, 1)
            / moment_over_unit_span(&vanishing_at_probes, 3);
        let mut shape = vanishing_at_probes.clone();
        shape.extend([0.0, 0.0]);
        for (power, &coefficient) in vanishing_at_probes.iter().enumerate() {
            shape[power + 2] += ends_at_rest * coefficient;
        }
        let rate = 2.0 / h;
        let scale = peak_mm_s2 / (peak_on_unit_span(&shape) * rate * rate);
        let shape: Vec<f64> = shape
            .iter()
            .map(|coefficient| coefficient * scale)
            .collect();
        let bump = integrated_from_minus_one(&integrated_from_minus_one(&shape));
        Self { t0, h, cubic, bump }
    }
}

impl TrackSignal for CubicWithProbeBlindBump {
    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        let tau = t - self.t0;
        let [p, v, a, j] = self.cubic;
        let x = 2.0 * tau / self.h - 1.0;
        let (mut bump, mut bump_d, mut bump_dd) = (0.0, 0.0, 0.0);
        for &coefficient in self.bump.iter().rev() {
            bump_dd = bump_dd * x + 2.0 * bump_d;
            bump_d = bump_d * x + bump;
            bump = bump * x + coefficient;
        }
        let rate = 2.0 / self.h;
        (
            p + tau * (v + tau * (a / 2.0 + tau * j / 6.0)) + bump,
            v + tau * (a + tau * j / 2.0) + bump_d * rate,
            a + tau * j + bump_dd * rate * rate,
        )
    }
}

#[test]
fn an_accepted_piece_holds_the_position_tolerance_between_the_ladder_probes() {
    let fit_tol = FitTol {
        pos_mm: 1e-4,
        accel_mm_s2: 50.0,
    };
    let (t_start, h) = (1.0, 0.05);
    let sig =
        CubicWithProbeBlindBump::position_blind(t_start, h, [0.3, 14.0, 300.0, -20_000.0], 0.042);
    let track = fit_axis_from_signal(
        AxisFit {
            axis: 3,
            t_start,
            t_end: t_start + h,
            seed_breakpoints: &[],
            fit_tol,
            fit_context: "probe-blind bump",
            velocity_continuous: true,
        },
        &sig,
    )
    .expect("a smooth signal fits");
    let (worst, at) = (0..=1000)
        .map(|k| {
            let t = t_start + h * f64::from(k) / 1000.0;
            ((nurbs::eval::eval(&track, t) - sig.eval(t)).abs(), t)
        })
        .fold((0.0, f64::NAN), |worst, sample| {
            if sample.0 > worst.0 { sample } else { worst }
        });
    assert!(
        worst <= fit_tol.pos_mm,
        "the fit leaves the signal by {worst:e} mm at {at}, past the {:e} mm tolerance",
        fit_tol.pos_mm
    );
}

struct JerkKink {
    t0: f64,
    h: f64,
    knot: [f64; 1],
    before: [f64; 4],
    jerk_after: f64,
}

impl JerkKink {
    fn new(t0: f64, h: f64, knot_fraction: f64, jerks: (f64, f64)) -> Self {
        Self {
            t0,
            h,
            knot: [t0 + knot_fraction * h],
            before: [0.3, 10.0, 200.0, jerks.0],
            jerk_after: jerks.1,
        }
    }

    fn fit(&self, fit_tol: FitTol) -> nurbs::ScalarNurbs {
        fit_axis_from_signal(
            AxisFit {
                axis: 3,
                t_start: self.t0,
                t_end: self.t0 + self.h,
                seed_breakpoints: &[],
                fit_tol,
                fit_context: "unseeded jerk kink",
                velocity_continuous: true,
            },
            self,
        )
        .expect("a kinked signal fits")
    }

    fn worst_deviation(
        &self,
        fitted: impl Fn(f64) -> f64,
        exact: impl Fn(f64) -> f64,
    ) -> (f64, f64) {
        (0..2000)
            .map(|k| {
                let t = self.t0 + self.h * (f64::from(k) + 0.5) / 2000.0;
                ((fitted(t) - exact(t)).abs(), t)
            })
            .fold((0.0, f64::NAN), |worst, sample| {
                if sample.0 > worst.0 { sample } else { worst }
            })
    }
}

impl TrackSignal for JerkKink {
    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        let [p, v, a, j] = self.before;
        let along = |(p, v, a, j): (f64, f64, f64, f64), tau: f64| {
            (
                p + tau * (v + tau * (a / 2.0 + tau * j / 6.0)),
                v + tau * (a + tau * j / 2.0),
                a + tau * j,
            )
        };
        let knot = self.knot[0] - self.t0;
        let tau = t - self.t0;
        if tau <= knot {
            return along((p, v, a, j), tau);
        }
        let (p, v, a) = along((p, v, a, j), knot);
        along((p, v, a, self.jerk_after), tau - knot)
    }

    fn interior_knots(&self, start: f64, end: f64) -> &[f64] {
        if self.knot[0] > start && self.knot[0] < end {
            &self.knot
        } else {
            &[]
        }
    }

    fn knot_jump(&self, _knot: f64) -> KnotJumpBound {
        KnotJumpBound {
            acceleration: 0.0,
            jerk: (self.jerk_after - self.before[3]).abs(),
        }
    }
}

const FOLLOWER_LIKE_TOLERANCE: FitTol = FitTol {
    pos_mm: 1e-4,
    accel_mm_s2: 50.0,
};

#[test]
fn an_accepted_piece_holds_the_acceleration_tolerance_across_an_unseeded_jerk_kink() {
    let sig = JerkKink::new(1.0, 0.01, 0.35, (1e5, -1e5));
    let track = sig.fit(FOLLOWER_LIKE_TOLERANCE);
    let acceleration = nurbs::eval::derivative(&nurbs::eval::derivative(&track));
    let (worst, at) = sig.worst_deviation(
        |t| nurbs::eval::eval(&acceleration, t),
        |t| sig.second_deriv(t),
    );
    assert!(
        worst <= FOLLOWER_LIKE_TOLERANCE.accel_mm_s2,
        "the fit's acceleration leaves the signal's by {worst} mm/s² at {at}, past the {} mm/s² \
         tolerance",
        FOLLOWER_LIKE_TOLERANCE.accel_mm_s2
    );
}

#[test]
fn an_accepted_piece_holds_the_position_tolerance_across_an_unseeded_jerk_kink() {
    let sig = JerkKink::new(1.0, 0.03, 0.41, (1e4, -1e4));
    let track = sig.fit(FOLLOWER_LIKE_TOLERANCE);
    let (worst, at) = sig.worst_deviation(|t| nurbs::eval::eval(&track, t), |t| sig.eval(t));
    assert!(
        worst <= FOLLOWER_LIKE_TOLERANCE.pos_mm,
        "the fit leaves the signal by {worst:e} mm at {at}, past the {:e} mm tolerance",
        FOLLOWER_LIKE_TOLERANCE.pos_mm
    );
}

#[test]
fn an_accepted_piece_holds_the_acceleration_tolerance_between_the_ladder_probes() {
    let fit_tol = FitTol {
        pos_mm: 1e-4,
        accel_mm_s2: 50.0,
    };
    let (t_start, h) = (1.0, 0.004);
    let sig = CubicWithProbeBlindBump::acceleration_blind(
        t_start,
        h,
        [0.3, 14.0, 300.0, -20_000.0],
        2.0 * fit_tol.accel_mm_s2,
    );
    let bump_travel = peak_on_unit_span(&sig.bump);
    assert!(
        bump_travel < 0.1 * fit_tol.pos_mm,
        "the bump moves {bump_travel:e} mm: it must leave position to the acceleration alone"
    );
    let track = fit_axis_from_signal(
        AxisFit {
            axis: 3,
            t_start,
            t_end: t_start + h,
            seed_breakpoints: &[],
            fit_tol,
            fit_context: "probe-blind acceleration bump",
            velocity_continuous: true,
        },
        &sig,
    )
    .expect("a smooth signal fits");
    let acceleration = nurbs::eval::derivative(&nurbs::eval::derivative(&track));
    let (worst, at) = (0..=1000)
        .map(|k| {
            let t = t_start + h * f64::from(k) / 1000.0;
            (
                (nurbs::eval::eval(&acceleration, t) - sig.second_deriv(t)).abs(),
                t,
            )
        })
        .fold((0.0, f64::NAN), |worst, sample| {
            if sample.0 > worst.0 { sample } else { worst }
        });
    assert!(
        worst <= fit_tol.accel_mm_s2,
        "the fit's acceleration leaves the signal by {worst} mm/s² at {at}, past the {} mm/s² \
         tolerance",
        fit_tol.accel_mm_s2
    );
}

struct SquareRootOnset {
    at: f64,
    offset_mm: f64,
    velocity: f64,
    coefficient: f64,
}

impl TrackSignal for SquareRootOnset {
    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        let tau = (t - self.at).max(0.0);
        let root = tau.sqrt();
        (
            self.offset_mm
                + self.velocity * (t - self.at)
                + self.coefficient * 4.0 / 15.0 * tau * tau * root,
            self.velocity + self.coefficient * 2.0 / 3.0 * tau * root,
            self.coefficient * root,
        )
    }
}

#[test]
fn a_square_root_onset_at_the_floor_is_fitted_by_the_closest_anchored_rung() {
    let fit_tol = FitTol {
        pos_mm: 0.005,
        accel_mm_s2: 50.0,
    };
    let sig = SquareRootOnset {
        at: 0.211_721_933,
        offset_mm: 150.0,
        velocity: 300.0,
        coefficient: 9.8e6,
    };
    let track = fit_axis_from_signal(
        AxisFit {
            axis: 1,
            t_start: sig.at,
            t_end: sig.at + 4e-6,
            seed_breakpoints: &[],
            fit_tol,
            fit_context: "square-root onset",
            velocity_continuous: false,
        },
        &sig,
    )
    .expect("a square-root onset fits down to the floor");
    let mut knots = track.knots().to_vec();
    knots.dedup();
    let (t0, t1) = (knots[0], knots[1]);
    let h = t1 - t0;
    let velocity = nurbs::eval::derivative(&track);
    let acceleration = nurbs::eval::derivative(&velocity);
    let state = |t: f64| {
        (
            nurbs::eval::eval(&track, t),
            nurbs::eval::eval(&velocity, t),
            nurbs::eval::eval(&acceleration, t),
        )
    };
    let quintic = quintic_in_u(state(t0), state(t1), h);
    let worst_error = |fitted: &dyn Fn(f64) -> f64| {
        (1..200)
            .map(|k| {
                let past_edge_inset = 0.25 + 0.75 * f64::from(k) / 200.0;
                let t = t0 + h * past_edge_inset;
                (fitted(t) - sig.second_deriv(t)).abs()
            })
            .fold(0.0, f64::max)
    };
    let accepted = worst_error(&|t| nurbs::eval::eval(&acceleration, t));
    let base_quintic =
        worst_error(&|t| eval_mono_dd(&quintic, 2.0 * (t - t0) / h - 1.0) * (2.0 / h).powi(2));
    assert!(
        accepted < 0.8 * base_quintic,
        "the {h:e} s floor piece at the onset misses the acceleration by {accepted} mm/s², \
         the anchored quintic by {base_quintic} mm/s²: the floor kept a rung no closer than \
         the quintic"
    );
}
