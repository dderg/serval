use super::ladder::{eval_mono, eval_mono_d};
use super::{
    AccelBudget, AccelerationExemption, FitTol, LadderBudget, LadderPolicy, LadderSpan,
    LadderTruth, SeamContinuity, VelocitySign, acceleration_bound, eval_mono_dd, ladder_fit,
    quintic_in_u,
};

const POSITION_EQUIVALENT: FitTol = FitTol {
    pos_mm: 1e-4,
    accel_mm_s2: 50.0,
    accel_budget: AccelBudget::PositionEquivalent {
        accel_limit_mm_s2: f64::INFINITY,
    },
};

#[test]
fn an_absolute_budget_holds_every_piece_to_the_configured_acceleration() {
    for h in [1e-5, 1e-3, 0.1, 1.0] {
        let piece = FitTol::absolute(1e-4, 50.0).for_piece(h);
        assert_eq!((piece.pos_mm, piece.accel_mm_s2), (1e-4, 50.0), "h = {h}");
    }
}

#[test]
fn a_position_equivalent_budget_is_the_acceleration_a_pinned_error_spends_the_position_budget_on() {
    for h in [1e-4, 1e-3, 2e-3] {
        let piece = POSITION_EQUIVALENT.for_piece(h);
        assert_eq!(piece.accel_budget, AccelBudget::Absolute);
        assert_eq!(piece.pos_mm, POSITION_EQUIVALENT.pos_mm);
        let pinned_parabola_peak = piece.accel_mm_s2 * h * h / 8.0;
        assert!(
            (pinned_parabola_peak - POSITION_EQUIVALENT.pos_mm).abs()
                <= 1e-12 * POSITION_EQUIVALENT.pos_mm,
            "over {h} s a pinned error at {} mm/s² peaks at {pinned_parabola_peak} mm",
            piece.accel_mm_s2
        );
    }
}

#[test]
fn a_position_equivalent_budget_never_holds_a_long_piece_tighter_than_the_configured_acceleration()
{
    let crossover = (8.0 * POSITION_EQUIVALENT.pos_mm / POSITION_EQUIVALENT.accel_mm_s2).sqrt();
    for h in [1.01 * crossover, 2.0 * crossover, 1.0] {
        assert_eq!(
            POSITION_EQUIVALENT.for_piece(h).accel_mm_s2,
            POSITION_EQUIVALENT.accel_mm_s2,
            "h = {h}"
        );
    }
}

const BOWED_SPAN_S: f64 = 2e-3;
const BOWED_BASE_ACCELERATION: f64 = 1000.0;
/// The acceleration bows by this much between the span's middle and its
/// ends: within the 50 mm/s² budget, which a cubic's straight acceleration
/// meets everywhere while missing it at both ends.
const BOWED_ACCELERATION_RISE: f64 = 30.0;

/// `(p, v, a)` of a signal whose acceleration is a parabola over the span:
/// `A + c·(τ − h/2)²`, reaching `A + rise` at both ends.
fn bowed_pva(tau: f64) -> (f64, f64, f64) {
    let half = 0.5 * BOWED_SPAN_S;
    let curvature = BOWED_ACCELERATION_RISE / (half * half);
    let centred = tau - half;
    let acceleration = BOWED_BASE_ACCELERATION + curvature * centred * centred;
    let velocity =
        20.0 + BOWED_BASE_ACCELERATION * tau + curvature * (centred.powi(3) + half.powi(3)) / 3.0;
    let position = 5.0
        + 20.0 * tau
        + 0.5 * BOWED_BASE_ACCELERATION * tau * tau
        + curvature * (centred.powi(4) - half.powi(4)) / 12.0
        + curvature * half.powi(3) * tau / 3.0;
    (position, velocity, acceleration)
}

/// The bowed span's fit under `seam_continuity`, and how far its
/// acceleration misses the signal's at either end.
fn bowed_fit(seam_continuity: SeamContinuity) -> (Vec<f64>, f64) {
    let h = BOWED_SPAN_S;
    let tau_of = |u: f64| 0.5 * (u + 1.0) * h;
    let position = |u: f64| bowed_pva(tau_of(u)).0;
    let velocity = |u: f64| bowed_pva(tau_of(u)).1;
    let acceleration = |u: f64| bowed_pva(tau_of(u)).2;
    let base = quintic_in_u(bowed_pva(0.0), bowed_pva(h), h);
    let fit = ladder_fit(
        &base,
        LadderSpan {
            h,
            endpoint_delta: position(1.0) - position(-1.0),
        },
        &LadderTruth {
            position: &position,
            velocity: &velocity,
            acceleration: &acceleration,
            held_miss: &|_| 0.0,
            held_ranks_misses: false,
            exemption: None,
        },
        LadderBudget {
            tol: FitTol::absolute(1e-4, 50.0),
            velocity: f64::INFINITY,
        },
        LadderPolicy {
            velocity_sign: VelocitySign::Unconstrained,
            acceleration_monotonicity: None,
            high_degree_span_floor: 0.0,
            seam_continuity,
        },
    )
    .unwrap_or_else(|miss| {
        panic!(
            "the bowed span misses at u={}: acceleration error {} mm/s²",
            miss.failure.u, miss.failure.error.acceleration
        )
    });
    let dd_scale = (2.0 / h) * (2.0 / h);
    let end_miss = [-1.0, 1.0]
        .into_iter()
        .map(|u| (eval_mono_dd(&fit, u) * dd_scale - acceleration(u)).abs())
        .fold(0.0, f64::max);
    (fit, end_miss)
}

#[test]
fn a_velocity_continuous_ladder_lets_a_cubic_miss_the_signal_acceleration_at_its_ends() {
    let (fit, end_miss) = bowed_fit(SeamContinuity::Velocity);
    assert_eq!(fit.len(), 4, "the cubic holds the bowed span: {fit:?}");
    assert!(
        end_miss > 0.5 * BOWED_ACCELERATION_RISE,
        "the cubic's straight acceleration misses the bow's ends by only {end_miss} mm/s²"
    );
}

#[test]
fn an_acceleration_continuous_ladder_meets_the_signal_acceleration_at_both_ends() {
    let (fit, end_miss) = bowed_fit(SeamContinuity::Acceleration);
    assert!(
        end_miss <= 1e-6 * BOWED_BASE_ACCELERATION,
        "the degree-{} fit misses the signal's end acceleration by {end_miss} mm/s²",
        fit.len() - 1
    );
}

const ONSET_SPAN_S: f64 = 4e-9;
const ONSET_GAIN: f64 = 1.0;
const ONSET_LEAD_S: f64 = 1e-15;
const ONSET_START_VELOCITY: f64 = 0.04;
const ONSET_CEILING: f64 = 2.005e4;
const ONSET_VELOCITY_ALLOWANCE: f64 = 2e-4;

/// `(p, v, a)` `tau` past a square-root onset an advance reads off an
/// unshaped leader: acceleration `K/√(τ + τ₀)`, past any ceiling at it.
fn onset_pva(tau: f64) -> (f64, f64, f64) {
    let root = (tau + ONSET_LEAD_S).sqrt();
    let lead_root = ONSET_LEAD_S.sqrt();
    let position = 1.0
        + ONSET_START_VELOCITY * tau
        + 2.0
            * ONSET_GAIN
            * ((2.0 / 3.0) * ((tau + ONSET_LEAD_S) * root - ONSET_LEAD_S * lead_root)
                - lead_root * tau);
    let velocity = ONSET_START_VELOCITY + 2.0 * ONSET_GAIN * (root - lead_root);
    (position, velocity, ONSET_GAIN / root)
}

/// The onset span's kept or accepted rung, with or without its
/// acceleration exempt.
fn onset_rung(exempt: bool) -> Vec<f64> {
    let h = ONSET_SPAN_S;
    let tau_of = |u: f64| 0.5 * (u + 1.0) * h;
    let position = |u: f64| onset_pva(tau_of(u)).0;
    let velocity = |u: f64| onset_pva(tau_of(u)).1;
    let acceleration = |u: f64| onset_pva(tau_of(u)).2;
    let everywhere = |_: f64| true;
    let fit = ladder_fit(
        &quintic_in_u(onset_pva(0.0), onset_pva(h), h),
        LadderSpan {
            h,
            endpoint_delta: position(1.0) - position(-1.0),
        },
        &LadderTruth {
            position: &position,
            velocity: &velocity,
            acceleration: &acceleration,
            held_miss: &|_| 0.0,
            held_ranks_misses: true,
            exemption: exempt.then_some(AccelerationExemption {
                exempt: &everywhere,
                velocity: ONSET_VELOCITY_ALLOWANCE,
                ceiling: ONSET_CEILING,
            }),
        },
        LadderBudget {
            tol: FitTol::absolute(1e-4, 50.0),
            velocity: f64::INFINITY,
        },
        LadderPolicy {
            velocity_sign: VelocitySign::Unconstrained,
            acceleration_monotonicity: None,
            high_degree_span_floor: 0.0,
            seam_continuity: SeamContinuity::Velocity,
        },
    );
    fit.unwrap_or_else(|miss| miss.closest_rung)
}

#[test]
fn a_rung_inside_an_exempt_onset_holds_the_ceiling_and_the_velocity_allowance() {
    let h = ONSET_SPAN_S;
    let rung = onset_rung(true);
    let bound = acceleration_bound(&rung, h);
    assert!(
        bound <= ONSET_CEILING,
        "the degree-{} rung reaches {bound} mm/s² past the {ONSET_CEILING} mm/s² ceiling",
        rung.len() - 1
    );
    for index in 0..=400 {
        let u = -1.0 + f64::from(index) / 200.0;
        let (position, velocity, _) = onset_pva(0.5 * (u + 1.0) * h);
        let velocity_error = (eval_mono_d(&rung, u) * (2.0 / h) - velocity).abs();
        assert!(
            velocity_error <= ONSET_VELOCITY_ALLOWANCE,
            "at u={u} the rung's velocity misses by {velocity_error} mm/s"
        );
        assert!(
            (eval_mono(&rung, u) - position).abs() <= 1e-4,
            "position at u={u}"
        );
    }
}

#[test]
fn without_the_exemption_the_kept_rung_chases_the_onset_past_the_ceiling() {
    let rung = onset_rung(false);
    let bound = acceleration_bound(&rung, ONSET_SPAN_S);
    assert!(
        bound > ONSET_CEILING,
        "the degree-{} rung stays at {bound} mm/s²",
        rung.len() - 1
    );
}

#[test]
fn a_span_below_the_high_degree_floor_whose_rung_is_kept_ranks_every_degree() {
    let h = 1e-3;
    let scale = 2.0 / h;
    let position = |u: f64| 1e-5 * u.powi(8);
    let velocity = |u: f64| 8e-5 * u.powi(7) * scale;
    let acceleration = |u: f64| 5.6e-4 * u.powi(6) * scale * scale;
    let pva = |u: f64| (position(u), velocity(u), acceleration(u));
    let fit = ladder_fit(
        &quintic_in_u(pva(-1.0), pva(1.0), h),
        LadderSpan {
            h,
            endpoint_delta: position(1.0) - position(-1.0),
        },
        &LadderTruth {
            position: &position,
            velocity: &velocity,
            acceleration: &acceleration,
            held_miss: &|_| 0.0,
            held_ranks_misses: true,
            exemption: None,
        },
        LadderBudget {
            tol: FitTol::absolute(1e-4, 50.0),
            velocity: f64::INFINITY,
        },
        LadderPolicy {
            velocity_sign: VelocitySign::Unconstrained,
            acceleration_monotonicity: None,
            high_degree_span_floor: 2.0 * h,
            seam_continuity: SeamContinuity::Velocity,
        },
    )
    .unwrap_or_else(|miss| {
        panic!(
            "the degree-{} closest rung misses at u={}",
            miss.closest_rung.len() - 1,
            miss.failure.u
        )
    });
    assert!(fit.len() > 6, "a degree-{} rung holds u⁸", fit.len() - 1);
}
