use super::{
    AccelBudget, FitTol, LadderBudget, LadderPolicy, LadderSpan, LadderTruth, SeamContinuity,
    VelocitySign, eval_mono_dd, ladder_fit, quintic_in_u,
};

const POSITION_EQUIVALENT: FitTol = FitTol {
    pos_mm: 1e-4,
    accel_mm_s2: 50.0,
    accel_budget: AccelBudget::PositionEquivalent,
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
