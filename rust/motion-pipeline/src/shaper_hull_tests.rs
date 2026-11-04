use super::{FittedPiece, HullBudget, first_unproven};
use crate::lowering::FitTol;
use crate::shaper::exact::{EXACT_CAPACITY, ExactPiece, LocalPolynomial};

const POSITION_ONLY: HullBudget = HullBudget {
    tol: FitTol::absolute(1.0, f64::INFINITY),
    velocity: f64::INFINITY,
};

const ACCELERATION_ONLY: HullBudget = HullBudget {
    tol: FitTol::absolute(f64::INFINITY, 1.0),
    velocity: f64::INFINITY,
};

const T0: f64 = 2.0;
const T1: f64 = 2.004;

fn local(coefficients: &[f64]) -> LocalPolynomial {
    let mut padded = [0.0; EXACT_CAPACITY];
    padded[..coefficients.len()].copy_from_slice(coefficients);
    LocalPolynomial {
        coefficients: padded,
        len: coefficients.len(),
        rounding: 0.0,
    }
}

/// A signal at rest over the whole span: the error is the fit itself.
fn resting_signal() -> [ExactPiece; 1] {
    [ExactPiece {
        lo: T0,
        hi: T1,
        center: 0.5 * (T0 + T1),
        radius: 0.5 * (T1 - T0),
        pva: [local(&[0.0]), local(&[0.0]), local(&[0.0])],
        unshaped_step: None,
    }]
}

/// Monomial coefficients of the Chebyshev polynomial `T_n`.
fn chebyshev(n: usize) -> Vec<f64> {
    let (mut previous, mut current) = (vec![1.0], vec![0.0, 1.0]);
    for _ in 1..n {
        let mut next = vec![0.0; current.len() + 1];
        for (power, &c) in current.iter().enumerate() {
            next[power + 1] += 2.0 * c;
        }
        for (power, &c) in previous.iter().enumerate() {
            next[power] -= c;
        }
        previous = current;
        current = next;
    }
    current
}

fn scaled(coefficients: &[f64], factor: f64) -> Vec<f64> {
    coefficients.iter().map(|c| c * factor).collect()
}

fn verdict(mono_u: &[f64], budget: HullBudget) -> Option<f64> {
    first_unproven(
        &resting_signal(),
        FittedPiece {
            mono_u,
            t0: T0,
            t1: T1,
        },
        (T0, T1),
        budget,
    )
}

/// `T₈` peaks at one on `[−1, 1]`, but its Bernstein hull on the whole
/// span reaches past 170: only halving settles a position error at 0.9 of
/// the budget, and one at 1.05 is a miss.
#[test]
fn halving_settles_an_error_whose_first_hull_is_inconclusive() {
    let wave = chebyshev(8);
    assert_eq!(verdict(&scaled(&wave, 0.9), POSITION_ONLY), None);
    let miss = verdict(&scaled(&wave, 1.05), POSITION_ONLY).expect("a miss past the budget");
    assert!(
        (-1.0..=1.0).contains(&miss),
        "the miss is reported inside the span, at u={miss}"
    );
}

/// A miss is located where the error leaves the budget: a ramp over the
/// budget only past `u ≈ 0.78` is reported there.
#[test]
fn a_miss_is_reported_where_the_error_leaves_the_budget() {
    let miss = verdict(&[0.3, 0.9], POSITION_ONLY).expect("the ramp reaches 1.2");
    assert!(
        miss > 0.7,
        "a ramp past the budget beyond u = 0.78 was reported at {miss}"
    );
}

/// An acceleration error is held to the acceleration budget: a cubic
/// whose second derivative peaks at 0.9 of it passes, one at 1.1 fails.
#[test]
fn the_acceleration_error_is_held_to_its_own_budget() {
    let half = 0.5 * (T1 - T0);
    let cubic = |peak: f64| [0.0, 0.0, 0.0, peak * half * half / 6.0];
    assert_eq!(verdict(&cubic(0.9), ACCELERATION_ONLY), None);
    assert!(verdict(&cubic(1.1), ACCELERATION_ONLY).is_some());
}

/// A signal's own rounding term is charged against the budget: an error
/// that would pass is not proven when the rounding could hide the rest.
#[test]
fn the_signal_rounding_term_is_charged_against_the_budget() {
    let mut signal = resting_signal();
    signal[0].pva[0].rounding = 0.2;
    let fitted = [0.9];
    let piece = FittedPiece {
        mono_u: &fitted,
        t0: T0,
        t1: T1,
    };
    assert!(first_unproven(&signal, piece, (T0, T1), POSITION_ONLY).is_some());
    signal[0].pva[0].rounding = 0.05;
    assert_eq!(
        first_unproven(&signal, piece, (T0, T1), POSITION_ONLY),
        None
    );
}

/// A rung whose coefficients are not numbers, or a signal whose enclosure
/// is not, is never proven: every comparison with a NaN is false, which a
/// peak taken by `>` would read as zero error.
#[test]
fn a_rung_or_an_enclosure_with_a_nan_is_never_proven() {
    assert!(verdict(&[0.0, f64::NAN, 0.0], POSITION_ONLY).is_some());
    assert!(verdict(&[0.0, f64::NAN, 0.0], ACCELERATION_ONLY).is_some());
    let mut signal = resting_signal();
    signal[0].pva[0] = local(&[f64::NAN]);
    let verdict = first_unproven(
        &signal,
        FittedPiece {
            mono_u: &[0.0],
            t0: T0,
            t1: T1,
        },
        (T0, T1),
        POSITION_ONLY,
    );
    assert!(
        verdict.is_some(),
        "a NaN signal is proven within the budget"
    );
}

/// Inside an unshaped leader's square-root onset the hull holds a rung's
/// velocity to the step's allowance and not its acceleration: a rung far
/// past the acceleration budget is proven while its velocity stays within
/// the allowance, and not once it leaves it.
#[test]
fn an_onset_exemption_holds_velocity_to_its_allowance_and_frees_acceleration() {
    let budget = HullBudget {
        tol: FitTol::absolute(f64::INFINITY, 1.0),
        velocity: f64::INFINITY,
    };
    let mut signal = resting_signal();
    signal[0].unshaped_step =
        Some(crate::follower_projection::UnshapedLeaderStep::with_allowance(1.0));
    let half = 0.5 * (T1 - T0);
    let swing = |velocity_peak: f64| [0.0, 0.0, 0.5 * velocity_peak * half];
    let verdict_on = |signal: &[ExactPiece], mono_u: &[f64]| {
        first_unproven(
            signal,
            FittedPiece {
                mono_u,
                t0: T0,
                t1: T1,
            },
            (T0, T1),
            budget,
        )
    };
    assert!(verdict_on(&resting_signal(), &swing(0.9)).is_some());
    assert_eq!(verdict_on(&signal, &swing(0.9)), None);
    assert!(verdict_on(&signal, &swing(1.1)).is_some());
}
