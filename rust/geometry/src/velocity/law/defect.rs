//! Proven bounds on how far a rail knot pair's interpolant strays from the
//! law it integrates.
//!
//! On a pair the interpolant reads speed `v`, acceleration `a` and arc `s`
//! as polynomials in the pair's variable — its time share `u`, or `z`, the
//! square root of the time from a departure end. The law asks
//! `a² = A² − κ(s)²·v⁴` below the cap and `a = 0` on it. Taylor models of
//! the three bound `R ≥ |a² − (A² − κ²v⁴)|` over the whole pair; wherever
//! the interpolant's `|a| ≥ m`, the law's acceleration `a_L ≥ 0` of the same
//! sign satisfies `|a − a_L|·(|a| + |a_L|) = |a² − a_L²| ≤ R` — on the cap,
//! `a_L = 0` and `a² ≤ R` — so `|a − a_L| ≤ min(R/m, √R)`. The arc's
//! defect is the linear correction the pair adds to the integral of its
//! speed to land on its far knot's arc.

use taylor_model::{Ball, TaylorModel, polynomial_of_balls, upper};

use super::{RailPair, RailPairShape, ScalarLaw};

/// How far a rail knot pair's interpolated acceleration may stray from its
/// law: a tenth of the default fit tolerance on acceleration, so the
/// rail's knots resolve the law well inside what a fit has to hold.
pub const RAIL_ACCELERATION_DEFECT: f64 = 5.0;

/// How far a rail knot pair's interpolated arc may stray from the integral
/// of its interpolated speed: a tenth of the default fit tolerance on
/// position.
pub const RAIL_ARC_DEFECT: f64 = 5e-4;

/// How far from a pair's own speed, in mm/s, the law its acceleration is
/// held to may be read: near a rising cap the law's acceleration is so
/// steep in the speed that the rounding of any interpolated speed moves it
/// by more than [`RAIL_ACCELERATION_DEFECT`]. A speed this far off moves
/// the rail by a ten-millionth of a millimetre a second.
pub const RAIL_SPEED_SLACK: f64 = 1e-7;

/// The share of [`RAIL_ACCELERATION_DEFECT`] a pair's defect against the
/// law at its own speed must pass before the law within
/// [`RAIL_SPEED_SLACK`] of it is read: below it the pair holds anyway.
const SLACK_TRIGGER_SHARE: f64 = 0.5;

/// Coefficient roundings each interpolant coefficient below carries: at
/// most four products and sums of the knot data.
const COEFFICIENT_ROUNDINGS: usize = 8;

/// Roundings a knot's speed and its pair's rise carry, at most.
pub(super) const SPEED_NOISE_ULPS: f64 = 8.0;

/// The proven defects of one knot pair against its law, and the floor its
/// knots' speed rounding puts under its acceleration: `8ε·v/h`, which no
/// shorter pair resolves.
#[derive(Clone, Copy, Debug)]
pub struct PairDefect {
    pub acceleration: f64,
    pub arc: f64,
    pub speed_noise: f64,
}

impl PairDefect {
    /// How far the pair may stray from its law in acceleration:
    /// [`RAIL_ACCELERATION_DEFECT`] over its speed-rounding floor.
    pub fn allowance(&self) -> f64 {
        RAIL_ACCELERATION_DEFECT + self.speed_noise
    }

    /// How many times over its allowance the worse defect is.
    pub fn excess(&self) -> f64 {
        (self.acceleration / self.allowance()).max(self.arc / RAIL_ARC_DEFECT)
    }
}

/// An upper bound on `|a|` of `pair`'s interpolant.
pub(super) fn acceleration_bound(pair: &RailPair) -> f64 {
    let ([_, acceleration, _], _) = pair_models(pair);
    acceleration.bound()
}

/// The interpolant of `pair` as Taylor models of speed, acceleration and
/// arc on the pair's variable, and the arc correction it adds.
fn pair_models(pair: &RailPair) -> ([TaylorModel; 3], f64) {
    let h = pair.t_hi - pair.t_lo;
    let variable = TaylorModel::affine(Ball::exact(0.5), 0.5);
    let ball = |c: f64| Ball::rounded(c, COEFFICIENT_ROUNDINGS);
    let model = |coefficients: &[f64]| {
        polynomial_of_balls(coefficients.iter().map(|&c| ball(c)), &variable)
    };
    match pair.shape {
        RailPairShape::Quintic { rise, slopes } => {
            let v = quintic_monomials(pair.v_lo, pair.v_hi - pair.v_lo, slopes);
            let rising = quintic_monomials(0.0, rise, slopes);
            let a: Vec<f64> = rising
                .iter()
                .enumerate()
                .skip(1)
                .map(|(power, c)| power as f64 * c / h)
                .collect();
            let mass: Vec<f64> = std::iter::once(0.0)
                .chain(
                    v.iter()
                        .enumerate()
                        .map(|(power, c)| c / (power + 1) as f64),
                )
                .collect();
            let full: f64 = mass.iter().sum();
            let correction = pair.s_hi - pair.s_lo - h * full;
            let mut s: Vec<f64> = mass.iter().map(|c| h * c).collect();
            s[0] += pair.s_lo;
            s[1] += correction;
            ([model(&v), model(&a), model(&s)], correction)
        }
        RailPairShape::Departure {
            from_lo,
            coefficients: [c2, c3, c4, c5],
        } => {
            let (v_end, sign) = if from_lo {
                (pair.v_lo, 1.0)
            } else {
                (pair.v_hi, -1.0)
            };
            let v = [v_end, 0.0, c2, c3, c4, c5];
            let a = [c2, 1.5 * c3, 2.0 * c4, 2.5 * c5].map(|c| sign * c / h);
            let rise_mass = [
                0.0,
                0.0,
                0.0,
                0.0,
                0.5 * c2,
                0.4 * c3,
                c4 / 3.0,
                2.0 * c5 / 7.0,
            ];
            let full = v_end + rise_mass.iter().sum::<f64>();
            let correction = pair.s_hi - pair.s_lo - h * full;
            let mut s = [0.0; 8];
            if from_lo {
                s[0] = pair.s_lo;
                s[2] = h * v_end + correction;
                for (slot, mass) in s.iter_mut().zip(rise_mass).skip(4) {
                    *slot = h * mass;
                }
            } else {
                s[0] = pair.s_lo + h * full + correction;
                s[2] = -h * v_end - correction;
                for (slot, mass) in s.iter_mut().zip(rise_mass).skip(4) {
                    *slot = -h * mass;
                }
            }
            ([model(&v), model(&a), model(&s)], correction)
        }
        RailPairShape::Clipped {
            coefficients: [c2, c3, c4, c5],
            h: parent,
            v_end,
            theta_lo,
            theta_hi,
        } => {
            let (z_lo, z_hi) = ((theta_lo / parent).sqrt(), (theta_hi / parent).sqrt());
            let z = TaylorModel::affine(Ball::rounded(0.5 * (z_lo + z_hi), 2), 0.5 * (z_hi - z_lo));
            let model = |coefficients: &[f64]| {
                polynomial_of_balls(coefficients.iter().map(|&c| ball(c)), &z)
            };
            let direction = (theta_hi - theta_lo).signum();
            let v = [v_end, 0.0, c2, c3, c4, c5];
            let a = [c2, 1.5 * c3, 2.0 * c4, 2.5 * c5].map(|c| direction * c / parent);
            let mass_from_end = [
                0.0,
                0.0,
                v_end * parent,
                0.0,
                0.5 * c2 * parent,
                0.4 * c3 * parent,
                c4 / 3.0 * parent,
                2.0 * c5 / 7.0 * parent,
            ];
            let at = |z: f64| mass_from_end.iter().rev().fold(0.0, |acc, &c| acc * z + c);
            let mass_lo = at(z_lo);
            let full = direction * (at(z_hi) - mass_lo);
            let correction = pair.s_hi - pair.s_lo - full;
            let share = correction / (theta_hi - theta_lo);
            let mut s = mass_from_end.map(|c| direction * c);
            s[0] += pair.s_lo - direction * mass_lo - share * theta_lo;
            s[2] += share * parent;
            ([model(&v), model(&a), model(&s)], correction)
        }
    }
}

/// The monomials in `u` of the C2 quintic Hermite from `p0` rising by
/// `rise` with end slopes and curvatures `[m0, c0, m1, c1]`.
pub(super) fn quintic_monomials(p0: f64, rise: f64, [m0, c0, m1, c1]: [f64; 4]) -> [f64; 6] {
    [
        p0,
        m0,
        0.5 * c0,
        10.0 * rise - 6.0 * m0 - 1.5 * c0 - 4.0 * m1 + 0.5 * c1,
        -15.0 * rise + 8.0 * m0 + 1.5 * c0 + 7.0 * m1 - c1,
        6.0 * rise - 3.0 * m0 - 0.5 * c0 - 3.0 * m1 + 0.5 * c1,
    ]
}

/// The proven defects of `pair`'s interpolant against `law`: its
/// acceleration's against the law's, and — between knots at or below the
/// curvature cap — how far its normal acceleration `|κ|·v²` overshoots
/// the budget.
pub fn pair_defect(law: &ScalarLaw, pair: &RailPair) -> PairDefect {
    let ([v, a, s], correction) = pair_models(pair);
    let arc = upper(correction.abs() * (1.0 + taylor_model::gamma(COEFFICIENT_ROUNDINGS)));
    let acceleration = match *law {
        ScalarLaw::ConstAccel { a0 } => a.add_ball(Ball::exact(-a0)).bound(),
        ScalarLaw::DiskRail {
            accel,
            kappa0,
            sigma,
            brake,
        } => {
            let kappa = s.scale(Ball::exact(sigma)).add_ball(Ball::exact(kappa0));
            let law_squared_of = |normal: &TaylorModel| {
                normal
                    .mul(normal)
                    .scale(Ball::exact(-1.0))
                    .add_ball(Ball::rounded(accel * accel, 1))
            };
            let normal_at = |v: &TaylorModel| kappa.mul(&v.mul(v));
            let normal = normal_at(&v);
            let law_squared = law_squared_of(&normal);
            let sign = if brake { -1.0 } else { 1.0 };
            let departs = matches!(pair.shape, RailPairShape::Departure { .. });
            let (centre, spread) = law_squared.spread();
            let law_peak = upper((centre + spread).max(0.0).sqrt() * (1.0 + f64::EPSILON));
            let fallback =
                either_side_defect(&a, law_peak, brake).min(squares_defect(&a, &law_squared));
            let direct = [
                direct_root_defect(&a, &law_squared, sign),
                departs
                    .then(|| departing_root_defect(&a, &law_squared, sign))
                    .flatten(),
            ]
            .into_iter()
            .flatten()
            .fold(fallback, f64::min);
            let slack = |shift: f64| law_squared_of(&normal_at(&v.add_ball(Ball::exact(shift))));
            let tangential = if direct > SLACK_TRIGGER_SHARE * RAIL_ACCELERATION_DEFECT {
                slack_root_defect(
                    &a,
                    [slack(-RAIL_SPEED_SLACK), slack(RAIL_SPEED_SLACK)],
                    sign,
                )
                .map_or(direct, |slacked| slacked.min(direct))
            } else {
                direct
            };
            let at_or_below_cap = |s: f64, v: f64| {
                law.cap_speed_squared(s)
                    .is_none_or(|cap| v * v <= cap * (1.0 + f64::EPSILON))
            };
            let overshoot =
                if at_or_below_cap(pair.s_lo, pair.v_lo) && at_or_below_cap(pair.s_hi, pair.v_hi) {
                    let (centre, spread) = normal.spread();
                    upper((centre.abs() + spread - accel).max(0.0))
                } else {
                    0.0
                };
            tangential.max(overshoot)
        }
    };
    let h = pair.t_hi - pair.t_lo;
    let speed_noise =
        upper(SPEED_NOISE_ULPS * f64::EPSILON * pair.v_lo.abs().max(pair.v_hi.abs()) / h);
    PairDefect {
        acceleration,
        arc,
        speed_noise,
    }
}

/// The defect of acceleration `a` against the law `sign·√law_squared`
/// read off a model of that square root, where `law_squared` stays
/// positive.
fn direct_root_defect(a: &TaylorModel, law_squared: &TaylorModel, sign: f64) -> Option<f64> {
    let root = law_squared.sqrt()?;
    Some(a.sub(&root.scale(Ball::exact(sign))).bound())
}

/// The defect of acceleration `a` against the law read anywhere within
/// [`RAIL_SPEED_SLACK`] of its own speed: `law_squared` at the speed less
/// and more the slack, between whose roots the law lies; the distance from
/// `a` to that range.
fn slack_root_defect(
    a: &TaylorModel,
    [slower, faster]: [TaylorModel; 2],
    sign: f64,
) -> Option<f64> {
    let (slower, faster) = (slower.sqrt()?, faster.sqrt()?);
    let (low, high) = if sign > 0.0 {
        (faster, slower)
    } else {
        (
            slower.scale(Ball::exact(-1.0)),
            faster.scale(Ball::exact(-1.0)),
        )
    };
    let past = |model: TaylorModel| {
        let (centre, spread) = model.spread();
        upper((centre + spread).max(0.0))
    };
    Some(past(a.sub(&high)).max(past(low.sub(a))))
}

/// The defect of acceleration `a` against the law `sign·√law_squared` on a
/// pair leaving the cap from `x = −1`, where the square vanishes as
/// `(x + 1)²`: read off the quotient `g` in `law_squared = (x + 1)²·g + E`,
/// for which `|√max(0, y²g + E) − y√g| ≤ √|E|` at `y = x + 1 ≥ 0`, and
/// `√max(0, y²g + E) ≤ √|E|` where `g ≤ 0` leaves the rail over its cap.
fn departing_root_defect(a: &TaylorModel, law_squared: &TaylorModel, sign: f64) -> Option<f64> {
    let quotient = divided_by_x_plus_one(&divided_by_x_plus_one(law_squared.coefficients()));
    let g = TaylorModel::from_polynomial(&quotient, 0.0);
    let y = TaylorModel::affine(Ball::exact(1.0), 1.0);
    let rest = upper(law_squared.sub(&y.mul(&y).mul(&g)).bound().sqrt() * (1.0 + f64::EPSILON));
    let (centre, spread) = g.spread();
    if centre + spread <= 0.0 {
        return Some(upper(a.bound() + rest));
    }
    let law = y.mul(&g.sqrt()?).scale(Ball::exact(sign));
    Some(upper(a.sub(&law).bound() + rest))
}

/// The quotient of `Σ cₖ xᵏ` by `x + 1`, its remainder dropped.
fn divided_by_x_plus_one(coefficients: &[f64]) -> Vec<f64> {
    let mut quotient = vec![0.0; coefficients.len().saturating_sub(1)];
    let mut carry = 0.0;
    for (power, &c) in coefficients.iter().enumerate().skip(1).rev() {
        carry = c - carry;
        quotient[power - 1] = carry;
    }
    quotient
}

/// The defect of acceleration `a` against a law of magnitude at most
/// `law_peak`, negative on a `brake` rail: `|a − a_L| ≤ max(|a|, |a_L|)`
/// where both lie on the law's side of zero, `|a| + |a_L|` otherwise.
fn either_side_defect(a: &TaylorModel, law_peak: f64, brake: bool) -> f64 {
    let (centre, spread) = a.spread();
    let (least, most) = if brake {
        (-centre - spread, -centre + spread)
    } else {
        (centre - spread, centre + spread)
    };
    if least >= 0.0 {
        upper(most.max(law_peak))
    } else {
        upper(a.bound() + law_peak)
    }
}

/// The defect of acceleration `a` against `√law_squared` where the law's
/// square is positive and `0` where the cap clamps it, read off the
/// difference of the squares.
fn squares_defect(a: &TaylorModel, law_squared: &TaylorModel) -> f64 {
    let residual = a.mul(a).sub(law_squared).bound();
    let (centre, spread) = a.spread();
    let least = centre.abs() - spread;
    let root = upper(residual.sqrt() * (1.0 + f64::EPSILON));
    if least > 0.0 {
        root.min(upper(residual / least * (1.0 + f64::EPSILON)))
    } else {
        root
    }
}
