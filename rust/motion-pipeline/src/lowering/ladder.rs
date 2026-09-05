use nurbs::bezier::BezierPiece;
use nurbs::chebyshev::taylor_shift;

use super::FitTol;

pub(crate) const LADDER_PROBES_U: [f64; 9] = [
    -1.0,
    -0.923_879_532_511_286_7,
    -std::f64::consts::FRAC_1_SQRT_2,
    -0.382_683_432_365_089_8,
    0.0,
    0.382_683_432_365_089_8,
    std::f64::consts::FRAC_1_SQRT_2,
    0.923_879_532_511_286_7,
    1.0,
];

/// `(1 − u²)³` — triple zeros at ±1, so adding it preserves endpoint p/v/a.
const BUMP6: [f64; 7] = [1.0, 0.0, -3.0, 0.0, 3.0, 0.0, -1.0];
/// `u·(1 − u²)³`.
const BUMP7: [f64; 8] = [0.0, 1.0, 0.0, -3.0, 0.0, 3.0, 0.0, -1.0];

pub(crate) fn eval_mono(c: &[f64], x: f64) -> f64 {
    c.iter().rev().fold(0.0, |acc, &ck| acc * x + ck)
}
fn eval_mono_d(c: &[f64], x: f64) -> f64 {
    c.iter()
        .enumerate()
        .skip(1)
        .rev()
        .fold(0.0, |acc, (k, &ck)| acc * x + k as f64 * ck)
}

pub(crate) fn eval_mono_dd(c: &[f64], x: f64) -> f64 {
    let mut acc = 0.0;
    for (k, &ck) in c.iter().enumerate().skip(2).rev() {
        acc = acc * x + (k * (k - 1)) as f64 * ck;
    }
    acc
}

/// Monomial-in-u quadratic that reproduces the left endpoint position and
/// `endpoint_delta` — the span's own relative travel — exactly, and carries
/// the *sampled* midpoint acceleration instead of solving one from the
/// endpoints. That keeps both seams anchored while the delta's error reaches
/// the fit as `Δ/h` of velocity rather than `2Δ/h²` of acceleration, which is
/// the only reading a resolution-scale span still resolves.
pub(crate) fn anchored_acceleration_quadratic_in_u(
    p_start: f64,
    acceleration: f64,
    h: f64,
    endpoint_delta: f64,
) -> Vec<f64> {
    let curvature = acceleration * h * h / 8.0;
    vec![
        p_start + 0.5 * endpoint_delta - curvature,
        0.5 * endpoint_delta,
        curvature,
    ]
}

/// Monomial-in-u quadratic that matches the left endpoint `(p, v)` exactly and
/// reproduces `endpoint_delta` — the span's own relative travel — exactly at
/// the right endpoint. In τ it is `p0 + v0·τ + ((Δp − v0·h)/h²)·τ²`; pushed
/// into u every coefficient but the anchor is built from `endpoint_delta` and
/// `v0·h` alone, so no absolute endpoint is ever subtracted and its constant
/// acceleration `2(Δp − v0·h)/h²` carries `2/h²` of the delta's error where a
/// cubic carries three times that through `c3`.
pub(crate) fn quadratic_in_u(p_start: f64, v_start: f64, h: f64, endpoint_delta: f64) -> Vec<f64> {
    let start_travel = v_start * h;
    vec![
        p_start + 0.25 * (start_travel + endpoint_delta),
        0.5 * endpoint_delta,
        0.25 * (endpoint_delta - start_travel),
    ]
}

/// `endpoint_delta` is `p(t1) − p(t0)` measured by the signal itself, not
/// recovered as `sb.0 − sa.0`: on a short span riding a large absolute
/// position the difference of the endpoints is denominated in ulps of that
/// absolute value, and `c3` hands the loss to the acceleration through
/// `(2/h)²`. `c0` keeps the absolute base so the piece stays anchored where
/// the track actually is.
pub(crate) fn cubic_in_u(sa: (f64, f64), sb: (f64, f64), h: f64, endpoint_delta: f64) -> Vec<f64> {
    let half_span = 0.5 * h;
    let slope_start = sa.1 * half_span;
    let slope_end = sb.1 * half_span;
    let chord_slope = 0.5 * endpoint_delta;
    let slope_half_difference = 0.5 * (slope_end - slope_start);
    let slope_mean = 0.5 * (slope_start + slope_end);
    let cubic = 0.5 * (slope_mean - chord_slope);
    vec![
        sa.0 + 0.5 * endpoint_delta - 0.5 * slope_half_difference,
        chord_slope - cubic,
        0.5 * slope_half_difference,
        cubic,
    ]
}

/// Monomial-in-u quintic matching `(p, v, a)` — time-domain derivatives — at
/// both ends of a span of duration `h`. Fitting in u keeps the coefficients
/// O(piece amplitude): the conditioning win over monomial-τ.
pub(crate) fn quintic_in_u(sa: (f64, f64, f64), sb: (f64, f64, f64), h: f64) -> Vec<f64> {
    let s = 0.5 * h;
    let q = quintic_hermite_coeffs(
        sa.0,
        sa.1 * s,
        sa.2 * s * s,
        sb.0,
        sb.1 * s,
        sb.2 * s * s,
        2.0,
    );
    taylor_shift(&q, 1.0)
}

/// Monomial coefficients `c0..c5` of the quintic matching `(s0, v0, a0)` at `τ = 0`
/// and `(s1, v1, a1)` at `τ = h`.
pub(super) fn quintic_hermite_coeffs(
    s0: f64,
    v0: f64,
    a0: f64,
    s1: f64,
    v1: f64,
    a1: f64,
    h: f64,
) -> [f64; 6] {
    let ds = s1 - s0;
    let h2 = h * h;
    let h3 = h2 * h;
    let c3 = (20.0 * ds - (8.0 * v1 + 12.0 * v0) * h - (3.0 * a0 - a1) * h2) / (2.0 * h3);
    let c4 =
        (-30.0 * ds + (14.0 * v1 + 16.0 * v0) * h + (3.0 * a0 - 2.0 * a1) * h2) / (2.0 * h3 * h);
    let c5 = (12.0 * ds - 6.0 * (v1 + v0) * h - (a0 - a1) * h2) / (2.0 * h3 * h2);
    [s0, v0, 0.5 * a0, c3, c4, c5]
}

const HIGH_LADDER_NODES_U: [f64; 8] = [
    -0.923_879_532_511_286_7,
    -std::f64::consts::FRAC_1_SQRT_2,
    -0.5,
    -0.382_683_432_365_089_8,
    0.382_683_432_365_089_8,
    0.5,
    std::f64::consts::FRAC_1_SQRT_2,
    0.923_879_532_511_286_7,
];

fn higher_ladder_candidate(base: &[f64], degree: usize, truth_p: &dyn Fn(f64) -> f64) -> Vec<f64> {
    let count = degree - 5;
    let first = (HIGH_LADDER_NODES_U.len() - count) / 2;
    let nodes = &HIGH_LADDER_NODES_U[first..first + count];
    let mut system = vec![vec![0.0; count + 1]; count];
    for (row, &u) in nodes.iter().enumerate() {
        let bump = eval_mono(&BUMP6, u);
        let mut power = 1.0;
        for column in 0..count {
            system[row][column] = bump * power;
            power *= u;
        }
        system[row][count] = truth_p(u) - eval_mono(base, u);
    }
    for column in 0..count {
        let pivot = (column..count)
            .max_by(|&left, &right| {
                system[left][column]
                    .abs()
                    .total_cmp(&system[right][column].abs())
            })
            .expect("empty ladder system");
        system.swap(column, pivot);
        let divisor = system[column][column];
        for value in &mut system[column][column..=count] {
            *value /= divisor;
        }
        for row in 0..count {
            if row == column {
                continue;
            }
            let factor = system[row][column];
            for entry in column..=count {
                system[row][entry] -= factor * system[column][entry];
            }
        }
    }
    let mut candidate = base.to_vec();
    candidate.resize(degree + 1, 0.0);
    for q_power in 0..count {
        let q = system[q_power][count];
        for (bump_power, &bump) in BUMP6.iter().enumerate() {
            candidate[q_power + bump_power] += q * bump;
        }
    }
    candidate
}

/// Degree-`degree` ladder candidate: the quintic base plus `(1−u²)³`-shaped
/// corrections whose coefficients come from interior residuals (u = 0 for
/// degree 6; u = ±½ with exact 27/64 denominators for degree 7).
pub(crate) fn ladder_candidate(
    base: &[f64],
    degree: usize,
    truth_p: &dyn Fn(f64) -> f64,
) -> Vec<f64> {
    let mut c = base.to_vec();
    match degree {
        5 => {}
        6 => {
            let r0 = truth_p(0.0) - eval_mono(base, 0.0);
            c.resize(7, 0.0);
            for (ci, &w) in c.iter_mut().zip(&BUMP6) {
                *ci += r0 * w;
            }
        }
        7 => {
            let rp = truth_p(0.5) - eval_mono(base, 0.5);
            let rm = truth_p(-0.5) - eval_mono(base, -0.5);
            let q0 = (rp + rm) * (32.0 / 27.0);
            let q1 = (rp - rm) * (64.0 / 27.0);
            c.resize(8, 0.0);
            for (ci, &w) in c.iter_mut().zip(&BUMP6) {
                *ci += q0 * w;
            }
            for (ci, &w) in c.iter_mut().zip(&BUMP7) {
                *ci += q1 * w;
            }
        }
        9 | 11 | 13 => return higher_ladder_candidate(base, degree, truth_p),
        _ => panic!("unsupported ladder degree {degree}"),
    }
    c
}

pub(crate) struct LadderFailure {
    pub u: f64,
    pub position_error: f64,
    pub velocity_error: f64,
    pub acceleration_error: f64,
    pub source_position: f64,
    pub source_velocity: f64,
    pub source_acceleration: f64,
    pub candidate_position: f64,
    pub candidate_acceleration: f64,
    pub left_position: f64,
    pub left_velocity: f64,
    pub left_acceleration: f64,
    pub right_position: f64,
    pub right_velocity: f64,
    pub right_acceleration: f64,
    pub candidate_velocity: f64,
}

/// `endpoint_anchored` forbids the midpoint constant/quadratic shortcuts:
/// those match `(p, v, a)` at `u = 0` only, so accepting one spends the fit
/// budget as a position/velocity jump at the span seams. Callers that own the
/// seam continuity of the piece they receive must set it.
///
/// `high_degree_span_floor` is the shortest span whose endpoint data still
/// carries enough signal for a rung solved from endpoint *acceleration* to
/// mean anything. Below it the quintic base and its bump corrections read
/// sampling noise amplified by `(2/h)²`, so only the cubic — which never
/// touches endpoint acceleration — is attempted.
#[derive(Debug, Clone, Copy)]
pub(crate) struct LadderPolicy {
    pub endpoint_anchored: bool,
    pub enforce_velocity_sign: bool,
    pub acceleration_monotonicity: Option<bool>,
    pub high_degree_span_floor: f64,
}

fn preserves_certified_velocity_sign(mono_u: &[f64], truth_v: &dyn Fn(f64) -> f64) -> bool {
    let nonnegative = LADDER_PROBES_U.iter().all(|&u| truth_v(u) >= 0.0);
    let nonpositive = LADDER_PROBES_U.iter().all(|&u| truth_v(u) <= 0.0);
    if !nonnegative && !nonpositive {
        return true;
    }
    if mono_u.len() <= 4 {
        let mut extrema = vec![eval_mono_d(mono_u, -1.0), eval_mono_d(mono_u, 1.0)];
        if mono_u.len() == 4 && mono_u[3] != 0.0 {
            let vertex = -mono_u[2] / (3.0 * mono_u[3]);
            if (-1.0..=1.0).contains(&vertex) {
                extrema.push(eval_mono_d(mono_u, vertex));
            }
        }
        return extrema.into_iter().all(|velocity| {
            (!nonnegative || velocity >= 0.0) && (!nonpositive || velocity <= 0.0)
        });
    }
    let bernstein = BezierPiece {
        u_start: -1.0,
        u_end: 1.0,
        coeffs: taylor_shift(mono_u, -1.0),
    }
    .to_bernstein();
    bernstein.windows(2).all(|controls| {
        (!nonnegative || controls[1] >= controls[0]) && (!nonpositive || controls[1] <= controls[0])
    })
}

fn preserves_acceleration_monotonicity(mono_u: &[f64], increasing: bool) -> bool {
    if mono_u.len() <= 3 {
        return true;
    }
    let jerk: Vec<f64> = mono_u
        .iter()
        .enumerate()
        .skip(3)
        .map(|(power, &coefficient)| (power * (power - 1) * (power - 2)) as f64 * coefficient)
        .collect();
    let degree = (mono_u.len() - 1) as f64;
    let conversion_operations = (degree + 1.0) * (degree + 2.0);
    let accumulated_epsilon = conversion_operations * f64::EPSILON;
    let conversion_gamma = accumulated_epsilon / (1.0 - accumulated_epsilon);
    let absolute_conversion_sum = mono_u.iter().rev().fold(0.0_f64, |sum, coefficient| {
        sum.mul_add(3.0, coefficient.abs())
    });
    let third_difference_scale = degree * (degree - 1.0) * (degree - 2.0);
    let roundoff = conversion_gamma * absolute_conversion_sum * third_difference_scale;
    if !roundoff.is_finite() {
        return false;
    }
    BezierPiece {
        u_start: -1.0,
        u_end: 1.0,
        coeffs: taylor_shift(&jerk, -1.0),
    }
    .to_bernstein()
    .iter()
    .all(|&value| {
        if increasing {
            value >= -roundoff
        } else {
            value <= roundoff
        }
    })
}

/// An endpoint-anchored rung is only allowed to hand its caller a piece whose
/// endpoint velocities are still the signal's: the position and acceleration
/// probes pass a velocity step at either end that a rung solved from the
/// relative travel is free to leave there.
fn candidate_ok(
    mono_u: &[f64],
    h: f64,
    tol: FitTol,
    truth_p: &dyn Fn(f64) -> f64,
    truth_v: &dyn Fn(f64) -> f64,
    truth_a: &dyn Fn(f64) -> f64,
    velocity_budget: f64,
    policy: LadderPolicy,
) -> bool {
    let dd_scale = (2.0 / h) * (2.0 / h);
    let endpoint_velocity_anchored = !policy.endpoint_anchored
        || [-1.0, 1.0]
            .into_iter()
            .all(|u| (eval_mono_d(mono_u, u) * (2.0 / h) - truth_v(u)).abs() <= velocity_budget);
    endpoint_velocity_anchored
        && (!policy.enforce_velocity_sign || preserves_certified_velocity_sign(mono_u, truth_v))
        && policy
            .acceleration_monotonicity
            .is_none_or(|increasing| preserves_acceleration_monotonicity(mono_u, increasing))
        && LADDER_PROBES_U.iter().all(|&u| {
            (eval_mono(mono_u, u) - truth_p(u)).abs() <= tol.pos_mm
                && (eval_mono_dd(mono_u, u) * dd_scale - truth_a(u)).abs() <= tol.accel_mm_s2
        })
}

pub(crate) fn ladder_fit(
    base: &[f64],
    h: f64,
    tol: FitTol,
    truth_p: &dyn Fn(f64) -> f64,
    truth_a: &dyn Fn(f64) -> f64,
    truth_v: &dyn Fn(f64) -> f64,
    endpoint_delta: f64,
    velocity_budget: f64,
    policy: LadderPolicy,
) -> Result<Vec<f64>, LadderFailure> {
    if !policy.endpoint_anchored {
        let constant = vec![truth_p(0.0)];
        if candidate_ok(
            &constant,
            h,
            tol,
            truth_p,
            truth_v,
            truth_a,
            velocity_budget,
            policy,
        ) {
            return Ok(constant);
        }
        let s = 0.5 * h;
        let quadratic = vec![truth_p(0.0), truth_v(0.0) * s, 0.5 * truth_a(0.0) * s * s];
        if candidate_ok(
            &quadratic,
            h,
            tol,
            truth_p,
            truth_v,
            truth_a,
            velocity_budget,
            policy,
        ) {
            return Ok(quadratic);
        }
    }
    if policy.endpoint_anchored
        && (policy.acceleration_monotonicity.is_none() || truth_a(-1.0) == truth_a(1.0))
    {
        let anchored_acceleration_quadratic =
            anchored_acceleration_quadratic_in_u(truth_p(-1.0), truth_a(0.0), h, endpoint_delta);
        if candidate_ok(
            &anchored_acceleration_quadratic,
            h,
            tol,
            truth_p,
            truth_v,
            truth_a,
            velocity_budget,
            policy,
        ) {
            return Ok(anchored_acceleration_quadratic);
        }
        let anchored_quadratic = quadratic_in_u(truth_p(-1.0), truth_v(-1.0), h, endpoint_delta);
        if candidate_ok(
            &anchored_quadratic,
            h,
            tol,
            truth_p,
            truth_v,
            truth_a,
            velocity_budget,
            policy,
        ) {
            return Ok(anchored_quadratic);
        }
        let cubic = cubic_in_u(
            (truth_p(-1.0), truth_v(-1.0)),
            (truth_p(1.0), truth_v(1.0)),
            h,
            endpoint_delta,
        );
        if candidate_ok(
            &cubic,
            h,
            tol,
            truth_p,
            truth_v,
            truth_a,
            velocity_budget,
            policy,
        ) {
            return Ok(cubic);
        }
    }
    let quintic = ladder_candidate(base, 5, truth_p);
    if candidate_ok(
        &quintic,
        h,
        tol,
        truth_p,
        truth_v,
        truth_a,
        velocity_budget,
        policy,
    ) {
        return Ok(quintic);
    }
    let mut last = quintic;
    if h >= policy.high_degree_span_floor {
        for degree in [6, 7, 9, 11, 13] {
            let candidate = ladder_candidate(base, degree, truth_p);
            if candidate_ok(
                &candidate,
                h,
                tol,
                truth_p,
                truth_v,
                truth_a,
                velocity_budget,
                policy,
            ) {
                return Ok(candidate);
            }
            last = candidate;
        }
    }
    let dd_scale = (2.0 / h) * (2.0 / h);
    let u = *LADDER_PROBES_U
        .iter()
        .max_by(|&&left, &&right| {
            let score = |probe| {
                let position = (eval_mono(&last, probe) - truth_p(probe)).abs() / tol.pos_mm;
                let velocity = (eval_mono_d(&last, probe) * (2.0 / h) - truth_v(probe)).abs()
                    / velocity_budget;
                let acceleration = (eval_mono_dd(&last, probe) * dd_scale - truth_a(probe)).abs()
                    / tol.accel_mm_s2;
                position.max(velocity).max(acceleration)
            };
            score(left).total_cmp(&score(right))
        })
        .expect("ladder probes are empty");
    let source_position = truth_p(u);
    let source_velocity = truth_v(u);
    let source_acceleration = truth_a(u);
    let candidate_position = eval_mono(&last, u);
    let candidate_velocity = eval_mono_d(&last, u) * (2.0 / h);
    let candidate_acceleration = eval_mono_dd(&last, u) * dd_scale;
    Err(LadderFailure {
        u,
        position_error: (candidate_position - source_position).abs(),
        velocity_error: (candidate_velocity - source_velocity).abs(),
        acceleration_error: (candidate_acceleration - source_acceleration).abs(),
        source_position,
        source_velocity,
        source_acceleration,
        candidate_position,
        candidate_velocity,
        candidate_acceleration,
        left_position: truth_p(-1.0),
        left_velocity: truth_v(-1.0),
        left_acceleration: truth_a(-1.0),
        right_position: truth_p(1.0),
        right_velocity: truth_v(1.0),
        right_acceleration: truth_a(1.0),
    })
}

pub(crate) fn exact_piece(mono_u: &[f64], u_start: f64, u_end: f64, h: f64) -> BezierPiece {
    let mut coeffs = taylor_shift(mono_u, -1.0);
    let mut scale = 1.0;
    for coefficient in &mut coeffs {
        *coefficient *= scale;
        scale *= 2.0 / h;
    }
    BezierPiece {
        u_start,
        u_end,
        coeffs,
    }
}
