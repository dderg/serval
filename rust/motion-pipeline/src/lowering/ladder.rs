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

pub(crate) const LADDER_MAX_DEGREE: usize = 13;

/// `(1 − u²)³` — triple zeros at ±1, so adding it preserves endpoint p/v/a.
const BUMP6: [f64; 7] = [1.0, 0.0, -3.0, 0.0, 3.0, 0.0, -1.0];
/// `u·(1 − u²)³`.
const BUMP7: [f64; 8] = [0.0, 1.0, 0.0, -3.0, 0.0, 3.0, 0.0, -1.0];

pub(crate) fn eval_mono(c: &[f64], x: f64) -> f64 {
    c.iter().rev().fold(0.0, |acc, &ck| acc * x + ck)
}
pub(crate) fn eval_mono_d(c: &[f64], x: f64) -> f64 {
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

/// An upper bound on the magnitude of the acceleration of the monomial-in-u
/// piece `mono_u` over its span of `h` seconds: the largest Bernstein
/// coefficient of its second derivative.
pub(crate) fn acceleration_bound(mono_u: &[f64], h: f64) -> f64 {
    let second: Vec<f64> = mono_u
        .iter()
        .enumerate()
        .skip(2)
        .map(|(power, &coefficient)| (power * (power - 1)) as f64 * coefficient)
        .collect();
    if second.is_empty() {
        return 0.0;
    }
    let peak = BezierPiece {
        u_start: -1.0,
        u_end: 1.0,
        coeffs: taylor_shift(&second, -1.0),
    }
    .to_bernstein()
    .iter()
    .fold(0.0_f64, |peak, control| peak.max(control.abs()));
    peak * (2.0 / h) * (2.0 / h)
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

#[derive(Debug, Clone, Copy)]
pub(crate) struct LadderPva {
    pub position: f64,
    pub velocity: f64,
    pub acceleration: f64,
}

pub(crate) struct LadderFailure {
    pub u: f64,
    pub error: LadderPva,
    pub source: LadderPva,
    pub candidate: LadderPva,
    pub left: LadderPva,
    pub right: LadderPva,
}

pub(crate) struct LadderMiss {
    pub closest_rung: Vec<f64>,
    pub failure: LadderFailure,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum AccelerationTrend {
    Increasing,
    Decreasing,
}

impl AccelerationTrend {
    pub(crate) fn of_direction(direction: f64) -> Self {
        if direction > 0.0 {
            Self::Increasing
        } else {
            Self::Decreasing
        }
    }
}

/// Only the ladder's own tests certify a span's velocity sign today.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum VelocitySign {
    #[cfg(test)]
    Certified,
    Unconstrained,
}

/// Which of the signal's own state every span seam carries, whichever rung
/// wins.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum SeamContinuity {
    /// Position only: the two anchored quadratics may solve an endpoint
    /// velocity from the travel instead of reading it from the signal.
    Position,
    /// Position and velocity: the anchored quadratics are withheld.
    Velocity,
    /// Position, velocity and acceleration: the cubic, which never reads
    /// the endpoint acceleration, is withheld too, so every piece starts
    /// from the quintic Hermite and its endpoint-preserving bumps. An
    /// advance with no kernel after it turns a fit's acceleration step at a
    /// seam into a velocity step of its motor.
    Acceleration,
}

/// Every rung is endpoint-anchored: it reproduces the span's own endpoint
/// positions, so the span seams stay C0 whichever rung wins.
///
/// `high_degree_span_floor` is the shortest span whose endpoint data still
/// carries enough signal for a rung solved from endpoint *acceleration* to
/// mean anything. Below it the quintic base and its bump corrections read
/// sampling noise amplified by `(2/h)²`, so only the cubic — which never
/// touches endpoint acceleration — is attempted.
#[derive(Debug, Clone, Copy)]
pub(crate) struct LadderPolicy {
    pub velocity_sign: VelocitySign,
    pub acceleration_monotonicity: Option<AccelerationTrend>,
    pub high_degree_span_floor: f64,
    pub seam_continuity: SeamContinuity,
}

#[cfg(test)]
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

/// The span's own convolution truth, read by ladder node `u ∈ [-1, 1]`.
pub(crate) struct LadderTruth<'a> {
    pub position: &'a dyn Fn(f64) -> f64,
    pub velocity: &'a dyn Fn(f64) -> f64,
    pub acceleration: &'a dyn Fn(f64) -> f64,
    /// A rung's worst miss over the tolerance where a span reaching a
    /// square-root onset is held to it between the probes; zero elsewhere.
    /// Read only once a rung has passed the probes, or to rank the misses;
    /// unless it ranks them, any miss past the tolerance may stand for the
    /// worst.
    pub held_miss: &'a dyn Fn(&[f64]) -> f64,
    /// Whether a miss's closest rung is the closest at the held points as
    /// well: only where the closest rung is kept, at the floor.
    pub held_ranks_misses: bool,
}

/// The signal at one node `x ∈ [-1, 1]` between the ladder's probes.
#[derive(Debug, Clone, Copy, Default)]
pub(crate) struct HeldSample {
    pub x: f64,
    pub position: f64,
    pub acceleration: f64,
}

/// The worst miss of a candidate at the held samples, over the tolerance.
pub(crate) fn held_miss(mono_u: &[f64], h: f64, held: &[HeldSample], tol: FitTol) -> f64 {
    let dd_scale = (2.0 / h) * (2.0 / h);
    held.iter()
        .map(|sample| {
            let position = (eval_mono(mono_u, sample.x) - sample.position).abs() / tol.pos_mm;
            let acceleration = (eval_mono_dd(mono_u, sample.x) * dd_scale - sample.acceleration)
                .abs()
                / tol.accel_mm_s2;
            position.max(acceleration)
        })
        .fold(0.0, f64::max)
}

/// The span geometry the rungs are solved over: its duration and the travel
/// the signal itself measures across it.
#[derive(Debug, Clone, Copy)]
pub(crate) struct LadderSpan {
    pub h: f64,
    pub endpoint_delta: f64,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct LadderBudget {
    pub tol: FitTol,
    pub velocity: f64,
}

fn preserves_acceleration_monotonicity(mono_u: &[f64], trend: AccelerationTrend) -> bool {
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
    .all(|&value| match trend {
        AccelerationTrend::Increasing => value >= -roundoff,
        AccelerationTrend::Decreasing => value <= roundoff,
    })
}

/// A rung's position and acceleration errors at the ladder's probes, each
/// read once for both its acceptance and, where it is refused, its rank:
/// acceptance reads them in order up to the first past the tolerance, and
/// only a refused rung's rank reads the rest.
struct ProbeErrors<'r> {
    mono_u: &'r [f64],
    dd_scale: f64,
    position: [f64; LADDER_PROBES_U.len()],
    acceleration: [f64; LADDER_PROBES_U.len()],
    read: usize,
}

impl<'r> ProbeErrors<'r> {
    fn of(mono_u: &'r [f64], h: f64) -> Self {
        Self {
            mono_u,
            dd_scale: (2.0 / h) * (2.0 / h),
            position: [0.0; LADDER_PROBES_U.len()],
            acceleration: [0.0; LADDER_PROBES_U.len()],
            read: 0,
        }
    }

    fn read_next(&mut self, truth: &LadderTruth) {
        let (index, u) = (self.read, LADDER_PROBES_U[self.read]);
        self.position[index] = (eval_mono(self.mono_u, u) - (truth.position)(u)).abs();
        self.acceleration[index] =
            (eval_mono_dd(self.mono_u, u) * self.dd_scale - (truth.acceleration)(u)).abs();
        self.read += 1;
    }

    fn within(&mut self, truth: &LadderTruth, tol: FitTol) -> bool {
        while self.read < LADDER_PROBES_U.len() {
            self.read_next(truth);
            let index = self.read - 1;
            if !(self.position[index] <= tol.pos_mm && self.acceleration[index] <= tol.accel_mm_s2)
            {
                return false;
            }
        }
        true
    }

    fn all(&mut self, truth: &LadderTruth) -> &Self {
        while self.read < LADDER_PROBES_U.len() {
            self.read_next(truth);
        }
        self
    }
}

/// An endpoint-anchored rung is only allowed to hand its caller a piece whose
/// endpoint velocities are still the signal's: the position and acceleration
/// probes pass a velocity step at either end that a rung solved from the
/// relative travel is free to leave there. Where the truth holds the span
/// between the probes, the rung must hold the tolerance there too.
fn candidate_ok(
    errors: &mut ProbeErrors,
    h: f64,
    truth: &LadderTruth,
    budget: LadderBudget,
    policy: LadderPolicy,
) -> bool {
    let mono_u = errors.mono_u;
    [-1.0, 1.0].into_iter().all(|u| {
        (eval_mono_d(mono_u, u) * (2.0 / h) - (truth.velocity)(u)).abs() <= budget.velocity
    }) && errors.within(truth, budget.tol)
        && (match policy.velocity_sign {
            #[cfg(test)]
            VelocitySign::Certified => preserves_certified_velocity_sign(mono_u, truth.velocity),
            VelocitySign::Unconstrained => true,
        })
        && policy
            .acceleration_monotonicity
            .is_none_or(|trend| preserves_acceleration_monotonicity(mono_u, trend))
        && (truth.held_miss)(mono_u) <= 1.0
}

/// The probe a refused rung misses worst at and its miss there over the
/// budgets, the held miss counted where it ranks the rungs.
fn worst_miss(
    errors: &mut ProbeErrors,
    h: f64,
    truth: &LadderTruth,
    budget: LadderBudget,
) -> (f64, f64) {
    let errors = errors.all(truth);
    let mono_u = errors.mono_u;
    let (probe, score) = LADDER_PROBES_U
        .iter()
        .enumerate()
        .map(|(index, &probe)| {
            let position = errors.position[index] / budget.tol.pos_mm;
            let velocity = (eval_mono_d(mono_u, probe) * (2.0 / h) - (truth.velocity)(probe)).abs()
                / budget.velocity;
            let acceleration = errors.acceleration[index] / budget.tol.accel_mm_s2;
            (probe, position.max(velocity).max(acceleration))
        })
        .max_by(|left, right| left.1.total_cmp(&right.1))
        .expect("ladder probes are empty");
    if truth.held_ranks_misses {
        (probe, score.max((truth.held_miss)(mono_u)))
    } else {
        (probe, score)
    }
}

pub(crate) fn ladder_fit(
    base: &[f64],
    span: LadderSpan,
    truth: &LadderTruth,
    budget: LadderBudget,
    policy: LadderPolicy,
) -> Result<Vec<f64>, LadderMiss> {
    let mut closest: Option<(f64, f64, Vec<f64>)> = None;
    let accepted = climb(base, span, truth, budget, policy, |errors| {
        let (probe, score) = worst_miss(errors, span.h, truth, budget);
        if closest
            .as_ref()
            .is_none_or(|(_, closest_score, _)| score < *closest_score)
        {
            closest = Some((probe, score, errors.mono_u.to_vec()));
        }
    });
    if let Some(rung) = accepted {
        return Ok(rung);
    }
    let (u, _, closest) = closest.expect("a refused ladder ranked its quintic");
    Err(ladder_miss(closest, u, span.h, truth))
}

/// [`ladder_fit`]'s accepted rung, or `None` where it refuses them all,
/// for a caller that discards a refusal: the refused rungs go unranked,
/// which spares reading every probe of each and its held miss.
pub(crate) fn ladder_accept(
    base: &[f64],
    span: LadderSpan,
    truth: &LadderTruth,
    budget: LadderBudget,
    policy: LadderPolicy,
) -> Option<Vec<f64>> {
    climb(base, span, truth, budget, policy, |_| {})
}

/// The rungs in order, the first one `candidate_ok` accepts returned;
/// `refused` is handed the probe errors, as far as they were read, of each
/// rung from the quintic up that it refuses.
fn climb(
    base: &[f64],
    span: LadderSpan,
    truth: &LadderTruth,
    budget: LadderBudget,
    policy: LadderPolicy,
    mut refused: impl FnMut(&mut ProbeErrors),
) -> Option<Vec<f64>> {
    let LadderSpan { h, endpoint_delta } = span;
    let truth_p = truth.position;
    let truth_v = truth.velocity;
    let truth_a = truth.acceleration;
    if policy.seam_continuity < SeamContinuity::Acceleration
        && (policy.acceleration_monotonicity.is_none() || truth_a(-1.0) == truth_a(1.0))
    {
        if policy.seam_continuity == SeamContinuity::Position {
            let anchored_acceleration_quadratic = anchored_acceleration_quadratic_in_u(
                truth_p(-1.0),
                truth_a(0.0),
                h,
                endpoint_delta,
            );
            let mut errors = ProbeErrors::of(&anchored_acceleration_quadratic, h);
            if candidate_ok(&mut errors, h, truth, budget, policy) {
                return Some(anchored_acceleration_quadratic);
            }
            let anchored_quadratic =
                quadratic_in_u(truth_p(-1.0), truth_v(-1.0), h, endpoint_delta);
            let mut errors = ProbeErrors::of(&anchored_quadratic, h);
            if candidate_ok(&mut errors, h, truth, budget, policy) {
                return Some(anchored_quadratic);
            }
        }
        let cubic = cubic_in_u(
            (truth_p(-1.0), truth_v(-1.0)),
            (truth_p(1.0), truth_v(1.0)),
            h,
            endpoint_delta,
        );
        let mut errors = ProbeErrors::of(&cubic, h);
        if candidate_ok(&mut errors, h, truth, budget, policy) {
            return Some(cubic);
        }
    }
    let quintic = ladder_candidate(base, 5, truth_p);
    let mut errors = ProbeErrors::of(&quintic, h);
    if candidate_ok(&mut errors, h, truth, budget, policy) {
        return Some(quintic);
    }
    refused(&mut errors);
    if h >= policy.high_degree_span_floor {
        for degree in [6, 7, 9, 11, LADDER_MAX_DEGREE] {
            let candidate = ladder_candidate(base, degree, truth_p);
            let mut errors = ProbeErrors::of(&candidate, h);
            if candidate_ok(&mut errors, h, truth, budget, policy) {
                return Some(candidate);
            }
            refused(&mut errors);
        }
    }
    None
}

/// The miss a refused ladder reports: its closest rung and where, at the
/// probe `u` it misses worst, it strays from the truth.
fn ladder_miss(closest: Vec<f64>, u: f64, h: f64, truth: &LadderTruth) -> LadderMiss {
    let (truth_p, truth_v, truth_a) = (truth.position, truth.velocity, truth.acceleration);
    let source = LadderPva {
        position: truth_p(u),
        velocity: truth_v(u),
        acceleration: truth_a(u),
    };
    let dd_scale = (2.0 / h) * (2.0 / h);
    let candidate = LadderPva {
        position: eval_mono(&closest, u),
        velocity: eval_mono_d(&closest, u) * (2.0 / h),
        acceleration: eval_mono_dd(&closest, u) * dd_scale,
    };
    LadderMiss {
        closest_rung: closest,
        failure: LadderFailure {
            u,
            error: LadderPva {
                position: (candidate.position - source.position).abs(),
                velocity: (candidate.velocity - source.velocity).abs(),
                acceleration: (candidate.acceleration - source.acceleration).abs(),
            },
            source,
            candidate,
            left: LadderPva {
                position: truth_p(-1.0),
                velocity: truth_v(-1.0),
                acceleration: truth_a(-1.0),
            },
            right: LadderPva {
                position: truth_p(1.0),
                velocity: truth_v(1.0),
                acceleration: truth_a(1.0),
            },
        },
    }
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
