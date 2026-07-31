//! Analytic triple-limited (velocity / acceleration / jerk) motion profile.
//!
//! A profile is a sequence of constant-jerk phases — the 7-segment S-curve and
//! its degeneracies — solved in closed form and evaluated exactly. Nothing is
//! integrated on a grid, so rest (`v = 0`) is an ordinary phase boundary rather
//! than a singularity, the acceleration is stored per phase (it is never a
//! finite difference of `v`), and sampling density is decoupled from
//! correctness: every sample is exact wherever it is taken.

use super::VelocityError;

const EPS: f64 = 1e-12;

const BRACKET_HALVINGS: u32 = 64;

const LENGTH_CLOSURE_TOL: f64 = 1e-9;

/// A hold-free shape has no free parameter left, so it can only be taken when
/// the boundary data it is over-determined by already agrees with it — to within
/// the accumulated rounding of the arithmetic that produced that data, not to
/// within the planner's much looser self-consistency guard.
const OVERDETERMINED_SHAPE_TOL: f64 = 1e-12;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StraightPhase {
    pub t0: f64,
    pub dt: f64,
    pub s0: f64,
    pub v0: f64,
    pub a0: f64,
    pub j: f64,
}

impl StraightPhase {
    pub fn state_at(&self, tau: f64) -> (f64, f64, f64) {
        (
            self.s0 + self.advance_of(tau),
            self.v0 + self.a0 * tau + 0.5 * self.j * tau * tau,
            self.a0 + self.j * tau,
        )
    }

    pub fn end_state(&self) -> (f64, f64, f64) {
        self.state_at(self.dt)
    }

    fn advance_of(&self, tau: f64) -> f64 {
        self.v0 * tau + 0.5 * self.a0 * tau * tau + self.j * tau * tau * tau / 6.0
    }

    /// Local time at which this phase has advanced `ds` in arc length. The
    /// integrand is the (non-negative) speed, so the root is unique and
    /// bracketed by `[0, dt]`; the bracket is halved until it collapses onto
    /// adjacent floats.
    fn solve_tau(&self, ds: f64) -> f64 {
        if ds <= 0.0 || self.dt <= 0.0 {
            return 0.0;
        }
        let (mut lo, mut hi) = (0.0_f64, self.dt);
        if self.advance_of(hi) <= ds {
            return hi;
        }
        for _ in 0..BRACKET_HALVINGS {
            let mid = 0.5 * (lo + hi);
            if mid <= lo || mid >= hi {
                break;
            }
            if self.advance_of(mid) <= ds {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        0.5 * (lo + hi)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BoundaryInfeasibility {
    NonFinite,
    AccelOverLimit { a: f64, a_max: f64 },
    UnboundedJerkWithAccelBoundary { a: f64 },
    UnwindBelowRest { v: f64 },
    UnwindOverCeiling { v: f64, v_max: f64 },
    LengthTooShort { length: f64, minimum: f64 },
    LengthNotClosed { requested: f64, achieved: f64 },
}

pub struct Profile {
    phases: Vec<StraightPhase>,
    exit: (f64, f64),
    length: f64,
}

impl Profile {
    /// `(v, a)` at arc-length `s`, clamped to the profile's span. The exit state
    /// is stored, not reconstructed, so a sub-ulp `solve_tau` shortfall cannot
    /// leave a residual acceleration at the far end.
    pub fn at(&self, s: f64) -> (f64, f64) {
        let s = s.clamp(0.0, self.length);
        if self.phases.is_empty() || s >= self.length - EPS {
            return self.exit;
        }
        let p = self.phases[self.locate(s)];
        let (_, v, a) = p.state_at(p.solve_tau(s - p.s0));
        (v, a)
    }

    pub fn phases(&self) -> &[StraightPhase] {
        &self.phases
    }

    pub fn length(&self) -> f64 {
        self.length
    }

    fn locate(&self, s: f64) -> usize {
        let mut i = 0;
        while i + 1 < self.phases.len() && self.phases[i + 1].s0 <= s {
            i += 1;
        }
        i
    }
}

fn triangular_dv_ceiling(a_max: f64, j_max: f64) -> f64 {
    a_max * a_max / j_max
}

fn unclipped_ramp_time(dv: f64, a_max: f64, j_max: f64) -> f64 {
    if dv <= 0.0 {
        return 0.0;
    }
    if !j_max.is_finite() {
        return dv / a_max;
    }
    if dv <= triangular_dv_ceiling(a_max, j_max) {
        2.0 * (dv / j_max).sqrt()
    } else {
        a_max / j_max + dv / a_max
    }
}

/// The emitter drops a flank whose speed change is sub-`EPS`, so its time is
/// quoted as zero too.
fn ramp_time(dv: f64, a_max: f64, j_max: f64) -> f64 {
    if dv <= EPS {
        return 0.0;
    }
    unclipped_ramp_time(dv, a_max, j_max)
}

/// A zero-accel-to-zero-accel ramp is point-symmetric about its midpoint, so its
/// mean speed is exactly `(v0 + v1) / 2`.
fn ramp_dist(v0: f64, v1: f64, a_max: f64, j_max: f64) -> f64 {
    0.5 * (v0 + v1) * ramp_time((v1 - v0).abs(), a_max, j_max)
}

/// Ramp distance priced *without* the sub-`EPS` discard. The peak search must
/// never quote a flank cheaper than the chain will pay for it: a flank whose
/// speed change straddles `EPS` still costs `2*v*sqrt(dv/j)` of arc, and a peak
/// chosen against a free quote leaves the cruise budget negative and the chain
/// overrunning its member.
fn ramp_price(v0: f64, v1: f64, a_max: f64, j_max: f64) -> f64 {
    0.5 * (v0 + v1) * unclipped_ramp_time((v1 - v0).abs(), a_max, j_max)
}

/// Peak reachable when both flanks saturate `a_max`: the ramp-distance sum is
/// then a quadratic in the peak.
fn trapezoidal_peak(v0: f64, v1: f64, length: f64, a_max: f64, j_max: f64) -> Option<f64> {
    let half_knee = 0.5 * triangular_dv_ceiling(a_max, j_max);
    let disc =
        half_knee * half_knee - half_knee * (v0 + v1) + 0.5 * (v0 * v0 + v1 * v1) + a_max * length;
    if disc < 0.0 {
        return None;
    }
    Some(disc.sqrt() - half_knee)
}

/// Peak reachable when both flanks stay jerk-limited and are mirror images
/// (`v0 == v1`): the ramp-distance sum reduces to the depressed cubic
/// `u^3 + (2 v0 / j) u = length / (2 j)` in `u = sqrt((peak - v0) / j)`, whose
/// non-negative `p` and negative `q` admit exactly one real root.
fn symmetric_triangular_peak(v_ends: f64, length: f64, j_max: f64) -> f64 {
    let half_q = 0.25 * length / j_max;
    let third_p = 2.0 * v_ends / (3.0 * j_max);
    let radical = (half_q * half_q + third_p * third_p * third_p).sqrt();
    let u = libm::cbrt(half_q + radical) + libm::cbrt(half_q - radical);
    v_ends + j_max * u * u
}

fn accel_limited_peak(v0: f64, v1: f64, length: f64, a_max: f64) -> f64 {
    (a_max * length + 0.5 * (v0 * v0 + v1 * v1)).sqrt()
}

/// Highest peak speed `<= v_max` whose ramp-up-from-`v0` plus ramp-down-to-`v1`
/// fits in `length`; surplus length becomes cruise at the caller's hands.
///
/// Each flank is jerk-limited below `a_max^2 / j_max` of speed change and
/// accel-limited above it, so `v0 + knee` and `v1 + knee` cut `[floor, v_max]`
/// into at most three intervals on each of which the distance sum is a single
/// algebraic form. The pure forms are solved outright; the mixed interval is
/// irrational in the peak and is bisected inside its own bracket.
fn peak_velocity(v0: f64, v1: f64, length: f64, v_max: f64, a_max: f64, j_max: f64) -> f64 {
    let span = |vp: f64| ramp_price(v0, vp, a_max, j_max) + ramp_price(vp, v1, a_max, j_max);
    if span(v_max) <= length {
        return v_max;
    }
    let floor = v0.max(v1);
    if length <= span(floor) {
        return floor;
    }
    if !j_max.is_finite() {
        return accel_limited_peak(v0, v1, length, a_max).min(v_max);
    }

    let knee = triangular_dv_ceiling(a_max, j_max);
    if let Some(vp) = trapezoidal_peak(v0, v1, length, a_max, j_max) {
        if vp >= floor + knee {
            return vp.min(v_max);
        }
    }
    if v0 == v1 {
        let vp = symmetric_triangular_peak(v0, length, j_max);
        if vp <= v0 + knee {
            return vp.min(v_max);
        }
    }

    let (mut lo, mut hi) = (floor, v_max);
    for edge in [v0 + knee, v1 + knee] {
        if edge > lo && edge < hi {
            if span(edge) <= length {
                lo = edge;
            } else {
                hi = edge;
            }
        }
    }
    for _ in 0..BRACKET_HALVINGS {
        let mid = 0.5 * (lo + hi);
        if mid <= lo || mid >= hi {
            break;
        }
        if span(mid) <= length {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    debug_assert!(hi - lo <= 8.0 * f64::EPSILON * (1.0 + hi));
    lo
}

struct Builder {
    phases: Vec<StraightPhase>,
    t: f64,
    s: f64,
    v: f64,
    a: f64,
}

impl Builder {
    fn new(v: f64, a: f64) -> Self {
        Self {
            phases: Vec::new(),
            t: 0.0,
            s: 0.0,
            v,
            a,
        }
    }

    fn phase(&mut self, j: f64, dt: f64) {
        if dt <= 0.0 {
            return;
        }
        let p = StraightPhase {
            t0: self.t,
            dt,
            s0: self.s,
            v0: self.v,
            a0: self.a,
            j,
        };
        let (s, v, a) = p.end_state();
        self.phases.push(p);
        self.t += dt;
        self.s = s;
        self.v = v;
        self.a = a;
    }

    /// Jerk on, optional accel hold, jerk off — or a single accel hold under
    /// infinite jerk, where the acceleration steps.
    fn ramp(&mut self, v_to: f64, a_max: f64, j_max: f64) {
        let dv = v_to - self.v;
        if dv.abs() <= EPS {
            return;
        }
        let dir = dv.signum();
        let adv = dv.abs();
        if !j_max.is_finite() {
            self.a = dir * a_max;
            self.phase(0.0, adv / a_max);
            self.a = 0.0;
            return;
        }
        if adv <= triangular_dv_ceiling(a_max, j_max) {
            let t_j = (adv / j_max).sqrt();
            self.phase(dir * j_max, t_j);
            self.phase(-dir * j_max, t_j);
        } else {
            let t_j = a_max / j_max;
            let t_h = adv / a_max - t_j;
            self.phase(dir * j_max, t_j);
            self.phase(0.0, t_h);
            self.phase(-dir * j_max, t_j);
        }
        self.a = 0.0;
    }

    /// Distance [`Builder::ramp`] would cover from the current state — the same
    /// legs, summed without emitting them, so the cruise budget cannot disagree
    /// with what the descent actually costs.
    fn ramp_span(&self, v_to: f64, a_max: f64, j_max: f64) -> f64 {
        let dv = v_to - self.v;
        if dv.abs() <= EPS {
            return 0.0;
        }
        let dir = dv.signum();
        let adv = dv.abs();
        if !j_max.is_finite() {
            let dt = adv / a_max;
            return self.v * dt + 0.5 * dir * a_max * dt * dt;
        }
        let legs = if adv <= triangular_dv_ceiling(a_max, j_max) {
            let t_j = (adv / j_max).sqrt();
            [(dir * j_max, t_j), (-dir * j_max, t_j), (0.0, 0.0)]
        } else {
            let t_j = a_max / j_max;
            [
                (dir * j_max, t_j),
                (0.0, adv / a_max - t_j),
                (-dir * j_max, t_j),
            ]
        };
        let mut state = (0.0, self.v, 0.0);
        for (j, dt) in legs {
            state = advance(state, j, dt);
        }
        state.0
    }

    /// Ramp to a peak, optionally cruise, ramp down to `v_to`, spanning
    /// `length`. Requires — and leaves — zero acceleration.
    ///
    /// The descent is measured with [`Builder::ramp_span`] from the peak actually
    /// reached, not quoted from the peak asked for: the two differ by rounding,
    /// and either side of `EPS` that turns a skipped ramp into an emitted one
    /// whose arc nothing paid for.
    fn run(&mut self, v_to: f64, length: f64, v_max: f64, a_max: f64, j_max: f64) {
        debug_assert!(self.a.abs() <= EPS);
        let v_peak = peak_velocity(self.v, v_to, length, v_max, a_max, j_max);
        let s_entry = self.s;
        self.ramp(v_peak, a_max, j_max);
        let cruise = length - (self.s - s_entry) - self.ramp_span(v_to, a_max, j_max);
        if cruise > EPS && self.v > EPS {
            self.phase(0.0, cruise / self.v);
        }
        self.ramp(v_to, a_max, j_max);
    }

    fn unwind_accel_to_zero(&mut self, j_max: f64) {
        let a = self.a;
        if a == 0.0 {
            return;
        }
        let dt = a.abs() / j_max;
        self.phase(-a / dt, dt);
        self.a = 0.0;
    }

    fn wind_accel_up_to(&mut self, a: f64, j_max: f64) {
        if a == 0.0 {
            return;
        }
        let dt = a.abs() / j_max;
        self.phase(a / dt, dt);
    }

    /// Slew the acceleration to `a_to` over `dt`, landing on it exactly rather
    /// than on `a0 + j*dt`.
    fn jerk_to(&mut self, a_to: f64, dt: f64) {
        if dt > 0.0 {
            self.phase((a_to - self.a) / dt, dt);
        }
        self.a = a_to;
    }
}

/// Speed change of a constant-jerk swing between zero acceleration and `a`.
fn swing_dv(a: f64, j_max: f64) -> f64 {
    a * a.abs() / (2.0 * j_max)
}

/// Arc length of a constant-jerk swing between zero acceleration and `a`, whose
/// zero-acceleration end sits at `v_at_zero_accel`.
fn swing_dist(v_at_zero_accel: f64, a: f64, j_max: f64) -> f64 {
    let dt = a.abs() / j_max;
    v_at_zero_accel * dt + a * dt * dt / 6.0
}

/// One constant-jerk leg applied to an `(s, v, a)` state, by the same arithmetic
/// [`Builder::phase`] uses, so a leg measured and a leg emitted agree bit for bit.
fn advance(state: (f64, f64, f64), j: f64, dt: f64) -> (f64, f64, f64) {
    if dt <= 0.0 {
        return state;
    }
    StraightPhase {
        t0: 0.0,
        dt,
        s0: state.0,
        v0: state.1,
        a0: state.2,
        j,
    }
    .end_state()
}

/// Speed change of a constant-jerk slew between two acceleration levels.
fn slew_dv(a_from: f64, a_to: f64, j_max: f64) -> f64 {
    (a_to - a_from).abs() * (a_to + a_from) / (2.0 * j_max)
}

/// `a0 -> a_hold` (jerk), hold, `a_hold -> a1` (jerk): the acceleration-monotone
/// chain that carries a boundary pair across a member without ever relaxing to
/// zero acceleration. The hold level alone decides how much length the speed
/// change costs — the hold *time* is whatever closes `v1 - v0`.
#[derive(Clone, Copy)]
struct HoldRamp {
    a_hold: f64,
    t_in: f64,
    t_hold: f64,
    t_out: f64,
}

impl HoldRamp {
    fn new(entry: (f64, f64), exit: (f64, f64), a_hold: f64, j_max: f64) -> Self {
        let (v0, a0) = entry;
        let (v1, a1) = exit;
        let slewed = slew_dv(a0, a_hold, j_max) + slew_dv(a_hold, a1, j_max);
        Self {
            a_hold,
            t_in: (a_hold - a0).abs() / j_max,
            t_hold: (v1 - v0 - slewed) / a_hold,
            t_out: (a1 - a_hold).abs() / j_max,
        }
    }

    fn legs(&self, a_exit: f64) -> [(f64, f64); 3] {
        [
            (self.a_hold, self.t_in),
            (self.a_hold, self.t_hold),
            (a_exit, self.t_out),
        ]
    }

    /// A member that is one slice of an ongoing jerk phase: the acceleration
    /// slews straight from `a0` to `a1` with no hold at all.
    fn slew(entry: (f64, f64), exit: (f64, f64), j_max: f64) -> Self {
        Self {
            a_hold: exit.1,
            t_in: (exit.1 - entry.1).abs() / j_max,
            t_hold: 0.0,
            t_out: 0.0,
        }
    }

    fn traverse(&self, entry: (f64, f64), a_exit: f64) -> (f64, f64, f64) {
        let (v0, a0) = entry;
        let mut state = (0.0, v0, a0);
        for (a_to, dt) in self.legs(a_exit) {
            if dt > 0.0 {
                state = advance(state, (a_to - state.2) / dt, dt);
            }
            state.2 = a_to;
        }
        state
    }

    fn span(&self, entry: (f64, f64), a_exit: f64) -> f64 {
        self.traverse(entry, a_exit).0
    }

    fn emit(&self, b: &mut Builder, a_exit: f64) {
        for (a_to, dt) in self.legs(a_exit) {
            b.jerk_to(a_to, dt);
        }
    }
}

/// Hold levels at which the two jerk slews alone already close `v1 - v0`, so no
/// hold time is left to place. They are the only places where the sign of the
/// hold time can turn over, and each region of `slew_dv` contributes at most one
/// pair of them.
fn hold_time_zeros(entry: (f64, f64), exit: (f64, f64), j_max: f64) -> [Option<f64>; 4] {
    let (v0, a0) = entry;
    let (v1, a1) = exit;
    let mean_square = 0.5 * (a0 * a0 + a1 * a1);
    let (below, above) = (a0.min(a1), a0.max(a1));
    let outward = mean_square + j_max * (v1 - v0);
    let inward = mean_square - j_max * (v1 - v0);
    let outward = if outward >= 0.0 {
        outward.sqrt()
    } else {
        f64::NAN
    };
    let inward = if inward >= 0.0 {
        inward.sqrt()
    } else {
        f64::NAN
    };
    let keep = |x: f64, admissible: bool| if admissible { Some(x) } else { None };
    [
        keep(outward, outward >= above),
        keep(-outward, -outward >= above),
        keep(inward, inward <= below),
        keep(-inward, -inward <= below),
    ]
}

/// Solve the boundary pair on a member too short to relax to zero acceleration.
///
/// A member that is one slice of an ongoing jerk phase is the degenerate shape —
/// no hold at all — and it has no free parameter, so it is taken only when it
/// closes both the length and the exit speed on its own.
///
/// Otherwise the hold level is the single free parameter, and it is admissible
/// exactly where the hold time it implies is non-negative — a union of at most
/// two intervals per sign, delimited by [`hold_time_zeros`] and the acceleration
/// rails. Span is monotone across each such window, so the window's own extremes
/// bound the lengths it can span and the level spanning `length` is bracketed and
/// bisected inside it. The window whose solution traverses fastest wins.
///
/// The reachable lengths are a union of those windows' ranges and need not be
/// contiguous: a boundary pair whose speed change the jerk slews already close
/// exactly can only be *lengthened* by braking, which costs a whole excursion to
/// negative acceleration. A refusal therefore reports the nearest length that is
/// reachable, never a bound the pair does not actually respect.
fn hold_ramp_chain(
    entry: (f64, f64),
    exit: (f64, f64),
    length: f64,
    a_max: f64,
    j_max: f64,
    zero_crossing_minimum: f64,
) -> Result<Vec<StraightPhase>, VelocityError> {
    let ramp_at = |a_hold: f64| HoldRamp::new(entry, exit, a_hold, j_max);
    let span_at = |a_hold: f64| ramp_at(a_hold).span(entry, exit.1);
    let closes = |achieved: f64, wanted: f64| {
        (achieved - wanted).abs() <= OVERDETERMINED_SHAPE_TOL * (1.0 + wanted.abs())
    };

    let slew = HoldRamp::slew(entry, exit, j_max);
    let (slew_span, slew_v, _) = slew.traverse(entry, exit.1);
    if closes(slew_span, length) && closes(slew_v, exit.0) {
        let mut b = Builder::new(entry.0, entry.1);
        slew.emit(&mut b, exit.1);
        return Ok(b.phases);
    }

    let zeros = hold_time_zeros(entry, exit, j_max);
    let mut minimum = zero_crossing_minimum;
    let mut fastest: Option<(f64, f64)> = None;

    for sign in [1.0_f64, -1.0] {
        let mut marks = [0.0_f64; 4];
        let mut n = 1;
        for zero in zeros.iter().flatten() {
            if zero * sign > 0.0 && zero.abs() < a_max {
                marks[n] = *zero;
                n += 1;
            }
        }
        marks[n] = sign * a_max;
        n += 1;
        let marks = &mut marks[..n];
        marks.sort_by(|x, y| x.abs().total_cmp(&y.abs()));

        for window in marks.windows(2) {
            let (near, far) = (window[0], window[1]);
            if ramp_at(0.5 * (near + far)).t_hold < 0.0 {
                continue;
            }
            let near_span = if near == 0.0 {
                f64::INFINITY
            } else {
                span_at(near)
            };
            let far_span = span_at(far);
            let (tight, slack) = if far_span <= near_span {
                (far, near)
            } else {
                (near, far)
            };
            let tight_span = span_at(tight);
            let reachable = |span: f64| if span > length { Some(span) } else { None };
            for candidate in [
                reachable(tight_span),
                reachable(near_span),
                reachable(far_span),
            ]
            .into_iter()
            .flatten()
            {
                minimum = minimum.min(candidate);
            }
            if length < tight_span && !closes(tight_span, length) {
                continue;
            }
            let slack = if slack == 0.0 {
                let mut relaxed = 0.5 * tight;
                while span_at(relaxed) < length {
                    relaxed *= 0.5;
                }
                relaxed
            } else if span_at(slack) < length {
                continue;
            } else {
                slack
            };

            let (mut lo, mut hi) = (slack, tight);
            for _ in 0..BRACKET_HALVINGS {
                let mid = 0.5 * (lo + hi);
                if mid == lo || mid == hi {
                    break;
                }
                if span_at(mid) >= length {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            let a_hold = if closes(tight_span, length) {
                tight
            } else if (span_at(hi) - length).abs() <= (span_at(lo) - length).abs() {
                hi
            } else {
                lo
            };
            let ramp = ramp_at(a_hold);
            let traversal = ramp.t_in + ramp.t_hold + ramp.t_out;
            if fastest.is_none_or(|(best, _)| traversal < best) {
                fastest = Some((traversal, a_hold));
            }
        }
    }

    let Some((_, a_hold)) = fastest else {
        return Err(VelocityError::InfeasibleBoundary(
            BoundaryInfeasibility::LengthTooShort { length, minimum },
        ));
    };
    let mut b = Builder::new(entry.0, entry.1);
    ramp_at(a_hold).emit(&mut b, exit.1);
    Ok(b.phases)
}

pub fn straight_chain(
    v0: f64,
    v1: f64,
    length: f64,
    v_max: f64,
    a_max: f64,
    j_max: f64,
) -> Vec<StraightPhase> {
    let mut b = Builder::new(v0, 0.0);
    b.run(v1, length, v_max, a_max, j_max);
    b.phases
}

/// Plan a triple-limited profile from `v0` to `v1` across `length` under a flat
/// ceiling `v_max`. Entry and exit sit at zero acceleration (run anchors).
pub fn plan(v0: f64, v1: f64, length: f64, v_max: f64, a_max: f64, j_max: f64) -> Profile {
    let phases = straight_chain(v0, v1, length, v_max, a_max, j_max);
    let span = phases.last().map_or(0.0, |p| p.end_state().0);
    Profile {
        phases,
        exit: (v1, 0.0),
        length: span,
    }
}

/// Plan across `length` between boundary states that both carry acceleration.
/// Two shapes cover the boundary-value problem. When the member is long enough
/// to relax acceleration to zero, the chain is the ordinary run bracketed by the
/// entry and exit slews — the shortest such member spans
/// `zero_crossing_minimum`. Below that the acceleration can never reach zero and
/// the chain becomes a single acceleration-monotone hold ramp; a pure
/// constant-acceleration slice of an ongoing ramp is that shape with both slews
/// empty, i.e. one zero-jerk phase.
pub fn straight_chain_between(
    entry: (f64, f64),
    exit: (f64, f64),
    length: f64,
    v_max: f64,
    a_max: f64,
    j_max: f64,
) -> Result<Vec<StraightPhase>, VelocityError> {
    let (v0, a0) = entry;
    let (v1, a1) = exit;
    let infeasible = |why| Err(VelocityError::InfeasibleBoundary(why));

    let finite = [v0, a0, v1, a1, length, v_max, a_max]
        .iter()
        .all(|x| x.is_finite());
    if !finite
        || j_max.is_nan()
        || !(v_max > 0.0 && a_max > 0.0 && j_max > 0.0 && length >= 0.0 && v0 >= 0.0 && v1 >= 0.0)
    {
        return infeasible(BoundaryInfeasibility::NonFinite);
    }
    for a in [a0, a1] {
        if a.abs() > a_max {
            return infeasible(BoundaryInfeasibility::AccelOverLimit { a, a_max });
        }
        if !j_max.is_finite() && a != 0.0 {
            return infeasible(BoundaryInfeasibility::UnboundedJerkWithAccelBoundary { a });
        }
    }

    let v_after_unwind = v0 + swing_dv(a0, j_max);
    let v_before_wind = v1 - swing_dv(a1, j_max);
    for v in [v_after_unwind, v_before_wind] {
        if v < 0.0 {
            return infeasible(BoundaryInfeasibility::UnwindBelowRest { v });
        }
    }
    for v in [v0, v1, v_after_unwind, v_before_wind] {
        if v > v_max {
            return infeasible(BoundaryInfeasibility::UnwindOverCeiling { v, v_max });
        }
    }

    let entry_dist = swing_dist(v_after_unwind, -a0, j_max);
    let exit_dist = swing_dist(v_before_wind, a1, j_max);
    let zero_crossing_minimum =
        entry_dist + exit_dist + ramp_dist(v_after_unwind, v_before_wind, a_max, j_max);

    let phases = if length >= zero_crossing_minimum {
        let mut b = Builder::new(v0, a0);
        b.unwind_accel_to_zero(j_max);
        b.run(v_before_wind, length - b.s - exit_dist, v_max, a_max, j_max);
        b.wind_accel_up_to(a1, j_max);
        b.phases
    } else {
        hold_ramp_chain(entry, exit, length, a_max, j_max, zero_crossing_minimum)?
    };

    let achieved = phases.last().map_or(0.0, |p| p.end_state().0);
    if (achieved - length).abs() > LENGTH_CLOSURE_TOL * (1.0 + length) {
        return infeasible(BoundaryInfeasibility::LengthNotClosed {
            requested: length,
            achieved,
        });
    }
    Ok(phases)
}

#[cfg(test)]
mod tests;
