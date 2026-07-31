#![cfg_attr(not(test), allow(dead_code))]
//! Closed-form profile solver for a curved member (clothoid, and its arc and
//! line degeneracies).
//!
//! The acceleration disk and the jerk ball are treated as constraints to stay
//! *under*, never as surfaces to ride: the emitted chain is pure
//! [`StraightPhase`], so it carries zero representation error, and every phase
//! is *proved* inside by [`certify::is_certified`] rather than sample-checked.
//!
//! Soundness comes from one reduction. On `[0, length]` the curvature
//! `kappa(s) = kappa0 + sigma*s` is linear, so `|kappa|` peaks at an endpoint;
//! call that bound `K`. For any state with `v <= V`, `|a| <= A_c` and phase
//! jerk `|j| <= J_c`:
//!
//! ```text
//!   disk:  a^2 + (kappa v^2)^2                     <= A_c^2 + (K V^2)^2
//!   ball:  |(j - kappa^2 v^3, sigma v^3 + 3 kappa v a)|
//!            <= |(kappa^2 v^3, sigma v^3)| + |(j, 3 kappa v a)|
//!            <= V^3 * hypot(K^2, |sigma|) + J_c + 3 K V A_c
//! ```
//!
//! so choosing `A_c` and `J_c` so that the two right-hand sides come to exactly
//! `accel^2` and `jerk` makes every state of the chain feasible for the *whole*
//! member at once, with no dependence on where along it the state sits.
//! `caps_at` does that for a given top speed `V`; the top speed itself is the
//! single scalar the solver searches over, because `A_c` and `J_c` both fall as
//! `V` rises. At `kappa0 = sigma = 0` the budgets are untouched
//! (`A_c = accel`, `J_c = jerk`, `V = flat_ceiling`) and the whole solver
//! collapses onto [`profile::straight_chain_between`].
//!
//! The regimes the search moves between are therefore the three that bound the
//! top speed — flat ceiling, the disk `v^2 = accel/|kappa|`, the jerk rail
//! `v^3 = jerk/hypot(kappa^2, |sigma|)` — plus, inside a fixed budget, the
//! ordinary triple-limited phase alphabet: full-jerk swing, constant
//! acceleration, cruise. Where the disk pins the speed exactly at
//! `sqrt(accel/|kappa|)` no acceleration is left at all and cruise is the only
//! profile; [`cruise_only`] is that regime.

use super::VelocityError;
use super::certify;
use super::disk::Kinematics;
use super::profile::{self, BoundaryInfeasibility, StraightPhase};

/// Share of the jerk budget the curvature's own unavoidable `v^3` demand may
/// consume at the top speed, leaving the rest for tangential jerk and for the
/// `3 kappa v a` cross term.
const STEADY_JERK_SHARE: f64 = 0.5;

/// Share of the jerk budget left after the `v^3` demand that is spent on the
/// `3 kappa v a` cross term rather than on tangential jerk.
const NORMAL_ACCEL_SHARE: f64 = 0.5;

/// Relative inflation of the curvature term `kappa v^2` before it is deducted
/// from the acceleration disk, so a chain at the top speed sits strictly inside
/// the disk instead of exactly on its edge. It scales with the curvature term
/// itself and so vanishes on a straight member.
const DISK_RAIL_MARGIN: f64 = 1e-3;

const TOP_BISECT_ITERS: u32 = 48;
const ARC_BISECT_ITERS: u32 = 64;

/// Lowest top speed the bracket reaches, as a share of the ceiling. The lower
/// the top speed the more acceleration authority the caps carry, so this floor
/// bounds the search rather than the physics.
const LOWEST_TOP_SHARE: f64 = 1.0e-3;

/// A phase the certificate refuses whole is split at its certified dwell and
/// retried; this many splits without clearing it is a hard failure. Shortening
/// never changes the trajectory, only its encoding, so a phase that keeps
/// needing splits is genuinely infeasible and must not be marched at.
const MAX_CERTIFY_SPLITS: usize = 64;

/// How close a caller's entry state must be to [`entry_requirement`], relative
/// to the state's own scale, for the extremal chain to be the answer.
const REQUIREMENT_MATCH_REL_TOL: f64 = 1.0e-9;

const LENGTH_CLOSURE_REL_TOL: f64 = 1.0e-9;

struct Caps {
    v: f64,
    a: f64,
    j: f64,
}

fn reversed(kin: &Kinematics) -> Kinematics {
    Kinematics {
        length: kin.length,
        accel: kin.accel,
        jerk: kin.jerk,
        kappa0: kin.kappa0 + kin.sigma * kin.length,
        sigma: -kin.sigma,
        flat_ceiling: kin.flat_ceiling,
    }
}

fn validate(kin: &Kinematics) {
    assert!(
        kin.accel > 0.0 && kin.jerk > 0.0 && kin.jerk.is_finite() && kin.flat_ceiling > 0.0,
        "curved: degenerate kinematics accel={} jerk={} flat_ceiling={}",
        kin.accel,
        kin.jerk,
        kin.flat_ceiling
    );
    assert!(
        kin.length > 0.0 && kin.length.is_finite(),
        "curved: member length must be positive and finite, got {}",
        kin.length
    );
    assert!(
        kin.kappa0.is_finite() && kin.sigma.is_finite(),
        "curved: non-finite geometry kappa0={} sigma={}",
        kin.kappa0,
        kin.sigma
    );
}

/// Largest `|kappa|` on the member. Curvature is linear in arc length, so it is
/// an endpoint value.
fn kappa_bound(kin: &Kinematics) -> f64 {
    let kappa_exit = kin.kappa0 + kin.sigma * kin.length;
    kin.kappa0.abs().max(kappa_exit.abs())
}

/// Magnitude of the normal-plus-tangential jerk a *steady* pass demands per
/// `v^3`: the `(kappa^2, sigma)` vector the profile can never cancel.
fn steady_jerk_gain(kin: &Kinematics) -> f64 {
    let k = kappa_bound(kin);
    libm::hypot(k * k, kin.sigma.abs())
}

/// Highest speed at which the member can be held at all: flat ceiling, disk
/// limit, and jerk rail, whichever binds first.
pub(super) fn top_speed_ceiling(kin: &Kinematics) -> f64 {
    let mut v = kin.flat_ceiling;
    let k = kappa_bound(kin);
    if k > 0.0 {
        v = v.min((kin.accel / k).sqrt());
    }
    let g = steady_jerk_gain(kin);
    if g > 0.0 {
        v = v.min(libm::cbrt(STEADY_JERK_SHARE * kin.jerk / g));
    }
    v
}

/// Acceleration and jerk budgets that are feasible everywhere on the member for
/// every state at or below `v_top`. Both fall monotonically as `v_top` rises.
fn caps_at(kin: &Kinematics, v_top: f64) -> Caps {
    let k = kappa_bound(kin);
    let g = steady_jerk_gain(kin);
    let ball_slack = kin.jerk - v_top * v_top * v_top * g;
    assert!(
        ball_slack > 0.0,
        "curved: top speed {v_top} is above the jerk rail; caps_at must be called at or below \
         top_speed_ceiling"
    );
    let rail = k * v_top * v_top * (1.0 + DISK_RAIL_MARGIN);
    let disk_accel = (kin.accel * kin.accel - rail * rail).max(0.0).sqrt();
    let cross_gain = 3.0 * k * v_top;
    let a = if cross_gain > 0.0 {
        disk_accel.min(NORMAL_ACCEL_SHARE * ball_slack / cross_gain)
    } else {
        disk_accel
    };
    Caps {
        v: v_top,
        a,
        j: ball_slack - cross_gain * a,
    }
}

/// Speed change of a constant-jerk swing between zero acceleration and `a`.
fn swing_dv(a: f64, j: f64) -> f64 {
    a * a.abs() / (2.0 * j)
}

fn infeasible<T>(why: BoundaryInfeasibility) -> Result<T, VelocityError> {
    Err(VelocityError::InfeasibleBoundary(why))
}

/// Lowest top speed at which the boundary states are admissible at all: neither
/// end, nor the speed its acceleration unwinds to, may sit above the cap. The
/// swing costs are measured with the *smallest* jerk budget in the bracket, so
/// the answer is a floor for every candidate above it.
fn required_top(
    kin: &Kinematics,
    entry: (f64, f64),
    exit: Option<(f64, f64)>,
    ceiling: f64,
) -> Result<f64, VelocityError> {
    let j = caps_at(kin, ceiling).j;
    let mut states = vec![entry];
    if let Some(e) = exit {
        states.push(e);
    }
    let mut need = 0.0_f64;
    for (v, a) in states {
        if !(v.is_finite() && a.is_finite()) {
            return infeasible(BoundaryInfeasibility::NonFinite);
        }
        if v < 0.0 {
            return infeasible(BoundaryInfeasibility::UnwindBelowRest { v });
        }
        let unwound = v + swing_dv(a, j);
        let wound = v - swing_dv(a, j);
        if unwound < 0.0 || wound < 0.0 {
            return infeasible(BoundaryInfeasibility::UnwindBelowRest {
                v: unwound.min(wound),
            });
        }
        need = need.max(v).max(unwound).max(wound);
    }
    if need > ceiling {
        return infeasible(BoundaryInfeasibility::UnwindOverCeiling {
            v: need,
            v_max: ceiling,
        });
    }
    Ok(need)
}

fn bracket_floor(need: f64, ceiling: f64) -> f64 {
    need.max(ceiling * LOWEST_TOP_SHARE).min(ceiling)
}

/// Largest `v_top` in `[lo, hi]` that `admits`, given `admits(lo)` and
/// `!admits(hi)`. The predicate is monotone: every cap the search trades on
/// falls as `v_top` rises.
fn largest_admissible(lo: f64, hi: f64, admits: impl Fn(f64) -> bool) -> f64 {
    let (mut good, mut bad) = (lo, hi);
    for _ in 0..TOP_BISECT_ITERS {
        let mid = 0.5 * (good + bad);
        if mid <= good || mid >= bad {
            break;
        }
        if admits(mid) {
            good = mid;
        } else {
            bad = mid;
        }
    }
    good
}

struct March {
    phases: Vec<StraightPhase>,
    t: f64,
    s: f64,
    v: f64,
    a: f64,
}

impl March {
    fn new(entry: (f64, f64)) -> Self {
        Self {
            phases: Vec::new(),
            t: 0.0,
            s: 0.0,
            v: entry.0,
            a: entry.1,
        }
    }

    fn push(&mut self, j: f64, dt: f64) {
        assert!(dt.is_finite(), "curved: march handed a non-finite dt {dt}");
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

    fn advance_of(&self, j: f64, tau: f64) -> f64 {
        self.v * tau + 0.5 * self.a * tau * tau + j * tau * tau * tau / 6.0
    }

    /// Largest `tau <= dt_max` whose arc advance stays within `ds`. Speed is
    /// non-negative over the whole span, so the advance is monotone and `[0,
    /// dt_max]` brackets the root.
    fn dt_for_arc(&self, j: f64, ds: f64, dt_max: f64) -> f64 {
        assert!(
            dt_max.is_finite() && dt_max >= 0.0,
            "curved: arc solve needs a finite bracket, got dt_max={dt_max}"
        );
        assert!(
            self.v >= 0.0,
            "curved: arc solve from a negative speed {}",
            self.v
        );
        if ds <= 0.0 {
            return 0.0;
        }
        if self.advance_of(j, dt_max) <= ds {
            return dt_max;
        }
        let (mut lo, mut hi) = (0.0_f64, dt_max);
        for _ in 0..ARC_BISECT_ITERS {
            let mid = 0.5 * (lo + hi);
            if mid <= lo || mid >= hi {
                break;
            }
            if self.advance_of(j, mid) <= ds {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        lo
    }

    /// Emit one stage, truncated where it would overrun `length`. Returns
    /// whether the member is closed.
    fn stage(&mut self, j: f64, dt_stage: f64, length: f64) -> bool {
        let remaining = length - self.s;
        if remaining <= 0.0 {
            return true;
        }
        let dt = self.dt_for_arc(j, remaining, dt_stage.max(0.0));
        self.push(j, dt);
        length - self.s <= LENGTH_CLOSURE_REL_TOL * length
    }
}

/// Local time under `+j` after which winding the acceleration back down to zero
/// lands exactly on `v_cap`. The speed-at-zero-acceleration is monotone in the
/// local time, so the trigger is unique; on the negative-acceleration stretch
/// the speed is still falling and the trigger cannot bind, so the solve restarts
/// where the acceleration crosses zero.
fn approach_trigger(v: f64, a: f64, v_cap: f64, j: f64) -> f64 {
    if !v_cap.is_finite() {
        return f64::INFINITY;
    }
    if a < 0.0 {
        let to_zero_accel = -a / j;
        let v_at_zero_accel = v - 0.5 * a * a / j;
        return to_zero_accel + approach_trigger(v_at_zero_accel, 0.0, v_cap, j);
    }
    let overshoot = v + 0.5 * a * a / j - v_cap;
    if overshoot >= 0.0 {
        return 0.0;
    }
    ((a * a - j * overshoot).sqrt() - a) / j
}

/// Local time at constant acceleration `a` after which the down-swing lands on
/// `v_cap`.
fn hold_trigger(v: f64, a: f64, v_cap: f64, j: f64) -> f64 {
    if !v_cap.is_finite() {
        return f64::INFINITY;
    }
    if a <= 0.0 {
        return f64::INFINITY;
    }
    ((v_cap - v - 0.5 * a * a / j) / a).max(0.0)
}

/// Local time at constant acceleration needed to advance `ds`, or `None` when
/// the span cannot cover it: at rest with no acceleration, or braking to a stop
/// first. The caller then leaves the member unclosed and the closure check
/// reports it.
fn zero_jerk_arc_time(v: f64, a: f64, ds: f64) -> Option<f64> {
    if ds <= 0.0 {
        return Some(0.0);
    }
    if a == 0.0 {
        return if v > 0.0 { Some(ds / v) } else { None };
    }
    let disc = v * v + 2.0 * a * ds;
    if disc < 0.0 {
        return None;
    }
    let t = (disc.sqrt() - v) / a;
    if t.is_finite() && t >= 0.0 {
        Some(t)
    } else {
        None
    }
}

/// Fastest chain across the whole member: wind the acceleration up to the cap,
/// hold it, wind it back down so the speed lands exactly on `cap_speed`, then
/// cruise. Every stage is truncated where the member ends.
fn march_forward(caps: &Caps, entry: (f64, f64), length: f64, cap_speed: f64) -> March {
    let mut m = March::new(entry);
    let j = caps.j;

    let to_cap_accel = ((caps.a - m.a) / j).max(0.0);
    let trigger = approach_trigger(m.v, m.a, cap_speed, j);
    if m.stage(j, to_cap_accel.min(trigger), length) {
        return m;
    }

    if m.a > 0.0 {
        let Some(arc) = zero_jerk_arc_time(m.v, m.a, length - m.s) else {
            return m;
        };
        let hold = hold_trigger(m.v, m.a, cap_speed, j).min(arc).max(0.0);
        if m.stage(0.0, hold, length) {
            return m;
        }
    }

    if m.a > 0.0 && m.stage(-j, m.a / j, length) {
        return m;
    }

    let Some(cruise) = zero_jerk_arc_time(m.v, m.a, length - m.s) else {
        return m;
    };
    m.stage(0.0, cruise, length);
    m
}

fn end_state(chain: &[StraightPhase], entry: (f64, f64)) -> (f64, f64) {
    match chain.last() {
        Some(p) => {
            let (_, v, a) = p.end_state();
            (v, a)
        }
        None => entry,
    }
}

/// Re-emit `chain` as phases the certificate proves feasible over their whole
/// span, splitting at the certified dwell where it refuses one whole. The split
/// states lie on the same cubic, so the trajectory is unchanged.
pub(super) fn certified_chain(
    kin: &Kinematics,
    chain: &[StraightPhase],
) -> Result<Vec<StraightPhase>, VelocityError> {
    let mut out: Vec<StraightPhase> = Vec::with_capacity(chain.len());
    let mut t = 0.0;
    for p in chain {
        let (mut s, mut v, mut a) = (p.s0, p.v0, p.a0);
        let mut left = p.dt;
        let mut splits = 0usize;
        while left > 0.0 {
            let dwell = certify::certified_dwell(kin, s, v, a, p.j, left);
            let step = if dwell >= left * (1.0 - certify::DWELL_REL_TOL) {
                left
            } else {
                dwell
            };
            if step <= 0.0 || splits >= MAX_CERTIFY_SPLITS {
                return Err(VelocityError::UncertifiedPhase {
                    s0: s,
                    v0: v,
                    a0: a,
                    j: p.j,
                    dt: left,
                });
            }
            let piece = StraightPhase {
                t0: t,
                dt: step,
                s0: s,
                v0: v,
                a0: a,
                j: p.j,
            };
            let (next_s, next_v, next_a) = piece.end_state();
            out.push(piece);
            t += step;
            s = next_s;
            v = next_v;
            a = next_a;
            left -= step;
            splits += 1;
        }
    }
    Ok(out)
}

/// Same trajectory, traversed the other way: arc length is measured from the far
/// end, the acceleration flips sign, and the jerk does not.
fn reverse_chain(length: f64, chain: &[StraightPhase]) -> Vec<StraightPhase> {
    let mut out = Vec::with_capacity(chain.len());
    let mut t = 0.0;
    for p in chain.iter().rev() {
        let (s_end, v_end, a_end) = p.end_state();
        out.push(StraightPhase {
            t0: t,
            dt: p.dt,
            s0: length - s_end,
            v0: v_end,
            a0: -a_end,
            j: p.j,
        });
        t += p.dt;
    }
    out
}

/// Top speed whose caps let the member actually reach that speed. Above it the
/// caps are too weak to climb to their own ceiling, below it the ceiling itself
/// is what limits the exit — so the crossing maximises the reachable speed.
fn reach_chain(kin: &Kinematics, entry: (f64, f64)) -> Result<Vec<StraightPhase>, VelocityError> {
    validate(kin);
    let ceiling = top_speed_ceiling(kin);
    let need = required_top(kin, entry, None, ceiling)?;
    let lo = bracket_floor(need, ceiling);

    let saturates = |v_top: f64| {
        let caps = caps_at(kin, v_top);
        caps.a >= entry.1.abs()
            && caps.v >= need
            && end_state(
                &march_forward(&caps, entry, kin.length, f64::INFINITY).phases,
                entry,
            )
            .0 >= v_top
    };

    let top = if saturates(ceiling) {
        ceiling
    } else {
        largest_admissible(lo, ceiling, saturates)
    };
    let caps = caps_at(kin, top);
    if caps.a < entry.1.abs() {
        return infeasible(BoundaryInfeasibility::AccelOverLimit {
            a: entry.1,
            a_max: caps.a,
        });
    }
    let marched = march_forward(&caps, entry, kin.length, caps.v);
    if kin.length - marched.s > LENGTH_CLOSURE_REL_TOL * kin.length {
        return infeasible(BoundaryInfeasibility::LengthNotClosed {
            requested: kin.length,
            achieved: marched.s,
        });
    }
    certified_chain(kin, &marched.phases)
}

/// The one profile a member with no acceleration authority left can execute: a
/// single cruise phase. It is where the disk pins the speed exactly at
/// `sqrt(accel/|kappa|)` — the fitter sizes corner blends so their apex speed
/// lands there — and any demand for a speed *change* is then unsatisfiable at
/// every length.
fn cruise_only(
    kin: &Kinematics,
    entry: (f64, f64),
    exit: (f64, f64),
) -> Result<Vec<StraightPhase>, VelocityError> {
    if entry.1 != 0.0 || exit.1 != 0.0 {
        return infeasible(BoundaryInfeasibility::AccelOverLimit {
            a: if entry.1 != 0.0 { entry.1 } else { exit.1 },
            a_max: 0.0,
        });
    }
    let v = entry.0;
    if v <= 0.0 || (exit.0 - v).abs() > LENGTH_CLOSURE_REL_TOL * (1.0 + v) {
        return infeasible(BoundaryInfeasibility::LengthTooShort {
            length: kin.length,
            minimum: f64::INFINITY,
        });
    }
    Ok(vec![StraightPhase {
        t0: 0.0,
        dt: kin.length / v,
        s0: 0.0,
        v0: v,
        a0: 0.0,
        j: 0.0,
    }])
}

/// Largest top speed whose caps still close the member between the boundary
/// states. Authority falls as the cap rises, so feasibility is monotone and the
/// bracket is `[required_top, top_speed_ceiling]`.
fn bounded_plan(
    kin: &Kinematics,
    entry: (f64, f64),
    exit: (f64, f64),
) -> Result<Vec<StraightPhase>, VelocityError> {
    let ceiling = top_speed_ceiling(kin);
    let need = required_top(kin, entry, Some(exit), ceiling)?;
    let lo = bracket_floor(need, ceiling);
    if caps_at(kin, lo).a <= 0.0 {
        return cruise_only(kin, entry, exit);
    }
    let plan = |v_top: f64| {
        let caps = caps_at(kin, v_top);
        profile::straight_chain_between(entry, exit, kin.length, caps.v, caps.a, caps.j)
    };
    if let Ok(chain) = plan(ceiling) {
        return Ok(chain);
    }
    let mut best = plan(lo)?;
    let mut good = lo;
    let mut bad = ceiling;
    for _ in 0..TOP_BISECT_ITERS {
        let mid = 0.5 * (good + bad);
        if mid <= good || mid >= bad {
            break;
        }
        match plan(mid) {
            Ok(chain) => {
                best = chain;
                good = mid;
            }
            Err(_) => bad = mid,
        }
    }
    Ok(best)
}

fn states_match(lhs: (f64, f64), rhs: (f64, f64)) -> bool {
    let close = |x: f64, y: f64| (x - y).abs() <= REQUIREMENT_MATCH_REL_TOL * (1.0 + x.abs());
    close(lhs.0, rhs.0) && close(lhs.1, rhs.1)
}

/// The `(v, a)` a member needs at its entry for the fastest pass that still
/// lands on `exit`. Solved backward: the reversed member's fastest forward reach
/// is this member's tightest entry demand, and its acceleration flips sign.
pub(super) fn entry_requirement(
    kin: &Kinematics,
    exit: (f64, f64),
) -> Result<(f64, f64), VelocityError> {
    let back = reversed(kin);
    let seed = (exit.0, -exit.1);
    let chain = reach_chain(&back, seed)?;
    let (v, a) = end_state(&chain, seed);
    Ok((v, -a))
}

/// Fastest state the member can hand on at its exit, entered at `entry`.
pub(super) fn curved_reach(kin: &Kinematics, entry: (f64, f64)) -> (f64, f64) {
    let chain = reach_chain(kin, entry)
        .expect("curved_reach: the member cannot be traversed from this entry state");
    end_state(&chain, entry)
}

/// Certified constant-jerk chain across the member between boundary states that
/// both carry acceleration.
pub(super) fn curved_chain(
    kin: &Kinematics,
    entry: (f64, f64),
    exit: (f64, f64),
) -> Result<Vec<StraightPhase>, VelocityError> {
    validate(kin);
    match bounded_plan(kin, entry, exit) {
        Ok(chain) => certified_chain(kin, &chain),
        Err(bounded) => {
            let back = reversed(kin);
            let seed = (exit.0, -exit.1);
            let Ok(backward) = reach_chain(&back, seed) else {
                return Err(bounded);
            };
            let (v, a) = end_state(&backward, seed);
            if states_match(entry, (v, -a)) {
                certified_chain(kin, &reverse_chain(kin.length, &backward))
            } else {
                Err(bounded)
            }
        }
    }
}
