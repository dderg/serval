#![cfg_attr(not(test), allow(dead_code))]
//! Closed-form profile solver for a curved member (clothoid, and its arc and
//! line degeneracies).
//!
//! The acceleration disk and the jerk ball are treated as constraints to stay
//! *under*, never as surfaces to ride: the emitted chain is pure
//! [`StraightPhase`], so it carries zero representation error, and every phase
//! is *proved* inside the disk, inside the ball, and free of motion reversal by
//! [`certify::certified_span`] rather than sample-checked.
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

/// Share of the curvature term `kappa v^2` withheld from the acceleration
/// authority left inside the disk, so a chain at the top speed sits strictly
/// inside the disk instead of exactly on its edge. It scales with the curvature
/// term itself and so vanishes on a straight member.
const DISK_RAIL_MARGIN: f64 = 1e-3;

/// Overshoot of the curvature term past `accel`, relative to `accel`, that is
/// attributable to the rounding in [`top_speed_ceiling`]'s own `sqrt(accel/k)`
/// rather than to a caller asking for a speed above the ceiling.
const DISK_RIM_ROUNDING: f64 = 1.0e-12;

const TOP_BISECT_ITERS: u32 = 48;
const ARC_BISECT_ITERS: u32 = 64;

/// Bisections spent locating one end of the admissible entry-acceleration
/// interval. Sixty halvings of `[0, accel]` resolve the end to well under the
/// planner's own tolerances at any printer acceleration.
const ACCEL_BISECT_ITERS: u32 = 18;

/// Grid points spanning `[-accel, accel]` in the fallback search for an
/// admissible entry acceleration. Coarse on purpose: it only has to land inside
/// the admissible interval, which the bisections then measure exactly.
const ACCEL_SCAN_STEPS: u32 = 16;

/// Geometric splits per monotone-curvature piece. Each band's curvature ratio is
/// then at most two, and the count is fixed, so the band search cannot run away.
const BANDS_PER_PIECE: u32 = 3;

pub(super) const MAX_BANDS: usize = 2 * BANDS_PER_PIECE as usize;

/// Shortest band worth keeping, as a share of the member. Below it the band edge
/// is merged away rather than emitting a phase with no room to act.
const BAND_MIN_SHARE: f64 = 1.0e-6;

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

pub(super) struct Caps {
    pub(super) v: f64,
    pub(super) a: f64,
    pub(super) j: f64,
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

/// Acceleration authority left inside the disk at `v_top` once the curvature
/// term has been paid for, with a margin proportional to that term so a chain at
/// the top speed sits strictly inside the disk rather than on its edge. Zero is
/// the [`cruise_only`] regime, not a failure; a curvature term genuinely outside
/// the disk is a caller contract breach and fails loudly.
fn disk_authority(kin: &Kinematics, k: f64, v_top: f64) -> f64 {
    let rail = k * v_top * v_top;
    assert!(
        rail <= kin.accel * (1.0 + DISK_RIM_ROUNDING),
        "curved: top speed {v_top} puts the curvature term {rail} outside the acceleration disk \
         {}; caps_at must be called at or below top_speed_ceiling",
        kin.accel
    );
    let on_the_rim = rail.min(kin.accel);
    let inside = (kin.accel * kin.accel - on_the_rim * on_the_rim).sqrt();
    let margin = DISK_RAIL_MARGIN * rail;
    if inside <= margin {
        0.0
    } else {
        inside - margin
    }
}

/// Acceleration and jerk budgets that are feasible everywhere on the member for
/// every state at or below `v_top`. The acceleration cap falls monotonically as
/// `v_top` rises; the jerk cap does *not* — see [`bracket_jerk_floor`].
pub(super) fn caps_at(kin: &Kinematics, v_top: f64) -> Caps {
    let k = kappa_bound(kin);
    let ball_slack = ball_slack_at(kin, v_top);
    assert!(
        ball_slack > 0.0,
        "curved: top speed {v_top} is above the jerk rail; caps_at must be called at or below \
         top_speed_ceiling"
    );
    let disk_accel = disk_authority(kin, k, v_top);
    let cross_gain = 3.0 * k * v_top;
    if cross_gain <= 0.0 {
        return Caps {
            v: v_top,
            a: disk_accel,
            j: ball_slack,
        };
    }
    let cross_can_afford = NORMAL_ACCEL_SHARE * ball_slack / cross_gain;
    if disk_accel <= cross_can_afford {
        Caps {
            v: v_top,
            a: disk_accel,
            j: ball_slack - cross_gain * disk_accel,
        }
    } else {
        Caps {
            v: v_top,
            a: cross_can_afford,
            j: (1.0 - NORMAL_ACCEL_SHARE) * ball_slack,
        }
    }
}

fn ball_slack_at(kin: &Kinematics, v_top: f64) -> f64 {
    kin.jerk - v_top * v_top * v_top * steady_jerk_gain(kin)
}

/// A jerk budget no larger than `caps_at(v).j` for any `v` in `[0, ceiling]`.
///
/// `caps.j` is *not* monotone in `v_top`: on the disk-limited branch the
/// `3 kappa v a` deduction vanishes together with the disk authority, so the cap
/// rises again as the ceiling is approached. Two bounds hold branch by branch —
/// the cross term never spends more than `NORMAL_ACCEL_SHARE` of the slack, and
/// the acceleration it multiplies never exceeds `accel` — and the slack itself
/// is smallest at the ceiling, so the larger of the two evaluated there is a
/// floor for the whole bracket.
pub(super) fn bracket_jerk_floor(kin: &Kinematics, ceiling: f64) -> f64 {
    let slack = ball_slack_at(kin, ceiling);
    assert!(
        slack > 0.0,
        "curved: ceiling {ceiling} is above the jerk rail; slack {slack}"
    );
    let ball_limited = (1.0 - NORMAL_ACCEL_SHARE) * slack;
    let disk_limited = slack - 3.0 * kappa_bound(kin) * ceiling * kin.accel;
    ball_limited.max(disk_limited)
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
/// swing costs are measured with [`bracket_jerk_floor`], so the answer is a
/// floor for every candidate in the bracket rather than for one point of it.
fn required_top(
    kin: &Kinematics,
    entry: (f64, f64),
    exit: Option<(f64, f64)>,
    ceiling: f64,
) -> Result<f64, VelocityError> {
    let j = bracket_jerk_floor(kin, ceiling);
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

/// Largest `v_top` in `[lo, hi]` that `admits`. The bracket preconditions are
/// asserted rather than assumed: the caps the predicates trade on are not all
/// monotone in `v_top`, so a caller that hands over an unbracketed predicate
/// must fail loudly instead of settling on `lo`.
fn largest_admissible(lo: f64, hi: f64, admits: impl Fn(f64) -> bool) -> f64 {
    assert!(
        lo <= hi,
        "curved: bisection bracket is inverted, lo={lo} hi={hi}"
    );
    assert!(
        admits(lo),
        "curved: bisection entered with an inadmissible floor v_top={lo}"
    );
    assert!(
        !admits(hi),
        "curved: bisection entered with an admissible ceiling v_top={hi}"
    );
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
///
/// The root is taken in the form `2 ds / (v + sqrt(disc))` rather than
/// `(sqrt(disc) - v) / a`: the latter cancels away the entire answer when `a` is
/// the rounding residue a wind-down leaves behind, and reports a cruise of zero
/// length on a member that closes comfortably.
fn zero_jerk_arc_time(v: f64, a: f64, ds: f64) -> Option<f64> {
    if ds <= 0.0 {
        return Some(0.0);
    }
    let disc = v * v + 2.0 * a * ds;
    if disc < 0.0 {
        return None;
    }
    let opening_speed = v + disc.sqrt();
    if opening_speed <= 0.0 {
        return None;
    }
    let t = 2.0 * ds / opening_speed;
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

/// Re-emit `chain` as phases the certificate proves feasible — inside the disk,
/// inside the jerk ball, and never reversing — over their whole span, splitting
/// at the certified dwell where it refuses one whole. The split states lie on
/// the same cubic, so the trajectory is unchanged.
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
            let step = certify::certified_span(kin, s, v, a, p.j, left);
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

/// Arc-length seams of the bands a member is planned in.
///
/// `kappa_bound` is a whole-member `max|kappa|`, so a single set of caps holds
/// every state on the member to its curvature peak — on a blend half that is the
/// apex, and the `kappa = 0` seam at the other end pays the apex's price for
/// authority it does not owe. Curvature is linear in arc length, so `|kappa|` is
/// monotone on each side of its zero crossing; splitting there and then
/// geometrically towards each piece's low-curvature end bounds every band's
/// curvature ratio while keeping the band count fixed.
fn band_edges(kin: &Kinematics) -> Vec<f64> {
    let zero_crossing = -kin.kappa0 / kin.sigma;
    let mut edges = vec![0.0, kin.length];
    let pieces: [(f64, f64); 2] =
        if zero_crossing.is_finite() && zero_crossing > 0.0 && zero_crossing < kin.length {
            edges.push(zero_crossing);
            [(0.0, zero_crossing), (zero_crossing, kin.length)]
        } else {
            [(0.0, kin.length), (0.0, 0.0)]
        };
    for (lo, hi) in pieces {
        let span = hi - lo;
        if span <= 0.0 {
            continue;
        }
        let low_end_at_lo =
            (kin.kappa0 + kin.sigma * lo).abs() <= (kin.kappa0 + kin.sigma * hi).abs();
        for i in 1..BANDS_PER_PIECE {
            let cut = span / f64::from(1u32 << i);
            edges.push(if low_end_at_lo { lo + cut } else { hi - cut });
        }
    }
    edges.sort_by(f64::total_cmp);
    edges.dedup_by(|a, b| *a - *b <= BAND_MIN_SHARE * kin.length);
    assert!(
        edges.len() <= MAX_BANDS + 1,
        "curved: {} band edges exceeds the fixed bound {}",
        edges.len(),
        MAX_BANDS + 1
    );
    if edges.last().is_some_and(|&s| s < kin.length) {
        let last = edges.len() - 1;
        edges[last] = kin.length;
    }
    edges
}

fn band_kin(kin: &Kinematics, s0: f64, s1: f64) -> Kinematics {
    Kinematics {
        length: s1 - s0,
        accel: kin.accel,
        jerk: kin.jerk,
        kappa0: kin.kappa0 + kin.sigma * s0,
        sigma: kin.sigma,
        flat_ceiling: kin.flat_ceiling,
    }
}

fn bands(kin: &Kinematics) -> Vec<Kinematics> {
    let edges = band_edges(kin);
    edges
        .windows(2)
        .map(|w| band_kin(kin, w[0], w[1]))
        .collect()
}

/// Speed cap each band must respect so the pass never enters a later band above
/// what that band can hold: the running minimum of the ceilings from the band to
/// the member's end.
fn tail_ceilings(bands: &[Kinematics]) -> Vec<f64> {
    let mut tail = vec![f64::INFINITY; bands.len()];
    let mut running = f64::INFINITY;
    for (i, band) in bands.iter().enumerate().rev() {
        running = running.min(top_speed_ceiling(band));
        tail[i] = running;
    }
    tail
}

/// Re-base a band's chain onto the member's own arc length and clock.
fn shifted(chain: &[StraightPhase], s0: f64, t0: f64) -> impl Iterator<Item = StraightPhase> + '_ {
    chain.iter().map(move |p| StraightPhase {
        t0: p.t0 + t0,
        dt: p.dt,
        s0: p.s0 + s0,
        v0: p.v0,
        a0: p.a0,
        j: p.j,
    })
}

fn chain_end(chain: &[StraightPhase]) -> Option<(f64, f64)> {
    chain.last().map(|p| {
        let (_, v, a) = p.end_state();
        (v, a)
    })
}

fn chain_time(chain: &[StraightPhase]) -> f64 {
    chain.iter().map(|p| p.dt).sum()
}

/// Top speed whose caps let the band actually reach that speed. Above it the caps
/// are too weak to climb to their own ceiling, below it the ceiling itself is what
/// limits the exit — so the crossing maximises the reachable speed.
fn reach_span(
    kin: &Kinematics,
    entry: (f64, f64),
    ceiling_cap: f64,
) -> Result<Vec<StraightPhase>, VelocityError> {
    let ceiling = top_speed_ceiling(kin).min(ceiling_cap);
    let need = required_top(kin, entry, None, ceiling)?;
    let lo = bracket_floor(need, ceiling);

    let floor_authority = caps_at(kin, lo).a;
    if floor_authority < entry.1.abs() {
        return infeasible(BoundaryInfeasibility::AccelOverLimit {
            a: entry.1,
            a_max: floor_authority,
        });
    }

    let saturates = |v_top: f64| {
        let caps = caps_at(kin, v_top);
        caps.a >= entry.1.abs()
            && end_state(
                &march_forward(&caps, entry, kin.length, f64::INFINITY).phases,
                entry,
            )
            .0 >= v_top
    };

    let top = if saturates(ceiling) {
        ceiling
    } else if saturates(lo) {
        largest_admissible(lo, ceiling, saturates)
    } else {
        lo
    };
    let caps = caps_at(kin, top);
    assert!(
        caps.a >= entry.1.abs(),
        "curved: chosen top speed {top} lost the entry acceleration fit {} against cap {}",
        entry.1,
        caps.a
    );
    let marched = march_forward(&caps, entry, kin.length, caps.v);
    if kin.length - marched.s > LENGTH_CLOSURE_REL_TOL * kin.length {
        return infeasible(BoundaryInfeasibility::LengthNotClosed {
            requested: kin.length,
            achieved: marched.s,
        });
    }
    Ok(marched.phases)
}

/// Concatenate per-band chains onto the member's own arc length and clock.
fn splice(bands: &[Kinematics], spans: &[Vec<StraightPhase>]) -> Vec<StraightPhase> {
    let mut out = Vec::new();
    let mut s0 = 0.0;
    let mut t0 = 0.0;
    for (band, span) in bands.iter().zip(spans) {
        out.extend(shifted(span, s0, t0));
        s0 += band.length;
        t0 += chain_time(span);
    }
    out
}

/// Fastest pass across the member band by band, each band's exit state the next
/// band's entry. Every band uses the same `caps_at` and marching primitives as a
/// whole member; what banding buys is caps derived from the curvature the band
/// actually carries rather than the member's peak.
fn banded_reach(
    kin: &Kinematics,
    entry: (f64, f64),
) -> Result<(Vec<Kinematics>, Vec<Vec<StraightPhase>>), VelocityError> {
    let bands = bands(kin);
    let tail = tail_ceilings(&bands);
    let mut spans = Vec::with_capacity(bands.len());
    let mut state = entry;
    for (band, &cap) in bands.iter().zip(&tail) {
        let span = reach_span(band, state, cap)?;
        state = chain_end(&span).unwrap_or(state);
        spans.push(span);
    }
    Ok((bands, spans))
}

/// Fastest pass across the whole member. The whole-member caps are tried first
/// because they are one `caps_at` rather than a march per band; banding is what
/// answers when a single cap set is too weak to close the member at all.
fn reach_chain(kin: &Kinematics, entry: (f64, f64)) -> Result<Vec<StraightPhase>, VelocityError> {
    validate(kin);
    certified_chain(kin, &reach_spans(kin, entry)?)
}

/// The uncertified pass [`reach_chain`] certifies. Feasibility predicates use
/// this: the certificate proves what is *emitted*, and a predicate emits nothing.
fn reach_spans(kin: &Kinematics, entry: (f64, f64)) -> Result<Vec<StraightPhase>, VelocityError> {
    match reach_span(kin, entry, f64::INFINITY) {
        Ok(chain) => Ok(chain),
        Err(whole) => {
            let (bands, spans) = banded_reach(kin, entry).map_err(|_| whole)?;
            Ok(splice(&bands, &spans))
        }
    }
}

/// Band edge states of one forward pass: per band a single cap set at the speed
/// that band and everything after it can hold, marched to the band's end. It is
/// not the fastest pass — no search for the best top speed — but it is a
/// feasible one, which is all the band edges have to be.
fn band_march_edges(bands: &[Kinematics], tail: &[f64], entry: (f64, f64)) -> Vec<(f64, f64)> {
    let mut edges = Vec::with_capacity(bands.len() + 1);
    edges.push(entry);
    for (band, &tail_cap) in bands.iter().zip(tail) {
        let carried = *edges.last().expect("the entry state seeds the edge list");
        let ceiling = top_speed_ceiling(band).min(tail_cap);
        let caps = caps_at(band, ceiling);
        let held = holdable(carried, ceiling, &caps);
        let marched = march_forward(&caps, held, band.length, caps.v);
        edges.push(end_state(&marched.phases, held));
    }
    edges
}

/// The carried state as this band can actually hold it: the speed under the
/// band's ceiling, and the acceleration inside both the band's authority and the
/// swing that keeps the unwind between rest and that ceiling. Marching from a
/// state whose unwind leaves the band would drive the speed negative.
fn holdable(carried: (f64, f64), ceiling: f64, caps: &Caps) -> (f64, f64) {
    let v = carried.0.clamp(0.0, ceiling);
    let swing_to = |headroom: f64| (2.0 * caps.j * headroom.max(0.0)).sqrt();
    let a = carried
        .1
        .clamp(-swing_to(v).min(caps.a), swing_to(ceiling - v).min(caps.a));
    (v, a)
}

/// Boundary states at the member's band edges: the forward march's state at each
/// edge, or the backward march's where that is the slower of the two, so no band
/// is asked to hold a speed the band beyond it cannot brake from. The two ends
/// are the caller's own boundary states, which the bands must honour exactly.
fn band_edge_states(bands: &[Kinematics], entry: (f64, f64), exit: (f64, f64)) -> Vec<(f64, f64)> {
    let n = bands.len();
    let tail = tail_ceilings(bands);
    let ahead = band_march_edges(bands, &tail, entry);
    let back_bands: Vec<Kinematics> = bands.iter().rev().map(reversed).collect();
    let back_tail = tail_ceilings(&back_bands);
    let behind = band_march_edges(&back_bands, &back_tail, (exit.0, -exit.1));
    let mut edges = vec![entry];
    for j in 1..n {
        let (v_back, a_back) = behind[n - j];
        edges.push(if v_back < ahead[j].0 {
            (v_back, -a_back)
        } else {
            ahead[j]
        });
    }
    edges.push(exit);
    edges
}

/// Chain between two boundary states planned band by band, with the band edges
/// carrying acceleration. Each band is the same `bounded_plan` a whole member
/// gets, so nothing new is assumed about the physics — only the caps tighten.
fn banded_plan(
    kin: &Kinematics,
    entry: (f64, f64),
    exit: (f64, f64),
) -> Result<Vec<StraightPhase>, VelocityError> {
    let bands = bands(kin);
    if bands.len() < 2 {
        return infeasible(BoundaryInfeasibility::LengthTooShort {
            length: kin.length,
            minimum: kin.length,
        });
    }
    let edges = band_edge_states(&bands, entry, exit);
    let mut spans = Vec::with_capacity(bands.len());
    for (i, band) in bands.iter().enumerate() {
        spans.push(bounded_plan(band, edges[i], edges[i + 1])?);
    }
    Ok(splice(&bands, &spans))
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
    if (exit.0 - v).abs() > LENGTH_CLOSURE_REL_TOL * (1.0 + v) {
        return infeasible(BoundaryInfeasibility::SpeedChangeWithoutAuthority {
            from: v,
            to: exit.0,
        });
    }
    if v <= 0.0 {
        return infeasible(BoundaryInfeasibility::LengthNotClosed {
            requested: kin.length,
            achieved: 0.0,
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

/// Whether a planning failure means *these caps* were too weak, so a lower top
/// speed — which buys authority — is worth trying. Everything else is a broken
/// boundary state or a solver bug and belongs to the caller, not to a bisection.
fn caps_too_weak(e: &VelocityError) -> bool {
    matches!(
        e,
        VelocityError::InfeasibleBoundary(
            BoundaryInfeasibility::LengthTooShort { .. }
                | BoundaryInfeasibility::UnwindBelowRest { .. }
                | BoundaryInfeasibility::UnwindOverCeiling { .. }
                | BoundaryInfeasibility::AccelOverLimit { .. }
        )
    )
}

/// The top-speed bracket every plan between two boundary states searches: the
/// authority floor the states themselves force, and the member's own ceiling.
fn plan_bracket(
    kin: &Kinematics,
    entry: (f64, f64),
    exit: (f64, f64),
) -> Result<(f64, f64), VelocityError> {
    let ceiling = top_speed_ceiling(kin);
    let need = required_top(kin, entry, Some(exit), ceiling)?;
    Ok((bracket_floor(need, ceiling), ceiling))
}

fn plan_at(
    kin: &Kinematics,
    entry: (f64, f64),
    exit: (f64, f64),
    v_top: f64,
) -> Result<Vec<StraightPhase>, VelocityError> {
    let caps = caps_at(kin, v_top);
    profile::straight_chain_between(entry, exit, kin.length, caps.v, caps.a, caps.j)
}

/// Largest top speed whose caps still close the member between the boundary
/// states. The bracket is `[required_top, top_speed_ceiling]`; its ends are
/// established by planning at both before the bisection starts.
fn bounded_plan(
    kin: &Kinematics,
    entry: (f64, f64),
    exit: (f64, f64),
) -> Result<Vec<StraightPhase>, VelocityError> {
    let (lo, ceiling) = plan_bracket(kin, entry, exit)?;
    if caps_at(kin, lo).a <= 0.0 {
        return cruise_only(kin, entry, exit);
    }
    match plan_at(kin, entry, exit, ceiling) {
        Ok(chain) => return Ok(chain),
        Err(e) if !caps_too_weak(&e) => return Err(e),
        Err(_) => {}
    }
    let mut best = plan_at(kin, entry, exit, lo)?;
    let mut good = lo;
    let mut bad = ceiling;
    for _ in 0..TOP_BISECT_ITERS {
        let mid = 0.5 * (good + bad);
        if mid <= good || mid >= bad {
            break;
        }
        match plan_at(kin, entry, exit, mid) {
            Ok(chain) => {
                best = chain;
                good = mid;
            }
            Err(e) if !caps_too_weak(&e) => return Err(e),
            Err(_) => bad = mid,
        }
    }
    Ok(best)
}

/// Chain between two boundary states: the whole member's caps first, its bands
/// where a single cap set is too weak. Banding costs a plan per band, so it is
/// the answer to a refusal rather than the default.
fn member_plan(
    kin: &Kinematics,
    entry: (f64, f64),
    exit: (f64, f64),
) -> Result<Vec<StraightPhase>, VelocityError> {
    match bounded_plan(kin, entry, exit) {
        Ok(chain) => Ok(chain),
        Err(whole) if caps_too_weak(&whole) => banded_plan(kin, entry, exit).map_err(|_| whole),
        Err(whole) => Err(whole),
    }
}

/// Whether the whole member's own caps close it between the boundary states,
/// without building the best chain: exactly the two bracket ends
/// [`bounded_plan`] establishes before it bisects, so success here is success
/// there.
fn probe_closes(
    kin: &Kinematics,
    entry: (f64, f64),
    exit: (f64, f64),
) -> Result<(), VelocityError> {
    let (lo, ceiling) = plan_bracket(kin, entry, exit)?;
    if caps_at(kin, lo).a <= 0.0 {
        return cruise_only(kin, entry, exit).map(drop);
    }
    match plan_at(kin, entry, exit, ceiling) {
        Ok(_) => return Ok(()),
        Err(e) if !caps_too_weak(&e) => return Err(e),
        Err(_) => {}
    }
    plan_at(kin, entry, exit, lo).map(drop)
}

/// Whether [`member_plan`] would find a chain. The whole member is probed first;
/// a refusal falls through to the bands, which is what `member_plan` does next.
/// Both probes stop at the bracket ends, so the answer costs a bounded number of
/// closed-form solves however many bands the member has.
fn closes(kin: &Kinematics, entry: (f64, f64), exit: (f64, f64)) -> Result<(), VelocityError> {
    match probe_closes(kin, entry, exit) {
        Ok(()) => Ok(()),
        Err(whole) if caps_too_weak(&whole) => {
            let bands = bands(kin);
            if bands.len() < 2 {
                return Err(whole);
            }
            let edges = band_edge_states(&bands, entry, exit);
            bands
                .iter()
                .enumerate()
                .try_for_each(|(i, band)| probe_closes(band, edges[i], edges[i + 1]))
                .map_err(|_| whole)
        }
        Err(whole) => Err(whole),
    }
}

/// Whether [`member_plan`] would close the member between these boundary states.
pub(super) fn member_closes(kin: &Kinematics, entry: (f64, f64), exit: (f64, f64)) -> bool {
    validate(kin);
    closes(kin, entry, exit).is_ok()
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

/// Interval of boundary accelerations a member admits at one of its ends, the
/// other end and both speeds being fixed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct AccelWindow {
    pub(super) lo: f64,
    pub(super) hi: f64,
}

impl AccelWindow {
    pub(super) fn nearest_to(&self, a: f64) -> f64 {
        a.clamp(self.lo, self.hi)
    }

    pub(super) fn contains(&self, a: f64) -> bool {
        self.lo <= a && a <= self.hi
    }

    pub(super) fn meet(&self, other: &Self) -> Option<Self> {
        let both = Self {
            lo: self.lo.max(other.lo),
            hi: self.hi.min(other.hi),
        };
        (both.lo <= both.hi).then_some(both)
    }
}

/// Entry acceleration of the one-constant-jerk pass from `(v0, a0)` to `exit`
/// over the whole member: `a0` is pinned by the three closure conditions
/// (speed, acceleration, arc length), which reduce to a quadratic in
/// `S = a0 + a_exit`. Both roots are offered; a swing that reverses the speed
/// or leaves the member unclosed is rejected by the admissibility test itself,
/// so the roots are seeds rather than answers.
fn single_jerk_entry_accels(v0: f64, exit: (f64, f64), length: f64) -> [Option<f64>; 2] {
    let dv = exit.0 - v0;
    if dv == 0.0 {
        return [None, None];
    }
    let b = -(6.0 * v0 * dv + 4.0 * dv * dv);
    let c = 2.0 * dv * dv * exit.1;
    let a = 3.0 * length;
    let disc = b * b - 4.0 * a * c;
    if disc < 0.0 {
        return [None, None];
    }
    let root = disc.sqrt();
    let solve = |s: f64| (s != 0.0).then_some(s - exit.1);
    [solve(0.5 * (-b + root) / a), solve(0.5 * (-b - root) / a)]
}

/// Entry acceleration of the pass that holds one acceleration the whole way.
fn zero_jerk_entry_accel(v0: f64, exit: (f64, f64), length: f64) -> f64 {
    (exit.0 * exit.0 - v0 * v0) / (2.0 * length)
}

/// Extreme admissible value on the ray from `seed` towards `limit`. The
/// admissible set is an intersection of intervals — the disk authority, the
/// unwind band, and the length requirement — so the ray crosses its boundary
/// once and a bisection converges on that crossing.
fn stretch_to(seed: f64, limit: f64, admits: impl Fn(f64) -> bool) -> f64 {
    if admits(limit) {
        return limit;
    }
    let (mut good, mut bad) = (seed, limit);
    for _ in 0..ACCEL_BISECT_ITERS {
        let mid = 0.5 * (good + bad);
        if mid == good || mid == bad {
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

/// Any admissible acceleration on an even grid across the whole rail, for the
/// case where every shaped candidate lands outside a narrow admissible band. The
/// band is an interval, so one grid point inside it is all the stretch needs.
fn scan_for_admissible(rail: f64, admits: &impl Fn(f64) -> bool) -> Option<f64> {
    (0..=ACCEL_SCAN_STEPS)
        .map(|i| rail * (2.0 * f64::from(i) / f64::from(ACCEL_SCAN_STEPS) - 1.0))
        .find(|&a| admits(a))
}

/// Widest interval of boundary accelerations the member admits at one end, given
/// shaped candidates to seed the search from and a forward closure test.
fn accel_window_around(
    rail: f64,
    shaped: impl IntoIterator<Item = f64>,
    admits: impl Fn(f64) -> bool,
) -> Option<AccelWindow> {
    let seed = shaped
        .into_iter()
        .filter(|a| a.is_finite())
        .map(|a| a.clamp(-rail, rail))
        .find(|&a| admits(a))
        .or_else(|| scan_for_admissible(rail, &admits))?;
    Some(AccelWindow {
        lo: stretch_to(seed, -rail, &admits),
        hi: stretch_to(seed, rail, &admits),
    })
}

/// Widest interval of entry accelerations from which the member, entered at
/// `v_entry`, can still be traversed to `exit`.
///
/// This is the query a backward pass needs but [`entry_requirement`] cannot
/// answer: the requirement names one *fastest* `(v, a)`, and where the envelope
/// holds the seam below that speed the acceleration it names is not the one the
/// slower pass needs. Handing the predecessor a zero there asks it to build the
/// member's whole interior brake inside its own length.
pub(super) fn entry_accel_window(
    kin: &Kinematics,
    v_entry: f64,
    exit: (f64, f64),
) -> Result<AccelWindow, VelocityError> {
    validate(kin);
    if !(v_entry.is_finite() && v_entry >= 0.0 && exit.0.is_finite() && exit.1.is_finite()) {
        return infeasible(BoundaryInfeasibility::NonFinite);
    }
    let admits = |a: f64| closes(kin, (v_entry, a), exit).is_ok();
    let [jerk_root, other_root] = single_jerk_entry_accels(v_entry, exit, kin.length);
    let shaped = [
        Some(zero_jerk_entry_accel(v_entry, exit, kin.length)),
        jerk_root,
        other_root,
        Some(exit.1),
        Some(0.0),
        Some(kin.accel),
        Some(-kin.accel),
    ];
    accel_window_around(kin.accel, shaped.into_iter().flatten(), admits).ok_or_else(|| {
        closes(kin, (v_entry, 0.0), exit)
            .expect_err("no admissible entry acceleration, yet zero closed the member")
    })
}

/// Entry accelerations the member can be traversed from at all when it is
/// entered at `v_entry`, its exit state left free.
///
/// Two bounds, in order. The caps and the unwind band give an interval centred on
/// zero: every one of those is monotone in `|a|` — the unwind swing widens with
/// it, and the acceleration authority the caps leave shrinks as that swing lifts
/// the top speed. Inside that, the member still has to *close its own length*,
/// and that is not symmetric in `a`: a brake steep enough to stop the toolhead
/// before the far end leaves the member unclosed however much authority the disk
/// has left. `None` means the member cannot hold `v_entry` at all, which is a
/// speed the envelope must give up rather than an acceleration it can choose.
pub(super) fn traversable_entry_accels(kin: &Kinematics, v_entry: f64) -> Option<AccelWindow> {
    validate(kin);
    let ceiling = top_speed_ceiling(kin);
    if !(v_entry.is_finite() && v_entry >= 0.0) || v_entry > ceiling {
        return None;
    }
    let j = bracket_jerk_floor(kin, ceiling);
    let holds = |mag: f64| {
        let swing = swing_dv(mag, j);
        v_entry + swing <= ceiling
            && v_entry - swing >= 0.0
            && caps_at(kin, bracket_floor(v_entry + swing, ceiling)).a >= mag
    };
    let rail = stretch_to(0.0, kin.accel, holds);
    let traversable = |a: f64| reach_spans(kin, (v_entry, a)).is_ok();
    accel_window_around(rail, [0.0, rail, -rail], traversable)
}

/// Widest interval of exit accelerations the member can hand on at speed
/// `v_exit`, entered at the state its predecessor actually leaves it in. This is
/// what a seam's acceleration is *deliverable* from; the successor's
/// [`entry_accel_window`] is what it is *needed* for, and a seam state has to sit
/// in both.
///
/// The test is the forward closure the emission itself will run, not the reversed
/// member's: at the edge of the interval the two agree only to within rounding,
/// and the edge is exactly where a clamped seam state lands.
pub(super) fn exit_accel_window(
    kin: &Kinematics,
    entry: (f64, f64),
    v_exit: f64,
) -> Result<AccelWindow, VelocityError> {
    validate(kin);
    if !(v_exit.is_finite() && v_exit >= 0.0 && entry.0.is_finite() && entry.1.is_finite()) {
        return infeasible(BoundaryInfeasibility::NonFinite);
    }
    let admits = |a: f64| closes(kin, entry, (v_exit, a)).is_ok();
    let [jerk_root, other_root] = single_jerk_entry_accels(v_exit, (entry.0, -entry.1), kin.length);
    let shaped = [
        Some(-zero_jerk_entry_accel(
            v_exit,
            (entry.0, -entry.1),
            kin.length,
        )),
        jerk_root.map(|a| -a),
        other_root.map(|a| -a),
        Some(entry.1),
        Some(0.0),
        Some(kin.accel),
        Some(-kin.accel),
    ];
    accel_window_around(kin.accel, shaped.into_iter().flatten(), admits).ok_or_else(|| {
        closes(kin, entry, (v_exit, 0.0))
            .expect_err("no admissible exit acceleration, yet zero closed the member")
    })
}

/// Fastest state the member can hand on at its exit, entered at `entry`, or
/// `None` when it cannot be traversed from that entry state at all.
pub(super) fn reachable_exit(kin: &Kinematics, entry: (f64, f64)) -> Option<(f64, f64)> {
    reach_chain(kin, entry)
        .ok()
        .map(|chain| end_state(&chain, entry))
}

/// Fastest speed the member may be entered at and still be brought down to
/// `v_exit`: the reversed member's own forward reach. This is the brake envelope
/// a curved member imposes and the disk sweep cannot see — the disk bounds
/// `kappa^2 v^4` against the acceleration budget and says nothing about the
/// `sigma v^3` of normal jerk a clothoid owes with no acceleration term to
/// cancel it, nor about the acceleration and jerk the curvature has already
/// spent.
pub(super) fn brake_reach(kin: &Kinematics, v_exit: f64) -> Option<f64> {
    reachable_exit(&reversed(kin), (v_exit, 0.0)).map(|(v, _)| v)
}

/// Fastest state the member can hand on at its exit, entered at `entry`.
pub(super) fn curved_reach(kin: &Kinematics, entry: (f64, f64)) -> (f64, f64) {
    reachable_exit(kin, entry)
        .expect("curved_reach: the member cannot be traversed from this entry state")
}

/// Certified constant-jerk chain across the member between boundary states that
/// both carry acceleration.
pub(super) fn curved_chain(
    kin: &Kinematics,
    entry: (f64, f64),
    exit: (f64, f64),
) -> Result<Vec<StraightPhase>, VelocityError> {
    validate(kin);
    match member_plan(kin, entry, exit) {
        Ok(chain) => certified_chain(kin, &chain),
        Err(bounded) => {
            let back = reversed(kin);
            let seed = (exit.0, -exit.1);
            match reach_chain(&back, seed) {
                Ok(backward) => {
                    let (v, a) = end_state(&backward, seed);
                    if states_match(entry, (v, -a)) {
                        certified_chain(kin, &reverse_chain(kin.length, &backward))
                    } else {
                        Err(bounded)
                    }
                }
                Err(backward) if !caps_too_weak(&backward) => Err(backward),
                Err(_) => Err(bounded),
            }
        }
    }
}
