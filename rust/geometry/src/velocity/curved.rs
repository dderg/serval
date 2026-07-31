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

/// Share of the jerk budget left after the curvature's own `v^3` demand that is
/// spent on the `3 kappa v a` cross term rather than on tangential jerk.
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

/// Overshoot of the curvature's own `v^3` jerk demand past the whole budget,
/// relative to it, attributable to the rounding in [`top_speed_ceiling`]'s own
/// `cbrt(jerk / g)` rather than to a caller asking for a speed above the rail.
const BALL_RAIL_ROUNDING: f64 = 1.0e-12;

const TOP_BISECT_ITERS: u32 = 48;
const ARC_BISECT_ITERS: u32 = 64;

/// Cap speeds the plan search samples across its bracket before closing in, and
/// halvings it then spends around the best sample. Coarse first because the
/// quickest cap speed is interior, so a bisection towards one end would walk
/// straight past it.
const PLAN_PROBE_STEPS: u32 = 16;
const PLAN_REFINE_ITERS: u32 = 12;

/// Rounds of the fixed point that sizes the jerk reserve against the swings the
/// boundary states actually need. The demand only climbs, and the ceiling bounds
/// it, so the iteration settles or leaves the member.
const SWING_FIXPOINT_ITERS: u32 = 32;

/// Bisections spent locating one end of the admissible entry-acceleration
/// interval. Sixty halvings of `[0, accel]` resolve the end to well under the
/// planner's own tolerances at any printer acceleration.
const ACCEL_BISECT_ITERS: u32 = 18;

/// Grid points spanning `[-accel, accel]` in the fallback search for an
/// admissible entry acceleration. Coarse on purpose: it only has to land inside
/// the admissible interval, which the bisections then measure exactly.
const ACCEL_SCAN_STEPS: u32 = 16;

/// Bands per monotone-curvature piece, split geometrically towards the piece's
/// low-curvature end. Two is what pays: it separates the seam's cheap curvature
/// from the apex's expensive curvature, and every further split buys a fraction
/// of that while costing another seam the lowering has to carry as its own
/// piece. The count is fixed, so the band search cannot run away.
const BANDS_PER_PIECE: u32 = 2;

pub(super) const MAX_BANDS: usize = 2 * BANDS_PER_PIECE as usize;

/// Shortest band worth keeping, as a share of the member. Below it the band edge
/// is merged away rather than emitting a phase with no room to act.
const BAND_MIN_SHARE: f64 = 1.0e-6;

/// Spread between the highest and lowest band ceiling, relative to the lowest,
/// above which planning band by band buys speed the member's single
/// peak-curvature cap set cannot reach. Below it every band is held to what is
/// effectively one ceiling, and the single cap set is the same answer for a
/// fraction of the work.
const BAND_CEILING_SPREAD: f64 = 1.0e-2;

/// Lowest top speed the bracket reaches, as a share of the ceiling. The lower
/// the top speed the more acceleration authority the caps carry, so this floor
/// bounds the search rather than the physics.
const LOWEST_TOP_SHARE: f64 = 1.0e-3;

/// Rungs the plan ladder climbs, spaced evenly in `v^2` so every rung takes an
/// equal share of the curvature term `kappa v^2` the disk charges for. One cap
/// set per member prices the whole ramp at the top speed's authority; a rung
/// prices its own stretch at its own speed's.
pub(super) const LADDER_RUNGS: usize = 8;

/// How close a rung climb must land on the top speed it was aimed at, relative
/// to that speed and to the authority left there, for the arrival to count.
const LADDER_ARRIVAL_REL_TOL: f64 = 1.0e-9;

/// Shortfall of the authority at the top speed against the authority at the
/// lowest rung, relative to the latter, below which the rungs are all holding
/// effectively one cap set and the member's own is the same answer in far fewer
/// phases. A straight member's caps do not vary with speed at all.
const LADDER_AUTHORITY_SPREAD: f64 = 1.0e-2;

/// Time the ladder must save against the member's own single cap set at the
/// same top speed, relative, to be worth the phases it costs the lowering.
const LADDER_TIME_WIN: f64 = 1.0e-2;

/// Overshoot of a rung's authority, relative to it, attributable to the
/// rounding in the wind-down that lands the rung below exactly on that
/// authority rather than to a state the rung genuinely cannot hold.
const RUNG_HANDOFF_ROUNDING: f64 = 1.0e-9;

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
/// limit, and the jerk rail where a steady pass — zero commanded jerk, zero
/// acceleration — spends the whole budget on the curvature's own `v^3` demand,
/// whichever binds first. At the rail nothing is left to swing `a` with, which
/// is the [`cruise_only`] regime rather than a failure.
pub(super) fn top_speed_ceiling(kin: &Kinematics) -> f64 {
    let mut v = kin.flat_ceiling;
    let k = kappa_bound(kin);
    if k > 0.0 {
        v = v.min((kin.accel / k).sqrt());
    }
    let g = steady_jerk_gain(kin);
    if g > 0.0 {
        v = v.min(libm::cbrt(kin.jerk / g));
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
/// `v_top` rises; the jerk cap does *not*. At the member's own ceiling the
/// curvature has taken the whole budget and both caps are zero: the reserve the
/// profile keeps back for swinging `a` is exactly what its swings cost, which on
/// a cruise is nothing.
pub(super) fn caps_at(kin: &Kinematics, v_top: f64) -> Caps {
    let k = kappa_bound(kin);
    let ball_slack = ball_slack_at(kin, v_top);
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

/// Jerk budget left once the curvature's own unavoidable `v^3` demand at `v_top`
/// is paid. Zero at the member's ceiling by construction; a genuinely negative
/// slack is a caller asking for a speed above the rail and fails loudly.
fn ball_slack_at(kin: &Kinematics, v_top: f64) -> f64 {
    let slack = kin.jerk - v_top * v_top * v_top * steady_jerk_gain(kin);
    assert!(
        slack >= -BALL_RAIL_ROUNDING * kin.jerk,
        "curved: top speed {v_top} is above the jerk rail by {}; caps_at must be called at or \
         below top_speed_ceiling",
        -slack
    );
    slack.max(0.0)
}

/// Speed the state `(v, a)` reaches once a constant-jerk swing has wound its
/// acceleration back to zero: a drive gains `a^2 / 2j`, a brake loses it, and a
/// profile that swings nothing needs no jerk to do it however little is left.
/// The direction is the whole point — pricing a brake as though it could also
/// swing the other way refuses states no march would ever visit.
fn unwound_speed(v: f64, a: f64, j: f64) -> f64 {
    if a == 0.0 {
        return v;
    }
    v + a * a.abs() / (2.0 * j)
}

fn infeasible<T>(why: BoundaryInfeasibility) -> Result<T, VelocityError> {
    Err(VelocityError::InfeasibleBoundary(why))
}

/// Lowest top speed at which the boundary states are admissible at all: neither
/// end, nor the speed its acceleration unwinds to, may sit above the cap. The
/// swing costs are measured with the jerk the caps leave at the answer itself,
/// so the budget held back is what these swings need rather than a fixed share
/// of it. The answer is the bracket floor rather than the bare demand, so the
/// caps the caller derives from it are the very caps the swings were measured
/// against; raising the top speed only tightens them, so the demand climbs
/// monotonically and either settles or leaves the member.
fn required_top(
    kin: &Kinematics,
    entry: (f64, f64),
    exit: Option<(f64, f64)>,
    ceiling: f64,
) -> Result<f64, VelocityError> {
    let mut states = vec![entry];
    if let Some(e) = exit {
        states.push(e);
    }
    let mut plain = 0.0_f64;
    for &(v, a) in &states {
        if !(v.is_finite() && a.is_finite()) {
            return infeasible(BoundaryInfeasibility::NonFinite);
        }
        if v < 0.0 {
            return infeasible(BoundaryInfeasibility::UnwindBelowRest { v });
        }
        plain = plain.max(v);
    }
    let over_ceiling =
        |v: f64| infeasible(BoundaryInfeasibility::UnwindOverCeiling { v, v_max: ceiling });
    if plain > ceiling {
        return over_ceiling(plain);
    }
    let mut need = plain;
    for _ in 0..SWING_FIXPOINT_ITERS {
        let floor = bracket_floor(need, ceiling);
        let j = caps_at(kin, floor).j;
        let mut grown = plain;
        for &(v, a) in &states {
            let unwound = unwound_speed(v, a, j);
            if unwound > ceiling {
                return over_ceiling(unwound);
            }
            if unwound < 0.0 {
                return infeasible(BoundaryInfeasibility::UnwindBelowRest { v: unwound });
            }
            grown = grown.max(unwound);
        }
        if grown <= floor {
            return Ok(floor);
        }
        need = grown;
    }
    over_ceiling(need)
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

/// Local time under `-j` after which winding the acceleration back up to zero
/// lands exactly on `v_cap`, entered above it. Mirror of [`approach_trigger`]:
/// on the positive-acceleration stretch the speed is still rising and the
/// trigger cannot bind, so the solve restarts where the acceleration crosses
/// zero.
fn brake_trigger(v: f64, a: f64, v_cap: f64, j: f64) -> f64 {
    if !v_cap.is_finite() {
        return f64::INFINITY;
    }
    if a > 0.0 {
        let to_zero_accel = a / j;
        let v_at_zero_accel = v + 0.5 * a * a / j;
        return to_zero_accel + brake_trigger(v_at_zero_accel, 0.0, v_cap, j);
    }
    let undershoot = v_cap - v + 0.5 * a * a / j;
    if undershoot >= 0.0 {
        return 0.0;
    }
    ((a * a - j * undershoot).sqrt() + a) / j
}

/// Local time at constant braking `a` after which the up-swing lands on
/// `v_cap`.
fn brake_hold_trigger(v: f64, a: f64, v_cap: f64, j: f64) -> f64 {
    if !v_cap.is_finite() {
        return f64::INFINITY;
    }
    if a >= 0.0 {
        return f64::INFINITY;
    }
    ((v_cap - v + 0.5 * a * a / j) / a).max(0.0)
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

/// Fastest chain across the whole member: swing the acceleration to the cap,
/// hold it, swing it back so the speed lands exactly on `cap_speed`, then
/// cruise. Entered above `cap_speed` the swings brake instead of drive, which is
/// how a band whose neighbours hold less is arrived at slowing down. Every stage
/// is truncated where the member ends.
fn march_forward(caps: &Caps, entry: (f64, f64), length: f64, cap_speed: f64) -> March {
    let mut m = March::new(entry);
    if caps.j <= 0.0 {
        assert!(
            m.a == 0.0,
            "curved: top speed {} leaves no jerk authority yet the march carries a0={}",
            caps.v,
            m.a
        );
        if let Some(cruise) = zero_jerk_arc_time(m.v, 0.0, length) {
            m.stage(0.0, cruise, length);
        }
        return m;
    }
    if entry.0 > cap_speed {
        return march_braking(caps, entry, length, cap_speed);
    }
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

fn march_braking(caps: &Caps, entry: (f64, f64), length: f64, cap_speed: f64) -> March {
    let mut m = March::new(entry);
    let j = caps.j;

    let to_cap_accel = ((caps.a + m.a) / j).max(0.0);
    let trigger = brake_trigger(m.v, m.a, cap_speed, j);
    if m.stage(-j, to_cap_accel.min(trigger), length) {
        return m;
    }

    if m.a < 0.0 {
        let Some(arc) = zero_jerk_arc_time(m.v, m.a, length - m.s) else {
            return m;
        };
        let hold = brake_hold_trigger(m.v, m.a, cap_speed, j).min(arc).max(0.0);
        if m.stage(0.0, hold, length) {
            return m;
        }
    }

    if m.a < 0.0 && m.stage(j, -m.a / j, length) {
        return m;
    }

    let Some(cruise) = zero_jerk_arc_time(m.v, m.a, length - m.s) else {
        return m;
    };
    m.stage(0.0, cruise, length);
    m
}

/// One rung of the ladder: swing the acceleration up under this rung's caps,
/// hold it, and swing it back so the speed lands on the rung carrying exactly
/// `handoff` — the authority the next rung has left. It is [`march_forward`]
/// without the trailing cruise: the arc length past the rung belongs to the
/// next rung, whose weaker caps are what the higher speed owes.
///
/// A wind-down to `handoff` rather than to rest is the whole point. Landing at
/// rest on every rung would spend a full acceleration swing per rung and reach
/// less than the single cap set it replaces; carrying the acceleration makes
/// the rungs a staircase down the disk rim instead of eight separate ramps.
fn climb_rung(caps: &Caps, entry: (f64, f64), max_len: f64, rung: f64, handoff: f64) -> March {
    let mut m = March::new(entry);
    let j = caps.j;
    let rung_at_handoff = rung + handoff * handoff / (2.0 * j);

    let to_cap_accel = ((caps.a - m.a) / j).max(0.0);
    let trigger = approach_trigger(m.v, m.a, rung_at_handoff, j);
    if m.stage(j, to_cap_accel.min(trigger), max_len) {
        return m;
    }

    if m.a > 0.0 {
        let Some(arc) = zero_jerk_arc_time(m.v, m.a, max_len - m.s) else {
            return m;
        };
        let hold = hold_trigger(m.v, m.a, rung_at_handoff, j).min(arc).max(0.0);
        if m.stage(0.0, hold, max_len) {
            return m;
        }
    }

    if m.a > handoff {
        m.stage(-j, (m.a - handoff) / j, max_len);
    }
    m
}

fn rung_speeds(v_top: f64) -> [f64; LADDER_RUNGS] {
    std::array::from_fn(|i| v_top * ((i + 1) as f64 / LADDER_RUNGS as f64).sqrt())
}

/// Acceleration a rung hands its successor: the authority the successor has,
/// and no more than the swing the remaining climb to `v_top` can still wind off
/// against the least jerk any rung above leaves. Winding off more than the
/// climb has speed left for would carry the pass past `v_top` still
/// accelerating, which is the one thing the cruise it hands over to cannot take.
fn handoff_accel(kin: &Kinematics, rungs: &[f64; LADDER_RUNGS], at: usize, v_top: f64) -> f64 {
    let successor = rungs.get(at + 1).map_or(0.0, |&up| caps_at(kin, up).a);
    let unwind_jerk = rungs[at..]
        .iter()
        .map(|&up| caps_at(kin, up).j)
        .fold(f64::INFINITY, f64::min);
    let climb_left = (v_top - rungs[at]).max(0.0);
    successor
        .min(caps_at(kin, rungs[at]).a)
        .min((2.0 * unwind_jerk * climb_left).sqrt())
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

/// Whether a boundary state sits inside a cap set: its acceleration within the
/// authority, its own speed under the top speed, and the speed its unwind lands
/// on between rest and that top speed.
fn state_fits(caps: &Caps, (v, a): (f64, f64)) -> bool {
    let unwound = unwound_speed(v, a, caps.j);
    a.abs() <= caps.a && v <= caps.v && unwound <= caps.v && unwound >= 0.0
}

/// Whether a rung can carry the state it is handed: the speed inside the rung,
/// the acceleration inside the authority the rung leaves. The unwind band
/// [`state_fits`] also demands is the ladder's business, not a rung's — a rung
/// hands its acceleration up to the rung above rather than winding it off
/// inside its own speed stretch.
fn rung_holds(caps: &Caps, (v, a): (f64, f64)) -> bool {
    a.abs() <= caps.a * (1.0 + RUNG_HANDOFF_ROUNDING) && v <= caps.v
}

/// Top speed whose caps let the band actually reach that speed. Above it the caps
/// are too weak to climb to their own ceiling, below it the ceiling itself is what
/// limits the exit — so the crossing maximises the reachable speed.
///
/// `hold_cap` is what the *rest* of the member can hold, so it targets the
/// cruise the march settles onto; the caps themselves are still derived at this
/// band's own ceiling, because that is the fastest state the band actually
/// visits when it is entered above what its neighbours can carry and has to
/// brake into them.
fn reach_span(
    kin: &Kinematics,
    entry: (f64, f64),
    hold_cap: f64,
) -> Result<Vec<StraightPhase>, VelocityError> {
    let ceiling = top_speed_ceiling(kin);
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
        state_fits(&caps, entry)
            && end_state(
                &march_forward(&caps, entry, kin.length, f64::INFINITY).phases,
                entry,
            )
            .0 >= v_top
    };

    let brakes_the_whole_way = entry.0 > hold_cap;
    let top = if brakes_the_whole_way || !saturates(lo) {
        lo
    } else if saturates(ceiling) {
        ceiling
    } else {
        largest_admissible(lo, ceiling, saturates)
    };
    let caps = caps_at(kin, top);
    assert!(
        state_fits(&caps, entry),
        "curved: chosen top speed {top} lost the entry state fit {entry:?} against caps \
         a={} j={}",
        caps.a,
        caps.j
    );
    let marched = march_forward(&caps, entry, kin.length, caps.v.min(hold_cap));
    if kin.length - marched.s > LENGTH_CLOSURE_REL_TOL * kin.length {
        return infeasible(BoundaryInfeasibility::LengthNotClosed {
            requested: kin.length,
            achieved: marched.s,
        });
    }
    Ok(marched.phases)
}

/// Adjacent phases carrying the same jerk are one phase: the state is continuous
/// across a band seam, so keeping both only encodes the same cubic twice.
fn coalesce(chain: Vec<StraightPhase>) -> Vec<StraightPhase> {
    let mut out: Vec<StraightPhase> = Vec::with_capacity(chain.len());
    for p in chain {
        match out.last_mut() {
            Some(prev) if prev.j == p.j => prev.dt += p.dt,
            _ => out.push(p),
        }
    }
    out
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
    coalesce(out)
}

/// Fastest pass across the member band by band, each band's exit state the next
/// band's entry. Every band uses the same `caps_at` and marching primitives as a
/// whole member; what banding buys is caps derived from the curvature the band
/// actually carries rather than the member's peak.
fn banded_reach(
    bands: &[Kinematics],
    entry: (f64, f64),
) -> Result<Vec<Vec<StraightPhase>>, VelocityError> {
    let tail = tail_ceilings(bands);
    let mut spans = Vec::with_capacity(bands.len());
    let mut state = entry;
    for (band, &cap) in bands.iter().zip(&tail) {
        let span = reach_span(band, state, cap)?;
        state = chain_end(&span).unwrap_or(state);
        spans.push(span);
    }
    Ok(spans)
}

/// Whether the bands' ceilings spread far enough apart for the tighter per-band
/// caps to buy speed the member's single peak-curvature cap set cannot. A blend
/// half spans a `kappa = 0` seam and a curvature apex, so it always pays; a
/// member whose bands all hold the same speed never does.
fn banding_pays(bands: &[Kinematics]) -> bool {
    if bands.len() < 2 {
        return false;
    }
    let (lowest, highest) = bands
        .iter()
        .map(top_speed_ceiling)
        .fold((f64::INFINITY, 0.0_f64), |(lowest, highest), ceiling| {
            (lowest.min(ceiling), highest.max(ceiling))
        });
    highest > lowest * (1.0 + BAND_CEILING_SPREAD)
}

fn reach_chain(kin: &Kinematics, entry: (f64, f64)) -> Result<Vec<StraightPhase>, VelocityError> {
    validate(kin);
    certified_chain(kin, &reach_spans(kin, entry)?)
}

/// The uncertified pass [`reach_chain`] certifies. Feasibility predicates use
/// this: the certificate proves what is *emitted*, and a predicate emits nothing.
fn reach_spans(kin: &Kinematics, entry: (f64, f64)) -> Result<Vec<StraightPhase>, VelocityError> {
    let bands = bands(kin);
    if banding_pays(&bands) {
        if let Ok(spans) = banded_reach(&bands, entry) {
            return Ok(splice(&bands, &spans));
        }
    }
    match reach_span(kin, entry, f64::INFINITY) {
        Ok(chain) => Ok(chain),
        Err(whole) => banded_reach(&bands, entry)
            .map(|spans| splice(&bands, &spans))
            .map_err(|_| whole),
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
/// is asked to hold a speed the band beyond it cannot brake from. Each march is
/// held to its own band's ceiling alone — the opposing march is what keeps a
/// band from carrying a speed its neighbours cannot, and capping the forward
/// march with the tail minimum as well would flatten the traversal onto the
/// slowest band's ceiling and erase the dip. The two ends are the caller's own
/// boundary states, which the bands must honour exactly.
fn band_edge_states(bands: &[Kinematics], entry: (f64, f64), exit: (f64, f64)) -> Vec<(f64, f64)> {
    let n = bands.len();
    let own_ceiling_only = vec![f64::INFINITY; n];
    let ahead = band_march_edges(bands, &own_ceiling_only, entry);
    let back_bands: Vec<Kinematics> = bands.iter().rev().map(reversed).collect();
    let behind = band_march_edges(&back_bands, &own_ceiling_only, (exit.0, -exit.1));
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
    bands: &[Kinematics],
    entry: (f64, f64),
    exit: (f64, f64),
) -> Result<Vec<StraightPhase>, VelocityError> {
    let edges = band_edge_states(bands, entry, exit);
    let mut spans = Vec::with_capacity(bands.len());
    for (i, band) in bands.iter().enumerate() {
        spans.push(bounded_plan(band, edges[i], edges[i + 1])?);
    }
    Ok(splice(bands, &spans))
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

/// A climb up the ladder: the phases, the arc length they spend, and their
/// duration.
struct Climb {
    phases: Vec<StraightPhase>,
    length: f64,
    time: f64,
}

/// Climb from `entry` to `v_top` rung by rung, each stretch planned with the
/// caps its own speed carries rather than the caps the top speed carries.
///
/// `None` is not a failure to report: it says this top speed is out of the
/// member's reach from this state within `max_len`, which is exactly what the
/// search over top speeds is there to discover.
fn ladder_climb(kin: &Kinematics, entry: (f64, f64), v_top: f64, max_len: f64) -> Option<Climb> {
    let top_caps = caps_at(kin, v_top);
    if top_caps.a <= 0.0 || top_caps.j <= 0.0 || max_len <= 0.0 {
        return None;
    }
    let unwound_entry = unwound_speed(entry.0, entry.1, top_caps.j);
    if unwound_entry < 0.0 || unwound_entry > v_top {
        return None;
    }
    let rungs = rung_speeds(v_top);
    let lowest_holding = rungs
        .iter()
        .position(|&rung| rung_holds(&caps_at(kin, rung), entry))?;

    let mut climb = Climb {
        phases: Vec::new(),
        length: 0.0,
        time: 0.0,
    };
    let mut state = entry;
    for (at, &rung) in rungs.iter().enumerate().skip(lowest_holding) {
        if state.0 >= rung {
            continue;
        }
        let caps = caps_at(kin, rung);
        if !rung_holds(&caps, state) {
            return None;
        }
        let handoff = handoff_accel(kin, &rungs, at, v_top);
        let marched = climb_rung(&caps, state, max_len - climb.length, rung, handoff);
        climb
            .phases
            .extend(shifted(&marched.phases, climb.length, climb.time));
        climb.length += marched.s;
        climb.time += marched.t;
        state = (marched.v, marched.a);
    }

    let arrived = (v_top - state.0).abs() <= LADDER_ARRIVAL_REL_TOL * v_top
        && state.1.abs() <= LADDER_ARRIVAL_REL_TOL * top_caps.a;
    arrived.then_some(climb)
}

/// Whether the authority the caps leave varies enough across the climb for the
/// rungs to buy anything the member's single cap set cannot. On a straight
/// member it does not vary at all, and the ladder is the same profile spread
/// over eight times the phases.
fn ladder_pays(kin: &Kinematics, v_top: f64) -> bool {
    let lowest = caps_at(kin, rung_speeds(v_top)[0]).a;
    caps_at(kin, v_top).a < lowest * (1.0 - LADDER_AUTHORITY_SPREAD)
}

/// Two ladders meeting at a cruise: the entry climbs to `v_top`, the exit
/// climbs to it on the reversed member, and the arc length neither spent is one
/// cruise phase. Every speed stretch is planned with `caps_at` its own speed, so
/// the ramp is charged the authority it has where it is rather than the
/// authority left at the ceiling.
fn ladder_plan(
    kin: &Kinematics,
    entry: (f64, f64),
    exit: (f64, f64),
    v_top: f64,
) -> Option<Vec<StraightPhase>> {
    if !ladder_pays(kin, v_top) {
        return None;
    }
    let up = ladder_climb(kin, entry, v_top, kin.length)?;
    let down = ladder_climb(
        &reversed(kin),
        (exit.0, -exit.1),
        v_top,
        kin.length - up.length,
    )?;
    let cruise_length = kin.length - up.length - down.length;
    if cruise_length < 0.0 {
        return None;
    }

    let cruise_speed = end_state(&up.phases, entry).0;
    let cruise_dt = cruise_length / cruise_speed;
    let mut out = up.phases;
    out.push(StraightPhase {
        t0: up.time,
        dt: cruise_dt,
        s0: up.length,
        v0: cruise_speed,
        a0: 0.0,
        j: 0.0,
    });
    out.extend(shifted(
        &reverse_chain(kin.length, &down.phases),
        0.0,
        up.time + cruise_dt,
    ));
    Some(coalesce(out))
}

fn plan_at(
    kin: &Kinematics,
    entry: (f64, f64),
    exit: (f64, f64),
    v_top: f64,
) -> Result<Vec<StraightPhase>, VelocityError> {
    let caps = caps_at(kin, v_top);
    if caps.a <= 0.0 || caps.j <= 0.0 {
        return cruise_only(kin, entry, exit).map_err(|_| {
            VelocityError::InfeasibleBoundary(BoundaryInfeasibility::AccelOverLimit {
                a: entry.1.abs().max(exit.1.abs()),
                a_max: caps.a,
            })
        });
    }
    profile::straight_chain_between(entry, exit, kin.length, caps.v, caps.a, caps.j)
}

/// A candidate plan: the cap speed it was built at, how long it takes, and the
/// chain itself.
struct Candidate {
    v_top: f64,
    time: f64,
    chain: Vec<StraightPhase>,
}

/// Keep the quickest plan at `v_top` if it beats the incumbent, weighing the
/// member's one cap set against the ladder. A refusal the caps explain is a
/// candidate the search discards; anything else is a broken boundary state and
/// belongs to the caller.
fn keep_quicker(
    kin: &Kinematics,
    entry: (f64, f64),
    exit: (f64, f64),
    v_top: f64,
    best: &mut Candidate,
) -> Result<(), VelocityError> {
    let mut offer = |chain: Vec<StraightPhase>| {
        let time = chain_time(&chain);
        if time < best.time {
            *best = Candidate { v_top, time, chain };
        }
    };
    let single = plan_at(kin, entry, exit, v_top);
    let worth_the_phases = single.as_ref().map_or(f64::INFINITY, |chain| {
        chain_time(chain) * (1.0 - LADDER_TIME_WIN)
    });
    if let Some(chain) = ladder_plan(kin, entry, exit, v_top) {
        if chain_time(&chain) < worth_the_phases {
            offer(chain);
        }
    }
    match single {
        Ok(chain) => {
            offer(chain);
            Ok(())
        }
        Err(e) if caps_too_weak(&e) => Ok(()),
        Err(e) => Err(e),
    }
}

/// Quickest chain any cap set in the bracket yields. Raising the cap speed
/// quickens the chain until the caps can no longer carry it that fast, and
/// slows it again beyond that as the authority they leave shrinks — so the
/// bracket is swept coarsely and then closed in on around the best sample,
/// rather than pushed to the highest cap speed that merely closes.
fn bounded_plan(
    kin: &Kinematics,
    entry: (f64, f64),
    exit: (f64, f64),
) -> Result<Vec<StraightPhase>, VelocityError> {
    let (lo, ceiling) = plan_bracket(kin, entry, exit)?;
    let floor_caps = caps_at(kin, lo);
    if floor_caps.a <= 0.0 || floor_caps.j <= 0.0 {
        return cruise_only(kin, entry, exit);
    }
    let seed = plan_at(kin, entry, exit, lo)?;
    let mut best = Candidate {
        v_top: lo,
        time: chain_time(&seed),
        chain: seed,
    };
    let span = ceiling - lo;
    let step = span / f64::from(PLAN_PROBE_STEPS);
    for i in 1..=PLAN_PROBE_STEPS {
        keep_quicker(kin, entry, exit, lo + step * f64::from(i), &mut best)?;
    }
    let mut below = (best.v_top - step).max(lo);
    let mut above = (best.v_top + step).min(ceiling);
    for _ in 0..PLAN_REFINE_ITERS {
        let anchor = best.v_top;
        keep_quicker(kin, entry, exit, 0.5 * (below + anchor), &mut best)?;
        keep_quicker(kin, entry, exit, 0.5 * (anchor + above), &mut best)?;
        if best.v_top < anchor {
            above = anchor;
        } else if best.v_top > anchor {
            below = anchor;
        } else {
            below = 0.5 * (below + anchor);
            above = 0.5 * (anchor + above);
        }
    }
    Ok(best.chain)
}

/// Chain between two boundary states: band by band where the band ceilings
/// spread far enough for the tighter caps to buy speed, the whole member's own
/// single cap set otherwise, and each the other's fallback. The order matters:
/// [`closes`] answers in the same order, so what the envelope was promised is
/// what gets emitted.
fn member_plan(
    kin: &Kinematics,
    entry: (f64, f64),
    exit: (f64, f64),
) -> Result<Vec<StraightPhase>, VelocityError> {
    let bands = bands(kin);
    if banding_pays(&bands) {
        if let Ok(chain) = banded_plan(&bands, entry, exit) {
            return Ok(chain);
        }
    }
    match bounded_plan(kin, entry, exit) {
        Ok(chain) => Ok(chain),
        Err(whole) if caps_too_weak(&whole) && bands.len() >= 2 => {
            banded_plan(&bands, entry, exit).map_err(|_| whole)
        }
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
    let floor_caps = caps_at(kin, lo);
    if floor_caps.a <= 0.0 || floor_caps.j <= 0.0 {
        return cruise_only(kin, entry, exit).map(drop);
    }
    match plan_at(kin, entry, exit, ceiling) {
        Ok(_) => return Ok(()),
        Err(e) if !caps_too_weak(&e) => return Err(e),
        Err(_) => {}
    }
    plan_at(kin, entry, exit, lo).map(drop)
}

fn probe_bands(
    bands: &[Kinematics],
    entry: (f64, f64),
    exit: (f64, f64),
) -> Result<(), VelocityError> {
    let edges = band_edge_states(bands, entry, exit);
    bands
        .iter()
        .enumerate()
        .try_for_each(|(i, band)| probe_closes(band, edges[i], edges[i + 1]))
}

/// Whether [`member_plan`] would find a chain, attempted in the same order it
/// attempts them. Every probe stops at the bracket ends, so the answer costs a
/// bounded number of closed-form solves however many bands the member has.
fn closes(kin: &Kinematics, entry: (f64, f64), exit: (f64, f64)) -> Result<(), VelocityError> {
    let bands = bands(kin);
    if banding_pays(&bands) && probe_bands(&bands, entry, exit).is_ok() {
        return Ok(());
    }
    match probe_closes(kin, entry, exit) {
        Ok(()) => Ok(()),
        Err(whole) if caps_too_weak(&whole) && bands.len() >= 2 => {
            probe_bands(&bands, entry, exit).map_err(|_| whole)
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

/// The member as its entry sees it: where it is planned band by band, the band
/// the entry state actually lands in, whose ceiling is what that end has to
/// respect. A blend's `kappa = 0` seam does not owe the curvature peak's price,
/// and gating the seam on the peak is what holds a whole blend to its apex
/// speed.
fn entry_band(kin: &Kinematics) -> Kinematics {
    let bands = bands(kin);
    let head = bands.first().filter(|_| banding_pays(&bands));
    band_kin(kin, 0.0, head.map_or(kin.length, |b| b.length))
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
    let at_entry = entry_band(kin);
    let ceiling = top_speed_ceiling(&at_entry);
    if !(v_entry.is_finite() && v_entry >= 0.0) || v_entry > ceiling {
        return None;
    }
    let holds = |mag: f64| match required_top(&at_entry, (v_entry, mag), None, ceiling) {
        Ok(need) => caps_at(&at_entry, need).a >= mag,
        Err(_) => false,
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

/// The member's own fastest pass, forward or backward, where that pass is
/// exactly what the boundary states name. A plan between two states cannot
/// always rebuild the reach that named one of them — it bands the member and
/// the band edges it computes are not the reach's own — but the reach is a
/// trajectory between the very states in hand, so it is the answer.
fn extremal_chain(
    kin: &Kinematics,
    entry: (f64, f64),
    exit: (f64, f64),
) -> Option<Vec<StraightPhase>> {
    if let Ok(forward) = reach_spans(kin, entry) {
        if states_match(exit, end_state(&forward, entry)) {
            return Some(forward);
        }
    }
    let back = reversed(kin);
    let seed = (exit.0, -exit.1);
    let backward = reach_spans(&back, seed).ok()?;
    let (v, a) = end_state(&backward, seed);
    states_match(entry, (v, -a)).then(|| reverse_chain(kin.length, &backward))
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
        Err(bounded) => match extremal_chain(kin, entry, exit) {
            Some(chain) => certified_chain(kin, &chain),
            None => Err(bounded),
        },
    }
}
