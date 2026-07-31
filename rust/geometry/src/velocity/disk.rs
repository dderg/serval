//! Run reconstruction: the routing that hands each member to the solver that
//! plans it, and the sample view read off the chain that solver emits.

use std::f64::consts::FRAC_PI_2;

use super::chain::{chain_is_continuous, chain_states, phase_end_s};
use super::profile::{self, BoundaryInfeasibility, StraightPhase};
use super::{VelocityError, certify, curved};

const RK_MIN_STEP_FRAC: f64 = 1e-6;
const RK_MAX_STEPS: u32 = 100_000;
const SAMPLE_MAX_POINTS: usize = 16_384;
const SAMPLE_STEP_MM: f64 = 0.01;
const SAMPLES_PER_MEMBER_MIN: usize = 16;
const SAMPLE_DEDUP_MM: f64 = 1e-9;
/// A curvature (or boundary) speed limit this close to the flat ceiling is
/// the fitter's own blend sizing — it solves the corner radius so the apex
/// speed lands *at* the feedrate, to float tolerance. Taking the raw `min`
/// would notch the cap by ~1e-6 mm/s at every blend, and the jerk-limited
/// pass would dutifully dip into each notch with a nanosecond full-rail
/// bang whose phase joints then ring through the lowering as absurd
/// acceleration slivers. Snap such limits up to the ceiling instead.
pub(super) const CAP_NOTCH_REL: f64 = 1e-6;

pub(super) fn notch_free_min(flat_ceiling: f64, limit: f64) -> f64 {
    if limit >= flat_ceiling * (1.0 - CAP_NOTCH_REL) {
        flat_ceiling
    } else {
        limit
    }
}
const KAPPA_EPS: f64 = 1e-9;

pub(super) struct Kinematics {
    pub length: f64,
    pub accel: f64,
    pub jerk: f64,
    pub kappa0: f64,
    pub sigma: f64,
    pub flat_ceiling: f64,
}

impl Kinematics {
    fn reversed(&self) -> Kinematics {
        Kinematics {
            length: self.length,
            accel: self.accel,
            jerk: self.jerk,
            kappa0: self.kappa0 + self.sigma * self.length,
            sigma: -self.sigma,
            flat_ceiling: self.flat_ceiling,
        }
    }

    fn kappa_abs(&self, s: f64) -> f64 {
        (self.kappa0 + self.sigma * s).abs()
    }

    fn ceiling_w(&self, s: f64) -> f64 {
        let flat_w = self.flat_ceiling * self.flat_ceiling;
        let k = self.kappa_abs(s);
        if k > 0.0 {
            flat_w.min(self.accel / k)
        } else {
            flat_w
        }
    }

    pub(super) fn is_straight(&self) -> bool {
        self.kappa0.abs() <= KAPPA_EPS && self.sigma.abs() <= KAPPA_EPS
    }
}

pub(super) fn limit_speed(kappa_abs: f64, accel: f64) -> f64 {
    if kappa_abs > 0.0 {
        (accel / kappa_abs).sqrt()
    } else {
        f64::INFINITY
    }
}

pub(super) fn const_kappa_reach_w(w_in: f64, length: f64, accel: f64, kappa_abs: f64) -> f64 {
    if kappa_abs == 0.0 {
        return w_in + 2.0 * accel * length;
    }
    let w_lim = accel / kappa_abs;
    let x0 = (w_in / w_lim).clamp(0.0, 1.0);
    let arg = 2.0 * kappa_abs * length + libm::asin(x0);
    if arg >= FRAC_PI_2 {
        w_lim
    } else {
        w_lim * libm::sin(arg)
    }
}

fn rk4_step<F: Fn(f64, f64) -> f64>(f: &F, s: f64, w: f64, h: f64) -> f64 {
    let k1 = f(s, w);
    let k2 = f(s + 0.5 * h, w + 0.5 * h * k1);
    let k3 = f(s + 0.5 * h, w + 0.5 * h * k2);
    let k4 = f(s + h, w + h * k3);
    w + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
}

fn integrate_ode(kin: &Kinematics, w_in: f64, target: f64, tol: f64) -> Option<f64> {
    let f = |s: f64, w: f64| {
        let k = kin.kappa0 + kin.sigma * s;
        2.0 * (kin.accel * kin.accel - (k * k) * (w * w)).max(0.0).sqrt()
    };
    let min_h = (kin.length * RK_MIN_STEP_FRAC).max(f64::MIN_POSITIVE);
    let mut s = 0.0;
    let mut w = w_in.min(kin.ceiling_w(0.0)).max(0.0);
    let mut next_h = target;
    let mut steps = 0u32;
    loop {
        if s >= target {
            return Some(w);
        }
        if steps >= RK_MAX_STEPS {
            return None;
        }
        let mut h = next_h.min(target - s);
        loop {
            let big = rk4_step(&f, s, w, h);
            let half = rk4_step(&f, s, w, 0.5 * h);
            let small = rk4_step(&f, s + 0.5 * h, half, 0.5 * h);
            let err = (big - small).abs();
            let bound = tol * (1.0 + small.abs());
            if err <= bound || h <= min_h {
                s += h;
                w = small.min(kin.ceiling_w(s)).max(0.0);
                next_h = if err < 0.1 * bound { 2.0 * h } else { h };
                break;
            }
            h *= 0.5;
        }
        steps += 1;
    }
}

fn disk_reach_w(kin: &Kinematics, w_in: f64, s: f64, tol: f64) -> Option<f64> {
    if kin.sigma == 0.0 {
        let flat_w = kin.flat_ceiling * kin.flat_ceiling;
        Some(const_kappa_reach_w(w_in, s, kin.accel, kin.kappa0.abs()).min(flat_w))
    } else {
        integrate_ode(kin, w_in, s, tol)
    }
}

pub(super) fn disk_reach_v(kin: &Kinematics, v_in: f64, s: f64, tol: f64) -> Option<f64> {
    Some(disk_reach_w(kin, v_in * v_in, s, tol)?.max(0.0).sqrt())
}

pub(super) fn disk_reach_v_rev(kin: &Kinematics, v_in: f64, s: f64, tol: f64) -> Option<f64> {
    disk_reach_v(&kin.reversed(), v_in, s, tol)
}

pub(super) struct RunMember<'a> {
    pub kin: &'a Kinematics,
    pub exit_v: f64,
    pub exit_a: f64,
}

fn certify_or_panic(kin: &Kinematics, chain: &[StraightPhase]) {
    for p in chain {
        assert!(
            certify::is_certified(kin, p.s0, p.v0, p.a0, p.j, p.dt),
            "straight phase is not certified feasible: s0={} v0={} a0={} j={} dt={} \
             (length={} accel={} jerk={} flat_ceiling={})",
            p.s0,
            p.v0,
            p.a0,
            p.j,
            p.dt,
            kin.length,
            kin.accel,
            kin.jerk,
            kin.flat_ceiling
        );
    }
}

/// A straight member owes no curvature term, so its optimum is the closed-form
/// triple-limited chain between the boundary states — solved outright rather
/// than marched, and certified phase by phase before it is handed on.
///
/// A boundary pair the member cannot realise is reported, exactly as the curved
/// solver reports one, and the caller turns it into a planning error: the
/// settlement is what owes every member a pair it can close. Any other failure is
/// a solver bug and panics.
fn certified_straight_chain(
    kin: &Kinematics,
    entry: (f64, f64),
    exit: (f64, f64),
) -> Result<Vec<StraightPhase>, VelocityError> {
    let chain = match profile::straight_chain_between(
        entry,
        exit,
        kin.length,
        kin.flat_ceiling,
        kin.accel,
        kin.jerk,
    ) {
        Ok(chain) => chain,
        Err(why @ VelocityError::InfeasibleBoundary(_)) => return Err(why),
        Err(why) => panic!(
            "straight member length {} entry {entry:?} exit {exit:?} under \
             v_max {} a_max {} j_max {} is unplannable: {why:?}",
            kin.length, kin.flat_ceiling, kin.accel, kin.jerk
        ),
    };
    assert!(
        !chain.is_empty(),
        "a straight member of length {} planned to an empty chain",
        kin.length
    );
    certify_or_panic(kin, &chain);
    Ok(chain)
}

fn closed_form_is_available(kin: &Kinematics) -> bool {
    kin.is_straight()
        && kin.jerk.is_finite()
        && kin.jerk > 0.0
        && kin.accel > 0.0
        && kin.flat_ceiling > 0.0
        && kin.length > 0.0
}

/// Sample positions for the member's `(s, v, a)` view: each phase of the chain
/// subdivided at the sample density, so every regime boundary lands on a node
/// and no consumer bridges a phase joint with a single chord.
fn sample_arcs(chain: &[StraightPhase], length: f64) -> Vec<f64> {
    let mut breaks = vec![0.0_f64];
    for p in chain {
        let s1 = phase_end_s(p).clamp(0.0, length);
        if s1 - breaks[breaks.len() - 1] > SAMPLE_DEDUP_MM {
            breaks.push(s1);
        }
    }
    let last = breaks.len() - 1;
    if length - breaks[last] > SAMPLE_DEDUP_MM {
        breaks.push(length);
    } else {
        breaks[last] = length;
    }
    if breaks.len() < 2 {
        return breaks;
    }
    let total_steps = ((length / SAMPLE_STEP_MM).ceil() as usize)
        .clamp(SAMPLES_PER_MEMBER_MIN, SAMPLE_MAX_POINTS);
    let mut arcs = Vec::with_capacity(total_steps + breaks.len());
    for w in breaks.windows(2) {
        let (lo, hi) = (w[0], w[1]);
        let steps = (((hi - lo) / length * total_steps as f64).ceil() as usize).max(1);
        for k in 0..steps {
            arcs.push(lo + (hi - lo) * (k as f64) / (steps as f64));
        }
    }
    arcs.push(length);
    arcs
}

/// A member's samples read off the chain it executes, with the run's own
/// boundary states at its ends: the samples are a view of the profile the
/// lowering executes, not an independent integration to be reconciled with it.
fn samples_from_chain(
    arcs: &[f64],
    chain: &[StraightPhase],
    entry: (f64, f64),
    exit: (f64, f64),
) -> Vec<(f64, f64, f64)> {
    let mut samples: Vec<(f64, f64, f64)> = if chain.is_empty() {
        arcs.iter().map(|&s| (s, entry.0, entry.1)).collect()
    } else {
        chain_states(chain, arcs)
            .into_iter()
            .zip(arcs)
            .map(|((v, a), &s)| (s, v, a))
            .collect()
    };
    let last = samples.len() - 1;
    samples[0].1 = entry.0;
    samples[0].2 = entry.1;
    samples[last].1 = exit.0;
    samples[last].2 = exit.1;
    samples
}

/// A member no chain can close between the two boundary states its envelope
/// neighbours fixed: the entry state its predecessor must hand it is not one
/// this member can be entered at and still land on its own exit.
pub(super) struct UnreachableMember {
    pub index: usize,
    pub entry: (f64, f64),
    pub exit: (f64, f64),
    pub why: VelocityError,
}

/// How many members of each kind the envelope planned, so the reachability
/// census has a denominator.
#[derive(Default)]
pub(super) struct MemberClassCounts {
    pub straight: u32,
    pub curved: u32,
}

pub(super) struct RunReconstruction {
    pub samples: Vec<Vec<(f64, f64, f64)>>,
    pub exit_states: Vec<(f64, f64)>,
    pub phases: Vec<Vec<StraightPhase>>,
    /// Per-member chain planned between the run's `(v, a)` boundary states.
    /// Empty where the member has no solver, or where no chain can close the
    /// boundary pair. Every member with a chain publishes it through `phases`.
    pub envelope_chains: Vec<Vec<StraightPhase>>,
    pub unreachable: Vec<UnreachableMember>,
    pub planned: MemberClassCounts,
}

/// The run's boundary states in order: the run's own entry, then each member's
/// envelope exit. Member `i` is planned from `boundary[i]` to `boundary[i + 1]`.
fn run_boundary_states(
    members: &[RunMember],
    run_start_v: f64,
    run_start_a: f64,
) -> Vec<(f64, f64)> {
    let mut boundary = Vec::with_capacity(members.len() + 1);
    boundary.push((run_start_v, run_start_a));
    boundary.extend(members.iter().map(|m| (m.exit_v, m.exit_a)));
    boundary
}

pub(super) fn curved_solver_is_available(kin: &Kinematics) -> bool {
    kin.length > 0.0
        && kin.length.is_finite()
        && kin.accel > 0.0
        && kin.jerk > 0.0
        && kin.flat_ceiling > 0.0
        && kin.kappa0.is_finite()
        && kin.sigma.is_finite()
}

/// A zero-length member spans no arc and so executes nothing: the boundary
/// handoff is the whole member. It carries no authority either, so a demand that
/// it change speed is reported rather than quietly dropped.
fn boundary_handoff(
    entry: (f64, f64),
    exit: (f64, f64),
) -> Result<Vec<StraightPhase>, VelocityError> {
    if entry.0 != exit.0 {
        return Err(VelocityError::InfeasibleBoundary(
            BoundaryInfeasibility::SpeedChangeWithoutAuthority {
                from: entry.0,
                to: exit.0,
            },
        ));
    }
    Ok(Vec::new())
}

/// The chain a member executes between two known boundary states: the closed form
/// on a straight the jerk budget bounds, the certified curved solver on everything
/// else — curvature, an unlimited jerk budget, or both. A degenerate budget or
/// ceiling is not a member the planner may quietly skip, and the solver's own
/// validation says so.
fn member_chain(
    kin: &Kinematics,
    entry: (f64, f64),
    exit: (f64, f64),
) -> Result<Vec<StraightPhase>, VelocityError> {
    if kin.length == 0.0 {
        return boundary_handoff(entry, exit);
    }
    if closed_form_is_available(kin) {
        certified_straight_chain(kin, entry, exit)
    } else {
        curved::curved_chain(kin, entry, exit)
    }
}

/// Reconstruct the run: per-member `(s, v, a)` samples, the profile state
/// `(v, a)` at each member's exit seam (what a streaming cut carries into the
/// next window to continue this exact curve), and per-member closed-form jerk
/// phases in move-local time/arc-length.
///
/// Every member emits the chain its own solver plans between the two `(v, a)`
/// boundary states the backward requirement pass fixed — so a blend inherits its
/// entry brake instead of manufacturing one at a seam with no authority to build
/// it — and reads its samples off that chain. A member whose boundary pair no
/// chain can close is reported; the settlement is what owes every member a pair
/// it can close.
pub(super) fn reconstruct_run(
    members: &[RunMember],
    run_start_v: f64,
    run_start_a: f64,
    _tol: f64,
) -> RunReconstruction {
    let boundary = run_boundary_states(members, run_start_v, run_start_a);

    let mut out = RunReconstruction {
        samples: Vec::with_capacity(members.len()),
        exit_states: Vec::with_capacity(members.len()),
        phases: Vec::with_capacity(members.len()),
        envelope_chains: Vec::with_capacity(members.len()),
        unreachable: Vec::new(),
        planned: MemberClassCounts::default(),
    };
    for (index, m) in members.iter().enumerate() {
        let envelope_entry = boundary[index];
        let envelope_exit = boundary[index + 1];
        if m.kin.length > 0.0 {
            if m.kin.is_straight() {
                out.planned.straight += 1;
            } else {
                out.planned.curved += 1;
            }
        }
        let chain = match member_chain(m.kin, envelope_entry, envelope_exit) {
            Ok(chain) => {
                assert!(
                    chain_is_continuous(&chain, m.kin.jerk.is_finite()),
                    "member {index} of length {} planned a discontinuous chain \
                     between entry {envelope_entry:?} and exit {envelope_exit:?}",
                    m.kin.length
                );
                chain
            }
            Err(why) => {
                out.unreachable.push(UnreachableMember {
                    index,
                    entry: envelope_entry,
                    exit: envelope_exit,
                    why,
                });
                Vec::new()
            }
        };
        let arcs = sample_arcs(&chain, m.kin.length);
        let samples = samples_from_chain(&arcs, &chain, envelope_entry, envelope_exit);
        out.envelope_chains.push(chain.clone());
        out.exit_states
            .push(samples.last().map_or(envelope_exit, |p| (p.1, p.2)));
        out.samples.push(samples);
        out.phases.push(chain);
    }
    out
}

#[cfg(test)]
pub(super) fn sample_profile(
    kin: &Kinematics,
    entry: f64,
    exit: f64,
    tol: f64,
) -> Vec<(f64, f64, f64)> {
    let member = RunMember {
        kin,
        exit_v: exit,
        exit_a: 0.0,
    };
    let mut run = reconstruct_run(&[member], entry, 0.0, tol);
    assert!(
        run.unreachable.is_empty(),
        "sample_profile was handed a boundary pair the member cannot close: {:?}",
        run.unreachable.first().map(|m| (m.entry, m.exit, m.why))
    );
    run.samples.swap_remove(0)
}

#[cfg(test)]
mod tests;
