use std::f64::consts::FRAC_PI_2;

use super::profile::StraightPhase;
use super::ride;

const RK_MIN_STEP_FRAC: f64 = 1e-6;
const RK_MAX_STEPS: u32 = 100_000;
/// Seed spacing for the integration grid: the resolution a short move needs.
const GRID_STEP_MM: f64 = 0.01;
/// Per-member ceiling on the *seed* grid's uniform nodes. `GRID_STEP_MM` on a
/// long member mints nodes the physics has no structure for. That is not free
/// sampling — the reconstruction is one quintic window per node, and the
/// lowering spends output pieces per window, so node count *is* piece count.
/// A 15.6 mm straight cruising at its step-rate ceiling sampled 1560 nodes,
/// lowered to 2336 pieces for one shaped segment, and eight such segments
/// cost 3.5 s each in shaper convolution fits on a Pi 4 (`repro_z14.gcode`,
/// line 2710) — the whole 2 s pump lead, spent on one segment.
///
/// Nothing about that count is an error bound, which is why it is only a
/// seed: [`reconstruct_flat`] then buys nodes back, member by member, for
/// the ones whose reconstruction [`ringing_member`] convicts, and for no
/// others.
///
/// Widening a cell does not blunt a member boundary: the interior-boundary
/// ladders below pin the straddling cells at `GRID_STEP_MM` at any cap, so
/// the ceiling step there reads super-rail exactly as an uncapped grid reads
/// it.
const MEMBER_SEED_MAX_POINTS: usize = 256;
const GRID_MIN_STEPS: usize = 16;
const REST_REFINE_MIN_MM: f64 = 1e-5;
const GRID_DEDUP_MM: f64 = 1e-9;
/// A reconstruction the lowering can fit puts the pass's acceleration inside
/// the 50 mm/s² budget its pieces are allowed to miss by
/// (`SHAPED_FIT_TOL_ACCEL_MM_S2` in the shaper, `fit_tol_accel_mm_s2` in the
/// planner config). An acceleration swing under that is noise the pieces
/// absorb; a swing over it is a plan the machine will actually execute, so
/// it is what the grid is held to.
const GRID_ACCEL_TOL_MM_S2: f64 = 50.0;
/// A straight member's cap is a constant ceiling under a monotone brake
/// envelope, so the fastest feasible profile across it rises, holds, and
/// falls: acceleration crosses zero on the way onto the ceiling and again on
/// the way off, and nowhere else. Two is therefore the physics; the third
/// crossing is the grid.
const PROFILE_REVERSALS_MAX: usize = 2;
/// Refining a ringing member goes straight to [`GRID_STEP_MM`] — the pitch
/// the seed cap widened away from — and then halves that up to this multiple.
/// Four times the pitch resolves a jerk swing at 20 mm/s into hundreds of
/// cells; a member still hunting there is not under-sampled.
const GRID_REFINE_GROWTH: usize = 4;
/// How far a chained run of near-constant-acceleration phases may bend away
/// from the arc its members describe: a fiftieth of the lowering's own
/// 0.005 mm position budget. The chained phase matches its run's end arc and
/// end velocity exactly, so this bounds the whole error it introduces there.
const CHAIN_MERGE_ARC_MM: f64 = 1e-4;
/// The acceleration spread a chained run may absorb. What the chaining is for
/// is float residue — a cruise the pass reaches by landing tangent holds `a`
/// to a ten-thousandth of a mm/s² of zero, not at zero — so a hundredth is
/// two orders of margin over that and five below the lowering's own budget.
/// Above it the phases describe different accelerations, and flattening them
/// into one leaves that difference as a step at the run's far joint.
const CHAIN_MERGE_ACCEL_MM_S2: f64 = 1e-2;

/// Why a run has no reconstruction.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) enum ReconstructError {
    /// The pass could not integrate the run: a brake envelope steeper than
    /// the acceleration rail, or a diverged ODE.
    Diverged,
    /// A member's reconstruction still rings at the finest grid refinement
    /// is allowed to build, so the profile the lowering would fit is a limit
    /// cycle rather than the plan.
    GridBudget {
        nodes: usize,
        reversals: usize,
        member: usize,
    },
}

use ReconstructError::Diverged;

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
const VELOCITY_FLOOR: f64 = 1e-9;
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

    fn is_straight(&self) -> bool {
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

pub(super) fn disk_rail_accel(accel: f64, kappa_abs: f64, v: f64) -> f64 {
    let a_n = kappa_abs * v * v;
    (accel * accel - a_n * a_n).max(0.0).sqrt()
}

pub(super) struct RunMember<'a> {
    pub kin: &'a Kinematics,
    pub exit_v: f64,
    pub fwd_s: f64,
}

/// A dense uniform arc-length grid for the profile pass. Each member is sampled
/// at a fixed step within its own span — per-member rather than per-run so
/// appending moves never reshuffles an earlier member's grid (the streaming
/// locked prefix must be invariant under append).
///
/// Two geometric refinements sit on top of the uniform nodes, both a function
/// of the run's own anchors alone, so the locked-prefix grids stay
/// append-invariant:
///
/// - A rest anchor: the jerk ramp from rest covers `a³/(6j²)` of arc (a cell
///   or two) over many milliseconds, and samples uniform in arc-length would
///   leave the lowering nothing to fit the ramp with.
/// - Every interior member boundary. The ceiling is piecewise constant per
///   member, so a boundary carries a velocity *step*, and the cell straddling
///   it is the only place the pass can read that step — as the chord
///   `Δ(v²)/2Δs`. Wide enough, and the chord lands in the band between the
///   accel rail and the jerk-shed bound where the pass classifies it as a
///   followable descent it must commit a whole-run brake to, instead of the
///   super-rail step it is. Holding the straddling cell at `GRID_STEP_MM`
///   keeps the step super-rail at any node cap, exactly as an uncapped grid
///   reads it.
#[cfg(test)]
fn seed_grid(members: &[RunMember], entry_rest: bool, exit_rest: bool) -> Vec<f64> {
    let steps: Vec<usize> = members.iter().map(|m| member_seed_steps(m.kin)).collect();
    grid_from_steps(members, &steps, entry_rest, exit_rest)
}

fn member_seed_steps(kin: &Kinematics) -> usize {
    ((kin.length / GRID_STEP_MM).ceil() as usize).clamp(GRID_MIN_STEPS, MEMBER_SEED_MAX_POINTS)
}

fn grid_from_steps(
    members: &[RunMember],
    steps_per_member: &[usize],
    entry_rest: bool,
    exit_rest: bool,
) -> Vec<f64> {
    let mut s: Vec<f64> = Vec::new();
    let mut member_step: Vec<f64> = Vec::with_capacity(members.len());
    for (m, &steps) in members.iter().zip(steps_per_member) {
        let len = m.kin.length;
        member_step.push(len / steps as f64);
        for k in 0..steps {
            s.push(m.fwd_s + len * (k as f64) / (steps as f64));
        }
    }
    let start = members[0].fwd_s;
    let last = &members[members.len() - 1];
    let end = last.fwd_s + last.kin.length;
    let total = end - start;
    // Each rest ladder climbs at least to `GRID_STEP_MM` and, where the node
    // cap has widened its own end's uniform spacing past that, all the way to
    // that spacing. Stopping short leaves a hole between the ladder's top
    // rung and the first uniform node, and the pass's first arc out of rest
    // jumps it in one step — an acceleration step at the rest anchor, exactly
    // what the ladder exists to prevent.
    if entry_rest {
        ladder_toward(
            start,
            -1.0,
            REST_REFINE_MIN_MM,
            member_step[0].max(GRID_STEP_MM).min(0.5 * total),
            &mut s,
        );
    }
    if exit_rest {
        ladder_toward(
            end,
            1.0,
            REST_REFINE_MIN_MM,
            member_step[member_step.len() - 1]
                .max(GRID_STEP_MM)
                .min(0.5 * total),
            &mut s,
        );
    }
    for k in 1..members.len() {
        let boundary = members[k].fwd_s;
        ladder_toward(boundary, 1.0, GRID_STEP_MM, member_step[k - 1], &mut s);
        ladder_toward(boundary, -1.0, GRID_STEP_MM, member_step[k], &mut s);
    }
    s.push(end);
    s.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    // Collapse sub-nanometre node pairs (ladder vs uniform collisions): the
    // per-node sample noise between two such nodes is a real velocity step
    // over a ~picosecond span, which the lowering would bridge as a sliver
    // piece with a physically absurd acceleration spike.
    s.dedup_by(|a, b| (*a - *b).abs() <= GRID_DEDUP_MM);
    s
}

/// Rungs approaching `anchor` from the side `sign` points back along: at
/// `anchor - sign·delta` for `delta` doubling from `first` while it stays
/// under `limit`. Empty when the uniform spacing is already at or below
/// `first` — there is nothing left to refine.
fn ladder_toward(anchor: f64, sign: f64, first: f64, limit: f64, s: &mut Vec<f64>) {
    let mut delta = first;
    while delta < limit {
        s.push(anchor - sign * delta);
        delta *= 2.0;
    }
}

/// Members whose span covers `s_run` (two at a seam, one in an interior).
fn members_at<'a, 'b>(
    members: &'b [RunMember<'a>],
    s_run: f64,
) -> impl Iterator<Item = (&'b RunMember<'a>, f64)> {
    members.iter().filter_map(move |m| {
        let lo = m.fwd_s;
        let hi = lo + m.kin.length;
        (s_run >= lo - 1e-9 && s_run <= hi + 1e-9)
            .then(|| (m, (s_run - lo).clamp(0.0, m.kin.length)))
    })
}

/// The per-node constraint arrays `(vlc, accel, kappa)` for ascending grid
/// arcs `s`: at each node the tightest ceiling and budget, and the largest
/// curvature, over every member whose span covers it (two at a seam, one in
/// an interior). At a curvature-discontinuous seam the ceiling must honor
/// the tighter side — otherwise the dip's own endpoint (read by the member
/// that ends there) could overshoot its curvature limit. Members tile the
/// run in order, so a forward walk visits each once instead of scanning the
/// whole run per node.
fn constraint_arrays(members: &[RunMember], s: &[f64]) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
    let mut vlc = Vec::with_capacity(s.len());
    let mut accel = Vec::with_capacity(s.len());
    let mut kappa = Vec::with_capacity(s.len());
    let mut first = 0usize;
    for &x in s {
        while first + 1 < members.len()
            && members[first].fwd_s + members[first].kin.length < x - 1e-9
        {
            first += 1;
        }
        let mut v = f64::INFINITY;
        let mut a = f64::INFINITY;
        let mut k = 0.0_f64;
        for (m, local) in members_at(&members[first..(first + 2).min(members.len())], x) {
            let kappa_abs = m.kin.kappa_abs(local);
            let curv = notch_free_min(m.kin.flat_ceiling, limit_speed(kappa_abs, m.kin.accel));
            v = v.min(m.kin.flat_ceiling.min(curv));
            a = a.min(m.kin.accel);
            k = k.max(kappa_abs);
        }
        vlc.push(v);
        accel.push(a);
        kappa.push(k);
    }
    (vlc, accel, kappa)
}

/// Per-cell slope-accel of a piecewise-linear-in-`v` cap: `d(v²/2)/ds` of the
/// chord, which is exactly zero on flat cells and self-consistent with the
/// node-interpolated cap values the pass lands on.
fn chord_slopes(s: &[f64], cap: &[f64]) -> Vec<f64> {
    let n = s.len();
    let mut out = vec![0.0_f64; n];
    for i in 0..n - 1 {
        let span = s[i + 1] - s[i];
        out[i] = if span > 1e-12 {
            (cap[i + 1] * cap[i + 1] - cap[i] * cap[i]) / (2.0 * span)
        } else {
            0.0
        };
    }
    out[n - 1] = out[n - 2];
    out
}

/// Reconstruct under infinite jerk. The optimum is a continuous curve with
/// an exact definition — accelerate on the disk rail (`d(v²/2)/ds =
/// √(A² − κ²v⁴)`, an ODE in arc-length), ride the velocity-limit curve
/// where it binds, brake onto the backward envelope — so it is integrated as
/// exactly that by [`integrate_disk`], not read off grid chords. Acceleration
/// is continuous between regime switches because the physics is; it steps
/// only at the switches, which are located by root-finding inside cells
/// rather than snapped to grid knots. If the integrator rejects the run
/// (interior rest, degenerate cell), the first-order staircase reconstruction
/// is emitted instead.
fn infinite_jerk_profile(
    s: &[f64],
    vlc: &[f64],
    accel: &[f64],
    kappa: &[f64],
    entry_v: f64,
    exit_v: f64,
) -> (Vec<(f64, f64, f64)>, Vec<StraightPhase>) {
    if let Some(chain) = disk_ride_chain(s, vlc, accel, kappa, entry_v, exit_v) {
        let samples = pinned_samples(&chain, s, entry_v, exit_v);
        return (samples, chain);
    }
    staircase_profile(s, vlc, accel, kappa, entry_v, exit_v)
}

fn pinned_samples(
    chain: &[StraightPhase],
    s: &[f64],
    entry_v: f64,
    exit_v: f64,
) -> Vec<(f64, f64, f64)> {
    let n = s.len();
    let mut samples: Vec<(f64, f64, f64)> = ride::chain_states(chain, s)
        .into_iter()
        .zip(s)
        .map(|((v, a), &x)| (x, v, a))
        .collect();
    samples[0].1 = entry_v;
    samples[n - 1].1 = exit_v;
    if entry_v <= VELOCITY_FLOOR {
        samples[0].2 = 0.0;
    }
    if exit_v <= VELOCITY_FLOOR {
        samples[n - 1].2 = 0.0;
    }
    samples
}

/// The pre-integrator reconstruction, kept as the bail-out: forward–backward
/// `v²` sweeps give grid velocities, lowered as merged zero-jerk spans — a
/// staircase in acceleration wherever the rail varies.
fn staircase_profile(
    s: &[f64],
    vlc: &[f64],
    accel: &[f64],
    kappa: &[f64],
    entry_v: f64,
    exit_v: f64,
) -> (Vec<(f64, f64, f64)>, Vec<StraightPhase>) {
    let n = s.len();
    let mut ceil = vlc.to_vec();
    ceil[0] = ceil[0].min(entry_v);
    ceil[n - 1] = ceil[n - 1].min(exit_v);
    for i in (0..n - 1).rev() {
        let ds = s[i + 1] - s[i];
        let budget = disk_rail_accel(accel[i + 1], kappa[i + 1], ceil[i + 1]);
        let vm = (ceil[i + 1] * ceil[i + 1] + 2.0 * budget * ds).sqrt();
        ceil[i] = ceil[i].min(vm);
    }
    for i in 1..n {
        let ds = s[i] - s[i - 1];
        let budget = disk_rail_accel(accel[i - 1], kappa[i - 1], ceil[i - 1]);
        let vm = (ceil[i - 1] * ceil[i - 1] + 2.0 * budget * ds).sqrt();
        ceil[i] = ceil[i].min(vm);
    }
    ceil[0] = entry_v;
    ceil[n - 1] = exit_v;

    let chain = zero_jerk_chain(s, &ceil);
    let samples = if chain.is_empty() {
        let mut out: Vec<(f64, f64, f64)> = (0..n)
            .map(|i| {
                let (lo, hi) = (i.saturating_sub(1), (i + 1).min(n - 1));
                let dvds = (ceil[hi] - ceil[lo]) / (s[hi] - s[lo]).max(1e-12);
                let rail = disk_rail_accel(accel[i], kappa[i], ceil[i]);
                (s[i], ceil[i], (ceil[i] * dvds).clamp(-rail, rail))
            })
            .collect();
        out[0].1 = entry_v;
        out[n - 1].1 = exit_v;
        if entry_v <= VELOCITY_FLOOR {
            out[0].2 = 0.0;
        }
        if exit_v <= VELOCITY_FLOOR {
            out[n - 1].2 = 0.0;
        }
        out
    } else {
        pinned_samples(&chain, s, entry_v, exit_v)
    };
    (samples, chain)
}

/// Merges maximal equal-`v²`-slope spans into single constant-acceleration
/// phases; an interior rest leaves no chain.
const INFINITE_JERK_SLOPE_MERGE_REL_TOL: f64 = 1e-6;

fn zero_jerk_chain(s: &[f64], v: &[f64]) -> Vec<StraightPhase> {
    let n = s.len();
    let slope = |i: usize| (v[i + 1] * v[i + 1] - v[i] * v[i]) / (2.0 * (s[i + 1] - s[i]));
    let mut chain = Vec::new();
    let mut t0 = 0.0;
    let mut i = 0;
    while i < n - 1 {
        if v[i] + v[i + 1] <= VELOCITY_FLOOR {
            return Vec::new();
        }
        let a_first = slope(i);
        let mut j = i + 1;
        while j < n - 1
            && v[j] + v[j + 1] > VELOCITY_FLOOR
            && (slope(j) - a_first).abs()
                <= INFINITE_JERK_SLOPE_MERGE_REL_TOL * (1.0 + a_first.abs())
        {
            j += 1;
        }
        let ds = s[j] - s[i];
        chain.push(StraightPhase {
            t0,
            dt: 2.0 * ds / (v[i] + v[j]),
            s0: s[i],
            v0: v[i],
            a0: (v[j] * v[j] - v[i] * v[i]) / (2.0 * ds),
            j: 0.0,
        });
        t0 += chain.last().unwrap().dt;
        i = j;
    }
    chain
}

/// Positive root of `c2·dt² + c1·dt − ds = 0` in the cancellation-stable
/// form `2·ds / (c1 + √(c1² + 4·c2·ds))`, which is the unique positive root
/// when `c2 ≥ 0` and the first (physical) arrival when `c2 < 0`.
fn cell_duration(c2: f64, c1: f64, ds: f64) -> Option<f64> {
    let disc = c1 * c1 + 4.0 * c2 * ds;
    if disc < 0.0 {
        return None;
    }
    let dt = 2.0 * ds / (c1 + disc.sqrt());
    (dt.is_finite() && dt > 0.0).then_some(dt)
}

const DISK_RIDE_BISECT_ITERS: u32 = 60;
const DISK_RIDE_DT_SANITY: f64 = 16.0;
/// The brake envelope counts as binding only strictly below the limit curve,
/// so a curve riding the same cap never chatters against it.
const BRAKE_BIND_REL: f64 = 1e-6;
/// Slack for anchoring a cell on a target velocity the law curve must already
/// be near; a larger gap means the run is genuinely infeasible or the
/// integration lost its constraint, and the run is rejected.
const LAND_SLACK_REL: f64 = 1e-3;
/// Cap-riding drifts off the limit curve by the per-cell consistency error;
/// beyond this the integration lost the cap.
const CAP_DRIFT_MAX_REL: f64 = 1e-3;
/// Implied landing accelerations within this of the entry acceleration snap
/// to it exactly, so constant-accel spans stay merge- and exact-lowerable.
const ACCEL_SNAP_MM_S2: f64 = 1e-3;
/// A binding brake envelope may descend past the disk rail by at most this
/// fraction of the accel budget per cell (chord-vs-node discretization of a
/// rail-riding curve). Where the envelope binds it *is* the profile — the
/// forward pass adopts its chain and the lowering consumes its samples as
/// exact states — so a super-disk descent there is not a chord-shadow
/// artifact but an infeasible plan, and rail-clamping its samples would hand
/// the lowering mutually inconsistent `(v, a)` knots (the executed
/// acceleration spikes). Reject the run loudly instead.
const ENVELOPE_BRAKE_SLACK_FRAC: f64 = 2e-2;

/// The infinite-jerk profile as an event-driven integration in run-local
/// arc-length, with three laws: accelerate on the disk rail, track the
/// velocity-limit curve, and brake on the rail toward the exit anchor. A
/// backward rail sweep supplies the brake envelope — at each node the highest
/// speed from which the exit and every downstream cap remain reachable. The
/// envelope is used only to locate events (where the forward curve meets a
/// braking constraint, found by bisection inside the cell); the braking
/// trajectory itself is integrated forward by the brake law and lands exactly
/// on the envelope\'s stretch-end node, so envelope discretization error
/// never enters the emitted chain. Returns `None` when a cell solve
/// degenerates or the track holds an interior rest; the caller falls back to
/// the staircase reconstruction.
fn disk_ride_chain(
    s: &[f64],
    vlc: &[f64],
    accel: &[f64],
    kappa: &[f64],
    entry_v: f64,
    exit_v: f64,
) -> Option<Vec<StraightPhase>> {
    let n = s.len();
    if n < 2 || vlc.iter().any(|&c| c <= VELOCITY_FLOOR) {
        return None;
    }
    let x: Vec<f64> = s.iter().map(|&v| v - s[0]).collect();
    let track = DiskTrack::new(&x, vlc, accel, kappa);

    let mut envelope = vlc.to_vec();
    envelope[n - 1] = envelope[n - 1].min(exit_v);
    for i in (0..n - 1).rev() {
        let ds = x[i + 1] - x[i];
        let budget = disk_rail_accel(accel[i + 1], kappa[i + 1], envelope[i + 1]);
        let reach = (envelope[i + 1] * envelope[i + 1] + 2.0 * budget * ds).sqrt();
        envelope[i] = envelope[i].min(reach);
    }

    let mut chain = integrate_disk(&track, &envelope, entry_v, exit_v)?;
    for p in &mut chain {
        p.s0 += s[0];
    }
    Some(chain)
}

struct DiskTrack<'a> {
    x: &'a [f64],
    vlc: &'a [f64],
    accel: &'a [f64],
    kappa: &'a [f64],
    cap_a: Vec<f64>,
}

impl<'a> DiskTrack<'a> {
    fn new(x: &'a [f64], vlc: &'a [f64], accel: &'a [f64], kappa: &'a [f64]) -> Self {
        let n = x.len();
        let cap_a = (0..n)
            .map(|i| {
                let (lo, hi) = (i.saturating_sub(1), (i + 1).min(n - 1));
                let span = x[hi] - x[lo];
                if span > 1e-12 {
                    (vlc[hi] * vlc[hi] - vlc[lo] * vlc[lo]) / (2.0 * span)
                } else {
                    0.0
                }
            })
            .collect();
        DiskTrack {
            x,
            vlc,
            accel,
            kappa,
            cap_a,
        }
    }

    fn rail(&self, i: usize, v: f64) -> f64 {
        disk_rail_accel(self.accel[i], self.kappa[i], v)
    }

    /// The cap slope of the cell ahead of node `k` (behind it at the track
    /// end). Adopting the *centered* slope when landing on a node would read
    /// half of whatever discontinuity sits behind the node — a feedrate step
    /// between moves injects tens of thousands of mm/s² that the cap on the
    /// far side never asked for.
    fn cap_a_forward(&self, k: usize) -> f64 {
        let n = self.x.len();
        let (lo, hi) = if k + 1 < n { (k, k + 1) } else { (k - 1, k) };
        let span = self.x[hi] - self.x[lo];
        if span <= 1e-12 {
            return 0.0;
        }
        (self.vlc[hi] * self.vlc[hi] - self.vlc[lo] * self.vlc[lo]) / (2.0 * span)
    }

    /// A nodal curve linearly interpolated inside cell `k`.
    fn interp(&self, nodal: &[f64], k: usize, x: f64) -> f64 {
        let span = self.x[k + 1] - self.x[k];
        if span <= 1e-12 {
            return nodal[k + 1];
        }
        let f = ((x - self.x[k]) / span).clamp(0.0, 1.0);
        nodal[k] + f * (nodal[k + 1] - nodal[k])
    }
}

/// `(v, a)` of the constant-jerk chain at arc `x`.
fn chain_state_at(chain: &[StraightPhase], x: f64) -> Option<(f64, f64)> {
    let idx = chain
        .partition_point(|p| p.s0 <= x + POS_INTERIOR_EPS)
        .saturating_sub(1);
    let p = chain.get(idx)?;
    let st = ride::state::State {
        t: 0.0,
        s: 0.0,
        v: p.v0,
        a: p.a0,
    };
    let tau = ride::state::time_to_cross(st, p.j, (x - p.s0).max(0.0))?.min(p.dt);
    let e = ride::state::advance(st, p.j, tau);
    Some((e.v, e.a))
}

const POS_INTERIOR_EPS: f64 = 1e-12;

/// One constant-jerk cell of the integration: from `(v0, a0)` it reaches
/// `ds` ahead with the end acceleration pinned at `a_end`; velocity follows.
fn cell_toward_accel(v0: f64, a0: f64, a_end: f64, ds: f64) -> Option<(f64, f64)> {
    let dt = cell_duration((2.0 * a0 + a_end) / 6.0, v0, ds)?;
    Some((dt, v0 + 0.5 * dt * (a0 + a_end)))
}

/// One constant-jerk cell with the end *velocity* pinned instead: the exact
/// landing used at anchors, where the implied end acceleration absorbs the
/// residual.
fn cell_toward_velocity(v0: f64, a0: f64, v_end: f64, ds: f64) -> Option<(f64, f64)> {
    let dv = v_end - v0;
    let dt = cell_duration(a0 / 6.0, v0 + dv / 3.0, ds)?;
    Some((dt, 2.0 * dv / dt - a0))
}

#[derive(Clone, Copy)]
enum Law {
    Rail,
    Cap,
    Brake { until: usize },
}

fn integrate_disk(
    track: &DiskTrack,
    envelope: &[f64],
    entry_v: f64,
    exit_v: f64,
) -> Option<Vec<StraightPhase>> {
    let n = track.x.len();
    let binding: Vec<bool> = (0..n)
        .map(|i| envelope[i] < track.vlc[i] - BRAKE_BIND_REL * (1.0 + track.vlc[i]))
        .collect();
    let stretch_end = |from: usize| -> usize {
        let mut m = from;
        while m < n - 1 && binding[m] {
            m += 1;
        }
        m
    };

    let mut chain: Vec<StraightPhase> = Vec::new();
    let mut t = 0.0;
    let mut x_cur = track.x[0];
    let mut k = 0usize;
    let mut v = entry_v.min(track.vlc[0]);
    if v > envelope[0] + LAND_SLACK_REL * (1.0 + envelope[0]) {
        return None;
    }
    let mut law = if binding[0] && v >= envelope[0] - LAND_SLACK_REL * (1.0 + envelope[0]) {
        Law::Brake {
            until: stretch_end(0),
        }
    } else if v >= track.vlc[0] * (1.0 - CAP_NOTCH_REL) {
        Law::Cap
    } else {
        Law::Rail
    };
    let mut a = match law {
        Law::Cap => track.cap_a[0],
        Law::Rail => track.rail(0, v),
        Law::Brake { .. } => -track.rail(0, v),
    };

    let push = |chain: &mut Vec<StraightPhase>, t: &mut f64, phase: StraightPhase| {
        if phase.dt > 1e-12 {
            *t += phase.dt;
            chain.push(phase);
        }
    };
    let law_reset = |track: &DiskTrack, law: Law, k: usize, v: f64| match law {
        Law::Cap => track.cap_a_forward(k),
        Law::Rail => track.rail(k, v),
        Law::Brake { .. } => -track.rail(k, v),
    };

    let mut law_flips_in_cell = 0u32;
    let mut iterations = 0u64;
    while k < n - 1 {
        iterations += 1;
        if iterations > 20 * n as u64 {
            debug_assert!(
                false,
                "disk_ride livelock: k={k} n={n} x_cur={x_cur} v={v} a={a}"
            );
        }
        let ds = track.x[k + 1] - x_cur;
        if ds <= GRID_DEDUP_MM {
            k += 1;
            law_flips_in_cell = 0;
            continue;
        }
        if let Law::Cap = law {
            if (v - track.interp(track.vlc, k, x_cur)).abs() > CAP_DRIFT_MAX_REL * (1.0 + v) {
                return None;
            }
        }

        // Anchored cells land their exact target velocity: a brake stretch on
        // its end node (the cap value there — envelope discretization error
        // does not enter), and the final cell on the exit anchor.
        let last_cell = k + 1 == n - 1;
        let brake_lands = matches!(law, Law::Brake { until } if until == k + 1);
        if brake_lands || last_cell {
            let v_land = if last_cell {
                exit_v.min(track.vlc[n - 1])
            } else {
                envelope[k + 1].min(track.vlc[k + 1])
            };
            if !matches!(law, Law::Brake { .. })
                && (v_land - v).abs()
                    > LAND_SLACK_REL * (1.0 + v_land) + 2.0 * a.abs() * ds / v.max(VELOCITY_FLOOR)
            {
                return None;
            }
            let (dt, a_end) = cell_toward_velocity(v, a, v_land, ds)?;
            // A physically-zero implied residual would still block the
            // zero-jerk merge (and the exact closed-form straight lowering);
            // snapping it moves the landed velocity by well under the seam
            // continuity tolerance.
            let a_end = if (a_end - a).abs() <= ACCEL_SNAP_MM_S2 {
                a
            } else {
                a_end
            };
            let dt_est = 2.0 * ds / (v + v_land).max(VELOCITY_FLOOR);
            if !(dt < DISK_RIDE_DT_SANITY * dt_est && dt * DISK_RIDE_DT_SANITY > dt_est) {
                return None;
            }
            let landing = StraightPhase {
                t0: t,
                dt,
                s0: x_cur,
                v0: v,
                a0: a,
                j: (a_end - a) / dt,
            };
            push(&mut chain, &mut t, landing);
            x_cur = track.x[k + 1];
            k += 1;
            law_flips_in_cell = 0;
            v = v_land;
            if !last_cell {
                law = if v >= track.vlc[k] * (1.0 - CAP_NOTCH_REL) {
                    Law::Cap
                } else {
                    Law::Rail
                };
                a = law_reset(track, law, k, v);
            }
            continue;
        }

        let cap_target = |v: f64, a: f64| {
            let dt_est = 2.0 * ds / (v + track.vlc[k + 1]).max(VELOCITY_FLOOR);
            let v_pred = v + 0.5 * dt_est * (a + track.cap_a[k + 1]);
            track.cap_a[k + 1] + (track.vlc[k + 1] - v_pred) / dt_est
        };
        if matches!(law, Law::Cap) && cap_target(v, a) > track.rail(k + 1, track.vlc[k + 1]) {
            // The cap (with its catch-up correction) outruns the disk: fall
            // behind onto the rail rather than accumulate drift chasing it.
            law = Law::Rail;
            law_flips_in_cell += 1;
            if law_flips_in_cell > 2 {
                return None;
            }
            continue;
        }
        let a_tgt = match law {
            // Track the cap slope with half-gain velocity feedback onto the
            // node cap value: the open-loop trapezoid drifts off a strongly
            // curved cap by O(v''·ds²) per cell, which compounds across a
            // short blend; half the node error per cell keeps the drift
            // bounded without the oscillation a full (reflecting) correction
            // would ring with.
            // A descending cap steeper than the rail is not followable —
            // the envelope brake owns that stretch and its trigger fires a
            // node later; clamping keeps the boundary cells on the disk
            // instead of letting the stiff catch-up correction ring.
            Law::Cap => cap_target(v, a).max(-track.rail(k + 1, v)),
            Law::Rail => {
                let v_pred = (v * v + 2.0 * a.max(0.0) * ds).max(0.0).sqrt();
                track.rail(k + 1, v_pred)
            }
            Law::Brake { .. } => {
                // Brake with half-gain velocity feedback onto the envelope:
                // within a binding stretch the envelope is the rail-brake
                // curve itself, and open-loop integration drifting a hair off
                // it stalls a cell early when the stretch ends at rest.
                let dt_est = 2.0 * ds / (v + envelope[k + 1]).max(VELOCITY_FLOOR);
                let rail_a = {
                    let v_pred = (v * v + 2.0 * a.min(0.0) * ds).max(0.0).sqrt();
                    -track.rail(k + 1, v_pred)
                };
                let v_pred = v + 0.5 * dt_est * (a + rail_a);
                rail_a + (envelope[k + 1] - v_pred) / dt_est
            }
        };
        let a_tgt = if (a_tgt - a).abs() <= ACCEL_SNAP_MM_S2 {
            a
        } else {
            a_tgt
        };
        let (dt, v1) = cell_toward_accel(v, a, a_tgt, ds)?;
        let a_end = a_tgt;
        let dt_est = 2.0 * ds / (v + v1.max(VELOCITY_FLOOR));
        if !(dt < DISK_RIDE_DT_SANITY * dt_est && dt * DISK_RIDE_DT_SANITY > dt_est) {
            return None;
        }
        let candidate = StraightPhase {
            t0: t,
            dt,
            s0: x_cur,
            v0: v,
            a0: a,
            j: (a_end - a) / dt,
        };

        if !matches!(law, Law::Brake { .. })
            && binding[k + 1]
            && v1 > envelope[k + 1] + VELOCITY_FLOOR
        {
            let until = stretch_end(k + 1);
            // Inside the onset cell the envelope is the backward rail curve
            // through the next node — the same form the sweep recursion uses.
            // Bisecting against a linear chord instead would start the brake
            // up to a chord-sag early and ring the feedback for cells after.
            let env_curve = |xq: f64| {
                let budget =
                    disk_rail_accel(track.accel[k + 1], track.kappa[k + 1], envelope[k + 1]);
                let w = envelope[k + 1] * envelope[k + 1] + 2.0 * budget * (track.x[k + 1] - xq);
                Some(w.max(0.0).sqrt())
            };
            if v >= env_curve(x_cur)? {
                law = Law::Brake { until };
                a = -track.rail(k + 1, v);
                law_flips_in_cell += 1;
                if law_flips_in_cell > 2 {
                    return None;
                }
                continue;
            }
            let (x_star, state) = bisect_crossing(&candidate, x_cur, track.x[k + 1], env_curve)?;
            push(&mut chain, &mut t, truncate_phase(&candidate, x_star));
            x_cur = x_star;
            v = state.0;
            a = -track.rail(k + 1, v);
            law = Law::Brake { until };
            law_flips_in_cell = 0;
            continue;
        }

        if matches!(law, Law::Rail) && v1 > track.vlc[k + 1] && law_flips_in_cell == 0 {
            if v >= track.interp(track.vlc, k, x_cur) {
                law = Law::Cap;
                a = track.cap_a_forward(k);
                law_flips_in_cell += 1;
                if law_flips_in_cell > 2 {
                    return None;
                }
                continue;
            }
            let (x_star, state) = bisect_crossing(&candidate, x_cur, track.x[k + 1], |xq| {
                Some(track.interp(track.vlc, k, xq))
            })?;
            push(&mut chain, &mut t, truncate_phase(&candidate, x_star));
            x_cur = x_star;
            v = state.0;
            a = track.cap_a_forward(k);
            law = Law::Cap;
            law_flips_in_cell = 0;
            continue;
        }

        push(&mut chain, &mut t, candidate);
        x_cur = track.x[k + 1];
        k += 1;
        law_flips_in_cell = 0;
        v = v1;
        a = a_end;
        if v <= VELOCITY_FLOOR && k < n - 1 {
            return None;
        }
    }
    if chain.is_empty() {
        return None;
    }
    Some(merge_constant_accel(chain))
}

/// Arc (and `(v, a)` state) inside `[lo, hi]` where the candidate phase\'s
/// velocity crosses the boundary curve `bound_v`, by bisection. The phase is
/// below the boundary at `lo` and its end state is above at `hi`.
fn bisect_crossing<F: Fn(f64) -> Option<f64>>(
    phase: &StraightPhase,
    lo: f64,
    hi: f64,
    bound_v: F,
) -> Option<(f64, (f64, f64))> {
    let eval = |xq: f64| -> Option<(f64, f64)> { chain_state_at(std::slice::from_ref(phase), xq) };
    let mut lo = lo;
    let mut hi = hi;
    for _ in 0..DISK_RIDE_BISECT_ITERS {
        let mid = 0.5 * (lo + hi);
        let (v, _) = eval(mid)?;
        if v >= bound_v(mid)? {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    let state = eval(hi)?;
    Some((hi, state))
}

/// The candidate phase cut at arc `x_star`.
fn truncate_phase(phase: &StraightPhase, x_star: f64) -> StraightPhase {
    let st = ride::state::State {
        t: 0.0,
        s: 0.0,
        v: phase.v0,
        a: phase.a0,
    };
    let tau = ride::state::time_to_cross(st, phase.j, (x_star - phase.s0).max(0.0))
        .map(|t| t.min(phase.dt))
        .unwrap_or(phase.dt);
    StraightPhase { dt: tau, ..*phase }
}

/// Chain adjacent zero-jerk phases whose accelerations agree into one cubic —
/// a straight rail or cruise integrates to them cell by cell — so a straight
/// move lowers one piece per regime instead of one per node.
///
/// "Agree" cannot mean bit-identical. A cruise the pass reaches by landing
/// tangent on the ceiling holds `a` to within a ten-thousandth of a mm/s² of
/// zero, not at zero, and refusing to chain over that residue costs a piece
/// per cell: the `repro_z14.gcode` line 3322 serpentine lowered its 15.7 mm
/// straights to 2285 phases each, all of them zero-jerk with `a0` inside
/// [0, 4.4e-4] mm/s².
///
/// The merged phase is solved for `(a0, j)` that reproduce the run's true end
/// arc *and* end velocity, so it is exact at both ends and no seam moves.
/// What it approximates is the interior, by at most the acceleration spread
/// over the span — bounded below [`CHAIN_MERGE_ARC_MM`] of arc, which is a
/// fiftieth of the lowering's own position budget.
fn merge_constant_accel(chain: Vec<StraightPhase>) -> Vec<StraightPhase> {
    let mut out: Vec<StraightPhase> = Vec::with_capacity(chain.len());
    let mut open: Option<(f64, f64)> = None;
    for p in chain {
        if p.j != 0.0 {
            open = None;
            out.push(p);
            continue;
        }
        let Some((lo, hi)) = open else {
            open = Some((p.a0, p.a0));
            out.push(p);
            continue;
        };
        let last = out.last_mut().expect("an open run has a phase");
        if last.a0 == p.a0 && last.j == 0.0 {
            last.dt += p.dt;
            continue;
        }
        let (lo, hi) = (lo.min(p.a0), hi.max(p.a0));
        match chain_phases(last, &p, hi - lo) {
            Some(one) => {
                *last = one;
                open = Some((lo, hi));
            }
            None => {
                open = Some((p.a0, p.a0));
                out.push(p);
            }
        }
    }
    out
}

/// One constant-jerk phase covering `open` and `next`, landing on `next`'s
/// true end arc and end velocity — exact at both ends, so no seam moves and
/// only the interior is approximated. `None` when the run's acceleration
/// `spread` would bend that interior further off the true arc than
/// [`CHAIN_MERGE_ARC_MM`].
fn chain_phases(open: &StraightPhase, next: &StraightPhase, spread: f64) -> Option<StraightPhase> {
    let dt = open.dt + next.dt;
    if dt <= 0.0 || spread > CHAIN_MERGE_ACCEL_MM_S2 || spread * dt * dt > 8.0 * CHAIN_MERGE_ARC_MM
    {
        return None;
    }
    let s_end = next.s0 + next.dt * (next.v0 + 0.5 * next.a0 * next.dt);
    let v_end = next.v0 + next.a0 * next.dt;
    let ds = s_end - open.s0 - open.v0 * dt;
    let dv = v_end - open.v0;
    let merged = StraightPhase {
        dt,
        a0: (6.0 * ds - 2.0 * dv * dt) / (dt * dt),
        j: 6.0 * (dv * dt - 2.0 * ds) / (dt * dt * dt),
        ..*open
    };
    (merged.a0.is_finite() && merged.j.is_finite()).then_some(merged)
}

/// Reconstruct the run's `(s, v, a)` profile and its phase chain, on a grid
/// the reconstruction itself has to justify.
///
/// The velocity-limit curve (feed and `sqrt(accel / kappa)`) is sampled on the
/// grid; the event-driven pass in [`ride`] integrates the brake envelope
/// backward from the exit anchor and the profile forward against
/// `min(vlc, brake)`, in time, as bang-bang constant-jerk phases with tangent
/// landings. The emitted `a` is the pass's true acceleration state. Every step
/// the pass takes is a constant-jerk cubic — curved substeps and cap-chord
/// rides included — so the phase chain is normally complete for any run and
/// the samples are re-derived from it exactly; only a stall or a rejected
/// splice leaves the chain empty, falling back to the node states.
///
/// A straight member's cap is its own constant ceiling under a brake envelope
/// that only descends toward what follows, so the fastest feasible profile on
/// it rises, holds and falls — its acceleration changes sign at most twice.
/// A grid too coarse for the pass to settle onto that ceiling instead makes
/// it hunt: the profile touches the cap, peels, jerks back up and touches
/// again, a limit cycle with one reversal per ~1.5 mm. `repro_z14.gcode`
/// line 3322 (15.7 mm straights cruising at F3600 between blends) rings 15
/// times across each at the seeded 256 nodes and once at [`GRID_STEP_MM`],
/// costing 1.9 mm/s of peak-to-peak velocity and ±270 mm/s² of acceleration
/// the plan never asked for, at 37 Hz.
///
/// A count over the bound is a suspicion, not a verdict: the pass has
/// structure the sign count cannot model — a splice joint, a stall, an entry
/// acceleration it has to shed — and those reversals are there at every
/// spacing. So the member is regridded and re-counted, and only a count that
/// *drops* convicts the grid. One that does not is the physics, and the
/// coarse grid, which said the same thing for a sixth of the nodes, is
/// restored. Nothing is spent that the reconstruction did not pay for.
fn reconstruct_flat(
    members: &[RunMember],
    run_start_v: f64,
    run_start_a: f64,
) -> Result<(Vec<(f64, f64, f64)>, Vec<StraightPhase>), ReconstructError> {
    let entry_rest = run_start_v <= VELOCITY_FLOOR;
    let exit_rest = members[members.len() - 1].exit_v <= VELOCITY_FLOOR;
    let mut steps: Vec<usize> = members.iter().map(|m| member_seed_steps(m.kin)).collect();
    let mut settled: Vec<bool> = members.iter().map(|m| !m.kin.is_straight()).collect();
    let mut coarser: Vec<Option<(usize, usize)>> = vec![None; members.len()];
    loop {
        let s = grid_from_steps(members, &steps, entry_rest, exit_rest);
        let out = reconstruct_flat_on(members, &s, run_start_v, run_start_a).ok_or(Diverged)?;
        let mut regridded = false;
        for (idx, m) in members.iter().enumerate() {
            if settled[idx] {
                continue;
            }
            let reversals = accel_reversals(&out.0, m.fwd_s, m.fwd_s + m.kin.length);
            if reversals <= PROFILE_REVERSALS_MAX {
                settled[idx] = true;
                continue;
            }
            if let Some((steps_before, before)) = coarser[idx] {
                if reversals >= before {
                    steps[idx] = steps_before;
                    settled[idx] = true;
                    regridded = true;
                    continue;
                }
            }
            let finer = refine_step_count(m.kin, steps[idx]);
            if finer == steps[idx] {
                return Err(ReconstructError::GridBudget {
                    nodes: s.len(),
                    reversals,
                    member: idx,
                });
            }
            coarser[idx] = Some((steps[idx], reversals));
            steps[idx] = finer;
            regridded = true;
        }
        if !regridded {
            return Ok(out);
        }
    }
}

/// How many times the reconstructed acceleration changes sign between `s0`
/// and `s1`, counting only excursions the lowering would actually execute.
///
/// Samples straddling a zero crossing are both near zero, so the gate cannot
/// sit on the crossing: it sits on the *excursion* each same-sign run
/// reaches. A run peaking under the fit budget is noise the pieces absorb,
/// and dropping it lets the runs on either side count as one.
fn accel_reversals(samples: &[(f64, f64, f64)], s0: f64, s1: f64) -> usize {
    let mut runs: Vec<(f64, f64)> = Vec::new();
    for p in samples
        .iter()
        .filter(|p| p.0 >= s0 - GRID_DEDUP_MM && p.0 <= s1 + GRID_DEDUP_MM)
    {
        if p.2 == 0.0 {
            continue;
        }
        let sign = p.2.signum();
        match runs.last_mut() {
            Some(last) if last.0 == sign => last.1 = last.1.max(p.2.abs()),
            _ => runs.push((sign, p.2.abs())),
        }
    }
    let executed: Vec<f64> = runs
        .iter()
        .filter(|r| r.1 > GRID_ACCEL_TOL_MM_S2)
        .map(|r| r.0)
        .collect();
    executed.windows(2).filter(|w| w[0] != w[1]).count()
}

/// The next spacing to try for a ringing member: the grid pitch it was
/// widened away from, then halvings of that. Returns the input unchanged once
/// the ceiling is reached — four times the pitch is a grid the pass has
/// nothing left to resolve, so a member still hunting there is not
/// under-sampled, it is a bug in the pass.
fn refine_step_count(kin: &Kinematics, steps: usize) -> usize {
    let pitch = (kin.length / GRID_STEP_MM).ceil() as usize;
    let ceiling = pitch.saturating_mul(GRID_REFINE_GROWTH);
    if steps < pitch {
        pitch
    } else {
        steps.saturating_mul(2).min(ceiling).max(steps)
    }
}

fn reconstruct_flat_on(
    members: &[RunMember],
    s: &[f64],
    run_start_v: f64,
    run_start_a: f64,
) -> Option<(Vec<(f64, f64, f64)>, Vec<StraightPhase>)> {
    let entry_v = run_start_v;
    let exit_v = members[members.len() - 1].exit_v;
    let n = s.len();
    let (vlc, accel, kappa) = constraint_arrays(members, s);

    let jerk = members
        .iter()
        .map(|m| m.kin.jerk)
        .fold(f64::INFINITY, f64::min);
    if !jerk.is_finite() {
        return Some(infinite_jerk_profile(
            s, &vlc, &accel, &kappa, entry_v, exit_v,
        ));
    }
    let (bwd_v, brake_chain, bwd_feasible) = {
        let s_rev: Vec<f64> = (0..n).map(|k| s[n - 1] - s[n - 1 - k]).collect();
        let rev = |a: &[f64]| -> Vec<f64> { (0..n).map(|k| a[n - 1 - k]).collect() };
        let (vlc_rev, accel_rev, kappa_rev) = (rev(&vlc), rev(&accel), rev(&kappa));
        let cap_a_rev = chord_slopes(&s_rev, &vlc_rev);
        let track = ride::Track {
            s: &s_rev,
            cap_v: &vlc_rev,
            cap_a: &cap_a_rev,
            accel: &accel_rev,
            kappa: &kappa_rev,
            j_max: jerk,
        };
        let pass = ride::reach_pass(&track, exit_v, 0.0, None);
        let chain = if pass.complete {
            ride::reverse_chain(&pass.phases, s[n - 1] - s[0])
        } else {
            Vec::new()
        };
        let feasible: Vec<bool> = (0..n).map(|k| pass.feasible[n - 1 - k]).collect();
        (rev(&pass.v), chain, feasible)
    };

    let cap_v: Vec<f64> = (0..n).map(|i| vlc[i].min(bwd_v[i])).collect();
    let binding: Vec<bool> = (0..n)
        .map(|i| bwd_v[i] <= vlc[i] && bwd_feasible[i])
        .collect();
    for i in 0..n - 1 {
        if !(binding[i] && binding[i + 1]) {
            continue;
        }
        let ds = s[i + 1] - s[i];
        if ds <= GRID_DEDUP_MM {
            continue;
        }
        let descent = (bwd_v[i] * bwd_v[i] - bwd_v[i + 1] * bwd_v[i + 1]) / (2.0 * ds);
        let rail = disk_rail_accel(accel[i], kappa[i], bwd_v[i]).max(disk_rail_accel(
            accel[i + 1],
            kappa[i + 1],
            bwd_v[i + 1],
        ));
        if descent > rail + ENVELOPE_BRAKE_SLACK_FRAC * accel[i] {
            return None;
        }
    }
    let cap_a = chord_slopes(s, &cap_v);
    let track = ride::Track {
        s,
        cap_v: &cap_v,
        cap_a: &cap_a,
        accel: &accel,
        kappa: &kappa,
        j_max: jerk,
    };
    let brake = ride::BrakeChain {
        phases: &brake_chain,
        binding: &binding,
    };
    let start_v = entry_v.min(cap_v[0]);
    let mut pass = ride::reach_pass(&track, start_v, run_start_a, Some(&brake));
    if pass.complete && !pass.phases.is_empty() {
        for (i, (v, a)) in ride::chain_states(&pass.phases, s).into_iter().enumerate() {
            pass.v[i] = v.min(cap_v[i]);
            let rail = disk_rail_accel(accel[i], kappa[i], pass.v[i]);
            pass.a[i] = a.clamp(-rail, rail);
        }
    }

    pass.v[0] = start_v;
    pass.v[n - 1] = exit_v.min(cap_v[n - 1]);
    if entry_v <= VELOCITY_FLOOR {
        pass.a[0] = 0.0;
    }
    if exit_v <= VELOCITY_FLOOR {
        pass.a[n - 1] = 0.0;
    }
    let samples = (0..n).map(|i| (s[i], pass.v[i], pass.a[i])).collect();
    let phases = if pass.complete {
        pass.phases
    } else {
        Vec::new()
    };
    Some((samples, phases))
}

/// Linearly interpolate the reconstructed flat profile at run-arc `s`. Member
/// boundaries read this single smoothed profile so a roll-off straddling a
/// collinear junction is sampled consistently on both sides of the seam.
fn interp_flat(flat: &[(f64, f64, f64)], s: f64) -> Option<(f64, f64)> {
    let first = flat.first()?;
    let last = flat.last()?;
    if s <= first.0 {
        return Some((first.1, first.2));
    }
    if s >= last.0 {
        return Some((last.1, last.2));
    }
    let i = flat.partition_point(|p| p.0 < s);
    let (lo, hi) = (flat[i - 1], flat[i]);
    let span = hi.0 - lo.0;
    let t = if span > 1e-12 { (s - lo.0) / span } else { 0.0 };
    Some((lo.1 + t * (hi.1 - lo.1), lo.2 + t * (hi.2 - lo.2)))
}

/// Reconstruct the run: per-member `(s, v, a)` samples, the profile state
/// `(v, a)` at each member's exit seam (what a streaming cut carries into the
/// next window to continue this exact curve), and per-member closed-form jerk
/// phases in move-local time/arc-length. Straight members get their clip of
/// the run's chain for the exact-cubic lowering. Curved members get theirs
/// only under unlimited jerk, where the chain is a handful of merged
/// constant-acceleration spans: the lowering fits axis positions against that
/// exact scalar profile instead of quintic windows over stepped samples. A
/// finite-jerk curved chain is left out — its per-substep phases would say
/// nothing the smooth samples don't already.
#[allow(clippy::type_complexity)]
pub(super) fn reconstruct_run(
    members: &[RunMember],
    run_start_v: f64,
    run_start_a: f64,
    _tol: f64,
) -> Result<
    (
        Vec<Vec<(f64, f64, f64)>>,
        Vec<(f64, f64)>,
        Vec<Vec<StraightPhase>>,
    ),
    ReconstructError,
> {
    let (flat, chain) = reconstruct_flat(members, run_start_v, run_start_a)?;

    let mut per_member: Vec<Vec<(f64, f64, f64)>> = Vec::with_capacity(members.len());
    let mut exit_states: Vec<(f64, f64)> = Vec::with_capacity(members.len());
    let mut per_member_phases: Vec<Vec<StraightPhase>> = Vec::with_capacity(members.len());
    for m in members {
        let s0 = m.fwd_s;
        let s1 = m.fwd_s + m.kin.length;
        let lo = flat.partition_point(|p| p.0 < s0 - 1e-9);
        let hi = flat.partition_point(|p| p.0 <= s1 + 1e-9);
        let mut local: Vec<(f64, f64, f64)> = flat[lo..hi]
            .iter()
            .map(|p| ((p.0 - s0).clamp(0.0, m.kin.length), p.1, p.2))
            .collect();
        local.dedup_by(|a, b| (a.0 - b.0).abs() <= GRID_DEDUP_MM);
        if local.first().is_none_or(|p| p.0 > 1e-9) {
            let (v, a) = interp_flat(&flat, s0).ok_or(Diverged)?;
            local.insert(0, (0.0, v, a));
        }
        if local
            .last()
            .is_none_or(|p| (p.0 - m.kin.length).abs() > 1e-9)
        {
            let (v, a) = interp_flat(&flat, s1).ok_or(Diverged)?;
            local.push((m.kin.length, v, a));
        }
        if let Some(first) = local.first_mut() {
            first.0 = 0.0;
        }
        if let Some(last) = local.last_mut() {
            last.0 = m.kin.length;
        }
        per_member.push(local);
        exit_states.push(exit_state_at(&flat, s1));
        let phases_apply = m.kin.is_straight() || !m.kin.jerk.is_finite();
        per_member_phases.push(if chain.is_empty() || !phases_apply {
            Vec::new()
        } else {
            let clipped = merge_constant_accel(ride::clip_phases(&chain, s0, s1));
            if ride::chain_is_continuous(&clipped, m.kin.jerk.is_finite()) {
                clipped
            } else {
                Vec::new()
            }
        });
    }
    Ok((per_member, exit_states, per_member_phases))
}

/// Profile state at run-arc `s`: the emitted samples carry the pass's true
/// acceleration state, and the grid contains every member boundary exactly.
fn exit_state_at(flat: &[(f64, f64, f64)], s: f64) -> (f64, f64) {
    let i = flat.partition_point(|p| p.0 < s - 1e-9);
    let i = i.min(flat.len() - 1);
    (flat[i].1, flat[i].2)
}

#[cfg(test)]
pub(super) fn sample_profile(
    kin: &Kinematics,
    entry: f64,
    exit: f64,
    tol: f64,
) -> Option<Vec<(f64, f64, f64)>> {
    let member = RunMember {
        kin,
        exit_v: exit,
        fwd_s: 0.0,
    };
    reconstruct_run(&[member], entry, 0.0, tol)
        .ok()?
        .0
        .into_iter()
        .next()
}

#[cfg(test)]
mod tests;
