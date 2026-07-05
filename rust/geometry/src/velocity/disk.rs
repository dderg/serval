use std::f64::consts::FRAC_PI_2;

use super::profile::StraightPhase;
use super::ride;

const RK_MIN_STEP_FRAC: f64 = 1e-6;
const RK_MAX_STEPS: u32 = 100_000;
const SAMPLE_MAX_POINTS: usize = 16_384;
const GRID_STEP_MM: f64 = 0.01;
const GRID_MIN_STEPS: usize = 256;
const REST_REFINE_MIN_MM: f64 = 1e-5;
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
    let arg = 2.0 * kappa_abs * length + nurbs::det::asin(x0);
    if arg >= FRAC_PI_2 {
        w_lim
    } else {
        w_lim * nurbs::det::sin(arg)
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
/// locked prefix must be invariant under append). A rest anchor gets a
/// geometric ladder of extra nodes: the jerk ramp from rest covers
/// `a³/(6j²)` of arc (a cell or two) over many milliseconds, and samples
/// uniform in arc-length would leave the lowering nothing to fit the ramp
/// with. Both refinements depend only on the run's own anchors, so the
/// locked-prefix grids stay append-invariant.
fn integration_grid(members: &[RunMember], entry_rest: bool, exit_rest: bool) -> Vec<f64> {
    let mut s: Vec<f64> = Vec::new();
    for m in members {
        let len = m.kin.length;
        let steps = ((len / GRID_STEP_MM).ceil() as usize).clamp(GRID_MIN_STEPS, SAMPLE_MAX_POINTS);
        for k in 0..steps {
            s.push(m.fwd_s + len * (k as f64) / (steps as f64));
        }
    }
    let start = members[0].fwd_s;
    let last = &members[members.len() - 1];
    let end = last.fwd_s + last.kin.length;
    let total = end - start;
    let mut delta = REST_REFINE_MIN_MM;
    while delta < GRID_STEP_MM.min(0.5 * total) {
        if entry_rest {
            s.push(start + delta);
        }
        if exit_rest {
            s.push(end - delta);
        }
        delta *= 2.0;
    }
    s.push(end);
    s.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    s.dedup_by(|a, b| (*a - *b).abs() <= 1e-12);
    s
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

/// Speed ceiling at run-arc `s_run`, the min over every member whose span covers
/// it. At a curvature-discontinuous seam two members abut, and the ceiling must
/// honor the tighter side — otherwise the dip's own endpoint (read by the member
/// that ends there) could overshoot its curvature limit.
fn vlc_at(members: &[RunMember], s_run: f64) -> f64 {
    members_at(members, s_run)
        .map(|(m, local)| {
            let kappa_abs = m.kin.kappa_abs(local);
            m.kin.flat_ceiling.min(limit_speed(kappa_abs, m.kin.accel))
        })
        .fold(f64::INFINITY, f64::min)
}

/// Acceleration budget at `s_run`, tightest across abutting members.
fn accel_cap(members: &[RunMember], s_run: f64) -> f64 {
    members_at(members, s_run)
        .map(|(m, _)| m.kin.accel)
        .fold(f64::INFINITY, f64::min)
}

fn kappa_abs_at(members: &[RunMember], s_run: f64) -> f64 {
    members_at(members, s_run)
        .map(|(m, local)| m.kin.kappa_abs(local))
        .fold(0.0, f64::max)
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

/// Reconstruct under infinite jerk: the acceleration-disk boundary itself is
/// the answer (nothing to ramp). Forward–backward disk passes over the
/// velocity-limit curve give the grid velocities; between grid points the
/// motion is constant-acceleration (`v²` linear in `s`), so the profile is
/// exactly a chain of zero-jerk phases whose boundaries — the trapezoid
/// corners — land on grid knots. Samples are re-derived from that chain, and
/// straight members lower each phase to an exact quadratic instead of fitting
/// cubics across an acceleration step.
fn infinite_jerk_profile(
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

    let entry_rest = entry_v <= VELOCITY_FLOOR;
    let exit_rest = exit_v <= VELOCITY_FLOOR;
    let chain = infinite_jerk_chain(s, &ceil);
    let mut samples: Vec<(f64, f64, f64)> = if chain.is_empty() {
        (0..n)
            .map(|i| {
                let (lo, hi) = (i.saturating_sub(1), (i + 1).min(n - 1));
                let dvds = (ceil[hi] - ceil[lo]) / (s[hi] - s[lo]).max(1e-12);
                let rail = disk_rail_accel(accel[i], kappa[i], ceil[i]);
                (s[i], ceil[i], (ceil[i] * dvds).clamp(-rail, rail))
            })
            .collect()
    } else {
        ride::chain_states(&chain, s)
            .into_iter()
            .zip(s)
            .map(|((v, a), &x)| (x, v, a))
            .collect()
    };
    samples[0].1 = entry_v;
    samples[n - 1].1 = exit_v;
    if entry_rest {
        samples[0].2 = 0.0;
    }
    if exit_rest {
        samples[n - 1].2 = 0.0;
    }
    (samples, chain)
}

/// The disk profile's `v²` slope is constant within a grid cell and — on rail
/// and cruise stretches — bit-identical across cells, so maximal equal-slope
/// spans merge into single constant-acceleration phases. A trapezoid becomes
/// rail / transition-cell / cruise / transition-cell / rail rather than one
/// phase per 0.01mm grid cell.
const INFINITE_JERK_SLOPE_MERGE_REL_TOL: f64 = 1e-6;

fn infinite_jerk_chain(s: &[f64], v: &[f64]) -> Vec<StraightPhase> {
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
        let a_span = (v[j] * v[j] - v[i] * v[i]) / (2.0 * ds);
        let dt = 2.0 * ds / (v[i] + v[j]);
        chain.push(StraightPhase {
            t0,
            dt,
            s0: s[i],
            v0: v[i],
            a0: a_span,
            j: 0.0,
        });
        t0 += dt;
        i = j;
    }
    chain
}

/// Reconstruct the run's `(s, v, a)` profile and its phase chain.
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
fn reconstruct_flat(
    members: &[RunMember],
    run_start_v: f64,
    run_start_a: f64,
) -> (Vec<(f64, f64, f64)>, Vec<StraightPhase>) {
    let entry_v = run_start_v;
    let exit_v = members[members.len() - 1].exit_v;
    let s = integration_grid(members, entry_v <= VELOCITY_FLOOR, exit_v <= VELOCITY_FLOOR);
    let n = s.len();
    let vlc: Vec<f64> = s.iter().map(|&x| vlc_at(members, x)).collect();
    let accel: Vec<f64> = s.iter().map(|&x| accel_cap(members, x)).collect();
    let kappa: Vec<f64> = s.iter().map(|&x| kappa_abs_at(members, x)).collect();

    let jerk = members
        .iter()
        .map(|m| m.kin.jerk)
        .fold(f64::INFINITY, f64::min);
    if !jerk.is_finite() {
        return infinite_jerk_profile(&s, &vlc, &accel, &kappa, entry_v, exit_v);
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
    let cap_a = chord_slopes(&s, &cap_v);
    let track = ride::Track {
        s: &s,
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
        for (i, (v, a)) in ride::chain_states(&pass.phases, &s).into_iter().enumerate() {
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
    (samples, phases)
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
/// the run's chain — the exact-cubic lowering assumes each axis is a constant
/// scale of arc-length, so curved members get none and lower by fitting.
#[allow(clippy::type_complexity)]
pub(super) fn reconstruct_run(
    members: &[RunMember],
    run_start_v: f64,
    run_start_a: f64,
    _tol: f64,
) -> Option<(
    Vec<Vec<(f64, f64, f64)>>,
    Vec<(f64, f64)>,
    Vec<Vec<StraightPhase>>,
)> {
    let (flat, chain) = reconstruct_flat(members, run_start_v, run_start_a);

    let mut per_member: Vec<Vec<(f64, f64, f64)>> = Vec::with_capacity(members.len());
    let mut exit_states: Vec<(f64, f64)> = Vec::with_capacity(members.len());
    let mut per_member_phases: Vec<Vec<StraightPhase>> = Vec::with_capacity(members.len());
    for m in members {
        let s0 = m.fwd_s;
        let s1 = m.fwd_s + m.kin.length;
        let mut local: Vec<(f64, f64, f64)> = flat
            .iter()
            .filter(|p| p.0 >= s0 - 1e-9 && p.0 <= s1 + 1e-9)
            .map(|p| ((p.0 - s0).clamp(0.0, m.kin.length), p.1, p.2))
            .collect();
        local.dedup_by(|a, b| (a.0 - b.0).abs() < 1e-12);
        if local.first().is_none_or(|p| p.0 > 1e-9) {
            let (v, a) = interp_flat(&flat, s0)?;
            local.insert(0, (0.0, v, a));
        }
        if local
            .last()
            .is_none_or(|p| (p.0 - m.kin.length).abs() > 1e-9)
        {
            let (v, a) = interp_flat(&flat, s1)?;
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
        per_member_phases.push(if chain.is_empty() || !m.kin.is_straight() {
            Vec::new()
        } else {
            let clipped = ride::clip_phases(&chain, s0, s1);
            if ride::chain_is_continuous(&clipped, m.kin.jerk.is_finite()) {
                clipped
            } else {
                Vec::new()
            }
        });
    }
    Some((per_member, exit_states, per_member_phases))
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
    reconstruct_run(&[member], entry, 0.0, tol)?
        .0
        .into_iter()
        .next()
}

#[cfg(test)]
mod tests;
