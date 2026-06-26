use std::f64::consts::FRAC_PI_2;

use super::scurve;

const RK_MIN_STEP_FRAC: f64 = 1e-6;
const RK_MAX_STEPS: u32 = 100_000;
const SAMPLE_MAX_DEPTH: u32 = 24;
const SAMPLE_MAX_POINTS: usize = 16_384;
const SAMPLE_MIN_STEP_DIV: f64 = 4096.0;
const GRID_STEP_MM: f64 = 0.01;
const GRID_MIN_STEPS: usize = 256;
const VELOCITY_FLOOR: f64 = 1e-9;

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
    let arg = 2.0 * kappa_abs * length + x0.asin();
    if arg >= FRAC_PI_2 {
        w_lim
    } else {
        w_lim * arg.sin()
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

pub(super) struct JerkAnchors {
    pub fwd_v: f64,
    pub fwd_s: f64,
    pub bwd_v: f64,
    pub bwd_s: f64,
}

#[derive(Clone, Copy)]
pub(super) struct ProfilePoint {
    pub v: f64,
    pub a: f64,
}

fn disk_rail_accel(accel: f64, kappa_abs: f64, v: f64) -> f64 {
    let a_n = kappa_abs * v * v;
    (accel * accel - a_n * a_n).max(0.0).sqrt()
}

fn clamp_to_disk(a_t: f64, accel: f64, kappa_abs: f64, v: f64) -> f64 {
    let budget = disk_rail_accel(accel, kappa_abs, v);
    a_t.clamp(-budget, budget)
}

fn forward_seg(kin: &Kinematics, jerk: &JerkAnchors) -> Option<scurve::SevenSeg> {
    scurve::breakpoints(
        jerk.fwd_v,
        0.0,
        jerk.fwd_s + kin.length,
        kin.accel,
        kin.jerk,
    )
    .ok()
}

fn backward_seg(kin: &Kinematics, jerk: &JerkAnchors) -> Option<scurve::SevenSeg> {
    scurve::breakpoints(
        jerk.bwd_v,
        0.0,
        jerk.bwd_s + kin.length,
        kin.accel,
        kin.jerk,
    )
    .ok()
}

const KAPPA_EPS: f64 = 1e-9;

fn curvature_ceiling_accel(kin: &Kinematics, s: f64) -> f64 {
    let kappa = kin.kappa0 + kin.sigma * s;
    let kappa_abs = kappa.abs();
    if kappa_abs <= KAPPA_EPS {
        return 0.0;
    }
    let a = -0.5 * kin.accel * kin.sigma * kappa.signum() / (kappa_abs * kappa_abs);
    a.clamp(-kin.accel, kin.accel)
}

fn ceiling_accel(kin: &Kinematics, s: f64) -> f64 {
    let v_curv = limit_speed(kin.kappa_abs(s), kin.accel);
    if v_curv <= kin.flat_ceiling {
        curvature_ceiling_accel(kin, s)
    } else {
        0.0
    }
}

fn forward_branch(
    kin: &Kinematics,
    entry: f64,
    seg: &scurve::SevenSeg,
    jerk: &JerkAnchors,
    s: f64,
    tol: f64,
) -> Option<ProfilePoint> {
    let v_disk = disk_reach_v(kin, entry, s, tol)?;
    let v_jerk = scurve::velocity_at(seg, jerk.fwd_s + s);
    let ceil = kin
        .flat_ceiling
        .min(limit_speed(kin.kappa_abs(s), kin.accel));
    let v = v_disk.min(v_jerk).min(ceil);
    let a = if v_jerk <= v_disk && v_jerk <= ceil {
        scurve::accel_at(seg, jerk.fwd_s + s)
    } else if v >= ceil - tol * (1.0 + ceil) {
        ceiling_accel(kin, s)
    } else {
        disk_rail_accel(kin.accel, kin.kappa_abs(s), v_disk)
    };
    Some(ProfilePoint { v, a })
}

fn backward_branch(
    kin: &Kinematics,
    exit: f64,
    seg: &scurve::SevenSeg,
    jerk: &JerkAnchors,
    s: f64,
    tol: f64,
) -> Option<ProfilePoint> {
    let rest = kin.length - s;
    let v_disk = disk_reach_v(&kin.reversed(), exit, rest, tol)?;
    let v_jerk = scurve::velocity_at(seg, jerk.bwd_s + rest);
    let ceil = kin
        .flat_ceiling
        .min(limit_speed(kin.kappa_abs(s), kin.accel));
    let v = v_disk.min(v_jerk).min(ceil);
    let a = if v_jerk <= v_disk && v_jerk <= ceil {
        -scurve::accel_at(seg, jerk.bwd_s + rest)
    } else if v >= ceil - tol * (1.0 + ceil) {
        ceiling_accel(kin, s)
    } else {
        -disk_rail_accel(kin.accel, kin.kappa_abs(s), v_disk)
    };
    Some(ProfilePoint { v, a })
}

fn eval_profile(
    kin: &Kinematics,
    entry: f64,
    exit: f64,
    fwd: &scurve::SevenSeg,
    bwd: &scurve::SevenSeg,
    jerk: &JerkAnchors,
    s: f64,
    tol: f64,
) -> Option<ProfilePoint> {
    let f = forward_branch(kin, entry, fwd, jerk, s, tol)?;
    let b = backward_branch(kin, exit, bwd, jerk, s, tol)?;
    Some(if f.v <= b.v { f } else { b })
}

fn profile_speed(
    kin: &Kinematics,
    entry: f64,
    exit: f64,
    jerk: &JerkAnchors,
    s: f64,
    tol: f64,
) -> Option<f64> {
    let fwd = forward_seg(kin, jerk)?;
    let bwd = backward_seg(kin, jerk)?;
    Some(eval_profile(kin, entry, exit, &fwd, &bwd, jerk, s, tol)?.v)
}

#[allow(clippy::too_many_arguments)]
fn refine(
    kin: &Kinematics,
    entry: f64,
    exit: f64,
    jerk: &JerkAnchors,
    tol: f64,
    s0: f64,
    v0: f64,
    s1: f64,
    v1: f64,
    depth: u32,
    out: &mut Vec<(f64, f64)>,
) -> Option<()> {
    let mid = 0.5 * (s0 + s1);
    let actual = profile_speed(kin, entry, exit, jerk, mid, tol)?;
    let interp = 0.5 * (v0 + v1);
    let needs_refine = (actual - interp).abs() > tol * (1.0 + actual.abs());
    // The rest cusp (`v ~ s^(2/3)`, infinite slope at `s = 0`) never satisfies the
    // tolerance, so without a floor on the interval it subdivides until the point
    // budget is spent at the cusp, starving the rest of the move. Stopping at a
    // fraction of the member length keeps the grid dense everywhere it matters.
    let above_floor = (s1 - s0) > kin.length / SAMPLE_MIN_STEP_DIV;
    if needs_refine && above_floor && depth < SAMPLE_MAX_DEPTH && out.len() < SAMPLE_MAX_POINTS {
        refine(
            kin,
            entry,
            exit,
            jerk,
            tol,
            s0,
            v0,
            mid,
            actual,
            depth + 1,
            out,
        )?;
        out.push((mid, actual));
        refine(
            kin,
            entry,
            exit,
            jerk,
            tol,
            mid,
            actual,
            s1,
            v1,
            depth + 1,
            out,
        )?;
    } else if needs_refine {
        out.push((mid, actual));
    }
    Some(())
}

pub(super) struct RunMember<'a> {
    pub kin: &'a Kinematics,
    pub entry_v: f64,
    pub exit_v: f64,
    pub fwd_s: f64,
    pub bwd_s: f64,
}

struct MemberCtx<'a> {
    m: &'a RunMember<'a>,
    fwd_seg: scurve::SevenSeg,
    bwd_seg: scurve::SevenSeg,
    jerk: JerkAnchors,
}

fn build_ctxs<'a>(members: &'a [RunMember<'a>], run_start_v: f64) -> Option<Vec<MemberCtx<'a>>> {
    let mut out = Vec::with_capacity(members.len());
    for m in members {
        let jerk = JerkAnchors {
            fwd_v: run_start_v,
            fwd_s: m.fwd_s,
            bwd_v: 0.0,
            bwd_s: m.bwd_s,
        };
        let fwd_seg = forward_seg(m.kin, &jerk)?;
        let bwd_seg = backward_seg(m.kin, &jerk)?;
        out.push(MemberCtx {
            m,
            fwd_seg,
            bwd_seg,
            jerk,
        });
    }
    Some(out)
}

fn base_samples(ctxs: &[MemberCtx], tol: f64) -> Option<Vec<(f64, f64, f64)>> {
    let mut out: Vec<(f64, f64, f64)> = Vec::new();
    for c in ctxs {
        let mut sv = vec![(0.0, c.m.entry_v)];
        refine(
            c.m.kin,
            c.m.entry_v,
            c.m.exit_v,
            &c.jerk,
            tol,
            0.0,
            c.m.entry_v,
            c.m.kin.length,
            c.m.exit_v,
            0,
            &mut sv,
        )?;
        sv.push((c.m.kin.length, c.m.exit_v));
        for (sl, v) in sv {
            let s_run = c.m.fwd_s + sl;
            if out.last().is_some_and(|p| (p.0 - s_run).abs() < 1e-12) {
                continue;
            }
            let a = eval_profile(
                c.m.kin,
                c.m.entry_v,
                c.m.exit_v,
                &c.fwd_seg,
                &c.bwd_seg,
                &c.jerk,
                sl,
                tol,
            )?
            .a;
            out.push((s_run, v, a));
        }
    }
    Some(out)
}

/// Reconstruct the run's `(s, v, a)` profile.
///
/// The envelope `base_samples` is the accel-limited optimum: correct velocity,
/// but its acceleration steps at every point where the binding constraint
/// switches (entry ramp → cruise → curve → brake). Those steps are infinite
/// jerk. We make the profile jerk-feasible in one uniform pass — `jerk_smooth`
/// — rather than detecting each step and splicing a hand-placed arc across it.
fn reconstruct_flat(ctxs: &[MemberCtx], tol: f64) -> Option<Vec<(f64, f64, f64)>> {
    let base = base_samples(ctxs, tol)?;
    Some(jerk_smooth(ctxs, base))
}

/// Build the jerk-limited `(s, v, a)` profile from the velocity-limit curve.
///
/// The velocity-limit curve `vlc(s)` is the purely geometric speed ceiling
/// (feed and `sqrt(accel / kappa)`), sampled on a dense uniform grid. A backward
/// jerk-ride of it gives `bwd(s)` — the fastest speed at each point that can
/// still brake to every downstream ceiling and the exit. The forward jerk-ride
/// then tracks `min(vlc, bwd)`: it accelerates at jerk-limited rate and peels off
/// to *land* on that cap with `a = 0` (peel once coasting the current accel back
/// to zero would already reach the cap, `v + a^2 / 2j >= cap`), riding it down
/// where it descends into a brake. One continuous integration — so the
/// acceleration is jerk-continuous, ceilings are reached tangentially, and `v`,
/// clamped to the cap, is feasible and non-negative by construction; no
/// per-corner arc, no root-finding.
///
/// Infinite jerk leaves the envelope untouched (the sharp accel apex is intended).
fn jerk_smooth(ctxs: &[MemberCtx], base: Vec<(f64, f64, f64)>) -> Vec<(f64, f64, f64)> {
    let jerk = ctxs[0].m.kin.jerk;
    if base.len() < 3 || !jerk.is_finite() {
        return base;
    }

    let run_len = base[base.len() - 1].0;
    let s = integration_grid(ctxs, run_len);
    let vlc: Vec<f64> = s.iter().map(|&x| vlc_at(ctxs, x)).collect();
    let cap: Vec<f64> = s.iter().map(|&x| accel_cap(ctxs, x)).collect();
    let n = s.len();

    let entry_v = base[0].1;
    let exit_v = base[base.len() - 1].1;
    let entry_a = if entry_v <= VELOCITY_FLOOR {
        0.0
    } else {
        base[0].2
    };

    let bwd = {
        let total = s[n - 1];
        let s_rev: Vec<f64> = (0..n).map(|k| total - s[n - 1 - k]).collect();
        let vlc_rev: Vec<f64> = (0..n).map(|k| vlc[n - 1 - k]).collect();
        let cap_rev: Vec<f64> = (0..n).map(|k| cap[n - 1 - k]).collect();
        let mut b = ride(&s_rev, &vlc_rev, &cap_rev, jerk, exit_v, 0.0);
        b.reverse();
        b
    };
    let forward_cap: Vec<f64> = (0..n).map(|i| vlc[i].min(bwd[i])).collect();

    let mut v = ride(&s, &forward_cap, &cap, jerk, entry_v, entry_a);
    v[0] = entry_v;
    v[n - 1] = exit_v;
    let a = accel_of(&s, &v);

    (0..n).map(|i| (s[i], v[i], a[i])).collect()
}

/// A dense uniform arc-length grid for the jerk-ride. Each member is sampled at a
/// fixed step within its own span — uniform (not velocity-adaptive, since the
/// roll-offs that must be resolved fall in flat-velocity cruise bands), and
/// per-member rather than per-run so appending moves never reshuffles an earlier
/// member's grid (the streaming locked prefix must be invariant under append).
fn integration_grid(ctxs: &[MemberCtx], _run_len: f64) -> Vec<f64> {
    let mut s: Vec<f64> = Vec::new();
    for c in ctxs {
        let len = c.m.kin.length;
        let steps = ((len / GRID_STEP_MM).ceil() as usize).clamp(GRID_MIN_STEPS, SAMPLE_MAX_POINTS);
        for k in 0..steps {
            s.push(c.m.fwd_s + len * (k as f64) / (steps as f64));
        }
    }
    s.push(ctxs[ctxs.len() - 1].m.fwd_s + ctxs[ctxs.len() - 1].m.kin.length);
    s.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    s.dedup_by(|a, b| (*a - *b).abs() <= 1e-12);
    s
}

/// March a jerk-limited speed profile that rides `vlc` from below (see
/// `jerk_smooth`). Reused forwards and (on a reversed grid) backwards.
fn ride(s: &[f64], vlc: &[f64], cap: &[f64], jerk: f64, start_v: f64, start_a: f64) -> Vec<f64> {
    let n = s.len();
    let mut v = vec![0.0_f64; n];
    v[0] = start_v.min(vlc[0]);
    let mut a_prev = start_a;
    for i in 1..n {
        let ds = s[i] - s[i - 1];
        // Time across `ds` from a constant-accel estimate of the step's exit speed
        // — `vlc[i]` would under-time the step (the ceiling sits above the actual
        // speed) and starve the velocity gain. Floored by the jerk-from-rest speed
        // scale `(j*ds^2)^(1/3)` so the launch cusp (`v=a=0`) takes a bounded step
        // instead of an unbounded `dt` that jumps `a` to `a_max` in one sample.
        let v_pred = (v[i - 1] * v[i - 1] + 2.0 * a_prev * ds).max(0.0).sqrt();
        let v_scale = 2.0 * ds / (6.0 * ds / jerk).cbrt();
        let dt = 2.0 * ds / (v[i - 1] + v_pred).max(v_scale);
        let a_ride = (vlc[i] - vlc[i - 1]) / dt;
        // Coast the current acceleration back to zero: the speed it would add is
        // `a^2 / 2j`. If that peak would reach the ceiling, peel off now so `v`
        // lands on it with `a = 0`; `a_ride` then holds `a` on a sloped ceiling and
        // lets it descend past zero into a brake. Keying off the ceiling itself
        // (not its local slope, ~0 until the last moment) makes the peel early
        // enough — even where a descending ceiling closes on a rising `v` at a
        // sub-ceiling peak.
        let coast_peak = v[i - 1] + a_prev.max(0.0) * a_prev.max(0.0) / (2.0 * jerk);
        let a = if coast_peak >= vlc[i] {
            (a_prev - jerk * dt).max(a_ride)
        } else {
            (a_prev + jerk * dt).min(cap[i])
        }
        .clamp(-cap[i], cap[i]);
        v[i] = (v[i - 1] + 0.5 * (a_prev + a) * dt).clamp(0.0, vlc[i]);
        a_prev = a;
    }
    v
}

/// Acceleration consistent with the speed profile: central time-difference of
/// `v`, one-sided at the ends, zeroed at a rest anchor.
fn accel_of(s: &[f64], v: &[f64]) -> Vec<f64> {
    let n = v.len();
    let mut t = vec![0.0_f64; n];
    for i in 1..n {
        t[i] = t[i - 1] + 2.0 * (s[i] - s[i - 1]) / (v[i - 1] + v[i]).max(VELOCITY_FLOOR);
    }
    (0..n)
        .map(|i| {
            if (i == 0 && v[0] <= VELOCITY_FLOOR) || (i == n - 1 && v[n - 1] <= VELOCITY_FLOOR) {
                0.0
            } else if i == 0 {
                (v[1] - v[0]) / (t[1] - t[0]).max(VELOCITY_FLOOR)
            } else if i == n - 1 {
                (v[n - 1] - v[n - 2]) / (t[n - 1] - t[n - 2]).max(VELOCITY_FLOOR)
            } else {
                (v[i + 1] - v[i - 1]) / (t[i + 1] - t[i - 1]).max(VELOCITY_FLOOR)
            }
        })
        .collect()
}

fn vlc_at(ctxs: &[MemberCtx], s_run: f64) -> f64 {
    let mut c = &ctxs[0];
    for ctx in ctxs {
        if s_run + 1e-12 >= ctx.m.fwd_s {
            c = ctx;
        } else {
            break;
        }
    }
    let local = (s_run - c.m.fwd_s).clamp(0.0, c.m.kin.length);
    c.m.kin
        .flat_ceiling
        .min(limit_speed(c.m.kin.kappa_abs(local), c.m.kin.accel))
}

fn accel_cap(ctxs: &[MemberCtx], s_run: f64) -> f64 {
    let mut cap = ctxs[0].m.kin.accel;
    for c in ctxs {
        if s_run + 1e-12 >= c.m.fwd_s {
            cap = c.m.kin.accel;
        } else {
            break;
        }
    }
    cap
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

/// A run is straight under a single flat ceiling when every member has zero
/// curvature and shares the same ceiling, acceleration, and jerk. Such a run is
/// one analytic triple-limited profile from the entry anchor to the exit anchor,
/// so it reconstructs in closed form (`profile::plan`) instead of on the grid.
fn flat_ceiling_run(members: &[RunMember]) -> Option<(f64, f64, f64)> {
    let head = members[0].kin;
    let (ceiling, accel, jerk) = (head.flat_ceiling, head.accel, head.jerk);
    for m in members {
        let k = m.kin;
        let straight = k.kappa0.abs() <= KAPPA_EPS && k.sigma.abs() <= KAPPA_EPS;
        let uniform = k.flat_ceiling == ceiling && k.accel == accel && k.jerk == jerk;
        if !(straight && uniform) {
            return None;
        }
    }
    Some((ceiling, accel, jerk))
}

/// Reconstruct a straight constant-ceiling run from its analytic profile. The
/// profile is built once across the whole run from the entry anchor to the exit
/// anchor; each member reads `(v, a)` from it at its own arc-length offset, so a
/// collinear seam is C1-continuous by construction — both sides read the same
/// profile, and the seam velocity is the profile's (the jerk-feasible speed that
/// paces to land on a ceiling at `a = 0`), never the raw velocity-limit ceiling
/// the seam sweep records as an upper bound.
fn reconstruct_straight(
    members: &[RunMember],
    run_start_v: f64,
    ceiling: f64,
    accel: f64,
    jerk: f64,
) -> Vec<Vec<(f64, f64, f64)>> {
    let length: f64 = members.iter().map(|m| m.kin.length).sum();
    let exit_v = members[members.len() - 1].exit_v;
    let profile = super::profile::plan(run_start_v, exit_v, length, ceiling, accel, jerk);
    members
        .iter()
        .map(|m| {
            let len = m.kin.length;
            let steps =
                ((len / GRID_STEP_MM).ceil() as usize).clamp(GRID_MIN_STEPS, SAMPLE_MAX_POINTS);
            let mut local: Vec<(f64, f64, f64)> = (0..=steps)
                .map(|k| {
                    let sl = len * (k as f64) / (steps as f64);
                    let (v, a) = profile.at(m.fwd_s + sl);
                    (sl, v, a)
                })
                .collect();
            let last = local.len() - 1;
            local[last].0 = len;
            local
        })
        .collect()
}

pub(super) fn reconstruct_run(
    members: &[RunMember],
    run_start_v: f64,
    tol: f64,
) -> Option<Vec<Vec<(f64, f64, f64)>>> {
    if let Some((ceiling, accel, jerk)) = flat_ceiling_run(members) {
        return Some(reconstruct_straight(
            members,
            run_start_v,
            ceiling,
            accel,
            jerk,
        ));
    }
    let ctxs = build_ctxs(members, run_start_v)?;
    let flat = reconstruct_flat(&ctxs, tol)?;

    let mut per_member: Vec<Vec<(f64, f64, f64)>> = Vec::with_capacity(ctxs.len());
    for c in &ctxs {
        let s0 = c.m.fwd_s;
        let s1 = c.m.fwd_s + c.m.kin.length;
        // Member boundaries are pinned to the jerk-feasible junction velocities
        // the seam sweep solved; the interior reads the reconstructed profile.
        let v0 = c.m.entry_v;
        let v1 = c.m.exit_v;
        let mut local: Vec<(f64, f64, f64)> = flat
            .iter()
            .filter(|p| p.0 >= s0 - 1e-9 && p.0 <= s1 + 1e-9)
            .map(|p| ((p.0 - s0).clamp(0.0, c.m.kin.length), p.1, p.2))
            .collect();
        local.dedup_by(|a, b| (a.0 - b.0).abs() < 1e-12);
        if local.first().is_none_or(|p| p.0 > 1e-9) {
            let (v, a) = interp_flat(&flat, s0)?;
            local.insert(0, (0.0, v, a));
        }
        if local
            .last()
            .is_none_or(|p| (p.0 - c.m.kin.length).abs() > 1e-9)
        {
            let (v, a) = interp_flat(&flat, s1)?;
            local.push((c.m.kin.length, v, a));
        }
        if let Some(first) = local.first_mut() {
            first.0 = 0.0;
            first.1 = v0;
        }
        if let Some(last) = local.last_mut() {
            last.0 = c.m.kin.length;
            last.1 = v1;
        }
        for p in &mut local {
            p.2 = clamp_to_disk(p.2, c.m.kin.accel, c.m.kin.kappa_abs(p.0), p.1);
        }
        per_member.push(local);
    }
    Some(per_member)
}

#[cfg(test)]
pub(super) fn sample_profile(
    kin: &Kinematics,
    entry: f64,
    exit: f64,
    anchors: &JerkAnchors,
    tol: f64,
) -> Option<Vec<(f64, f64, f64)>> {
    let member = RunMember {
        kin,
        entry_v: entry,
        exit_v: exit,
        fwd_s: anchors.fwd_s,
        bwd_s: anchors.bwd_s,
    };
    reconstruct_run(&[member], anchors.fwd_v, tol)?
        .into_iter()
        .next()
}

#[cfg(test)]
mod tests;
