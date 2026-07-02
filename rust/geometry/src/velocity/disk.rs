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

/// Cruise-speed ceiling from the path's vector jerk. Held at constant speed `v`,
/// a path of curvature `kappa` and curvature rate `sigma` still has a turning,
/// growing acceleration vector: the centripetal share `kappa v^2` rotates with
/// the heading (rate `kappa v`) and grows as the curvature tightens (rate
/// `sigma v`), so `|d a_vec / dt| = v^3 sqrt(kappa^4 + sigma^2)`. Capping that at
/// `jerk` is the jerk analog of the centripetal `sqrt(accel / kappa)` ceiling —
/// and the fixpoint that makes `a = 0` cruise jerk-feasible, which is what a
/// reachability arc lands on when it rejoins the ceiling.
pub(super) fn jerk_limit_speed(kappa_abs: f64, sigma: f64, jerk: f64) -> f64 {
    if !jerk.is_finite() {
        return f64::INFINITY;
    }
    let k2 = kappa_abs * kappa_abs;
    let coeff = (k2 * k2 + sigma * sigma).sqrt();
    if coeff > 0.0 {
        (jerk / coeff).cbrt()
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
    /// Acceleration at the forward anchor: zero at a rest anchor, the carried
    /// mid-flight state when the run continues a profile cut at a window
    /// boundary.
    pub fwd_a: f64,
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

fn forward_seg(kin: &Kinematics, jerk: &JerkAnchors) -> Option<scurve::SevenSeg> {
    scurve::breakpoints(
        jerk.fwd_v,
        jerk.fwd_a.clamp(-kin.accel, kin.accel),
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

fn build_ctxs<'a>(
    members: &'a [RunMember<'a>],
    run_start_v: f64,
    run_start_a: f64,
) -> Option<Vec<MemberCtx<'a>>> {
    let mut out = Vec::with_capacity(members.len());
    for m in members {
        let jerk = JerkAnchors {
            fwd_v: run_start_v,
            fwd_a: run_start_a,
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
fn reconstruct_flat(
    ctxs: &[MemberCtx],
    entry_a: f64,
    tol: f64,
) -> Option<(Vec<(f64, f64, f64)>, Vec<f64>)> {
    let base = base_samples(ctxs, tol)?;
    Some(jerk_smooth(ctxs, base, entry_a))
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
///
/// Returns the `(s, v, a)` samples (the emitted `a` is the central-difference
/// derivative of the emitted `v`, so the pair is self-consistent) alongside the
/// forward integrator's internal acceleration at every grid point — the state a
/// streaming cut must carry to continue this exact curve in the next window.
fn jerk_smooth(
    ctxs: &[MemberCtx],
    base: Vec<(f64, f64, f64)>,
    entry_a: f64,
) -> (Vec<(f64, f64, f64)>, Vec<f64>) {
    if base.len() < 3 {
        let acc = base.iter().map(|p| p.2).collect();
        return (base, acc);
    }

    let run_len = base[base.len() - 1].0;
    let s = integration_grid(ctxs, run_len);
    let n = s.len();
    let vlc: Vec<f64> = s.iter().map(|&x| vlc_at(ctxs, x)).collect();
    let accel: Vec<f64> = s.iter().map(|&x| accel_cap(ctxs, x)).collect();
    let kappa: Vec<f64> = s.iter().map(|&x| kappa_abs_at(ctxs, x)).collect();

    let entry_v = base[0].1;
    let exit_v = base[base.len() - 1].1;

    // `planSpeed`: pure acceleration-budget (disk) forward–backward pass. The
    // tangential budget is whatever the disk leaves after the centripetal share
    // `sqrt(a_max^2 - (kappa v^2)^2)`, so the total acceleration vector rides the
    // budget boundary and the scalar magnitude stays at `a_max` through the trade.
    // This is the velocity *ceiling* — the corner trade is baked in here.
    let mut ceil = vlc.clone();
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

    let jerk = ctxs[0].m.kin.jerk;
    if !jerk.is_finite() {
        // Infinite jerk: the disk boundary itself is the answer (nothing to ramp).
        let a: Vec<f64> = (0..n)
            .map(|i| {
                if (i == 0 && entry_v <= VELOCITY_FLOOR) || (i == n - 1 && exit_v <= VELOCITY_FLOOR)
                {
                    return 0.0;
                }
                let (lo, hi) = (i.saturating_sub(1), (i + 1).min(n - 1));
                let dvds = (ceil[hi] - ceil[lo]) / (s[hi] - s[lo]).max(1e-12);
                (ceil[i] * dvds).clamp(-disk_rail_accel(accel[i], kappa[i], ceil[i]), {
                    disk_rail_accel(accel[i], kappa[i], ceil[i])
                })
            })
            .collect();
        let samples = (0..n).map(|i| (s[i], ceil[i], a[i])).collect();
        return (samples, a);
    }

    // Reconstruct under the full vector jerk limit by reachability: the backward
    // pass integrates the fastest brake arc that still lands on the velocity-limit
    // curve and the exit; the forward pass then accelerates from the entry, capped
    // by that brake envelope, landing on the cap with `a = 0`. Each pass integrates
    // *away* from the constraint, never marches along it, so an unreachable ceiling
    // notch (a tiny clothoid whose `vlc` plunges faster than the disk can brake)
    // is simply bounded by the arc through its reachable neighborhood.
    let bwd = {
        let s_rev: Vec<f64> = (0..n).map(|k| s[n - 1] - s[n - 1 - k]).collect();
        let rev = |a: &[f64]| -> Vec<f64> { (0..n).map(|k| a[n - 1 - k]).collect() };
        let (mut b, _) = reach_pass(
            &s_rev,
            &rev(&vlc),
            &rev(&accel),
            &rev(&kappa),
            jerk,
            exit_v,
            0.0,
        );
        b.reverse();
        b
    };
    let track: Vec<f64> = (0..n).map(|i| vlc[i].min(bwd[i])).collect();
    let (mut v, acc) = reach_pass(&s, &track, &accel, &kappa, jerk, entry_v, entry_a);
    // Pin the seam speeds, but never above the reachable cap: a planned anchor sits
    // at or below it, while a stale over-ceiling anchor clamps instead of injecting a
    // boundary discontinuity that the central-difference accel would read as a spike.
    v[0] = entry_v.min(track[0]);
    v[n - 1] = exit_v.min(track[n - 1]);

    // The tangential accel is the true derivative of the reconstructed speed:
    // `a = d(v^2/2)/ds`, by central difference, so it agrees with the emitted `v`
    // exactly (and stays disk-feasible, since `v` is). Rest ends carry `a = 0`.
    let mut a = vec![0.0_f64; n];
    for i in 0..n {
        let (lo, hi) = (i.saturating_sub(1), (i + 1).min(n - 1));
        let span = s[hi] - s[lo];
        a[i] = if span > 1e-12 {
            (v[hi] * v[hi] - v[lo] * v[lo]) / (2.0 * span)
        } else {
            0.0
        };
    }
    if entry_v <= VELOCITY_FLOOR {
        a[0] = 0.0;
    }
    if exit_v <= VELOCITY_FLOOR {
        a[n - 1] = 0.0;
    }

    ((0..n).map(|i| (s[i], v[i], a[i])).collect(), acc)
}

/// Integrate the fastest jerk- and disk-feasible acceleration arc that stays under
/// the `cap`, carrying the tangential accel so it is continuous.
///
/// Over a step the Frenet frame turns by `dtheta = kappa * ds`, so the world-frame
/// acceleration increment has a normal part `j_norm = (a_n - a_n_prev) + a_prev *
/// dtheta` fixed by the speed and curvature, and a tangential part we steer.
/// Keeping their hypotenuse within `j_max * dt` is a disk on the increment — the
/// scalar-magnitude band it replaces was only its radial slice, leaving the
/// vector's rotation (a fillet whipping it around, a clothoid's growing
/// centripetal) unbilled. The normal part spends jerk budget first; the circle
/// that remains bounds the tangential accel about its rotation-driven `center`.
///
/// The arc accelerates at the disk rail until coasting the accel back to zero
/// would already reach the `cap`, then jerks the accel down to land tangent to it
/// (`a = 0`) — so it rejoins the ceiling, or meets a brake arc at a velocity peak,
/// with continuous acceleration. It only integrates where it is *below* the cap,
/// off the constraint boundary, so a `cap` that plunges faster than the disk can
/// brake never traps it on an unreachable ceiling.
#[allow(clippy::too_many_arguments)]
fn reach_pass(
    s: &[f64],
    cap: &[f64],
    accel: &[f64],
    kappa: &[f64],
    j_max: f64,
    start_v: f64,
    start_a: f64,
) -> (Vec<f64>, Vec<f64>) {
    let n = s.len();
    let mut v = vec![0.0_f64; n];
    let mut acc = vec![0.0_f64; n];
    v[0] = start_v.min(cap[0]);
    let rail0 = disk_rail_accel(accel[0], kappa[0], v[0]);
    let mut a_prev = start_a.clamp(-rail0, rail0);
    acc[0] = a_prev;
    for i in 1..n {
        let ds = s[i] - s[i - 1];
        let v_pred = (v[i - 1] * v[i - 1] + 2.0 * a_prev * ds).max(0.0).sqrt();
        let v_scale = 2.0 * ds / (6.0 * ds / j_max).cbrt();
        let dt = 2.0 * ds / (v[i - 1] + v_pred).max(v_scale);
        let a_n = kappa[i] * v_pred * v_pred;
        let a_n_prev = kappa[i - 1] * v[i - 1] * v[i - 1];
        let dtheta = kappa[i] * ds;
        let j_norm = (a_n - a_n_prev) + a_prev * dtheta;
        let center = a_prev + a_n_prev * dtheta;
        let tang = ((j_max * dt) * (j_max * dt) - j_norm * j_norm)
            .max(0.0)
            .sqrt();
        let rail = disk_rail_accel(accel[i], kappa[i], v_pred);
        // Accelerate at the rail until coasting the accel to zero would reach the
        // cap, then aim for zero accel so the speed lands tangent to the cap.
        let coast_peak = v_pred + a_prev.max(0.0) * a_prev.max(0.0) / (2.0 * j_max);
        let a_target = if coast_peak >= cap[i] { 0.0 } else { rail };
        let a = a_target
            .clamp(center - tang, center + tang)
            .clamp(-rail, rail);
        v[i] = (v[i - 1] * v[i - 1] + (a_prev + a) * ds)
            .max(0.0)
            .sqrt()
            .min(cap[i]);
        acc[i] = a;
        a_prev = a;
    }
    (v, acc)
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

/// Speed ceiling at run-arc `s_run`, the min over every member whose span covers
/// it. At a curvature-discontinuous seam two members abut, and the ceiling must
/// honor the tighter side — otherwise the dip's own endpoint (read by the member
/// that ends there) could overshoot its curvature limit.
fn vlc_at(ctxs: &[MemberCtx], s_run: f64) -> f64 {
    members_at(ctxs, s_run)
        .map(|(c, local)| {
            let kappa_abs = c.m.kin.kappa_abs(local);
            c.m.kin
                .flat_ceiling
                .min(limit_speed(kappa_abs, c.m.kin.accel))
        })
        .fold(f64::INFINITY, f64::min)
}

/// Members whose span covers `s_run` (two at a seam, one in an interior).
fn members_at<'a, 'b>(
    ctxs: &'b [MemberCtx<'a>],
    s_run: f64,
) -> impl Iterator<Item = (&'b MemberCtx<'a>, f64)> {
    ctxs.iter().filter_map(move |c| {
        let lo = c.m.fwd_s;
        let hi = lo + c.m.kin.length;
        (s_run >= lo - 1e-9 && s_run <= hi + 1e-9)
            .then(|| (c, (s_run - lo).clamp(0.0, c.m.kin.length)))
    })
}

/// Acceleration budget inputs at `s_run`, tightest across abutting members: the
/// smallest `a_max` and the largest curvature, so a seam between members of
/// different curvature bounds the tangential accel by the more restrictive side.
fn accel_cap(ctxs: &[MemberCtx], s_run: f64) -> f64 {
    members_at(ctxs, s_run)
        .map(|(c, _)| c.m.kin.accel)
        .fold(f64::INFINITY, f64::min)
}

fn kappa_abs_at(ctxs: &[MemberCtx], s_run: f64) -> f64 {
    members_at(ctxs, s_run)
        .map(|(c, local)| c.m.kin.kappa_abs(local))
        .fold(0.0, f64::max)
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
) -> (Vec<Vec<(f64, f64, f64)>>, Vec<(f64, f64)>) {
    let length: f64 = members.iter().map(|m| m.kin.length).sum();
    let exit_v = members[members.len() - 1].exit_v;
    let profile = super::profile::plan(run_start_v, exit_v, length, ceiling, accel, jerk);
    let samples = members
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
        .collect();
    let exit_states = members
        .iter()
        .map(|m| profile.at(m.fwd_s + m.kin.length))
        .collect();
    (samples, exit_states)
}

/// The per-move closed-form jerk phases of a straight constant-ceiling run, in
/// each move's local time/arc-length, or `None` when the run is not such a run.
/// Built from the same single analytic profile as [`reconstruct_straight`], so a
/// straight move can lower one exact cubic per phase instead of fitting cubics to
/// the resampled grid.
pub(super) fn reconstruct_run_phases(
    members: &[RunMember],
    run_start_v: f64,
    run_start_a: f64,
) -> Option<Vec<Vec<super::profile::StraightPhase>>> {
    if run_start_a != 0.0 {
        return None;
    }
    let (ceiling, accel, jerk) = flat_ceiling_run(members)?;
    let length: f64 = members.iter().map(|m| m.kin.length).sum();
    let exit_v = members[members.len() - 1].exit_v;
    let profile = super::profile::plan(run_start_v, exit_v, length, ceiling, accel, jerk);
    let spans: Vec<(f64, f64)> = members.iter().map(|m| (m.fwd_s, m.kin.length)).collect();
    Some(profile.phases_for_spans(&spans))
}

/// Reconstruct the run: per-member `(s, v, a)` samples, plus the profile
/// state `(v, a)` at each member's exit seam — the `a` there is the forward
/// integrator's internal state (analytic for straight runs), which is what a
/// streaming cut carries into the next window to continue this exact curve.
pub(super) fn reconstruct_run(
    members: &[RunMember],
    run_start_v: f64,
    run_start_a: f64,
    tol: f64,
) -> Option<(Vec<Vec<(f64, f64, f64)>>, Vec<(f64, f64)>)> {
    if run_start_a == 0.0 {
        if let Some((ceiling, accel, jerk)) = flat_ceiling_run(members) {
            return Some(reconstruct_straight(
                members,
                run_start_v,
                ceiling,
                accel,
                jerk,
            ));
        }
    }
    let ctxs = build_ctxs(members, run_start_v, run_start_a)?;
    let (flat, integrator_a) = reconstruct_flat(&ctxs, run_start_a, tol)?;

    let mut per_member: Vec<Vec<(f64, f64, f64)>> = Vec::with_capacity(ctxs.len());
    let mut exit_states: Vec<(f64, f64)> = Vec::with_capacity(ctxs.len());
    for c in &ctxs {
        let s0 = c.m.fwd_s;
        let s1 = c.m.fwd_s + c.m.kin.length;
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
        }
        if let Some(last) = local.last_mut() {
            last.0 = c.m.kin.length;
        }
        per_member.push(local);
        exit_states.push(exit_state_at(&flat, &integrator_a, s1));
    }
    Some((per_member, exit_states))
}

/// Profile state at run-arc `s`: velocity from the emitted samples, accel from
/// the forward integrator (the grid contains every member boundary exactly, so
/// a seam lookup hits a grid point; the nearest-point fallback covers the
/// short-run path where the grid was not built).
fn exit_state_at(flat: &[(f64, f64, f64)], integrator_a: &[f64], s: f64) -> (f64, f64) {
    let i = flat.partition_point(|p| p.0 < s - 1e-9);
    let i = i.min(flat.len() - 1);
    let a = integrator_a.get(i).copied().unwrap_or(flat[i].2);
    (flat[i].1, a)
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
    reconstruct_run(&[member], anchors.fwd_v, anchors.fwd_a, tol)?
        .0
        .into_iter()
        .next()
}

#[cfg(test)]
mod tests;
